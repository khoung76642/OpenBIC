/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <shell/shell.h>
#include <stdlib.h>
#include "sensor.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "plat_cpld.h"

static int get_vr_reg_to_int(uint8_t vr_rail, uint8_t reg)
{
	uint8_t data[2] = { 0 };
	if (!get_raw_data_from_sensor_id(vr_rail_table[vr_rail].sensor_id, reg, data, 2))
		return -1;

	uint16_t raw_val = (data[1] << 8) | data[0];
	return (int)raw_val;
}

static int cmd_vr_phase_current_get(const struct shell *shell, size_t argc, char **argv)
{
	if (!(argc == 2)) {
		shell_error(shell, "vr_phase_current get <voltage-rail>");
		return -1;
	}

	if (!(get_is_ubc_enabled() && is_ubc_enabled_delayed_enabled())) {
		shell_error(shell, "Can't get vr_phase_current because VR has no power yet.");
		return -1;
	}

	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}

	/* Stop sensor polling */
	set_plat_sensor_polling_enable_flag(false);

	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	switch (cfg->type) {
	case sensor_dev_mp2971: {
		uint8_t page = get_vr_page(rail);
		uint8_t start_reg = (page == 0) ? 0x82 : 0x85;
		uint8_t reg_cnt = (page == 0) ? 6 : 3;

		for (int i = 0; i < reg_cnt; i++) {
			uint8_t reg = start_reg + i;
			int raw_data = get_vr_reg_to_int(rail, reg);

			uint8_t cs_low = raw_data & 0xFF; // bit [7:0]
			uint8_t cs_high = (raw_data >> 8) & 0xFF; // bit [15:8]

			float phase_current_low =
				((cs_low * 0.0125f) - 1.23f) / 0.005f;
			float phase_current_high =
				((cs_high * 0.0125f) - 1.23f) / 0.005f;

			int phase_low = (i * 2) + 1;
			int phase_high = (i * 2) + 2;

			shell_print(shell,
				    "CS%-2d: %8.1f A (reg=0x%02X, raw_data=0x%04X, cs_low=0x%02X)",
				    phase_low, phase_current_low, reg, raw_data, cs_low);

			shell_print(shell,
				    "CS%-2d: %8.1f A (reg=0x%02X, raw_data=0x%04X, cs_high=0x%02X)",
				    phase_high, phase_current_high, reg, raw_data, cs_high);
		}
	} break;
	case sensor_dev_mp29816a: {
		uint8_t start_reg = 0xD0;
		uint8_t reg_cnt = 9;
		for (int i = 0; i < reg_cnt; i++) {
			uint8_t reg = start_reg + i;
			int raw_data = get_vr_reg_to_int(rail, reg);
			int phase = i + 1;
			float phase_current = raw_data * 0.003125f / 0.0025f;

			shell_print(shell, "CS%-2d: %8.1f A (reg=0x%02X, raw_data=0x%04X)", phase,
				    phase_current, reg, raw_data);
		}
	} break;
	case sensor_dev_raa228249: {
		uint8_t page_data = 0x80;
		if (!plat_set_vr_reg(rail, 0x00, &page_data, 1)) {
			shell_error(shell, "vr %d set page fail", rail);
			return -1;
		}
		uint8_t start_phase = 0x00;
		uint8_t phase_cnt = 9;
		for (int i = 0; i < phase_cnt; i++) {
			uint8_t phase_set_data = start_phase + i;
			if (!plat_set_vr_reg(rail, 0x04, &phase_set_data, 1)) {
				shell_error(shell, "vr %d set phase=0x%02X fail", rail,
					    phase_set_data);
				continue;
			}
			int raw_data = get_vr_reg_to_int(rail, 0xE4);
			float phase_current = raw_data * 0.1f;

			shell_print(shell,
				    "CS%-2d: %8.1f A (phase_set_data=0x%02X, raw_data=0x%04X)",
				    phase_set_data, phase_current, phase_set_data, raw_data);
		}
	} break;
	default:
		shell_print(shell, "Unsupport VR type(%d)", cfg->type);
		break;
	}

	int iout_value;
	if (vr_rail_iout_value_get(rail, &iout_value) == false) {
		shell_error(shell, "Invalid rail to get iout value");
	}

	float sensor_reading = 0, decimal = 0;
	int16_t integer = 0;

	integer = iout_value & 0xffff;
	decimal = (float)(iout_value >> 16) / 1000.0;

	if (integer >= 0) {
		sensor_reading = (float)integer + decimal;
	} else {
		sensor_reading = (float)integer - decimal;
	}

	shell_print(shell, "%-50s iout: %10.3fA", argv[1], sensor_reading);

	/* Start sensor polling */
	set_plat_sensor_polling_enable_flag(true);

	return 0;
}

static void voltage_rname_get(size_t idx, struct shell_static_entry *entry)
{
	if (((get_asic_board_id() != ASIC_BOARD_ID_EVB)) && (idx == VR_RAIL_E_P3V3_OSFP_VOLT_V))
		idx++;
	uint8_t *name = NULL;
	vr_rail_name_get((uint8_t)idx, &name);

	if (idx == VR_RAIL_E_P3V3_OSFP_VOLT_V) {
		return;
	}

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(vr_rname_for_vr_phase_current, voltage_rname_get);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_vr_phase_current_cmds,
			       SHELL_CMD(get, &vr_rname_for_vr_phase_current, "get <voltage-rail>",
					 cmd_vr_phase_current_get),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(vr_phase_current, &sub_vr_phase_current_cmds, "vr phase current get commands",
		   NULL);
