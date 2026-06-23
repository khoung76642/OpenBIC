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
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "plat_event.h"
#include "plat_vr_test_mode.h"
#include "plat_util.h"

#define DETECTOR_CONF_REG_RAIL_1 0xEA38
#define DETECTOR_CONF_REG_RAIL_2 0xEAB8
#define READ_RAIL_RESULT_REG_RAIL_1 0xED33
#define READ_RAIL_RESULT_REG_RAIL_2 0xED34
#define MEASUREMENT_WINDOW 0x7 // until reset
#define DETECTOR_ENABLE_BIT BIT(13) // bit-13
#define RAIL_VOLT 0x0
#define RAIL_UNFILTER_CURR 0x11
#define PHASE_CURR_BIT_FILTER 0x3E0 // bit 9:5
#define PHASE_CURR_BASE 0xED00
#define CUR_SHIFT_REG_RAIL_1 0xE882
#define CUR_SHIFT_REG_RAIL_2 0xE883

sensor_cfg *temp_cfg = NULL;
uint8_t temp_page = 0;
bool temp_is_current = false;
enum VR_RAIL_E temp_rail = 0;

void detector_set_config(const struct shell *shell, size_t argc, char **argv)
{
	if (!is_dc_on()) {
		shell_warn(shell, "please iris power on first");
		return;
	}
	uint8_t vr = get_vr_module();
	if (vr != VR_MODULE_RNS) {
		shell_error(shell, "vr dtb dma write only support RNS");
		return;
	}
	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return;
	}
	sensor_cfg *cfg = NULL;
	if (get_sensor_cfg_by_rail(rail, &cfg) != 0) {
		shell_error(shell, "Failed to get sensor config for rail: %s", argv[1]);
		return;
	}
	uint8_t page = get_vr_page(rail);
	// page 0 for rail 1, page 1 for rail 2
	shell_print(shell, "vr %d dma write to page: %d", rail, page);
	uint16_t reg = page ?  DETECTOR_CONF_REG_RAIL_2: DETECTOR_CONF_REG_RAIL_1;
	/*
	Construct 16-bit detector config value from bit fields:
	Bit 13 = 0x0 (detector disabled)
	Bits 12:10 = 0x7 (measurement window)
	Bits 9:5 = 0x01 (phase current bits)
	Bits 4:0 = voltage (0x0) or current (0x11)
	*/
	uint16_t config_value = 0;
	config_value |= (MEASUREMENT_WINDOW & 0x7) << 10;  // bits 12:10
	config_value |= 0x01 << 5;                         // bits 9:5
	bool is_current = false;

	if (strcmp(argv[2], "current") == 0) {
		is_current = true;
	} else if (strcmp(argv[2], "voltage") != 0) {
		shell_error(shell, "Invalid type: %s (use voltage/current)", argv[2]);
		return;
	}
	
	// Determine mode from argv[2]
	if (is_current) {
		config_value |= RAIL_UNFILTER_CURR;  // bits 4:0 = 0x11
	} else {
		config_value |= RAIL_VOLT;           // bits 4:0 = 0x0
	}
	
	// Convert to 2 bytes (little-endian)
	uint8_t data[2];
	data[0] = config_value & 0xFF;
	data[1] = (config_value >> 8) & 0xFF;

	shell_print(shell, "vr %d dma write 0x%x 0x%x to 0x%x:", rail, data[0], data[1], reg);
	if (!dma_write_vr(rail, reg, data, 2)) {
		shell_warn(shell, "vr %d dma write to 0x%x fail", rail, reg);
		//clear temp_cfg and temp_page
		temp_cfg = NULL;
		temp_page = 0;
		temp_is_current = false;
		return;
	}
	shell_print(shell, "vr %d dma write to 0x%x:", rail, reg);
	// set enable bit after config is set successfully written
	config_value |= DETECTOR_ENABLE_BIT; // set enable bit
	data[0] = config_value & 0xFF;
	data[1] = (config_value >> 8) & 0xFF;
	if (!dma_write_vr(rail, reg, data, 2)) {
		shell_warn(shell, "vr %d dma write to 0x%x fail", rail, reg);
		//clear temp_cfg and temp_page
		temp_cfg = NULL;
		temp_page = 0;
		temp_is_current = false;
		return;
	}
	shell_print(shell, "vr %d dma write 0x%x 0x%x to 0x%x:", rail, data[0], data[1], reg);
	//update temp_cfg, temp_page, and temp_rail
	temp_cfg = cfg;
	temp_page = page;
	temp_rail = rail;
	temp_is_current = is_current;
}

void cmd_stop_min_max_detector(const struct shell *shell, size_t argc, char **argv)
{
	enum VR_RAIL_E stop_rail = 0;
	uint8_t stop_page = 0;

	// Check if user provided a rail argument
	if (argc > 1) {
		// User specified a rail directly
		if (vr_rail_enum_get(argv[1], &stop_rail) == false) {
			shell_error(shell, "Invalid rail name: %s", argv[1]);
			return;
		}
		stop_page = get_vr_page(stop_rail);
		shell_print(shell, "stop detector for rail: %d, page: %d (direct mode)", stop_rail, stop_page);
	} else {
		// Use previously started detector
		if (temp_cfg == NULL) {
			shell_warn(shell, "min-max detector is not started. Usage: stop [rail_name]");
			return;
		}
		if (temp_page > 1) {
			shell_warn(shell, "invalid page stored in temp_page: %d", temp_page);
			return;
		}
		stop_rail = temp_rail;
		stop_page = temp_page;
	}

	// set enable bit to 0 to stop the detector
	uint16_t reg = stop_page ?  DETECTOR_CONF_REG_RAIL_2: DETECTOR_CONF_REG_RAIL_1;
	uint32_t config_value = 0;
	if (!dma_read_vr(stop_rail, reg, (uint8_t *)&config_value, 2)) {
		shell_warn(shell, "vr %d dma read from 0x%x fail", stop_rail, reg);
		return;
	}
	config_value &= ~DETECTOR_ENABLE_BIT; // 13-bit = 0
	uint8_t data[2];
	data[0] = config_value & 0xFF;
	data[1] = (config_value >> 8) & 0xFF;
	if (!dma_write_vr(stop_rail, reg, data, 2)) {
		shell_warn(shell, "vr %d dma write to 0x%x fail", stop_rail, reg);
		return;
	}
	shell_print(shell, "vr %d dma write 0x%x 0x%x to 0x%x to stop detector", stop_rail, data[0], data[1], reg);
	
	// Only clear temp state if stopping the previously started detector
	if (argc <= 1) {
		temp_cfg = NULL;
		temp_page = 0;
		temp_rail = 0;
		temp_is_current = false;
	}
}

void cmd_min_max_detector_status(const struct shell *shell, size_t argc, char **argv)
{
	enum VR_RAIL_E status_rail = 0;
	uint8_t status_page = 0;

	// Check if user provided a rail argument
	if (argc > 1) {
		if (vr_rail_enum_get(argv[1], &status_rail) == false) {
			shell_error(shell, "Invalid rail name: %s", argv[1]);
			return;
		}
		status_page = get_vr_page(status_rail);
	} else {
		if (temp_cfg == NULL) {
			shell_print(shell, "min-max detector is not started");
			return;
		}
		status_rail = temp_rail;
		status_page = temp_page;
	}

	uint16_t reg = status_page ?  DETECTOR_CONF_REG_RAIL_2: DETECTOR_CONF_REG_RAIL_1;
	uint32_t config_value = 0;
	if (!dma_read_vr(status_rail, reg, (uint8_t *)&config_value, 2)) {
		shell_warn(shell, "vr %d dma read from 0x%x fail", status_rail, reg);
		return;
	}
	if (config_value & DETECTOR_ENABLE_BIT) {
		shell_print(shell, "enabled");
	} else {
		shell_print(shell, "disabled");
		return;
	}
	// show rail name and type (voltage/current)
	uint8_t type = config_value & 0x1F; // bit 4:0
	char *type_str = (type == RAIL_VOLT) ? "Voltage" : (type == RAIL_UNFILTER_CURR) ? "Current" : "Unknown";
	uint8_t *rail_name = NULL;
	vr_rail_name_get(status_rail, &rail_name);
	shell_print(shell, "rail: %s, type: %s", rail_name ? (char *)rail_name : "Unknown", type_str);
}

void cmd_fetch_min_max_detector_value(const struct shell *shell, size_t argc, char **argv)
{
	enum VR_RAIL_E fetch_rail = 0;
	uint8_t fetch_page = 0;
	bool fetch_is_current = false;

	// Check if user provided a rail argument
	if (argc > 1) {
		// User specified a rail directly
		if (vr_rail_enum_get(argv[1], &fetch_rail) == false) {
			shell_error(shell, "Invalid rail name: %s", argv[1]);
			return;
		}
		fetch_page = get_vr_page(fetch_rail);
		// Inherit current/voltage mode from started detector if rail matches, else default to voltage
		fetch_is_current = (temp_cfg != NULL && fetch_rail == temp_rail) ? temp_is_current : false;
		shell_print(shell, "fetch data for rail: %d, page: %d, type: %s", fetch_rail, fetch_page, fetch_is_current ? "Current" : "Voltage");
	} else {
		// Use previously started detector
		if (temp_cfg == NULL) {
			shell_warn(shell, "min-max detector is not started. Usage: fetch [rail_name]");
			return;
		}
		if (temp_page > 1) {
			shell_warn(shell, "invalid page stored in temp_page: %d", temp_page);
			return;
		}
		fetch_rail = temp_rail;
		fetch_page = temp_page;
		fetch_is_current = temp_is_current;
	}

	// read min-max value from corresponding register
	uint16_t conf_reg = fetch_page ? DETECTOR_CONF_REG_RAIL_2 : DETECTOR_CONF_REG_RAIL_1;
	uint32_t config_value = 0;
	if (!dma_read_vr(fetch_rail, conf_reg, (uint8_t *)&config_value, 2)) {
		shell_warn(shell, "vr %d dma read from 0x%x fail", fetch_rail, conf_reg);
		return;
	}
	if (!(config_value & DETECTOR_ENABLE_BIT)) {
		shell_warn(shell, "detector is disabled, please start detector first");
		return;
	}

	uint16_t reg = fetch_page ?  READ_RAIL_RESULT_REG_RAIL_2: READ_RAIL_RESULT_REG_RAIL_1;
	uint8_t data[4];
	if (!dma_read_vr(fetch_rail, reg, data, 4)) {
		shell_warn(shell, "vr %d dma read from 0x%x fail", fetch_rail, reg);
		return;
	}
	uint8_t *rail_name = NULL;
	vr_rail_name_get(fetch_rail, &rail_name);
	shell_print(shell, "rail: %s, type: %s", rail_name ? (char *)rail_name : "Unknown", fetch_is_current ? "Current" : "Voltage");
	int16_t min_val = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
	int16_t max_val = (int16_t)(((uint16_t)data[3] << 8) | data[2]);

	if (!fetch_is_current) {
		double vout_min = (double)min_val / 8000.0;
		double vout_max = (double)max_val / 8000.0;
		shell_print(shell,
			// change line to show vout max
			"Vout min(raw=%d): %.6f V, \nVout max(raw=%d): %.6f V", // V (Vout = Value/8000)
			min_val, vout_min, max_val, vout_max);
		return;
	}

	uint16_t shift_reg = fetch_page ? CUR_SHIFT_REG_RAIL_2 : CUR_SHIFT_REG_RAIL_1;
	uint8_t shift_data[2] = { 0 };
	if (!dma_read_vr(fetch_rail, shift_reg, shift_data, 2)) {
		shell_warn(shell, "vr %d dma read from 0x%x fail", fetch_rail, shift_reg);
		return;
	}

	uint16_t shift_raw = ((uint16_t)shift_data[1] << 8) | shift_data[0];
	uint8_t shift = (shift_raw >> 12) & 0xF;
	double scale_iout = 0.1;

	if (shift >= 5) {
		scale_iout /= (1U << (shift - 5));
	} else {
		scale_iout *= (1U << (5 - shift));
	}

	double iout_max = (double)max_val * scale_iout;
	shell_print(shell,
		"Iout max(raw=%d): %.6f A",//, SHIFT=%u (Scale = 1/(10*2^(SHIFT-5)))
		max_val, iout_max, shift);
}
static void voltage_rname_get(size_t idx, struct shell_static_entry *entry)
{
	if (((get_asic_board_id() != ASIC_BOARD_ID_EVB)) && (idx == VR_RAIL_E_P3V3_OSFP_VOLT_V))
		return;
	uint8_t *name = NULL;
	vr_rail_name_get((uint8_t)idx, &name);
	// only core power
	if (idx == VR_RAIL_E_P3V3_OSFP_VOLT_V) {
		return;
	}

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(voltage_rname, voltage_rname_get);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_rns_dtb_cmds, SHELL_CMD_ARG(start, &voltage_rname, "start <voltage-rail> <voltage/current> ",
		      					detector_set_config, 3, 1),
			       SHELL_CMD_ARG(stop, &voltage_rname, "stop min-Max detector [rail_name]", cmd_stop_min_max_detector, 1, 1),
				   SHELL_CMD_ARG(get_status, &voltage_rname, "get min-Max detector status [rail_name]", cmd_min_max_detector_status, 1, 1),
			       SHELL_CMD_ARG(fetch, &voltage_rname, "fetch value for min-max detector [rail_name]", cmd_fetch_min_max_detector_value, 1, 1),
				   SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(rns_dtb, &sub_rns_dtb_cmds, "RNS DTB commands", NULL);
