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

#include <stdlib.h>
#include <shell/shell.h>

#include "mctp.h"
#include "pldm.h"
#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_class.h"
#include "plat_adc.h"
#include "plat_mctp.h"
#include "shell_plat_power_sequence.h"
#include "plat_log.h"
#include "plat_i2c.h"

bool shell_clk_312_5mhz_read_or_write_reg_value(const struct shell *shell, uint8_t tx_len,
						uint8_t rx_len, uint8_t hsb_value,
						uint8_t lsb_value, uint8_t *write_read_value)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS3;
	i2c_msg.target_addr = 0x8; // 7-bit
	i2c_msg.tx_len = tx_len;
	i2c_msg.rx_len = rx_len;
	i2c_msg.data[0] = hsb_value; //offset HSB
	i2c_msg.data[1] = lsb_value; //offset LSB
	if (rx_len > 0) {
		if (i2c_master_read(&i2c_msg, retry)) {
			shell_error(shell, "Failed to read clk 312.5MHz reg, offset: 0x%02x%02x",
				    hsb_value, lsb_value);
			return false; // return invalid value
		}
	} else {
		for (int i = 2; i < tx_len; i++) {
			i2c_msg.data[i] = write_read_value[i - 2];
		}
		if (i2c_master_write(&i2c_msg, retry)) {
			shell_error(shell, "Failed to write clk 312.5MHz reg, offset: 0x%02x%02x",
				    hsb_value, lsb_value);
			return false; // return invalid value
		}
		return true;
	}
	// save read back value in *write_read_value
	memcpy(write_read_value, i2c_msg.data, rx_len);
	return true;
}
// test command
void cmd_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "Hello world!");
	uint8_t tx = strtol(argv[1], NULL, 16);
	uint8_t rx = strtol(argv[2], NULL, 16);
	uint8_t hsb = strtol(argv[3], NULL, 16);
	uint8_t lsb = strtol(argv[4], NULL, 16);
	shell_print(shell, "argc: 0x%02x, tx: 0x%02x, rx: 0x%02x, hsb: 0x%02x, lsb: 0x%02x", argc,
		    tx, rx, hsb, lsb);
	uint8_t write_read_value[10] = { 0 };
	if (rx == 0) {
		for (int i = 0; i < argc - 5; i++) {
			write_read_value[i] = strtol(argv[5 + i], NULL, 16);
			shell_print(shell, "Write Data[%d]: 0x%02x", i, write_read_value[i]);
		}
	}

	if (!shell_clk_312_5mhz_read_or_write_reg_value(shell, tx, rx, hsb, lsb,
							write_read_value)) {
		shell_error(shell, "Failed to read clk 312.5MHz reg, offset: 0x%02x%02x", hsb, lsb);
	}
	shell_hexdump(shell, write_read_value, rx);
}

void cmd_read_raw(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_id = strtoul(argv[1], NULL, 16);
	uint8_t offset = strtoul(argv[2], NULL, 16);
	uint8_t len = strtoul(argv[3], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if ((sensor_id == 0) || (sensor_id >= SENSOR_NUM_NUMBERS)) {
		if (!plat_read_cpld(offset, data, 1)) {
			shell_warn(shell, "cpld read 0x%02x fail", offset);
			return;
		}
	} else {
		if (!get_raw_data_from_sensor_id(sensor_id, offset, data, len)) {
			shell_warn(shell, "sensor_id 0x%02x read 0x%02x fail", sensor_id, offset);
			return;
		}
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}

void cmd_read_info(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_id = strtoul(argv[1], NULL, 16);

	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	if (cfg == NULL)
		return;

	shell_print(shell, "sensor_id 0x%02x bus: %d, addr: 0x%x(0x%x)", sensor_id, cfg->port,
		    cfg->target_addr, (cfg->target_addr >> 1));
}

void cmd_cpld_dump(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld dump <offset> <length>");
		return;
	}

	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t len = strtoul(argv[2], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if (!plat_read_cpld(offset, data, len)) {
		shell_warn(shell, "cpld read 0x%02x fail", offset);
		return;
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}
void cmd_cpld_write(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld write <offset> <data>");
		return;
	}

	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t data = strtoul(argv[2], NULL, 16);

	if (!plat_write_cpld(offset, &data)) {
		shell_warn(shell, "cpld write 0x%02x fail", offset);
		return;
	}

	shell_warn(shell, "cpld write %02x to offset %02x", data, offset);
}

void pldm_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_warn(shell, "Help: pldm <eid> <pldm_type> <pldm_cmd> <pldm_data>");
		return;
	}

	const uint8_t eid = strtol(argv[1], NULL, 16);

	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	pldm_msg pmsg = { 0 };
	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = strtol(argv[2], NULL, 16);
	pmsg.hdr.cmd = strtol(argv[3], NULL, 16);
	pmsg.hdr.rq = PLDM_REQUEST;
	pmsg.len = argc - 4;
	uint8_t req_buf[pmsg.len];
	pmsg.buf = req_buf;

	for (int i = 0; i < pmsg.len; i++) {
		pmsg.buf[i] = strtol(argv[i + 4], NULL, 16);
	}

	mctp *mctp_inst = NULL;
	if (get_mctp_info_by_eid(eid, &mctp_inst, &pmsg.ext_params) == false) {
		shell_print(shell, "Failed to get mctp info by eid 0x%x", eid);
		return;
	}

	uint16_t resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		shell_print(shell, "Failed to get mctp-pldm response");
		return;
	}

	shell_print(shell, "RESP");
	shell_hexdump(shell, resp_buf, resp_len);

	return;
}

void cmd_info(const struct shell *shell, size_t argc, char **argv)
{
	static const char *const vr_module_str[] = {
		[VR_MODULE_MPS] = "MPS",
		[VR_MODULE_RNS] = "RNS",
	};

	static const char *const ubc_module_str[] = {
		[UBC_MODULE_DELTA] = "DELTA",
		[UBC_MODULE_MPS] = "MPS",
		[UBC_MODULE_FLEX] = "FLEX",
		[UBC_MODULE_LUXSHARE] = "LUXSHARE",
	};

	static const char *const asic_board_id_str[] = {
		[ASIC_BOARD_ID_RSVD1] = "RSVD1",
		[ASIC_BOARD_ID_RSVD2] = "RSVD2",
		[ASIC_BOARD_ID_RAINBOW] = "RAINBOW",
		[ASIC_BOARD_ID_EVB] = "EVB",
	};

	uint8_t vr = get_vr_module();
	uint8_t ubc = get_ubc_module();
	uint8_t board_id = get_asic_board_id();
	uint8_t board_rev = get_board_rev_id();
	uint8_t adc_idx = get_adc_type();
	uint8_t tray_loc = get_tray_location();

	shell_warn(shell, "vr module: %s",
		   (vr < VR_MODULE_UNKNOWN) ? vr_module_str[vr] : "UNKNOWN");
	shell_warn(shell, "ubc module: %s",
		   (ubc < UBC_MODULE_UNKNOWN) ? ubc_module_str[ubc] : "UNKNOWN");
	shell_warn(shell, "mmc slot: %d", get_mmc_slot() + 1);
	shell_warn(shell, "asic board id: %s",
		   (board_id < ASIC_BOARD_ID_UNKNOWN) ? asic_board_id_str[board_id] : "UNKNOWN");
	shell_warn(shell, "asic board rev id: %d", board_rev);
	shell_warn(shell, "adc idx: %d (0:ADI, 1:TI)", adc_idx);
	shell_warn(shell, "tray location: %d", tray_loc);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpld_cmds, SHELL_CMD(dump, NULL, "cpld dump", cmd_cpld_dump),
			       SHELL_CMD(write, NULL, "write cpld register", cmd_cpld_write),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_test_cmds, SHELL_CMD(test, NULL, "test command", cmd_test),
	SHELL_CMD(read_raw, NULL, "read raw data test command", cmd_read_raw),
	SHELL_CMD(read_info, NULL, "read sensor info test command", cmd_read_info),
	SHELL_CMD(cpld, &sub_cpld_cmds, "cpld commands", NULL),
	SHELL_CMD(pldm, NULL, "send pldm to bmc", pldm_cmd),
	SHELL_CMD(info, NULL, "info commands", cmd_info), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands", NULL);
