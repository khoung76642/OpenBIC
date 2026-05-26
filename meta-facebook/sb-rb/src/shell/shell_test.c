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
#include "pldm_monitor.h"
#include "plat_isr.h"
#include "plat_iris_smbus.h"

void cmd_test2(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "Hello world!");

	// test cper
	uint8_t eid = 0x08;
	uint8_t resp_buf[PLDM_MAX_DATA_SIZE];
	pldm_msg pmsg;
	mctp *mctp_inst;
	uint16_t resp_len;
	bool ret;

	memset(&pmsg, 0, sizeof(pmsg));
	memset(resp_buf, 0, sizeof(resp_buf));

	/*
     * Special case:
     * PlatformEventMessage with CPER payload.
     * Do NOT use argv for payload to avoid SHELL_ARGC_MAX limit.
     */

	static const uint8_t cper_record[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};

	static uint8_t event_buf[sizeof(struct pldm_platform_event_msg) +
				 sizeof(struct pldm_cper_event_data) + sizeof(cper_record)];

	struct pldm_platform_event_msg *evt = (struct pldm_platform_event_msg *)event_buf;
	struct pldm_cper_event_data *cper_evt;

	evt->format_version = 0x01;
	evt->tid = 0x01;
	evt->event_class = 0x07; /* eventClass = CPEREvent */

	cper_evt = (struct pldm_cper_event_data *)(evt->event_data);
	cper_evt->cper_format_version = 0x01; /* CPER formatVersion */
	cper_evt->cper_format_type = 0x01; /* formatType = Single CPER Section */
	cper_evt->cper_data_length = sizeof(cper_record);

	memcpy(cper_evt->cper_record, cper_record, sizeof(cper_record));

	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = 0x02;
	pmsg.hdr.cmd = 0x0A;
	pmsg.hdr.rq = PLDM_REQUEST;

	pmsg.buf = event_buf;
	pmsg.len = sizeof(event_buf);

	ret = get_mctp_info_by_eid(eid, &mctp_inst, &pmsg.ext_params);
	if (!ret) {
		shell_error(shell, "Failed to get mctp info by eid 0x%x", eid);
		return;
	}

	resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (!resp_len) {
		shell_error(shell, "Failed to get mctp-pldm response");
		return;
	}

	shell_print(shell, "RESP");
	shell_hexdump(shell, resp_buf, resp_len);
}

void cmd_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "Hello world!");

	ISR_GPIO_SMB_HAMSA_MMC_LVC33_ALERT_N();
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
	shell_warn(shell, "adc idx: %s",
		   (adc_idx == ADC_TYPE_AD4058)	 ? "AD4058" :
		   (adc_idx == ADC_TYPE_ADS7066) ? "ADS7066" :
						   "UNKNOWN");
	shell_warn(shell, "tray location: %d", tray_loc);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpld_cmds, SHELL_CMD(dump, NULL, "cpld dump", cmd_cpld_dump),
			       SHELL_CMD(write, NULL, "write cpld register", cmd_cpld_write),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_test_cmds, SHELL_CMD(test, NULL, "test command", cmd_test),
	SHELL_CMD(test2, NULL, "test command", cmd_test2),
	SHELL_CMD(read_raw, NULL, "read raw data test command", cmd_read_raw),
	SHELL_CMD(read_info, NULL, "read sensor info test command", cmd_read_info),
	SHELL_CMD(cpld, &sub_cpld_cmds, "cpld commands", NULL),
	SHELL_CMD(pldm, NULL, "send pldm to bmc", pldm_cmd),
	SHELL_CMD(info, NULL, "info commands", cmd_info), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands", NULL);
