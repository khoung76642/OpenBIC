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

#include <stdio.h>
#include "hal_gpio.h"
#include "plat_pwm.h"
#include <logging/log.h>
#include "plat_class.h"
#include "plat_modbus.h"
#include "plat_util.h"
#include "hal_gpio.h"
#include "plat_threshold.h"
#include "plat_log.h"
#include "plat_gpio.h"
#include "hal_i2c.h"
#include "nct7363.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"
#include "plat_fsc.h"
#include "plat_isr.h"

LOG_MODULE_REGISTER(plat_init);

#define DEF_PROJ_GPIO_PRIORITY 78

static void pump_board_init();
K_WORK_DELAYABLE_DEFINE(up_15sec_handler, pump_board_init);

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e789108, 0x00000500 }, // uart1,2 RS485 DE/nRE
	{ 0x7e6e24b0, 0x00008000 }, // uart2 NRTS2
	{ 0x7e6e24bc, 0x00000001 }, // uart1 NRTS1
	// disable internal PD
	{ 0x7e6e2610, 0x0FFFBFFF },
	{ 0x7e6e2614, 0x018F0100 },
	{ 0x7e6e2618, 0x0F00FF00 },
	{ 0x7e6e261C, 0xFE300005 },
	{ 0x7e6e2630, 0x00000002 },
	// OTP
	{ 0x7e620064, 0x00000000 },
};

uint8_t pump_board_init_tbl[] = {
	// pump board 1
	SENSOR_NUM_PB_1_PUMP_TACH_RPM, SENSOR_NUM_PB_1_FAN_1_TACH_RPM,
	SENSOR_NUM_PB_1_FAN_2_TACH_RPM, SENSOR_NUM_PB_1_HDC1080DMBR_TEMP_C,
	SENSOR_NUM_PB_1_HUM_PCT_RH, SENSOR_NUM_PB_1_HSC_P48V_TEMP_C,
	SENSOR_NUM_PB_1_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_PB_1_HSC_P48V_IOUT_CURR_A,
	SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W,
	// pump board 2
	SENSOR_NUM_PB_2_PUMP_TACH_RPM, SENSOR_NUM_PB_2_FAN_1_TACH_RPM,
	SENSOR_NUM_PB_2_FAN_2_TACH_RPM, SENSOR_NUM_PB_2_HDC1080DMBR_TEMP_C,
	SENSOR_NUM_PB_2_HUM_PCT_RH, SENSOR_NUM_PB_2_HSC_P48V_TEMP_C,
	SENSOR_NUM_PB_2_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_PB_2_HSC_P48V_IOUT_CURR_A,
	SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W,
	// pump board 3
	SENSOR_NUM_PB_3_PUMP_TACH_RPM, SENSOR_NUM_PB_3_FAN_1_TACH_RPM,
	SENSOR_NUM_PB_3_FAN_2_TACH_RPM, SENSOR_NUM_PB_3_HDC1080DMBR_TEMP_C,
	SENSOR_NUM_PB_3_HUM_PCT_RH, SENSOR_NUM_PB_3_HSC_P48V_TEMP_C,
	SENSOR_NUM_PB_3_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_PB_3_HSC_P48V_IOUT_CURR_A,
	SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W
};

uint8_t pb_sen_tbl[] = { SENSOR_NUM_PB_1_HSC_P48V_TEMP_C,      SENSOR_NUM_PB_1_HSC_P48V_VIN_VOLT_V,
			 SENSOR_NUM_PB_1_HSC_P48V_IOUT_CURR_A, SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W,
			 SENSOR_NUM_PB_2_HSC_P48V_TEMP_C,      SENSOR_NUM_PB_2_HSC_P48V_VIN_VOLT_V,
			 SENSOR_NUM_PB_2_HSC_P48V_IOUT_CURR_A, SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W,
			 SENSOR_NUM_PB_3_HSC_P48V_TEMP_C,      SENSOR_NUM_PB_3_HSC_P48V_VIN_VOLT_V,
			 SENSOR_NUM_PB_3_HSC_P48V_IOUT_CURR_A, SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W };

#define ADM1272_MFR_ID 0x494441
#define XDP710_MFR_ID 0x004649
#define PB_ADM1272_ADDR (0x24 >> 1)
#define XDP710_PB_ADDR (0x2C >> 1)
void pump_board_init()
{
	// init pump board 1, 2 and 3
	static uint8_t count = 0;
	uint16_t sen_nums = 0;
	const uint8_t *sen_tbl = NULL;
	uint8_t dvt_xdp_addr = 0;
	xdp710_init_arg *xdp_init_arr = NULL;

	sen_nums = ARRAY_SIZE(pb_sen_tbl);
	sen_tbl = pb_sen_tbl;
	dvt_xdp_addr = XDP710_PB_ADDR;
	xdp_init_arr = &xdp710_init_args[1];
	LOG_INF("pump board start init: %d", count);
	for (uint8_t i = 0; i < ARRAY_SIZE(pump_board_init_tbl); i++) {
		sensor_cfg *cfg = get_common_sensor_cfg_info(pump_board_init_tbl[i]);

		if (cfg->pre_sensor_read_hook) {
			if (cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args) != true) {
				LOG_ERR("init pump board pre-read fail, sensor num: 0x%x",
					cfg->num);
			}
		}

		if (cfg->num >= SENSOR_NUM_PB_1_HSC_P48V_TEMP_C &&
		    cfg->num <= SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W) {
			const uint32_t result_mfr_id =
				get_pmbus_mfr_id(cfg->port, cfg->target_addr);
			LOG_DBG("Sensor %x's HSC module MFR_ID first: 0x%06x", cfg->num,
				result_mfr_id);
			switch (result_mfr_id) {
			case 0:
				LOG_ERR("pump hsc address not access !");
				uint32_t read_xdp710_mfrid =
					get_pmbus_mfr_id(cfg->port, XDP710_PB_ADDR);
				LOG_DBG("Sensor %x's HSC module MFR_ID second: 0x%06x", cfg->num,
					read_xdp710_mfrid);
				LOG_DBG("xdp710 mfrid match: 0x%06x", XDP710_MFR_ID);
				if (read_xdp710_mfrid == XDP710_MFR_ID) {
					cfg->type = sensor_dev_xdp710;
					cfg->target_addr = dvt_xdp_addr;
					cfg->init_args = xdp_init_arr;
				}
				break;
			case XDP710_MFR_ID:
				cfg->type = sensor_dev_xdp710;
				cfg->target_addr = dvt_xdp_addr;
				cfg->init_args = xdp_init_arr;
				break;
			case ADM1272_MFR_ID:
				break;
			default:
				LOG_ERR("unknown HSC module !");
				break;
			}
			LOG_DBG("num, type, addr: %x, %x, %x", cfg->num, cfg->type,
				cfg->target_addr);
		}

		bool ret = init_drive_type_delayed(cfg);

		if (ret != true)
			LOG_ERR("init drive type fail, sensor num: 0x%x", cfg->num);

		if (cfg->post_sensor_read_hook) {
			cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL);
		}
	}

	if (count < 20) {
		k_work_schedule(&up_15sec_handler, K_SECONDS(1));
		count++;
	}
}

void pal_pre_init()
{
	set_boot_source();
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
	init_aalc_config();
	gpio_set(FM_BIC_READY_R_N, 0); //MM4 for bus3 power up
	k_msleep(10);
	// pull bpb nct7363 sensor box power high first;
	sensor_cfg plat_sensor_config = { SENSOR_NUM_BPB_RACK_LEVEL_1,
					  sensor_dev_nct7363,
					  I2C_BUS5,
					  BPB_NCT7363_ADDR,
					  NCT7363_GPIO_READ_OFFSET,
					  stby_access,
					  NCT7363_5_PORT,
					  0,
					  SAMPLE_COUNT_DEFAULT,
					  POLL_TIME_DEFAULT,
					  ENABLE_SENSOR_POLLING,
					  0,
					  SENSOR_INIT_STATUS,
					  NULL,
					  NULL,
					  NULL,
					  NULL,
					  &nct7363_init_args[17] };

	uint8_t ret = nct7363_init(&plat_sensor_config);

	if (ret)
		LOG_ERR("init result fail:0x%x", ret);

	k_work_schedule(&up_15sec_handler, K_SECONDS(1));
}

void pal_post_init()
{
	init_load_eeprom_log();
	init_pwm_dev();
	init_custom_modbus_server();
	init_modbus_command_table();
	quick_sensor_poll_init();
	set_manual_pwm_cache_to_default();
	deassert_all_rpu_ready_pin();
	fan_pump_pwrgd();
}

void pal_device_init()
{
	return;
}

void pal_set_sys_status()
{
	return;
}

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);