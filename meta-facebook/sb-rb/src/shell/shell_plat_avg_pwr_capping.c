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
#include <logging/log.h>
#include <shell/shell.h>
#include "plat_hook.h"
#include "plat_class.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_user_setting.h"
#include "plat_fru.h"
#include "plat_cpld.h"
#include "plat_power_capping.h"
#include "plat_adc.h"

#define TEMP_DATA_SIZE 10000 // 10K data
LOG_MODULE_REGISTER(plat_avg_pwr_capping_shell, LOG_LEVEL_DBG);

void cmd_medha0_power_get(const struct shell *shell, size_t argc, char **argv)
{
	//get avg pwr time = time windows * 10,000 /1000
    uint16_t time_windows = get_power_capping_time_w(CAPPING_VR_IDX_MEDHA0, CAPPING_LV_IDX_LV3);
    uint16_t avg_pwr_time = time_windows * TEMP_DATA_SIZE / 1000;
    printf("Getting MEDHA0 average power, may spend %d s\n", avg_pwr_time);
    printf("Please do not enter any commands during this time....\n");
    set_avg_pwr_flag_medha0(1);
}

void cmd_medha1_power_get(const struct shell *shell, size_t argc, char **argv)
{
	//get avg pwr time = time windows * 10,000 /1000
    uint16_t time_windows = get_power_capping_time_w(CAPPING_VR_IDX_MEDHA1, CAPPING_LV_IDX_LV3);
    uint16_t avg_pwr_time = time_windows * TEMP_DATA_SIZE / 1000;
    printf("Getting MEDHA1 average power, may spend %d s\n", avg_pwr_time);
    printf("Please do not enter any commands during this time....\n");
    set_avg_pwr_flag_medha1(1);
}

SHELL_STATIC_SUBCMD_SET_CREATE(
                    sub_pwr_capping_avg_pwr_cmds,
			        SHELL_CMD(medha0, NULL, "average power get power commands", cmd_medha0_power_get),
					SHELL_CMD(medha1, NULL, "average power get power commands", cmd_medha1_power_get),
					SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(pwr_capping_avg_pwr_get, &sub_pwr_capping_avg_pwr_cmds, "pwr_capping_avg_pwr command",
		   NULL);
