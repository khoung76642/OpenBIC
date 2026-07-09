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
#include <logging/log.h>
#include "sensor.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_event.h"
#include "pldm_sensor.h"
#include "plat_user_setting.h"
#include "plat_cpld.h"

LOG_MODULE_REGISTER(plat_medha_power_shell, LOG_LEVEL_DBG);

static int cmd_medha_power_get(const struct shell *shell, size_t argc, char **argv)
{
	/* is_ubc_enabled_delayed_enabled() is to wait for all VR to be enabled  */
	/* (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) is to shut down polling immediately when UBC is disabled */
	if (!(get_is_ubc_enabled() && is_ubc_enabled_delayed_enabled())) {
		shell_error(shell, "Can't get power command because VR has no power yet.");
		return -1;
	}

	for (int i = 0; i < 4; i++) {
		uint8_t *rail_name = NULL;
		if (!medha_rail_name_get((uint8_t)i, &rail_name)) {
			shell_print(shell, "Can't find medha_rail_name by rail index: %d", i);
			continue;
		}

		uint32_t max_power = 0;
		if (!medha_get_max_power_history_by_rail(i, &max_power)) {
			shell_print(shell, "Can't find max_power by rail index: %d", i);
			continue;
		}
		sensor_val tmp_reading;
		tmp_reading.integer = (int16_t)(max_power & 0xFFFF);
		tmp_reading.fraction = (int16_t)((max_power >> 16) & 0xFFFF);

		shell_print(shell, "%4x|%-50s| %5d.%03d", i, rail_name, tmp_reading.integer,
			    tmp_reading.fraction);
	}

	return 0;
}

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_medha_power_cmds,
			       SHELL_CMD(get, NULL, "medha power get power commands",
					 cmd_medha_power_get),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(medha_power, &sub_medha_power_cmds, "medha power commands", NULL);
