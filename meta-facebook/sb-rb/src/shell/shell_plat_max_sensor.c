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
#include "plat_class.h"
#include "plat_hook.h"
#include "plat_cpld.h"
#include "plat_vr_test_mode.h"

void cmd_max_sensor_get(const struct shell *shell, size_t argc, char **argv)
{
	if (get_vr_module() == VR_MODULE_MPS) {
		shell_print(shell, "not supportedfor MPS");
		return;
	}

	if (!(get_is_ubc_enabled() && is_ubc_enabled_delayed_enabled())) {
		shell_error(shell, "Can't get medha vi limits because VR has no power yet.");
		return;
	}

	for (int i = 0; i < vr_vi_limit_rail_table_size; i++) {
		uint16_t val;

		if (!plat_get_vi_limit_value(i, &val)) {
			shell_error(shell, "Failed to dma read %s, reg: 0x%04x",
				    vr_vi_limit_rail_table[i].sensor_name,
				    vr_vi_limit_rail_table[i].dma_reg);
			return;
		}

		switch (vr_vi_limit_rail_table[i].dma_reg) {
		case DMA_VOUT_MAX:
		case DMA_VOUT_MIN:
			shell_print(shell, "%s: %dmV", vr_vi_limit_rail_table[i].sensor_name, val);
			break;

		case DMA_IOUT_MAX:
			shell_print(shell, "%s: %d.%03dA", vr_vi_limit_rail_table[i].sensor_name,
				    (val * 100) / 1000, (val * 100) % 1000);
			break;

		default:
			break;
		}
	}
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_max_sensor_cmds,
			       SHELL_CMD(get, NULL, "max_sensor get command", cmd_max_sensor_get),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(max_sensor, &sub_max_sensor_cmds, "max sensor commands", NULL);
