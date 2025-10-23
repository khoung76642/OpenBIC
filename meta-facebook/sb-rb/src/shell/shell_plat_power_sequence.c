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

#include <zephyr.h>
#include <shell/shell.h>
#include <stdlib.h>
#include <logging/log.h>
#include "plat_isr.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_cpld.h"
#include "shell_plat_power_sequence.h"

LOG_MODULE_REGISTER(shell_plat_power_sequence, LOG_LEVEL_INF);

power_sequence power_sequence_on_table[] = {
	{ 0, P12V_ON_REG, "P12V_ON_REG", 0x00 },
	{ 1, P3V3_ON_REG, "P3V3_ON_REG", 0x00 },
	{ 2, P5V_ON_REG, "P5V_ON_REG", 0x00 },
	{ 3, LDO_IN_1V2_ON_REG, "LDO_IN_1V2_ON_REG", 0x00 },
	{ 4, P1V8_ON_REG, "P1V8_ON_REG", 0x00 },
	{ 5, P0V75_AVDD_HCSL_ON_REG, "P0V75_AVDD_HCSL_ON_REG", 0x00 },
	{ 6, HAMSA_VDD_ON_REG, "HAMSA_VDD_ON_REG", 0x00 },
	{ 7, MEDHA1_VDD_ON_REG, "MEDHA1_VDD_ON_REG", 0x00 },
	{ 8, MEDHA0_VDD_ON_REG, "MEDHA0_VDD_ON_REG", 0x00 },
	{ 9, OWL_E_VDD_ON_REG, "OWL_E_VDD_ON_REG", 0x00 },
	{ 10, OWL_W_VDD_ON_REG, "OWL_W_VDD_ON_REG", 0x00 },
	{ 11, MAX_M_VDD_ON_REG, "MAX_M_VDD_ON_REG", 0x00 },
	{ 12, MAX_N_VDD_ON_REG, "MAX_N_VDD_ON_REG", 0x00 },
	{ 13, MAX_S_VDD_ON_REG, "MAX_S_VDD_ON_REG", 0x00 },
	{ 14, OWL_E_TRVDD0P75_ON_REG, "OWL_E_TRVDD0P75_ON_REG", 0x00 },
	{ 15, OWL_W_TRVDD0P75_ON_REG, "OWL_W_TRVDD0P75_ON_REG", 0x00 },
	{ 16, VDDPHY_HBM0_HBM2_HBM4_HBM6_ON_REG, "VDDPHY_HBM0_HBM2_HBM4_HBM6_ON_REG", 0x00 },
	{ 17, VDDPHY_HBM1_HBM3_HBM5_HBM7_ON_REG, "VDDPHY_HBM1_HBM3_HBM5_HBM7_ON_REG", 0x00 },
	{ 18, P1V5_PLL_VDDA_OWL_ON_REG, "P1V5_PLL_VDDA_OWL_ON_REG", 0x00 },
	{ 19, P1V5_PLL_VDDA_SOC_ON_REG, "P1V5_PLL_VDDA_SOC_ON_REG", 0x00 },
	{ 20, PLL_VDDA15_HBM0_HBM2_ON_REG, "PLL_VDDA15_HBM0_HBM2_ON_REG", 0x00 },
	{ 21, PLL_VDDA15_HBM1_HBM3_ON_REG, "PLL_VDDA15_HBM1_HBM3_ON_REG", 0x00 },
	{ 22, PLL_VDDA15_HBM4_HBM6_ON_REG, "PLL_VDDA15_HBM4_HBM6_ON_REG", 0x00 },
	{ 23, PLL_VDDA15_HBM5_HBM7_ON_REG, "PLL_VDDA15_HBM5_HBM7_ON_REG", 0x00 },
	{ 24, IRIS_CLK_100MHZ_ON_REG, "IRIS_CLK_100MHZ_ON_REG", 0x00 },
	{ 25, IRIS_CLK_48MHZ_ON_REG, "IRIS_CLK_48MHZ_ON_REG", 0x00 },
	{ 26, IRIS_CLK_312_5_MHZ_ON_REG, "IRIS_CLK_312.5MHZ_ON_REG", 0x00 },
	{ 27, VPP_HBM0_HBM2_HBM4_HBM6_ON_REG, "VPP_HBM0_HBM2_HBM4_HBM6_ON_REG", 0x00 },
	{ 28, VPP_HBM1_HBM3_HBM5_HBM7_ON_REG, "VPP_HBM1_HBM3_HBM5_HBM7_ON_REG", 0x00 },
	{ 29, VDDQC_HBM0_HBM2_HBM4_HBM6_ON_REG, "VDDQC_HBM0_HBM2_HBM4_HBM6_ON_REG", 0x00 },
	{ 30, VDDQC_HBM1_HBM3_HBM5_HBM7_ON_REG, "VDDQC_HBM1_HBM3_HBM5_HBM7_ON_REG", 0x00 },
	{ 31, VDDQL_HBM0_HBM2_HBM4_HBM6_ON_REG, "VDDQL_HBM0_HBM2_HBM4_HBM6_ON_REG", 0x00 },
	{ 32, VDDQL_HBM1_HBM3_HBM5_HBM7_ON_REG, "VDDQL_HBM1_HBM3_HBM5_HBM7_ON_REG", 0x00 },
	{ 33, HAMSA_AVDD_PCIE_ON_REG, "HAMSA_AVDD_PCIE_ON_REG", 0x00 },
	{ 34, OWL_E_TRVDD0P9_ON_REG, "OWL_E_TRVDD0P9_ON_REG", 0x00 },
	{ 35, OWL_W_TRVDD0P9_ON_REG, "OWL_W_TRVDD0P9_ON_REG", 0x00 },
	{ 36, OWL_E_PVDD0P9_ON_REG, "OWL_E_PVDD0P9_ON_REG", 0x00 },
	{ 37, OWL_W_PVDD0P9_ON_REG, "OWL_W_PVDD0P9_ON_REG", 0x00 },
	{ 38, OWL_E_RVDD1P5_ON_REG, "OWL_E_RVDD1P5_ON_REG", 0x00 },
	{ 39, OWL_W_RVDD1P5_ON_REG, "OWL_W_RVDD1P5_ON_REG", 0x00 },
	{ 40, HAMSA_VDDHRXTX_PCIE_ON_REG, "HAMSA_VDDHRXTX_PCIE_ON_REG", 0x00 },
	{ 41, PVDD1P5_ON_REG, "PVDD1P5_ON_REG", 0x00 },
	{ 42, HAMSA_POWER_ON_RESET_PLD_L_ON_REG, "HAMSA_POWER_ON_RESET_PLD_L_ON_REG", 0x00 },
	{ 43, MEDHA0_POWER_ON_RESET_PLD_L_ON_REG, "MEDHA0_POWER_ON_RESET_PLD_L_ON_REG", 0x00 },
	{ 44, MEDHA1_POWER_ON_RESET_PLD_L_ON_REG, "MEDHA1_POWER_ON_RESET_PLD_L_ON_REG", 0x00 },
	{ 45, HAMSA_SYS_RST_PLD_L_ON_REG, "HAMSA_SYS_RST_PLD_L_ON_REG", 0x00 },
	{ 46, MEDHA0_SYS_RST_PLD_L_ON_REG, "MEDHA0_SYS_RST_PLD_L_ON_REG", 0x00 },
	{ 47, MEDHA1_SYS_RST_PLD_L_ON_REG, "MEDHA1_SYS_RST_PLD_L_ON_REG", 0x00 },
};

power_sequence power_sequence_off_table[] = {
	{ 0, PVDD1P5_DOWN_REG, "PVDD1P5_DOWN_REG", 0x00 },
	{ 1, HAMSA_VDDHRXTX_PCIE_DOWN_REG, "HAMSA_VDDHRXTX_PCIE_DOWN_REG", 0x00 },
	{ 2, P1V5_E_RVDD_DOWN_REG, "P1V5_E_RVDD_DOWN_REG", 0x00 },
	{ 3, P1V5_W_RVDD_DOWN_REG, "P1V5_W_RVDD_DOWN_REG", 0x00 },
	{ 4, P0V9_OWL_E_PVDD_DOWN_REG, "P0V9_OWL_E_PVDD_DOWN_REG", 0x00 },
	{ 5, P0V9_OWL_W_PVDD_DOWN_REG, "P0V9_OWL_W_PVDD_DOWN_REG", 0x00 },
	{ 6, OWL_E_TRVDD0P9_DOWN_REG, "OWL_E_TRVDD0P9_DOWN_REG", 0x00 },
	{ 7, OWL_W_TRVDD0P9_DOWN_REG, "OWL_W_TRVDD0P9_DOWN_REG", 0x00 },
	{ 8, HAMSA_AVDD_PCIE_DOWN_REG, "HAMSA_AVDD_PCIE_DOWN_REG", 0x00 },
	{ 9, VDDQL_HBM0_HBM2_HBM4_HBM6_DOWN_REG, "VDDQL_HBM0_HBM2_HBM4_HBM6_DOWN_REG", 0x00 },
	{ 10, VDDQL_HBM1_HBM3_HBM5_HBM7_DOWN_REG, "VDDQL_HBM1_HBM3_HBM5_HBM7_DOWN_REG", 0x00 },
	{ 11, VDDQC_HBM0_HBM2_HBM4_HBM6_DOWN_REG, "VDDQC_HBM0_HBM2_HBM4_HBM6_DOWN_REG", 0x00 },
	{ 12, VDDQC_HBM1_HBM3_HBM5_HBM7_DOWN_REG, "VDDQC_HBM1_HBM3_HBM5_HBM7_DOWN_REG", 0x00 },
	{ 13, VPP_HBM0_HBM2_HBM4_HBM6_DOWN_REG, "VPP_HBM0_HBM2_HBM4_HBM6_DOWN_REG", 0x00 },
	{ 14, VPP_HBM1_HBM3_HBM5_HBM7_DOWN_REG, "VPP_HBM1_HBM3_HBM5_HBM7_DOWN_REG", 0x00 },
	{ 15, FM_IRIS_CLK_100MHZ_EN_N_DOWN_REG, "FM_IRIS_CLK_100MHZ_EN_N_DOWN_REG", 0x00 },
	{ 16, FM_CLK_48MHZ_EN_DOWN_REG, "FM_CLK_48MHZ_EN_DOWN_REG", 0x00 },
	{ 17, FM_IRIS_CLK_312MHZ_EN_N_DOWN_REG, "FM_IRIS_CLK_312MHZ_EN_N_DOWN_REG", 0x00 },
	{ 18, PLL_VDDA15_HBM0_HBM2_DOWN_REG, "PLL_VDDA15_HBM0_HBM2_DOWN_REG", 0x00 },
	{ 19, PLL_VDDA15_HBM1_HBM3_DOWN_REG, "PLL_VDDA15_HBM1_HBM3_DOWN_REG", 0x00 },
	{ 20, PLL_VDDA15_HBM4_HBM6_DOWN_REG, "PLL_VDDA15_HBM4_HBM6_DOWN_REG", 0x00 },
	{ 21, PLL_VDDA15_HBM5_HBM7_DOWN_REG, "PLL_VDDA15_HBM5_HBM7_DOWN_REG", 0x00 },
	{ 22, P1V5_PLL_VDDA_SOC_DOWN_REG, "P1V5_PLL_VDDA_SOC_DOWN_REG", 0x00 },
	{ 23, P1V5_PLL_VDDA_OWL_DOWN_REG, "P1V5_PLL_VDDA_OWL_DOWN_REG", 0x00 },
	{ 24, P1V5_PLL_VDDA_OWL_DOWN_REG, "P1V5_PLL_VDDA_OWL_DOWN_REG", 0x00 },
	{ 25, VDDPHY_HBM0_HBM2_HBM4_HBM6_DOWN_REG, "VDDPHY_HBM0_HBM2_HBM4_HBM6_DOWN_REG", 0x00 },
	{ 26, VDDPHY_HBM1_HBM3_HBM5_HBM7_DOWN_REG, "VDDPHY_HBM1_HBM3_HBM5_HBM7_DOWN_REG", 0x00 },
	{ 27, OWL_E_TRVDD0P75_DOWN_REG, "OWL_E_TRVDD0P75_DOWN_REG", 0x00 },
	{ 28, OWL_W_TRVDD0P75_DOWN_REG, "OWL_W_TRVDD0P75_DOWN_REG", 0x00 },
	{ 29, MAX_S_VDD_DOWN_REG, "MAX_S_VDD_DOWN_REG", 0x00 },
	{ 30, MAX_N_VDD_DOWN_REG, "MAX_N_VDD_DOWN_REG", 0x00 },
	{ 31, MAX_M_VDD_DOWN_REG, "MAX_M_VDD_DOWN_REG", 0x00 },
	{ 32, OWL_E_VDD_DOWN_REG, "OWL_E_VDD_DOWN_REG", 0x00 },
	{ 33, OWL_W_VDD_DOWN_REG, "OWL_W_VDD_DOWN_REG", 0x00 },
	{ 34, MEDHA0_VDD_DOWN_REG, "MEDHA0_VDD_DOWN_REG", 0x00 },
	{ 35, MEDHA1_VDD_DOWN_REG, "MEDHA1_VDD_DOWN_REG", 0x00 },
	{ 36, HAMSA_VDD_DOWN_REG, "HAMSA_VDD_DOWN_REG", 0x00 },
	{ 37, P0V75_AVDD_HCSL_DOWN_REG, "P0V75_AVDD_HCSL_DOWN_REG", 0x00 },
	{ 38, P1V8_DOWN_REG, "P1V8_DOWN_REG", 0x00 },
	{ 39, LDO_IN_1V2_DOWN_REG, "LDO_IN_1V2_DOWN_REG", 0x00 },
	{ 40, P5V_DOWN_REG, "P5V_DOWN_REG", 0x00 },
	{ 41, P3V3_DOWN_REG, "P3V3_DOWN_REG", 0x00 },
	{ 42, P12V_UBC_DOWN_REG, "P12V_UBC_DOWN_REG", 0x00 },
	{ 43, PERST_DELAY_DOWN_REG, "PERST DELAY_DOWN_REG", 0x00 },
};

size_t power_sequence_on_table_size = ARRAY_SIZE(power_sequence_on_table);
size_t power_sequence_off_table_size = ARRAY_SIZE(power_sequence_off_table);

void bubble_sort_power_sequence_table(const struct shell *shell,
				      const power_sequence *power_sequence_table, size_t size)
{
	power_sequence sorted_power_sequence_table[size];
	memcpy(sorted_power_sequence_table, power_sequence_table, sizeof(power_sequence) * size);

	for (int i = 0; i < size - 1; i++) {
		for (int j = 0; j < size - 1 - i; j++) {
			if (sorted_power_sequence_table[j].value >
				    sorted_power_sequence_table[j + 1].value ||
			    (sorted_power_sequence_table[j].value ==
				     sorted_power_sequence_table[j + 1].value &&
			     sorted_power_sequence_table[j].index >
				     sorted_power_sequence_table[j + 1].index)) {
				power_sequence temp = sorted_power_sequence_table[j];
				sorted_power_sequence_table[j] = sorted_power_sequence_table[j + 1];
				sorted_power_sequence_table[j + 1] = temp;
			}
		}
	}
	uint8_t fail_flag = 0;
	for (size_t i = 0; i < size; i++) {
		if (sorted_power_sequence_table[i].value == 0) {
			if (fail_flag == 0) {
				shell_print(shell, "Power-up sequence fail -> %s",
					    sorted_power_sequence_table[i].power_rail_name);
				fail_flag = 1;
			}
		}
		shell_print(shell, "            [%2d]%-50s %d ms", i,
			    sorted_power_sequence_table[i].power_rail_name,
			    sorted_power_sequence_table[i].value * 2);
	}
}

int cmd_power_sequence(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: power_sequence <power_up|power_down>");
		return -1;
	}

	power_sequence *power_sequence_table;
	size_t size;
	if (!strcmp(argv[0], "power_up")) {
		shell_print(shell, "UBC_ENABLE ->");
		power_sequence_table = power_sequence_on_table;
		size = power_sequence_on_table_size;
	} else if (!strcmp(argv[0], "power_down")) {
		shell_print(shell, "RST_ATH ->");
		power_sequence_table = power_sequence_off_table;
		size = power_sequence_off_table_size;
	} else {
		shell_print(shell, "Unsupport power_sequence: %s", argv[0]);
		return -1;
	}

	for (size_t i = 0; i < size; i++) {
		uint8_t data;
		uint8_t cpld_offset = power_sequence_table[i].cpld_offsets;

		if (!plat_read_cpld(cpld_offset, &data, 1)) {
			LOG_ERR("Failed to read cpld register from cpld");
			continue;
		}
		LOG_DBG("read offset 0x%x value 0x%x", cpld_offset, data);
		power_sequence_table[i].value = data;
	}

	bubble_sort_power_sequence_table(shell, power_sequence_table, size);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_power_sequence_cmds,
			       SHELL_CMD(power_up, NULL, "power_sequence power_up command",
					 cmd_power_sequence),
			       SHELL_CMD(power_down, NULL, "power_sequence power_down command",
					 cmd_power_sequence),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(power_sequence, &sub_power_sequence_cmds, "power_sequence <power_up|power_down>",
		   NULL);
