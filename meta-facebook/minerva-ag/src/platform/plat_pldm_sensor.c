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

#include <logging/log.h>
#include "pmbus.h"
#include "ast_adc.h"
#include "pdr.h"
#include "tmp431.h"
#include "sensor.h"
#include "pldm_sensor.h"
#include "pldm_monitor.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_pldm_sensor.h"
#include "plat_class.h"
#include "plat_gpio.h"

LOG_MODULE_REGISTER(plat_pldm_sensor);

void plat_pldm_sensor_change_ubc_dev();
void plat_pldm_sensor_change_vr_dev();
void plat_pldm_sensor_change_vr_addr();
void plat_pldm_sensor_change_vr_init_args();
void find_vr_addr_by_sensor_id(uint8_t sensor_id, uint8_t *vr_addr);
typedef struct plat_sensor_vr_extend_info {
	uint16_t sensor_id;
	uint8_t target_rns_addr; // ISL69260 or RAA228238
	void *mps_vr_init_args;
	void *rns_vr_init_args;
} plat_sensor_vr_extend_info;

plat_sensor_vr_extend_info plat_sensor_vr_extend_table[] = {
	{ SENSOR_NUM_OSFP_P3V3_TEMP_C, P3V3_ISL69260_ADDR },
	{ SENSOR_NUM_OSFP_P3V3_VOLT_V, P3V3_ISL69260_ADDR, .mps_vr_init_args = &mp2971_init_args[0],
	  .rns_vr_init_args = &isl69259_init_args[0] },
	{ SENSOR_NUM_OSFP_P3V3_CURR_A, P3V3_ISL69260_ADDR },
	{ SENSOR_NUM_OSFP_P3V3_PWR_W, P3V3_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C, P0V85_PVDD_RAA228238_ADDR },
	{ SENSOR_NUM_CPU_P0V85_PVDD_VOLT_V, P0V85_PVDD_RAA228238_ADDR },
	{ SENSOR_NUM_CPU_P0V85_PVDD_CURR_A, P0V85_PVDD_RAA228238_ADDR },
	{ SENSOR_NUM_CPU_P0V85_PVDD_PWR_W, P0V85_PVDD_RAA228238_ADDR },

	{ SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C, P0V75_PVDD_CH_N_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_PVDD_CH_N_VOLT_V, P0V75_PVDD_CH_N_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_PVDD_CH_N_CURR_A, P0V75_PVDD_CH_N_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_PVDD_CH_N_PWR_W, P0V75_PVDD_CH_N_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V75_MAX_PHY_N_TEMP_C, P0V75_MAX_PHY_N_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_MAX_PHY_N_VOLT_V, P0V75_MAX_PHY_N_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_MAX_PHY_N_CURR_A, P0V75_MAX_PHY_N_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_MAX_PHY_N_PWR_W, P0V75_MAX_PHY_N_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C, P0V75_PVDD_CH_S_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_PVDD_CH_S_VOLT_V, P0V75_PVDD_CH_S_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_PVDD_CH_S_CURR_A, P0V75_PVDD_CH_S_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_PVDD_CH_S_PWR_W, P0V75_PVDD_CH_S_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V75_MAX_PHY_S_TEMP_C, P0V75_MAX_PHY_S_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_MAX_PHY_S_VOLT_V, P0V75_MAX_PHY_S_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_MAX_PHY_S_CURR_A, P0V75_MAX_PHY_S_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_MAX_PHY_S_PWR_W, P0V75_MAX_PHY_S_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C, P0V75_TRVDD_ZONEA_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_VOLT_V, P0V75_TRVDD_ZONEA_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_CURR_A, P0V75_TRVDD_ZONEA_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_PWR_W, P0V75_TRVDD_ZONEA_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_TEMP_C, P1V8_VPP_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_VOLT_V, P1V8_VPP_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_CURR_A, P1V8_VPP_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_PWR_W, P1V8_VPP_HBM0_HBM2_HBM4_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C, P0V75_TRVDD_ZONEB_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_VOLT_V, P0V75_TRVDD_ZONEB_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_CURR_A, P0V75_TRVDD_ZONEB_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_PWR_W, P0V75_TRVDD_ZONEB_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C, P0V4_VDDQL_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_VOLT_V, P0V4_VDDQL_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_CURR_A, P0V4_VDDQL_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_PWR_W, P0V4_VDDQL_HBM0_HBM2_HBM4_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C, P1V1_VDDC_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_VOLT_V, P1V1_VDDC_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_CURR_A, P1V1_VDDC_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_PWR_W, P1V1_VDDC_HBM0_HBM2_HBM4_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C, P0V75_VDDPHY_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_VOLT_V, P0V75_VDDPHY_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_CURR_A, P0V75_VDDPHY_HBM0_HBM2_HBM4_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_PWR_W, P0V75_VDDPHY_HBM0_HBM2_HBM4_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C, P0V9_TRVDD_ZONEA_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_VOLT_V, P0V9_TRVDD_ZONEA_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_CURR_A, P0V9_TRVDD_ZONEA_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_PWR_W, P0V9_TRVDD_ZONEA_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_TEMP_C, P1V8_VPP_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_VOLT_V, P1V8_VPP_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_CURR_A, P1V8_VPP_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_PWR_W, P1V8_VPP_HBM1_HBM3_HBM5_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C, P0V9_TRVDD_ZONEB_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_VOLT_V, P0V9_TRVDD_ZONEB_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_CURR_A, P0V9_TRVDD_ZONEB_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_PWR_W, P0V9_TRVDD_ZONEB_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C, P0V4_VDDQL_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_VOLT_V, P0V4_VDDQL_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_CURR_A, P0V4_VDDQL_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_PWR_W, P0V4_VDDQL_HBM1_HBM3_HBM5_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C, P1V1_VDDC_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_VOLT_V, P1V1_VDDC_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_CURR_A, P1V1_VDDC_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_PWR_W, P1V1_VDDC_HBM1_HBM3_HBM5_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C, P0V75_VDDPHY_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_VOLT_V, P0V75_VDDPHY_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_CURR_A, P0V75_VDDPHY_HBM1_HBM3_HBM5_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_PWR_W, P0V75_VDDPHY_HBM1_HBM3_HBM5_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C, P0V8_VDDA_PCIE_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V8_VDDA_PCIE_VOLT_V, P0V8_VDDA_PCIE_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V8_VDDA_PCIE_CURR_A, P0V8_VDDA_PCIE_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P0V8_VDDA_PCIE_PWR_W, P0V8_VDDA_PCIE_ISL69260_ADDR },

	{ SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_TEMP_C, P1V2_VDDHTX_PCIE_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_VOLT_V, P1V2_VDDHTX_PCIE_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_CURR_A, P1V2_VDDHTX_PCIE_ISL69260_ADDR },
	{ SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_PWR_W, P1V2_VDDHTX_PCIE_ISL69260_ADDR },
};

static struct pldm_sensor_thread pal_pldm_sensor_thread[MAX_SENSOR_THREAD_ID] = {
	// thread id, thread name
	{ UBC_SENSOR_THREAD_ID, "UBC_PLDM_SENSOR_THREAD" },
	{ VR_SENSOR_THREAD_ID, "VR_PLDM_SENSOR_THREAD" },
	{ TEMP_SENSOR_THREAD_ID, "TEMP_SENSOR_THREAD" },
};

pldm_sensor_info plat_pldm_sensor_ubc_table[] = {
	{
		{
			// AEGIS_UBC_1_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_1_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0001, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_1_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_UBC_1_P50V_VIN_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_1_P50V_VIN_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_1_ADDR,
			.offset = PMBUS_READ_VIN,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_UBC_1_P12V_VOUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_1_P12V_VOUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_1_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_UBC_1_P12V_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_1_P12V_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0004, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_1_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_UBC_1_P12V_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_1_P12V_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0005, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_1_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_UBC_2_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_2_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0006, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_2_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_UBC_2_P50V_VIN_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_2_P50V_VIN_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0007, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_2_ADDR,
			.offset = PMBUS_READ_VIN,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_UBC_2_P12V_VOUT_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_2_P12V_VOUT_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0008, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_2_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_UBC_2_P12V_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_2_P12V_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0009, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_2_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_UBC_2_P12V_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_UBC_2_P12V_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x000A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_u50su4p180pmdafc,
			.port = I2C_BUS1,
			.target_addr = DC_BRICK_2_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_vr_table[] = {
	{
		{
			// AEGIS_OSFP_P3V3_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_OSFP_P3V3_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0001, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = P3V3_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_OSFP_P3V3_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_OSFP_P3V3_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = P3V3_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
			.init_args = NULL,
		},
	},
	{
		{
			// AEGIS_OSFP_P3V3_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_OSFP_P3V3_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = P3V3_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_OSFP_P3V3_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_OSFP_P3V3_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0004, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS1,
			.target_addr = P3V3_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V85_PVDD_VOLT_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0001, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2891,
			.port = I2C_BUS2,
			.target_addr = P0V85_PVDD_MP2891_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V85_PVDD_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V85_PVDD_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2891,
			.port = I2C_BUS2,
			.target_addr = P0V85_PVDD_MP2891_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V85_PVDD_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V85_PVDD_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2891,
			.port = I2C_BUS2,
			.target_addr = P0V85_PVDD_MP2891_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V85_PVDD_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V85_PVDD_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0004, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2891,
			.port = I2C_BUS2,
			.target_addr = P0V85_PVDD_MP2891_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_PVDD_CH_N_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0005, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_PVDD_CH_N_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_PVDD_CH_N_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_PVDD_CH_N_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0006, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_PVDD_CH_N_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_PVDD_CH_N_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_PVDD_CH_N_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0007, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_PVDD_CH_N_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_PVDD_CH_N_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_PVDD_CH_N_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0008, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_PVDD_CH_N_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_MAX_PHY_N_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_MAX_PHY_N_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0009, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_MAX_PHY_N_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_MAX_PHY_N_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_MAX_PHY_N_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x000A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_MAX_PHY_N_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_MAX_PHY_N_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_MAX_PHY_N_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x000B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_MAX_PHY_N_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_MAX_PHY_N_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_MAX_PHY_N_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x000C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_MAX_PHY_N_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_PVDD_CH_S_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x000D, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_PVDD_CH_S_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_PVDD_CH_S_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_PVDD_CH_S_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x000E, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_PVDD_CH_S_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_PVDD_CH_S_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_PVDD_CH_S_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x000F, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_PVDD_CH_S_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_PVDD_CH_S_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_PVDD_CH_S_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0010, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_PVDD_CH_S_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_MAX_PHY_S_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_MAX_PHY_S_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0011, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_MAX_PHY_S_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_MAX_PHY_S_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_MAX_PHY_S_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0012, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_MAX_PHY_S_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_MAX_PHY_S_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_MAX_PHY_S_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0013, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_MAX_PHY_S_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_MAX_PHY_S_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_MAX_PHY_S_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0014, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_MAX_PHY_S_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_TRVDD_ZONEA_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0015, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_TRVDD_ZONEA_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_TRVDD_ZONEA_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0016, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_TRVDD_ZONEA_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_TRVDD_ZONEA_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0017, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_TRVDD_ZONEA_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_TRVDD_ZONEA_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0018, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_TRVDD_ZONEA_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P1V8_VPP_HBM0_2_4_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0019, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P1V8_VPP_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V8_VPP_HBM0_2_4_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x001A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P1V8_VPP_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V8_VPP_HBM0_2_4_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x001B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P1V8_VPP_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V8_VPP_HBM0_2_4_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x001C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P1V8_VPP_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_TRVDD_ZONEB_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x001D, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_TRVDD_ZONEB_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_TRVDD_ZONEB_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x001E, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_TRVDD_ZONEB_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_TRVDD_ZONEB_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x001F, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_TRVDD_ZONEB_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_TRVDD_ZONEB_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0020, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_TRVDD_ZONEB_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0021, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V4_VDDQL_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0022, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V4_VDDQL_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0023, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V4_VDDQL_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0024, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V4_VDDQL_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0025, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P1V1_VDDC_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P1V1_VDDC_HBM0_2_4_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0026, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P1V1_VDDC_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P1V1_VDDC_HBM0_2_4_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0027, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P1V1_VDDC_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P1V1_VDDC_HBM0_2_4_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0028, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P1V1_VDDC_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0029, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_VDDPHY_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x002A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_VDDPHY_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x002B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_VDDPHY_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x003B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS2,
			.target_addr = P0V75_VDDPHY_HBM0_HBM2_HBM4_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V9_TRVDD_ZONEA_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x003C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V9_TRVDD_ZONEA_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V9_TRVDD_ZONEA_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x003D, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V9_TRVDD_ZONEA_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V9_TRVDD_ZONEA_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x003E, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V9_TRVDD_ZONEA_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V9_TRVDD_ZONEA_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x003F, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V9_TRVDD_ZONEA_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P1V8_VPP_HBM1_3_5_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0040, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V8_VPP_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V8_VPP_HBM1_3_5_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0041, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V8_VPP_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V8_VPP_HBM1_3_5_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0042, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V8_VPP_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V8_VPP_HBM1_3_5_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0043, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V8_VPP_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V9_TRVDD_ZONEB_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0044, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V9_TRVDD_ZONEB_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V9_TRVDD_ZONEB_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0045, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V9_TRVDD_ZONEB_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V9_TRVDD_ZONEB_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0046, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V9_TRVDD_ZONEB_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V9_TRVDD_ZONEB_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0047, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V9_TRVDD_ZONEB_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0048, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V4_VDDQL_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0049, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V4_VDDQL_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x004A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V4_VDDQL_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x004B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V4_VDDQL_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x004C, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V1_VDDC_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P1V1_VDDC_HBM1_3_5_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x004D, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V1_VDDC_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P1V1_VDDC_HBM1_3_5_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x004E, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V1_VDDC_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P1V1_VDDC_HBM1_3_5_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x004F, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V1_VDDC_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0050, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V75_VDDPHY_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0051, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V75_VDDPHY_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0052, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V75_VDDPHY_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0053, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V75_VDDPHY_HBM1_HBM3_HBM5_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P0V8_VDDA_PCIE_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0054, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V8_VDDA_PCIE_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V8_VDDA_PCIE_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V8_VDDA_PCIE_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0055, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V8_VDDA_PCIE_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V8_VDDA_PCIE_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V8_VDDA_PCIE_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0056, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V8_VDDA_PCIE_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P0V8_VDDA_PCIE_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P0V8_VDDA_PCIE_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0057, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P0V8_VDDA_PCIE_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[0],
		},
	},
	{
		{
			// AEGIS_CPU_P1V2_VDDHTX_PCIE_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0058, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V2_VDDHTX_PCIE_MP2971_ADDR,
			.offset = PMBUS_READ_TEMPERATURE_1,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V2_VDDHTX_PCIE_VOLT_V
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_VOLT_V, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0059, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x05, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V2_VDDHTX_PCIE_MP2971_ADDR,
			.offset = PMBUS_READ_VOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V2_VDDHTX_PCIE_CURR_A
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_CURR_A, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x005A, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x06, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V2_VDDHTX_PCIE_MP2971_ADDR,
			.offset = PMBUS_READ_IOUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
	{
		{
			// AEGIS_CPU_P1V2_VDDHTX_PCIE_PWR_W
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_PWR_W, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x005B, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x07, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_mp2971,
			.port = I2C_BUS3,
			.target_addr = P1V2_VDDHTX_PCIE_MP2971_ADDR,
			.offset = PMBUS_READ_POUT,
			.access_checker = is_dc_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
			.pre_sensor_read_hook = pre_vr_read,
			.pre_sensor_read_args = &vr_pre_read_args[1],
		},
	},
};

pldm_sensor_info plat_pldm_sensor_temp_table[] = {
	{
		{
			// AEGIS_TOP_INLET_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_TOP_INLET_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0001, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = TOP_INLET_TEMP_ADDR,
			.offset = TMP75_TEMP_OFFSET,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// AEGIS_TOP_OUTLET_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_TOP_OUTLET_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0002, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = TOP_OUTLET_TEMP_ADDR,
			.offset = TMP75_TEMP_OFFSET,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// AEGIS_BOT_INLET_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_BOT_INLET_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0003, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = BOT_INLET_TEMP_ADDR,
			.offset = TMP75_TEMP_OFFSET,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// AEGIS_BOT_OUTLET_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_BOT_OUTLET_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0004, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp75,
			.port = I2C_BUS1,
			.target_addr = BOT_OUTLET_TEMP_ADDR,
			.offset = TMP75_TEMP_OFFSET,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// AEGIS_ON_DIE_1_TEMP__C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ON_DIE_1_TEMP__C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0005, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp431,
			.port = I2C_BUS1,
			.target_addr = ON_DIE_1_TEMP_ADDR,
			.offset = TMP431_LOCAL_TEMPERATRUE,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
	{
		{
			// AEGIS_ON_DIE_2_TEMP_C
			/*** PDR common header***/
			{
				0x00000000, //uint32_t record_handle
				0x01, //uint8_t PDR_header_version
				PLDM_NUMERIC_SENSOR_PDR, //uint8_t PDR_type
				0x0000, //uint16_t record_change_number
				0x0000, //uint16_t data_length
			},

			/***numeric sensor format***/
			0x0000, //uint16_t PLDM_terminus_handle;
			SENSOR_NUM_ON_DIE_2_TEMP_C, //uint16_t sensor_id;
			0x0000, //uint16_t entity_type; //Need to check
			0x0006, //uint16_t entity_instance_number;
			0x0000, //uint16_t container_id;
			0x00, //uint8_t sensor_init; //Need to check
			0x01, //uint8_t sensor_auxiliary_names_pdr;
			0x02, //uint8_t base_unit;  //unit
			0, //int8_t unit_modifier; //Need to check
			0x00, //uint8_t rate_unit;
			0x00, //uint8_t base_oem_unit_handle;
			0x00, //uint8_t aux_unit;
			0x00, //int8_t aux_unit_modifier;
			0x00, //uint8_t auxrate_unit;
			0x00, //uint8_t rel;
			0x00, //uint8_t aux_oem_unit_handle;
			0x00, //uint8_t is_linear;
			0x4, //uint8_t sensor_data_size;
			1, //real32_t resolution;
			0, //real32_t offset;
			0x0000, //uint16_t accuracy;
			0x00, //uint8_t plus_tolerance;
			0x00, //uint8_t minus_tolerance;
			0x00000000, //uint32_t hysteresis;
			0x00, //uint8_t supported_thresholds; //Need to check
			0x00, //uint8_t threshold_and_hysteresis_volatility;
			0, //real32_t state_transition_interval;
			UPDATE_INTERVAL_1S, //real32_t update_interval;
			0x00000000, //uint32_t max_readable; //Need to check
			0x00000000, //uint32_t min_readable;
			0x04, //uint8_t range_field_format;
			0x00, //uint8_t range_field_support; //Need to check
			0x00000000, //uint32_t nominal_value;
			0x00000000, //uint32_t normal_max;
			0x00000000, //uint32_t normal_min;
			0x00000000, //uint32_t warning_high; //Need to check
			0x00000000, //uint32_t warning_low; //Need to check
			0x00000000, //uint32_t critical_high; //Need to check
			0x00000000, //uint32_t critical_low; //Need to check
			0x00000000, //uint32_t fatal_high; //Need to check
			0x00000000, //uint32_t fatal_low; //Need to check
		},
		.update_time = 0,
		{
			.type = sensor_dev_tmp431,
			.port = I2C_BUS1,
			.target_addr = ON_DIE_2_TEMP_ADDR,
			.offset = TMP431_LOCAL_TEMPERATRUE,
			.access_checker = stby_access,
			.sample_count = SAMPLE_COUNT_DEFAULT,
			.cache = 0,
			.cache_status = PLDM_SENSOR_INITIALIZING,
		},
	},
};

PDR_sensor_auxiliary_names plat_pdr_sensor_aux_names_table[] = {
	{

		// AEGIS_UBC_1_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_1_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_1_TEMP_C",
	},
	{

		// AEGIS_UBC_1_P50V_VIN_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_1_P50V_VIN_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_1_P50V_VIN_VOLT_V",
	},
	{

		// AEGIS_UBC_1_P12V_VOUT_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_1_P12V_VOUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_1_P12V_VOUT_VOLT_V",
	},
	{

		// AEGIS_UBC_1_P12V_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_1_P12V_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_1_P12V_CURR_A",
	},
	{

		// AEGIS_UBC_1_P12V_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_1_P12V_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_1_P12V_PWR_W",
	},
	{

		// AEGIS_UBC_2_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_2_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_2_TEMP_C",
	},
	{

		// AEGIS_UBC_2_P50V_VIN_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_2_P50V_VIN_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_2_P50V_VIN_VOLT_V",
	},
	{

		// AEGIS_UBC_2_P12V_VOUT_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_2_P12V_VOUT_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_2_P12V_VOUT_VOLT_V",
	},
	{

		// AEGIS_UBC_2_P12V_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_2_P12V_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_2_P12V_CURR_A",
	},
	{

		// AEGIS_UBC_2_P12V_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_UBC_2_P12V_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_UBC_2_P12V_PWR_W",
	},
	{

		// AEGIS_TOP_INLET_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_TOP_INLET_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_TOP_INLET_TEMP_C",
	},
	{

		// AEGIS_TOP_OUTLET_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_TOP_OUTLET_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_TOP_OUTLET_TEMP_C",
	},
	{

		// AEGIS_BOT_INLET_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_BOT_INLET_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_BOT_INLET_TEMP_C",
	},
	{

		// AEGIS_BOT_OUTLET_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_BOT_OUTLET_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_BOT_OUTLET_TEMP_C",
	},
	{

		// AEGIS_ON_DIE_1_TEMP__C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ON_DIE_1_TEMP__C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_ON_DIE_1_TEMP__C",
	},
	{

		// AEGIS_ON_DIE_2_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_ON_DIE_2_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_ON_DIE_2_TEMP_C",
	},
	{

		// AEGIS_OSFP_P3V3_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_OSFP_P3V3_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_OSFP_P3V3_TEMP_C",
	},
	{

		// AEGIS_OSFP_P3V3_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_OSFP_P3V3_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_OSFP_P3V3_VOLT_V",
	},
	{

		// AEGIS_OSFP_P3V3_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_OSFP_P3V3_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_OSFP_P3V3_CURR_A",
	},
	{

		// AEGIS_OSFP_P3V3_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_OSFP_P3V3_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_OSFP_P3V3_PWR_W",
	},
	{

		// AEGIS_CPU_P0V85_PVDD_VOLT_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V85_PVDD_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V85_PVDD_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V85_PVDD_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V85_PVDD_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V85_PVDD_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V85_PVDD_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V85_PVDD_CURR_A",
	},
	{

		// AEGIS_CPU_P0V85_PVDD_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V85_PVDD_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V85_PVDD_PWR_W",
	},
	{

		// AEGIS_CPU_P0V75_PVDD_CH_N_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_PVDD_CH_N_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V75_PVDD_CH_N_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_PVDD_CH_N_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_PVDD_CH_N_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V75_PVDD_CH_N_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_PVDD_CH_N_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_PVDD_CH_N_CURR_A",
	},
	{

		// AEGIS_CPU_P0V75_PVDD_CH_N_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_PVDD_CH_N_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_PVDD_CH_N_PWR_W",
	},
	{

		// AEGIS_CPU_P0V75_MAX_PHY_N_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_MAX_PHY_N_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_MAX_PHY_N_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V75_MAX_PHY_N_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_MAX_PHY_N_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_MAX_PHY_N_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V75_MAX_PHY_N_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_MAX_PHY_N_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_MAX_PHY_N_CURR_A",
	},
	{

		// AEGIS_CPU_P0V75_MAX_PHY_N_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_MAX_PHY_N_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_MAX_PHY_N_PWR_W",
	},
	{

		// AEGIS_CPU_P0V75_PVDD_CH_S_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_PVDD_CH_S_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V75_PVDD_CH_S_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_PVDD_CH_S_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_PVDD_CH_S_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V75_PVDD_CH_S_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_PVDD_CH_S_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_PVDD_CH_S_CURR_A",
	},
	{

		// AEGIS_CPU_P0V75_PVDD_CH_S_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_PVDD_CH_S_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_PVDD_CH_S_PWR_W",
	},
	{

		// AEGIS_CPU_P0V75_MAX_PHY_S_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_MAX_PHY_S_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_MAX_PHY_S_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V75_MAX_PHY_S_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_MAX_PHY_S_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_MAX_PHY_S_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V75_MAX_PHY_S_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_MAX_PHY_S_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_MAX_PHY_S_CURR_A",
	},
	{

		// AEGIS_CPU_P0V75_MAX_PHY_S_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_MAX_PHY_S_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_MAX_PHY_S_PWR_W",
	},
	{

		// AEGIS_CPU_P0V75_TRVDD_ZONEA_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_TRVDD_ZONEA_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V75_TRVDD_ZONEA_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_TRVDD_ZONEA_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V75_TRVDD_ZONEA_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_TRVDD_ZONEA_CURR_A",
	},
	{

		// AEGIS_CPU_P0V75_TRVDD_ZONEA_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_TRVDD_ZONEA_PWR_W",
	},
	{

		// AEGIS_CPU_P1V8_VPP_HBM0_2_4_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V8_VPP_HBM0_2_4_TEMP_C",
	},
	{

		// AEGIS_CPU_P1V8_VPP_HBM0_2_4_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V8_VPP_HBM0_2_4_VOLT_V",
	},
	{

		// AEGIS_CPU_P1V8_VPP_HBM0_2_4_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V8_VPP_HBM0_2_4_CURR_A",
	},
	{

		// AEGIS_CPU_P1V8_VPP_HBM0_2_4_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V8_VPP_HBM0_2_4_PWR_W",
	},
	{

		// AEGIS_CPU_P0V75_TRVDD_ZONEB_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_TRVDD_ZONEB_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V75_TRVDD_ZONEB_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_TRVDD_ZONEB_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V75_TRVDD_ZONEB_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_TRVDD_ZONEB_CURR_A",
	},
	{

		// AEGIS_CPU_P0V75_TRVDD_ZONEB_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_TRVDD_ZONEB_PWR_W",
	},
	{

		// AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_CURR_A",
	},
	{

		// AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V4_VDDQL_HBM0_2_4_PWR_W",
	},
	{

		// AEGIS_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C",
	},
	{

		// AEGIS_CPU_P1V1_VDDC_HBM0_2_4_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V1_VDDC_HBM0_2_4_VOLT_V",
	},
	{

		// AEGIS_CPU_P1V1_VDDC_HBM0_2_4_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V1_VDDC_HBM0_2_4_CURR_A",
	},
	{

		// AEGIS_CPU_P1V1_VDDC_HBM0_2_4_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V1_VDDC_HBM0_2_4_PWR_W",
	},
	{

		// AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_CURR_A",
	},
	{

		// AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_VDDPHY_HBM0_2_4_PWR_W",
	},
	{

		// AEGIS_CPU_P0V9_TRVDD_ZONEA_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V9_TRVDD_ZONEA_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V9_TRVDD_ZONEA_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V9_TRVDD_ZONEA_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V9_TRVDD_ZONEA_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V9_TRVDD_ZONEA_CURR_A",
	},
	{

		// AEGIS_CPU_P0V9_TRVDD_ZONEA_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V9_TRVDD_ZONEA_PWR_W",
	},
	{

		// AEGIS_CPU_P1V8_VPP_HBM1_3_5_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V8_VPP_HBM1_3_5_TEMP_C",
	},
	{

		// AEGIS_CPU_P1V8_VPP_HBM1_3_5_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V8_VPP_HBM1_3_5_VOLT_V",
	},
	{

		// AEGIS_CPU_P1V8_VPP_HBM1_3_5_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V8_VPP_HBM1_3_5_CURR_A",
	},
	{

		// AEGIS_CPU_P1V8_VPP_HBM1_3_5_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V8_VPP_HBM1_3_5_PWR_W",
	},
	{

		// AEGIS_CPU_P0V9_TRVDD_ZONEB_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V9_TRVDD_ZONEB_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V9_TRVDD_ZONEB_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V9_TRVDD_ZONEB_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V9_TRVDD_ZONEB_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V9_TRVDD_ZONEB_CURR_A",
	},
	{

		// AEGIS_CPU_P0V9_TRVDD_ZONEB_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V9_TRVDD_ZONEB_PWR_W",
	},
	{

		// AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_CURR_A",
	},
	{

		// AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V4_VDDQL_HBM1_3_5_PWR_W",
	},
	{

		// AEGIS_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C",
	},
	{

		// AEGIS_CPU_P1V1_VDDC_HBM1_3_5_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V1_VDDC_HBM1_3_5_VOLT_V",
	},
	{

		// AEGIS_CPU_P1V1_VDDC_HBM1_3_5_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V1_VDDC_HBM1_3_5_CURR_A",
	},
	{

		// AEGIS_CPU_P1V1_VDDC_HBM1_3_5_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V1_VDDC_HBM1_3_5_PWR_W",
	},
	{

		// AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_CURR_A",
	},
	{

		// AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V75_VDDPHY_HBM1_3_5_PWR_W",
	},
	{

		// AEGIS_CPU_P0V8_VDDA_PCIE_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V8_VDDA_PCIE_TEMP_C",
	},
	{

		// AEGIS_CPU_P0V8_VDDA_PCIE_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V8_VDDA_PCIE_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V8_VDDA_PCIE_VOLT_V",
	},
	{

		// AEGIS_CPU_P0V8_VDDA_PCIE_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V8_VDDA_PCIE_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V8_VDDA_PCIE_CURR_A",
	},
	{

		// AEGIS_CPU_P0V8_VDDA_PCIE_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P0V8_VDDA_PCIE_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P0V8_VDDA_PCIE_PWR_W",
	},
	{

		// AEGIS_CPU_P1V2_VDDHTX_PCIE_TEMP_C
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_TEMP_C,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V2_VDDHTX_PCIE_TEMP_C",
	},
	{

		// AEGIS_CPU_P1V2_VDDHTX_PCIE_VOLT_V
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_VOLT_V,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V2_VDDHTX_PCIE_VOLT_V",
	},
	{

		// AEGIS_CPU_P1V2_VDDHTX_PCIE_CURR_A
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_CURR_A,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V2_VDDHTX_PCIE_CURR_A",
	},
	{

		// AEGIS_CPU_P1V2_VDDHTX_PCIE_PWR_W
		/*** PDR common header***/
		{
			.record_handle = 0x00000000,
			.PDR_header_version = 0x01,
			.PDR_type = PLDM_SENSOR_AUXILIARY_NAMES_PDR,
			.record_change_number = 0x0000,
			.data_length = 0x0000,
		},
		.terminus_handle = 0x0000,
		.sensor_id = SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_PWR_W,
		.sensor_count = 0x1,
		.nameStringCount = 0x1,
		.nameLanguageTag = "en",
		.sensorName = u"AEGIS_CPU_P1V2_VDDHTX_PCIE_PWR_W",
	},
};

uint32_t plat_get_pdr_size(uint8_t pdr_type)
{
	int total_size = 0, i = 0;

	switch (pdr_type) {
	case PLDM_NUMERIC_SENSOR_PDR:
		for (i = 0; i < MAX_SENSOR_THREAD_ID; i++) {
			total_size += plat_pldm_sensor_get_sensor_count(i);
		}
		break;
	case PLDM_SENSOR_AUXILIARY_NAMES_PDR:
		total_size = ARRAY_SIZE(plat_pdr_sensor_aux_names_table);
		break;
	default:
		break;
	}

	return total_size;
}

pldm_sensor_thread *plat_pldm_sensor_load_thread()
{
	return pal_pldm_sensor_thread;
}

pldm_sensor_info *plat_pldm_sensor_load(int thread_id)
{
	switch (thread_id) {
	case UBC_SENSOR_THREAD_ID:
		plat_pldm_sensor_change_ubc_dev();
		return plat_pldm_sensor_ubc_table;
	case VR_SENSOR_THREAD_ID:
		plat_pldm_sensor_change_vr_dev();
		plat_pldm_sensor_change_vr_addr();
		plat_pldm_sensor_change_vr_init_args();
		return plat_pldm_sensor_vr_table;
	case TEMP_SENSOR_THREAD_ID:
		return plat_pldm_sensor_temp_table;
	default:
		LOG_ERR("Unknow pldm sensor thread id %d", thread_id);
		return NULL;
	}
}

int plat_pldm_sensor_get_sensor_count(int thread_id)
{
	int count = 0;

	switch (thread_id) {
	case UBC_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_ubc_table);
		break;
	case VR_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_vr_table);
		break;
	case TEMP_SENSOR_THREAD_ID:
		count = ARRAY_SIZE(plat_pldm_sensor_temp_table);
		break;
	default:
		count = -1;
		LOG_ERR("Unknow pldm sensor thread id %d", thread_id);
		break;
	}

	return count;
}

void plat_pldm_sensor_get_pdr_numeric_sensor(int thread_id, int sensor_num,
					     PDR_numeric_sensor *numeric_sensor_table)
{
	switch (thread_id) {
	case UBC_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_ubc_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case VR_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_vr_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	case TEMP_SENSOR_THREAD_ID:
		memcpy(numeric_sensor_table,
		       &plat_pldm_sensor_temp_table[sensor_num].pdr_numeric_sensor,
		       sizeof(PDR_numeric_sensor));
		break;
	default:
		LOG_ERR("Unknow pldm sensor thread id %d", thread_id);
		break;
	}
}

void plat_load_numeric_sensor_pdr_table(PDR_numeric_sensor *numeric_sensor_table)
{
	int thread_id = 0, sensor_num = 0;
	int max_sensor_num = 0, current_sensor_size = 0;

	for (thread_id = 0; thread_id < MAX_SENSOR_THREAD_ID; thread_id++) {
		max_sensor_num = plat_pldm_sensor_get_sensor_count(thread_id);
		for (sensor_num = 0; sensor_num < max_sensor_num; sensor_num++) {
			plat_pldm_sensor_get_pdr_numeric_sensor(
				thread_id, sensor_num, &numeric_sensor_table[current_sensor_size]);
			current_sensor_size++;
		}
	}
}

void plat_load_aux_sensor_names_pdr_table(PDR_sensor_auxiliary_names *aux_sensor_name_table)
{
	memcpy(aux_sensor_name_table, &plat_pdr_sensor_aux_names_table,
	       sizeof(plat_pdr_sensor_aux_names_table));
}

void find_vr_addr_by_sensor_id(uint8_t sensor_id, uint8_t *vr_addr)
{
	for (int index = 0; index < ARRAY_SIZE(plat_sensor_vr_extend_table); index++) {
		if (plat_sensor_vr_extend_table[index].sensor_id == sensor_id) {
			*vr_addr = plat_sensor_vr_extend_table[index].target_rns_addr;
			return;
		}
	}
}

void find_init_args_by_sensor_id(uint16_t sensor_id, void **init_args)
{
	uint8_t vr_type = get_vr_type();

	for (int index = 0; index < plat_pldm_sensor_get_sensor_count(VR_SENSOR_THREAD_ID);
	     index++) {
		if (plat_pldm_sensor_vr_table[index].pdr_numeric_sensor.sensor_id ==
		    SENSOR_NUM_OSFP_P3V3_VOLT_V) {
			if (vr_type == VR_MPS_MP2971_MP2891) {
				LOG_INF("change vr init args for MPS_MP2971_MP2891");
				*init_args = plat_sensor_vr_extend_table[index].mps_vr_init_args;
			} else if (vr_type == VR_RNS_ISL69260_RAA228238) {
				LOG_INF("change vr init args for RNS_ISL69260_RAA228238");
				*init_args = plat_sensor_vr_extend_table[index].rns_vr_init_args;
			} else {
				*init_args = NULL;
			}
		}
	}
}

void plat_pldm_sensor_change_vr_init_args()
{
	void *init_args;

	for (int index = 0; index < plat_pldm_sensor_get_sensor_count(VR_SENSOR_THREAD_ID);
	     index++) {
		if (plat_pldm_sensor_vr_table[index].pdr_numeric_sensor.sensor_id ==
		    SENSOR_NUM_OSFP_P3V3_VOLT_V) {
			find_init_args_by_sensor_id(
				plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.num, &init_args);
			plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.init_args = init_args;
		}
	}
}

void plat_pldm_sensor_change_vr_addr()
{
	uint8_t vr_type = get_vr_type();
	if (vr_type == VR_UNKNOWN) {
		LOG_ERR("Unable to change the VR device due to its unknown status.");
		return;
	}

	uint8_t addr;

	if (vr_type == VR_RNS_ISL69260_RAA228238) {
		for (int index = 0; index < plat_pldm_sensor_get_sensor_count(VR_SENSOR_THREAD_ID);
		     index++) {
			find_vr_addr_by_sensor_id(
				plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.num, &addr);
			plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.target_addr = addr;
		}
	} else if (vr_type != VR_MPS_MP2971_MP2891) {
		LOG_ERR("Unable to change the VR device due to its unknown status.");
	}
}

void plat_pldm_sensor_change_vr_dev()
{
	uint8_t vr_type = get_vr_type();
	if (vr_type == VR_UNKNOWN) {
		LOG_ERR("Unable to change the VR device due to its unknown status.");
		return;
	}

	if (vr_type == VR_RNS_ISL69260_RAA228238) {
		for (int index = 0; index < plat_pldm_sensor_get_sensor_count(VR_SENSOR_THREAD_ID);
		     index++) {
			if (plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.type ==
			    sensor_dev_mp2971)
				plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.type =
					sensor_dev_isl69259;
			else if (plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.type ==
				 sensor_dev_mp2891)
				plat_pldm_sensor_vr_table[index].pldm_sensor_cfg.type =
					sensor_dev_raa228238;
		}
	} else if (vr_type != VR_MPS_MP2971_MP2891) {
		LOG_ERR("Unable to change the VR device due to its unknown status.");
	}
}

void plat_pldm_sensor_change_ubc_dev()
{
	uint8_t ubc_type = get_ubc_type();
	if (ubc_type == UBC_UNKNOWN) {
		LOG_ERR("Unable to change the VR device due to its unknown status.");
		return;
	}

	if (ubc_type == UBC_FLEX_BMR313) {
		for (int index = 0; index < plat_pldm_sensor_get_sensor_count(UBC_SENSOR_THREAD_ID);
		     index++) {
			plat_pldm_sensor_ubc_table[index].pldm_sensor_cfg.type = sensor_dev_bmr313;
		}
	} else if (ubc_type == UBC_MPS_MPC12109) {
		for (int index = 0; index < plat_pldm_sensor_get_sensor_count(UBC_SENSOR_THREAD_ID);
		     index++) {
			plat_pldm_sensor_ubc_table[index].pldm_sensor_cfg.type =
				sensor_dev_mpc12109;
		}
		LOG_INF("UBC_MPS_MPC12109 driver loaded");
	} else if (ubc_type != UBC_DELTA_U50SU4P180PMDAFC) {
		LOG_ERR("Unable to change the UBC device due to its unknown status.");
	}
}

bool is_dc_access(uint8_t sensor_num)
{
	return is_mb_dc_on();
}