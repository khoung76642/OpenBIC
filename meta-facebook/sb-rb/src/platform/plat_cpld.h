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

#ifndef PLAT_CPLD_H
#define PLAT_CPLD_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <zephyr.h>

#define RESET 0x00
#define CPLD_OFFSET_BOARD_REV_ID 0x14
#define CPLD_OFFSET_VR_VENDER_TYPE 0x15
#define CPLD_OFFSET_POWER_CLAMP 0x25
#define CPLD_OFFSET_USERCODE 0x32
#define CPLD_OFFSET_MMC_PWR_EN 0x38
#define CPLD_OFFSET_ASIC_BOARD_ID 0x3C
#define VR_AND_CLK_EN 0x3E
#define VR_1_EN 0x3F
#define VR_2_EN 0x40
#define VR_3_EN 0x41
#define VR_4_EN 0x42
#define PREST_DELAY_REG 0x9D

#define CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS11
typedef struct _cpld_info_ cpld_info;

typedef struct _cpld_info_ {
	uint8_t cpld_offset;
	uint8_t dc_off_defaut;
	uint8_t dc_on_defaut;
	bool is_fault_log; // if true, check the value is defaut or not
	uint8_t is_fault_bit_map; //flag for fault

	//flag for 1st polling
	bool is_first_polling;

	//flag for 1st polling after changing DC status
	bool is_first_polling_after_dc_change;

	//temp data for last polling
	uint8_t last_polling_value;

	bool (*status_changed_cb)(cpld_info *, uint8_t *);

	uint8_t bit_check_mask; //bit check mask

	uint8_t event_type;

} cpld_info;

void check_ubc_delayed(struct k_work *work);
bool is_ubc_enabled_delayed_enabled(void);
bool plat_read_cpld(uint8_t offset, uint8_t *data, uint8_t len);
bool plat_write_cpld(uint8_t offset, uint8_t *data);
void init_cpld_polling(void);
void check_cpld_polling_alert_status(void);
void check_ubc_delayed_timer_handler(struct k_timer *timer);
bool set_cpld_bit(uint8_t cpld_offset, uint8_t bit, uint8_t value);

#endif