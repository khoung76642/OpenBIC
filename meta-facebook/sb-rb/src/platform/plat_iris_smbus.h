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

#ifndef PLAT_IRIS_SMBUS_H
#define PLAT_IRIS_SMBUS_H

/*
 * smbus sideband operating modes
 */
#define FASTBOOT_MODE BIT(0)
#define CMRT_SIC_MODE BIT(1)
#define RECOVERY_MODE BIT(2)

/* fw download status/control bytes */
#define FW_DL_START BIT(7)
#define FW_DL_SLV_RDY BIT(6)
#define FW_DL_HST_ABRT BIT(5)
#define FW_DL_SLV_ABRTD BIT(4)
#define FW_DL_FINISH BIT(3)
#define FW_DL_SLV_DONE BIT(2)
#define FW_DL_SLV_PROG BIT(1)
#define FW_SB_EXIT_CMD BIT(0)

#define MEDHA0_I2C_ADDR 0x33
#define MEDHA1_I2C_ADDR 0x34
#define OWLW_I2C_ADDR 0x6E
#define OWLE_I2C_ADDR 0x6E

#define MAX_DATA_PKT_SIZE 24
#define STATUS_RETRY_CNT 2

/* smbus/firmware protocol error */
#define FW_SMBUS_ERROR 130
/* custom device error report */
#define SB_MODE_QUERY 135
/* fw download control write */
#define FW_CTRL_WRITE 136
/* fw download control read */
#define FW_CTRL_READ 137


#endif