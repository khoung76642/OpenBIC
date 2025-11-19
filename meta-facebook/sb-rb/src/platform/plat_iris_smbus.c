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

#include "libutil.h"
#include "plat_util.h"
#include "plat_i2c.h"
#include "plat_iris_smbus.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_iris_smbus);

uint8_t slave_id = MEDHA0_I2C_ADDR;
const struct device *i2c_dev;
static char *prt_mode(uint8_t mode)
{
	char *str = NULL;

	switch (mode) {
	case FASTBOOT_MODE:
		str = "fastboot";
		break;
	case CMRT_SIC_MODE:
		str = "cmrt_sic";
		break;
	case RECOVERY_MODE:
		str = "recovery";
		break;
	default:
		str = "unknown";
		break;
	}
	return str;
}

int smbus_read_byte(uint8_t cmd, uint8_t *data)
{
	uint8_t rd;
	int ret;
	ret = i2c_write_read(i2c_dev, slave_id, &cmd, 1, &rd, 1);
	*data = rd;

	return ret;
}

int sb_write_byte(uint8_t cmd, uint8_t data)
{
	return i2c_reg_write_byte(i2c_dev, slave_id, cmd, data);
}

uint32_t smbus_mode_query(void)
{
	uint8_t rd = 0;
	uint8_t cmd = SB_MODE_QUERY;
	int err = 0xffff;
	err = i2c_write_read(i2c_dev, slave_id, &cmd, 1, &rd, 1);

	LOG_INF("slave:0x%x in mode:%d, %s", slave_id, rd, prt_mode(rd));

	return err == 0 ? rd & 0x7 : err;
}
int smbus_img_send()
{
	int err = 0, idx, tries;
	uint32_t write_size, write_addr, size;
	uint8_t *data_buf, rd;
	write_size = MAX_DATA_PKT_SIZE;
	err = sb_write_byte(FW_CTRL_WRITE, FW_DL_START);
	if (err < 0) {
		printk("Firmware download start request failed\n");
		return err;
	}
	err = sb_write_fwblock(dst_addr, &size, 4);
	if (err < 0) {
		printk("sending fw addr-size failed\n");
		goto err_i2c;
    }
}
int iris_smbus_fast_boot(uint32_t img_src_addr, uint32_t img_dest_addr, uint32_t img_size)
{
	int idx;
	uint8_t rd;
	// pre do: check is download mode
	int err = smbus_mode_query();
	if (!err) {
		LOG_ERR("slave not in download mode");
		goto err_i2c;
	}
	// udpate: start transfer image to IRIS 
	LOG_INF("Starting Firmware Download");
    err = smbus_img_send();
    if (err) {
       	LOG_ERR("image send failed\n");
        goto err_i2c;
    }

	// finished: set the bit to inform device that download is complete
	err = sb_write_byte(FW_CTRL_WRITE, FW_DL_FINISH);
	if (err < 0) {
		// retain original err code returned, i.e. set by ioctl
		LOG_ERR("Unable to notify download completion state, aborting");
		goto err_i2c;
	}
	printk("Sent firmware activate command\n");

	idx = 0;

	/* Wait for the slave to complete firmware update */
	while (1) {
		rd = 0;
		err = sb_read_byte(FW_CTRL_READ, &rd);
		if (err < 0) {
			LOG_ERR("fw upgrade state read failed, aborting");
			goto err_i2c;
		}

		if (rd & FW_DL_SLV_DONE) {
			LOG_ERR("Read status: FW update complete");
			break;
		}

		if (idx++ > STATUS_RETRY_CNT) {
			LOG_ERR("Invalid completion status(0x%x) read", rd);
			err = -1; // no errno, set card fw error
			goto err_i2c;
		}
	}
	printk("##### Firmware Update Completed #####\n");
	return 0;

err_i2c:
	// send download abort command to device
	LOG_INF("partial download, abort now");
	err = sb_write_byte(FW_CTRL_WRITE, FW_DL_HST_ABRT);
	if (err < 0) {
		LOG_ERR("cmd write failed, %d", err);
	}

	LOG_INF("check abort status...");
	// read slave status and check of device aborted download
	err = sb_read_byte(FW_CTRL_READ, &rd);
	if (err < 0) {
		LOG_ERR("cmd status read failed, %d", err);
	}
	if (rd & FW_DL_SLV_ABRTD) {
		LOG_WRN("SMBus device 0x%x aborted download",
				slave_id);
	} else {
		LOG_ERR("SMBus device 0x%x failed to abort(0x%x)",
				slave_id, rd);
	}
	// check what smbus error occurred
	uint8_t rd;
	err = smbus_read_byte(FW_SMBUS_ERROR, &rd);
	if (err < 0)
		LOG_ERR("read error status failed, %d", err);
	else
		LOG_ERR("smbus error status = 0x%x\n", rd);
	return err;
}
