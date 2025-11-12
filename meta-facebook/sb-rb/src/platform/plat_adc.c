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

#include <drivers/adc.h>
#include <drivers/spi.h>
#include "libutil.h"
#include <logging/log.h>
#include "plat_gpio.h"
#include "plat_adc.h"
#include "plat_class.h"
#include "plat_cpld.h"

LOG_MODULE_REGISTER(plat_adc);

#define BUFFER_SIZE 2
#define ADC_STACK_SIZE 1024

#define MEDHA0_ADC_CHANNEL 8
#define MEDHA1_ADC_CHANNEL 6

#define ADI_AD4058 0x0
#define TIC_ADS7066 0x1

#define ADS7066_SPI_FREQ 6000000

K_THREAD_STACK_DEFINE(adc_thread_stack, ADC_STACK_SIZE);
struct k_thread adc_poll_thread;

K_THREAD_STACK_DEFINE(adc_rainbow_thread_stack, ADC_STACK_SIZE);
struct k_thread adc_rainbow_poll_thread;

static bool adc_poll_flag = true;
uint8_t adc_idx = 0;
float ad4058_val_0 = 0;
float ad4058_val_1 = 0;
float ads7066_val_0 = 0;
float ads7066_val_1 = 0;

typedef struct {
	uint16_t avg_times; // 20ms at a time
	uint16_t buf[ADC_AVERGE_TIMES_MAX];
	uint8_t buf_idx;
	uint16_t avg_val;
	uint32_t sum;
	uint16_t ucr; // amps
	bool ucr_status; // over current
	struct k_work ucr_work;
} adc_info_t;

adc_info_t adc_info[ADC_IDX_MAX] = { { .avg_times = 1, .ucr = 980 },
				     { .avg_times = 3, .ucr = 895 },
				     { .avg_times = 30, .ucr = 538 },
				     { .avg_times = 40, .ucr = 497 } };

static const struct device *spi_dev;
static struct spi_config spi_cfg = {
	.frequency = ADS7066_SPI_FREQ,
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
	.slave = 0,
	.cs = NULL,
};

static void adc_poll_init()
{
	for (uint8_t i = ADC_IDX_MEDHA0_1; i < ADC_IDX_MAX; i++) {
		adc_info[i].sum = 0;
		adc_info[i].buf_idx = 0;
		adc_info[i].avg_val = 0;
		memset(adc_info[i].buf, 0, sizeof(adc_info[i].buf));
	}
}

uint16_t get_adc_averge_val(uint8_t idx)
{
	return adc_info[idx].avg_val;
}

uint16_t *get_adc_buf(uint8_t idx)
{
	return adc_info[idx].buf;
}

uint16_t get_adc_averge_times(uint8_t idx)
{
	return adc_info[idx].avg_times;
}
void adc_set_averge_times(uint8_t idx, uint16_t time)
{
	if (time >= ADC_AVERGE_TIMES_MIN && time <= ADC_AVERGE_TIMES_MAX) {
		adc_info[idx].avg_times = time;
		adc_poll_init();
	} else {
		LOG_WRN("invalid adc %d poll times: %d\n", idx, time);
	}
}

uint16_t get_adc_ucr(uint8_t idx)
{
	return adc_info[idx].ucr;
}
void set_adc_ucr(uint8_t idx, uint16_t ucr)
{
	adc_info[idx].ucr = ucr;
}

bool get_adc_ucr_status(uint8_t idx)
{
	return adc_info[idx].ucr_status;
}
void set_adc_ucr_status(uint8_t idx, bool status)
{
	adc_info[idx].ucr_status = status;
}

float adc_raw_mv_to_apms(uint16_t mv)
{
	return (get_vr_module() == VR_MODULE_MPS) ? 2 * mv * 0.796 : 2 * mv * 0.797;
}

static bool adc_ucr_handler(uint8_t idx, bool state) // state is ucr or not
{
	uint8_t data = 0;
	if (!plat_read_cpld(CPLD_OFFSET_POWER_CLAMP, &data, 1))
		return false;

	uint8_t bit = (idx == ADC_IDX_MEDHA0_1) ? 7 :
		      (idx == ADC_IDX_MEDHA1_1) ? 6 :
		      (idx == ADC_IDX_MEDHA0_2) ? 5 :
		      (idx == ADC_IDX_MEDHA1_2) ? 4 :
						  0xFF;
	if (bit == 0xFF)
		return false;

	// if ucr, pull up
	if (state)
		data |= (1 << bit);
	else
		data &= ~(1 << bit);

	if (!plat_write_cpld(CPLD_OFFSET_POWER_CLAMP, &data))
		return false;

	return true;
}
static void adc_ucr_work_handler(struct k_work *work)
{
	const adc_info_t *adc = CONTAINER_OF(work, adc_info_t, ucr_work);

	uint8_t idx = adc - adc_info;
	adc_ucr_handler(idx, adc->ucr_status);
}
K_WORK_DEFINE(adc_ucr_work, adc_ucr_work_handler);

static void plat_adc_read(void)
{
	uint16_t m_sample_buffer[BUFFER_SIZE];
	const struct device *adc_dev;
	int retval;

	adc_dev = device_get_binding("ADC_0");
	if (adc_dev == NULL) {
		LOG_INF("ADC device not found\n");
		return;
	}

	const struct adc_sequence sequence = {
		.channels = (BIT(MEDHA0_ADC_CHANNEL) | BIT(MEDHA1_ADC_CHANNEL)),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = 10,
		.calibrate = 1,
	};

	retval = adc_read(adc_dev, &sequence);

	for (uint8_t i = ADC_IDX_MEDHA0_1; i < ADC_IDX_MAX; i++) {
		adc_info_t *adc = &adc_info[i];
		adc->sum -= adc->buf[adc->buf_idx];
		adc->buf[adc->buf_idx] = ((i == ADC_IDX_MEDHA0_1) || (i == ADC_IDX_MEDHA0_2)) ?
						 m_sample_buffer[1] :
						 m_sample_buffer[0];
		adc->sum += adc->buf[adc->buf_idx];
		adc->avg_val = adc->sum / adc->avg_times;
		adc->buf_idx = (adc->buf_idx + 1) % adc->avg_times;

		// check status
		bool curr_status = (adc_raw_mv_to_apms(adc->avg_val) >= adc->ucr);
		if (adc->ucr_status != curr_status) {
			adc->ucr_status = curr_status;
			k_work_submit(&adc->ucr_work);
		}
	}
}

bool adc_get_poll_flag()
{
	return adc_poll_flag;
}
void adc_set_poll_flag(uint8_t onoff)
{
	adc_poll_flag = onoff ? true : false;
	if (!adc_poll_flag)
		adc_poll_init();
}
void spi_enable_medha0()
{
	LOG_DBG("set SPI_MUX_MEDHA0 to MMC");
	gpio_set(SPI_MEDHA0_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 1);
	gpio_set(QSPI_CPLD_SEL_1, 0);
}

void spi_enable_medha1()
{
	LOG_DBG("set SPI_MUX_MEDHA1 to MMC");
	gpio_set(SPI_MEDHA1_MUX_IN1, 1);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 1);
}

void spi_disable()
{
	LOG_DBG("set All SPI_MUX to ASIC");
	gpio_set(SPI_HAMSA_MUX_IN1, 0);
	gpio_set(SPI_MEDHA0_MUX_IN1, 0);
	gpio_set(SPI_MEDHA1_MUX_IN1, 0);
	gpio_set(QSPI_CPLD_SEL_0, 0);
	gpio_set(QSPI_CPLD_SEL_1, 0);
}

void get_adc_info()
{
	adc_idx = plat_read_cpld(CPLD_OFFSET_ADC_IDX, &adc_idx, 1);
}

static int ads7066_write_reg(uint8_t reg, uint8_t write_val)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t tx_buf[3] = { 0x08, reg, write_val };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}

	return 0;
}

static void ads7066_read_voltage(uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
		return;
	}

	if (idx == ADC_RB_IDX_MEDHA0) {
		spi_enable_medha0();
	} else if (idx == ADC_RB_IDX_MEDHA1) {
		spi_enable_medha1();
	}
	else {
		LOG_ERR("idx error");
		return;
	}

	uint8_t tx_buf[3] = { 0x00, 0x00, 0x00};
	uint8_t rx_buf[3] = { 0 };
	uint8_t out_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	memset(rx_buf, 0, 3);
	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI failed: %d", ret);
		return;
	}

	memcpy(out_buf, rx_buf, 3);

	k_msleep(20);

	LOG_HEXDUMP_DBG(out_buf, 3, "ads7066_read_voltage_value");
	// get out_buf[0] and out_buf[1] to culculate the voltage
	/*
	example: b6 e9 00
	(0xb6e9 / 0xffff) * 3.3 = 1.78625925V  
	i wan't to get Three decimal places
	*/
	float vref = 3.3;
	if (idx == ADC_RB_IDX_MEDHA0) {
		ads7066_val_0 = ((out_buf[0] << 8 | out_buf[1]) / 0xffff) * vref;
	} else if (idx == ADC_RB_IDX_MEDHA1) {
		ads7066_val_1 = ((out_buf[0] << 8 | out_buf[1]) / 0xffff) * vref;
	}

	return;
}

static int ad4058_write_reg(uint8_t reg, uint8_t write_val)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t tx_buf[2] = { reg, write_val };
	uint8_t rx_buf[2] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = 2 };
	struct spi_buf rx = { .buf = rx_buf, .len = 2 };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}

	LOG_HEXDUMP_DBG(tx_buf, 2, "ad4058_write_reg");
	return 0;
}

static void ad4058_read_voltage(uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t cnv_pin = 0;

	if (idx == ADC_RB_IDX_MEDHA0) {
		spi_enable_medha0();
		cnv_pin = MEDHA0_CNV;
	} else if (idx == ADC_RB_IDX_MEDHA1) {
		spi_enable_medha1();
		cnv_pin = MEDHA1_CNV;
	}
	else {
		LOG_ERR("idx error");
		return;
	}

	uint8_t tx_buf[2] = { 0 };
	uint8_t rx_buf[3] = { 0 };
	uint8_t out_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = 2 };
	struct spi_buf rx = { .buf = rx_buf, .len = 3 };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };
	// set cnv_pin to low
	gpio_set(cnv_pin, 0);
	k_msleep(20);

	memset(rx_buf, 0, 3);

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI failed: %d", ret);
		return;
	}

	memcpy(out_buf, rx_buf, 3);
	/*
	(Read_back 20 bits data / 0xFFFFF ) * Vref 
	example: 0b 62 83
	0x0b6283 / 0xfffff = (0x0b6283 / 0xfffff) * 3.3
	*/
	float vref = 3.3;
	if (idx == ADC_RB_IDX_MEDHA0) {
		ad4058_val_0 = (float)(out_buf[0] << 16 | out_buf[1] << 8 | out_buf[2]) / 0xfffff * vref;
	} else if (idx == ADC_RB_IDX_MEDHA1) {
		ad4058_val_1 = (float)(out_buf[0] << 16 | out_buf[1] << 8 | out_buf[2]) / 0xfffff * vref;
	}
	// set cnv_pin to high
	gpio_set(cnv_pin, 1);

	return;
}

void get_ads7066_voltage()
{
	LOG_INF(" ads7066 medha0 voltage is %f V", ads7066_val_0);
	LOG_INF(" ads7066 medha1 voltage is %f V", ads7066_val_1);
}

void get_ad4058_voltage()
{
	LOG_INF(" ad4058 medha0 voltage is %f V", ad4058_val_0);
	LOG_INF(" ad4058 medha1 voltage is %f V", ad4058_val_1);
}

void ads7066_mode_init()
{

	//set auto-sequence mode
	// medha0
	spi_enable_medha0();
	ads7066_write_reg(0,0x1);
	ads7066_write_reg(0x1,0x2);
	ads7066_write_reg(0x12,0x1);
	ads7066_write_reg(0x3,0x6);
	ads7066_write_reg(0x4,0x8);
	ads7066_write_reg(0x10,0x11);
	ads7066_write_reg(0x2,0x10);
	// medha1
	spi_enable_medha1();
	ads7066_write_reg(0,0x1);
	ads7066_write_reg(0x1,0x2);
	ads7066_write_reg(0x12,0x1);
	ads7066_write_reg(0x3,0x6);
	ads7066_write_reg(0x4,0x8);
	ads7066_write_reg(0x10,0x11);
	ads7066_write_reg(0x2,0x10);
	spi_disable();
}

void ad4058_mode_init()
{
	// set ad4058 to burst averaging mode 
	spi_enable_medha0();
	ad4058_write_reg(0x27, 0x20);
	ad4058_write_reg(0x23, 0x8);
	ad4058_write_reg(0x21, 0x1);
	ad4058_write_reg(0x20, 0x1);
	spi_enable_medha1();
	ad4058_write_reg(0x27, 0x20);
	ad4058_write_reg(0x23, 0x8);
	ad4058_write_reg(0x21, 0x1);
	ad4058_write_reg(0x20, 0x1);
	spi_disable();
}

void adc_polling_handler(void *p1, void *p2, void *p3)
{
	while (1) {
		if (adc_poll_flag)
			plat_adc_read();
		k_msleep(1);
	}
}

void adc_rainbow_polling_handler(void *p1, void *p2, void *p3)
{
	get_adc_info();
	if (adc_idx == ADI_AD4058)
		ad4058_mode_init();
	else if (adc_idx == TIC_ADS7066)
		ads7066_mode_init();
	else
		LOG_ERR("Invalid ADC index %d", adc_idx);

	while (1) {
		if (adc_poll_flag)
		{
			switch (adc_idx)
			{
			case ADI_AD4058:
				ad4058_read_voltage(ADC_RB_IDX_MEDHA0);
				ad4058_read_voltage(ADC_RB_IDX_MEDHA1);
				spi_disable();
				break;
			case TIC_ADS7066:
				ads7066_read_voltage(ADC_RB_IDX_MEDHA0);
				ads7066_read_voltage(ADC_RB_IDX_MEDHA1);
				spi_disable();
				break;
			default:
				LOG_ERR("Invalid ADC index %d", adc_idx);
				break;
			}
		}
			
		k_msleep(1);
	}
}

void plat_adc_init(void)
{
	for (uint8_t i = 0; i < ADC_IDX_MAX; i++) {
		k_work_init(&adc_info[i].ucr_work, adc_ucr_work_handler);
	}

	k_thread_create(&adc_poll_thread, adc_thread_stack, ADC_STACK_SIZE, adc_polling_handler,
			NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&adc_poll_thread, "platform adc read");

	LOG_INF("ADC polling thread started...\n");
}

void plat_adc_rainbow_init(void)
{
	k_thread_create(&adc_rainbow_poll_thread, adc_rainbow_thread_stack, ADC_STACK_SIZE, adc_rainbow_polling_handler,
			NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&adc_rainbow_poll_thread, "platform adc(rainbow) read");

	LOG_INF("ADC(rainbow) polling thread started...\n");
}