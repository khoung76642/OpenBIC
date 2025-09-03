#include <zephyr.h>
#include <device.h>
#include <stdlib.h>
#include <drivers/spi.h>
#include <shell/shell.h>
#include <logging/log.h>
#include "hal_gpio.h"
#include "plat_gpio.h"

LOG_MODULE_REGISTER(ad4058, LOG_LEVEL_INF);

#define AD4058_SPI_FREQ 6000000

static const struct device *spi_dev;
static struct spi_config spi_cfg = {
	.frequency = AD4058_SPI_FREQ,
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
	.slave = 0,
	.cs = NULL,
};

static int ad4058_read_reg(const struct shell *shell, uint8_t cmd, uint8_t rlen)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}
	rlen = (rlen == 2) ? 3 : 2;

	uint8_t tx_buf[3] = { 0x80, 0x00, 0x00};
	uint8_t rx_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = rlen };
	struct spi_buf rx = { .buf = rx_buf, .len = rlen };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	tx_buf[0] += cmd;

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}

	shell_hexdump(shell, rx_buf, rlen);
	return 0;
}

static int ad4058_write_reg(const struct shell *shell, uint8_t cmd, uint8_t input)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t tx_buf[2] = { cmd, input};
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

	shell_hexdump(shell, rx_buf, 2);
	return 0;
}

static int ad4058_read_m(const struct shell *shell, uint8_t rlen, uint8_t delay)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	if((rlen > 5) || (rlen < 2)){
		shell_error(shell, "rlen incorrect");
		return 1;
	}

	uint8_t tx_buf[5] = { 0 };
	uint8_t rx_buf[5] = { 0 };
	uint8_t out_buf[40] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = rlen };
	struct spi_buf rx = { .buf = rx_buf, .len = rlen };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };


	for (int i=0; i<8; i++) {
		//gpio_set(SPI_MEDHA0_MUX_IN1, 0);
		gpio_set(GPIO_TEST_AD4058, 0);
		if (delay != 0){
			k_msleep(delay);
		}

		memset(rx_buf, 0, 5);

		int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
		if (ret < 0) {
			LOG_ERR("SPI failed: %d", ret);
			return ret;
		}
		memcpy(&out_buf[i*5], rx_buf, 5);
		//gpio_set(SPI_MEDHA0_MUX_IN1, 1);
		gpio_set(GPIO_TEST_AD4058, 1);
	}
	shell_hexdump(shell, out_buf, 40);

	return 0;
}

static int cmd_ad4058_id(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t op = strtoul(argv[1], NULL, 16);
	uint8_t cmd = strtoul(argv[2], NULL, 16);
	uint8_t input = strtoul(argv[3], NULL, 16);
	if (op == 0) {
		int ret = ad4058_read_reg(shell, cmd, input);
		if (ret < 0) {
			shell_error(shell, "read reg fail (err=%d)", ret);
			return ret;
		}
	} else if (op == 1) {
		int ret = ad4058_write_reg(shell, cmd, input);
		if (ret < 0) {
			shell_error(shell, "write fail (err=%d)", ret);
			return ret;
		}
	} else if (op == 2) {
		int ret = ad4058_read_m(shell, cmd, input);
		if (ret < 0) {
			shell_error(shell, "read m (err=%d)", ret);
			return ret;
		}
	}

	return 0;
}

SHELL_CMD_REGISTER(ad4058_id, NULL, "read AD4058 Device ID", cmd_ad4058_id);
