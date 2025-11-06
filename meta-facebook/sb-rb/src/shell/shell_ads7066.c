#include <zephyr.h>
#include <device.h>
#include <stdlib.h>
#include <drivers/spi.h>
#include <shell/shell.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(ads7066, LOG_LEVEL_INF);

#define ADS7066_SPI_FREQ 6000000

static const struct device *spi_dev;
static struct spi_config spi_cfg = {
	.frequency = ADS7066_SPI_FREQ,
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
	.slave = 0,
	.cs = NULL,
};

static int ads7066_read_reg(const struct shell *shell, uint8_t *data, uint8_t cmd)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t tx_buf[3] = { 0x10, cmd, 0x00 }; // bit15=1: read
	uint8_t rx_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}
	ret = spi_read(spi_dev, &spi_cfg, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI read failed: %d", ret);
		return ret;
	}

	memcpy(data, rx_buf, 3);
	shell_hexdump(shell, data, 3);
	return 0;
}

static int ads7066_write_reg(uint8_t *data, uint8_t cmd, uint8_t input)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t tx_buf[3] = { 0x08, cmd, input };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}

	return 0;
}

static int ads7066_read_return(const struct shell *shell, uint8_t *data, uint8_t cmd)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t tx_buf[3] = { 0x00, 0x00 };
	uint8_t rx_buf[3] = { 0 };
	uint8_t out_buf[48] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	for (int i=0; i<16; i++) {
		memset(rx_buf, 0, 3);
		if (tx_buf[2] == 0) {
			tx_buf[2] = cmd;
		} else {
			tx_buf[2] = 0x00;
		}
		int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
		if (ret < 0) {
			LOG_ERR("SPI failed: %d", ret);
			return ret;
		}
		memcpy(&out_buf[i*3], rx_buf, 3);
		//shell_hexdump(shell, rx_buf, 3);
		k_msleep(20);

	}
	shell_hexdump(shell, out_buf, 48);
	return 0;
}

static int ads7066_read_m(const struct shell *shell, uint8_t cmd, uint8_t delay)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t tx_buf[3] = { 0x08, 0x11, 0x00 };
	uint8_t rx_buf[3] = { 0 };
	uint8_t out_buf[48] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	for (int i=0; i<16; i++) {
		memset(rx_buf, 0, 3);
		if (tx_buf[2] == 0) {
			tx_buf[2] = cmd;
		} else {
			tx_buf[2] = 0x00;
		}
		int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
		if (ret < 0) {
			LOG_ERR("SPI failed: %d", ret);
			return ret;
		}
		memcpy(&out_buf[i*3], rx_buf, 3);
		//shell_hexdump(shell, rx_buf, 3);
		if (delay != 0){
			k_msleep(delay);
		}
	}
	shell_hexdump(shell, out_buf, 48);

	return 0;
}

static int cmd_ads7066_id(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t data[3] = { 0 };
	uint8_t op = strtoul(argv[1], NULL, 16);
	uint8_t cmd = strtoul(argv[2], NULL, 16);
	uint8_t input = strtoul(argv[3], NULL, 16);
	if (op == 0) {
		int ret = ads7066_read_reg(shell, data, cmd);
		if (ret < 0) {
			shell_error(shell, "read reg fail (err=%d)", ret);
			return ret;
		}
	} else if (op == 1) {
		int ret = ads7066_write_reg(data, cmd, input);
		if (ret < 0) {
			shell_error(shell, "write fail (err=%d)", ret);
			return ret;
		}
	} else if (op == 2) {
		int ret = ads7066_read_m(shell, cmd, input);
		if (ret < 0) {
			shell_error(shell, "read m (err=%d)", ret);
			return ret;
		}
	}
	  else if (op == 3) {
		int ret = ads7066_read_return(shell, data, cmd);
		if (ret < 0) {
			shell_error(shell, "read return (err=%d)", ret);
			return ret;
		}
	}

	return 0;
}

SHELL_CMD_REGISTER(ads7066_id, NULL, "read ADS7066 Device ID", cmd_ads7066_id);
