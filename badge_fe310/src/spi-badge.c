/*
 * Copyright (c) 2017 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <string.h>
#include <misc/printk.h>
#include <device.h>
#include <spi.h>
#include "spi-badge.h"

int spi_send_buffer(struct spi_config *spi_conf,
		    unsigned char *data, size_t len)
{
	struct spi_buf tx_buf = {
		.buf = data,
		.len = len,
	};

	int err;

	err = spi_write(spi_conf, &tx_buf, 1);
	if (err != 0) {
		printk("spi_write failed\n");
	}
	return err;
}

int spi_transceive_buffer(struct spi_config *spi_conf,
			unsigned char *tx_data, size_t tx_len,
			unsigned char *rx_data, size_t rx_len)
{
	struct spi_buf tx_buf = {
		.buf = tx_data,
		.len = tx_len,
	};

	struct spi_buf rx_buf = {
		.buf = rx_data,
		.len = rx_len,
	};

	int err;

	err = spi_transceive(spi_conf, &tx_buf, 1, &rx_buf, 1);
	if (err != 0) {
		printk("spi_transceive failed\n");
	}

	return err;
}

void spi_setup_for_epd(struct spi_config *spi_conf)
{
	spi_conf->frequency = 8000000UL;
	spi_conf->operation = (8 << 5); /* 8bit data */
	spi_conf->slave = LCD_CS;
}
