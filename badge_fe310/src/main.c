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
#include <i2c.h>
#include "gpio_fe310.h"
#include "spi-badge.h"
#include "epd.h"
#include "eeprom.h"
#include "risc-v-logo.h"

#define POWER_SEQ_GPIO 0x13 /* gpio 19 */

extern void __soc_enter_deep_sleep(void);

/* We use global buffers so they are allocated in the data section */
unsigned char uncompressed_image[2400];
unsigned char eeprom_img_buffer[2400];

void power_seq_on(void)
{
	GPIO_REG(GPIO_OUTPUT_EN) |= (0x1 << POWER_SEQ_GPIO);
	GPIO_REG(GPIO_OUTPUT_VAL) |= (0x1 << POWER_SEQ_GPIO);
}

void power_seq_off(void)
{
	GPIO_REG(GPIO_OUTPUT_EN) |= (0x1 << POWER_SEQ_GPIO);
	GPIO_REG(GPIO_OUTPUT_VAL) &= ~(0x1 << POWER_SEQ_GPIO);
}


void main(void)
{
	struct EPD_configuration epd;
	struct device *spi_dev;
	struct device *i2c_dev;
	struct spi_config spi_conf;

	unsigned short eeprom_img_size;
	int ret;

	unsigned char *draw_buffer;

	power_seq_on();
	k_sleep(3000);

	printk("\nDate of compilation: %s\n", __DATE__);
	printk("Last commit hash: %s\n\n", COMMIT);

	spi_dev = device_get_binding(CONFIG_SPI_0_NAME);

	if (!spi_dev) {
		printk("Failed to bind SPI driver\n");
	}

	spi_conf.dev = spi_dev,
	spi_conf.frequency = 8000000UL,
	spi_conf.operation = (8 << 5),
	spi_conf.slave = LCD_CS,
	spi_conf.cs = NULL,

	i2c_dev = device_get_binding(CONFIG_I2C_0_NAME);
	if (!i2c_dev) {
		printk("Failed to bind I2C driver\n");
	}

	/* generic*/
	epd.size = EPD_2_0;
	epd.middle_scan = true;

	/* display specific */
	epd.lines_per_display = 96;
	epd.dots_per_line = 200;
	epd.bytes_per_line = 200 / 8;
	epd.bytes_per_scan = 96 / 4;
	epd.pre_border_byte = true;
	epd.border_byte = EPD_BORDER_BYTE_NONE;

	epd.stage_repetitions = 10;

	/* because our display has middle_scan set */
	epd.line_buffer_size = 2 * epd.bytes_per_line + epd.bytes_per_scan + 3;

	{
		/* line_buffer_size + 4096*/
		static unsigned char _lb[400 + 96/4 + 3 + 4096];

		epd.line_buffer =  (unsigned char *)&_lb;
	}

	/* clear line buffer */
	memset(epd.line_buffer, 0x00, epd.line_buffer_size);

	epd_gpio_init();
	printk("(Re)initializing devices\n");
	spi_setup_for_epd(&spi_conf);
	epd_init(&spi_conf);

	epd_clear(&epd);

	printk("Reading eeprom\n");
	ret = read_image_from_eeprom(i2c_dev, eeprom_img_buffer);
	eeprom_img_size = (unsigned short)ret;

	if(ret > 0) {
		printk("Uncompressing image from eeprom\n");
		uncompress_eeprom_image(uncompressed_image,
					eeprom_img_buffer,
					eeprom_img_size);
		draw_buffer = uncompressed_image;
	} else {
		/* if there is no image in the EEPROM
		 * use the default one */
		printk("Using default image \n");
		draw_buffer = riscv_logo_bits;
	}

	printk("Drawing\n");
	/* Drawing image */
	spi_setup_for_epd(&spi_conf);
	epd_image_0(&epd, draw_buffer);

	printk("De-initializing LCD\n");
	/* Power off LCD */
	epd_poweroff();

	/* We cannot leave CS for LCD, so setup for i2c */
	i2c_configure(i2c_dev, 0);

	/* Discharge the pumps */
	epd_discharge();

	/* Force power off through the power sequencer */
	printk("Powering off\n");
	k_sleep(10);
	power_seq_off();

	/* When battery powered we will not get here.
	 * On USB we will print dots to mark that the
	 * device is working. */

	while (1) {
		printk(".");
		k_sleep(1000);
	}

}
