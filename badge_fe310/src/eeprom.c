/*
 * Copyright (c) 2017 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <string.h>
#include <misc/printk.h>
#include <device.h>
#include <i2c.h>

#include "eeprom.h"

static inline int eeprom_burst_read(struct device *dev, u16_t dev_addr,
			unsigned char *start_addr, unsigned char *buf,
			unsigned char num_bytes)
{
	const struct i2c_driver_api *api = dev->driver_api;
	struct i2c_msg msg[2];

	msg[0].buf = start_addr;
	msg[0].len = 2;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return api->transfer(dev, msg, 2, dev_addr);

}

int read_image_from_eeprom(struct device *i2c_dev,
			   unsigned char *eeprom_img_buf)
{
	/* Start at adress 0x0000, MSB first */
	unsigned char start_adress[] = {0x00, 0x00};
	unsigned int magic;
	const int chunk_size = 64;
	int bytes_to_read;
	unsigned short eeprom_img_size = 0;

	/* Read 4 bytes and check for 0x0deadbeef */
	bytes_to_read = 4;
	eeprom_burst_read(i2c_dev, I2C_EEPROM_ADDR,
			start_adress, (unsigned char *) &magic, bytes_to_read);

	printk("Magic bytes in eeprom: %08x\n", magic);
	if (magic != 0xdeadbeef) {
		printk("No deadbeef\n");
		return -1;
	}
	/* Set next unread adress */
	start_adress[1] = 0x04;
	/* Read 2 bytes, should be size of image */
	bytes_to_read = 2;
	eeprom_burst_read(i2c_dev, I2C_EEPROM_ADDR,
			start_adress, (unsigned char *) &eeprom_img_size,
		       	bytes_to_read);
	printk("Compressed eeprom image size: %d\n", eeprom_img_size);

	for (int bytes_read = 6; bytes_read < eeprom_img_size;
			bytes_read += bytes_to_read) {
		bytes_to_read = (eeprom_img_size - bytes_read);
		if (bytes_to_read > chunk_size)
			bytes_to_read = chunk_size;
		start_adress[1] = (bytes_read & 0xFF);
		start_adress[0] = (bytes_read >> 8) & 0xFF;
		eeprom_burst_read(i2c_dev, I2C_EEPROM_ADDR,
			start_adress, (eeprom_img_buf + bytes_read),
			bytes_to_read);
	}

	return eeprom_img_size;
}

void uncompress_eeprom_image(unsigned char *uncompressed_image,
			     unsigned char *eeprom_img_buf,
			     unsigned short eeprom_img_size)
{
	int i;
	unsigned char o = 0;
	unsigned char sym = 0;
	unsigned char syc = 0;

	int j = 0;

	for (i = 0; i < eeprom_img_size; ++i) {
		sym = eeprom_img_buf[i] & 0x80;
		syc = eeprom_img_buf[i] & 0x7F;

		while (syc) {
			if (sym)
				uncompressed_image[j] |= (0x1 << o);
			o++;
			syc--;
			if (o > 7) {
				o = 0;
				j++;
				uncompressed_image[j] = 0x0;
			}
		}
	}
}

