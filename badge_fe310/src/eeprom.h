#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <device.h>
#include <i2c.h>

#define I2C_EEPROM_ADDR 166 /* 0b10100110 */

int read_image_from_eeprom(struct device *i2c_dev,
			   unsigned char *eeprom_img_buf);

void uncompress_eeprom_image(unsigned char *uncompressed_image,
			     unsigned char *eeprom_img_buf,
			     unsigned short eeprom_img_size);
#endif
