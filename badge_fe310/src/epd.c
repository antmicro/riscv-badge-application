/*
 * Copyright (c) 2013-2015 Pervasive Displays, Inc.
 * Copyright (c) 2017 Antmicro <www.antmicro.com>
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <string.h>
#include <misc/printk.h>
#include <device.h>
#include <spi.h>
#include "gpio_fe310.h"
#include "spi-badge.h"
#include "epd.h"

#define PWM_GPIO 21
static void frame_fixed(struct EPD_configuration *epd, unsigned char fixed_value,
			 enum EPD_stage stage);

static void frame_data(struct EPD_configuration *epd, const unsigned char *image,
			const unsigned char *mask, enum EPD_stage stage);

static void frame_fixed_repeat(struct EPD_configuration *epd, unsigned char fixed_value,
			enum EPD_stage stage);

static void frame_data_repeat(struct EPD_configuration *epd, const unsigned char *image,
			const unsigned char *mask, enum EPD_stage stage);

static void one_line(struct EPD_configuration *epd, u16_t line, const unsigned char *data,
			unsigned char fixed_value, const unsigned char *mask,
			enum EPD_stage stage);

static struct EPD_private priv;

void epd_gpio_init(void)
{
	GPIO_REG(GPIO_OUTPUT_EN) |= ((0x1 << LCD_RESET_OFFSET));
	GPIO_REG(GPIO_OUTPUT_EN) |= (0x1 << LCD_DISCHARGE_OFFSET);
	GPIO_REG(GPIO_OUTPUT_EN) |= (0x1 << PWM_GPIO);
	GPIO_REG(GPIO_OUTPUT_EN) &= ~((0x1 << LCD_BUSY_OFFSET));
	GPIO_REG(GPIO_INPUT_EN) |= ((0x1 << LCD_BUSY_OFFSET));
	GPIO_REG(GPIO_INPUT_EN) &= ~((0x1 << LCD_RESET_OFFSET));
	GPIO_REG(GPIO_INPUT_EN) &= ~(0x1 << PWM_GPIO);

}


void epd_poweron(void)
{
	GPIO_REG(GPIO_OUTPUT_VAL) &= ~(0x1 << LCD_RESET_OFFSET);
	GPIO_REG(GPIO_OUTPUT_VAL) &= ~(0x1 << PWM_GPIO);
	k_sleep(5);

	unsigned char buf = 0x00; /* Dummy byte */

	spi_send_buffer(priv.spi_conf, &buf, 1);
	/* Drive panel_on pin here if present */

	k_sleep(10);

	GPIO_REG(GPIO_OUTPUT_VAL) |=  (0x1 << LCD_RESET_OFFSET);

	k_sleep(5);

	GPIO_REG(GPIO_OUTPUT_VAL) &= ~(0x1 << LCD_RESET_OFFSET);

	k_sleep(5);

	GPIO_REG(GPIO_OUTPUT_VAL) |=  (0x1 << LCD_RESET_OFFSET);
}

void epd_discharge(void)
{
	GPIO_REG(GPIO_OUTPUT_VAL) |=  (0x1 << LCD_DISCHARGE_OFFSET);
	k_sleep(150);
	GPIO_REG(GPIO_OUTPUT_VAL) &=  ~(0x1 << LCD_DISCHARGE_OFFSET);
}

void epd_poweroff(void)
{
	/* Undocumented but apparently required */
	epd_w1b(0x0b, 0x00);

	/* latch reset turn on */
	epd_w1b(0x03, 0x01);

	/* power off charge pump Vcom */
	epd_w1b(0x05, 0x03);

	/* power off charge pump neg voltage */
	epd_w1b(0x05, 0x01);

	k_sleep(120);

	/* discharge internal */
	epd_w1b(0x04, 0x80);

	/* turn off all charge pumps */
	epd_w1b(0x05, 0x00);

	/* turn of osc */
	epd_w1b(0x07, 0x01);

	k_sleep(50);

	GPIO_REG(GPIO_OUTPUT_VAL) &= ~(0x1 << LCD_RESET_OFFSET);
	/* Disable panel_on pin here if present */
	GPIO_REG(GPIO_OUTPUT_VAL) &= ~(0x1 << PWM_GPIO);

}

void epd_busy(void)
{
	volatile unsigned int input_val;

	while ((input_val = GPIO_REG(GPIO_INPUT_VAL))
		& (0x1 << LCD_BUSY_OFFSET)) {
	}
}

void epd_cmd(unsigned char cmd)
{
	/* disable RX fifo */
	unsigned char buf[2] = {LCD_COMMAND_INDEX_HEADER, cmd};

	spi_send_buffer(priv.spi_conf, buf, 2);
}

void epd_data(unsigned char *buff, u32_t len, unsigned char mode)
{
	size_t bufsize = len + 1;
	unsigned char buf[bufsize];
	size_t index = 0;

	buf[index++] = LCD_COMMAND_DATA_HEADER | mode;
	for (u32_t i = 0; i < len; i++) {
		/* If mode is SPI_READ, transfer dummy byte;
		 * if not transmit actual data
		 */
		buf[index++] = mode ? 0x00 : buff[i];
	}
	spi_send_buffer(priv.spi_conf, buf, bufsize);
}

unsigned char epd_getid(void)
{
	unsigned char rx_buf[2];
	/* Transmit header and dummy byte */
	unsigned char tx_buf[2] = {LCD_GET_ID_HEADER, 0x00};

	spi_transceive_buffer(priv.spi_conf, tx_buf, 2, rx_buf, 2);
	return rx_buf[1];
}

void epd_w1b(unsigned char cmd, unsigned char val)
{
	epd_cmd(cmd);
	epd_data(&val, 1, SPI_WRITE);
}

void epd_draw_line(unsigned char *data, unsigned char fixed_val)
{
	int b;
	unsigned char line_buffer[200];
	unsigned char *p = line_buffer;

	/* prepare data */
	/*  set border byte 0x00 */
	*(p++) = 0x00;
	/* fixed value */
	if (data == NULL) {
		/* set odd pixels (200 / 8 = 25 bytes) */
		for (b = 0; b < 25; b++)
			*(p++) = fixed_val;
		/* set 24 scan bytes (96 lines / 4 = 24 bytes) */
		for (b = 0; b < 24; b++)
			*(p++) = 0xFF;
		/* set even pixels (200 / 8 = 25 bytes) */
		for (b = 0; b < 25; b++)
			*(p++) = fixed_val;
	} else {
		printk("NYI\n");
		/* Here we have to interlace the data frame and convert */
		/* 200B of data to 50B of data */
	}
	epd_cmd(0x0A);
	epd_data(line_buffer, 75, SPI_WRITE);
	epd_w1b(0x02, 0x07);
}

void epd_init(struct spi_config *spi_conf)
{
	unsigned char data[128];
	unsigned char id;

	priv.spi_conf = spi_conf;

	epd_poweron();
	printk("Waiting for LCD...\n");
	epd_busy();
	id = epd_getid();
	id = epd_getid();

	if (id == 0x12)
		printk("ID OK: %x\n", id);
	else
		printk("Faulty ID: %x\n", id);

	epd_w1b(0x02, 0x40);

	epd_cmd(0x0F);
	epd_data(data, 1, SPI_READ);

	if ((data[0] & 0x80) != 0x80)
		printk("LCD may be broken. Breakage: %x\n", data[0]);

	epd_w1b(0x0B, 0x02);

	epd_cmd(0x01);
	data[0] = 0x00;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x01;
	data[5] = 0xFF;
	data[6] = 0xE0;
	data[7] = 0x00;

	epd_data(data, 8, SPI_WRITE);

	epd_w1b(0x07, 0xD1);
	epd_w1b(0x08, 0x02);
	epd_w1b(0x09, 0xC2);
	epd_w1b(0x04, 0x03);
	epd_w1b(0x03, 0x01);
	epd_w1b(0x03, 0x00);
	k_sleep(10);

	/* enable charge pumps */
	int j;

	for (j = 0; j < 4; j++) {
		epd_w1b(0x05, 0x01);
		k_sleep(10);
		epd_w1b(0x05, 0x03);
		k_sleep(10);
		epd_w1b(0x05, 0x0F);
		k_sleep(10);

		epd_cmd(0x0F);
		epd_data(data, 1, SPI_READ);
		/* we should get 0x04 here, unfortunately I receise 0x00 */
		printk("Voltage status: %x\n", data[0]);
		if ((data[0] & 0x40) == 0x40)
			break;
	}

	epd_w1b(0x02, 0x04);
	printk("LCD initialized.\n");
}


static void frame_fixed(struct EPD_configuration *epd, unsigned char fixed_value,
			 enum EPD_stage stage)
{
	for (unsigned char l = 0; l < epd->lines_per_display ; ++l) {
		one_line(epd, l, NULL, fixed_value, NULL, stage);
	}
}


static void frame_data(struct EPD_configuration *epd, const unsigned char *image,
			const unsigned char *mask, enum EPD_stage stage)
{
	if (mask == NULL) {
		for (unsigned char l = 0; l < epd->lines_per_display ; ++l) {
			one_line(epd, l, &image[l * epd->bytes_per_line],
				 0, NULL, stage);
		}
	} else {
		for (unsigned char l = 0; l < epd->lines_per_display ; ++l) {
			size_t n = l * epd->bytes_per_line;

			one_line(epd, l, &image[n], 0, &mask[n], stage);
		}
	}
}

static void frame_fixed_repeat(struct EPD_configuration *epd, unsigned char fixed_value,
			enum EPD_stage stage)
{
	/* We should base it on stage time,
	 * for now just do a fixed number of repetitions
	 */
	for (int i = 0; i < epd->stage_repetitions; ++i) {
		frame_fixed(epd, fixed_value, stage);
	}
}


static void frame_data_repeat(struct EPD_configuration *epd, const unsigned char *image,
			const unsigned char *mask, enum EPD_stage stage)
{
	/* We should base it on stage time,
	 * for now just do a fixed number of repetitions
	 */
	for (int i = 0; i < epd->stage_repetitions; ++i) {
		frame_data(epd, image, mask, stage);
	}
}

/* pixels on display are numbered from 1 so even is actually bits 1,3,5,... */
static void even_pixels(struct EPD_configuration *epd, unsigned char **pp, const unsigned char *data,
			unsigned char fixed_value, const unsigned char *mask,
			enum EPD_stage stage)
{

	for (u16_t b = 0; b < epd->bytes_per_line; ++b) {
		if (data != NULL) {
			unsigned char pixels = data[b] & 0xaa;
			unsigned char pixel_mask = 0xff;

			if (mask != NULL) {
				pixel_mask = (mask[b] ^ pixels) & 0xaa;
				pixel_mask |= pixel_mask >> 1;
			}
			switch (stage) {
			/* B -> W, W -> B (Current Image) */
			case EPD_compensate:
				pixels = 0xaa | ((pixels ^ 0xaa) >> 1);
				break;
			/* B -> N, W -> W (Current Image) */
			case EPD_white:
				pixels = 0x55 + ((pixels ^ 0xaa) >> 1);
				break;
			/* B -> N, W -> B (New Image) */
			case EPD_inverse:
				pixels = 0x55 | (pixels ^ 0xaa);
				break;
			/* B -> B, W -> W (New Image) */
			case EPD_normal:
				pixels = 0xaa | (pixels >> 1);
				break;
			}
			pixels = (pixels & pixel_mask) | (~pixel_mask & 0x55);
			unsigned char p1 = (pixels >> 6) & 0x03;
			unsigned char p2 = (pixels >> 4) & 0x03;
			unsigned char p3 = (pixels >> 2) & 0x03;
			unsigned char p4 = (pixels >> 0) & 0x03;

			pixels = (p1 << 0) | (p2 << 2) | (p3 << 4) | (p4 << 6);
			*(*pp)++ = pixels;
		} else {
			*(*pp)++ = fixed_value;
		}
	}
}

/* pixels on display are numbered from 1 so odd is actually bits 0,2,4,... */
static void odd_pixels(struct EPD_configuration *epd, unsigned char **pp, const unsigned char *data,
			unsigned char fixed_value, const unsigned char *mask,
			enum EPD_stage stage)
{
	for (u16_t b = epd->bytes_per_line; b > 0; --b) {
		if (data != NULL) {
			unsigned char pixels = data[b - 1] & 0x55;
			unsigned char pixel_mask = 0xff;

			if (mask != NULL) {
				pixel_mask = (mask[b - 1] ^ pixels) & 0x55;
				pixel_mask |= pixel_mask << 1;
			}
			switch (stage) {
			/* B -> W, W -> B (Current Image) */
			case EPD_compensate:
				pixels = 0xaa | (pixels ^ 0x55);
				break;
			/* B -> N, W -> W (Current Image) */
			case EPD_white:
				pixels = 0x55 + (pixels ^ 0x55);
				break;
			/* B -> N, W -> B (New Image) */
			case EPD_inverse:
				pixels = 0x55 | ((pixels ^ 0x55) << 1);
				break;
			/* B -> B, W -> W (New Image) */
			case EPD_normal:
				pixels = 0xaa | pixels;
				break;
			}
			pixels = (pixels & pixel_mask) | (~pixel_mask & 0x55);
			*(*pp)++ = pixels;
		} else {
			*(*pp)++ = fixed_value;
		}
	}
}

/* interleave bits: (byte)76543210 -> (16 bit).7.6.5.4.3.2.1 */
static inline u16_t interleave_bits(u16_t value)
{
	value = (value | (value << 4)) & 0x0f0f;
	value = (value | (value << 2)) & 0x3333;
	value = (value | (value << 1)) & 0x5555;
	return value;
}

/* pixels on display are numbered from 1 */
static void all_pixels(struct EPD_configuration *epd, unsigned char **pp, const unsigned char *data,
			unsigned char fixed_value, const unsigned char *mask,
			enum EPD_stage stage)
{
	for (u16_t b = epd->bytes_per_line; b > 0; --b) {
		if (data != NULL) {
			u16_t pixels = interleave_bits(data[b - 1]);
			u16_t pixel_mask = 0xffff;

			if (mask != NULL) {
				u16_t pixel_mask = interleave_bits(mask[b - 1]);

				pixel_mask = (pixel_mask ^ pixels) & 0x5555;
				pixel_mask |= pixel_mask << 1;
			}
			switch (stage) {
			/* B -> W, W -> B (Current Image) */
			case EPD_compensate:
				pixels = 0xaaaa | (pixels ^ 0x5555);
				break;
			/* B -> N, W -> W (Current Image) */
			case EPD_white:
				pixels = 0x5555 + (pixels ^ 0x5555);
				break;
			/* B -> N, W -> B (New Image) */
			case EPD_inverse:
				pixels = 0x5555 | ((pixels ^ 0x5555) << 1);
				break;
			/* B -> B, W -> W (New Image) */
			case EPD_normal:
				pixels = 0xaaaa | pixels;
				break;
			}
			pixels = (pixels & pixel_mask) | (~pixel_mask & 0x5555);
			*(*pp)++ = pixels >> 8;
			*(*pp)++ = pixels;
		} else {
			*(*pp)++ = fixed_value;
			*(*pp)++ = fixed_value;
		}
	}
}

/* output one line of scan and data bytes to the display */
static void one_line(struct EPD_configuration *epd, u16_t line, const unsigned char *data,
			unsigned char fixed_value, const unsigned char *mask,
			enum EPD_stage stage)
{
	/* send data */
	epd_cmd(0x0a);

	/* CS low */
	unsigned char *p = epd->line_buffer;

	if (epd->pre_border_byte) {
		*p++ = 0x00;
	}

	if (epd->middle_scan) {
		/* data bytes */
		odd_pixels(epd, &p, data, fixed_value, mask, stage);

		/* scan line */
		for (u16_t b = epd->bytes_per_scan; b > 0; --b) {
			if (line / 4 == b - 1) {
				*p++ = 0x03 << (2 * (line & 0x03));
			} else {
				*p++ = 0x00;
			}
		}

		/* data bytes */
		even_pixels(epd, &p, data, fixed_value, mask, stage);

	} else {
		/* even scan line, but as lines on display are numbered
		 * from 1, line: 1,3,5,...
		 */
		for (u16_t b = 0; b < epd->bytes_per_scan; ++b) {
			if (0 != (line & 0x01) && line / 8 == b) {
				*p++ = 0xc0 >> (line & 0x06);
			} else {
				*p++ = 0x00;
			}
		}

		/* data bytes */
		all_pixels(epd, &p, data, fixed_value, mask, stage);

		/* odd scan line, but as lines on display are numbered
		 * from 1, line: 0,2,4,6,...
		 */
		for (u16_t b = epd->bytes_per_scan; b > 0; --b) {
			if (0 == (line & 0x01) && line / 8 == b - 1) {
				*p++ = 0x03 << (line & 0x06);
			} else {
				*p++ = 0x00;
			}
		}
	}

	/* post data border byte */
	switch (epd->border_byte) {
	/* no border byte requred */
	case EPD_BORDER_BYTE_NONE:
		break;
	/* border byte == 0x00 requred */
	case EPD_BORDER_BYTE_ZERO:
		*p++ = 0x00;
		break;
	/* border byte needs to be set */
	case EPD_BORDER_BYTE_SET:
		switch (stage) {
		case EPD_compensate:
		case EPD_white:
		case EPD_inverse:
			*p++ = 0x00;
			break;
		case EPD_normal:
			*p++ = 0xaa;
			break;
		}
		break;
	}
	/* send the accumulated line buffer */
	epd_data(epd->line_buffer, p - epd->line_buffer, SPI_WRITE);

	/* output data to panel */
	epd_w1b(0x02, 0x07);

	/* Delay_ms(1); */
}

/* clear display (anything -> white) */
void epd_clear(struct EPD_configuration *epd)
{
	printk("Clearing screen\n");
	frame_fixed_repeat(epd, 0xff, EPD_compensate);
	frame_fixed_repeat(epd, 0xff, EPD_white);
	frame_fixed_repeat(epd, 0xaa, EPD_inverse);
	frame_fixed_repeat(epd, 0xaa, EPD_normal);
}

/* assuming a clear (white) screen output an image */
void epd_image_0(struct EPD_configuration *epd, const unsigned char *image)
{
	frame_fixed_repeat(epd, 0xaa, EPD_compensate);
	frame_fixed_repeat(epd, 0xaa, EPD_white);
	frame_data_repeat(epd, image, NULL, EPD_inverse);
	frame_data_repeat(epd, image, NULL, EPD_normal);
}

/* change from old image to new image */
void epd_image(struct EPD_configuration *epd, const unsigned char *old_image,
		const unsigned char *new_image)
{
	frame_data_repeat(epd, old_image, NULL, EPD_compensate);
	frame_data_repeat(epd, old_image, NULL, EPD_white);
	frame_data_repeat(epd, new_image, NULL, EPD_inverse);
	frame_data_repeat(epd, new_image, NULL, EPD_normal);
}

/* change from old image to new image */
void epd_partial_image(struct EPD_configuration *epd, const unsigned char *old_image,
		const unsigned char *new_image)
{
	/* Only need last stage for partial update */
	/* See discussion on issue #19 in the repaper/gratis
	 * repository on github
	 */
	frame_data_repeat(epd, new_image, old_image, EPD_normal);
}
