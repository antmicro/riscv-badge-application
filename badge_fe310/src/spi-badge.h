#ifndef __SPI_CONFIG_H__
#define __SPI_CONFIG_H__

#include <zephyr.h>
#include <string.h>
#include <misc/printk.h>
#include <device.h>
#include <spi.h>

#define LCD_CS 2
#define LCD_RESET_OFFSET 0x1 /* gpio 1 */
#define LCD_BUSY_OFFSET 0x0 /* gpio 0 */
#define LCD_DISCHARGE_OFFSET 0xb /* gpio 11 */

#define SPI_WRITE 0x0
#define SPI_READ  0x1

int spi_transceive_buffer(struct spi_config *spi_conf,
			unsigned char *tx_data, size_t tx_len,
			unsigned char *rx_data, size_t rx_len);

int spi_send_buffer(struct spi_config *spi_conf,
		    unsigned char *data, size_t len);

void spi_setup_for_epd(struct spi_config *spi_conf);

#endif
