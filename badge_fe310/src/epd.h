#ifndef __EPD_H__
#define __EPD_H__

#define LCD_COMMAND_INDEX_HEADER 0x70
#define LCD_COMMAND_DATA_HEADER 0x72
#define LCD_GET_ID_HEADER 0x71

void epd_data(unsigned char *buff, u32_t len, unsigned char mode);
void epd_cmd(unsigned char cmd);
void epd_w1b(unsigned char cmd, unsigned char val);
void epd_poweron(void);

enum EPD_size {
	EPD_1_44,        /* 128 x 96 */
	EPD_1_9,         /* 144 x 128 */
	EPD_2_0,         /* 200 x 96 */
	EPD_2_6,         /* 232 x 128 */
	EPD_2_7          /* 264 x 176 */
};

enum EPD_error {        /* error codes */
	EPD_OK,
	EPD_UNSUPPORTED_COG,
	EPD_PANEL_BROKEN,
	EPD_DC_FAILED,
	EPD_UNDEFINED
};

/* types */
enum EPD_stage {           /* Image pixel -> Display pixel */
	EPD_compensate,  /* B -> W, W -> B (Current Image) */
	EPD_white,       /* B -> N, W -> W (Current Image) */
	EPD_inverse,     /* B -> N, W -> B (New Image) */
	EPD_normal       /* B -> B, W -> W (New Image) */
};

enum EPD_border_byte {
	EPD_BORDER_BYTE_NONE,  /* no border byte requred */
	EPD_BORDER_BYTE_ZERO,  /* border byte == 0x00 requred */
	EPD_BORDER_BYTE_SET,   /* border byte needs to be set */
};

/* panel configuration */
struct EPD_configuration {
	enum EPD_size size;
	int stage_repetitions;
	int lines_per_display;
	int dots_per_line;
	int bytes_per_line;
	int bytes_per_scan;
	bool middle_scan;

	bool pre_border_byte;
	enum EPD_border_byte border_byte;

	unsigned char *line_buffer;
	size_t line_buffer_size;
};

struct EPD_private {
	struct spi_config *spi_conf;
};

void epd_init(struct spi_config *spi_conf);
void epd_gpio_init(void);
void epd_clear(struct EPD_configuration *epd);
void epd_image_0(struct EPD_configuration *epd, const unsigned char *image);
void epd_image(struct EPD_configuration *epd, const unsigned char *old_image,
		const unsigned char *new_image);
void epd_poweroff(void);
void epd_discharge(void);

#endif
