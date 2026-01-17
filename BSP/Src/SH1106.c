/*
 * SH1106.c
 *
 *  Created on: Nov 24, 2024
 *      Author: katog
 */

#include "SH1106.h"



/*static uint8_t SH1106_BUFFER[SH1106_BUFFER_SIZE];*/

/*static SH1106_Handle_t SH1106_Handle;*/

#define EN_RPT_START			1
#define DI_RPT_START			0


/* Function Definitions */
/**
 * @brief
 * @param
 */
uint8_t SH1106_Init(SH1106_Handle_t* SH1106_HANDLE, SH1106_Comms_t* SH1106_COMMS_HANDLE)
{
    uint8_t SH1106_init_cmd_str[] = {
    COMMAND_STREAM, // control byte
    0xAE, //display off
	0xB0, //Set Page Start Address for Page Addressing Mode,0-7
	0x81, //--set contrast control register
	0xFF, // contrast value
	0xA1, //--set segment re-map 0 to 127
	0xA6, //--set normal display
	0xA8, //--set multiplex ratio 1 to 64
	0x3F, // multiplex value
	0xAD, // Set Pump Mode
	0x8B, // Pump ON
	0x32, // Set Pump Voltage 8.0
	0xC8, //Set COM Output Scan Direction
	0xD3, //-set display offset
	0x00, //-not offset
	0xD5, //--set display clock divide ratio/oscillator frequency
	0x80, //--set divide ratio
	0xD9, //--set pre-charge period
	0x1F, //
	0xDA, //--set com pins hardware configuration
	0x12,
	0xDB, //--set vcomh
	0x40, //
	0xAF, //--turn on SH1106 panel
    };
    SH1106_HANDLE->SH1106_COMMS = SH1106_COMMS_HANDLE;

    /* Fill screen with black */
    SH1106_FILL_BUFFER(SH1106_HANDLE, SH1106_COLOR_BLACK);
    if(SH1106_HANDLE->SH1106_COMMS->SH1106_WRITE(SH1106_I2C_ADDR, SH1106_init_cmd_str, sizeof(SH1106_init_cmd_str)/sizeof(SH1106_init_cmd_str[0]), DI_RPT_START) == 0){
    	return 0;
    }
    /* update displayed screen */
    SH1106_UPDATE_SCRN(SH1106_HANDLE, SH1106_I2C_ADDR);
    SH1106_HANDLE->current_x = 0;
    SH1106_HANDLE->current_y = 0;

    return 1;

}
/**
 * @brief
 * @param
 */
void SH1106_FILL_BUFFER(SH1106_Handle_t* SH1106_HANDLE, SH1106_Color_t color)
{
	SH1106_HANDLE->color_fill = color;

	if(color == SH1106_COLOR_BLACK || color == SH1106_COLOR_WHITE){
		memset(SH1106_HANDLE->SH1106_DISP_BUFFER, color, sizeof(SH1106_HANDLE->SH1106_DISP_BUFFER));
	}
}

/**
 * @brief
 * @param
 */
void SH1106_UPDATE_SCRN(SH1106_Handle_t* SH1106_HANDLE, uint8_t Slave_Addr)
{
	uint8_t command[3];
	uint8_t SH1106_BUFFER_temp[133]; // set the temporary buffer to size of main buffer + 1 for allocation of command

	command[0] = COMMAND_STREAM; // I2C command to send multiple commands
	command[1] = 0x00; // 4 lower bits
	command[2] = 0x10; // 4 Higher bits


	for(uint8_t m = 0; m < 8; m++){
		command[1] = 0xB0 + m; // sets the page address for the RAM (offset 0 - 7)
		SH1106_HANDLE->SH1106_COMMS->SH1106_WRITE(Slave_Addr, command, sizeof(command)/sizeof(command[0]), EN_RPT_START);

		for(uint8_t i = 0; i < 133; i++){ // send 1 command + 128 data bytes for each page address
				SH1106_BUFFER_temp[0] = DATA_STREAM;
				SH1106_BUFFER_temp[i + 1] = SH1106_HANDLE->SH1106_DISP_BUFFER[(SH1106_WIDTH * m) + i];
		}

		// sends the buffer value to be displayed on the OLED
		SH1106_HANDLE->SH1106_COMMS->SH1106_WRITE(Slave_Addr, SH1106_BUFFER_temp, 129, DI_RPT_START);
	}
}
/*
 * functions that reflects the bitmap chosen by the user
 */
/**
 * @brief
 * @param
 */
void SH1106_upload_bitmap(SH1106_Handle_t* SH1106_HANDLE, uint8_t *bitmap)
{
	memcpy(SH1106_HANDLE->SH1106_DISP_BUFFER, bitmap, sizeof(SH1106_HANDLE->SH1106_DISP_BUFFER));
}

/*
 * inverts all screen contents
 */
/**
 * @brief
 * @param
 */
void SH1106_Toggle_Invert(SH1106_Handle_t* SH1106_HANDLE)
{
	/* memory invert */
	for(uint16_t i = 0; i < sizeof(SH1106_HANDLE->SH1106_DISP_BUFFER); i++){
		SH1106_HANDLE->SH1106_DISP_BUFFER[i] = ~(SH1106_HANDLE->SH1106_DISP_BUFFER[i]);
	}
	/*  toggle invert */
	SH1106_HANDLE->color_fill = ~(SH1106_HANDLE->color_fill) & 0xFF;
}

/**
 * @brief
 * @param
 */
uint8_t SH1106_Draw_Pixel(SH1106_Handle_t* SH1106_HANDLE, uint8_t x, uint8_t y, SH1106_Color_t color)
{
	if(x >= SH1106_WIDTH || y >= SH1106_HEIGHT){
		return 0;
	}
	if (SH1106_HANDLE->color_fill == color)
	{
		color = ~(color);
	}
	if (color == SH1106_COLOR_WHITE) {
		SH1106_HANDLE->SH1106_DISP_BUFFER[x + (y / 8) * SH1106_WIDTH] |= 1 << (y % 8);

	} else {
		SH1106_HANDLE->SH1106_DISP_BUFFER[x + (y / 8) * SH1106_WIDTH] &= ~(1 << (y % 8));
	}

	return 1;

}

/**
 * @brief
 * @param
 */
void SH1106_Draw_Line(SH1106_Handle_t* SH1106_HANDLE, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SH1106_Color_t color)
{
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if (x0 >= SH1106_WIDTH) {
		x0 = SH1106_WIDTH - 1;
	}
	if (x1 >= SH1106_WIDTH) {
		x1 = SH1106_WIDTH - 1;
	}
	if (y0 >= SH1106_HEIGHT) {
		y0 = SH1106_HEIGHT - 1;
	}
	if (y1 >= SH1106_HEIGHT) {
		y1 = SH1106_HEIGHT - 1;
	}

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			SH1106_Draw_Pixel(SH1106_HANDLE, x0, i, color);
		}

		/* Return from function */
		return;
	}

	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			SH1106_Draw_Pixel(SH1106_HANDLE, i, y0, color);
		}

		/* Return from function */
		return;
	}

	while (1) {
		SH1106_Draw_Pixel(SH1106_HANDLE, x0, y0, color);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}

/**
 * @brief
 * @param
 */
void SH1106_Draw_Rectangle(SH1106_Handle_t* SH1106_HANDLE, uint16_t x, uint16_t y, uint16_t w, uint16_t h, SH1106_Color_t color)
{
	/* Check input parameters */
	if (
		x >= SH1106_WIDTH ||
		y >= SH1106_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SH1106_WIDTH) {
		w = SH1106_WIDTH - x;
	}
	if ((y + h) >= SH1106_HEIGHT) {
		h = SH1106_HEIGHT - y;
	}

	/* Draw 4 lines */
	SH1106_Draw_Line(SH1106_HANDLE, x, y, x + w, y, color);         /* Top line */
	SH1106_Draw_Line(SH1106_HANDLE, x, y + h, x + w, y + h, color); /* Bottom line */
	SH1106_Draw_Line(SH1106_HANDLE, x, y, x, y + h, color);         /* Left line */
	SH1106_Draw_Line(SH1106_HANDLE, x + w, y, x + w, y + h, color); /* Right line */
}

/**
 * @brief
 * @param
 */
void SH1106_Draw_Filled_Rectangle(SH1106_Handle_t* SH1106_HANDLE, uint16_t x, uint16_t y, uint16_t w, uint16_t h, SH1106_Color_t color)
{
	uint8_t i;

	/* Check input parameters */
	if (
		x >= SH1106_WIDTH ||
		y >= SH1106_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SH1106_WIDTH) {
		w = SH1106_WIDTH - x;
	}
	if ((y + h) >= SH1106_HEIGHT) {
		h = SH1106_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		SH1106_Draw_Line(SH1106_HANDLE, x, y + i, x + w, y + i, color);
	}
}

/**
 * @brief
 * @param
 */
void SH1106_GotoXY(SH1106_Handle_t* SH1106_HANDLE, uint16_t x, uint16_t y)
{
	/* Set write pointers */
	SH1106_HANDLE->current_x = x;
	SH1106_HANDLE->current_y = y;
}

/**
 * @brief
 * @param
 */
char SH1106_Putc(SH1106_Handle_t* SH1106_HANDLE, char ch, FontDef_t* Font, SH1106_Color_t color)
{
	uint32_t i, b, j;

	/* Check available space in LCD */
	if ( SH1106_WIDTH <= (SH1106_HANDLE->current_x + Font->FontWidth) ||
		SH1106_HEIGHT <= (SH1106_HANDLE->current_y + Font->FontHeight) ) {
		/* Error */
		return 0;
	}

	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++){
		b = Font->data[(ch - 32) * Font->FontHeight + i];

		for(j = 0; j < Font->FontWidth; j++){
			if((b << j) & 0x8000) {
				SH1106_Draw_Pixel(SH1106_HANDLE, SH1106_HANDLE->current_x + j, (SH1106_HANDLE->current_y + i), color);
			} else {
				SH1106_Draw_Pixel(SH1106_HANDLE, SH1106_HANDLE->current_x + j, (SH1106_HANDLE->current_y + i), !color);
			}
		}
	}

	/* Increase pointer */
	SH1106_HANDLE->current_x += Font->FontWidth;

	/* Return character written */
	return ch;
}

/**
 * @brief
 * @param
 */
char SH1106_Puts(SH1106_Handle_t* SH1106_HANDLE, char* str, FontDef_t* Font, SH1106_Color_t color)
{
	/* Write characters */
	while(*str){
		/* Write character by character */
		if(SH1106_Putc(SH1106_HANDLE, *str, Font, color) != *str){
			/* Return error */
			return *str;
		}

		/* Increase string pointer */
		str++;
	}

	/* Everything OK, zero should be returned */
	return *str;
}
