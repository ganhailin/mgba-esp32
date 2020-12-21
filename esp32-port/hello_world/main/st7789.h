/* vim: set ai et ts=4 sw=4: */
#ifndef __ST7789_H__
#define __ST7789_H__

#include "fonts.h"
#include <stdbool.h>
#define ST7789_MOSI_Pin        19
#define ST7789_CLK_Pin        18
#define ST7789_CS_Pin        22
#define ST7789_DC_Pin        21
#define ST7789_RST_Pin        -1

#define PARALLEL_LINES 16
#define ST7789_COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))
#define CONFIG_LCD_OVERCLOCK

void ST7789_Init() ;
void ST7789_Flush(uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint8_t *data);



#endif // __ST7735_H__
