#ifndef TEST_H
#define TEST_H

#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "gpio.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

extern int cursor_x;
extern int cursor_y;

/*
#define SPI_BASE        SPIA0_BASE
#define SPI_CLK_PIN     PIN_05   # CLK
#define SPI_MOSI_PIN    PIN_07   # MOSI
#define OLED_DC_PIN     PIN_62   # DC
#define OLED_CS_PIN     PIN_63   # CS
#define OLED_RESET_PIN  PIN_61   # RESET*/


#define SPI_IF_BIT_RATE  100000


// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF


void initSPI();
void delay(unsigned long ulCount);
void testfastlines(unsigned int color1, unsigned int color2);
void testdrawrects(unsigned int color);
void testfillrects(unsigned int color1, unsigned int color2);
void testfillcircles(unsigned char radius, unsigned int color);
void testdrawcircles(unsigned char radius, unsigned int color);
void testtriangles();
void testroundrects();
void testlines(unsigned int color) ;
void lcdTestPattern(void);
void lcdTestPattern2(void);

#endif
