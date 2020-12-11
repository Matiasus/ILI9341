/** 
 * ---------------------------------------------------------------+ 
 * @desc        ILI9341 LCD Driver 
 * ---------------------------------------------------------------+ 
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @datum       10.12.2020
 * @file        ili9341.h
 * @tested      AVR Atmega16
 *
 * @depend      spi, font
 * ---------------------------------------------------------------+
 * @interface   8080-I/II Series Parallel Interface
 * @pins        5V, 3.3V, GND, RST, CS, RS, WR, RD, D[7:0] 
 *
 */

#include <avr/pgmspace.h>

#ifndef __ILI9341_H__
#define __ILI9341_H__

  /* HARDWARE DEFINITION */
  #define DDR_D0_D7         DDRD
  #define PORT_D0_D7        PORTD
  #define PIN_D0_D7         PIND

  #define DDR_CONTROL       DDRB
  #define PORT_CONTROL      PORTB
  #define PIN_RST           0
  #define PIN_CS            1
  #define PIN_RS            2
  #define PIN_WR            3
  #define PIN_RD            4

  /* SOFTWARE DEFINITION */
  #define ILI9341_SUCCESS   0
  #define ILI9341_ERROR     1
  
  #define ILI9341_NOOPR     0x00  // No Operation
  #define ILI9341_SWRST     0x01  // Software Reset
  #define ILI9341_RDDID     0x04  // Read Display Identification Information
  #define ILI9341_RDDST     0x09  // Read Display Status
  #define ILI9341_RDPMD     0x0A  // Read Display Power Mode
  #define ILI9341_RMDCT     0x0B  // Read Display MADCTL
  #define ILI9341_RDPFT     0x0C  // Read Display Pixel Format
  #define ILI9341_RDIFT     0x0D  // Read Display Image Format
  #define ILI9341_RDSMD     0x0E  // Read Display Signal Mode
  #define ILI9341_RDSDR     0x0F  // Read Display Self Diagnostics Result

  #define ILI9341_SLPIN     0x10  // Enter Sleep Mode
  #define ILI9341_SLOUT     0x11  // Sleep Out
  #define ILI9341_PTLON     0x12  // Partial Mode On
  #define ILI9341_NORON     0x13  // Normal Display On

  #define ILI9341_INOFF     0x20  // Dislpay Inversion Off
  #define ILI9341_INVON     0x21  // Dislpay Inversion On
  #define ILI9341_GAMST     0x26  // Gamma Set  
  #define ILI9341_DIOFF     0x28  // Display OFF
  #define ILI9341_DISON     0x29  // Display ON
  #define ILI9341_CASET     0x2A  // Column Address Set
  #define ILI9341_PASET     0x2B  // Page Address Set
  #define ILI9341_MEMWR     0x2C  // Memory Write
  #define ILI9341_COLST     0x2D  // Color Set
  #define ILI9341_MEMRD     0x2E  // Memory Read

  #define ILI9341_PTLAR     0x30  // Partial Area
  #define ILI9341_VSDEF     0x33  // Vertical Scroll Definition
  #define ILI9341_TELOF     0x34  // Tearing Effect Line OFF
  #define ILI9341_TELON     0x35  // Tearing Effect Line ON
  #define ILI9341_MEMAC     0x36  // Memory Access Control
  #define ILI9341_VSSAD     0x37  // Vertical Scrolling Start Address
  #define ILI9341_IDLOF     0x38  // Idle Mode OFF
  #define ILI9341_IDLON     0x39  // Idle Mode ON
  #define ILI9341_PFSET     0x3A  // Pixel Format Set
  #define ILI9341_WMCON     0x3C  // Write Memory Continue
  #define ILI9341_RMCON     0x3E  // Read Memory Continue

  #define ILI9341_RGBIC     0xB0  // RGB Interface Signal Control
  #define ILI9341_FRCRN     0xB1  // Frame Control (In Normal Mode)
  #define ILI9341_FRCRI     0xB2  // Frame Control (In Idle Mode)
  #define ILI9341_FRCRP     0xB3  // Frame Control (In Partial Mode)
  #define ILI9341_INVCR     0xB4  // Display Inversion Control
  #define ILI9341_BLPCR     0xB5  // Blanking Porch Control
  #define ILI9341_DISCR     0xB6  // Display Function Control
  #define ILI9341_EMSET     0xB7  // Entry Mode Set
  #define ILI9341_BKCR1     0xB8  // Backlight Control 1
  #define ILI9341_BKCR2     0xB9  // Backlight Control 2
  #define ILI9341_BKCR3     0xBA  // Backlight Control 3
  #define ILI9341_BKCR4     0xBB  // Backlight Control 4
  #define ILI9341_BKCR5     0xBC  // Backlight Control 5
  #define ILI9341_BKCR7     0xBE  // Backlight Control 7
  #define ILI9341_BKCR8     0xBF  // Backlight Control 8

  #define ILI9341_PWCR1     0xC0  // Power Control 1
  #define ILI9341_PWCR2     0xC1  // Power Control 2
  #define ILI9341_VCCR1     0xC5  // VCOM Control 1
  #define ILI9341_VCCR2     0xC6  // VCOM Control 2
 
  #define ILI9341_RDID1     0xDA  // Read ID1
  #define ILI9341_RDID2     0xDB  // Read ID2
  #define ILI9341_RDID3     0xDC  // Read ID3

  #define ILI9341_POGCR     0xE0  // Positive Gamma Correction
  #define ILI9341_NEGCO     0xE1  // Neagtove Gamma Correction

  // Colors
  #define BLACK           0x0000
  #define WHITE           0xFFFF
  #define RED             0xF000

  // MV = 0 in MADCTL
  // max columns
  #define MAX_X   161
  // max rows
  #define MAX_Y   130
  // columns max counter
  #define SIZE_X  MAX_X - 1
  // rows max counter
  #define SIZE_Y  MAX_Y - 1
  // whole pixels
  #define CACHE_SIZE_MEM (MAX_X * MAX_Y)
  // number of columns for chars
  #define CHARS_COLS_LEN 5
  // number of rows for chars
  #define CHARS_ROWS_LEN 8

  /** @const Command list ILI9341B */
  extern const uint8_t INIT_ILI9341[];

#endif
