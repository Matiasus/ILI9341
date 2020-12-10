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
 */

#include <avr/pgmspace.h>

#ifndef __ILI9341_H__
#define __ILI9341_H__

  #ifndef PORT
    #define PORT PORTB
  #endif
  #ifndef DDR
    #define DDR  DDRB
  #endif
  #ifndef ILI9341_DC_LD
    #define ILI9341_DC_LD  1
  #endif
  #ifndef ILI9341_BL
    #define ILI9341_BL     2
  #endif
  #ifndef ILI9341_CS_SD
    #define ILI9341_CS_SD  3
  #endif
  #ifndef ILI9341_CS_LD
    #define ILI9341_CS_LD  4
  #endif
  #ifndef ILI9341_MOSI         // SDA
    #define ILI9341_MOSI   5
  #endif
  #ifndef ILI9341_MISO
    #define ILI9341_MISO   6
  #endif
  #ifndef ILI9341_SCK         // SCL
    #define ILI9341_SCK    7
  #endif

  #ifndef HW_RESET_DDR
    #define HW_RESET_DDR  DDRB
  #endif
  #ifndef HW_RESET_PORT
    #define HW_RESET_PORT PORTB
  #endif
  #ifndef HW_RESET_PIN
    #define HW_RESET_PIN  0
  #endif

  #define ILI9341_SUCCESS 0
  #define ILI9341_ERROR   1
  
  #define NOP             0x00
  #define SWRESET         0x01
  #define RDDID           0x04
  #define RDDST           0x09

  #define SLPIN           0x10
  #define SLPOUT          0x11
  #define PTLON           0x12
  #define NORON           0x13

  #define INVOFF          0x20
  #define INVON           0x21
  #define DISPOFF         0x28
  #define DISPON          0x29
  #define RAMRD           0x2E
  #define CASET           0x2A
  #define RASET           0x2B
  #define RAMWR           0x2C

  #define PTLAR           0x30
  #define MADCTL          0x36
  #define COLMOD          0x3A

  #define FRMCTR1         0xB1
  #define FRMCTR2         0xB2
  #define FRMCTR3         0xB3
  #define INVCTR          0xB4
  #define DISSET5         0xB6

  #define PWCTR1          0xC0
  #define PWCTR2          0xC1
  #define PWCTR3          0xC2
  #define PWCTR4          0xC3
  #define PWCTR5          0xC4
  #define VMCTR1          0xC5

  #define RDID1           0xDA
  #define RDID2           0xDB
  #define RDID3           0xDC
  #define RDID4           0xDD

  #define GMCTRP1         0xE0
  #define GMCTRN1         0xE1

  #define PWCTR6          0xFC

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
