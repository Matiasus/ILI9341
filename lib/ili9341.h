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
 * @pins        5V, 3.3V -> NC, GND, RST, CS, RS, WR, RD, D[7:0] 
 *
 */

#include <avr/pgmspace.h>

#ifndef __ILI9341_H__
#define __ILI9341_H__

  // HARDWARE DEFINITION
  // 
  // Data
  // ---------------------------------------------------------------
  #define ILI9341_PORT_DATA     PORTD
  #define ILI9341_DDR_DATA      DDRD
  #define ILI9341_PIN_DATA      PIND
  // Control 
  // ---------------------------------------------------------------
  #define ILI9341_DDR_CONTROL   DDRB
  #define ILI9341_PORT_CONTROL  PORTB
  #define ILI9341_PIN_RST       0
  #define ILI9341_PIN_WR        3     // Write
  #define ILI9341_PIN_RS        2     // Register Select -> D/C
  #define ILI9341_PIN_CS        1     // Chip Select
  #define ILI9341_PIN_RD        4     // Read

  // COMMAND DEFINITION
  // ---------------------------------------------------------------
  #define ILI9341_NOP           0x00  // No Operation
  #define ILI9341_SWRESET       0x01  // Software Reset
  #define ILI9341_RDDIDIF       0x04  // Read Display Identification Information
  #define ILI9341_RDDST         0x09  // Read Display Status
  #define ILI9341_RDDPM         0x0A  // Read Display Power Mode
  #define ILI9341_RDDMADCTL     0x0B  // Read Display MADCTL
  #define ILI9341_RDDCOLMOD     0x0C  // Read Display Pixel Format
  #define ILI9341_RDDIM         0x0D  // Read Display Image Format
  #define ILI9341_RDDSM         0x0E  // Read Display Signal Mode
  #define ILI9341_RDDSDR        0x0F  // Read Display Self Diagnostics Result
  // ---------------------------------------------------------------
  #define ILI9341_SLPIN         0x10  // Enter Sleep Mode
  #define ILI9341_SLPOUT        0x11  // Sleep Out
  #define ILI9341_PTLON         0x12  // Partial Mode On
  #define ILI9341_NORON         0x13  // Normal Display On
  // ---------------------------------------------------------------
  #define ILI9341_DINVOFF       0x20  // Dislpay Inversion Off
  #define ILI9341_DINVON        0x21  // Dislpay Inversion On
  #define ILI9341_GAMSET        0x26  // Gamma Set  
  #define ILI9341_DISPOFF       0x28  // Display OFF
  #define ILI9341_DISPON        0x29  // Display ON
  #define ILI9341_CASET         0x2A  // Column Address Set
  #define ILI9341_PASET         0x2B  // Page Address Set
  #define ILI9341_RAMWR         0x2C  // Memory Write
  #define ILI9341_RGBSET        0x2D  // Color Set
  #define ILI9341_RAMRD         0x2E  // Memory Read
  // ---------------------------------------------------------------
  #define ILI9341_PLTAR         0x30  // Partial Area
  #define ILI9341_VSCRDEF       0x33  // Vertical Scroll Definition
  #define ILI9341_TEOFF         0x34  // Tearing Effect Line OFF
  #define ILI9341_TEON          0x35  // Tearing Effect Line ON
  #define ILI9341_MADCTL        0x36  // Memory Access Control
  #define ILI9341_VSSAD         0x37  // Vertical Scrolling Start Address
  #define ILI9341_IDMOFF        0x38  // Idle Mode OFF
  #define ILI9341_IDMON         0x39  // Idle Mode ON
  #define ILI9341_COLMOD        0x3A  // Pixel Format Set
  #define ILI9341_WMCON         0x3C  // Write Memory Continue
  #define ILI9341_RMCON         0x3E  // Read Memory Continue
  // ---------------------------------------------------------------
  #define ILI9341_IFMODE        0xB0  // RGB Interface Signal Control
  #define ILI9341_FRMCRN1       0xB1  // Frame Control (In Normal Mode)
  #define ILI9341_FRMCRN2       0xB2  // Frame Control (In Idle Mode)
  #define ILI9341_FRMCRN3       0xB3  // Frame Control (In Partial Mode)
  #define ILI9341_INVTR         0xB4  // Display Inversion Control
  #define ILI9341_PRCTR         0xB5  // Blanking Porch Control
  #define ILI9341_DISCTRL       0xB6  // Display Function Control
  #define ILI9341_ETMOD         0xB7  // Entry Mode Set
  #define ILI9341_BKCR1         0xB8  // Backlight Control 1
  #define ILI9341_BKCR2         0xB9  // Backlight Control 2
  #define ILI9341_BKCR3         0xBA  // Backlight Control 3
  #define ILI9341_BKCR4         0xBB  // Backlight Control 4
  #define ILI9341_BKCR5         0xBC  // Backlight Control 5
  #define ILI9341_BKCR7         0xBE  // Backlight Control 7
  #define ILI9341_BKCR8         0xBF  // Backlight Control 8
  // ---------------------------------------------------------------
  #define ILI9341_PWCTRL1       0xC0  // Power Control 1
  #define ILI9341_PWCTRL2       0xC1  // Power Control 2
  #define ILI9341_VCCR1         0xC5  // VCOM Control 1
  #define ILI9341_VCCR2         0xC7  // VCOM Control 2
   // ---------------------------------------------------------------
  #define ILI9341_RDID1         0xDA  // Read ID1
  #define ILI9341_RDID2         0xDB  // Read ID2
  #define ILI9341_RDID3         0xDC  // Read ID3
  // ---------------------------------------------------------------
  #define ILI9341_GMCTRP1       0xE0  // Positive Gamma Correction
  #define ILI9341_GMCTRN1       0xE1  // Neagtove Gamma Correction

  // Extend register commands
  // --------------------------------------------------------------- 
  // @source https://github.com/fagcinsk/stm-ILI9341-spi/blob/master/lib/ILI9341/commands.h
  #define ILI9341_LCD_POWERA    0xCB   // Power control A register
  #define ILI9341_LCD_POWERB    0xCF   // Power control B register
  #define ILI9341_LCD_DTCA      0xE8   // Driver timing control A
  #define ILI9341_LCD_DTCB      0xEA   // Driver timing control B
  #define ILI9341_LCD_POWER_SEQ 0xED   // Power on sequence register
  #define ILI9341_LCD_3GAMMA_EN 0xF2   // 3 Gamma enable register
  #define ILI9341_LCD_PRC       0xF7   // Pump ratio control register

  // set bit
  #define SETBIT(REG, BIT)      { REG |= (1 << BIT); }
  // clear bit
  #define CLRBIT(REG, BIT)      { REG &= ~(1 << BIT); }

  // SOFTWARE DEFINITION
  // ---------------------------------------------------------------
  #define ILI9341_SUCCESS       0
  #define ILI9341_ERROR         1

  // Colors
  #define ILI9341_BLACK         0x0000
  #define ILI9341_WHITE         0xFFFF
  #define ILI9341_RED           0xF000

  // max columns
  #define ILI9341_MAX_X         240
  // max rows
  #define ILI9341_MAX_Y         320
  // columns max counter
  #define ILI9341_SIZE_X        ILI9341_MAX_X - 1
  // rows max counter
  #define ILI9341_SIZE_Y        ILI9341_MAX_Y - 1
  // whole pixels
  #define ILI9341_CACHE_MEM     (ILI9341_MAX_X * ILI9341_MAX_Y)

  /** @const Command list ILI9341B */
  extern const uint16_t INIT_ILI9341[];

  /**
   * @desc    LCD Init
   *
   * @param   void
   *
   * @return  void
   */
  void ILI9341_Init (void);

  /**
   * @desc    LCD Hardware reset
   *
   * @param   void
   *
   * @return  void
   */
  void ILI9341_HWReset (void);

 /**
   * @desc    LCD Init PORTs
   *
   * @param   void
   *
   * @return  void
   */
  void ILI9341_InitPorts (void);

  /**
   * @desc    LCD Transmit Command
   *
   * @param   char
   *
   * @return  void
   */
  void ILI9341_TransmitCmmd (char);

  /**
   * @desc    LCD Transmit Data
   *
   * @param   char
   *
   * @return  void
   */
  void ILI9341_TransmitData (char);

  /**
   * @desc    Delay
   *
   * @param   unsigned short int
   *
   * @return  void
   */
  void ILI9341_Delay(unsigned short int);

#endif
