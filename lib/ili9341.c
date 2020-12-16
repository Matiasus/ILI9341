/** 
 * -----------------------------------------------------------------------+ 
 * @desc        ILI9341 LCD Driver 
 * -----------------------------------------------------------------------+
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @datum       10.12.2020
 * @update      13.12.2020
 * @file        ili9341.c
 * @tested      AVR Atmega16
 *
 * @depend      font
 * -----------------------------------------------------------------------+
 * @inspir      https://github.com/notro/fbtft/blob/master/fb_ili9341.c
                https://github.com/adafruit/Adafruit_ILI9341/blob/master/Adafruit_ILI9341.cpp
 */

#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include "font.h"
#include "ili9341.h"

/** @array Init command */
const uint8_t INIT_ILI9341[] PROGMEM = {
  // number of initializers
  11,

  // --------------------------------------------  
  0,  50, ILI9341_SWRESET,                                      // 0x01 -> Software reset
  0,   0, ILI9341_DISPOFF,                                      // 0x28 -> Display OFF
/*
  // --------------------------------------------
  3,   0, 0xEF, 0x03, 0x80, 0x02,                               // 0xEF
  3,   0, ILI9341_LCD_POWERB, 0x00, 0xC1, 0x30,                 // 0xCF -> Power control B
  4,   0, ILI9341_LCD_POWER_SEQ, 0x64, 0x03, 0x12, 0x81,        // 0xED -> Power on sequence
  3,   0, ILI9341_LCD_DTCA, 0x85, 0x00, 0x78,                   // 0xE8 -> Driver timing control A
  5,   0, ILI9341_LCD_POWERA, 0x39, 0x2C, 0x00, 0x34, 0x02,     // 0xCB -> Power control A
  1,   0, ILI9341_LCD_PRC, 0x20,                                // 0xF7 -> Pump ratio control
  2,   0, ILI9341_LCD_DTCB, 0x00, 0x00,                         // 0xEA -> Driver timing control B
*/
  // --------------------------------------------  
  1,   0, ILI9341_PWCTRL1, 0x23,                                // 0xC0 -> Power Control 1
  1,   0, ILI9341_PWCTRL2, 0x10,                                // 0xC1 -> Power Control 2
  2,   0, ILI9341_VCCR1, 0x2B, 0x2B,                            // 0xC5 -> VCOM Control 1
  1,   0, ILI9341_VCCR2, 0xC0,                                  // 0xC7 -> VCOM Control 2

  // -------------------------------------------- 
  1,   0, ILI9341_MADCTL, 0x40,                                 // 0x36 -> Memory Access Control
  1,   0, ILI9341_COLMOD, 0x55,                                 // 0x3A -> Pixel Format Set
  2,   0, ILI9341_FRMCRN1, 0x00, 0x1B,                          // 0xB1 -> Frame Rate Control
/*
  3,   0, ILI9341_DISCTRL, 0x08, 0x82, 0x27,                    // 0xB6 -> Display Function Control
  1,   0, 0xF2, 0x00,                                           // 0xF2 -> gamma function disable
  2,   0, ILI9341_GAMSET, 0x00, 0x1B,                           // 0x26 -> Gamma Set
*/
  1,   0, ILI9341_ETMOD, 0x07,                                  // 0xB7 -> Entry Mode Set
/*  
  // Set Gamma - positive
  15,  0, ILI9341_GMCTRP1 , 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  // Set Gamma - negative
  15,  0, ILI9341_GMCTRN1 , 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
*/
  // --------------------------------------------
  0, 150, ILI9341_SLPOUT,                                       // 0x11 -> Sleep Out
//  0, 200, ILI9341_DISPON                                        // 0x29 -> Display on
};

/** @var array Chache memory char index row */
unsigned short int cacheMemIndexRow = 0;
/** @var array Chache memory char index column */
unsigned short int cacheMemIndexCol = 0;

/**
 * @desc    LCD init
 *
 * @param   void
 *
 * @return  void
 */
void ILI9341_Init (void)
{
  // variables
  const uint8_t *commands = INIT_ILI9341;
  // number of commands
  unsigned short int no_of_commands = pgm_read_byte(commands++);
  // arguments
  char no_of_arguments;
  // command
  char command;
  // delay
  unsigned short int delay;

  // Init ports
  ILI9341_InitPorts();

  // Init hardware reset
  ILI9341_HWReset();

  // loop throuh commands
  while (no_of_commands--) {

    // number of arguments
    no_of_arguments = pgm_read_byte(commands++);
    // delay
    delay = pgm_read_byte(commands++);
    // command
    command = pgm_read_byte(commands++);

    // send command
    // -------------------------    
    ILI9341_TransmitCmmd(command);

    // send arguments
    // -------------------------
    while (no_of_arguments--) {
      // send arguments
      ILI9341_Transmit8bitData(pgm_read_byte(commands++));
    }
    // delay
    ILI9341_Delay(delay);
  }
  // set window -> after this function display show RAM content
  ILI9341_SetWindow(0, 0, ILI9341_MAX_X-1, ILI9341_MAX_Y-1);
}

/**
 * @desc    LCD Transmit Command
 *
 * @param   
 *
 * @return  void
 */
void ILI9341_SetWindow (uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)
{
  ILI9341_TransmitCmmd(ILI9341_CASET);
  ILI9341_Transmit16bitData(xs);
  ILI9341_Transmit16bitData(xe);

  ILI9341_TransmitCmmd(ILI9341_PASET);
  ILI9341_Transmit16bitData(ys);
  ILI9341_Transmit16bitData(ye);
}

/**
 * @desc    LCD Write color pixels
 *
 * @param   uint16_t
 * @param   uint16_t
 *
 * @return  void
 */
void ILI9341_SendColor565 (uint16_t color, uint32_t count)
{
  // access to RAM
  ILI9341_TransmitCmmd(ILI9341_RAMWR);
  // counter
  while (count--) {
    // write color
    ILI9341_Transmit16bitData(color);
  }
}

/**
 * @desc    LCD Draw Pixel
 *
 * @param   uint16_t
 * @param   uint16_t
 * @param   uint16_t
 *
 * @return  void
 */
void ILI9341_DrawPixel (uint16_t x, uint16_t y, uint16_t color)
{
  // set window
  ILI9341_SetWindow(x, y, x, y);
  // draw pixel by 565 mode
  ILI9341_SendColor565(color, 1);
}

/**
 * @desc    Clear screen
 *
 * @param   uint16_t color
 *
 * @return  void
 */
void ILI9341_ClearScreen (uint16_t color)
{
  // set whole window
  ILI9341_SetWindow(0, ILI9341_MAX_X - 1, 0, ILI9341_MAX_Y - 1);
  // draw individual pixels
  ILI9341_SendColor565(color, ILI9341_CACHE_MEM);
}

/**
 * @desc    LCD Inverse Screen
 *
 * @param   void
 *
 * @return  void
 */
void ILI9341_InverseScreen (void)
{
  // display on
  ILI9341_TransmitCmmd(ILI9341_DINVON);
}

/**
 * @desc    LCD Normal Screen
 *
 * @param   void
 *
 * @return  void
 */
void ILI9341_NormalScreen (void)
{
  // display on
  ILI9341_TransmitCmmd(ILI9341_DINVOFF);
}

/**
 * @desc    LCD Update Screen
 *
 * @param   void
 *
 * @return  void
 */
void ILI9341_UpdateScreen (void)
{
  // display on
  ILI9341_TransmitCmmd(ILI9341_DISPON);
}

/**
 * @desc    LCD Transmit Command
 *
 * @param   char
 *
 * @return  void
 */
void ILI9341_TransmitCmmd (char cmmd)
{
  // D/C -> LOW
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RS);
  // enable chip select -> LOW
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);

  // Write data timing diagram
  // --------------------------------------------
  //              ___
  // D0 - D7:  __/   \__
  //          __    __
  //      WR:   \__/

  // set command on PORT
  ILI9341_PORT_DATA = cmmd;
  // WR -> LOW
  WR_IMPULSE();

  // D/C -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RS);
  // disable chip select -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS); 
}

/**
 * @desc    LCD transmit
 *
 * @param   uint8_t
 *
 * @return  void
 */
void ILI9341_Transmit8bitData (uint8_t data)
{
  // D/C -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RS);
  // enable chip select -> LOW
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);

  // Write data timing diagram
  // --------------------------------------------
  //              ___
  // D0 - D7:  __/   \__
  //          __    __
  //      WR:   \__/

  // set data on PORT
  ILI9341_PORT_DATA = data;
  // WR -> LOW
  WR_IMPULSE();

  // disable chip select -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);
}

/**
 * @desc    LCD transmit
 *
 * @param   uint16_t
 *
 * @return  void
 */
void ILI9341_Transmit16bitData (uint16_t data)
{
  // D/C -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RS);
  // enable chip select -> LOW
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);

  // Write data timing diagram
  // --------------------------------------------
  //              ___
  // D0 - D7:  __/   \__
  //          __    __
  //      WR:   \__/

  // set higher 8 bit data on PORT
  ILI9341_PORT_DATA = (uint8_t) data >> 8;
  // WR -> LOW
  WR_IMPULSE();

  // set lower 8 bit data on PORT
  ILI9341_PORT_DATA = (uint8_t) data;
  // WR -> LOW
  WR_IMPULSE();

  // disable chip select -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);
}

/**
 * @desc    LCD init PORTs
 *
 * @param   void
 *
 * @return  void
 */
void ILI9341_HWReset (void)
{
  // set RESET as Output
  SETBIT(ILI9341_DDR_CONTROL, ILI9341_PIN_RST);

  // Idle Mode
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS); 
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RD); 
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_WR);

  // RESET SEQUENCE
  // --------------------------------------------
  // set Reset LOW
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RST);
  // delay > 10us
  _delay_ms(10);
  // set Reset HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RST);

  // CS Active
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);
  // Command Active
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RS);  

  // set command on PORT
  ILI9341_PORT_DATA = 0x00;
  // WR strobe
  WR_IMPULSE();
  // WR strobe
  WR_IMPULSE();
  // WR strobe
  WR_IMPULSE();

  // Idle Mode
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);

  // delay > 10us
  _delay_ms(200);  
}

/**
 * @desc    LCD init PORTs
 *
 * @param   void
 *
 * @return  void
 */
void ILI9341_InitPorts (void)
{
  // set control pins as output
  SETBIT(ILI9341_DDR_CONTROL, ILI9341_PIN_CS);
  SETBIT(ILI9341_DDR_CONTROL, ILI9341_PIN_RS);
  SETBIT(ILI9341_DDR_CONTROL, ILI9341_PIN_RD);
  SETBIT(ILI9341_DDR_CONTROL, ILI9341_PIN_WR);

  // set HIGH Level on all pins - IDLE MODE
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS); 
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RS); 
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RD); 
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_WR);

  // set all pins as output
  ILI9341_DDR_DATA = 0xFF;
}


/**
 * @desc    Delay
 *
 * @param   unsigned short int
 *
 * @return  void
 */
void ILI9341_Delay(unsigned short int time)
{
  // loop through real time
  while (time--) {
    // 1 s delay
    _delay_ms(1);
  }
}
