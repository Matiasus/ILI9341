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
 */

#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include "font.h"
#include "ssd1306.h"
#include "ili9341.h"

/** @array Init command */
const uint8_t INIT_ILI9341[] PROGMEM = {

  // number of initializers
  // --------------------------------------------
  13,

  // 0x01 -> Software reset
  // --------------------------------------------  
  0,   5, ILI9341_SWRST,

  // 0x28 -> Display OFF
  // --------------------------------------------
  0,   0, ILI9341_DIOFF,

  // 0xC0 -> Power Control 1
  // --------------------------------------------
  // GVDD Voltage 0x17 = 4.00V (GVDD =< AVDD-0.5)
  1,   0, ILI9341_PWCR1, 0x26,

  // 0xC1 -> Power Control 2
  // --------------------------------------------
  // VGH - VGL =< 32V
  1,   0, ILI9341_PWCR2, 0x11,

  // 0xC5 -> VCOM Control 1
  // --------------------------------------------
  // 0x31 = +4.325 (VCOMH)
  // 0x3C = -0.600 (VCOML)  
  2,   0, ILI9341_VCCR1, 0x31, 0x3C,

  // 0xC7 -> VCOM Control 2
  // --------------------------------------------
  // 0xC0 -> nVM VMF[6:0]
  1,   0, ILI9341_VCCR2, 0xC0,

  // 0x3A -> Pixel Format Set
  // --------------------------------------------
  // 0x55 -> 16 bits/ pixel
  1,   0, ILI9341_PFSET, 0x55,

  // 0xB1 -> Frame Rate Control
  // --------------------------------------------
  // 0x00 -> focs / 1
  // 0xB1 -> 70Hz
  2,   0, ILI9341_FRCRN, 0x00, 0x1B,

  // 0x26 -> Gamma Set
  // --------------------------------------------
  // 0x01 -> Gamma curve 1
  2,   0, ILI9341_GAMST, 0x00, 0x1B, 

  // 0xB7 -> Entry Mode Set
  // --------------------------------------------
  // 0 0 0 0 DSTB GON DTE GAS => 0x07
  // DSTB = 0 -> Deep Standby Mode disable
  // GON:DTE = 1 1 -> Normal Display
  // GAS = 1 -> Low Voltage Detec. disable
  1,   0, ILI9341_EMSET, 0x07,

  // 0xB6 -> Display Function Control
  // --------------------------------------------
  // 0x0A -> 0 0 0 0 PTG[1:0] PT[1:0]
  // 0x82 -> REV GS SS SM ISC[3:0]
  // 0x27 -> 0 0 NL[5:0]
  // 0x00 -> 0 0 PCDIV[5:0]
  4,   0, ILI9341_DISCR, 0x0A, 0x82, 0x27, 0x00,

  // 0x11 -> Sleep Out
  // --------------------------------------------
  0, 100, ILI9341_SLOUT,

  // 0x29 -> Display on
  // --------------------------------------------
  0,  20, ILI9341_DISON
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
/*
  char *txt = "ERRORISIS";
  // Init ssd1306 oled
  SSD1306_Init();
  // clear screen
  SSD1306_ClearScreen();
*/  
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
  ILI9341_InitPortsWithRES();

  // loop throuh commands
  while (no_of_commands--) {

    // number of arguments
    no_of_arguments = pgm_read_byte(commands++);
    // delay
    delay = pgm_read_byte(commands++);
    // command
    command = pgm_read_byte(commands++);
/*
    // string
    sprintf(txt, "%x-%d ", command, no_of_arguments);
    SSD1306_DrawString(txt);
*/
    // send command
    // -------------------------    
    ILI9341_TransmitCmmd(command);

    // send arguments
    // -------------------------
    while (no_of_arguments--) {
/*
      // string
      sprintf(txt, "[%x]", pgm_read_byte(commands));
      SSD1306_DrawString(txt);
*/
      // send arguments
      ILI9341_TransmitData(pgm_read_byte(commands++));
    }
    // delay
    ILI9341_Delay(delay);
  }
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
  // enable chip select -> LOW
  ILI9341_PORT_CONTROL &= ~(1 << ILI9341_PIN_CS);
  // D/C -> LOW
  ILI9341_PORT_CONTROL &= ~(1 << ILI9341_PIN_RS);

  // Write data timing diagram
  // --------------------------------------------
  //         __    __
  // WR        \__/
  //             ___
  // D0 - D7  __/   \__

  // WR -> LOW
  ILI9341_PORT_CONTROL &= ~(1 << ILI9341_PIN_WR);
  // set command on PORT
  ILI9341_PORT_DATA = cmmd;
  // WR -> HIGH
  ILI9341_PORT_CONTROL &= ~(1 << ILI9341_PIN_WR);

  // D/C -> HIGH
  ILI9341_PORT_CONTROL |= (1 << ILI9341_PIN_RS);
  // disable chip select -> HIGH
  ILI9341_PORT_CONTROL |= (1 << ILI9341_PIN_CS); 
}

/**
 * @desc    LCD transmit
 *
 * @param   char
 *
 * @return  void
 */
void ILI9341_TransmitData (char data)
{
  // enable chip select -> LOW
  ILI9341_PORT_CONTROL &= ~(1 << ILI9341_PIN_CS);
  // D/C -> HIGH
  ILI9341_PORT_CONTROL |= (1 << ILI9341_PIN_RS);

  // Write data timing diagram
  // --------------------------------------------
  //         __    __
  // WR        \__/
  //             ___
  // D0 - D7  __/   \__

  // WR -> LOW
  ILI9341_PORT_CONTROL &= ~(1 << ILI9341_PIN_WR);
  // set data on PORT
  ILI9341_PORT_DATA = data;
  // WR -> HIGH
  ILI9341_PORT_CONTROL &= ~(1 << ILI9341_PIN_WR);

  // disable chip select -> HIGH
  ILI9341_PORT_CONTROL |= (1 << ILI9341_PIN_CS);
}

/**
 * @desc    LCD init PORTs
 *
 * @param   void
 *
 * @return  void
 */
void ILI9341_InitPortsWithRES (void)
{
  // set control pins as output
  ILI9341_DDR_CONTROL = ILI9341_DDR_CONTROL |
    (1 << ILI9341_PIN_RST) | 
    (1 << ILI9341_PIN_WR)  | 
    (1 << ILI9341_PIN_RS)  | 
    (1 << ILI9341_PIN_RD)  | 
    (1 << ILI9341_PIN_WR);

  // delay > 10us
  _delay_us(100); 
  // set Reset HIGH
  ILI9341_PORT_CONTROL |= (1 << ILI9341_PIN_RST);

  // set HIGH Level on all pins
  ILI9341_PORT_CONTROL |= ILI9341_PORT_CONTROL |
    (1 << ILI9341_PIN_WR)  | 
    (1 << ILI9341_PIN_RS)  | 
    (1 << ILI9341_PIN_RD)  | 
    (1 << ILI9341_PIN_WR);

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
