/** 
 * ---------------------------------------------------------------+ 
 * @desc        ILI9341 LCD Driver 
 * ---------------------------------------------------------------+ 
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @datum       10.12.2020
 * @file        ili9341.c
 * @tested      AVR Atmega16
 *
 * @depend      spi, font
 * ---------------------------------------------------------------+
 */

#include <avr/pgmspace.h>
#include <util/delay.h>
#include "font.h"
#include "ili9341.h"

/** @array Init command */
const uint8_t INIT_ILI9341[] PROGMEM = {
  // number of initializers
  2,
  // ---------------------------------------
  // 0x01 -> Software reset
  0, ILI9341_SWRST,
  // 0x29 -> Display on
  0, ILI9341_DISON
  // ---------------------------------------
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
  // argument
  char no_of_arguments;
  // command
  char command;

  // loop throuh commands
  while (no_of_commands) {

    // number of arguments
    no_of_arguments = pgm_read_byte(commands++);
    // command
    command = pgm_read_byte(commands++);

    // send command
    // -------------------------    
    ILI9341_TransmitCmmd(command);

    // send arguments
    // -------------------------
    while (no_of_arguments--) {
      // send command
    }
    // decrement
    no_of_commands--;
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
  // set data onn PORT
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
 * @param   char
 *
 * @return  void
 */
void ILI9341_TransmitData (char data)
{
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
  ILI9341_DDR_CONTROL = ILI9341_DDR_CONTROL |
    (1 << ILI9341_PIN_RST) | 
    (1 << ILI9341_PIN_WR)  | 
    (1 << ILI9341_PIN_RS)  | 
    (1 << ILI9341_PIN_RD)  | 
    (1 << ILI9341_PIN_WR);
  // set HIGH Level on all pins
  ILI9341_PORT_CONTROL = ILI9341_PORT_CONTROL |
    (1 << ILI9341_PIN_RST) | 
    (1 << ILI9341_PIN_WR)  | 
    (1 << ILI9341_PIN_RS)  | 
    (1 << ILI9341_PIN_RD)  | 
    (1 << ILI9341_PIN_WR);
  // set all pins as output
  ILI9341_DDR_DATA = 0xFF;
}
