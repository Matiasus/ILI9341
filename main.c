/** 
 * --------------------------------------------------------------------------------------------+ 
 * @desc        Main example ILI9341 LCD driver
 * --------------------------------------------------------------------------------------------+ 
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @datum       12.12.2020
 * @file        main.c
 * @tested      AVR Atmega16
 *
 * @depend      ili9341.h
 * --------------------------------------------------------------------------------------------+
 * @inspir      
 */
#include <util/delay.h>
#include "lib/ili9341.h"

/**
 * @desc    Main function
 *
 * @param   Void
 *
 * @return  Void
 */
int main(void)
{
  // init lcd
  ILI9341_Init();

  // clear Screen
  ILI9341_ClearScreen(ILI9341_WHITE);


  for (int i=0; i<ILI9341_MAX_X; i++) {
    // draw line
    ILI9341_DrawLine(0, i, 0, ILI9341_SIZE_Y, ILI9341_RED);
    // delay
    _delay_ms(10);
  }

  int i = ILI9341_MAX_Y;

  while (i--) {
    // draw line
    ILI9341_DrawLine(0, ILI9341_SIZE_X, i, ILI9341_SIZE_Y, 0xF0FC);
    // delay
    _delay_ms(10);
  }


  // EXIT
  // ------------------------------------------------- 
  // return & exit
  return 0;
}




