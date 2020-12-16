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

  for (int i=20; i<50; i++) {
    // 
    ILI9341_DrawPixel(10, i, 0xffff);
  }

  for (int i=20; i<50; i++) {
    // 
    ILI9341_DrawPixel(i, 20, 0x0000);
  }

  for (int i=20; i<50; i++) {
    // 
    ILI9341_DrawPixel(i, i, 0x0ff0);
  }


  // Update screen
  ILI9341_UpdateScreen();

  // EXIT
  // ------------------------------------------------- 
  // return & exit
  return 0;
}




