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
  ILI9341_ClearScreen(ILI9341_BLACK);

  // draw horizontal fast line
  ILI9341_DrawLineHorizontal(10, ILI9341_MAX_X - 10, 12, ILI9341_WHITE);
  // draw horizontal fast line
  ILI9341_DrawLineHorizontal(10, ILI9341_MAX_X - 10, 50, ILI9341_WHITE);

  // set position
  ILI9341_SetPosition(10, 25);  
  // draw string
  ILI9341_DrawString("ILI9341 LCD DRIVER", 0xcff0, X3);

  // set position
  ILI9341_SetPosition(2, 60);  
  // draw string
  ILI9341_DrawString("The HiFi Engine library has images, specifications and reviews for thousands of audio components, along with free downloads of owners manuals, service manuals, schematics and product catalogues for amplifiers, pre-amps, power amps, tuners, tape decks, cd players etc.", 0xffff, X2);

  // EXIT
  // ------------------------------------------------- 
  // return & exit
  return 0;
}




