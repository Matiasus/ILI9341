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
  ILI9341_ClearScreen(ILI9341_RED);

  // Update screen
  ILI9341_UpdateScreen();

  // EXIT
  // ------------------------------------------------- 
  // return & exit
  return 0;
}




