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
 * @version     1.0
 * @tested      AVR Atmega328p
 *
 * @depend      font
 * -----------------------------------------------------------------------+
 * @interface   8080-I Series Parallel Interface
 * @pins        5V, 3.3V -> NC, GND, RST, CS, RS, WR, RD, D[7:0] 
 *
 */

#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include "font.h"
#include "ili9341.h"

/** @array Init command */
const uint8_t INIT_ILI9341[] PROGMEM = {
  // number of initializers
  12,

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
  1,   0, ILI9341_MADCTL, 0x48,                                 // 0x36 -> Memory Access Control
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
  0, 200, ILI9341_DISPON                                        // 0x29 -> Display on
};

/** @var array Chache memory char index row */
unsigned short int _ili9341_cache_index_row = 0;
/** @var array Chache memory char index column */
unsigned short int _ili9341_cache_index_col = 0;

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
 * @desc    LCD Hardware Reset
 *
 * @param   void
 *
 * @return  void
 */
void ILI9341_HWReset (void)
{
  // set RESET as Output
  SETBIT(ILI9341_DDR_CONTROL, ILI9341_PIN_RST);

  // RESET SEQUENCE
  // --------------------------------------------
  // set Reset LOW
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RST);
  // delay LOW > 10us
  _delay_ms(10);
  // set Reset HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RST);
/*
  // Adafruit 
  // --------------------------------------------
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
*/
  // delay HIGH > 120ms
  _delay_ms(200);  
}

/**
 * @desc    LCD Transmit Command
 *
 * @param   uint8_t
 *
 * @return  void
 */
void ILI9341_TransmitCmmd (uint8_t cmmd)
{
  // D/C -> LOW
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RS);
  // enable chip select -> LOW
  CLRBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);

  // Write data / command timing diagram
  // --------------------------------------------
  //              ___
  // D0 - D7:  __/   \__
  //          __    __
  //      WR:   \__/

  // set command on PORT
  ILI9341_PORT_DATA = cmmd;
  // Write impulse
  WR_IMPULSE();

  // D/C -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_RS);
  // disable chip select -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS); 
}

/**
 * @desc    LCD transmit 8 bits data
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

  // Write data / command timing diagram
  // --------------------------------------------
  //              ___
  // D0 - D7:  __/   \__
  //          __    __
  //      WR:   \__/

  // set data on PORT
  ILI9341_PORT_DATA = data;
  // Write impulse
  WR_IMPULSE();

  // disable chip select -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);
}

/**
 * @desc    LCD transmit 16 bits data
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

  // set byte data on PORT
  //   __
  // 0x0000
  ILI9341_PORT_DATA = (uint8_t) (data >> 8);
  // Write impulse
  WR_IMPULSE();

  // set byte data on PORT
  //     __
  // 0x0000
  ILI9341_PORT_DATA = (uint8_t) data;
  // Write impulse
  WR_IMPULSE();

  // disable chip select -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);
}

/**
 * @desc    LCD transmit 32 bits data
 *
 * @param   uint16_t
 *
 * @return  void
 */
void ILI9341_Transmit32bitData (uint32_t data)
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

  // set byte data on PORT
  //   __
  // 0x00000000
  ILI9341_PORT_DATA = (uint8_t) (data >> 24);
  // Write impulse
  WR_IMPULSE();

  // set byte data on PORT
  //     __
  // 0x00000000
  ILI9341_PORT_DATA = (uint8_t) (data >> 16);
  // Write impulse
  WR_IMPULSE();

  // set byte data on PORT
  //       __
  // 0x00000000
  ILI9341_PORT_DATA = (uint8_t) (data >> 8);
  // Write impulse
  WR_IMPULSE();

  // set byte data on PORT
  //         __
  // 0x00000000
  ILI9341_PORT_DATA = (uint8_t) data;
  // Write impulse
  WR_IMPULSE();

  // disable chip select -> HIGH
  SETBIT(ILI9341_PORT_CONTROL, ILI9341_PIN_CS);
}

/**
 * @desc    LCD Set address window
 *
 * @param   uint16_t
 * @param   uint16_t
 * @param   uint16_t
 * @param   uint16_t
 *
 * @return  char
 */
char ILI9341_SetWindow (uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)
{
  // check if coordinates is out of range
  if ((xs > xe) || (xe > ILI9341_SIZE_X) ||
      (ys > ye) || (ye > ILI9341_SIZE_Y)) 
  { 
    // out of range
    return ILI9341_ERROR;
  }  

  // set column
  ILI9341_TransmitCmmd(ILI9341_CASET);
  // set column -> set column
  ILI9341_Transmit32bitData(((uint32_t) xs << 16) | xe);
  // set page
  ILI9341_TransmitCmmd(ILI9341_PASET);
  // set page -> high byte first
  ILI9341_Transmit32bitData(((uint32_t) ys << 16) | ye);
  // success
  return ILI9341_SUCCESS;
}

/**
 * @desc    LCD Draw Pixel
 *
 * @param   uint16_t
 * @param   uint16_t
 * @param   uint16_t
 *
 * @return  char
 */
char ILI9341_DrawPixel (uint16_t x, uint16_t y, uint16_t color)
{
  // check dimension
  if ((x > ILI9341_SIZE_X) || (y > ILI9341_SIZE_Y)) {
    // error
    return ILI9341_ERROR;
  }
  // set window
  ILI9341_SetWindow(x, y, x, y);
  // draw pixel by 565 mode
  ILI9341_SendColor565(color, 1);
  // success
  return ILI9341_SUCCESS;
}

/**
 * @desc    LCD Write Color Pixels
 *
 * @param   uint16_t
 * @param   uint32_t
 *
 * @return  void
 */
void ILI9341_SendColor565 (uint16_t color, uint32_t count)
{
  // access to RAM
  ILI9341_TransmitCmmd(ILI9341_RAMWR);
  // counter
  while (count--) {
    // write color - first colors byte
    ILI9341_Transmit16bitData(color);
  }
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
  ILI9341_SetWindow(0, 0, ILI9341_SIZE_X, ILI9341_SIZE_Y);
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
 * @desc    Draw line by Bresenham algoritm
 * @source  https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 *  
 * @param   uint16_t - x start position / 0 <= cols <= ILI9341_SIZE_X
 * @param   uint16_t - x end position   / 0 <= cols <= ILI9341_SIZE_X
 * @param   uint16_t - y start position / 0 <= rows <= ILI9341_SIZE_Y 
 * @param   uint16_t - y end position   / 0 <= rows <= ILI9341_SIZE_Y
 * @param   uint16_t - color
 *
 * @return  void
 */
void ILI9341_DrawLine(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2, uint16_t color)
{
  // determinant
  int16_t D;
  // deltas
  int16_t delta_x, delta_y;
  // steps
  int16_t trace_x = 1, trace_y = 1;

  // delta x
  delta_x = x2 - x1;
  // delta y
  delta_y = y2 - y1;

  // check if x2 > x1
  if (delta_x < 0) {
    // negate delta x
    delta_x = -delta_x;
    // negate step x
    trace_x = -trace_x;
  }

  // check if y2 > y1
  if (delta_y < 0) {
    // negate detla y
    delta_y = -delta_y;
    // negate step y
    trace_y = -trace_y;
  }

  // Bresenham condition for m < 1 (dy < dx)
  if (delta_y < delta_x) {
    // calculate determinant
    D = (delta_y << 1) - delta_x;
    // draw first pixel
    ILI9341_DrawPixel(x1, y1, color);
    // check if x1 equal x2
    while (x1 != x2) {
      // update x1
      x1 += trace_x;
      // check if determinant is positive
      if (D >= 0) {
        // update y1
        y1 += trace_y;
        // update determinant
        D -= 2*delta_x;    
      }
      // update deteminant
      D += 2*delta_y;
      // draw next pixel
      ILI9341_DrawPixel(x1, y1, color);
    }
  // for m > 1 (dy > dx)    
  } else {
    // calculate determinant
    D = delta_y - (delta_x << 1);
    // draw first pixel
    ILI9341_DrawPixel(x1, y1, color);
    // check if y2 equal y1
    while (y1 != y2) {
      // update y1
      y1 += trace_y;
      // check if determinant is positive
      if (D <= 0) {
        // update y1
        x1 += trace_x;
        // update determinant
        D += 2*delta_y;    
      }
      // update deteminant
      D -= 2*delta_x;
      // draw next pixel
      ILI9341_DrawPixel(x1, y1, color);
    }
  }
}


/**
 * @desc    LCD Fast draw line horizontal - depend on MADCTL
 *
 * @param   uint16_t - xs start position
 * @param   uint16_t - xe end position
 * @param   uint16_t - y position
 * @param   uint16_t - color
 *
 * @return  char
 */
char ILI9341_DrawLineHorizontal (uint16_t xs, uint16_t xe, uint16_t y, uint16_t color)
{
  // temp variable
  uint16_t temp;
  // check if out of range
  if ((xs > ILI9341_SIZE_X) || (xe > ILI9341_SIZE_X) || (y > ILI9341_SIZE_Y)) {
    // error
    return ILI9341_ERROR;
  }
  // check if start is > as end  
  if (xs > xe) {
    // temporary safe
    temp = xe;
    // start change for end
    xe = xs;
    // end change for start
    xs = temp;
  }
  // set window
  ILI9341_SetWindow(xs, y, xe, y);
  // draw pixel by 565 mode
  ILI9341_SendColor565(color, xe - xs);
  // success
  return ILI9341_SUCCESS;
}

/**
 * @desc    LCD Fast draw line vertical - depend on MADCTL
 *
 * @param   uint16_t - x position
 * @param   uint16_t - ys start position
 * @param   uint16_t - ye end position
 * @param   uint16_t - color
 *
 * @return  char
 */
char ILI9341_DrawLineVertical (uint16_t x, uint16_t ys, uint16_t ye, uint16_t color)
{
  // temp variable
  uint16_t temp;
  // check if out of range
  if ((ys > ILI9341_SIZE_Y) || (ye > ILI9341_SIZE_Y) || (x > ILI9341_SIZE_X)) {
    // error
    return ILI9341_ERROR;
  }  
  // check if start is > as end
  if (ys > ye) {
    // temporary safe
    temp = ye;
    // start change for end
    ye = ys;
    // end change for start
    ys = temp;
  }
  // set window
  ILI9341_SetWindow(x, ys, x, ye);
  // draw pixel by 565 mode
  ILI9341_SendColor565(color, ye - ys);
  // success
  return ILI9341_SUCCESS;
}

/**
 * @desc    Draw character 2x larger
 *
 * @param   char -> character
 * @param   uint16_t -> color
 * @param   ILI9341_Sizes -> size
 *
 * @return  void
 */
char ILI9341_DrawChar (char character, uint16_t color, ILI9341_Sizes size)
{
  // variables
  uint8_t letter, idxCol, idxRow;
  // check if character is out of range
  if ((character < 0x20) &&
      (character > 0x7f)) { 
    // out of range
    return 0;
  }
  // last column of character array - 5 columns 
  idxCol = CHARS_COLS_LENGTH;
  // last row of character array - 8 rows / bits
  idxRow = CHARS_ROWS_LENGTH;

  // --------------------------------------
  // SIZE X1 - normal font 1x high, 1x wide
  // --------------------------------------
  if (size == X1) {  
    // loop through 5 bits
    while (idxCol--) {
      // read from ROM memory 
      letter = pgm_read_byte(&FONTS[character - 32][idxCol]);
      // loop through 8 bits
      while (idxRow--) {
        // check if bit set
        if (letter & (1 << idxRow)) {
          // draw pixel 
          ILI9341_DrawPixel(_ili9341_cache_index_col + idxCol, _ili9341_cache_index_row + idxRow, color);
        }
      }
      // fill index row again
      idxRow = CHARS_ROWS_LENGTH;
    }
    // update x position
    _ili9341_cache_index_col += CHARS_COLS_LENGTH + 1;
  
  // --------------------------------------
  // SIZE X2 - font 2x higher, normal wide
  // --------------------------------------
  } else if (size == X2) {
    // loop through 5 bytes
    while (idxCol--) {
      // read from ROM memory 
      letter = pgm_read_byte(&FONTS[character - 32][idxCol]);
      // loop through 8 bits
      while (idxRow--) {
        // check if bit set
        if (letter & (1 << idxRow)) {
          // draw first left up pixel; 
          // (idxRow << 1) - 2x multiplied 
          ILI9341_DrawPixel(_ili9341_cache_index_col + idxCol, _ili9341_cache_index_row + (idxRow << 1), color);
          // draw second left down pixel
          ILI9341_DrawPixel(_ili9341_cache_index_col + idxCol, _ili9341_cache_index_row + (idxRow << 1) + 1, color);
        }
      }
      // fill index row again
      idxRow = CHARS_ROWS_LENGTH;
    }
    // update x position
    _ili9341_cache_index_col += CHARS_COLS_LENGTH + 2;

  // --------------------------------------
  // SIZE X3 - font 2x higher, 2x wider
  // --------------------------------------
  } else if (size == X3) {
    // loop through 5 bytes
    while (idxCol--) {
      // read from ROM memory 
      letter = pgm_read_byte(&FONTS[character - 32][idxCol]);
      // loop through 8 bits
      while (idxRow--) {
        // check if bit set
        if (letter & (1 << idxRow)) {
          // draw first left up pixel; 
          // (idxRow << 1) - 2x multiplied 
          ILI9341_DrawPixel(_ili9341_cache_index_col + (idxCol << 1), _ili9341_cache_index_row + (idxRow << 1), color);
          // draw second left down pixel
          ILI9341_DrawPixel(_ili9341_cache_index_col + (idxCol << 1), _ili9341_cache_index_row + (idxRow << 1) + 1, color);
          // draw third right up pixel
          ILI9341_DrawPixel(_ili9341_cache_index_col + (idxCol << 1) + 1, _ili9341_cache_index_row + (idxRow << 1), color);
          // draw fourth right down pixel
          ILI9341_DrawPixel(_ili9341_cache_index_col + (idxCol << 1) + 1, _ili9341_cache_index_row + (idxRow << 1) + 1, color);
        }
      }
      // fill index row again
      idxRow = CHARS_ROWS_LENGTH;
    }
    // update x position *2
    _ili9341_cache_index_col += (CHARS_COLS_LENGTH << 1) + 2;
  }
  // return exit
  return ILI9341_SUCCESS;
}

/**
 * @desc    Draw string
 *
 * @param   char* -> string 
 * @param   uint16_t -> color
 * @param   ILI9341_Sizes -> size
 *
 * @return  void
 */
void ILI9341_DrawString (char *str, uint16_t color, ILI9341_Sizes size)
{
  // variables
  unsigned int i = 0;
  char check;
  uint16_t delta_y;
  uint16_t max_y_pos;
  uint16_t new_x_pos;
  uint16_t new_y_pos;

  // loop through character of string
  while (str[i] != '\0') {
    // max x position character
    new_x_pos = _ili9341_cache_index_col + CHARS_COLS_LENGTH + (size & 0x0F);
    // delta y
    delta_y = CHARS_ROWS_LENGTH + (size >> 4);
    // max y position character
    new_y_pos = _ili9341_cache_index_row + delta_y;
    // max y pos
    max_y_pos = ILI9341_SIZE_Y - delta_y;
    // control if will be in range
    check = ILI9341_CheckPosition(new_x_pos, new_y_pos, max_y_pos, size);
    // update position
    if (ILI9341_SUCCESS == check) {
      // read characters and increment index
      ILI9341_DrawChar(str[i++], color, size);
    }
  }
}

/**
 * @desc    Check text position x, y
 *
 * @param   uint16_t
 * @param   uint16_t
 * @param   uint16_t
 * @param   ILI9341_Sizes
 *
 * @return  char
 */
char ILI9341_CheckPosition (uint16_t x, uint16_t y, uint16_t max_y, ILI9341_Sizes size)
{
  // check if coordinates is out of range
  if ((x > ILI9341_SIZE_X) && (y > max_y)) {  
    // error
    return ILI9341_ERROR;
  }
  // if next line
  if ((x > ILI9341_SIZE_X) && (y <= max_y)) {
    // set position y
    _ili9341_cache_index_row = y;
    // set position x
    _ili9341_cache_index_col = 2;
  } 
  // return exit
  return ILI9341_SUCCESS;
}

/**
 * @desc    LCD Set text position x, y
 *
 * @param   uint16_t x - position
 * @param   uint16_t y - position
 *
 * @return  char
 */
char ILI9341_SetPosition (uint16_t x, uint16_t y)
{
  // check if coordinates is out of range
  if ((x > ILI9341_SIZE_X) && (y > ILI9341_SIZE_Y)) {
    // error
    return ILI9341_ERROR;
  // x overflow, y in range
  } else if ((x > ILI9341_SIZE_X) && (y <= ILI9341_SIZE_Y)) {
    // set position y
    _ili9341_cache_index_row = y;
    // set position x
    _ili9341_cache_index_col = 2;
  } else {
    // set position y 
    _ili9341_cache_index_row = y;
    // set position x
    _ili9341_cache_index_col = x;
  }
  // return exit
  return ILI9341_SUCCESS;
}

/**
 * @desc    Delay
 *
 * @param   uint16_t
 *
 * @return  void
 */
void ILI9341_Delay (uint16_t time)
{
  // loop through real time
  while (time--) {
    // 1 s delay
    _delay_ms(1);
  }
}
