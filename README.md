# C Library for ILI9341 2.8 TFT LCD Display

## ILI9341 Description
Detailed information are described in [Datasheet ILI9341](https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf).

## Library
C library is aimed for driving [ILI9341 2.8 TFT LCD display](#demonstration) 240x320 using 8080-I Series Parallel Interface.

## Hardware connection
| PORT LCD | PORT ATMEGA328P | Description |
| :---: | :---: |  :---: |
| 5V | 5V | Supply Voltage |
| 3.3V | NC | Not Connected |
| GND | GND | Ground |
| RST | PORTC 4 | Reset |
| CS | PORTC 3 | Chip Select |
| RS | PORTC 2 | Register Select (Data / Command) |
| WR | PORTC 1 | Write |
| RD | PORTC 0 | Read |
| D0 | PORTD 0 | Data bit 0 |
| D1 | PORTD 1 | Data bit 1 |
| D2 | PORTD 2 | Data bit 2 |
| D3 | PORTD 3 | Data bit 3 |
| D4 | PORTD 4 | Data bit 4 |
| D5 | PORTD 5 | Data bit 5 |
| D6 | PORTD 6 | Data bit 6 |
| D7 | PORTD 7 | Data bit 7 |

### Usage
Prior defined for MCU Atmega328p / Atmega8 / Atmega16. Need to be carefull with pins definition. **_Data pins D[7:0] using one port and are in order!_**.

### Tested
Library was tested and proved on a **_ILI9341 2.8″ TFT Dispay_** with **_Atmega328p_**.
  
## Demonstration
<img src="img/img.jpg" />

## Links
- [Datasheet ILI9341](https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf)

## Acknowledgement
- [Adafuit TFT](https://github.com/adafruit/TFTLCD-Library)
- [notro](https://github.com/notro/fbtft/blob/master/fb_ili9341.c)
- [thefallenidealist](https://github.com/thefallenidealist/ili9341/blob/master/glcd.c)
- [fagcinsk](https://github.com/fagcinsk/stm-ILI9341-spi/blob/master/lib/ILI9341/commands.h)
