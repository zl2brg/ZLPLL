/* Code to drive HD44780 Display
 * via PCF8574 I2C 8bit I/O Expander
 *
 * Version 1.0
 *
 * (c) Michael "ScriptKiller" Arndt
 * http://scriptkiller.de/
 * <scriptkiller@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *
 */




/* configuration is done in lcd.h */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#include "lcd.h"
#include "i2c.h"


extern uint8_t I2CErrors;

extern void I2CSendStart();
extern void I2CSendStop();
extern void I2CSendByte(uint8_t);
extern void I2CInit();

static int lcd_addr;

void lcd_usleep(unsigned int msec) {
    /* 16-bit count, 4 cycles/loop */
    _delay_loop_2(8000*msec);
}

/* output (raw) data to lcd */
void lcdOut(unsigned char c) {
    I2CSendStart();
    I2CSendByte(lcd_addr<<1);
    I2CSendByte(c);
    I2CSendStop();
}


/* output data in 4bit mode to lcd
 * if data != 0, register select is set high!
 */
void lcdWrite(unsigned char c, unsigned char data) {

  unsigned char out;
  unsigned char rs;

  rs=data?LCD_RS:0; // Set RS bit if data set

  /* 4 upper bits */
#if LCD_D4 == P4
  out = c & 0xf0;
#else
  out = (c & 0xf0)>>4;
#endif
 
  I2CSendByte(out | rs | LCD_BL);
  I2CSendByte(out | rs | LCD_E | LCD_BL);
  I2CSendByte(out | rs | LCD_BL);

  /* 4 lower bits */
#if LCD_D4 == P4
  out = (c & 0x0f) << 4;
#else
  out = (c & 0x0f);
#endif
  //I2CSendByte(out | rs | LCD_BL);
  I2CSendByte(out | rs | LCD_E | LCD_BL);
  I2CSendByte(out | rs | LCD_BL);
}

void lcd4BitOut(unsigned char c, unsigned char data) {
  I2CSendStart();
  I2CSendByte(lcd_addr<<1);
  lcdWrite(c, data);
  I2CSendStop();
}

/* clear LCD */
void lcdClear() {
  lcd4BitOut(0x01, 0); // clear display
}

/* init lcd */
uint8_t lcdInit() {
	uint8_t  addr;
	
	#define LCD_I2C_LOW		0x20
	#define LCD_I2C_HIGH	0x27
  
  	I2CInit();
	lcd_addr = 0;
	for (addr=LCD_I2C_LOW; addr<=LCD_I2C_HIGH; addr++) {
		if (I2CPoll(addr<<1)) {
			lcd_addr = addr;	// Use highest LCD address
		}
	}
	
	if (!lcd_addr)
		return 0;

	_delay_ms(50);   // Allow I2C signals to reach steady state voltage
    
    I2CSendStart();
    I2CSendByte(lcd_addr<<1);
    
    I2CSendByte(0x00);
    // A1
    I2CSendByte(LCD_BL | LCD_D4 | LCD_D5);
    lcd_usleep(10);
    I2CSendByte(LCD_BL |LCD_D4 | LCD_D5 | LCD_E);
    lcd_usleep(10);
    I2CSendByte(LCD_BL |LCD_D4 | LCD_D5);
    lcd_usleep(5);

    // A2
    I2CSendByte(LCD_BL |LCD_D4 | LCD_D5);
    lcd_usleep(10);
    I2CSendByte(LCD_BL |LCD_D4 | LCD_D5 | LCD_E);
    lcd_usleep(10);
    I2CSendByte(LCD_BL |LCD_D4 | LCD_D5);
    lcd_usleep(2);

    // A3
    I2CSendByte(LCD_BL |LCD_D4 | LCD_D5);
    lcd_usleep(10);
    I2CSendByte(LCD_BL |LCD_D4 | LCD_D5 | LCD_E);
    lcd_usleep(10);
    I2CSendByte(LCD_BL |LCD_D4 | LCD_D5);
    lcd_usleep(10);

    // A4
    /* set mode to 4 bit */
    I2CSendByte(LCD_D5);
    lcd_usleep(10);
    I2CSendByte(LCD_D5 | LCD_E);
    lcd_usleep(10);
    I2CSendByte(LCD_D5);
    lcd_usleep(10);
    I2CSendStop();

    /* from now on display can be interfaced
     * using 4bit mode */
    lcd4BitOut(0x28, 0); // A5
    lcd4BitOut(0x08, 0); // A6
    lcd4BitOut(0x01, 0); // A7
    lcd4BitOut(0x07, 0); // A8
    lcd4BitOut(0x0C, 0); // display + cursor off + blink off
    lcd4BitOut(0x06, 0); // increment addr

    lcdClear();
	return lcd_addr;	// Non zero to confirm LCD is present
}


void lcdPrintString(const char *str) {
  unsigned char c;

  I2CSendStart();
  I2CSendByte(lcd_addr<<1);

  while( (c=*str++) != (char)0 ) {
    lcdWrite(c, 1);
  }
  I2CSendStop();
}

void lcdPrintString_P(const char *str) {
  unsigned char c;

  I2CSendStart();
  I2CSendByte(lcd_addr<<1);
  while( (c=pgm_read_byte(str++)) ) {
    lcdWrite(c, 1);
  }
  I2CSendStop();
}


void lcdGotoXY(uint8_t x, uint8_t y)
{
    if ( y==0 )
        lcd4BitOut((1<<LCD_DDRAM)+LCD_START_LINE1+x, 0);
    else if ( y==1)
        lcd4BitOut((1<<LCD_DDRAM)+LCD_START_LINE2+x, 0);
    else if ( y==2)
        lcd4BitOut((1<<LCD_DDRAM)+LCD_START_LINE3+x, 0);
    else /* y==3 */
        lcd4BitOut((1<<LCD_DDRAM)+LCD_START_LINE4+x, 0);
}

