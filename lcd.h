/* Code to drive HD44780 Display
 * via PCF8574 I2C 8bit I/O Expander
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
 */

#ifndef __LCD_H__
#define __LCD_H__

#include <stdlib.h>
#include <stdint.h>


/* instruction register bit positions, see HD44780U data sheet */
#define LCD_CLR               0      /* DB0: clear display                  */
#define LCD_HOME              1      /* DB1: return to home position        */
#define LCD_ENTRY_MODE        2      /* DB2: set entry mode                 */
#define LCD_ENTRY_INC         1      /*   DB1: 1=increment, 0=decrement     */
#define LCD_ENTRY_SHIFT       0      /*   DB2: 1=display shift on           */
#define LCD_ON                3      /* DB3: turn lcd/cursor on             */
#define LCD_ON_DISPLAY        2      /*   DB2: turn display on              */
#define LCD_ON_CURSOR         1      /*   DB1: turn cursor on               */
#define LCD_ON_BLINK          0      /*     DB0: blinking cursor ?          */
#define LCD_MOVE              4      /* DB4: move cursor/display            */
#define LCD_MOVE_DISP         3      /*   DB3: move display (0-> cursor) ?  */
#define LCD_MOVE_RIGHT        2      /*   DB2: move right (0-> left) ?      */
#define LCD_FUNCTION          5      /* DB5: function set                   */
#define LCD_FUNCTION_8BIT     4      /*   DB4: set 8BIT mode (0->4BIT mode) */
#define LCD_FUNCTION_2LINES   3      /*   DB3: two lines (0->one line)      */
#define LCD_FUNCTION_10DOTS   2      /*   DB2: 5x10 font (0->5x7 font)      */
#define LCD_CGRAM             6      /* DB6: set CG RAM address             */
#define LCD_DDRAM             7      /* DB7: set DD RAM address             */
#define LCD_BUSY              7      /* DB7: LCD is busy                    */


#define LCD_LINE_LENGTH  0x40     /**< internal line length of the display    */
#define LCD_START_LINE1  0x00     /**< DDRAM address of first char of line 1 */
#define LCD_START_LINE2  0x40     /**< DDRAM address of first char of line 2 */
#define LCD_START_LINE3  0x14     /**< DDRAM address of first char of line 3 */
#define LCD_START_LINE4  0x54     /**< DDRAM address of first char of line 4 */
#define LCD_WRAP_LINES      0     /**< 0: no wrap, 1: wrap at end of visibile line */

/* pin definitions */
#define P0 0x01
#define P1 0x02
#define P2 0x04
#define P3 0x08
#define P4 0x10
#define P5 0x20
#define P6 0x40
#define P7 0x80

/* i2c addr of expander chip */
// ID= 0x2x where x = ID jumpers installed
//     No jumpers  = 0x27
//     All Jumpers = 0x20

/* define what display-pin is connected
 * to what io-pin of expander */

#define LCD_D4 P4
#define LCD_D5 P5
#define LCD_D6 P6
#define LCD_D7 P7

#define LCD_RS P0
#define LCD_RW P1
#define LCD_E  P2
#define LCD_BL P3


/* what is the width of the display
 * (height is assumed to be 2) */
#define LCD_WIDTH 16


/* init the lcd */
uint8_t lcdInit();

/* clear the display */
void lcdClear(void);

/* print a string on display,
 * continues automatically in 2nd line,
 * note that lcdClear() should be called
 * before using this function */
void lcdPrintString(const char *str);
void lcdPrintString_P(const char *str);

/* low level function, see lcd.c */
void lcd4BitOut(unsigned char c, unsigned char data);

void lcdGotoXY(uint8_t x, uint8_t y);
void lcd_bargraph_init(void);
void lcdProgressBar(uint16_t, uint16_t, uint8_t);
void lcdOut(unsigned char c);

void lcdControl(char c);

#define  lcd_command(c)    lcd4BitOut(c, 0)
#define  lcd_data(c)       lcd4BitOut(c, 1)

#define  HD44780_CURSOR_OFF		(0x0C)
#define  HD44780_CURSOR_ON		(0x0E)
#define  HD44780_CURSOR_BLINK	(0x0F)
#endif
