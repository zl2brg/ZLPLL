/*
 * menu.c
 *
 **   Copyright (C) 2008-2022 Wanye Knowles ZL2BKC. 2023 Simon Eatough ZL2BRG
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "zlpll.h"
#include "encoder.h"
#include "lcd.h"

static uint8_t menu1;
static uint8_t menu2;

extern uint8_t channel;

void cmd_channel(int);

extern uint16_t	pwmval;
extern uint16_t	pwmtop;

extern void PWM_set(uint16_t, uint16_t);



union value {
	uint16_t	ui;
	float		f;
};
typedef  union value  value_t;

struct PACKED  entry {
	uint8_t xpos, ypos;
	uint8_t digits;

	value_t val, min, max;
	value_t mult;
	uint8_t base;
	uint8_t pos;
	uint8_t type;
#define EN_INT16	1
#define EN_HEX16	2
#define EN_FLOAT	4
	const char *fmt;
	void (*callback)(uint8_t op, struct entry *e);
} entry;

#define ENTRY_UPDATE	0
#define ENTRY_SAVE		1

#if PWM
void pwm_update(uint8_t op, struct entry *e) {
	extern void pwm_save();
	
	pwmval = e->val.ui;
	PWM_set(pwmval, pwmtop);
	if (op == ENTRY_SAVE)
		pwm_save();
}
#endif

void freq_update(uint8_t op, struct entry *e) {
	rf.rf_freq = e->val.f;
	set_freq(rf.rf_freq);
}

void entry_paint() {
	char buf[16];
	// Display Value
	if (entry.type == EN_FLOAT)
		sprintf_P(buf, entry.fmt, entry.val.f);
	else
		sprintf_P(buf, entry.fmt, entry.val.ui);
	lcdGotoXY(entry.xpos, entry.ypos);
	lcdPrintString(buf);
	lcdGotoXY(entry.xpos+entry.pos, entry.ypos);
}

void entry_start_INT(uint8_t type, uint16_t val, uint16_t top, void (*callback)(uint8_t, struct entry *)) {
	lcdClear();
	entry.type = type;
	entry.xpos = 0;
	entry.ypos = 1;
	entry.min.ui = 0;
	entry.max.ui = top;
	entry.digits = 4;
	entry.val.ui = val;
	entry.pos = 0;
	if (type == EN_HEX16) {
		entry.mult.ui = 0x1000;
		entry.fmt = PSTR("%04X");
		entry.base=16;
	} else if (type == EN_INT16) {
		entry.mult.ui = 1000;
		entry.fmt = PSTR("%04d");
		entry.base=10;
	}
	entry_paint();
	entry.callback = callback;
	lcd_command(HD44780_CURSOR_BLINK);
}

void entry_start_FLOAT(uint8_t x, uint8_t y, float val, float min, float max, 
					void (*callback)(uint8_t, struct entry *)) {
	//lcdClear();
	entry.type = EN_FLOAT;
	entry.xpos = x;
	entry.ypos = y;
	entry.min.f = min;
	entry.max.f = max;
	entry.val.f = val;
	entry.digits = 8;
	entry.pos = 0;

	entry.mult.f = 1000.0;
	entry.fmt = PSTR("%8.3f");
	entry.base = 10;

	entry_paint();
	entry.callback = callback;
	lcd_command(HD44780_CURSOR_BLINK);
}

void entry_event(uint8_t ev) {
	uint8_t  update;
	
	update=0;
	if (entry.type == EN_FLOAT)	{	// Floating point value
		switch (ev) {
		case DIR_PRESS:
			if (entry.pos >= entry.digits-1)
			goto done;
			// Fallthough
		case DIR_CW2:
			if (entry.pos < entry.digits-1) {
				entry.pos++;
				if (entry.mult.f == 1.0)
					entry.pos++;		// Skip over decimal point
				entry.mult.f /= entry.base;
				++update;
			}
			break;
		case DIR_CCW2:
			if (entry.pos > 0) {
				entry.pos--;
				entry.mult.f *= entry.base;
				if (entry.mult.f == 1.0)
					entry.pos--;		// Skip over decimal point
				++update;
			}
			break;
		case DIR_CW:
			if (entry.val.f <= entry.max.f-entry.mult.f) {
				entry.val.f += entry.mult.f;
				++update;
			}
			break;
		case DIR_CCW:
			if (entry.val.f >= entry.mult.f+entry.min.f) {
				entry.val.f -= entry.mult.f;
				++update;
			}
			break;
		case DIR_PRESS2:
			done:
			lcd_command(HD44780_CURSOR_OFF);
			menu1 = 0;
			if (entry.callback) {
				(*entry.callback)(ENTRY_SAVE, &entry);	// Fixme
			}
			break;
		}
		if (update) {
			entry_paint();		// Update!
			if (entry.callback) {
				(*entry.callback)(ENTRY_UPDATE, &entry);
			}
		}
	} else {
		switch (ev) {
			case DIR_PRESS:
				if (entry.pos >= entry.digits-1)
					goto done2; 
				// Fall-though
			case DIR_CW2:
				if (entry.pos < entry.digits-1) {
					entry.pos++;
					entry.mult.ui /= entry.base;
					++update;
				}
				break;
			case DIR_CCW2:
				if (entry.pos > 0) {
					entry.pos--;
					entry.mult.ui *= entry.base;
					++update;
				}
				break;
			case DIR_CW:
				if (entry.val.ui <= entry.max.ui-entry.mult.ui) {
					entry.val.ui += entry.mult.ui;
					++update;
				}
				break;
			case DIR_CCW:
				if (entry.val.ui >= entry.mult.ui+entry.min.ui) {
					entry.val.ui -= entry.mult.ui;
					++update;
				}
				break;
			case DIR_PRESS2:
			done2:
				lcd_command(HD44780_CURSOR_OFF);
				menu1 = 0;
				if (entry.callback) {
					(*entry.callback)(ENTRY_SAVE, &entry);
				}
				break;
		}
		if (update) {
			entry_paint();		// Update!
			if (entry.callback) {
				(*entry.callback)(ENTRY_UPDATE, &entry);
			}
		}
	}

}

void menu_init()
{
	menu1 = menu2 = 0;
}

void menu_paint() {
	char buf[25];

	lcdClear();	
	switch (menu1) {
	case 0:
			sprintf_P(buf, PSTR("%6.3f MHz"), rf.rf_freq);
			lcdPrintString(buf);
			lcdGotoXY(0,1);
			sprintf_P(buf, PSTR("Level: %d"), rf.level);
			lcdPrintString(buf);
			break;
	case 1:	
			lcdPrintString_P(PSTR("Frequency >"));
			lcdGotoXY(0,1);
			sprintf_P(buf, PSTR("%6.3f MHz"), rf.rf_freq);
			lcdPrintString(buf);
			break;
	case 2: 			lcdPrintString_P(PSTR("Level >"));
			break;
	case 3: 			lcdPrintString_P(PSTR("Multiplier >"));
			break;
	case 4: 			lcdPrintString_P(PSTR("Save >"));
			break;
	case 5: 			lcdPrintString_P(PSTR("Recall >"));
			break;	
	}
	lcdGotoXY(0,1);
}

void  toplevel() {
	char buf[25];
		
	lcd_command(HD44780_CURSOR_OFF);		// Turn off cursor
	lcdClear();
	sprintf_P(buf, PSTR("%2d: %6.3f MHz"), channel, rf.rf_freq);
	lcdPrintString(buf);
	lcdGotoXY(0,1);
	sprintf_P(buf, PSTR("Level: %d"), rf.level);
	lcdPrintString(buf);
}


void menu_top(uint8_t key) {

	uint8_t   update = 0;

	switch (key) {
	case DIR_CW:	if (channel < 15) {
						channel++;	update++;
					}			
					break;
	case DIR_CCW:	if (channel > 0) {
						channel--;	update++;
					}
					break;
	case DIR_CW2:	 break;
	case DIR_CCW2:	 break;
	case DIR_PRESS2: break;
	
	case DIR_PRESS:	menu1 = 2;
					//entry_start_INT(EN_INT16, pwmval, pwmtop, pwm_update);
					entry_start_FLOAT(4, 0, rf.rf_freq, 31.5, 4400, freq_update);
					break;
	}
	if (update) {
		cmd_channel(channel);		// Load default frequencies etc.
		set_freq(rf.rf_freq);
		toplevel();
	}
}


void menu_event(uint8_t key) {
	if (menu1 == 0) {
		menu_top(key);
	} else {
		entry_event(key);
	}
}

	
#if 0
	switch (key) {
		case DIR_CW:	if (menu1 < 5)
		menu1++;
		break;
		
		case DIR_CCW:	if (menu1)
		menu1--;
		break;

		case DIR_CW2:	 putchar('>');  break;
		case DIR_CCW2:	 putchar('<');  break;
		case DIR_PRESS2:	putchar('!'); break;
		
		case DIR_PRESS:
		lcd_command(HD44780_CURSOR_ON);
		break;
	}
	menu_paint();
#endif

