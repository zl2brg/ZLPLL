/*
 * dial.c
 *
 *
 *   Copyright (C) 2008-2022 Wanye Knowles ZL2BKC. 2023 Simon Eatough ZL2BRG
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

/*
 * Based on work from https://github.com/buxtronix/arduino/blob/master/libraries/Rotary/Rotary.cpp
 *
 *
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "zlpll.h"
#include "encoder.h"

static uint8_t dial_state;
static uint16_t button_stream;
static uint8_t button_save;

#define R_START 0x0
// Use the half-step state table (emits a code at 00 and 11)
#if 0
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5
const unsigned char ttable[6][4] = {
	// R_START (00)
	{R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
	// R_CCW_BEGIN
	{R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
	// R_CW_BEGIN
	{R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
	// R_START_M (11)
	{R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
	// R_CW_BEGIN_M
	{R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
	// R_CCW_BEGIN_M
	{R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};

#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

const unsigned char ttable[7][4] = {
	// R_START
	{R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
	// R_CW_FINAL
	{R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
	// R_CW_BEGIN
	{R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
	// R_CW_NEXT
	{R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
	// R_CCW_BEGIN
	{R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
	// R_CCW_FINAL
	{R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
	// R_CCW_NEXT
	{R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

void dial_init() {
	// Setup input pins for rotary encoder dial
	//
	// input with pullup
	//
	INPUT(DIAL_SW);		SET(DIAL_SW);
	INPUT(DIAL_PH1);	SET(DIAL_PH1);
	INPUT(DIAL_PH2);	SET(DIAL_PH2);
	
	dial_state = R_START;
	button_save = 0;		// Initial state
	button_stream = 0;
}

uint8_t dial_read() {
	uint8_t button, result;
	static uint16_t timer = 0;
	
	button_stream = (button_stream<<1) | (ISSET(DIAL_SW)?0:1);
	
	button = (button_stream == 0xffff);
	
	unsigned char pinstate = (PINC >>1)&0x03;
	// Determine new state from the pins and state table.
	dial_state = ttable[dial_state & 0xf][pinstate];
	
	// Return emit bits, ie the generated event.
	
	result = (dial_state & 0x30);

	if (button != button_save) {		// New button press or release detected
		button_save = button;
		_delay_ms(4);		// wait for button to settle
		// Change of button state detected
		if (button)			// Button pressed 
			timer = 1;		// Start timer to determine press duration
		else {  // Button release - send button events on release
				// based on length of time pressed
			if (timer > 50000U)		// Long Press
				result = DIR_PRESS2;
			else if (timer > 200)
				result = DIR_PRESS;	// Short Press
			timer = 0;		// Press complete - timer no longer required
		}
	} else 
		if (button) {
			if (result == DIR_CW || result == DIR_CCW)  {
				result |= DIR_PRESS;
				timer = 0;		// Disable button press timer so PRESS is not returned
				// when released after knob is turned.
			}
			if (timer > 0 && timer < 65000U)
				timer++;
		}

	return result;
}


