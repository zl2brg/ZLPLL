/*

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

/*  Bits that are set inside interrupt routines, and watched outside in
 *  the program's main loop.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 

#include "usart.h"
#include <stdio.h>


struct FLAGS flags;


void USART_init( unsigned int ubrr )
{
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;

    /*
     * Initialize the USART for 9600 baud operation
     */
    UCSR0B = _BV(TXEN0)|_BV(RXEN0); /* tx/rx enable, rx complete intr */
    UCSR0C = (3 << UCSZ00);    //asynchronous 8 N 1

    flags.rx_int = 0;
    flags.rx_echo = 1;
    flags.cmd_int = 0;
}


// Check status of the receiver
//
// Return
//    0 = Receive buffer empty
//    1 = Receive buffer has character available
// 
uint8_t USART_rxready() {
    return (UCSR0A & _BV(RXC0)) != 0;
}

int USART_getchar(FILE *FP) {
    while (!(UCSR0A & _BV(RXC0))) 
	wdt_reset(); // Wait for RX Ready
    return UDR0;
}


// Writes a character to the serial port.
int USART_putchar( const char c, FILE *fp)
{
    while (!(UCSR0A & (1 <<UDRE0)))
		{ /* NOP */ }
    UDR0 = c;
    if (c == '\n') { // Newline -> CRLF
	while (!(UCSR0A & (1 <<UDRE0)))
	    { /* NOP */ }
	UDR0 = '\r';
    }
    return c;
}


// Write a NULL-terminated string from RAM to the serial port
void USART_puts( const char *s )
{
    while ( *s ) {
	USART_putchar(*s++, NULL);
    }
}

// Write a NULL-terminated string from program-space (flash) 
// to the serial port. example: USART_puts_p(PSTR("test"))
void USART_puts_p( const char *prg_s )
{
    char c;

    while ( ( c = pgm_read_byte( prg_s++ ) ) ) {
		USART_putchar(c, NULL);
    }
}

// Helper-Macro - "automatically" inserts PSTR
// when used: include avr/pgmspace.h before this include-file
#define USART_puts_P(s___) USART_puts_p(PSTR(s___))
