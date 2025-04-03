/*
 * zlpll.h
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

#ifndef ZLPLL_H_
#define ZLPLL_H_


#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>

#define PACKED	__attribute__((__packed__))


#define ZLPLL_MAGIC		0x4002

#define ADF4351_DATA	PORTB,PB3		// aka MOSI in SPI mode
#define ADF4351_CLK		PORTB,PB5		// aka SCK
#define ADF4351_LE		PORTB,PB2		// aka SS

#define PIN_CHANSW	7
#define PIN_REFSEL	PINC2
#define PIN_AUXLED	PINB7

#define	AUXLED		PORTB,PB7
//#define LOCKLED		PORTD,PD5		// 4.0
#define LOCKLED		PORTB,PB6
#define LED1		PORTD,PD6
#define LED2		PORTD,PD7
#define PIN_TXD		PORTD,PD0
#define LOCKIN		PORTD,PD4
#define MUXOUTPIN	PORTD,PD5
#define REFSEL		PORTD,PD2
#define OSCPOWER	PORTD,PD3

#define DIAL_SW		PORTC,PC0
#define DIAL_PH1	PORTC,PC1
#define DIAL_PH2	PORTC,PC2

#define PWM_OC1A	PORTB,PB1	// PWM -> LPF -> TCXO Vc

#define ADC_EXTREFLEVEL	6		// ADC6 = detector for external ref


#define BAUD_RATE	9600

//  Print to console using format string in program memory.   Implicit newline in output
#define PRINTLN(fmt, args...) \
		do { printf_P(PSTR(fmt "\n"), ##args); } while (0)

//  Print to console using format string in program memory.
#define PRINT(fmt, args...) \
		do { printf_P(PSTR(fmt), ##args); } while (0)

#define DEBUG(module, fmt, args...) \
		do { if (debug_##module) printf_P(PSTR(fmt "\n"), ##args); } while (0)

#define DEBUG2(module, fmt, args...) \
		do { if (debug_##module) printf_P(PSTR(fmt), ##args); } while (0)
	
#define DEBUG_CHAR(module, c) \
		do { if (debug_##module) putchar(c); } while (0)

//  Macros to set or clear a bit
#define _SETBIT(PORT, BIT)		\
		do { (PORT) |= _BV(BIT); } while (0)

#define _CLRBIT(PORT, BIT)		\
		do { (PORT) &= ~_BV(BIT); } while (0)
	
#define _TGLBIT(PORT, BIT)		\
		do { (PORT) ^= _BV(BIT); } while (0)
			
#define DEBUG(module, fmt, args...) \
do { if (debug_##module) printf_P(PSTR(fmt "\n"), ##args); } while (0)

#define DEBUG_CHAR(module, c) \
do { if (debug_##module) putchar(c); } while (0)

// Variants for PORT,BIT specifications from macros
#define SET(PORTBIT)	_SETBIT(PORTBIT)
#define CLR(PORTBIT)	_CLRBIT(PORTBIT)
#define TGL(PORTBIT)	_TGLBIT(PORTBIT)

#define ISSET(PORTBIT)	(_GETBIT(PORTBIT))
#define ISCLR(PORTBIT)	(~(_GETBIT(PORTBIT)))

#define PIN(x) (*(&x - 2)) // Address Of Data Direction Register Of Port x
#define DDR(x) (*(&x - 1)) // Address Of Input Register Of Port x
#define PORT(x) (x)

#define _DDRSET(PORT, BIT)		\
		do { DDR(PORT) |= _BV(BIT); } while (0)

#define _DDRCLR(PORT, BIT)		\
		do { DDR(PORT) &= ~_BV(BIT); } while (0)

#define _GETBIT(PORT, BIT)		\
		(PIN(PORT) & _BV(BIT))
					
#define OUTPUT(PORTBIT)	_DDRSET(PORTBIT)
#define INPUT(PORTBIT)	_DDRCLR(PORTBIT)

typedef struct {
	uint32_t	freq;			// RF Output Frequnecy (in 10Hz steps)
	uint16_t	INT;
	uint16_t	MOD;
	uint8_t		RDIV;
	uint32_t	FRAC1;
	uint16_t    FRAC2;
	uint32_t	reg[16];
	uint32_t	ref;
	uint32_t	freqerr;
	int16_t		fsk_step;
	int8_t		fsk_offset;
	uint8_t		rf_mult;		// RF Multiplication factor (1=VCO Freq, 2=Doubler enabled)
	uint8_t		rf_div;			// RF Division factor (1=VCO Freq, 2 = /2, 4=/4 etc)
	uint8_t		rfA_enable;		// For ADF5355
	uint8_t		rfB_enable;		// For ADF5355
} PLL_t;

struct zlpll_data {
	uint32_t rf_freq;
	uint32_t	step;
	uint8_t	rdiv;
	uint8_t	level;
	uint8_t	mult;
	uint8_t mode;		// LO, Ext Key Beacon, RF Key Beacon
	uint8_t msg;;		// CW Beacon message number
	uint16_t spare1;
	uint16_t spare2;
	uint16_t spare3;
	uint16_t spare4;
};




union param_data {
	uint16_t    v;
	struct {
		/* LSB First */
		uint8_t	cp:4;    // Charge Pump
		uint8_t	mtld:1;  // Mute until lock detect
		uint8_t spur:1;  // Low Spur Mode=1, Low PN=0
		uint8_t doubler:1; // Disable doubler
	} p;
};

struct config_data {
	uint16_t	magic;
	uint32_t	spare;
	uint32_t	ext_ref;
	uint32_t	spare1;
	uint8_t		bleed;		
	uint8_t		max_doubler;	// Maximum doubler frequency
	uint8_t		charge_pump;
	uint8_t		mtld:1;
	uint8_t		spur:1;
	uint8_t		gcd:1;
	uint8_t		csr:1;
	
	uint8_t		cw_speed;	// wpm
	uint8_t		t1;			// ms
	uint8_t		t2;			// ms
	uint8_t		gap;
	uint8_t		spare2;	

#if PWM_ENABLE	
	uint16_t	pwmval;
	uint16_t	pwmtop;
#endif
#if _ADF4351
	uint16_t	maxmod;
	uint8_t		extlevel;
#endif
	
	uint8_t		i2c_addr;
};


#define SB(val,pos)	(((uint32_t)(val))<<(pos))

void default_config();

uint8_t check_pll_locked();
void cmd_channel(int ch);
void SPI_MasterTransmit(char cData);
uint16_t GCD(uint16_t a, uint16_t b);
void set_freq(uint32_t freq);

extern struct config_data cf;
extern struct zlpll_data  rf;
extern PLL_t	PLL;
extern uint8_t	debug_I2C;
extern uint8_t  debug_PLL;
extern uint8_t  debug_REF;
extern uint8_t  debug_CHAN;
extern uint8_t clkmode;


#define EEPROM_CONF		0		// EEPROM Location for configuration data

#define MUXOUT_TRISTATE	0
#define MUXOUT_DVDD		1
#define MUXOUT_DGND		2
#define MUXOUT_RDIV		3
#define MUXOUT_NDIV		4
#define MUXOUT_ALOCK	5
#define MUXOUT_DLOCK	6
#define MUXOUT_RESERVED	7



#endif /* ZLPLL_H_ */
