/*
 *  PLL Synthesized Local Oscillator
 *
 *  ADF435x version
 *
 *   Copyright (C) 2008-2023 Wanye Knowles ZL2BKC,2025 Simon Eatough ZL2BRG
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

*
*/
// Rev 3.0
// Rev 3.1
//
// Rev 3.1A 	WDK	20-Jan-2013
//	- Change sense of oscillator select output to cater for added BC847
//	  Inverter driving the oscillator power switch.
//      - Allow space in command (ie F 2200)
//
//	PCB includes pins to take lock and ext signals to external LED's
//
// Rev 3.1B	WDK	10-Mar-2013
//	- Restructure to allow for Beacon ID version
//	- Internal version only
//
// Rev 3.1C     WDK     23-Mar-2013
//      - Multiple channel support
//	- Fixed bugs with I/O pins
//	- Stopped transmitting when text entered
//	- Test for bad messages, ie [0] by itself which looped!
//
// Rev 3.1D	WDK	3-Oct-2013
//	- Frequency set to 0 MHz enables external PLL
//	  support for connection to Multi Beacon Controller (MBC)
//      - Channel spacing (N) changed from MHz to Hz
//
//      - Add V command for setting the RDIV value
//      - Set E to 0 to disable external ref selection
//      - Set R to 0 to disable internal ref selection
//        NB.  Power cycle may be required to change clock modes
//
//  Rev 3.1E 
//		- Improve soft UART receiver to sample data in middle of bit. Fixed issue with
//		  corrupted data on input 
//		- EXT_THRESHOLD improvements to avoid false triggering of external clock 
//		- From S/N 200 (excl 201 and 206) plus S/N 184E 
//
//  Rev 3.1F
//		- Add extra stop bit to support PL2303TX USB/RS232 interface
//		- Change TX output mode to better handle the reading of the channel offset (+8 to the channel)
//
//  Rev 4.0
//		- Fork codebase for ATMega328 controller version
//
//  Rev 4.1
//		- Updated board design.  Lockdetect separated from Muxout pins
//		- Use Timer1 for Muxout
//		- Add LCD and encoder support
//
//  Rev 4.1B
//		- Support for ADF5355
//		- Split code functions to allow support for 2 PLL chips
//		- Add bleed config setting for ADF5355
//
//  Rev 4.1C
//      -  Update help file
//		-  Disable bleed on ADF4351
//	Rev 4.1D
//		-  Fix bug with MUXOUT test that failed if the PLL was unlocked before test started 
//
//  Rev 4.1E
//		-  Fix bug when freq=0 (external program mode) SPI was disabled so a 2nd call to set_freq
//		   would lockup.  More importantly the PLL would not boot when freq=0 by default.
//		-  External programming mode (Multi Beacon controller) when freq <30 MHz
//			mode=0 for external reference, mode=1 for internal
//		-  ADF5355:  *TODO* Support enabling both RF outputs for 2nd conversion IF applications
//
//  Rev 4.1F
//		-   RDIV on ADF5355 board (ZLPLL 14G) used to enable RFA Divisor while RFB (doubler) is active
//			RDIV = divisor for RFA
//  Rev 4.1G
//      -	Bugfix:  RFB Output not keyed in CW Mode
//		-	Verbose mode display in show command (LO, CW, RFON, RFOFF etc)
//	Rev 4.1H
//		-	Bugfix:  RF Multiplier rounding errors with high RF Freq (ie. 75887 with Mult 6)
//
//  Rev 4.2A
//		-	Reference divisor (RDIV) improvements with pfd command.
//	Rev V4.4A
//      - Fixed floating point precision register calculation inaccuracies by moving to 64/32bit integer math. 
//      - Register calculations are based on Andy G4JNT's code. Frequency calculation now accurate to 10Hz below 6.8Ghz and //		- 20Hz above. 
//		- Added range checking for ref frequency
// 
// TODO:
//		Check I2C Pullups to stop lockup at power on
//


#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include "zlpll.h"
#include "usart.h"
#include "lcd.h"
#include "twi.h"
#include "cw.h"
#include "encoder.h"
#include "menu.h"
#include "config.h"


#if SWEEP
float  step, end_freq, start_freq;
uint16_t time_step;
uint16_t sweep;
#endif
uint8_t channel;

uint8_t clkmode;
volatile uint8_t extcnt;
volatile uint8_t load_freq;

uint16_t	intcnt=100;

#if CW_BEACON
uint8_t  cw_bits=1, key_down;
uint16_t dotlen;
uint8_t *cw_msgp, *cw_msgstart;
uint8_t cw_msgtype;
#endif

//  Valid values for the rf.mode settings
//
//  Note that bit 0 = RF on/off status
//            bit 1 = LO(0) / CW Mode(1)
//			  bit 2 = Sweep enable bit
//
#define	MODE_LO_RFOFF		0		// 0b000
#define MODE_LO_RFON		1		// 0b001
#define MODE_CW_RFKEY		2		// 0b010
#define MODE_CW_RFON		3		// 0b011
#define MODE_SWEEP			5		// 0b101

#define MODE_RFON_BIT			0b001
#define MODE_BEACON_BIT			0b010

//  Delayed operation for keying RF envelope during CW keying
uint8_t  opdelay;
uint8_t  delayed_op;
#define CARRIER_NOP		0
#define CARRIER_KEYON	1
#define CARRIER_KEYOFF	2
#define CARRIER_RFON	3
#define CARRIER_RFOFF	4


uint16_t t1sec, ticks, tick4;
uint8_t  auxled_pattern;		// FIFO for Aux LED updated 4 times per second
uint8_t  reference_state;		// State of reference			
#define  REF_INTERNAL		0	// Internal reference
#define  REF_EXTWAIT		1	// Loss of lock detected
#define  REF_EXTERNAL		2	// External Reference
#define  REF_EXTFAIL		4	// Lock failure when in external mode

uint8_t  extref_count;			// counter for external ref active

uint8_t  softint_1sec, softint_timer;
uint32_t hr_cnt;

uint8_t	 debug_I2C = 0;
uint8_t  debug_PLL = 0;
uint8_t  debug_REF = 0;
uint8_t  debug_CHAN = 0;

uint8_t	 lcd_active, dial_active;



#define TIME_1SEC	(t1sec)  // 1 second of interrupts

void set_freq(uint32_t);
void rf_envelope(uint8_t op);
void set_rflevel(int8_t enable, int8_t level);

struct config_data cf;
struct zlpll_data  rf;
PLL_t	PLL;

#define EEPROM_TEXT1	100    	// EEPROM location for Beacon text
#define EEPROM_TEXT2	200    	// EEPROM location for Beacon text
#define EEPROM_TEXT3	300    	// EEPROM location for Beacon text
#define EEPROM_TXTEND	400	// Room for 400 Bytes message
#define TEXT_MAXLEN		98

#define EEPROM_HOURS	410
#define EEPROM_PWRON	414

#define EEPROM_CHAN		512		// Start of per-channel frequency data



void SPI_Init(void)
{
	//
	// Configure SPI Master Mode
	//
	// **IMPORTANT**
	//
	// The SS pin must be programmed as a output otherwise the SPI
	// controller waits for the pin to become active before sending
	//

//	power_spi_enable();

	OUTPUT(ADF4351_DATA);		// Setup SPI pins for output
	OUTPUT(ADF4351_CLK);
	OUTPUT(ADF4351_LE);
	CLR(ADF4351_LE);

	// Enable SPI, Master Master Mode, Fosc/16
	SPCR = _BV(SPE) | _BV(MSTR) |_BV(SPR0); //| _BV(CPOL); // _BV(CPHA); // SPI Mode 3
}

void
SPI_MasterTransmit(char cData)
{
	SPDR = cData;

	while(!(SPSR & _BV(SPIF)))
		{ /* NOP */ }
	(volatile void) SPDR;
}

void eeprom_increment(uint32_t *addr) {
	uint32_t val;

	val = eeprom_read_dword(addr);
	val++;
	eeprom_write_dword(addr, val);
}

int16_t adc_read(uint8_t mux)
{
	if (mux == 8) {
			ADMUX=_BV(REFS1)|_BV(REFS0);   // Vref=1.1V for reading internal temperature
	} else {
			ADMUX=_BV(REFS0);           // For Aref=AVcc;
	}
	
	ADCSRA=_BV(ADEN)|_BV(ADPS2)|_BV(ADPS1); // Prescalar div factor = 64 for 8MHz FCPU

	ADMUX |= mux & 0x1f;

	ADCSRA|=(1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	
	ADCSRA|=(1<<ADIF);
	return ADC;
}

// Read the channel select input and convert the voltage to the 
// closest channel number
static uint8_t KV[] = {255,197,222,177,237,186,208,168,246,191};
int8_t chan_read(uint8_t mux)
{
	uint8_t adcval, i, key, k, off;

	key = ~PINC & 0x0f;
	return key;
	
	adcval = adc_read(mux) >> 2; // Only need 8 bits
	//PRINTLN("ADC: %x", adcval);
	key = 0;
	k = 255;
	for (i=0; i<10; i++) {
		off = abs(adcval - KV[i]);
		if (off <= k) {
			k = off;
			key = i;
		}
	}	
	return key;
}

void crlf()
{
	putchar('\n');
}

uint16_t GCD(uint16_t a, uint16_t b)
{
	uint16_t c;
	while(1)
	{
		c = a%b;
		if (c==0)
			return b;
		a = b;
		b = c;
	}
}

#if CW_BEACON

void key_cw(uint8_t offon) {
	if (offon) {
		if (cf.t1 > 0) {
			rf_envelope(CARRIER_RFON);
			delayed_op = CARRIER_KEYON;  opdelay = cf.t1;
		} else { // t1 <= 0
			rf_envelope(CARRIER_RFON);
			rf_envelope(CARRIER_KEYON);
		}
	} else {
		if (cf.t2 > 0) {
			rf_envelope(CARRIER_KEYOFF);
			delayed_op = CARRIER_RFOFF;  opdelay = cf.t2;
		} else { // t2 <= 0
			rf_envelope(CARRIER_KEYOFF);
			rf_envelope(CARRIER_RFOFF);
		}
	}
}

struct t_mtab { char c, pat; } ;
struct t_mtab morsetab[] = {
	{'^', 255},  // Carrier on for 1 second
	{'_', 254},  // Carrier off for 1 second

	{'.', 106},	{',', 115},	{'?', 76},	{'/', 41},	{'A', 6},	{'B', 17},	{'C', 21},	{'D', 9},
	{'E', 2},	{'F', 20},	{'G', 11},	{'H', 16},	{'I', 4},	{'J', 30},	{'K', 13},	{'L', 18},
	{'M', 7},	{'N', 5},	{'O', 15},	{'P', 22},	{'Q', 27},	{'R', 10},	{'S', 8},	{'T', 3},
	{'U', 12},	{'V', 24},	{'W', 14},	{'X', 25},	{'Y', 29},	{'Z', 19},	{'1', 62},	{'2', 60},
	{'3', 56},	{'4', 48},	{'5', 32},	{'6', 33},	{'7', 35},	{'8', 39},	{'9', 47},	{'0', 63}
} ;

#define N_MORSE  (sizeof(morsetab)/sizeof(morsetab[0]))

// CW Rate is 1200/W in ms
//
// Interrupt rate = baudrate * 3, ie 28800 for 9600 baud
//
// 1ms = 28.8 ticks, therefore interrupts = 28.8*1200 / W
//

#endif


#if CW_BEACON

void cw_start(uint8_t type, uint8_t *msg, uint8_t delay) {
	uint8_t cw_wpm;
	cw_wpm = cf.cw_speed;
	if (cw_wpm < 5 || cw_wpm > 50)
		cw_wpm = 12;
	dotlen = ((t1sec+t1sec/5)/cw_wpm);  // 1200 ms per dot length

	cw_msgtype = type;
	cw_msgp = cw_msgstart = msg;
	
	if (delay == 0)		//  Delay can't be zero as this stops interrupts and CW is disabled
		delay = 5;
	
	// If running beacon mode setup delay of about 5 secs before sending first bit
	intcnt = rf.mode & MODE_BEACON_BIT ? dotlen*delay:0;
}

void cw_stop(uint8_t rfon) {
		intcnt = dotlen = 0;		// Disable CW Mode
		key_cw(rfon);				// Carrier ON
}
	
uint8_t cw_getchar(void)
{
	switch (cw_msgtype) {
		case MSGTYPE_EEPROM:	return (eeprom_read_byte(cw_msgp));	break;
		case MSGTYPE_PGMMEM:	return (pgm_read_byte(cw_msgp));	break;
		case MSGTYPE_RAM:		return (*cw_msgp);	break;
	}
	return (' ');
}

uint8_t get_next(void)
{
	uint8_t c;

	c = cw_getchar();

	// EEPROM Messages wrap automatically, and others will stop in idle
	// waiting for next mode to start.   This is mainly for CW acknowledgment
	// messages which should not repeat
	//
	if (c == '~')		// Indefinite carrier off
		return '_';
	if (c == (char)0) {  // Wrap message
		cw_msgp = cw_msgstart;
		c = cw_getchar();
		if (c == 0)
			c = ' ';
		else
			cw_msgp++;
	} else
		cw_msgp++;
	
	return c;
}


// Handle special test of CHAN input
//
// [^MSG]   Send MSG if PINA2 is High
// [_MSG]   Send MSG if PINA2 is Low
//
// [0]	    Set TX Output pin PINA1 Low
// [1]	    Set TX Output pin PINA1 High (+3.3v)
//
uint8_t next_char() {
	uint8_t i;
	uint8_t c, c2;

	c = get_next();
	i = 5;
loop:
	while (c=='[' && --i > 0) {
		c2=get_next();
		if (c2 == '^' || c2 == '_') {
			if (c2 == '^') { c2 = _BV(PINC2); }
			else { c2 = 0; }
			if (c2 != (PINC & _BV(PINC2))) {
				// Read up to closing ] 
				c=get_next();
				while (c != '[' && c != ']')
				c = get_next();
			}
		} 
		else if (c2 == '0') {
			PORTC &=  ~_BV(PINC1);   // TX Pin = Lo
		}
		else if (c2 == '1') {
			PORTC |=  _BV(PINC1);    // TX Pin = High
		}

		if (c2 >= '0' && c2 <= '7') {
			key_cw(0);  // Ensure carrier is off before shifting frequency
			cmd_channel(c2-'0');
		}
		c = get_next();
	}
	if (c == ']') {
		c=get_next(); 
		goto loop;
	}

	for (i=0; i<N_MORSE; i++) {
		if (morsetab[i].c == c) {
			return  morsetab[i].pat ;
		}
	}
	return 1;
}
#endif


void rf_envelope(uint8_t op) {
	switch (op) {
		case CARRIER_NOP:									break;
		case CARRIER_KEYON:  	SET(AUXLED);	SET(LED2);	break;
		case CARRIER_KEYOFF:	CLR(AUXLED);	CLR(LED2);	break;
		case CARRIER_RFON:
					PLL(set_rflevel)(1, rf.level);	//PLL_Write(reg4|(1L<<5));
				break;
		case CARRIER_RFOFF:
					PLL(set_rflevel)(0, 0);			//PLL_Write(reg4&~(1L<<5)); 
				break;
	}
}

// Timer callback interrupt, called from soft serial timer interrupt routine
void timer_callback()
{

#if CW_BEACON

		if (rf.rf_freq == 0)
			return;
		if (key_down) {
			// Have just sent a dot or dash, need to send a gap before
			// the next symbol
			intcnt = dotlen;
			key_cw(key_down=0);
		} else {
			if (cw_bits != 1) {  // More symbol to send
				intcnt = (cw_bits&1)?3*dotlen:dotlen;
				key_cw(key_down=1);
				cw_bits >>= 1;
			} else {  // End of symbol
				cw_bits = next_char();
				if (cw_bits == 1) 
					intcnt = (7+(cf.gap))*dotlen;  // Send a space
				else if (cw_bits > 250) {
					// Special characters
					// ^ = 1 second carrier on
					// _ = 1 second carrier off
					key_cw(cw_bits&1);
					intcnt = TIME_1SEC;
					// Trick into pulling new character after 1 second delay
					// also need to prevent a inter symbol space being inserted
					key_down = 0;
					cw_bits=1;  // trick into pulling next char after timeout
				} else
					intcnt = (2+cf.gap)*dotlen; // Inter-char gap.  ne dotlen already passed
			}
		}
#endif
}

void save_config() {
	eeprom_write_block(&cf,	(char *)(EEPROM_CONF),	sizeof(cf));
}

void set_freq(uint32_t freq) {
	// Disable all timer operations
	softint_timer = 0;
	intcnt = 0;
	opdelay = 0;
	DEBUG(PLL>3, "set_freq(%lu)", freq);

	// We may have been powered down previously and SPI mode disabled.  Ensure SPI mode
	// is enabled again so we can program the PLL
	DEBUG(PLL>4, "SPI_Init");
	SPI_Init();
			
	// Frequency 0 disables the PLL programming and places it into slave mode
	if (freq < 30.0 ) {
		PLL(powerdown)();
		SPCR = 0;		// Disable SPI Master mode
		INPUT(ADF4351_DATA);	INPUT(ADF4351_CLK);		INPUT(ADF4351_LE);
		CLR(ADF4351_DATA);		CLR(ADF4351_CLK);		CLR(ADF4351_LE);
		DEBUG(PLL>4, "SPI_3State");
		
		if (rf.mode == 1)	{
			DEBUG(PLL>1, "REF_INTERNAL");
			clkmode = 0;	// Internal
			CLR(AUXLED);	SET(REFSEL);	SET(OSCPOWER);
		} else {
			DEBUG(PLL>1, "REF_EXTERNAL");
			clkmode = 1;	// External
			SET(AUXLED);	CLR(REFSEL);	CLR(OSCPOWER);
		}
		
#if CW_BEACON
		dotlen = intcnt = 0;
#endif
		return;
	} 

	PLL(set_freq)(freq);		// Initialize PLL
	
#if CW_BEACON
	cw_start(MSGTYPE_EEPROM, (uint8_t *)EEPROM_TEXT1, 20);
#endif

	// If we are running in LO mode then set LED2 (external RF enable)
	//
	if (!(rf.mode & MODE_BEACON_BIT)) {		// LO Mode
		SET(LED2);
		softint_timer = 0;
		intcnt = 0;
		opdelay = 0;		// Cancel special operations timer
	}
	
	//
	//  Reprogramming the PLL results in several transitions of the Lock status
	//  These should new be behind us, so reset the loss lock detector
	//
	_delay_ms(2);
}

// Timer1 interrupts
ISR(TIMER1_COMPA_vect)
{
}

// Timer callback interrupt, called from soft serial timer interrupt routine
ISR(TIMER2_COMPA_vect) // Timer1 compare match interrupt
{
	if (--ticks == 0) {
		ticks = t1sec;		// Reset 1sec counter for next cycle
		softint_1sec++;
	}
	
	// LED ticker is 4 interrupts per second
	if (tick4 && --tick4 == 0) {
		if (auxled_pattern & 1)		SET(AUXLED);
	else							CLR(AUXLED); 
		auxled_pattern >>= 1;		// Update pattern for next flash
		tick4 = t1sec/4;		
	}
	
	// opdelay has higher priority than intcnt, ie. when opdeley is in effect intcnt will
	// not count so the CW symbol length is not truncated
	if (opdelay && --opdelay == 0)
		rf_envelope(delayed_op);
	else if (intcnt && --intcnt == 0) {
		softint_timer++;
	}
}

//  Check lock status of PLL and update locked LED to reflect status
//	Returns:  0 = Unlocked
//			  1 = Locked
uint8_t check_pll_locked() {
	uint8_t islocked;
	
	islocked = ISSET(LOCKINPUT); // && (TCNT0 == 0);
	// Update Lock status LED's
	if (islocked)	{ SET(LOCKLED);	SET(LED1); }	// PLL is locked
	else 			{ CLR(LOCKLED); CLR(LED1); }	// PLL is unlocked

	//TCNT0 = 0;		// Reset lock loss counter
	return islocked;
}

#if PWM_ENABLE
void PWM_init() {
		// Output OC1A controls the TCXO frequency
		// Uses 16bit Frequency/Phase PWM mode which feeds a low pass filter
		// with Fc < 22Hz to generate the tuning voltage for the TCXO

		OUTPUT(PWM_OC1A);		// Configure PWM pin as output
		TCCR1A = _BV(COM1A1) /* | _BV(WGM10) */ | _BV(WGM11);
		TCCR1B = _BV(CS10) | _BV(WGM13);		// Prescale x1, Full 16 bit PWM
}

void PWM_set(uint16_t val, uint16_t top) {
	if (val == 0 || val == top) {
		TCCR1B = TCCR1A = 0;		// Disable PWM
		CLR(PWM_OC1A);				// Toggle low 
	} else {
		OCR1A = val;
		ICR1  = top;
	}
}

void cmd_cal() {
	char		key;
	uint16_t    pwmtop, pwmval;
		
	pwmval = cf.pwmval;
	pwmtop = cf.pwmtop;		
	PRINTLN("Freq Cal.  +/- = 1Hz   Q/W=1000  A/S=100  Z/X=10  +/-=1");
	PRINTLN("TOP :  D/F = 100 C/V=1 < >  \n/=Quit");
		
	while (1) {
		PRINT("PWM: %u/%u\n", pwmval, pwmtop);
						
		while (!USART_rxready()) {
			wdt_reset();
		}
		key = getchar() & 0x7f;
		if (key >= 'A' && key <= 'Z')
				key -= (char)32;		// Upper -> lower
				
		switch (key) {
			case '+':   pwmval ++; break;
			case '-':   pwmval --; break;
			
			case 'w':	pwmval += 1000; break;
			case 'q':	pwmval -= 1000; break;

			case 's':	pwmval += 100; break;
			case 'a':	pwmval -= 100; break;

			case 'x':	pwmval += 10; break;
			case 'z':	pwmval -= 10; break;
				
			case 'f':	pwmtop ++; break;
			case 'd':   pwmtop --; break;
			
			case 'v':	pwmtop += 100; break;
			case 'c':   pwmtop -= 100; break;
			
			case '<':	if (pwmtop < 32000) {
							pwmtop <<=1;	pwmval <<=1; 
						}
						break;
			case '>':	if (pwmtop > 1024) {
							pwmtop >>=1;	pwmval >>=1; 
						}
						break;
				
			case '/':	case '\n':
			case ' ': PRINT("Saving...");
						cf.pwmtop = pwmtop;
						cf.pwmval = pwmval;
						save_config();
						PRINT("OK\n");
						return;  break;		
		}
		
		if (pwmtop < 1024)  pwmtop = 1024;
		if (pwmval > 60000) pwmval = 0;
		if (pwmval > pwmtop) pwmval = pwmtop;

		PWM_set(pwmval, pwmtop);
	}		
}
#endif

void cmd_adc() {
	int i;
	PRINT("ADC Values:\n");
	for (i=0; i<16; i++) {
		PRINT("    %2d: %4d", i, adc_read(i));
		if ((i & 0x3) == 0x3)
			putchar('\n');
	}
}


void cmd_freq(uint32_t freq) {
	rf.rf_freq = freq;
#if SWEEP
	start_freq = rf.rf_freq;
#endif	
}

void cmd_help() {
		PRINTLN("Command           Description");
		PRINTLN("cw speed #        Set the CW Speed in WPM");
		PRINTLN("cw text           Update CW message text");
		PRINTLN("channel #         Load channel for reconfiguration");
		PRINTLN("save #            Save channel");
		PRINTLN("freq #.###        Set RF frequency");
		PRINTLN("level #           Set RF Level (0=0ff to 4=High)");
		PRINTLN("config param #    Setup PLL Parameters");
		PRINTLN("ref #.##       Reference frequency - External");
		PRINTLN("diag              Display PLL data for channel");
		PRINTLN("debug # #         Display debugging information");
		PRINTLN("test              Enter RF Test mode");
		PRINTLN("init              Reload PLL Setup");
		PRINTLN("i2c_addr          Set I2C Slave address");
#if PWM_ENABLE
		PRINTLN("cal               TCXO Frequency calibration");
#endif
		PRINTLN("show              Display current config");
		PRINTLN("default		   Reset configuration to default");

}

void cmd_cwtext(int16_t arg) {
	uint8_t key, *ptr;
	uint8_t  cnt;
	
	ptr = (uint8_t *) (arg ? EEPROM_TEXT2: EEPROM_TEXT1);
	cnt = 0;

	PRINT("\n\rText: ");

	while ((key = getchar()) != (char)0 && ++cnt < TEXT_MAXLEN) {
		if (key == '\n' || key == '\r')
		break;
		if (key >= 'a' && key <= 'z')
		key -= 32; // lower case -> upper case
		eeprom_write_byte(ptr, key);
		putchar(key);
		ptr++;
	}
	eeprom_write_byte(ptr, (uint8_t) 0);
	PRINTLN("\nOK");
}


//
// Power off as much of the CPU to reduce any potential EMI/EMC issues 
//
// Mainly used for troubleshooting purposes
//
void cmd_sleep() {
	PRINTLN("Halting Micro....");

	wdt_reset();
	MCUSR = 0;
	wdt_disable( );
	cli();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	PORTB = 0;	DDRB = 0;
	PORTC = 0;  DDRC = 0;
	PORTD = 0;  DDRD = 0;
//	power_spi_disable();
	TCCR0A = 0;  TCCR0B = 0;
	TCCR1A = 0;  TCCR1B = 0;
	TCCR2A = 0;  TCCR2B = 0;
	CLKPR = _BV(CLKPCE);
	CLKPR = _BV(CLKPS3);		// Prescaler = 256 = Slowest clock
	sleep_enable();
	while (1) {
		sleep_mode();
		putchar('*');
	}
	// Never returns....
}
void cmd_cwspeed(uint16_t arg) {
	if (arg >=5 && arg < 50) {
		cf.cw_speed = arg;
		save_config();
	} else {
		PRINTLN("Range Error");
	}
	//cw_init();
}

void cmd_config(uint8_t argc, char *argv[]) {
	uint16_t val;
	
	if (argc != 3)
		goto err;
	val = atol(argv[2]);
	if (strcasecmp_P(argv[1], PSTR("cp"))==0) 
		 cf.charge_pump = val & 0x0F;
	else  if (strcasecmp_P(argv[1], PSTR("spur"))==0) 
		cf.spur = val & 0x01;
	else if (strcasecmp_P(argv[1], PSTR("mtld"))==0)
		cf.mtld = val & 0x01;
	else if (strcasecmp_P(argv[1], PSTR("csr"))==0)
		cf.csr = val & 0x01;
#if _ADF5355
	else if (strcasecmp_P(argv[1], PSTR("bleed"))==0)
		cf.bleed = val;
#else
	else if (strcasecmp_P(argv[1], PSTR("extlevel"))==0)
		cf.extlevel = val;
	else if (strcasecmp_P(argv[1], PSTR("maxmod"))==0)
		cf.maxmod = val;
#endif /* _ADF5355 */
	else {
err:
			PRINTLN("Usage: config  cp|spur|mtld|doubler|pfd  <val>");
			return;
	}
	save_config();
}

void cmd_diag() {
		uint8_t  i, max;

		PRINTLN("RF Freq:      %lu", rf.rf_freq);
		PRINTLN("PLL Freq:     %12.3f", PLL.freq);
		PRINTLN("R Freq:       %12.3f", PLL.ref);
		PRINTLN("Freq Error:   %+.2f Hz", PLL.freqerr*1e6);
		PRINTLN("  RDIV=%d, INT=%d, FRAC=%d, MOD=%d",
					PLL.RDIV, PLL.INT, PLL.FRAC1, PLL.MOD);
		crlf();
#if _ADF5355
		max = 12;
#else	
		max = 5;
#endif

		for (i=0; i<=max; i++) {
			PRINT("  reg%-2d= 0x%08lx", i, PLL.reg[i]);
			if ((i%2) == 1)
			putchar('\n');
		}
		putchar('\n');
}


//  Measure the time to lock in millisecond
uint8_t  lock_time(uint8_t max) 
{
	uint8_t t;
	
	for (t=0; t<max; t++) {
		if (check_pll_locked())
			break;
		_delay_us(100);
	}
	return t;
}

void sweep_delay(uint8_t t) 
{
	while (t--) {
		_delay_ms(1);
	}
}

//  Test the range of the VCO by sweeping 
void sweep_test() {
	int16_t deltaf, lckcnt, cycle;
	uint8_t l, lmax;

	// Setup 16 bit Counter/Timer 1 for tests using MUXOUT pin of ADF4351
	TCCR1A |= 0;
	TCCR1B |= _BV(CS01) | _BV(CS02);		// Clock on falling edge of T0 input
	
	PLL(set_muxout)(MUXOUT_DLOCK);
	CLR(AUXLED);
	
	lckcnt = 0;
	PRINTLN("Test Starting");
	cycle = 0;
	while (!USART_rxready()) {
		cycle++;
		lmax = 0;
		for (deltaf=0; deltaf < 100; deltaf++)	{
			// -ve direction sweep
			PLL(fsk)(10, -deltaf);
			l = lock_time(100);
			if (l > lmax)  lmax =l;
			
			if (l == 100)
				PRINTLN("Unlck -%d", deltaf);
			
			TCNT1 = 0;
			sweep_delay(l);
			
			if (!check_pll_locked() || TCNT1 != 0)
				lckcnt++;
			
			// +ve direction sweep	
			PLL(fsk)(100, +deltaf);
			l = lock_time(100);
			if (l > lmax)  lmax =l;

			if (l == 100)
				PRINTLN("Unlck +%d", deltaf);
			
			TCNT1 = 0;
			sweep_delay(l);
			
			if (!check_pll_locked() || TCNT1 != 0)
				lckcnt++;
			wdt_reset();
		}
		PRINTLN("Test %d: Unlocks=%d  LockMax=%d", cycle, lckcnt, lmax);
		if (lckcnt)
			SET(AUXLED);
	}
	PRINTLN("Exit");
}


void cmd_test() {
	char key;
	uint8_t  muxout;
	int16_t deltaf;
	
	PRINTLN("ZLPLL Test Mode");
	PRINTLN("Set Muxout=  T, N, R, 0, 1, A, D");
	PRINTLN("   Toggle (P)ullup, (Q)uit");
	PRINTLN("   J/K = PLL Freq adjust (fsk)  S)weep test");

	INPUT(MUXOUTPIN);  SET(MUXOUTPIN);		// Input with pullup to help VCO test which
											// delivers short pulses
	deltaf = 0;
		
	while (1) {
		muxout = 99;
					
		while (!USART_rxready()) {
			wdt_reset();
			(void) check_pll_locked();		//  Update Lock LED
		}
		key = getchar() & 0x7f;
		if (key >= 'a' && key <= 'z')
				key -= (char)32;		// Lower -> Upper
			
		switch (key) {
			case 'N':   muxout = MUXOUT_NDIV;	break;
			case 'R':	muxout = MUXOUT_RDIV;	break;
			case '1':	muxout = MUXOUT_DVDD;	break;
			case '0':	muxout = MUXOUT_DGND;	break;
			case 'T':   muxout = MUXOUT_TRISTATE; break;
			case 'A':	muxout = MUXOUT_ALOCK;	break;
			case 'D':	muxout = MUXOUT_DLOCK; break;
			case 'P':	TGL(MUXOUTPIN);			// Toggle pullup
						break;
			case 'Q':   PRINTLN("Quit");	
						return;
			case 'J':	PLL(fsk)(10, --deltaf);  break;
			case 'K':	PLL(fsk)(10, ++deltaf);  break;
			
			case 'S':	sweep_test();   break;
		}
		if (muxout != 99)
			PLL(set_muxout)(muxout);
	}
}

void cmd_show_config() {
	int i;
	struct zlpll_data eedata;
#if CW_BEACON
	uint8_t *ptr, c;
#endif

	PRINT("Frequency:  %lu", rf.rf_freq);
	crlf();
	PRINTLN("Level:       %d", rf.level);
	PRINTLN("Ref Freq: %lu MHz", cf.ext_ref);
#if _ADF5355
	PRINTLN("Config:       cp=%d, bleed=%d",	cf.charge_pump, cf.bleed);
#endif
	PRINTLN("Config:       mtld=%d, csr=%d, gcd=%d, spur=%d",
				cf.mtld, cf.csr, cf.gcd, cf.spur);
	PRINTLN("I2C Addr:     0x%02X\n", cf.i2c_addr);

	wdt_reset();
#if CW_BEACON
	USART_puts_P("CW Text=");
	ptr = (uint8_t *) EEPROM_TEXT1;
	c = eeprom_read_byte(ptr++);
	while (c) {
		USART_putchar(c, NULL);
		c = eeprom_read_byte(ptr++);
	}
	crlf();
	PRINTLN("CW Speed=%d WPM  Gap=%d T1=%dms T2=%dms", cf.cw_speed, cf.gap, cf.t1, cf.t2);
#endif

	crlf();
	wdt_reset();
	for (i=0; i < MAX_CHANNELS; i++) {
		eeprom_read_block(&eedata, (char *)(EEPROM_CHAN+sizeof(eedata)*i),	sizeof(eedata));

		PRINT("%2d:   Freq %lu  level=%d", i, eedata.rf_freq, eedata.level);
		PRINT(" mode=%d (", eedata.mode); 
		switch (eedata.mode) {
			case 0:	PRINT("LO, RFOFF");	break;
			case 1:	PRINT("LO, RFON");	break;
			case 2: PRINT("CW, RFKEY");	break;
			case 3: PRINT("CW, RFON");	break;
		} 
		putchar(')');

		if (eedata.mode & MODE_BEACON_BIT)
			PRINT(" msg=#%d", eedata.msg);
		crlf();
		wdt_reset();
	}
}

void cmd_channel(int ch) {
	eeprom_read_block(&rf, (char *)(EEPROM_CHAN+sizeof(rf)*ch), sizeof(rf));
	if (rf.rdiv < 1)	rf.rdiv=1;

#if SWEEP
	start_freq = rf.rf_freq;
	end_freq = 0;		/* Disable sweep mode if active */
#endif	
}

void cmd_save(int ch) {
	if (ch >=0 && ch <MAX_CHANNELS)
		eeprom_write_block(&rf,	(char *)(EEPROM_CHAN+sizeof(rf)*ch), sizeof(rf));
	else
		PRINTLN("Channel %d out of range", ch);
}

void cmd_debug(uint8_t argc, char *argv[]) {
	uint8_t *var, val;
	
	var = NULL;
	if (strcasecmp_P(argv[1], PSTR("I2C"))==0)  var = &debug_I2C;	else
	if (strcasecmp_P(argv[1], PSTR("PLL"))==0)  var = &debug_PLL;	else
	if (strcasecmp_P(argv[1], PSTR("CHAN"))==0) var = &debug_CHAN;	else
	if (strcasecmp_P(argv[1], PSTR("REF"))==0)  var = &debug_REF;

	else {
		PRINTLN("  I2C  = %d", debug_I2C);
		PRINTLN("  PLL  = %d", debug_PLL);
		PRINTLN("  REF  = %d", debug_REF);
		PRINTLN("  CHAN = %d", debug_CHAN);
		return;
	}
	
	val = atol(argv[2]);
	if (var) {
		*var = val;
		PRINTLN("%s = %d", argv[1], val);
	}
}

#define MAXARGS  10

#include <stdint.h>
#include <limits.h>
#include <stdint.h>
#include <limits.h>
uint32_t parse_decimal_number(const char *str) {
	uint32_t result = 0;
	uint8_t decimal_places = 0;
	uint8_t decimal_seen = 0;

	while (*str) {
		if (*str == '.') {
			if (decimal_seen++) return 0; // Invalid: multiple decimal points
		} else if (*str >= '0' && *str <= '9') {
			result = result * 10 + (*str - '0');
			if (decimal_seen && ++decimal_places > 5) return 0; // Invalid: >5 digits after decimal
		} else {
			return 0; // Invalid character
		}
		str++;
	}

	while (decimal_places++ < 5) result *= 10; // Normalize to 5 decimal places

	return result;
}


	
void exec_command(char *buf)
{
	int16_t	  intarg;
	double    dblarg;
	uint32_t  longintarg;

	char *argv[MAXARGS];
	uint8_t argc, v;
	
	argc = 0;
	v = 0;
	while (*buf && *buf == ' ') *buf++=(char)0;	// skip spaces
	for (v=0; *buf && v<MAXARGS; v++) {
		// replace spaces with null terminator to help terminate each word
		argv[v] = buf;
		argc++;
		while (*buf && *buf != ' ') buf++;	// digest command
		while (*buf && *buf == ' ') *buf++=(char)0;	// skip spaces
	}
	for (; v<MAXARGS; v++)
		argv[v] = buf;		//  Point to NULL terminator

	if (argc == 0)
		return;  // No command

	intarg = atol(argv[1]);
	dblarg = atof(argv[1]);
	longintarg = parse_decimal_number(argv[1]);


	if (strcasecmp_P(argv[0], PSTR("debug"))==0) {
		cmd_debug(argc, argv);		return;
	} else
	if (strcasecmp_P(argv[0], PSTR("sleep"))==0) {
		cmd_sleep();		return;
	} else
	if (strcasecmp_P(argv[0], PSTR("config"))==0) {
		cmd_config(argc, argv);		goto reload;
	} else
	if (argc >= 2) {
		if (strcasecmp_P(argv[0], PSTR("cw"))==0) {
			if (strcasecmp_P(argv[1], PSTR("text"))==0) {
				cmd_cwtext(0);  return;
			}
			intarg = atol(argv[2]);
			if (strcasecmp_P(argv[1], PSTR("speed"))==0) {
				cmd_cwspeed(intarg);  return;
			}
			if (strcasecmp_P(argv[1], PSTR("gap"))==0) {
				cf.gap = intarg;  return;
			}
			if (strcasecmp_P(argv[1], PSTR("t1"))==0) {
				cf.t1 = intarg;  return;
			}
			if (strcasecmp_P(argv[1], PSTR("t2"))==0) {
				cf.t2 = intarg;  return;
			}
		} else
		if (strcasecmp_P(argv[0], PSTR("freq"))==0) {
			cmd_freq(longintarg);
			goto reload;
		} else
#ifdef _ADF4351
		if (strcasecmp_P(argv[0], PSTR("ref_int"))==0) {
			cf.int_ref = dblarg;				goto save;
		} else
#endif
		if (strcasecmp_P(argv[0], PSTR("ref"))==0) {
			cf.ext_ref = longintarg;
			if (cf.ext_ref < 500000 || cf.ext_ref > 10000000){
				PRINTLN("ref out of range");
				cf.ext_ref = 1000000;
			}
			goto save;
		} else
		if (strcasecmp_P(argv[0], PSTR("i2c_addr"))==0) {
			cf.i2c_addr = intarg;				goto save;
		} else
		if (strcasecmp_P(argv[0], PSTR("level"))==0) {
			rf.level = intarg &3;
			goto reload;
		} else
		if (strcasecmp_P(argv[0], PSTR("mode"))==0) {
			rf.mode = intarg & 3;
			intcnt = TIME_1SEC*5;	// Enable CW interrupts after 5 secs
			goto reload;
		} else
		if (strcasecmp_P(argv[0], PSTR("multiplier"))==0) {
			rf.mult = intarg?intarg:1;  
			goto reload;
		} else
		if (strcasecmp_P(argv[0], PSTR("step"))==0) {
			rf.step = dblarg;					goto reload;
		} else		
		if (strcasecmp_P(argv[0], PSTR("rdiv"))==0) {
			rf.rdiv = intarg & 0xff;			goto reload;
		} else
		if (strcasecmp_P(argv[0], PSTR("param"))==0) {
			PRINTLN("Unimplemented");
			goto reload;
		}	else
		if (strcasecmp_P(argv[0], PSTR("channel"))==0) {
			cmd_channel(intarg);
			goto reload;
		}	else
		if (strcasecmp_P(argv[0], PSTR("save"))==0) {
			cmd_save(intarg);
			return;
		}		
	}
	
	if (strcasecmp_P(argv[0], PSTR("show"))==0) {
		cmd_show_config();
		return;
	}
	if (strcasecmp_P(argv[0], PSTR("help"))==0 || *argv[0] == '?') {
		cmd_help();
		return;
	}
	if (strcasecmp_P(argv[0], PSTR("adc"))==0) {
		cmd_adc();
		return;
	}
	if (strcasecmp_P(argv[0], PSTR("diag"))==0) {
		cmd_diag();
		return;
	}
#if PWM_ENABLE
	if (strcasecmp_P(argv[0], PSTR("cal"))==0) {
		cmd_cal();
		return;
	}
#endif

	if (strcasecmp_P(argv[0], PSTR("test"))==0) {
		cmd_test();
		return;
	}
	if (strcasecmp_P(argv[0], PSTR("default"))==0) {
		default_config();
		goto reload;
	}

	PRINTLN("**Unknown Command: %s", argv[0]);
	return;

	save:
		save_config();	
	reload:
		cli();
		set_freq(rf.rf_freq);
		sei();
}


// Toggle diagnostic LED's during startup process.  Used by the pll_test()
// function
static void diag_ON() {
	SET(LOCKLED);
	_delay_ms(100);
	wdt_reset();
	// putchar('.');
}

static void diag_OFF() {
	CLR(LOCKLED);
	_delay_ms(100);
	wdt_reset();
	_delay_ms(100);
}


void load_config() {

	eeprom_read_block(&cf,	(char *)(EEPROM_CONF),	sizeof(cf));

		PRINTLN("Magic number %d",cf.magic);

	if (cf.magic != ZLPLL_MAGIC) {
		cf.magic = ZLPLL_MAGIC;
		cf.ext_ref = 1000000;
		cf.max_doubler = 32;
		cf.charge_pump = 3;
		cf.bleed = 20;
		cf.mtld = 1;
		cf.spur = 0;
		cf.gcd = 0;
		cf.csr = 0;
		cf.cw_speed = 12;	// wpm
		cf.t1 = 0;			// ms
		cf.t2 = 5;			// ms
		cf.gap = 0;
#if PWM_ENABLE
		cf.pwmval = 2002;
		cf.pwmtop = 8192;
#endif
#if _ADF4351
		cf.extlevel = 20;
		cf.maxmod = 4096;
#endif
		PRINTLN("Factory Defaults Loaded");
		save_config();
	}
}


void pll_test() {
	uint16_t i;
	uint8_t  err, lock;
	
	PRINT("\n" PLLCHIP " Test");
	err = 0;
	INPUT(LOCKIN);
	INPUT(MUXOUTPIN);  SET(MUXOUTPIN);		// Input with pullup to help VCO test which
											// delivers short pulses

	// Setup 8 bit Counter/Timer 0 to monitor Lock Detect pin for loss of lock
	//TCCR0A |= 0;
	//TCCR0B |= _BV(CS01) | _BV(CS02);		// Clock on falling edge of T0 input
	//TCNT0 = 0;

	// PLL could be locked and create a false clock when the MUXOUT test runs
	//  Set initial state of MUXOUT to logic 0
	PLL(set_muxout)(MUXOUT_DGND);		// Mux=DGND
	
	// Setup 16 bit Counter/Timer 1 for tests using MUXOUT pin of ADF4351
	TCCR1A |= 0;
//	TCCR1B |= _BV(CS11) | _BV(CS12);		// Clock on falling edge of T0 input
	TCCR1B |= _BV(CS10) | _BV(CS11) | _BV(CS12);		// Clock on rising edge of T0 input
	TCNT1 = 0;
	
	// Generate 120 manual pulses to test SPI interface and Muxout pin
	diag_ON();
	for (i=120; i; i--) {
		PLL(set_muxout)(MUXOUT_DGND);		// Mux=DGND
		PLL(set_muxout)(MUXOUT_DVDD);		// Mux=DVDD
	}
	diag_OFF();
		
	if ((i=TCNT1) != 120) {
		PRINT("\n*MUX FAIL* TCNT %u!=120", i);
		err++;		SET(AUXLED);
	}

	TCNT1 = 0;
	PLL(set_muxout)(MUXOUT_RDIV);			// Mux=R Counter Output

	diag_ON();  // implicit 250ms delay.  Timer1 expected to overflow and set timer1_ovf flag
	if (TCNT1 < 50) {
		PRINT("\n*REF FAIL* R=%u", TCNT1);
		err++;		SET(AUXLED);
	}
	diag_OFF();
		
	PLL(set_muxout)(MUXOUT_DLOCK);		// Mux=Digital Lock
	TCNT1 = 0;
		
	_delay_ms(1);		// Allow signal to rise
		
	// Should be in locked state from startup
	// TCNT0 will increment if lock is lost ant any time
	lock = ISSET(LOCKINPUT);

	diag_ON(); diag_OFF();		// 750ms delay
		
	if ((i=TCNT1) != 0 || !lock) {
		PRINT("\n LOCK FAIL: Count =%u\n", i);
		err++;	SET(AUXLED);
	}
		
	if (!err)
		PRINT("OK");
	else {
		SET(AUXLED);  diag_OFF(); diag_OFF();	// Show error for extra time
	}
	putchar('\n');
		
	//
	// Disable Timer1 interrupts.
	TIMSK1 = 0;
	(void)check_pll_locked();		// Update Locked LED
}


FILE usart = FDEV_SETUP_STREAM(USART_putchar, USART_getchar,_FDEV_SETUP_RW);
#define CMDLEN   50

// Timer0 is used to produce regular timer interrupts at the rate of n per second
//
//  If rate == 0 the timer is disabled
//
void init_timer(uint8_t rate) {
	if (rate) {
		TCNT2 = 0;
		OCR2A = F_CPU/1024/rate;
		TCCR2A = _BV(WGM21);				// CTC Mode
		TCCR2B = _BV(CS22) | _BV(CS21) |_BV(CS20);		// clkIO / 1024 (8KHz)
		TIMSK2 = _BV(OCIE2A);		// Interrupt on compare match 
	} else {
		TCCR2B = 0;		// Disable Timer0
		OCR2A = 0;
		TIMSK2 = 0;
	}
}

void every_1sec() {
#if _ADF4351
	uint8_t  pll_locked, extref_ok, next_state;
	uint16_t reflevel;
#endif
	
	
#if _ADF5355
	// Update PLL Locked LED
	(void) check_pll_locked();
	return;
#else
	// Check PLL is currently locked, and there has been no loss of lock in last
	// 1 second period
	pll_locked = check_pll_locked();
	
	if (rf.rf_freq < 30)
		return;
	
	// Check for presence of external reference by checking RF level 		
	reflevel = adc_read(ADC_EXTREFLEVEL);
	extref_ok = (reflevel > cf.extlevel) ? 1: 0;
	DEBUG(REF>4, "ExtLevel=%d ok=%d", reflevel, extref_ok);
	
	next_state = reference_state;
	tick4 = 0;
		
	switch (reference_state) {
	case REF_INTERNAL:	
			if (extref_ok) 
				next_state = REF_EXTWAIT;
			extref_count = 0;
			if (~rf.mode & MODE_BEACON_BIT)		// LO Mode only
				CLR(AUXLED);
			break;
			
	case REF_EXTWAIT:
		if (extref_ok) {
			if (extref_count++ > 4) {	// 4 seconds before switching to external ref
				next_state = REF_EXTERNAL;
			} else {
				auxled_pattern = 0b0000;	// Flash AUX for 250ms in "settling" state
				tick4 = t1sec/4;	
			}
			if (~rf.mode & MODE_BEACON_BIT)		// LO Mode only
				SET(AUXLED);		// External 
		} else
			next_state = REF_INTERNAL;	// Lost external ref signal - use internal ref
		break;
		
	case REF_EXTERNAL:
		if (!pll_locked)	next_state = REF_EXTFAIL;
		if (!extref_ok)		next_state = REF_INTERNAL;	// Lock may have been lost
		if (~rf.mode & MODE_BEACON_BIT)		// LO Mode only
				SET(AUXLED);
		break;
	
	case REF_EXTFAIL:
		if (!extref_ok)		next_state = REF_INTERNAL;	// Lock may have been lost
		if (~rf.mode & MODE_BEACON_BIT)	{	// LO Mode only
				SET(AUXLED);
				auxled_pattern = 0b01010;	// Fast Flash = waiting for ext ref to be removed
				tick4 = t1sec/4;
		}
		break;
	}	

	//TCNT0 = 0;			// Reset lock loss counter
	
	// Update for new state 
	if (next_state != reference_state) {
		DEBUG(REF>2, "Ref %d -> %d", reference_state, next_state);
		
		switch (next_state) {
		case REF_INTERNAL:
			DEBUG(REF>1, "REF_INTERNAL ");
			clkmode = 0;	// Internal
			CLR(AUXLED);	SET(REFSEL);	SET(OSCPOWER);
			set_freq(rf.rf_freq);		// reprogram PLL for new reference
			//TCNT0 = 0;					// Reset loss of lock counter
			break;
		case REF_EXTERNAL:
			DEBUG(REF>1, "REF_EXTERNAL ");
			clkmode = 1;	// External
			SET(AUXLED);	CLR(REFSEL);	CLR(OSCPOWER);
			set_freq(rf.rf_freq);		// reprogram PLL for new reference
			//TCNT0 = 0;					// Reset loss of lock counter
			break;
		case REF_EXTFAIL:
			DEBUG(REF>1, "REF_EXTFAIL ");
			break;
		case REF_EXTWAIT:
			DEBUG(REF>1, "REF_EXTWAIT ");
			break;
		}

		_delay_ms(10);		// Should lock in 10ms or less
		//TCNT0 = 0;
		pll_locked = check_pll_locked();	// Update Lock LED
		
		reference_state = next_state;
	}
#endif
}


int
main(void) {
	uint8_t  key;
	char cmd;
	uint8_t  show_prompt;
	char cmdbuf[CMDLEN+1], *cmdp;
	uint8_t  ch;

	wdt_enable(WDTO_2S);
	wdt_reset();

	// OSCCAL = 0xAD;
	
	// LED Output pins
	OUTPUT(LOCKLED);
	OUTPUT(AUXLED);
	OUTPUT(LED1);
	OUTPUT(LED2);
	
	OUTPUT(PIN_TXD);
	INPUT(LOCKINPUT);
	
	OUTPUT(OSCPOWER);		// TCXO oscillator power (MIC2090)
	SET(OSCPOWER);			// Enable TCXO power
	
	OUTPUT(REFSEL);			// Signal to ADG719 PLL Reference switch
	SET(REFSEL);			// 1=Internal reference
	
	PORTC |= 0x0f;			// Channel select pins input with pullups		
	DDRC &= ~0x0f;
	
	eeprom_increment((uint32_t *)EEPROM_PWRON);
	_delay_ms(50);

	//--------------------------------------------------------------------------
#if PWM_ENABLE
	PWM_init();
#endif
	clkmode = 0;

	// --------------------------------------------------------------------------
	//  Serial port configuration
	USART_init((uint16_t)(F_CPU/(BAUD_RATE*16L)));
	stdout = stdin = &usart;
	// --------------------------------------------------------------------------

	PRINTLN("ZLPLL Local Oscillator Rev " VERSION);
	PRINTLN("(C)2008 - 2022 Wayne Knowles ZL2BKC, 2023 - 2025 Simon Eatough ZL2BRG");

	
	// --------------------------------------------------------------------------
	//  SPI Interface to ADF435x PLL Chip
	// --------------------------------------------------------------------------	
	SPI_Init();

	cw_start(MSGTYPE_EEPROM, (uint8_t *)EEPROM_TEXT1, 20);
	opdelay = intcnt = 0;		// CW Off for now
	
	// --------------------------------------------------------------------------
	//  Initialize LCD Interface over I2C
	// --------------------------------------------------------------------------
	lcd_active = 0; //lcdInit();
	wdt_reset();
	if (lcd_active) {
		lcdPrintString_P(PSTR("ZLPLL Rev " VERSION));
		wdt_reset();
		PRINTLN("LCD Found (addr=0x%X)", lcd_active);
	}

	load_config();	
	dial_init();
	menu_init();
#if PWM_ENABLE
	PWM_set(cf.pwmval, cf.pwmtop);
#endif

	rf.level = 3;
	load_freq = cmd = 0;

	reference_state = REF_INTERNAL;

	channel = chan_read(PIN_CHANSW);
	
	cmd_channel(channel);		// Load default frequencies etc.
	rf.mode = MODE_LO_RFOFF;	// Mute RF at power up while diags run
	set_freq(rf.rf_freq);
	
	I2C_Init(cf.i2c_addr);
		
	sei();
	
	//  We cant test PLL If it is setup for external control	
	if (rf.rf_freq >= 30.0 ) {
		pll_test();
	}

#if PWM_ENABLE	
	PWM_init();					// Timer1 has been reconfigured by pll_test()
	PWM_set(cf.pwmval, cf.pwmtop);
#endif
	
	// Timer interrupts using TimerCounter 1
	// (shared with PWM function setup above)
	//--------------------------------------------------------------------------
	ticks = t1sec = 100;		// 
	tick4 = 0;					// Disable AUXLED updating
	init_timer(ticks);
//	TIMSK1 = _BV(OCIE1A); 		// Enable timer interrupt
	//--------------------------------------------------------------------------

	cmd_channel(channel);		// Reload this time with RF enabled		
	set_freq(rf.rf_freq);
	//TCNT0 = 0;		
	
	cmdp = cmdbuf;
	show_prompt=1;
	softint_timer = softint_1sec = 0;
		
	// Display frequency and level on LCD (if active)
	if (lcd_active) {
		lcdClear();
		sprintf_P(cmdbuf, PSTR("%lu MHz"), rf.rf_freq);
		lcdPrintString(cmdbuf);
		lcdGotoXY(0,1);
		sprintf_P(cmdbuf, PSTR("Level: %d"), rf.level);
		lcdPrintString(cmdbuf);	
	}

	I2C_Start();
	
	while (1) {
		wdt_reset();
		_delay_us(50);				// Workaround Power EMI issue 	
		if (softint_timer) {
			timer_callback();
			softint_timer--;
		} else
		if (softint_1sec) {
			every_1sec();			// health checks every second
			softint_1sec--;
			
			ch = chan_read(PIN_CHANSW);
			if (ch != channel) {
				channel = ch;
				DEBUG(CHAN, "Channel=%d", channel);
				cmd_channel(channel);
				set_freq(rf.rf_freq);
			}
		}

		//  Only poll encoder if LCD is detected.  Reduces potential EMI issues
		if (lcd_active) {
			key = dial_read();
			if (key)
				menu_event(key);
		}

		if (show_prompt) {
			PRINT("> ");
			show_prompt = 0;
		}

		I2C_check();
		
		if (USART_rxready()) {
			key = getchar();
			if (key == '\n' || key == '\r') {  // Newline
				putchar('\n');
				*cmdp = (char)0;
				// Sanitize input buffer before executing the command
				for (char *p = cmdbuf; *p; ++p) {
					if (!isprint((unsigned char)*p) && *p != ' ') {
						PRINTLN("Invalid input detected. Command aborted.");
						cmdp = cmdbuf; // Reset buffer
						show_prompt = 1;
						return;
					}
				}
				exec_command(cmdbuf);
				show_prompt = 1;		// Display prompt
				cmdp = cmdbuf;
			} else if ((key == (char)0x08 || key == (char)0x7F) && cmdp > cmdbuf) { // Backspace
				cmdp--; // Move pointer back
				PRINT("\b \b"); // Erase last character on screen
			} else if (key == (char) 3) { // ^C
				cmdp = cmdbuf;
				PRINTLN("*CANCEL*");
				show_prompt = 1;
			} else if (cmdp < &cmdbuf[CMDLEN-1]) {
				*cmdp++ = key;
				putchar(key);
			}
		}
	}
	return 0;
}



const __flash struct zlpll_data default_chanel_config [MAX_CHANNELS]={
	{1022400000,0,0,1,0,0,0,0,0.0,0,0},
	{1195200000,0.0,1,0,0,0,0,0,0,0,0},
	{1173600000,0.0,1,0,0,0,0,0,0,0,0},
	{1264800000,0.0,1,0,0,0,0,0,0,0,0},
	{993600000 ,0.0,1,0,0,0,0,0,0,0,0},
	{1180800000,0.0,1,0,0,0,0,0,0,0,0},
	{561600000 ,0.0,1,0,0,0,0,0,0,0,0},
	{1036800000,0.0,1,0,0,0,0,0,0,0,0},
	{1036800000,0.0,1,0,0,0,0,0,0,0,0},
	{1036800000,0.0,1,0,0,0,0,0,0,0,0},
	{1036800000,0.0,1,0,0,0,0,0,0,0,0},
	{1036800000,0.0,1,0,0,0,0,0,0,0,0},
	{1036800000,0.0,1,0,0,0,0,0,0,0,0},
	{1036800000,0.0,1,0,0,0,0,0,0,0,0},
	{1036800000,0.0,1,0,0,0,0,0,0,0,0}
};


void default_config()
{
	uint8_t channel;
	struct zlpll_data rf;
	for (channel = 0;channel < MAX_CHANNELS;channel++)
	{
		memcpy_P(&rf, &default_chanel_config[channel],sizeof(rf));
		eeprom_write_block(&rf,(const char *) (EEPROM_CHAN + sizeof(rf)* channel) ,	sizeof(rf));
	}
}


	
