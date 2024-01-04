/*
*   Copyright (C) 2008-2022 Wanye Knowles ZL2BKC. 2024 Simon Eatough ZL2BRG
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
//


#if 0  // ZLPLL
#define SWEEP 1
#define CHANNELS 16
#define CW_BEACON 0
#else  // CW Beacon
#define SWEEP 0
#define CHANNELS 1
#define CW_BEACON 1
#endif

#if CHANNELS > 16
#error "CHANNELS must be <= 16"
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "softuart.h"

#define DDR_SPI    	PORTA
#define SPI_DO     	PINA5
#define SPI_SCK    	PINA4
#define PLL_SS     	PINA6  // Rev 3.0 PCB ADF4350

#define PIN_CHANSW	PINA2
#define PIN_REFSEL	PINB2
#define PIN_AUXLED	PINB1

#define MIN_FREQ   35

// Secret number used to reset the hours counter
#define RESET_SECRET 123.997

float  rf_freq;
#if SWEEP
float  step, end_freq, start_freq;
uint16_t time_step;
uint16_t sweep;
#endif

uint8_t level, multiplier;
uint8_t clkmode;
volatile uint8_t extcnt;
volatile uint8_t load_freq;
uint32_t  reg4;

#if CW_BEACON
uint16_t intcnt=100;
uint8_t  cw_bits=1, key_down;
uint16_t dotlen;
uint8_t *mp;
uint8_t cw_mode;
#endif

uint32_t hr_cnt;

#define TIME_1SEC	(28800)  // 1 second of interrupts

void set_freq(float, uint8_t);

struct nvram_data {
    float	rf_freq;
    uint8_t	lev:2;
    uint8_t	mult:6;
};

union param_data {
    uint16_t    v;
    struct {
	/* LSB First */
	uint8_t	cp:4;    // Charge Pump 
	uint8_t	mtld:1;  // Mute til lock detect
	uint8_t spur:1;  // Low Spur Mode=1, Low PN=0
	uint8_t doubler:1; // Disable doubler
    } p;
};

#define EEPROM_PARAMS    80
#define EEPROM_REF_FREQ  84
#define EEPROM_N_FREQ    88
#define EEPROM_EXT_FREQ  92

#define EEPROM_WPM	96	// CW Rate (WPM)
#define EEPROM_MODE	98   	// Beacon operation mode
#define EEPROM_TEXT	100    	// EEPROM location for Beacon text
#define EEPROM_TXTEND	400	// Room for 400 Bytes message

#define EEPROM_HOURS	410
#define EEPROM_PWRON	414

void SPI_MasterInit(void)
{	
    // Setup SPI pins for output
    DDRA |= _BV(SPI_DO) | _BV(SPI_SCK) | _BV(PLL_SS);
    PORTA &= ~_BV(PLL_SS);		// SS = Low
}

void SPI_MasterTransmit(char cData)
{
    USIDR = cData;
    USISR = _BV(USIOIF);	// Clear overflow flag

    // Set SPI mode, Software Clock
    USICR = _BV(USIWM0) |	// 3 Wire SPI Mode
	_BV(USICS1) | _BV(USICLK); // Software clock via USITC

    while(!(USISR & _BV(USIOIF))) {
	// Set SPI mode, clock = Counter/Timer0 Match
	USICR = _BV(USIWM0) |	// 3 Wire SPI Mode
	    _BV(USICS1) | _BV(USICLK) | _BV(USITC); // Toggle software clock
    }	// Wait for transmission
}

void
PLL_Write(uint32_t out) {
    PORTA &= ~_BV(PLL_SS);	// Ensure SS is low
    SPI_MasterTransmit((out>>24) & 0xff); /* MSB */
    SPI_MasterTransmit((out>>16) & 0xff);
    SPI_MasterTransmit((out>>8) & 0xff);
    SPI_MasterTransmit((out) & 0xff);   /* LSB */
    PORTA |= _BV(PLL_SS);	// SS = High - Latch Data
    PORTA |= _BV(PLL_SS);	// Delay
    PORTA &= ~_BV(PLL_SS);	// SS = Low
}

void eeprom_increment(uint32_t *addr) {
    uint32_t val;

    val = eeprom_read_dword(addr);
    val++;
    eeprom_write_dword(addr, val);
}


void
delay_sec(uint16_t s) {
    s *= 10;
    while (s--) {
	_delay_ms(100);
    }	
}

#if CHANNELS > 1
#define ADC_PRESCALER (ADPS2)

int16_t adc_read(uint8_t mux)
{
    uint8_t low;

    ADCSRA = (1<<ADEN) | ADC_PRESCALER;             // enable ADC
    ADMUX = /* REFS1 | REFS0 | */  (mux & 0x1F);    // configure mux input
    ADCSRA = (1<<ADEN) | ADC_PRESCALER | (1<<ADSC); // start the conversion
    while (ADCSRA & (1<<ADSC)) ;                    // wait for result
    low = ADCL;                                     // must read LSB first
    return (ADCH << 8) | low;                       // must read MSB only once!
}

// Read the channel select input and convert the voltage to the 
// closest channel number
static uint8_t KV[] = {255,197,222,177,237,186,208,168,246,191};
int8_t chan_read(uint8_t mux)
{
    uint8_t adcval, i, key, k, off;
    
    adcval = adc_read(mux) >> 2; // Only need 8 bits
    
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
#endif

void crlf()
{
    softuart_putchar('\r');
    softuart_putchar('\n');
}

void drain_tx()
{
    while (softuart_transmit_busy()) {
	; // NOP
    }
}

int GCD(int a, int b)
{
    int c;
    while(1)
	{
	    c = a%b;
	    if (c==0)
		return b;
	    a = b;
	    b = c;
	}
}

void
print_int(const char *s, uint16_t num)
{
    char buf[6];

    if (s)
	softuart_puts_p(s);    
    softuart_puts(itoa(num, buf, 10));
}

void
print_float(const char *s, float num)
{
    char buf[16];
    if (s)
	softuart_puts_p(s);    
    softuart_puts(dtostrf(num, 9, 3, buf));
}

#if CW_BEACON

void key_cw(uint8_t offon) {
    if (offon) {
	PORTB |= _BV(PIN_AUXLED); /* AUX LED:  On */
	PLL_Write(reg4|(1L<<5));
    } else {
	PORTB &= ~_BV(PIN_AUXLED); /* AUX LED: Off */
	if (cw_mode > 0)
	    PLL_Write(reg4&~(1L<<5));
    }	
}

#define KEY_OFF   (PORTB &= ~_BV(0))
#define KEY_ON    (PORTB |= _BV(0))

struct t_mtab { char c, pat; } ;
struct t_mtab morsetab[] = {
    {'^', 255},  // Carrier on for 1 second
    {'_', 254},  // Carrier off for 1 second

    {'.', 106},
    {',', 115},
    {'?', 76},
    {'/', 41},
    {'A', 6},
    {'B', 17},
    {'C', 21},
    {'D', 9},
    {'E', 2},
    {'F', 20},
    {'G', 11},
    {'H', 16},
    {'I', 4},
    {'J', 30},
    {'K', 13},
    {'L', 18},
    {'M', 7},
    {'N', 5},
    {'O', 15},
    {'P', 22},
    {'Q', 27},
    {'R', 10},
    {'S', 8},
    {'T', 3},
    {'U', 12},
    {'V', 24},
    {'W', 14},
    {'X', 25},
    {'Y', 29},
    {'Z', 19},
    {'1', 62},
    {'2', 60},
    {'3', 56},
    {'4', 48},
    {'5', 32},
    {'6', 33},
    {'7', 35},
    {'8', 39},
    {'9', 47},
    {'0', 63}
} ;

#define N_MORSE  (sizeof(morsetab)/sizeof(morsetab[0]))

// CW Rate is 1200/W in ms
//
// Interrupt rate = baudrate * 3, ie 28800 for 9600 baud
//
// 1ms = 28.8 ticks, therefore interrupts = 28.8*1200 / W
//

#endif

// Interrupt recieved whenever a signal is detected on the external
// reference input.
ISR(PCINT0_vect)
{
    if (PINA & _BV(7)) {
	if (extcnt++ >= 5) {
	    PCMSK0 = 0;       //  We have seen enough
	}
    }
}

#if CW_BEACON

uint8_t get_next() {
    uint8_t c;

    c = eeprom_read_byte(mp);
    mp++;
    if (c == (char)0) {  // Wrap message
	mp = (uint8_t *)EEPROM_TEXT;
	c = eeprom_read_byte(mp);
	mp++;
    };
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
 loop:
    while (c=='[') {
	c2=get_next();
	if (c2 == '^' || c2 == '_') {
	    if (c2 == '^') { c2 = _BV(PINA2); }
	    else { c2 = 0; }
	    if (c2 != (PINA & _BV(PINA2))) {
		// Read up to closing ] 
		c=get_next();
		while (c != '[' && c != ']')
		    c = get_next();
	    }
	} 
	else if (c2 == '0') {
	    PORTA &=  ~_BV(PA1);   // TX Pin = Lo
	}
	else if (c2 == '1') {
	    PORTA |=  _BV(PA1);    // TX Pin = High
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

// Timer callback interrupt, called from soft serial timer interrupt routine
void timer_callback()
{
    if (++hr_cnt > TIME_1SEC*60L*60L) {
	eeprom_increment((uint32_t *)EEPROM_HOURS);
	hr_cnt = 0;
    }

#if SWEEP
    if (sweep && --sweep == 0) {
	rf_freq += step;
	if (rf_freq > end_freq) {
	    rf_freq = start_freq;
	    PORTA &= ~_BV(PIN_CHANSW);
	} else {
	    PORTA |= _BV(PIN_CHANSW);
	}	    
	load_freq = 1;

	sweep = time_step * 28; // 28 interrupts = 1ms
    }
#endif
#if CW_BEACON
    if (intcnt-- == 0) {
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
		    intcnt = 7*dotlen;  // Send a space
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
		    intcnt = 2*dotlen; // DOT duration has already passed
	    }
	}
    }
#endif
}


void 
set_freq(float freq, uint8_t fast_load)
{
    uint16_t INT;
    uint16_t FRAC;
    uint16_t MOD;
    uint8_t  RDIV = 1;
    uint16_t gcd;
    uint8_t  div;
    uint8_t  refdbl = 0;
    float    ref;
    float    chan;
    float    rem;
    union param_data params;
#if CW_BEACON
    uint8_t cw_wpm;
#endif
    
    freq = freq / multiplier;
    chan = eeprom_read_float((void *)EEPROM_N_FREQ)/multiplier;
    ref = clkmode ? eeprom_read_float((void *)EEPROM_EXT_FREQ) : 
	eeprom_read_float((void *)EEPROM_REF_FREQ);
    params.v = eeprom_read_word((const uint16_t *)EEPROM_PARAMS);

    // PFD frequency must be less than 32MHz.  If fed with a low reference
    // frequency (eg 10MHz) then enable the reference frequency doubler
    // which generally results in lower phase noise
    if (!params.p.doubler && ref < 16) {
	refdbl = 1;
	ref = ref * 2;
    }
    // If using a high reference frequency (>32MHz) then divide it down
    // to ensure the PFD frequency is on target.
    // Note the PFS can use a max of 100MHz for INT-N frequencies
    // but given our desires to sythesize frequencies in around 1MHz steps
    // we are not considering this possability even if it improves the PN
    // by 6dB
    if (ref > 32) {
	RDIV = ref / 25;	/* Calculate divider to achieve ~25MHz */
	while (ref/RDIV > 32)
	    RDIV++;
	ref = ref / RDIV;
    }
    //print_float(PSTR("R="), ref); crlf();
    //print_int(PSTR("rdiv="), RDIV); crlf();

    if (freq < MIN_FREQ)		/* ADF4350  */
	freq = MIN_FREQ;
	
    // VCO frequency must be between 2200-4400MHz, so calculate a suitable
    // divider value to generate the target frequency 
    for (div=0; freq < 2200; div++) {
	freq *= 2;
    }

    INT = (uint16_t)(freq/ref);
    rem = freq - (INT*ref);

    MOD = ref/chan;
    // Round down Frac-N to ensure is below the ADF435x limit of 4096
    while (MOD >= 4096) {
	MOD>>=1;
    }
    FRAC = (MOD*rem)/ref+0.5;
    
    gcd = GCD(MOD,FRAC);
    MOD = MOD/gcd;
    FRAC = FRAC/gcd;

    //print_float(freq); crlf();
    //print_int(MOD); crlf();
    //print_int(FRAC); crlf();

    if (!fast_load) {
	PLL_Write( 0x5 | (1L<<22) | (3L<<19) );

	reg4= 0x4 |((level&0x3)<<3)|
	    (1L<<5)|
	    ((uint32_t)params.p.mtld<<10)|
	    (255L<<12)|
	    (1L<<23)|
	    ((uint32_t)div<<20);
	PLL_Write(reg4);
	PLL_Write( 0x3 | 0L );
	PLL_Write( 0x2 |
		   (6L << 26) |	/* MuxOut = Digital Lock Detect  */
		   ((uint32_t)refdbl << 25) | /* Ref Doubler */
		   (0L << 8)|	/* LDF */
		   ((uint32_t)RDIV<<14) | /*  */
		   /*  (1L<<25)|(1L<<1) | */
		   (1L << 6) |	/* +ve PD Polarity */
		   ((uint32_t)params.p.cp<<9) |	/* Charge Pump Current */
		   (0L << 24) |	/* Ref Div2 */
		   ((params.p.spur?3L:0L) << 29));	/* Low Noise Mode */
    }
    PLL_Write( 0x1 | ((uint32_t)MOD << 3)|(1L<<27)|(1L<<15) );
    PLL_Write( 0x0 | ((uint32_t)INT << 15)|((uint32_t)FRAC << 3));

#if CW_BEACON
    cw_mode = eeprom_read_byte((uint8_t *)EEPROM_MODE);
    cw_wpm = eeprom_read_byte((uint8_t *)EEPROM_WPM);
    if (cw_wpm < 5 || cw_wpm > 50) 
	cw_wpm = 12;
    dotlen = (34560/cw_wpm);
#endif
}

void exec_command(uint8_t command, float arg)
{
    struct nvram_data eedata;
    union param_data params;

    uint16_t  intarg = (uint16_t)arg;
#if CW_BEACON
    uint8_t key, *ptr;    
#endif

    switch (command) {
    case 'a':
    case 'A':  // A1 Enable External refernce, A0 = Internal Reference
	// A2 = External reference with no auto select
	if (intarg) {
	    PORTB |= _BV(PIN_AUXLED); /* External Ref */
#if v3_1 // Old Revision
	    PORTB |= _BV(PIN_REFSEL); /* External Ref */
#else
	    PORTB &= ~_BV(PIN_REFSEL); /* External Ref */
#endif
	    clkmode = intarg;
	} else {
	    PORTB &= ~_BV(PIN_AUXLED); /* Internal Ref */
#if v3_1 // Old
	    PORTB &= ~_BV(PIN_REFSELLED); /* Internal Ref */
#else
	    PORTB |= _BV(PIN_REFSEL); /* Internal Ref */
#endif
	    clkmode = 0;
	}
	break;

	    /* Change to specified channel */
    case 'c':
    case 'C':  eeprom_read_block(&eedata, 
				 (char *)(sizeof(eedata)*intarg), sizeof(eedata));
	level = eedata.lev;
	multiplier = eedata.mult;
	rf_freq = eedata.rf_freq;
#if SWEEP
	start_freq = rf_freq;
	end_freq = 0;		/* Disable sweep mode if active */
#endif
	break;
	/* Set Frequency */
    case 'F':
    case 'f':  
	rf_freq = arg;
#if SWEEP
	start_freq = rf_freq;
#endif 
	break;
#if SWEEP
    case 'G':
    case 'g':  
	end_freq = arg;
	break;
    case 'T':
    case 't':
	time_step = arg;
	break;
	/*  Set frequency Increment */
    case 'i':
    case 'I':
	step = arg;
	return;
    case '+':	
	rf_freq += step;
	break;
    case '-':	
	rf_freq -= step;
	break;
#endif

    /* Save Frequency and Power level to specified channel register */
    case 'S':
    case 's':
	eedata.rf_freq = rf_freq;
	eedata.lev = level;
	eedata.mult = multiplier;
	if (arg >=0 && arg <16) {
	    eeprom_write_block(&eedata, 
			       (char *)(sizeof(eedata)*intarg), sizeof(eedata));
	}
	return;
	/* Display configuration */
    case 'd':
    case 'D':
	{
	    print_float(PSTR("RF="), rf_freq);
	    print_int(PSTR(", L="), level);
	    if (multiplier > 1)
		    print_int(PSTR(", M="), multiplier);
	    crlf();
	    print_float(PSTR("Ref="), eeprom_read_float((void *)EEPROM_REF_FREQ));
	    print_float(PSTR(", N="), eeprom_read_float((void *)EEPROM_N_FREQ));
	    crlf(); 
	    print_float(PSTR("ExtRef="), eeprom_read_float((void *)EEPROM_EXT_FREQ));
	    crlf(); 
	    params.v = eeprom_read_word((const uint16_t *)EEPROM_PARAMS);
	    print_int(PSTR("Params="), params.v);
	    print_int(PSTR(", CP="), params.p.cp);
	    crlf(); 
#if CW_BEACON
	    softuart_puts_P("Text=");
	    ptr = (uint8_t *) EEPROM_TEXT;
	    key = eeprom_read_byte(ptr++);
	    while (key) {
		softuart_putchar(key);
		key = eeprom_read_byte(ptr++);
	    }
	    crlf();
	    print_int(PSTR("Speed="), eeprom_read_byte((uint8_t *)EEPROM_WPM));
	    softuart_puts_P(" WPM");
	    crlf();
	    print_int(PSTR("Mode="), eeprom_read_byte((uint8_t *)EEPROM_MODE));
	    crlf();
#endif
	    crlf();
	
	    for (command=0; command < CHANNELS; command++) {
		eeprom_read_block(&eedata, 
				  (char *)(sizeof(eedata)*command),
				  sizeof(eedata));
		print_int(NULL, command);
		print_float(PSTR(" RF="), (float)eedata.rf_freq);
		print_int(PSTR(", L="), eedata.lev);
		if (eedata.mult > 1)
		    print_int(PSTR(", M="), eedata.mult);
		crlf();
	    }
	}
	return;
	/* Set Frequency */
    case 'r':
    case 'R':
	eeprom_write_float((void *)EEPROM_REF_FREQ, arg);
	break;
    case 'e':
    case 'E':
	eeprom_write_float((void *)EEPROM_EXT_FREQ, arg);
	break;
	/* Set fractional-N step frequency */
    case 'n':
    case 'N':
	eeprom_write_float((void *)EEPROM_N_FREQ, arg);
	break;
	/* Set output level */
    case 'l':
    case 'L':
	level = arg;
	break;
    case 'm':
    case 'M':
	multiplier = arg?arg:1;  // Cannot be 0
	break;
	/* Set Parameter */
    case 'p':
    case 'P':
	params.v = (uint16_t)arg;
	eeprom_write_block(&params, (char *)EEPROM_PARAMS, sizeof(params));
	break;
#if CW_BEACON
    case 'h':
    case 'H':
	if (intarg > 0) {
	    eeprom_write_dword((uint32_t *)EEPROM_PWRON, 0L);
	    eeprom_write_dword((uint32_t *)EEPROM_HOURS, (uint32_t)intarg);
	}
	print_int(PSTR("PowerCycles:"), 
		  eeprom_read_dword((uint32_t *)EEPROM_PWRON));
	crlf();
	print_int(PSTR("Hours:"), eeprom_read_dword((uint32_t *)EEPROM_HOURS));
	crlf();
	break;

    // Beacon CW Rate in WPM
    case 'w':
    case 'W':
	eeprom_write_byte((uint8_t *)EEPROM_WPM, (uint8_t)intarg);
	break;
    // Beacon Mode
    //  B0 = Continuous carrier
    //  B1 = Keyed Carrier
    case 'b':
    case 'B':
	eeprom_write_byte((uint8_t *)EEPROM_MODE, (uint8_t)intarg);
	break;
    case 't':
    case 'T':
	ptr = (uint8_t *) EEPROM_TEXT;
	softuart_puts_P("Text: ");
	while (!softuart_kbhit()) {
	    wdt_reset();
	}
	key = softuart_getchar() & 0x7f;
	while (key != '\n' && key != '\r' && ptr < (uint8_t *)EEPROM_TXTEND) {
	    if (key >= 'a' && key <= 'z')
		key -= 32; // lower case -> upper case
	    softuart_putchar(key);
	    eeprom_write_byte(ptr, key);
	    ptr++;
	    while (!softuart_kbhit()) {
		wdt_reset();
	    }
	    key = softuart_getchar() & 0x7f;
	}
	eeprom_write_byte(ptr, (uint8_t) 0);
	softuart_puts_P("\n\rOK\n\r");
	mp = (uint8_t *)EEPROM_TEXT;
	intcnt = dotlen;
	break;
#endif
    case ' ':
    case '\n':
    case '\r':				
	return;
    default:
	softuart_puts_P("*ERR*\n\r");
	return;
    }
    set_freq(rf_freq, 0);
#if SWEEP
    if (end_freq > 0 && time_step > 0 && step > 0 && end_freq > rf_freq) {
	sweep = time_step * 28; // 28 interrupts = 1ms
	DDRA |= _BV(PIN_CHANSW);     // Port A2 becomes trigger signal
	PORTA &= ~_BV(PIN_CHANSW);
    } else {
	sweep = 0;
	DDRA &= ~_BV(PIN_CHANSW);     // Port A2 becomes channel select again
    }
#endif
}

int
main(void) {
    uint8_t  key;
    uint8_t  chan, prev_chan;
    float    num, dp;
    char cmd;

    wdt_enable(WDTO_500MS);
    wdt_reset();

    eeprom_increment((uint32_t *)EEPROM_PWRON);

    _delay_ms(50);

    DDRA &= ~0x0F | _BV(7);		/* PINA0 to PINA3 = channel select */
    PORTA |= 0x0F;		/* Enable internal pullups */

    // PINB1 = AUX LED, PINB2=Ref Select

    DDRB |= _BV(PIN_AUXLED) | _BV(PIN_REFSEL); 
    PORTB = _BV(PIN_REFSEL); /* External Ref */
#if v3_1    
    PORTB = 0; /* Internal Ref */
#endif    
    clkmode = 0;

#if CW_BEACON
    mp = (uint8_t *)EEPROM_TEXT;
    DDRA &= ~_BV(PIN_CHANSW);  // Channel switch is TTL level input
#endif

    SPI_MasterInit();
    
    softuart_init();
    softuart_turn_rx_on(); /* redundant - on by default */
    sei();

    softuart_puts_P("\rZLPLL "
#if CW_BEACON
		    "CW Beacon"
#else
		    "Universal Local Oscillator"
#endif
		    " Rev3.1B\n\r"
		    "(C)2012 W. Knowles, ZL2BKC\n\r");
    
#if SWEEP
    step = 1;
    end_freq = 0.0;
#endif

    level = 3;
    load_freq = cmd = 0;
    dp = num = 0.0;
    
    prev_chan = 255; // Out of range to force load of first value

    GIMSK |= _BV(PCIE0);   // enable PCINT interrupt 

    while (1) {
	wdt_reset();

	drain_tx();	       // Prevents corruption to transmitted char
			       // when checking for external clock
	
	extcnt = 0;
	PCMSK0 = _BV(7);       //  Check for external clock

	if (~PINA & _BV(3)) { 
	    set_freq(rf_freq, 0); // Lock Lost, so attempt to reprogram PLL
#ifdef DEBUG	    
	    softuart_putchar('.');
#endif
	}
#if SWEEP
	if (sweep) {
	    if (load_freq) {
		load_freq = 0;
		set_freq(rf_freq, 1);
	    }
	} else
#endif
	    {
#if CHANNELS > 1	     
	    chan = chan_read(PIN_CHANSW);
#else
	    chan = 0;
#endif

	    DDRA &= ~_BV(1);  // Input (with pullup) in order to check if pulled low
	    _delay_ms(20);
	    if (~PINA & _BV(1)) { 
		chan ^= 0x8; // If alt frequency mode then add 8 to the channel number
	    }
	    DDRA |= _BV(1);  // Output again...

	    if (chan != prev_chan) {
		exec_command('C', chan);
		prev_chan = chan;
	    }
	}

	if (extcnt && clkmode == 0) { 
	    // Using internal clock and external clk has been seen
	    exec_command('A', 1);
#ifdef DEBUG	    
	    softuart_putchar('E');
#endif
	}
	if (!extcnt && clkmode == 1) {  // Using external clock and lock is lost
	    exec_command('A', 0);
#ifdef DEBUG
	    softuart_putchar('I');
#endif
	}

	if (softuart_kbhit()) {
	    key = softuart_getchar() & 0x7f;
	    if (key >= '0' && key <= '9') { //Numeric
		if (dp == 0.0)
		    num = num*10.0 + key-'0';
		else {
		    num += dp*(key-'0');
		    dp /= 10.0;
		}
		softuart_putchar(key);
	    } else if (key == '.') {
		dp = 0.1;   // start of decimal part
		softuart_putchar(key);
	    } else {
		if (key == '\n' || key == '\r') {
		    // Execute the previous command
		    crlf();
		    exec_command(cmd, num);
		} else if (key == '+' || key == '-') {
		    exec_command(key, num);
		} else {
		    softuart_putchar(key);
		    if (key != ' ') {
			cmd = key;
			dp = num = 0.0;
		    }
		}
	    }
	}
    }

    return 0;
}

