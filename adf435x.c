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
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "zlpll.h"
#include "adf435x.h"


#if _ADF4351
// ADF4351 Version
static void
PLL_Write(uint32_t out) {
	uint8_t   reg;

	reg = out&0x7;
	if (reg > 5) {
		// This should not happen!
		UDR0 = '@';	// Signal error via serial port
		return;
	}
	CLR(ADF4351_LE);
	SPI_MasterTransmit((out>>24) & 0xff); /* MSB */
	SPI_MasterTransmit((out>>16) & 0xff);
	SPI_MasterTransmit((out>>8) & 0xff);
	SPI_MasterTransmit((out) & 0xff);   /* LSB */
	SET(ADF4351_LE);	// Pulse LE pin high
	PLL.reg[reg] = out;	// Implicit delay to increase length of LE pulse
	CLR(ADF4351_LE);
	DEBUG(PLL > 4, "PLL_Write(0x%08lx)", out);
}

void
ADF4351_set_freq(float freq)
{
	uint16_t INT;
	uint16_t FRAC;
	uint16_t MOD;
	uint8_t  RDIV;
	//uint16_t gcd;
	uint8_t  div;
	uint8_t  refdbl = 0;
	float    ref, pfd, intf;
	float    rem;
	
	float besterr;
	float error;
	float actual;
	uint16_t tmod;
	uint16_t tfrac;
	uint32_t reg4;
	
#if CW_BEACON
	uint8_t cw_wpm;
#endif

	freq = freq / rf.mult;

	if (freq < ADF4351_MIN_FREQ)
		freq = ADF4351_MIN_FREQ;

	ref = clkmode ? cf.ext_ref : cf.int_ref;

	RDIV = rf.rdiv;

	// PFD frequency must be less than 32MHz.  If fed with a low reference
	// frequency (eg 10MHz) then enable the reference frequency doubler
	// which generally results in lower phase noise
	
	if (cf.max_doubler && ref*2 < cf.max_doubler) {
		refdbl = 1;
		ref = ref * 2;
	}
	// If using a high reference frequency (>32MHz) then divide it down
	// to ensure the PFD frequency is on target.
	// Note the PFS can use a max of 100MHz for INT-N frequencies
	// but given our desires to synthesize frequencies in around 1MHz steps
	// we are not considering this possibility even if it improves the PN
	// by 6dB
	if (ref > 32) {
		RDIV = ref / 25;	/* Calculate divider to achieve ~25MHz */
		while (ref/RDIV > 32)
		RDIV++;
	}


	// VCO frequency must be between 2200-4400MHz, so calculate a suitable
	// divider value to generate the target frequency
	for (div=0; freq < 2200; div++) {
		freq *= 2;
	}

	#if 1 // New way
	pfd = ref / RDIV;
	

	INT = (uint16_t)(freq/pfd);
	intf = INT * pfd;			// intf = integer-N frequency

	rem = freq - intf;			// frac = remainder which needs to be generated
	// via fractional-N
	besterr = pfd;				// High value so it will always reduce
	tmod = 0;
	tfrac = 0;
	MOD=10;						// Default values to keep compiler happy
	FRAC=0;
	
	if (rem < 64)
	rem = (uint16_t)(rem*1000.0+0.5)/1000.0;		//  Cleanup error from single prec math
	DEBUG(PLL>=4, "ADF4351: Remainder=%f", rem);
	
	for (tmod=1; tmod<cf.maxmod; tmod++)  {
		tfrac = (int16_t) (rem/pfd*tmod);
		actual = (float)(tfrac * pfd)/(float)tmod;
		error = rem - actual;
		if (error < besterr) {
			besterr = error;
			MOD = tmod;
			FRAC = tfrac;
		}
		if (error < 1e-6)
		break;			// Stop searching if less than 1Hz
	}
	PLL.freqerr = besterr;
	
	#else
	chan = rf.step / rf.mult / 1000000.0;

	ref = ref / RDIV;
	//
	// Calculate the modulus to achieve the desired channel spacing
	//
	MOD = ref/chan;
	// Round down Frac-N to ensure is below the ADF435x limit of 4096
	while (MOD >= 4096) {
		DEBUG(PLL>1, "MOD Overflow: %u", MOD);
		MOD>>=1;
	}

	INT = (uint16_t)(freq/ref);
	rem = freq - (INT*ref);

	FRAC = (MOD*rem)/ref+0.5;
	//
	// Catch case where roundup to closest frequency requires the
	// next integer divisor
	//
	if (FRAC == MOD) {
		FRAC=0;
		INT++;
	}
	
	// To improve phase noise the lowest possible modulus should be selected.   Calculate greatest
	// common divisor to determine and reduce modulus to minimum acceptable value.
	if (cf.gcd) {
		gcd = GCD(MOD,FRAC);
		MOD = MOD/gcd;
		FRAC = FRAC/gcd;
	}
	#endif
	
	DEBUG(PLL, "ADF4351 Freq=%f, Ref=%.3f, RDIV=%d, MOD=%d, FRAC=%d",
	freq, ref, RDIV, MOD, FRAC);

	reg4= 0x4 |((rf.level&0x3)<<3) |
				((uint32_t)(cf.mtld&1)<<10) |
				(255L<<12) |
				(1L<<23) |
				((uint32_t)div<<20);

#if CW_BEACON
	if (rf.mode & MODE_RFON_BIT)
#endif
		reg4 |= (1L << 5);

	PLL_Write( 0x5 | (1L<<22) | (3L<<19) );
	PLL_Write(reg4);
	PLL_Write( 0x3 | 0L );
	PLL_Write( 0x2 |
					(6L << 26) |	/* MuxOut = Digital Lock Detect  */
					((uint32_t)refdbl << 25) | /* Ref Doubler */
					(0L << 8)|	/* LDF */
					((uint32_t)RDIV<<14) | /*  */
					/*  (1L<<25)|(1L<<1) | */
					(1L << 6) |	/* +ve PD Polarity */
					((uint32_t)(cf.charge_pump&0xf)<<9) |	/* Charge Pump Current */
					(0L << 24) |	/* Ref Div2 */
					((cf.spur?3L:0L) << 29));	/* Low Noise Mode */

	PLL_Write( 0x1 | ((uint32_t)MOD << 3)|(1L<<27)|(1L<<15) );
	PLL_Write( 0x0 | ((uint32_t)INT << 15)|((uint32_t)FRAC << 3));

	PLL.freq = freq;
	PLL.INT = INT;
	PLL.FRAC = FRAC;
	PLL.MOD = MOD;
	PLL.RDIV = RDIV;
	PLL.rf_mult = 1;		// Multiplication factor
	PLL.rf_div = 1<<div;		// Division factor
	PLL.ref = ref;
}

void ADF4351_set_muxout(uint8_t v)
{
	PLL_Write((PLL.reg[2] & ~SB(0x7,26)) | SB(v&7,26));
}


void ADF4351_set_rflevel(int8_t enable, int8_t level) {
	uint32_t reg;
	
	reg = PLL.reg[4] & ~SB(1, 5);
	if (enable)
		reg |= SB(1, 5);	// PLL_Write(reg4|(1L<<5));
	PLL_Write(reg);
}

void ADF4351_powerdown() {
	uint32_t reg;
	
	reg = PLL.reg[4] & ~SB(0b00111111, 3);
	reg |= SB(1, 11);		// VCO Power Down = 1
	PLL_Write(reg);	
}

void ADF4351_fsk(uint16_t step, int16_t delta) {
	int32_t frac;
	int32_t intx;

	frac = PLL.FRAC + delta;
	intx = PLL.INT;
	//  Adjust integer divisor if fractional part is now outr of range
	while (frac < 0) {
		intx--;
		frac += PLL.MOD;
	}
	while (frac >= PLL.MOD) {
		intx++;
		frac -= PLL.MOD;
	}
	
	if (PLL.reg[4] & (1UL<<10)) {
		PLL_Write(PLL.reg[4] & ~(1UL<<10));		// Disable MTLD
	}
	
	PLL_Write( 0x0 | ((uint32_t)intx << 15)|((uint32_t)frac << 3));

}

#endif
