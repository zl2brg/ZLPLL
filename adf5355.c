/*
*   Copyright (C) 2017-2022 Wanye Knowles ZL2BKC. 2023 Simon Eatough ZL2BRG
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
#include "adf5355.h"

// ADF5355 Version
static void
PLL_Write2(uint32_t out) {
	uint8_t   reg;

	reg = out&0xF;
	if (reg > 12) {
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

void ADF5355_set_rflevel(int8_t enable, int8_t level) {
	uint32_t reg;

	// 
	// Reg 6
	//   Bit 4-5 = RFA Current for level control  0=Low to 3=High
	//   Bit 6   = RFA_Out control   0=Disable  1=Enable
	//	 Bit 10  = RFB_Out control   0=Enable   1=Disable (negative logic)
	// 	
	reg = PLL.reg[6] & ~SB(0b111, 4);	//  RFA Disabled and level bits cleared (bits 4-6)
	reg |= SB(1, 10);		// RFB Disabled
	
	if (enable) {
		if (PLL.rfA_enable)		
				reg |= SB(1, 6); 	// Enable RFA if low frequency enabled
		if (PLL.rfB_enable)		
				reg &= ~SB(1, 10);	// Enable RFB if high frequency enabled
	}

	reg |= SB(level&0x03, 4);	// Update RF Level
	PLL_Write2(reg);
}

#define MOD1	16777215UL		// Fixed modulus for ADF5355
//#define STEPSZ	4000			// 20.000MHz / 2000 = 10kHz Steps
//			   4000 =  5kHz steps

void ADF5355_set_freq(float freq)
{
	uint16_t INT;
	uint32_t FRAC1;
	uint16_t FRAC2;
	uint16_t MOD2;
	uint8_t  RDIV;
	uint16_t adcdiv, vcodiv, alc_tmo, synth_tmo, timeout;
	uint16_t gcd;
	uint8_t  div, rfB_enable, rfA_enable;
	uint8_t  refdbl = 0;
	float    ref, pfd, intf, frac1f;
	float    rem, rem2;
	float	 freq2;
	float	 STEPSZ;
	uint8_t  intmode, prescaler;
	
	rfB_enable = freq >= 6800 ? 1: 0;
	rfA_enable = !rfB_enable;
	if (rfB_enable)
		freq = freq / 2.0;			// Calculate VCO frequency
		
	freq2 = freq / rf.mult;

	if (freq2 < ADF5355_MIN_FREQ)
		freq2 = ADF5355_MIN_FREQ;

	ref = clkmode ? cf.ext_ref : cf.int_ref;

	if (rf.step < 5000)	rf.step=5000.0;	// FIXME:  Minimum value is 5KHz
	
	RDIV = 1;		//  Reference divisor not needed for ADF5355 in most situations

	// PFD frequency must be less than XXXMHz.  If fed with a low reference
	// frequency (eg 10MHz) then enable the reference frequency doubler
	// which generally results in lower phase noise
	
	if (cf.max_doubler && ref*2 <= cf.max_doubler) {
		refdbl = 1;
		ref = ref * 2;
	}
	
	// Calculate the nearest RDIV value to achieve the target phase detector freq
	RDIV = (int) (ref+cf.pfd_freq/2)/cf.pfd_freq;
	if (RDIV < 0) RDIV=1;			// Cant be zero
	
	STEPSZ = (ref*1e6/rf.step) * (float)rf.mult;
		
	// If using a high reference frequency (>32MHz) then divide it down
	// to ensure the PFD frequency is on target.
	// Note the PFS can use a max of 100MHz for INT-N frequencies
	// but given our desires to synthesize frequencies in around 1MHz steps
	// we are not considering this possibility even if it improves the PN
	// by 6dB


	// VCO frequency must be between 3400-6800MHz, so calculate a suitable
	// divider value to generate the target frequency
	for (div=0; freq2 < 3400; div++) {
		freq2 *= 2;
	}
	
	// If rdiv is > 2 we want RFA and B outputs enabled simultaneously
	// and value indicates the 2^N divisor for the RFA output  
	// Note that rdiv is based on the frequency with the multiplier enabled (>6.8GHz)
	// so rdiv=2 enabled baseband outputt, and rdiv=4 enables a /2 divider etc. 
	if (rfB_enable) {
		if (rf.rdiv >= 2) { 
			for (div = 0; div <= 7; div++) {
				if ((1<<div) >= (rf.rdiv/2))
					break;
			}
			rfA_enable = 1;
		}
	}

	pfd = ref / RDIV;
	
	intf = freq2/pfd;
	INT = (uint16_t)intf;
	rem = intf - INT;			// calc remainder which needs to be generated
	// via fractional-N synthesis
	
	rem = ((uint16_t)(rem*STEPSZ+0.5))/(float)STEPSZ;	// Fix floating point rounding issues
	frac1f = rem*MOD1;

	FRAC1 = (uint32_t)frac1f;
	MOD2 = STEPSZ;
	rem2 = frac1f-FRAC1;
	FRAC2 = rem2*MOD2;

	DEBUG(PLL>7, "ADF5355: intf=%f rem=%f frac1f=%f rem2=%f",
	intf, rem, frac1f, rem2);

	// To improve phase noise the lowest possible modulus should be selected.   Calculate greatest
	// common divisor to determine and reduce modulus to minimum acceptable value.
	if (cf.gcd) {
		gcd = GCD(MOD2,FRAC2);
		DEBUG(PLL>4, "MOD2=%d FRAC2=%d gcd=%d", MOD2, FRAC2, gcd);
		MOD2 /= gcd;
		FRAC2 /= gcd;
	}

	// Different phase detector delay when fractional part is zero
	intmode = (FRAC1 == 0 && FRAC2 == 0) ? 1:0;
	
	DEBUG(PLL, "ADF5355 Freq=%f, pfd=%.3f, ref=%.3f, RDIV=%d",
				freq, pfd, ref, RDIV);
	DEBUG(PLL, "        INT=%d, FRAC1=%ld, FRAC2=%d, MOD2=%d",
				INT, FRAC1, FRAC2, MOD2);

	// Preference to use 4/5 prescaler
	prescaler = 0;
	if (INT < 23)
		DEBUG(PLL, "**INT %d Too Low for prescaler", INT);
	if (INT >= 32768U) prescaler = 1;
	
	// Force External reference
	CLR(REFSEL);	CLR(OSCPOWER);

	PLL_Write2(12	| SB(0xFFFF,16)
					| SB(1,10)
					| SB(1, 4)
					);	// Disable phase resync
	PLL_Write2(11 | 0x0061300BUL);							// As per datasheet page 32

	adcdiv = (pfd*10 - 2)/4+1;
	if (adcdiv > 255) adcdiv = 255;
	DEBUG(PLL>4, "ADC_CLK_DIV=%d", adcdiv);
	PLL_Write2(10	| SB(3, 22)
					| SB(adcdiv, 6)
					| SB(3, 4)
					);		// ADC Control register
	
	vcodiv = 9;			// ceil(pfd/2.4);
	timeout = 34;		// *3 from datasheet
	alc_tmo = 30;
	synth_tmo = 12;
	
	if (vcodiv > 255)  vcodiv = 255;
	DEBUG(PLL>4, "VCO_DIV=%d, TIMEOUT=%d, ALC_TMO=%d, SYNTH_TMO=%d", vcodiv, timeout, alc_tmo, synth_tmo);

	PLL_Write2(9	| SB(vcodiv, 24) | SB(timeout, 14)
					| SB(alc_tmo, 9) | SB(synth_tmo, 4)
					);
	PLL_Write2(8	| 0x102D0428UL);		// Datasheet page 31
	PLL_Write2(7	| SB(1, 28)			// Reserved
					| SB(0, 25)			// LE Sync
					| SB(0, 8)				// Lock Detect count = 1024 cycles
					| SB(1, 7)				// Loss of lock detect enabled
					| SB(3, 5)				// Frac-N Lock Detect 12ms
					| SB(intmode?1:0, 4)	// Lock Detect mode Integer or Frac-N
					);
	PLL_Write2(6	| SB(0xA, 25)			// Reserved bits
					| SB(1, 24)			// Feedback = fundamental
					| SB(div, 21)			// RF Out A divider
					| SB(cf.spur&1, 30)	// Spur reduction = gated bleed
					| SB(cf.bleed?1:0, 29)		// Negative bleed enable
					| SB(cf.bleed&0xff, 13)	// Bleed current (was 5 for 10MHz ref)
					| SB(cf.mtld&1, 11)		// MTLD
					| SB(rfB_enable?0:1, 10)		// RFoutB ON (OFF=1, On=0)
					| SB(rfA_enable?1:0, 6)			// RFOut A ON
					| SB(rf.level&3, 4)	// RF Level
					);
	PLL_Write2(5	 | 0x00800025UL);		// Page 27 Figure 46
	PLL_Write2(4	| SB(MUXOUT_DLOCK, 27)	//  MuxOut = Digital Lock Detect
					| SB(refdbl?1:0, 26)	// Reference Doubler
					| SB(RDIV, 15)			// 10 bit R counter
					| SB(1, 14)				// Enable double buffering
					| SB(cf.charge_pump&0xf, 10) // Charge Pump
					| SB(1, 8)				// 3.3V Logic
					| SB(1, 7)				// +ve PD polarity
					);
	
	PLL_Write2(3	| 0L);					// No phase stuff
	PLL_Write2(2	| SB(FRAC2, 18) | SB(MOD2, 4));   //
	PLL_Write2(1	| SB(FRAC1, 4));
	
	_delay_us(500);  // TODO - Use correct delay for ADC calibration
	
	PLL_Write2(0	| SB(1, 21)			// Enable Autocal
					| SB(prescaler, 20)	//   4/5 or 8/9 prescaler
					| SB(INT, 4));

	PLL.freq = freq2;
	PLL.INT = INT;
	PLL.FRAC1 = FRAC1;
	PLL.FRAC = FRAC2;
	PLL.MOD = MOD2;
	PLL.RDIV = RDIV;
	PLL.rf_mult = rfB_enable+1;		// Multiplication factor 
	PLL.rf_div = 1<<div;		// Division factor
	PLL.ref = ref;
	PLL.rfA_enable = rfA_enable;
	PLL.rfB_enable = rfB_enable;
}

void ADF5355_set_muxout(uint8_t v)
{
	PLL_Write2((PLL.reg[4] & ~SB(0x7,27)) | SB(v&7,27));
}

void ADF5355_powerdown() {
	uint32_t reg;
	
	DEBUG(PLL>2, "ADF5355 Power Down");
	reg = PLL.reg[6] & ~SB(0b01111111, 4);
	reg |= SB(1, 10);		// Disable RFOutB
	PLL_Write2(reg);
	reg = PLL.reg[4];
	PLL_Write2(reg | SB(0b011, 5)); // CP=Tristate, Power Down
}

//
//  FSK modulation of frequency.  
//
//  Still experiencing occasional lock losses when AUTOCAL is disabled (none if enabled)
//  No issues when Loss of Lock mode enabled in register 7. 
//
void ADF5355_fsk(uint16_t step, int16_t delta) {
	int32_t FRAC1;
	uint16_t intN;
	float  stepsz, df1;
	uint16_t MOD2, FRAC2, scale;
	int16_t deltaF1;
	uint8_t	 autocal;
	uint32_t reg;
	
	autocal = ISSET(MUXOUTPIN)?0:1;		//  If unlocked then perform autocal
	
	step = step*(float)PLL.rf_div/(float)PLL.rf_mult;	// Scale step to VCO frequency range
	stepsz = PLL.ref*1.0e6/MOD1;
	df1 = (float)step*delta/stepsz;
	deltaF1 = (int16_t)df1;
	MOD2 = 8000;				// Fixme 
	FRAC2 = (df1-deltaF1) * MOD2;
	scale = MOD2 / PLL.MOD;
	
	//  Remove the GCD calculation
	PLL.MOD = MOD2;
	PLL.FRAC *= scale;

	intN = PLL.INT;
	FRAC1 = (int32_t)PLL.FRAC1 + deltaF1;
	FRAC2 += PLL.FRAC;
		
	if (FRAC2 >= MOD2) {
		FRAC2 -= MOD2;
		FRAC1++;
	}
	if (FRAC1 < 0L) {		// Next integer divisor down
		intN--;
		FRAC1 = ((int32_t)MOD1 + FRAC1);
	}
	if (FRAC1 >= MOD1) {		// Next integer divisor up
		intN++;
		FRAC1 -= MOD1;
	}
	FRAC1 &= 0x00FFFFFFUL;

	//PRINTLN("PLL step=%d, N=%d", step, deltaF1);
	//PRINTLN("      INT=%d  FRAC1=%ld  FRAC2=%d  MOD2=%d", intN, FRAC1, FRAC2, MOD2);

	// Use sigma-delta bit to determine if fsk mode is enabled.  We only need to change
	// the registers once when first entering fsk mode
	reg = PLL.reg[3];
	if (!(reg & SB(1, 30))) {
		DEBUG(PLL>2, "FSK disable sigma-delta");
		//  Disable ADC (reg10) appears to stop random unlocks
		//  Technically these 3 registers only need to be changed once when enter fsk mode
		PLL_Write2(PLL.reg[10]  & ~SB(1, 5));	// Disable ADC Calibration
		PLL_Write2(PLL.reg[7] & ~SB(1, 4));		// Fractional-N Lock detect
		PLL_Write2(PLL.reg[6] & ~SB(1, 11));	// Disable MTLD
		PLL_Write2(PLL.reg[4] & ~SB(1, 4));		// Counter reset disable
		PLL_Write2(PLL.reg[3] | SB(1, 30));		// Disable sigma-delta reset on reload
		autocal = 1;
	}

	//  Update lock detect to either integer to fractional-N
//	reg = (FRAC1 == 0 && FRAC2 == 0) ? PLL.reg[7]|SB(1, 4) : PLL.reg[7] & ~SB(1, 4);
//	if (reg != PLL.reg[7]) {
//		DEBUG(PLL>2, "FSK change lock mode %02x @%d", (uint8_t)reg, delta);
//		PLL_Write2(reg);		// Update register7
//	}

	if (FRAC1 == 0 && FRAC2 == 0)
		FRAC2++;		//  Workaround to prevent integer lock detect issue
	//  
	PLL_Write2(2 | SB(FRAC2, 18) | SB(MOD2, 4)); 
	PLL_Write2(1 | SB(FRAC1, 4));			// FRAC1

	if (autocal)
		_delay_us(500);							// Appears this is still needed

	// XXX: If intN changes the S-D counter resets.  Need to AUTOCAL?
	//      For small steps intN will not change - only in extreme situations
	//		At the integer boundary (ie FRAC=0 and INT needs to change +/- 1)
	PLL_Write2((PLL.reg[0] & SB(1, 20))		// Preserve prescaler setting
				| SB(intN, 4)
				| SB(autocal, 21)			// Reload
			);	// AUTOCAL Disabled
}
