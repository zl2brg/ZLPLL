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
#include <math.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "zlpll.h"
#include "adf5355.h"

// ADF5355 Version
static void
PLL_Write2(uint32_t out)
{
	uint8_t reg;

	reg = out & 0xF;
	if (reg > 12)
	{
		// This should not happen!
		UDR0 = '@'; // Signal error via serial port
		return;
	}
	CLR(ADF4351_LE);
	SPI_MasterTransmit((out >> 24) & 0xff); /* MSB */
	SPI_MasterTransmit((out >> 16) & 0xff);
	SPI_MasterTransmit((out >> 8) & 0xff);
	SPI_MasterTransmit((out) & 0xff); /* LSB */
	SET(ADF4351_LE);				  // Pulse LE pin high
	PLL.reg[reg] = out;				  // Implicit delay to increase length of LE pulse
	CLR(ADF4351_LE);
	DEBUG(PLL > 4, "PLL_Write(0x%08lx)", out);
}

void ADF5355_set_rflevel(int8_t enable, int8_t level)
{
	uint32_t reg;

	//
	// Reg 6
	//   Bit 4-5 = RFA Current for level control  0=Low to 3=High
	//   Bit 6   = RFA_Out control   0=Disable  1=Enable
	//	 Bit 10  = RFB_Out control   0=Enable   1=Disable (negative logic)
	//
	reg = PLL.reg[6] & ~SB(0b111, 4); //  RFA Disabled and level bits cleared (bits 4-6)
	reg |= SB(1, 10);				  // RFB Disabled - negative logic

	if (enable)
	{
		if (PLL.rfA_enable)
			reg |= SB(1, 6); // Enable RFA if low frequency enabled
		if (PLL.rfB_enable)
			reg &= ~SB(1, 10); // Enable RFB if high frequency enabled
	}
	reg |= SB(level & 0x03, 4); // Update RF Level
	PLL_Write2(reg);
}

#define MOD1 16777215UL // Fixed modulus for ADF5355
// #define STEPSZ	4000			// 20.000MHz / 2000 = 10kHz Steps
//			   4000 =  5kHz steps

PACKED_RESULT_40 result;

void ADF5355_set_freq(uint32_t freq)
{
	uint32_t fref = cf.ext_ref;
	uint32_t VCO_Frq = 0;
	uint16_t INT = 0;
	uint32_t FRAC1 = 0;
	uint32_t FRAC2 = 0;
	uint8_t RDIV = 1;
	uint16_t MOD2 = 256;
	uint16_t adcdiv, alc_tmo, synth_tmo, timeout;
	uint8_t rfB_enable, rfA_enable;
	uint8_t refdbl = 0;
	uint32_t pfd;
	uint8_t intmode, prescaler;
	uint8_t output_div = 0;
	uint8_t vco_mult;

	// VCO frequency must be between 3400-6800MHz, so calculate a suitable
	// divider value to generate the target frequency

	// PFD frequency must be less than 100Mhz.  If fed with a low reference
	// frequency (eg 10MHz) then enable the reference frequency doubler
	// which generally results in lower phase noise

	// If using a high reference frequency (>32MHz) then divide it down
	// to ensure the PFD frequency is on target.
	// Note the PFS can use a max of 100MHz for INT-N frequencies
	// but given our desires to synthesize frequencies in around 1MHz steps
	// we are not considering this possibility even if it improves the PN
	// by 6dB

	// VCO frequency must be between 3400-6800MHz, so calculate a suitable
	// divider value to generate the target frequency

	if (freq >= 680000000)
	{
		output_div = 1;
		vco_mult = 0;
	}
	else if (freq >= 340000000 && freq < 680000000)
	{
		vco_mult = 0;
		output_div = 1;
	}
	else if (freq >= 170000000 && freq < 340000000)
	{
		vco_mult = 1;
		output_div = 2;
	}
	else if (freq >= 85000000 && freq < 170000000)
	{
		vco_mult = 2;
		output_div = 4;
	}
	else if (freq >= 42500000 && freq < 85000000)
	{
		vco_mult = 3;
		output_div = 8;
	}
	else if (freq >= 21250000 && freq < 42500000)
	{
		vco_mult = 4;
		output_div = 16;
	}
	else if (freq >= 10625000 && freq < 21250000)
	{
		vco_mult = 5;
		output_div = 32;
	}
	else
	{
		vco_mult = 6;
		output_div = 64;
	}
	rfB_enable = freq >= MAX_VCO_FREQ ? 1 : 0;
	rfA_enable = !rfB_enable;
	// Calculate VCO frequency
	if (rfB_enable)
		VCO_Frq = freq >> 1;
	else
	{
		uint32_t temp = freq << vco_mult;
		VCO_Frq = temp;
	}

	RDIV = 1; //  Reference divisor not needed for ADF5355 in most situations

	// frequency (eg 10MHz) then enable the reference frequency doubler
	// which generally results in lower phase noise
	pfd = fref;
	if (fref * 2 < MAX_REF_FREQ)
	{
		refdbl = 1;
		pfd <<= 1;
	}

	uint32_t fract1_quot;
	uint32_t fract1_rem;

	fract1_quot = VCO_Frq / pfd;
	fract1_rem = VCO_Frq % pfd;

	INT = (uint16_t)fract1_quot;
	uint32_t kconst = ((uint64_t)1 << 40) / ((uint64_t)pfd);
	result.R = ((uint64_t)(kconst) * (uint64_t)fract1_rem);
	FRAC2 = (result.fs.fract2);
	FRAC1 = result.fs.fract1;

	intmode = (FRAC1 == 0 && FRAC2 == 0) ? 1 : 0;

	// Log critical frequency parameters for debugging
	DEBUG(PLL, "ADF5355 Freq=%lu, VCO Freq=%lu, pfd=%lu, ref=%lu, RDIV=%d, vco_mult=%d, output_div=%d ",
		  freq,		 // Output frequency in 10Hz units
		  VCO_Frq,	 // VCO frequency in Hz
		  pfd,		 // Phase Frequency Detector frequency in 10Hz units
		  fref,		 // Reference frequency in 10Hz units
		  RDIV,		 // Reference Divider
		  vco_mult,	 // VCO multiplier
		  output_div // Output divider
	);				 // Reference divider value

	// Log calculated constants and fractional parameters
	DEBUG(PLL, "KCONST=%lu  INT=%u, FRAC1=%lx, FRAC2=%lx, MOD2=%d",
		  kconst, // Constant used for fractional calculations
		  INT,	  // Integer division factor
		  FRAC1,  // First fractional component
		  FRAC2,  // Second fractional component
		  MOD2);  // Modulus for second fractional component

	// Preference to use 4/5 prescaler
	prescaler = 0;
	if (INT < 23)
		DEBUG(PLL, "**INT %d Too Low for prescaler", INT);
	if (INT >= 32768U)
		prescaler = 1;

	// Force External reference
	CLR(REFSEL);
	CLR(OSCPOWER);

	PLL_Write2(12 | SB(0xFFFF, 16) | SB(1, 10) | SB(1, 4)); // Disable phase resync
	PLL_Write2(11 | 0x0061300BUL);							// As per datasheet page 32
	adcdiv = (((pfd / 10000) - 2) + 4 - 1) / 4;
	if (adcdiv > 255)
		adcdiv = 255;
	DEBUG(PLL > 4, "ADC_CLK_DIV=%d", adcdiv);
	PLL_Write2(10 | SB(3, 22) | SB(adcdiv, 6) | SB(3, 4)); // ADC Control register

	uint8_t vco_band_div = 9; // (pfd + 240000 - 1) / 240000;
	timeout = 34;			  // *3 from datasheet
	alc_tmo = 30;
	synth_tmo = 12;

	DEBUG(PLL > 4, "VCO_BAND_DIV=%d, TIMEOUT=%d, ALC_TMO=%d, SYNTH_TMO=%d", vco_band_div, timeout, alc_tmo, synth_tmo);

	PLL_Write2(9 | SB(vco_band_div, 24) | SB(timeout, 14) | SB(alc_tmo, 9) | SB(synth_tmo, 4));
	PLL_Write2(8 | 0x102D0428UL);		// Datasheet page 31
	PLL_Write2(7 | SB(1, 28)			// Reserved
			   | SB(0, 25)				// LE Sync
			   | SB(0, 8)				// Lock Detect count = 1024 cycles
			   | SB(1, 7)				// Loss of lock detect enabled
			   | SB(3, 5)				// Frac-N Lock Detect 12ms
			   | SB(intmode ? 1 : 0, 4) // Lock Detect mode Integer or Frac-N
	);
	PLL_Write2(6 | SB(0xA, 25)				// Reserved bits
			   | SB(1, 24)					// Feedback = fundamental
			   | SB(output_div, 21)			// RF Out A divider
			   | SB(cf.spur & 1, 30)		// Spur reduction = gated bleed
			   | SB(cf.bleed ? 1 : 0, 29)	// Negative bleed enable
			   | SB(cf.bleed & 0xff, 13)	// Bleed current (was 5 for 10MHz ref)
			   | SB(cf.mtld & 1, 11)		// MTLD
			   | SB(rfB_enable ? 0 : 1, 10) // RFoutB ON (OFF=1, On=0)
			   | SB(rfA_enable ? 1 : 0, 6)	// RFOut A ON
			   | SB(rf.level & 3, 4)		// RF Level
	);
	PLL_Write2(5 | 0x00800025UL);			  // Page 27 Figure 46
	PLL_Write2(4 | SB(MUXOUT_DLOCK, 27)		  //  MuxOut = Digital Lock Detect
			   | SB(refdbl ? 1 : 0, 26)		  // Reference Doubler
			   | SB(RDIV, 15)				  // 10 bit R counter
			   | SB(1, 14)					  // Enable double buffering
			   | SB(cf.charge_pump & 0xf, 10) // Charge Pump
			   | SB(1, 8)					  // 3.3V Logic
			   | SB(1, 7)					  // +ve PD polarity
	);

	PLL_Write2(3 | 0L);							 // No phase stuff
	PLL_Write2(2 | SB(FRAC2, 18) | SB(MOD2, 4)); //
	PLL_Write2(1 | SB(FRAC1, 4));

	_delay_us(500); // TODO - Use correct delay for ADC calibration

	PLL_Write2(0 | SB(1, 21)	   // Enable Autocal
			   | SB(prescaler, 20) //   4/5 or 8/9 prescaler
			   | SB(INT, 4));

	PLL.freq = freq;
	PLL.INT = INT;
	PLL.FRAC1 = FRAC1;
	PLL.FRAC2 = FRAC2;
	PLL.MOD = MOD2;
	PLL.RDIV = RDIV;
	PLL.rf_mult = rfB_enable + 1; // Multiplication factor
	PLL.rf_div = 1 << output_div; // Division factor
								  //	TO DO
								  //	PLL.ref = ref;
	PLL.rfA_enable = rfA_enable;
	PLL.rfB_enable = rfB_enable;
}

void ADF5355_set_muxout(uint8_t v)
{
	PLL_Write2((PLL.reg[4] & ~SB(0x7, 27)) | SB(v & 7, 27));
}

void ADF5355_powerdown()
{
	uint32_t reg;

	DEBUG(PLL > 2, "ADF5355 Power Down");
	reg = PLL.reg[6] & ~SB(0b01111111, 4);
	reg |= SB(1, 10); // Disable RFOutB
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
void ADF5355_fsk(uint16_t step, int16_t delta)
{
	int32_t FRAC1;
	uint16_t intN;
	float stepsz, df1;
	uint16_t MOD2, FRAC2, scale;
	int16_t deltaF1;
	uint8_t autocal;
	uint32_t reg;

	autocal = ISSET(MUXOUTPIN) ? 0 : 1; //  If unlocked then perform autocal

	step = step * (float)PLL.rf_div / (float)PLL.rf_mult; // Scale step to VCO frequency range
	stepsz = PLL.ref * 1.0e5 / MOD1;
	df1 = (float)step * delta / stepsz;
	deltaF1 = (int16_t)df1;
	MOD2 = 8000; // Fixme
	FRAC2 = (df1 - deltaF1) * MOD2;
	scale = MOD2 / PLL.MOD;

	//  Remove the GCD calculation
	PLL.MOD = MOD2;
	PLL.FRAC2 *= scale;

	intN = PLL.INT;
	FRAC1 = (int32_t)PLL.FRAC1 + deltaF1;
	FRAC2 += PLL.FRAC2;

	if (FRAC2 >= MOD2)
	{
		FRAC2 -= MOD2;
		FRAC1++;
	}
	if (FRAC1 < 0L)
	{ // Next integer divisor down
		intN--;
		FRAC1 = ((int32_t)MOD1 + FRAC1);
	}
	if (FRAC1 >= MOD1)
	{ // Next integer divisor up
		intN++;
		FRAC1 -= MOD1;
	}
	FRAC1 &= 0x00FFFFFFUL;

	// PRINTLN("PLL step=%d, N=%d", step, deltaF1);
	// PRINTLN("      INT=%d  FRAC1=%ld  FRAC2=%d  MOD2=%d", intN, FRAC1, FRAC2, MOD2);

	// Use sigma-delta bit to determine if fsk mode is enabled.  We only need to change
	// the registers once when first entering fsk mode
	reg = PLL.reg[3];
	if (!(reg & SB(1, 30)))
	{
		DEBUG(PLL > 2, "FSK disable sigma-delta");
		//  Disable ADC (reg10) appears to stop random unlocks
		//  Technically these 3 registers only need to be changed once when enter fsk mode
		PLL_Write2(PLL.reg[10] & ~SB(1, 5)); // Disable ADC Calibration
		PLL_Write2(PLL.reg[7] & ~SB(1, 4));	 // Fractional-N Lock detect
		PLL_Write2(PLL.reg[6] & ~SB(1, 11)); // Disable MTLD
		PLL_Write2(PLL.reg[4] & ~SB(1, 4));	 // Counter reset disable
		PLL_Write2(PLL.reg[3] | SB(1, 30));	 // Disable sigma-delta reset on reload
		autocal = 1;
	}

	//  Update lock detect to either integer to fractional-N
	//	reg = (FRAC1 == 0 && FRAC2 == 0) ? PLL.reg[7]|SB(1, 4) : PLL.reg[7] & ~SB(1, 4);
	//	if (reg != PLL.reg[7]) {
	//		DEBUG(PLL>2, "FSK change lock mode %02x @%d", (uint8_t)reg, delta);
	//		PLL_Write2(reg);		// Update register7
	//	}

	if (FRAC1 == 0 && FRAC2 == 0)
		FRAC2++; //  Workaround to prevent integer lock detect issue
	//
	PLL_Write2(2 | SB(FRAC2, 18) | SB(MOD2, 4));
	PLL_Write2(1 | SB(FRAC1, 4)); // FRAC1

	if (autocal)
		_delay_us(500); // Appears this is still needed

	// XXX: If intN changes the S-D counter resets.  Need to AUTOCAL?
	//      For small steps intN will not change - only in extreme situations
	//		At the integer boundary (ie FRAC=0 and INT needs to change +/- 1)
	PLL_Write2((PLL.reg[0] & SB(1, 20))		   // Preserve prescaler setting
			   | SB(intN, 4) | SB(autocal, 21) // Reload
	);										   // AUTOCAL Disabled
}
