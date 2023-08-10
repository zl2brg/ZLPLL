/**
 * \file
 *
 * \brief Application to generate sample driver to AVRs TWI module
 *
 * Copyright (C) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel micro controller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

/* 
Test Cases

# Send test CW Message
[0x20 0x21 0 "_ZL2BKC_" 0]		
[0x20 0x29 12 0]		// Start CW Message @12WPM
[0x20 0x24 1]			// Carrier ON

# FSK Mode
[0x20 0x23 0 100 0]		// Init with 100Hz steps
[0x20 0x80]				// Symbol 0
[0x20 0x81]				// Symbol 1 (+100Hz)

[0x20 0x23 0 100 0x40]	//  Offset by 64 (+/- 64 symbols)
[0x20 0xC0]				// Symbol 0
[0x20 0xBF]				// Symbol -1


*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2c_slave.h"
#include "avr/sleep.h"

#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "zlpll.h"
#include "mgm.h"
#include "cw.h"

// Sample TWI transmission commands
#define I2C_CMD_MASTER_WRITE 0x10
#define I2C_CMD_MASTER_READ  0x20

// The AVR can be waken up by a TWI address match from all sleep modes,
// But it only wakes up from other TWI interrupts when in idle mode.
// If POWER_MANAGEMENT_ENABLED is defined the device will enter power-down 
// mode when waiting for a new command and enter idle mode when waiting
// for TWI receives and transmits to finish.
#define POWER_MANAGEMENT_ENABLED

// When there has been an error, this function is run and takes care of it
unsigned char I2C_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg );


union FBT {
	float f;
	uint8_t b[4];
	uint32_t i;
	};
	

// Process a I2C message and return an acknowledgment
static void I2C_Message(uint8_t *msg, uint8_t len) {
	uint8_t cmd, reload, i;
	uint8_t start, len2;
	uint8_t respbuf[10], resplen, *resp = respbuf;
	int8_t  s8;
	union FBT fbt;
	
	if (len == 0)
		return;
	resplen = 0;
	reload = 0;
		
	cmd = msg[0];
	switch (cmd) {
	case 0:		// Read H/W Config
				DEBUG(I2C, "I2C_GET_HWREV");
				resp[0] = 0x10;		//  TODO ADF5355/4351
				resp[1] = '4';		// Major/Minor version
				resp[2] = '0';
				resp[3] = 'D';
				resp[4] = 35>>8;		//  Minfreq
				resp[5]	= 35&0xFF;		// 
				resp[6] = 4400U>>8;		// Maxfreq
				resp[7] = 4400U&0xFF;
				resplen = 8;
				break;
	case 1:		// Read lock status
				DEBUG(I2C, "I2C_READ_STATUS");
				resp[0] = check_pll_locked();	//  Lock status - 0 = unlock, 1 = lock
				resp[1] = TCNT0;	//  Loss lock count 
				resplen = 2;	//  2 byte response
				break;
	case 2:		// Read RF Freq and level
				DEBUG(I2C, "I2C_READ_RFCONFIG");
				fbt.f = rf.rf_freq;
				resp[0]=fbt.b[0];
				resp[1]=fbt.b[1];
				resp[2]=fbt.b[2];
				resp[3]=fbt.b[3];
				resp[4]=rf.mult;			// Multiplier
				resp[5]=rf.level;			// RF Level
				resplen = 6;
				break;
				
	case 16:	// Set RF Channel 
				DEBUG(I2C, "I2C_SET_RFCHAN(%x)", msg[1]);
				if (len >= 2 && msg[1] < 16) {
						cmd_channel(msg[1] & 0xF);
						resp[0] = 1;		// OK
				} else {
					resp[0] = 2;		// Error
					resplen = 1;
					reload = 1;
				}
				break;
	case 17:	// Set RF Frequency (Float)
				if (len >= 7 && msg[5] < 10 && msg[6] <= 3) {
					fbt.b[0] = msg[1];
					fbt.b[1] = msg[2];
					fbt.b[2] = msg[3];
					fbt.b[3] = msg[4];
					DEBUG(I2C, "I2C_SET_RFCHAN(freq=%.3f, mult=%d, lev=%d)", fbt.f, msg[5], msg[6]);
					rf.rf_freq = fbt.f;
					rf.mult = msg[5];
					rf.level = msg[6];				
					resp[0] = 1;		// OK
					reload = 1;
				} else {
					resp[0] = 2;		// Error
					resplen = 1;
					reload = 1;
				}
				break;

	case 0x20:	// Read MGM Array
				if (len >= 3) {
					//msg[0];		//  Array type (always set to 0)
					start = msg[1];		//  Start Address
					len2 = msg[2];		//  Length (Bytes)
					if (len2 > 32) len2 = 32;		//  Limit to 32 bytes
					if (start+len2 <= MGM_MAXMSGLEN) {}
						resp = &MGM_Message[start];
						resplen = len2;
					}
				break;
				
	case 0x21:	// Write MGM Array
				if (len >= 3) {
					//msg[0];		//  Array type (always set to 0)
					start = msg[1];		//  Start Address
					len -= 2;			//  
					for (i=0; i<len; i++) {
						MGM_Message[start+i] = msg[i+2];
					}
				}
				break;
	case 0x22:	// Write PLL registers
				//  Not implemented....
				break;

	case 0x23:	// MGM Mode init
				if (len >= 3) {
					PLL.fsk_step = msg[1]<<8 | msg[2];
					PLL.fsk_offset = msg[3];
					DEBUG(I2C>=1, "I2C_MGMINIT(shift=%d, offset=%d", PLL.fsk_step, PLL.fsk_offset);
				}
				break;
	case 0x24:	//  Set RF Mode
				 if (len >= 1) {
					cw_stop(msg[1]);
					rf.mode = 1; 
				 }
				 break;
											
	case 0x28:	// RF_FSK
				// TODO:  Check step is initialized correctly....
				if ( len >= 1) {
					s8 = (int8_t) msg[1] - PLL.fsk_offset;
					PLL(fsk)(PLL.fsk_step, s8);		//  Change frequency 
					DEBUG2(I2C>=2, ">%02X", s8&0xFF);
				}
				break;
		
	case 0x29:	// Start MGM CW Transmission
				// msg[1] = CW Speed in WPM
				//			If speed = 0, then CW is disabled and RF carrier is enabled
				// msg[2] = Message byte offset in Message Buffer
				if (len >= 3) {
					i = msg[1];
					if (i < 5 || i > 25)
						i = 12;
					else {
						cf.cw_speed = i;
						i = msg[2];
						rf.mode = 2;
						cw_start(MSGTYPE_RAM, (uint8_t *) MGM_Message+i, 2);
					}
				}
				break;
				
	case 0x2a:		
				//msg[0];		//  Array Index (set to 0)
				//msg[1];		// Type CW or MGM
				//msg[2];		// FSK Step
				//msg[3];		// symbol rate
				//msg[4];		// Number of bits to TX in this message	
				break;			
	default:
				if (cmd >= 0x80) {		//  Quick FSK Mode
					s8 = (cmd - 0x80) - PLL.fsk_offset;
					PLL(fsk)(PLL.fsk_step, s8);		//  Change frequency
					DEBUG2(I2C>=2, ">%02X", s8&0xFF);
				}
	}
	if (resplen) {
		if (debug_I2C > 4) {
			PRINT("  resp len=%d:", resplen);
			for (i=0; i<resplen; i++)
				PRINT(" %02X", resp[i]);
			putchar('\n');
		}
		I2C_Start_Transceiver_With_Data(resp, resplen);
	} else {
		I2C_Start_Transceiver();		//  No response
	}
	if (reload) {
		cli();
		set_freq(rf.rf_freq);
		sei();
	}
}


//
//  Initialize the I2C/TWI interface for Slave operation
//
void I2C_Init(uint8_t addr)
{

  unsigned char I2C_slaveAddress;

  if (addr) {
	  // Own TWI slave address
	  I2C_slaveAddress     = (addr<<I2C_ADR_BITS);
 
	  // Initialize TWI module for slave operation. Include address and/or enable General Call.
	  I2C_Slave_Initialise( (unsigned char)(I2C_slaveAddress | (TRUE<<I2C_GEN_BIT))); 
	
  } else {
	  TWAR = TWCR = 0;		// Disable I2C
  }
}

void I2C_Start(void) 
{
  // Start the TWI transceiver to enable reception of the first command from the TWI Master.
  I2C_Start_Transceiver();
}

void I2C_check(void)
{
	unsigned char messageBuf[I2C_BUFFER_SIZE];
	uint8_t len, i, err;

    // Check if the TWI Transceiver has completed an operation.
    if ( ! I2C_Transceiver_Busy() )                              
    {
      // Check if the last operation was successful
      if ( I2C_statusReg.status.lastTransOK )
      {
        // Check if the last operation was a reception
        if ( I2C_statusReg.status.RxDataInBuf )
        {
			  len = I2C_Get_Data_From_Transceiver(messageBuf, I2C_BUFFER_SIZE);         
			  // Check if the last operation was a reception as General Call        
			  if ( I2C_statusReg.status.genAddressCall )
			  {      
				DEBUG2(I2C>3, "I2C gen (len=%d):", len);
			  }               
			  else // Ends up here if the last operation was a reception as Slave Address Match   
			  {
				// Example of how to interpret a command and respond.
            
				DEBUG2(I2C>3, "I2C (len=%d):", len);
			  }
			for (i=0; i<len; i++) 
				DEBUG2(I2C>3, " %02X", messageBuf[i]);
			DEBUG_CHAR(I2C>3, '\n');
			I2C_Message(messageBuf, len);
        }                
        else // Ends up here if the last operation was a transmission  
        {
		}
			
        // Check if the TWI Transceiver has already been started.
        // If not then restart it to prepare it for new receptions.             
        if ( ! I2C_Transceiver_Busy() )
        {
          I2C_Start_Transceiver();
        }
      }
      else // Ends up here if the last operation completed unsuccessfully
      {
		err = I2C_state;
		if (err != I2C_NO_STATE)
			I2C_Act_On_Failure_In_Last_Transmission(err);
      }
    }
}


unsigned char I2C_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
  DEBUG(I2C>=3, "I2C Fail=%02X", TWIerrorMsg);
  I2C_Start_Transceiver();
  return TWIerrorMsg; 
}
