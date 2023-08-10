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

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stddef.h>
#include "i2c_slave.h"
#include "avr/sleep.h"


#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <util/twi.h>
#include "zlpll.h"

uint16_t fred;

static unsigned char I2C_buf[I2C_BUFFER_SIZE];     // Transceiver buffer. Set the size in the header file
static unsigned char I2C_msgSize  = 0;             // Number of bytes to be transmitted.
unsigned char I2C_state    = I2C_NO_STATE;  // State byte. Default set to I2C_NO_STATE.

static unsigned char I2C_bufPtr;				// Data count for RX and TX
 
// This is true when the TWI is in the middle of a transfer
// and set to false when all bytes have been transmitted/received
// Also used to determine how deep we can sleep.
static unsigned char I2C_busy = 0;

union I2C_statusReg_t I2C_statusReg = {0};           // I2C_statusReg is defined in I2C_Slave.h

/****************************************************************************
Call this function to set up the TWI slave to its initial standby state.
Remember to enable interrupts from the main application after initializing the TWI.
Pass both the slave address and the requirements for triggering on a general call in the
same byte. Use e.g. this notation when calling this function:
I2C_Slave_Initialise( (I2C_slaveAddress<<I2C_ADR_BITS) | (TRUE<<I2C_GEN_BIT) );
The TWI module is configured to NACK on any requests. Use a I2C_Start_Transceiver function to 
start the TWI.
****************************************************************************/
void I2C_Slave_Initialise( unsigned char I2C_ownAddress )
{
  TWAR = I2C_ownAddress;                            // Set own TWI slave address. Accept TWI General Calls.
  TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.
         (0<<TWIE)|(0<<TWINT)|                      // Disable TWI Interrupt.
         (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Do not ACK on any requests, yet.
         (0<<TWWC);                                 //
  I2C_busy = 0;
}    
    
/****************************************************************************
Call this function to test if the I2C_ISR is busy transmitting.
****************************************************************************/
unsigned char I2C_Transceiver_Busy( void )
{
  return I2C_busy;
}

/****************************************************************************
Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
until the I2C_ISR has completed with the previous operation. If there was an error, then the function 
will return the TWI State code. 
****************************************************************************/
unsigned char I2C_Get_State_Info( void )
{
  while ( I2C_Transceiver_Busy() ) { putchar('b'); }             // Wait until TWI has completed the transmission.
  return ( I2C_state );                         // Return error state. 
}

/****************************************************************************
Call this function to send a prepared message, or start the Transceiver for reception. Include
a pointer to the data to be sent if a SLA+W is received. The data will be copied to the TWI buffer. 
Also include how many bytes that should be sent. Note that unlike the similar Master function, the
Address byte is not included in the message buffers.
The function will hold execution (loop) until the I2C_ISR has completed with the previous operation,
then initialize the next operation and return.
****************************************************************************/
void I2C_Start_Transceiver_With_Data( unsigned char *msg, unsigned char msgSize )
{
  unsigned char temp;

  while ( I2C_Transceiver_Busy() ) { putchar('c'); }             // Wait until TWI is ready for next transmission.

  I2C_msgSize = msgSize;                        // Number of data to transmit.
  for ( temp= 0; temp < msgSize; temp++ )      // Copy data that may be transmitted if the TWI Master requests data.
  {
    I2C_buf[ temp ] = msg[ temp ];
  }
  I2C_statusReg.all = 0;      
  I2C_state         = I2C_NO_STATE;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
         (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Prepare to ACK next time the Slave is addressed.
         (0<<TWWC);                             //
  I2C_busy = 1;
}

/****************************************************************************
Call this function to start the Transceiver without specifying new transmission data. Useful for restarting
a transmission, or just starting the transceiver for reception. The driver will reuse the data previously put
in the transceiver buffers. The function will hold execution (loop) until the I2C_ISR has completed with the 
previous operation, then initialize the next operation and return.
****************************************************************************/
void I2C_Start_Transceiver( void )
{
  while ( I2C_Transceiver_Busy() ) { putchar('d'); }             // Wait until TWI is ready for next transmission.
  I2C_statusReg.all = 0;      
  I2C_state         = I2C_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
         (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Prepare to ACK next time the Slave is addressed.
         (0<<TWWC);                             //
  I2C_busy = 0;
}
/****************************************************************************
Call this function to read out the received data from the TWI transceiver buffer. I.e. first call
I2C_Start_Transceiver to get the TWI Transceiver to fetch data. Then Run this function to collect the
data when they have arrived. Include a pointer to where to place the data and the number of bytes
to fetch in the function call. The function will hold execution (loop) until the I2C_ISR has completed 
with the previous operation, before reading out the data and returning.
If there was an error in the previous transmission the function will return the TWI State code.
****************************************************************************/
uint8_t I2C_Get_Data_From_Transceiver( unsigned char *msg, unsigned char msgSize )
{
  uint8_t i;

  while ( I2C_Transceiver_Busy() ) {putchar('a'); }           // Wait until TWI is ready for next transmission.

  if( I2C_statusReg.status.lastTransOK )               // Last transmission completed successfully.
  {                                             
    for (i=0; i<msgSize && i<I2C_bufPtr; i++)	// Copy data from Transceiver buffer.
    {
      msg[i] = I2C_buf[i];
    }
    I2C_statusReg.status.RxDataInBuf = FALSE;          // Slave Receive data has been read from buffer.
	return i;
  }
  return 0;		// Error                                   
}


// ********** Interrupt Handlers ********** //
/****************************************************************************
This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
that is whenever a TWI event has occurred. This function should not be called directly from the main
application.
****************************************************************************/
ISR(TWI_vect)
{
   uint8_t  acknak;

	DEBUG(I2C>=10, "I2Cintr: TWSR=%02X", TWSR);
	  
   switch (TWSR)
  {
    case I2C_STX_ADR_ACK:            // Own SLA+R has been received; ACK has been returned
     I2C_bufPtr   = 0;                                 // Set buffer pointer to first data location
    case I2C_STX_DATA_ACK:           // Data byte in TWDR has been transmitted; ACK has been received
	  if (I2C_msgSize && I2C_bufPtr < I2C_msgSize) {	 
		TWDR = I2C_buf[I2C_bufPtr++];
	  } else {
		  DEBUG_CHAR(I2C>4, '*');
		  TWDR = 0xFF;		//  Give FF to indicate no data
	  }
	  //acknak = (I2C_msgSize && I2C_bufPtr < I2C_msgSize)?_BV(TWEA):0;
	  acknak = _BV(TWEA);
      TWCR = _BV(TWEN)|                                 // TWI Interface enabled
             _BV(TWIE)|_BV(TWINT)|                      // Enable TWI Interrupt and clear the flag to send byte
             (0<<TWSTA)|(0<<TWSTO)|						// 
             (0<<TWWC)|acknak;                          //
      I2C_busy = 1;
      break;
    case I2C_STX_DATA_NACK:          // Data byte in TWDR has been transmitted; NACK has been received. 
                                     // I.e. this could be the end of the transmission.
      if (I2C_bufPtr >= I2C_msgSize) // Have we transfered all expected data?
      {
        I2C_statusReg.status.lastTransOK = TRUE;               // Set status bits to completed successfully.
      } 
      else                          // Master has sent a NACK before all data where sent.
      {
        I2C_state = TWSR;                               // Store TWI State as error message.      
      }        
                                                        
      TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (1<<TWIE)|(1<<TWINT)|                      // Keep interrupt enabled and clear the flag
             (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Answer on next address match
             (0<<TWWC);                                 //
      
      I2C_busy = 0;   // Transmit is finished, we are not busy anymore
      break;     
    case I2C_SRX_GEN_ACK:            // General call address has been received; ACK has been returned
    case I2C_SRX_ADR_ACK:            // Own SLA+W has been received ACK has been returned
                                     // don't need to clear I2C_S_statusRegister.generalAddressCall due to that it is the default state.

	  I2C_statusReg.status.genAddressCall = (TWSR == I2C_SRX_GEN_ACK)?TRUE:FALSE;
      I2C_statusReg.status.RxDataInBuf = TRUE;
      I2C_bufPtr   = 0;                                 // Set buffer pointer to first data location
                                                        // Reset the TWI Interrupt to wait for a new event.
      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to send byte
             (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Expect ACK on this transmission
             (0<<TWWC);  
      I2C_busy = 1;
      
      break;
    case I2C_SRX_ADR_DATA_ACK:       // Previously addressed with own SLA+W; data has been received; ACK has been returned
    case I2C_SRX_GEN_DATA_ACK:       // Previously addressed with general call; data has been received; ACK has been returned
	  if (I2C_bufPtr < I2C_BUFFER_SIZE) {
		I2C_buf[I2C_bufPtr++]     = TWDR;
	  }  else {
		(void)TWDR;								//  Read and discard if buffer full
	  } 
	  acknak = (I2C_bufPtr < I2C_BUFFER_SIZE)?_BV(TWEA):0;
			
      I2C_statusReg.status.lastTransOK = TRUE;                 // Set flag transmission successfully.
                                                        // Reset the TWI Interrupt to wait for a new event.
      TWCR = _BV(TWEN)|                                 // TWI Interface enabled
             _BV(TWIE)|_BV(TWINT)|                      // Enable TWI Interrupt and clear the flag to send byte
             (0<<TWSTA)|(0<<TWSTO)|						
             (0<<TWWC) | acknak;                        // 
      I2C_busy = 1;
      break;
    case I2C_SRX_STOP_RESTART:       // A STOP condition or repeated START condition has been received while still addressed as Slave    
                                                        // Enter not addressed mode and listen to address match
      TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (1<<TWIE)|(1<<TWINT)|                      // Enable interrupt and clear the flag
             (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Wait for new address match
             (0<<TWWC);                                 //
      
      I2C_busy = 0;  // We are waiting for a new address match, so we are not busy
      
      break;           
    case I2C_SRX_ADR_DATA_NACK:      // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
    case I2C_SRX_GEN_DATA_NACK:      // Previously addressed with general call; data has been received; NOT ACK has been returned
    case I2C_STX_DATA_ACK_LAST_BYTE: // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
    case I2C_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
      I2C_state = TWSR;                 //Store TWI State as error message, operation also clears noErrors bit
      TWCR =  (1<<TWSTO)|(1<<TWINT);   //Recover from I2C_BUS_ERROR, this will release the SDA and SCL pins thus enabling other devices to use the bus
	  DEBUG(I2C>=5, "*I2CBE=%02X", I2C_state);
	  I2C_statusReg.status.lastTransOK = FALSE;		// Invalidate the data buffer?
	  I2C_busy = 0;								// I2C transaction has finished
      break;
    default:     
      I2C_state = TWSR;                                 // Store TWI State as error message, operation also clears the Success bit.      
      TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (1<<TWIE)|(1<<TWINT)|                      // Keep interrupt enabled and clear the flag
             (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Acknowledge on any new requests.
             (0<<TWWC);                                 //
      
      I2C_busy = 0; // Unknown status, so we wait for a new address match that might be something we can handle
  }
}
