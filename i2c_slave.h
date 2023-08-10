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
/****************************************************************************
  TWI Status/Control register definitions
****************************************************************************/
#ifndef I2C_SLAVE_H_INCLUDED
#define I2C_SLAVE_H_INCLUDED

#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>



#define I2C_BUFFER_SIZE 40      // Reserves memory for the drivers transceiver buffer. 
                               // Set this to the largest message size that will be sent including address byte.

/****************************************************************************
  Global definitions
****************************************************************************/
  
union I2C_statusReg_t                       // Status byte holding flags.
{
    unsigned char all;
    struct
    {
        unsigned char lastTransOK:1;      
        unsigned char RxDataInBuf:1;
        unsigned char genAddressCall:1;                        // TRUE = General call, FALSE = TWI Address;
        unsigned char unusedBits:5;
    }status;
};

union I2C_statusReg_t I2C_statusReg;

extern unsigned char I2C_state;

//static unsigned char dont_sleep = 0;

/****************************************************************************
  Function definitions
****************************************************************************/
void I2C_Slave_Initialise( unsigned char );
unsigned char I2C_Transceiver_Busy( void );
unsigned char I2C_Get_State_Info( void );
void I2C_Start_Transceiver_With_Data( unsigned char * , unsigned char );
void I2C_Start_Transceiver( void );
uint8_t I2C_Get_Data_From_Transceiver( unsigned char *, unsigned char );


/****************************************************************************
  Bit and byte definitions
****************************************************************************/
#define I2C_READ_BIT  0   // Bit position for R/W bit in "address byte".
#define I2C_ADR_BITS  1   // Bit position for LSB of the slave address bits in the init byte.
#define I2C_GEN_BIT   0   // Bit position for LSB of the general call bit in the init byte.

#define TRUE          1
#define FALSE         0

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master status codes                      
#define I2C_START                  0x08  // START has been transmitted  
#define I2C_REP_START              0x10  // Repeated START has been transmitted
#define I2C_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter status codes                      
#define I2C_MTX_ADR_ACK            0x18  // SLA+W has been transmitted and ACK received
#define I2C_MTX_ADR_NACK           0x20  // SLA+W has been transmitted and NACK received 
#define I2C_MTX_DATA_ACK           0x28  // Data byte has been transmitted and ACK received
#define I2C_MTX_DATA_NACK          0x30  // Data byte has been transmitted and NACK received 

// TWI Master Receiver status codes  
#define I2C_MRX_ADR_ACK            0x40  // SLA+R has been transmitted and ACK received
#define I2C_MRX_ADR_NACK           0x48  // SLA+R has been transmitted and NACK received
#define I2C_MRX_DATA_ACK           0x50  // Data byte has been received and ACK transmitted
#define I2C_MRX_DATA_NACK          0x58  // Data byte has been received and NACK transmitted

// TWI Slave Transmitter status codes
#define I2C_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define I2C_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define I2C_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define I2C_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define I2C_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received

// TWI Slave Receiver status codes
#define I2C_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define I2C_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define I2C_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define I2C_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define I2C_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define I2C_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define I2C_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define I2C_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define I2C_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define I2C_NO_STATE               0xF8  // No relevant state information available; TWINT = “0”
#define I2C_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

#endif // I2C_SLAVE_H_INCLUDED
