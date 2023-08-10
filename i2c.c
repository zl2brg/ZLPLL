//************************************************************************
//**
//** Project......: Firmware USB AVR Si570 controler.
//**
//** Platform.....: ATtiny45
//**
//** Licence......: This software is freely available for non-commercial 
//**                use - i.e. for research and experimentation only!
//**                Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
//**                Based on ObDev's AVR USB driver by Christian Starkjohann
//**
//** Programmer...: F.W. Krom, PE0FKO and
//**                thanks to Tom Baier DG8SAQ for the initial program.
//** 
//** Description..: I2C Protocol, trying to simulate a open Collector output.
//**
//** History......: V15.1 02/12/2008: First release of PE0FKO.
//**                Check the main.c file
//**
//**************************************************************************

#define __HAS_DELAY_CYCLES 0  

#include <avr/io.h>
#include <util/delay.h>

#define I2C_KBITRATE	200



#define I2C_SDA_LO			I2C_DDR |= SDA
#define I2C_SDA_HI			I2C_DDR &= ~SDA
#define I2C_SCL_LO			I2C_DDR |= SCL
#define I2C_SCL_HI			I2C_DDR &= ~SCL

#define	I2C_DELAY_uS		1000/I2C_KBITRATE // 8uS = 125KHz

uint8_t I2CErrors;


void I2CInit(void)
{
    //set SCL to 100kHz
    TWSR = _BV(TWPS1); // Prescaler = 4
    TWBR = (int)((long)F_CPU/(I2C_KBITRATE*1000L*4L));
    //enable TWI
    TWCR = (1<<TWEN);
}


void I2CSendStart(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}
//send stop signal
void I2CSendStop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void I2CSendByte(uint8_t u8data)
{
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

uint8_t I2CRecieveByte(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while ((TWCR & (1<<TWINT)) == 0);
    return TWDR;
}

//read byte with NACK
uint8_t TWIReadNACK(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
    return TWDR;
}

uint8_t TWIGetStatus(void)
{
    uint8_t status;
    //mask status
    status = TWSR & 0xF8;
    return status;
}

// Check if  device is present on the I2C bus at specified address
//
//  Returns 0 = Error (No device found)
//			1 = OK (Device found)
//
uint8_t I2CPoll(uint8_t addr)
{
	I2CSendStart();
	if (TWIGetStatus() != 0x08)  goto error;
	I2CSendByte(addr);
	if (TWIGetStatus() != 0x18)  goto error;	// Check for MT_SLA_ACK
	I2CSendStop();
	return (1);
error:
	TWCR = (1<<TWINT);		// Free I2C Bus
	return 0;
}
