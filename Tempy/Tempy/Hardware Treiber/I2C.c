#ifndef  F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/twi.h>
#include "I2C.h"


void i2c_init(void)
{
	TWBR = I2C_TWBR;
}

uint8_t i2c_start(uint8_t address)
{
	// reset TWI control register
 	TWCR = 0;
 	// transmit START condition
 	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
 	// wait for end of transmission
 	while( !(TWCR & (1<<TWINT)) );

 	// check if the start condition was successfully transmitted
 	if((TWSR & 0xF8) != TW_START){ return 1; }

 	// load slave address into data register
 	TWDR = address;
 	// start transmission of address
 	TWCR = (1<<TWINT) | (1<<TWEN);
 	// wait for end of transmission
 	while( !(TWCR & (1<<TWINT)) );

 	// check if the device has acknowledged the READ / WRITE mode
 	uint8_t twst = TW_STATUS & 0xF8;
 	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

 	return 0;
}

void i2c_start_wait(uint8_t address)
{
	while( (i2c_start( address ) ) );	
}

uint8_t i2c_rep_start(uint8_t address)
{
	return ( i2c_start( address ) );
}

uint8_t i2c_write(uint8_t data)
{
 	// load data into data register
 	TWDR = data;
 	// start transmission of data
 	TWCR = (1<<TWINT) | (1<<TWEN);
 	// wait for end of transmission
 	while( !(TWCR & (1<<TWINT)) );

 	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }

 	return 0;
}

uint8_t i2c_readAck(void)
{

 	// start TWI module and acknowledge data after reception
 	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
 	// wait for end of transmission
 	while( !(TWCR & (1<<TWINT)) );
 	// return received data from TWDR
 	return TWDR;
}

uint8_t i2c_readNak(void)
{
	// start receiving without acknowledging reception
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

void i2c_stop(void)
{
	// transmit STOP condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}
