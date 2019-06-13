/*
 * twi.c
 *
 *  Created on: Jan 2, 2019
 *      Author: zz
 */

#include <compat/twi.h>
#include <avr/interrupt.h>
#include "twi.h"
#include "main.h"


#define TWI_SLAVE_ADDR       0x50
#define BUFFSIZE 10

uint8_t rxBuffer[BUFFSIZE];
uint8_t txBuffer[BUFFSIZE];
uint8_t txBuffer2[BUFFSIZE];

volatile uint8_t i2c_state;
volatile uint8_t twi_status;

volatile uint8_t regaddr; // Store the Requested Register Address
volatile uint8_t regdata; // Store the Register Address Data

volatile uint16_t TWI_error;

volatile uint8_t txCRC,txCRC2,updateTX,buffId,buffId2;;

///////CRC//////////////

/*
 * The width of the CRC calculation and result.
 * Modify the typedef for a 16 or 32-bit CRC standard.
 */

#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))

#define POLYNOMIAL 0x07
//////////////////////////

uint8_t crcTable[256];

void InitCRC(void)
{
    crc  remainder;
    //Compute the remainder of each possible dividend.
    for (int dividend = 0; dividend < 256; ++dividend)
    {
        //Start with the dividend followed by zeros.
        remainder = dividend << (WIDTH - 8);
        //Perform modulo-2 division, a bit at a time.
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            // Try to divide the current data bit.
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
        // Store the result into the table.
        crcTable[dividend] = remainder;
    }

}  /* crcInit() */

crc CalculateCRC( uint8_t message[], int nBytes)
{
    uint8_t data;
    crc remainder = 0;
    //Divide the message by the polynomial, a byte at a time.
    for (int byte = 0; byte < nBytes; ++byte)
    {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = (uint8_t)(crcTable[data] ^ (remainder << 8));
    }
    //The final remainder is the CRC.
    return (remainder);
}   /* crcFast() */

void ValidateData(uint8_t crc){
	//validate if received data are ok
	if(CalculateCRC(rxBuffer,1) == crc){
		//we can now copy rxBuffer data to specific vars
		if(rxBuffer[0]==11)
			goOff=1;//go off (sleep forever)
	}
}

void UpdateTxData(){//called at the beginning and every time after CRC value is read by master
	if(updateTX==0)return;

	uint8_t *buff;

	if(buffId==0)
		buff=txBuffer2;
	else
		buff=txBuffer;

	buff[0]=currentState;
	buff[1]=cellA & 0xFF;
	buff[2]=(cellA & 0xFF00)>>8;
	buff[3]=cellB & 0xFF;
	buff[4]=(cellB & 0xFF00)>>8;
	buff[5]=cellC & 0xFF;
	buff[6]=(cellC & 0xFF00)>>8;

	if(buffId==0)
		txCRC2=CalculateCRC(buff,7);
	else
		txCRC=CalculateCRC(buff,7);

	buffId2=buffId?0:1;		//now the TWI interrupt routine can read the updated one

	updateTX=0;
}


void TWI_Init(){
	// Initial I2C Slave
	TWAR = TWI_SLAVE_ADDR & 0xFE;    // Set I2C Address, Ignore I2C General Address 0x00
	TWDR = 0x00;            // Default Initial Value

	// Start Slave Listening: Clear TWINT Flag, Enable ACK, Enable TWI, TWI Interrupt Enable
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);

	// Initial Variable Used
	regaddr=0;
	regdata=0;

	updateTX=1;//update tx data for first time
	buffId=0;
}


void TWI_SlaveAction(uint8_t rw_status)
{
	if(regaddr==0){// this is just for communication check
		if (rw_status == 0)
			regdata = 66;
	}else if(regaddr == 200){//master is sending CRC to us or wants CRC of our data
		if (rw_status == 0){
			if(buffId)//switch between two buffers to prevent data corruption while updating
				regdata = txCRC2;
			else
				regdata = txCRC;
			updateTX=1;//update next data
			buffId=buffId2;
		}else
			ValidateData(regdata);
	}else if(regaddr>0 && regaddr<BUFFSIZE+1){
		if(rw_status == 0){ //read
			if(buffId)//switch between two buffers to prevent data corruption while updating
				regdata = txBuffer2[regaddr-1];
			else
				regdata = txBuffer[regaddr-1];
		}else{//write
			rxBuffer[regaddr-1] = regdata;
		}
    }
}


ISR(TWI_vect)
{
    // Get TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
    twi_status=TWSR & 0xF8;

    switch(twi_status) {
        case TW_SR_SLA_ACK: // 0x60: SLA+W received, ACK returned
            i2c_state=0;    // Start I2C State for Register Address required
            break;

        case TW_SR_DATA_ACK:    // 0x80: data received, ACK returned
            if (i2c_state == 0) {
                regaddr = TWDR; // Save data to the register address
                i2c_state = 1;
            } else {
                regdata = TWDR; // Save to the register data
                i2c_state = 2;
            }
            break;

        case TW_SR_STOP:    // 0xA0: stop or repeated start condition received while selected
            if (i2c_state == 2) {
            	TWI_SlaveAction(1);    // Call Write I2C Action (rw_status = 1)
                i2c_state = 0;      // Reset I2C State
            }
            break;

        case TW_ST_SLA_ACK: // 0xA8: SLA+R received, ACK returned
        case TW_ST_DATA_ACK:    // 0xB8: data transmitted, ACK received
            if (i2c_state == 1) {

            	TWI_SlaveAction(0);    // Call Read I2C Action (rw_status = 0)

                TWDR = regdata;     // Store data in TWDR register
                i2c_state = 0;      // Reset I2C State
            }
            break;

        case TW_BUS_ERROR:  // 0x00: illegal start or stop condition
        	TWCR |= (1<<TWSTO)|(1<<TWINT);//recover from this state
        	TWI_error++;
        	return;//EXIT
        case TW_ST_DATA_NACK:   // 0xC0: data transmitted, NACK received
		case TW_ST_LAST_DATA:   // 0xC8: last data byte transmitted, ACK received
        default:
            i2c_state = 0;  // Back to the Begining State
    }

    // Clear TWINT Flag
    TWCR |= (1<<TWINT);
}




