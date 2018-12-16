/*
 * main.c
 *
 *  Created on: 3. 4. 2016
 *      Author: Zbysek
 */

#include <avr/io.h>
#include <util/delay.h>
#include "main.h"
//#include <stdlib.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include <avr/sleep.h>


/***************************************************/

#define pLockRele PORTB
#define lockRele PB0

#define pDebugLed PORTB
#define debugLed PB1

#define pExtendedPower PORTB
#define extendedPower PB2

/***************************************************/
int main(void)
{

	//nastavit výstupy
	DDRB = (1<<PB0) | (1<<PB1) | (1<<PB2);

	//natavit výstupy
	DDRD =1<<PD5;

	//natavit výstupy
	DDRC =(1<<PC2) | (1<<PC3);


	_delay_s(1);

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: 15,625 kHz
/*	TCCR0=0x05;
	TCNT0=0x00;

	// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK=0x01;*/

	//external interrupt INT0 - accel
	//GICR=(1<<INT0);
	//MCUCR = (1<<ISC01);//pro accel sestupna

	// Use the Power Down sleep mode
	//set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	//init interrupt
	sei();


	_delay_ms(50);


	while(1){//lze blokovat pomocí delay

	}

return 0;
}
// Timer 0 overflow interrupt service routine    // Clock value: 15 Hz
ISR(TIMER0_OVF_vect)
{


}
ISR(INT0_vect){

}
ISR(INT1_vect){//klicek

}

void Blik(void){//debug led
	setBit(&pDebugLed,debugLed);
	_delay_s(1);
	clearBit(&pDebugLed,debugLed);
	_delay_s(1);
}
void _delay_s(int sec){
	for(int c=0;c<sec*10;c++)
		_delay_ms(100);
}

void setBit(volatile uint8_t *port, int bit){
	*port|=(1<<bit);
}
void clearBit(volatile uint8_t *port, int bit){
	*port&=0xFF-(1<<bit);
}
uint8_t getBit(volatile uint8_t port, int bit){
	if((port&(1<<bit))==0)return 0;else return 1;
}
