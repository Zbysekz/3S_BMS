/*
 * main.c
 *
 *  Created on: 18. 12. 2018
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

#define pCHARGE PORTD
#define CHARGE PD2

#define pOUT PORTD
#define OUT PD3

#define pCHARGE_SIG PIND
#define CHARGE_SIG PD4

#define pOPTO1 PORTB
#define OPTO1 PB3

#define pOPTO2 PORTB
#define OPTO2 PB4

#define pOPTO3 PORTB
#define OPTO3 PB5

#define pLED_G PORTB
#define LED_G PB7

#define pLED_R PORTB
#define LED_R PB6


//0,01V
uint16_t parLow=310; // when go to LOW
uint16_t parOk=340; // when go back to NORMAL
uint16_t parBurnStart=410; // when start burning for specific cell
uint16_t parBurnStop=400; // when start burning for specific cell
uint16_t parMinBurnTime=10; // 0,1s


uint8_t currentState=0,nextState=0;

uint8_t stepTimer=0;

/***************************************************/
int main(void)
{

	//set outputs
	DDRB = (1<<OPTO1) | (1<<OPTO2) | (1<<OPTO3);
	//set outputs
	DDRD = (1<<CHARGE) | (1<<OUT);



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


	
	_delay_s(1);


	while(1){

		setBit(&pLED_G,LED_G);
		clearBit(&pLED_R,LED_R);
		
		_delay_s(1);
		
		clearBit(&pLED_G,LED_G);
		setBit(&pLED_R,LED_R);
		
		_delay_s(1);
		
		setBit(&pLED_G,LED_G);
		clearBit(&pLED_R,LED_R);
		
		_delay_s(1);
		
		setBit(&pOPTO1,OPTO1);
		clearBit(&pOPTO2,OPTO2);
		clearBit(&pOPTO3,OPTO3);
		
		_delay_s(1);
		
		clearBit(&pOPTO1,OPTO1);
		setBit(&pOPTO2,OPTO2);
		clearBit(&pOPTO3,OPTO3);
		
		_delay_s(1);
		
		clearBit(&pOPTO1,OPTO1);
		clearBit(&pOPTO2,OPTO2);
		setBit(&pOPTO3,OPTO3);
		
		_delay_s(1);
		
		clearBit(&pOPTO3,OPTO3);
		
		
		switch(currentState){
			case STATE_NORMAL:
			
				CellBalancing();
				
				//if one of the cell is below threshold go to LOW
				
				
				//if charger is connected, go to charging
				if(!getBit(pCHARGE_SIG,CHARGE_SIG)){
					nextState=STATE_CHARGING;
					break;
				}
				
				//output is on
				setBit(&pOUT,OUT);
			break;
			
			case STATE_CHARGING:
			
				CellBalancing();
				
				//run output only if cells have ok voltage
				//if(ok with delay)setBit(&pOUT,OUT);else clearBit(&pOUT,OUT);
				
				//if charger is not connected anymore, go to normal operation
				if(!getBit(pCHARGE_SIG,CHARGE_SIG)){
					nextState=STATE_NORMAL;
					break;
				}
			break;
			
			case STATE_LOW:
			
				ResetOptos();//turn off all optos
				
				//sleep and check sometimes following
				
				//if voltages are really ok, go to normal
				
				//if charger is connected, go to CHARGING
				if(getBit(pCHARGE_SIG,CHARGE_SIG)){
					nextState=STATE_CHARGING;
					break;
				}
				
				//output is cut off
				clearBit(&pOUT,OUT);
			break;
			
		}
		
		if(nextState!=currentState){
			currentState = nextState;
			stepTimer = 0;
		}
		
		
		////////////////////////LED STATE/////////////////////////////////////////////
		if(currentState == STATE_CHARGING){//red color for charging
			clearBit(&pLED_G,LED_G);
			setBit(&pLED_R,LED_R);
		}else if (currentState == STATE_LOW){//nothing for low
			clearBit(&pLED_G,LED_G);
			clearBit(&pLED_R,LED_R);
		}else if (currentState == STATE_NORMAL){//green for normal operation
			setBit(&pLED_G,LED_G);
			clearBit(&pLED_R,LED_R);
		}
		//////////////////////////////////////////////////////////////////////////////
		
	}

return 0;
}

uint16_t ReadADC(uint8_t channel){//read voltages in 0,01V

}

void CellBalancing(){
	//CELL BALANCING
	//if one of the cell is above threshold, set opto
	//if it is lower then threshold reset opto

}

void ResetOptos(){
	clearBit(&pOPTO1,OPTO1);
	clearBit(&pOPTO2,OPTO2);
	clearBit(&pOPTO3,OPTO3);
}

// Timer 0 overflow interrupt service routine    // Clock value: 15 Hz
ISR(TIMER0_OVF_vect)
{


}
ISR(INT0_vect){

}
ISR(INT1_vect){

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
