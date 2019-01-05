/*
 * routines.c
 *
 *  Created on: 19. 12. 2018
 *      Author: DESKTOP
 */
#include "routines.h"
#include <util/delay.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define USART_BAUDRATE 9600
#define UBRR_VALUE ((F_CPU / (USART_BAUDRATE * 16UL)) - 1)


// Read the 8 most significant bits
// of the AD conversion result
uint16_t ReadADC(uint8_t adc_input) {//read voltages in 0,01V
  ADMUX = adc_input | (1 << REFS0);
// Delay needed for the stabilization of the ADC input voltage
  _delay_us(10);
// Start the AD conversion
  ADCSRA |= (1 << ADSC); // Start conversion
// Wait for the AD conversion to complete
  while (ADCSRA & (1 << ADSC))
    ;
  ADCSRA |= 0x10;
  return ADCW;
}


void USARTInit(void)
{
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);                      // shift the register right by 8 bits
	UBRR0L = (uint8_t)UBRR_VALUE;                           // set baud rate
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0);    // enable receiver and transmitter
	//UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format

}
/*
 * Send character c down the UART Tx, wait until tx holding register
 * is empty.
 */
/*int
uart_putchar(char c, FILE *stream)
{

  if (c == '\a')
    {
      fputs("*ring*\n", stderr);
      return 0;
    }

  if (c == '\n')
    uart_putchar('\r', stream);
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;

  return 0;
}*/

void USART_Transmit( uint8_t data )
{
/* Wait for empty transmit buffer */
while ( !( UCSR0A & (1<<UDRE0)) )
;
/* Put data into buffer, sends the data */
UDR0 = data;
}
