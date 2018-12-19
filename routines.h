/*
 * routines.h
 *
 *  Created on: 19. 12. 2018
 *      Author: DESKTOP
 */

#ifndef ROUTINES_H_
#define ROUTINES_H_

#include <stdio.h>

void USARTInit(void);
uint16_t ReadADC(uint8_t adc_input);
int uart_putchar(char c, FILE *stream);
void USART_Transmit( uint8_t data );
#endif /* ROUTINES_H_ */
