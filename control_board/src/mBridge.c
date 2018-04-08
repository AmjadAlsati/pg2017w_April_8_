/*
 * mBridge.c
 *
 *  Created on: 14.04.2012
 *      Author: sid
 */

#include <avr/io.h>

#include "flexport.h"
#include "mBridge.h"

void MB_Init(void)
{
	OUTPUT(MB_M0A);
	OUTPUT(MB_M0B);
	OUTPUT(MB_PWM0PIN);

	OUTPUT(MB_M1A);
	OUTPUT(MB_M1B);
	OUTPUT(MB_PWM1PIN);

	SET(MB_M0A);
	SET(MB_M0B);
	SET(MB_M1B);
	SET(MB_M1B);

	TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11) | (0<<WGM10);		//clean on match | Fast PWM Top:ICRn
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);				//Fast PWM Top:ICRn | div 1
	ICR1 = MB_PWMMASK;							//see .h

	MB_Pwr0(0);
	MB_Pwr1(0);

	MB_Enable();
}

void MB_Disable(void)
{
	OUTPUT(MB_DIAG);
	CLR(MB_DIAG);
}

void MB_Enable(void)
{
	INPUT(MB_DIAG);
	SET(MB_DIAG);
}

uint8_t MB_isEnabled(void)
{
	return GET(MB_DIAG);
}
