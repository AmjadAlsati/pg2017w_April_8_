/*
 * timer.c
 *
 *  Created on: 05.06.2012
 *      Author: sid
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "mCtl.h"
#include "led.h"

volatile uint8_t timer;

ISR(TIMER0_OVF_vect)		//76,3Hz
{
	if(timer)
		LEDred(255);

	timer++;

	MC_updateVelo();
}

void T_Init(void)
{
	TCCR0B = 5<<CS00;	//div 1024
	TIMSK0 |= 1<<TOIE0;	//enable int

	timer = 0;
}
