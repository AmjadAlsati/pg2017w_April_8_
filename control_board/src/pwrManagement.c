/*
 * pwrManagement.c
 *
 *  Created on: 14.04.2012
 *      Author: sid
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <inttypes.h>

#include "flexport.h"
#include "pwrManagement.h"
#include "mBridge.h"
#include "led.h"
#include "adc.h"

//requires ADC
//polling: switch and ad value


void PM_Init(void)
{
	INPUT(PM_SN);
	CLR(PM_SN);

	OUTPUT(PM_HD);
	CLR(PM_HD);
	_delay_ms(100);
	if(GET(PM_SN))
		SET(PM_HD);

	/* indicate power up */
	for(uint8_t i=0; i<64; i++)
	{
		LEDred(i);
		_delay_ms(4);
	}
	for(uint8_t i=63; i>0; i--)
	{
		LEDred(i);
		_delay_ms(4);
	}
	while(GET(PM_SN))
		wdt_reset();
	LEDred(0);
}

void PM_Off(void)
{
	/* indicate power off */

	//timer off
	TCCR0B = 0;

	while(GET(PM_SN))
	{
		for(uint8_t i=0; i<64; i++)
		{
			LEDred(i);
			_delay_ms(4);
		}
		for(uint8_t i=63; i>0; i--)
		{
			LEDred(i);
			_delay_ms(4);
		}
		LEDred(0);
		_delay_ms(4);
		wdt_reset();
	}

	for(uint8_t i=0; i<64; i++)
	{
		LEDblue(i);
		_delay_ms(4);
	}
	for(uint8_t i=63; i>0; i--)
	{
		LEDblue(i);
		_delay_ms(4);
	}

	CLR(PM_HD);
}

void PM_Poll(void)
{
	static uint8_t powerButton = 0;
	static uint8_t overHeat = 0;

	if(GET(PM_SN))
		powerButton++;
	else
		powerButton = 0;

	if(powerButton == 255 || ADC_Buf[ADC_CHBAT] < PM_MINVCC)
		PM_Off();

	if(!GET(MB_DIAG))
	{
		LEDred(63);
		overHeat = 1;
	}
	else if(overHeat)
	{
		LEDred(0);
		overHeat = 0;
	}

}
