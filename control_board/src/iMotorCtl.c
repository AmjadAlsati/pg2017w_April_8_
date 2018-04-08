/*
 * iMotorCtl.c
 *
 *  Created on: 14.04.2012
 *      Author: sid
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#include "flexport.h"
#include "mCtl.h"
#include "pwrManagement.h"
#include "led.h"
#include "sensor.h"
#include "adc.h"
#include "comTwi.h"
#include "timer.h"

#ifdef PIDDBG
	int8_t dbgbuf[128];
	uint8_t idx = 0;
#endif


/* currently no power-savings */

//top speed 80 (93 @ 6,2V)



int main(void)
{
	/* Watchdog */
	wdt_enable(WDTO_1S);
	wdt_reset();

	/* Init hardware */
	LED_Init();
	ADC_Init();
	PM_Init();
	S_Init();
	MC_Init();
	cTwi_Init(0xD2);
	T_Init();

	/* Enabel interrupts */
	sei();

#ifdef PIDDBG
	MC_rVelo0 = 40;
#endif

	while(1)
	{
		if(timer)
		{
			/* major priority */
			timer = 0;
			MC_Control();

#ifdef PIDDBG
			if(idx < 128)
				dbgbuf[idx++] = MC_velo0;
			else if(idx == 128)
			{
				MC_rVelo0 = 0;
				idx++;
			}
#endif
		}
		else
		{
			if(!S_Poll())		//sensors
			{
				static uint8_t rr = 0;
				if(rr++ & 1)
					PM_Poll();	//power management if sensors busy
				else
					MC_Convert();

				S_Adv();	//Advanced Sensor abilities
			}
		}

		wdt_reset();
	}
	return 0;
}
