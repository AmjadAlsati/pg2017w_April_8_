/*
 * mCtl.c
 *
 *  Created on: 14.04.2012
 *      Author: sid
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#include "flexport.h"
#include "mCtl.h"
#include "mBridge.h"
#include "led.h"

volatile int8_t last0, last1;
volatile int8_t MC_encDelta0, MC_encDelta1;

volatile int8_t MC_velo0, MC_velo1;
volatile int8_t MC_rVelo0, MC_rVelo1;

volatile uint16_t MC_duration;


struct PositionLong MC_OdoRaw;
struct Position MC_Odo;
struct Position MC_OdoIn;
uint8_t MC_updatePos = 0;


ISR(MC_PC_SIG)
{
	  int8_t new, diff;

	  /* Channel 0 */
	  new = 0;
	  if(GET(MC_S0A))
		  new = 3;
	  if(GET(MC_S0B))
		  new ^= 1;
	  diff = last0 - new;           		// something changed?
	  if( diff & 1 )
	  {
		  last0 = new;                 		// store new as next last
		  MC_encDelta0 += (diff & 2) - 1;
	  }

	  /* Channel 1 */
	  new = 0;
	  if(GET(MC_S1A))
		  new = 3;
	  if(GET(MC_S1B))
		  new ^= 1;
	  diff = last1 - new;           		// something changed?
	  if( diff & 1 )
	  {
		  last1 = new;                 		// store new as next last
		  MC_encDelta1 += (diff & 2) - 1;
	  }
}

void MC_Init(void)
{
	INPUT(MC_S0A);
	INPUT(MC_S0B);
	INPUT(MC_S1A);
	INPUT(MC_S1B);

	last0 = 0;		// power on state 0
	if(GET(MC_S0A))
		last0 = 3;
	if(GET(MC_S0B))
		last0 ^= 1;
	MC_encDelta0 = 0;

	last1 = 0;		// power on state 1
	if(GET(MC_S1A))
		last1 = 3;
	if(GET(MC_S1B))
		last1 ^= 1;
	MC_encDelta1 = 0;

	MC_rVelo0 = 0;
	MC_rVelo1 = 0;

	MC_PCREG = (1<<MC_S0A_SIG)|(1<<MC_S0B_SIG)|(1<<MC_S1A_SIG)|(1<<MC_S1B_SIG);
	PCICR |= 1<<MC_PC_BIT;

	MB_Init();

	MC_OdoRaw.x = 0;
	MC_OdoRaw.y = 0;
	MC_OdoRaw.alpha = 0.0f;
}

void MC_Control(void)
{
	/* PID */
	int16_t e0, e1;
	int16_t e0_d, e1_d;
	static int16_t e0_dLast, e1_dLast;
	static int16_t e0_i, e1_i;
	int16_t u0, u1;

	/* Auto drive */
	cli();
	if(MC_duration)
	{
		MC_duration--;
		if(MC_duration == 0)
		{
			MC_rVelo0 = 0;
			MC_rVelo1 = 0;
		}
	}
	sei();

	/* ODO */
	float dist, angle;

	/****** PID ******/
	//E
	e0 = MC_rVelo0 - MC_velo0;
	e1 = MC_rVelo1 - MC_velo1;


	//D element
	e0_d = e0 - e0_dLast;
	e1_d = e1 - e1_dLast;
	e0_dLast = e0;
	e1_dLast = e1;

	//I element
	e0_i += e0;
	e1_i += e1;
	if(e0_i < -MC_Imax)
		e0_i = -MC_Imax;
	if(e0_i > MC_Imax)
		e0_i = MC_Imax;
	if(e1_i < -MC_Imax)
		e1_i = -MC_Imax;
	if(e1_i > MC_Imax)
		e1_i = MC_Imax;
	if(MC_rVelo0 == 0 && MC_velo0 < 10 && MC_velo0 > -10)
		e0_i = 0;
	if(MC_rVelo1 == 0 && MC_velo1 < 10 && MC_velo1 > -10)
		e1_i = 0;

	//U
	u0 = (MC_Kp * e0) + (MC_Ki * e0_i) + (MC_Kd * e0_d);
	u1 = (MC_Kp * e1) + (MC_Ki * e1_i) + (MC_Kd * e1_d);
	if(u0 > MB_PWMMASK)
		u0 = MB_PWMMASK;
	if(u0 < -MB_PWMMASK)
		u0 = -MB_PWMMASK;
	if(u1 > MB_PWMMASK)
		u1 = MB_PWMMASK;
	if(u1 < -MB_PWMMASK)
		u1 = -MB_PWMMASK;

	//avoid oscilations
	if(MC_rVelo0 == 0 && MC_velo0 < 4 && MC_velo0 > -4)
		u0 = 0;
	if(MC_rVelo1 == 0 && MC_velo1 < 4 && MC_velo1 > -4)
		u1 = 0;

	MB_Pwr0(u0);
	MB_Pwr1(u1);
	/*****************/


	/****** ODO ******/
	//relative movement
	dist = ((float)(MC_velo0 + MC_velo1)) / 2.0f;
	angle = ((float)(MC_velo0 - MC_velo1)) / (WHEEL_DIST_RAW); //tan - kleinwinkel naeherung

	//position increment
	MC_OdoRaw.x += (int32_t) (dist * cos(MC_OdoRaw.alpha));
	MC_OdoRaw.y += (int32_t) (dist * sin(MC_OdoRaw.alpha));
	MC_OdoRaw.alpha += angle;

	if(MC_OdoRaw.alpha > M_PI)
		MC_OdoRaw.alpha -= 2 * M_PI;
	if(MC_OdoRaw.alpha < -M_PI)
		MC_OdoRaw.alpha += 2 * M_PI;

	MC_OdoRaw.changed = 1;
	/*****************/
}

void MC_Convert(void)
{
	if(MC_updatePos)
	{
		MC_OdoRaw.x = MC_OdoIn.x * TICKTS_TO_CM;
		MC_OdoRaw.y = MC_OdoIn.y * TICKTS_TO_CM;
		MC_OdoRaw.alpha = ((float)MC_OdoIn.alpha) / 57.2957795f;
		MC_updatePos = 0;
		MC_OdoRaw.changed = 1;
	}
	if(MC_OdoRaw.changed)
	{
		MC_Odo.x = (int16_t) (((float)MC_OdoRaw.x) / TICKTS_TO_CM);
		MC_Odo.y = (int16_t) (((float)MC_OdoRaw.y) / TICKTS_TO_CM);
		MC_Odo.alpha = (int16_t) (MC_OdoRaw.alpha * 57.2957795f);
		MC_OdoRaw.changed = 0;
	}
}

