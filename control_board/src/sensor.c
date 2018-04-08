/*
 * sensor.c
 *
 *  Created on: 18.04.2012
 *      Author: sid
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "flexport.h"
#include "sensor.h"
#include "mCtl.h"
#include "comTwi.h"
#include "mBridge.h"

volatile uint8_t sens;

struct S_Sensors S_sens;
struct S_Sensors S_int, S_moff;

ISR(S_SIG)
{
	sens |=  (PIN(S_0) ^ MASK(S_0, 5)) & MASK(S_0, 5);
}

void S_Init(void)
{
	INPUT(S_0);
	INPUT(S_1);
	INPUT(S_2);
	INPUT(S_3);
	INPUT(S_4);

	SET(S_0);
	SET(S_1);
	SET(S_2);
	SET(S_3);
	SET(S_4);

	S_PCREG = MASK(S_0, 5);
	PCICR = (1<<S_PCIE);

	_delay_ms(100);

	S_GetBumpers();

	S_int.bumpers = 0;
	S_int.analog[0] = 0;
	S_int.analog[1] = 0;
	S_int.analog[2] = 0;
	S_int.analog[3] = 0;
	S_int.analog[4] = 0;
	S_int.analog[5] = 0;
	S_int.analog[6] = 0;

	S_moff.bumpers = 0;
	S_moff.analog[0] = 0;
	S_moff.analog[1] = 0;
	S_moff.analog[2] = 0;
	S_moff.analog[3] = 0;
	S_moff.analog[4] = 0;
	S_moff.analog[5] = 0;
	S_moff.analog[6] = 0;
}

uint8_t S_Poll(void)
{
	int8_t sensor = ADC_Poll();

	if(sensor < 0)
		return 0;

	switch(sensor)
	{
    // case for big sensor in the center of the robot front
	case ADC_CHBAT:
		S_sens.analog[ADC_CHBAT] = ADC_Buf[ADC_CHBAT] / 10.24f;
		break;
	case ADC_CH4:
		if(ADC_Buf[sensor] > 81)
            // received more than the lowest available voltage (0.4V = 81/1023)
            // --> value is useful, let's calculate the actual distance in centimeters by curve fitting from data sheet
			S_sens.analog[sensor] = (13888 / (ADC_Buf[sensor] + 7)) - 5; // (14925 / (ADC_Buf[sensor] + 7)) - 5;
		else
            // received less then the lowest available voltage
            // --> value is not usefull, return infinite distance in centimeters
			S_sens.analog[sensor] = 255;
		break;
    // fall through cases for small sensors
	case ADC_CH0:
	case ADC_CH1:
	case ADC_CH2:
	case ADC_CH3:
		if(ADC_Buf[sensor] > 71)
            // received more than the lowest available voltage (0.4V = 71/1023)
            // // --> value is useful, let's calculate the actual distance in centimeters by curve fitting from data sheet
			S_sens.analog[sensor] = (6250 / (ADC_Buf[sensor] + 2)) - 4; // 6666 / (ADC_Buf[sensor] + 2)) - 4;
		else
            // received less then the lowest available voltage
            // --> value is not usefull, return infinite distance in centimeters
			S_sens.analog[sensor] = 255;
		break;
	default:
		S_sens.analog[sensor] = 255;
	}

	return 1;
}

void S_Adv(void)
{
	if(S_int.bumpers & S_sens.bumpers || S_sens.analog[0] < S_int.analog[0] || S_sens.analog[1] < S_int.analog[1] || S_sens.analog[2] < S_int.analog[2]
	   || S_sens.analog[3] < S_int.analog[3] || S_sens.analog[4] < S_int.analog[4] || S_sens.analog[5] < S_int.analog[5] || S_sens.analog[6] < S_int.analog[6])
		CTWI_INTact;
	else
		CTWI_INTdis;

	if(S_moff.bumpers & S_sens.bumpers || S_sens.analog[0] < S_moff.analog[0] || S_sens.analog[1] < S_moff.analog[1] || S_sens.analog[2] < S_moff.analog[2]
	   || S_sens.analog[3] < S_moff.analog[3] || S_sens.analog[4] < S_moff.analog[4] || S_sens.analog[5] < S_moff.analog[5] || S_sens.analog[6] < S_moff.analog[6])
	{
		MC_rVelo0 = 0;
		MC_rVelo1 = 0;
		MC_duration = 0;
	}
}
uint8_t S_GetBumpers(void)
{
	S_sens.bumpers = sens;
	sens =  ((PIN(S_0) ^ MASK(S_0, 5)) & MASK(S_0, 5)) | ((GET(MB_DIAG)^1) << 5);
	return S_sens.bumpers;
}
