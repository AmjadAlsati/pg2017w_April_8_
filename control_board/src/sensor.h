/*
 * sensor.h
 *
 *  Created on: 18.04.2012
 *      Author: sid
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "adc.h"

#define S_0	B,0		//Free
#define S_1	B,1		//Top right
#define S_2	B,2		//Top left
#define S_3	B,3		//Bottom right
#define S_4	B,4		//Bottom left

#define S_PCREG	PCMSK1
#define S_PCIE	PCIE1
#define S_SIG	PCINT1_vect

struct S_Sensors
{
	uint8_t bumpers;
	uint8_t analog[ADC_WIDTH];
};

extern struct S_Sensors S_sens;
extern struct S_Sensors S_int, S_moff;

void S_Init(void);

uint8_t S_GetBumpers(void);

uint8_t S_Poll(void);

void S_Adv(void);

#endif /* SENSOR_H_ */
