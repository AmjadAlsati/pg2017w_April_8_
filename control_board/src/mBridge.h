/*
 * mBridge.h
 *
 *  Created on: 14.04.2012
 *      Author: sid
 */

#ifndef MBRIDGE_H_
#define MBRIDGE_H_

#define MB_M0A		C,6
#define MB_M0B		C,5
#define MB_PWM0PIN	D,5	//OC1A
#define MB_PWM0REG	OCR1A

#define MB_M1A		C,4
#define MB_M1B		C,3
#define MB_PWM1PIN	D,4	//OC1B
#define MB_PWM1REG	OCR1B

#define MB_DIAG		C,7

#define MB_Set0(a)	MB_PWM0REG=a
#define MB_Set1(b)	MB_PWM1REG=b

#define MB_PWMMASK	0x3FF 	//10bit (0-1023); 19,4khz @ 20Mhz		//0x7FF	//11bit (0-2047); 9,7khz @ 20Mhz

void MB_Init(void);

void MB_Disable(void);
void MB_Enable(void);
uint8_t MB_isEnabled(void);

inline void MB_Pwr0(int16_t p)
{
	if(p > 0)
	{
		SET(MB_M0A);
		CLR(MB_M0B);
	}
	else if(p < 0)
	{
		CLR(MB_M0A);
		SET(MB_M0B);
		p = -p;
	}
	else
	{
		SET(MB_M0A);
		SET(MB_M0B);
		//p = MB_PWMMASK;
	}

	MB_Set0(p&MB_PWMMASK);
}

inline void MB_Pwr1(int16_t p)
{
	if(p > 0)
	{
		SET(MB_M1A);
		CLR(MB_M1B);
	}
	else if(p < 0)
	{
		CLR(MB_M1A);
		SET(MB_M1B);
		p = -p;
	}
	else
	{
		SET(MB_M1A);
		SET(MB_M1B);
		//p = MB_PWMMASK;
	}

	MB_Set1(p&MB_PWMMASK);
}

#endif /* MBRIDGE_H_ */
