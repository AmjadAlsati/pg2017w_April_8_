/*
 * ad.h
 *
 *  Created on: 26.10.2011
 *      Author: sid
 */

#ifndef ADC_H_
#define ADC_H_

#define AD_BAT		A,1
#define AD_S0		A,2
#define AD_S1		A,3
#define AD_S2		A,4
#define AD_S3		A,5
#define AD_S4		A,6
#define AD_S5		A,7

#define AD_FIRST	AD_BAT
#define AD_LAST		AD_S5

#define ADC_WIDTH	7
#define ADC_CHBAT	0
#define ADC_CH0		1
#define ADC_CH1		2
#define ADC_CH2		3
#define ADC_CH3		4
#define ADC_CH4		5
#define ADC_CH5		6

extern uint16_t ADC_Buf[ADC_WIDTH];

void ADC_Init(void);

int8_t ADC_Poll(void);

#endif /* AD_H_ */
