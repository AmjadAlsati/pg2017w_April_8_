/*
 * ad.c
 *
 *  Created on: 26.10.2011
 *      Author: sid
 */

#include <avr/io.h>
#include <inttypes.h>

#include "flexport.h"
#include "adc.h"

uint16_t ADC_Buf[ADC_WIDTH];
uint8_t channel;

/* ADC initialisieren */
void ADC_Init(void)
{
	channel = BIT(AD_LAST);
	ADMUX = (1 << REFS0); 				// Vref = Vcc
	ADCSRA = (1 << ADEN) | (7 << ADPS0); 		// enable | div 128

	ADC_Buf[ADC_CHBAT] = 1024;			//tweak Vcc
}

int8_t ADC_Poll(void)
{
	int8_t ret;

	if(ADCSRA & (1 << ADSC))			// ADC still working
		return -1;

	ret = channel - BIT(AD_FIRST);

	ADC_Buf[channel - BIT(AD_FIRST)] += ADCW;		// sliding average
	ADC_Buf[channel - BIT(AD_FIRST)] >>=1;

	if(++channel > BIT(AD_LAST))
		channel = BIT(AD_FIRST);

	ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);	// select new channel
	ADCSRA |= (1 << ADSC);				// start conversion

	return ret;
}
