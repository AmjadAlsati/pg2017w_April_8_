/*
 * led.c
 *
 *  Created on: 14.04.2012
 *      Author: sid
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "flexport.h"
#include "led.h"

const uint8_t pwmTable[64] PROGMEM = {
		0, 0, 0, 0, 1, 1, 1, 2, 3, 4, 4, 5, 7, 8, 9,
		11, 13, 14, 16, 18, 20, 23, 25, 28, 31, 33,
		36, 40, 43, 46, 50, 54, 57, 61, 66, 70, 74,
		79, 84, 89, 94, 99, 105, 110, 116, 122, 128,
		134, 140, 147, 153, 160, 167, 174, 182, 189,
		197, 205, 213, 221, 229, 238, 246, 255
};

void LED_Init(void)
{
	OUTPUT(LED0p);
	OUTPUT(LED1p);

	TCCR2A = (1<<COM2A1) | (1<<COM2B1) | (1<<WGM20);	//clr on up, set on down | PWM phase correct
	TCCR2B = (3<<CS20);					//div 32 -> 2,4kHz @ 20Mhz

	LEDred(0);
	LEDblue(0);
}

void LEDblue(uint8_t b)
{
	if(b > 63)
		b = 63;
	OCR2A=pgm_read_byte(pwmTable + b);
}

void LEDred(uint8_t r)
{
	if(r > 63)
		r = 63;
	OCR2B=pgm_read_byte(pwmTable + r);
}
