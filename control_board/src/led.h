/*
 * led.h
 *
 *  Created on: 14.04.2012
 *      Author: sid
 */

#ifndef LED_H_
#define LED_H_

#define LED0p		D,7	//OCR2A
#define LED1p		D,6	//OCR2B

void LED_Init(void);
void LEDblue(uint8_t b);
void LEDred(uint8_t r);
#endif /* LED_H_ */
