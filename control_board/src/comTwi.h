/*
 * comTwi.h
 *
 *  Created on: 09.05.2012
 *      Author: sid
 */

#ifndef COMTWI_H_
#define COMTWI_H_

//ACK nach empfangenen Daten senden/ ACK nach gesendeten Daten erwarten
#define TWCR_ACK 	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC)

//NACK nach empfangenen Daten senden/ NACK nach gesendeten Daten erwarten
#define TWCR_NACK 	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC)

//switched to the non addressed slave mode...
#define TWCR_RESET 	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC)

/* Different interrup usage */
#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
	#error "This library requires AVR-GCC 3.4.5 or later, update to newer AVR-GCC compiler !"
#endif

#define CTWI_INT		C,2
#define CTWI_INTact		CLR(CTWI_INT)
#define CTWI_INTdis		SET(CTWI_INT)

#define CTWI_SENSORS		0x10				//current sensor values		(Digits (0..4 = Bumpers, 5 = OVerheat), Vcc (ADC0), Sensor1 (ADC1) .. Sensor 6 (ADC6))
#define CTWI_SENS_INT		0x11				//Thresholds for INT signal
#define CTWI_SENS_MOFF		0x12				//Thresholds for motor off
#define CTWI_SENSORS_L		sizeof(S_sens)
#define CTWI_VELO		0x1A				//velocity of motors		(velo0, velo1)
#define CTWI_VELO_L		2
#define CTWI_POS		0x1B				//position of the platform	(struct Position)
#define CTWI_POS_L		sizeof(struct Position)
#define CTWI_GO			0x1C				// go forward/backward (1 parameter: value in cm) 
#define CTWI_GO_L		1
#define CTWI_TURN		0x1D				// turn a certain degree left or right (1 parameter: value in 0-90? degree)
#define CTWI_TURN_L		1
#define CTWI_LED		0x20				//leds (red,blue (values: 0..63))
#define CTWI_LED_L		2

#ifdef PIDDBG
	#define CTWI_DBG		0x70
	#define CTWI_DBG_L		128
	extern int8_t dbgbuf[128];
#endif

void cTwi_Init(uint8_t addr);

#endif /* COMTWI_H_ */
