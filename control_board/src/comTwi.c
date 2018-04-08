/*
 * comTwi.c
 *
 *  Created on: 09.05.2012
 *      Author: sid
 */

#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#include "comTwi.h"
#include "sensor.h"
#include "mCtl.h"
#include "led.h"
#include "flexport.h"


volatile uint8_t cTwi_Buf[20];

volatile uint8_t *cTwi_Data = cTwi_Buf;
volatile uint8_t cTwi_Count = 0;
volatile uint8_t cTwi_RxCmd;
volatile uint8_t cTwi_DataLength = 20;

struct Position *PosPtr;

ISR (TWI_vect)
{
	uint8_t data=0;
	switch (TW_STATUS) 								// TWI-Statusregister prüfen und nötige Aktion bestimmen
		{
		case TW_SR_SLA_ACK: 							//SLA+W received, ACK returned
			TWCR_ACK;
			cTwi_Count = 0;
			break;

		case TW_SR_DATA_ACK: 							// data received, ACK returned
			data = TWDR;
			if(cTwi_Count == 0)						//first received data
			{
				cTwi_RxCmd = data;
				switch(cTwi_RxCmd)					//decide operation
				{
				case CTWI_SENSORS:					//get sensor readings
					cTwi_Data = (void*) &S_sens;
					S_GetBumpers();
					cTwi_DataLength = CTWI_SENSORS_L;
					TWCR_ACK;
					break;
				case CTWI_SENS_INT:					//get/set sensor int line boards
					cTwi_Data = (void*) &S_int;
					cTwi_DataLength = CTWI_SENSORS_L;
					TWCR_ACK;
					break;
				case CTWI_SENS_MOFF:					//get/set sensor motor off boards
					cTwi_Data = (void*) &S_moff;
					cTwi_DataLength = CTWI_SENSORS_L;
					TWCR_ACK;
					break;
				case CTWI_VELO:						//get/set velocity data
					cTwi_Data = cTwi_Buf;
					cTwi_Buf[0] = MC_velo0;
					cTwi_Buf[1] = MC_velo1;
					cTwi_DataLength = CTWI_VELO_L;
					TWCR_ACK;
					break;
				case CTWI_POS:						//get/set odometry position
					PosPtr = (void*) cTwi_Buf;
					PosPtr->x = MC_Odo.x;
					PosPtr->y = MC_Odo.y;
					PosPtr->alpha = MC_Odo.alpha;
					cTwi_Data = cTwi_Buf;
					cTwi_DataLength = CTWI_POS_L;
					TWCR_ACK;
					break;
				case CTWI_TURN:						//get/set drive
				case CTWI_GO:
					cTwi_Data = cTwi_Buf;
					if(MC_rVelo0 != MC_rVelo1)
						cTwi_Buf[0] = MC_duration;
					else
						cTwi_Buf[0] = MC_duration >> 2;
					cTwi_DataLength = CTWI_TURN_L;
					TWCR_ACK;
					break;
				case CTWI_LED:						//set led
					cTwi_Data = cTwi_Buf;
					cTwi_DataLength = CTWI_LED_L;
					TWCR_ACK;
					break;
#ifdef PIDDBG
				case CTWI_DBG:						//get dgb buffer
					cTwi_Data = (void*) &dbgbuf;
					cTwi_DataLength = CTWI_DBG_L;
					TWCR_ACK;
					break;
#endif
				default:
					TWCR_NACK;					//unknown command
				}
			}
			else 								// further received data
			{
				switch(cTwi_RxCmd)
				{
				case CTWI_LED:
					if(cTwi_Count == 1)
						LEDred(data);
					else if(cTwi_Count == 2)
						LEDblue(data);

					if(cTwi_Count < 3)
						TWCR_ACK;
					else
						TWCR_NACK;
					break;
				case CTWI_VELO:
					if(cTwi_Count == 1)
					{
						cTwi_Buf[0] = data;
					}
					else if(cTwi_Count == 2)
					{
						MC_duration = 0;
						MC_rVelo0 = cTwi_Buf[0];
						MC_rVelo1 = data;
					}
					if(cTwi_Count < 3)
						TWCR_ACK;
					else
						TWCR_NACK;
					break;
				case CTWI_SENS_INT:
				case CTWI_SENS_MOFF:
					cTwi_Data[cTwi_Count-1] = data;
					if(cTwi_Count > cTwi_DataLength) 				// buffer end reached
					{
						TWCR_NACK;
						cTwi_Count = 0;
					}
					else
					{
						TWCR_ACK;
					}
					break;
				case CTWI_POS:
					cTwi_Data[cTwi_Count-1] = data;
					if(cTwi_Count == sizeof(struct Position))
					{
						MC_OdoIn.x = PosPtr->x;
						MC_OdoIn.y = PosPtr->y;
						MC_OdoIn.alpha = PosPtr->alpha;
						MC_updatePos = 1;
					}
					if(cTwi_Count > cTwi_DataLength) 				// buffer end reached
					{
						TWCR_NACK;
						cTwi_Count = 0;
					}
					else
					{
						TWCR_ACK;
					}
					break;
				case CTWI_TURN:
					if(cTwi_Count == 1)
					{
						int8_t velo = data;
						if(velo > 0)
						{
							MC_rVelo0 = MC_TURN_VELO;
							MC_rVelo1 = -MC_TURN_VELO;
						}
						else if(velo < 0)
						{
							MC_rVelo0 = -MC_TURN_VELO;
							MC_rVelo1 = MC_TURN_VELO;
							velo = -velo;
						}
						else
						{
							MC_rVelo0 = 0;
							MC_rVelo1 = 0;
						}
						MC_duration = velo + 1;
					}
					if(cTwi_Count > cTwi_DataLength) 				// buffer end reached
					{
						TWCR_NACK;
						cTwi_Count = 0;
					}
					else
					{
						TWCR_ACK;
					}
					break;
					case CTWI_GO:
						if(cTwi_Count == 1)
						{
							int8_t velo = data;
							if(velo > 0)
							{
								MC_rVelo0 = MC_GO_VELO;
								MC_rVelo1 = MC_GO_VELO;
							}
							else if(velo < 0)
							{
								MC_rVelo0 = -MC_GO_VELO;
								MC_rVelo1 = -MC_GO_VELO;
								velo = -velo;
							}
							else
							{
								MC_rVelo0 = 0;
								MC_rVelo1 = 0;
							}
							MC_duration = (velo<<2) + 1;
						}
						if(cTwi_Count > cTwi_DataLength) 				// buffer end reached
						{
							TWCR_NACK;
							cTwi_Count = 0;
						}
						else
						{
							TWCR_ACK;
						}
						break;
				default:						//no further data required
					TWCR_NACK;
				}
			}
			cTwi_Count++;
			break;

		case TW_ST_SLA_ACK: 							// SLA+R received, ACK returned
			cTwi_Count = 0;
		case TW_ST_DATA_ACK: 							// data transmitted, ACK received
			TWDR = cTwi_Data[cTwi_Count++]; 				// send data
			if(cTwi_Count >= cTwi_DataLength) 				// buffer end reached
			{
				TWCR_NACK;
				cTwi_Count = 0;
			}
			else
			{
				TWCR_ACK;
			}
			break;
		case TW_SR_STOP: 							//stop or repeated start condition received while selected
			TWCR |= (1<<TWINT);
			break;

		case TW_ST_DATA_NACK: 							// data transmitted, NACK received
		case TW_SR_DATA_NACK: 							// 0x88
		case TW_ST_LAST_DATA: 							// 0xC8  Last data byte in TWDR has been transmitted; ACK has been received
		default:
			TWCR_RESET; 							// Übertragung beenden, warten bis zur nächsten Adressierung
			break;
		}
}

void cTwi_Init(uint8_t addr)
{
	TWAR = addr;
	TWCR &= ~(1<<TWSTA)|(1<<TWSTO);
	TWCR |= (1<<TWEA) | (1<<TWEN)|(1<<TWIE);

	CTWI_INTdis;
	OUTPUT(CTWI_INT);
}
