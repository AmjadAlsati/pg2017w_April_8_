/*
 * mCtl.h
 *
 *  Created on: 14.04.2012
 *      Author: sid
 */

#ifndef MCTL_H_
#define MCTL_H_

#define MC_S0A		D,0		//INT0
#define MC_S0A_SIG	PCINT24
#define MC_S0B		D,2
#define MC_S0B_SIG	PCINT26
#define MC_S1A		D,3		//INT1
#define MC_S1A_SIG	PCINT27
#define MC_S1B		D,1
#define MC_S1B_SIG	PCINT25

#define MC_PCREG	PCMSK3
#define MC_PC_SIG	PCINT3_vect
#define MC_PC_BIT	PCIE3

/* PID */
/*#define MC_Kp		22
#define MC_Ki		5
#define MC_Kd		1
*/
#define MC_Kp		12
#define MC_Ki		1
#define MC_Kd		6
#define MC_Imax		1024


/* Drive */
#define MC_TURN_VELO	13
#define MC_GO_VELO	15

#define DURCHMESSER_REIFEN_in_mm	90.0f
#define ABSTAND_REIFEN_in_mm		190.0f
#define IMPULES_PRO_U_Encoder		48
#define GETRIEBE_UNTERSETZ		34.014f

/* ODO */
//#define UMFANG_REIFEN_in_mm		(DURCHMESSER_REIFEN_in_mm * M_PI)
#define WHEEL_DIST_RAW			1097.1f //(((GETRIEBE_UNTERSETZ * IMPULES_PRO_U_Encoder) / UMFANG_REIFEN_in_mm) * ABSTAND_REIFEN_in_mm)
#define TICKTS_TO_CM			57.744f //(((GETRIEBE_UNTERSETZ * IMPULES_PRO_U_Encoder) / UMFANG_REIFEN_in_mm) * 10

extern volatile int8_t MC_encDelta0, MC_encDelta1;
extern volatile int8_t MC_velo0, MC_velo1;
extern volatile int8_t MC_rVelo0, MC_rVelo1;
extern volatile uint16_t MC_duration;
extern struct Position MC_Odo,MC_OdoIn;
extern uint8_t MC_updatePos;

//#define MC_INSERT_ENC_TO(a,b)	{/*cli();*/ a = MC_encDelta0; b = MC_encDelta1; MC_encDelta0 = 0; MC_encDelta1 = 0; /*sei();*/}

struct Position
{
	int16_t x;
	int16_t y;
	int16_t alpha;
};

struct PositionLong
{
	int32_t x;
	int32_t y;
	float alpha;
	uint8_t changed;
};

inline void __attribute__((always_inline)) MC_updateVelo(void)
{
	MC_velo0 = MC_encDelta0;
	MC_velo1 = MC_encDelta1;
	MC_encDelta0 = 0;
	MC_encDelta1 = 0;
}

void MC_Init(void);

void MC_Control(void);

void MC_Convert(void);

#endif /* MCTL_H_ */
