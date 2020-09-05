#ifndef		_motor_h_
#define		_motor_h_
#include "main.h"
#include "tim.h"

typedef enum{
	FRONT_LEFT,
	FRONT_RIGHT,
	BACK_LEFT,
	BACK_RIGHT
}Wheel_Typedef;

typedef enum{
	FORWARD,
	BACKWARD,
	BRAKING,
	FLOATING
}Direction_Typedef;

extern int32_t enc3;
extern short l_enc3;				/* encoder value at now */
extern short l_pre_enc3;			/* encoder value at 1s ago */
extern int16_t l_cnt3;		/* revs of the value range */

void speed_run(Wheel_Typedef wheel, int16_t speed);
void CountPulse(short l_enc, short l_pre_enc, short *l_cnt, int32_t* enc, TIM_HandleTypeDef* htim);
#endif
