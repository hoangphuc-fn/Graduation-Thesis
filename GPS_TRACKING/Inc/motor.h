#ifndef		_motor_h_
#define		_motor_h_
#include "main.h"
#include "tim.h"
#include "utils.h"
#include <stdbool.h>

typedef enum {
	FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
} Wheel_Typedef;

typedef enum {
	FORWARD, BACKWARD, BRAKING, FLOATING
} Direction_Typedef;

typedef struct pid {
	float _kP;
	float _kI;
	float _kD;
	int _setPoint;
	int _input;
	int _err;
	int _preErr;
	float _outP;
	float _outI;
	float _outD;
	float _output;
} PID;

extern PID pidFL;
extern PID pidFR;
extern PID pidBL;
extern PID pidBR;

extern int32_t enc_FL;
extern short l_enc_FL; /* encoder value at now */
extern short l_pre_enc_FL; /* encoder value at 1s ago */
extern int16_t l_cnt_FL; /* revs of the value range */
//--------
//extern int32_t enc_FR;
extern short l_enc_FR; /* encoder value at now */
//extern short l_pre_enc_FR;			/* encoder value at 1s ago */
//extern int16_t l_cnt_FR;		/* revs of the value range */
//--------
extern int32_t enc_BL;
extern short l_enc_BL; /* encoder value at now */
extern short l_pre_enc_BL; /* encoder value at 1s ago */
extern int16_t l_cnt_BL; /* revs of the value range */
//--------
extern int32_t enc_BR;
extern short l_enc_BR; /* encoder value at now */
extern short l_pre_enc_BR; /* encoder value at 1s ago */
extern int16_t l_cnt_BR; /* revs of the value range */
//---------
extern uint8_t running_type;

typedef enum {
	RUN_STRAIGHT = 0, SHILF_LEFT, SHIFT_RIGHT
} Type_Run_Typedef;

/* Arguments are PWM values */
void speed_run(Wheel_Typedef wheel, int16_t speed);

/* Arguments are PID values
 Values are from -100 to 100*/
void speed_run_pid(int8_t speedFL, int8_t speedFR, int8_t speedBL,
		int8_t speedBR);

void run_following_heading(int16_t target, int16_t actual, int16_t speed,
		int8_t base_diff, uint8_t diff_speed);

void run_shift_left(int16_t target, int16_t actual, int16_t speed,
		uint8_t diff_speed);

void run_shift_right(int16_t target, int16_t actual, int16_t speed,
		uint8_t diff_speed);
bool calib(int16_t target, int16_t actual, int16_t speed, int8_t base_diff,
		uint8_t diff_speed);
bool turnAngle(float target, float feedback, int16_t speed);

void CountPulse(short l_enc, short l_pre_enc, short *l_cnt, int32_t *enc,
		TIM_HandleTypeDef *htim);

PID newPID(float kP, float kI, float kD);
/*setPoint value is from -100 to 100
 if value = 255 or -255 => floating*/
void computePID(PID *A, int8_t setPoint);

#endif
