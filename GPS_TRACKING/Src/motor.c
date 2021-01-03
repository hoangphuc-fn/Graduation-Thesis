#include "motor.h"

int32_t enc_FL = 0;
short l_enc_FL = 0; /* encoder value at now */
short l_pre_enc_FL = 0; /* encoder value at 1s ago */
int16_t l_cnt_FL = 0; /* revs of the value range */
//---------
//int32_t enc_FR = 0;
short l_enc_FR = 0; /* encoder value at now */
//short l_pre_enc_FR = 0; /* encoder value at 1s ago */
//int16_t l_cnt_FR = 0; /* revs of the value range */
//---------
int32_t enc_BL = 0;
short l_enc_BL = 0; /* encoder value at now */
short l_pre_enc_BL = 0; /* encoder value at 1s ago */
int16_t l_cnt_BL = 0; /* revs of the value range */
//---------
int32_t enc_BR = 0;
short l_enc_BR = 0; /* encoder value at now */
short l_pre_enc_BR = 0; /* encoder value at 1s ago */
int16_t l_cnt_BR = 0; /* revs of the value range */
//---------
PID pidFL;
PID pidFR;
PID pidBL;
PID pidBR;

uint8_t running_type;

void speed_run(Wheel_Typedef wheel, int16_t speed) {
	if (speed > 0 && speed <= 1000) { // FORWARD
		switch (wheel) {
		case FRONT_LEFT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed);
			break;
		case FRONT_RIGHT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, speed);
			break;
		case BACK_LEFT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, speed);
			break;
		case BACK_RIGHT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, speed);
			break;
		}
	} else if (speed < 0 && speed >= -1000) {	// BACKWARD
		switch (wheel) {
		case FRONT_LEFT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, -speed);
			break;
		case FRONT_RIGHT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, -speed);
			break;
		case BACK_LEFT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, -speed);
			break;
		case BACK_RIGHT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, -speed);
			break;
		}
	} else if (speed == 9999) {	// BRAKING
		switch (wheel) {
		case FRONT_LEFT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			break;
		case FRONT_RIGHT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			break;
		case BACK_LEFT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case BACK_RIGHT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			break;
		}
	} else if (speed == 0) { // FLOATING
		switch (wheel) {
		case FRONT_LEFT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			break;
		case FRONT_RIGHT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
			break;
		case BACK_LEFT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case BACK_RIGHT:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			break;
		}
	}
}

void run_straight(int16_t target, int16_t actual, int16_t speed,
		int8_t base_diff, uint8_t diff_speed) {
	running_type = RUN_STRAIGHT;
//	if (actual - target >= 1) {
//		pidFL._setPoint = speed;
//		pidFR._setPoint = speed + base_diff + diff_speed;
//		pidBL._setPoint = speed + base_diff;
//		pidBR._setPoint = speed + diff_speed;
//	} else if (actual - target <= -2.5) {
//		pidFL._setPoint = speed - base_diff;
//		pidFR._setPoint = speed + base_diff;
//		pidBL._setPoint = speed + base_diff + diff_speed;
//		pidBR._setPoint = speed - base_diff;
//	} else {
//		pidFL._setPoint = speed - base_diff;
//		pidFR._setPoint = speed + base_diff;
//		pidBL._setPoint = speed + base_diff;
//		pidBR._setPoint = speed - base_diff;
//	}
//	if (actual - target >= 1.5) {
//		pidFL._setPoint = speed;
//		pidFR._setPoint = speed + diff_speed;
//		pidBL._setPoint = speed;
//		pidBR._setPoint = speed + diff_speed;
//	} else if (actual - target <= -2.5) {
//		pidFL._setPoint = speed + diff_speed;
//		pidFR._setPoint = speed;
//		pidBL._setPoint = speed + diff_speed;
//		pidBR._setPoint = speed;
//	} else {
//		pidFL._setPoint = speed - diff_speed;
//		pidFR._setPoint = speed + diff_speed;
//		pidBL._setPoint = speed + diff_speed;
//		pidBR._setPoint = speed - diff_speed;
//	}
	if (actual - target >= 1.5) {
		pidFL._setPoint = speed - diff_speed;
		pidFR._setPoint = speed + diff_speed;
		pidBL._setPoint = speed - diff_speed;
		pidBR._setPoint = speed + diff_speed;
	} else if (actual - target <= -2.5) {
		pidFL._setPoint = speed + diff_speed;
		pidFR._setPoint = speed - diff_speed;
		pidBL._setPoint = speed + diff_speed;
		pidBR._setPoint = speed - diff_speed;
	} else {
		pidFL._setPoint = speed;
		pidFR._setPoint = speed;
		pidBL._setPoint = speed;
		pidBR._setPoint = speed;
	}
}
void run_shift_left(int16_t target, int16_t actual, int16_t speed,
		uint8_t diff_speed) {
	running_type = SHILF_LEFT;

	if (actual - target >= 1) {
		pidFL._setPoint = -(speed + diff_speed);
		pidFR._setPoint = speed;
		pidBL._setPoint = speed;
		pidBR._setPoint = -speed;
	} else if (actual - target <= -1) {
		pidFL._setPoint = -speed;
		pidFR._setPoint = speed;
		pidBL._setPoint = speed + diff_speed;
		pidBR._setPoint = -speed;
	} else {
		pidFL._setPoint = -speed;
		pidFR._setPoint = speed;
		pidBL._setPoint = speed;
		pidBR._setPoint = -speed;
	}
}
void run_shift_right(int16_t target, int16_t actual, int16_t speed,
		uint8_t diff_speed) {
	running_type = SHIFT_RIGHT;
	if (actual - target >= 1) {
		pidFL._setPoint = speed;
		pidFR._setPoint = -speed;
		pidBL._setPoint = -(speed + diff_speed);
		pidBR._setPoint = speed + diff_speed;
	} else if (actual - target <= -1) {
		pidFL._setPoint = speed + diff_speed;
		pidFR._setPoint = -(speed + diff_speed);
		pidBL._setPoint = -speed;
		pidBR._setPoint = speed;
	} else {
		pidFL._setPoint = speed;
		pidFR._setPoint = -speed;
		pidBL._setPoint = -speed;
		pidBR._setPoint = speed + diff_speed;
	}
}

void CountPulse(short l_enc, short l_pre_enc, short *l_cnt, int32_t *enc,
		TIM_HandleTypeDef *htim) {
	if (l_enc < 0 && l_pre_enc > 15000) {
		/* from 32767 -> -32768 => increase round counter */
		(*l_cnt)++;
		__HAL_TIM_SET_COUNTER(htim, 0);
	} else if (l_enc > 0 && l_pre_enc < -15000) {
		/* from -32768 -> 32767 => decrease round counter */
		(*l_cnt)--;
		__HAL_TIM_SET_COUNTER(htim, 0);
	}
	/* The actual number of pulses from the encoder */
	*enc = *l_cnt * 32768 + l_enc;
}

PID newPID(float kP, float kI, float kD) {
	PID A;
	A._kP = kP;
	A._kI = kI;
	A._kD = kD;
	A._input = 0;
	A._err = 0;
	A._preErr = 0;
	A._outP = 0;
	A._outI = 0;
	A._outD = 0;
	A._output = 0;
	return A;
}

void computePID(PID *A, int setPoint) {
	A->_err = setPoint - A->_input;
	A->_outP = A->_kP * A->_err;
	A->_outI += (A->_kI * (A->_err + A->_preErr));
	A->_outD = A->_kD * (A->_err - A->_preErr);
	A->_output += (A->_outP + A->_outI + A->_outD);
	if (A->_output >= 1000) {
		A->_output = 1000;
	} else if (A->_output <= -1000) {
		A->_output = -1000;
	}
	A->_preErr = A->_err;
}
