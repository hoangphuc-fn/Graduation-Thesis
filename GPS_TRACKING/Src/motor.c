#include "motor.h"

int32_t enc3 = 0;
short l_enc3 = 0; /* encoder value at now */
short l_pre_enc3 = 0; /* encoder value at 1s ago */
int16_t l_cnt3 = 0; /* revs of the value range */

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
	} else if (speed == 0) {	// BRAKING
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
	} else { // FLOATING
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
