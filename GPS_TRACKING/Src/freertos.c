/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "mjson.h"
#include "DirectionData.h"
#include "hal_i2c_mpu6050.h"
#include "hal_i2c_hmc5883l.h"
#include "kalman.h"
#include "motor.h"

#include "ST7920_SERIAL.h"
#include "bitmap.h"
#include "delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

char strO[50] = "";
char lcdLine00[10] = "";
char lcdLine01[10] = "";
char lcdLine1[50] = "";
char lcdLine2[50] = "";
char lcdLine3[50] = "";
char lcdLine4[50] = "";
char strLat[50] = "";
char strLon[50] = "";

/* For GPS */
double realLat = 0;
double realLon = 0;
uint8_t C;
char gpsData[100];
uint64_t hehe = 0;
uint16_t hihi = 0;
bool isOverFlow = false;

/* For MPU */
extern float gyro_deg[3];
extern float gyro_result[3];
extern float acce_result[3];
extern uint8_t check;
float compass[3] = { 0, 0, 0 };
double headingDegrees, fixedHeadingDegrees;
double headingDegrees2;
double declinationAngle;
double heading;
double heading2;
float accPitch = 0;
float accRoll = 0;
float kalPitch = 0;
float kalRoll = 0;
float yaw_gyro_deg = 0;
bool initGyro = false;
bool mapAngle = false;

uint64_t sec = 0;

/* For Encoder */
short enc1 = 0;
short enc2 = 0;
short enc4 = 0;
/* For PID */

float O_pid;
int setPoint;
int8_t diff = 2;
int16_t speed = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define STR_INDIR(x) #x
#define STR(x) STR_INDIR(x)
#define TEST_INT 123
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId lcdTaskHandle;
osThreadId uartGPSHandle;
osThreadId gpsTaskHandle;
osThreadId mpuTaskHandle;
osThreadId uartESPHandle;
osThreadId motorTaskHandle;
osMutexId gpsDataMutexHandle;
osSemaphoreId gpsDataSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void cvtCoordinates(double tempCoor, double *gCoor);
bool getCoordinates(char rawStr[], double *pLat, double *pLon);
int json_myobj_read(const char *buf, DirectionData *myobj);
void resetArray(char pArr[], uint8_t length);
void resetArray(char pArr[], uint8_t length);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartLcdTask(void const * argument);
void StartUartGPS(void const * argument);
void StartGpsTask(void const * argument);
void StartMPUTask(void const * argument);
void StartUartESP(void const * argument);
void StartMotorTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask,
		signed char *pcTaskName) {
	//char *taskName = malloc(20*sizeof(char));
	//strcpy(taskName, pcTaskName);
	isOverFlow = true;
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of gpsDataMutex */
  osMutexDef(gpsDataMutex);
  gpsDataMutexHandle = osMutexCreate(osMutex(gpsDataMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of gpsDataSem */
  osSemaphoreDef(gpsDataSem);
  gpsDataSemHandle = osSemaphoreCreate(osSemaphore(gpsDataSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of lcdTask */
  osThreadDef(lcdTask, StartLcdTask, osPriorityIdle, 0, 250);
  lcdTaskHandle = osThreadCreate(osThread(lcdTask), NULL);

  /* definition and creation of uartGPS */
  osThreadDef(uartGPS, StartUartGPS, osPriorityRealtime, 0, 128);
  uartGPSHandle = osThreadCreate(osThread(uartGPS), NULL);

  /* definition and creation of gpsTask */
  osThreadDef(gpsTask, StartGpsTask, osPriorityHigh, 0, 200);
  gpsTaskHandle = osThreadCreate(osThread(gpsTask), NULL);

  /* definition and creation of mpuTask */
  osThreadDef(mpuTask, StartMPUTask, osPriorityHigh, 0, 250);
  mpuTaskHandle = osThreadCreate(osThread(mpuTask), NULL);

  /* definition and creation of uartESP */
  osThreadDef(uartESP, StartUartESP, osPriorityIdle, 0, 200);
  uartESPHandle = osThreadCreate(osThread(uartESP), NULL);

  /* definition and creation of motorTask */
  osThreadDef(motorTask, StartMotorTask, osPriorityIdle, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint32_t time_until;
	uint8_t time_sample = 10;
	uint8_t cnt = 0;
	/* Infinite loop */
	while (1) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
			speed += 5;
			HAL_Delay(300);
		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
			break;
		}
		osDelayUntil(&time_until, time_sample);
	}
	for (;;) {
//		run_straight(0, yaw_gyro_deg * 2.5, speed, 25);
		run_shift_left(0, yaw_gyro_deg * 2.5, speed, 25);
//		run_shift_right(0, yaw_gyro_deg * 2.5, speed, 25);
		cnt++;
		if (cnt >= 100) {
			hihi++;
			cnt = 0;
		}
		osDelayUntil(&time_until, time_sample);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLcdTask */
/**
 * @brief Function implementing the lcdTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLcdTask */
void StartLcdTask(void const * argument)
{
  /* USER CODE BEGIN StartLcdTask */
	delay_init();
	ST7920_Init();
	ST7920_Clear();
	/* Infinite loop */
	for (;;) {
		sprintf(lcdLine00, "H:%d  ", (int) headingDegrees2);
		ST7920_SendString(0, 0, lcdLine00);
		sprintf(lcdLine01, "A:%d  ", (int) yaw_gyro_deg);
		ST7920_SendString(0, 3, lcdLine01);
		ST7920_SendString(2, 0, strLat);
		ST7920_SendString(3, 0, strLon);
		osDelay(10);
	}
  /* USER CODE END StartLcdTask */
}

/* USER CODE BEGIN Header_StartUartGPS */
/**
 * @brief Function implementing the uartGPS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUartGPS */
void StartUartGPS(void const * argument)
{
  /* USER CODE BEGIN StartUartGPS */
	uint8_t cnt = 0;
	HAL_UART_Receive_IT(&huart4, &C, 1);
	/* Infinite loop */
	for (;;) {
		osThreadSuspend(uartGPSHandle);

		gpsData[cnt++] = C;
		if (gpsData[cnt - 1] == '\n') {
			if (strstr(gpsData, "GNGGA") != NULL) {
				hehe++;
				sprintf(strO, "count: %d", hehe);
				osSemaphoreRelease(gpsDataSemHandle);
			} else {
				resetArray(gpsData, strlen(gpsData));
			}
			cnt = 0;
		}
		HAL_UART_Receive_IT(&huart4, &C, 1);
	}
  /* USER CODE END StartUartGPS */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
 * @brief Function implementing the gpsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void const * argument)
{
  /* USER CODE BEGIN StartGpsTask */
	char tempStr[100];
	/* Infinite loop */
	for (;;) {
		osSemaphoreWait(gpsDataSemHandle, 2000);
		resetArray(tempStr, strlen(tempStr));
		strcpy(tempStr, gpsData);
		resetArray(gpsData, strlen(gpsData));
		if (getCoordinates(tempStr, &realLat, &realLon)) {
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			sprintf(strLat, "%d.%d", (int) realLat,
					(int) (realLat * 10000000) % 10000000);
			sprintf(strLon, "%d.%d", (int) realLon,
					(int) (realLon * 10000000) % 106000000);
		}
		osDelay(1);
	}
  /* USER CODE END StartGpsTask */
}

/* USER CODE BEGIN Header_StartMPUTask */
/**
 * @brief Function implementing the mpuTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMPUTask */
void StartMPUTask(void const * argument)
{
  /* USER CODE BEGIN StartMPUTask */
	uint32_t time_until;
	uint8_t time_sample = 5;

	while (MPU6050_Init(&hi2c1, MPU6050_DataRate_1KHz, MPU6050_Accelerometer_2G,
			MPU6050_Gyroscope_500s) != MPU6050_Result_Ok) {
		HAL_Delay(100);
	}
	while (!HMC5883L_initialize()) {
		HAL_Delay(100);
	}

	setRange(HMC5883L_RANGE_1_3GA); //HMC5883L_RANGE_8_1GA
	setMeasurementMode(HMC5883L_CONTINOUS);
	setDataRate(HMC5883L_DATARATE_75HZ);
	setSamples(HMC5883L_SAMPLES_8);

	MyKalman kalmanY = newKalman(0.001, 0.003, 0.03);
	MyKalman kalmanX = newKalman(0.001, 0.003, 0.03);
	/* Infinite loop */
	for (;;) {
		/* Gyroscope */
		MPU6050_Read_Gyro(&hi2c1, gyro_result, MPU6050_GYRO_SENS_250);
		for (uint8_t i = 0; i < 3; i++) {
			if (abs(gyro_result[i]) > 0.1) {
				gyro_deg[i] -= gyro_result[i] * time_sample / 1000;
			}
		}
		/* Compass */
		readNormalize(compass);

		/* Accelerometer */
		MPU6050_Read_Accel(&hi2c1, acce_result, MPU6050_ACCE_SENS_2);
		accPitch = -atan2(acce_result[0],
				sqrt(
						acce_result[1] * acce_result[1]
								+ acce_result[2] * acce_result[2]));
		accRoll = atan2(acce_result[1], acce_result[2]);
		kalPitch = kalmanUpdate(&kalmanY, accPitch, gyro_result[1],
				time_sample * 1000);
		kalPitch = kalmanUpdate(&kalmanX, accRoll, gyro_result[0],
				time_sample * 1000);

		float cosRoll = cos(accRoll);
		float sinRoll = sin(accRoll);
		float cosPitch = cos(accPitch);
		float sinPitch = sin(accPitch);

		float Xh = compass[0] * cosPitch + compass[2] * sinPitch;
		float Yh = compass[0] * sinRoll * sinPitch + compass[1] * cosRoll
				- compass[2] * sinRoll * cosPitch;

		heading2 = atan2(Yh, Xh);

		heading = atan2(compass[1], compass[0]);
//		declinationAngle = (0 + (37.0 / 60.0)) / (180 / M_PI);
		declinationAngle = (0 + (39.0 / 60.0)) / (180 / M_PI);
		heading -= declinationAngle;
		heading2 -= declinationAngle;
		if (heading < 0) {
			heading += 2 * M_PI;
			heading2 += 2 * M_PI;
		} else if (heading > 2 * M_PI) {
			heading -= 2 * M_PI;
			heading2 += 2 * M_PI;
		}
		headingDegrees = heading * 180 / M_PI;
		headingDegrees2 = heading2 * 180 / M_PI;

//		if (abs(headingDegrees2 - 320) <= 1.5) {
//		if (1) {
//			initGyro = true;
//			if (!mapAngle) {
//				// Init value for Gyro_z - map angle
//				gyro_deg[2] = headingDegrees2;
//				mapAngle = true;
//			}
//		}
//		if (initGyro) {
		if (1) {
			/* Gyroscope */
			MPU6050_Read_Gyro(&hi2c1, gyro_result, MPU6050_GYRO_SENS_250); //MPU6050_GYRO_SENS_250
			for (uint8_t i = 0; i < 3; i++) {
				if (abs(gyro_result[i]) > 0.01) {
					gyro_deg[i] -= gyro_result[i] * time_sample / 1000;
				}
			}
			int16_t t = gyro_deg[2] / 360;
			yaw_gyro_deg = gyro_deg[2] / 2 - t * 360;
		}
		osDelayUntil(&time_until, time_sample);
	}
  /* USER CODE END StartMPUTask */
}

/* USER CODE BEGIN Header_StartUartESP */
/**
 * @brief Function implementing the uartESP thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUartESP */
void StartUartESP(void const * argument)
{
  /* USER CODE BEGIN StartUartESP */
	uint32_t time_until;
	/* Infinite loop */
	for (;;) {
		//sprintf(data_send, "%d.%d, %d.%d\n",(int)realLat, (int)(realLat*1000000)%10000000, (int)realLon, (int)(realLon*1000000)%106000000);
//		gcvt(temp1, 6, buf1);
//		gcvt(temp2, 8, buf2);
//		sprintf(data_send, "%s,%s\n", buf1,buf2);
//		UART_Print(&huart5,data_send);
//		//UART_Print(&huart5,"235\n");
//		while(HAL_UART_GetState(&huart5)!= HAL_UART_STATE_BUSY_TX);
		osDelayUntil(&time_until, 5);
	}
  /* USER CODE END StartUartESP */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
 * @brief Function implementing the motorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
	uint32_t time_until;
	uint8_t time_sample = 15;
	speed_run(FRONT_LEFT, 0);
	speed_run(FRONT_RIGHT, 0);
	speed_run(BACK_LEFT, 0);
	speed_run(BACK_RIGHT, 0);
	/* init PID */
	pidFL = newPID(0.3, 0, 20); //2 0 10
	pidFR = newPID(0.3, 0, 20);
	pidBL = newPID(0.3, 0, 20);
	pidBR = newPID(0.3, 0, 20);
	/* counter variable to generate interrupt each 1s */
	uint8_t l_cntT = 0;
	/* Infinite loop */
	for (;;) {
		/* each 15ms */
		l_cntT++;

		/* read encoder */
		l_enc_FL = __HAL_TIM_GET_COUNTER(&htim2);
		l_enc_FR = __HAL_TIM_GET_COUNTER(&htim3);
		l_enc_BL = __HAL_TIM_GET_COUNTER(&htim1);
		l_enc_BR = __HAL_TIM_GET_COUNTER(&htim4);

		pidFL._input = l_enc_FL;
		pidFR._input = l_enc_FR;
		pidBL._input = l_enc_BL;
		pidBR._input = l_enc_BR;

		computePID(&pidFL, pidFL._setPoint);
		computePID(&pidFR, pidFR._setPoint);
		computePID(&pidBL, pidBL._setPoint);
		computePID(&pidBR, pidBR._setPoint);

		speed_run(FRONT_LEFT, pidFL._output);
		speed_run(BACK_LEFT, pidBL._output);
		speed_run(BACK_RIGHT, pidBR._output);
		if (running_type == RUN_STRAIGHT) {
			if (abs(pidFR._input) < 5) {
				/* encoder of FR wheel DOES NOT work */
				speed_run(FRONT_RIGHT, pidBR._output);
			} else {
				/* encoder of FR wheel works normally */
				speed_run(FRONT_RIGHT, pidFR._output);
			}
		} else if (running_type == SHILF_LEFT) {
			if (abs(pidFR._input) < 5) {
				/* encoder of FR wheel DOES NOT work */
				speed_run(FRONT_RIGHT, -pidBR._output);
			} else {
				/* encoder of FR wheel works normally */
				speed_run(FRONT_RIGHT, pidFR._output);
			}
		} else if (running_type == SHIFT_RIGHT) {
			if (abs(pidFR._input) < 5) {
				/* encoder of FR wheel DOES NOT work */
				speed_run(FRONT_RIGHT, -pidBR._output);
			} else {
				/* encoder of FR wheel works normally */
				speed_run(FRONT_RIGHT, pidFR._output);
			}
		}

		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		/* each 1s */
		if (l_cntT >= 60) {
			l_cntT = 0;
		}
		osDelayUntil(&time_until, time_sample);
	}
  /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void cvtCoordinates(double tempCoor, double *gCoor) {
	int temp = (int) tempCoor / 100;
	*gCoor = temp + ((tempCoor - 100 * temp) / 60);
}

//bool getCoordinates(char rawStr[], double* pLat, double* pLon){
//    bool isAvailable = false;
//    double lLat = 0;
//    double lLon = 0;
//    if(strstr(rawStr,"$GNGGA")!=NULL){
//        int cnt = 0;
//        char* subStr = strtok(rawStr,",");
//        while(subStr!=NULL){
//            cnt++;
//            if(cnt==3){
//                lLat = atof(subStr);
//            }
//            else if(cnt == 5){
//                lLon = atof(subStr);
//            }
//            subStr = strtok(NULL,",");
//            if(cnt > 8){
//                isAvailable = true;
//            }
//        }
//    }
//    if(isAvailable){
//        cvtCoordinates(lLat, pLat);
//        cvtCoordinates(lLon, pLon);
//        return true;
//    } else{
//        return false;
//    }
//}

bool getCoordinates(char rawStr[], double *pLat, double *pLon) {
	uint8_t cntComma = 2;
	uint8_t index = 17;
	char lat[11] = "", lon[12] = "";
	uint8_t latIndex = 0, lonIndex = 0;
	if (strstr(rawStr, "$GNGGA") != NULL) {
		while (cntComma != 5) {
			if (rawStr[index] == ',') {
				index++;
				cntComma++;
			} else {
				switch (cntComma) {
				case 1:
				case 2:
					lat[latIndex] = rawStr[index];
					index++;
					latIndex++;
					break;
				case 4:
					lon[lonIndex] = rawStr[index];
					index++;
					lonIndex++;
					break;
				default:
					index++;
					break;
				}
			}
		}
		lat[latIndex - 1] = '\0';
		lon[lonIndex - 1] = '\0';
		if (strcmp(lat, "") != 0 && strcmp(lon, "") != 0) {
			cvtCoordinates(atof(lat), pLat);
			cvtCoordinates(atof(lon), pLon);
			return true;
		}
		return false;
	}
	return false;
}

int json_myobj_read(const char *buf, DirectionData *myobj) {
	/* Mapping of JSON attributes to C my_object's struct members */
	const struct json_attr_t json_attrs[] = { { "lat", t_real, .addr.real =
			&(myobj->lat) }, { "lon", t_real, .addr.real = &(myobj->lon) }, {
			"angle", t_real, .addr.real = &(myobj->angle) }, { "dis", t_real,
			.addr.real = &(myobj->dis) }, { NULL }, };
	/* Parse the JSON string from buffer */
	return json_read_object(buf, json_attrs, NULL);
}

void resetArray(char pArr[], uint8_t length) {
	for (int i = 0; i < length; i++) {
		pArr[i] = 0;
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
