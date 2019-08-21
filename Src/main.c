/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <math.h>

#include "sim7020.h"
#include "bh1750.h"
#include "filter_sred.h"
#include "troykaI2CHub.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	open,
	opened,
	close,
	closed,
	still
} operateCmd;

typedef enum {
	top,
	bottom,
	middle
} mechState;

typedef enum {
	cw,
	ccw,
	stp
} motorCmd;

typedef enum {
	mech_wakeup_cmd,
	mech_sleep_cmd,
	mech_still,
	mech_suntrack_cmd,
	mech_suntrack,
	mech_wakeup,
	mech_sleep,
} relictState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VREF 3.20f

#define SUN_ERROR_VERT 10
#define SUN_ERROR_HORZ 10


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_data[2];
uint8_t rx_buffer[rx_buffer_size];
char buffer[rx_buffer_size];

uint8_t transfer;
uint8_t rx_flag = 0;
uint8_t ok_flag;

_Bool endstop00State;
_Bool endstop01State;
_Bool endstop10State;
_Bool endstop11State;
_Bool endstop20State;
_Bool endstop21State;
_Bool endstop30State;
_Bool endstop31State;

_Bool batKey1State;
_Bool batKey2State;

_Bool chrKey1State;
_Bool chrKey2State;

_Bool key3VBusState;

_Bool dvertCorrect;
_Bool dhorzCorrect;
_Bool refreshState;

_Bool mqttReportState;

_Bool rainSensor0Dv;
_Bool rainSensor1Dv;
double rainSensor0Av;
double rainSensor1Av;

operateCmd farm0 = still;
operateCmd farm1 = still;
operateCmd lift = still;

motorCmd motor0Cmd = stp;
motorCmd motor1Cmd = stp;
motorCmd motor5Cmd = stp;
motorCmd motor2Cmd = stp;
motorCmd motor3Cmd = stp;
motorCmd motor4Cmd = stp;

mechState farm0State = middle;
mechState farm1State = middle;
mechState lift0State = middle;

relictState relict = mech_still;


double vBat0;
double vBat0Percent;
double vDif0;
double vBat1;
double vBat1Percent;
double vDif1;

float ls_tl, ls_tr, ls_bl, ls_br;
int16_t dvert, dhorz;
int16_t avt, avb, avl, avr;

uint16_t ADC_value[4];

uint16_t ADC11, ADC12;
uint16_t ADC13, ADC14;

uint16_t buf_11[COUNT_FILTER], buf_12[COUNT_FILTER];
uint16_t buf_13[COUNT_FILTER], buf_14[COUNT_FILTER];

uint16_t filter_11, filter_12;
uint16_t filter_13, filter_14;

FILTER_REG F11, F12;
FILTER_REG F13, F14;

uint8_t c_hour;
uint8_t c_min;
uint8_t c_sec;

uint8_t c_year;
uint8_t c_month;
uint8_t c_day;
uint8_t c_dow;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void buttonsHandler();

void endstop0Control();
void endstop1Control();
void endstop2Control();

void batKey1Handler();
void batKey2Handler();
void key3VBusHandler();


void mechWakeUpControl();
void mechSleepControl();
void refreshStateControl();
void sunTrackControl();

void endstop00handler();
void endstop01handler();
void endstop10handler();
void endstop11handler();
void endstop20handler();
void endstop21handler();
void endstop30handler();
void endstop31handler();

void motor0Handler();
void motor1Handler();
void motor2Handler();
void motor3Handler();
void motor4Handler();
void motor5Handler();

void leftFarmPositionControl();
void rightFarmPositionControl();

void liftPositionControl();

void calculateVBat0();
void calculateVBat1();

void adcRead();

void readLs();
void resetLs();

void readRain();

void debugPrint0();
void debugPrint1();


void getDateTime();
_Bool RTC_Set(uint8_t year, uint8_t month, uint8_t day,uint8_t hour, uint8_t min, uint8_t sec,uint8_t dow);

void lcdPrintLine0();
void lcdPrintLine1();
void lcdPrintLine2();
void lcdPrintLine3();

void relictMqttReportEstop();
void relictMqttReportMechState();
void relictMqttReportHandler();

uint8_t* createBuffer(size_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t* createBuffer(size_t size)
{
	uint8_t* _buffer = malloc (sizeof(uint8_t) * size);
	if (_buffer == NULL) exit (1);
	memset(_buffer, 0, sizeof(uint8_t) * size);
	return _buffer;
}

void getDateTime()
{
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;

	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);

	c_hour = time.Hours;
	c_min = time.Minutes;
	c_sec = time.Seconds;

	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	c_dow = date.WeekDay;
	c_year = date.Year;
	c_month = date.Month;
	c_day =  date.Date;
}

_Bool RTC_Set(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t dow)
{
    HAL_StatusTypeDef res;
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    memset(&time, 0, sizeof(time));
    memset(&date, 0, sizeof(date));

    date.WeekDay = dow;
    date.Year = year;
    date.Month = month;
    date.Date = day;

    res = HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
    if(res != HAL_OK) {
        return 1;
    }

    time.Hours = hour;
    time.Minutes = min;
    time.Seconds = sec;

    res = HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
    if(res != HAL_OK) {
        return 1;
    }
    return 0;
}

void batKey1Handler()
{
	if(batKey1State == 1){
		HAL_GPIO_WritePin(GPIOA, BAT_KEY_1_Pin, GPIO_PIN_SET);
	} else if (batKey1State == 0) {
		HAL_GPIO_WritePin(GPIOA, BAT_KEY_1_Pin, GPIO_PIN_RESET);
	}
}

void batKey2Handler()
{
	if(batKey2State == 1){
		HAL_GPIO_WritePin(GPIOA, BAT_KEY_2_Pin, GPIO_PIN_SET);
	} else if (batKey2State == 0) {
		HAL_GPIO_WritePin(GPIOA, BAT_KEY_2_Pin, GPIO_PIN_RESET);
	}
}

void key3VBusHandler()
{
	if(key3VBusState == 1){
		HAL_GPIO_WritePin(GPIOA, KEY_3V_BUS_Pin, GPIO_PIN_SET);
	} else if (key3VBusState == 0) {
		HAL_GPIO_WritePin(GPIOA, KEY_3V_BUS_Pin, GPIO_PIN_RESET);
	}
}

void adcRead()
{
	HAL_ADC_Stop_DMA(&hadc);

	ADC11 = ADC_value[0];
	ADC12 = ADC_value[1];
	ADC13 = ADC_value[2];
	ADC14 = ADC_value[3];


	filter_11 = filter_sred(ADC11, buf_11, &F11);
	filter_12 = filter_sred(ADC12, buf_12, &F12);
	filter_13 = filter_sred(ADC13, buf_13, &F13);
	filter_14 = filter_sred(ADC14, buf_14, &F14);

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)&ADC_value, 4);
}

void readLs()
{
	troykaI2CHub_setBusChannel(0);
	BH1750_ReadLight(&ls_tl);

	troykaI2CHub_setBusChannel(1);
	BH1750_ReadLight(&ls_tr);

	troykaI2CHub_setBusChannel(3);
	BH1750_ReadLight(&ls_bl);

	troykaI2CHub_setBusChannel(4);
	BH1750_ReadLight(&ls_br);

	avt = (ls_tl + ls_tr)/2;
	avb = (ls_bl + ls_br)/2;
	avl = (ls_tl + ls_bl)/2;
	avr = (ls_tr + ls_br)/2;

	dvert = avt - avb;
	dhorz = avl - avr;
}

void readRain()
{
	if (HAL_GPIO_ReadPin (GPIOC, RAIN_0_D_Pin) == GPIO_PIN_RESET) {
		rainSensor0Dv = 1;
	} else if (HAL_GPIO_ReadPin (GPIOC, RAIN_0_D_Pin) == GPIO_PIN_SET) {
		rainSensor0Dv = 0;
	}

	if (HAL_GPIO_ReadPin (GPIOC, RAIN_1_D_Pin) == GPIO_PIN_RESET) {
		rainSensor1Dv = 1;
	} else if (HAL_GPIO_ReadPin (GPIOC, RAIN_1_D_Pin) == GPIO_PIN_SET) {
		rainSensor1Dv = 0;
	}


	rainSensor0Av = VREF / 4095.0 * filter_13;
	rainSensor1Av = VREF / 4095.0 * filter_14;

}

void resetLs()
{
	troykaI2CHub_setBusChannel(0);
	BH1750_Reset();
	troykaI2CHub_setBusChannel(1);
	BH1750_Reset();
	troykaI2CHub_setBusChannel(3);
	BH1750_Reset();
	troykaI2CHub_setBusChannel(4);
	BH1750_Reset();
}

void calculateVBat0()
{
	double r_divider, r1, r2;
	r1 = 1880.0;
	r2 = 2070.0;
	r_divider = r2 / (r1 + r2);

	vBat0 = VREF / 4095.0 * filter_11;
	vBat0 = vBat0 / r_divider;

	if (vBat0 > 3.7f){
		vDif0 = vBat0 - 3.7f;
		vBat0Percent = 100 * vDif0 / 0.5;
	}
}

void calculateVBat1()
{
	double r_divider, r1, r2;
	r1 = 1890.0;
	r2 = 2090.0;
	r_divider = r2 / (r1 + r2);

	vBat1 = VREF / 4095.0 * filter_12;
	vBat1 = vBat1 / r_divider;

	if (vBat1 > 3.7f){
		vDif1 = vBat1 - 3.7f;
		vBat1Percent = 100 * vDif1 / 0.5;
	}
}

void I2C_Scan()
{
    char info[] = "SCANNING I2C BUS\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    HAL_StatusTypeDef res;
    for(uint16_t i = 0; i < 128; i++) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
        if(res == HAL_OK) {
            char msg[64];
            snprintf(msg, sizeof(msg), "0x%02X ", i);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

void leftFarmPositionControl()
{
	if (farm0 == open && farm0State == middle) {
		motor0Cmd = cw;
	}
	if (farm0 == open && farm0State == bottom) {
		motor0Cmd = cw;
	}
	if (farm0 == open && farm0State == top) {
		motor0Cmd = stp;
		farm0 = opened;
	}
	if (farm0 == close && farm0State == middle) {
		motor0Cmd = ccw;
	}
	if (farm0 == close && farm0State == top) {
		motor0Cmd = ccw;
	}
	if (farm0 == close && farm0State == bottom) {
		motor0Cmd = stp;
		farm0 = closed;
	}
}

void rightFarmPositionControl()
{
	if (farm1 == open && farm1State == middle) {
		motor1Cmd = cw;
	}
	if (farm1 == open && farm1State == bottom) {
		motor1Cmd = cw;
	}
	if (farm1 == open && farm1State == top) {
		motor1Cmd = stp;
		farm1 = opened;
	}
	if (farm1 == close && farm1State == middle) {
		motor1Cmd = ccw;
	}
	if (farm1 == close && farm1State == top) {
		motor1Cmd = ccw;
	}
	if (farm1 == close && farm1State == bottom) {
		motor1Cmd = stp;
		farm1 = closed;
	}
}

void liftPositionControl()
{
	if (lift == open && lift0State == middle) {
		motor5Cmd = ccw;
	}
	if (lift == open && lift0State == bottom) {
		motor5Cmd = ccw;
	}
	if (lift == open && lift0State == top) {
		motor5Cmd = stp;
		lift = opened;
	}
	if (lift == close && lift0State == middle) {
		motor5Cmd = cw;
	}
	if (lift == close && lift0State == top) {
		motor5Cmd = cw;
	}
	if (lift == close && lift0State == bottom) {
		motor5Cmd = stp;
		lift = closed;
	}
}

void refreshStateControl()
{
	if (endstop30State == 1 && endstop31State == 1){
		refreshState = 1;
	} else {
		refreshState = 0;
	}
}

//void sunTrackControl()
//{
//	if(relict == mech_suntrack_cmd){
//		if (refreshState == 0){
//			if (endstop30State == 0){
//				motor3Cmd = ccw;
//				motor4Cmd = ccw;
//			}
//			if (endstop30State == 1){
//				motor3Cmd = stp;
//				motor4Cmd = stp;
//				if(endstop31State == 0){
//					motor2Cmd = cw;
//				}
//				if(endstop31State == 1){
//					motor2Cmd = stp;
//					refreshState = 1;
//				}
//			}
//		}
//
//		if (refreshState == 1){
//			if(dhorz > -SUN_ERROR_HORZ && dhorz < SUN_ERROR_HORZ) {
//				motor2Cmd = stp;
//				dhorzCorrect = 1;
//			}
//			if(dhorz < -SUN_ERROR_HORZ || dhorz > SUN_ERROR_HORZ){
//				dhorzCorrect = 0;
//				motor2Cmd = ccw;
//			}
//			if(dvert > -SUN_ERROR_VERT && dvert < SUN_ERROR_VERT) {
//					motor3Cmd = stp;
//					motor4Cmd = stp;
//					dvertCorrect = 1;
//			}
//			if(dvert < -SUN_ERROR_VERT || dvert > SUN_ERROR_VERT){
//					dvertCorrect = 0;
//					motor3Cmd = cw;
//					motor4Cmd = cw;
//			}
//			if (dvertCorrect == 1 && dhorzCorrect == 1){
//				relict = mech_suntrack;
//			}
//		}
//
//	}
//}

void sunTrackControl()
{
	if(relict == mech_suntrack_cmd){

			if(dhorz > -SUN_ERROR_HORZ && dhorz < SUN_ERROR_HORZ) {
				motor2Cmd = stp;
				dhorzCorrect = 1;
			}
			if(dhorz < -SUN_ERROR_HORZ || dhorz > SUN_ERROR_HORZ){
				dhorzCorrect = 0;
				if (dhorz > 0) {
				motor2Cmd = cw;
				}
				if (dhorz < 0) {
				motor2Cmd = ccw;
				}
				if (endstop31State == 1){
					motor2Cmd = stp;
				}
			}

			if(dvert > -SUN_ERROR_VERT && dvert < SUN_ERROR_VERT) {
					motor3Cmd = stp;
					motor4Cmd = stp;
					dvertCorrect = 1;
			}
			if(dvert < -SUN_ERROR_VERT || dvert > SUN_ERROR_VERT){
					dvertCorrect = 0;
					if(dvert > 0){
						motor3Cmd = cw;
						motor4Cmd = cw;
					}
					if(dvert < 0){
						motor3Cmd = ccw;
						motor4Cmd = ccw;
					}
					if(endstop30State == 1) {
						motor3Cmd = stp;
						motor4Cmd = stp;
					}
			}

		if (dvertCorrect == 1 && dhorzCorrect == 1){
			relict = mech_suntrack;
		}
	}
}

void mechWakeUpControl()
{
	if (relict == mech_wakeup_cmd){
		if(lift != opened){
			lift = open;
		}
		if(lift == opened){
			if(farm0 != opened){
				farm0 = open;
			}
			if(farm1 != opened){
				farm1 = open;
			}
			if(farm0 == opened && farm1 == opened){
				relict = mech_suntrack_cmd;
			}
		}
	}
}

void mechSleepControl()
{
	if (relict == mech_sleep_cmd){
		if(farm0 != closed ){
			farm0 = close;
		}
		if(farm1 != closed ){
			farm1 = close;
		}
		if(farm0 == closed && farm1 == closed){
			if (endstop30State == 0){
				motor3Cmd = ccw;
				motor4Cmd = ccw;
			}
			if (endstop30State == 1){
				motor3Cmd = stp;
				motor4Cmd = stp;

				if(endstop31State == 0){
					motor2Cmd = cw;
				}
				if(endstop31State == 1){
					motor2Cmd = stp;
					lift = close;
				}
				if(lift == closed){
					relict = mech_sleep;
				}
			}
		}
	}
}

void motor0Handler()
{
	if (motor0Cmd == stp) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_EN_Pin, GPIO_PIN_RESET);
	}

	if (motor0Cmd == cw) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_D_Pin, GPIO_PIN_RESET);
	}

	if (motor0Cmd == ccw) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_D_Pin, GPIO_PIN_SET);
	}
}

void motor1Handler()
{
	if (motor1Cmd == stp) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_1_EN_Pin, GPIO_PIN_RESET);
	}

	if (motor1Cmd == cw) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_1_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_1_D_Pin, GPIO_PIN_RESET);
	}

	if (motor1Cmd == ccw) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_1_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_1_D_Pin, GPIO_PIN_SET);
	}
}

void motor2Handler()
{
	if (motor2Cmd == stp) {
		HAL_GPIO_WritePin(GPIOC, MOTOR_2_EN_Pin, GPIO_PIN_RESET);
	}

	if (motor2Cmd == cw) {
		HAL_GPIO_WritePin(GPIOC, MOTOR_2_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, MOTOR_2_D_Pin, GPIO_PIN_RESET);
	}

	if (motor2Cmd == ccw) {
		HAL_GPIO_WritePin(GPIOC, MOTOR_2_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, MOTOR_2_D_Pin, GPIO_PIN_SET);
	}
}

void motor3Handler()
{
	if (motor3Cmd == stp) {
		HAL_GPIO_WritePin(GPIOC, MOTOR_3_EN_Pin, GPIO_PIN_RESET);
	}

	if (motor3Cmd == cw) {
		HAL_GPIO_WritePin(GPIOC, MOTOR_3_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_3_D_Pin, GPIO_PIN_RESET);
	}

	if (motor3Cmd == ccw) {
		HAL_GPIO_WritePin(GPIOC, MOTOR_3_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_3_D_Pin, GPIO_PIN_SET);
	}
}

void motor4Handler()
{
	if (motor4Cmd == stp) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_4_EN_Pin, GPIO_PIN_RESET);
	}

	if (motor4Cmd == cw) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_4_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_4_D_Pin, GPIO_PIN_RESET);
	}

	if (motor4Cmd == ccw) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_4_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_4_D_Pin, GPIO_PIN_SET);
	}
}

void motor5Handler()
{
	if (motor5Cmd == stp) {
		HAL_GPIO_WritePin(GPIOD, MOTOR_5_EN_Pin, GPIO_PIN_RESET);
	}

	if (motor5Cmd == cw) {
		HAL_GPIO_WritePin(GPIOD, MOTOR_5_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MOTOR_5_D_Pin, GPIO_PIN_RESET);
	}

	if (motor5Cmd == ccw) {
		HAL_GPIO_WritePin(GPIOD, MOTOR_5_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MOTOR_5_D_Pin, GPIO_PIN_SET);
	}
}


void endstop0Control()
{
	if (endstop00State == 0 && endstop01State == 0) {
		farm0State = middle;
	}
	if (endstop00State == 1 && endstop01State == 0) {
		farm0State = top;
	}
	if (endstop00State == 0 && endstop01State == 1) {
		farm0State = bottom;
	}
}

void endstop1Control()
{
	if (endstop10State == 0 && endstop11State == 0) {
		farm1State = middle;
	}
	if (endstop10State == 1 && endstop11State == 0) {
		farm1State = top;
	}
	if (endstop10State == 0 && endstop11State == 1) {
		farm1State = bottom;
	}
}

void endstop2Control()
{
	if (endstop20State == 0 && endstop21State == 0) {
		lift0State = middle;
	}
	if (endstop20State == 1 && endstop21State == 0) {
		lift0State = top;
	}
	if (endstop20State == 0 && endstop21State == 1) {
		lift0State = bottom;
	}
}

void endstop00Handler()
{
	if (HAL_GPIO_ReadPin (GPIOA, ESTOP_0_0_Pin) == GPIO_PIN_RESET) {
		endstop00State = 0;
	} else if (HAL_GPIO_ReadPin (GPIOA, ESTOP_0_0_Pin) == GPIO_PIN_SET) {
		endstop00State = 1;
	}
}

void endstop01Handler()
{
	if (HAL_GPIO_ReadPin (GPIOA, ESTOP_0_1_Pin) == GPIO_PIN_RESET) {
		endstop01State = 0;
	} else if (HAL_GPIO_ReadPin (GPIOA, ESTOP_0_1_Pin) == GPIO_PIN_SET) {
		endstop01State = 1;
	}
}

void endstop10Handler()
{
	if (HAL_GPIO_ReadPin (GPIOA, ESTOP_1_0_Pin) == GPIO_PIN_RESET) {
		endstop10State = 0;
	} else if (HAL_GPIO_ReadPin (GPIOA, ESTOP_1_0_Pin) == GPIO_PIN_SET) {
		endstop10State = 1;
	}
}

void endstop11Handler()
{
	if (HAL_GPIO_ReadPin (GPIOA, ESTOP_1_1_Pin) == GPIO_PIN_RESET) {
		endstop11State = 0;
	} else if (HAL_GPIO_ReadPin (GPIOA, ESTOP_1_1_Pin) == GPIO_PIN_SET) {
		endstop11State = 1;
	}
}

void endstop20Handler()
{
	if (HAL_GPIO_ReadPin (GPIOC, ESTOP_2_1_Pin) == GPIO_PIN_RESET) {
		endstop20State = 0;
	} else if (HAL_GPIO_ReadPin (GPIOC, ESTOP_2_1_Pin) == GPIO_PIN_SET) {
		endstop20State = 1;
	}
}

void endstop21Handler()
{
	if (HAL_GPIO_ReadPin (GPIOC, ESTOP_2_0_Pin) == GPIO_PIN_RESET) {
		endstop21State = 0;
	} else if (HAL_GPIO_ReadPin (GPIOC, ESTOP_2_0_Pin) == GPIO_PIN_SET) {
		endstop21State = 1;
	}
}

void endstop30Handler()
{
	if (HAL_GPIO_ReadPin (GPIOA, ESTOP_3_0_Pin) == GPIO_PIN_RESET) {
		endstop30State = 0;
	} else if (HAL_GPIO_ReadPin (GPIOA, ESTOP_3_0_Pin) == GPIO_PIN_SET) {
		endstop30State = 1;
	}
}

void endstop31Handler()
{
	if (HAL_GPIO_ReadPin (GPIOA, ESTOP_3_1_Pin) == GPIO_PIN_RESET) {
		endstop31State = 0;
	} else if (HAL_GPIO_ReadPin (GPIOA, ESTOP_3_1_Pin) == GPIO_PIN_SET) {
		endstop31State = 1;
	}
}

void debugPrint0()
{
    char dateMsg[64];
    snprintf(dateMsg, sizeof(dateMsg), "%d.%d.%d-%d", c_day, c_month, c_year, c_dow);
    HAL_UART_Transmit(&huart2, (uint8_t*)dateMsg, strlen(dateMsg),HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)"-", 1, HAL_MAX_DELAY);

	char timeMsg[64];
	snprintf(timeMsg, sizeof(timeMsg), "%d:%d:%d", c_hour, c_min, c_sec);
	HAL_UART_Transmit(&huart2, (uint8_t*)timeMsg, strlen(timeMsg),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	char val0[7];
	int tmpInt01 = vBat0;
	float tmpFrac0 = vBat0 - tmpInt01;
	int tmpInt02 = trunc(tmpFrac0 * 100);
	sprintf (val0,"%01d.%02d", tmpInt01, tmpInt02);

	HAL_UART_Transmit(&huart2, (uint8_t*)val0, strlen(val0), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	char val1[7];
	int tmpInt11 = vBat0Percent;
	float tmpFrac1 = vBat0Percent - tmpInt11;
	int tmpInt12 = trunc(tmpFrac1 * 100);
	sprintf (val1,"%02d.%02d", tmpInt11, tmpInt12);

	HAL_UART_Transmit(&huart2, (uint8_t*)val1, strlen(val1), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	char val2[7];
	int tmpInt21 = vBat1;
	float tmpFrac2 = vBat1 - tmpInt21;
	int tmpInt22 = trunc(tmpFrac2 * 100);
	sprintf (val2,"%01d.%02d", tmpInt21, tmpInt22);

	HAL_UART_Transmit(&huart2, (uint8_t*)val2, strlen(val2), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	char val3[7];
	int tmpInt31 = vBat1Percent;
	float tmpFrac3 = vBat1Percent - tmpInt31;
	int tmpInt32 = trunc(tmpFrac3 * 100);
	sprintf (val3,"%02d.%02d", tmpInt31, tmpInt32);

	HAL_UART_Transmit(&huart2, (uint8_t*)val3, strlen(val3), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 2, HAL_MAX_DELAY);

	char val4[7];
	sprintf (val4, "%d", (int16_t)ls_tl);

	char val5[7];
	sprintf (val5, "%d", (int16_t)ls_tr);

	char val6[7];
	sprintf (val6, "%d", (int16_t)ls_bl);

	char val7[7];
	sprintf (val7, "%d", (int16_t)ls_br);

    HAL_UART_Transmit(&huart2, (uint8_t*)val4, strlen(val4), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, (uint8_t*)val5, strlen(val5), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, (uint8_t*)val6, strlen(val6), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, (uint8_t*)val7, strlen(val7), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 3, HAL_MAX_DELAY);

	char val8[7];
	sprintf (val8, "%d", dvert);
	char val9[7];
	sprintf (val9, "%d", dhorz);

	HAL_UART_Transmit(&huart2, (uint8_t*)val8, strlen(val8), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, (uint8_t*)val9, strlen(val9), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	char val10[7];
	sprintf (val10, "%d", rainSensor0Dv);
	char val11[7];
	int tmpInt111 = rainSensor0Av;
	float tmpFrac11 = rainSensor0Av - tmpInt111;
	int tmpInt112 = trunc(tmpFrac11 * 100);
	sprintf (val11,"%01d.%02d", tmpInt111, tmpInt112);

	char val12[7];
	sprintf (val12, "%d", rainSensor1Dv);


	char val13[7];
	int tmpInt131 = rainSensor1Av;
	float tmpFrac13 = rainSensor1Av - tmpInt131;
	int tmpInt132 = trunc(tmpFrac13 * 100);
	sprintf (val13,"%01d.%02d", tmpInt131, tmpInt132);

	HAL_UART_Transmit(&huart2, (uint8_t*)val10, strlen(val10), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, (uint8_t*)val11, strlen(val11), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, (uint8_t*)val12, strlen(val12), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, (uint8_t*)val13, strlen(val13), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"	", 1, HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

void debugPrint1()
{
    char estop[64];
    snprintf(estop, sizeof(estop), "left_farm: %d %d | right_farm: %d %d | lift: %d %d | rotate: %d %d",
    		(uint8_t)endstop00State,
			(uint8_t)endstop01State,
			(uint8_t)endstop10State,
			(uint8_t)endstop11State,
			(uint8_t)endstop20State,
			(uint8_t)endstop21State,
			(uint8_t)endstop30State,
			(uint8_t)endstop31State);
    HAL_UART_Transmit(&huart2, (uint8_t*)estop, strlen(estop), HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint8_t*)"	MECH STATE: ", 13, HAL_MAX_DELAY);

    char line0[100];
    char* val0;
	char* val1;
	char* val01;


	if (farm0State == middle) {
		val0 = "middle";
	}
	if (farm0State == top) {
		val0 = "top";
	}
	if (farm0State == bottom) {
		val0 = "bottom";
	}
	if (farm1State == middle) {
		val1 = "middle";
	}
	if (farm1State == top) {
		val1 = "top";
	}
	if (farm1State == bottom) {
		val1 = "bottom";
	}
	if (lift0State == middle) {
		val01 = "middle";
	}
	if (lift0State == top) {
		val01 = "top";
	}
	if (lift0State == bottom) {
		val01 = "bottom";
	}

	char* par0 = "l_f_s: ";
	char* par1 = "r_f_s: ";
	char* par01 = "lft_s: ";

	char* space0 = " ";

	strcpy(line0,par0);
	strcat(line0,val0);
	strcat(line0,space0);

	strcat(line0,par1);
	strcat(line0,val1);
	strcat(line0,space0);

	strcat(line0,par01);
	strcat(line0,val01);
	strcat(line0,space0);

    HAL_UART_Transmit(&huart2, (uint8_t*)line0, strlen(line0),HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)"	OPERATE STATE: ", 16, HAL_MAX_DELAY);

    char line1[100];
    char* val21;
    char* val22;
	char* val3;
	char* val4;

	if (farm0 == open) {
		val21 = "open";
	}
	if (farm0 == close) {
		val21 = "close";
	}
	if (farm0 == still) {
		val21 = "still";
	}
	if (farm0 == opened) {
		val21 = "opened";
	}
	if (farm0 == closed) {
		val21 = "closed";
	}

	if (farm1 == open) {
		val22 = "open";
	}
	if (farm1 == close) {
		val22 = "close";
	}
	if (farm1 == still) {
		val22 = "still";
	}
	if (farm1 == opened) {
		val22 = "opened";
	}
	if (farm1 == closed) {
		val22 = "closed";
	}


	if (lift == open) {
		val3 = "open";
	}
	if (lift == close) {
		val3 = "close";
	}
	if (lift == still) {
		val3 = "still";
	}
	if (lift == opened) {
		val3 = "opened";
	}
	if (lift == closed) {
		val3 = "closed";
	}

	if (relict == mech_wakeup) {
		val4 = "mech_wakeup";
	}
	if (relict == mech_sleep) {
		val4 = "mech_sleep";
	}
	if (relict == mech_wakeup_cmd) {
		val4 = "mech_wakeup_cmd";
	}
	if (relict == mech_sleep_cmd) {
		val4 = "mech_sleep_cmd";
	}
	if (relict == mech_suntrack) {
		val4 = "mech_suntrack";
	}
	if (relict == mech_suntrack_cmd) {
		val4 = " mech_suntrack_cmd";
	}
	if (relict == mech_still) {
		val4 = "mech_still";
	}

	char* par4 = "| relict: ";
	char* par21 = "| f0: ";
	char* par22 = "| f1: ";
	char* par3 = "|  l_s:  ";

	strcpy(line1,par21);
	strcat(line1,val21);
	strcat(line1,space0);
	strcat(line1,par22);
	strcat(line1,val22);
	strcat(line1,space0);
	strcat(line1,par3);
	strcat(line1,val3);
	strcat(line1,space0);
	strcat(line1,par4);
	strcat(line1,val4);
	strcat(line1,space0);

    HAL_UART_Transmit(&huart2, (uint8_t*)line1, strlen(line1),HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}



void buttonsHandler()
{
	if (HAL_GPIO_ReadPin (GPIOB, BUTTON_0_Pin) == GPIO_PIN_RESET) {
		mqttReportState = 1;
	}
	if (HAL_GPIO_ReadPin (GPIOB, BUTTON_1_Pin) == GPIO_PIN_RESET) {
		farm0 = close;
		farm1 = close;
	}
	if (HAL_GPIO_ReadPin (GPIOB, BUTTON_2_Pin) == GPIO_PIN_RESET) {
		relict = mech_wakeup_cmd;
	}
	if (HAL_GPIO_ReadPin (GPIOB, BUTTON_3_Pin) == GPIO_PIN_RESET) {
		relict = mech_sleep_cmd;
	}
}

void relictMqttReportEstop()
{

	HAL_UART_Transmit(&huart2, (uint8_t*)"REPORT ESTP STATE", 17, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	char NBcmqpub_at_msg[8][100];
	char NBcmqpub_val[8][3];

	snprintf(NBcmqpub_val[0], 3, "%d",(uint8_t)endstop10State + 30);
	snprintf(NBcmqpub_val[1], 3, "%d",(uint8_t)endstop11State + 30);
	snprintf(NBcmqpub_val[2], 3, "%d",(uint8_t)endstop00State + 30);
	snprintf(NBcmqpub_val[3], 3, "%d",(uint8_t)endstop01State + 30);
	snprintf(NBcmqpub_val[4], 3, "%d",(uint8_t)endstop20State + 30);
	snprintf(NBcmqpub_val[5], 3, "%d",(uint8_t)endstop21State + 30);
	snprintf(NBcmqpub_val[6], 3, "%d",(uint8_t)endstop30State + 30);
	snprintf(NBcmqpub_val[7], 3, "%d",(uint8_t)endstop31State + 30);

	char* atStringWrapper = "\"";
	char* NBcmqpub_cmd = "AT+CMQPUB=0,";
	char* NBcmqpub_par_0 = ",1,0,0,2,";

	const char* NBcmqpub_feed[8] = {"Gabbapeople/feeds/relict-estop-state.right-farm-0",
	   					  	  	  	"Gabbapeople/feeds/relict-estop-state.right-farm-1",
									"Gabbapeople/feeds/relict-estop-state.left-farm-0",
									"Gabbapeople/feeds/relict-estop-state.left-farm-1",
									"Gabbapeople/feeds/relict-estop-state.lift-0",
									"Gabbapeople/feeds/relict-estop-state.lift-1",
									"Gabbapeople/feeds/relict-estop-state.rotate-0",
									"Gabbapeople/feeds/relict-estop-state.rotate-1"
	    							};

	for(uint8_t i = 0; i < 8; i++){
	    strcpy(NBcmqpub_at_msg[i], NBcmqpub_cmd);
		strcat(NBcmqpub_at_msg[i], atStringWrapper);
		strcat(NBcmqpub_at_msg[i], NBcmqpub_feed[i]);
		strcat(NBcmqpub_at_msg[i], atStringWrapper);
		strcat(NBcmqpub_at_msg[i], NBcmqpub_par_0);
		strcat(NBcmqpub_at_msg[i], atStringWrapper);
		strcat(NBcmqpub_at_msg[i], NBcmqpub_val[i]);
		strcat(NBcmqpub_at_msg[i], atStringWrapper);

	    HAL_UART_Transmit(&huart2, (uint8_t*)NBcmqpub_at_msg[i], strlen(NBcmqpub_at_msg[i]),HAL_MAX_DELAY);
	    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

		const char* NBcmqpub[] = {
				NBcmqpub_at_msg[i],
				"\0"
		};

		sim7020_NBcmqpub(NBcmqpub);

	}

}

void relictMqttReportMechState()
{

	HAL_UART_Transmit(&huart2, (uint8_t*)"REPORT MECH STATE", 17, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	char NBcmqpub_at_msg[3][100];
	char NBcmqpub_val[3][13];

	char* atStringWrapper = "\"";
	char* NBcmqpub_cmd = "AT+CMQPUB=0,";
	char* NBcmqpub_par_0 = ",1,0,0,12,";

//	char* str_open = "6f70656e2020";
//	char* str_close = "636c6f736520";
//	char* str_still = "7374696c6c20";
//	char* str_opened = "6f70656e6564";
//	char* str_closed = "636c6f736564";


	const char* NBcmqpub_feed[3] = {"Gabbapeople/feeds/relict-mech-state.right-farm-state",
	   					  	  	  	"Gabbapeople/feeds/relict-mech-state.left-farm-state",
									"Gabbapeople/feeds/relict-mech-state.lift-state"
	    							};

	HAL_UART_Transmit(&huart2, (uint8_t*)"!!!!!!!!!!!1", 12, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	if (farm0 == open) {
		strcpy(NBcmqpub_val[0],"6f70656e2020");
	}
	if (farm0 == close) {
		strcpy(NBcmqpub_val[0], "636c6f736520");
	}
	if (farm0 == still) {
		strcpy(NBcmqpub_val[0],"7374696c6c20");
	}
	if (farm0 == opened) {
		strcpy(NBcmqpub_val[0],"6f70656e6564");
	}
	if (farm0 == closed) {
		strcpy(NBcmqpub_val[0],"636c6f736564");
	}

	if (farm1 == open) {
		strcpy(NBcmqpub_val[1],"6f70656e2020");
	}
	if (farm1 == close) {
		strcpy(NBcmqpub_val[1],"636c6f736520");
	}
	if (farm1 == still) {
		strcpy(NBcmqpub_val[1],"7374696c6c20");
	}
	if (farm1 == opened) {
		strcpy(NBcmqpub_val[1],"6f70656e6564");
	}
	if (farm1 == closed) {
		strcpy(NBcmqpub_val[1],"636c6f736564");
	}

	if (lift == open) {
		strcpy(NBcmqpub_val[2],"6f70656e2020");
	}
	if (lift == close) {
		strcpy(NBcmqpub_val[2],"636c6f736520");
	}
	if (lift == still) {
		strcpy(NBcmqpub_val[2],"7374696c6c20");
	}
	if (lift == opened) {
		strcpy(NBcmqpub_val[2],"6f70656e6564");
	}
	if (lift == closed) {
		strcpy(NBcmqpub_val[2],"636c6f736564");
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)"!!!!!!!!!!!2", 12, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	for(uint8_t i = 0; i < 3; i++){

		HAL_UART_Transmit(&huart2, (uint8_t*)"!!!!!!!!!!!3", 12, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

		strcpy(NBcmqpub_at_msg[i], NBcmqpub_cmd);
		strcat(NBcmqpub_at_msg[i], atStringWrapper);
		strcat(NBcmqpub_at_msg[i], NBcmqpub_feed[i]);
		strcat(NBcmqpub_at_msg[i], atStringWrapper);
		strcat(NBcmqpub_at_msg[i], NBcmqpub_par_0);
		strcat(NBcmqpub_at_msg[i], atStringWrapper);
		strcat(NBcmqpub_at_msg[i], NBcmqpub_val[i]);
		strcat(NBcmqpub_at_msg[i], atStringWrapper);

	    HAL_UART_Transmit(&huart2, (uint8_t*)NBcmqpub_at_msg[i], strlen(NBcmqpub_at_msg[i]),HAL_MAX_DELAY);
	    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

		const char* NBcmqpub[] = {
				NBcmqpub_at_msg[i],
				"\0"
		};

		sim7020_NBcmqpub(NBcmqpub);

	}

}


void relictMqttReportHandler()
{
	if (mqttReportState == 1){

		//sim7020_NBmaxfun();
		generateRandomClientID();
		sim7020_NBcmqnewUntillConnect(100);
		sim7020_NBcmqcon();
		//relictMqttReportEstop();
		relictMqttReportMechState();
		sim7020_NBcmqdiscon();
		mqttReportState = 0;
		//sim7020_NBminfun();

	}

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  F11.Val = 0;
  F12.Val = 0;
  F13.Val = 0;
  F14.Val = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  batKey1State = 1;
  batKey2State = 1;
  key3VBusState = 1;

  mqttReportState = 0;


  batKey1Handler();
  batKey2Handler();
  key3VBusHandler();

  HAL_Delay(2000);

  RTC_Set(19, 6, 26, 18, 7, 0, 3);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  I2C_Scan();


  troykaI2CHub_init(&hi2c1);
  troykaI2CHub_setBusChannel(0);

  BH1750_Init(&hi2c1);
  BH1750_PowerState(1);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);

  troykaI2CHub_setBusChannel(1);
  BH1750_PowerState(1);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);

  troykaI2CHub_setBusChannel(3);
  BH1750_PowerState(1);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);

  troykaI2CHub_setBusChannel(4);
  BH1750_PowerState(1);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);

  HAL_Delay(2000);
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)&ADC_value, 2);


  sim7020_init(&huart3, &huart2);
  sim7020_dtr(0);
  sim7020_powerCycle();


  sim7020_hardwareInfo();
  sim7020_NBcsq();
  //sim7020_NBmaxfun();
  sim7020_NBcpin();

  HAL_Delay(5000);

  sim7020_NBcopsUntillConnect(255);

  //sim7020_NBminfun();



  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  relictMqttReportHandler();

	  getDateTime();

	  batKey1Handler();
	  batKey2Handler();
	  key3VBusHandler();

	  adcRead();
	  calculateVBat0();
	  calculateVBat1();
	  readLs();
	  readRain();

	  buttonsHandler();

	  endstop00Handler();
	  endstop01Handler();
	  endstop10Handler();
	  endstop11Handler();
	  endstop20Handler();
	  endstop21Handler();
	  endstop30Handler();
	  endstop31Handler();

	  endstop0Control();
	  endstop1Control();
	  endstop2Control();



	  leftFarmPositionControl();
	  rightFarmPositionControl();
	  liftPositionControl();

	  mechWakeUpControl();
	  mechSleepControl();

	  refreshStateControl();
	  sunTrackControl();

	  motor0Handler();
	  motor1Handler();
	  motor2Handler();
	  motor3Handler();
	  motor4Handler();
	  motor5Handler();

	  //debugPrint0();
	  //debugPrint1();

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
////	const char info[] = "TX OVER \r\n";
////	HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
//}
//
//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
//{
////	const char info[] = "RX HALF BUFF \r\n";
////	HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	    uint8_t i;
	    if (huart->Instance == USART3) {

	        if (rx_flag == 0) {
	            for (i = 0; i < 100; i++)
	                rx_buffer[i] = 0;
	        }

	        if (rx_flag > rx_buffer_size){
	            rx_flag = 0;
	        }

	        rx_buffer[rx_flag++] = rx_data[0];

	        if (rx_buffer[rx_flag - 4] == 79 && rx_buffer[rx_flag - 3] == 75){
	        	ok_flag = 1;
	            rx_flag = 0;
	            transfer = 1;
	        }

	        if (transfer) {
	            sprintf(buffer, "%s", rx_buffer);
	            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sizeof(buffer), 1000);

	            transfer = 0;
	        }
	        HAL_UART_Receive_IT(&huart3, rx_data, 1);
	    }
}

////  if (huart->Instance == USART3)
////  {
//	buffer[buffer_counter++] = byte;
//    HAL_UART_Receive_IT(&huart3, &byte, 1);
// // }


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
