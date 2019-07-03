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
	close,
	still
} farmOperateCmd;

typedef enum {
	gotop,
	gobottom,
	stop
} motorCmd;

typedef enum {
	cw,
	ccw,
	stp
} motorCmd2;

typedef enum {
	top,
	bottom,
	middle
} farmState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VREF 3.29f

#define SUN_ERROR_VERT 80
#define SUN_ERROR_HORZ 40


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

_Bool batKey1State;
_Bool batKey2State;

_Bool chrKey1State;
_Bool chrKey2State;

_Bool key3VBusState;

_Bool sunTrackState;
_Bool dvertCorrect;
_Bool dhorzCorrect;

farmOperateCmd farms = still;
motorCmd motor0Cmd = stop;
motorCmd motor1Cmd = stop;
farmState farm0State = middle;
farmState farm1State = middle;

motorCmd2 motor2Cmd = stp;
motorCmd2 motor3Cmd = stp;
motorCmd2 motor4Cmd = stp;

double vBat0;
double vBat0Percent;
double vDif0;
double vBat1;
double vBat1Percent;
double vDif1;

float ls_tl, ls_tr, ls_bl, ls_br;
int16_t dvert, dhorz;
int16_t avt, avb, avl, avr;

uint16_t ADC_value[2];
uint16_t ADC11, ADC12;
uint16_t buf_11[COUNT_FILTER], buf_12[COUNT_FILTER];
uint16_t filter_11, filter_12;

FILTER_REG F11, F12;

uint8_t c_hour;
uint8_t c_min;
uint8_t c_sec;

uint8_t c_year;
uint8_t c_month;
uint8_t c_day;
uint8_t c_dow;


//unsigned char byte;
//uint8_t buffer_counter;
//unsigned char *buffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void buttonsHandler();

void endstop0Control();
void endstop1Control();

void batKey1Handler();
void batKey2Handler();

void chrKey1Handler();
void chrKey2Handler();

void key3VBusHandler();

void sunTrackControl();

void endstop00handler();
void endstop01handler();
void endstop10handler();
void endstop11handler();

void motor0Handler();
void motor1Handler();

void motor2Handler();
void motor3Handler();
void motor4Handler();

void motor0PositionControl();
void motor1PositionControl();

void calculateVBat0();
void adcRead();
void readLs();
void resetLs();

void statusPrint();

void getDateTime();
_Bool RTC_Set(uint8_t year, uint8_t month, uint8_t day,uint8_t hour, uint8_t min, uint8_t sec,uint8_t dow);

void lcdPrintLine0();
void lcdPrintLine1();
void lcdPrintLine2();
void lcdPrintLine3();

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

void chrKey1Handler()
{
	if(chrKey1State == 1){
		HAL_GPIO_WritePin(GPIOA, CHR_KEY_1_Pin, GPIO_PIN_SET);
	} else if (chrKey1State == 0) {
		HAL_GPIO_WritePin(GPIOA, CHR_KEY_1_Pin, GPIO_PIN_RESET);
	}
}

void chrKey2Handler()
{
	if(chrKey2State == 1){
		HAL_GPIO_WritePin(GPIOA, CHR_KEY_2_Pin, GPIO_PIN_SET);
	} else if (chrKey2State == 0) {
		HAL_GPIO_WritePin(GPIOA, CHR_KEY_2_Pin, GPIO_PIN_RESET);
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

	filter_11 = filter_sred(ADC11, buf_11, &F11);
	filter_12 = filter_sred(ADC12, buf_12, &F12);

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)&ADC_value, 2);
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

void sunTrackControl()
{
	if(sunTrackState == 1){
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
		}
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
		}
		if (dvertCorrect == 1 && dhorzCorrect == 1){
			sunTrackState = 0;
		}
	}
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

void motor0PositionControl()
{
	if (farms == open && farm0State == middle) {
		motor0Cmd = gotop;
	}
	if (farms == open && farm0State == bottom) {
		motor0Cmd = gotop;
	}
	if (farms == open && farm0State == top) {
		motor0Cmd = stop;
	}
	if (farms == close && farm0State == middle) {
		motor0Cmd = gobottom;
	}
	if (farms == close && farm0State == top) {
		motor0Cmd = gobottom;
	}
	if (farms == close && farm0State == bottom) {
		motor0Cmd = stop;
	}
}

void motor1PositionControl()
{
	if (farms == open && farm1State == middle) {
		motor1Cmd = gotop;
	}
	if (farms == open && farm1State == bottom) {
		motor1Cmd = gotop;
	}
	if (farms == open && farm1State == top) {
		motor1Cmd = stop;
	}
	if (farms == close && farm1State == middle) {
		motor1Cmd = gobottom;
	}
	if (farms == close && farm1State == top) {
		motor1Cmd = gobottom;
	}
	if (farms == close && farm1State == bottom) {
		motor1Cmd = stop;
	}
}

void motor0Handler()
{
	if (motor0Cmd == stop) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_EN_Pin, GPIO_PIN_RESET);
	}

	if (motor0Cmd == gotop) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_D_Pin, GPIO_PIN_RESET);
	}

	if (motor0Cmd == gobottom) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_0_D_Pin, GPIO_PIN_SET);
	}
}

void motor1Handler()
{
	if (motor1Cmd == stop) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_1_EN_Pin, GPIO_PIN_RESET);
	}

	if (motor1Cmd == gotop) {
		HAL_GPIO_WritePin(GPIOB, MOTOR_1_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MOTOR_1_D_Pin, GPIO_PIN_RESET);
	}

	if (motor1Cmd == gobottom) {
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


void statusPrint()
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

    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

void buttonsHandler()
{
	if (HAL_GPIO_ReadPin (GPIOB, BUTTON_0_Pin) == GPIO_PIN_RESET) {
		farms = open;
	}
	if (HAL_GPIO_ReadPin (GPIOB, BUTTON_1_Pin) == GPIO_PIN_RESET) {
		farms = close;
	}
	if (HAL_GPIO_ReadPin (GPIOB, BUTTON_2_Pin) == GPIO_PIN_RESET) {
		key3VBusState = 1;

	}
	if (HAL_GPIO_ReadPin (GPIOB, BUTTON_3_Pin) == GPIO_PIN_RESET) {
		key3VBusState = 0;
	}

	if (HAL_GPIO_ReadPin (GPIOC, BUTTON_4_Pin) == GPIO_PIN_RESET) {
		chrKey1State = 1;
		chrKey2State = 0;
	}

	if (HAL_GPIO_ReadPin (GPIOC, BUTTON_5_Pin) == GPIO_PIN_RESET) {
		chrKey1State = 0;
		chrKey2State = 1;
	}

	if (HAL_GPIO_ReadPin (GPIOC, BUTTON_6_Pin) == GPIO_PIN_RESET) {
		batKey1State = 0;
		batKey2State = 1;

	}
	if (HAL_GPIO_ReadPin (GPIOC, BUTTON_7_Pin) == GPIO_PIN_RESET) {
		batKey1State = 1;
		batKey2State = 0;
	}


	if (HAL_GPIO_ReadPin (GPIOC, BUTTON_8_Pin) == GPIO_PIN_RESET) {
		sunTrackState = 1;
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
  key3VBusState = 0;

  chrKey1State = 0;
  chrKey2State = 0;

  sunTrackState = 0;

  batKey1Handler();
  batKey2Handler();
  key3VBusHandler();

  chrKey1Handler();
  chrKey2Handler();

  RTC_Set(19, 6, 26, 18, 7, 0, 3);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */






  I2C_Scan();

  troykaI2CHub_init(&hi2c1);
  troykaI2CHub_setBusChannel(0);

  BH1750_Init(&hi2c1);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);
  troykaI2CHub_setBusChannel(1);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);
  troykaI2CHub_setBusChannel(3);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);
  troykaI2CHub_setBusChannel(4);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);

  HAL_ADC_Start_DMA(&hadc, (uint32_t*)&ADC_value, 2);


  sim7020Init(&huart3, &huart2);
  sim7020Dtr();
  sim7020PowerCycle();
  sim7020HardwareInfo();




  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  getDateTime();

	  batKey1Handler();
	  batKey2Handler();
	  chrKey1Handler();
	  chrKey2Handler();

	  key3VBusHandler();

	  adcRead();
	  calculateVBat0();
	  calculateVBat1();
	  readLs();

	  buttonsHandler();

	  endstop00Handler();
	  endstop01Handler();
	  endstop10Handler();
	  endstop11Handler();

	  endstop0Control();
	  endstop1Control();

	  motor0PositionControl();
	  motor1PositionControl();

	  motor0Handler();
	  motor1Handler();

	  sunTrackControl();

	  motor2Handler();
	  motor3Handler();
	  motor4Handler();

	  //statusPrint();

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
