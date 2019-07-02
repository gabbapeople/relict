/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define BUTTON_6_Pin GPIO_PIN_0
#define BUTTON_6_GPIO_Port GPIOC
#define BUTTON_7_Pin GPIO_PIN_1
#define BUTTON_7_GPIO_Port GPIOC
#define BUTTON_4_Pin GPIO_PIN_2
#define BUTTON_4_GPIO_Port GPIOC
#define BUTTON_5_Pin GPIO_PIN_3
#define BUTTON_5_GPIO_Port GPIOC
#define ESTOP_0_0_Pin GPIO_PIN_4
#define ESTOP_0_0_GPIO_Port GPIOA
#define ESTOP_0_1_Pin GPIO_PIN_5
#define ESTOP_0_1_GPIO_Port GPIOA
#define ESTOP_1_0_Pin GPIO_PIN_6
#define ESTOP_1_0_GPIO_Port GPIOA
#define ESTOP_1_1_Pin GPIO_PIN_7
#define ESTOP_1_1_GPIO_Port GPIOA
#define MOTOR_1_EN_Pin GPIO_PIN_0
#define MOTOR_1_EN_GPIO_Port GPIOB
#define MOTOR_1_D_Pin GPIO_PIN_1
#define MOTOR_1_D_GPIO_Port GPIOB
#define MOTOR_0_EN_Pin GPIO_PIN_2
#define MOTOR_0_EN_GPIO_Port GPIOB
#define MOTOR_0_D_Pin GPIO_PIN_10
#define MOTOR_0_D_GPIO_Port GPIOB
#define MOTOR_3_D_Pin GPIO_PIN_12
#define MOTOR_3_D_GPIO_Port GPIOB
#define MOTOR_4_D_Pin GPIO_PIN_13
#define MOTOR_4_D_GPIO_Port GPIOB
#define MOTOR_4_EN_Pin GPIO_PIN_14
#define MOTOR_4_EN_GPIO_Port GPIOB
#define SIM_PWR_Pin GPIO_PIN_15
#define SIM_PWR_GPIO_Port GPIOB
#define MOTOR_3_EN_Pin GPIO_PIN_6
#define MOTOR_3_EN_GPIO_Port GPIOC
#define MOTOR_2_D_Pin GPIO_PIN_7
#define MOTOR_2_D_GPIO_Port GPIOC
#define MOTOR_2_EN_Pin GPIO_PIN_8
#define MOTOR_2_EN_GPIO_Port GPIOC
#define SIM_SLP_Pin GPIO_PIN_9
#define SIM_SLP_GPIO_Port GPIOC
#define KEY_3V_BUS_Pin GPIO_PIN_8
#define KEY_3V_BUS_GPIO_Port GPIOA
#define CHR_KEY_1_Pin GPIO_PIN_9
#define CHR_KEY_1_GPIO_Port GPIOA
#define CHR_KEY_2_Pin GPIO_PIN_10
#define CHR_KEY_2_GPIO_Port GPIOA
#define BAT_KEY_2_Pin GPIO_PIN_11
#define BAT_KEY_2_GPIO_Port GPIOA
#define BAT_KEY_1_Pin GPIO_PIN_12
#define BAT_KEY_1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define BUTTON_8_Pin GPIO_PIN_11
#define BUTTON_8_GPIO_Port GPIOC
#define BUTTON_9_Pin GPIO_PIN_12
#define BUTTON_9_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define BUTTON_3_Pin GPIO_PIN_4
#define BUTTON_3_GPIO_Port GPIOB
#define BUTTON_2_Pin GPIO_PIN_5
#define BUTTON_2_GPIO_Port GPIOB
#define BUTTON_0_Pin GPIO_PIN_6
#define BUTTON_0_GPIO_Port GPIOB
#define BUTTON_1_Pin GPIO_PIN_7
#define BUTTON_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
