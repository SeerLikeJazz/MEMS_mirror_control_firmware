/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/*脑电盒状态*/
typedef enum
{
    IMPEDANCING    = 'Z',
    WAVE           = 'W',
    INTERNALSHORT  = 'S',
    TESTSIGAL      = 'T',
    STOP           = 'R'
} EEG_StateTypeDef ;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/** 一帧发送的长度*/
#define  Frame_len  512
/* 是否使用printf */
//#define DEBUG_OUT
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define ADS_DRDY_Pin GPIO_PIN_8
#define ADS_DRDY_GPIO_Port GPIOD
#define ADS_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define KEY_Pin GPIO_PIN_4
#define KEY_GPIO_Port GPIOB
#define KEY_EXTI_IRQn EXTI4_IRQn
#define BUTTON1_Pin GPIO_PIN_0
#define BUTTON1_GPIO_Port GPIOE
#define BUTTON1_EXTI_IRQn EXTI0_IRQn
#define BUTTON2_Pin GPIO_PIN_1
#define BUTTON2_GPIO_Port GPIOE
#define BUTTON2_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
