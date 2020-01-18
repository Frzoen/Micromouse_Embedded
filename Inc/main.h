/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

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
#define SW_1_Pin GPIO_PIN_0
#define SW_1_GPIO_Port GPIOC
#define SW_2_Pin GPIO_PIN_1
#define SW_2_GPIO_Port GPIOC
#define SW_3_Pin GPIO_PIN_2
#define SW_3_GPIO_Port GPIOC
#define SW_4_Pin GPIO_PIN_3
#define SW_4_GPIO_Port GPIOC
#define MOT1_ENCODER_A_Pin GPIO_PIN_0
#define MOT1_ENCODER_A_GPIO_Port GPIOA
#define MOT1_ENCODER_B_Pin GPIO_PIN_1
#define MOT1_ENCODER_B_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOA
#define I_MOT2_Pin GPIO_PIN_0
#define I_MOT2_GPIO_Port GPIOB
#define I_MOT1_Pin GPIO_PIN_1
#define I_MOT1_GPIO_Port GPIOB
#define MOT1_DIR_Pin GPIO_PIN_13
#define MOT1_DIR_GPIO_Port GPIOB
#define MOT1_EN_Pin GPIO_PIN_14
#define MOT1_EN_GPIO_Port GPIOB
#define MOT2_EN_Pin GPIO_PIN_15
#define MOT2_EN_GPIO_Port GPIOB
#define MOT1_PWM_Pin GPIO_PIN_6
#define MOT1_PWM_GPIO_Port GPIOC
#define MOT2_PWM_Pin GPIO_PIN_7
#define MOT2_PWM_GPIO_Port GPIOC
#define MOT2_DIR_Pin GPIO_PIN_8
#define MOT2_DIR_GPIO_Port GPIOC
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOC
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_9
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_10
#define RX_GPIO_Port GPIOA
#define MOT2_ENCODER_B_Pin GPIO_PIN_15
#define MOT2_ENCODER_B_GPIO_Port GPIOA
#define MOT2_ENCODER_A_Pin GPIO_PIN_3
#define MOT2_ENCODER_A_GPIO_Port GPIOB
#define DIST1_EN_Pin GPIO_PIN_4
#define DIST1_EN_GPIO_Port GPIOB
#define DIST2_EN_Pin GPIO_PIN_5
#define DIST2_EN_GPIO_Port GPIOB
#define DIST3_EN_Pin GPIO_PIN_6
#define DIST3_EN_GPIO_Port GPIOB
#define DIST4_EN_Pin GPIO_PIN_7
#define DIST4_EN_GPIO_Port GPIOB
#define DIST5_EN_Pin GPIO_PIN_8
#define DIST5_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
