/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "vcnl4010.h"
#include "led_switch.h"
#include "motors.h"
#include "controller.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t VCNL4010_data_16;	//	Temporary 16b data for VCNL4010
uint8_t VCNL4010_data_8;	//	Temporary 8b data for VCNL4010
int detected_VCNL4010 = 0;
double test_distance;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Motor_Controller();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;

	MOT_RIGHT_ENABLE;
	MOT_LEFT_ENABLE;
	MOT_RIGHT_FORWARD;
	MOT_LEFT_FORWARD;
	MOT_RIGHT_SET_SPEED(0);
	MOT_LEFT_SET_SPEED(0);
	InitMouseController();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);


	VCNL4010_disableAll();
	VCNL4010_FRONT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &VCNL4010_data_8, 1, 100);
	if (VCNL4010_data_8 == VCNL4010_ANSWER_ID_REVISION)
	{
		detected_VCNL4010++;
		VCNL4010_begin();
		LED1_ON;
	}
/*
	VCNL4010_disableAll();
	VCNL4010_FRONT_LEFT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &VCNL4010_data_8, 1, 100);
	if (VCNL4010_data_8 == VCNL4010_ANSWER_ID_REVISION)
	{
		detected_VCNL4010++;
		VCNL4010_begin();
	}

	VCNL4010_disableAll();
	VCNL4010_FRONT_RIGHT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &VCNL4010_data_8, 1, 100);
	if (VCNL4010_data_8 == VCNL4010_ANSWER_ID_REVISION)
	{
		detected_VCNL4010++;
		VCNL4010_begin();
	}

	VCNL4010_disableAll();
	VCNL4010_LEFT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &VCNL4010_data_8, 1, 100);
	if (VCNL4010_data_8 == VCNL4010_ANSWER_ID_REVISION)
	{
		detected_VCNL4010++;
		VCNL4010_begin();
	}

	VCNL4010_disableAll();
	VCNL4010_RIGHT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &VCNL4010_data_8, 1, 100);
	if (VCNL4010_data_8 == VCNL4010_ANSWER_ID_REVISION)
	{
		detected_VCNL4010++;
		VCNL4010_begin();
	}
*/
	/*
	switch (detected_VCNL4010)
	{
	case 1:
		LED1_ON;
		LED2_OFF;
		LED3_OFF;
		break;
	case 2:
		LED1_OFF;
		LED2_ON;
		LED3_OFF;
		break;
	case 3:
		LED1_ON;
		LED2_ON;
		LED3_OFF;
		break;
	case 4:
		LED1_OFF;
		LED2_OFF;
		LED3_ON;
		break;
	case 5:
		LED1_ON;
		LED2_OFF;
		LED3_ON;
		break;
	default:
		LED1_OFF;
		LED2_OFF;
		LED3_OFF;
		break;
	}
	*/

	HAL_Delay(100);
	RIGHT_ENCODER_CNT_SET(_INT16_MID);
	LEFT_ENCODER_CNT_SET(_INT16_MID);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//VCNL4010_FRONT_ENABLE;
	//setDriveForward(1500);

	while (1)
	{

		HAL_Delay(100);
		VCNL4010_data_16 = VCNL4010_readProximity();
		test_distance = exp(log(68000.0 / VCNL4010_data_16) / 1.765);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM10)
	{
		MouseController();
	}
	if (htim->Instance == TIM11)
	{
		VCNL4010_Controller();
	}
}

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
