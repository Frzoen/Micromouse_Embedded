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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* LED */
#define LED1_ON 				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF 				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_TOGGLE 			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)
#define LED2_ON 				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF 				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_TOGGLE 			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)
#define LED3_ON 				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define LED3_OFF				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED3_TOGGLE 			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)

/* Motor */
#define MOT1_ENABLE 			HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_RESET)
#define MOT1_DISABLE 			HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET)
#define MOT1_FORWARD 			HAL_GPIO_WritePin(MOT1_DIR_GPIO_Port, MOT1_DIR_Pin, GPIO_PIN_SET)
#define MOT1_BACKWARD 			HAL_GPIO_WritePin(MOT1_DIR_GPIO_Port, MOT1_DIR_Pin, GPIO_PIN_RESET)
#define MOT1_SET_SPEED(Duty) 	TIM3->CCR1 = Duty
#define MOT2_ENABLE 			HAL_GPIO_WritePin(MOT2_EN_GPIO_Port, MOT2_EN_Pin, GPIO_PIN_RESET)
#define MOT2_DISABLE 			HAL_GPIO_WritePin(MOT2_EN_GPIO_Port, MOT2_EN_Pin, GPIO_PIN_SET)
#define MOT2_FORWARD 			HAL_GPIO_WritePin(MOT2_DIR_GPIO_Port, MOT2_DIR_Pin, GPIO_PIN_SET)
#define MOT2_BACKWARD 			HAL_GPIO_WritePin(MOT2_DIR_GPIO_Port, MOT2_DIR_Pin, GPIO_PIN_RESET)
#define MOT2_SET_SPEED(Duty)	TIM3->CCR2 = Duty

/* Distance sensors */
#define ENABLE_FRONT 			HAL_GPIO_WritePin(DIST3_EN_GPIO_Port, DIST3_EN_Pin, GPIO_PIN_RESET)
#define ENABLE_RIGHT 			HAL_GPIO_WritePin(DIST1_EN_GPIO_Port, DIST1_EN_Pin, GPIO_PIN_RESET)
#define ENABLE_LEFT				HAL_GPIO_WritePin(DIST5_EN_GPIO_Port, DIST5_EN_Pin, GPIO_PIN_RESET)
#define ENABLE_FRONT_RIGHT		HAL_GPIO_WritePin(DIST2_EN_GPIO_Port, DIST2_EN_Pin, GPIO_PIN_RESET)
#define ENABLE_FRONT_LEFT		HAL_GPIO_WritePin(DIST4_EN_GPIO_Port, DIST4_EN_Pin, GPIO_PIN_RESET)
#define DISABLE_FRONT			HAL_GPIO_WritePin(DIST3_EN_GPIO_Port, DIST3_EN_Pin, GPIO_PIN_RESET)
#define DISABLE_RIGHT			HAL_GPIO_WritePin(DIST1_EN_GPIO_Port, DIST1_EN_Pin, GPIO_PIN_RESET)
#define DISABLE_LEFT			HAL_GPIO_WritePin(DIST5_EN_GPIO_Port, DIST5_EN_Pin, GPIO_PIN_RESET)
#define DISABLE_FRONT_RIGHT		HAL_GPIO_WritePin(DIST2_EN_GPIO_Port, DIST2_EN_Pin, GPIO_PIN_RESET)
#define DISABLE_FRONT_LEFT		HAL_GPIO_WritePin(DIST4_EN_GPIO_Port, DIST4_EN_Pin, GPIO_PIN_RESET)

#define VCNL4010_I2CADDR_DEFAULT 0x13

// commands and constants
#define VCNL4010_COMMAND 0x80
#define VCNL4010_PRODUCTID 0x81
#define VCNL4010_PROXRATE 0x82
#define VCNL4010_IRLED 0x83
#define VCNL4010_AMBIENTPARAMETER 0x84
#define VCNL4010_AMBIENTDATA 0x85
#define VCNL4010_PROXIMITYDATA 0x87
#define VCNL4010_INTCONTROL 0x89
#define VCNL4010_PROXINITYADJUST 0x8A
#define VCNL4010_INTSTAT 0x8E
#define VCNL4010_MODTIMING 0x8F

typedef enum
{
	VCNL4010_1_95 = 0,
	VCNL4010_3_90625 = 1,
	VCNL4010_7_8125 = 2,
	VCNL4010_16_625 = 3,
	VCNL4010_31_25 = 4,
	VCNL4010_62_5 = 5,
	VCNL4010_125 = 6,
	VCNL4010_250 = 7,
} vcnl4010_freq;

#define VCNL4010_MEASUREAMBIENT 0x10
#define VCNL4010_MEASUREPROXIMITY 0x08
#define VCNL4010_AMBIENTREADY 0x40
#define VCNL4010_PROXIMITYREADY 0x20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
bool 		VCNL4010_begin();
uint8_t		VCNL4010_getLEDcurrent(void);
void 		VCNL4010_setLEDcurrent(uint8_t c);
void		VCNL4010_setFrequency(vcnl4010_freq f);
uint16_t 	VCNL4010_readProximity(void);
uint16_t 	VCNL4010_readAmbient(void);
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
	/* USER CODE BEGIN 2 */
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	HAL_Delay(1000);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		LED2_TOGGLE;
		HAL_Delay(500);
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
bool VCNL4010_begin()
{
}
uint8_t VCNL4010_getLEDcurrent(void)
{

}
void VCNL4010_setLEDcurrent(uint8_t c)
{

}
void VCNL4010_setFrequency(vcnl4010_freq f)
{

}
uint16_t VCNL4010_readProximity(void)
{

}
uint16_t VCNL4010_readAmbient(void)
{
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
