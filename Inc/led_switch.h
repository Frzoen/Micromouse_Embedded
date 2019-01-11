/*
 * led_switch.h
 *
 *  Created on: 11.01.2019
 *      Author: bartek
 */

#ifndef LED_SWITCH_H_
#define LED_SWITCH_H_

/* LED */
#define LED1_ON 						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF 						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_TOGGLE 					HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)
#define LED2_ON 						HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF 						HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_TOGGLE 					HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)
#define LED3_ON 						HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define LED3_OFF						HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED3_TOGGLE 					HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)

#endif /* LED_SWITCH_H_ */
