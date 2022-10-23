/*
 * L4_main_header.h
 *
 *  Created on: Nov 1, 2019
 *      Author: Pavel
 */

#ifndef L4_MAIN_HEADER_H_
#define L4_MAIN_HEADER_H_

#include "stm32l4xx_hal.h"



/* Private defines -----------------------------------------------------------*/
#define TFT_RESET_Pin 			GPIO_PIN_5
#define TFT_RESET_GPIO_Port 	GPIOF
#define TP_CLK_Pin 				GPIO_PIN_1
#define TP_CLK_GPIO_Port 		GPIOA
#define TFT_CS_Pin 				GPIO_PIN_4
#define TFT_CS_GPIO_Port 		GPIOA
#define TP_DOUT_Pin 			GPIO_PIN_6
#define TP_DOUT_GPIO_Port 		GPIOA
#define TP_DIN_Pin 				GPIO_PIN_7
#define TP_DIN_GPIO_Port 		GPIOA
#define TP_CS_Pin 				GPIO_PIN_12
#define TP_CS_GPIO_Port 		GPIOE
#define TFT_DC_Pin 				GPIO_PIN_15
#define TFT_DC_GPIO_Port 		GPIOB
#define LCD_LED_Pin 			GPIO_PIN_8
#define LCD_LED_GPIO_Port 		GPIOC
#define TFT_LED_Pin 			GPIO_PIN_9
#define TFT_LED_GPIO_Port 		GPIOC
#define TFT_SCK_Pin 			GPIO_PIN_10
#define TFT_SCK_GPIO_Port 		GPIOC
#define TFT_MISO_Pin 			GPIO_PIN_4
#define TFT_MISO_GPIO_Port 		GPIOB
#define TFT_MOSI_Pin 			GPIO_PIN_5
#define TFT_MOSI_GPIO_Port 		GPIOB
/* USER CODE BEGIN Private defines */

#endif /* L4_MAIN_HEADER_H_ */
