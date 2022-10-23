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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "L4_main_header.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_RW_Pin GPIO_PIN_2
#define LCD_RW_GPIO_Port GPIOE
#define ENC_A_Pin GPIO_PIN_3
#define ENC_A_GPIO_Port GPIOE
#define ENC_B_Pin GPIO_PIN_4
#define ENC_B_GPIO_Port GPIOE
#define LCD_E_Pin GPIO_PIN_5
#define LCD_E_GPIO_Port GPIOE
#define GPIO_PE6_Pin GPIO_PIN_6
#define GPIO_PE6_GPIO_Port GPIOE
#define I2C2_SDA_Pin GPIO_PIN_0
#define I2C2_SDA_GPIO_Port GPIOF
#define I2C2_SCL_Pin GPIO_PIN_1
#define I2C2_SCL_GPIO_Port GPIOF
#define GPIO_PF2_Pin GPIO_PIN_2
#define GPIO_PF2_GPIO_Port GPIOF
#define _485_DIR_Pin GPIO_PIN_3
#define _485_DIR_GPIO_Port GPIOF
#define TFT_RESET_Pin GPIO_PIN_5
#define TFT_RESET_GPIO_Port GPIOF
#define OUT_MK3_Pin GPIO_PIN_7
#define OUT_MK3_GPIO_Port GPIOF
#define OUT_MK4_Pin GPIO_PIN_8
#define OUT_MK4_GPIO_Port GPIOF
#define GPIO_PF9_Pin GPIO_PIN_9
#define GPIO_PF9_GPIO_Port GPIOF
#define ADC_TEMPER_Pin GPIO_PIN_0
#define ADC_TEMPER_GPIO_Port GPIOC
#define ADC_POT_Pin GPIO_PIN_3
#define ADC_POT_GPIO_Port GPIOC
#define RS232_TxD_Pin GPIO_PIN_0
#define RS232_TxD_GPIO_Port GPIOA
#define TP_SCK_Pin GPIO_PIN_1
#define TP_SCK_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define TFT_CS_Pin GPIO_PIN_4
#define TFT_CS_GPIO_Port GPIOA
#define DAC_REPRO_Pin GPIO_PIN_5
#define DAC_REPRO_GPIO_Port GPIOA
#define TP_MISO_Pin GPIO_PIN_6
#define TP_MISO_GPIO_Port GPIOA
#define TP_MOSI_Pin GPIO_PIN_7
#define TP_MOSI_GPIO_Port GPIOA
#define ADC1_IN13_Pin GPIO_PIN_4
#define ADC1_IN13_GPIO_Port GPIOC
#define ADC1_IN14_Pin GPIO_PIN_5
#define ADC1_IN14_GPIO_Port GPIOC
#define IN_MK5_Pin GPIO_PIN_0
#define IN_MK5_GPIO_Port GPIOB
#define GPIO_PB2_Pin GPIO_PIN_2
#define GPIO_PB2_GPIO_Port GPIOB
#define IN_TL2_Pin GPIO_PIN_12
#define IN_TL2_GPIO_Port GPIOF
#define IN_TL2_EXTI_IRQn EXTI15_10_IRQn
#define GPIO_PF13_Pin GPIO_PIN_13
#define GPIO_PF13_GPIO_Port GPIOF
#define IN_TL1_Pin GPIO_PIN_15
#define IN_TL1_GPIO_Port GPIOF
#define IN_TL1_EXTI_IRQn EXTI15_10_IRQn
#define OUT_MK1_Pin GPIO_PIN_0
#define OUT_MK1_GPIO_Port GPIOG
#define OUT_MK2_Pin GPIO_PIN_1
#define OUT_MK2_GPIO_Port GPIOG
#define OUT_LED3_Pin GPIO_PIN_7
#define OUT_LED3_GPIO_Port GPIOE
#define OUT_LED2_Pin GPIO_PIN_8
#define OUT_LED2_GPIO_Port GPIOE
#define OUT_LED_R_Pin GPIO_PIN_9
#define OUT_LED_R_GPIO_Port GPIOE
#define OUT_LED1_Pin GPIO_PIN_10
#define OUT_LED1_GPIO_Port GPIOE
#define OUT_LED_G_Pin GPIO_PIN_11
#define OUT_LED_G_GPIO_Port GPIOE
#define TP_CS_Pin GPIO_PIN_12
#define TP_CS_GPIO_Port GPIOE
#define OUT_LED_B_Pin GPIO_PIN_13
#define OUT_LED_B_GPIO_Port GPIOE
#define ENC_BUT_Pin GPIO_PIN_15
#define ENC_BUT_GPIO_Port GPIOE
#define IN_MK7_Pin GPIO_PIN_10
#define IN_MK7_GPIO_Port GPIOB
#define IN_MK8_Pin GPIO_PIN_11
#define IN_MK8_GPIO_Port GPIOB
#define RTC_CS_Pin GPIO_PIN_12
#define RTC_CS_GPIO_Port GPIOB
#define TP_IRQ_Pin GPIO_PIN_13
#define TP_IRQ_GPIO_Port GPIOB
#define TP_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define TFT_DC_Pin GPIO_PIN_15
#define TFT_DC_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_14
#define SD_CS_GPIO_Port GPIOD
#define GPIO_PD15_Pin GPIO_PIN_15
#define GPIO_PD15_GPIO_Port GPIOD
#define HEATER_Pin GPIO_PIN_6
#define HEATER_GPIO_Port GPIOC
#define LCD_LED_Pin GPIO_PIN_8
#define LCD_LED_GPIO_Port GPIOC
#define TFT_LED_Pin GPIO_PIN_9
#define TFT_LED_GPIO_Port GPIOC
#define TFT_SCK_Pin GPIO_PIN_10
#define TFT_SCK_GPIO_Port GPIOC
#define RS232_xD_Pin GPIO_PIN_11
#define RS232_xD_GPIO_Port GPIOC
#define _485_TxD_Pin GPIO_PIN_12
#define _485_TxD_GPIO_Port GPIOC
#define CAN_RxD_Pin GPIO_PIN_0
#define CAN_RxD_GPIO_Port GPIOD
#define CAN_TxD_Pin GPIO_PIN_1
#define CAN_TxD_GPIO_Port GPIOD
#define _485_RxD_Pin GPIO_PIN_2
#define _485_RxD_GPIO_Port GPIOD
#define LCD_RS_Pin GPIO_PIN_3
#define LCD_RS_GPIO_Port GPIOD
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOD
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOD
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOD
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOD
#define TFT_MISO_Pin GPIO_PIN_4
#define TFT_MISO_GPIO_Port GPIOB
#define TFT_MOSI_Pin GPIO_PIN_5
#define TFT_MOSI_GPIO_Port GPIOB
#define GPIO_PB6_Pin GPIO_PIN_6
#define GPIO_PB6_GPIO_Port GPIOB
#define SHT_SCL_Pin GPIO_PIN_8
#define SHT_SCL_GPIO_Port GPIOB
#define SHT_SDA_Pin GPIO_PIN_9
#define SHT_SDA_GPIO_Port GPIOB
#define IN_MK6_Pin GPIO_PIN_0
#define IN_MK6_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
void StavSwitch(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
