/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rt_init.h"
#include "flash_if.h"
#include "iap.h"
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
void Set_Comms_Mode(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TP1_Pin GPIO_PIN_2
#define TP1_GPIO_Port GPIOA
#define TP2_Pin GPIO_PIN_3
#define TP2_GPIO_Port GPIOA
#define STM_MODE_SEL_Pin GPIO_PIN_5
#define STM_MODE_SEL_GPIO_Port GPIOC
#define STM_MODE_SEL_EXTI_IRQn EXTI9_5_IRQn
#define EXT_LED1_Pin GPIO_PIN_8
#define EXT_LED1_GPIO_Port GPIOA
#define DBG_TX_Pin GPIO_PIN_9
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_10
#define DBG_RX_GPIO_Port GPIOA
#define EXT_LED2_Pin GPIO_PIN_11
#define EXT_LED2_GPIO_Port GPIOA
#define INT_LED1_Pin GPIO_PIN_11
#define INT_LED1_GPIO_Port GPIOC
#define INT_LED2_Pin GPIO_PIN_12
#define INT_LED2_GPIO_Port GPIOC
#define LCD_SCL_Pin GPIO_PIN_6
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_7
#define LCD_SDA_GPIO_Port GPIOB
#define LCD_RES_Pin GPIO_PIN_8
#define LCD_RES_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define COMMS_MODE_PDIO		0x00
#define COMMS_MODE_CONF		0x01
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
