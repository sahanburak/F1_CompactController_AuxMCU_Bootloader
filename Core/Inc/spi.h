/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */
#define RT_PROTO_HEAD_AND_TAIL_LEN	9
#define FW_PACKET_FLASH_ADDRES_SIZE	4
#define FW_PACKET_IV_DATA_SIZE		16
#define FW_PACKET_MAGIC_DATA_SIZE	(4 + 8 + 4)		/* Magic(ROTA) + Random Data + Magic(ROTA)*/
#define FW_RAW_DATA_PACKET_SIZE 	(1024)			/* Firmware send packet len*/
#define FW_ENC_DATA_PACKET_SIZE 	(FW_RAW_DATA_PACKET_SIZE + FW_PACKET_MAGIC_DATA_SIZE)
#define FW_UPDATE_PACKET_SIZE 		(FW_PACKET_IV_DATA_SIZE + FW_ENC_DATA_PACKET_SIZE)
#define MAX_SPI_PACKET_SIZE 		128//(FW_UPDATE_PACKET_SIZE + RT_PROTO_HEAD_AND_TAIL_LEN)		/* 128+16+16+4=164*/

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
void SPI_DMA_Reset(void);
void SPI_Comms_Init(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
