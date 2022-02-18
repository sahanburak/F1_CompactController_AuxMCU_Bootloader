/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        io.c
 * @brief
 *
 * @author      Burak Sahan
 * @date        Dec 10, 2020
 *
 * @ingroup     F1_CompactController_IOExp_Application
 * @{
 *****************************************************************************/

/*============================================================================*/
/* Includes                                                                   */
/*============================================================================*/
#include "stm32f1xx_hal.h"
#include "io.h"
#include "main.h"
#include "spi.h"
#include <stdint.h>
#include <stm32f1xx_hal_def.h>
/*============================================================================*/
/* Forward declarations                                                       */
/*============================================================================*/
#if 0
GPIO_TypeDef* DIRECT_OUTPUT_PORT_LIST[12]={
		DAC_20mA_SEL_GPIO_Port,
		DAC_40mA_SEL_GPIO_Port,
		DAC_100mA_SEL_GPIO_Port,
		LCD_RES_GPIO_Port,
		AIN3_MODE_SEL_GPIO_Port,
		AIN2_MODE_SEL_GPIO_Port,
		AIN1_MODE_SEL_GPIO_Port,
		AIN0_MODE_SEL_GPIO_Port,
		EXT_LED1_GPIO_Port,
		EXT_LED2_GPIO_Port,
		EXT_LED3_GPIO_Port,
		EXT_LED4_GPIO_Port};

uint16_t DIRECT_OUTPUT_PIN_LIST[12]={
		DAC_20mA_SEL_Pin,
		DAC_40mA_SEL_Pin,
		DAC_100mA_SEL_Pin,
		LCD_RES_Pin,
		AIN3_MODE_SEL_Pin,
		AIN2_MODE_SEL_Pin,
		AIN1_MODE_SEL_Pin,
		AIN0_MODE_SEL_Pin,
		EXT_LED1_Pin,
		EXT_LED2_Pin,
		EXT_LED3_Pin,
		EXT_LED4_Pin};

#define DIRECT_OUTPUT_COUNT sizeof(DIRECT_OUTPUT_PIN_LIST)/sizeof(uint16_t)
#define INDIRECT_OUTPUT_COUNT 8
#endif
/*============================================================================*/
/* Constants and macros                                                       */
/*============================================================================*/

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/

/*============================================================================*/
/* Global data                                                                */
/*============================================================================*/
tPDO g_PDO;
tPDI g_PDI;
tPDO prev_PDO;
tPDI prev_PDI;
uint8_t flag = 0;
/*============================================================================*/
/* Module global data                                                         */
/*============================================================================*/

/*============================================================================*/
/* Implementation of functions                                                */
/*============================================================================*/

void io_init(void)
{
	prev_PDI.dout = 0;
#if 0
	PIN_RST(EXP_LATCH_GPIO_Port,EXP_LATCH_Pin);
	PIN_SET(EXP_LATCH_GPIO_Port,EXP_LATCH_Pin);
	PIN_RST(EXP_CLK_GPIO_Port,EXP_CLK_Pin);
	PIN_SET(EXP_OEN_GPIO_Port,EXP_OEN_Pin);
	PIN_SET(EXP_DOUT_GPIO_Port,EXP_DOUT_Pin);
	PIN_RST(EXP_OEN_GPIO_Port,EXP_OEN_Pin); // Enable Outputs
#endif
	io_update();
}

void io_do(uint8_t ch, uint8_t val)
{
	if (val )
	{
		g_PDI.dout |= (1<<ch);
	}
	else
	{
		g_PDI.dout &= (~(1<<ch));
	}
}

uint8_t io_update(void)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	unsigned char rxData = 0;
#if 0
	unsigned char txData = (unsigned char)(g_PDI.dout & 0xFF);
	status = HAL_SPI_TransmitReceive(&hspi3, &txData, &rxData, 1, 10000);
	HAL_GPIO_WritePin(EXP_LATCH_GPIO_Port, EXP_LATCH_Pin, 0);
	HAL_GPIO_WritePin(EXP_LATCH_GPIO_Port, EXP_LATCH_Pin, 1);
	uint8_t direct_output = 0;
	for(direct_output=0;direct_output<DIRECT_OUTPUT_COUNT;direct_output++){
		HAL_GPIO_WritePin(DIRECT_OUTPUT_PORT_LIST[direct_output],DIRECT_OUTPUT_PIN_LIST[direct_output], (g_PDI.dout & (1<<(((direct_output+INDIRECT_OUTPUT_COUNT)%DIRECT_OUTPUT_COUNT)+8))));
	}
#endif
	g_PDO.din = rxData & 0x000000FF;
	return status;
}


/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
