/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        iap.c
 * @brief       
 *
 * @author      Burak Sahan
 * @date        Dec 1, 2020
 *
 * @ingroup     F1_CompactController_IOExp
 * @{
 *****************************************************************************/

/*============================================================================*/
/* Includes                                                                   */
/*============================================================================*/
#include "main.h"
#include "iap.h"
#include "sha1.h"
#include "spi.h"
#include "rt_init.h"
#include <string.h>
/*============================================================================*/
/* Forward declarations                                                       */
/*============================================================================*/

/*============================================================================*/
/* Constants and macros                                                       */
/*============================================================================*/

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/

/*============================================================================*/
/* Global data                                                                */
/*============================================================================*/
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

/*============================================================================*/
/* Module global data                                                         */
/*============================================================================*/

/*============================================================================*/
/* Implementation of functions                                                */
/*============================================================================*/

/**
 * @brief 	Prepares the Booader to jump into the app
 * @param  None
 * @retval  None
 */
void iap_PrepareForJump(void)
{
	/* Disable all enabled interrupts */
	HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_DisableIRQ(SysTick_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(SPI1_IRQn);

	HAL_DMA_Abort(&hdma_spi1_tx);
	HAL_DMA_Abort(&hdma_spi1_rx);
	HAL_DMA_Abort(&hdma_usart1_tx);

	__HAL_SPI_DISABLE((&hspi1));

	__disable_irq();
}

/**
 * @brief  Jump into the application
 * @param  None
 * @retval None
 */
void iap_JumpToApplication(void)
{
	uint32_t  applicationStack;
	pFunction applicationEntry;

	iap_PrepareForJump();

	/* Retrieve values */
	applicationStack = *(__IO uint32_t *) (APPLICATION_ADDRESS);
	applicationEntry = (pFunction) *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);

	/* Set a valid stack pointer for the application */
	__set_MSP(applicationStack);

	dbprintf("Jumping to application...\n");
	/* Start the application */
	applicationEntry();
}

/**
 * @brief  Checks the application is available.  If the application has written to flash.
 * @param  None
 * @retval IAP_APP_SUCCESS 			: application is available in flash area
 *         IAP_APP_ADDR_NOT_IN_RAGE : jumping addres is not in flash area occurred
 *         IAP_APP_SIZE_OVER_RAGE 	: application size is out of range
 *         IAP_APP_CHCKSUM_ERR 		: application checksum is not correct
 */
uint8_t iap_CheckApplication(void)
{
	SHA1_CTX sha1_ctx;
	uint8_t csha1[20];

	uint32_t length;
	unsigned char *p = (uint8_t *)(APPLICATION_ADDRESS);
	uint32_t JumpAddress= *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);

	if ((JumpAddress < APPLICATION_ADDRESS)|| (JumpAddress > APPLICATION_ADDRESS + MAX_APPLICATION_SIZE))
		return IAP_APP_ADDR_NOT_IN_RAGE;

	length = *(__IO uint32_t*) (APPLICATION_LEN_ADDRESS);
	if ((length <= VECTOR_TABLE_SIZE) || (length > MAX_APPLICATION_SIZE))
	{
		return IAP_APP_SIZE_OVER_RAGE;
	}

	sha1_init(&sha1_ctx);
	sha1_update(&sha1_ctx,p,length);
	sha1_final(&sha1_ctx,csha1);

	if (memcmp(csha1,&p[length],(size_t)20)!=0)
	{
		return IAP_APP_CHCKSUM_ERR;
	}
	return IAP_APP_SUCCESS;
}

/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
