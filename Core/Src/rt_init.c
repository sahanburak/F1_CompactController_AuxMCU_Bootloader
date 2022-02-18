/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        rt_init.c
 * @brief       
 *
 * @author      Burak Sahan
 * @date        Dec 1, 2020
 *
 * @ingroup     F1_CompactController_IOExp_Bootloader
 * @{
 *****************************************************************************/

/*============================================================================*/
/* Includes                                                                   */
/*============================================================================*/
#include "rt_init.h"
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
const tAppInfo appinfo __attribute__((section (".app_info"))) = {.length = 0x00, .version = APPLICATION_VERS};
const uint8_t appsha[20] __attribute__((section (".app_sha"))) = {0x0};

volatile uint32_t VectorTable[128] __attribute__((section(".RAMVectorTable")));
volatile uint32_t iapMailbox[2] __attribute__((section(".IAPMailbox")));
/*============================================================================*/
/* Module global data                                                         */
/*============================================================================*/

/*============================================================================*/
/* Implementation of functions                                                */
/*============================================================================*/
void rt_init()
{
	SPI_Comms_Init();
	FLASH_If_Init();
}

/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
