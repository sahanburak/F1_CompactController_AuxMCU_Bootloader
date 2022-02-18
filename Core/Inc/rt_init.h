/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        rt_init.h
 * @brief
 *
 * @author      Burak Sahan
 * @date        Dec 1, 2020
 *
 * @ingroup     F1_CompactController_IOExp_Bootloader
 * @{
 *****************************************************************************/
#ifndef INC_RT_INIT_H_
#define INC_RT_INIT_H_
/*============================================================================*/
/* Includes                                                                   */
/*============================================================================*/
#include <stdint.h>
#include "flash_if.h"
#include "usart.h"
#include "spi.h"
#include "rt_info.h"
/*============================================================================*/
/* Forward declarations                                                       */
/*============================================================================*/
#define APPLICATION_VERS	1

#define VECTOR_TABLE_SIZE		0x1e4					/* Application isr vector size*/

#define MAX_APPLICATION_SIZE	0x37000
#define APPLICATION_ADDRESS     (uint32_t)0x08008000      /* Start user code address: ADDR_FLASH_PAGE_8 */

#define APPLICATION_LEN_ADDRESS (uint32_t) (APPLICATION_ADDRESS + VECTOR_TABLE_SIZE)
#define APPLICATION_CRC_ADDRESS	(uint32_t) (APPLICATION_ADDRESS + 0x50)

#define FLASH_START_ADDRESS		0x08000000
#define FLASH_END_ADDRESS		0x08040000	//Max allowed range 256k

#define RAM_START_ADDRESS		0x20000000
#define RAM_END_ADDRESS			0x2000C000

#define BL_ADDRESS				(uint32_t)0x08000000

#define BOOTLOADER_START_DELAY	500


#define MAGIC_1 				(0x524F5441)		/* Magic 1 : ROTA */
#define MAGIC_2 				(0x54454b4e)		/* Magic 2 : TEKN */
/*============================================================================*/
/* Constants and macros                                                       */
/*============================================================================*/

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/
typedef struct
{
	uint32_t length;
	uint32_t version;
}tAppInfo;

typedef void (*pFunction)(void);
/*============================================================================*/
/* Global data                                                                */
/*============================================================================*/

/*============================================================================*/
/* Declarations                                                               */
/*============================================================================*/
void rt_init();

#endif /* INC_RT_INIT_H_ */
/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
