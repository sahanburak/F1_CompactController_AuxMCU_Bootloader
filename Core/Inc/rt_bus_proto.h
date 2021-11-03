/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        rt_bus_proto.h
 * @brief
 *
 * @author      Burak Sahan
 * @date        Dec 1, 2020
 *
 * @ingroup     F1_CompactController_IOExp_Bootloader
 * @{
 *****************************************************************************/
#ifndef INC_RT_BUS_PROTO_H_
#define INC_RT_BUS_PROTO_H_
/*============================================================================*/
/* Includes                                                                   */
/*============================================================================*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "crc16.h"
/*============================================================================*/
/* Forward declarations                                                       */
/*============================================================================*/

/*============================================================================*/
/* Constants and macros                                                       */
/*============================================================================*/

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/
typedef enum
{
	CMD_PING=0x00,

	CMD_Read,
	CMD_CONF_Write,
	CMD_SMP_Read,
	CMD_SMP_Write,
	CMD_Write,
	CMD_CalRead,
	CMD_CalWrite,
	CMD_Ident,

	CMD_Get_RunMode = 0xB0,

	CMD_EnterBootloader = 0xC0,

	CMD_Info_Read 	= 0xD0,

	CMD_BL_Read 	= 0xFA,
	CMD_BL_Write 	= 0xFB,
	CMD_BL_ReadInfo = 0xFC,
	CMD_BL_Erase 	= 0xFD,
	CMD_BL_Stay 	= 0xFE,

	CMD_RESET 		= 0xFF,
}eBusCommand;

typedef enum
{
	RT_PROTO_OK,
	RT_PROTO_FrameError,
	RT_PROTO_DataError,
	RT_PROTO_UnKownCommand,
	RT_PROTO_ExcError,
}eReturnCodes;

typedef enum
{
	RT_RunMode_None,
	RT_RunMode_BL,
	RT_RunMode_APP,
	RT_RunMode_Jumping
}eRunModes;

typedef uint32_t (*tCmdHandler)(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);

typedef struct
{
	uint8_t 	cmd;
	tCmdHandler handler;
}tBusCommand;


//	0		1		2		3		4		5		6		n-3		n-2		n-1
//			MSB		LSB		MSB 	LSB						MSB		LSB
//	STX		AD0		AD1		LEN0	LEN1	CMD		DATA	CRC0	CRC1	ETX
#define PRT_FIX_BYTE_NUM	1
#define PRT_FIX_BYTE_END_NUM	1

#define PRT_STX_IDX			0
#define PRT_AD_MSB_IDX		1
#define PRT_AD_LSB_IDX		2
#define PRT_LEN_MSB_IDX		3
#define PRT_LEN_LSB_IDX		4
#define PRT_CMD_IDX			5
#define PRT_DATA_S_IDX		6
#define PRT_CRC_MSB_LEFT_IDX		3
#define PRT_CRC_LSB_LEFT_IDX		2
#define PRT_ETX_LEFT_IDX			1

#define PRT_STX		0x01
#define PRT_ETX		0x03

#define PRT_ACK		0x00
#define PRT_NCK		0x01

#define SPI_RX_BUF_SIZE		2048
#define SPI_TX_BUF_SIZE		2048
/*============================================================================*/
/* Global data                                                                */
/*============================================================================*/
extern const tBusCommand 		commands[];
extern const int gCommandCount;
/*============================================================================*/
/* Declarations                                                               */
/*============================================================================*/
uint32_t rt_bus_cmd_ping_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_reset (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_read_info_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_get_runmode_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_bl_stay (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_bl_write_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_bl_erase_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_read_data_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
void rt_bus_proto_bl_process();
void rt_bus_proto_bl_dt(void);
void rt_get_io_values(void);

#endif /* INC_RT_BUS_PROTO_H_ */
/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
