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
 * @ingroup     F1_CompactController_AuxMCU_Bootloader
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
#include "aes.h"
#include "spi.h"
#include "rt_info.h"
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
	CMD_PING			= 0,
	CMD_Read			= 1,
	CMD_CONF_Write		= 2,
	CMD_SMP_Read		= 3,
	CMD_SMP_Write		= 4,
	CMD_Write			= 5,
	CMD_CalRead			= 6,
	CMD_CalWrite		= 7,
	CMD_Ident			= 8,
	CMD_Prepare_Response= 9,

	CMD_Get_RunMode 	= 0xB0,

	CMD_EnterBootloader = 0xC0,

	CMD_Info_Read 		= 0xD0,

	CMD_BL_Read		 	= 0xFA,
	CMD_BL_Write 		= 0xFB,
	CMD_BL_ReadInfo		= 0xFC,
	CMD_BL_Erase 		= 0xFD,
	CMD_BL_Stay 		= 0xFE,

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

typedef uint32_t (*tCmdHandler)(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);

typedef struct
{
	uint8_t 	cmd;
	tCmdHandler handler;
}tBusCommand;

#define PRT_STX		0x01
#define PRT_ETX		0x03

#define PRT_ACK		0x00
#define PRT_NCK		0x01

typedef struct
{
	uint8_t 	stx;
	uint16_t 	address;
	uint16_t 	len;
	uint8_t 	cmd;
	uint8_t 	data[MAX_SPI_PACKET_SIZE];
	uint16_t 	crc;
	uint8_t 	etx;
}tRT_Command_Packet;

typedef struct
{
	uint16_t 	pck_idx;
	uint16_t 	total_pck_count;
	uint32_t 	wr_addr;							/* Firmware write start address */
	uint8_t 	iv[FW_PACKET_IV_DATA_SIZE];
	uint8_t 	enc_data[FW_ENC_DATA_PACKET_SIZE];
}tRT_FW_Update_Command_Packet;

/*============================================================================*/
/* Global data                                                                */
/*============================================================================*/
extern const tBusCommand commands[];
extern const int gCommandCount;
/*============================================================================*/
/* Declarations                                                               */
/*============================================================================*/
uint32_t rt_bus_cmd_ping_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_reset (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_read_info_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_get_runmode_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_bl_stay_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_bl_write_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_bl_erase_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_read_data_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
uint32_t rt_bus_cmd_prepare_response_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen);
tRT_Command_Packet rt_bus_proto_pack_parser(uint8_t *data);
void rt_bus_proto_bl_process(tRT_Command_Packet sRT_Command_Packet);
void rt_bus_proto_bl_dt(void);
void rt_get_io_values(void);

#endif /* INC_RT_BUS_PROTO_H_ */
/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
