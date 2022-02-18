/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        rt_bus_proto.c
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
#include "rt_bus_proto.h"
#include "main.h"
#include "io.h"
#include "usart.h"
/*============================================================================*/
/* Forward declarations                                                       */
/*============================================================================*/
#define MEM_TYPE_FLASH		0x00
#define MEM_TYPE_RAM		0x01
#define MEM_TYPE_UNK		0x03
/*============================================================================*/
/* Constants and macros                                                       */
/*============================================================================*/
const tBusCommand commands[] =
{
		{CMD_PING, 				&rt_bus_cmd_ping_handler},
		{CMD_Get_RunMode,		&rt_bus_cmd_get_runmode_handler},
		//{CMD_Info_Read,			&rt_bus_cmd_read_info_handler},
		{CMD_RESET,				&rt_bus_cmd_reset},
		{CMD_BL_Stay, 			&rt_bus_cmd_bl_stay_handler},
		{CMD_BL_Write,			&rt_bus_cmd_bl_write_handler},
		{CMD_BL_Erase,			&rt_bus_cmd_bl_erase_handler},
		{CMD_Prepare_Response,	&rt_bus_cmd_prepare_response_handler},

};
/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/

/*============================================================================*/
/* Global data                                                                */
/*============================================================================*/
extern SPI_HandleTypeDef hspi1;

//extern const tAppInfo appinfo;
extern const uint8_t appsha[20];
extern uint32_t gFrameCount;
extern uint32_t iapMailbox[2];
extern const uint8_t aes_key[];
extern uint8_t gStayInBootloader;
extern uint8_t gRunMode;
extern uint16_t currentDMA;
extern uint16_t prevDMA;
extern tPDO g_PDO;
extern tPDI g_PDI;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
/*============================================================================*/
/* Module global data                                                         */
/*============================================================================*/
uint8_t flashBuffer[1024 +16];
const int gCommandCount = (sizeof(commands)/sizeof(tBusCommand));
uint8_t gSPI_Tx_DMA_Buf[SPI_TX_BUF_SIZE];
uint8_t gSPI_Rx_DMA_Buf[SPI_RX_BUF_SIZE];
uint8_t gSPI_Rx_Buf[SPI_RX_BUF_SIZE];
uint8_t isFrameReady=0;
uint32_t rxFrameSize = 0;
uint32_t lastRxTime = 0;
uint32_t gFrameCount = 0;
uint16_t prevDMACnt=SPI_RX_BUF_SIZE;
tRT_Command_Packet gRT_Command_Packet;

/*============================================================================*/
/* Implementation of functions                                                */
/*============================================================================*/

/**
 * @brief  Communication protocol frame packager
 * @param  sRT_Command_Packet 	:  structure of the communication protocol packet
 * @param  datalength			:  Communication frame size
 * @retval none
 */
void rt_bus_proto_frame_pack(tRT_Command_Packet sRT_Command_Packet, uint16_t *datalength)
{
	uint16_t cCRC = 0;

	sRT_Command_Packet.stx = PRT_STX;
	sRT_Command_Packet.address = 0;
	sRT_Command_Packet.len = (*datalength)+sizeof(sRT_Command_Packet.cmd);

	cCRC =  crc16((unsigned char *) &sRT_Command_Packet.address, (*datalength)+5);
	sRT_Command_Packet.crc = cCRC;
	sRT_Command_Packet.etx = PRT_ETX;

	int offset = 0;
	memcpy(&gSPI_Tx_DMA_Buf[offset],&sRT_Command_Packet.stx,sizeof(sRT_Command_Packet.stx));
	offset +=sizeof(sRT_Command_Packet.stx);
	memcpy(&gSPI_Tx_DMA_Buf[offset],&sRT_Command_Packet.address,sizeof(sRT_Command_Packet.address));
	offset +=sizeof(sRT_Command_Packet.address);
	memcpy(&gSPI_Tx_DMA_Buf[offset],&sRT_Command_Packet.len,sizeof(sRT_Command_Packet.len));
	offset +=sizeof(sRT_Command_Packet.len);
	memcpy(&gSPI_Tx_DMA_Buf[offset],&sRT_Command_Packet.cmd,sizeof(sRT_Command_Packet.cmd));
	offset +=sizeof(sRT_Command_Packet.cmd);
	memcpy(&gSPI_Tx_DMA_Buf[offset],&sRT_Command_Packet.data[0],(sRT_Command_Packet.len-sizeof(sRT_Command_Packet.cmd)));
	offset +=(sRT_Command_Packet.len-sizeof(sRT_Command_Packet.cmd));
	memcpy(&gSPI_Tx_DMA_Buf[offset],&sRT_Command_Packet.crc,sizeof(sRT_Command_Packet.crc));
	offset +=sizeof(sRT_Command_Packet.crc);
	memcpy(&gSPI_Tx_DMA_Buf[offset],&sRT_Command_Packet.etx,sizeof(sRT_Command_Packet.etx));
	offset +=sizeof(sRT_Command_Packet.etx);

	rtprintf("Tx Frame Packet: [ ");
	for(int i=0;i<offset;i++){
		rtprintf("%02X ",gSPI_Tx_DMA_Buf[i]);
	}
	rtprintf("]\n\r");
}

/**
 * @brief  Parser from Raw data to communication structure
 * @param  data	 				: raw data pointer
 * @retval tRT_Command_Packet	: structured data from raw packet
 */
tRT_Command_Packet rt_bus_proto_pack_parser(uint8_t *data){

	tRT_Command_Packet sRT_Command_Packet;
	int offset = 0;
	memcpy(&sRT_Command_Packet.stx,&data[0],sizeof(sRT_Command_Packet.stx));
	offset +=sizeof(sRT_Command_Packet.stx);
	memcpy(&sRT_Command_Packet.address,&data[offset],sizeof(sRT_Command_Packet.address));
	offset +=sizeof(sRT_Command_Packet.address);
	memcpy(&sRT_Command_Packet.len,&data[offset],sizeof(sRT_Command_Packet.len));
	offset +=sizeof(sRT_Command_Packet.len);
	memcpy(&sRT_Command_Packet.cmd,&data[offset],sizeof(sRT_Command_Packet.cmd));
	offset +=sizeof(sRT_Command_Packet.cmd);
	memcpy(&sRT_Command_Packet.data,&data[offset],(sRT_Command_Packet.len-sizeof(sRT_Command_Packet.cmd)));
	offset +=(sRT_Command_Packet.len-sizeof(sRT_Command_Packet.cmd));
	memcpy(&sRT_Command_Packet.crc,&data[offset],sizeof(sRT_Command_Packet.crc));
	offset +=sizeof(sRT_Command_Packet.crc);
	memcpy(&sRT_Command_Packet.etx,&data[offset],sizeof(sRT_Command_Packet.etx));
	return sRT_Command_Packet;
}

/**
 * @brief  Ping function handler
 * @param  rxData	: received data
 * @param  rxLen	: length of the received data
 * @param  txData	: data to be transmit
 * @param  txLen	: length of to be transmit data
 * @retval RT_PROTO_OK			: successfull
 * @retval RT_PROTO_FrameError	: Frame error
 */
uint32_t rt_bus_cmd_ping_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	if (rxLen != 1)
	{
		return RT_PROTO_FrameError;
	}

	txData[0] = rxData[0] + 1;
	*txLen = 1;
	return RT_PROTO_OK;
}

/**
 * @brief  Soft reset function handler
 * @param  rxData	: received data
 * @param  rxLen	: length of the received data
 * @param  txData	: data to be transmit
 * @param  txLen	: length of to be transmit data
 * @retval RT_PROTO_OK			: successfull
 */
uint32_t rt_bus_cmd_reset(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	/*iapMailbox[0] = 0;
	iapMailbox[1] = 0;*/
	NVIC_SystemReset();
	return RT_PROTO_OK;
}

/**
 * @brief  Get run mode function hadler
 * @param  rxData	: received data
 * @param  rxLen	: length of the received data
 * @param  txData	: data to be transmit
 * @param  txLen	: length of to be transmit data
 * @retval RT_PROTO_OK			: successfull
 */
uint32_t rt_bus_cmd_get_runmode_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	dbprintf("%s gRunMode: %d",__func__,gRunMode);
	txData[0] = gRunMode;
	(*txLen) = (*txLen)+1;
	return RT_PROTO_OK;
}

/*
uint32_t rt_bus_cmd_read_info_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	uint32_t length = 0;

	for (int i=11;i>=0;i--)
	{
		txData[length++] = *((uint8_t *)UID_BASE + i); // device id
	}


	sprintf((char *)&txData[length],"%s %s",__DATE__,__TIME__);
	length += 32;

	memcpy(&txData[length],&appinfo,sizeof(appinfo));		//Copy App Version
	length += sizeof(appinfo);

	memcpy(&txData[length],appsha,sizeof(appsha));			// Copy SHA of application
	length += sizeof(appsha);

	memcpy(&txData[length],&gFrameCount,sizeof(uint32_t));	// Copy Total Received Frame Count
	length += sizeof(uint32_t);

	//length += bus_rd_info_hook(&txData[length]);	// let application to add data to this frame

 *txLen = length;
	return RT_PROTO_OK;
}*/

uint32_t get_mem_type(uint32_t address)
{
	if ((address >= FLASH_START_ADDRESS) & (address<=FLASH_END_ADDRESS))
		return MEM_TYPE_FLASH;
	else if ((address >= RAM_START_ADDRESS) & (address <= RAM_END_ADDRESS))
		return MEM_TYPE_RAM;
	else
		return MEM_TYPE_UNK;
}

/**
 * @brief  Stays bootloader without jump to application function handler
 * @param  rxData	: received data
 * @param  rxLen	: length of the received data
 * @param  txData	: data to be transmit
 * @param  txLen	: length of to be transmit data
 * @retval RT_PROTO_OK			: successfull
 */
uint32_t rt_bus_cmd_bl_stay_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	dbprintf("%s",__func__);
	gStayInBootloader = 1;
	return RT_PROTO_OK;
}

/**
 * @brief  Writes the firmware to the flash.
 * @param  rxData	: received data
 * @param  rxLen	: length of the received data
 * @param  txData	: data to be transmit
 * @param  txLen	: length of to be transmit data
 * @retval RT_PROTO_OK			: successfull
 * @retval RT_PROTO_DataError	: received data error
 * @retval RT_PROTO_ExcError	: function execution error
 */
uint32_t rt_bus_cmd_bl_write_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	dbprintf("%s",__func__);
	txData[0] = gRunMode;
	(*txLen) = (*txLen)+1;
	uint32_t writeaddress;
	uint32_t mtype;
	uint8_t decBuffer[16];
	uint8_t *iv;
	uint8_t *data;
	if (rxLen != FW_UPDATE_PACKET_SIZE)
		return RT_PROTO_DataError;

	memcpy(&writeaddress,&rxData[0],4);

	if (writeaddress < APPLICATION_ADDRESS){
		dbprintf("Write address error!!! writeaddress : %08X, APPLICATION_ADDRESS:%08X",writeaddress,APPLICATION_ADDRESS);
		return RT_PROTO_DataError;
	}
	/* Firmware update packet frame */
	// Addr       IV      [ Magic      Random Data      Magic     Raw Data	]
	//  4         16      [   4      		8      		  4       	1024 	]
	//				      [ Encrypted      							   		]
	// 0..3      4..19	  [ 20..23		  24..31    	32..35	  36..1060	]
	#define RT_PROTO_HEAD_AND_TAIL_LEN	9

	iv = &rxData[4];
	data = &rxData[20];

	AES_CBC_decrypt_buffer(decBuffer,data,FW_PACKET_MAGIC_DATA_SIZE,AES_KEY,iv);
	if (memcmp("ROTA",&decBuffer[0],4) != 0){
		return RT_PROTO_DataError;
	}
	if (memcmp("ROTA",&decBuffer[12],4) != 0){
		return RT_PROTO_DataError;
	}

	AES_CBC_decrypt_buffer(flashBuffer,data,FW_ENC_DATA_PACKET_SIZE,AES_KEY,iv);
	mtype = get_mem_type(writeaddress);
	if (mtype == MEM_TYPE_FLASH)
	{
		FLASH_If_Init();
		uint32_t ret = FLASH_If_Write(writeaddress,(uint32_t *)&flashBuffer[16],FW_RAW_DATA_PACKET_SIZE/4 /* 32-bit conversion*/ );
		if (ret == HAL_OK){
			dbprintf("Writing firmware to 0x%08X...",writeaddress);
		}else{
			dbprintf("Write error!!!");
			return RT_PROTO_ExcError;
		}
	}
	else if (mtype == MEM_TYPE_RAM)
	{
		memcpy((uint8_t *)writeaddress,&rxData[4],FW_RAW_DATA_PACKET_SIZE);
	}
	else
	{
		dbprintf("RT_PROTO_DataError");
		return RT_PROTO_DataError;
	}

	return RT_PROTO_OK;
}

/**
 * @brief  Erases the firmware from the flash.
 * @param  rxData	: received data
 * @param  rxLen	: length of the received data
 * @param  txData	: data to be transmit
 * @param  txLen	: length of to be transmit data
 * @retval RT_PROTO_OK			: successfull
 * @retval RT_PROTO_DataError	: received data error
 * @retval RT_PROTO_ExcError	: function execution error
 */
uint32_t rt_bus_cmd_bl_erase_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	uint32_t eraseaddress;
	uint32_t eraseLen;
	uint32_t mtype = MEM_TYPE_UNK;

	if (rxLen != 0x08)
		return RT_PROTO_DataError;

	memcpy(&eraseaddress,&rxData[0],4);
	memcpy(&eraseLen,&rxData[4],4);

	dbprintf("Erasing flash... Start Address : %08X, Size: %d",eraseaddress,eraseLen);

	if (eraseaddress % FLASH_PAGE_SIZE){
		dbprintf("%s_%d",__func__,__LINE__);
		return RT_PROTO_DataError;
	}
	if (eraseLen % FLASH_PAGE_SIZE){
		dbprintf("%s_%d",__func__,__LINE__);
		return RT_PROTO_DataError;
	}

	mtype = get_mem_type(eraseaddress);

	if (mtype == MEM_TYPE_FLASH)
	{
		dbprintf("MEM_TYPE_FLASH");
		if (FLASH_If_Init() != HAL_OK){
			dbprintf("Flash init error");
			return RT_PROTO_ExcError;
		}
		if (FLASH_If_Erase(eraseaddress,eraseLen) != HAL_OK){
			dbprintf("RT_PROTO_ExcError");
			return RT_PROTO_ExcError;
		}
		dbprintf("Erased flash");
	}
	else if (mtype == MEM_TYPE_RAM)
	{
		dbprintf("MEM_TYPE_RAM");
		memset((unsigned char *)eraseaddress,0,eraseLen);
	}
	else
	{
		return RT_PROTO_DataError;
	}

	return RT_PROTO_OK;
}

/**
 * @brief  Idle function for to prepare the response.  Its about SPI communication.
 * @param  rxData	: received data
 * @param  rxLen	: length of the received data
 * @param  txData	: data to be transmit
 * @param  txLen	: length of to be transmit data
 * @retval RT_PROTO_OK			: successfull
 * @retval RT_PROTO_DataError	: received data error
 * @retval RT_PROTO_ExcError	: function execution error
 */
uint32_t rt_bus_cmd_prepare_response_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	/* Must not do nothing */
	return RT_PROTO_OK;
}

void rt_bus_proto_bl_process(tRT_Command_Packet sRT_Command_Packet )
{
	uint16_t cCRC = 0;
	uint16_t txSize = 0;
	uint32_t ret;
	if (isFrameReady)
	{
		cCRC = crc16((unsigned char *) &sRT_Command_Packet.address, (sizeof(sRT_Command_Packet.address)+sizeof(sRT_Command_Packet.len)+sRT_Command_Packet.len));
		if(memcmp(&sRT_Command_Packet.crc,&cCRC,2) == 0){
			for (int i=0;i<gCommandCount ;i++)
			{
				if (commands[i].cmd == sRT_Command_Packet.cmd)
				{
					txSize = 0;
					gFrameCount++;
					ret = commands[i].handler(&sRT_Command_Packet.data[0],(sRT_Command_Packet.len-sizeof(sRT_Command_Packet.cmd)),&gRT_Command_Packet.data[1],&txSize);
					if(commands[i].cmd != CMD_Prepare_Response){
						gRT_Command_Packet.cmd = commands[i].cmd;
						if (ret == RT_PROTO_OK)
						{
							gRT_Command_Packet.data[0]= PRT_ACK;
							txSize +=1;
						}
						else
						{
							gRT_Command_Packet.data[0]= PRT_NCK;
							gRT_Command_Packet.data[1]= ret;
							txSize = 2;
						}
						rt_bus_proto_frame_pack(gRT_Command_Packet/*commands[i].cmd*/,&txSize);
					}
					break;
				}
			}
		}else{
			dbprintf("CRC ERROR: Calculated: %04X Received: %04X",cCRC,sRT_Command_Packet.crc);
			dbprintf("********************************************************");
			dbprintf("STX :%02X",sRT_Command_Packet.stx);
			dbprintf("ADDR :%04X",sRT_Command_Packet.address);
			dbprintf("LEN :%04X",sRT_Command_Packet.len);
			dbprintf("CMD :%02X",sRT_Command_Packet.cmd);
			dbprintf("Data :%02X %02X %02X",sRT_Command_Packet.data[0],sRT_Command_Packet.data[1],sRT_Command_Packet.data[2]);
			dbprintf("CRC :%04X",sRT_Command_Packet.crc);
			dbprintf("ETX :%02X",sRT_Command_Packet.etx);
			dbprintf("********************************************************");
		}
		memset(&sRT_Command_Packet,0,sizeof(tRT_Command_Packet));
		SPI_DMA_Reset();
		isFrameReady = 0;
	}
	/*if (rxFrameSize)
	{
		__disable_irq();
		//dbprintf("lastRxTime: %d now: %d",lastRxTime,HAL_GetTick());
		if (HAL_GetTick() > (lastRxTime + 2000))
		{
			dbprintf("Timeout resetting...");
			SPI_DMA_Reset();
		}
		__enable_irq();
	}*/
}

void rt_bus_proto_bl_dt_old(void)
{
	uint16_t currentDMACnt = hspi1.hdmarx->Instance->CNDTR;
	uint16_t size=0;
	uint16_t start = 0;
	tRT_Command_Packet sRT_Command_Packet;
	//
		if(currentDMACnt !=prevDMACnt){
			dbprintf("currentDMACnt: %d",currentDMACnt);
		}
	if (prevDMACnt > currentDMACnt)
	{
		lastRxTime = HAL_GetTick();
		size = prevDMACnt - currentDMACnt;
		if (size > SPI_RX_BUF_SIZE)
			return;

		if(size < 9)
			return;
		start = (SPI_RX_BUF_SIZE - prevDMACnt);

		if (rxFrameSize + size < SPI_RX_BUF_SIZE)
		{
			sRT_Command_Packet = rt_bus_proto_pack_parser(&gSPI_Rx_DMA_Buf[start]);
			rxFrameSize += size;
			if(sRT_Command_Packet.len < size){
				if(sRT_Command_Packet.stx == PRT_STX && sRT_Command_Packet.etx==PRT_ETX){
					isFrameReady = 0x01;
					//dbprintf("Packet Ready");
				}
			}
		}
		prevDMACnt = currentDMACnt;

	}
	else if (prevDMACnt < currentDMACnt)
	{

		lastRxTime = HAL_GetTick();
		size = prevDMACnt;
		if (size > SPI_RX_BUF_SIZE)
			return;
		start = (SPI_RX_BUF_SIZE - prevDMACnt);

		if (rxFrameSize + size < SPI_RX_BUF_SIZE)
		{
			sRT_Command_Packet = rt_bus_proto_pack_parser(&gSPI_Rx_DMA_Buf[start]);
			rxFrameSize += size;
		}
		size = SPI_RX_BUF_SIZE - currentDMACnt;
		start = 0;


		if (rxFrameSize + size < SPI_RX_BUF_SIZE)
		{
			sRT_Command_Packet = rt_bus_proto_pack_parser(&gSPI_Rx_DMA_Buf[start]);
			rxFrameSize += size;
			if(sRT_Command_Packet.len < size){
				if(sRT_Command_Packet.stx == PRT_STX && sRT_Command_Packet.etx==PRT_ETX){
					isFrameReady = 0x01;
					//dbprintf("Packet Ready");
				}
			}

		}
		prevDMACnt = currentDMACnt;
	}
	rt_bus_proto_bl_process(sRT_Command_Packet);
}




void rt_bus_proto_bl_dt(void)
{
	tRT_Command_Packet sRT_Command_Packet;
	memcpy(gSPI_Rx_Buf, gSPI_Rx_DMA_Buf, SPI_RX_BUF_SIZE);
	sRT_Command_Packet = rt_bus_proto_pack_parser(&gSPI_Rx_Buf[0]);
	if(sRT_Command_Packet.len < 1068){
		if(sRT_Command_Packet.stx == PRT_STX && sRT_Command_Packet.etx==PRT_ETX){
			isFrameReady = 0x01;
			//dbprintf("Packet Ready");
		}
	}
	memset(gSPI_Rx_DMA_Buf, 0, SPI_RX_BUF_SIZE);
	rt_bus_proto_bl_process(sRT_Command_Packet);
}

void rt_get_io_values(void){
	//dbprintf("gSPI_Tx_Buf: %02X%02X%02X%02X sizeof(tPDO): %d",gSPI_Tx_Buf[4],gSPI_Tx_Buf[5],gSPI_Tx_Buf[6],gSPI_Tx_Buf[7],sizeof(tPDO));
	return;
	uint16_t currentDMACnt = hspi1.hdmarx->Instance->CNDTR;
	if((prevDMACnt-currentDMACnt) > 0 && (prevDMACnt-currentDMACnt) != 13){
		//dbprintf("diff : %d",(prevDMACnt-currentDMACnt));
	}
	if((prevDMACnt-currentDMACnt) >= (sizeof(tPDI)+1)){
		//lastRxTime = HAL_GetTick();
		HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin,0);
		memcpy(&g_PDI,&gSPI_Rx_DMA_Buf[1],sizeof(tPDI));
		//io_update();
		/*g_PDO.ssi = ssi_read();
		memcpy(&gSPI_Tx_Buf[4],&g_PDO.ssi,4);*/
		memcpy(&gSPI_Tx_DMA_Buf[0],&g_PDO,8);
		//dbprintf("SSI RAW: %08X  Din: %08X",g_PDO.ssi,g_PDO.din);

		prevDMACnt = currentDMACnt;
		SPI_DMA_Reset();
		HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin,1);
	}
	/*if ((prevDMACnt-currentDMACnt) != 0)
	{
		if (HAL_GetTick() > (lastRxTime + 100))
		{
			dbprintf("Timeout resetting  size: %d...",(prevDMACnt-currentDMACnt));
			SPI_DMA_Reset();
		}
	}*/
}
/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
