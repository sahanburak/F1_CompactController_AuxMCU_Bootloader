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
//#include "ssi.h"
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
		{CMD_Read,				&rt_bus_cmd_read_data_handler},
		{CMD_Get_RunMode,		&rt_bus_cmd_get_runmode_handler},
		//{CMD_Info_Read,			&rt_bus_cmd_read_info_handler},
		{CMD_RESET,				&rt_bus_cmd_reset},
		{CMD_BL_Stay, 			&rt_bus_cmd_bl_stay},
		{CMD_BL_Write,			&rt_bus_cmd_bl_write_handler},
		{CMD_BL_Erase,			&rt_bus_cmd_bl_erase_handler},

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
/*============================================================================*/
/* Module global data                                                         */
/*============================================================================*/
uint8_t flashBuffer[1024 +16];
const int gCommandCount = (sizeof(commands)/sizeof(tBusCommand));
uint8_t gSPI_Tx_Buf[SPI_TX_BUF_SIZE];
uint8_t gSPI_Rx_Buf[SPI_RX_BUF_SIZE];
uint8_t rxBuffer[SPI_RX_BUF_SIZE];
uint8_t txBuffer[SPI_RX_BUF_SIZE];
uint8_t isFrameReady=0;
uint32_t rxFrameSize = 0;
uint32_t lastRxTime = 0;
uint32_t gFrameCount = 0;
uint8_t str[255];
/*static*/ uint16_t prevDMACnt=SPI_RX_BUF_SIZE;

uint32_t test[2]= {0, 0};
/*============================================================================*/
/* Implementation of functions                                                */
/*============================================================================*/
void rt_bus_proto_frame_pack(uint8_t cmd, uint16_t *datalength)
{
	uint16_t cCRC = 0;


	txBuffer[0] = PRT_STX;

	txBuffer[1] = (/*uBusId*/0 & 0xFF00)>>8;
	txBuffer[2] = (/*uBusId*/0 & 0x00FF);

	txBuffer[3] = (((*datalength)+1) & 0xFF00)>>8;
	txBuffer[4] = (((*datalength)+1) & 0xFF);

	txBuffer[5] = cmd;

	cCRC =  crc16(&txBuffer[PRT_AD_MSB_IDX], *datalength+5);

	txBuffer[*datalength + 6] = (cCRC & 0xFF00)>>8;
	txBuffer[*datalength + 7] = (cCRC & 0x00FF);
	txBuffer[*datalength + 8] = PRT_ETX;

	*datalength += 9;

	memcpy(&gSPI_Tx_Buf[0],&txBuffer[0],*datalength);

}

uint32_t rt_bus_cmd_ping_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	if (rxLen != 1)
	{
		return RT_PROTO_FrameError;
	}

	txData[0] = rxData[0] + 1;
	*txLen = 1;
	return RT_PROTO_OK;
}

uint32_t rt_bus_cmd_reset (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	/*iapMailbox[0] = 0;
	iapMailbox[1] = 0;*/
	NVIC_SystemReset();
	return RT_PROTO_OK;
}


uint32_t rt_bus_cmd_get_runmode_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	uint32_t length = 0;
	txData[0] = gRunMode;
	length ++;
	*txLen = length;
	return RT_PROTO_OK;
}

uint32_t rt_bus_cmd_read_data_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	test[0] +=0x01010101;
	test[1] +=0x01010101;
	memcpy(&txData[0],&test[0],8);
	*txLen = 8;
	return RT_PROTO_OK;
}
/*
uint32_t rt_bus_cmd_read_info_handler(uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	uint32_t length = 0;

	for (int i=11;i>=0;i--)
	{
		txData[length++] = *((uint8_t *)UID_BASE + i);
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
	//if ((address >= FLASH_START_ADDRESS) & (address<=FLASH_END_ADDRESS))
	return MEM_TYPE_FLASH;
	//else if ((address >= RAM_START_ADDRESS) & (address <= RAM_END_ADDRESS))
	return MEM_TYPE_RAM;
	//else
	return MEM_TYPE_UNK;
}

uint32_t rt_bus_cmd_bl_stay (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	gStayInBootloader = 1;
	return RT_PROTO_OK;
}

uint32_t rt_bus_cmd_bl_write_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	uint32_t writeaddress;
	uint32_t mtype;
	uint8_t decBuffer[16];
	uint8_t *iv;
	uint8_t *data;
	if (rxLen != 1060)
		return RT_PROTO_DataError;

	memcpy(&writeaddress,&rxData[0],4);

	/*if (writeaddress < APPLICATION_ADDRESS){
		dbprintf("Write address error!!! writeaddress : %08X, APPLICATION_ADDRESS:%08X",writeaddress,APPLICATION_ADDRESS);
		return RT_PROTO_DataError;
	}
	 */
	// Addr       IV      [ Magic     Data ]
	//  4         16      [  4        1024 ]
	//				      [ Encrypted      ]
	// 0..3      4..19	  [ 20..23	..    1060 ]

	iv = &rxData[4];
	data = &rxData[20];

	//AES_CBC_decrypt_buffer(decBuffer,data,16,AES_KEY,iv);
	if (memcmp("ROTA",&decBuffer[0],4) != 0){
		return RT_PROTO_DataError;
	}
	if (memcmp("ROTA",&decBuffer[12],4) != 0){
		return RT_PROTO_DataError;
	}

	//AES_CBC_decrypt_buffer(flashBuffer,data,1040,AES_KEY,iv);
	mtype = get_mem_type(writeaddress);
	if (mtype == MEM_TYPE_FLASH)
	{
		//FLASH_If_Init();
		uint32_t ret = 0;/*FLASH_If_Write(writeaddress,(uint32_t *)&flashBuffer[16],1024/4);*/
		if (ret == HAL_OK){
			dbprintf("Writing firmware to 0x%08X...",writeaddress);
		}else{
			dbprintf("Write error!!!");
			return RT_PROTO_ExcError;
		}
	}
	else if (mtype == MEM_TYPE_RAM)
	{
		memcpy((uint8_t *)writeaddress,&rxData[4],1024);
	}
	else
	{
		dbprintf("RT_PROTO_DataError");
		return RT_PROTO_DataError;
	}

	return RT_PROTO_OK;
}

uint32_t rt_bus_cmd_bl_erase_handler (uint8_t *rxData,uint16_t rxLen,uint8_t *txData,uint16_t *txLen)
{
	uint32_t eraseaddress;
	uint32_t eraseLen;
	uint32_t mtype;

	if (rxLen != 0x08)
		return RT_PROTO_DataError;

	memcpy(&eraseaddress,&rxData[0],4);
	memcpy(&eraseLen,&rxData[4],4);

	dbprintf("Erasing flash... Start Address : %08X, Size: %d\n\r",eraseaddress,eraseLen);


	if (eraseaddress % FLASH_PAGE_SIZE)
		return RT_PROTO_DataError;
	if (eraseLen % FLASH_PAGE_SIZE)
		return RT_PROTO_DataError;



	mtype = get_mem_type(eraseaddress);
	if (mtype == MEM_TYPE_FLASH)
	{
		/*FLASH_If_Init();
		if (FLASH_If_Erase(eraseaddress,eraseLen) != HAL_OK){
			dbprintf("RT_PROTO_ExcError\n\r");
			return RT_PROTO_ExcError;
		}*/
	}
	else if (mtype == MEM_TYPE_RAM)
	{
		dbprintf("MEM_TYPE_RAM\n\r");
		memset((unsigned char *)eraseaddress,0,eraseLen);
	}
	else
	{
		return RT_PROTO_DataError;
	}

	return RT_PROTO_OK;
}


void SPI_DMA_Reset(){
	HAL_SPI_DMAStop(&hspi1);
	HAL_SPI_TransmitReceive_DMA(&hspi1,gSPI_Tx_Buf, gSPI_Rx_Buf, SPI_RX_BUF_SIZE);
	prevDMACnt = SPI_RX_BUF_SIZE;
	rxFrameSize = 0;
	prevDMA = SPI_TX_BUF_SIZE;
}

void rt_bus_proto_bl_process()
{
	uint16_t pSize = 0;
	uint16_t addr = 0;
	uint16_t cCRC = 0;
	uint16_t txSize = 0;
	uint32_t ret;
	if (isFrameReady)
	{
		if ((rxBuffer[PRT_STX_IDX+PRT_FIX_BYTE_NUM] == PRT_STX) && (rxBuffer[rxFrameSize-PRT_ETX_LEFT_IDX-PRT_FIX_BYTE_END_NUM] == PRT_ETX)) // check stx and etx on received frame
		{
			//dbprintf("start - end ok\n\r");
			pSize = ((uint16_t)rxBuffer[PRT_LEN_MSB_IDX+PRT_FIX_BYTE_NUM] << 8) | (rxBuffer[PRT_LEN_LSB_IDX+PRT_FIX_BYTE_NUM]);
			//dbprintf("pSize: %d  %02X %02X rxFrameSize: %d ok",pSize,rxBuffer[PRT_LEN_MSB_IDX],rxBuffer[PRT_LEN_LSB_IDX],rxFrameSize);
			//
			if (pSize <= rxFrameSize - (8 + PRT_FIX_BYTE_NUM + PRT_FIX_BYTE_END_NUM))
			{
				//dbprintf("size ok");
				addr = ((uint16_t)rxBuffer[PRT_AD_MSB_IDX + PRT_FIX_BYTE_NUM] << 8) | (rxBuffer[PRT_AD_LSB_IDX + PRT_FIX_BYTE_NUM]);
				cCRC = crc16(&rxBuffer[PRT_AD_MSB_IDX+PRT_FIX_BYTE_NUM], rxFrameSize- (4 + PRT_FIX_BYTE_NUM+PRT_FIX_BYTE_END_NUM));
				if (memcmp(&rxBuffer[rxFrameSize-(3+PRT_FIX_BYTE_END_NUM)],&cCRC,2)==0)
				{
					//dbprintf("crc ok");
					for (int i=0;i<gCommandCount ;i++)
					{
						if (commands[i].cmd == rxBuffer[PRT_CMD_IDX+PRT_FIX_BYTE_NUM])
						{
							txSize = 0;
							gFrameCount++;
							ret = commands[i].handler(&rxBuffer[PRT_DATA_S_IDX+PRT_FIX_BYTE_NUM],pSize,&txBuffer[PRT_DATA_S_IDX+1],&txSize);
							if (ret == RT_PROTO_OK)
							{
								txBuffer[PRT_DATA_S_IDX] = PRT_ACK;
								txSize ++;
							}
							else
							{
								txBuffer[PRT_DATA_S_IDX] = PRT_NCK;
								txBuffer[PRT_DATA_S_IDX+1] = ret;
								txSize = 2;
							}
							rt_bus_proto_frame_pack(commands[i].cmd,&txSize);

							break;
						}
					}
				}
				//dbprintf("rxFrameSize: %d startTick:%d , now: %d",rxFrameSize,lastRxTime,HAL_GetTick());
				SPI_DMA_Reset();
			}
		}
		isFrameReady = 0;
	}

	if (rxFrameSize)
	{
		__disable_irq();
		//dbprintf("lastRxTime: %d now: %d",lastRxTime,HAL_GetTick());
		if (HAL_GetTick() > (lastRxTime + 2000))
		{
			dbprintf("Timeout resetting...");
			SPI_DMA_Reset();
		}
		__enable_irq();
	}
}

void rt_bus_proto_bl_dt(void)
{
	uint16_t currentDMACnt = hspi1.hdmarx->Instance->CNDTR;
	uint16_t size=0;
	uint16_t start = 0;
	dbprintf("%s",__func__);
	//dbprintf("currentDMACnt: %d",currentDMACnt);
	//if (hspi1.hdmarx->State == HAL_DMA_STATE_BUSY)
	{
		if (prevDMACnt > currentDMACnt)
		{
			lastRxTime = HAL_GetTick();
			//dbprintf("Received Data Count: %d\n\r",(prevDMACnt-currentDMACnt));
			size = prevDMACnt - currentDMACnt;
			if (size > SPI_RX_BUF_SIZE)
				return;

			start = (SPI_RX_BUF_SIZE - prevDMACnt);

			if (rxFrameSize + size < SPI_RX_BUF_SIZE)
			{
				memcpy(&rxBuffer[rxFrameSize],&gSPI_Rx_Buf[start],size);
				rxFrameSize += size;
				if (rxBuffer[rxFrameSize-1-PRT_FIX_BYTE_END_NUM] == 0x03){
					isFrameReady = 0x01;
				}
			}

			prevDMACnt = currentDMACnt;
		}
		else if (prevDMACnt < currentDMACnt)
		{
			lastRxTime = HAL_GetTick();
			//dbprintf("Received Data Count: %d\n\r",(prevDMACnt-currentDMACnt));
			size = prevDMACnt;
			if (size > SPI_RX_BUF_SIZE)
				return;
			start = (SPI_RX_BUF_SIZE - prevDMACnt);

			if (rxFrameSize + size < SPI_RX_BUF_SIZE)
			{
				memcpy(&rxBuffer[rxFrameSize],&gSPI_Rx_Buf[start],size);
				rxFrameSize +=size;
			}

			size = SPI_RX_BUF_SIZE - currentDMACnt;
			start = 0;
			if (rxFrameSize + size < SPI_RX_BUF_SIZE)
			{
				memcpy(&rxBuffer[rxFrameSize],&gSPI_Rx_Buf[start],size);
				rxFrameSize += size;

				if (rxBuffer[rxFrameSize-1-PRT_FIX_BYTE_END_NUM] == 0x03){
					isFrameReady = 0x01;
				}
			}
			prevDMACnt = currentDMACnt;
		}
		rt_bus_proto_bl_process();
	}
}



void rt_get_io_values(void){
	dbprintf("%s",__func__);
	//dbprintf("gSPI_Tx_Buf: %02X%02X%02X%02X sizeof(tPDO): %d",gSPI_Tx_Buf[4],gSPI_Tx_Buf[5],gSPI_Tx_Buf[6],gSPI_Tx_Buf[7],sizeof(tPDO));
	uint16_t currentDMACnt = hspi1.hdmarx->Instance->CNDTR;
	if((prevDMACnt-currentDMACnt) > 0 && (prevDMACnt-currentDMACnt) != 13){
		//dbprintf("diff : %d",(prevDMACnt-currentDMACnt));
	}
	if((prevDMACnt-currentDMACnt) >= (sizeof(tPDI)+1)){
		//lastRxTime = HAL_GetTick();
		HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin,0);
		memcpy(&g_PDI,&gSPI_Rx_Buf[1],sizeof(tPDI));
		//io_update();
		/*g_PDO.ssi = ssi_read();
		memcpy(&gSPI_Tx_Buf[4],&g_PDO.ssi,4);*/
		memcpy(&gSPI_Tx_Buf[0],&g_PDO,8);
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
