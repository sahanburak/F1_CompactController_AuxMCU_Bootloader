/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        io.h
 * @brief
 *
 * @author      Burak Sahan
 * @date        Dec 10, 2020
 *
 * @ingroup     F1_CompactController_IOExp_Application
 * @{
 *****************************************************************************/
#ifndef INC_IO_H_
#define INC_IO_H_
/*============================================================================*/
/* Includes                                                                   */
/*============================================================================*/

/*============================================================================*/
/* Forward declarations                                                       */
/*============================================================================*/
/*Connected to IOExp Outputs*/
#define HAL_GPIO_DO_UNUSED4		(0)
#define HAL_GPIO_DO_UNUSED3		(1)
#define HAL_GPIO_DO_UNUSED2		(2)
#define HAL_GPIO_DO_UNUSED1		(3)

#define HAL_GPIO_DO_DO1			(4)
#define HAL_GPIO_DO_DO2			(5)
#define HAL_GPIO_DO_DO3			(6)
#define HAL_GPIO_DO_DO4			(7)

/*Connected to STM Outputs*/
#define HAL_GPIO_DO_20MA_RES		(8)		//PB0
#define HAL_GPIO_DO_40MA_RES		(9)		//PB1
#define HAL_GPIO_DO_100MA_RES		(10)	//PB2
#define HAL_GPIO_DO_LCD_RES			(11)	//PB8

#define HAL_GPIO_DO_AN3_OPTO		(12)	//PC0
#define HAL_GPIO_DO_AN2_OPTO		(13)	//PC1
#define HAL_GPIO_DO_AN1_OPTO		(14)	//PC2
#define HAL_GPIO_DO_AN0_OPTO		(15)	//PC3

#define HAL_GPIO_DO_EXT_LED1		(16)	//PA8
#define HAL_GPIO_DO_EXT_LED2		(17)	//PB10
#define HAL_GPIO_DO_EXT_LED3		(18)	//PB11
#define HAL_GPIO_DO_EXT_LED4		(19)	//PB12

#define HAL_GPIO_DO_Reserved1		(20)
#define HAL_GPIO_DO_Reserved2		(21)
#define HAL_GPIO_DO_Reserved3		(22)
#define HAL_GPIO_DO_Reserved4		(23)

/*Connected to IOExp Inputs*/
#define HAL_GPIO_DI_DI_4			(0)
#define HAL_GPIO_DI_DI_3			(1)
#define HAL_GPIO_DI_DI_2			(2)
#define HAL_GPIO_DI_DI_1			(3)

#define HAL_GPIO_DI_BTN_3			(4)
#define HAL_GPIO_DI_BTN_2			(5)
#define HAL_GPIO_DI_BTN_1			(6)
#define HAL_GPIO_DI_BTN_0			(7)

/*Connected to STM Inputs*/
#define HAL_GPIO_DI_EXT_BTN_1		(8)		//PA11
#define HAL_GPIO_DI_EXT_BTN_2		(9)		//PA12
#define HAL_GPIO_DI_Reserved1		(10)
#define HAL_GPIO_DI_Reserved2		(11)
#define HAL_GPIO_DI_Reserved3		(12)
#define HAL_GPIO_DI_Reserved4		(13)
#define HAL_GPIO_DI_Reserved5		(14)
#define HAL_GPIO_DI_Reserved6		(15)

#define HAL_GPIO_DI_Reserved7		(20)
#define HAL_GPIO_DI_Reserved8		(21)
#define HAL_GPIO_DI_Reserved9		(22)
#define HAL_GPIO_DI_Reserved10		(23)
#define HAL_GPIO_DI_Reserved11		(20)
#define HAL_GPIO_DI_Reserved12		(21)
#define HAL_GPIO_DI_Reserved13		(22)
#define HAL_GPIO_DI_Reserved14		(23)


#define ERR_LED1_On()			HAL_GPIO_WritePin(INT_LED1_GPIO_Port, INT_LED1_Pin, GPIO_PIN_RESET)
#define ERR_LED1_Off()			HAL_GPIO_WritePin(INT_LED1_GPIO_Port, INT_LED1_Pin, GPIO_PIN_SET)
#define ERR_LED1_Toggle()		HAL_GPIO_TogglePin(INT_LED1_GPIO_Port, INT_LED1_Pin)
#define ERR_LED2_On()			HAL_GPIO_WritePin(INT_LED2_GPIO_Port, INT_LED2_Pin, GPIO_PIN_RESET)
#define ERR_LED2_Off()			HAL_GPIO_WritePin(INT_LED2_GPIO_Port, INT_LED2_Pin, GPIO_PIN_SET)
#define ERR_LED2_Toggle()		HAL_GPIO_TogglePin(INT_LED2_GPIO_Port, INT_LED2_Pin)
#define PIN_SET(port,pin)		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define PIN_RST(port,pin)		HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
/*============================================================================*/
/* Constants and macros                                                       */
/*============================================================================*/

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/
typedef struct
{
	uint32_t dout;
	uint32_t reserved;
}tPDI;

typedef struct
{
	uint32_t din;
	uint32_t ssi;

}tPDO;
/*============================================================================*/
/* Global data                                                                */
/*============================================================================*/

/*============================================================================*/
/* Declarations                                                               */
/*============================================================================*/

void io_init(void);
void io_update_old(uint8_t count);
void io_do(uint8_t ch, uint8_t val);
uint8_t io_update(void);

#endif /* INC_IO_H_ */
/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
