/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        rt_info.h
 * @brief
 *
 * @author      Burak Sahan
 * @date        Feb 10, 2022
 *
 * @ingroup     F1_CompactController_IOExp_Bootloader
 * @{
 *****************************************************************************/
#ifndef INC_RT_INFO_H_
#define INC_RT_INFO_H_
/*============================================================================*/
/* Includes                                                                   */
/*============================================================================*/
#include "rt_app_info.h"
#include <stdint.h>
#include "flash_if.h"
/*============================================================================*/
/* Forward declarations                                                       */
/*============================================================================*/
#define ROTA_MANUFACTURER_ID	(0x524F5441)			/* 'ROTA' */
/*============================================================================*/
/* Constants and macros                                                       */
/*============================================================================*/

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/
typedef enum {
	DEVICE_NONE			= 0,
	DEVICE_DAQ			= 1,
	DEVICE_CONTROLLER	= 2
}eDeviceClass;


typedef enum {
	SWLEVEL_NONE		= 0,
	SWLEVEL_BOOTLOADER	= 1,
	SWLEVEL_APPLICATION	= 2,
	SWLEVEL_JUMPING		= 3
}eSwLevel;

typedef enum {
	RUNMODE_NONE		= 0,
	RUNMODE_INIT		= 1,
	RUNMODE_OPERATION	= 2,
	RUNMODE_CONF		= 3,
	RUNMODE_CALIBRATION	= 4,
	RUNMODE_FW_UPDATE	= 5
}eRunMode;

typedef enum{
	LED_MODE_NONE		= 0,			/* Not flashing */
	LED_MODE_LS_FLASH 	= 640000,		/* Low speed flash */
	LED_MODE_MS_FLASH	= 320000,		/* Medium speed flash */
	LED_MODE_HS_FLASH	= 160000		/* High speed flash */
}eLEDFlashMode;

typedef struct{
	uint16_t ubSwLevel;				/* Software Level See eSwLevel for all type*/
	uint16_t ubRunMode;				/* Run mode See eRunMode for all type*/
	uint32_t ulManufacturerID;      /* Manufacturer ID */
	uint32_t ubDeviceClass;			/* Rota device class */
	uint32_t aulUniqueID[4];		/* Device Unique ID */
	uint32_t ulHWVersion;			/* Hardware version */
	char acFWVersion[20];			/* Firmware version */
	char acProjectName[100];		/* Project Name*/
	char acManufacturer[20];		/* Manufacturer Name*/
	char acSerialNumber[100];		/* Device Serial Number */
}tRotaDeviceInfo;

typedef struct{
	tRotaDeviceInfo deviceInfo;
	char acProductionDate[20];		/* Production Date */
	char acFwCompileDate[20];		/* Firmware Compile Date */
	char acCalibrationDate[20];		/* Calibration Date */
	char acTestDate[20];			/* Test Date */
}tRotaAdvanceDeviceInfo;

/*============================================================================*/
/* Global data                                                                */
/*============================================================================*/

/*============================================================================*/
/* Declarations                                                               */
/*============================================================================*/
void rt_get_app_info(void);
#endif /* INC_RT_INFO_H_ */
/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
