/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        rt_info.c
 * @brief
 *
 * @author      Burak Sahan
 * @date        Feb 10, 2022
 *
 * @ingroup     F1_CompactController_IOExp_Bootloader
 * @{
 *****************************************************************************/

/*============================================================================*/
/* Includes                                                                   */
/*============================================================================*/
#include "rt_info.h"
#include <string.h>
#include "usart.h"
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
eLEDFlashMode g_led_flash_mode = LED_MODE_LS_FLASH;
tRotaAdvanceDeviceInfo sRotaAdvanceDeviceInfo;
tRotaDeviceInfo sRotaDeviceInfo;
/*============================================================================*/
/* Module global data                                                         */
/*============================================================================*/

/*============================================================================*/
/* Implementation of functions                                                */
/*============================================================================*/
/**
 * @brief Application Get Info
 * @param none
 * @return none
 */
void rt_get_app_info(void){

	sRotaDeviceInfo.ubSwLevel = SWLEVEL_APPLICATION;
	sRotaDeviceInfo.ubRunMode = RUNMODE_OPERATION;
	sRotaDeviceInfo.ulManufacturerID = ROTA_MANUFACTURER_ID;
	sRotaDeviceInfo.ubDeviceClass =DEVICE_CONTROLLER;
	uint32_t flash_ret = HAL_Read_Unique_ID(&sRotaDeviceInfo.aulUniqueID[0]);
	if(flash_ret != HAL_OK){
		sRotaDeviceInfo.aulUniqueID[0] = 0;
		sRotaDeviceInfo.aulUniqueID[1] = 0;
		sRotaDeviceInfo.aulUniqueID[2] = 0;
		sRotaDeviceInfo.aulUniqueID[3] = 0;
	}

	sRotaDeviceInfo.ulHWVersion = BOARD_VERSION;
	strncpy(sRotaDeviceInfo.acFWVersion, git_version, sizeof(sRotaDeviceInfo.acFWVersion));
	strncpy(sRotaDeviceInfo.acProjectName, project_description, sizeof(sRotaDeviceInfo.acProjectName));
	strncpy(sRotaDeviceInfo.acManufacturer, company, sizeof(sRotaDeviceInfo.acManufacturer));
	const char *serial = "RTTCxxxxxxxxxx";
	strncpy(sRotaDeviceInfo.acSerialNumber, serial, sizeof(sRotaDeviceInfo.acSerialNumber));
	strncpy(sRotaDeviceInfo.acManufacturer, company, sizeof(sRotaDeviceInfo.acManufacturer));

	sRotaAdvanceDeviceInfo.deviceInfo = sRotaDeviceInfo;
	strncpy(sRotaAdvanceDeviceInfo.acProductionDate, timestamp, sizeof(sRotaAdvanceDeviceInfo.acProductionDate));
	strncpy(sRotaAdvanceDeviceInfo.acFwCompileDate, timestamp, sizeof(sRotaAdvanceDeviceInfo.acFwCompileDate));
	strncpy(sRotaAdvanceDeviceInfo.acCalibrationDate, timestamp, sizeof(sRotaAdvanceDeviceInfo.acCalibrationDate));
	strncpy(sRotaAdvanceDeviceInfo.acTestDate, timestamp, sizeof(sRotaAdvanceDeviceInfo.acTestDate));

	dbprintf("------- %s -------",sRotaAdvanceDeviceInfo.deviceInfo.acProjectName);
	dbprintf("Manufacturer\t\t: %s",sRotaAdvanceDeviceInfo.deviceInfo.acManufacturer);
	dbprintf("Firmware Version\t: %s",sRotaAdvanceDeviceInfo.deviceInfo.acFWVersion);
	dbprintf("Hardware Version\t: %lu.%lu",(sRotaAdvanceDeviceInfo.deviceInfo.ulHWVersion/10),(sRotaAdvanceDeviceInfo.deviceInfo.ulHWVersion%10));
	dbprintf("Production Time\t\t: %s",sRotaAdvanceDeviceInfo.acProductionDate);
	dbprintf("UID\t\t\t: %08lu%08lu%08lu%08lu",	sRotaDeviceInfo.aulUniqueID[3], sRotaDeviceInfo.aulUniqueID[2],	sRotaDeviceInfo.aulUniqueID[1], sRotaDeviceInfo.aulUniqueID[0]);
	dbprintf("----------------------------------------------------------------");

}

/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
