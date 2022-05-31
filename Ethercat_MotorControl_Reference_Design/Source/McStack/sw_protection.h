/*
 ******************************************************************************
 * Software License Agreement
 *     Copyright (c) 2021 ASIX Electronics Corporation   All rights reserved.
 * (1) This software is owned by ASIX Electronics Corporation and is protected
 *     under all applicable laws, including copyright laws.
 * (2) This software is provided by ASIX Electronics Corporation, including
 *     modifications and/or derivative works of this software, are only authorized
 *     for use on ASIX products.
 * (3) Redistribution and use of this software without specific written permission
 *     is void and will automatically terminate your rights under this license.
 * (4) Redistribution of source code or in binary form must retain/reproduce the
 *     copyright notice above and the following disclaimer in the documentation or other
 *     materials provided with the distribution.
 *
 * DISCLAIMER
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ASIX MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
 * LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
 * TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER ASIX
 * ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
 * FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
 * ANY REASON RELATED TO THIS SOFTWARE, EVEN IF ASIX OR ITS AFFILIATES HAVE
 * BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * ASIX reserves the right, without notice, to make changes to this software
 * and to discontinue the availability of this software.
 ******************************************************************************
 */

#ifndef __SW_PROTECTION_H__
#define __SW_PROTECTION_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_cfg.h"
#include "McHal_gpio.h"
#include "bus_vol_sensor.h"
#include "temp_sensor.h"
#include "curr_fdbk.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */
typedef enum
{
	SAT_NO_ERR			    = 0,
	SAT_UNDER_VOLT			= 1,
	SAT_OVER_VOLT		    = 2,
	SAT_OVER_TEMP		    = 3,
	SAT_PHASE_ERR       	= 4,
	SAT_OVER_CURR_ID		= 5,
	SAT_OVER_CURR_IQ		= 6,
} SPT_ALARM_TYPE_E;

typedef struct {
	MH_SwProtRegConfig_t    SwProtRegister;
	MS_Vbus_Handle_t        *pVbusInfo;
	MS_Temp_Handle_t        *pTempInfo;
	MS_Curr_Handle_t        *pCurrInfo;
} MS_SwProt_Handle_t;

/* MACRO DECLARATIONS */

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
void MS_MapProtectBlkInit(void);
void MS_GetAvBusVoltage_V(MS_SwProt_Handle_t *pHandle);
void MS_GetAvTemp_V(MS_SwProt_Handle_t *pHandle);
void MS_GetAvI2T_V(MS_SwProt_Handle_t *pHandle);
void SWP_Run(MS_SwProt_Handle_t *pHandle);
void SWP_DriveCtrl(MS_SwProt_Handle_t *pHandle);

#endif /* __SW_PROTECTION_H__ */

/* End of sw_protection.h */
