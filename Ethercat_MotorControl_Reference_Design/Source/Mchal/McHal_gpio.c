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

/* INCLUDE FILE DECLARATIONS */
#include "McHal_gpio.h"
#include "McHal_misc.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */     

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_SwProtectionRegInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MH_SwProtRegInit(MH_SwProtRegConfig_t *pHandle)
{
	/* Enable PWM gate drive control pin */
	MH_SetMFP(pHandle->pPortGateDriveEnb, pHandle->PinGateDriveEnb, 0);
	GPIO_SetMode(pHandle->pPortGateDriveEnb, pHandle->PinGateDriveEnb, GPIO_MODE_OUTPUT);	
	GPIO_SetSlewCtl(pHandle->pPortGateDriveEnb, pHandle->PinGateDriveEnb, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(pHandle->pPortGateDriveEnb, pHandle->PinGateDriveEnb, GPIO_PUSEL_DISABLE);
	MH_WRITE_GPIO_PIN(pHandle->pPortGateDriveEnb,pHandle->PinGateDriveEnb, MC_DISABLE);          
	
	/* Enable PWM watch-dog timer clear pin */
	MH_SetMFP(pHandle->pPortPwmWdtTrg, pHandle->PinPwmWdtTrg, 0);
	GPIO_SetMode(pHandle->pPortPwmWdtTrg, pHandle->PinPwmWdtTrg, GPIO_MODE_OUTPUT);
	GPIO_SetSlewCtl(pHandle->pPortPwmWdtTrg, pHandle->PinPwmWdtTrg, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(pHandle->pPortPwmWdtTrg, pHandle->PinPwmWdtTrg, GPIO_PUSEL_DISABLE);
	MH_WRITE_GPIO_PIN(pHandle->pPortPwmWdtTrg,pHandle->PinPwmWdtTrg, MC_DISABLE);          
	
	/* Enable motor brake control pin */	
	MH_SetMFP(pHandle->pPortMotorBrake, pHandle->PinMotorBrake, 0);
	GPIO_SetMode(pHandle->pPortMotorBrake, pHandle->PinMotorBrake, GPIO_MODE_OUTPUT);	
	GPIO_SetSlewCtl(pHandle->pPortMotorBrake, pHandle->PinMotorBrake, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(pHandle->pPortMotorBrake, pHandle->PinMotorBrake, GPIO_PUSEL_DISABLE);
	MH_WRITE_GPIO_PIN(pHandle->pPortMotorBrake,pHandle->PinMotorBrake, MC_DISABLE);          
	
	return MCSTS_OK;
} /* End of MH_SwProtRegInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_SwProtRegDeInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_SwProtRegDeInit(MH_SwProtRegConfig_t *pHandle)
{

} /* End of MH_SwProtRegDeInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_DebugHwInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_DebugHwInit(void)
{
#ifdef MC_DEBUG_ENABLE	
	/* Motor Control Debug Pins */
	MH_SetMFP(DBG_FOC_PROC_PORT, DBG_FOC_PROC_PIN, 0);
	GPIO_SetMode(DBG_FOC_PROC_PORT, DBG_FOC_PROC_PIN, GPIO_MODE_OUTPUT);
	GPIO_SetSlewCtl(DBG_FOC_PROC_PORT, DBG_FOC_PROC_PIN, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(DBG_FOC_PROC_PORT, DBG_FOC_PROC_PIN, GPIO_PUSEL_PULL_UP);
	MH_WRITE_GPIO_PIN(DBG_FOC_PROC_PORT, DBG_FOC_PROC_PIN, MC_DISABLE);
	
	MH_SetMFP(DBG_CTRLLOOP_PROC_PORT, DBG_CTRLLOOP_PROC_PIN, 0);
	GPIO_SetMode(DBG_CTRLLOOP_PROC_PORT, DBG_CTRLLOOP_PROC_PIN, GPIO_MODE_OUTPUT);
	GPIO_SetSlewCtl(DBG_CTRLLOOP_PROC_PORT, DBG_CTRLLOOP_PROC_PIN, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(DBG_CTRLLOOP_PROC_PORT, DBG_CTRLLOOP_PROC_PIN, GPIO_PUSEL_PULL_UP);
	MH_WRITE_GPIO_PIN(DBG_CTRLLOOP_PROC_PORT, DBG_CTRLLOOP_PROC_PIN, MC_DISABLE);	
#endif
} /* End of MH_DebugHwInit() */

/* End of McHal_gpio.c */

