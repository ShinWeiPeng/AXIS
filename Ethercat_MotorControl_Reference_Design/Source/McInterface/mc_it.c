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
#include "mc_it.h"
#include "mc_math.h"
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
 * Function Name: TMRTASK1K_IRQHandler
 * Purpose:
 * Params:	None
 * Returns:	None
 * Note:
 * ----------------------------------------------------------------------------
 */
void TMRTASK1K_IRQHandler(void)
{
	TIMER_ClearIntFlag(TMRTASK1K_INSTANCE);
#ifdef MC_DEBUG_ENABLE
 	MH_WRITE_GPIO_PIN(DBG_CTRLLOOP_PROC_PORT, DBG_CTRLLOOP_PROC_PIN, MC_ENABLE);
#endif

	MC_TaskRun();

#ifdef MC_DEBUG_ENABLE
 	MH_WRITE_GPIO_PIN(DBG_CTRLLOOP_PROC_PORT, DBG_CTRLLOOP_PROC_PIN, MC_DISABLE);
#endif
} /* End of TMRTASK1K_IRQHandler() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: PWM_IRQHandler
 * Purpose:
 * Params:	None
 * Returns:	None
 * Note:
 * ----------------------------------------------------------------------------
 */
void PWM_IRQHandler(void)
{
    MH_EpwmRegConfig_t *pHandle = &PwmParamsM1.PwmInfo.EpwmRegister;
#ifdef MC_DEBUG_ENABLE
	MH_WRITE_GPIO_PIN(DBG_FOC_PROC_PORT, DBG_FOC_PROC_PIN, MC_ENABLE);
#endif
	/* Clear interupt flag */
	//EPWMx->INTSTS0 = 0x01;
	EPWM_ClearZeroIntFlag(pHandle->pInst, PwmParamsM1.PwmInfo.EpwmRegister.EpwmChU);

	/* Get current feedback */
	MS_GetCurr_d(&CurrentParamsM1.CurrInfo);
	MS_GetBusVoltage_d(&BusVoltageParamsM1.VbusInfo);
	MS_GetTemp_d(&TemperatureParamsM1.TempInfo);

	if(abzENB)
	{
		MS_GetAbzCount(&IncremEncParamsM1.IncremEncInfo);
	}

	MI_FocCurrController();
	SWP_DriveCtrl(&SwProtParamsM1.SwProtInfo);

#ifdef MC_DEBUG_ENABLE
 	MH_WRITE_GPIO_PIN(DBG_FOC_PROC_PORT, DBG_FOC_PROC_PIN, MC_DISABLE);
#endif
} /* End of PWM_IRQHandler() */

/* End of mc_it.c */
