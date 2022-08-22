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
#include "curr_fdbk.h"
#include "svpwm.h"
/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_GetCurr_d()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
MS_Curr_Handle_t MS_GetCurr_d(MS_Curr_Handle_t *pHandle)
{
    static uint16_t i = 0, count_100us = 0;

    count_100us++;

	/* Get Current adc value */
	MH_AdcConv(&pHandle->CurrentRegister, pHandle->ChannelNum);

	adcU = ((pHandle->CurrentRegister.ConvBuffer[0]) + (pHandle->CurrentRegister.ConvBuffer[4])) >> 1;
	adcV = ((pHandle->CurrentRegister.ConvBuffer[1]) + (pHandle->CurrentRegister.ConvBuffer[3])) >> 1;
	adcW = pHandle->CurrentRegister.ConvBuffer[2];
/*
    if(g_MsSvpwmDbgMessage.record == TRUE)

    {
        if(count_100us >= g_MsSvpwmDbgMessage.SamplingTimeMiliSec)
        {
            g_MsSvpwmDbgMessage.Buf1[i] = adcU;
            g_MsSvpwmDbgMessage.Buf2[i] = adcV;
            g_MsSvpwmDbgMessage.Buf3[i] = adcW;
            g_MsSvpwmDbgMessage.Buf4[i] = adcIU;
            i++;
            count_100us = 0;
        }

        if(i >= MS_SVPWM_DBG_BUF_MAX)
            g_MsSvpwmDbgMessage.record = FALSE;
    }
*/

	return (*pHandle);
} /* End of MS_GetCurr_d() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_GetSignedCurr_d()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
MS_Curr_Handle_t MS_GetSignedCurr_d(MS_Curr_Handle_t *pHandle)
{
	int32_t avg_amp, com_amp;
	static int32_t sum_amp = 0;

	/* Get average of three phase current */
	avg_amp = (adcU + adcV + adcW) / 3;

	/* Get three phase current(unit=mA) */

	adcIU = (adcU - adcSUM) * adcK / 100;
	adcIV = (adcV - adcSUM) * adcK / 100;
	adcIW = (adcW - adcSUM) * adcK / 100;


	/* Low pass filter(20kHz/32768=0.6Hz) */
	com_amp = sum_amp >> 15;
	sum_amp += (avg_amp - com_amp);
	adcSUM = com_amp;
	return (*pHandle);
} /* End of MS_GetSignedCurr_d() */

/* End of curr_fdbk.c */

