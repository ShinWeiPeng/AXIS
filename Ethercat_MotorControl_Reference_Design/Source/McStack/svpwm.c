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
 * Function Name: MS_MapPwmBlkInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MS_MapPwmBlkInit(void)
{
	pwmMAX = 1200;
	pwmCMD = 0;
	pwmMS = 5;
	pwmANG = 360;
	pwmINV = 0;
	pwmU = 0;
	pwmV = 0;
	pwmW = 0;
	pwmRST = 0;
	pwmFLT = 4;
	pwmENB = 0;
	pwmBRK = 0;
} /* End of MS_MapPwmBlkInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_SetPhaseVoltage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MS_SetPhaseVoltage(MS_Epwm_Handle_t *pHandle, MCMATH_AB_PHASE_T Vab)
{
	int32_t x, y, z, u, v, w, inv;

	/* Convert DQ vector to UVW vector */ /* 1774=sqrt(3)*1024 */
	x = Vab.Beta;
	y = (Vab.Beta + Vab.Alpha * 1774 / 1024) >> 1;
	z = (Vab.Beta - Vab.Alpha * 1774 / 1024) >> 1;

	if (y < 0)
	{
		/* sector#5 */
		if (z< 0)
		{
			u=(1000+y-z)>>1;
			v=u+z;
			w=u-y;
			pHandle->Sector = 5;
		}
		/* sector#4 */
		else if (x<=0)
		{
			u=(1000+x-z)>>1;
			v=u+z;
			w=v-x;
			pHandle->Sector = 4;
		}
		/* sector#3 */
		else
		{
			u=(1000+y-x)>>1;
			w=u-y;
			v=w+x;
			pHandle->Sector = 3;
		}
	}
	else
	{
		/* sector#2 */
		if (z>=0)
		{
			u=(1000+y-z)>>1;
			v=u+z;
			w=u-y;
			pHandle->Sector = 2;
		}
		/* sector#1 */
		else if (x> 0)
		{
			u=(1000+x-z)>>1;
			v=u+z;
			w=v-x;
			pHandle->Sector = 1;
		}
		/* sector#6 */
		else
		{
			u=(1000+y-x)>>1;
			w=u-y;
			v=w+x;
			pHandle->Sector = 6;
		}
	}

	/* when inverse vw change */
	if (pwmINV)
	{
		inv = v;
		v = w;
		w = inv;
	}

	/* PwmU、PwmV、PwmW+/-1000*/
	pwmU = (u * 2) - 1000;
	pwmV = (v * 2) - 1000;
	pwmW = (w * 2) - 1000;

	/* Set three phase pwm voltage */
	MH_SET_PWMU_DUTY(pHandle->EpwmRegister.pInst, (pwmMAX - pwmU));
	MH_SET_PWMV_DUTY(pHandle->EpwmRegister.pInst, (pwmMAX - pwmV));
	MH_SET_PWMW_DUTY(pHandle->EpwmRegister.pInst, (pwmMAX - pwmW));
} /* End of MS_SetPhaseVoltage() */

/* End of svpwm.c */


