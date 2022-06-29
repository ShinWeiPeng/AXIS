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
#include "typedef.h"
#include "math.h"

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

typedef struct
{
    int32_t Nx;
    int32_t Ny;
}_MsSvpwm6StepVector;

typedef struct
{
    int32_t Max;
    int32_t Medium;
    int32_t Min;
}_MsSvpwm6StepDuty;

void MS_GetSvpwmDuty(_MsSvpwm6StepVector *v, _MsSvpwm6StepDuty *duty)
{
    duty->Min = ((float)PWM_DUTY_MAX * ((float)(1000 - v->Nx - v->Ny) / 1000)) / 2;
    duty->Medium = duty->Max + v->Nx;
    duty->Max = duty->Medium + v->Ny;
}

typedef enum
{
    MSSA_SECTION0,
    MSSA_SECTION1,
    MSSA_SECTION2,
    MSSA_SECTION3,
    MSSA_SECTION4,
    MSSA_SECTION5,
    MSSA_SECTION6,
    MSSA_SECTION7
}_MsSvpwm6StepArea;

_MsSvpwm6StepArea MS_GetSvpwmArea(int32_t u, int32_t v, int32_t w)
{
    uint8_t n = 0;
    _MsSvpwm6StepArea area;

    if(u > 0)
        BIT_SET(n, 0);

    if(v > 0)
        BIT_SET(n, 1);

    if(w > 0)
        BIT_SET(n, 2);

    if(n == 3)
        area = MSSA_SECTION1;
    else if(n == 1)
        area = MSSA_SECTION2;
    else if(n == 5)
        area = MSSA_SECTION3;
    else if(n == 4)
        area = MSSA_SECTION4;
    else if(n == 6)
        area = MSSA_SECTION5;
    else
        area = MSSA_SECTION6;

    return area;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_SetPhaseVoltage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
#define VOLTAGE_BUS 24
void MS_SetPhaseVoltage(MS_Epwm_Handle_t *pHandle, MCMATH_AB_PHASE_T Vab)
{
	int32_t x, y, z, u, v, w, inv;
	int32_t u_prime[3];
	_MsSvpwm6StepVector vector;
	_MsSvpwm6StepDuty duty;
	_MsSvpwm6StepArea area;

	/* Convert DQ vector to UVW vector */ /* 1774=sqrt(3)*1024 */
	x = Vab.Beta;
	z = (-Vab.Beta - Vab.Alpha * 1774 / 1024) >> 1;
	y = (-Vab.Beta + Vab.Alpha * 1774 / 1024) >> 1;

	u_prime[0] = x * 1774 / VOLTAGE_BUS / 1024;
	u_prime[1] = y * 1774 / VOLTAGE_BUS / 1024;
	u_prime[2] = z  * 1774 / VOLTAGE_BUS / 1024;

	area = MS_GetSvpwmArea(x, y, z);
	switch(area)
	{
	case MSSA_SECTION1:
        vector.Nx = u_prime[1];
        vector.Ny = u_prime[0];
        MS_GetSvpwmDuty(&vector, &duty);
        u = duty.Max;
        v = duty.Medium;
        w = duty.Min;
        pHandle->Sector = 1;
	    break;

    case MSSA_SECTION2:
        vector.Nx = -u_prime[1];
        vector.Ny = -u_prime[2];
        MS_GetSvpwmDuty(&vector, &duty);
        u = duty.Medium;
        v = duty.Max;
        w = duty.Min;
        pHandle->Sector = 2;
        break;

    case MSSA_SECTION3:
        vector.Nx = u_prime[0];
        vector.Ny = u_prime[2];
        MS_GetSvpwmDuty(&vector, &duty);
        u = duty.Min;
        v = duty.Max;
        w = duty.Medium;
        pHandle->Sector = 3;
        break;

    case MSSA_SECTION4:
        vector.Nx = -u_prime[0];
        vector.Ny = -u_prime[1];
        MS_GetSvpwmDuty(&vector, &duty);
        u = duty.Min;
        v = duty.Medium;
        w = duty.Max;
        pHandle->Sector = 4;
        break;

    case MSSA_SECTION5:
        vector.Nx = u_prime[2];
        vector.Ny = u_prime[1];
        MS_GetSvpwmDuty(&vector, &duty);
        u = duty.Medium;
        v = duty.Min;
        w = duty.Max;
        pHandle->Sector = 5;
        break;

    case MSSA_SECTION6:
    default:
        vector.Nx = -u_prime[2];
        vector.Ny = -u_prime[0];
        MS_GetSvpwmDuty(&vector, &duty);
        u = duty.Max;
        v = duty.Min;
        w = duty.Medium;
        pHandle->Sector = 6;
        break;
	}

	/* when inverse vw change */
	if (pwmINV)
	{
		inv = v;
		v = w;
		w = inv;
	}

	/* PwmU、PwmV、PwmW+/-1000*/
	pwmU = u;
	pwmV = v;
	pwmW = w;

	/* Set three phase pwm voltage */
	MH_SET_PWMU_DUTY(pHandle->EpwmRegister.pInst, (pwmU));
	MH_SET_PWMV_DUTY(pHandle->EpwmRegister.pInst, (pwmV));
	MH_SET_PWMW_DUTY(pHandle->EpwmRegister.pInst, (pwmW));
} /* End of MS_SetPhaseVoltage() */

/* End of svpwm.c */


