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
#include <string.h>
#include "stepper_mode.h"
#include "svpwm.h"
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
 * Function Name: STPM_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
MC_STATUS_E STPM_Init(STPM_CTRL_T *pStpMod)
{
	memset(pStpMod, 0, sizeof(STPM_CTRL_T));
	return MCSTS_OK;
} /* End of STPM_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: STPM_TickRun()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void STPM_TickRun(STPM_CTRL_T *pStpMod)
{
	uint32_t i;

	/* Run tick counter */
	for (i = 0; i < STPM_MAX_TICK_CNTS; i++)
	{
		if (pStpMod->TickMs[i])
		{
			pStpMod->TickMs[i]--;
		}
	}
} /* End of STPM_TickRun() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: STPM_RunStepping()
 * Purpose:
 * Params:
 * Returns:
 * Note:  Step mode control:
          pwmCMD>0,pwmMS>0,pwmANG=x,pwmENB=1 : Continuous operation
          pwmCMD=0,pwmMS= ,pwmANG= ,pwmENB=1 : Stop operation
          pwmCMD>0,pwmMS<0,pwmANG>0,pwmENB=1 : Single step forward(360°C)
          pwmCMD>0,pwmMS<0,pwmANG<0,pwmENB=1 : Single step reversal(360°C)
          pwmCMD>0,pwmMS=0,pwmANG=x,pwmENB=1 : Rotor alignment
          Calibration result:pwmANG=Reference value,pwmDEG=Calibration value
 * ----------------------------------------------------------------------------
 */
STPM_STATE_E STPM_RunStepping(STPM_CTRL_T *pStpMod)
{
	STPM_STATE_E cur_ste;

	/* Emergency event process */
	if (pStpMod->State!=STPM_IDLE && pwmENB==MC_DISABLE)
	{
		pStpMod->State = STPM_DONE;
	}

	/* Record current state */
	cur_ste = pStpMod->State;

	switch (pStpMod->State)
	{
	case STPM_IDLE:
		/* Waiting for request */
		if (pwmRST==0 && pwmCMD!=0 && cmdSIM==MC_DISABLE)
		{
			/* Ready to enter stepping motion */
			cmdCMD = LC_ALL_OFF;//Disable control loops
			uvwRDY = SCM_DISABLE;//Disable vector output
			pwmENB = MC_ENABLE;//Enable PWM hardware driving
			pStpMod->State = STPM_START;
			pStpMod->TickMs[0] = 200;
		}
		break;

	case STPM_START:
		/* Wait ready */
		if (pStpMod->TickMs[0])
		{
			break;
		}

		/* Setup contorl parameters */
		if (pwmMS < 0)
		{
			/* Single turn operation */
			pStpMod->IsSingleTurn = 1;
			pStpMod->State = (pwmANG > 0) ? STPM_FORWARD:STPM_BACKWARD;//Get direction
			pwmMS = -pwmMS;
			pwmANG = 360;
		}
		else
		{
			/* Continuous operation */
			pStpMod->IsSingleTurn = 0;
			pStpMod->State = STPM_FORWARD;

			/* Get positive angle command */
			if (pwmANG < 0)
			{
				pwmANG = -pwmANG;
			}
		}
		pwmDEG = 0;//Specify starting zero vector angle
		pStpMod->StpAng = 1;//Set step angle
		/* Output torgue at phase-D */
		iopVQ = 0;
		iopVD = pwmCMD;
		uvwRDY = SCM_MANUAL_CTRL;//Enable vector manual control

		pStpMod->TickMs[0] = pwmMS;
		break;

	case STPM_FORWARD:
		/* Wait for step delay */
		if (pStpMod->TickMs[0])
		{
			break;
		}

		/* Check end of forward rotating */
		if (pwmDEG < pwmANG)
		{
			pwmDEG += pStpMod->StpAng;
		}
		else
		{
			pStpMod->State = (pStpMod->IsSingleTurn!=0) ? STPM_DONE:STPM_BACKWARD;//Stop motor or backward rotation ?
		}

		/* If command has reset, do zero vector alignment procedure */
		if (pwmCMD == 0)
		{
			pStpMod->State = STPM_ZERO_VECTOR;
		}
		pStpMod->TickMs[0] = pwmMS;
		break;

	case STPM_BACKWARD:
		/* Wait for step delay */
		if (pStpMod->TickMs[0])
		{
			break;
		}

		/* Check end of backward rotating */
		if (pwmDEG > -pwmANG)
		{
			pwmDEG -= pStpMod->StpAng;
		}
		else
		{
			pStpMod->State = (pStpMod->IsSingleTurn!=0) ? STPM_DONE:STPM_FORWARD;//Stop motor or forward rotation ?
		}

		/* If command has reset, do zero vector alignment procedure */
		if (pwmCMD == 0)
		{
			pStpMod->State = STPM_ZERO_VECTOR;
		}
		pStpMod->TickMs[0] = pwmMS;
		break;

	case STPM_ZERO_VECTOR:
		/* Wait for step delay */
		if (pStpMod->TickMs[0])
		{
			break;
		}

		/* Return to zero vector angle */
		if (pwmDEG > pStpMod->StpAng)
		{
			pwmDEG -= pStpMod->StpAng;
		}
		else if (pwmDEG < -pStpMod->StpAng)
		{
			pwmDEG += pStpMod->StpAng;
		}
		else if (pwmDEG > 0)
		{
			pwmDEG--;
		}
		else if (pwmDEG < 0)
		{
			pwmDEG++;
		}
		else
		{
			pStpMod->State = STPM_DONE;
		}
		pStpMod->TickMs[0] = pwmMS;
		break;

	default:
	case STPM_DONE:
		uvwRDY = SCM_DISABLE;//Disable vector output
		iopVQ = 0;
		iopVD = 0;//Clear vector torque
 		pwmDEG = 0;//Clear vector angle
		pwmCMD = 0;//Clear step mode voltage command
		pStpMod->State = STPM_IDLE;//Reset state machine
		break;
	}

	return cur_ste;
} /* End of STPM_RunStepping() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: STPM_VectorAlignment()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
STPM_VECALI_STATE_E STPM_VectorAlignment(STPM_CTRL_T *pStpMod)
{
	STPM_VECALI_STATE_E cur_ste;

	/* Emergency event process */
	if (pStpMod->VecAliState!=STPMVA_IDLE && pwmENB==MC_DISABLE)
	{
		pStpMod->TickMs[0] = 0;
		pStpMod->VecAliState = STPMVA_DONE;
	}

	/* Record current state */
	cur_ste = pStpMod->VecAliState;

	switch (pStpMod->VecAliState)
	{
	default:
	case STPMVA_IDLE:
		/* Waiting for request */
		if (pwmCMD==0 && pwmRST!=0 && cmdSIM==MC_DISABLE)
		{
			/* Ready to enter stepping mode */
			cmdCMD = LC_ALL_OFF;//Disable control loops
			uvwRDY = SCM_DISABLE;//Disable vector output
			pwmENB = MC_ENABLE;//Enable PWM Drving
			uvwDEG0 = 0;//Clear rotor angle offset
			pStpMod->VecAliState = STPMVA_START;
			pStpMod->TickMs[0] = 200;
		}
		break;

	case STPMVA_START:
		/* Wait an duration time */
		if (pStpMod->TickMs[0])
		{
			break;
		}

		/* Driving specified torque at phase-D(rotor) with zero angle */
		uvwRDY = SCM_MANUAL_CTRL;//Enable vector manual control
		pwmDEG = 0;//Set vector angle to be target angle
		iopVQ = 0;//Clear phase-Q torque
		iopVD = pwmRST;//Driving torque to phase-D

		pStpMod->TickMs[0] = STPM_VECTOR_ALIGN_TIMEOUT_MS;//Set timeout waiting for vector alignment
		pStpMod->TickMs[1] = STPM_ROTOR_STABLE_CHECK_GAPTIME_MS;
		pStpMod->RotorStableCnt = 10;
		pStpMod->RotorAngT[1] = uvwDEG;
		pStpMod->VecAliState = STPMVA_WAIT_ALIGN;
		break;

	case STPMVA_WAIT_ALIGN:
		/* Wait an duration time */
		if (pStpMod->TickMs[1])
		{
			break;
		}
		pStpMod->TickMs[1] = STPM_ROTOR_STABLE_CHECK_GAPTIME_MS;

		/* Check rotor has stable */
		pStpMod->RotorAngT[0] = uvwDEG;
		if (pStpMod->RotorAngT[1] == pStpMod->RotorAngT[0])
		{
			if (pStpMod->RotorStableCnt)
			{
				pStpMod->RotorStableCnt--;
			}
		}
		if (pStpMod->RotorStableCnt == 0)
		{
			/* Rotor has stable and calculate zero vector offset between +/-180 edeg */
			pStpMod->RotorAngT[0] = pStpMod->RotorAngT[1] - pwmDEG;
			while(pStpMod->RotorAngT[0]>180)
			{
				pStpMod->RotorAngT[0] -= 360;
			}
			while(pStpMod->RotorAngT[0]<-180)
			{
				pStpMod->RotorAngT[0] +=360;
			}
			uvwDEG0 = pStpMod->RotorAngT[0];
			MC_PRINT("Vector alignment done, uvwDEG0(%d)=uvwDEG(%d)-pwmDEG(%d)\r\n", uvwDEG0, pStpMod->RotorAngT[1], pwmDEG);
			pStpMod->VecAliState = STPMVA_DONE;
			pStpMod->TickMs[0] = 10;
			break;
		}
		else if (pStpMod->TickMs[0] == 0)
		{
			MC_PRINT("Fail to wait rotor stable, rotor_ang_t[1]=%d, rotor_ang_t[0]=%d\r\n", pStpMod->RotorAngT[1], pStpMod->RotorAngT[0]);
			pStpMod->VecAliState = STPMVA_FAIL;
			break;
		}
		pStpMod->RotorAngT[1] = pStpMod->RotorAngT[0];
		break;

	case STPMVA_DONE:
	case STPMVA_FAIL:
		/* Wait delay */
		if (pStpMod->TickMs[0])
		{
			break;
		}
		uvwRDY = SCM_DISABLE;//Disable vector output
		iopVQ = 0;
		iopVD = 0;//Clear vector torque
		pwmRST = 0;//Clear step mode voltage command
		pStpMod->VecAliState = STPMVA_IDLE;//Reset state machine
		break;
	}

	return cur_ste;
} /* End of STPM_VectorAlignment() */

/* End of stepper_mode.c */

