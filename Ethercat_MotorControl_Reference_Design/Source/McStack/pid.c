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
#include "pid.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_MapPosBlkInit()
 * Purpose:
 * Params:
 * Returns:
 * Note: posOUT: Position Pid output,unit: ticks/ms, range: +/-5000
 *       posSUM: Integraler
 * ----------------------------------------------------------------------------
 */
void MS_MapPosBlkInit(void)
{
	posSUM = 0;
	posOUT = 0;
	posERR = 0;
} /* End of MS_MapPosBlkInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_MapSpdBlkInit()
 * Purpose:
 * Params:
 * Returns:
 * Note: spdOUT: Speed loop Pid output,unit: 0.1%, range: +/-1000
 *       spdSUM: Integraler
 * ----------------------------------------------------------------------------
 */
void MS_MapSpdBlkInit(void)
{
	spdSUM = 0;
	spdOUT = 0;
	spdERR = 0;
	spdDIV = 1;
} /* End of MS_MapSpdBlkInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_MapAmpBlkInit()
 * Purpose:
 * Params:
 * Returns:
 * Note: ampCMD: Current loop command, unit: 0.1%, range: +/-1000.
 *       iopID/iopIQ: Id,Iq ,unit: 0.1%, range: +/-1000.
 *       iopVD/iopVQ: Vd,Vq ,unit: 0.1%, range: +/-1000.
 *       ampSD/ampSQ: DQ Integraler.
 *       ampLMT: Current loop command limit.
 * ----------------------------------------------------------------------------
 */
void MS_MapAmpBlkInit(void)
{
	ampSD = 0;
	ampSQ = 0;
	iopVD = 0;
	iopVQ = 0;
	ampCMD = 0;
	ampLMT = 1000;
	ampERR = 0;
	ampERR2 = 0;
	ampDIV = 1;
} /* End of MS_MapAmpBlkInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_PidController()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MS_PiController(MS_PI_Handle_t *pHandle, int32_t type)
{
	static int32_t PosCmd_old = 0,SpdCmd_old = 0,AmpCmd_old = 0;
	static int32_t AmpCmdCnt = 0;

	if(type == Pos)
	{
		/* if mode change then map init */
		if (PosCmd_old != cmdCMD)
		{
			PosCmd_old = cmdCMD;
			MS_MapPosBlkInit();
		}

		/* position control is not needed, it will return.*/
		if (cmdCMD < LC_POS_CTRL)
		{
			return;
		}
		else if (cmdCMD > LC_POS_CTRL)
		{
			posCMD = cmdPOS;
		}
		else
		{
			posCMD = pdoPOS;
		}
	}
	else if(type == Spd)
	{
		/* if mode change then map init */
		if (SpdCmd_old != cmdCMD)
		{
			SpdCmd_old = cmdCMD;
			MS_MapSpdBlkInit();
		}

		/* speed control is not needed, it will return. */
		if (cmdCMD < LC_SPD_CTRL)
		{
			return;
		}
		else if (cmdCMD > LC_POS_CTRL)
		{
			spdCMD = posOUT;
		}
		else if (cmdCMD > LC_SPD_CTRL)
		{
			spdCMD = pdoVEL + posOUT;
		}
		else
		{
			spdCMD = pdoVEL;
		}
	}
	else if(type == Amp)
	{
		if (++AmpCmdCnt >= 20)
		{
			AmpCmdCnt = 0;
			/* if mode change then map init */
			if (AmpCmd_old != cmdCMD)
			{
				AmpCmd_old = cmdCMD;
				MS_MapAmpBlkInit();
			}

			/* current control is not needed, it will return.*/
			if (cmdCMD < LC_AMP_CTRL)
			{
				return;
			}
			else if (cmdCMD < LC_PP_CTRL)
			{
				cmdTRQ = pdoTRQ;
			}

			if (cmdCMD > LC_AMP_CTRL)
			{
				ampCMD = cmdTRQ + spdOUT;
			}
			else
			{
				ampCMD = cmdTRQ;
			}

			/* current Limit */
			if (ampCMD > ampLMT)
			{
				ampCMD = ampLMT;
			}

			if (ampCMD < -ampLMT)
			{
				ampCMD = -ampLMT;
			}
		}
	}

	*(pHandle->Error) = (*(pHandle->Cmd) - *(pHandle->Feedback)) / (*(pHandle->Division));

	/* Error Limit */
	if (*(pHandle->Error) > *(pHandle->ErrorLimit))
	{
		*(pHandle->Error) = *(pHandle->ErrorLimit);
	}

	if (*(pHandle->Error) < -*(pHandle->ErrorLimit))
	{
		*(pHandle->Error) = -*(pHandle->ErrorLimit);
	}

	*(pHandle->Integraler) += *(pHandle->Error);

	/* Integral Limit */
	if(type == Amp)
	{
		/* 20khz interrupt*/
		int32_t IntegralLimit;
		IntegralLimit = *(pHandle->IntegralLimit) * 20;

		if (*(pHandle->Integraler)> IntegralLimit)
		{
			*(pHandle->Integraler)= IntegralLimit;
		}
		if (*(pHandle->Integraler)<-IntegralLimit)
		{
			*(pHandle->Integraler)=-IntegralLimit;
		}
	}
	else
	{
		if (*(pHandle->Integraler) > *(pHandle->IntegralLimit))
		{
			*(pHandle->Integraler) = *(pHandle->IntegralLimit);
		}
		if (*(pHandle->Integraler) < -*(pHandle->IntegralLimit))
		{
			*(pHandle->Integraler) = -*(pHandle->IntegralLimit);
		}
	}


	/* PI controller calculation */
	*(pHandle->Output) = ((*(pHandle->Error) * *(pHandle->Kp) / (pHandle->KpDiv)) +
													(*(pHandle->Integraler) * *(pHandle->Ki) / (pHandle->KiDiv)));

	/* Output Limit */
	if (*(pHandle->Output) > *(pHandle->OutputLimit))
	{
		*(pHandle->Output) = *(pHandle->OutputLimit);
	}
	if (*(pHandle->Output) < -*(pHandle->OutputLimit))
	{
		*(pHandle->Output) = -*(pHandle->OutputLimit);
	}
} /* End of MS_PiController() */

/* End of pid.c */
