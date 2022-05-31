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
#include "trajectory_ctrl.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */
int TC_SpeedCurveGenerator(TC_MODE_E mode);

/* LOCAL SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TC_SpeedCurveGenerator()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int TC_SpeedCurveGenerator(TC_MODE_E mode)
{
	int d, dst;
	static int dir, spd, vel, acc, dec, dec2;

	/* Start generator */
	if (mode == TCM_START)
	{
		acc = segMAX*1000/segACC;//Calculate acceleration
		dec = segMAX*1000/segDEC;//Calculate deceleration
		dec2 = dec*2;

		/* Calculate speed limitation */
		if (segVEL <= 0)
		{
			vel = segMAX*10;//Minimum speed
		}
		else if (segVEL > segMAX)
		{
			vel = segMAX*1000;//Maximum speed
		}
		else
		{
			vel = segVEL*1000;//Speed limitation equal user specified speed
		}

		if (segPOS > cmdPOS)
		{
			/* Forward rotating */
			dir = TCD_FORWARD;
			segSTS = TCS_ADJ_SPD;
		}
		else if (segPOS < cmdPOS)
		{
			/* Backward rotating */
			dir = TCD_BACKWARD;
			segSTS = TCS_ADJ_SPD;
		}
		else
		{
			/* Reach target position command */
			spd = 0;//Clear speed command
			segSTS = TCS_STOP;
		}
		cmdACC = 0;//Reset acceleration after planning
		return 0;
	}
	else if (mode == TCM_GET_SPD)
	{
		if (segSTS > TCS_DEC_SPD)
		{
			spd = 0;
		}
		return (spd/1000);
	}

	/* Skip process for unsed status */
	if (segSTS > TCS_DEC_SPD)
	{
		return 0;
	}

	/* TCM_RUN mode */
  dst = segPOS - cmdPOS;//Update the distance to target position
	d = cmdVEL*cmdVEL/dec2;//deceleration distance(d) = (cmdVEL^2)/(2*dec)

	switch (segSTS)
	{
	case TCS_ADJ_SPD:
		// Acceleration area check
		if ((dir*spd) >= 0 && (dir*spd) <= vel)
		{
			segSTS = TCS_ACC_SPD;
		}
		break;

	case TCS_ACC_SPD:
		// Deceleration area check
		if ((dir*dst) <= d)
		{
			segSTS = TCS_DEC_SPD;
			break;
		}

		// Constant speed area check
		if ((dir*cmdVEL) >= segVEL)
		{
			segSTS = TCS_CONST_SPD;
		}
		break;

	case TCS_CONST_SPD:
		// Deceleration area check
		if ((dir*dst) <= d)
		{
			segSTS = TCS_DEC_SPD;
		}
		break;

	case TCS_DEC_SPD:
		// Stop area check
		if (spd == 0)
		{
			segSTS = TCS_STOP;
		}
		break;
	}

	/* Speed T-curve generation */
	switch (segSTS)
	{
	case TCS_ADJ_SPD:
		if ((dir*spd) > vel)
		{
			spd = spd - (dir*dec);
			if ((dir*spd) < vel)
			{
				spd = (dir*vel);
			}
			return (dir*(-dec));
		}
		spd = spd + (dir*dec);
		if ((dir*spd) > 0)
		{
			spd = 0;
		}
		return (dir*dec);

	case TCS_ACC_SPD:
		/* Give acceleration at speed up area */
		spd = spd + (dir*acc);
		if ((dir*spd) > vel)
		{
			spd = (dir*vel);
		}
		return (dir*acc);

	case TCS_CONST_SPD:
		/* Give zero acceleration at constant speed area */
		return 0;

	case TCS_DEC_SPD:
		/* Give deceleration at speed down area */
		spd = spd - (dir*dec);
		if ((dir*spd) < 0)
		{
			spd = 0;
		}
		return (dir*(-dec));
	} /* End of speed T-curve generation */

	return 0;

} /* End of TC_SpeedCurveGenerator() */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TC_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void TC_Init(void)
{
	cmdPOS = iopANG;    //Set position command
	segPOS = iopANG;    //Set segment command
	segCMD = TCC_NO_CMD;//Reset command
	segSTS = TCS_STOP;  //Reset status
	MC_PRINT("TC_Init: cmdPOS=segPOS=iopANG=%d\r\n", segPOS);
} /* End of TC_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TC_Run()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void TC_Run(void)
{
	static int old = 0;

	/* Trigger reset event */
	if (old != cmdCMD)
  {
		old = cmdCMD;
		TC_Init();
	}

	/* Skip process with invalid status */
	if (cmdCMD < LC_PP_CTRL)
	{
		return;
	}

	/* Process command */
	if (segCMD > TCC_NO_CMD)
	{
		if (segCMD == TCC_ABS_POS)
		{
			segPOS = pdoPOS;
		}
		else if (segCMD == TCC_REL_POS)
		{
			segPOS = cmdPOS + pdoPOS;
		}
		else if (segCMD == TCC_BRAKE)
		{
			segPOS = cmdPOS;
		}
		segCMD = TCC_NO_CMD;//Clear command and indicate process has done.
		if (segPOS != cmdPOS)
		{
			segVEL = (pdoVEL < 0) ? (-pdoVEL):pdoVEL;
			if (segVEL)
			{
				TC_SpeedCurveGenerator(TCM_START);
			}
		}
	}

	if (segVEL)
	{
		/* Update acceleration, velocity and position command */
		cmdACC = TC_SpeedCurveGenerator(TCM_RUN);
		cmdVEL = TC_SpeedCurveGenerator(TCM_GET_SPD);
		cmdPOS += cmdVEL;
	}
	else
	{
		cmdPOS = segPOS;
	}
} /* End of TC_Run() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TC_Brake()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void TC_Brake(void)
{
	segCMD = TCC_BRAKE;
	segSTS = TCS_STOP;
} /* End of TC_Brake() */

/* End of trajectory_ctrl.c */
