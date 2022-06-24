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
#include "mc_math.h"
#include "mc.h"
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
 * Function Name: MI_GetSIN(),Trigonometrical functions type definition
 * Purpose: This function calculation sine functions of the angle fed in input
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int MI_GetSIN(int d)
{
	/* SIN[deg] Table */
    //SIN[deg] * 256 = tab
	static int tab[]=
	{
		0,  4,  9, 13, 18, 22, 27, 31, 36, 40,
		44, 49, 53, 57, 62, 66, 71, 75, 79, 83,
		87, 92, 96,100,104,108,112,116,120,124,
		128,132,136,139,143,147,150,154,158,161,
		164,168,171,174,178,181,184,187,190,193,
		196,199,202,204,207,210,212,215,217,219,
		222,224,226,228,230,232,234,236,237,239,
		240,242,243,245,246,247,248,249,250,251,
		252,253,253,254,254,255,255,256,256,256,256
	};

	while (d > 180)
	{
		d -= 360;
	}

	while (d < -180)
	{
		d += 360;
	}

	if (d < -90)
	{
		d = -tab[180+d];
	}
	else if (d < 0)
	{
		d = -tab[-d];
	}
	else if (d < 90)
	{
		d = tab[d];
	}
	else
	{
		d = tab[180-d];
	}

	return(d);
} /* End of MI_GetSIN() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MI_Clake()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
MCMATH_AB_PHASE_T MI_Clake(void)
{
	MCMATH_AB_PHASE_T phase_ab;
	int u, v;

	u = adcIU;
	v = adcIV;
	if (pwmINV)
	{
		v = adcIW;
	}

	/* Clark transfer */
	phase_ab.Alpha = u;

	/* 591 = (1/sqrt(3))* 1024 */
	phase_ab.Beta = ((u + v + v) * 591) >> 10;
	return phase_ab;
} /* End of MI_Clake() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MI_Park()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MI_Park(MCMATH_AB_PHASE_T *pPhaseAB)
{
	int d_val, q_val, d_filtered, q_filtered;
	static int d_sum = 0, q_sum = 0;

	/* Calculate mapped value on axis-D/Q */
	d_val = ((pPhaseAB->Alpha * uvwCOS) + (pPhaseAB->Beta * uvwSIN)) >> 8;
	q_val = ((pPhaseAB->Beta * uvwCOS) - (pPhaseAB->Alpha * uvwSIN)) >> 8;

	/* Do filtering on axis-D/Q */
	d_filtered = d_sum / ampFLT;
	d_sum += (d_val - d_filtered);
	q_filtered = q_sum / ampFLT;
	q_sum += (q_val - q_filtered);

	/* Output D/Q result */
	iopID = (d_filtered * adcK0) / adcSTD;
	iopIQ = (q_filtered * adcK0) / adcSTD;

} /* End of MI_Park() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MI_RevPark()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
#define MC_LAMBDA 0.00811523
#define MC_KE 0.03975637
#define MC_LQ 0.0047304
#define MC_LD 0.0046008
MCMATH_AB_PHASE_T MI_RevPark(void)
{
	MCMATH_DQ_PHASE_T phase_dq;
	MCMATH_AB_PHASE_T phase_ab;
	static int d_sum = 0, q_sum = 0;
	int wr, decouple_q, decouple_d;

	phase_ab.Alpha = 0;
	phase_ab.Beta = 0;

	/* If vector has disabled,  */
	if (uvwRDY == SCM_DISABLE)
	{
		return phase_ab;
	}

	if (uvwRDY == SCM_AUTO_CTRL)
	{
		/*
			If under auto vector control mode,
			the magnetic field shall follows rotor angle
		*/
		pwmDEG = uvwDEG + ((iopSPD * uvwKO) / 4096);
	}

	wr = iopANG * 0.1 * 0.01 * 4 * angPOLE / angREL * 2 * 3.14159 * 20000 / 1000;

	/* Do filtering on axis-D/Q */
	decouple_d = ampCMD * MC_LQ * wr;
	phase_dq.Direct = d_sum / pwmFLT - decouple_d;
	d_sum += (iopVD - phase_dq.Direct);

	decouple_q = wr * MC_LAMBDA;
	phase_dq.Quadrature = q_sum / pwmFLT + decouple_q;
	q_sum += (iopVQ - phase_dq.Quadrature);

	/* Calculate mapped value on axis-A/B */
	uvwSIN = MI_GetSIN(pwmDEG);
	uvwCOS = MI_GetSIN(pwmDEG + 90);
	phase_ab.Alpha = ((phase_dq.Direct * uvwCOS) - (phase_dq.Quadrature * uvwSIN)) >> 8;
	phase_ab.Beta = ((phase_dq.Quadrature * uvwCOS) + (phase_dq.Direct * uvwSIN)) >> 8;

	return phase_ab;
} /* End of MI_RevPark() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MI_MapUvwBlkInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MI_MapUvwBlkInit(void)
{
	uvwDEG = 0;
	uvwRDY = SCM_DISABLE;//Disable vector output
	uvwKI = 0;
	uvwKO = 0;
	uvwDEG0 = 0;
} /* End of MI_MapUvwBlkInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MI_CalcRotorVector()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MI_CalcRotorVector(void)
{
	int delta_ang;

	/* Calculate feedback delta angle at mechanic domain */
	delta_ang = iopREL - iopIDX;

	/* Find the nearest index(Z) point position */
	if (delta_ang > angREL)
	{
		iopIDX += angREL;
	}

	if (delta_ang < -angREL)
	{
		iopIDX -= angREL;
	}

	/* Inverse feedback delta angle polarity if need */
	if (angINV & 0x01)
	{
		delta_ang = -delta_ang;
	}

	/* Convert feedback delta angle to vector domain */
	//pulse -> mechanical angle -> electrical angle
	delta_ang = delta_ang * 360 * angPOLE / angREL;

	delta_ang -= uvwDEG0;//Calculate rotor angle

	while (delta_ang < 0)
	{
		delta_ang += 360;
	}

	while (delta_ang > 360)
	{
	    delta_ang -= 360;
	}

	/* Get rotor angle */
	uvwDEG = delta_ang + ((iopSPD * uvwKI) / 4096);
	uvwSIN = MI_GetSIN(uvwDEG);
	uvwCOS = MI_GetSIN(uvwDEG + 90);
} /* End of MI_CalcRotorVector() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MI_MapAngBlkInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MI_MapAngBlkInit(void)
{
	angABS = 0x20000;
	angPOLE = 4;
	angGEAR = 101;
	angREL = 102400;
	angKN = 400;
	angKD = 512;
	angINV = 0x02;
	angABS0 = 0;
	angREL0 = 0;
	angIDX0 = 0;
	spdFLT = 4;
} /* End of MI_MapAngBlkInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MI_GetEncAngle_AbzMode()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MI_GetEncAngle_AbzMode(void)
{
	int cur_ang, delta_ang;
	static int rst = 1, pre_ang = 0;

	/* Reset feedback angle */
	if (rst || iopRST)
	{
		rst = 0;
		pre_ang = iopREL;

		if (iopMOD > 1)
		{
			iopANG = 0;
		}

		if (iopMOD)
		{
			return;
		}
	}

	/* Calculate feedback angle */
	cur_ang = iopREL;
	delta_ang = (cur_ang - pre_ang) & 0x7fff;
	pre_ang = cur_ang;

	if (delta_ang & 0x4000)
	{
	    delta_ang -= 0x8000;
	}

	if (angINV & 0x01)
	{
		iopANG -= delta_ang;
	}
	else
	{
		iopANG += delta_ang;
	}
} /* End of MI_GetEncAngle_AbzMode() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MI_CalcEncAngle()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MI_CalcEncAngle(uint32_t ResetAngle)
{
	int cur_ang, delta_ang, spd;
	static int gain = 0, div_cnt = 0, pre_ang = 0, spd_sum = 0;

	/* Get feedback angle */
	MI_GetEncAngle_AbzMode();

	/* Calculate gain value to amplify feedback angle and promote speed resolution */
	if (!gain || iopRST || ResetAngle)
	{
		gain = 0x8000 / angREL;
		if (gain < 2)
		{
			gain = 2;
		}

		if (gain > 8)
		{
			gain = 8;
		}

		div_cnt = 0;
		spd_sum = 0;
		pre_ang = iopANG * gain;
	}

	/* Adjust execution freq = 20k/10 = 2kHz */
	if ((++div_cnt) < 10)
	{
        return;
	}

	div_cnt = 0;

	/* Calculate speed feedback */
	cur_ang = iopANG * gain;
	delta_ang = cur_ang - pre_ang;
	pre_ang = cur_ang;

	/* Speed low pass filter */
	spd = spd_sum / spdFLT;	//Bandwidth = 2k/spdFLT (Hz)
	spd_sum += (delta_ang - spd);

	/* Get speed feedback */
	iopSPD = spd;
} /* End of MI_CalcEncAngle() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MI_FocCurrController()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MI_FocCurrController(void)
{
	static int rst = 0;

	if (iopRST)
	{
		rst = 1;//**iopRST enable
	}

	if (powerON == 0)
	{
		powerON = 1;//**20khz Start up
	}

	if (powerON > 100)
	{
		MI_CalcEncAngle(0);//**After 100ms: Angle and speed feedback
	}

	if (powerON > 200)
	{
		MI_CalcRotorVector();//**After 200ms: Vector feedback
	}
	if (powerON > 700)//**After 700ms: Current sensor feedback
	{
		MS_GetSignedCurr_d(&CurrentParamsM1.CurrInfo);
		CurrentParamsM1.Iab =  MI_Clake();
		MI_Park(&(CurrentParamsM1.Iab));
	}

	if (powerON > 900)//**After 900ms:Current loop PI controller start up
	{
		if(cmdCMD >= LC_AMP_CTRL)
		{
			MS_PiController(&AmpParamsM1.PiInfo,Amp);
			MS_PiController(&AmpParamsM2.PiInfo,Amp);
		}
	}

	if (powerON > 900)//**After 900ms:Vector control
	{
		PwmParamsM1.Vab = MI_RevPark();
		MS_SetPhaseVoltage(&PwmParamsM1.PwmInfo,PwmParamsM1.Vab);
	}

	if (rst)//**iopRST disable
	{
		rst = 0;
		iopRST = 0;
	}
} /* End of MI_FocCurrController() */

/* End of mc_math.c */

