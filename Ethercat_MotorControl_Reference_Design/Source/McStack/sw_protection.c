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
#include "sw_protection.h"
#include "McHal_misc.h"
#include "mc.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_MapProtectBlkInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MS_MapProtectBlkInit(void)
{
	prtOVP = 30;
	prtUVP = 10;
	prtOTP = 85;
	prtOCP = 2500;
	prtUBC = 200;
	ovpFLT = 5;
	otpFLT = 5;
	i2cFLT = 5;
	ovpCNT = 0;
	uvpCNT = 0;
	otpCNT = 0;

} /* End of MS_MapProtectBlkInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_GetAvBusVoltage_V()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MS_GetAvBusVoltage_V(MS_SwProt_Handle_t *pHandle)
{
	static int VbusSum = 0;
	iopVBUS = MS_GetBusVoltage_V(pHandle->pVbusInfo);

	/* Low pass filter */
	iopPWR = VbusSum >> ovpFLT;
	VbusSum += (iopVBUS - iopPWR);
} /* End of MS_GetAvBusVoltage_V() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_GetAvTemp_V()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MS_GetAvTemp_V(MS_SwProt_Handle_t *pHandle)
{
	static int TempSum=0;

	/* 1°C= 10mV, 0°C=500mV */
	iopTEMP = MS_GetTemp_V(pHandle->pTempInfo);

	/* Low pass filter */
	iopTMP = TempSum >> otpFLT;
	TempSum += (iopTEMP - iopTMP);
} /* End of MS_GetAvTemp_V() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_GetAvI2T_V()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MS_GetAvI2T_V(MS_SwProt_Handle_t *pHandle)
{
	int realIq;
	static int realIqSum = 0;
	static uint32_t CurrLmt = 0;
	static uint32_t CurrLmt2 = 0;
	realIq = iopIQ;
	if (realIq < 0)
	{
		realIq =- realIq;
	}
	/* Low pass filter(N=8192) */
	iopI2T = realIqSum >> i2cFLT;
	realIqSum += (realIq - iopI2T);
	realIq = 3000 - iopI2T;

	/* Temperature<50°C: Current limit=200% */
	if (realIq > 2000 || iopTMP < 50)
	{
		if(++CurrLmt > 1000)
		{
			realIq = 2000;
			CurrLmt = 0;
		}
	}
	else
	{
		CurrLmt = 0;
	}

	/* Temperature>70°C: Current limit=100%*/
	if (realIq < 1000 || iopTMP > 70)
	{
		if(++CurrLmt2 > 1000)
		{
			realIq = 1000;
			CurrLmt2 = 0;
		}
	}
	else
	{
		CurrLmt2 = 0;
	}

	/* Set Current limit(100%~200%)*/
	ampLMT = realIq;
} /* End of MS_GetAvI2T_V() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: SWP_Run()
 * Purpose:
 * Params:
 * Returns:
 * Note: iopBlk : Servo ON/OFF and Alarm handle
 *       iopALM=0: ok
 *       iopALM=1: Under voltage(<10V)
 *       iopALM=2: Over voltage(>30V)
 *       iopALM=3: Over temperature(>85°C)
 *       iopALM=4: Current damage
 *       iopALM=5: Over Current Id(>250%)
 *       iopALM=6: Over Current Iq(>250%)
 * ----------------------------------------------------------------------------
 */
void SWP_Run(MS_SwProt_Handle_t *pHandle)
{
	int curr;
	static int t0 = 0, on = 0;
	static uint32_t curr_count = 0;
	static uint32_t curr_count2 = 0;
	static uint32_t curr_count3 = 0;

	/* Delay 1ms */
	if (MH_Delay_1ms(t0) == 0)
	{
		return;
	}
	t0 = MH_ClkTick_1ms();

	MS_GetAvBusVoltage_V(pHandle);
	MS_GetAvTemp_V(pHandle);
	MS_GetAvI2T_V(pHandle);

	/* Servo On */
	if (cmdON && !on)
	{
		on = 1;
		iopALM = SAT_NO_ERR;
		pwmENB = 1;
		pwmBRK = 1;
	}

	/* Servo Off */
	if (!cmdON && on)
	{
		on = 0;
		pwmENB = 0;
		pwmBRK = 0;
	}

	if (!cmdON)
	{
		return;
	}
#ifdef MC_SW_PROTECT_ENABLE
	/* Over Voltage */
	if (iopPWR > prtOVP)
	{
		if ((++ovpCNT) >= 300)
		{
			cmdON = MC_DISABLE;
			iopALM = SAT_OVER_VOLT;
			ovpCNT = 0;
			return;
		}
	}
	else
	{
		ovpCNT = 0;
	}

	/* Under Voltage */
	if((iopPWR < prtUVP) && (iopPWR > 3))
	{
		if ((++ovpCNT) >= 300)
		{
			cmdON = MC_DISABLE;
			iopALM = SAT_UNDER_VOLT;
			uvpCNT = 0;
			return;
		}
	}
	else
	{
		uvpCNT = 0;
	}

	/* Current damage */
	curr = adcSUM - 2048;
	if ((curr > prtUBC) || (curr < (-prtUBC)))
	{
		if(++curr_count > 10)
		{
			cmdON = MC_DISABLE;
			iopALM = SAT_PHASE_ERR;
			curr_count = 0;
			return;
		}
	}
	else
	{
		curr_count = 0;
	}
	/* Over Iq current */
	curr = iopIQ;
	if ((curr > prtOCP) || (curr < (-prtOCP)))
	{
		if(++curr_count2 > 250)
		{
			cmdON = MC_DISABLE;
			iopALM = SAT_OVER_CURR_IQ;
			curr_count2 = 0;
			return;
		}
	}
	else
	{
		curr_count2=0;
	}

	/* Over Id current */
	curr = iopID;
	if ((curr > prtOCP) || (curr < (-prtOCP)))
	{
		if(++curr_count3 > 250)
		{
			cmdON = MC_DISABLE;
			iopALM = SAT_OVER_CURR_ID;
			curr_count3 = 0;
			return;
		}
	}
	else
	{
		curr_count3 = 0;
	}

	/* Over Temperature */
	if (iopTMP > prtOTP)
	{
		if((++otpCNT) >= 1000)
		{
			cmdON = MC_DISABLE;
			iopALM = SAT_OVER_TEMP;
			otpCNT = 0;
			return;
		}
	}
	else
	{
		otpCNT = 0;
	}
#endif
} /* End of SWP_Run() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: SWP_DriveCtrl()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void SWP_DriveCtrl(MS_SwProt_Handle_t *pHandle)
{
	static int rst=0,sts=0;
	sts++;

	/* rst count pluse one at 20Khz */
	if (rst < 99999)
	{
		rst++;
	}

	/* 20kHz*500ms=10000 */
	if (rst < 10000)
	{
		return;
	}

	MH_WRITE_GPIO_PIN(PWMCTL_PORT_TRIGGER, PWMCTL_PIN_TRIGGER,(sts&0x01)?MC_ENABLE:MC_DISABLE);//Clear PWM watchdog timer@10kHz
	MH_WRITE_GPIO_PIN(PWMCTL_PORT_ENABLE, PWMCTL_PIN_ENABLE,(pwmENB)?MC_ENABLE:MC_DISABLE);//Enable/Disable MOSFET gate driver
	MH_WRITE_GPIO_PIN(PWMCTL_PORT_BRAKE, PWMCTL_PIN_BRAKE, (pwmBRK)?MC_ENABLE:MC_DISABLE);//Lock/Unlock brake

} /* End of SWP_DriveCtrl() */

/* End of sw_protection.c */
