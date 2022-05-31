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
#include "McHal_eadc.h"
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
 * Function Name: MH_AdcRegInit()
 * Purpose:
 * Params: v: Indicates the number of channels
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MH_AdcRegInit(MH_AdcRegConfig_t *pHandle, uint16_t v)
{
	uint16_t i;

	/* Enable EADC module clock */
	MH_EadcClkSource(pHandle->pInst, MC_ENABLE);

	/* Generate sample module flags */
	// pHandle->SampleModuleMask[i] |= (0x01<<i);

	/* EADC pin configuration */
	for (i = 0; i < v; i++)
	{
		MH_SetMFP(pHandle->pPort[i], pHandle->Channel[i], pHandle->MultiFuncValue);
		GPIO_SetMode(pHandle->pPort[i], pHandle->Channel[i], GPIO_MODE_INPUT);
		GPIO_DISABLE_DIGITAL_PATH(pHandle->pPort[i], pHandle->Channel[i]);
	}

	/* Set input mode as single-end and enable the A/D converter */
	EADC_Open(pHandle->pInst, EADC_CTL_DIFFEN_SINGLE_END | EADC_CTL_RESSEL_Msk);

	/* Configure EADC modules */
	for (i = 0; i < v; i++)
	{
		EADC_ConfigSampleModule(pHandle->pInst, pHandle->SampleModuleMask[i], EADC_PWM1TG0_TRIGGER, MH_BitToNum(pHandle->Channel[i]));
	}

	return MCSTS_OK;
} /* End of MH_AdcRegInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_AdcConv()
 * Purpose:
 * Params: v: Indicates the number of channels
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint16_t MH_AdcConv(MH_AdcRegConfig_t *pHandle, uint16_t v)
{
	uint16_t retVal,i;
	/* Clear the interrupt flag for safe */
	EADC_CLR_INT_FLAG(pHandle->pInst, EADC_STATUS2_ADIF0_Msk);

	/* Get converted data into application buffer */
	for (i = 0; i < v ; i++)
	{
		pHandle->ConvBuffer[i] = (uint16_t)(EADC_GET_CONV_DATA(pHandle->pInst, pHandle->SampleModuleMask[i]));
	}

	return retVal;
} /* End of MH_AdcConv() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_AdcRegDeInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_AdcRegDeInit(MH_AdcRegConfig_t *pHandle)
{
	/* Close ADC module */
	EADC_Close(pHandle->pInst);

	/* Disable EADC module clock */
	MH_EadcClkSource(pHandle->pInst, MC_DISABLE);

} /* End of MH_AdcRegDeInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_AdcStop()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_AdcStop(MH_AdcRegConfig_t *pHandle, uint16_t v)
{
	uint16_t i;
	/* Cancel pended conversion */
	for (i = 0; i < v ;i++)
	{
		EADC_STOP_CONV(pHandle->pInst, pHandle->SampleModuleMask[i]);
	}
} /* End of MH_AdcStop() */

/* End of McHal_eadc.c */
