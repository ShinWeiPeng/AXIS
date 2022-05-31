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
#include "McHal_qei.h"
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
 * Function Name: MH_QeiRegInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MH_QeiRegInit(MH_QeiRegConfig_t *pHandle)
{
	MH_QeiClkSource(pHandle->pInst, MC_ENABLE);

	/* Enable ABZ encoder pins */
	MH_SetMFP(pHandle->pPortIncEnc, pHandle->PinA | pHandle->PinB | pHandle->PinZ, pHandle->MultiFuncValue);
	GPIO_SetMode(pHandle->pPortIncEnc, pHandle->PinA | pHandle->PinB | pHandle->PinZ, GPIO_MODE_INPUT);
	GPIO_SetSlewCtl(pHandle->pPortIncEnc, pHandle->PinA | pHandle->PinB | pHandle->PinZ, GPIO_SLEWCTL_NORMAL);
	GPIO_SetPullCtl(pHandle->pPortIncEnc, pHandle->PinA | pHandle->PinB | pHandle->PinZ, GPIO_PUSEL_DISABLE);
	//Enable schmitt trigger input
	pHandle->pPortIncEnc->SMTEN |= (pHandle->PinA | pHandle->PinB | pHandle->PinZ);

	//pHandle->pInst->CTL = 0x22000072;
    QEI_ENABLE_INDEX_LATCH(pHandle->pInst);
    QEI_ENABLE_INPUT(pHandle->pInst, QEI_CTL_CHAEN_Msk | QEI_CTL_CHBEN_Msk | QEI_CTL_IDXEN_Msk);
    QEI_ENABLE_NOISE_FILTER(pHandle->pInst, QEI_CTL_NFCLKSEL_DIV4);
    QEI_SET_CNT_MODE(pHandle->pInst, QEI_CTL_X4_FREE_COUNTING_MODE);
    QEI_Start(pHandle->pInst);

	/* Enable wire control for differential/sinle-end wires */
	MH_SetMFP(pHandle->pPortWireCtrl, pHandle->PinWireCtrl, 0);
	GPIO_SetMode(pHandle->pPortWireCtrl, pHandle->PinWireCtrl, GPIO_MODE_OUTPUT);
	MH_WRITE_GPIO_PIN(pHandle->pPortWireCtrl, pHandle->PinWireCtrl, MC_DISABLE);

	return MCSTS_OK;
} /* End of MH_QeiRegInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_QeiRegDeInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_QeiRegDeInit(MH_QeiRegConfig_t *pHandle)
{

} /* End of MH_QeiRegDeInit() */

/* End of McHal_qei.c */
