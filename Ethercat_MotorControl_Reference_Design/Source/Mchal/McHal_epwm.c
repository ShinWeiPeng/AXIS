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
#include "McHal_epwm.h"
#include "McHal_misc.h"
#include "McHal_eadc.h"
#include "McHal_qei.h"
#include "typedef.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

void MH_EPWMSetCntType(EPWM_T *epwm, uint32_t ch, uint32_t type)
{
    uint8_t i;

    for(i = 0; i < EPWM_CHANNEL_NUM; i++)
    {
        if(i == ch)
        {
            epwm->CTL1 &= ~(0x03 << (i * 2));
            epwm->CTL1 |= (type << (i * 2));
        }
    }
}

void MH_EpwmSetCntMode(EPWM_T *epwm, uint32_t ch, uint32_t mode)
{
    uint8_t i;

    for(i = 0; i < EPWM_CHANNEL_NUM; i++)
    {
        if(i == ch)
        {
            epwm->CTL1 &= ~(0x03 << (i + EPWM_CTL1_CNTMODE0_Pos));
            epwm->CTL1 |= (mode << (i + EPWM_CTL1_CNTMODE0_Pos));
        }
    }
}

void MH_EpwmSetOutMode(EPWM_T *epwm, uint32_t ch_mask, uint32_t mode)
{
    if(EPWM_CH_0_MASK & ch_mask)
    {
        epwm->CTL1 &= ~EPWM_CTL1_OUTMODE0_Msk;
        epwm->CTL1 |= (mode << EPWM_CTL1_OUTMODE0_Pos);
    }

    if(EPWM_CH_2_MASK & ch_mask)
    {
        epwm->CTL1 &= ~EPWM_CTL1_OUTMODE2_Msk;
        epwm->CTL1 |= (mode << EPWM_CTL1_OUTMODE2_Pos);
    }

    if(EPWM_CH_4_MASK & ch_mask)
    {
        epwm->CTL1 &= ~EPWM_CTL1_OUTMODE4_Msk;
        epwm->CTL1 |= (mode << EPWM_CTL1_OUTMODE4_Pos);
    }
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_EpwmRegInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MH_EpwmRegInit(MH_EpwmRegConfig_t *pHandle)
{
    uint32_t epwm_ch_mask = BIT(pHandle->EpwmChU) | BIT(pHandle->EpwmChV) | BIT(pHandle->EpwmChW);

    /* Enable EPWM module clock */
	MH_EpwmClkSource(pHandle->pInst, MC_ENABLE);

	/* EPWM pin configuration */
	MH_SetMFP(pHandle->pPortPwm, pHandle->PinPwmU | pHandle->PinPwmV | pHandle->PinPwmW, pHandle->MultiFuncValue);
	GPIO_SetMode(pHandle->pPortPwm, pHandle->PinPwmU | pHandle->PinPwmV | pHandle->PinPwmW, GPIO_MODE_OUTPUT);
	GPIO_SetSlewCtl(pHandle->pPortPwm, pHandle->PinPwmU | pHandle->PinPwmV | pHandle->PinPwmW, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(pHandle->pPortPwm, pHandle->PinPwmU | pHandle->PinPwmV | pHandle->PinPwmW, GPIO_PUSEL_PULL_UP);

	/* CTRLDx=1:center re-load, x = 0,2,4 */
	//pHandle->pInst->CTL0 = 0x00000015;
	EPWM_EnableLoadMode(pHandle->pInst, pHandle->EpwmChU, EPWM_LOAD_MODE_CENTER);
	EPWM_EnableLoadMode(pHandle->pInst, pHandle->EpwmChV, EPWM_LOAD_MODE_CENTER);
	EPWM_EnableLoadMode(pHandle->pInst, pHandle->EpwmChW, EPWM_LOAD_MODE_CENTER);

	/* OUTMODEx=0:independent mode
	   CNTMODEx=0:auto-reload mode
	   CNTTYPEx=10:up-down counter */
	//pHandle->pInst->CTL1 = 0x00000222;
	MH_EPWMSetCntType(pHandle->pInst, pHandle->EpwmChU, EPWM_UP_DOWN_COUNTER);
	MH_EpwmSetCntMode(pHandle->pInst, pHandle->EpwmChU, MH_EPWM_CNT_MODE_AUTO_RELOAD);

    MH_EPWMSetCntType(pHandle->pInst, pHandle->EpwmChV, EPWM_UP_DOWN_COUNTER);
    MH_EpwmSetCntMode(pHandle->pInst, pHandle->EpwmChV, MH_EPWM_CNT_MODE_AUTO_RELOAD);

    MH_EPWMSetCntType(pHandle->pInst, pHandle->EpwmChW, EPWM_UP_DOWN_COUNTER);
    MH_EpwmSetCntMode(pHandle->pInst, pHandle->EpwmChW, MH_EPWM_CNT_MODE_AUTO_RELOAD);

    MH_EpwmSetOutMode(pHandle->pInst, epwm_ch_mask, MH_EPWM_OUTPUT_MODE_INDEPENDENT);

	/* ZPCTLn=01:EPWM zero point output Low. */
	//pHandle->pInst->WGCTL0 = 0x00000111;

	/* CMPDCTLn=01:EPWM compare down point output Low.
	   CMPUCTLn=10:EPWM compare up point output High. */
	//pHandle->pInst->WGCTL1 = 0x01110222;
    EPWM_SET_OUTPUT_LEVEL(pHandle->pInst, epwm_ch_mask,
                            EPWM_OUTPUT_LOW,
                            EPWM_OUTPUT_HIGH,
                            EPWM_OUTPUT_NOTHING,
                            EPWM_OUTPUT_LOW);

	/* prescale=(1+1)=2 */
	//pHandle->pInst->CLKPSC[0] = 1;
	//pHandle->pInst->CLKPSC[1] = 1;
	//pHandle->pInst->CLKPSC[2] = 1;
	EPWM_SET_PRESCALER(pHandle->pInst, pHandle->EpwmChU, 1);
	EPWM_SET_PRESCALER(pHandle->pInst, pHandle->EpwmChV, 1);
	EPWM_SET_PRESCALER(pHandle->pInst, pHandle->EpwmChW, 1);

	/* Set PWM_U PERIOD, CMPDAT */
	//pHandle->pInst->PERIOD[0] = PWM_PERIOD;
	//pHandle->pInst->CMPDAT[0] = PWM_DUTY_MAX;
	EPWM_SET_CNR(pHandle->pInst, 0, PWM_PERIOD);
	EPWM_SET_CMR(pHandle->pInst, 0, PWM_DUTY_MAX);

	/* Set PWM_V PERIOD, CMPDAT */
	//pHandle->pInst->PERIOD[2] = PWM_PERIOD;
	//pHandle->pInst->CMPDAT[2] = PWM_DUTY_MAX;
    EPWM_SET_CNR(pHandle->pInst, 2, PWM_PERIOD);
    EPWM_SET_CMR(pHandle->pInst, 2, PWM_DUTY_MAX);

	/* Set PWM_W PERIOD, CMPDAT */
	//pHandle->pInst->PERIOD[4] = PWM_PERIOD;
	//pHandle->pInst->CMPDAT[4] = PWM_DUTY_MAX;
    EPWM_SET_CNR(pHandle->pInst, 4, PWM_PERIOD);
    EPWM_SET_CMR(pHandle->pInst, 4, PWM_DUTY_MAX);

	/* Set Clock Source Select */
	//pHandle->pInst->CLKSRC = 0x00000000;
	EPWM_SetClockSource(pHandle->pInst, pHandle->EpwmChU, EPWM_CLKSRC_EPWM_CLK);
    EPWM_SetClockSource(pHandle->pInst, pHandle->EpwmChV, EPWM_CLKSRC_EPWM_CLK);
    EPWM_SetClockSource(pHandle->pInst, pHandle->EpwmChW, EPWM_CLKSRC_EPWM_CLK);

	/* Set EPWM Counter Enable Register Ch0,Ch2,Ch4*/
	//pHandle->pInst->CNTEN = 0x00000015;
	EPWM_Start(pHandle->pInst, epwm_ch_mask);

	/* Set EPWM Output Enable Register Ch0,Ch2,Ch4*/
	//pHandle->pInst->POEN = 0x00000015;
	EPWM_EnableOutput(pHandle->pInst, epwm_ch_mask);

	/* Set EPWM Synchronous Start Control Register */
	//pHandle->pInst->SSCTL = 0x00000115;
	EPWM_ENABLE_TIMER_SYNC(pHandle->pInst, epwm_ch_mask, EPWM_SSCTL_SSRC_EPWM1);

	/* Set EPWM Synchronous Start Trigger Register */
	//pHandle->pInst->SSTRG = 1;
	EPWM_TRIGGER_SYNC_START(pHandle->pInst);

	/* Set EPWM Trigger EADC Source Select Register 0 */
	//pHandle->pInst->EADCTS0 = 0x00000081;
	EPWM_EnableADCTrigger(pHandle->pInst, pHandle->EpwmChU, EPWM_TRG_ADC_EVEN_PERIOD);

	/* Set EPWM Interrupt Enable Register 0 */
	//pHandle->pInst->INTEN0 = 0x00000001;
	EPWM_EnableZeroInt(pHandle->pInst, pHandle->EpwmChU);

	/* Set Priority High */
	NVIC_SetPriority(pHandle->IrqType, NVIC_EncodePriority(MC_INT_PRIORITY_GROUP, pHandle->PreemptPrio, pHandle->SubPrio));
	NVIC_EnableIRQ(pHandle->IrqType);

	return MCSTS_OK;
} /* End of MH_EpwmRegInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_EpwmRegDeInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_EpwmRegDeInit(MH_EpwmRegConfig_t *pHandle)
{

} /* End of MH_EpwmRegDeInit() */

/* End of McHal_epwm.c */
