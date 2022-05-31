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

#ifndef __MCHAL_EPWM_H__
#define __MCHAL_EPWM_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_cfg.h"

/* NAMING CONSTANT DECLARATIONS */
#define PWM_PERIOD 		2400
#define PWM_DUTY_MAX 	1200

/* TYPE DECLARATIONS */
typedef struct {
	EPWM_T    *pInst;
	IRQn_Type  IrqType;
	uint32_t   PreemptPrio;
	uint32_t   SubPrio;
	GPIO_T    *pPortPwm;
	uint32_t   PinPwmU;
	uint32_t   PinPwmV;
	uint32_t   PinPwmW;
	uint8_t    MultiFuncValue;
	uint32_t   EpwmChU;
	uint32_t   EpwmChV;
	uint32_t   EpwmChW;
} MH_EpwmRegConfig_t;

/* MACRO DECLARATIONS */
#define MH_SET_PWMU_DUTY(pInst, v)       (pInst->CMPDAT[0] = v)
#define MH_SET_PWMV_DUTY(pInst, v)       (pInst->CMPDAT[2] = v)
#define MH_SET_PWMW_DUTY(pInst, v)       (pInst->CMPDAT[4] = v)

#define MH_EPWM_CNT_MODE_AUTO_RELOAD    0
#define MH_EPWM_CNT_MODE_ONE_SHOT       1

#define MH_EPWM_OUTPUT_MODE_INDEPENDENT     0
#define MH_EPWM_OUTPUT_MODE_COMPLEMENTARY   1

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
int32_t MH_EpwmRegInit(MH_EpwmRegConfig_t *pHandle);
void MH_EpwmRegDeInit(MH_EpwmRegConfig_t *pHandle);
void MH_EpwmPutU(MH_EpwmRegConfig_t *pHandle , uint32_t value);
void MH_EpwmPutV(MH_EpwmRegConfig_t *pHandle , uint32_t value);
void MH_EpwmPutW(MH_EpwmRegConfig_t *pHandle , uint32_t value);
#endif /* __MCHAL_EPWM_H__ */
/* End of McHal_epwm.h */

