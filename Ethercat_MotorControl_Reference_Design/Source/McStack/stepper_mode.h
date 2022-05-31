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

#ifndef __STEPPER_MODE_H__
#define __STEPPER_MODE_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_cfg.h"

/* NAMING CONSTANT DECLARATIONS */
#define STPM_MAX_TICK_CNTS									3
#define STPM_VECTOR_ALIGN_TIMEOUT_MS				3000
#define STPM_ROTOR_STABLE_CHECK_GAPTIME_MS	10

/* TYPE DECLARATIONS */
typedef enum
{
	STPM_IDLE		        = 0,
	STPM_START			    = 1,
	STPM_FORWARD		    = 10,
	STPM_BACKWARD		    = 20,
	STPM_ZERO_VECTOR	    = 30,
	STPM_DONE			    = 99,
} STPM_STATE_E;

typedef enum
{
	STPMVA_IDLE = 0,
	STPMVA_SETUP,
	STPMVA_START,
	STPMVA_WAIT_ALIGN,
	STPMVA_DONE,
	STPMVA_FAIL,
} STPM_VECALI_STATE_E;

typedef struct
{
	/* Step mode for servo tuning */
	STPM_STATE_E   			State;
	volatile uint32_t       TickMs[STPM_MAX_TICK_CNTS];
	int32_t				    IsSingleTurn;
	int32_t				    StpAng;

	/* For vector alignment */
	STPM_VECALI_STATE_E	    VecAliState;
	int32_t 			    RotorAngT[2];
	int32_t 			    RotorStableCnt;
} STPM_CTRL_T;

/* MACRO DECLARATIONS */

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
MC_STATUS_E STPM_Init(STPM_CTRL_T *pStpMod);
void STPM_TickRun(STPM_CTRL_T *pStpMod);
STPM_STATE_E STPM_RunStepping(STPM_CTRL_T *pStpMod);
STPM_VECALI_STATE_E STPM_VectorAlignment(STPM_CTRL_T *pStpMod);

#endif /* __STEPPER_MODE_H__ */

/* End of stepper_mode.h */
