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
#include "McHal_tim.h"
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
 * Function Name: MH_TimRegInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_TimRegInit(void)
{	 
	/* Set TimerX Module Clock */
	MH_TmrClkSource(TMRTASK1K_INSTANCE, MC_ENABLE);
	
	/* Disable Timer interrupt */
	TIMER_DisableInt(TMRTASK1K_INSTANCE);

	/* Clear Timer interrupt flag */
	TIMER_ClearIntFlag(TMRTASK1K_INSTANCE);

	/* Open Timer1 : 1kHz */
	TIMER_Open(TMRTASK1K_INSTANCE, TIMER_PERIODIC_MODE, 1000);

	/* Enable Timer1 interrupt */
	TIMER_EnableInt(TMRTASK1K_INSTANCE);

	/* Set Priority Middle */
	NVIC_SetPriority(TMRTASK1K_IRQ, NVIC_EncodePriority(MC_INT_PRIORITY_GROUP, TMRTASK1K_INT_PREEMPTPRIO, TMRTASK1K_INT_SUBPRIO));
	NVIC_EnableIRQ(TMRTASK1K_IRQ);

	/* Start Timer1 */
	TIMER_Start(TMRTASK1K_INSTANCE);
} /* End of MH_TimRegInit() */ 

/* End of McHal_tim.c */

