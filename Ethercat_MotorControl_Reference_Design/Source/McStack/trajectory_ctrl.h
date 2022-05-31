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

#ifndef __TRAJECTORY_CTRL_H__
#define __TRAJECTORY_CTRL_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_cfg.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */
typedef enum
{
	TCM_START		= 0,
	TCM_RUN  		= 1,
	TCM_GET_SPD	= 2,	
} TC_MODE_E;

typedef enum
{
	TCC_NO_CMD		= 0,
	TCC_ABS_POS   = 1,
	TCC_REL_POS		= 2,	
	TCC_RESTART   = 3,	
	TCC_BRAKE    	= 9,
} TC_COMMAND_E;

typedef enum
{
	TCS_ADJ_SPD 	= 0,
	TCS_ACC_SPD   = 1,
	TCS_CONST_SPD = 2,	
	TCS_DEC_SPD   = 3,	
	TCS_STOP    	= 9,
} TC_STATE_E;

typedef enum
{
	TCD_FORWARD 	= 1,
	TCD_BACKWARD  = -1,
} TC_DIRECTION_E;

/* MACRO DECLARATIONS */

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
void TC_Init(void);  
void TC_Run(void);
void TC_Brake(void);

#endif /* __TRAJECTORY_CTRL_H__ */

/* End of trajectory_ctrl.h */

