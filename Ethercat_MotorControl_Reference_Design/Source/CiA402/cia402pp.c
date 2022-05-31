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
#include "AX58200_MotorControl.h"
#include "cia402pp.h"

/* NAMING CONSTANT DECLARATIONS */
/* MACRO DECLARATIONS */
/* TYPE DECLARATIONS */
/* GLOBAL VARIABLES DECLARATIONS */
CIA402PP_CTRL_OBJ CIA402PP_CtrlObj;

CIA402_FUNC_INTF CIA402PP_FuncIntf = {
	(void*)&CIA402PP_CtrlObj,
	CIA402PP_Init,
	CIA402PP_DeInit,
	CIA402PP_Run,
};

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402PP_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t CIA402PP_Init(void *pAxisObj)
{
	CiA402_AXIS_T *pAxis = pAxisObj;
	CIA402PP_CTRL_OBJ *pCtrl = pAxis->pFuncIntf->pCtrlObj;
	
	memset(pCtrl, 0 , sizeof(CIA402PP_CTRL_OBJ));
	return 0;
} /* End of CIA402PP_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402PP_DeInit()
 * Purpose:
 *   :
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t CIA402PP_DeInit(void *pAxisObj)
{

	return 0;
} /* End of CIA402PP_DeInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402PP_Run()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t CIA402PP_Run(void *pAxisObj)
{
	CiA402_AXIS_T *pAxis = pAxisObj;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);	
	
	MC_SetTargetPosition(*(pCiA402->pTargetPosition0x607A), *(pCiA402->pTargetVelocity0x60FF));	

	return 0;
} /* End of CIA402PP_Run() */


/* End of cia402pp.c */
