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
#include "cia402hm.h"

/* NAMING CONSTANT DECLARATIONS */
/* MACRO DECLARATIONS */
/* TYPE DECLARATIONS */
/* GLOBAL VARIABLES DECLARATIONS */
CIA402HM_CTRL_OBJ CIA402HM_CtrlObj;

CIA402_FUNC_INTF CIA402HM_FuncIntf = {
	(void*)&CIA402HM_CtrlObj,
	CIA402HM_Init,
	CIA402HM_DeInit,
	CIA402HM_Run,
};

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402HM_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t CIA402HM_Init(void *pAxisObj)
{
	CiA402_AXIS_T *pAxis = pAxisObj;
	CIA402HM_CTRL_OBJ *pCtrl = pAxis->pFuncIntf->pCtrlObj;
	
	memset(pCtrl, 0 , sizeof(CIA402HM_CTRL_OBJ));
	return 0;
} /* End of CIA402HM_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402HM_DeInit()
 * Purpose:
 *   :
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t CIA402HM_DeInit(void *pAxisObj)
{

	return 0;
} /* End of CIA402HM_DeInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402HM_Run()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t CIA402HM_Run(void *pAxisObj)
{
	CiA402_AXIS_T *pAxis = pAxisObj;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);	
	CIA402HM_CTRL_OBJ *pCtrl = pAxis->pFuncIntf->pCtrlObj;	
	CIA402HM_CONTROLWORD_OBJ *pControlWord = (CIA402HM_CONTROLWORD_OBJ*)(pCiA402->pControlword0x6040);	
	CIA402HM_STATUSWORD_OBJ  *pStatusWord = (CIA402HM_STATUSWORD_OBJ*)(pCiA402->pStatusword0x6041);
	static uint16_t curTime=0;
	static uint32_t timeOut=0;
	
	switch (pCtrl->State)
	{
	case CIA402HM_IDLE:
	default:
		pStatusWord->b.homing_attained = 1;
		if (pControlWord->b.homing_op_start == 1)
		{	
			pCtrl->State = CIA402HM_START;
		}
		break;
	
	case CIA402HM_START:		
		/* Start homing */		
		pStatusWord->b.target_reached = 0;
		pStatusWord->b.internal_limit_active = 0;		
		pStatusWord->b.homing_attained = 0;
		pStatusWord->b.homing_error = 0;

		if (CIA402_CheckEmulationEnable() == CIA402_EMUL_DISABLED)
		{
			if (pCiA402->pHomingSpeed0x6099->SpeedDuringSearchForZero <= 0)
			{
				pCiA402->pHomingSpeed0x6099->SpeedDuringSearchForZero = 1;
			}
			MC_StartHoming(pCiA402->pHomingSpeed0x6099->SpeedDuringSearchForZero);
		}
		
		pCtrl->State = CIA402HM_RUN;
		DBG_PRINT("Start homing\r\n");			
		
		curTime = HW_GetTimer();								
		break;

	case CIA402HM_RUN:
		/* Active homing */
		if (CIA402_CheckEmulationEnable())
		{
			if (HW_CheckTimeout(curTime, 4000))
			{
				MC_ClearActualPosition();
			
				/* Homing successfully completed */
				pStatusWord->b.target_reached = 1;
				
				*(pCiA402->pPositionActualValue0x6064) = 0;
				
				pCtrl->State = CIA402HM_DONE;
				DBG_PRINT("Homing successfully done\r\n");						
			}
		}
		else if (MC_CheckHomingDone())
		{
			/* Run offset */
			if (*(pCiA402->pHomeOffset0x607C))
			{
				int32_t pos;
				
				if (pCiA402->pHomingSpeed0x6099->SpeedDuringSearchForZero <= 0)
				{
					pCiA402->pHomingSpeed0x6099->SpeedDuringSearchForZero = 1;
				}
				pos = *(pCiA402->pPositionActualValue0x6064)+*(pCiA402->pHomeOffset0x607C);
				MC_SetTargetPosition(pos, pCiA402->pHomingSpeed0x6099->SpeedDuringSearchForZero);
				timeOut = (pos*1000)/pCiA402->pHomingSpeed0x6099->SpeedDuringSearchForZero;
				curTime = HW_GetTimer();
				
				pCtrl->State = CIA402HM_RUN_OFFSET;
			}
			else
			{
				/* Homing successfully completed */
				pStatusWord->b.target_reached = 1;

				pCtrl->State = CIA402HM_DONE;
				DBG_PRINT("Homing successfully done\r\n");				
			}
		}
		else if (HW_CheckTimeout(curTime, 5000))
		{
			pStatusWord->b.target_reached = 0;			
			pStatusWord->b.homing_error = 1;
			
			/* Report error */
			CiA402_LocalError(ERROR_INCREMENTAL_SENSOR_1_FAULT);
			
			pCtrl->State = CIA402HM_DONE;			
			DBG_PRINT("Homing timeout\r\n");
		}
		else if (pControlWord->b.homing_op_start == 0)
		{
			pStatusWord->b.target_reached = 0;		
			
			pCtrl->State = CIA402HM_DONE;			
			DBG_PRINT("Terminate homing\r\n");			
		}
		break;
		
	case CIA402HM_RUN_OFFSET:
		if (HW_CheckTimeout(curTime, timeOut))
		{
			MC_ClearActualPosition();
			
			pStatusWord->b.target_reached = 1;			
			pCtrl->State = CIA402HM_DONE;
			DBG_PRINT("Home offset done\r\n");					
		}
		break;
	
	case CIA402HM_DONE:
		timeOut = 1000;		
		curTime = HW_GetTimer();		
		pCtrl->State = CIA402HM_WAIT;	
		break;
		
	case CIA402HM_WAIT:
		if (HW_CheckTimeout(curTime, timeOut))
		{
			pStatusWord->b.homing_attained = 1;
			pCtrl->State = CIA402HM_STOP;				
		}
		break;
	
	case CIA402HM_STOP:
		/* Wait homing_op_start bit to be down */
		if (pControlWord->b.homing_op_start == 0)
		{
			pCtrl->State = CIA402HM_IDLE;		
			DBG_PRINT("Stop homing\r\n");						
		}
		break;
	}
	return 0;
} /* End of CIA402HM_Run() */


/* End of cia402hm.c */
