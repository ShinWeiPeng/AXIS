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
 
/**
\addtogroup AX58200 MotorControl implementation file
@{
*/

/**
\file AX58200_MotorControl.c
\brief Implementation

\version 1.0.0.0
*/


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecat_def.h"

#include "applInterface.h"

#define _AX58200__MOTOR_CONTROL_ 1
#include "AX58200_MotorControl.h"
#include "cia402hm.h"
#include "cia402csp.h"
#include "cia402pp.h"
#undef _AX58200__MOTOR_CONTROL_

/*-----------------------------------------------------------------------------------------
------
------    global variables and constants
------
-----------------------------------------------------------------------------------------*/
extern TOBJECT OBJMEM ApplicationObjDic[];
CiA402_AXIS_T  CiA402Axis;

/*--------------------------------------------------------------------------------------
------
------    local types and defines
------
--------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/
void CiA402_Application(CiA402_AXIS_T *pAxis);
int  CiA402_MotionControl(CiA402_AXIS_T *pAxis);
void CiA402_LocalError(UINT16 ErrorCode);
BOOL CiA402_TransitionAction(INT16 Characteristic, CiA402_AXIS_T *pAxis);
/*
 * ----------------------------------------------------------------------------
 * Function Name: CiA402_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void CiA402_Init(void)
{
	CiA402_AXIS_T  *pAxis = &CiA402Axis;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);
	
	/* Initialize axis data structure */
	HMEMSET(pAxis, 0, SIZEOF(CiA402_AXIS_T));
	
	pAxis->bAxisIsActive = FALSE;
	pAxis->bBrakeApplied = TRUE;
	pAxis->bLowLevelPowerApplied = TRUE;
	pAxis->bHighLevelPowerApplied = FALSE;
	pAxis->bAxisFunctionEnabled = FALSE;
	pAxis->bConfigurationAllowed = TRUE;
	
	pAxis->i16State = STATE_NOT_READY_TO_SWITCH_ON;
	pAxis->u16PendingOptionCode = 0x00;
	
	/* Binding object dictionary into axis data structure */
	pAxis->pObjDic = ApplicationObjDic;

#ifdef MC_STACK_ENABLE
	pAxis->flags.b.axis_is_act = (pAxis->bAxisIsActive == FALSE) ? 0:1;
	pAxis->flags.b.brake_applied = (pAxis->bBrakeApplied == FALSE) ? 0:1;
	pAxis->flags.b.low_pwr_applied = (pAxis->bLowLevelPowerApplied == FALSE) ? 0:1;
	pAxis->flags.b.high_pwr_applied = (pAxis->bHighLevelPowerApplied == FALSE) ? 0:1;
	pAxis->flags.b.axis_func_enb = (pAxis->bAxisFunctionEnabled == FALSE) ? 0:1;
	pAxis->flags.b.cfg_allowed = (pAxis->bConfigurationAllowed == FALSE) ? 0:1;

#endif

	/* Link CiA402 variables */
	pCiA402->pRxPdoMappingCspCsv0x1600 = &RxPdoMappingCspCsv0x1600;
	pCiA402->pRxPdoMappingCsp0x1601 = &RxPdoMappingCsp0x1601;
	pCiA402->pRxPdoMappingCsv0x1602 = &RxPdoMappingCsv0x1602;
	pCiA402->pTxPdoMappingCspCsv0x1A00 = &TxPdoMappingCspCsv0x1A00;
	pCiA402->pTxPdoMappingCsp0x1A01 = &TxPdoMappingCsp0x1A01;
	pCiA402->pTxPdoMappingCsv0x1A02 = &TxPdoMappingCsv0x1A02;	
	pCiA402->psRxPDOassign = &sRxPDOassign;	
	pCiA402->psTxPDOassign = &sTxPDOassign;	
	pCiA402->pErrorCode0x603F = &ErrorCode0x603F;
	pCiA402->pControlword0x6040 = &Controlword0x6040;
	pCiA402->pStatusword0x6041 = &Statusword0x6041;
	pCiA402->pVlRampFunctionTime0x604F = &VlRampFunctionTime0x604F;
	pCiA402->pQuickStopOptionCode0x605A = &QuickStopOptionCode0x605A;	
	pCiA402->pShutdownOptionCode0x605B = &ShutdownOptionCode0x605B;
	pCiA402->pDisableOperationOptionCode0x605C = &DisableOperationOptionCode0x605C;
	pCiA402->pFaultReactionOptionCode0x605E = &FaultReactionOptionCode0x605E;
	pCiA402->pModesOfOperation0x6060 = &ModesOfOperation0x6060;
	pCiA402->pModesOfOperationDisplay0x6061 = &ModesOfOperationDisplay0x6061;
	pCiA402->pPositionActualValue0x6064 = &PositionActualValue0x6064;
	pCiA402->pVelocityActualValue0x606C = &VelocityActualValue0x606C;
	pCiA402->pTorqueActualValue0x6077 = &TorqueActualValue0x6077;
	pCiA402->pTargetPosition0x607A = &TargetPosition0x607A;
#if 1 /* For homing mode */
	pCiA402->pHomeOffset0x607C = &HomeOffset0x607C;
	pCiA402->pHomingMethod0x6098 = &HomingMethod0x6098;
	pCiA402->pHomingSpeed0x6099 = &HomingSpeed0x6099;
	pCiA402->pHomingAcceleration0x609A = &HomingAcceleration0x609A;
	pCiA402->pSupportedHomingMode0x60E3 = &SupportedHomingMode0x60E3;
	pCiA402->pDigitalInputs0x60FD = &DigitalInputs0x60FD;			
#endif
	pCiA402->pSoftwarePositionLimit0x607D = &SoftwarePositionLimit0x607D;
	pCiA402->pQuickStopDeceleration0x6085 = &QuickStopDeceleration0x6085;
	pCiA402->pInterpolationTimePeriod0x60C2 = &InterpolationTimePeriod0x60C2;
	pCiA402->pTargetVelocity0x60FF = &TargetVelocity0x60FF;
	pCiA402->pSupportedDriveModes0x6502 = &SupportedDriveModes0x6502;
	pCiA402->pModularDeviceProfile0xF000 = &ModularDeviceProfile0xF000;	
	pCiA402->pModuleProfileList0xF010 = &ModuleProfileList0xF010;		

	
} /* End of CiA402_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CiA402_DeInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void CiA402_DeInit(void)
{
	nPdOutputSize = 0;
	nPdInputSize = 0;
	
} /* End of CiA402_DeInit() */

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    CiA402_MotionControl
 \brief
*////////////////////////////////////////////////////////////////////////////////////////
int CiA402_MotionControl(CiA402_AXIS_T *pAxis)
{
#ifdef MC_STACK_ENABLE
	UINT16 tmp16;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);

	/* Report digital inputs */
	*(pCiA402->pDigitalInputs0x60FD) = 0;
	
	/* Configurable objects */
	if (pAxis->bConfigurationAllowed)
	{
		/* Set mode of operation */
		if (*(pCiA402->pModesOfOperationDisplay0x6061) != *(pCiA402->pModesOfOperation0x6060))
		{
			*(pCiA402->pModesOfOperationDisplay0x6061) = *(pCiA402->pModesOfOperation0x6060);			
			DBG_PRINT("Set objModesOfOperationDisplay = %04x\r\n", *(pCiA402->pModesOfOperationDisplay0x6061));
		}
	}
	
	pdoCMD = *(pCiA402->pControlword0x6040);
	pdoSTS = *(pCiA402->pStatusword0x6041);
	if (pAxis->EmulationEnable)
	{
		/* Run servo emulation mode */
		
		/* Return actual velocity = target velocity */
		*(pCiA402->pTorqueActualValue0x6077) = 0;		
		*(pCiA402->pVelocityActualValue0x606C) = *(pCiA402->pTargetVelocity0x60FF);
		*(pCiA402->pPositionActualValue0x6064) = *(pCiA402->pTargetPosition0x607A);
		return 0;
	}

	/* Report actual torque */
	*(pCiA402->pTorqueActualValue0x6077) = MC_GetActualTorque();
	
	/* Report actual velocity */
	*(pCiA402->pVelocityActualValue0x606C) = MC_GetActualVelocity();

	/* Report position actual value */
	*(pCiA402->pPositionActualValue0x6064) = MC_GetActualPosition();
	
#if 0
	/* Set target velocity */
	if (pAxis->TargetVelocity != *(pCiA402->pTargetVelocity0x60FF))
	{
		pAxis->TargetVelocity = *(pCiA402->pTargetVelocity0x60FF);
		MC_SetTargetVelocity(*(pCiA402->pTargetVelocity0x60FF), 500);
		DBG_PRINT("Set objTargetVelocity=%ld\r\n", *(pCiA402->pTargetVelocity0x60FF));			
	}		
#endif
	
	/* Report error code */
	tmp16 = *(pCiA402->pStatusword0x6041) & STATUSWORD_STATE_MASK;
	if (tmp16 != STATUSWORD_STATE_FAULTREACTIONACTIVE && 
			tmp16 != STATUSWORD_STATE_FAULT &&
			(*(pCiA402->pErrorCode0x603F) == 0))
	{
		tmp16 = MC_GetFaultStatus();
    if (tmp16 & MCFS_OVER_VOLT)
		{
			CiA402_LocalError(ERROR_DC_LINK_OVER_VOLTAGE);
			DBG_PRINT("MC Error: MCFS_OVER_VOLT\r\n");		
		}
		else if (tmp16 & MCFS_UNDER_VOLT)
		{
			CiA402_LocalError(ERROR_DC_LINK_UNDER_VOLTAGE);
			DBG_PRINT("MC Error: MCFS_UNDER_VOLT\r\n");		
		}
		else if (tmp16 & MCFS_OVER_TEMP)
		{
			CiA402_LocalError(ERROR_EXCESS_TEMPERATURE_DRIVE);
			DBG_PRINT("MC Error: MCFS_OVER_TEMP\r\n");		
		}
		else if (tmp16 & MCFS_PHASE_ERR)
		{
			CiA402_LocalError(ERROR_PHASE_FAILURE);
			DBG_PRINT("MC Error: MCFS_PHASE_ERR\r\n");		
		}
		else if (tmp16 & MCFS_OVER_CURR)
		{
			CiA402_LocalError(ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL);
			DBG_PRINT("MC Error: MCFS_OVER_CURR\r\n");		
		}
		else if (tmp16 & MCFS_SW_ERR)
		{
			CiA402_LocalError(ERROR_SOFTWARE);
			DBG_PRINT("MC Error: MCFS_SW_ERR\r\n");		
		}
		else if (tmp16 & MCFS_VEC_ALI_ERR)
		{
			CiA402_LocalError(ERROR_VECTOR_ALIGMENT_FAIL);
			DBG_PRINT("MC Error: MCFS_VEC_ALI_ERR\r\n");		
		}		
		else if (tmp16 & MCFS_ENC_ALI_ERR)
		{
			CiA402_LocalError(ERROR_HOMING);
			DBG_PRINT("MC Error: MCFS_ENC_ALI_ERR\r\n");		
		}				
		else if (tmp16)
		{
			CiA402_LocalError(ERROR_UNKNOWN);			
			DBG_PRINT("MC Error: UNKNOWN (0x%04x)\r\n", tmp16);		
		}
	}/* End of error code report */
	
#endif
	
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    CiA402-Statemachine
        This function handles the state machine for devices using the CiA402 profile.
        called cyclic from MainLoop()
        All described transition numbers are referring to the document
        "ETG Implementation Guideline for the CiA402 Axis Profile" located on the EtherCAT.org download section

*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_StateMachine(CiA402_AXIS_T *pAxis)
{
	UINT16 StatusWord = 0;
	UINT16 ControlWord = 0;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);
	
	/* Skip unavailable axis */
	if(pAxis->bAxisIsActive == 0)
	{
		return;
	}

	StatusWord = *(pCiA402->pStatusword0x6041);
	ControlWord = *(pCiA402->pControlword0x6040);

	/* Clear statusword state and controlword processed complete bits */
	StatusWord &= ~(STATUSWORD_STATE_MASK | STATUSWORD_REMOTE);

	/* Skip state state transition if the previous transition is pending */
	if(pAxis->u16PendingOptionCode!= 0x0)
	{
		return;
	}
		
	/* Skip transition 1 and 2 if Slave has been under OP mode */
	if(pAxis->i16State < STATE_READY_TO_SWITCH_ON && nAlStatus == STATE_OP)
	{
		pAxis->i16State = STATE_READY_TO_SWITCH_ON;
	}
				
	switch(pAxis->i16State)
	{
	case STATE_NOT_READY_TO_SWITCH_ON:
		StatusWord |= (STATUSWORD_STATE_NOTREADYTOSWITCHON);
		if(nAlStatus == STATE_OP)
		{
			// Automatic transition -> Communication shall be activated
			pAxis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 1
			DBG_PRINT("Transition 1, NOT_READY_TO_SWITCH_ON->SWITCH_ON_DISABLED\r\n");
		}
		else
		{
			/* CiA402 statemachine shall stay in "STATE_NOT_READY_TO_SWITCH_ON" if EtherCAT state is not OP. */
			pAxis->i16State = STATE_NOT_READY_TO_SWITCH_ON; // stay in current state
		}
		break;
			
	case STATE_SWITCH_ON_DISABLED:
		StatusWord |= (STATUSWORD_STATE_SWITCHEDONDISABLED);
		if ((ControlWord & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
		{
			pAxis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 2
			DBG_PRINT("Transition 2, SWITCH_ON_DISABLED->READY_TO_SWITCH_ON\r\n");
		}
		break;
			
	case STATE_READY_TO_SWITCH_ON:
		StatusWord |= (STATUSWORD_STATE_READYTOSWITCHON);
		if (((ControlWord & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)
			|| ((ControlWord & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
		{
			pAxis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 7
			if (nAlStatus != STATE_OP)
			{
				DBG_PRINT("Transition 7, READY_TO_SWITCH_ON->SWITCH_ON_DISABLED(0x%04x)\r\n", ControlWord);							
			}
		}
		else if (((ControlWord & CONTROLWORD_COMMAND_SWITCHON_MASK) == CONTROLWORD_COMMAND_SWITCHON) ||
							((ControlWord & CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION))
		{
			pAxis->i16State = STATE_SWITCHED_ON;        // Transition 3
			DBG_PRINT("Transition 3, READY_TO_SWITCH_ON->SWITCHED_ON\r\n");
		}
		break;
			
	case STATE_SWITCHED_ON:
		StatusWord |= (STATUSWORD_STATE_SWITCHEDON);
		if ((ControlWord & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
		{
			pAxis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 6
			DBG_PRINT("Transition 6, SWITCHED_ON->READY_TO_SWITCH_ON\r\n");
		} 
		else if (((ControlWord & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP
						|| (ControlWord & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
		{
			pAxis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 10
			DBG_PRINT("Transition 10, SWITCHED_ON->SWITCH_ON_DISABLED\r\n");
		}
		else if ((ControlWord & CONTROLWORD_COMMAND_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_ENABLEOPERATION)
		{
			pAxis->i16State = STATE_OPERATION_ENABLED;  // Transition 4
			DBG_PRINT("Transition 4, SWITCHED_ON->OPERATION_ENABLED\r\n");
			//The Axis function shall be enabled and all internal set-points cleared.
		}
		break;
			
	case STATE_OPERATION_ENABLED:
		StatusWord |= (STATUSWORD_STATE_OPERATIONENABLED);
		if ((ControlWord & CONTROLWORD_COMMAND_DISABLEOPERATION_MASK) == CONTROLWORD_COMMAND_DISABLEOPERATION)
		{
			if(*(pCiA402->pDisableOperationOptionCode0x605C)!= DISABLE_DRIVE)
			{
				/* Disable operation pending */
				pAxis->u16PendingOptionCode = 0x605C; //STATE_TRANSITION (STATE_OPERATION_ENABLED,STATE_SWITCHED_ON);
				return;
			}
			pAxis->i16State = STATE_SWITCHED_ON;           // Transition 5
			DBG_PRINT("Transition 5, OPERATION_ENABLED->SWITCHED_ON\r\n");
		}
		else if ((ControlWord & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)
		{
			pAxis->i16State = STATE_QUICK_STOP_ACTIVE;  // Transition 11
			DBG_PRINT("Transition 11, OPERATION_ENABLED->QUICK_STOP_ACTIVE\r\n");
		}
		else if ((ControlWord & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
		{
			if (*(pCiA402->pShutdownOptionCode0x605B) != DISABLE_DRIVE)
			{
				/* Shutdown operation required */
				pAxis->u16PendingOptionCode = 0x605B; //STATE_TRANSITION (STATE_OPERATION_ENABLED,STATE_READY_TO_SWITCH_ON);
				return;
			}
			pAxis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 8
			DBG_PRINT("Transition 8, OPERATION_ENABLED->READY_TO_SWITCH_ON\r\n");
		}
		else if ((ControlWord & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE)
		{
			pAxis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 9
			DBG_PRINT("Transition 9, OPERATION_ENABLED->SWITCH_ON_DISABLED\r\n");
		}
		break;
			
	case STATE_QUICK_STOP_ACTIVE:
		StatusWord |= STATUSWORD_STATE_QUICKSTOPACTIVE;
		if ((*(pCiA402->pQuickStopOptionCode0x605A) != DISABLE_DRIVE) &&
				((*(pCiA402->pStatusword0x6041) & STATUSWORD_STATE_MASK)!= STATUSWORD_STATE_QUICKSTOPACTIVE))
		{
			/* Only execute quick stop action in state transition 11 */
			pAxis->u16PendingOptionCode = 0x605A;//STATE_TRANSITION (STATE_OPERATION_ENABLED,STATE_QUICK_STOP_ACTIVE);
			DBG_PRINT("Set u16PendingOptionCode = 0x605A\r\n");
			return;
		}

		if ((ControlWord & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE)
		{
			pAxis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 12
			DBG_PRINT("Transition 12, QUICK_STOP_ACTIVE->SWITCH_ON_DISABLED\r\n");
		}
		/* NOTE: it is not recommend to support transition 16 */
		break;
			
	case STATE_FAULT_REACTION_ACTIVE:
		StatusWord |= (STATUSWORD_STATE_FAULTREACTIONACTIVE);
		if (*(pCiA402->pFaultReactionOptionCode0x605E)!= DISABLE_DRIVE)
		{
			/* Fault reaction pending */
			pAxis->u16PendingOptionCode = 0x605E;
			return;
		}

		// Automatic transition
		pAxis->i16State = STATE_FAULT;// Transition 14
//			DBG_PRINT("Transition 14, FAULT_REACTION_ACTIVE->FAULT\r\n");
		break;
			
	case STATE_FAULT:
		StatusWord |= (STATUSWORD_STATE_FAULT);

		if ((ControlWord & CONTROLWORD_COMMAND_FAULTRESET_MASK) == CONTROLWORD_COMMAND_FAULTRESET)
		{
#ifdef MC_STACK_ENABLE
			/* Clear motor driver fault until state change */
			if (MC_GetState() == MCSTE_WAIT_CLR_ERR)
			{
				MC_FaultReset();
			}
			*(pCiA402->pErrorCode0x603F) = 0;
			DBG_PRINT("Fault reset\r\n");				
#endif
			pAxis->i16State = STATE_SWITCH_ON_DISABLED;// Transition 15
			DBG_PRINT("Transition 15, FAULT->SWITCH_ON_DISABLED\r\n");							
		}
		break;

	default:    //the sate variable is set to in invalid value => rest Axis
		StatusWord = (STATUSWORD_STATE_NOTREADYTOSWITCHON);
		pAxis->i16State = STATE_NOT_READY_TO_SWITCH_ON;
		break;

	}// switch(current state)

	/* Update operational functions (the low level power supply is always TRUE */
	switch(pAxis->i16State)
	{
	case STATE_NOT_READY_TO_SWITCH_ON:
	case STATE_SWITCH_ON_DISABLED:
	case STATE_READY_TO_SWITCH_ON:
		pAxis->bBrakeApplied = TRUE;
		pAxis->bHighLevelPowerApplied =  FALSE;
		pAxis->bAxisFunctionEnabled = FALSE;
		pAxis->bConfigurationAllowed = TRUE;
		break;
	case STATE_SWITCHED_ON:
		pAxis->bBrakeApplied = TRUE;
		pAxis->bHighLevelPowerApplied =  TRUE;
		pAxis->bAxisFunctionEnabled = FALSE;
		pAxis->bConfigurationAllowed = TRUE;
		break;
	case STATE_OPERATION_ENABLED:
	case STATE_QUICK_STOP_ACTIVE:
	case STATE_FAULT_REACTION_ACTIVE:
		pAxis->bBrakeApplied = FALSE;
		pAxis->bHighLevelPowerApplied =  TRUE;
		pAxis->bAxisFunctionEnabled = TRUE;
		pAxis->bConfigurationAllowed = FALSE;
		break;
	case STATE_FAULT:
		pAxis->bBrakeApplied = TRUE;
		pAxis->bHighLevelPowerApplied =  FALSE;
		pAxis->bAxisFunctionEnabled = FALSE;
		pAxis->bConfigurationAllowed = TRUE;
		break;
	default:
		pAxis->bBrakeApplied = TRUE;
		pAxis->bHighLevelPowerApplied =  FALSE;
		pAxis->bAxisFunctionEnabled = FALSE;
		pAxis->bConfigurationAllowed = TRUE;
		break;
	}

	if (pAxis->bHighLevelPowerApplied == TRUE)
	{
		StatusWord |= STATUSWORD_VOLTAGE_ENABLED;
	}
	else
	{
		StatusWord &= ~STATUSWORD_VOLTAGE_ENABLED;
	}
		
	/* State transition finished set controlword complete bit and update status object 0x6041 */
	*(pCiA402->pStatusword0x6041) = (StatusWord | STATUSWORD_REMOTE);

	/* Motor control driver */
#ifdef MC_STACK_ENABLE	
	CiA402_MotionControl(pAxis);

	/* Check if the motor need to start */
	if (pAxis->flags.b.high_pwr_applied==0 && (pAxis->bHighLevelPowerApplied!=0))
	{
		MC_StartMotor();
		DBG_PRINT("Start motor\r\n");
	}
	else if (pAxis->flags.b.high_pwr_applied!=0 && (pAxis->bHighLevelPowerApplied==0))
	{
		MC_StopMotor();
		DBG_PRINT("Stop motor\r\n");
	}

	/* Function mode process */
	if (pAxis->bAxisFunctionEnabled)
	{
		if (pAxis->flags.b.axis_func_enb==0)
		{
			switch (*(pCiA402->pModesOfOperationDisplay0x6061))
			{
			case HOMING_MODE:
				pAxis->pFuncIntf = &CIA402HM_FuncIntf;
				DBG_PRINT("Change to HOMING_MODE\r\n");
				break;
			case CYCLIC_SYNC_POSITION_MODE:
				pAxis->pFuncIntf = &CIA402CSP_FuncIntf;
				DBG_PRINT("Change to CYCLIC_SYNC_POSITION_MODE\r\n");			
				break;			
			case PROFILE_POSITION_MODE:
				pAxis->pFuncIntf = &CIA402PP_FuncIntf;
				DBG_PRINT("Change to PROFILE_POSITION_MODE\r\n");
				break;				
			default:
				pAxis->pFuncIntf = 0;
				DBG_PRINT("Change to undefined mode\r\n");			
				break;
			}
			
			if (pAxis->pFuncIntf)
			{
				if (pAxis->pFuncIntf->FuncStart)
				{
					DBG_PRINT("Start function\r\n");					
					pAxis->pFuncIntf->FuncStart(pAxis);
				}				
			}					
		}/* End of (pAxis->flags.b.axis_func_enb==0) */

		/* Run selected function mode */			
		if (MC_GetState() == MCSTE_RUN)
		{
			pAxis->pFuncIntf->FuncRun(pAxis);
		}/* End of MCSTE_RUN */
		
	}/* End of (pAxis->bAxisFunctionEnabled) */
	else if (pAxis->flags.b.axis_func_enb)
	{
		DBG_PRINT("Stop function\r\n");					
		pAxis->pFuncIntf->FuncStop(pAxis);
	}/* End of (pAxis->flags.b.axis_func_enb) */

		/* Update flags */
//		pAxis->flags.b.axis_is_act = (pAxis->bAxisIsActive == FALSE) ? 0:1;
//		pAxis->flags.b.brake_applied = (pAxis->bBrakeApplied == FALSE) ? 0:1;
//		pAxis->flags.b.low_pwr_applied = (pAxis->bLowLevelPowerApplied == FALSE) ? 0:1;
		pAxis->flags.b.high_pwr_applied = (pAxis->bHighLevelPowerApplied == FALSE) ? 0:1;
		pAxis->flags.b.axis_func_enb = (pAxis->bAxisFunctionEnabled == FALSE) ? 0:1;
//		pAxis->flags.b.cfg_allowed = (pAxis->bConfigurationAllowed == FALSE) ? 0:1;	
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param ErrorCode

 \brief    CiA402_LocalError
 \brief this function is called if an error was detected
*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_LocalError(UINT16 ErrorCode)
{
	CiA402_AXIS_T *pAxis = &CiA402Axis;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);
	
	if(pAxis->bAxisIsActive)
	{
		pAxis->i16State = STATE_FAULT_REACTION_ACTIVE;
		*(pCiA402->pErrorCode0x603F) = ErrorCode;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return TRUE if moving on predefined ramp is finished

 \brief    CiA402-TransitionAction
 \brief this function shall calculate the desired Axis input values to move on a predefined ramp
 \brief if the ramp is finished return TRUE
*////////////////////////////////////////////////////////////////////////////////////////
BOOL CiA402_TransitionAction(INT16 Characteristic, CiA402_AXIS_T *pAxis)
{
	switch(Characteristic)
	{
	case DISABLE_DRIVE:
#ifdef MC_STACK_ENABLE
		if (MC_GetState() > MCSTE_STOP)
		{
			MC_StopMotor();
			return FALSE;
		}
		DBG_PRINT("DISABLE_DRIVE\r\n");			
#endif
		return TRUE;
	case SLOW_DOWN_RAMP:
#ifdef MC_STACK_ENABLE
#if 0
		if (MC_GetState() > MCSTE_STOP)
		{
			MC_StopMotor();
			return FALSE;
		}
#endif
		DBG_PRINT("SLOW_DOWN_RAMP\r\n");			
#endif
		return TRUE;
	case QUICKSTOP_RAMP:
#ifdef MC_STACK_ENABLE
		if (MC_GetState() > MCSTE_STOP)
		{
			MC_StopMotor();
			return FALSE;
		}
		DBG_PRINT("QUICKSTOP_RAMP\r\n");			
#endif
		return TRUE;
	case STOP_ON_CURRENT_LIMIT:
#ifdef MC_STACK_ENABLE
		if (MC_GetState() > MCSTE_STOP)
		{
			MC_StopMotor();
			return FALSE;
		}
		DBG_PRINT("STOP_ON_CURRENT_LIMIT\r\n");
#endif
		return TRUE;
	case STOP_ON_VOLTAGE_LIMIT:
#ifdef MC_STACK_ENABLE
		if (MC_GetState() > MCSTE_STOP)
		{
			MC_StopMotor();
			return FALSE;
		}
		DBG_PRINT("STOP_ON_VOLTAGE_LIMIT\r\n");			
#endif
		return TRUE;			
	default:
		break;
	}
	return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    CiA402-Application
 \brief check if a state transition is pending and pass desired ramp-code to CiA402TransitionAction()
 \brief if this functions returns true the state transition is finished.
*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_Application(CiA402_AXIS_T *pAxis)
{
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);
	
	/*clear "Drive follows the command value" flag if the target values from the master overwritten by the local application*/
	if(pAxis->u16PendingOptionCode != 0 &&
			(*(pCiA402->pModesOfOperationDisplay0x6061) == CYCLIC_SYNC_POSITION_MODE ||
			 *(pCiA402->pModesOfOperationDisplay0x6061) == CYCLIC_SYNC_VELOCITY_MODE))
	{
		*(pCiA402->pStatusword0x6041) &= ~ STATUSWORD_DRIVE_FOLLOWS_COMMAND;
	}
    else if (*(pCiA402->pModesOfOperationDisplay0x6061) != HOMING_MODE)
	{
		*(pCiA402->pStatusword0x6041) |= STATUSWORD_DRIVE_FOLLOWS_COMMAND;
	}

	switch(pAxis->u16PendingOptionCode)
	{
	case 0x605A:
		/*state transition 11 is pending analyse shutdown option code (0x605A)*/
		{
			UINT16 ramp = *(pCiA402->pQuickStopOptionCode0x605A);
			/*masked and execute specified quick stop ramp characteristic */
			if(*(pCiA402->pQuickStopOptionCode0x605A) > 4 && *(pCiA402->pQuickStopOptionCode0x605A) <9)
			{
				if(*(pCiA402->pQuickStopOptionCode0x605A) == 5)
					ramp = 1;
				if(*(pCiA402->pQuickStopOptionCode0x605A) == 6)
					ramp = 2;
				if(*(pCiA402->pQuickStopOptionCode0x605A) == 7)
					ramp = 3;
				if(*(pCiA402->pQuickStopOptionCode0x605A) == 8)
					ramp = 4;
			}

			if(CiA402_TransitionAction(ramp,pAxis))
			{
				/*quick stop ramp is finished complete state transition*/
				pAxis->u16PendingOptionCode = 0x0;
				if(*(pCiA402->pQuickStopOptionCode0x605A) > 0 && *(pCiA402->pQuickStopOptionCode0x605A) < 5)
				{
					pAxis->i16State = STATE_SWITCH_ON_DISABLED;    //continue state transition 12
					DBG_PRINT("Transition 12(pending), QUICK_STOP_ACTIVE->SWITCH_ON_DISABLED\r\n");
				}
				else if(*(pCiA402->pQuickStopOptionCode0x605A) > 4 && *(pCiA402->pQuickStopOptionCode0x605A) < 9)
					*(pCiA402->pStatusword0x6041) |= STATUSWORD_TARGET_REACHED;
			}
		}
		break;
	case 0x605B:
		/*state transition 8 is pending analyse shutdown option code (0x605B)*/
		{
			if(CiA402_TransitionAction(*(pCiA402->pShutdownOptionCode0x605B),pAxis))
			{
				/*shutdown ramp is finished complete state transition*/
				pAxis->u16PendingOptionCode = 0x0;
				pAxis->i16State = STATE_READY_TO_SWITCH_ON;    //continue state transition 8
				DBG_PRINT("Transition 8(pending), OPERATION_ENABLED->READY_TO_SWITCH_ON\r\n");							
			}
		}
		break;
	case 0x605C:
		/*state transition 5 is pending analyse Disable operation option code (0x605C)*/
		{
			if(CiA402_TransitionAction(*(pCiA402->pDisableOperationOptionCode0x605C),pAxis))
			{
				/*disable operation ramp is finished complete state transition*/
				pAxis->u16PendingOptionCode = 0x0;
				pAxis->i16State = STATE_SWITCHED_ON;    //continue state transition 5
				DBG_PRINT("Transition 5(pending), OPERATION_ENABLED->SWITCHED_ON\r\n");
			}
		}
		break;
	case 0x605E:
		/*state transition 14 is pending analyse Fault reaction option code (0x605E)*/
		{
			if(CiA402_TransitionAction(*(pCiA402->pFaultReactionOptionCode0x605E),pAxis))
			{
				/*fault reaction ramp is finished complete state transition*/
				pAxis->u16PendingOptionCode = 0x0;
				pAxis->i16State = STATE_FAULT;    //continue state transition 14
//			DBG_PRINT("Transition 4(pending), FAULT_REACTION_ACTIVE->FAULT\r\n");
			}
		}
		break;
	default:
		//pending transition code is invalid => values from the master are used
		if (*(pCiA402->pModesOfOperationDisplay0x6061) != HOMING_MODE)
		{
			*(pCiA402->pStatusword0x6041) |= STATUSWORD_DRIVE_FOLLOWS_COMMAND;
		}
		break;
	}
		
    if(bDcSyncActive
        && (pAxis->u32CycleTime != 0)
        && ((*(pCiA402->pSupportedDriveModes0x6502) >> (*(pCiA402->pModesOfOperation0x6060) - 1)) & 0x1)) //Mode of Operation (0x6060) - 1 specifies the Bit within Supported Drive Modes (0x6502)

    {

    }
}

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    The function is called when an error state was acknowledged by the master

*////////////////////////////////////////////////////////////////////////////////////////

void	APPL_AckErrorInd(UINT16 stateTrans)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
             all general settings were checked to start the mailbox handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case this function will be called cyclically
            until a value unequal NOERROR_INWORK is returned

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
             to stop the mailbox handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                        register (0x204) when this function is succeeded. The event mask can be adapted
                        in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
           all general settings were checked to start the input handler. This function
           informs the application about the state transition, the application can refuse
           the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
	CiA402_AXIS_T *pAxis = &CiA402Axis;
	
	UINT32 Sync0CycleTime = 0;

	HW_EscReadDWord(Sync0CycleTime, ESC_DC_SYNC0_CYCLETIME_OFFSET);
	Sync0CycleTime = SWAPDWORD(Sync0CycleTime);

	/*Init CiA402 structure if the device is in SM Sync mode
		the CiA402 structure will be Initialized after calculation of the Cycle time*/
	if(bDcSyncActive == TRUE)
	{
		Sync0CycleTime = Sync0CycleTime / 1000; //get cycle time in us
		if(pAxis->bAxisIsActive)
			pAxis->u32CycleTime = Sync0CycleTime;
	}
		
	return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopInputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartOutputHandler(void)
{
#ifdef MC_STACK_ENABLE
	CiA402_AXIS_T *pAxis = &CiA402Axis;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);
	
	if(pAxis->bAxisIsActive)
	{
		if (MC_GetFaultStatus() != MCFS_NO_ERR || *(pCiA402->pErrorCode0x603F) != 0)
		{
			MC_FaultReset();
			*(pCiA402->pErrorCode0x603F) = 0;
			pAxis->i16State = STATE_SWITCH_ON_DISABLED;
			DBG_PRINT("Clear error due to ESC state transition from SAFEOP to OP\r\n");
		}	
	}
#endif
		
	return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopOutputHandler(void)
{
	/* Stop motor */
#ifdef MC_STACK_ENABLE
	CiA402_AXIS_T *pAxis = &CiA402Axis;
//	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);
	
	if(pAxis->bAxisIsActive)
	{
		if (MC_GetState() > MCSTE_STOP)
		{
			MC_StopMotor();
			DBG_PRINT("Stop motor due to ESC state transition from OP to SAFEOP\r\n");
		}
		CiA402_LocalError(ERROR_COMMUNICATION);
	}
#endif
		
	return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GenerateMapping(UINT16 *pInputSize,UINT16 *pOutputSize)
{
	UINT16 result = ALSTATUSCODE_NOERROR;
	UINT16 PDOAssignEntryCnt = 0;
	UINT16 u16cnt = 0;
	UINT16 InputSize = 0;
	UINT16 OutputSize = 0;
	CiA402_AXIS_T *pAxis = &CiA402Axis;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);	
	UINT32 *pEntries;
	
	pAxis->bAxisIsActive = TRUE;

	/*Scan object 0x1C12 RXPDO assign*/
	for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
	{
		switch ((sRxPDOassign.aEntries[PDOAssignEntryCnt] & 0x000F))    //mask Axis type (supported modes)
		{
		case 0:
			/*drive mode supported    csv(cyclic sync velocity) : bit 8
                                csp (cyclic sync position) : bit 7*/
			*(pCiA402->pSupportedDriveModes0x6502) = 0x180;
			pEntries = &(pCiA402->pRxPdoMappingCspCsv0x1600->SI1);
			for(u16cnt =0 ; u16cnt < pCiA402->pRxPdoMappingCspCsv0x1600->u16SubIndex0;u16cnt++)
			{
				OutputSize +=(UINT16)(pEntries[u16cnt] & 0xFF);
			}
			break;
		case 1:
			/*drive mode supported    csp (cyclic sync position) : bit 7*/
			*(pCiA402->pSupportedDriveModes0x6502) = 0xA1;
			pEntries = &(pCiA402->pRxPdoMappingCsp0x1601->SI1);		
			for(u16cnt =0 ; u16cnt < pCiA402->pRxPdoMappingCsp0x1601->u16SubIndex0;u16cnt++)
			{
				OutputSize +=(UINT16)(pEntries[u16cnt] & 0xFF);
			}
			break;
		case 2:
			/*drive mode supported    csv(cyclic sync velocity) : bit 8*/
			*(pCiA402->pSupportedDriveModes0x6502) = 0x100;
			pEntries = &(pCiA402->pRxPdoMappingCsv0x1602->SI1);			
			for(u16cnt =0 ; u16cnt < pCiA402->pRxPdoMappingCsv0x1602->u16SubIndex0;u16cnt++)
			{
				OutputSize += (UINT16)(pEntries[u16cnt] & 0xFF);;
			}
			break;
		}
	}

	OutputSize = OutputSize >> 3;

	if(result == 0)
	{
		/*Scan Object 0x1C13 TXPDO assign*/
		for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
		{
			switch ((sTxPDOassign.aEntries[PDOAssignEntryCnt] & 0x000F))    //mask Axis type (supported modes)
			{
			case 0: /*csp/csv*/
				pEntries = &(pCiA402->pTxPdoMappingCspCsv0x1A00->SI1);				
				for(u16cnt =0 ; u16cnt < pCiA402->pTxPdoMappingCspCsv0x1A00->u16SubIndex0;u16cnt++)
				{
					InputSize +=(UINT16)(pEntries[u16cnt] & 0xFF);
				}
				break;
			case 1: /*csp*/
				pEntries = &(pCiA402->pTxPdoMappingCsp0x1A01->SI1);				
				for(u16cnt =0 ; u16cnt < pCiA402->pTxPdoMappingCsp0x1A01->u16SubIndex0;u16cnt++)
				{
					InputSize +=(UINT16)(pEntries[u16cnt] & 0xFF);
				}
				break;
			case 2: /*csv*/
				pEntries = &(pCiA402->pTxPdoMappingCsv0x1A02->SI1);				
				for(u16cnt =0 ; u16cnt < pCiA402->pTxPdoMappingCsv0x1A02->u16SubIndex0;u16cnt++)
				{
					InputSize +=(UINT16)(pEntries[u16cnt] & 0xFF);
				}
				break;
			}
		}
        
		InputSize = InputSize >> 3;
	}

	*pInputSize = InputSize;
	*pOutputSize = OutputSize;
	return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to input process data
\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_InputMapping(UINT16* pData)
{
	UINT16 j = 0;
	UINT8 *pTmpData = (UINT8 *)pData;
	CiA402_AXIS_T *pAxis = &CiA402Axis;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);	
	
	for (j = 0; j < sTxPDOassign.u16SubIndex0; j++)
	{
		switch ((sTxPDOassign.aEntries[j]& 0x000F))
		{
		case 0://copy csp/csv TxPDO entries
			{
				TCiA402PDO1A00 *pInputs = (TCiA402PDO1A00 *)pTmpData;

				pInputs->ObjStatusWord = SWAPWORD(*(pCiA402->pStatusword0x6041));
				pInputs->ObjPositionActualValue = SWAPDWORD(*(pCiA402->pPositionActualValue0x6064));
				pInputs->ObjVelocityActualValue = SWAPDWORD(*(pCiA402->pVelocityActualValue0x606C));
				pInputs->ObjModesOfOperationDisplay = SWAPWORD(0x00FF & *(pCiA402->pModesOfOperationDisplay0x6061));

				/*shift pointer PDO mapping object following*/
				if(j < (sTxPDOassign.u16SubIndex0 - 1))
					pTmpData += SIZEOF(TCiA402PDO1A00);
				
			}
			break;
		case 1://copy csp TxPDO entries
			{
				TCiA402PDO1A01 *pInputs = (TCiA402PDO1A01 *)pTmpData;

				pInputs->ObjStatusWord = SWAPWORD(*(pCiA402->pStatusword0x6041));
				pInputs->ObjPositionActualValue = SWAPDWORD(*(pCiA402->pPositionActualValue0x6064));

				/*shift pointer PDO mapping object following*/
				if(j < (sTxPDOassign.u16SubIndex0 - 1))
					pTmpData += SIZEOF(TCiA402PDO1A01);
							
			}
			break;
		case 2://copy csv TxPDO entries
			{
				TCiA402PDO1A02 *pInputs = (TCiA402PDO1A02 *)pTmpData;

#if 0
				if (pInputs->ObjStatusWord != SWAPWORD(*(pCiA402->pStatusword0x6041))
				{
					DBG_PRINT("csv TxPDO: objStatusWord=0x%04x\r\n", *(pCiA402->pStatusword0x6041));
				}
							
				if (pInputs->ObjPositionActualValue != SWAPDWORD(*(pCiA402->pPositionActualValue0x6064))
				{
					DBG_PRINT("csv TxPDO: objPositionActualValue=%ld\r\n", *(pCiA402->pPositionActualValue0x6064));									
				}
#endif	
				pInputs->ObjStatusWord = SWAPWORD(*(pCiA402->pStatusword0x6041));
				pInputs->ObjPositionActualValue = SWAPDWORD(*(pCiA402->pPositionActualValue0x6064));
            
				/*shift pointer PDO mapping object following*/
				if(j < (sTxPDOassign.u16SubIndex0 - 1))
					pTmpData += SIZEOF(TCiA402PDO1A02);
			}
			break;
		}//switch TXPDO entry
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_OutputMapping(UINT16* pData)
{
	UINT16 j = 0;
	UINT8 *pTmpData = (UINT8 *)pData;
	CiA402_AXIS_T *pAxis = &CiA402Axis;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);	
	
	for (j = 0; j < sRxPDOassign.u16SubIndex0; j++)
	{
		switch ((sRxPDOassign.aEntries[j] & 0x000F))
		{
		case 0://csp/csv RxPDO    entries
			{
				TCiA402PDO1600 *pOutputs = (TCiA402PDO1600 *)pTmpData;

				*(pCiA402->pControlword0x6040) = SWAPWORD(pOutputs->ObjControlWord);
				*(pCiA402->pTargetPosition0x607A) = SWAPDWORD(pOutputs->ObjTargetPosition);
				*(pCiA402->pTargetVelocity0x60FF) = SWAPDWORD(pOutputs->ObjTargetVelocity);
				*(pCiA402->pModesOfOperation0x6060) = SWAPWORD((pOutputs->ObjModesOfOperation & 0x00FF));

				/*shift pointer PDO mapping object following*/
				if(j < (sRxPDOassign.u16SubIndex0 - 1))
					pTmpData += SIZEOF(TCiA402PDO1600);
			}
			break;
		case 1://csp RxPDO    entries
			{
				TCiA402PDO1601 *pOutputs = (TCiA402PDO1601 *)pTmpData;

				*(pCiA402->pControlword0x6040) = SWAPWORD(pOutputs->ObjControlWord);
				*(pCiA402->pTargetPosition0x607A) = SWAPDWORD(pOutputs->ObjTargetPosition);

				/*shift pointer PDO mapping object following*/
				if(j < (sRxPDOassign.u16SubIndex0 - 1))
					pTmpData += SIZEOF(TCiA402PDO1601);
			}
			break;
		case 2://csv RxPDO    entries
			{
				TCiA402PDO1602 *pOutputs = (TCiA402PDO1602 *)pTmpData;

#if 0										
				if (*(pCiA402->pControlword0x6040) != SWAPWORD(pOutputs->ObjControlWord))
				{
					DBG_PRINT("csp RxPDO: objControlWord=0x%04x\r\n", SWAPWORD(pOutputs->ObjControlWord));
				}
				if (*(pCiA402->pTargetVelocity0x60FF) != SWAPDWORD(pOutputs->ObjTargetVelocity))
				{
					DBG_PRINT("csp RxPDO: objTargetVelocity=%ld\r\n", SWAPDWORD(pOutputs->ObjTargetVelocity));							
				}
#endif
				*(pCiA402->pControlword0x6040) = SWAPWORD(pOutputs->ObjControlWord);
				*(pCiA402->pTargetVelocity0x60FF) = SWAPDWORD(pOutputs->ObjTargetVelocity);

				/*shift pointer PDO mapping object following*/
				if(j < (sRxPDOassign.u16SubIndex0 - 1))
					pTmpData += SIZEOF(TCiA402PDO1602);
			}
			break;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief    This function will called from the synchronisation ISR 
            or from the mainloop if no synchronisation is supported
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_Application(void)
{
	CiA402_AXIS_T *pAxis = &CiA402Axis;

	if(bEcatInputUpdateRunning)
	{
		CiA402_StateMachine(pAxis);
	}
		
	if (pAxis->bAxisIsActive)
	{
		CiA402_Application(pAxis);
	}

}

#if EXPLICIT_DEVICE_ID
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    The Explicit Device ID of the EtherCAT slave

 \brief     Calculate the Explicit Device ID
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GetDeviceID()
{
	/* Explicit Device 5 is expected by Explicit Device ID conformance tests
	   Explicit Device from switch digit of AX58200-EXB-ADIO for general application
	*/
	return 5;
}
#endif

/** @} */


