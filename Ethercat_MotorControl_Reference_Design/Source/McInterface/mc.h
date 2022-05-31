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

#ifndef __MC_H__
#define __MC_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_cfg.h"
#include "mc_math.h"
#include "bus_vol_sensor.h"
#include "temp_sensor.h"
#include "curr_fdbk.h"
#include "svpwm.h"
#include "incremental_fdbk.h"
#include "stepper_mode.h"
#include "flash.h"
#include "loop_control.h"
#include "pid.h"
#include "servo_commu.h"
#include "sw_protection.h"
#include "trajectory_ctrl.h"
#include "das.h"

/* NAMING CONSTANT DECLARATIONS */
#define MC_MAX_TICK_CNTS				3
#define MC_VECTOR_ALIGN_TORQUE	250//iopVD(0.1%)
#define ADC_RESOLUTION          (1<<12)     // adc 12bit
#define ADC_REFERENCE_VOLTAGE   3300        // adc reference voltage 3.3v magnify 1000 times
#define MCP9700_V0C             500         // V0C 0.5v magnify 1000 times
#define MCP9700_TC              10          // TC 0.01v magnify 1000 times

/* TYPE DECLARATIONS */
typedef enum
{
	MCSTE_IDLE = 0,
	MCSTE_TUNE,
	MCSTE_FAULT,
	MCSTE_WAIT_CLR_ERR,
	MCSTE_STOP,
	MCSTE_START_VEC_ALIGN,
	MCSTE_VEC_ALIGN,
	MCSTE_START_ENC_ALIGN,
	MCSTE_ENC_ALIGN,
	MCSTE_STARTUP,
	MCSTE_RUN,
	MCSTE_SHUTDOWN,
} MC_STATE_E;

typedef enum
{
	MCCMD_NO_CMD = 0,
	MCCMD_START,
	MCCMD_ENC_ALIGN,
	MCCMD_SET_POS,
	MCCMD_STOP,
	MCCMD_FAULT_RESET,
} MC_CMD_E;

typedef struct
{

	MC_STATE_E   				State;
	volatile MC_CMD_E		    Cmd;
	MC_FAULT_E   				FaultFlags;
	volatile uint32_t           TickMs[MC_MAX_TICK_CNTS];

	/* Step mode */
	STPM_CTRL_T					StpMod;
	/* Homing mode */
	int32_t					    HomingReach;
	int32_t					    HomingVelocity;
	/* Speed loop */
	int32_t					    VelocityCmd;
	/* Position loop */
	int32_t					    PositionCmd;
	uint32_t				    ConvFactor_mDegToTick;
} MC_T;

/* MapMemory */
typedef struct
{
    MS_Vbus_Handle_t VbusInfo;
} MI_Vbus_Handle_t;

typedef struct
{
    MS_Temp_Handle_t TempInfo;
} MI_Temp_Handle_t;

typedef struct
{
    MS_Curr_Handle_t    CurrInfo;
    MCMATH_AB_PHASE_T   Iab;
} MI_Curr_Handle_t;

typedef struct
{
    MS_IncremEnc_Handle_t IncremEncInfo;
} MI_IncremEnc_Handle_t;

typedef struct
{
    MS_Epwm_Handle_t PwmInfo;
    MCMATH_AB_PHASE_T Vab;
} MI_Pwm_Handle_t;

typedef struct
{
    MS_SwProt_Handle_t SwProtInfo;
} MI_SwProt_Handle_t;

typedef struct
{
    MS_PI_Handle_t PiInfo;
} MI_Pi_Handle_t ;

/* MACRO DECLARATIONS */

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
extern MC_T MotorCtrl;
extern MI_Vbus_Handle_t BusVoltageParamsM1;
extern MI_Temp_Handle_t TemperatureParamsM1;
extern MI_Curr_Handle_t CurrentParamsM1;
extern MI_IncremEnc_Handle_t IncremEncParamsM1;
extern MI_Pwm_Handle_t PwmParamsM1;
extern MI_SwProt_Handle_t SwProtParamsM1;
extern MI_Pi_Handle_t PosParamsM1;
extern MI_Pi_Handle_t SpdParamsM1;
extern MI_Pi_Handle_t AmpParamsM1;
extern MI_Pi_Handle_t AmpParamsM2;

void MI_IncremEncModuleInit(void);
void MI_PwmModuleInit(void);
void MI_AbsoluteEncModuleInit(void);
void MI_SwProtModuleInit(void);

/* API functions */
	/* General Purppose */
int32_t  		MC_Init(void);
void     		MC_DeInit(void);
void     		MC_TickRun(void);
MC_STATE_E	MC_TaskRun(void);
void        MC_StartMotor(void);
void        MC_StopMotor(void);
MC_STATE_E  MC_GetState(void);
	/* Current Loop */
int32_t     MC_GetActualTorque(void);
	/* Speed Loop */
void        MC_SetTargetVelocity(int32_t Velocity);
int32_t     MC_GetActualVelocity(void);
	/* Position Loop */
void        MC_ClearActualPosition(void);
void				MC_SetTargetPosition(int32_t Position, int32_t Velocity);
int32_t     MC_GetActualPosition(void);
	/* Fault Process */
void        MC_FaultReset(void);
MC_FAULT_E	MC_GetFaultStatus(void);
	/* Alignment Process */
void				MC_StartHoming(int32_t TargetVelocity);
int32_t 		MC_CheckHomingDone(void);

#endif /* __MC_H__ */

/* End of mc.h */
