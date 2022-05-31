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
#include <string.h>
#include "mc.h"
#include "McHal_misc.h"
#include "McHal_tim.h"
#include "McHal_husb.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */
MC_T MotorCtrl;

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Params: BusVoltageParamsM1
 * Note:
 * ----------------------------------------------------------------------------
 */
MI_Vbus_Handle_t BusVoltageParamsM1 =
{
	.VbusInfo =
	{
		.ConversionFactor = (uint16_t)(3.3 * ((100 + 2) / 2)),
		.VbusRegister =
		{
			.pInst = ADCMNG_INSTANCE,
			.pPort = ADCMNG_PORT,
			.Channel = ADCMNG_POWER_PIN,
			.IrqType = ADCMNG_IRQ,
			.MultiFuncValue = ADCMNG_MFP,
			.SampleModuleMask = 5,
		},
		.ChannelNum = 1,
	},
};

/*
 * ----------------------------------------------------------------------------
 * Params: TemperatureParamsM1
 * Note:
 * ----------------------------------------------------------------------------
 */
MI_Temp_Handle_t TemperatureParamsM1 =
{
	.TempInfo =
	{
		.TempRegister =
		{
			.pInst = ADCMNG_INSTANCE,
			.pPort = ADCMNG_PORT,
			.Channel = ADCMNG_TEMPERATURE_PIN,
			.IrqType = ADCMNG_IRQ,
			.MultiFuncValue = ADCMNG_MFP,
			.SampleModuleMask = 6,
		},
		.MCP9700V0C = ADC_RESOLUTION*MCP9700_V0C/ADC_REFERENCE_VOLTAGE,
		.MCP9700TC = ADC_RESOLUTION*MCP9700_TC/ADC_REFERENCE_VOLTAGE,
		.ChannelNum = 1,
	},
};

/*
 * ----------------------------------------------------------------------------
 * Params: CurrentParamsM1
 * Note:
 * ----------------------------------------------------------------------------
 */
MI_Curr_Handle_t CurrentParamsM1 =
{
	.CurrInfo =
	{
		.CurrentRegister =
		{
			.pInst = ADCMNG_INSTANCE,
			.IrqType = ADCMNG_IRQ,
			.MultiFuncValue = ADCMNG_MFP,

			/* pin u */
			.pPort[0] = ADCMNG_PORT,
			.Channel[0] = ADCMNG_CURFBK_U_PIN,
			.SampleModuleMask[0] = 0,

			/* pin v */
			.pPort[1] = ADCMNG_PORT,
			.Channel[1] = ADCMNG_CURFBK_V_PIN,
			.SampleModuleMask[1] = 1,

			/* pin w */
			.pPort[2] = ADCMNG_PORT,
			.Channel[2] = ADCMNG_CURFBK_W_PIN,
			.SampleModuleMask[2] = 2,

			/* pin v */
			.pPort[3] = ADCMNG_PORT,
			.Channel[3] = ADCMNG_CURFBK_V_PIN,
			.SampleModuleMask[3] = 3,

			/* pin u */
			.pPort[4] = ADCMNG_PORT,
			.Channel[4] = ADCMNG_CURFBK_U_PIN,
			.SampleModuleMask[4] = 4,
		},
		.ChannelNum = 5,
	},
};

/*
 * ----------------------------------------------------------------------------
 * Params: IncremEncParamsM1
 * Note:
 * ----------------------------------------------------------------------------
 */
MI_IncremEnc_Handle_t IncremEncParamsM1 =
{
	.IncremEncInfo = {
		.QeiRegister = {
			.pInst = INCREMENTAL_INSTANCE,
			.pPortIncEnc = INCREMENTAL_PORT,
			.PinA = INCREMENTAL_PIN_A,
			.PinB = INCREMENTAL_PIN_B,
			.PinZ = INCREMENTAL_PIN_Z,

			.pPortWireCtrl = INCREMENTAL_PORT_RE,
			.PinWireCtrl = INCREMENTAL_PIN_RE,
			.MultiFuncValue = INCREMENTAL_MFP,
		},
	},
};

/*
 * ----------------------------------------------------------------------------
 * Params: PwmParamsM1
 * Note:
 * ----------------------------------------------------------------------------
 */
MI_Pwm_Handle_t PwmParamsM1 = {
	.PwmInfo = {
		.EpwmRegister = {
			.pInst = PWM_INSTANCE,
			.IrqType = PWM_IRQ,
			.PreemptPrio = PWM_INT_PREEMPTPRIO,
			.SubPrio = PWM_INT_SUBPRIO,
			.pPortPwm = PWM_PORT,
			.PinPwmU = PWM_PIN_U,
			.PinPwmV = PWM_PIN_V,
			.PinPwmW = PWM_PIN_W,
			.MultiFuncValue = PWM_MFP,
			.EpwmChU = 0,
			.EpwmChV = 2,
			.EpwmChW = 4,
		},
	},
};

/*
 * ----------------------------------------------------------------------------
 * Params: SwProtParamsM1
 * Note:
 * ----------------------------------------------------------------------------
 */
MI_SwProt_Handle_t SwProtParamsM1 = {
	.SwProtInfo = {
		.SwProtRegister = {
			.pPortGateDriveEnb = PWMCTL_PORT_ENABLE,
			.PinGateDriveEnb = PWMCTL_PIN_ENABLE,
			.pPortPwmWdtTrg = PWMCTL_PORT_TRIGGER,
			.PinPwmWdtTrg = PWMCTL_PIN_TRIGGER,
			.pPortMotorBrake = PWMCTL_PORT_BRAKE,
			.PinMotorBrake = PWMCTL_PIN_BRAKE,
		},
		.pVbusInfo = &BusVoltageParamsM1.VbusInfo,
		.pCurrInfo = &CurrentParamsM1.CurrInfo,
		.pTempInfo = &TemperatureParamsM1.TempInfo,
	},
};
/*
 * ----------------------------------------------------------------------------
 * Params: PosParamsM1
 * Note: posCMD: Position loop command, unit: ticks, range: +/-5000000
 *       iopANG: Position loop feedback,unit: ticks, range: +/-5000000
 *       posOUT: Position loop Pid output,unit: ticks/ms, range: +/-5000
 *       posSUM: Integraler
 *       posKP/posKI: PI controler params
 *       posMXE: Error limit
 *       posMXI: Integraler limit
 *       posMXO: Output limit
 * ----------------------------------------------------------------------------
 */
MI_Pi_Handle_t PosParamsM1 = {
	.PiInfo = {
		.Cmd = &posCMD,
		.Feedback = &iopANG,
		.Kp = &posKP,
		.Ki = &posKI,
		.KpDiv = 100,
		.KiDiv = 10000,
		.Error = &posERR,
		.Integraler = &posSUM,
		.ErrorLimit = &posMXE,
		.IntegralLimit = &posMXI,
		.OutputLimit = &posMXO,
		.Division = &posDIV,
		.Output = &posOUT,
	},
};
/*
 * ----------------------------------------------------------------------------
 * Params: SpdParamsM1
 * Note: spdCMD: Speed loop command, unit: ticks/ms, range: +/-5000
 *       iopSPD: Speed loop feedback,unit: ticks/ms, range: +/-5000
 *       spdOUT: Speed loop Pid output,unit: 0.1%, range: +/-1000
 *       spdSUM: Integraler
 *       spdKP/spdKI: PI controler params
 *       spdMXE: Error limit
 *       spdMXI: Integraler limit
 *       spdMXO: Output limit
 * ----------------------------------------------------------------------------
 */
MI_Pi_Handle_t SpdParamsM1 = {
	.PiInfo = {
		.Cmd = &spdCMD,
		.Feedback = &iopSPD,
		.Kp = &spdKP,
		.Ki = &spdKI,
		.KpDiv = 100,
		.KiDiv = 10000,
		.Error = &spdERR,
		.Integraler = &spdSUM,
		.ErrorLimit = &spdMXE,
		.IntegralLimit = &spdMXI,
		.OutputLimit = &spdMXO,
		.Division = &spdDIV,
		.Output = &spdOUT,
	},
};
/*
 * ----------------------------------------------------------------------------
 * Params: AmpParamsM1
 * Note: ampCMD: Current loop command, unit: 0.1%, range: +/-1000
 *       ampAMP: Current loop feedback,unit: 0.1%, range: +/-1000
 *       iopID/iopIQ: Id,Iq ,unit: 0.1%, range: +/-1000
 *       iopVD/iopVQ: Vd,Vq ,unit: 0.1%, range: +/-1000
 *       ampSD/ampSQ: DQ Integraler
 *       ampKP/ampKI: PI controler params
 *       ampMXE: Error limit
 *       ampMXI: Integraler limit
 *       ampMXO: Output limit
 * ----------------------------------------------------------------------------
 */
MI_Pi_Handle_t AmpParamsM1 = {
	.PiInfo = {
		.Cmd = &ampCMD,
		.Feedback = &iopIQ,
		.Kp = &ampKP,
		.Ki = &ampKI,
		.KpDiv = 100,
		.KiDiv = 200000,
		.Error = &ampERR,
		.Integraler = &ampSQ,
		.ErrorLimit = &ampMXE,
		.IntegralLimit = &ampMXI,
		.OutputLimit = &ampMXO,
		.Division = &ampDIV,
		.Output = &iopVQ,
	},
};
/*
 * ----------------------------------------------------------------------------
 * Params: AmpParamsM2
 * Note: ampCMD: Current loop command, unit: 0.1%, range: +/-1000
 *       ampAMP: Current loop feedback,unit: 0.1%, range: +/-1000
 *       iopID/iopIQ: Id,Iq ,unit: 0.1%, range: +/-1000
 *       iopVD/iopVQ: Vd,Vq ,unit: 0.1%, range: +/-1000
 *       ampSD/ampSQ: DQ Integraler
 *       ampKP/ampKI: PI controler params
 *       ampMXE: Error limit
 *       ampMXI: Integraler limit
 *       ampMXO: Output limit
 * ----------------------------------------------------------------------------
 */
int32_t ampIdCmd = 0;
MI_Pi_Handle_t AmpParamsM2 = {
	.PiInfo = {
		.Cmd = &ampIdCmd,
		.Feedback = &iopID,
		.Kp = &ampKP,
		.Ki = &ampKI,
		.KpDiv = 100,
		.KiDiv = 200000,
		.Error = &ampERR2,
		.Integraler = &ampSD,
		.ErrorLimit = &ampMXE,
		.IntegralLimit = &ampMXI,
		.OutputLimit = &ampMXO,
		.Division = &ampDIV,
		.Output = &iopVD,
	},
};

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MC_Init(void)
{
	MC_T *pMc = &MotorCtrl;

	/* Init Modules */
	memset(mapMemory, 0, MC_PARAM_TABLE_SIZE);
	memset(pMc, 0, sizeof(MC_T));

	/* Init step mode module */
	STPM_Init(&(pMc->StpMod));
	MS_MapAdcBlkInit();
	MS_MapPwmBlkInit();
	MS_MapAbzBlkInit();
	MI_MapUvwBlkInit();
	MI_MapAngBlkInit();
	TC_Init();
	MS_MapCmdBlkInit();
	MS_MapPosBlkInit();
	MS_MapSpdBlkInit();
	MS_MapAmpBlkInit();
	DAS_Init();
	MS_MapProtectBlkInit();

	/* Initialize VCOM */
	MS_GetFlashParamsAreaData();
	MH_VCP_Init();
	/* Initialize Vbus monitor */
	MH_AdcRegInit(&(BusVoltageParamsM1.VbusInfo.VbusRegister), 1);
	/* Initialize temperature monitor */
	MH_AdcRegInit(&(TemperatureParamsM1.TempInfo.TempRegister), 1);
	/* Initialize current feedback */
	MH_AdcRegInit(&(CurrentParamsM1.CurrInfo.CurrentRegister), 5);

	/* Initialize software protector */
	MH_SwProtRegInit(&SwProtParamsM1.SwProtInfo.SwProtRegister);

	/* Initialize PWM peripheral */
	MH_EpwmRegInit(&PwmParamsM1.PwmInfo.EpwmRegister);

	/* Initialize QEI peripheral */
	MH_QeiRegInit(&IncremEncParamsM1.IncremEncInfo.QeiRegister);

	/* Initialize timer1 peripheral */
	MH_TimRegInit();

	MH_DebugHwInit();

	return MCSTS_OK;
} /* End of MC_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_DeInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MC_DeInit(void)
{

} /* End of MC_DeInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_TickRun()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MC_TickRun(void)
{
	MC_T *pMc = &MotorCtrl;
	uint32_t i;

	sysCLK++;
	if (powerON > 0 && powerON < 999999)
	{
		powerON++;
	}

	for (i = 0; i < MC_MAX_TICK_CNTS; i++)
	{
		if (pMc->TickMs[i])
		{
			pMc->TickMs[i]--;
		}
	}

	STPM_TickRun(&(pMc->StpMod));
} /* End of MC_TickRun() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: STPM_VectorAlignment()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
MC_STATE_E MC_TaskRun(void)
{
	MC_T *pMc = &MotorCtrl;
	MC_STATE_E cur_ste;
	STPM_VECALI_STATE_E vec_ali_ste;

	/* Software protection */
	SWP_Run(&SwProtParamsM1.SwProtInfo);

	/* DAS process */
	DAS_Run();

	/* Trajectory Control */
	TC_Run();

	if(cmdCMD >= LC_POS_CTRL)
	{
		MS_PiController(&PosParamsM1.PiInfo,Pos);
	}

	if(cmdCMD >= LC_SPD_CTRL)
	{
		MS_PiController(&SpdParamsM1.PiInfo,Spd);
	}

	/* Update Input PD Data */
	pdoANG = iopANG;//Position feedback(ticks)
	pdoSPD = iopSPD;//Speed feedback(ticks/ms)
	pdoAMP = iopIQ;//Current feedback(0.1%)
	pdoOUT = spdOUT;//Close loop output(0.1%)

	if (iopALM != SAT_NO_ERR)
	{
		if (iopALM == SAT_OVER_VOLT)
		{
			pMc->FaultFlags |= MCFS_OVER_VOLT;
		}
		if (iopALM == SAT_UNDER_VOLT)
		{
			pMc->FaultFlags |= MCFS_UNDER_VOLT;
		}
		if (iopALM == SAT_OVER_TEMP)
		{
			pMc->FaultFlags |= MCFS_OVER_TEMP;
		}
		if (iopALM == SAT_PHASE_ERR)
		{
			pMc->FaultFlags |= MCFS_PHASE_ERR;
		}
		if (iopALM == SAT_OVER_CURR_ID || iopALM == SAT_OVER_CURR_IQ)
		{
			pMc->FaultFlags |= MCFS_OVER_CURR;
		}
	}

	/* Emergency event process */
	if (pMc->FaultFlags)
	{
		/* Turn off torque output if motor is running */
		if (pMc->State > MCSTE_STOP)
		{
			pMc->State = MCSTE_FAULT;
			MC_PRINT("Emergency motor stop\r\n");
		}
	}

	/* Record current state */
	cur_ste = pMc->State;

	switch (pMc->State)
	{
	default:
	case MCSTE_IDLE:
	case MCSTE_TUNE:
		/* Waiting for available command */
		if (pMc->Cmd == MCCMD_START)
		{
			pMc->Cmd = MCCMD_NO_CMD;
			pMc->State = MCSTE_START_VEC_ALIGN;
			pMc->TickMs[0] = 10;
			break;
		}

		/* Run servo tune process while at IDLE/TUNE state */
		STPM_RunStepping(&(pMc->StpMod));
		STPM_VectorAlignment(&(pMc->StpMod));
		break;

	case MCSTE_START_VEC_ALIGN:
		/* Wait delay */
		if (pMc->TickMs[0])
		{
			break;
		}
		pwmRST = MC_VECTOR_ALIGN_TORQUE;//Set torque for vector alignment
		pMc->State = MCSTE_VEC_ALIGN;

	case MCSTE_VEC_ALIGN:
		vec_ali_ste = STPM_VectorAlignment(&(pMc->StpMod));
		if (vec_ali_ste == STPMVA_DONE)
		{
			/* Vector alignment OK */
			pMc->State = MCSTE_STARTUP;
			pMc->TickMs[0] = 10;
		}
		else if (vec_ali_ste >= STPMVA_FAIL)
		{
			/* Fail to vector alignment */
			pMc->FaultFlags |= MCFS_VEC_ALI_ERR;
			pMc->State = MCSTE_FAULT;
		}
		break;

	case MCSTE_START_ENC_ALIGN:
		IncremEncParamsM1.IncremEncInfo.AlignRequest = 1;
		pMc->HomingReach = 0;
		segCMD = TCC_ABS_POS;
		pdoPOS = iopANG + (angREL*11/10);
		pdoVEL = pMc->HomingVelocity;
		pMc->State = MCSTE_ENC_ALIGN;
		pMc->TickMs[0] = 10000;
	case MCSTE_ENC_ALIGN:
		/* Wait for homing reach */
		if (pMc->TickMs[0] == 0)
		{
			TC_Init();
			pMc->FaultFlags |= MCFS_ENC_ALI_ERR;
			pMc->State = MCSTE_FAULT;
		}
		else if ((IncremEncParamsM1.IncremEncInfo.AlignRequest == 0) && (segCMD == TCC_NO_CMD))
		{
			MC_ClearActualPosition();
			pMc->Cmd = MCCMD_NO_CMD;
			pMc->State = MCSTE_RUN;
			pMc->HomingReach = 1;
		}
		break;

	case MCSTE_STARTUP:
		/* Wait an duration time */
		if (pMc->TickMs[0])
		{
			break;
		}

		/* Enable position control loop */
//		cmdCMD = LC_ALL_OFF;//Disable control loops

		/* Reset control loop commands */
    pdoTRQ = 0;//Set open current loop compensation
    pdoMXC = 1000;//Full close loop operation mode
		cmdON = MC_ENABLE;//Enable PWM drving hardware
		uvwRDY = SCM_AUTO_CTRL;//Enable vector auto control
    pdoPOS = iopANG;//Align command to current position
		pdoVEL = pMc->VelocityCmd;
		cmdCMD = LC_PP_CTRL;//Enable position loop + trajectory generator

		pMc->State = MCSTE_RUN;
		MC_PRINT("Start motor!\r\n");
		break;

	case MCSTE_RUN:
		/* Waiting for available commands */
		if (pMc->Cmd == MCCMD_SET_POS)
		{
			segCMD = TCC_ABS_POS;
			pdoPOS = pMc->PositionCmd;
			pdoVEL = pMc->VelocityCmd;
			pMc->Cmd = MCCMD_NO_CMD;
		}
		else if (pMc->Cmd == MCCMD_STOP)
		{
			pMc->State = MCSTE_SHUTDOWN;
		}
		else if (pMc->Cmd == MCCMD_ENC_ALIGN)
		{
			pMc->State = MCSTE_START_ENC_ALIGN;
		}
		break;

	case MCSTE_SHUTDOWN:
		/* Normally stop motor */
		cmdON = MC_DISABLE;//Disable PWM drving hardware
		pwmENB = MC_DISABLE;//Disable gate drive
		uvwRDY = SCM_DISABLE;//Disable vector output
		cmdCMD = LC_ALL_OFF;//Disable control loops
		pMc->State = MCSTE_STOP;
		break;

	case MCSTE_STOP:
		pMc->Cmd = MCCMD_NO_CMD;//Clear command
		pMc->State = MCSTE_IDLE;
		MC_PRINT("Stop motor!\r\n");
		break;

	case MCSTE_FAULT:
		/* Emergency stop motor */
		cmdON = MC_DISABLE;//Disable PWM drving hardware
		pwmENB = MC_DISABLE;//Disable gate drive
		uvwRDY = SCM_DISABLE;//Disable vector output
		cmdCMD = LC_ALL_OFF;//Disable control loops
		pwmCMD = 0;
		pwmRST = 0;
		pMc->Cmd = MCCMD_NO_CMD;//Clear command
		pMc->State = MCSTE_WAIT_CLR_ERR;
		MC_PRINT("Enter fault state: FaultFlags=0x%08x, iopALM=%d\r\n", pMc->FaultFlags, iopALM);
		break;

	case MCSTE_WAIT_CLR_ERR:
		/* Waiting for FAULT_RESET command */
		if (pMc->Cmd == MCCMD_FAULT_RESET)
		{
			pMc->Cmd = MCCMD_NO_CMD;//Clear command
			iopALM = 0;
			pMc->FaultFlags = MCFS_NO_ERR;//Clear error flags
			pMc->State = MCSTE_IDLE;
			MC_PRINT("Fault Reset OK!\r\n");
		}
		break;
	}

	return cur_ste;
} /* End of MC_TaskRun() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_StartMotor()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MC_StartMotor(void)
{
	MC_T *pMc = &MotorCtrl;

	if (pMc->State <= MCSTE_STOP)
	{
		pMc->Cmd = MCCMD_START;
	}
} /* End of MC_StartMotor() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_StopMotor()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MC_StopMotor(void)
{
	MC_T *pMc = &MotorCtrl;

	if (pMc->State > MCSTE_STOP)
	{
		pMc->Cmd = MCCMD_STOP;
	}
} /* End of MC_StopMotor() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_GetState()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
MC_STATE_E MC_GetState(void)
{
	MC_T *pMc = &MotorCtrl;

	return pMc->State;
} /* End of MC_GetState() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_GetActualTorque()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MC_GetActualTorque(void)
{
	return iopIQ;
} /* End of MC_GetActualTorque() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_SetTargetVelocity()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MC_SetTargetVelocity(int32_t Velocity)
{
	MC_T *pMc = &MotorCtrl;

	pMc->VelocityCmd = Velocity;
} /* End of MC_SetTargetVelocity() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_GetActualVelocity()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MC_GetActualVelocity(void)
{
	return iopSPD;
} /* End of MC_GetActualVelocity() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_ClearActualPosition()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MC_ClearActualPosition(void)
{
	/* Reset position loop to zero angle */
	iopANG = 0;
	posCMD = 0;
//	posSUM = 0;
//	posOUT = 0;

	/* Reset trajectory generator */
	cmdPOS = 0;
	segPOS = 0;
	segCMD = TCC_NO_CMD;
	segSTS = TCS_STOP;

	pdoPOS = 0;

	MI_CalcEncAngle(1);
} /* End of MC_ClearActualPosition() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_SetTargetPosition()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MC_SetTargetPosition(int32_t Position, int32_t Velocity)
{
	MC_T *pMc = &MotorCtrl;

	if (pMc->Cmd == MCCMD_NO_CMD)
	{
		pMc->PositionCmd = Position;
		pMc->VelocityCmd = Velocity;
		pMc->Cmd = MCCMD_SET_POS;
	}

//	MC_PRINT("TargetPos: Position=%d, Velocity=%d\r\n", Position, Velocity);

} /* End of MC_SetTargetPosition() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_GetActualPosition()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MC_GetActualPosition(void)
{
	return iopANG;
} /* End of MC_GetActualPosition() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_FaultReset()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MC_FaultReset(void)
{
	MC_T *pMc = &MotorCtrl;

	pMc->Cmd = MCCMD_FAULT_RESET;
} /* End of MC_FaultReset() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_GetFaultStatus()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
MC_FAULT_E MC_GetFaultStatus(void)
{
	MC_T *pMc = &MotorCtrl;

	return pMc->FaultFlags;
} /* End of MC_GetFaultStatus() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_StartHoming()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MC_StartHoming(int32_t TargetVelocity)
{
	MC_T *pMc = &MotorCtrl;

	if (pMc->Cmd == MCCMD_NO_CMD)
	{
		pMc->HomingReach = 0;
		pMc->Cmd = MCCMD_ENC_ALIGN;
		pMc->HomingVelocity = TargetVelocity;
	}
} /* End of MC_StartHoming() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MC_CheckHomingDone()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MC_CheckHomingDone(void)
{
	MC_T *pMc = &MotorCtrl;

	return (pMc->HomingReach);
} /* End of MC_CheckHomingDone() */

/* End of mc.c */
