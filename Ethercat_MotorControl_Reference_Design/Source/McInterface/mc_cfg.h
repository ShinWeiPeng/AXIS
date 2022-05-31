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

#ifndef __MC_CFG_H__
#define __MC_CFG_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_types.h"

/* NAMING CONSTANT DECLARATIONS */
#define MC_STACK_ENABLE
#define MC_DEBUG_ENABLE
#define MC_DBG_PARAM_ENABLE
#define MC_SW_PROTECT_ENABLE

#define MC_PARAM_TABLE_SIZE				1600
#define MC_DAS_BUF_SIZE				    8192

#include "param_def.h"

#ifdef MC_DEBUG_ENABLE
#include <stdio.h>
	#define MC_PRINT                 printf
#else
	#define MC_PRINT(f, ...)         {}
#endif

/* Interrupt Priority Setting */
/*
		Priority Group=0 -> 0bits: preemption-priority, 4bit: sub-priority
		Priority Group=1 -> 1bits: preemption-priority, 3bit: sub-priority
		Priority Group=2 -> 2bits: preemption-priority, 2bit: sub-priority
		Priority Group=3 -> 3bits: preemption-priority, 1bit: sub-priority
		Priority Group=4 -> 4bits: preemption-priority, 0bit: sub-priority
*/
#define MC_INT_PRIORITY_GROUP	3

/* INCREMENTAL Interface IO Definition */
#define INCREMENTAL_INSTANCE	QEI0
#define INCREMENTAL_MFP         14
#define INCREMENTAL_PORT        PA
#define INCREMENTAL_PIN_A       BIT_4
#define INCREMENTAL_PIN_B       BIT_3
#define INCREMENTAL_PIN_Z       BIT_5
#define INCREMENTAL_PORT_RE     PC
#define INCREMENTAL_PIN_RE      BIT_4

/* LED Interface IO Definition */
#if 0
#define LED_PORT			    PF
#define LED_PORT2nd			    PE
#define LED_PIN_RED         	BIT_5
#define LED_PIN_ORANGE      	BIT_0
#define LED_PIN_YELLOW      	BIT_4
#define LED_PIN_GREEN       	BIT_6
#endif

/* PWM/Gate Driver Interface IO Definition */
#define PWMCTL_PORT_ENABLE      PC
#define PWMCTL_PIN_ENABLE       BIT_2
#define PWMCTL_PORT_TRIGGER     PE
#define PWMCTL_PIN_TRIGGER      BIT_1
#define PWMCTL_PORT_BRAKE       PB
#define PWMCTL_PIN_BRAKE        BIT_6

/* PWM U/V/W IO Definition */
#define PWM_INSTANCE        	EPWM1
#define PWM_MFP             	12
#ifdef __ICCARM__
  #define PWM_IRQHandler      	PWM1P0_IRQHandler
#else
  #define PWM_IRQHandler      	EPWM1P0_IRQHandler
#endif
#define PWM_IRQ             	EPWM1P0_IRQn
#define PWM_INT_PREEMPTPRIO     1
#define PWM_INT_SUBPRIO         0
#define PWM_PORT            	PC
#define PWM_PIN_U           	BIT_5
#define PWM_PIN_V           	BIT_3
#define PWM_PIN_W           	BIT_1

/* Three Phase Current Sensing IO Definition */
#define ADCMNG_INSTANCE         EADC0
#define ADCMNG_MFP              1
#define ADCMNG_IRQHandler       EADC00_IRQHandler
#define ADCMNG_IRQ              EADC00_IRQn
#define ADCMNG_PORT             PB
#define ADCMNG_CURFBK_U_PIN     BIT_12
#define ADCMNG_CURFBK_V_PIN     BIT_13
#define ADCMNG_CURFBK_W_PIN     BIT_14
#define ADCMNG_POWER_PIN 	    BIT_0
#define ADCMNG_TEMPERATURE_PIN 	BIT_7

/* Timer Task 1Khz Definition */
#define TMRTASK1K_INSTANCE         TIMER1
#define TMRTASK1K_INT_PREEMPTPRIO  5
#define TMRTASK1K_INT_SUBPRIO      0
#define TMRTASK1K_IRQHandler       TMR1_IRQHandler
#define TMRTASK1K_IRQ              TMR1_IRQn

/* Motor Control Debug IO */
#ifdef MC_DEBUG_ENABLE
#define DBG_FOC_PROC_PORT       PB
#define DBG_FOC_PROC_PIN        BIT_1

#define DBG_CTRLLOOP_PROC_PORT  PC
#define DBG_CTRLLOOP_PROC_PIN   BIT_0
#endif

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */

#endif /* __MC_CFG_H__ */

/* End of mc_cfg.h */
