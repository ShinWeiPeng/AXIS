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

#ifndef __MC_MISC_H__
#define __MC_MISC_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_cfg.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* MACRO DECLARATIONS */
#define MH_WRITE_GPIO_PIN(port, pin, ctrl)    {if (ctrl!=0){port->DOUT |= pin;}else{port->DOUT &= (~pin);}}
#define MH_TOGGLE_GPIO_PIN(port, pin)         (port->DOUT ^= pin)
#define MH_READ_GPIO_PIN(port, pin)           ((port->PIN & pin)?1:0)

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
uint32_t MH_BitToNum(uint32_t Bit);
int32_t  MH_SetMFP(GPIO_T *pInst, uint32_t Pins, uint8_t MultiFuncValue);
void     MH_TmrClkSource(TIMER_T *pInst, MC_EADA_E Ctrl);
void     MH_SpiClkSource(SPI_T *pInst, MC_EADA_E Ctrl);
void     MH_UartClkSource(UART_T *pInst, MC_EADA_E Ctrl);
void     MH_EcapClkSource(ECAP_T *pInst, MC_EADA_E Ctrl);
void     MH_EpwmClkSource(EPWM_T *pInst, MC_EADA_E Ctrl);
void     MH_EadcClkSource(EADC_T *pInst, MC_EADA_E Ctrl);
void     MH_QeiClkSource(QEI_T *pInst, MC_EADA_E Ctrl);
int32_t  MH_ClkTick_1ms(void);
int32_t  MH_Delay_1ms(int time);
#endif /* __MC_MISC_H__ */

/* End of mc_misc.h */
