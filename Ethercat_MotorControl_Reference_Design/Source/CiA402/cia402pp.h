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

#ifndef __CIA402PP_H__
#define __CIA402PP_H__

/* INCLUDE FILE DECLARATIONS */
#include "cia402.h"

/* NAMING CONSTANT DECLARATIONS */
#define CIA402HM_

typedef enum{
	CIA402PP_IDLE = 0,
	CIA402PP_STOP,	
	CIA402PP_ERR,
	CIA402PP_START,
	CIA402PP_RUN,	
	CIA402PP_RUN_OFFSET,		
}CIA402PP_STATE;

/* TYPE DECLARATIONS */
typedef union{
  uint16_t d16;
  struct{
    uint16_t switch_on:               1;  /* Controlword.0 */
    uint16_t enable_voltage:          1;  /* Controlword.1 */
    uint16_t quick_stop:              1;  /* Controlword.2 */
    uint16_t enable_operation:        1;  /* Controlword.3 */	
    uint16_t homing_op_start:         1;  /* Controlword.4
                                           0: Homing mode inactive
                                        0->1: Start homing mode
                                           1: Homing mode active
                                        1->0: Interrupt homing mode
                                     */
    uint16_t reserved_6_5:            2;  /* Controlword.6:5 */		
    uint16_t fault_reset:             1;  /* Controlword.7 */
    uint16_t halt:                    1;  /* Controlword.8
                                        0: Execute the instruction of bit 4
                                        1: Stop axle with homing acceleration
                                     */
    uint16_t reserved_10_9:           2;  /* Controlword.10:9 */
    uint16_t manufacturer_spec_15_11: 5;  /* Controlword.15:11 */		
  } b;
} CIA402PP_CONTROLWORD_OBJ;/* 0x6040 */

typedef union{
  uint16_t d16;
  struct{
    uint16_t ready_to_switch_on:      1;  /* Statusword.0 */
    uint16_t switch_on:               1;  /* Statusword.1 */
    uint16_t operation_enabled:       1;  /* Statusword.2 */		
    uint16_t fault:                   1;  /* Statusword.3 */
    uint16_t voltage_enabled:         1;  /* Statusword.4 */
    uint16_t quick_stop:              1;  /* Statusword.5 */
    uint16_t switch_on_disabled:      1;  /* Statusword.6 */
    uint16_t warning:                 1;  /* Statusword.7 */		
    uint16_t manufacturer_spec_8:     1;  /* Statusword.8 */				
    uint16_t remote:                  1;  /* Statusword.9 */						
    uint16_t target_reached:          1;  /* Statusword.10
                                        0: Halt=0: Home position not reached
                                           Halt=1: Axle decelerates
                                        1: Halt=0: Home position reached
                                           Halt=1: Axle has velocity 0
                                  	 */
    uint16_t internal_limit_active:   1;  /* Statusword.11 */
    uint16_t homing_attained:         1;  /* Statusword.12
                                        0: Homing mode not yet completed
                                        1: Homing mode carried out successfully
                                     */																						 
    uint16_t homing_error:            1;  /* Statusword.13
                                        0: No homing error
                                        1: Homing mode carried out not successfully
                                     */																				
    uint16_t manufacturer_spec_15_14: 2;  /* Statusword.15:14 */		
  } b;
} CIA402PP_STATUSWORD_OBJ;/* 0x6041 */

typedef struct{
	CIA402PP_STATE State;
} CIA402PP_CTRL_OBJ;

/* GLOBAL VARIABLES */
extern CIA402_FUNC_INTF CIA402PP_FuncIntf;

/* EXPORTED SUBPROGRAM SPECIFICATIONS */ 
int32_t CIA402PP_Init(void *pAxis);
int32_t CIA402PP_DeInit(void *pAxis);
int32_t CIA402PP_Run(void *pAxis);

#endif /* __CIA402PP_H__ */

/* End of cai402pp.h */
