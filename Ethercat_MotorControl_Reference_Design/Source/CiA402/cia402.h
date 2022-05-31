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

#ifndef __CIA402_H__
#define __CIA402_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_types.h"

/* NAMING CONSTANT DECLARATIONS */


/* TYPE DECLARATIONS */
typedef enum{
	CIA402_EMUL_DISABLED = 0,
	CIA402_ACTUAL_IS_TARGET_POS,
} CIA402_EMUL_MODE;

typedef union {
  uint32_t d32;
  struct{
    uint32_t pp:               1;  /* bit0 */
    uint32_t vl:               1;  /* bit1 */
    uint32_t pv:               1;  /* bit2 */
    uint32_t tq:               1;  /* bit3 */		
    uint32_t reserved_4:       1;  /* bit4 */				
    uint32_t hm:               1;  /* bit5 */						
    uint32_t ip:               1;  /* bit6 */
    uint32_t csp:              1;  /* bit7 */
    uint32_t csv:              1;  /* bit8 */		
    uint32_t cst:              1;  /* bit9 */				
    uint32_t cstca:            1;  /* bit10 */						
    uint32_t reserved_31_11:  21;  /* bit31:11 */
  } b;
} OBJ0x6502_SUPPORTED_DRIVE_MODES;

typedef union {
  uint32_t d32;
  struct{
    uint32_t axis_is_act:      1;  /* bit0 */
    uint32_t brake_applied:    1;  /* bit1 */
    uint32_t low_pwr_applied:  1;  /* bit2 */
    uint32_t high_pwr_applied: 1;  /* bit3 */		
    uint32_t axis_func_enb:    1;  /* bit4 */				
    uint32_t cfg_allowed:      1;  /* bit5 */						
    uint32_t reserved_31_6:   26;  /* bit31:6 */
  } b;
} CIA402_FLAGS;

typedef struct{
	void *pCtrlObj;
  int32_t (*FuncStart)(void *pAxis);
  int32_t (*FuncStop)(void *pAxis);
  int32_t (*FuncRun)(void *pAxis);		
} CIA402_FUNC_INTF;

/* GLOBAL VARIABLES */
/* EXPORTED SUBPROGRAM SPECIFICATIONS */ 
int32_t CIA402_Init(void);
int32_t CIA402_DeInit(void);
int32_t CIA402_EmulationControl(uint8_t mode);
uint8_t CIA402_CheckEmulationEnable(void);
uint8_t CIA402_GetHallStatus(void);

#endif /* __CIA402_H__ */

/* End of cai402.h */
