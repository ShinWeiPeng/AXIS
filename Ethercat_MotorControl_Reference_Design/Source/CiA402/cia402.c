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
#include "cia402.h"
#include "cia402hm.h"
#include "cia402csp.h"
#include "cia402pp.h"
#include "AX58200_MotorControl.h"
#include "main.h"

/* NAMING CONSTANT DECLARATIONS */
/* MACRO DECLARATIONS */
/* TYPE DECLARATIONS */
/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */
static uint8_t  CIA402_EmulationEnable = 0;

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t CIA402_Init(void)
{
	
	return 0;
} /* End of CIA402_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402_DeInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t CIA402_DeInit(void)
{

	return 0;
} /* End of CIA402_DeInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402_EmulationControl()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t CIA402_EmulationControl(uint8_t Enb)
{
	CIA402_EmulationEnable = Enb ? 1:0;
	return 0;
} /* End of CIA402_EmulationControl() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402_CheckEmulationEnable()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint8_t CIA402_CheckEmulationEnable(void)
{
	return CIA402_EmulationEnable;
} /* End of CIA402_CheckEmulationEnable() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CIA402_GetHallStatus()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint8_t CIA402_GetHallStatus(void)
{
	uint8_t tmp8=0;
	
	if (CIA402_CheckEmulationEnable())
	{
		tmp8 = 0x01;
		return tmp8;
	}
#if 0
	if (HAL_GPIO_ReadPin(HS1_GPIO_Port, HS1_Pin) == GPIO_PIN_SET)
	{
		tmp8 |= 0x01;
	}
	
	if (HAL_GPIO_ReadPin(HS2_GPIO_Port, HS2_Pin) == GPIO_PIN_SET)
	{
		tmp8 |= 0x02;
	}	
	
	if (HAL_GPIO_ReadPin(HS3_GPIO_Port, HS3_Pin) == GPIO_PIN_SET)
	{
		tmp8 |= 0x04;
	}	
#endif	
	return tmp8;
} /* End of CIA402_GetHallStatus() */

/* End of cia402.c */
