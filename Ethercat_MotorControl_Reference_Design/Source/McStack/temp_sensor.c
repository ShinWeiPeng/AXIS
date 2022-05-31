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
#include "temp_sensor.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_GetTemp_d()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint16_t MS_GetTemp_d(MS_Temp_Handle_t *pHandle)
{
  /* Get Temprature adc value */
  MH_AdcConv(&pHandle->TempRegister,pHandle->ChannelNum);
  adcT = *(pHandle->TempRegister.ConvBuffer);

  return adcT;
} /* End of MS_GetTemp_d() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_GetTemp_V()
 * Purpose: Temperature digital to analog
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint16_t MS_GetTemp_V( MS_Temp_Handle_t * pHandle )
{
  uint16_t TA;
  
  /* TA = (Vout - V0C) / TC */
  TA = (adcT - pHandle->MCP9700V0C) / pHandle->MCP9700TC; 

  return ( ( uint16_t )TA );
} /* End of MS_GetTemp_V() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_CheckTemp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint16_t MS_CheckTemp( MS_Temp_Handle_t * pHandle )
{
  return ( pHandle->FaultState );
} /* End of MS_CheckTemp() */

/* End of temp_sensor.c */

