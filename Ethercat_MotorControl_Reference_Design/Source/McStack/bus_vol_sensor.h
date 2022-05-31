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

#ifndef __BUS_VOL_SENSOR_H__
#define __BUS_VOL_SENSOR_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_cfg.h"
#include "McHal_eadc.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */
typedef struct
{
    MH_AdcRegConfig_t	    VbusRegister;
    uint16_t			    ConversionFactor;
    uint16_t			    FaultState;
    uint8_t				    ChannelNum;
} MS_Vbus_Handle_t;

/* MACRO DECLARATIONS */

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
void MS_MapAdcBlkInit(void);
int32_t MS_GetBusVoltage_d(MS_Vbus_Handle_t *pHandle);
uint16_t MS_GetAvBusVoltage_d(MS_Vbus_Handle_t *pHandle);
uint16_t MS_GetBusVoltage_V(MS_Vbus_Handle_t *pHandle);
uint16_t MS_CheckVbus(MS_Vbus_Handle_t *pHandle);

#endif /* __BUS_VOL_SENSOR_H__ */

/* End of bus_vol_sensor.h */
