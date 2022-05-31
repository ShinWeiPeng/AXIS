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

#ifndef __MC_TYPES_H__
#define __MC_TYPES_H__

/* INCLUDE FILE DECLARATIONS */
#include "NuMicro.h"

/* NAMING CONSTANT DECLARATIONS */


/* TYPE DECLARATIONS */
typedef enum
{
	MCSTS_BUSY      = 1,
	MCSTS_OK				= 0,
	MCSTS_ERR       = -1,
} MC_STATUS_E;

typedef enum
{
	MCFS_NO_ERR     	= 0x0000,
	MCFS_OVER_VOLT  	= 0x0001,
	MCFS_UNDER_VOLT 	= 0x0002,
	MCFS_OVER_TEMP  	= 0x0004,
	MCFS_PHASE_ERR  	= 0x0008,
	MCFS_OVER_CURR  	= 0x0010,
	MCFS_SW_ERR     	= 0x0020,
	MCFS_VEC_ALI_ERR	= 0x0040,
	MCFS_ENC_ALI_ERR	= 0x0080,
} MC_FAULT_E;

typedef enum
{
	MC_STOP = 0,
	MC_START,
} MC_STSP_E;

typedef enum
{
	MC_DISABLE = 0,
	MC_ENABLE,
} MC_EADA_E;

typedef enum
{
	MC_FORWARD = 0,
	MC_BACKWARD,
} MC_DIR_E;

/* MACRO DECLARATIONS */
#define MC_SWAP16(w)  (uint16_t)(((w & 0x00FF) << 8) | ((w & 0xFF00) >> 8))
#define MC_SWAP32(dw) (uint32_t)(((dw & 0x000000FF) << 24) | ((dw & 0xFF000000) >> 24) | ((dw & 0x0000FF00) << 8) | ((dw & 0x00FF0000) >> 8))

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */


#endif /* __MC_TYPES_H__ */

/* End of mc_types.h */
