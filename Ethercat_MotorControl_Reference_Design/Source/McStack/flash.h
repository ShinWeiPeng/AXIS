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

#ifndef __FLASH_H__
#define __FLASH_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_cfg.h"

/* NAMING CONSTANT DECLARATIONS */
#define PARAMS_MAX_SIZE			(0x07FE0)
#define PARAMS_BASE_ADDR1   (0x38000)
#define PARAMS_BASE_ADDR2   (0x78000)
/* TYPE DECLARATIONS */
typedef struct
{
	uint8_t	  Signature[4];
	uint16_t	ImageType;
	uint16_t	Compress;
	uint32_t	ContentOffset;
	uint32_t	FileLen;
	uint32_t	Reserved0;
	uint32_t	Checksum32;	
	uint32_t	Reserved1[2];
} MS_Firmware_Header_t;

typedef struct{
    MS_Firmware_Header_t FwHeader;
    int32_t FileSize;
    int32_t FileSize_div_4;
    int32_t BaseAddr;
    int32_t BufferLen;
    uint8_t Buffer[256];
    uint32_t CalcChecksum32;
} MS_Flash_Handle_t;
/* MACRO DECLARATIONS */

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
void MS_GetFlashParamsAreaData(void);
static void doCommand(int last);
static int MS_GetLineData(void);
static int MS_CalcCheckSum(int base);
static int MS_CheckSignature(uint8_t *p);
static uint32_t MS_Swap(uint32_t d);
static void MS_ReadFlashRom(int addr,int n,uint8_t *buf);
#endif /* __FLASH_H__ */

/* End of flash.h */
