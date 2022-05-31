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
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "servo_commu.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */
static MS_Flash_Handle_t FlashObj;

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_ReadFlashRom()
 * Purpose: Read flashrom
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void MS_ReadFlashRom(int addr,int n,uint8_t *buf)
{
	/* Read 4 bytes at a time */
	for (int i=0; i<n; i+=4,addr+=4,buf+=4)
	{
		*(uint32_t *)buf=*(uint32_t *)addr;
	}
} /* End of MS_ReadFlashRom() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_Swap()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static uint32_t MS_Swap(uint32_t d)
{
	return(((d>>24)&0xff)|((d>>8)&0xff00)|((d<<8)&0xff0000)|((d<<24)&0xff000000));
} /* End of MS_Swap() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_CheckSignature()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int32_t MS_CheckSignature(uint8_t *p)
{
	return((p[0]=='A')&&(p[1]=='S')&&(p[2]=='I')&&(p[3]=='X'));
} /* End of MS_CheckSignature() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_CalcCheckSum()
 * Purpose:Calculation CheckSum value
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int32_t MS_CalcCheckSum(int32_t BaseAddr)
{
	int32_t i,baseAddr32; uint32_t wData;
	FlashObj.BaseAddr = BaseAddr;

	MS_ReadFlashRom(FlashObj.BaseAddr,sizeof(FlashObj.FwHeader),(uint8_t *)&(FlashObj.FwHeader));

	/* Check Signature */
	if (!MS_CheckSignature((uint8_t *)&FlashObj.FwHeader.Signature))
	{
#ifdef MC_DBG_PARAM_ENABLE
		foeERR++;
#endif
		return(0);
	}
	FlashObj.FileSize=MS_Swap(FlashObj.FwHeader.FileLen)-sizeof(FlashObj.FwHeader);
	FlashObj.FileSize_div_4=FlashObj.FileSize>>2;
	FlashObj.CalcChecksum32=0;
	baseAddr32 = FlashObj.BaseAddr + sizeof(FlashObj.FwHeader);

	/* Read 4 byte check every time */
	for (i=0; i<FlashObj.FileSize_div_4; i++,baseAddr32+=4)
	{
		MS_ReadFlashRom(baseAddr32,4,(uint8_t *)&wData);
		FlashObj.CalcChecksum32+=wData;

		/* CalcChecksum32 Overflow pluse 1 */
		if (FlashObj.CalcChecksum32<wData)
		{
			FlashObj.CalcChecksum32++;
		}
	}

	/* Under 4byte calculation checksum */
	if (FlashObj.FileSize%4)
	{
		wData=0;
		memcpy((uint8_t *)&wData,(uint8_t *)baseAddr32,FlashObj.FileSize%4);
		FlashObj.CalcChecksum32+=wData;
		if (FlashObj.CalcChecksum32<wData)
		{
			FlashObj.CalcChecksum32++;
		}
	}
	return(FlashObj.CalcChecksum32== ~(FlashObj.FwHeader.Checksum32)&0xffffffff);
} /* End of MS_CalcCheckSum() */

static int   nLine=0;
static char  sLine[128];
/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_GetLineData()
 * Purpose: Read single line command
 * Params:
 * Returns:
 * Note:Parameter file decoding:
 *      1. Assuming a single-line instruction <128B, ending with <cr> or <lf>
 *      2. Decode after reading 128B each time
 *      3. Execute all single-line instructions, only keep the last less than one line string
 *      Repeat step 2 until the end of the file
 *      4. Execute the last line of instructions before the end
 *      The single-line instruction format is:
 *      m0.map.<idx>.L=d,d,...
 * ----------------------------------------------------------------------------
 */
static int32_t MS_GetLineData(void)
{
	int32_t i,k,skip;
	int8_t c;

	nLine=0;
	for (i=0; i<FlashObj.BufferLen; i++)
	{
		c=FlashObj.Buffer[i];

		/* \r = enter */
		if (c=='\n' || c=='\r' || c==';')
		{
			if (nLine)
			{
				break;
			}
			continue;
		}
		/* \t = tab */
		if (c==' ' || c=='\t')
		{
			continue;
		}
		/* c = Annotation */
		if (c=='#')
		{
			for (i++,skip=0; i<FlashObj.BufferLen; i++)
			{
				c=FlashObj.Buffer[i];

				/* skip utf-8 */
				if (c>=128)
				{
					skip=1;
					continue;
				}
				if (skip)
				{
					skip=0;
					continue;
				}
				if (c=='\n' || c=='\r')
				{
					break;
				}
			}
			if (nLine)
			{
				break;
			}
			continue;
		}
		sLine[nLine++]=c;
	}
	sLine[nLine]=0;

	/* Keep the unfinished part */
	if (i>=FlashObj.BufferLen)
	{
		return(1);
	}

	for (k=0,i++; i<FlashObj.BufferLen; i++,k++)
	{
		FlashObj.Buffer[k]=FlashObj.Buffer[i];
	}

	FlashObj.BufferLen=k;
	/* Remove processed part */
	return(0);
} /* End of MS_GetLineData() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_UpdateMap()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void MS_UpdateMap(int last)
{
	for (int i=0; i<10 && FlashObj.BufferLen>0; i++)
	{
		/* Keep remaining strings */
		if (MS_GetLineData())
		{
			break;
		}
		if (nLine>0)
		{
			esc_doCommand(0,0,"put",sLine,nLine,0);
		}
	}

	/* Enforce the last line */
	if (last)
	{
		if (nLine>0)
		{
			esc_doCommand(0,0,"put",sLine,nLine,0);
		}
	}
} /* End of MS_UpdateMap() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MS_GetFlashParamsAreaData()
 * Purpose: Map Write To Flash Init
 * Params:
 * Returns:
 * Note:put("m@.foe.<idx>.B=d ..."): FoE Write
 *      get("m@.foe.<idx>.<sub>.B"): FoE Read
 *        idx=0:WRQ/RRQ, idx>0:DATA
 *        sub=0:program, sub>0:parameter
 * ----------------------------------------------------------------------------
 */
void MS_GetFlashParamsAreaData(void)
{
	int32_t i,DataLen,baseAddr32;

	if (MS_CalcCheckSum(PARAMS_BASE_ADDR1))
	{
		FlashObj.BaseAddr = PARAMS_BASE_ADDR1;
	}
	else if(MS_CalcCheckSum(PARAMS_BASE_ADDR2))
	{
		FlashObj.BaseAddr = PARAMS_BASE_ADDR2;
	}
	/* Checksum error */
	else
	{
#ifdef MC_DBG_PARAM_ENABLE
		ldpERR++;
#endif
		return;
	}

	/* File size oversized */
	if (FlashObj.FileSize>=PARAMS_MAX_SIZE)
	{
#ifdef MC_DBG_PARAM_ENABLE
		ldpERR++;
#endif
		return;
	}
	baseAddr32 = FlashObj.BaseAddr + sizeof(FlashObj.FwHeader);

	for (i = 0,FlashObj.BufferLen = 0; i < FlashObj.FileSize; i += DataLen,baseAddr32 += DataLen)
	{
		DataLen = FlashObj.FileSize - i;
		if (DataLen > 128)
		{
			DataLen = 128;
		}
		MS_ReadFlashRom(baseAddr32, DataLen, &FlashObj.Buffer[FlashObj.BufferLen]);
		FlashObj.BufferLen += DataLen;

		/* Activate update map */
		MS_UpdateMap(0);
	}

	/* Activate last data */
	MS_UpdateMap(1);
} /* End of MS_GetFlashParamsAreaData() */

/* End of flash.c */

