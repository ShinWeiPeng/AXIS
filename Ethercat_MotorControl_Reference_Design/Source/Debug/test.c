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
#include "NuMicro.h"
#include <string.h>
#include <stdlib.h>
#include "test.h"
#include "AX58200_Hw.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */
TEST_TIME				TestTime;

/* LOCAL VARIABLES DECLARATIONS */
static uint8_t TEST_SPI_DATA_SIZE_SELECT[] = {TEST_SPI_1BYTE_DATA, TEST_SPI_2BYTE_DATA, TEST_SPI_4BYTE_DATA};
static uint8_t TEST_SPI_ADDR_SIZE_SELECT[] = {TEST_SPI_2BYTE_ADDR, TEST_SPI_3BYTE_ADDR};

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: BSP_PDI_EscRead()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int BSP_PDI_EscRead(uint32_t addr, uint8_t *pbuf, uint32_t ByteSize)
{
	
	HW_EscRead((MEM_ADDR*)pbuf, (UINT16)addr, (UINT16)ByteSize);
	
	return 0;
} /* End of BSP_PDI_EscRead() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: BSP_PDI_EscWrite()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int BSP_PDI_EscWrite(uint32_t addr, uint8_t *pbuf, uint32_t ByteLen)
{
	
	HW_EscWrite((MEM_ADDR*)pbuf, (UINT16)addr, (UINT16)ByteLen);
	
	return 0;
} /* End of BSP_PDI_EscWrite() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TEST_Timer()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void TEST_Timer(void const *argument)
{
	uint32_t i;
	
	/* Run software time */
	if (TestTime.RunFlag)
	{
		if (++TestTime.TickCnt >= TEST_SEC_TICK_COUNT)
		{
			TestTime.TickCnt = 0;
			
			if (TestTime.Second < 59)
			{
				TestTime.Second++;
			}
			else if (TestTime.Minute < 59)
			{
				TestTime.Second = 0;
				TestTime.Minute++;
			}
			else if (TestTime.Hour < 23)
			{
				TestTime.Second = 0;	
				TestTime.Minute = 0;
				TestTime.Hour++;
			}
			else
			{
				TestTime.Second = 0;	
				TestTime.Minute = 0;
				TestTime.Hour = 0;		
				TestTime.Day++;
			}
		}
	}
	
	for (i=0; i<TEST_MAX_DOWN_COUNT; i++)
	{
		if (TestTime.DownCounter[i])
		{
			TestTime.DownCounter[i]--;
		}
	}
} /* End of TEST_Timer() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TEST_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void TEST_Init(void)
{
	/* Init. timer */
	TEST_ResetTime();
	
} /* End of TEST_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TEST_ResetTime()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void TEST_ResetTime(void) 
{
	TestTime.RunFlag = 0;
	TestTime.TickCnt = 0;	
	TestTime.Second = 0;
	TestTime.Minute = 0;
	TestTime.Hour = 0;
	TestTime.Day = 0;
	
} /* End of TEST_ResetTime() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TEST_CtrlTime()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void TEST_CtrlTime(uint8_t start) 
{
	TestTime.RunFlag = (start != 0) ? 1:0;
	
} /* End of TEST_CtrlTime() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TEST_GetTime()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
TEST_TIME* TEST_GetTime(void) 
{
	return (&TestTime);
} /* End of TEST_GetTime() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TEST_PatternInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t TEST_PatternInit(TEST_PATTERN_TEMP *pPattern, uint8_t PatternType, uint8_t *pInitData, uint32_t InitDataLen) 
{
	
	if (InitDataLen > TEST_INIT_DATA_BUF_SIZE)
	{
		pPattern->DataLen = TEST_INIT_DATA_BUF_SIZE;
	}
	else
	{
		pPattern->DataLen = InitDataLen;
	}
	memcpy(pPattern->Data, pInitData, pPattern->DataLen);
	pPattern->Type = PatternType;
	pPattern->Seed = 0;
	
	return 0;
} /* End of TEST_PatternInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TEST_PatternGenerator()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t TEST_PatternGenerator(TEST_PATTERN_TEMP *pPattern, uint8_t *pBuf, uint32_t Len) 
{
	uint32_t i;
		
	switch(pPattern->Type)
	{
	case TEST_PATTERN_FIXED:
		do
		{
			i = ((pPattern->DataLen > Len) ? Len : pPattern->DataLen);
			memcpy(pBuf, pPattern->Data, i);
			Len -= i;
			pBuf += i;
		} while (Len);
		break;
		
	case TEST_PATTERN_INCREMENT:
		for (i=0; i<Len; i++)
		{
			pBuf[i] = pPattern->Data[0]+i;
		}
		pPattern->Data[0] = pPattern->Data[0]+i;
		break;
		
	case TEST_PATTERN_DECREMENT:
		for (i=0; i<Len; i++)
		{
			pBuf[i] = pPattern->Data[0]-i;
		}
		pPattern->Data[0] = pPattern->Data[0]-i;		
		break;
		
	case TEST_PATTERN_RANDOM:
	default:
		srand((unsigned int)HW_GetTimer()+pPattern->Seed);		
		for (i=0; i<Len; i++)
		{
			pBuf[i] = (uint8_t)rand();							
		}					
		pPattern->Seed++;
		break;		
	}
	
	return 0;
} /* End of TEST_PatternGenerator() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: TEST_PdiMemoryTest()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int16_t TEST_PdiMemoryTest(TEST_CONTROL *pCtrl) 
{
		TEST_PARAMETER	*pParameter = (TEST_PARAMETER*)&pCtrl->Parameter;
		TEST_TEMP				*pTemp = (TEST_TEMP*)&pCtrl->Temp;
		TEST_RECORD			*pRecord = (TEST_RECORD*)&pCtrl->Record;
		uint32_t tmp;
	
		switch (pCtrl->State)
		{
		case TEST_IDLE:
			/* Reset temporary object and recorders */
			memset((uint8_t*)&pCtrl->Temp, 0, sizeof(pCtrl->Temp));
			memset((uint8_t*)&pCtrl->Record, 0, sizeof(pCtrl->Record));			
		
			/* Initiate DUT hardware */
		
			/* Initiate test timer */
			TEST_ResetTime();
			TEST_CtrlTime(1);			
			pCtrl->State = TEST_ROUND_START;
			break;
		
		case TEST_ROUND_START:
			/* Initiate test conditions */			
			srand((unsigned int)HW_GetTimer());
				
			/* Initiate data size of write operation */
			if (pParameter->Spis.DataSizeInWrite == TEST_SPI_DATA_SIZE_AUTO)
			{
				tmp = rand() % sizeof(TEST_SPI_DATA_SIZE_SELECT);
				pTemp->Spis.DataSizeInWrite = TEST_SPI_DATA_SIZE_SELECT[tmp];
			}
			else
			{
				pTemp->Spis.DataSizeInWrite = pParameter->Spis.DataSizeInWrite;
			}
			
			/* Initiate data size of read operation */
			if (pParameter->Spis.DataSizeInRead == TEST_SPI_DATA_SIZE_AUTO)
			{
				tmp = rand() % sizeof(TEST_SPI_DATA_SIZE_SELECT);
				pTemp->Spis.DataSizeInRead = TEST_SPI_DATA_SIZE_SELECT[tmp];
			}
			else
			{
				pTemp->Spis.DataSizeInRead = pParameter->Spis.DataSizeInRead;
			}
			
			/* Initiate address size both in write and read operations */
			if (pParameter->Spis.AddrSize == TEST_SPI_ADDR_SIZE_AUTO)
			{
				tmp = rand() % sizeof(TEST_SPI_ADDR_SIZE_SELECT);
				pTemp->Spis.AddrSizeInWrite = TEST_SPI_ADDR_SIZE_SELECT[tmp];
				tmp = rand() % sizeof(TEST_SPI_ADDR_SIZE_SELECT);
				pTemp->Spis.AddrSizeInRead = TEST_SPI_ADDR_SIZE_SELECT[tmp];
			}
			else
			{
				pTemp->Spis.AddrSizeInWrite = pParameter->Spis.AddrSize;
				pTemp->Spis.AddrSizeInRead = pParameter->Spis.AddrSize;						
			}
				
			pTemp->RandomCaseCounter[0] = 0;
			pTemp->RandomCaseCounter[1] = 0;				
			pTemp->RandomCaseCounter[2] = 0;				

			/* Initiate pattern generator */
			TEST_PatternInit(&pTemp->Pattern, pParameter->PatternType, pParameter->InitData, pParameter->InitDataLen);
			
			/* Initiate memory test area and size */
			pTemp->AddrOffset = pParameter->StartAddress;
			pTemp->RemainLen = (pParameter->EndAddress - pParameter->StartAddress) + 1;	
			
			/* Increase ROUND count */
			pRecord->RoundCnt++;
			pCtrl->State = TEST_RUN;			
			break;

		case TEST_RUN:
			/* Configure data size and addres size */

			/* Generate data pattern */
			pTemp->TxLen = (pTemp->RemainLen > TEST_TX_BUF_SIZE) ? TEST_TX_BUF_SIZE:pTemp->RemainLen;
			TEST_PatternGenerator(&pTemp->Pattern, pTemp->TxBuf,	pTemp->TxLen);

			/* Clear RX buffer */
			memset(pTemp->RxBuf, 0, pTemp->TxLen);
		
			/* Write to target */		
			BSP_PDI_EscWrite(pTemp->AddrOffset, pTemp->TxBuf, pTemp->TxLen);
			pTemp->RxLen = pTemp->TxLen;
				
			/* Read from target */
			BSP_PDI_EscRead(pTemp->AddrOffset, pTemp->RxBuf, pTemp->RxLen);					

			/* Check result */
			if (memcmp(pTemp->TxBuf, pTemp->RxBuf, pTemp->TxLen))
			{
				DBG_TriggerOutput(1);
				DBG_TriggerOutput(0);
				pRecord->ErrorCnt++;
				pCtrl->State = TEST_ERROR;
				break;				
			}

			/* Command termination check */
			if (pCtrl->Terminated)
			{
				pCtrl->State = TEST_EXIT;
				break;
			}
			
			pCtrl->State = TEST_RUN_WAIT;
			break;		
			
		case TEST_RUN_WAIT:
			pCtrl->State = TEST_RUN;
		
			/* Next test memory area */
			pTemp->AddrOffset += pTemp->TxLen;
			pTemp->RemainLen -= pTemp->TxLen;		
			if (pTemp->RemainLen)
			{
				break;
			}
			pTemp->AddrOffset = pParameter->StartAddress;
			pTemp->RemainLen = (pParameter->EndAddress - pParameter->StartAddress) + 1;
			
			pCtrl->State = TEST_CHANGE_CONDITION;
			break;		

		case TEST_CHANGE_CONDITION:
			pCtrl->State = TEST_RUN;
			
			/* Change data size of write operation */
			if (pParameter->Spis.DataSizeInWrite == TEST_SPI_DATA_SIZE_AUTO)
			{
				if ((pTemp->RandomCaseCounter[0]+1) < 5)
				{
					tmp = rand() % sizeof(TEST_SPI_DATA_SIZE_SELECT);
					pTemp->Spis.DataSizeInWrite = TEST_SPI_DATA_SIZE_SELECT[tmp];
					pTemp->RandomCaseCounter[0]++;
					break;
				}
			}
				
			/* Change data size of read operation */
			if (pParameter->Spis.DataSizeInRead == TEST_SPI_DATA_SIZE_AUTO)
			{
				if ((pTemp->RandomCaseCounter[1]+1) < 5)
				{					
					/* Reset precceding conditions */
					pTemp->RandomCaseCounter[0] = 0;
					
					tmp = rand() % sizeof(TEST_SPI_DATA_SIZE_SELECT);
					pTemp->Spis.DataSizeInRead = TEST_SPI_DATA_SIZE_SELECT[tmp];
					pTemp->RandomCaseCounter[1]++;
					break;
				}
			}
								
			/* Change address size */
			if (pParameter->Spis.AddrSize == TEST_SPI_ADDR_SIZE_AUTO)
			{
				if ((pTemp->RandomCaseCounter[2]+1) < 5)
				{					
					/* Reset precceding conditions */
					pTemp->RandomCaseCounter[0] = 0;
					pTemp->RandomCaseCounter[1] = 0;
						
					tmp = rand() % sizeof(TEST_SPI_ADDR_SIZE_SELECT);
					pTemp->Spis.AddrSizeInWrite = TEST_SPI_ADDR_SIZE_SELECT[tmp];
					tmp = rand() % sizeof(TEST_SPI_ADDR_SIZE_SELECT);
					pTemp->Spis.AddrSizeInRead = TEST_SPI_ADDR_SIZE_SELECT[tmp];		
					pTemp->RandomCaseCounter[2]++;
					break;
				}
			}
			
			pCtrl->State = TEST_ROUND_DONE;		
			break;
		
		case TEST_ROUND_DONE:			
			/* Increase OK count */
			pRecord->OkCnt++;

			/* Maximum round count check */
			if (pParameter->LoopCount != 0 && pRecord->RoundCnt >= pParameter->LoopCount)
			{
				pCtrl->State = TEST_EXIT;
				break;
			}
		
			pCtrl->State = TEST_ROUND_START;
			break;	
		
		case TEST_EXIT:
			/* Stop DUT hardware */
		
			/* Recovery data size as before test */
		
		case TEST_ERROR:
			
			/* Stop test timer */	
			TEST_CtrlTime(0);			
			pCtrl->State = TEST_DONE;
			break;				
		}
		
	return 0;
} /* End of TEST_PdiMemoryTest() */

/* End of test.c */
