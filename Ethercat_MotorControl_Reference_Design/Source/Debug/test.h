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

#ifndef __TEST_H__
#define __TEST_H__

/* INCLUDE FILE DECLARATIONS */
#include "NuMicro.h"

/* NAMING CONSTANT DECLARATIONS */
#define TEST_SEC_TICK_COUNT			1000
#define TEST_INIT_DATA_BUF_SIZE	128
#define TEST_TX_BUF_SIZE				1024
#define TEST_RX_BUF_SIZE				TEST_TX_BUF_SIZE
#define TEST_MAX_DOWN_COUNT     6

#if (MC_DBGPIN)
#define DBG_TriggerOutput(x)            MCMISC_TOGGLE_GPIO_PIN(MC_DBG_PORT3, MC_DBG_PIN3)
#else
#define DBG_TriggerOutput(x) 
#endif

	/* Default value */
#define TEST_DEF_LOOP_COUNT							0
#define TEST_DEF_START_RAMADDR					0x1000
#define TEST_DEF_RAM_SIZE								0x0FFF
#define TEST_DEF_PATTERN_TYPE						TEST_PATTERN_RANDOM
#define TEST_DEF_LOG_UPDATE_TIME				500

#define TEST_DEF_SPI_DATA_SIZE_IN_WR		TEST_SPI_DATA_SIZE_AUTO
#define TEST_DEF_SPI_DATA_SIZE_IN_RD		TEST_SPI_4BYTE_DATA
#define TEST_DEF_SPI_WRITE_DUMMY_CYCLES	0
#define TEST_DEF_SPI_ADDR_SIZE					TEST_SPI_ADDR_SIZE_AUTO

#define TEST_AUTO_CHANGE_CONDITION   		0xff
#define TEST_MAX_RANDOM_COUNT           5

/* TYPE DECLARATIONS */
typedef enum{
	TEST_IDLE = 0,
	TEST_ROUND_START,
	TEST_BUSY,	
	TEST_RUN,
	TEST_RUN_WAIT,
	TEST_CHANGE_CONDITION,
	TEST_ROUND_DONE,
	TEST_EXIT,
	TEST_ERROR,	
	TEST_DONE,
} TEST_STATE;

typedef enum{
	TEST_PATTERN_FIXED = 0,
	TEST_PATTERN_INCREMENT,
	TEST_PATTERN_DECREMENT,	
	TEST_PATTERN_RANDOM,
	TEST_MAX_PATTERN_TYPE,	
} TEST_PATTERN_TYPE;

typedef enum
{
	TEST_SPI_DATA_SIZE_AUTO = 0,
	TEST_SPI_1BYTE_DATA     = 1,
	TEST_SPI_2BYTE_DATA     = 2,
	TEST_SPI_4BYTE_DATA     = 4,	
} TEST_SPI_DATA_SIZE_TYPE;

typedef enum
{
	TEST_SPI_ADDR_SIZE_AUTO = 0,	
	TEST_SPI_2BYTE_ADDR     = 2,
	TEST_SPI_3BYTE_ADDR     = 3,
} TEST_SPI_ADDR_SIZE_TYPE;

typedef struct{
  TEST_SPI_DATA_SIZE_TYPE  DataSizeInWrite;
  TEST_SPI_DATA_SIZE_TYPE  DataSizeInRead;	
  TEST_SPI_ADDR_SIZE_TYPE  AddrSize;		
} TEST_SPIS_PARAMETER;

typedef struct{
	uint32_t	LoopCount;
	uint32_t	LogUpdateTime;
	uint8_t   DashboardEnabled;
	
	/* PDI memory test parameters */
	uint32_t  StartAddress;
	uint32_t  EndAddress;	
	uint32_t  PatternType;
	uint32_t  Range;
	uint8_t   InitData[TEST_INIT_DATA_BUF_SIZE];	
	uint32_t	InitDataLen;
	
	TEST_SPIS_PARAMETER	Spis;
} TEST_PARAMETER;

typedef struct{
	uint32_t Second;
	uint32_t Minute;
	uint32_t Hour;
	uint32_t Day;
	uint32_t RunFlag;
	uint32_t TickCnt;	
	
	uint32_t DownCounter[TEST_MAX_DOWN_COUNT];	
} TEST_TIME;

typedef struct{
	uint32_t RoundCnt;
	uint32_t OkCnt;
	uint32_t ErrorCnt;
	uint8_t  *pErrMsg;
	
} TEST_RECORD;

typedef struct{
	uint32_t DataSizeInWrite;
	uint32_t DataSizeInRead;	
	uint32_t AddrSizeInWrite;
	uint32_t AddrSizeInRead;
} TEST_SPIS_TEMP;

typedef struct{
	uint32_t  Type;
	uint8_t 	Data[TEST_INIT_DATA_BUF_SIZE];	
	uint32_t	DataLen;	
	uint32_t	Seed;		
} TEST_PATTERN_TEMP;

typedef struct{
	/* General use */
	uint32_t	CtrlFlags;	
	uint8_t  	HostInterfaceErrorStatus;	
	uint32_t	RandomCaseCounter[TEST_MAX_RANDOM_COUNT];
	
	/* Memory test */	
	uint8_t	  TxBuf[TEST_TX_BUF_SIZE];
	uint32_t	TxLen;	
	uint8_t	  RxBuf[TEST_RX_BUF_SIZE];
	uint32_t	RxLen;
	uint32_t	RemainLen;
	uint32_t	AddrOffset;
	TEST_PATTERN_TEMP Pattern;
	
	/* SPI Slave test */	
	TEST_SPIS_TEMP Spis;
	
} TEST_TEMP;

typedef struct{
	void           *pDbgObj;
	uint32_t				State;	
	uint32_t				Terminated;
	TEST_RECORD			Record;
	TEST_PARAMETER	Parameter;
	TEST_TEMP				Temp;	
	
} TEST_CONTROL;

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
void TEST_Init(void);

void TEST_Timer(void const *argument);
void TEST_ResetTime(void);
void TEST_CtrlTime(uint8_t start);
TEST_TIME* TEST_GetTime(void);

int32_t TEST_PatternInit(TEST_PATTERN_TEMP *pPattern, uint8_t PatternType, uint8_t *pInitData, uint32_t InitDataLen);
int32_t TEST_PatternGenerator(TEST_PATTERN_TEMP *pPattern, uint8_t *pBuf, uint32_t Len);

	/* Test Process Funcrions */
int16_t TEST_PdiMemoryTest(TEST_CONTROL *pCtrl);

#endif /* __TEST_H__ */

/* End of test.h */
