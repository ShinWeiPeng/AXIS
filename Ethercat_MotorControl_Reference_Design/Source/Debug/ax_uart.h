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

#ifndef __AX_UART_H__
#define __AX_UART_H__

/* INCLUDE FILE DECLARATIONS */
#include "NuMicro.h"

/* NAMING CONSTANT DECLARATIONS */
#define AX_UART_TX_BUF_SIZE                1024
#define AX_UART_RX_BUF_SIZE                512
#define AX_UART_BAUDRATE                   115200
#define AX_UART_INT_PRIORITY_GROUP         3
#define AX_UART_INT_PREEMPTION_PRIORITY    7
#define AX_UART_INT_SUB_PRIORITY           1

#define AX_UART_INSTANCE                   UART0
#define AX_UART_IRQHandler                 UART0_IRQHandler
#define AX_UART_MULTI_FUNC                 7
#define AX_UART_IRQ                        UART0_IRQn
#define AX_UART_GPIO_PORT                  PA
#define AX_UART_GPIO_TX_PIN                BIT_7
#define AX_UART_GPIO_RX_PIN                BIT_6

/* TYPE DECLARATIONS */
typedef struct{
	UART_T*                  Instance;
	
	unsigned char            RxBuf[AX_UART_RX_BUF_SIZE];
	volatile unsigned long   RxWrPtr;
	volatile unsigned long   RxRdPtr;
	volatile unsigned long   RxBufFree;
	
	unsigned char            TxBuf[AX_UART_TX_BUF_SIZE];
	volatile unsigned long   TxWrPtr;
	volatile unsigned long   TxRdPtr;
	volatile unsigned long   TxBufFree;	
} AX_UART_OBJECT;
																								
/* GLOBAL VARIABLES */
/* EXPORTED SUBPROGRAM SPECIFICATIONS */ 
int AX_UART_Init(void);
int AX_UART_DeInit(void);
char AX_UART_GetChar(void);
unsigned char AX_UART_PutChar(unsigned char);
int AX_UART_CheckTxBufEmpty(void);
#endif /* __AX_UART_H__ */

/* End of ax_uart.h */
