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
#include "ax_uart.h"
#include "AX58200_Hw.h"

/* NAMING CONSTANT DECLARATIONS */
/* MACRO DECLARATIONS */
/* TYPE DECLARATIONS */
/* GLOBAL VARIABLES DECLARATIONS */
AX_UART_OBJECT	UART_Obj;

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: AX_UART_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int AX_UART_Init(void)
{
	UINT8 tmp8;	

	AX_UART_OBJECT *pUartHandle = &UART_Obj;
	
	/* Initializes UART transfer structure */
	memset (&UART_Obj, 0, sizeof(UART_Obj));
	UART_Obj.RxBufFree = AX_UART_RX_BUF_SIZE;	
	UART_Obj.TxBufFree = AX_UART_TX_BUF_SIZE;	
	UART_Obj.Instance = AX_UART_INSTANCE;
	
  /* Enable UART clock source */
	HW_UART_ClkSource(pUartHandle->Instance, ENABLE);

	/* Initializes UART module */
//	UART_DisableFlowCtrl(pUartHandle->Instance);


	/* Enable UART GPIO */
	HW_MultiFuncPins(AX_UART_GPIO_PORT, AX_UART_GPIO_TX_PIN|AX_UART_GPIO_RX_PIN, AX_UART_MULTI_FUNC);

	/* Enable UART RX interrupt */		
	while (UART_GET_INT_FLAG(pUartHandle->Instance, UART_INTSTS_RDAIF_Msk))
	{
		UART_Read(pUartHandle->Instance, &tmp8, 1);
	}
	UART_EnableInt(pUartHandle->Instance, UART_INTEN_RDAIEN_Msk);

	
	/* Enable NVIC for UART */
	NVIC_SetPriority(AX_UART_IRQ, NVIC_EncodePriority(AX_UART_INT_PRIORITY_GROUP, AX_UART_INT_PREEMPTION_PRIORITY, AX_UART_INT_SUB_PRIORITY));
	NVIC_EnableIRQ(AX_UART_IRQ);

	
	/* Enable UART module */	
	UART_Open(pUartHandle->Instance, AX_UART_BAUDRATE);	

	return 0;
} /* End of AX_UART_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: AX_UART_DeInit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int AX_UART_DeInit(void)
{
	/* Disable UART interrupt */	
	NVIC_DisableIRQ(AX_UART_IRQ);
	UART_DisableInt(UART_Obj.Instance, UART_INTEN_TXENDIEN_Msk | UART_INTEN_RDAIEN_Msk);

	
	/* Disable UART periphral */
	
	/* Disable UART module */
	UART_Close(UART_Obj.Instance);

	return 0;
} /* End of AX_UART_DeInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: AX_UART_IRQHandler()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void AX_UART_IRQHandler(void)
{
	uint32_t Flags, Control, tmp32;
	UART_T *pUartReg = UART_Obj.Instance;
	
	Flags   = pUartReg->INTSTS;
	Control = pUartReg->INTEN;

	if((Control & UART_INTEN_RDAIEN_Msk) && (Flags & (UART_INTSTS_RDAIF_Msk | UART_INTSTS_BUFERRIF_Msk)))	
	{
		/* RX IRQ handling */
		while(pUartReg->INTSTS & UART_INTSTS_RDAIF_Msk)
		{
			/* Read data from RX data reg. and clear RX interrupt flag */
			tmp32 = pUartReg->DAT;
			if (UART_Obj.RxBufFree != 0)
			{
				UART_Obj.RxBuf[UART_Obj.RxWrPtr] = (uint8_t)tmp32;
				UART_Obj.RxWrPtr ++;
				if(UART_Obj.RxWrPtr >= AX_UART_RX_BUF_SIZE)
				{
					UART_Obj.RxWrPtr = 0;
				}
				UART_Obj.RxBufFree--;
			}
		}
	}
	if((Control & UART_INTEN_TXENDIEN_Msk) && (Flags & UART_INTSTS_TXENDIF_Msk))
	{
		/* TX IRQ handling */		
		while(pUartReg->INTSTS & UART_INTSTS_TXENDIF_Msk)
		{
			/* Write TX data reg. and clear TX interrupt flag */
			pUartReg->DAT = UART_Obj.TxBuf[UART_Obj.TxRdPtr];
			UART_Obj.TxRdPtr++;
			if(UART_Obj.TxRdPtr >= AX_UART_TX_BUF_SIZE)
			{
				UART_Obj.TxRdPtr = 0;
			}
			UART_Obj.TxBufFree++;
			if (UART_Obj.TxBufFree >= AX_UART_TX_BUF_SIZE)
			{
				UART_Obj.TxBufFree = AX_UART_TX_BUF_SIZE;
				UART_DISABLE_INT(pUartReg, UART_INTEN_TXENDIEN_Msk);
				return;
			}
		}
	}
} /* End of AX_UART_IRQHandler() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: AX_UART_GetChar()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
char AX_UART_GetChar(void)
{
	char ch;

	if (UART_Obj.RxBufFree != AX_UART_RX_BUF_SIZE)
	{
		UART_DISABLE_INT(UART_Obj.Instance, UART_INTEN_RDAIEN_Msk);
		ch = UART_Obj.RxBuf[UART_Obj.RxRdPtr];
		UART_Obj.RxBufFree++;
		UART_Obj.RxRdPtr++;
		if(UART_Obj.RxRdPtr >= AX_UART_RX_BUF_SIZE)
		{
			UART_Obj.RxRdPtr = 0;
		}
		UART_ENABLE_INT(UART_Obj.Instance, UART_INTEN_RDAIEN_Msk);
		return ch;
	}
	else
	{
		return 0;
	}

} /* End of AX_UART_GetChar() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: AX_UART_PutChar()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
unsigned char AX_UART_PutChar(unsigned char ch)
{
	/* block mode */	
	while (UART_Obj.TxBufFree == 0)
	{
	}

	UART_DISABLE_INT(UART_Obj.Instance, UART_INTEN_TXENDIEN_Msk);
	UART_Obj.TxBuf[UART_Obj.TxWrPtr] = ch;
	UART_Obj.TxBufFree--;
	UART_Obj.TxWrPtr++;
	if (UART_Obj.TxWrPtr >= AX_UART_TX_BUF_SIZE)
	{
		UART_Obj.TxWrPtr = 0;
	}
	UART_ENABLE_INT(UART_Obj.Instance, UART_INTEN_TXENDIEN_Msk);

	return ch;

} /* End of AX_UART_PutChar() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: AX_UART_CheckTxBufEmpty()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int AX_UART_CheckTxBufEmpty(void)
{
	if (UART_Obj.TxBufFree >= AX_UART_TX_BUF_SIZE)
	{
		return 1;
	}
	return 0;

} /* End of AX_UART_CheckTxBufEmpty() */

/* End of ax_uart.c */
