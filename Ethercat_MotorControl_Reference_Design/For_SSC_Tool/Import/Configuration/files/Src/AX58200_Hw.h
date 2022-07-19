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

/**
 * \addtogroup AX58200 hardware header file
 * @{
 */

/**
\file AX58200_Hw.h
\brief AX58200_Hw function prototypes and defines

\version 1.0.0.0
 */

#ifndef __AX58200_HW_H__
#define __AX58200_HW_H__

/* INCLUDE FILE DECLARATIONS */
#include <stdio.h>
#include "NuMicro.h"
#include "esc.h"

/* NAMING CONSTANT DECLARATIONS */
	/* Debug or evaluation requirement */
#define AX58200_DEBUG_ENABLE        0

#if (AX58200_DEBUG_ENABLE)
#include "printd.h"
	#define DBG_PRINT               printd
#else
	#define DBG_PRINT(f, ...)       {}
#endif
	/* ESC EEPROM done check */
#define ESC_EEPDONE_CHECK_ENABLE    0

	/* ESC timer tick per ms */
#define ECAT_TIMER_INC_P_MS         1

	/* ESC alias register */
#define ESC_ALIAS_ADDRESS_OFFSET    0x0012
		
	/* ESC PID/VID/REV check */
#define ESC_PRODUCT_ID_REG          0x0E00
#define ESC_VENDOR_ID_REG           0x0E08
#define ESC_CHIP_REVISION           0x01
#define ESC_PRODUCT_ID              0x00058100
#define ESC_VENDOR_ID               0x00000B95
#define AX_INTF_EscRead(a,b,c)      HW_EscRead((MEM_ADDR*)b, (UINT16)a, (UINT16)c)
	/* ESC PHY enhancement */
#define ESC_PHY_ENHANCEMENT					1

	/* Interrupt, Sync0 and Syn1 */
/*
		Priority Group=0 -> 0bits: preemption-priority, 4bit: sub-priority
		Priority Group=1 -> 1bits: preemption-priority, 3bit: sub-priority
		Priority Group=2 -> 2bits: preemption-priority, 2bit: sub-priority
		Priority Group=3 -> 3bits: preemption-priority, 1bit: sub-priority
		Priority Group=4 -> 4bits: preemption-priority, 0bit: sub-priority		
*/		
#define HW_INT_PRIORITY_GROUP               3
#define HW_SYNCx_INT_PREEMPTION_PRIORITY    6
#define HW_SYNCx_INT_SUB_PRIORITY           0
#define HW_ALEVENT_INT_PREEMPTION_PRIORITY  HW_SYNCx_INT_PREEMPTION_PRIORITY
#define HW_ALEVENT_INT_SUB_PRIORITY         0
#define HW_TIMETASK_INT_PREEMPTION_PRIORITY HW_SYNCx_INT_PREEMPTION_PRIORITY
#define HW_TIMETASK_INT_SUB_PRIORITY        1
#define HW_SYNC0_PORT                       PE
#define HW_SYNC0_PIN                        BIT_13
#define HW_SYNC0_IRQ                        GPE_IRQn
#define HW_SYNC0_IRQHandler                 GPE_IRQHandler
#define HW_SYNC1_PORT                       PC
#define HW_SYNC1_PIN                        BIT_8
#define HW_SYNC1_IRQ                        GPC_IRQn
#define HW_ALEVENT_SYNC1_IRQHandler         GPC_IRQHandler

	/* Timer for tick or delay function use */
#define HW_TIMETICK_INSTANCE                TIMER2
#define HW_TIMETICK_SHIFT_BITS              6
#define HW_TIMETICK_PRESCALER               187 //For 12Mhz HXL clock source, 187 = 12000/64(right-shift 6bits).
#define HW_TIMETICK_MASK                    0xffffful
#define HW_TIMETICK_MAX_VALUE               (HW_TIMETICK_MASK>>HW_TIMETICK_SHIFT_BITS)

	/* Timer for periodic task execution */
#define HW_TIMETASK_INSTANCE                TIMER3
#define HW_TIMETASK_FREQUENCY               1000 //1khz task execution frequency
#define HW_TIMETASK_IRQHandler              TMR3_IRQHandler
#define HW_TIMETASK_IRQ                     TMR3_IRQn

	/* SPI PDI */
#define HW_SPI_MAX_DATA_FRAGMENT_SIZE       512
#define HW_SPI_DUMMY_BYTES                  4
#define HW_SPI_MAX_XFER_BUF_SIZE            (HW_SPI_DUMMY_BYTES + HW_SPI_MAX_DATA_FRAGMENT_SIZE)
#define HW_SPI_XFER_TIMEOUT                 10 //ms

	/* PDI configuration */
#define HW_SPI_ESC_BAUDRATE                 30000000 //Hz
#define HW_SPI_ESC_MULTI_FUNC               5
#define HW_SPI_ESC_INSTANCE                 SPI1
#define HW_SPI_ESC_CS_PORT                  PB
#define HW_SPI_ESC_CS_PIN                   BIT_2
#define HW_SPI_ESC_SCLK_PORT                PB
#define HW_SPI_ESC_SCLK_PIN                 BIT_3
#define HW_SPI_ESC_MISO_PORT                PB
#define HW_SPI_ESC_MISO_PIN                 BIT_5
#define HW_SPI_ESC_MOSI_PORT                PB
#define HW_SPI_ESC_MOSI_PIN                 BIT_4
#define HW_ALEVENT_PORT                     PC
#define HW_ALEVENT_PIN                      BIT_6
#define HW_ALEVENT_IRQ                      GPC_IRQn

#if (ESC_EEPDONE_CHECK_ENABLE)
	/* EEPROM done check pin */
	#define HW_EEPROM_RELOAD_TIMEOUT          10000//unit in 1ms
	#define HW_EEPROM_PORT                    PB
	#define HW_EEPROM_PIN                     BIT_14
#endif //#if (ESC_EEPDONE_CHECK_ENABLE)

	/* SPI command definition */
#define HW_SPI_NOP_CMD                      0x00
#define HW_SPI_READ_CMD                     0x02
#define HW_SPI_READ_WITH_WAIT_CMD           0x03
#define HW_SPI_WRITE_CMD                    0x04
#define HW_SPI_ADDR_EXT_CMD                 0x06
#define HW_SPI_CMD_MASK                     0x07

/* TYPE DECLARATIONS */
typedef union
{
	UINT8	Byte[2];
	UINT16	Word;
} UALEVENT;

typedef union
{
	UINT32	d32[2];
	struct{
		UINT32 vendor_id:      32; /* Bit31:0, 0x0000 0b95 */
		UINT32 reserved_63_32: 32; /* Bit63:32 */
	} b;
} oIC_VENDOR_ID;

typedef union
{
	UINT32	d32[2];
	struct{
		UINT32 chip_revision:	 8;  /* Bit7:0, 0x01 */
		UINT32 package_type:	 4;	 /* Bit11:8, 0x00 */
		UINT32 product_id:		 20; /* Bit31:12, 0x5 8100 */
		UINT32 reserved_63_32: 32; /* Bit63:32 */
	} b;
} oIC_PRODUCT_ID;

#if (AX58200_DEBUG_ENABLE)
#define DBG_TMRTASK_PORT								PE
#define DBG_TMRTASK_PIN									BIT_0
#define DBG_CYCLE_TOO_SMALL_PORT				PE
#define DBG_CYCLE_TOO_SMALL_PIN					BIT_2
#define DBG_SYNCERR_PORT								PE
#define DBG_SYNCERR_PIN									BIT_3
#define DBG_ALEVENT_ISR_PORT						PE
#define DBG_ALEVENT_ISR_PIN							BIT_5
#define DBG_SYNC0ISR_PORT								PE
#define DBG_SYNC0ISR_PIN								BIT_6
#define DBG_SYNC1ISR_PORT								PE
#define DBG_SYNC1ISR_PIN								BIT_7
#endif //#if (AX58200_DEBUG_ENABLE)

typedef struct{
	UINT32	EscIsrCnt;
	UINT32	FunIsrCnt;
	UINT32	Sync0IsrCnt;
	UINT32	Sync1IsrCnt;
	UINT32	AdcTrgCnt;	
	UINT32	TIM1IsrCnt;
	UINT32	EncIsrCnt;	
	UINT32	TmrTskIsrCnt;	
} HW_DEBUG;

typedef struct{
	SPI_T*		pInst;
	GPIO_T*		pCsPort;
	UINT32		CsPin;
	__IO UINT8	Lock;
	UINT32		ReentryCnt;
	UINT8		ThreeByteAddrMode;
	UINT8		TxBuf[HW_SPI_MAX_XFER_BUF_SIZE];
	UINT8		RxBuf[HW_SPI_MAX_XFER_BUF_SIZE];
} HW_SPI_OBJECT;

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
/**
  * @brief  Disable interrupt source INT1
  */
#ifndef DISABLE_ESC_INT
	#define DISABLE_ESC_INT()
#endif //#ifndef DISABLE_ESC_INT

/**
  * @brief  ENABLE interrupt source INT1
  */
#ifndef ENABLE_ESC_INT
	#define ENABLE_ESC_INT()
#endif //#ifndef ENABLE_ESC_INT

/**
  * @brief  Access to the hardware timer
  */
#ifndef HW_GetTimer
#define HW_GetTimer()          ((UINT16)((HW_TIMETICK_INSTANCE->CNT & HW_TIMETICK_MASK)>>HW_TIMETICK_SHIFT_BITS))
#endif //#ifndef HW_GetTimer

/**
  * @brief  Clear the hardware timer
  */
#ifndef HW_ClearTimer
#define HW_ClearTimer()			TIMER_ResetCounter(HW_TIMETICK_INSTANCE)
#endif //#ifndef HW_ClearTimer

/**
  * @brief  No interrupts are supported so the default Get AL Event register (0x220) function is used
  */
#if !(INTERRUPTS_SUPPORTED)
#define HW_GetALEventRegister_Isr	HW_GetALEventRegister
#endif //#if !(INTERRUPTS_SUPPORTED)

/**
  * @brief  16Bit ESC read access
  */
#define HW_EscReadWord(WordValue, Address) HW_EscRead(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2)

/**
  * @brief  32Bit ESC read access
  */
#define HW_EscReadDWord(DWordValue, Address) HW_EscRead(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4)

/**
  * @brief  8Bit ESC read access
  */
#if !(ESC_16BIT_ACCESS)
#define HW_EscReadByte(ByteValue, Address) HW_EscRead(((MEM_ADDR *)&(ByteValue)),((UINT16)(Address)),1)
#endif //#if !(ESC_16BIT_ACCESS)

/**
  * @brief  The mailbox data is stored in the local uC memory therefore the default read function is used.
  */
#define HW_EscReadMbxMem(pData,Address,Len) HW_EscRead(((MEM_ADDR *)(pData)),((UINT16)(Address)),(Len))

#if (INTERRUPTS_SUPPORTED)
/**
  * @brief  Interrupt specific 16Bit ESC read access
  */
#define HW_EscReadWordIsr(WordValue, Address) HW_EscReadIsr(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2)

/**
  * @brief  Interrupt specific 32Bit ESC read access
  */
#define HW_EscReadDWordIsr(DWordValue, Address) HW_EscReadIsr(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4)

/**
  * @brief  Interrupt specific 8Bit ESC read access
  */
#if !(ESC_16BIT_ACCESS)
#define HW_EscReadByteIsr(ByteValue, Address) HW_EscReadIsr(((MEM_ADDR *)&(ByteValue)),((UINT16)(Address)),1)
#endif //#if !(ESC_16BIT_ACCESS)

#else //!(INTERRUPTS_SUPPORTED)

/**
  * @brief  No Interrupts are supported so the generic ESC write access function is used
  */
#define HW_EscReadIsr(pData, Address, Len ) HW_EscRead(((MEM_ADDR *)(pData)),((UINT16)(Address)),(Len))

/**
  * @brief  No Interrupts are used so the generic (16Bit) ESC read access function is used
  */
#define HW_EscReadWordIsr(WordValue, Address) HW_EscRead(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2)

/**
  * @brief  No Interrupts are used so the generic (32Bit) ESC read access function is used
  */
#define HW_EscReadDWordIsr(DWordValue, Address) HW_EscRead(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4)

/**
  * @brief  No Interrupts are used so the generic (8Bit) ESC read access function is used
  */
#if !(ESC_16BIT_ACCESS)
#define HW_EscReadByteIsr(ByteValue, Address) HW_EscRead(((MEM_ADDR *)&(ByteValue)),((UINT16)(Address)),1)
#endif //#if !(ESC_16BIT_ACCESS)

#endif //#if (INTERRUPTS_SUPPORTED)

/**
  * @brief  16Bit ESC write access
  */
#define HW_EscWriteWord(WordValue, Address) HW_EscWrite(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2)

/**
  * @brief  32Bit ESC write access
  */
#define HW_EscWriteDWord(DWordValue, Address) HW_EscWrite(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4)

/**
  * @brief  8Bit ESC write access
  */
#if !(ESC_16BIT_ACCESS)
#define HW_EscWriteByte(ByteValue, Address) HW_EscWrite(((MEM_ADDR *)&(ByteValue)),((UINT16)(Address)),1)
#endif //#if !(ESC_16BIT_ACCESS)

/**
  * @brief  The mailbox data is stored in the local uC memory therefore the default write function is used.
  */
#define HW_EscWriteMbxMem(pData,Address,Len) HW_EscWrite(((MEM_ADDR *)(pData)),((UINT16)(Address)),(Len))

#if (INTERRUPTS_SUPPORTED)
/**
  * @brief  Interrupt specific 16Bit ESC write access
  */
#define HW_EscWriteWordIsr(WordValue, Address) HW_EscWriteIsr(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2)

/**
  * @brief  Interrupt specific 32Bit ESC write access
  */
#define HW_EscWriteDWordIsr(DWordValue, Address) HW_EscWriteIsr(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4)

/**
  * @brief  Interrupt specific 8Bit ESC write access
  */
#if !(ESC_16BIT_ACCESS)
#define HW_EscWriteByteIsr(ByteValue, Address) HW_EscWriteIsr(((MEM_ADDR *)&(ByteValue)),((UINT16)(Address)),1)
#endif //#if !(ESC_16BIT_ACCESS)

#else //!(INTERRUPTS_SUPPORTED)

/**
  * @brief  No Interrupts are supported so the generic ESC write access function is used
  */
#define HW_EscWriteIsr(pData, Address, Len ) HW_EscWrite(((MEM_ADDR *)(pData)),((UINT16)(Address)),(Len))

/**
  * @brief  No Interrupts are used so the generic (16Bit) ESC write access function is used
  */
#define HW_EscWriteWordIsr(WordValue, Address) HW_EscWrite(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2)

/**
  * @brief  No Interrupts are used so the generic (32Bit) ESC write access function is used
  */
#define HW_EscWriteDWordIsr(DWordValue, Address) HW_EscWrite(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4)

/**
  * @brief  No Interrupts are used so the generic (8Bit) ESC write access function is used
  */
#if !(ESC_16BIT_ACCESS)
#define HW_EscWriteByteIsr(ByteValue, Address) HW_EscWrite(((MEM_ADDR *)&(ByteValue)),((UINT16)(Address)),1)
#endif //#if !(ESC_16BIT_ACCESS)
#endif //#if (INTERRUPTS_SUPPORTED)

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
#define PROTO
PROTO UINT8 HW_Init(void);
PROTO void HW_Release(void);
PROTO UINT16 HW_GetALEventRegister(void);

#if (INTERRUPTS_SUPPORTED)
PROTO UINT16 HW_GetALEventRegister_Isr(void);
#endif //#if (INTERRUPTS_SUPPORTED)

PROTO void HW_EscRead( MEM_ADDR * pData, UINT16 Address, UINT16 Len );
#if (INTERRUPTS_SUPPORTED)
PROTO void HW_EscReadIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len );
#endif //#if (INTERRUPTS_SUPPORTED)

PROTO void HW_EscWrite( MEM_ADDR *pData, UINT16 Address, UINT16 Len );

#if (INTERRUPTS_SUPPORTED)
PROTO void HW_EscWriteIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len );
#endif //#if (INTERRUPTS_SUPPORTED)

#if (BOOTSTRAPMODE_SUPPORTED)
PROTO void HW_RestartTarget(void);
#endif //#if (BOOTSTRAPMODE_SUPPORTED)

#if (ESC_EEPROM_EMULATION)
PROTO UINT16 HW_EepromReload(void);
#endif //#if (ESC_EEPROM_EMULATION)

PROTO UINT32 HW_GPIO_PinToIntrNum(UINT32 Pin);
PROTO INT32 HW_MultiFuncPins(GPIO_T* Instance, UINT32 Pins, UINT8 MultiFuncValue);
PROTO void HW_TMR_ClkSource(TIMER_T* Instance, UINT32 NewState);
PROTO void HW_SPI_ClkSource(SPI_T* Instance, UINT8 NewState);
PROTO void HW_UART_ClkSource(UART_T* Instance, UINT8 NewState);
PROTO void HW_GPIO_WritePin(GPIO_T* Instance, UINT32 Pin, UINT8 NewState);
PROTO void HW_GPIO_TogglePin(GPIO_T* Instance, UINT32 Pin);
PROTO UINT8 HW_GPIO_ReadPin(GPIO_T* Instance, UINT32 Pin);
#undef PROTO

UINT8 HW_CheckTimeout(UINT16 StartTime, UINT16 Timeout);
HW_DEBUG* HW_GetDebugCounter(void);

#endif /* __AX58200_HW_H__ */

/* End of AX58200_Hw.h */
/** @}*/
