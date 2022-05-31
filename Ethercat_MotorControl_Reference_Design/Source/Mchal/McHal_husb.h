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

#ifndef __MCHAL_HUSB_H__
#define __MCHAL_HUSB_H__

/* INCLUDE FILE DECLARATIONS */
#include "mc_cfg.h"

/* NAMING CONSTANT DECLARATIONS */
#define USB_VENDOR_ID			0x0B95
#define USB_PRODUCT_ID		0x5820
#define USB_TXBUF_SIZE		4096
#define USB_RXBUF_SIZE		16384

/* USB Endpoint Transfer Type */
#define EP_XFER_CTRL			0x00
#define EP_XFER_ISO				0x01
#define EP_XFER_BULK			0x02
#define EP_XFER_INTR      0x03
#define EP_DIR_IN         0x80
#define EP_DIR_OUT        0x00

/* USB Descriptor Type */
#define DESC_TYPE_DEVICE				0x01
#define DESC_TYPE_CONFIG    		0x02
#define DESC_TYPE_STRING    		0x03
#define DESC_TYPE_INTERFACE			0x04
#define DESC_TYPE_ENDPOINT  		0x05
#define DESC_TYPE_QUALIFIER 		0x06
#define DESC_TYPE_OTHER     		0x07
#define DESC_TYPE_CS_INTERFACE	0x24

/* USB Descriptor SubType */
#define SUBTYPE_HEADER_FUNC					0x00
#define SUBTYPE_CALL_MANAGE_FUNC		0x01
#define SUBTYPE_ACM_FUNC						0x02
#define SUBTYPE_UNION_FUNC					0x06

#define SET_LINE_CODE           0x20
#define GET_LINE_CODE           0x21
#define SET_CONTROL_LINE_STATE  0x22

#define CEP_MAX         64
#define EPA_MAX         1024
#define EPA_OMX         64
#define EPB_MAX         1024
#define EPB_OMX         64
#define EPC_MAX         64
#define EPC_OMX         64

#define CEP_BUF_BASE    0
#define CEP_BUF_LEN     CEP_MAX
#define EPA_BUF_BASE    0x200
#define EPA_BUF_LEN     EPA_MAX
#define EPB_BUF_BASE    0x600
#define EPB_BUF_LEN     EPB_MAX
#define EPC_BUF_BASE    0xA00
#define EPC_BUF_LEN     EPC_MAX

#define BULK_IP         0x01
#define BULK_OP         0x02
#define INTR_IP         0x03

/* TYPE DECLARATIONS */
typedef enum
{
	VCPSTE_NOT_INIT           = -1,	
	VCPSTE_RECV_CMD           = 0,
	VCPSTE_PARSE_CMD          = 1,
	VCPSTE_EXEC_CMD           = 2,
	VCPSTE_PREPARE_END_RESP   = 9,	
	VCPSTE_WAIT_END_RESP      = 10,	
} MH_VCP_STATE_E;

typedef struct
{
	int flgRxReady; 
	int flgTxReady;
	int RxPnt; 
	int TxPnt;
	int RxLength; 
	int TxLength;
	uint8_t RxBuffer[USB_RXBUF_SIZE];
	uint8_t TxBuffer[USB_TXBUF_SIZE];
} MH_VCP_OBJECT_T;

/* MACRO DECLARATIONS */

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
void MH_VCP_Init(void);
void MH_VCP_Run(void);

#endif /* __MCHAL_HUSB_H__ */

/* End of McHal_husb.h */
