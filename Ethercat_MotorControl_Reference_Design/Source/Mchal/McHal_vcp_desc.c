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
//#include <string.h>
#include "McHal_husb.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* MACRO DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* 
	USB Descriptor Declaration for CDC Class
	Line coding structure
  0-3 dwDTERate    Data terminal rate (baudrate), in bits per second
  4   bCharFormat  Stop bits: 0 - 1 Stop bit, 1 - 1.5 Stop bits, 2 - 2 Stop bits
  5   bParityType  Parity:    0 - None, 1 - Odd, 2 - Even, 3 - Mark, 4 - Space
  6   bDataBits    Data bits: 5, 6, 7, 8, 16
*/

/* USB Device Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t DeviceDescriptor[] =
#else
static uint8_t DeviceDescriptor[] __attribute__((aligned(4))) =
#endif
{
	0x12,				//bLength
	DESC_TYPE_DEVICE,  	//bDescriptorType
	0x00,0x02,	//bcdUSB
	0x02,				//bDeviceClass
	0x00,				//bDeviceSubClass
	0x00,				//bDeviceProtocol
	CEP_MAX,		//bMaxPacketSize0
	USB_VENDOR_ID&0xFF,((USB_VENDOR_ID>>8)&0xFF),	//idVendor
	USB_PRODUCT_ID&0xFF,((USB_PRODUCT_ID>>8)&0xFF),	//iProduct
	0x00,0x03,	//bcdDevice
	0x01,				//iManufacturer
	0x02,				//iProduct
	0x03,				//iSerialNumber
	0x01				//bNumConfigruation
};

/* USB Qualifier Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t QualifierDescriptor[] =
#else
static uint8_t QualifierDescriptor[] __attribute__((aligned(4))) =
#endif
{
	0x0A,				//bLength
	DESC_TYPE_QUALIFIER,	//bDescriptorType
	0x00,0x02,	//bcdUSB
	0x00,				//bDeviceClass
	0x00,				//bDeviceSubClass
	0x00,				//bDeviceProtocol
	CEP_MAX,		//bMaxPacketSize0
	0x01,				//bNumConfigurations
	0x00				//bReserved
};

/* USB Configure Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t ConfigDescriptor[] =
#else
static uint8_t ConfigDescriptor[] __attribute__((aligned(4))) =
#endif
{
	0x09,				//bLength
	DESC_TYPE_CONFIG,		//bDescriptorType
	0x43,0x00,	//wTotalLength
	0x02,				//bNumInterfaces
	0x01,				//bConfigurationValue
	0x00,				//iConfiguration
	0xC0,				//bmAttributes
	0x32,				//bMaxPower
	
	/* Inerface Descriptor[0] */
	0x09,				//bLength
	DESC_TYPE_INTERFACE,	//bDescriptorType
	0x00,				//bInterfaceNumber
	0x00,				//bAlternateSetting
	0x01,				//bNumEndpoints
	0x02,				//bInterfaceClass
	0x02,				//bInterfaceSubClass
	0x01,				//bInterfaceProtocol
	0x00,				//iInterface

	/* Header Functional Descriptor */
	0x05,				//bFuncoLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_HEADER_FUNC,			//bDescriptorSubtype
	0x10, 0x01,	//Compliant to ver.1.10

	/* Call Management Functional Descriptor */
	0x05,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_CALL_MANAGE_FUNC,	//bDescriptorSubtype
	0x00,				//bmCapabilities
	0x01,				//bDataInterface

	/* Union Functional Descriptor */	
	0x04,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_ACM_FUNC,					//bDescriptorSubtype
	0x00,				//bmCapabilities
		 
	/* Abstract Control Management Functional Descriptor */			 
	0x05,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_UNION_FUNC,				//bDescriptorSubType
	0x00,				//bMasterInterface
	0x01,				//bSlaveInterface0
	
		/* Endpoint Descriptor */		 
	0x07,							//bLength
	DESC_TYPE_ENDPOINT,		//bDescriptorType
	(EP_DIR_IN|INTR_IP),	//bEndpointAddress
	EP_XFER_INTR,					//bmAttributes
	EPC_MAX&0xFF, (EPC_MAX>>8)&0xFF,//wMaxPacketSize
	0x01,							//bInterval
		 
	/* Inerface Descriptor[1] */		 
	0x09,				//bLength
	DESC_TYPE_INTERFACE,	//bDescriptorType
	0x01,				//bInterfaceNumber
	0x00,				//bAlternateSetting
	0x02,				//bNumEndpoints
	0x0A,				//bInterfaceClass
	0x00,				//bInterfaceSubClass
	0x00,				//bInterfaceProtocol
	0x00,				//iInterface

		/* Endpoint Descriptor[0] */
	0x07,								//bLength
	DESC_TYPE_ENDPOINT,	//bDescriptorType
	(EP_DIR_IN|BULK_IP),//bEndpointAddress
	EP_XFER_BULK,				//bmAttributes
	EPA_MAX&0xFF, (EPA_MAX>>8)&0xFF,//wMaxPacketSize
	0x00,								//bInterval
	
		/* Endpoint Descriptor[1] */
	0x07,								//bLength
	DESC_TYPE_ENDPOINT,	//bDescriptorType
	(EP_DIR_OUT|BULK_OP),	//bEndpointAddress
	EP_XFER_BULK,				//bmAttributes
	EPB_MAX&0xFF, (EPB_MAX>>8)&0xFF,//wMaxPacketSize
	0x00								//bInterval
};

/* USB Other Speed Configure Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t OtherConfigDescriptorHS[] =
#else
static uint8_t OtherConfigDescriptorHS[] __attribute__((aligned(4))) =
#endif
{
	0x09,				//bLength
	DESC_TYPE_OTHER,			//bDescriptorType
	0x43, 0x00,	//wTotalLength
	0x02,				//bNumInterfaces
	0x01,				//bConfigurationValue
	0x00,				//iConfiguration
	0xC0,				//bmAttributes
	0x32,				//bMaxPower

	/* Inerface Descriptor[0] */
	0x09,				//bLength
	DESC_TYPE_INTERFACE,	//bDescriptorType
	0x00,				//bInterfaceNumber
	0x00,				//bAlternateSetting
	0x01,				//bNumEndpoints
	0x02,				//bInterfaceClass
	0x02,				//bInterfaceSubClass
	0x01,				//bInterfaceProtocol
	0x00,				//iInterface
	
	/* Header Functional Descriptor */
	0x05,				//bFuncoLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_HEADER_FUNC,			//bDescriptorSubtype
	0x10, 0x01,	//Compliant to ver.1.10

	/* Call Management Functional Descriptor */
	0x05,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_CALL_MANAGE_FUNC,	//bDescriptorSubtype
	0x00,				//bmCapabilities
	0x01,				//bDataInterface

	/* Union Functional Descriptor */	
	0x04,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_ACM_FUNC,					//bDescriptorSubtype
	0x00,				//bmCapabilities
		 
	/* Abstract Control Management Functional Descriptor */			 
	0x05,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_UNION_FUNC,				//bDescriptorSubType
	0x00,				//bMasterInterface
	0x01,				//bSlaveInterface0
	
		/* Endpoint Descriptor[0] */
	0x07,								//bLength
	DESC_TYPE_ENDPOINT,	//bDescriptorType
	(EP_DIR_IN|INTR_IP),//bEndpointAddress
	EP_XFER_INTR,				//bmAttributes
	EPC_OMX&0xFF,(EPC_OMX>>8)&0xFF,//wMaxPacketSize
	0x01,								//bInterval
	
	/* Inerface Descriptor[1] */	
	0x09,				//bLength
	DESC_TYPE_INTERFACE,	//bDescriptorType
	0x01,				//bInterfaceNumber
	0x00,				//bAlternateSetting
	0x02,				//bNumEndpoints
	0x0A,				//bInterfaceClass
	0x00,				//bInterfaceSubClass
	0x00,				//bInterfaceProtocol
	0x00,				//iInterface
	
		/* Endpoint Descriptor[0] */
	0x07,								//bLength
	DESC_TYPE_ENDPOINT,	//bDescriptorType
	(EP_DIR_IN|BULK_IP),//bEndpointAddress
	EP_XFER_BULK,				//bmAttributes
	EPA_OMX&0xFF,(EPA_OMX>>8)&0xFF,//wMaxPacketSize
	0x00,								//bInterval
	
		/* Endpoint Descriptor[1] */
	0x07,									//bLength
	DESC_TYPE_ENDPOINT,		//bDescriptorType
	(EP_DIR_OUT|BULK_OP),	//bEndpointAddress
	EP_XFER_BULK,					//bmAttributes
	EPB_OMX&0xFF,(EPB_OMX>>8)&0xFF,//wMaxPacketSize
	0x00							//bInterval
};

#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t ConfigDescriptorFS[] =
#else
static uint8_t ConfigDescriptorFS[] __attribute__((aligned(4))) =
#endif
{
	0x09,				//bLength
	DESC_TYPE_CONFIG,		//bDescriptorType
	0x43, 0x00,	//wTotalLength
	0x02,				//bNumInterfaces
	0x01,				//bConfigurationValue
	0x00,				//iConfiguration
	0xC0,				//bmAttributes
	0x32,				//bMaxPower
	
	/* Inerface Descriptor[0] */	
	0x09,					//bLength
	DESC_TYPE_INTERFACE,	//bDescriptorType
	0x00,				//bInterfaceNumber
	0x00,				//bAlternateSetting
	0x01,				//bNumEndpoints
	0x02,				//bInterfaceClass
	0x02,				//bInterfaceSubClass
	0x01,				//bInterfaceProtocol
	0x00,				//iInterface
	
	/* Header Functional Descriptor */
	0x05,				//bFuncoLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_HEADER_FUNC,			//bDescriptorSubtype
	0x10, 0x01,	//Compliant to ver.1.10

	/* Call Management Functional Descriptor */
	0x05,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_CALL_MANAGE_FUNC,	//bDescriptorSubtype
	0x00,				//bmCapabilities
	0x01,				//bDataInterface

	/* Union Functional Descriptor */	
	0x04,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_ACM_FUNC,					//bDescriptorSubtype
	0x00,				//bmCapabilities
		 
	/* Abstract Control Management Functional Descriptor */			 
	0x05,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_UNION_FUNC,				//bDescriptorSubType
	0x00,				//bMasterInterface
	0x01,				//bSlaveInterface0
	
		/* Endpoint Descriptor[0] */	
	0x07,				//bLength
	DESC_TYPE_ENDPOINT,				//bDescriptorType
	(EP_DIR_IN|INTR_IP),	//bEndpointAddress
	EP_XFER_INTR,					//bmAttributes
	EPC_OMX&0xFF,(EPC_OMX>>8)&0xFF,//wMaxPacketSize
	0x01,				//bInterval
	
	/* Inerface Descriptor[1] */
	0x09,				//bLength
	DESC_TYPE_INTERFACE,	//bDescriptorType
	0x01,				//bInterfaceNumber
	0x00,				//bAlternateSetting
	0x02,				//bNumEndpoints
	0x0A,				//bInterfaceClass
	0x00,				//bInterfaceSubClass
	0x00,				//bInterfaceProtocol
	0x00,				//iInterface
	
		/* Endpoint Descriptor[0] */
	0x07,							//bLength
	DESC_TYPE_ENDPOINT,		//bDescriptorType
	(EP_DIR_IN|BULK_IP),	//bEndpointAddress
	EP_XFER_BULK,			//bmAttributes
	EPA_OMX&0xFF,(EPA_OMX>>8)&0xFF,//wMaxPacketSize
	0x00,							//bInterval

		/* Endpoint Descriptor[1] */	
	0x07,							//bLength
	DESC_TYPE_ENDPOINT,		//bDescriptorType
	(EP_DIR_OUT|BULK_OP),	//bEndpointAddress
	EP_XFER_BULK,					//bmAttributes
	EPB_OMX&0xFF,(EPB_OMX>>8)&0xFF,//wMaxPacketSize
	0x00							//bInterval
};

#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t OtherConfigDescriptorFS[] =
#else
static uint8_t OtherConfigDescriptorFS[] __attribute__((aligned(4))) =
#endif
{
	0x09,				//bLength
	DESC_TYPE_OTHER,			//bDescriptorType
	0x43, 0x00,	//wTotalLength
	0x02,				//bNumInterfaces
	0x01,				//bConfigurationValue
	0x00,				//iConfiguration
	0xC0,				//bmAttributes
	0x32,				//bMaxPower

	/* Inerface Descriptor[0] */
	0x09,				//bLength
	DESC_TYPE_INTERFACE,	//bDescriptorType
	0x00,				//bInterfaceNumber
	0x00,				//bAlternateSetting
	0x01,				//bNumEndpoints
	0x02,				//bInterfaceClass
	0x02,				//bInterfaceSubClass
	0x01,				//bInterfaceProtocol
	0x00,				//iInterface
	
	/* Header Functional Descriptor */
	0x05,				//bFuncoLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_HEADER_FUNC,			//bDescriptorSubtype
	0x10, 0x01,	//Compliant to ver.1.10

	/* Call Management Functional Descriptor */
	0x05,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_CALL_MANAGE_FUNC,	//bDescriptorSubtype
	0x00,				//bmCapabilities
	0x01,				//bDataInterface

	/* Union Functional Descriptor */	
	0x04,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_ACM_FUNC,					//bDescriptorSubtype
	0x00,				//bmCapabilities
		 
	/* Abstract Control Management Functional Descriptor */			 
	0x05,				//bLength
	DESC_TYPE_CS_INTERFACE,		//bDescriptorType
	SUBTYPE_UNION_FUNC,				//bDescriptorSubType
	0x00,				//bMasterInterface
	0x01,				//bSlaveInterface0
	
		/* Endpoint Descriptor[0] */
	0x07,				//bLength
	DESC_TYPE_ENDPOINT,				//bDescriptorType
	(EP_DIR_IN|INTR_IP),	//bEndpointAddress
	EP_XFER_INTR,					//bmAttributes
	EPC_MAX&0xFF,(EPC_MAX>>8)&0xFF,//wMaxPacketSize
	0x01,							//bInterval
	
	/* Inerface Descriptor[1] */	
	0x09,				//bLength
	DESC_TYPE_INTERFACE,	//bDescriptorType
	0x01,				//bInterfaceNumber
	0x00,				//bAlternateSetting
	0x02,				//bNumEndpoints
	0x0A,				//bInterfaceClass
	0x00,				//bInterfaceSubClass
	0x00,				//bInterfaceProtocol
	0x00,				//iInterface
	
		/* Endpoint Descriptor[0] */	
	0x07,				//bLength
	DESC_TYPE_ENDPOINT,				//bDescriptorType
	(EP_DIR_IN|BULK_IP),	//bEndpointAddress
	EP_XFER_BULK,					//bmAttributes
	EPA_MAX&0xFF,(EPA_MAX>>8)&0xFF,//wMaxPacketSize
	0x00,							//bInterval
	
		/* Endpoint Descriptor[1] */	
	0x07,				//bLength
	DESC_TYPE_ENDPOINT,				//bDescriptorType
	(EP_DIR_OUT|BULK_OP),	//bEndpointAddress
	EP_XFER_BULK,					//bmAttributes
	EPB_MAX&0xFF,(EPB_MAX>>8)&0xFF,//wMaxPacketSize
	0x00							//bInterval
};

#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t stringLang[] =
#else
static uint8_t stringLang[] __attribute__((aligned(4))) =
#endif
{
	0x04,				//bLength
	DESC_TYPE_STRING,		//bDescriptorType
	0x09,				//wLANGID[0]
	0x04				//wLANGID[1]
};

#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t stringVendorDesc[] =
#else
static uint8_t stringVendorDesc[] __attribute__((aligned(4))) =
#endif
{
	0x24,				//bLength
	DESC_TYPE_STRING,		//bDescriptorType
	'A',0,'S',0,'I',0,'X',0,' ',0,'E',0,'l',0,'e',0,'c',0,'.',0,' ',0,'C',0,'o',0,'r',0,'p',0,'.',0,' ',0
};

#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t stringProductDesc[] =
#else
static uint8_t stringProductDesc[] __attribute__((aligned(4))) =
#endif
{
	0x20,				//bLength
	DESC_TYPE_STRING,		//bDescriptorType
	'S',0,'l',0,'a',0,'v',0,'e',0,' ',0,' ',0,'U',0,'S',0,'B',0,' ',0,'V',0,'C',0,'O',0,'M',0
};

#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t stringSerialNumberDesc[] =
#else
static uint8_t stringSerialNumberDesc[] __attribute__((aligned(4))) =
#endif
{
	0x10,				//bLength
	DESC_TYPE_STRING,		//bDescriptorType
	'S',0,'0',0,'0',0,'0',0,'0',0,'0',0,'2',0
};

static uint8_t *StringDescriptors[5] =
{
	stringLang,
	stringVendorDesc,
	stringProductDesc,
	stringSerialNumberDesc,
	NULL,
};

static uint8_t *HidReport[3] =
{
	NULL,
	NULL,
	NULL,
};

static uint32_t HidReportLength[3] =
{
	0,
	0,
	0,
};

S_HSUSBD_INFO_T sHSInfo =
{
	DeviceDescriptor,
	ConfigDescriptor,
	StringDescriptors,
	QualifierDescriptor,
	ConfigDescriptorFS,
	OtherConfigDescriptorHS,
	OtherConfigDescriptorFS,
	HidReport,
	HidReportLength,
	0,
	0,
};

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/* End of McHal_vcp_desc.c */
