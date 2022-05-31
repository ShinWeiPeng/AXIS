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
#include "clicmd.h"
#include "string.h"
#include <stdlib.h>
#include "test.h"
#include "AX58200_Hw.h"
#include "AX58200_MotorControl.h"
#include "mc.h"
#include "ax_esc_regs.h"

#ifdef MC_STACK_ENABLE
#include "printd.h"
#endif

/* NAMING CONSTANT DECLARATIONS */
#define IsDigit(x) ((x < 0x3a && x > 0x2f) ? 1 : 0)
#define STR_ENABLE		"Enable "
#define STR_DISABLE		"Disable"
#define STR_8_BITS		"8bits "
#define STR_16_BITS		"16bits"
#define STR_HIGH			"High"
#define STR_LOW				"Low "
#define STR_TRUE			"TRUE "
#define STR_FALSE			"FLASE"

#define CLICMD_CTRL_C	0x03
#define CLICMD_SPACE	0x20
#define CLICMD_STR_BUF_SIZE		512

/* MACRO DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */
extern BOOL bDcSyncActive;
extern BOOL bEscIntEnabled;
extern BOOL bEcatFirstOutputsReceived;
extern HW_SPI_OBJECT HW_SpiObj;
extern CiA402_AXIS_T CiA402Axis;

/* LOCAL VARIABLES DECLARATIONS */
static uint8_t StrBuf[MAX_TEMP_BUF_SIZE];
static TEST_CONTROL clicmd_testCtrl;

/* LOCAL SUBPROGRAM DECLARATIONS */
static uint8_t clicmd_DecText2Char(uint8_t *pbuf, uint8_t *pValue, uint8_t len);
static uint8_t clicmd_HexText2Char(uint8_t *pbuf, uint8_t *pValue, uint8_t len);
static uint8_t clicmd_DecText2Short(uint8_t *pbuf, uint16_t *pValue, uint8_t len);
static uint8_t clicmd_HexText2Short(uint8_t *pbuf, uint16_t *pValue, uint8_t len);
static uint8_t clicmd_DecText2Long(uint8_t *pbuf, uint32_t *pValue, uint8_t len);
static uint8_t clicmd_HexText2Long(uint8_t *pbuf, uint32_t *pValue, uint8_t len);
static uint8_t clicmd_DecText2SignLong(uint8_t *pbuf, int32_t *pValue, uint8_t len);
static void clicmd_ShowMemoryInHex8b(CONSOLE_Inst *pInst, uint8_t *pbuf, uint32_t len);
static void clicmd_ShowMemoryInHex16b(CONSOLE_Inst *pInst, uint16_t *pbuf, uint32_t len);
static int clicmd_SystemReboot(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_SystemRebootHelp(CONSOLE_Inst *inst);
static int clicmd_SystemRebootUsage(CONSOLE_Inst *inst);
static int clicmd_PdiRead(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_PdiReadHelp(CONSOLE_Inst *inst);
static int clicmd_PdiReadUsage(CONSOLE_Inst *inst);
static int clicmd_PdiWrite(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_PdiWriteHelp(CONSOLE_Inst *inst);
static int clicmd_PdiWriteUsage(CONSOLE_Inst *inst);
static void clicmd_ShowEscFeatures(CONSOLE_Inst *inst);
static void clicmd_ShowEscAlInfo(CONSOLE_Inst *inst);
static void clicmd_ShowEscWdInfo(CONSOLE_Inst *inst);
static void clicmd_ShowEscFmmuSmInfo(CONSOLE_Inst *inst);
static void clicmd_ShowEscDcInfo(CONSOLE_Inst *inst);

#if CLI_MCU_DBG_CMD_ENABLE
static int clicmd_ClkRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_ClkRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_ClkRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_SysRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_SysRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_SysRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_FmcRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_FmcRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_FmcRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_SpimRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_SpimRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_SpimRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_UARTxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_UARTxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_UARTxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_TMRxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_TMRxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_TMRxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_SPIxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_SPIxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_SPIxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_GPIOxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_GPIOxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_GPIOxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_NvicRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_NvicRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_NvicRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_EadcRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_EadcRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_EadcRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_QEIxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_QEIxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_QEIxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_ECAPxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_ECAPxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_ECAPxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_EPWMxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_EPWMxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_EPWMxRegisterUsage(CONSOLE_Inst *inst);
#endif /* End of CLI_MCU_DBG_CMD_ENABLE */

#if CLI_ESC_DBG_CMD_ENABLE
static int clicmd_EscRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_EscRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_EscRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_PdiMemoryTest(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_PdiMemoryTestHelp(CONSOLE_Inst *inst);
static int clicmd_PdiMemoryTestUsage(CONSOLE_Inst *inst);
static int clicmd_PdiReset(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_PdiResetHelp(CONSOLE_Inst *inst);
#endif /* End of CLI_ESC_DBG_CMD_ENABLE */

static int clicmd_EscStack(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_EscStackHelp(CONSOLE_Inst *inst);

static int clicmd_McStack(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_McStackHelp(CONSOLE_Inst *inst);
static int clicmd_McStackUsage(CONSOLE_Inst *inst);

static int clicmd_CiA402(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_CiA402Help(CONSOLE_Inst *inst);
static int clicmd_CiA402Usage(CONSOLE_Inst *inst);

CONSOLE_Account CLICMD_userTable[MAX_USER_ACCOUNT];

CONSOLE_CmdEntry CLICMD_userCmdTable[]=
{
	{";", NULL, NULL, 5},	
	{"reboot", clicmd_SystemReboot, clicmd_SystemRebootHelp, 5},
	
#if CLI_MCU_DBG_CMD_ENABLE
	{"sys", clicmd_SysRegister, clicmd_SysRegisterHelp, 5},	
	{"clk", clicmd_ClkRegister, clicmd_ClkRegisterHelp, 5},	
	{"fmc", clicmd_FmcRegister, clicmd_FmcRegisterHelp, 5},
	{"spim", clicmd_SpimRegister, clicmd_SpimRegisterHelp, 5},	
	{"uartx", clicmd_UARTxRegister, clicmd_UARTxRegisterHelp, 5},		
	{"tmrx", clicmd_TMRxRegister, clicmd_TMRxRegisterHelp, 5},
	{"spix", clicmd_SPIxRegister, clicmd_SPIxRegisterHelp, 5},		
	{"gpiox", clicmd_GPIOxRegister, clicmd_GPIOxRegisterHelp, 5},		
	{"nvic", clicmd_NvicRegister, clicmd_NvicRegisterHelp, 5},		
	{"eadc", clicmd_EadcRegister, clicmd_EadcRegisterHelp, 5},	
	{"qeix", clicmd_QEIxRegister, clicmd_QEIxRegisterHelp, 5},
	{"ecapx", clicmd_ECAPxRegister, clicmd_ECAPxRegisterHelp, 5},	
	{"epwmx", clicmd_EPWMxRegister, clicmd_EPWMxRegisterHelp, 5},		
#endif /* End of CLI_MCU_DBG_CMD_ENABLE */
	
#if CLI_ESC_DBG_CMD_ENABLE
	{"pdird", clicmd_PdiRead, clicmd_PdiReadHelp, 5}, 	
	{"pdiwr", clicmd_PdiWrite, clicmd_PdiWriteHelp, 5},	
	{"pdimt", clicmd_PdiMemoryTest, clicmd_PdiMemoryTestHelp, 5},		
	{"escreg", clicmd_EscRegister, clicmd_EscRegisterHelp, 5},	
	{"pdirst", clicmd_PdiReset, clicmd_PdiResetHelp, 5},
#endif /* End of CLI_ESC_DBG_CMD_ENABLE */

	{"esc", clicmd_EscStack, clicmd_EscStackHelp, 5},		
	{"mc", clicmd_McStack, clicmd_McStackHelp, 5},
	{"cia402", clicmd_CiA402, clicmd_CiA402Help, 5},		
};

/* LOCAL SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_DecText2Char
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static uint8_t clicmd_DecText2Char(uint8_t *pbuf, uint8_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 3))
	{
		return 0xFF;
	}

	while (len--)
	{
		*pValue *= 10;

		if ((*pbuf < 0x3A) && (*pbuf > 0x2F))
		{
			*pValue += (*pbuf - 0x30);
		}
		else
		{
			return 0xFF;
		}

		pbuf++;
	}

	if (*pValue > 255)
	{
		return 0xFF;
	}

	return 0;

} /* End of  clicmd_DecText2Char() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_HexText2Char
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static uint8_t clicmd_HexText2Char(uint8_t *pbuf, uint8_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 2))
	{
		return 0xFF;
	}

	while (len--)
	{
		*pValue *= 16;

		if ((*pbuf < 0x3A) && (*pbuf > 0x2F))
			*pValue += (*pbuf - 0x30);
		else if ((*pbuf < 0x47) && (*pbuf > 0x40))
			*pValue += (*pbuf - 0x37);
		else if ((*pbuf < 0x67) && (*pbuf > 0x60))
			*pValue += (*pbuf - 0x57);
		else
			return 0xFF;

		pbuf++;
	}

	return 0;

} /* End of  clicmd_HexText2Char() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_DecText2Short
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static uint8_t clicmd_DecText2Short(uint8_t *pbuf, unsigned short *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 5))
		return 0xff;

	while (len--)
	{
		*pValue *= 10;

		if ((*pbuf < 0x3a) && (*pbuf > 0x2f))
			*pValue += (*pbuf - 0x30);
		else
			return 0xff;

		pbuf++;
	}

	if (*pValue > 65535)
		return 0xff;

	return 0;

} /* End of  clicmd_DecText2Short() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_HexText2Short
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static uint8_t clicmd_HexText2Short(uint8_t *pbuf, uint16_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 4))
		return 0xff;

	while (len--)
	{
		*pValue *= 16;

		if ((*pbuf < 0x3a) && (*pbuf > 0x2f))
			*pValue += (*pbuf - 0x30);
		else if ((*pbuf < 0x47) && (*pbuf > 0x40))
			*pValue += (*pbuf - 0x37);
		else if ((*pbuf < 0x67) && (*pbuf > 0x60))
			*pValue += (*pbuf - 0x57);
		else
			return 0xff;

		pbuf++;
	}

	return 0;

} /* End of  clicmd_HexaText2Short() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_DecText2Long
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static uint8_t clicmd_DecText2Long(uint8_t *pbuf, uint32_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 10))
		return 0xff;

	while (len--)
	{
		*pValue *= 10;

		if ((*pbuf < 0x3a) && (*pbuf > 0x2f))
			*pValue += (*pbuf - 0x30);
		else
			return 0xff;

		pbuf++;
	}

	if (*pValue > 0xffffffff)
		return 0xff;

	return 0;

} /* End of  clicmd_DecText2Long() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_HexText2Long
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static uint8_t clicmd_HexText2Long(uint8_t *pbuf, uint32_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 8))
		return 0xff;

	while (len--)
	{
		*pValue *= 16;

		if ((*pbuf < 0x3a) && (*pbuf > 0x2f))
			*pValue += (*pbuf - 0x30);
		else if ((*pbuf < 0x47) && (*pbuf > 0x40))
			*pValue += (*pbuf - 0x37);
		else if ((*pbuf < 0x67) && (*pbuf > 0x60))
			*pValue += (*pbuf - 0x57);
		else
			return 0xff;

		pbuf++;
	}

	return 0;

} /* End of  clicmd_HexText2Long() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_DecText2SignLong
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static uint8_t clicmd_DecText2SignLong(uint8_t *pbuf, int32_t *pValue, uint8_t len)
{
	uint8_t negSign = 0, ret;
	uint32_t Value;
	
	*pValue = 0;

	if (*pbuf == '-')
	{
		len--;
		pbuf++;
		negSign=1;
	}
	
	ret = clicmd_DecText2Long(pbuf, &Value, len);
	*pValue = Value;
	if (negSign)
	{
		*pValue = (-1)*(*pValue);
	}
	return ret;

} /* End of  clicmd_DecText2SignLong() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowMemoryInHex8b()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void clicmd_ShowMemoryInHex8b(CONSOLE_Inst *inst, uint8_t *pbuf, uint32_t len)
{
	uint32_t i;
	
	for (i=0; i<len; i++)
	{
		CONSOLE_PutMessage(inst, "0x%02x%s", pbuf[i], ((i+1) == len) ? "\r\n":", ");
		if ((i%16)==15)
		{
			CONSOLE_PutMessage(inst, "\r\n");
		}
	}
	CONSOLE_PutMessage(inst, "\r\n");	
} /* End of clicmd_ShowMemoryInHex8b() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowMemoryInHex16b()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void clicmd_ShowMemoryInHex16b(CONSOLE_Inst *inst, uint16_t *pbuf, uint32_t WordLen)
{
	uint32_t i;
	
	for (i=0; i<WordLen; i++)
	{
		CONSOLE_PutMessage(inst, "0x%04x%s", pbuf[i], ((i+1) == WordLen) ? "\r\n":", ");
		if ((i%16)==15)
		{
			CONSOLE_PutMessage(inst, "\r\n");
		}
	}
	CONSOLE_PutMessage(inst, "\r\n");	
} /* End of clicmd_ShowMemoryInHex16b() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowEscFeatures()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void clicmd_ShowEscFeatures(CONSOLE_Inst *inst)
{
	char *pStr;
	uint8_t tmp8;
	uint16_t tmp16;	
	uint32_t i;
	oESC_CORE_BUILD oEscCoreBuild;
	oPORT_DESCRIPTOR oPortDescriptor;
	oESC_FEATURES_SUPPORTED oEscFeaturesSupported;
	oREG_WRITE_ENABLE       oRegWriteEnable;
	oREG_WRITE_PROTECTION   oRegWriteProtect;
	oESC_WRITE_ENABLE       oEscWriteEnable;
	oESC_WRITE_PROTECTION   oEscWriteProtect;
//	oESC_CONFIG             oEscConfig;
	oIC_PRODUCT_ID          oProductId;
	oIC_VENDOR_ID           oVendorId;
	
	CONSOLE_PutMessage(inst, "\r\nESC Features...\r\n");
	
	AX_INTF_EscRead(ESC_CORE_TYPE, &tmp8, 1);
	switch(tmp8)
	{
	case 0x11: pStr = "ET1100";	break;
	case 0x12: pStr = "ET1200";	break;
	case 0xC8: pStr = "AX58100";	break;		
	default: pStr = "Unsupported"; break;
	}
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Type = %s\r\n", ESC_CORE_TYPE, tmp8, pStr);

	AX_INTF_EscRead(ESC_CORE_MAJOR_REV_X, &tmp8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Revision\r\n", ESC_CORE_MAJOR_REV_X, tmp8);

	AX_INTF_EscRead(ESC_CORE_BUILD, &oEscCoreBuild.d16, 2);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Build\r\n", ESC_CORE_BUILD, tmp8);
	CONSOLE_PutMessage(inst, "      maintenance_ver_z: %u\r\n", oEscCoreBuild.b.maintenance_ver_z);	
	CONSOLE_PutMessage(inst, "      minor_ver_y: %u\r\n", oEscCoreBuild.b.minor_ver_y);	
	CONSOLE_PutMessage(inst, "      patch_level: %u\r\n", oEscCoreBuild.b.patch_level);		
	
	AX_INTF_EscRead(FMMUS_SUPPORTED, &tmp8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, FMMUS supporrted\r\n", FMMUS_SUPPORTED, tmp8);

	AX_INTF_EscRead(SYNC_MANAGERS_SUPPORTED, &tmp8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Sync managers supporrted\r\n", SYNC_MANAGERS_SUPPORTED, tmp8);

	AX_INTF_EscRead(RAM_SIZE, &tmp8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, RAM size\r\n", RAM_SIZE, tmp8);

	AX_INTF_EscRead(PORT_DESCRIPTOR, &oPortDescriptor.d8, 1);

	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Port descriptor\r\n", PORT_DESCRIPTOR, oPortDescriptor.d8);
	for (i=0; i<4; i++)
	{
		tmp8 = ((oPortDescriptor.d8 >> (2*i)) & 0x03);
		switch(tmp8)
		{
		case 0x00: pStr = "Not implemented";	break;
		case 0x01: pStr = "Not configured";	break;
		case 0x02: pStr = "EBUS";	break;		
		default:   pStr = "MII/RMII/RGMII"; break;
		}
		CONSOLE_PutMessage(inst, "      port%u_config: %s\r\n", i, pStr);		
	}
	
	AX_INTF_EscRead(ESC_FEATURES_SUPPORTED, &oEscFeaturesSupported.d16, 2);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, ESC features supported\r\n", ESC_FEATURES_SUPPORTED, oEscFeaturesSupported.d16);
	CONSOLE_PutMessage(inst, "      fmmu_operation: %u, %s oriented\r\n"
																								, oEscFeaturesSupported.b.fmmu_operation
																								, (oEscFeaturesSupported.b.fmmu_operation==0)?"Bit":"Byte");	
	CONSOLE_PutMessage(inst, "      unused_register_access: %u, %s\r\n"
																								, oEscFeaturesSupported.b.unused_register_access
																								, (oEscFeaturesSupported.b.unused_register_access==0)?"Allowed":"Not supported");		
	CONSOLE_PutMessage(inst, "      distributed_clocks: %u, %s Available\r\n"
																								, oEscFeaturesSupported.b.distributed_clocks
																								, (oEscFeaturesSupported.b.distributed_clocks==0)?"Not":"");			
	CONSOLE_PutMessage(inst, "      dc_width: %u, %sbit\r\n"
																								, oEscFeaturesSupported.b.dc_width
																								, (oEscFeaturesSupported.b.dc_width==0)?"32":"64");				
#if 0	
	CONSOLE_PutMessage(inst, "      low_jitter_ebus: %u, %s Available\r\n"
																								, oEscFeaturesSupported.b.low_jitter_ebus
																								, (oEscFeaturesSupported.b.low_jitter_ebus==0)?"Not":"");
	CONSOLE_PutMessage(inst, "      enhanced_link_detec_ebus: %u, %s Available\r\n"
																								, oEscFeaturesSupported.b.enhanced_link_detec_ebus
																								, (oEscFeaturesSupported.b.enhanced_link_detec_ebus==0)?"Not":"");	
#endif
	CONSOLE_PutMessage(inst, "      enhanced_link_detec_mii: %u, %s Available\r\n"
																								, oEscFeaturesSupported.b.enhanced_link_detec_mii
																								, (oEscFeaturesSupported.b.enhanced_link_detec_mii==0)?"Not":"");							
	CONSOLE_PutMessage(inst, "      separate_handle_of_fcs_err: %u, %s Supported\r\n"
																								, oEscFeaturesSupported.b.separate_handle_of_fcs_err
																								, (oEscFeaturesSupported.b.separate_handle_of_fcs_err==0)?"Not":"");
	CONSOLE_PutMessage(inst, "      enhanced_dc_sync_act: %u, %s Available\r\n"
																								, oEscFeaturesSupported.b.enhanced_dc_sync_act
																								, (oEscFeaturesSupported.b.enhanced_dc_sync_act==0)?"Not":"");
	CONSOLE_PutMessage(inst, "      lrw_command_support: %u, %s Supported\r\n"
																								, oEscFeaturesSupported.b.lrw_command_support
																								, (oEscFeaturesSupported.b.lrw_command_support==0)?"":"Not");
	CONSOLE_PutMessage(inst, "      read_write_cmd_support: %u, %s Supported\r\n"
																								, oEscFeaturesSupported.b.read_write_cmd_support
																								, (oEscFeaturesSupported.b.read_write_cmd_support==0)?"":"Not");										
	CONSOLE_PutMessage(inst, "      fixed_fmmu_sm_config: %u, %s configuration\r\n"
																								, oEscFeaturesSupported.b.fixed_fmmu_sm_config
																								, (oEscFeaturesSupported.b.fixed_fmmu_sm_config==0)?"Variable":"Fixed");

	AX_INTF_EscRead(CONFIG_STATION_ADDR, &tmp16, 2);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Configured station address\r\n", CONFIG_STATION_ADDR, tmp16);	
	
	AX_INTF_EscRead(CONFIG_STATION_ALIAS, &tmp16, 2);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Configured station Alias\r\n", CONFIG_STATION_ALIAS, tmp16);		

	AX_INTF_EscRead(REG_WRITE_ENABLE, &oRegWriteEnable.d8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Register write enable\r\n", REG_WRITE_ENABLE, oRegWriteEnable.d8);		
	CONSOLE_PutMessage(inst, "      reg_write_enable: %u, %s\r\n"
																								, oRegWriteEnable.b.reg_write_enable
																								, (oRegWriteEnable.b.reg_write_enable==0)?"enable":"disable");	

	AX_INTF_EscRead(REG_WRITE_PROTECTION, &oRegWriteProtect.d8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Register write protection\r\n", REG_WRITE_PROTECTION, oRegWriteProtect.d8);		
	CONSOLE_PutMessage(inst, "      reg_write_protect: %u, %s\r\n"
																								, oRegWriteProtect.b.reg_write_protect
																								, (oRegWriteProtect.b.reg_write_protect==0)?"disable":"enable");	

	AX_INTF_EscRead(ESC_WRITE_ENABLE, &oEscWriteEnable.d8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, ESC write enable\r\n", ESC_WRITE_ENABLE, oEscWriteEnable.d8);		
	CONSOLE_PutMessage(inst, "      esc_write_enable: %u, %s\r\n"
																								, oEscWriteEnable.b.esc_write_enable
																								, (oEscWriteEnable.b.esc_write_enable==0)?"enable":"disable");	

	AX_INTF_EscRead(ESC_WRITE_PROTECTION, &oEscWriteProtect.d8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, ESC write protection\r\n", ESC_WRITE_PROTECTION, oEscWriteProtect.d8);		
	CONSOLE_PutMessage(inst, "      esc_write_protect: %u, %s\r\n"
																								, oEscWriteProtect.b.esc_write_protect
																								, (oEscWriteProtect.b.esc_write_protect==0)?"disable":"enable");	
	
	AX_INTF_EscRead(IC_PRODUCT_ID, &oProductId.d32, sizeof(oProductId));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, Product ID\r\n", IC_PRODUCT_ID, oProductId.d32[0], oProductId.d32[1]);
	CONSOLE_PutMessage(inst, "      chip_version: 0x%02x\r\n", oProductId.b.chip_revision);
	CONSOLE_PutMessage(inst, "      package_type: 0x%02x\r\n", oProductId.b.package_type);
	CONSOLE_PutMessage(inst, "      product_id: 0x%x\r\n", oProductId.b.product_id);	
	
	AX_INTF_EscRead(IC_VENDOR_ID, &oVendorId.d32, sizeof(oVendorId));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, Vendor ID\r\n", IC_VENDOR_ID, oVendorId.d32[0], oVendorId.d32[1]);
	
} /* End of clicmd_ShowEscFeatures() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowEscAlInfo()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void clicmd_ShowEscAlInfo(CONSOLE_Inst *inst)
{
	char *pStr;
	uint8_t tmp8, pdi_type;
	uint16_t tmp16;	
	uint32_t tmp32;
	oESC_CONFIG             oEscConfig;
	oPDI_CONFIG             oPdiConfig;
	oAL_CONTROL             oAlControl;
	oAL_STATUS              oAlStatus;
	oECAT_EVENT             oEcatEvent;
	oAL_EVENT               oAlEvent;
	oPDI_ERR_CODE           oPdiErrCode;
	
	CONSOLE_PutMessage(inst, "\r\nESC Application Layer Info...\r\n");
	
	AX_INTF_EscRead(AL_CONTROL, &oAlControl.d16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, AL Control\r\n", AL_CONTROL, oAlControl.d16);
	switch (oAlControl.b.initiate_state)
	{
	case 1:	pStr = "Init"; break;
	case 3:	pStr = "Bootstrap";	break;
	case 2:	pStr = "Pre-Operation";	break;
	case 4:	pStr = "Safe-Operation"; break;
	case 8:	pStr = "Operation";	break;
	default: pStr = "Unsupported"; break;
	}
	CONSOLE_PutMessage(inst, "      initiate_state: %u, %s\r\n", oAlControl.b.initiate_state, pStr);
	CONSOLE_PutMessage(inst, "      err_ind_ack: %u\r\n", oAlControl.b.err_ind_ack);
	CONSOLE_PutMessage(inst, "      device_identification: %u\r\n", oAlControl.b.device_identification);	

	AX_INTF_EscRead(AL_STATUS, &oAlStatus.d16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, AL Status\r\n", AL_STATUS, oAlStatus.d16);
	switch (oAlStatus.b.actual_state)
	{
	case 1:	pStr = "Init"; break;
	case 3:	pStr = "Bootstrap";	break;
	case 2:	pStr = "Pre-Operation";	break;
	case 4:	pStr = "Safe-Operation"; break;
	case 8:	pStr = "Operation";	break;
	default: pStr = "Unsupported"; break;
	}
	CONSOLE_PutMessage(inst, "      actual_state: %u, %s\r\n", oAlStatus.b.actual_state, pStr);
	CONSOLE_PutMessage(inst, "      err_ind: %u\r\n", oAlStatus.b.err_ind);
	CONSOLE_PutMessage(inst, "      device_identification: %u\r\n", oAlStatus.b.device_identification);	
	
	AX_INTF_EscRead(AL_STATUS_CODE, &tmp16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, AL Status Code\r\n", AL_STATUS_CODE, tmp16);
	
	AX_INTF_EscRead(PDI_CONTROL, &pdi_type, 1);
	switch (pdi_type)
	{
	case 0x00:	pStr = "Interface deactiveated(no PDI)"; break;
	case 0x01:	pStr = "4 Digital Input";	break;
	case 0x02:	pStr = "4 Digital Output";	break;
	case 0x03:	pStr = "2 Digital Input and 2 Digital Output"; break;
	case 0x04:	pStr = "Digital I/O";	break;
	case 0x05:	pStr = "SPI Slave";	break;
	case 0x06:	pStr = "Oversample I/O";	break;
	case 0x07:	pStr = "EtherCAT Bridge(port 3)";	break;		
	case 0x08:	pStr = "16Bit asynchronous Microcontroller";	break;		
	case 0x09:	pStr = "8Bit asynchronous Microcontroller";	break;		
	case 0x0A:	pStr = "16Bit synchronous Microcontroller";	break;		
	case 0x0B:	pStr = "8Bit synchronous Microcontroller";	break;
	case 0x10:	pStr = "32 Digital Input and 0 Digital Output";	break;				
	case 0x11:	pStr = "24 Digital Input and 8 Digital Output";	break;				
	case 0x12:	pStr = "16 Digital Input and 16 Digital Output";	break;		
	case 0x13:	pStr = "8 Digital Input and 24 Digital Output";	break;		
	case 0x14:	pStr = "0 Digital Input and 32 Digital Output";	break;				
	case 0x80:	pStr = "On-chip bus";	break;						
	default:    pStr = "Unsupported"; break;
	}
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, PDI Control, %s\r\n", PDI_CONTROL, pdi_type, pStr);
	
	AX_INTF_EscRead(ESC_CONFIG, &oEscConfig.d8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, ESC configuration\r\n", ESC_CONFIG, oEscConfig.d8);		
	CONSOLE_PutMessage(inst, "      device_emulation: %u\r\n", oEscConfig.b.device_emulation);
	CONSOLE_PutMessage(inst, "      enhanced_link_detec_all_port: %u\r\n", oEscConfig.b.enhanced_link_detec_all_port);
	CONSOLE_PutMessage(inst, "      distributed_clk_sync_out_unit: %u\r\n", oEscConfig.b.distributed_clk_sync_out_unit);
	CONSOLE_PutMessage(inst, "      distributed_clk_latch_in_unit: %u\r\n", oEscConfig.b.distributed_clk_latch_in_unit);
	CONSOLE_PutMessage(inst, "      enhanced_link_port0: %u\r\n", oEscConfig.b.enhanced_link_port0);
	CONSOLE_PutMessage(inst, "      enhanced_link_port1: %u\r\n", oEscConfig.b.enhanced_link_port1);
	CONSOLE_PutMessage(inst, "      enhanced_link_port2: %u\r\n", oEscConfig.b.enhanced_link_port2);
	CONSOLE_PutMessage(inst, "      enhanced_link_port3: %u\r\n", oEscConfig.b.enhanced_link_port3);	

	AX_INTF_EscRead(PDI_CONFIG, &oPdiConfig.d32, sizeof(oPdiConfig.d32));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ESC configuration\r\n", PDI_CONFIG, oPdiConfig.d32);
	CONSOLE_PutMessage(inst, "      sync0_driver_pol: %u\r\n", oPdiConfig.bGeneralTerm.sync0_driver_pol);
	CONSOLE_PutMessage(inst, "      sync0_latch0_cfg: %u\r\n", oPdiConfig.bGeneralTerm.sync0_latch0_cfg);
	CONSOLE_PutMessage(inst, "      sync0_map_al_event: %u\r\n", oPdiConfig.bGeneralTerm.sync0_map_al_event);
	CONSOLE_PutMessage(inst, "      sync1_driver_pol: %u\r\n", oPdiConfig.bGeneralTerm.sync1_driver_pol);
	CONSOLE_PutMessage(inst, "      sync1_latch1_cfg: %u\r\n", oPdiConfig.bGeneralTerm.sync1_latch1_cfg);	
	CONSOLE_PutMessage(inst, "      sync1_map_al_event: %u\r\n", oPdiConfig.bGeneralTerm.sync1_map_al_event);		
	
	if (pdi_type == 0x05)
	{
		CONSOLE_PutMessage(inst, "      PDI Type: SPI\r\n");		
		CONSOLE_PutMessage(inst, "      spi_mode: Mode%u\r\n", oPdiConfig.bSPIS.spi_mode);
		switch (oPdiConfig.bSPIS.spi_irq_driver_pol)
		{
		case 0x00:	pStr = "Push-Pull/Active Low"; break;
		case 0x01:	pStr = "Open-Drain/Active Low";	break;
		case 0x02:	pStr = "Push-Pull/Active High";	break;
		default:	  pStr = "Open-Source/Active High";	break;			
		}
		CONSOLE_PutMessage(inst, "      spi_irq_driver_pol: %u, %s\r\n"
																									, oPdiConfig.bSPIS.spi_irq_driver_pol, pStr);
		CONSOLE_PutMessage(inst, "      spi_sel_pol: %u, Active %s\r\n"
																									, oPdiConfig.bSPIS.spi_sel_pol
																									, (oPdiConfig.bSPIS.spi_sel_pol==0)?"Low":"High");		
		CONSOLE_PutMessage(inst, "      data_out_sampl_mode: %u, %s sample\r\n"
																									, oPdiConfig.bSPIS.data_out_sampl_mode
																									, (oPdiConfig.bSPIS.data_out_sampl_mode==0)?"Normal":"Late");				
	}
	
	AX_INTF_EscRead(ECAT_EVENT_MASK, &oEcatEvent.d16, sizeof(oEcatEvent.d16));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT event mask\r\n", ECAT_EVENT_MASK, oEcatEvent.d16);
	CONSOLE_PutMessage(inst, "      dc_latch_event_mask: %u\r\n", oEcatEvent.b.dc_latch_event);
	CONSOLE_PutMessage(inst, "      dc_status_event_mask: %u\r\n", oEcatEvent.b.dc_status_event);	
	CONSOLE_PutMessage(inst, "      al_status_event_mask: %u\r\n", oEcatEvent.b.al_status_event);
	CONSOLE_PutMessage(inst, "      sync_0_7_manager_status_mask: %u\r\n", oEcatEvent.b.sync_0_7_manager_status);	
	
	AX_INTF_EscRead(PDI_AL_EVENT_MASK, &oAlEvent.d32, sizeof(oAlEvent.d32));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT event mask\r\n", PDI_AL_EVENT_MASK, oAlEvent.d32);
	CONSOLE_PutMessage(inst, "      al_control_event_mask: %u\r\n", oAlEvent.b.al_control_event);	
	CONSOLE_PutMessage(inst, "      dc_latch_event_mask: %u\r\n", oAlEvent.b.dc_latch_event);
	CONSOLE_PutMessage(inst, "      state_of_dc_sync_0_mask: %u\r\n", oAlEvent.b.state_of_dc_sync_0);
	CONSOLE_PutMessage(inst, "      state_of_dc_sync_1_mask: %u\r\n", oAlEvent.b.state_of_dc_sync_1);	
	CONSOLE_PutMessage(inst, "      sm_activation_mask: %u\r\n", oAlEvent.b.sm_activation);
	CONSOLE_PutMessage(inst, "      eep_emulation_mask: %u\r\n", oAlEvent.b.eep_emulation);
	CONSOLE_PutMessage(inst, "      watchdog_process_data_mask: %u\r\n", oAlEvent.b.watchdog_process_data);	
	CONSOLE_PutMessage(inst, "      sm_0_15_interrupts_mask: %u\r\n", oAlEvent.b.sm_0_15_interrupts);		
	
	AX_INTF_EscRead(ECAT_EVENT_REQUEST, &oEcatEvent.d16, sizeof(oEcatEvent.d16));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT event request\r\n", ECAT_EVENT_REQUEST, oEcatEvent.d16);
	CONSOLE_PutMessage(inst, "      dc_latch_event: %u\r\n", oEcatEvent.b.dc_latch_event);	
	CONSOLE_PutMessage(inst, "      dc_status_event: %u\r\n", oEcatEvent.b.dc_status_event);	
	CONSOLE_PutMessage(inst, "      al_status_event: %u\r\n", oEcatEvent.b.al_status_event);
	CONSOLE_PutMessage(inst, "      sync_0_7_manager_status: %u\r\n", oEcatEvent.b.sync_0_7_manager_status);		

	AX_INTF_EscRead(AL_EVENT_REQUEST, &oAlEvent.d32, sizeof(oAlEvent.d32));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT event request\r\n", AL_EVENT_REQUEST, oAlEvent.d32);
	CONSOLE_PutMessage(inst, "      al_control_event: %u\r\n", oAlEvent.b.al_control_event);	
	CONSOLE_PutMessage(inst, "      dc_latch_event: %u\r\n", oAlEvent.b.dc_latch_event);
	CONSOLE_PutMessage(inst, "      state_of_dc_sync_0: %u\r\n", oAlEvent.b.state_of_dc_sync_0);
	CONSOLE_PutMessage(inst, "      state_of_dc_sync_1: %u\r\n", oAlEvent.b.state_of_dc_sync_1);	
	CONSOLE_PutMessage(inst, "      sm_activation: %u\r\n", oAlEvent.b.sm_activation);
	CONSOLE_PutMessage(inst, "      eep_emulation: %u\r\n", oAlEvent.b.eep_emulation);
	CONSOLE_PutMessage(inst, "      watchdog_process_data: %u\r\n", oAlEvent.b.watchdog_process_data);	
	CONSOLE_PutMessage(inst, "      sm_0_15_interrupts: %u\r\n", oAlEvent.b.sm_0_15_interrupts);		
	
	AX_INTF_EscRead(PDI_ERR_COUNTER, &tmp8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, PDI Error Counter\r\n", PDI_ERR_COUNTER, tmp8);	

	AX_INTF_EscRead(PDI_ERR_CODE, &oPdiErrCode.d8, 1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, PDI Error Code\r\n", PDI_ERR_CODE, oPdiErrCode.d8);	
	if (pdi_type == 0x05)
	{
		CONSOLE_PutMessage(inst, "      spi_clk_num_of_access: %u\r\n", oPdiErrCode.bSPI.spi_clk_num_of_access);
		CONSOLE_PutMessage(inst, "      busy_violation_during_read: %u\r\n", oPdiErrCode.bSPI.busy_violation_during_read);
		CONSOLE_PutMessage(inst, "      read_termination_missing: %u\r\n", oPdiErrCode.bSPI.read_termination_missing);
		CONSOLE_PutMessage(inst, "      access_continue_after_read: %u\r\n", oPdiErrCode.bSPI.access_continue_after_read);		
		CONSOLE_PutMessage(inst, "      cmd_err: %u\r\n", oPdiErrCode.bSPI.cmd_err);				
	}
	else if (pdi_type == 0x08 || pdi_type == 0x09 || pdi_type == 0x0A || pdi_type == 0x0B)
	{
		CONSOLE_PutMessage(inst, "      busy_viola_during_read: %u\r\n", oPdiErrCode.bASYNC_SYNC.busy_viola_during_read);
		CONSOLE_PutMessage(inst, "      busy_viola_during_write: %u\r\n", oPdiErrCode.bASYNC_SYNC.busy_viola_during_write);
		CONSOLE_PutMessage(inst, "      addr_err_for_read: %u\r\n", oPdiErrCode.bASYNC_SYNC.addr_err_for_read);
		CONSOLE_PutMessage(inst, "      addr_err_for_write: %u\r\n", oPdiErrCode.bASYNC_SYNC.addr_err_for_write);		
	}
	
	AX_INTF_EscRead(LOST_LINK_COUNTER, &tmp32, 4);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Lost link counter\r\n", LOST_LINK_COUNTER, tmp32);		
	
} /* End of clicmd_ShowEscAlInfo() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowEscWdInfo()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void clicmd_ShowEscWdInfo(CONSOLE_Inst *inst)
{
	uint8_t tmp8;
	uint16_t tmp16;	
	oWD_STATUS_PROCESS_DATA     oWdStatusProcessData;
	
	CONSOLE_PutMessage(inst, "\r\nESC Watchdog Info...\r\n");
	
	AX_INTF_EscRead(WD_DIVIDER, &tmp16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Watchdog Divider\r\n", WD_DIVIDER, tmp16);

	AX_INTF_EscRead(WD_TIME_PDI, &tmp16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Watchdog Time PDI\r\n", WD_TIME_PDI, tmp16);

	AX_INTF_EscRead(WD_TIME_PROCESS_DATA, &tmp16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Watchdog Time Process Data\r\n", WD_TIME_PROCESS_DATA, tmp16);

	AX_INTF_EscRead(WD_STATUS_PROCESS_DATA, &oWdStatusProcessData.d16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Watchdog Status Process Data\r\n", WD_STATUS_PROCESS_DATA, oWdStatusProcessData.d16);
	CONSOLE_PutMessage(inst, "      wd_process_data_status: %u\r\n", oWdStatusProcessData.b.wd_process_data_status);	
	
	AX_INTF_EscRead(WD_COUNTER_PROCESS_DATA, &tmp8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Watchdog Counter Process Data\r\n", WD_COUNTER_PROCESS_DATA, tmp8);

	AX_INTF_EscRead(WD_COUNTER_PDI, &tmp8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Watchdog Counter PDI\r\n", WD_COUNTER_PDI, tmp8);

} /* End of clicmd_ShowEscWdInfo() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowEscFmmuSmInfo()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void clicmd_ShowEscFmmuSmInfo(CONSOLE_Inst *inst)
{
	char *pStr;
	uint32_t tmp8;
	uint16_t tmp16;	
	uint32_t i, tmp32, addr;		
	oFMMU_LOGIC_START_STOP_BIT  oFmmuLogicBit;
	oSM_CONTROL                 oSmControl;
	oSM_STATUS                  oSmStatus;
	oSM_ACTIVATE                oSmActivate;
	
	CONSOLE_PutMessage(inst, "\r\nESC FMMU Info...\r\n");
	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "FMMU[%u]:\r\n", i);
		
		addr = FMMU_0_LOGIC_START_ADDR + (FMMU_OFFSET*i);
		AX_INTF_EscRead(addr, &tmp32, 4);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Logical start address\r\n", addr, tmp32);

		addr = FMMU_0_LENGTH + (FMMU_OFFSET*i);
		AX_INTF_EscRead(addr, &tmp16, 2);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Length\r\n", addr, tmp16);

		addr = FMMU_0_LOGIC_START_BIT + (FMMU_OFFSET*i);
		AX_INTF_EscRead(addr, &oFmmuLogicBit.d8, 1);	
		CONSOLE_PutMessage(inst, "0x%04x: %u, Logical start bit\r\n", addr, oFmmuLogicBit.b.logical_bit);
		
		addr = FMMU_0_LOGIC_STOP_BIT + (FMMU_OFFSET*i);
		AX_INTF_EscRead(addr, &oFmmuLogicBit.d8, 1);	
		CONSOLE_PutMessage(inst, "0x%04x: %u, Logical stop bit\r\n", addr, oFmmuLogicBit.b.logical_bit);		
		
		addr = FMMU_0_PHY_START_ADDR + (FMMU_OFFSET*i);
		AX_INTF_EscRead(addr, &tmp16, 2);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Physical start address\r\n", addr, tmp16);		
		
		addr = FMMU_0_PHY_START_BIT + (FMMU_OFFSET*i);
		AX_INTF_EscRead(addr, &oFmmuLogicBit.d8, 1);	
		CONSOLE_PutMessage(inst, "0x%04x: %u, Physical start bit\r\n", addr, oFmmuLogicBit.b.logical_bit);				
		
		addr = FMMU_0_TYPE + (FMMU_OFFSET*i);
		AX_INTF_EscRead(addr, &tmp8, 1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Type\r\n", addr, tmp8);			
		
		addr = FMMU_0_ACTIVATE + (FMMU_OFFSET*i);
		AX_INTF_EscRead(addr, &tmp8, 1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Activate\r\n", addr, tmp8);	
	}
	
	
	CONSOLE_PutMessage(inst, "\r\nESC SM Info...\r\n");
	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "SM[%u]:\r\n", i);
		
		addr = SM_0_PHY_START_ADDR + (SM_OFFSET*i);
		AX_INTF_EscRead(addr, &tmp16, 2);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Physical start address\r\n", addr, tmp16);

		addr = SM_0_LENGTH + (SM_OFFSET*i);
		AX_INTF_EscRead(addr, &tmp16, 2);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Length\r\n", addr, tmp16);

		addr = SM_0_CONTROL + (SM_OFFSET*i);
		AX_INTF_EscRead(addr, &oSmControl.d8, 1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SyncManager Control\r\n", addr, oSmControl.d8);
		switch (oSmControl.b.operation_mode)
		{
		case 0x00:	pStr = "3 buffer mode"; break;
		case 0x02:	pStr = "Mailbox";	break;
		default:	  pStr = "Reserved";	break;			
		}		
		CONSOLE_PutMessage(inst, "      operation_mode: %u, %s\r\n", oSmControl.b.operation_mode, pStr);	
		switch (oSmControl.b.direction)
		{
		case 0x00:	pStr = "ECAT read, PDI write"; break;
		case 0x01:	pStr = "ECAT write, PDI read"; break;
		default:	  pStr = "Reserved";	break;			
		}			
		CONSOLE_PutMessage(inst, "      direction: %u, %s\r\n", oSmControl.b.direction, pStr);
		CONSOLE_PutMessage(inst, "      intr_in_ecat_event_request: %u\r\n", oSmControl.b.intr_in_ecat_event_request);	
		CONSOLE_PutMessage(inst, "      intr_in_al_event_request: %u\r\n", oSmControl.b.intr_in_al_event_request);			
		CONSOLE_PutMessage(inst, "      watchdog_trigger_enable: %u\r\n", oSmControl.b.watchdog_trigger_enable);					
		
		addr = SM_0_STATUS + (SM_OFFSET*i);
		AX_INTF_EscRead(addr, &oSmStatus.d8, 1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SyncManager Status\r\n", addr, oSmStatus.d8);
		CONSOLE_PutMessage(inst, "      intr_write: %u\r\n", oSmStatus.b.intr_write);			
		CONSOLE_PutMessage(inst, "      intr_read: %u\r\n", oSmStatus.b.intr_read);					
		CONSOLE_PutMessage(inst, "      mailbox_status: %s\r\n", oSmStatus.b.mailbox_status ? "full":"empty");
		switch (oSmStatus.b.three_buf_status)
		{
		case 0:	pStr = "1st buffer"; break;
		case 1:	pStr = "2nd buffer"; break;
		case 2:	pStr = "3rd buffer"; break;			
		default:	  pStr = "no buffer written";	break;			
		}				
		CONSOLE_PutMessage(inst, "      direction: %u, %s\r\n", oSmStatus.b.three_buf_status, pStr);		
		CONSOLE_PutMessage(inst, "      read_buf_in_use: %u\r\n", oSmStatus.b.read_buf_in_use);					
		CONSOLE_PutMessage(inst, "      write_buf_in_use: %u\r\n", oSmStatus.b.write_buf_in_use);					
		
		addr = SM_0_ACTIVATE + (SM_OFFSET*i);
		AX_INTF_EscRead(addr, &oSmActivate.d8, 1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SyncManager Activate\r\n", addr, oSmActivate.d8);		
		CONSOLE_PutMessage(inst, "      sm_enable: %u\r\n", oSmActivate.b.sm_enable);					
		CONSOLE_PutMessage(inst, "      repeat_request: %u\r\n", oSmActivate.b.repeat_request);							
		CONSOLE_PutMessage(inst, "      latch_event_ecat: %u\r\n", oSmActivate.b.latch_event_ecat);									
		CONSOLE_PutMessage(inst, "      latch_event_pdi: %u\r\n", oSmActivate.b.latch_event_pdi);											

	}

} /* End of clicmd_ShowEscFmmuSmInfo() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowEscDcInfo()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void clicmd_ShowEscDcInfo(CONSOLE_Inst *inst)
{
	uint16_t tmp16; 
	uint32_t tmp32, tmp32_p[2];		
	oDC_SYSTEM_TIME           oSystemTime;
	oDC_SYSTEM_TIME_DIFF      oSystemTimeDiff;
	oDC_SPEED_COUNTER_START   oSpeedCounterStart;
	oDC_SYSTIME_DIFF_FILTER_DEPTH   oSystimeDiffFilterDepth;
	oDC_SPEED_COUNTER_FILTER_DEPTH  oSpeedCntFilterDepth;
	oDC_REV_TIME_LATCH_MODE         oRevTimeLatchMode;
	oDC_CYCLIC_UNIT_CTRL            oCyclicUnitControl;
	oDC_SYNC_ACTIVATION             oSyncActivation;
	oDC_ACTIVATION_STATUS           oActivationStatus;
	oDC_SYNCx_STATUS                oSyncXStatus;
	oDC_LATCHx_CONTROL              oLatchControl;
	oDC_LATCHx_STATUS               oLatchStatus;
	
	CONSOLE_PutMessage(inst, "\r\nESC DC Info...\r\n");

	AX_INTF_EscRead(DC_PORT0_RECV_TIME, &tmp32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Receive Time Port 0\r\n", DC_PORT0_RECV_TIME, tmp32);	
	
	AX_INTF_EscRead(DC_PORT1_RECV_TIME, &tmp32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Receive Time Port 1\r\n", DC_PORT1_RECV_TIME, tmp32);	

	AX_INTF_EscRead(DC_PORT2_RECV_TIME, &tmp32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Receive Time Port 2\r\n", DC_PORT2_RECV_TIME, tmp32);	

	AX_INTF_EscRead(DC_PROC_UNIT_RECV_TIME, tmp32_p, 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, Receive Time ECAT Processing Unit\r\n"
													, DC_PROC_UNIT_RECV_TIME, tmp32_p[1], tmp32_p[0]);
													
	AX_INTF_EscRead(DC_SYSTEM_TIME, &oSystemTime.d32[0], 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, System Time\r\n", DC_SYSTEM_TIME, oSystemTime.d32[1], oSystemTime.d32[0]);
	CONSOLE_PutMessage(inst, "      value_compare_with_sys_time: %uns\r\n", oSystemTime.b.value_compare_with_systim);	
	CONSOLE_PutMessage(inst, "      local_copy_of_sys_time: %uns\r\n", oSystemTime.b.local_copy_of_systim);		

	AX_INTF_EscRead(DC_SYSTEM_TIME_OFFSET, tmp32_p, 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, System Time Offset\r\n", DC_SYSTEM_TIME_OFFSET, tmp32_p[1], tmp32_p[0]);

	AX_INTF_EscRead(DC_SYSTEM_TIME_DELAY, tmp32_p, 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, System Time Delay\r\n", DC_SYSTEM_TIME_DELAY, tmp32_p[1], tmp32_p[0]);

	AX_INTF_EscRead(DC_SYSTEM_TIME_DIFF, &oSystemTimeDiff.d32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: %ld, System Time Diff\r\n", DC_SYSTEM_TIME_DIFF, oSystemTimeDiff.d32);

	AX_INTF_EscRead(DC_SPEED_COUNTER_START, &oSpeedCounterStart.d16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, System Counter Start\r\n", DC_SPEED_COUNTER_START, oSpeedCounterStart.d16);
	CONSOLE_PutMessage(inst, "      bw_for_adj_of_loc_copy_of_sys_time: %u\r\n", oSpeedCounterStart.b.bw_for_adj_of_loc_copy_of_systim);			
	
	AX_INTF_EscRead(DC_SPEED_COUNTER_DIFF, &tmp16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, System Counter Diff\r\n", DC_SPEED_COUNTER_DIFF, tmp16);
	
	AX_INTF_EscRead(DC_SYSTIME_DIFF_FILTER_DEPTH, &oSystimeDiffFilterDepth.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, System Time Difference Filter Depth\r\n", DC_SYSTIME_DIFF_FILTER_DEPTH, oSystimeDiffFilterDepth.d8);
	CONSOLE_PutMessage(inst, "      filt_depth_for_avg_rev_sys_time_deva: %u\r\n", oSystimeDiffFilterDepth.b.filt_depth_for_avg_rev_systim_deva);				

	AX_INTF_EscRead(DC_SPEED_COUNTER_FILTER_DEPTH, &oSpeedCntFilterDepth.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Speed Counter Filter Depth\r\n", DC_SPEED_COUNTER_FILTER_DEPTH, oSpeedCntFilterDepth.d8);
	CONSOLE_PutMessage(inst, "      filt_depth_for_avg_clk_period_deva: %u\r\n", oSpeedCntFilterDepth.b.filt_depth_for_avg_clk_period_deva);				

	AX_INTF_EscRead(DC_REV_TIME_LATCH_MODE, &oRevTimeLatchMode.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Receive Time Latch Mode\r\n", DC_REV_TIME_LATCH_MODE, oRevTimeLatchMode.d8);
	CONSOLE_PutMessage(inst, "      rev_time_latch_mode: %u mode\r\n"
																	, oRevTimeLatchMode.b.rev_time_latch_mode
																	, (oRevTimeLatchMode.b.rev_time_latch_mode==0)?"Forwarding":"Reverse");				

	AX_INTF_EscRead(DC_CYCLIC_UNIT_CTRL, &oCyclicUnitControl.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Cyclic Unit Control\r\n", DC_CYCLIC_UNIT_CTRL, oCyclicUnitControl.d8);
	CONSOLE_PutMessage(inst, "      sync_out_unit_ctrl: %u, %s controlled\r\n"
																	, oCyclicUnitControl.b.sync_out_unit_ctrl
																	, (oCyclicUnitControl.b.sync_out_unit_ctrl==0)?"ECAT":"PDI");			
	CONSOLE_PutMessage(inst, "      latch_in_unit_0: %u, %s controlled\r\n"
																	, oCyclicUnitControl.b.latch_in_unit_0
																	, (oCyclicUnitControl.b.latch_in_unit_0==0)?"ECAT":"PDI");			
	CONSOLE_PutMessage(inst, "      latch_in_unit_1: %u, %s controlled\r\n"
																	, oCyclicUnitControl.b.latch_in_unit_1
																	, (oCyclicUnitControl.b.latch_in_unit_1==0)?"ECAT":"PDI");					
	
	AX_INTF_EscRead(DC_SYNC_ACTIVATION, &oSyncActivation.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Sync. Acviation\r\n", DC_SYNC_ACTIVATION, oSyncActivation.d8);
	CONSOLE_PutMessage(inst, "      sync_out_unit_act: %u, %s\r\n"
																	, oSyncActivation.b.sync_out_unit_act
																	, (oSyncActivation.b.sync_out_unit_act==0)?"Deactivated":"Activated");			
	CONSOLE_PutMessage(inst, "      sync0_gen: %u\r\n", oSyncActivation.b.sync0_gen);			
	CONSOLE_PutMessage(inst, "      sync1_gen: %u\r\n", oSyncActivation.b.sync1_gen);				
	CONSOLE_PutMessage(inst, "      auto_act_by_wr_start_time_cyc_op: %u\r\n", oSyncActivation.b.auto_act_by_wr_start_time_cyc_op);					
	CONSOLE_PutMessage(inst, "      ext_of_start_time_cyc_op: %u\r\n", oSyncActivation.b.ext_of_start_time_cyc_op);					
	CONSOLE_PutMessage(inst, "      start_time_plausibility_chk: %u\r\n", oSyncActivation.b.start_time_plausibility_chk);						
	CONSOLE_PutMessage(inst, "      near_future_configuration: %u\r\n", oSyncActivation.b.near_future_configuration);							
	CONSOLE_PutMessage(inst, "      sync_debug_pulse: %u\r\n", oSyncActivation.b.sync_debug_pulse);								

	AX_INTF_EscRead(DC_PULSE_LEN_OF_SYNC_SIGNAL, &tmp16, 2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x(%uns), Pulse length of SyncSignals\r\n", DC_PULSE_LEN_OF_SYNC_SIGNAL, tmp16, (tmp16*10));
	
	AX_INTF_EscRead(DC_ACTIVATION_STATUS, &oActivationStatus.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Activation Status\r\n", DC_ACTIVATION_STATUS, oActivationStatus.d8);
	CONSOLE_PutMessage(inst, "      sync0_act_state: %u, First pulse %s pending\r\n"
																	, oActivationStatus.b.sync0_act_state
																	, (oActivationStatus.b.sync0_act_state == 0)?"not":"");				
	CONSOLE_PutMessage(inst, "      sync1_act_state: %u, First pulse %s pending\r\n"
																	, oActivationStatus.b.sync1_act_state
																	, (oActivationStatus.b.sync1_act_state == 0)?"not":"");					
	CONSOLE_PutMessage(inst, "      start_time_plausibility_chk_result: %u\r\n", oActivationStatus.b.start_time_plausibility_chk_result);						
	
	AX_INTF_EscRead(DC_SYNC0_STATUS, &oSyncXStatus.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SYNC0 Status\r\n", DC_SYNC0_STATUS, oSyncXStatus.d8);
	CONSOLE_PutMessage(inst, "      sync0_state_for_ack_mode: %u\r\n", oSyncXStatus.b.syncx_state_for_ack_mode);					

	AX_INTF_EscRead(DC_SYNC1_STATUS, &oSyncXStatus.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SYNC1 Status\r\n", DC_SYNC1_STATUS, oSyncXStatus.d8);
	CONSOLE_PutMessage(inst, "      sync1_state_for_ack_mode: %u\r\n", oSyncXStatus.b.syncx_state_for_ack_mode);					

	AX_INTF_EscRead(DC_START_TIME_CYCLIC_OPERATION, tmp32_p, 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, Start Time Cyclic Operation\r\n"
																				, DC_START_TIME_CYCLIC_OPERATION, tmp32_p[1], tmp32_p[0]);

	AX_INTF_EscRead(DC_NEXT_SYNC1_PULSE, &tmp32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx(%luns), System time of next SYNC1\r\n", DC_NEXT_SYNC1_PULSE, tmp32, tmp32);
	
	AX_INTF_EscRead(DC_SYNC0_CYCLE_TIME, &tmp32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx(%luns), Time between two consecutive SYNC0 pulse\r\n", DC_SYNC0_CYCLE_TIME, tmp32, tmp32);

	AX_INTF_EscRead(DC_SYNC1_CYCLE_TIME, &tmp32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx(%luns), Time between SYNC0 pulse and SYNC1 pulse\r\n", DC_SYNC1_CYCLE_TIME, tmp32, tmp32);

	AX_INTF_EscRead(DC_LATCH0_CONTROL, &oLatchControl.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, LATCH 0 Control\r\n", DC_LATCH0_CONTROL, oLatchControl.d8);
	CONSOLE_PutMessage(inst, "      latch0_positive_edge: %u, %s latched\r\n"
																	, oLatchControl.b.latchx_positive_edge
																	, (oLatchControl.b.latchx_positive_edge == 0)?"Continue":"Single");
	CONSOLE_PutMessage(inst, "      latch0_negative_edge: %u, %s latched\r\n"
																	, oLatchControl.b.latchx_positive_edge
																	, (oLatchControl.b.latchx_negative_edge == 0)?"Continue":"Single");					
	
	AX_INTF_EscRead(DC_LATCH1_CONTROL, &oLatchControl.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, LATCH 1 Control\r\n", DC_LATCH1_CONTROL, oLatchControl.d8);	
	CONSOLE_PutMessage(inst, "      latch1_positive_edge: %u, %s latched\r\n"
																	, oLatchControl.b.latchx_positive_edge
																	, (oLatchControl.b.latchx_positive_edge == 0)?"Continue":"Single");
	CONSOLE_PutMessage(inst, "      latch1_negative_edge: %u, %s latched\r\n"
																	, oLatchControl.b.latchx_negative_edge
																	, (oLatchControl.b.latchx_negative_edge == 0)?"Continue":"Single");	
	
	AX_INTF_EscRead(DC_LATCH0_STATUS, &oLatchStatus.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, LATCH 0 Status\r\n", DC_LATCH0_STATUS, oLatchStatus.d8);	
	CONSOLE_PutMessage(inst, "      event_latch0_positive_edge: %u, Positive edge %s detected\r\n"
																	, oLatchStatus.b.event_latchx_positive_edge
																	, (oLatchStatus.b.event_latchx_positive_edge == 0)?"not ":"");					
	CONSOLE_PutMessage(inst, "      event_latch0_negative_edge: %u, Negative edge %s detected\r\n"
																	, oLatchStatus.b.event_latchx_positive_edge
																	, (oLatchStatus.b.event_latchx_negative_edge == 0)?"not ":"");		
	CONSOLE_PutMessage(inst, "      latch0_pin_state: %u\r\n", oLatchStatus.b.latchx_pin_state);		

	AX_INTF_EscRead(DC_LATCH1_STATUS, &oLatchStatus.d8, 1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, LATCH 1 Status\r\n", DC_LATCH1_STATUS, oLatchStatus.d8);	
	CONSOLE_PutMessage(inst, "      event_latch1_positive_edge: %u, Positive edge %s detected\r\n"
																	, oLatchStatus.b.event_latchx_positive_edge
																	, (oLatchStatus.b.event_latchx_positive_edge == 0)?"not ":"");					
	CONSOLE_PutMessage(inst, "      event_latch1_negative_edge: %u, Negative edge %s detected\r\n"
																	, oLatchStatus.b.event_latchx_positive_edge
																	, (oLatchStatus.b.event_latchx_negative_edge == 0)?"not ":"");		
	CONSOLE_PutMessage(inst, "      latch1_pin_state: %u\r\n", oLatchStatus.b.latchx_pin_state);		

	AX_INTF_EscRead(DC_LATCH0_TIME_POSITIVE_EDGE, tmp32_p, 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 0 Time Positive Edge\r\n", DC_LATCH0_TIME_POSITIVE_EDGE, tmp32_p[1], tmp32_p[0]);	
	
	AX_INTF_EscRead(DC_LATCH0_TIME_NEGATIVE_EDGE, tmp32_p, 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 0 Time Negative Edge\r\n", DC_LATCH0_TIME_NEGATIVE_EDGE, tmp32_p[1], tmp32_p[0]);	

	AX_INTF_EscRead(DC_LATCH1_TIME_POSITIVE_EDGE, tmp32_p, 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 1 Time Positive Edge\r\n", DC_LATCH1_TIME_POSITIVE_EDGE, tmp32_p[1], tmp32_p[0]);	

	AX_INTF_EscRead(DC_LATCH1_TIME_NEGATIVE_EDGE, tmp32_p, 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 1 Time Negative Edge\r\n", DC_LATCH1_TIME_NEGATIVE_EDGE, tmp32_p[1], tmp32_p[0]);	

	AX_INTF_EscRead(DC_LATCH1_TIME_NEGATIVE_EDGE, tmp32_p, 8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 1 Time Negative Edge\r\n", DC_LATCH1_TIME_NEGATIVE_EDGE, tmp32_p[1], tmp32_p[0]);	
	
	AX_INTF_EscRead(DC_ECAT_BUF_CHANGE_EVENT_TIME, &tmp32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT Buffer Change Event Time\r\n", DC_ECAT_BUF_CHANGE_EVENT_TIME, tmp32);	

	AX_INTF_EscRead(DC_PDI_BUF_START_EVENT_TIME, &tmp32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, PDI Buffer Start Event Time\r\n", DC_PDI_BUF_START_EVENT_TIME, tmp32);	
	
	AX_INTF_EscRead(DC_PDI_BUF_CHANGE_EVENT_TIME, &tmp32, 4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, PDI Buffer Change Event Time\r\n", DC_PDI_BUF_CHANGE_EVENT_TIME, tmp32);		

} /* End of clicmd_ShowEscDcInfo() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CLICMD_GetCmdTableSize()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint16_t CLICMD_GetCmdTableSize(void)
{
    return sizeof(CLICMD_userCmdTable)/sizeof(CONSOLE_CmdEntry);
} /* End of CLICMD_GetCmdTableSize() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CLICMD_GetCmdTable()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void *CLICMD_GetCmdTable(void)
{
    return ((void*)CLICMD_userCmdTable);
} /* End of CLICMD_GetCmdTableSize() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SystemReboot()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SystemReboot(CONSOLE_Inst *inst, int argc, char **argv)
{
	NVIC_SystemReset();
	return 1;
} /* End of clicmd_SystemReboot() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SystemRebootHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SystemRebootHelp(CONSOLE_Inst *inst)
{
	// Set processor clock to default before system reset
	CONSOLE_PutMessage(inst, "        reboot: System reboot\r\n");
	return 1;
} /* End of clicmd_SystemRebootHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SystemRebootUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SystemRebootUsage(CONSOLE_Inst *inst)
{
	// Set processor clock to default before system reset
	clicmd_SystemRebootHelp(inst);
	
	CONSOLE_PutMessage(inst, "Usage: reboot [-tp DecValue]\r\n");
	return 1;
} /* End of clicmd_SystemRebootUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiRead()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiRead(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, ByteLen;
	
	if (argc == 1)
	{
		clicmd_PdiReadUsage(inst);
		return 1;
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}

	if (clicmd_HexText2Long((uint8_t*)argv[2], &ByteLen, strlen(argv[2])) != 0)
	{
		return -1;
	}
		
	CONSOLE_PutMessage(inst, "Start byte read, addr=0x%x, len=0x%x byte use ESC chip select\r\n", HexOffset, ByteLen);
	memset(StrBuf, 0, ByteLen);
	HW_EscRead((MEM_ADDR*)StrBuf, HexOffset,  ByteLen);
	clicmd_ShowMemoryInHex8b(inst, StrBuf, ByteLen);		
	return 1;
} /* End of clicmd_PdiRead() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiReadHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiReadHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         pdird: Process data interface reading\r\n");
	return 1;
} /* End of clicmd_PdiReadHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiReadUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiReadUsage(CONSOLE_Inst *inst)
{
	clicmd_PdiReadHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: pdird <HexOffset> <HexByteLen>\r\n");

	return 1;
} /* End of clicmd_PdiReadUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiWrite()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiWrite(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, i, ByteLen;
	uint8_t *pValue8b;
	
	if (argc == 1)
	{
		clicmd_PdiWriteUsage(inst);
		return 1;
	}
	else if (argc < 3)
	{
		return -1;
	}

	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0) 
	{
		return -1;
	}
	
	ByteLen = argc - 2;
		
	pValue8b = StrBuf;
	for (i=0; i<ByteLen; i++)
	{
		if (clicmd_HexText2Char((uint8_t*)argv[i+2], pValue8b, strlen(argv[i+2])) != 0)
		{
			return -1;
		}
		pValue8b++;				
	}
	
	CONSOLE_PutMessage(inst, "Start byte write, addr=0x%x, len=0x%x byte use ESC chip select\r\n", HexOffset, ByteLen);
	HW_EscWrite((MEM_ADDR*)StrBuf, HexOffset,  ByteLen);			
	clicmd_ShowMemoryInHex8b(inst, StrBuf, ByteLen);		

	return 1;
} /* End of clicmd_PdiWrite() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiWriteHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiWriteHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         pdiwr: Process data interface writing\r\n");
	return 1;
} /* End of clicmd_PdiWriteHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiWriteUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiWriteUsage(CONSOLE_Inst *inst)
{
	clicmd_PdiWriteHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: pdiwr <HexOffset> <HexValue 0> <HexValue 1>..<HexValue N>]\r\n");
	return 1;
} /* End of clicmd_PdiWriteUsage() */

#if CLI_MCU_DBG_CMD_ENABLE
/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SysRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SysRegisterDisplay(CONSOLE_Inst *inst)
{
	SYS_T *SysOffset = 0;
	
	/* Show current settings */	
	CONSOLE_PutMessage(inst, "Current SYS registers...\r\n");	
	CONSOLE_PutMessage(inst, "PDID(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->PDID), SYS->PDID);
	CONSOLE_PutMessage(inst, "RSTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->RSTSTS), SYS->RSTSTS);
	CONSOLE_PutMessage(inst, "IPRST0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IPRST0), SYS->IPRST0);
	CONSOLE_PutMessage(inst, "IPRST1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IPRST1), SYS->IPRST1);
	CONSOLE_PutMessage(inst, "IPRST2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IPRST2), SYS->IPRST2);	
	CONSOLE_PutMessage(inst, "BODCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->BODCTL), SYS->BODCTL);
	CONSOLE_PutMessage(inst, "IVSCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IVSCTL), SYS->IVSCTL);
	CONSOLE_PutMessage(inst, "PORCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->PORCTL), SYS->PORCTL);
	CONSOLE_PutMessage(inst, "VREFCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->VREFCTL), SYS->VREFCTL);
	CONSOLE_PutMessage(inst, "USBPHY(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->USBPHY), SYS->USBPHY);
	CONSOLE_PutMessage(inst, "GPA_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPA_MFPL), SYS->GPA_MFPL);
	CONSOLE_PutMessage(inst, "GPA_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPA_MFPH), SYS->GPA_MFPH);
	CONSOLE_PutMessage(inst, "GPB_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPB_MFPL), SYS->GPB_MFPL);
	CONSOLE_PutMessage(inst, "GPB_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPB_MFPH), SYS->GPB_MFPH);
	CONSOLE_PutMessage(inst, "GPC_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPC_MFPL), SYS->GPC_MFPL);
	CONSOLE_PutMessage(inst, "GPC_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPC_MFPH), SYS->GPC_MFPH);
	CONSOLE_PutMessage(inst, "GPD_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPD_MFPL), SYS->GPD_MFPL);
	CONSOLE_PutMessage(inst, "GPD_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPD_MFPH), SYS->GPD_MFPH);
	CONSOLE_PutMessage(inst, "GPE_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPE_MFPL), SYS->GPE_MFPL);
	CONSOLE_PutMessage(inst, "GPE_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPE_MFPH), SYS->GPE_MFPH);
	CONSOLE_PutMessage(inst, "GPF_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPF_MFPL), SYS->GPF_MFPL);
	CONSOLE_PutMessage(inst, "GPF_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPF_MFPH), SYS->GPF_MFPH);
	CONSOLE_PutMessage(inst, "GPG_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPG_MFPL), SYS->GPG_MFPL);
	CONSOLE_PutMessage(inst, "GPG_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPG_MFPH), SYS->GPG_MFPH);
	CONSOLE_PutMessage(inst, "GPH_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPH_MFPL), SYS->GPH_MFPL);
	CONSOLE_PutMessage(inst, "GPH_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPH_MFPH), SYS->GPH_MFPH);
	CONSOLE_PutMessage(inst, "GPA_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPA_MFOS), SYS->GPA_MFOS);
	CONSOLE_PutMessage(inst, "GPB_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPB_MFOS), SYS->GPB_MFOS);
	CONSOLE_PutMessage(inst, "GPC_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPC_MFOS), SYS->GPC_MFOS);
	CONSOLE_PutMessage(inst, "GPD_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPD_MFOS), SYS->GPD_MFOS);
	CONSOLE_PutMessage(inst, "GPE_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPE_MFOS), SYS->GPE_MFOS);
	CONSOLE_PutMessage(inst, "GPF_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPF_MFOS), SYS->GPF_MFOS);
	CONSOLE_PutMessage(inst, "GPG_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPG_MFOS), SYS->GPG_MFOS);
	CONSOLE_PutMessage(inst, "GPH_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPH_MFOS), SYS->GPH_MFOS);
	CONSOLE_PutMessage(inst, "SRAM_INTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_INTCTL), SYS->SRAM_INTCTL);
	CONSOLE_PutMessage(inst, "SRAM_STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_STATUS), SYS->SRAM_STATUS);
	CONSOLE_PutMessage(inst, "SRAM_ERRADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_ERRADDR), SYS->SRAM_ERRADDR);
	CONSOLE_PutMessage(inst, "SRAM_BISTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_BISTCTL), SYS->SRAM_BISTCTL);
	CONSOLE_PutMessage(inst, "SRAM_BISTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_BISTSTS), SYS->SRAM_BISTSTS);	
	CONSOLE_PutMessage(inst, "IRCTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IRCTCTL), SYS->IRCTCTL);
	CONSOLE_PutMessage(inst, "IRCTIEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IRCTIEN), SYS->IRCTIEN);
	CONSOLE_PutMessage(inst, "IRCTISTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IRCTISTS), SYS->IRCTISTS);
	CONSOLE_PutMessage(inst, "REGLCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->REGLCTL), SYS->REGLCTL);
	CONSOLE_PutMessage(inst, "PLCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->PLCTL), SYS->PLCTL);
	CONSOLE_PutMessage(inst, "PLSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->PLSTS), SYS->PLSTS);
	CONSOLE_PutMessage(inst, "AHBMCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->AHBMCTL), SYS->AHBMCTL);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SysRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SysRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	
	if (argc == 1)
	{
		clicmd_SysRegisterUsage(inst);
		clicmd_SysRegisterDisplay(inst);
		return 1;		
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	/* Get register offset */
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}
	
	/* Get register value */	
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
	{
		return -1;
	}

	/* Set register value */
	*((uint32_t *)(((uint32_t)SYS) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write SYS Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
	return 1;
} /* End of clicmd_SysRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SysRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SysRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "           sys: Access MCU SYS registers\r\n");
	return 1;
} /* End of clicmd_SysRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SysRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SysRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_SysRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: sys [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_SysRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ClkRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ClkRegisterDisplay(CONSOLE_Inst *inst)
{
	CLK_T *ClkOffset = 0;
	
	/* Show current settings */	
	CONSOLE_PutMessage(inst, "Current CLK registers...\r\n");	
	CONSOLE_PutMessage(inst, "PWRCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PWRCTL), CLK->PWRCTL);
	CONSOLE_PutMessage(inst, "AHBCLK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->AHBCLK), CLK->AHBCLK);
	CONSOLE_PutMessage(inst, "APBCLK0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->APBCLK0), CLK->APBCLK0);	
	CONSOLE_PutMessage(inst, "APBCLK1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->APBCLK1), CLK->APBCLK1);
	CONSOLE_PutMessage(inst, "CLKSEL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKSEL0), CLK->CLKSEL0);
	CONSOLE_PutMessage(inst, "CLKSEL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKSEL1), CLK->CLKSEL1);
	CONSOLE_PutMessage(inst, "CLKSEL2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKSEL2), CLK->CLKSEL2);
	CONSOLE_PutMessage(inst, "CLKSEL3(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKSEL3), CLK->CLKSEL3);	
	CONSOLE_PutMessage(inst, "CLKDIV0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDIV0), CLK->CLKDIV0);
	CONSOLE_PutMessage(inst, "CLKDIV1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDIV1), CLK->CLKDIV1);
	CONSOLE_PutMessage(inst, "CLKDIV3(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDIV3), CLK->CLKDIV3);
	CONSOLE_PutMessage(inst, "CLKDIV4(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDIV4), CLK->CLKDIV4);
	CONSOLE_PutMessage(inst, "PCLKDIV(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PCLKDIV), CLK->PCLKDIV);	
	CONSOLE_PutMessage(inst, "PLLCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PLLCTL), CLK->PLLCTL);
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->STATUS), CLK->STATUS);
	CONSOLE_PutMessage(inst, "CLKOCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKOCTL), CLK->CLKOCTL);
	CONSOLE_PutMessage(inst, "CLKDCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDCTL), CLK->CLKDCTL);
	CONSOLE_PutMessage(inst, "CLKDSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDSTS), CLK->CLKDSTS);	
	CONSOLE_PutMessage(inst, "CDUPB(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CDUPB), CLK->CDUPB);
	CONSOLE_PutMessage(inst, "CDLOWB(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CDLOWB), CLK->CDLOWB);
	CONSOLE_PutMessage(inst, "PMUCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PMUCTL), CLK->PMUCTL);
	CONSOLE_PutMessage(inst, "PMUSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PMUSTS), CLK->PMUSTS);
	CONSOLE_PutMessage(inst, "LDOCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->LDOCTL), CLK->LDOCTL);	
	CONSOLE_PutMessage(inst, "SWKDBCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->SWKDBCTL), CLK->SWKDBCTL);
	CONSOLE_PutMessage(inst, "PASWKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PASWKCTL), CLK->PASWKCTL);
	CONSOLE_PutMessage(inst, "PBSWKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PBSWKCTL), CLK->PBSWKCTL);
	CONSOLE_PutMessage(inst, "PCSWKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PCSWKCTL), CLK->PCSWKCTL);
	CONSOLE_PutMessage(inst, "PDSWKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PDSWKCTL), CLK->PDSWKCTL);	
	CONSOLE_PutMessage(inst, "IOPDCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->IOPDCTL), CLK->IOPDCTL);
		
	CONSOLE_PutMessage(inst, "\r\nCurrent Clock Information...\r\n");
	CONSOLE_PutMessage(inst, "HXT_Frequency = %d Hz\r\n", CLK_GetHXTFreq());
	CONSOLE_PutMessage(inst, "LXT_Frequency = %d Hz\r\n", CLK_GetLXTFreq());
	CONSOLE_PutMessage(inst, "HCLK_Frequency = %d Hz\r\n", CLK_GetHCLKFreq());
	CONSOLE_PutMessage(inst, "PCLK0_Frequency = %d Hz\r\n", CLK_GetPCLK0Freq());		
	CONSOLE_PutMessage(inst, "PCLK1_Frequency = %d Hz\r\n", CLK_GetPCLK1Freq());
	CONSOLE_PutMessage(inst, "CPU_Frequency = %d Hz\r\n", CLK_GetCPUFreq());
	CONSOLE_PutMessage(inst, "PLL_Frequency = %d Hz\r\n", CLK_GetPLLClockFreq());

	CONSOLE_PutMessage(inst, "PMUWK_Source = %d\r\n", CLK_GetPMUWKSrc());
	CONSOLE_PutMessage(inst, "TIMER2_ClockSource = %d\r\n", CLK_GetModuleClockSource(TMR2_MODULE));
	CONSOLE_PutMessage(inst, "TIMER2_ClockDivider = %d\r\n", CLK_GetModuleClockDivider(TMR2_MODULE));	
	CONSOLE_PutMessage(inst, "TIMER3_ClockSource = %d\r\n", CLK_GetModuleClockSource(TMR3_MODULE));
	CONSOLE_PutMessage(inst, "TIMER3_ClockDivider = %d\r\n", CLK_GetModuleClockDivider(TMR3_MODULE));
	CONSOLE_PutMessage(inst, "SPI0_ClockSource = %d\r\n", CLK_GetModuleClockSource(SPI0_MODULE));
	CONSOLE_PutMessage(inst, "SPI0_ClockDivider = %d\r\n", CLK_GetModuleClockDivider(SPI0_MODULE));
	CONSOLE_PutMessage(inst, "UART2_ClockSource = %d\r\n", CLK_GetModuleClockSource(UART2_MODULE));
	CONSOLE_PutMessage(inst, "UART2_ClockDivider = %d\r\n", CLK_GetModuleClockDivider(UART2_MODULE));

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ClkRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ClkRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	
	if (argc == 1)
	{
		clicmd_ClkRegisterUsage(inst);
		clicmd_ClkRegisterDisplay(inst);
		return 1;		
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	/* Get register offset */
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}
	
	/* Get register value */	
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
	{
		return -1;
	}

	/* Set register value */
	*((uint32_t *)(((uint32_t)CLK) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write CLK Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
	return 1;
} /* End of clicmd_ClkRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ClkRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ClkRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "           clk: Access MCU CLK registers\r\n");
	return 1;
} /* End of clicmd_ClkRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ClkRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ClkRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_ClkRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: clk [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_ClkRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_FmcRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_FmcRegisterDisplay(CONSOLE_Inst *inst)
{
	FMC_T *FmcOffset = 0;	
	uint32_t config[2];

	/* Show current settings */	
	CONSOLE_PutMessage(inst, "Current FMC registers...\r\n");	
	CONSOLE_PutMessage(inst, "ISPCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPCTL), FMC->ISPCTL);
	CONSOLE_PutMessage(inst, "ISPADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPADDR), FMC->ISPADDR);
	CONSOLE_PutMessage(inst, "ISPDAT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPDAT), FMC->ISPDAT);
	CONSOLE_PutMessage(inst, "ISPCMD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPCMD), FMC->ISPCMD);
	CONSOLE_PutMessage(inst, "ISPTRG(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPTRG), FMC->ISPTRG);
	CONSOLE_PutMessage(inst, "DFBA(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->DFBA), FMC->DFBA);
	CONSOLE_PutMessage(inst, "ISPSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPSTS), FMC->ISPSTS);
	CONSOLE_PutMessage(inst, "CYCCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->CYCCTL), FMC->CYCCTL);
	CONSOLE_PutMessage(inst, "KPKEY0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEY0), FMC->KPKEY0);
	CONSOLE_PutMessage(inst, "KPKEY1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEY1), FMC->KPKEY1);
	CONSOLE_PutMessage(inst, "KPKEY2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEY2), FMC->KPKEY2);
	CONSOLE_PutMessage(inst, "KPKEYTRG(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEYTRG), FMC->KPKEYTRG);
	CONSOLE_PutMessage(inst, "KPKEYSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEYSTS), FMC->KPKEYSTS);
	CONSOLE_PutMessage(inst, "KPKEYCNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEYCNT), FMC->KPKEYCNT);
	CONSOLE_PutMessage(inst, "KPCNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPCNT), FMC->KPCNT);
	CONSOLE_PutMessage(inst, "MPDAT0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPDAT0), FMC->MPDAT0);
	CONSOLE_PutMessage(inst, "MPDAT1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPDAT1), FMC->MPDAT1);
	CONSOLE_PutMessage(inst, "MPDAT2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPDAT2), FMC->MPDAT2);
	CONSOLE_PutMessage(inst, "MPDAT3(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPDAT3), FMC->MPDAT3);
	CONSOLE_PutMessage(inst, "MPSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPSTS), FMC->MPSTS);
	CONSOLE_PutMessage(inst, "MPADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPADDR), FMC->MPADDR);

	CONSOLE_PutMessage(inst, "\r\nOther information...\r\n");	
	CONSOLE_PutMessage(inst, "APROM base address: 0x%08lx\r\n", FMC_APROM_BASE);
	CONSOLE_PutMessage(inst, "APROM end address: 0x%08lx\r\n", FMC_APROM_END);		
	CONSOLE_PutMessage(inst, "APROM bank0 end address: 0x%08lx\r\n", FMC_APROM_BANK0_END);	
	CONSOLE_PutMessage(inst, "LDROM base address: 0x%08lx\r\n", FMC_LDROM_BASE);	
	CONSOLE_PutMessage(inst, "LDROM end address: 0x%08lx\r\n", FMC_LDROM_END);	
	CONSOLE_PutMessage(inst, "SPROM base address: 0x%08lx\r\n", FMC_SPROM_BASE);	
	CONSOLE_PutMessage(inst, "SPROM end address: 0x%08lx\r\n", FMC_SPROM_END);	
	CONSOLE_PutMessage(inst, "User Configuration address: 0x%08lx\r\n", FMC_CONFIG_BASE);	
	CONSOLE_PutMessage(inst, "User Config 0 address: 0x%08lx\r\n", FMC_USER_CONFIG_0);	
	CONSOLE_PutMessage(inst, "User Config 1 address: 0x%08lx\r\n", FMC_USER_CONFIG_1);	
	CONSOLE_PutMessage(inst, "User Config 2 address: 0x%08lx\r\n", FMC_USER_CONFIG_2);	
	FMC_ReadConfig(config, 2);	
	CONSOLE_PutMessage(inst, "User Configuration: 0x%08lx%08lx\r\n", config[0], config[1]);	
	CONSOLE_PutMessage(inst, "Security ROM base address: 0x%08lx\r\n", FMC_KPROM_BASE);	
	CONSOLE_PutMessage(inst, "OTP flash base address: 0x%08lx\r\n", FMC_OTP_BASE);	
	CONSOLE_PutMessage(inst, "Flash Page Size (4K bytes): 0x%08lx\r\n", FMC_FLASH_PAGE_SIZE);	
	CONSOLE_PutMessage(inst, "APROM Size: 0x%08lx\r\n", FMC_APROM_SIZE);	
	CONSOLE_PutMessage(inst, "APROM Bank Size: 0x%08lx\r\n", FMC_BANK_SIZE);	
	CONSOLE_PutMessage(inst, "LDROM Size (4 Kbytes): 0x%08lx\r\n", FMC_LDROM_SIZE);	
	CONSOLE_PutMessage(inst, "SPROM Size (4 Kbytes): 0x%08lx\r\n", FMC_SPROM_SIZE);	
	CONSOLE_PutMessage(inst, "OTP entry number: 0x%08lx\r\n", FMC_OTP_ENTRY_CNT);	
	
	CONSOLE_PutMessage(inst, "Company ID/CID: 0x%08lx\r\n", FMC_ReadCID());
	CONSOLE_PutMessage(inst, "Product ID/PID: 0x%08lx\r\n", FMC_ReadPID());
	CONSOLE_PutMessage(inst, "Unique ID/UID[2..0]: 0x%08lx%08lx%08lx\r\n", FMC_ReadUID(2), FMC_ReadUID(1), FMC_ReadUID(0));	
	CONSOLE_PutMessage(inst, "UCID[3..0]: 0x%08lx%08lx%08lx%08lx\r\n", FMC_ReadUCID(3), FMC_ReadUCID(2), FMC_ReadUCID(1), FMC_ReadUCID(0));	
	CONSOLE_PutMessage(inst, "Vector mapping address/VECMAP: 0x%08lx\r\n", FMC_GetVECMAP());	
	CONSOLE_PutMessage(inst, "Boot source: %s\r\n", FMC_GetBootSource() ? "LDROM":"APROM");

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_FmcRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_FmcRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	
	if (argc == 1)
	{
		clicmd_FmcRegisterUsage(inst);
		clicmd_FmcRegisterDisplay(inst);
		return 1;		
	}
	
	if (argc == 3)
	{
			/* Get register offset */
			if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
			{
				return -1;
			}
			
			/* Get register value */	
			if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
			{
				return -1;
			}
			
			/* Set register value */
			*((uint32_t *)(((uint32_t)FMC) + HexOffset)) = HexValue;

			CONSOLE_PutMessage(inst, "Write FMC Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
			return 1;
	}

	return -1;
} /* End of clicmd_FmcRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_FmcRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_FmcRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "           fmc: Access MCU FMC registers\r\n");
	return 1;
} /* End of clicmd_FmcRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_FmcRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_FmcRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_FmcRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: fmc [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_FmcRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SpimRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SpimRegisterDisplay(CONSOLE_Inst *inst)
{
	SPIM_T *SpimOffset = 0;

	/* Show current settings */	
	CONSOLE_PutMessage(inst, "Current SPIM registers...\r\n");	
	CONSOLE_PutMessage(inst, "CTL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->CTL0), SPIM->CTL0);
	CONSOLE_PutMessage(inst, "CTL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->CTL1), SPIM->CTL1);
	CONSOLE_PutMessage(inst, "RXCLKDLY(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RXCLKDLY), SPIM->RXCLKDLY);
	CONSOLE_PutMessage(inst, "RX[0](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RX[0]), SPIM->RX[0]);	
	CONSOLE_PutMessage(inst, "RX[1](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RX[1]), SPIM->RX[1]);	
	CONSOLE_PutMessage(inst, "RX[2](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RX[2]), SPIM->RX[2]);	
	CONSOLE_PutMessage(inst, "RX[3](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RX[3]), SPIM->RX[3]);		
	CONSOLE_PutMessage(inst, "TX[0](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->TX[0]), SPIM->TX[0]);	
	CONSOLE_PutMessage(inst, "TX[1](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->TX[1]), SPIM->TX[1]);	
	CONSOLE_PutMessage(inst, "TX[2](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->TX[2]), SPIM->TX[2]);	
	CONSOLE_PutMessage(inst, "TX[3](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->TX[3]), SPIM->TX[3]);			
	CONSOLE_PutMessage(inst, "SRAMADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->SRAMADDR), SPIM->SRAMADDR);
	CONSOLE_PutMessage(inst, "DMACNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->DMACNT), SPIM->DMACNT);	
	CONSOLE_PutMessage(inst, "FADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->FADDR), SPIM->FADDR);
	CONSOLE_PutMessage(inst, "KEY1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->KEY1), SPIM->KEY1);
	CONSOLE_PutMessage(inst, "KEY2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->KEY2), SPIM->KEY2);
	CONSOLE_PutMessage(inst, "DMMCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->DMMCTL), SPIM->DMMCTL);
	CONSOLE_PutMessage(inst, "CTL2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIM->CTL2), SPIM->CTL2);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SpimRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SpimRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t i, j, HexOffset, HexLength, HexValue;
	
	if (argc == 1)
	{
		clicmd_SpimRegisterUsage(inst);
		clicmd_SpimRegisterDisplay(inst);
		return 1;		
	}
	
	for (i=1; i<argc; i++)
	{
		if (!strcmp(argv[i], "-rd") && strlen(argv[i])==3)
		{
			/* Get start address */			
			i++;
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexOffset, strlen(argv[i])) != 0)
			{
				return -1;
			}			
			
			/* Get length */
			i++;
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexLength, strlen(argv[i])) != 0)
			{
				return -1;
			}			
			
			CONSOLE_PutMessage(inst, "Read extFlash, offset: 0x%08lx, length: %u\r\n", HexOffset, HexLength);
			
			SPIM_IO_Read(HexOffset, 0, HexLength, StrBuf, OPCODE_FAST_READ, 1, 1, 1, 1);				
			clicmd_ShowMemoryInHex8b(inst, StrBuf, HexLength);
			return 1;
		}
		else if (!strcmp(argv[i], "-wr") && strlen(argv[i])==3)
		{
			/* Get start address */
			i++;
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexOffset, strlen(argv[i])) != 0)
			{
				return -1;
			}
			
			/* Get data stream */
			i++;
			if (i>=argc)
			{
				return -1;
			}
			for (j=0;(i+j)<argc;j++)
			{
				if (clicmd_HexText2Char((uint8_t*)argv[i+j], &StrBuf[j], strlen(argv[i+j])) != 0)
				{
					break;
				}
			}
			i+=j;
			
			/* Erase flash page */
			CONSOLE_PutMessage(inst, "Erase SPI flash block 0x%x...", HexOffset);
			SPIM_EraseBlock(HexOffset, 0, OPCODE_BE_64K, 1, 1);
			CONSOLE_PutMessage(inst, "done.\r\n");
			
			/* Program data to extFLASH */
			CONSOLE_PutMessage(inst, "Program 0x%x bytes to SPI flash from 0x%x...", j, HexOffset);			
      SPIM_IO_Write(HexOffset, 0, j, StrBuf, OPCODE_PP, 1, 1, 1);
			CONSOLE_PutMessage(inst, "done.\r\n");
			
			return 1;
		}
		else
		{
			/* Get register offset */
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexOffset, strlen(argv[i])) != 0)
			{
				return -1;
			}
			i++;
			
			/* Get register value */	
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexValue, strlen(argv[i])) != 0)
			{
				return -1;
			}
			i++;
			
			/* Set register value */
			*((uint32_t *)(((uint32_t)SPIM) + HexOffset)) = HexValue;

			CONSOLE_PutMessage(inst, "Write SPIM Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
			return 1;
		}
	}
	
	return -1;
} /* End of clicmd_SpimRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SpimRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SpimRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          spim: Access MCU SPIM registers\r\n");
	return 1;
} /* End of clicmd_SpimRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SpimRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SpimRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_SpimRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: spim [<HexOffset> <HexValue>] [-rd HexOffset HexLength]  [-wr HexOffset HexData0...HexDataN]\r\n");
	CONSOLE_PutMessage(inst, "       -rd: Read data from external SPI Flash.\r\n");
	CONSOLE_PutMessage(inst, "       -wr: Write data to external SPI Flash.\r\n");	
	return 1;
} /* End of clicmd_SpimRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_UARTxRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_UARTxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, UART_T* UARTx)
{
	UART_T *UARTxOffset = 0;

	/* Show current settings */	
	CONSOLE_PutMessage(inst, "Current UART%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "DAT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->DAT), UARTx->DAT);
	CONSOLE_PutMessage(inst, "INTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->INTEN), UARTx->INTEN);
	CONSOLE_PutMessage(inst, "FIFO(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->FIFO), UARTx->FIFO);
	CONSOLE_PutMessage(inst, "LINE(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->LINE), UARTx->LINE);
	CONSOLE_PutMessage(inst, "MODEM(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->MODEM), UARTx->MODEM);
	CONSOLE_PutMessage(inst, "MODEMSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->MODEMSTS), UARTx->MODEMSTS);
	CONSOLE_PutMessage(inst, "FIFOSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->FIFOSTS), UARTx->FIFOSTS);
	CONSOLE_PutMessage(inst, "INTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->INTSTS), UARTx->INTSTS);
	CONSOLE_PutMessage(inst, "TOUT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->TOUT), UARTx->TOUT);
	CONSOLE_PutMessage(inst, "BAUD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->BAUD), UARTx->BAUD);	
	CONSOLE_PutMessage(inst, "IRDA(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->IRDA), UARTx->IRDA);	
	CONSOLE_PutMessage(inst, "ALTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->ALTCTL), UARTx->ALTCTL);	
	CONSOLE_PutMessage(inst, "FUNCSEL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->FUNCSEL), UARTx->FUNCSEL);	
	CONSOLE_PutMessage(inst, "LINCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->LINCTL), UARTx->LINCTL);	
	CONSOLE_PutMessage(inst, "LINSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->LINSTS), UARTx->LINSTS);	
	CONSOLE_PutMessage(inst, "BRCOMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->BRCOMP), UARTx->BRCOMP);	
	CONSOLE_PutMessage(inst, "WKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->WKCTL), UARTx->WKCTL);		
	CONSOLE_PutMessage(inst, "WKSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->WKSTS), UARTx->WKSTS);		
	CONSOLE_PutMessage(inst, "DWKCOMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->DWKCOMP), UARTx->DWKCOMP);			

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_UARTxRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_UARTxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	UART_T *UARTx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		/* Get instance index */	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			UARTx = UART0;
			break;
		case 1:
			UARTx = UART1;
			break;
		case 2:
			UARTx = UART2;
			break;
		case 3:
			UARTx = UART3;
			break;
		case 4:
			UARTx = UART4;
			break;
		case 5:
			UARTx = UART5;
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_UARTxRegisterUsage(inst);
			clicmd_UARTxRegisterDisplay(inst, HexIndex, UARTx);
			return 1;
		}
	
		/* Get register offset */
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		/* Get register value */	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		/* Set register value */
		*((uint32_t *)(((uint32_t)UARTx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write UART%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
} /* End of clicmd_UARTxRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_UARTxRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_UARTxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         uartx: Access MCU UARTx reigisters\r\n");
	return 1;
} /* End of clicmd_UARTxRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_UARTxRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_UARTxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_UARTxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: uartx <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_UARTxRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_TMRxRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_TMRxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, TIMER_T* TMRx)
{
	TIMER_T *TMRxOffset = 0;
	
	/* Show current settings */
	CONSOLE_PutMessage(inst, "Current TMR%u clock = %uHz, registers...\r\n", index, TIMER_GetModuleClock(TMRx));
	CONSOLE_PutMessage(inst, "CTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->CTL), TMRx->CTL);
	CONSOLE_PutMessage(inst, "CMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->CMP), TMRx->CMP);
	CONSOLE_PutMessage(inst, "INTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->INTSTS), TMRx->INTSTS);
	CONSOLE_PutMessage(inst, "CNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->CNT), TMRx->CNT);	
	CONSOLE_PutMessage(inst, "CAP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->CAP), TMRx->CAP);	
	CONSOLE_PutMessage(inst, "EXTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->EXTCTL), TMRx->EXTCTL);	
	CONSOLE_PutMessage(inst, "EINTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->EINTSTS), TMRx->EINTSTS);	
	CONSOLE_PutMessage(inst, "TRGCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->TRGCTL), TMRx->TRGCTL);	
	CONSOLE_PutMessage(inst, "ALTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->ALTCTL), TMRx->ALTCTL);	
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_TMRxRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_TMRxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	TIMER_T *TMRx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		/* Get instance index */	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			TMRx = TIMER0;
			break;
		case 1:
			TMRx = TIMER1;
			break;
		case 2:
			TMRx = TIMER2;
			break;
		case 3:
			TMRx = TIMER3;
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_TMRxRegisterUsage(inst);
			clicmd_TMRxRegisterDisplay(inst, HexIndex, TMRx);
			return 1;
		}
	
		/* Get register offset */
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		/* Get register value */	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		/* Set register value */
		*((uint32_t *)(((uint32_t)TMRx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write TMR%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
} /* End of clicmd_TMRxRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_TMRxRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_TMRxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          tmrx: Access MCU TMRx registers\r\n");
	return 1;
} /* End of clicmd_TMRxRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_TMRxRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_TMRxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_TMRxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: tmrx <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_TMRxRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SPIxRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SPIxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, SPI_T* SPIx)
{
	SPI_T *SPIxOffset = 0;
	
	/* Show current settings */
	CONSOLE_PutMessage(inst, "Current SPI%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "CTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->CTL), SPIx->CTL);	
	CONSOLE_PutMessage(inst, "CLKDIV(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->CLKDIV), SPIx->CLKDIV);		
	CONSOLE_PutMessage(inst, "SSCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->SSCTL), SPIx->SSCTL);			
	CONSOLE_PutMessage(inst, "PDMACTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->PDMACTL), SPIx->PDMACTL);				
	CONSOLE_PutMessage(inst, "FIFOCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->FIFOCTL), SPIx->FIFOCTL);				
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->STATUS), SPIx->STATUS);
	CONSOLE_PutMessage(inst, "TX(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->TX), SPIx->TX);
	CONSOLE_PutMessage(inst, "RX(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->RX), SPIx->RX);				
	CONSOLE_PutMessage(inst, "I2SCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->I2SCTL), SPIx->I2SCTL);				
	CONSOLE_PutMessage(inst, "I2SCLK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->I2SCLK), SPIx->I2SCLK);				
	CONSOLE_PutMessage(inst, "I2SSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->I2SSTS), SPIx->I2SSTS);				
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SPIxRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SPIxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	SPI_T *SPIx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		/* Get instance index */	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			SPIx = SPI0;
			break;
		case 1:
			SPIx = SPI1;
			break;
		case 2:
			SPIx = SPI2;
			break;
		case 3:
			SPIx = SPI3;
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_SPIxRegisterUsage(inst);
			clicmd_SPIxRegisterDisplay(inst, HexIndex, SPIx);
			return 1;
		}
	
		/* Get register offset */
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		/* Get register value */	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		/* Set register value */
		*((uint32_t *)(((uint32_t)SPIx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write SPI%01d Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
} /* End of clicmd_SPIxRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SPIxRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SPIxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          spix: Access MCU SPIx registers\r\n");
	return 1;
} /* End of clicmd_SPIxRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_SPIxRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_SPIxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_SPIxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: spix <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_SPIxRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_GPIOxRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_GPIOxRegisterDisplay(CONSOLE_Inst *inst, uint8_t Port, GPIO_T *GPIOx)
{
	GPIO_T *GPIOxOffet = 0;
	
	CONSOLE_PutMessage(inst, "Current GPIO%c(0x%08lx) registers...\r\n", Port, (uint32_t)GPIOx);			
	CONSOLE_PutMessage(inst, "MODE(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->MODE), GPIOx->MODE);
	CONSOLE_PutMessage(inst, "DINOFF(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->DINOFF), GPIOx->DINOFF);
	CONSOLE_PutMessage(inst, "DOUT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->DOUT), GPIOx->DOUT);
	CONSOLE_PutMessage(inst, "DATMSK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->DATMSK), GPIOx->DATMSK);
	CONSOLE_PutMessage(inst, "PIN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->PIN), GPIOx->PIN);
	CONSOLE_PutMessage(inst, "DBEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->DBEN), GPIOx->DBEN);
	CONSOLE_PutMessage(inst, "INTTYPE(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->INTTYPE), GPIOx->INTTYPE);
	CONSOLE_PutMessage(inst, "INTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->INTEN), GPIOx->INTEN);
	CONSOLE_PutMessage(inst, "INTSRC(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->INTSRC), GPIOx->INTSRC);
	CONSOLE_PutMessage(inst, "SMTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->SMTEN), GPIOx->SMTEN);
	CONSOLE_PutMessage(inst, "SLEWCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->SLEWCTL), GPIOx->SLEWCTL);
	CONSOLE_PutMessage(inst, "PUSEL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->PUSEL), GPIOx->PUSEL);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_GPIOxRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_GPIOxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexPort;
	GPIO_T *GPIOx;
	
	if (argc == 1)
	{
		clicmd_GPIOxRegisterUsage(inst);
		return 1;		
	}
	else if (argc == 2)
	{
		HexPort = argv[1][0] & 0xdf;
	
		switch (HexPort)
		{
		case 'A': GPIOx = (GPIO_T*)GPIOA_BASE; break;
		case 'B': GPIOx = (GPIO_T*)GPIOB_BASE; break;
		case 'C': GPIOx = (GPIO_T*)GPIOC_BASE; break;
		case 'D': GPIOx = (GPIO_T*)GPIOD_BASE; break;
		case 'E': GPIOx = (GPIO_T*)GPIOE_BASE; break;
		case 'F': GPIOx = (GPIO_T*)GPIOF_BASE; break;
		case 'G': GPIOx = (GPIO_T*)GPIOG_BASE; break;
		case 'H': GPIOx = (GPIO_T*)GPIOH_BASE; break;
		default: return -1;
		}
		clicmd_GPIOxRegisterDisplay(inst, HexPort, (GPIO_T*)GPIOx);
		return 1;
	}
	else if (argc != 4)
	{
		return -1;
	}
	
	/* Get port index */
	if (strlen(argv[1]) != 1)
	{
		return -1;
	}

	HexPort = argv[1][0] & 0xdf;
	
	switch (HexPort)
	{
	case 'A': GPIOx = (GPIO_T*)GPIOA_BASE; break;
	case 'B': GPIOx = (GPIO_T*)GPIOB_BASE; break;
	case 'C': GPIOx = (GPIO_T*)GPIOC_BASE; break;
	case 'D': GPIOx = (GPIO_T*)GPIOD_BASE; break;
	case 'E': GPIOx = (GPIO_T*)GPIOE_BASE; break;
	case 'F': GPIOx = (GPIO_T*)GPIOF_BASE; break;
	case 'G': GPIOx = (GPIO_T*)GPIOG_BASE; break;
	case 'H': GPIOx = (GPIO_T*)GPIOH_BASE; break;
	default: return -1;		
	}
	
	/* Get register offset */
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
	{
		return -1;
	}
	
	/* Get register value */	
	if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
	{
		return -1;
	}
	
	/* Set register value */
	*((uint32_t *)(((uint32_t)GPIOx) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write GPIO%c Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexPort, HexOffset, HexValue);	
	return 1;
} /* End of clicmd_GPIOxRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_GPIOxRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_GPIOxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         gpiox: Access MCU GPIOx registers\r\n");
	return 1;
} /* End of clicmd_GPIOxRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_GPIOxRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_GPIOxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_GPIOxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: gpiox [<HexPort> <HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_GPIOxRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_NvicRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_NvicRegisterDisplay(CONSOLE_Inst *inst)
{
	NVIC_Type *NvicOffset = 0;
	uint32_t i;
	
	CONSOLE_PutMessage(inst, "Current NVIC(0x%08lx) registers...\r\n", (uint32_t)NVIC);		
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "ISER[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->ISER[i]), NVIC->ISER[i]);
	}
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "ICER[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->ICER[i]), NVIC->ICER[i]);
	}	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "ISPR[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->ISPR[i]), NVIC->ISPR[i]);
	}	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "ICPR[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->ICPR[i]), NVIC->ICPR[i]);
	}	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "IABR[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->IABR[i]), NVIC->IABR[i]);
	}	
	CONSOLE_PutMessage(inst, "\r\n");				

	
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_NvicRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_NvicRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
		
	if (argc == 1)
	{
		clicmd_NvicRegisterUsage(inst);
		clicmd_NvicRegisterDisplay(inst);		
		return 1;		
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	/* Get register offset */
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}
	
	/* Get register value */	
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
	{
		return -1;
	}
	
	/* Set register value */
	*((uint32_t *)(((uint32_t)NVIC) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write MVIC Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
	return 1;
} /* End of clicmd_NvicRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_NvicRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_NvicRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          nvic: Access MCU NVIC registers\r\n");
	return 1;
} /* End of clicmd_NvicRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_NvicRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_NvicRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_NvicRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: nvic [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_NvicRegisterUsage() */


/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EadcRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EadcRegisterDisplay(CONSOLE_Inst *inst)
{
	EADC_T *EadcOffset = 0;
	uint32_t i=0;
	
	/* Show current settings */
	CONSOLE_PutMessage(inst, "Current EADC(0x%08lx) registers...\r\n", (uint32_t)EADC);			
	for (i=0; i<19; i++)
	{
		CONSOLE_PutMessage(inst, "DAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->DAT[i]), EADC->DAT[i]);
	}
	CONSOLE_PutMessage(inst, "CURDAT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->CURDAT), EADC->CURDAT);
	CONSOLE_PutMessage(inst, "CTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->CTL), EADC->CTL);
	CONSOLE_PutMessage(inst, "SWTRG(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->SWTRG), EADC->SWTRG);
	CONSOLE_PutMessage(inst, "PENDSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->PENDSTS), EADC->PENDSTS);
	CONSOLE_PutMessage(inst, "OVSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->OVSTS), EADC->OVSTS);	
	for (i=0; i<19; i++)
	{
		CONSOLE_PutMessage(inst, "SCTL[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->SCTL[i]), EADC->SCTL[i]);
	}
	for (i=0; i<4; i++)
	{
		CONSOLE_PutMessage(inst, "INTSRC[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->INTSRC[i]), EADC->INTSRC[i]);
	}
	for (i=0; i<4; i++)
	{
		CONSOLE_PutMessage(inst, "CMP[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->CMP[i]), EADC->CMP[i]);
	}
	CONSOLE_PutMessage(inst, "STATUS0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->STATUS0), EADC->STATUS0);
	CONSOLE_PutMessage(inst, "STATUS1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->STATUS1), EADC->STATUS1);
	CONSOLE_PutMessage(inst, "STATUS2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->STATUS2), EADC->STATUS2);
	CONSOLE_PutMessage(inst, "STATUS3(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->STATUS3), EADC->STATUS3);
	for (i=0; i<4; i++)
	{
		CONSOLE_PutMessage(inst, "DDAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->DDAT[i]), EADC->DDAT[i]);
	}
	CONSOLE_PutMessage(inst, "PWRM(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->PWRM), EADC->PWRM);
	CONSOLE_PutMessage(inst, "CALCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->CALCTL), EADC->CALCTL);
	CONSOLE_PutMessage(inst, "CALDWRD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->CALDWRD), EADC->CALDWRD);
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EadcRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EadcRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
		
	if (argc == 1)
	{
		clicmd_EadcRegisterUsage(inst);
		clicmd_EadcRegisterDisplay(inst);		
		return 1;		
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	/* Get register offset */
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}
	
	/* Get register value */	
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
	{
		return -1;
	}
	
	/* Set register value */
	*((uint32_t *)(((uint32_t)EADC) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write EADC Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
	return 1;
} /* End of clicmd_EadcRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EadcRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EadcRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          eadc: Access MCU EADC registers\r\n");
	return 1;
} /* End of clicmd_EadcRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EadcRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EadcRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_EadcRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: eadc [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_EadcRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_QEIxRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_QEIxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, QEI_T* QEIx)
{
	QEI_T *QEIxOffset = 0;
	
	/* Show current settings */
	CONSOLE_PutMessage(inst, "Current QEI%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "CNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNT), QEIx->CNT);	
	CONSOLE_PutMessage(inst, "CNTHOLD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNTHOLD), QEIx->CNTHOLD);
	CONSOLE_PutMessage(inst, "CNTLATCH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNTLATCH), QEIx->CNTLATCH);	
	CONSOLE_PutMessage(inst, "CNTCMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNTCMP), QEIx->CNTCMP);	
	CONSOLE_PutMessage(inst, "CNTMAX(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNTMAX), QEIx->CNTMAX);
	CONSOLE_PutMessage(inst, "CTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CTL), QEIx->CTL);
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->STATUS), QEIx->STATUS);	
	
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_QEIxRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_QEIxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	QEI_T *QEIx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		/* Get instance index */	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			QEIx = QEI0;
			break;
		case 1:
			QEIx = QEI1;
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_QEIxRegisterUsage(inst);
			clicmd_QEIxRegisterDisplay(inst, HexIndex, QEIx);
			return 1;
		}
	
		/* Get register offset */
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		/* Get register value */	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		/* Set register value */
		*((uint32_t *)(((uint32_t)QEIx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write QEI%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
} /* End of clicmd_QEIxRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_QEIxRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_QEIxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          qeix: Access MCU QEIx registers\r\n");
	return 1;
} /* End of clicmd_QEIxRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_QEIxRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_QEIxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_QEIxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: qeix <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_QEIxRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ECAPxRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ECAPxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, ECAP_T* ECAPx)
{
	ECAP_T *ECAPxOffset = 0;
	
	/* Show current settings */
	CONSOLE_PutMessage(inst, "Current ECAP%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "CNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->CNT), ECAPx->CNT);
	CONSOLE_PutMessage(inst, "HLD0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->HLD0), ECAPx->HLD0);
	CONSOLE_PutMessage(inst, "HLD1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->HLD1), ECAPx->HLD1);
	CONSOLE_PutMessage(inst, "HLD2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->HLD2), ECAPx->HLD2);
	CONSOLE_PutMessage(inst, "CNTCMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->CNTCMP), ECAPx->CNTCMP);
	CONSOLE_PutMessage(inst, "CTL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->CTL0), ECAPx->CTL0);
	CONSOLE_PutMessage(inst, "CTL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->CTL1), ECAPx->CTL1);
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->STATUS), ECAPx->STATUS);	

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ECAPxRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ECAPxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	ECAP_T *ECAPx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		/* Get instance index */	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			ECAPx = ECAP0;
			break;
		case 1:
			ECAPx = ECAP1;
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_ECAPxRegisterUsage(inst);
			clicmd_ECAPxRegisterDisplay(inst, HexIndex, ECAPx);
			return 1;
		}
	
		/* Get register offset */
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		/* Get register value */	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		/* Set register value */
		*((uint32_t *)(((uint32_t)ECAPx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write ECAP%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
} /* End of clicmd_ECAPxRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ECAPxRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ECAPxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         ecapx: Access MCU ECAPx registers\r\n");
	return 1;
} /* End of clicmd_ECAPxRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ECAPxRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ECAPxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_ECAPxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: ecapx <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_ECAPxRegisterUsage() */


/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EPWMxRegisterDisplay()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EPWMxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, EPWM_T* EPWMx)
{
	uint32_t i;
	EPWM_T *EPWMxOffset = 0;
	
	/* Show current settings */
	CONSOLE_PutMessage(inst, "Current EPWM%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "CTL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CTL0), EPWMx->CTL0);
	CONSOLE_PutMessage(inst, "CTL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CTL1), EPWMx->CTL1);
	CONSOLE_PutMessage(inst, "SYNC(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SYNC), EPWMx->SYNC);
	CONSOLE_PutMessage(inst, "SWSYNC(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SWSYNC), EPWMx->SWSYNC);
	CONSOLE_PutMessage(inst, "CLKSRC(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CLKSRC), EPWMx->CLKSRC);
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "CLKPSC[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CLKPSC[i]), EPWMx->CLKPSC[i]);
	}
	CONSOLE_PutMessage(inst, "CNTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CNTEN), EPWMx->CNTEN);
	CONSOLE_PutMessage(inst, "CNTCLR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CNTCLR), EPWMx->CNTCLR);
	CONSOLE_PutMessage(inst, "LOAD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->LOAD), EPWMx->LOAD);
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "PERIOD[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->PERIOD[i]), EPWMx->PERIOD[i]);
	}
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "CMPDAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CMPDAT[i]), EPWMx->CMPDAT[i]);
	}
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "DTCTL[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->DTCTL[i]), EPWMx->DTCTL[i]);
	}	
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "PHS[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->PHS[i]), EPWMx->PHS[i]);
	}		
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "CNT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CNT[i]), EPWMx->CNT[i]);
	}
	CONSOLE_PutMessage(inst, "WGCTL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->WGCTL0), EPWMx->WGCTL0);
	CONSOLE_PutMessage(inst, "WGCTL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->WGCTL1), EPWMx->WGCTL1);
	CONSOLE_PutMessage(inst, "MSKEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->MSKEN), EPWMx->MSKEN);
	CONSOLE_PutMessage(inst, "MSK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->MSK), EPWMx->MSK);	
	CONSOLE_PutMessage(inst, "BNF(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->BNF), EPWMx->BNF);
	CONSOLE_PutMessage(inst, "FAILBRK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->FAILBRK), EPWMx->FAILBRK);
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "BRKCTL[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->BRKCTL[i]), EPWMx->BRKCTL[i]);
	}	
	CONSOLE_PutMessage(inst, "POLCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->POLCTL), EPWMx->POLCTL);
	CONSOLE_PutMessage(inst, "POEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->POEN), EPWMx->POEN);
	CONSOLE_PutMessage(inst, "SWBRK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SWBRK), EPWMx->SWBRK);
	CONSOLE_PutMessage(inst, "INTEN0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->INTEN0), EPWMx->INTEN0);
	CONSOLE_PutMessage(inst, "INTEN1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->INTEN1), EPWMx->INTEN1);
	CONSOLE_PutMessage(inst, "INTSTS0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->INTSTS0), EPWMx->INTSTS0);
	CONSOLE_PutMessage(inst, "INTSTS1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->INTSTS1), EPWMx->INTSTS1);
	CONSOLE_PutMessage(inst, "DACTRGEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->DACTRGEN), EPWMx->DACTRGEN);
	CONSOLE_PutMessage(inst, "EADCTS0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->EADCTS0), EPWMx->EADCTS0);
	CONSOLE_PutMessage(inst, "EADCTS1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->EADCTS1), EPWMx->EADCTS1);
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "FTCMPDAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->FTCMPDAT[i]), EPWMx->FTCMPDAT[i]);
	}		
	CONSOLE_PutMessage(inst, "SSCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SSCTL), EPWMx->SSCTL);	
	CONSOLE_PutMessage(inst, "SSTRG(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SSTRG), EPWMx->SSTRG);	
	CONSOLE_PutMessage(inst, "LEBCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->LEBCTL), EPWMx->LEBCTL);	
	CONSOLE_PutMessage(inst, "LEBCNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->LEBCNT), EPWMx->LEBCNT);	
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->WGCTL1), EPWMx->STATUS);	
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "IFA[%u](0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->IFA[i]), EPWMx->IFA[i]);
	}	
	CONSOLE_PutMessage(inst, "AINTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->AINTSTS), EPWMx->AINTSTS);	
	CONSOLE_PutMessage(inst, "AINTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->AINTEN), EPWMx->AINTEN);	
	CONSOLE_PutMessage(inst, "APDMACTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->APDMACTL), EPWMx->APDMACTL);	
	CONSOLE_PutMessage(inst, "CAPINEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPINEN), EPWMx->CAPINEN);	
	CONSOLE_PutMessage(inst, "CAPCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPCTL), EPWMx->CAPCTL);	
	CONSOLE_PutMessage(inst, "CAPSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPSTS), EPWMx->CAPSTS);	
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "CAPDAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CAPDAT[i]), EPWMx->CAPDAT[i]);
	}		
	CONSOLE_PutMessage(inst, "PDMACTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->PDMACTL), EPWMx->PDMACTL);	
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "PDMACAP[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->PDMACAP[i]), EPWMx->PDMACAP[i]);
	}		
	CONSOLE_PutMessage(inst, "CAPIEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPIEN), EPWMx->CAPIEN);	
	CONSOLE_PutMessage(inst, "CAPIF(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPIF), EPWMx->CAPIF);		
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "PBUF[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->PBUF[i]), EPWMx->PBUF[i]);
	}	
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "CMPBUF[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CMPBUF[i]), EPWMx->CMPBUF[i]);
	}		
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "CPSCBUF[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CPSCBUF[i]), EPWMx->CPSCBUF[i]);
	}	
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "FTCBUF[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->FTCBUF[i]), EPWMx->FTCBUF[i]);
	}		
	CONSOLE_PutMessage(inst, "FTCI(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->FTCI), EPWMx->FTCI);		

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EPWMxRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EPWMxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	EPWM_T *EPWMx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		/* Get instance index */	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			EPWMx = EPWM0;
			break;
		case 1:
			EPWMx = EPWM1;
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_EPWMxRegisterUsage(inst);
			clicmd_EPWMxRegisterDisplay(inst, HexIndex, EPWMx);
			return 1;
		}
	
		/* Get register offset */
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		/* Get register value */	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		/* Set register value */
		*((uint32_t *)(((uint32_t)EPWMx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write EPWM%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
} /* End of clicmd_EPWMxRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EPWMxRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EPWMxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         epwmx: Access MCU EPWMx registers\r\n");
	return 1;
} /* End of clicmd_EPWMxRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ECAPxRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EPWMxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_EPWMxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: epwmx <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
} /* End of clicmd_EPWMxRegisterUsage() */

#endif /* End of CLI_MCU_DBG_CMD_ENABLE */

#if CLI_ESC_DBG_CMD_ENABLE

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EscRegister()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EscRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t i;
	
	if (argc == 1)
	{
		clicmd_EscRegisterUsage(inst);
		
		clicmd_ShowEscFeatures(inst);
		clicmd_ShowEscAlInfo(inst);
		clicmd_ShowEscWdInfo(inst);
		clicmd_ShowEscFmmuSmInfo(inst);
		clicmd_ShowEscDcInfo(inst);
	}
	else
	{
		for (i=1; i<argc; i++)
		{
			if (!strcmp(argv[i], "-ft") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscFeatures(inst);
			}
			else if (!strcmp(argv[i], "-al") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscAlInfo(inst);
			}		
			else if (!strcmp(argv[i], "-wd") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscWdInfo(inst);
			}	
			else if (!strcmp(argv[i], "-sm") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscFmmuSmInfo(inst);
			}			
			else if (!strcmp(argv[i], "-dc") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscDcInfo(inst);
			}			
			else
			{
				CONSOLE_PutMessage(inst, "Unsupported parameter:%s\r\n", argv[i]);				
				return -1;
			}
		}//for
		
	}
	
	return 1;
} /* End of clicmd_EscRegister() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EscRegisterHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EscRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "        escreg: Show ESC registers\r\n");

	return 1;
} /* End of clicmd_EscRegisterHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EscRegisterUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EscRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_EscRegisterHelp(inst);
	
	CONSOLE_PutMessage(inst, "Usage: escreg [-ft] [-al] [-wd] [-sm] [-dc]\r\n");
	return 1;
} /* End of clicmd_EscRegisterUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowTestBuffer()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ShowTestBuffer(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	TEST_TEMP *pTemp = &pTestCtrl->Temp;
	
	CONSOLE_PutMessage(inst, "Current transfer buffer content...\r\n");
	CONSOLE_PutMessage(inst, "StartAddress=0x%x, EndAddress=0x%x\r\n"
								, pTestCtrl->Parameter.StartAddress
								, pTestCtrl->Parameter.EndAddress
								);

	CONSOLE_PutMessage(inst, "TxLen=0x%x, TxBuf[]=\r\n", pTemp->TxLen);
	clicmd_ShowMemoryInHex8b(inst, pTemp->TxBuf, pTemp->TxLen);		
	
	CONSOLE_PutMessage(inst, "RxLen=0x%x, RxBuf[]=\r\n", pTemp->RxLen);
	clicmd_ShowMemoryInHex8b(inst, pTemp->RxBuf, pTemp->RxLen);		
	
	CONSOLE_PutMessage(inst, "\r\n");	
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowTestTemporaryStatus()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ShowTestTemporaryStatus(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	TEST_TEMP *pTemp = &pTestCtrl->Temp;
	
	CONSOLE_PutMessage(inst, "Current temporary status...\r\n");
	CONSOLE_PutMessage(inst, "  SPI DataSizeInWrite = %d\r\n", pTemp->Spis.DataSizeInWrite);
	CONSOLE_PutMessage(inst, "  SPI DataSizeInRead = %d\r\n", pTemp->Spis.DataSizeInRead);	
	CONSOLE_PutMessage(inst, "  SPI AddrSizeInWrite = %d\r\n", pTemp->Spis.AddrSizeInWrite);
	CONSOLE_PutMessage(inst, "  SPI AddrSizeInRead = %d\r\n", pTemp->Spis.AddrSizeInRead);
	
	CONSOLE_PutMessage(inst, "\r\n");	
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowPdiMemoryTestLog()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ShowPdiMemoryTestLog(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	int lineNum;
	TEST_TIME *pTime;
	TEST_TEMP *pTemp = &pTestCtrl->Temp;
	
	CONSOLE_PutMessage(inst, "PDI echo test log...\r\n");
	CONSOLE_PutMessage(inst, "DD:HH:MM:SS Round    Ok       Err      SpiWr/RdAddrSize SpiWr/RdDataSize HSTSR\r\n");
	lineNum	= 2;

	pTime = TEST_GetTime();
	CONSOLE_PutMessage(inst, "%02d:%02d:%02d:%02d %08lx %08lx %08lx %01d/%01d              %01d/%01d              %02x\r\n"
								, pTime->Day
								, pTime->Hour
								, pTime->Minute
								, pTime->Second
								, pTestCtrl->Record.RoundCnt
								, pTestCtrl->Record.OkCnt
								, pTestCtrl->Record.ErrorCnt
								, pTemp->Spis.AddrSizeInWrite
								, pTemp->Spis.AddrSizeInRead
								, pTemp->Spis.DataSizeInWrite
								, pTemp->Spis.DataSizeInRead
								, pTemp->HostInterfaceErrorStatus
								);								
	lineNum++;
	return lineNum;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowPdiMemoryTestParameter()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ShowPdiMemoryTestParameter(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	TEST_PARAMETER *pParameter = &pTestCtrl->Parameter;
	
	CONSOLE_PutMessage(inst, "Your test parameter...\r\n");
	CONSOLE_PutMessage(inst, "-loop=0x%x, -sta=0x%x, -end=0x%x, -ptt=0x%x, -lut=%d, -dfm=0x%x, -dfmr=0x%x, -sam=0x%x\r\n"
								, pParameter->LoopCount
								, pParameter->StartAddress
								, pParameter->EndAddress
								, pParameter->PatternType	
								, pParameter->LogUpdateTime		
								, pParameter->Spis.DataSizeInWrite			
								, pParameter->Spis.DataSizeInRead				
								, pParameter->Spis.AddrSize
								);
	CONSOLE_PutMessage(inst, "-ptn(%dBytes)=", pParameter->InitDataLen);
	clicmd_ShowMemoryInHex8b(inst, pParameter->InitData, pParameter->InitDataLen);		
	
	CONSOLE_PutMessage(inst, "\r\n");	
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiMemoryTest()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiMemoryTest(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint8_t lineNum;
	uint32_t i;
	TEST_PARAMETER *pParameter = &clicmd_testCtrl.Parameter;
	
	if (inst->State == CLI_STATE_COMMAND_WAIT)
	{
		if (inst->GetChar() == CLICMD_CTRL_C)
		{
			clicmd_testCtrl.Terminated = 1;
		}		
			
		TEST_PdiMemoryTest(&clicmd_testCtrl);			

		if (inst->Timer==0)
		{
			lineNum = clicmd_ShowPdiMemoryTestLog(inst, &clicmd_testCtrl);			
			if (clicmd_testCtrl.State == TEST_DONE)
			{
				inst->State = CLI_STATE_COMMAND;
				CONSOLE_PutMessage(inst, "\r\n");
				if (clicmd_testCtrl.Record.ErrorCnt)
				{
					clicmd_ShowTestBuffer(inst, &clicmd_testCtrl);				
					clicmd_ShowTestTemporaryStatus(inst, &clicmd_testCtrl);					
					return -1;
				}
				else
				{
					return 1;		
				}
			}
			else
			{			
				inst->PutChar(0x0D);
				while (lineNum--)
				{
					inst->PutChar(0x1B);
					inst->PutChar(0x5B);		
					inst->PutChar(0x41);					
				}
				inst->Timer = pParameter->LogUpdateTime;
			}
		}
		return 1;
	}
	else if (argc == 1)
	{
		clicmd_PdiMemoryTestUsage(inst);
		clicmd_ShowTestBuffer(inst, &clicmd_testCtrl);
		clicmd_ShowTestTemporaryStatus(inst, &clicmd_testCtrl);
		return 1;
	}
	else
	{
		memset((uint8_t*)&clicmd_testCtrl, 0, sizeof(clicmd_testCtrl));
		
		/* Load default parameters */
		pParameter->LoopCount = TEST_DEF_LOOP_COUNT;
		pParameter->LogUpdateTime = TEST_DEF_LOG_UPDATE_TIME;
		pParameter->StartAddress = TEST_DEF_START_RAMADDR;
		pParameter->EndAddress = TEST_DEF_START_RAMADDR + TEST_DEF_RAM_SIZE;		
		pParameter->PatternType = TEST_DEF_PATTERN_TYPE;				
		pParameter->InitDataLen = 2;
		pParameter->InitData[0] = 0x55;		
		pParameter->InitData[1] = 0xAA;				
		pParameter->Spis.AddrSize = TEST_DEF_SPI_ADDR_SIZE;
		pParameter->Spis.DataSizeInWrite = TEST_DEF_SPI_DATA_SIZE_IN_WR;				
		pParameter->Spis.DataSizeInRead = TEST_DEF_SPI_DATA_SIZE_IN_RD;		
		
		for (i=1; i<argc; i++)
		{
			if (!strcmp(argv[i], "-loop") && strlen(argv[i])==5)
			{
				i++;			
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->LoopCount, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-lut") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &pParameter->LogUpdateTime, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}			
			else if (!strcmp(argv[i], "-sta") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->StartAddress, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}		
			else if (!strcmp(argv[i], "-end") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->EndAddress, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-ptt") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->PatternType, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-ptn") && strlen(argv[i])==4)
			{
				i++;
				
					uint8_t *pbuf8 = pParameter->InitData;	
					pParameter->InitDataLen = 0;
					for (; i<argc; i++)
					{					
						if ((argv[i][0] == '-') ||
								(pParameter->InitDataLen >= TEST_INIT_DATA_BUF_SIZE) || 
								(clicmd_HexText2Char((uint8_t*)argv[i], pbuf8, strlen(argv[i])) != 0))
						{
							i--;
							break;
						}
						pbuf8++;
						pParameter->InitDataLen++;						
					}					
			}
			else if (!strcmp(argv[i], "-dfm") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Char((uint8_t*)argv[i], (uint8_t*)&pParameter->Spis.DataSizeInWrite, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-dfmr") && strlen(argv[i])==5)
			{
				i++;
				if (clicmd_HexText2Char((uint8_t*)argv[i], (uint8_t*)&pParameter->Spis.DataSizeInRead, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}			
			else if (!strcmp(argv[i], "-sam") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Char((uint8_t*)argv[i], (uint8_t*)&pParameter->Spis.AddrSize, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-trg") && strlen(argv[i])==4)
			{
			  DBG_TriggerOutput(1);
				DBG_TriggerOutput(0);
				CONSOLE_PutMessage(inst, "Generate trigger output!\r\n");
				return 1;
			}			
			else
			{
				CONSOLE_PutMessage(inst, "Unsupported parameter:%s\r\n", argv[i]);				
				return -1;
			}
		}//for
		
		/* Parameter check */
		if (pParameter->StartAddress > pParameter->EndAddress)
		{
			CONSOLE_PutMessage(inst, "StartAddress cannot larger than EndAddress\r\n");			
			return -1;
		}
		
		clicmd_ShowPdiMemoryTestParameter(inst, &clicmd_testCtrl);
		CONSOLE_PutMessage(inst, "Start PDI memory test use ESC chip select...\r\n");
		inst->State = CLI_STATE_COMMAND_WAIT;		
	}

	return 1;
} /* End of clicmd_PdiMemoryTest() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiMemoryTestHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiMemoryTestHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         pdimt: PDI memory access test\r\n");	
	return 1;
} /* End of clicmd_PdiMemoryTest() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiMemoryTestUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiMemoryTestUsage(CONSOLE_Inst *inst)
{
	clicmd_PdiMemoryTestHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: pdimt [-loop HexValue] [-sta HexValue] [-end HexValue] [-ptt HexValue] [-ptn <HexValue0>...<HexValueN>]\r\n");
	CONSOLE_PutMessage(inst, "             [-lut DecValue] [-dfm HexValue] [-dfmr HexValue] [-sam HexValue] [-csm HexValue] [-trg]\r\n");	
	CONSOLE_PutMessage(inst, "       -loop: Specifies the number of test rounds.\r\n");	
	CONSOLE_PutMessage(inst, "              0: Infinite test, until press \"Ctrl-C\".\r\n");
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_DEF_LOOP_COUNT);
	CONSOLE_PutMessage(inst, "       -lut: Specifies the update time of log, unit in 100ms\r\n");		
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_DEF_LOG_UPDATE_TIME);		
	CONSOLE_PutMessage(inst, "       -sta: Specifies the start address of target memory to be test.\r\n");	
	CONSOLE_PutMessage(inst, "              Default: 0x%08lx\r\n", TEST_DEF_START_RAMADDR);	
	CONSOLE_PutMessage(inst, "       -end: Specifies the end address of target memory to be test.\r\n");		
	CONSOLE_PutMessage(inst, "              Default: 0x%08lx\r\n", TEST_DEF_START_RAMADDR + TEST_DEF_RAM_SIZE);
	CONSOLE_PutMessage(inst, "       -ptt: Specifies the data pattern type.\r\n");	
	CONSOLE_PutMessage(inst, "              0: Fixed, using \"-ptn\" parameter to specify pattern.\r\n");		
	CONSOLE_PutMessage(inst, "              1: Increment, using \"-ptn\" parameter to specify initial pattern.\r\n");			
	CONSOLE_PutMessage(inst, "              2: Decrement, using \"-ptn\" parameter to specify initial pattern.\r\n");		
	CONSOLE_PutMessage(inst, "              3: Random.\r\n");
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_DEF_PATTERN_TYPE);
	CONSOLE_PutMessage(inst, "       -ptn: Specifies the pattern for \"Fixed\" type\r\n");		
	CONSOLE_PutMessage(inst, "             or initial pattern for other types.\r\n");
	CONSOLE_PutMessage(inst, "              Default: 0x55 0xAA\r\n");	
	CONSOLE_PutMessage(inst, "       -dfm: Specifies data fragment mode for ASIX_SPI/BECKHOFF_SPI write operation.\r\n");		
	CONSOLE_PutMessage(inst, "             this parameter is valid only for ASIX_SPI/BECKHOFF_SPI pdi mode.\r\n");
	CONSOLE_PutMessage(inst, "              0: Randomly change data fragment size being 1, 2 or 4Byte(s) for write operation\r\n");
	CONSOLE_PutMessage(inst, "              1: 1 byte write\r\n");
	CONSOLE_PutMessage(inst, "              2: 2 byte write\r\n");		
	CONSOLE_PutMessage(inst, "              3: 3 byte write\r\n");				
	CONSOLE_PutMessage(inst, "              4: 4 byte write\r\n");			
	CONSOLE_PutMessage(inst, "              N: N byte write\r\n");
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_DEF_SPI_DATA_SIZE_IN_WR);
	CONSOLE_PutMessage(inst, "       -dfmr: Specifies data fragment mode for ASIX_SPI/BECKHOFF_SPI read operation.\r\n");		
	CONSOLE_PutMessage(inst, "             this parameter is valid only for ASIX_SPI/BECKHOFF_SPI pdi mode.\r\n");
	CONSOLE_PutMessage(inst, "              0: Randomly change data fragment size being 1, 2 or 4Byte(s) for read operation\r\n");
	CONSOLE_PutMessage(inst, "              1: 1 byte read\r\n");
	CONSOLE_PutMessage(inst, "              2: 2 byte read\r\n");		
	CONSOLE_PutMessage(inst, "              3: 3 byte read\r\n");				
	CONSOLE_PutMessage(inst, "              4: 4 byte read\r\n");			
	CONSOLE_PutMessage(inst, "              N: N byte read\r\n");
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_DEF_SPI_DATA_SIZE_IN_RD);	
	CONSOLE_PutMessage(inst, "       -sam: Specifies ASIX_SPI/BECKHOFF_SPI address mode.\r\n");
	CONSOLE_PutMessage(inst, "             this parameter is valid only for ASIX_SPI/BECKHOFF_SPI pdi mode.\r\n");
	CONSOLE_PutMessage(inst, "              0: Randomly address byte size between 2, 3Bytes for write/read operation\r\n");
	CONSOLE_PutMessage(inst, "              2: 2 byte address\r\n");		
	CONSOLE_PutMessage(inst, "              3: 3 byte address\r\n");				
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_DEF_SPI_ADDR_SIZE);
	CONSOLE_PutMessage(inst, "       -csm: Chip Select Mode.\r\n");	
	CONSOLE_PutMessage(inst, "             0: FUN_CS, 1: ESC_CS\r\n");			
	CONSOLE_PutMessage(inst, "       -trg: Trigger output test for debugging.\r\n");	
	CONSOLE_PutMessage(inst, "             Note: please connect PA.15 of Nucleo EVB. to trigger in of your instrument\r\n");			
	CONSOLE_PutMessage(inst, "\r\n");		
	return 1;
} /* End of clicmd_PdiMemoryTestUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiReset()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiReset(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint8_t tmp8;
	
	tmp8 = 'R';
	HW_EscWrite((MEM_ADDR*)&tmp8, 0x41, 1);
	tmp8 = 'E';
	HW_EscWrite((MEM_ADDR*)&tmp8, 0x41, 1);	
	tmp8 = 'S';
	HW_EscWrite((MEM_ADDR*)&tmp8, 0x41, 1);	
	
	return 1;
} /* End of clicmd_PdiReset() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_PdiResetHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_PdiResetHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "        pdirst: Perform PDI reset\r\n");	
	return 1;
} /* End of clicmd_PdiResetHelp() */

#endif /* End of CLI_ESC_DBG_CMD_ENABLE */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EscStack()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EscStack(CONSOLE_Inst *inst, int argc, char **argv)
{
	HW_DEBUG *pDbgCnt;
	pDbgCnt = HW_GetDebugCounter();
		
	CONSOLE_PutMessage(inst, "EscIsrCnt=0x%08lx\r\n", pDbgCnt->EscIsrCnt);		
	CONSOLE_PutMessage(inst, "Sync0IsrCnt=0x%08lx\r\n", pDbgCnt->Sync0IsrCnt);	
	CONSOLE_PutMessage(inst, "Sync1IsrCnt=0x%08lx\r\n", pDbgCnt->Sync1IsrCnt);
	CONSOLE_PutMessage(inst, "TmrTskIsrCnt=0x%08lx\r\n", pDbgCnt->TmrTskIsrCnt);	
	CONSOLE_PutMessage(inst, "PdiReentryCnt=0x%08lx\r\n", HW_SpiObj.ReentryCnt);
	
	CONSOLE_PutMessage(inst, "\r\n/* ESC Stack Status */\r\n");	
	CONSOLE_PutMessage(inst, "bDcSyncActive=%d\r\n", bDcSyncActive);	


	return 1;
} /* End of clicmd_EscStack() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_EscStackHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_EscStackHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "           esc: Shows ESC stack status\r\n");	
	return 1;
} /* End of clicmd_EscStackHelp() */


/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowMcStack()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ShowMcStack(CONSOLE_Inst *inst)
{
//	char *str;

	CONSOLE_PutMessage(inst, "\r\n/* MC Stack Status */\r\n");

	CONSOLE_PutMessage(inst, "\r\n");	
	return 1;
} /* End of clicmd_ShowMcStack() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowMcDashboard()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ShowMcDashboard(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	int lineNum=0;
	char *stateStr, *faultStr;
	MC_T *pMC = &MotorCtrl;
	
	CONSOLE_PutMessage(inst, "MC Dashboard\r\n");
	lineNum	= 1;

	/* System status */		
	switch(MC_GetState())
	{
	case MCSTE_IDLE:            stateStr = "MCSTE_IDLE           "; break;/* 21 chars */
	case MCSTE_TUNE:            stateStr = "MCSTE_TUNE           "; break;
	case MCSTE_FAULT:           stateStr = "MCSTE_FAULT          "; break;		
	case MCSTE_WAIT_CLR_ERR:    stateStr = "MCSTE_WAIT_CLR_ERR   "; break;				
	case MCSTE_STOP:            stateStr = "MCSTE_STOP           "; break;		
	case MCSTE_START_VEC_ALIGN: stateStr = "MCSTE_START_VEC_ALIGN"; break;
	case MCSTE_VEC_ALIGN:       stateStr = "MCSTE_VEC_ALIGN      "; break;		
	case MCSTE_START_ENC_ALIGN: stateStr = "MCSTE_START_ENC_ALIGN"; break;				
	case MCSTE_ENC_ALIGN:       stateStr = "MCSTE_ENC_ALIGN      "; break;
	case MCSTE_STARTUP:         stateStr = "MCSTE_STARTUP        "; break;	  
	case MCSTE_RUN:             stateStr = "MCSTE_RUN            "; break;
	case MCSTE_SHUTDOWN:        stateStr = "MCSTE_SHUTDOWN       "; break;
	}
	
	switch(MC_GetFaultStatus())
	{
	case MCFS_NO_ERR:      faultStr = "MCFS_NO_ERR     "; break;/* 16 chars */
	case MCFS_OVER_VOLT:   faultStr = "MCFS_OVER_VOLT  "; break;
	case MCFS_UNDER_VOLT:  faultStr = "MCFS_UNDER_VOLT "; break;
	case MCFS_OVER_TEMP:   faultStr = "MCFS_OVER_TEMP  "; break;
	case MCFS_PHASE_ERR:   faultStr = "MCFS_PHASE_ERR  "; break;
	case MCFS_OVER_CURR:   faultStr = "MCFS_OVER_CURR  "; break;
	case MCFS_SW_ERR:      faultStr = "MCFS_SW_ERR     "; break;
	case MCFS_VEC_ALI_ERR: faultStr = "MCFS_VEC_ALI_ERR"; break;
	case MCFS_ENC_ALI_ERR: faultStr = "MCFS_ENC_ALI_ERR"; break;		
	}	
	CONSOLE_PutMessage(inst, "McState               FaultStatus\r\n");	
	CONSOLE_PutMessage(inst, "%s %s\r\n"
										, stateStr
										, faultStr
										);
	lineNum+=2;

	return lineNum;

} /* End of clicmd_ShowMcDashboard() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_McStackUsage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_McStackUsage(CONSOLE_Inst *inst)
{
	clicmd_McStackHelp(inst);
	
	CONSOLE_PutMessage(inst, "Usage: mc [-dsb] [-clrf] [-st DecEnb]\r\n");
	CONSOLE_PutMessage(inst, "          [-alg DecDuartion] [-rstpos]\r\n");	
	CONSOLE_PutMessage(inst, "          [-pos DecPosition DecSpeed] [-pospid DecValue DecValue] [-spdpid DecValue DecValue] [-amppid DecValue DecValue]\r\n");	
	
	/* Generic parameters */
	CONSOLE_PutMessage(inst, "      -dsb: Enabled dashboard mode.\r\n");
	CONSOLE_PutMessage(inst, "     -clrf: Clear fault status.\r\n");		
	CONSOLE_PutMessage(inst, "       -st: 1/0=Start/Stop motor control.\r\n");
	/* Encoder alignement parameters */	
	CONSOLE_PutMessage(inst, "      -alg: Perform encoder alignment with specified duration time(ms).\r\n");	
	CONSOLE_PutMessage(inst, "   -rstpos: Reset position.\r\n");	
	/* Position control loop parameters */		
	CONSOLE_PutMessage(inst, "      -pos: Move to target position(deg) with specified duration(ms).\r\n");		
	CONSOLE_PutMessage(inst, "    -pospid: Specify KP/KI value for position control loop.\r\n");
	/* Speed control loop parameters */
	CONSOLE_PutMessage(inst, "    -spdpid: Specify KP/KI value for speed control loop.\r\n");	
	/* Current control loop parameters */
	CONSOLE_PutMessage(inst, "    -amppid: Specify KP/KI value for current control loop.\r\n");	
	return 1;
} /* End of clicmd_McStackUsage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_McStack()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_McStack(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint8_t lineNum, dashBoardEnable=0, tmp8;
	uint32_t i, tmp32;
	int32_t stmp32[3];
	TEST_PARAMETER *pParameter = &clicmd_testCtrl.Parameter;
	MC_T *pMC = &MotorCtrl;
	
	if (inst->State == CLI_STATE_COMMAND_WAIT)
	{
		if (inst->GetChar() == CLICMD_CTRL_C)
		{
			clicmd_testCtrl.Terminated = 1;
			
			clicmd_testCtrl.State = TEST_DONE;
		}		
			
		if (inst->Timer==0)
		{
			lineNum = clicmd_ShowMcDashboard(inst, &clicmd_testCtrl);			
			if (clicmd_testCtrl.State == TEST_DONE)
			{
				printdCtrl(1);/* Enable debug messages */		
#if 0
				/* Auto stop motor if leave dashboard */
				MC_StopMotor(pMC);			
#endif				
				inst->State = CLI_STATE_COMMAND;
				CONSOLE_PutMessage(inst, "\r\n");
				if (clicmd_testCtrl.Record.ErrorCnt)
				{
					return -1;
				}
				else
				{
					return 1;		
				}
			}
			else
			{			
				inst->PutChar(0x0D);
				while (lineNum--)
				{
					inst->PutChar(0x1B);
					inst->PutChar(0x5B);		
					inst->PutChar(0x41);					
				}
				inst->Timer = pParameter->LogUpdateTime;
			}
		}
		return 1;
	}
	else if (argc == 1)
	{
		clicmd_McStackUsage(inst);
		return 1;
	}
	else
	{
	
		memset((uint8_t*)&clicmd_testCtrl, 0, sizeof(clicmd_testCtrl));
		
		/* Load default parameters */
		pParameter->LoopCount = TEST_DEF_LOOP_COUNT;
		pParameter->LogUpdateTime = TEST_DEF_LOG_UPDATE_TIME;
		
		for (i=1; i<argc; i++)
		{
			if (!strcmp(argv[i], "-loop") && strlen(argv[i])==5)
			{
				i++;			
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->LoopCount, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-lut") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &tmp32, strlen(argv[i])) != 0)
				{
					return -1;
				}
				pParameter->LogUpdateTime = tmp32;
			}
			else if (!strcmp(argv[i], "-dsb") && strlen(argv[i])==4)
			{
				dashBoardEnable = 1;
			}		
			else if (!strcmp(argv[i], "-clrf") && strlen(argv[i])==5)
			{
				MC_FaultReset();			
			}			
			else if (!strcmp(argv[i], "-st") && strlen(argv[i])==3)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &tmp32, strlen(argv[i])) != 0)
				{
					return -1;
				}
				if (tmp32)
				{
					/* Start Motor */
					MC_StartMotor();
				}
				else
				{
					/* Stop Motor */
					MC_StopMotor();					
				}
			}		
			/* Encoder alignment */
			else if (!strcmp(argv[i], "-alg") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &tmp32, strlen(argv[i])) != 0)
				{
					return -1;
				}
				
				/* Start encoder alignment */				
				MC_StartHoming(tmp32);				
			}	
			else if (!strcmp(argv[i], "-rstpos") && strlen(argv[i])==7)
			{
				/* Reset position */
				MC_ClearActualPosition();				
			}			
			/* Position control loop */			
			else if (!strcmp(argv[i], "-pos") && strlen(argv[i])==4)
			{
				/* Target position */
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[0]), strlen(argv[i])) != 0)
				{
					return -1;
				}

				/* Target speed */				
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[1]), strlen(argv[i])) != 0)
				{
					return -1;
				}				
				MC_SetTargetPosition(stmp32[0], stmp32[1]);	
			}
			else if (!strcmp(argv[i], "-pospid") && strlen(argv[i])==7)
			{
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[0]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[1]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				posKP = stmp32[0];
				posKI = stmp32[1];
			}
			/* Current control loop */			
			else if (!strcmp(argv[i], "-spdpid") && strlen(argv[i])==7)
			{
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[0]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[1]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				spdKP = stmp32[0];
				spdKI = stmp32[1];
			}			
			/* Current control loop */
			else if (!strcmp(argv[i], "-amppid") && strlen(argv[i])==7)
			{
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[0]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[1]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				ampKP = stmp32[0];
				ampKI = stmp32[1];
			}
			else
			{
				CONSOLE_PutMessage(inst, "Unsupported parameter:%s\r\n", argv[i]);				
				return -1;
			}
		}//for
		
		/* Parameter check */
		if (dashBoardEnable)
		{
			printdCtrl(0);/* Disable debug messages */
#if 0
			/* Auto start motor if dashboard enabled */
			MC_StartMotor(pMC);			
#endif			
			inst->State = CLI_STATE_COMMAND_WAIT;		
		}
	}
	return 1;
} /* End of clicmd_McStack() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_McStackHelp()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_McStackHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "            mc: Motor control\r\n");	
	return 1;
} /* End of clicmd_McStackHelp() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_ShowCiA402Dashboard()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_ShowCiA402Dashboard(CONSOLE_Inst *inst, uint8_t AxisId)
{
	int lineNum=0;
#ifdef MC_STACK_ENABLE
	char *stateStr;//, *cmdStr, *ctrlStr, *faultStr[2];
	CiA402_AXIS_T *pAxis = &CiA402Axis;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);	
	HW_DEBUG *pDbgCnt = HW_GetDebugCounter();	
//	MC_T *pMC = &McObj;
	
	CONSOLE_PutMessage(inst, "CiA402 Dashboard\r\n");
	lineNum	= 1;

	/* ESC status */
	CONSOLE_PutMessage(inst, "EscIsrCnt  Sync0IsrCnt Sync1IsrCnt TmrTskIsrCnt PdiReentryCnt\r\n");	
	CONSOLE_PutMessage(inst, "0x%08lx 0x%08lx  0x%08lx  0x%08lx   0x%08lx\r\n"
										, pDbgCnt->EscIsrCnt
										, pDbgCnt->Sync0IsrCnt	
										, pDbgCnt->Sync1IsrCnt
										, pDbgCnt->TmrTskIsrCnt
										, HW_SpiObj.ReentryCnt
										);
	lineNum+=2;	
	
	/* CiA402 status */		
	switch(pAxis->i16State)
	{
	case STATE_NOT_READY_TO_SWITCH_ON:       stateStr = "NOT_READY_TO_SWITCH_ON "; break;/* 23 chars */
	case STATE_SWITCH_ON_DISABLED:           stateStr = "SWITCH_ON_DISABLED     "; break;
	case STATE_READY_TO_SWITCH_ON:           stateStr = "READY_TO_SWITCH_ON     "; break;
	case STATE_SWITCHED_ON:                  stateStr = "SWITCHED_ON            "; break;	  
	case STATE_OPERATION_ENABLED:            stateStr = "OPERATION_ENABLED      "; break;
	case STATE_QUICK_STOP_ACTIVE:            stateStr = "QUICK_STOP_ACTIVE      "; break;
	case STATE_FAULT_REACTION_ACTIVE:        stateStr = "FAULT_REACTION_ACTIVE  "; break;
	case STATE_FAULT:                        stateStr = "FAULT                  "; break;		
	}

	CONSOLE_PutMessage(inst, "CiA402State             PendCode AxisAct BrakeApplied LowPower HighPower FuncEnb CfgAllowed CycleTime\r\n");	
	CONSOLE_PutMessage(inst, "%s 0x%04x   %01u       %01u            %01u        %01u         %01u       %01u          %010u\r\n"
										, stateStr
										, pAxis->u16PendingOptionCode	
										, pAxis->bAxisIsActive
										, pAxis->bBrakeApplied
										, pAxis->bLowLevelPowerApplied
										, pAxis->bHighLevelPowerApplied
										, pAxis->bAxisFunctionEnabled
										, pAxis->bConfigurationAllowed
										, pAxis->u32CycleTime
										);
	lineNum+=2;
		
	/* General objects status */
	CONSOLE_PutMessage(inst, "ControlWord StatusWord ErrorCode ModesOfOp ModesOfOpDisp SuppDrivModes\r\n");	
	CONSOLE_PutMessage(inst, "0x%04x      0x%04x     0x%04x    0x%02x      0x%02x          0x%08lx\r\n"
										, *(pCiA402->pControlword0x6040)
										, *(pCiA402->pStatusword0x6041)			
										, *(pCiA402->pErrorCode0x603F)		
										, *(pCiA402->pModesOfOperation0x6060)
										, *(pCiA402->pModesOfOperationDisplay0x6061)	
										, *(pCiA402->pSupportedDriveModes0x6502)
										);
	lineNum+=2;

	/* Option code objects status */
	CONSOLE_PutMessage(inst, "QuickStopOC ShutDownOC DisableOpOC FaultReactOC\r\n");	
	CONSOLE_PutMessage(inst, "0x%04x      0x%04x     0x%04x      0x%04x\r\n"
										, *(pCiA402->pQuickStopOptionCode0x605A)
										, *(pCiA402->pShutdownOptionCode0x605B)			
										, *(pCiA402->pDisableOperationOptionCode0x605C)		
										, *(pCiA402->pFaultReactionOptionCode0x605E)
										);
	lineNum+=2;
	
	/* Csp mode objects */
	CONSOLE_PutMessage(inst, "TrgPos     PosAct     DigitalInputs\r\n");	
	CONSOLE_PutMessage(inst, "%010d %010d %08lx\r\n"
										, *(pCiA402->pTargetPosition0x607A)
										, *(pCiA402->pPositionActualValue0x6064)
										, *(pCiA402->pDigitalInputs0x60FD)
										);
										
	lineNum+=2;

	/* MC State */
	switch(MC_GetState())
	{
	case MCSTE_IDLE:           stateStr = "MCSTE_IDLE        "; break;//18 chars
	case MCSTE_ENC_ALIGN:      stateStr = "MCSTE_ENC_ALIGN   "; break;
	case MCSTE_STARTUP :       stateStr = "MCSTE_STARTUP     "; break;	  
	case MCSTE_RUN:            stateStr = "MCSTE_RUN         "; break;
	case MCSTE_SHUTDOWN:       stateStr = "MCSTE_SHUTDOWN    "; break;
	case MCSTE_STOP:           stateStr = "MCSTE_STOP        "; break;		
	case MCSTE_FAULT:          stateStr = "MCSTE_FAULT       "; break;
	case MCSTE_WAIT_CLR_ERR:   stateStr = "MCSTE_WAIT_CLR_ERR"; break;
	}
	
	/* Homing objects */
	CONSOLE_PutMessage(inst, "HomeOffset HomeMethod HomeAcc    McState\r\n");	
	CONSOLE_PutMessage(inst, "%010d %010u %010u %s\r\n"
										, *(pCiA402->pHomeOffset0x607C)	
										, *(pCiA402->pHomingMethod0x6098)			
										, *(pCiA402->pHomingAcceleration0x609A)			
										, stateStr										
										);
	lineNum+=2;
#endif		
	return lineNum;

} /* End of clicmd_ShowCiA402Dashboard() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_CiA402Usage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_CiA402Usage(CONSOLE_Inst *inst)
{
	clicmd_CiA402Help(inst);
	
	CONSOLE_PutMessage(inst, "Usage: cia402 [-dsb] [-clrf] [em [DecValue]]\r\n");
	
	/* Generic parameters */
	CONSOLE_PutMessage(inst, "      -dsb: Enabled dashboard mode.\r\n");
	CONSOLE_PutMessage(inst, "     -clrf: Clear fault.\r\n");
	CONSOLE_PutMessage(inst, "       -em: Enable emulation mode.\r\n");
	CONSOLE_PutMessage(inst, "            0: Disable emulation mode.\r\n");	
	CONSOLE_PutMessage(inst, "            1: Set actual position equal to target position.\r\n");		
	CONSOLE_PutMessage(inst, "            2: Set actual position equal to trajectory output position.\r\n");
	return 1;
} /* End of clicmd_CiA402Usage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_CiA402()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_CiA402(CONSOLE_Inst *inst, int argc, char **argv)
{
#ifdef MC_STACK_ENABLE
	uint8_t lineNum, dashBoardEnable=0, tmp8;
	uint32_t i, tmp32;
//	int32_t stmp32;
//	int16_t stmp16;	
	TEST_PARAMETER *pParameter = &clicmd_testCtrl.Parameter;
	CiA402_AXIS_T *pAxis = &CiA402Axis;
//	MC_T *pMC = &McObj;
	
	if (inst->State == CLI_STATE_COMMAND_WAIT)
	{
		if (inst->GetChar() == CLICMD_CTRL_C)
		{
			clicmd_testCtrl.Terminated = 1;
			
			clicmd_testCtrl.State = TEST_DONE;
		}		
			
		if (inst->Timer==0)
		{
			lineNum = clicmd_ShowCiA402Dashboard(inst, 0);			
			if (clicmd_testCtrl.State == TEST_DONE)
			{
				printdCtrl(1);/* Enable debug messages */
				inst->State = CLI_STATE_COMMAND;
				CONSOLE_PutMessage(inst, "\r\n");
				if (clicmd_testCtrl.Record.ErrorCnt)
				{
					return -1;
				}
				else
				{
					return 1;		
				}
			}
			else
			{			
				inst->PutChar(0x0D);
				while (lineNum--)
				{
					inst->PutChar(0x1B);
					inst->PutChar(0x5B);		
					inst->PutChar(0x41);					
				}
				inst->Timer = pParameter->LogUpdateTime;
			}
		}
		return 1;
	}
	else if (argc == 1)
	{
		clicmd_CiA402Usage(inst);
		return 1;
	}
	else
	{
	
		memset((uint8_t*)&clicmd_testCtrl, 0, sizeof(clicmd_testCtrl));
		
		/* Load default parameters */
		pParameter->LoopCount = TEST_DEF_LOOP_COUNT;
		pParameter->LogUpdateTime = TEST_DEF_LOG_UPDATE_TIME;
		
		for (i=1; i<argc; i++)
		{
			if (!strcmp(argv[i], "-loop") && strlen(argv[i])==5)
			{
				i++;			
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->LoopCount, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-lut") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &tmp32, strlen(argv[i])) != 0)
				{
					return -1;
				}
				pParameter->LogUpdateTime = tmp32;
			}
			else if (!strcmp(argv[i], "-dsb") && strlen(argv[i])==4)
			{
				dashBoardEnable = 1;
			}		
			else if (!strcmp(argv[i], "-clrf") && strlen(argv[i])==5)
			{
				MC_FaultReset();				
			}	
			else if (!strcmp(argv[i], "-em") && strlen(argv[i])==3)
			{
				i++;
				if (clicmd_DecText2Char((uint8_t*)argv[i], &tmp8, strlen(argv[i])) != 0)
				{
					return -1;
				}				
				pAxis->EmulationEnable = tmp8;
				CIA402_EmulationControl(tmp8);
			}						
			else
			{
				CONSOLE_PutMessage(inst, "Unsupported parameter:%s\r\n", argv[i]);				
				return -1;
			}
		}//for
		
		/* Parameter check */
		if (dashBoardEnable)
		{
			printdCtrl(0);/* Disable debug messages */
			inst->State = CLI_STATE_COMMAND_WAIT;		
		}
	}
#endif
	return 1;
} /* End of clicmd_CiA402() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: clicmd_CiA402Help()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int clicmd_CiA402Help(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "        cia402: CiA402 status\r\n");	
	return 1;
} /* End of clicmd_CiA402Help() */


/* End of clicmd.c */
