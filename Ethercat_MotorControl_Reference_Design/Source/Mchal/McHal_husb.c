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
#include <string.h>
#include "McHal_husb.h"
#include "McHal_misc.h"
#include "hsusbd.h"

extern int mapMemory[]; 
extern int esc_doCommand(char *,int,char *,char *,int,int);
extern S_HSUSBD_INFO_T sHSInfo;

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */
typedef struct
{    
	uint32_t  Baudrate;
	uint8_t   StopBit;
	uint8_t   ParityType;
	uint8_t   DataBits;
} MHVCP_LINE_CODING;

/* MACRO DECLARATIONS */
#define MH_VCP_PUT_ERR()  {MH_VCP_PutStr("*** error !!");}

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */
MHVCP_LINE_CODING lineCoding={115200,0,0,8};
MH_VCP_OBJECT_T Husb;

/* LOCAL SUBPROGRAM DECLARATIONS */
static void MH_VCP_SetupEndpoints(uint32_t as_high_speed);
static void MH_VCP_ClassRequest(void);
static void MH_VCP_Xmit(void);
static void MH_VCP_PutChar(char c);
static void MH_VCP_PutStr(char *s);
static int  MH_VCP_StrToInt(char *s, int n);
static void MH_VCP_ParseCommandLine(char *buf, int n);
static void MH_VCP_Recv(void);

/* LOCAL SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: USBD20_IRQHandler()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void USBD20_IRQHandler(void)
{
	__IO uint32_t irq, sts;
	int i, len;
	
	irq = HSUSBD->GINTSTS & HSUSBD->GINTEN;
	if (!irq)
	{
		return;
	}

	if (irq & HSUSBD_GINTSTS_USBIF_Msk)
	{    
		sts = HSUSBD->BUSINTSTS & HSUSBD->BUSINTEN;
		/* SOF receive control */
		if (sts & HSUSBD_BUSINTSTS_SOFIF_Msk)
		{         
			HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SOFIF_Msk);
		}
		/* Reset status */
		if (sts & HSUSBD_BUSINTSTS_RSTIF_Msk)
		{    
			HSUSBD_SwReset();
			HSUSBD_ResetDMA();
			HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EPRSPCTL_FLUSH_Msk;
			HSUSBD->EP[EPB].EPRSPCTL = HSUSBD_EPRSPCTL_FLUSH_Msk;
      MH_VCP_SetupEndpoints(((HSUSBD->OPER&0x04) != 0) ? 1:0);
			HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
			HSUSBD_SET_ADDR(0);
			HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk|HSUSBD_BUSINTEN_RESUMEIEN_Msk|HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
			HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RSTIF_Msk);
			HSUSBD_CLR_CEP_INT_FLAG(0x1ffc);
		}
		/* Resume */
		if (sts & HSUSBD_BUSINTSTS_RESUMEIF_Msk)
		{    
			HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk|HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
			HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RESUMEIF_Msk);
		}
		/* Suspend request */
		if (sts & HSUSBD_BUSINTSTS_SUSPENDIF_Msk)
		{    
			HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk|HSUSBD_BUSINTEN_RESUMEIEN_Msk);
			HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SUSPENDIF_Msk);
		}
		/* High-speed settle */
		if (sts & HSUSBD_BUSINTSTS_HISPDIF_Msk)
		{    
			HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
			HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_HISPDIF_Msk);
		}
		/* DMA complete */
		if (sts & HSUSBD_BUSINTSTS_DMADONEIF_Msk)
		{    
			g_hsusbd_DmaDone = 1;
			HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_DMADONEIF_Msk);
			if (!(HSUSBD->DMACTL & HSUSBD_DMACTL_DMARD_Msk))
			{
				HSUSBD_ENABLE_EP_INT(EPB, HSUSBD_EPINTEN_RXPKIEN_Msk);
			}
			if (HSUSBD->DMACTL & HSUSBD_DMACTL_DMARD_Msk)
			{    
				/* Packet end */
				if (g_hsusbd_ShortPacket == 1)
				{    
					g_hsusbd_ShortPacket = 0;
					HSUSBD->EP[EPA].EPRSPCTL = (HSUSBD->EP[EPA].EPRSPCTL & 0x10)|HSUSBD_EP_RSPCTL_SHORTTXEN;
				}
			}
		}
		/* Usable clock interrupt */
		if (sts & HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk)
		{
			HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk);
		}
		/* VBUS detection interrupt */
		if (sts & HSUSBD_BUSINTSTS_VBUSDETIF_Msk)
		{    
			if (HSUSBD_IS_ATTACHED())
			{
				HSUSBD_ENABLE_USB();
			}
			else                      
			{
				HSUSBD_DISABLE_USB();
			}
			HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_VBUSDETIF_Msk);
		}
	}
	if  (irq & HSUSBD_GINTSTS_CEPIF_Msk)
	{    
		sts = HSUSBD->CEPINTSTS & HSUSBD->CEPINTEN;
		/* Setup token interrupt */
		if (sts & HSUSBD_CEPINTSTS_SETUPTKIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPTKIF_Msk);
			return;
		}
		/* Setup packet interrupt */
		if (sts & HSUSBD_CEPINTSTS_SETUPPKIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPPKIF_Msk);
			HSUSBD_ProcessSetupPacket();
			return;
		}
    /* Out token interrupt */
		if (sts & HSUSBD_CEPINTSTS_OUTTKIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_OUTTKIF_Msk);
			HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
			return;
		}
		/* In token interrupt */
		if (sts & HSUSBD_CEPINTSTS_INTKIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
			if (!(sts & HSUSBD_CEPINTSTS_STSDONEIF_Msk))
			{    
				HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
				HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_TXPKIEN_Msk);
				HSUSBD_CtrlIn();
			} 
			else
			{    
				HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
				HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_TXPKIEN_Msk|HSUSBD_CEPINTEN_STSDONEIEN_Msk);
			}
			return;
		}
		/* Ping token interrupt */
		if (sts & HSUSBD_CEPINTSTS_PINGIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_PINGIF_Msk);
			return;
		}
		/* Data packet Tx interrupt */
		if (sts & HSUSBD_CEPINTSTS_TXPKIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
			HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
			if (g_hsusbd_CtrlInSize)
			{    
				HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
				HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_INTKIEN_Msk);
			} 
			else
			{    
				if (g_hsusbd_CtrlZero == 1)
				{
					HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_ZEROLEN);
				}
				HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
				HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk|HSUSBD_CEPINTEN_STSDONEIEN_Msk);
			}
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
			return;
		}
		/* Data packet Rx interrupt	*/
		if (sts & HSUSBD_CEPINTSTS_RXPKIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_RXPKIF_Msk);
			HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
			HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk|HSUSBD_CEPINTEN_STSDONEIEN_Msk);
			return;
		}
		/* NAK sent interrupt */
		if (sts & HSUSBD_CEPINTSTS_NAKIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_NAKIF_Msk);
			return;
		}
		/* STALL sent interrupt */
		if (sts & HSUSBD_CEPINTSTS_STALLIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STALLIF_Msk);
			return;
		}
		/* USB error interrupt */
		if (sts & HSUSBD_CEPINTSTS_ERRIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_ERRIF_Msk);
			return;
		}
		/* Status completion interrupt */
		if (sts & HSUSBD_CEPINTSTS_STSDONEIF_Msk)
		{    
			HSUSBD_UpdateDeviceState();
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
			HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
			return;
		}
		/* Buffer full interrupt */
		if (sts & HSUSBD_CEPINTSTS_BUFFULLIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFFULLIF_Msk);
			return;
		}
		/* Buffer empty interrupt */
		if (sts & HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk)
		{    
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk);
			return;
		}
	}
	/* Endpoint A interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPAIF_Msk)
	{    
		sts = HSUSBD->EP[EPA].EPINTSTS & HSUSBD->EP[EPA].EPINTEN;
		HSUSBD_ENABLE_EP_INT(EPA, 0);       //**bulk input
		HSUSBD_CLR_EP_INT_FLAG(EPA,sts);
		Husb.flgTxReady = 1;
	}
	/* Endpoint B interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPBIF_Msk)
	{    
		sts = HSUSBD->EP[EPB].EPINTSTS & HSUSBD->EP[EPB].EPINTEN;
		len = HSUSBD->EP[EPB].EPDATCNT & 0xfff;//**bulk output
		for (i=0; i<len; i++)
		{
			if (Husb.RxLength<USB_RXBUF_SIZE)
			{
				Husb.RxBuffer[Husb.RxLength++] = HSUSBD->EP[EPB].EPDAT_BYTE;
			}
		}
		HSUSBD_CLR_EP_INT_FLAG(EPB,sts); 
		Husb.flgRxReady=1;
	}
	/* Endpoint C interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPCIF_Msk)
	{    
		sts = HSUSBD->EP[EPC].EPINTSTS & HSUSBD->EP[EPC].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPC,sts);
	}
	/* Endpoint D interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPDIF_Msk)
	{    
		sts = HSUSBD->EP[EPD].EPINTSTS & HSUSBD->EP[EPD].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPD,sts);
	}
	/* Endpoint E interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPEIF_Msk)
	{    
		sts = HSUSBD->EP[EPE].EPINTSTS & HSUSBD->EP[EPE].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPE,sts);
	}
	/* Endpoint F interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPFIF_Msk)
	{    
		sts = HSUSBD->EP[EPF].EPINTSTS & HSUSBD->EP[EPF].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPF,sts);
	}
	/* Endpoint G interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPGIF_Msk)
	{    
		sts = HSUSBD->EP[EPG].EPINTSTS & HSUSBD->EP[EPG].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPG,sts);
	}
	/* Endpoint H interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPHIF_Msk)
	{    
		sts = HSUSBD->EP[EPH].EPINTSTS & HSUSBD->EP[EPH].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPH,sts);
	}
	/* Endpoint I interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPIIF_Msk)
	{    
		sts = HSUSBD->EP[EPI].EPINTSTS & HSUSBD->EP[EPI].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPI,sts);
	}
	/* Endpoint J interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPJIF_Msk)
	{    
		sts = HSUSBD->EP[EPJ].EPINTSTS & HSUSBD->EP[EPJ].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPJ,sts);
	}
	/* Endpoint K interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPKIF_Msk)
	{    
		sts = HSUSBD->EP[EPK].EPINTSTS & HSUSBD->EP[EPK].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPK,sts);
	}
	/* Endpoint L interrupt	*/
	if (irq & HSUSBD_GINTSTS_EPLIF_Msk)
	{    
		sts = HSUSBD->EP[EPL].EPINTSTS & HSUSBD->EP[EPL].EPINTEN;
		HSUSBD_CLR_EP_INT_FLAG(EPL,sts);
	}

} /* End of USBD20_IRQHandler() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_SetupEndpoints()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void MH_VCP_SetupEndpoints(uint32_t as_high_speed)
{
	/* EPA=Bulk IN endpoint, address=1 */
	HSUSBD_SetEpBufAddr(EPA, EPA_BUF_BASE, EPA_BUF_LEN);
	if (as_high_speed)
	{
		HSUSBD_SET_MAX_PAYLOAD(EPA, EPA_MAX);
	}
	else
	{
		HSUSBD_SET_MAX_PAYLOAD(EPA, EPA_OMX);
	}
	HSUSBD_ConfigEp(EPA, BULK_IP, HSUSBD_EP_CFG_TYPE_BULK, HSUSBD_EP_CFG_DIR_IN);
	
	/* EPB=Bulk OUT endpoint, address=2 */
	HSUSBD_SetEpBufAddr(EPB, EPB_BUF_BASE, EPB_BUF_LEN);
	if (as_high_speed)
	{
		HSUSBD_SET_MAX_PAYLOAD(EPB, EPB_MAX);
	}
	else
	{
		HSUSBD_SET_MAX_PAYLOAD(EPB, EPB_OMX);
	}
	HSUSBD_ConfigEp(EPB, BULK_OP, HSUSBD_EP_CFG_TYPE_BULK, HSUSBD_EP_CFG_DIR_OUT);
	HSUSBD_ENABLE_EP_INT(EPB, HSUSBD_EPINTEN_RXPKIEN_Msk | HSUSBD_EPINTEN_SHORTRXIEN_Msk);
	
	/* EPC=Interrupt IN endpoint, address=3 */
	HSUSBD_SetEpBufAddr(EPC, EPC_BUF_BASE, EPC_BUF_LEN);
	if (as_high_speed)
	{
		HSUSBD_SET_MAX_PAYLOAD(EPC, EPC_MAX);
	}
	else
	{
		HSUSBD_SET_MAX_PAYLOAD(EPC, EPC_OMX);
	}
	HSUSBD_ConfigEp(EPC, INTR_IP, HSUSBD_EP_CFG_TYPE_INT, HSUSBD_EP_CFG_DIR_IN);

} /* End of MH_VCP_SetupEndpoints() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_ClassRequest()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void MH_VCP_ClassRequest(void)
{
	if (gUsbCmd.bmRequestType & 0x80)
	{
		/* Device to Host */
		switch (gUsbCmd.bRequest)
		{
		case GET_LINE_CODE:
			if ((gUsbCmd.wIndex & 0xff)==0)
			{
				HSUSBD_PrepareCtrlIn((uint8_t *)&lineCoding,7);
			}
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
			HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_INTKIEN_Msk);
			break;
		
		default:
			/* Setup error, stall the device */
			HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
			break;
		}
	} 
	else
	{
    /* Host to Device */
		switch (gUsbCmd.bRequest)
		{
		case SET_CONTROL_LINE_STATE:
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
			HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
			HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
			break;
			
		case SET_LINE_CODE:
			if ((gUsbCmd.wIndex & 0xff)==0)
			{
				HSUSBD_CtrlOut((uint8_t *)&lineCoding,7);
			}
			HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
			HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
			HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
			break;
			
		default:
			/* Setup error, stall the device */
			HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
			break;
		}
	}
} /* End of MH_VCP_ClassRequest() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_Xmit()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void MH_VCP_Xmit(void)
{    
	uint32_t i;
	
	if (Husb.TxPnt >= Husb.TxLength)
	{
		return;
	}
	if (!HSUSBD_IS_ATTACHED()) 
	{
		return;
	}
	for (i=0; i<EPA_MAX && Husb.TxPnt<Husb.TxLength; i++)
	{
		HSUSBD->EP[EPA].EPDAT_BYTE = Husb.TxBuffer[Husb.TxPnt++];
	}
	HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;
	HSUSBD->EP[EPA].EPTXCNT = i;
  Husb.flgTxReady = 0;
	HSUSBD_ENABLE_EP_INT(EPA,HSUSBD_EPINTEN_INTKIEN_Msk);
} /* End of MH_VCP_Xmit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_PutChar()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void MH_VCP_PutChar(char c)
{
	if (Husb.TxLength < USB_TXBUF_SIZE)
	{
		Husb.TxBuffer[Husb.TxLength++] = c;
	}		
} /* End of MH_VCP_PutChar() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_PutStr()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void MH_VCP_PutStr(char *s)
{
	uint32_t i, len;
	
	len = strlen(s);
	for (i=0; i<len; i++)
	{
		if (Husb.TxLength < USB_TXBUF_SIZE)
		{
			Husb.TxBuffer[Husb.TxLength++] = *s++;
		}		
	}
} /* End of MH_VCP_PutStr() */
		
/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_StrToInt()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int MH_VCP_StrToInt(char *s,int n)
{   		
	int i, d, neg;
	char c;
	
	if (n <= 0)
	{
		/* Return zero for invalid string length */
		return 0;
	}
	
	/* Hex string */
	if (n>2 && s[0]=='0' && (s[1]=='X' || s[1]=='x'))
	{
		for (s+=2, i=2, d=0; i<n; i++)
		{    
			c = *s++;
			if (c >= 'a')
			{
				c = c-'a'+10;
			}
			else if (c >= 'A') 
			{
				c = c-'A'+10;
			}
			else
			{
				c = c-'0';
			}
			if (c<0 || c>15) 
			{
				return(0);
			}
			d = d*16 + c;
		}
	}
	/* Decimal string */	
	else
	{    
		if (*s == '-')//Negative
		{
			neg = 1; 
			i = 1; 
			s++;
		}
		else//Positive
		{
			neg = 0; 
			i = 0;
		}
		for (d=0; i<n; i++)
		{
			c = *s++;
			c -= '0';
			/* Return zero while invalid char appear */
			if (c<0 || c>9)
			{
				return 0;
			}
			d = d*10+c;
		}  
		/* Add sign */
		if (neg)
		{
			d = -d;
		}
	}    
	return d;
} /* End of MH_VCP_StrToInt() */

static char sCmd[8],sNum[8],*sLine,*sEcho;
static int  nCmd=0, nNum=0,  nLine, nEcho;

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_ParseCommandLine()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void MH_VCP_ParseCommandLine(char *buf,int n)
{
	int i, sts;
	char c;
	
	nCmd = 0;
	nLine = 0;
	nNum = 0;
	sLine = buf;
	
	for (i=0,sts=1; i<n && sts>0; i++)
	{
		c = buf[i];
		
		if (c=='\n' || c=='\r' || c==';')
		{
			break;
		}
		if (c==' '  || c=='\t')
		{
			continue;
		}
		switch (sts)
		{
		case 1:                      //**sCmd[]
			if (c=='(')
			{
				sts++;
			}
			else if (nCmd<7)
			{
				sCmd[nCmd++] = c;
			}
			break;
			
		case 2:                      //**wait ".."
			if (c=='\'' || c=='\"')
			{
				sts++;
				sLine = &buf[i+1];
				nLine = 0;
			}
			break;
			
		case 3:                      //**sLine[]
			if (c=='\'' || c=='\"')
			{
				sts++;
			}
			else
			{
				sLine[nLine++]=c;
			}
			break;
			
		case 4:                      //**wait ","
			if (c==')')
			{
				sts = 0;
			}
			else if (c==',')
			{
				sts++;
			}
			break;
			
		case 5:                      //**wait ")"
			if (c==')')
			{
				sts = 0;
			}
			else if (nNum<7)
			{
				sNum[nNum++] = c;
			}
			break;
		}/* End of switch */
	}/* End of for loop */
	
	sCmd[nCmd] = 0;
	sNum[nNum] = 0;
	sLine[nLine] = 0;
} /* End of MH_VCP_StrToInt() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_Recv()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void MH_VCP_Recv(void)
{
	int i, n, d;
	char c;
	static int sts = VCPSTE_NOT_INIT, old = VCPSTE_NOT_INIT, t0=0, t1=0, p=0;
	
	if (sts < VCPSTE_RECV_CMD)
	{
		/* Start up initailization */
		sts = VCPSTE_RECV_CMD; 
		Husb.flgRxReady = 0;
		Husb.RxPnt = 0;
		Husb.RxLength = 0; 
		Husb.flgTxReady = 0;
		Husb.TxPnt = 0;
		Husb.TxLength = 0;
	}
													//**timeout=100ms
	if (old!=sts)
	{
		t0 = MH_ClkTick_1ms(); 
		old = sts;
	}
	else if (sts>0 && MH_Delay_1ms(t0)>100) 
	{
		old = VCPSTE_RECV_CMD;
		sts = VCPSTE_RECV_CMD; 
		Husb.flgRxReady = 0;
		Husb.RxLength = 0;
	}
	switch(sts)
	{
	case VCPSTE_RECV_CMD:
		/* Waiting for complete data line to be received, include '\r' char */
		if (Husb.flgRxReady==0 || Husb.RxLength==0)
		{
			break;
		}
		Husb.flgRxReady = 0;
		p = 0; 
		if (Husb.RxBuffer[Husb.RxLength-1] != '\r')
		{
			break;
		}
		/* Echo received data line */		
		Husb.flgTxReady = 0;
		Husb.TxPnt = 0;
		Husb.TxLength = 0;
		Husb.RxPnt = 0;
		for (i=0; i<Husb.RxLength && i<50; i++) 
		{
			MH_VCP_PutChar(Husb.RxBuffer[i]);
		}
		if (Husb.TxBuffer[i-1] == '\r') 
		{
			MH_VCP_PutChar('\n');
		}
		else     
		{
			MH_VCP_PutStr("\r\n");
		}
		sts = VCPSTE_PARSE_CMD; 
		break;
		
	case VCPSTE_PARSE_CMD:
		/* If RX buffer has overrun, force process terminated */
		if (Husb.RxPnt >= Husb.RxLength) 
		{
			sts = VCPSTE_PREPARE_END_RESP; 
			break;
		}
		/* Find the data line */		
		for (n=Husb.RxPnt; n<Husb.RxLength; n++)
		{
			c = Husb.RxBuffer[n]; 
			if (c=='\n' || c=='\r' || c==';') 
			{
				n++; 
				break;
			}
		}
		MH_VCP_ParseCommandLine((char*)&Husb.RxBuffer[Husb.RxPnt],(Husb.RxLength-Husb.RxPnt));
		Husb.RxPnt = n;
		sEcho = (char *)&Husb.TxBuffer[Husb.TxLength]; 
		nEcho = USB_TXBUF_SIZE-Husb.TxLength;
		if (nNum>0)
		{
			d = MH_VCP_StrToInt(sNum, nNum);
		}
		else
		{      
			d = 0;
		}
		if (nCmd <= 0)
		{
			break;
		}
		if (nLine <= 0)       
		{
			MH_VCP_PUT_ERR();
			break;
		}
		n = esc_doCommand(sEcho, nEcho, sCmd, sLine, nLine, d);
		if (n >= 0)
		{
			Husb.TxLength += n;
		}
		else       
		{
			sts = VCPSTE_EXEC_CMD;
		}
		break;
		
	case VCPSTE_EXEC_CMD:
		n = esc_doCommand(sEcho, nEcho, sCmd, sLine, nLine, -1);
		if (n >= 0)
		{
			Husb.TxLength += n; 
			sts = VCPSTE_PARSE_CMD;
		}
		break;
		
	case VCPSTE_PREPARE_END_RESP:
		if (p == 0)
		{    
			if (Husb.TxBuffer[Husb.TxLength-1] != '\n') 
			{
				MH_VCP_PutStr("\r\n");
			}
			MH_VCP_PutStr(">>>");
		}
		Husb.flgTxReady = 1;
		Husb.TxPnt = 0;
		sts = VCPSTE_WAIT_END_RESP;
		break;
		
	case VCPSTE_WAIT_END_RESP:
		/* Waiting for response data transmitted completely */
		if (Husb.TxPnt && MH_Delay_1ms(t1) < 3)
		{
			break;
		}
		if (!Husb.flgTxReady)
		{
			break;
		}
		MH_VCP_Xmit(); 
		t1 = MH_ClkTick_1ms();
		if (Husb.TxPnt >= Husb.TxLength)   
		{
			sts = VCPSTE_RECV_CMD; 
			Husb.RxLength=0; 
			Husb.TxLength=0;
		}
		break;
	}
} /* End of MH_VCP_Recv() */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_VCP_Init(void)
{
	int volatile i;
	
	memset(&Husb, 0, sizeof(Husb));
	
	SYS->USBPHY&=~SYS_USBPHY_HSUSBROLE_Msk;
	SYS->USBPHY =(SYS->USBPHY
               & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk))
               |   SYS_USBPHY_HSUSBEN_Msk;
	for (i=0; i<0x1000; i++);
	SYS->USBPHY|= SYS_USBPHY_HSUSBACT_Msk;
	CLK_EnableModuleClock(HSUSBD_MODULE);
	HSUSBD_Open(&sHSInfo, MH_VCP_ClassRequest, NULL);

	HSUSBD_ENABLE_USB_INT(HSUSBD_GINTEN_USBIEN_Msk |
                           HSUSBD_GINTEN_CEPIEN_Msk |
                           HSUSBD_GINTEN_EPAIEN_Msk |      //**endpoint A
                           HSUSBD_GINTEN_EPBIEN_Msk |      //**endpoint B
                           HSUSBD_GINTEN_EPCIEN_Msk);      //**endpoint C
                                       //**enable BUS interrupt
	HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_DMADONEIEN_Msk|
                           HSUSBD_BUSINTEN_RESUMEIEN_Msk |
                           HSUSBD_BUSINTEN_RSTIEN_Msk    |
                           HSUSBD_BUSINTEN_VBUSDETIEN_Msk);
	HSUSBD_SET_ADDR(0);               //**reset address to 0
                                       //**control endpoint
	HSUSBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_BUF_LEN);
	HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk|
                           HSUSBD_CEPINTEN_STSDONEIEN_Msk);
	MH_VCP_SetupEndpoints(1);	

	NVIC_SetPriority(USBD20_IRQn,15);
	NVIC_EnableIRQ(USBD20_IRQn);
} /* End of MH_VCP_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_VCP_Run()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_VCP_Run(void)
{    
	static uint32_t started = 0;
	
	if (started == 0)
	{
		if (HSUSBD_IS_ATTACHED())
		{
			HSUSBD_Start();
			started = 1;
			printf("USB2.0 link OK!\n");
		}
	}
	else
	{  
		MH_VCP_Recv();
	}		
} /* End of MH_VCP_Run() */

/* End of McHal_husb.c */
