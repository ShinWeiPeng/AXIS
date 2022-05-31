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
#include "McHal_misc.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_BitToNum()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint32_t MH_BitToNum(uint32_t Bit)
{
	if (Bit & 0x000000ff)
	{
		switch (Bit)
		{
		case BIT_0: return 0;
		case BIT_1: return 1;
		case BIT_2: return 2;
		case BIT_3: return 3;
		case BIT_4: return 4;
		case BIT_5: return 5;
		case BIT_6: return 6;
		case BIT_7: return 7;
		}
	}
	else if (Bit & 0x0000ff00)
	{
		switch(Bit)
		{
		case BIT_8: return 8;
		case BIT_9: return 9;
		case BIT_10: return 10;
		case BIT_11: return 11;
		case BIT_12: return 12;
		case BIT_13: return 13;
		case BIT_14: return 14;
		case BIT_15: return 15;
		}
	}
	else if (Bit & 0x00ff0000)
	{
		switch(Bit)
		{
		case BIT_16: return 16;
		case BIT_17: return 17;
		case BIT_18: return 18;
		case BIT_19: return 19;
		case BIT_20: return 20;
		case BIT_21: return 21;
		case BIT_22: return 22;
		case BIT_23: return 23;
		}
	}
	else
	{
		switch(Bit)
		{
		case BIT_24: return 24;
		case BIT_25: return 25;
		case BIT_26: return 26;
		case BIT_27: return 27;
		case BIT_28: return 28;
		case BIT_29: return 29;
		case BIT_30: return 30;
		case BIT_31: return 31;
		}
	}
	return 0;
} /* End of MH_BitToNum() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_SetMFP()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int32_t MH_SetMFP(GPIO_T* Instance, uint32_t Pins, uint8_t MultiFuncValue)
{
	uint32_t i, bitMask = 0x1ul, funcMask = 0xful, mfunc = (MultiFuncValue & 0xful);
	uint32_t *pGPx_MFPL = 0, *pGPx_MFPH = 0;

	if (Instance == PA)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPA_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPA_MFPH;
	}
	else if (Instance == PB)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPB_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPB_MFPH;
	}
	else if (Instance == PC)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPC_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPC_MFPH;
	}
	else if (Instance == PD)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPD_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPD_MFPH;
	}
	else if (Instance == PE)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPE_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPE_MFPH;
	}
	else if (Instance == PF)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPF_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPF_MFPH;
	}
	else if (Instance == PG)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPG_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPG_MFPH;
	}
	else if (Instance == PH)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPH_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPH_MFPH;
	}
	else
	{
		return -1;
	}

	for (i = 0; i < GPIO_PIN_MAX; i++)
	{
		if (Pins & bitMask)
		{
			if (i < 8)
			{
				/* Clear multi-function setting in MFPL reg. */
				(*pGPx_MFPL) &= ~funcMask;

				/* Config multi-function in MFPL reg. */
				(*pGPx_MFPL) |= mfunc;
			}
			else
			{
				/* Clear multi-function setting in MFPH reg. */
				(*pGPx_MFPH) &= ~funcMask;

				/* Config multi-function in MFPH reg. */
				(*pGPx_MFPH) |= mfunc;
			}
		}
		bitMask <<= 1;
		if (i == 7)
		{
			funcMask = 0xful;
			mfunc = MultiFuncValue;
		}
		else
		{
			funcMask <<= 4;
			mfunc <<= 4;
		}
	}
	return 0;
} /* End of MH_SetMFP() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_TmrClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_TmrClkSource(TIMER_T *pInst, MC_EADA_E Ctrl)
{
	if (pInst == TIMER0)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(TMR0_MODULE);
			CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
		}
		else
		{
			CLK_DisableModuleClock(TMR0_MODULE);
		}
	}
	else if (pInst == TIMER1)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(TMR1_MODULE);
			CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, 0);
		}
		else
		{
			CLK_DisableModuleClock(TMR1_MODULE);
		}
	}
	else if (pInst == TIMER2)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(TMR2_MODULE);
			CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HXT, 0);
		}
		else
		{
			CLK_DisableModuleClock(TMR2_MODULE);
		}
	}
	else if (pInst == TIMER3)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(TMR3_MODULE);
			CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HXT, 0);
		}
		else
		{
			CLK_DisableModuleClock(TMR3_MODULE);
		}
	}
} /* End of MH_TmrClkSource() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_SpiClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_SpiClkSource(SPI_T *pInst, MC_EADA_E Ctrl)
{
	if (pInst == SPI0)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(SPI0_MODULE);
			CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
		}
		else
		{
			CLK_DisableModuleClock(SPI0_MODULE);
		}
	}
	else if (pInst == SPI1)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(SPI1_MODULE);
			CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);
		}
		else
		{
			CLK_DisableModuleClock(SPI1_MODULE);
		}
	}
	else if (pInst == SPI2)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(SPI2_MODULE);
			CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PCLK1, MODULE_NoMsk);
		}
		else
		{
			CLK_DisableModuleClock(SPI2_MODULE);
		}
	}
	else if (pInst == SPI3)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(SPI3_MODULE);
			CLK_SetModuleClock(SPI3_MODULE, CLK_CLKSEL2_SPI3SEL_PCLK0, MODULE_NoMsk);
		}
		else
		{
			CLK_DisableModuleClock(SPI3_MODULE);
		}
	}
} /* End of MH_SpiClkSource() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_UartClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_UartClkSource(UART_T *pInst, MC_EADA_E Ctrl)
{
	if (pInst == UART0)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(UART0_MODULE);
			CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PLL, CLK_CLKDIV0_UART0(1));
		}
		else
		{
			CLK_DisableModuleClock(UART0_MODULE);
		}
	}
	else if (pInst == UART1)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(UART1_MODULE);
			CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_PLL, CLK_CLKDIV0_UART1(1));
		}
		else
		{
			CLK_DisableModuleClock(UART1_MODULE);
		}
	}
	else if (pInst == UART2)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(UART2_MODULE);
			CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_PLL, CLK_CLKDIV4_UART2(1));
		}
		else
		{
			CLK_DisableModuleClock(UART2_MODULE);
		}
	}
	else if (pInst == UART3)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(UART3_MODULE);
			CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL3_UART3SEL_PLL, CLK_CLKDIV4_UART3(1));
		}
		else
		{
			CLK_DisableModuleClock(UART3_MODULE);
		}
	}
	else if (pInst == UART4)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(UART4_MODULE);
			CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_PLL, CLK_CLKDIV4_UART3(1));
		}
		else
		{
			CLK_DisableModuleClock(UART4_MODULE);
		}
	}
	else if (pInst == UART5)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(UART5_MODULE);
			CLK_SetModuleClock(UART5_MODULE, CLK_CLKSEL3_UART5SEL_PLL, CLK_CLKDIV4_UART5(1));
		}
		else
		{
			CLK_DisableModuleClock(UART5_MODULE);
		}
	}
} /* End of MH_UartClkSource() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_EcapClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_EcapClkSource(ECAP_T *pInst, MC_EADA_E Ctrl)
{
	if (pInst == ECAP0)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(ECAP0_MODULE);
		}
		else
		{
			CLK_DisableModuleClock(ECAP0_MODULE);
		}
	}
	else if (pInst == ECAP1)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(ECAP1_MODULE);
		}
		else
		{
			CLK_DisableModuleClock(ECAP1_MODULE);
		}
	}
} /* End of MH_EcapClkSource() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_EpwmClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_EpwmClkSource(EPWM_T *pInst, MC_EADA_E Ctrl)
{
	if (pInst == EPWM0)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(EPWM0_MODULE);
			CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PLL, NULL);
		}
		else
		{
			CLK_DisableModuleClock(EPWM0_MODULE);
		}
	}
	else if (pInst == EPWM1)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(EPWM1_MODULE);
			CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PLL, NULL);
		}
		else
		{
			CLK_DisableModuleClock(EPWM1_MODULE);
		}
	}
} /* End of MH_EpwmClkSource() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_EadcClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_EadcClkSource(EADC_T *pInst, MC_EADA_E Ctrl)
{
	if (pInst == EADC0)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(EADC_MODULE);
			CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(2));
		}
		else
		{
			CLK_DisableModuleClock(EADC_MODULE);
		}
	}
} /* End of MH_EadcClkSource() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: MH_QeiClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void MH_QeiClkSource(QEI_T *pInst, MC_EADA_E Ctrl)
{
	if (pInst == QEI0)
	{
		if (Ctrl == MC_ENABLE)
		{
			CLK_EnableModuleClock(QEI0_MODULE);
		}
		else
		{
			CLK_DisableModuleClock(QEI0_MODULE);
		}
	}
} /* End of MH_QeiClkSource() */

int32_t MH_ClkTick_1ms(void)
{
	return(sysCLK);
}

int32_t MH_Delay_1ms(int time)
{
	return((MH_ClkTick_1ms() - time) & 0xfffffff);
}

/* End of McHal_misc.c */

