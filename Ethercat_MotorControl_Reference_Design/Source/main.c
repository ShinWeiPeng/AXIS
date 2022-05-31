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
#include "applinterface.h"
#include "AX58200_MotorControl.h"

#include "main.h"
#if (AX58200_DEBUG_ENABLE)
#include "console.h"
#include "test.h"
#endif

/* NAMING CONSTANT DECLARATIONS */
#define PLL_CLOCK					192000000

/* GLOBAL VARIABLES DECLARATIONS */


/* LOCAL SUBPROGRAM DECLARATIONS */


/* LOCAL SUBPROGRAM BODIES */
/*
 * ----------------------------------------------------------------------------
 * Function Name: int main()
 * Purpose: Main program of this firmware
 * Params:	None
 * Returns:	None, it is infinite loop
 * Note:
 * ----------------------------------------------------------------------------
 */
int main()
{
	SYS_UnlockReg();

	/* System initialization */
	SysInit();

#if (AX58200_DEBUG_ENABLE)
	/* Initialize console */
	CONSOLE_Init();
	TEST_Init();
#endif

	/* Boot message */
	printf("ASIX Electronics Coporation\n");
	printf("%s_Firmware v%s, %s \n", DEVICE_NAME, DEVICE_SW_VERSION, __DATE__);

	/* Initialize MC stack */
	MC_Init();
	printf("MC Stack Info: sysVERS=%u, sysDATE=%u, romVERS=%u, romDATE=%u\r\n", sysVERS, sysDATE, romVERS, romDATE);

	/* ESC hardware initialization */
	if (HW_Init())
	{
		printf("Initialize ESC Hardware Timeout!\n");
		HW_Release();
	}
	else
	{
		/* ESC software initialization */
		MainInit();
		CiA402_Init();
		AX58200_FoeInit();
		printf("Start EtherCAT Slave Stack OK!\n");
	}

	SYS_LockReg();

	while (1)
	{
		MH_VCP_Run();

		/* Check if we need to jump to LDROM */
		BL_JumpToLdrom();

#if (AX58200_DEBUG_ENABLE)
		/* User code here */
		CONSOLE_Task();
#endif

	}
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: void SYS_Init(void)
 * Purpose: System initialization
 * Params:	None
 * Returns:	None
 * Note:
 * ----------------------------------------------------------------------------
 */
void SysInit(void)
{
	uint32_t volatile i;

	/* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
	PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

	/* Enable External XTAL (4~24 MHz) */
	CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

	/* Waiting for 12MHz clock ready */
	CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

	/* Switch HCLK clock source to HXT */
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

	/* Set core clock as PLL_CLOCK from PLL */
	CLK_SetCoreClock(FREQ_192MHZ);

	/* Set both PCLK0 and PCLK1 as HCLK/2 */
	CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

	SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;    /* select HSUSBD */
	/* Enable USB PHY */
	SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSUSBEN_Msk;
	for (i = 0; i < 0x1000; i++);      // delay > 10 us
        SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

	/* Enable IP clock */
	CLK_EnableModuleClock(HSUSBD_MODULE);

	/* Enable system tick */
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(MC_INT_PRIORITY_GROUP, 3, 0));
	CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_HXT, (CLK_GetHXTFreq() / 1000)); //Set 1ms tick

	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
	SystemCoreClockUpdate();

	/* Enable IP clock */
	CLK_EnableModuleClock(UART0_MODULE);

	/* Select IP clock source */
	CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

	/* Set GPA multi-function pins for UART0 RXD(GPA.6) and TXD(GPA.7) */
	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
	SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA6MFP_UART0_RXD | SYS_GPA_MFPL_PA7MFP_UART0_TXD);

	/* Init UART to 115200-8n1 for print message */
	UART_Open(UART0, 115200);

} /* End of SysInit() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: void SysTick_Handler(void)
 * Purpose:
 * Params:	None
 * Returns:	None
 * Note:
 * ----------------------------------------------------------------------------
 */
void SysTick_Handler(void)
{
	MC_TickRun();
#if (AX58200_DEBUG_ENABLE)
	CONSOLE_TimeTick();
	TEST_Timer(0);
#endif

} /* End of SysTick_Handler() */
