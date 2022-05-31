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
#include "das.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */
int dasMemory[MC_DAS_BUF_SIZE];

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */


/*
 * ----------------------------------------------------------------------------
 * Function Name: DAS_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void DAS_Init(void)
{
	dasCMD = 0;
	dasCH = 0;
	dasMAX = 0;
	dasN = 0;
	dasMS = 0;
	dasSIZ = MC_DAS_BUF_SIZE;
} /* End of DAS_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: DAS_Run()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
DAS_STATE_E DAS_Run(void)
{
	static DAS_STATE_E ste = DASSTE_IDLE;
	DAS_STATE_E cur_ste;
	int i, *pBuf, *pVarInd = &dasV1;

	cur_ste = ste;

	switch (ste)
	{
	default:
	case DASSTE_IDLE:
		if (dasCMD == DASSTE_START)
		{
			/* Force channels between available range */
			if (dasCH < DAS_MIN_NUM_CHS)
			{
				dasCH = DAS_MIN_NUM_CHS;
			}
			else if (dasCH > DAS_MAX_NUM_CHS)
			{
				dasCH = DAS_MAX_NUM_CHS;
			}

			/* Force sample time between available range */
			if (dasMS < DAS_MIN_SAMPLE_TIME)
			{
				dasMS = DAS_MIN_SAMPLE_TIME;
			}
			else if (dasMS > DAS_MAX_SAMPLE_TIME)
			{
				dasMS = DAS_MAX_SAMPLE_TIME;
			}

			/* Check variable index value to be record are valid */
			for (i=0, pVarInd=&dasV1; i<dasCH; i++, pVarInd++)
			{
				if (*pVarInd<0 || *pVarInd>=MC_PARAM_TABLE_SIZE)
				{
					/* Cancel the process if variable index out of range */
					dasCMD = DASSTE_IDLE;
					break;
				}
			}

			/* Reset DAS buffer */
			dasMAX = dasSIZ/dasCH;
			dasN = 0;
			/* Update status */
			ste = DASSTE_RUN;
			dasCMD = ste;
			/* Assign sample time */
			dasDT = dasMS;
		}
		else
		{
			break;
		}
	case DASSTE_START:
	case DASSTE_RUN:
		/* Check abort */
		if (dasCMD <= DASSTE_IDLE)
		{
			ste = DASSTE_IDLE;
			break;
		}
		/* Wait sample delay */
		if (--dasDT)
		{
			break;
		}
		dasDT = dasMS;

		/* Terminate process if buffer has full */
		if (dasN >= dasMAX)
		{
			ste = DASSTE_IDLE;
			dasCMD = ste;
			break;
		}

		/* Record data */
		pVarInd = &dasV1;
		pBuf = &dasMemory[dasN*dasCH];
		dasN++;
		for (i=0; i<dasCH; i++)
		{
			*pBuf++ = mapMemory[*pVarInd++];
		}
		break;
	}
	return cur_ste;

} /* End of DAS_Run() */

/* End of das.c */
