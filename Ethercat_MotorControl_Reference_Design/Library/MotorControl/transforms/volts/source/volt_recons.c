//#############################################################################
// $TI Release: MotorControl SDK v4.00.00.00 $
// $Release Date: Thu Feb 17 18:05:19 CST 2022 $
// $Copyright:
// Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################


//! \file   \libraries\transforms\volts\source\volt_recons.c
//! \brief  Portable C fixed point code.  These functions define the
//!         calculate the phase voltage
//!

#include "volt_recons.h"

#ifdef __TMS320C28XX_CLA__
#pragma CODE_SECTION(VOLREC_init, "Cla1Prog2");
#endif

//*****************************************************************************
//
// VOLREC_init
//
//*****************************************************************************
VOLREC_Handle VOLREC_init(void *pMemory, const size_t numBytes)
{
    VOLREC_Handle handle;

    if((int16_t)numBytes < (int16_t)sizeof(VOLREC_Obj))
    {
        return((VOLREC_Handle)NULL);
    }

    // Assign the handle
    handle = (VOLREC_Handle)pMemory;

    return(handle);
} // end of VOLREC_init() function


//*****************************************************************************
//
// VOLREC_reset
//
//*****************************************************************************
void VOLREC_reset(VOLREC_Handle handle)
{
    VOLREC_Obj *obj = (VOLREC_Obj *)handle;

    obj->sf = 1.0f;
    obj->sfCalc = 1.0f;

    obj->VaSenSum = 0.0f;
    obj->VaSum = 0.0f;

    obj->VaSenRms = 0.0f;
    obj->VaRms = 0.0f;

    obj->numSamples = 0;

    obj->signPrev = 0;
    obj->signCurr = 0;
}

//*****************************************************************************
//
// VOLREC_setCoeffs
//
//*****************************************************************************
void VOLREC_setParams(VOLREC_Handle handle,
                const float32_t filterPole_rps, const float32_t ctrlFreq_Hz)
{
    VOLREC_Obj *obj = (VOLREC_Obj *)handle;
    float32_t beta_lp_rad;
    uint16_t cn;

    beta_lp_rad = filterPole_rps / ctrlFreq_Hz;

    obj->a1 = (beta_lp_rad - (float_t)2.0f) / (beta_lp_rad + (float_t)2.0f);
    obj->b0 = beta_lp_rad / (beta_lp_rad + (float_t)2.0f);
    obj->b1 = obj->b0;

    for(cn = 0; cn < 3; cn++)
    {
        obj->x1.value[cn] = 0.0f;
        obj->y1.value[cn] = 0.0f;

        obj->Vin_V.value[cn] = 0.0f;
        obj->Vs_V.value[cn] = 0.0f;
    }

    obj->sf = 0.925f;
    obj->sfCalc = 0.925f;
    obj->threshold = MOTOR_THRESHOLD_VOLTAGE_V;      // 1.0V

    obj->VaSenSum = 0.0f;
    obj->VaSum = 0.0f;

    obj->numSamples = 0;
    obj->minSamples = (int32_t)(ctrlFreq_Hz / MOTOR_MAX_ELEC_FREQ_Hz);
    obj->maxSamples = (int32_t)(ctrlFreq_Hz / MOTOR_MIN_ELEC_FREQ_Hz);

    obj->signPrev = 0;
    obj->signCurr = 0;

    return;
}

// end of file
