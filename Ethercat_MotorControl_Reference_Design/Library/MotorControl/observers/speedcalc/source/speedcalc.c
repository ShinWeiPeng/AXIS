//#############################################################################
// $TI Release: MotorControl SDK v4.00.00.00 $
// $Release Date: Thu Feb 17 18:05:19 CST 2022 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
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

//! \file   \libraries\observers\speedcalc\source\speed_Observer.c
//! \brief  Portable C fixed point code.  These functions define the
//!         speed observer in generic form for Motor Control
//!

#include "speedcalc.h"    // Include speed observation definitions

// **************************************************************************
// the defines

// **************************************************************************
// the globals


// **************************************************************************
// the functions

SPDCALC_Handle SPDCALC_init(void *pMemory, const size_t numBytes)
{
    SPDCALC_Handle handle;

    if(numBytes < sizeof(SPDCALC_Obj))
    {
        return((SPDCALC_Handle)NULL);
    }

    // assign the handle
    handle = (SPDCALC_Handle)pMemory;

    return(handle);
} // end of SPDCALC_init() function

//------------------------------------------------------------------------------
void SPDCALC_reset(SPDCALC_Handle handle)
{
    SPDCALC_Obj *obj = (SPDCALC_Obj *)handle;

    obj->fbk = 0.0f;
    obj->Ui = 0.0f;

    return;
}

//------------------------------------------------------------------------------
void SPDCALC_setParams(SPDCALC_Handle handle, const USER_Params *pUserParams)
{
    SPDCALC_Obj *obj = (SPDCALC_Obj *)handle;

    obj->thetaDelta = pUserParams->ctrlPeriod_sec;

    obj->Umax = pUserParams->maxFrequency_Hz * MATH_TWO_PI;
    obj->Umin = -pUserParams->maxFrequency_Hz * MATH_TWO_PI;

    obj->Kp = 100.0f;
    obj->Ki = 50.0f * pUserParams->ctrlPeriod_sec;

    obj->Ui = 0.0f;

    return;
} // end of ESMO_run() function


//----------------------------------------------------------------

// end of file
