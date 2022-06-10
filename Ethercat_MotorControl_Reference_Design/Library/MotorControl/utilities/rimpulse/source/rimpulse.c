//#############################################################################
// $TI Release: MotorControl SDK v4.00.00.00 $
// $Release Date: Thu Feb 17 18:05:20 CST 2022 $
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

//! \file   libraries/utilities/rimpulse/source/rimpulse.c
//! \brief  Contains the public interface to the ramp control
//! \brief  and impulse (RIMPULSE)
//!         module routines
//!

// **************************************************************************
//
// the includes
//
#include "rimpulse.h"


#ifdef __TMS320C28XX_CLA__
#pragma CODE_SECTION(RIMPULSE_init, "Cla1Prog2");
#endif


//*****************************************************************************
//
// RIMPULSE_init
//
//*****************************************************************************
RIMPULSE_Handle RIMPULSE_init(void *pMemory, const size_t numBytes)
{
    RIMPULSE_Handle handle;

    if(numBytes < sizeof(RIMPULSE_Obj))
    {
        return((RIMPULSE_Handle)NULL);
    }

    //
    // assign the handle
    //
    handle = (RIMPULSE_Handle)pMemory;

    return(handle);
} // end of RIMPULSE_init() function

// Sets up parameters for angle generation
void RIMPULSE_setParams(RIMPULSE_Handle handle, const USER_Params *pUserParams,
                        const float32_t freqStart_Hz, const float32_t freqEnd_Hz,
                        const uint16_t rampDelay)
{
    RIMPULSE_Obj *obj = (RIMPULSE_Obj *)handle;

    obj->desiredOutEnd = (uint32_t)(pUserParams->ctrlFreq_Hz / freqEnd_Hz / 6.0f);
    obj->desiredOutStart = (uint32_t)(pUserParams->ctrlFreq_Hz / freqStart_Hz / 6.0f);

    obj->periodOut = obj->desiredOutStart;
    obj->desiredOutMin = obj->desiredOutEnd;

    obj->counter = 0;

    obj->ramp3Delay = rampDelay;
    obj->ramp3DelayCount = 0;
    obj->ramp3DoneFlag = false;
    obj->trigFlag = false;

    return;
} // end of RIMPULSE_setParams() function

// end of the file

