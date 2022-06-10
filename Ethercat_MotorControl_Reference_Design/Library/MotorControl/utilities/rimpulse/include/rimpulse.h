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

//! \file   ~/libraries/utilities/rimpulse/include/rimpulse.h
//! \brief  Contains the public interface to the ramp control
//! \brief  and impulse (RIMPULSE)
//!         module routines
//!

#ifndef RIMPULSE_H
#define RIMPULSE_H


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif


//*****************************************************************************
//
//! \defgroup RIMPULSE RIMPULSE
//! @{
//
//*****************************************************************************

//
// the includes
//
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include <math.h>
#endif // __TMS320C28XX_CLA__

#include "libraries/math/include/math.h"

//modules
#include "userParams.h"

//! \brief Defines the ramp control and impulse (RIMPULSE) object
//!
typedef struct _RIMPULSE_Obj_
{
    uint32_t    desiredOutEnd;      // desired ramp input end
    uint32_t    desiredOutStart;    // desired ramp input start
    uint32_t    desiredOutMin;      // desired ramp input start
    uint32_t    periodOut;          // Ramp3 output
    uint32_t    counter;            // Variable: Impulse generator counter

    uint16_t    ramp3Delay;         // ramp3 delay expressed in no of sampling period
    uint16_t    ramp3DelayCount;    // counter for rmp3 delay

    bool        trigFlag;           // Impulse generator output
    bool        ramp3DoneFlag;      // Output: Flag output

} RIMPULSE_Obj;


//! \brief Defines the RIMPULSE handle
//!
typedef struct _RIMPULSE_Obj_  *RIMPULSE_Handle;


//
// the function prototypes
//

//! \brief     Gets the ramp3 done flag
//! \param[in] handle  The ramp control and impulse (RIMPULSE) handle
//! \return    The ramp3 done flag
static inline float32_t RIMPULSE_getRmpDoneFlag(RIMPULSE_Handle handle)
{
    RIMPULSE_Obj *obj = (RIMPULSE_Obj *)handle;

    return(obj->ramp3DoneFlag);
} // end of RIMPULSE_getRmpDoneFlag() function

//! \brief     Gets the impulse trig flag
//! \param[in] handle  The ramp control and impulse (RIMPULSE) handle
//! \return    The impulse trig flag
static inline float32_t RIMPULSE_getTrigFlag(RIMPULSE_Handle handle)
{
    RIMPULSE_Obj *obj = (RIMPULSE_Obj *)handle;

    return(obj->trigFlag);
} // end of RIMPULSE_getTrigFlag() function

//! \brief     Initializes the ramp control and impulse (RIMPULSE) module
//! \param[in] pMemory   A pointer to the memory for the object
//! \param[in] numBytes  The number of bytes allocated for the object, bytes
//! \return    The ramp control and impulse (RIMPULSE) handle
extern RIMPULSE_Handle RIMPULSE_init(void *pMemory, const size_t numBytes);

//! \brief     Sets the parameters
//! \param[in] handle       The ramp control and impulse (RIMPULSE) handle
//! \param[in] pUserParams  The pointer to the user parameters
//! \param[in] freqStart_Hz The start frequency of the motor
//! \param[in] freqEnd_Hz   The end frequency of the motor
extern void
RIMPULSE_setParams(RIMPULSE_Handle handle, const USER_Params *pUserParams,
                   const float32_t freqStart_Hz, const float32_t freqEnd_Hz,
                   const uint16_t rampDelay);

//! \brief  runs ramp control and impulse
//! \param[in] handle     The ramp control and impulse (RIMPULSE) handle
//! \return    N/A
static inline void RIMPULSE_resetParameters(RIMPULSE_Handle handle)
{
    RIMPULSE_Obj *obj = (RIMPULSE_Obj *)handle;

    obj->periodOut = obj->desiredOutStart;
    obj->desiredOutMin = obj->desiredOutEnd;

    obj->counter = 0;

    obj->ramp3DelayCount = 0;
    obj->ramp3DoneFlag = false;
    obj->trigFlag = false;

    return;
} // end of RIMPULSE_resetParameters()

//! \brief  runs ramp control and impulse
//! \param[in] handle     The ramp control and impulse (RIMPULSE) handle
//! \return    N/A
static inline void RIMPULSE_run(RIMPULSE_Handle handle)
{
    RIMPULSE_Obj *obj = (RIMPULSE_Obj *)handle;

    if(obj->periodOut <= obj->desiredOutEnd)
    {
        obj->ramp3DoneFlag = true;
    }
    else
    {
        obj->ramp3DelayCount++;

        if(obj->ramp3DelayCount >= obj->ramp3Delay)
        {
            obj->periodOut--;

            if(obj->periodOut < obj->desiredOutMin)
            {
                obj->periodOut = obj->desiredOutMin;
            }

            obj->ramp3DelayCount = 0;
        }
    }

    obj->trigFlag = false;

    obj->counter++;

    if(obj->counter >= obj->periodOut)
    {
        obj->trigFlag = true;
        obj->counter = 0;
    }

    return;
} // end of RIMPULSE_run()


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif // extern "C"

#endif // end of RIMPULSE_H definition

