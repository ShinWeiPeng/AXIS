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

//! \file   ~/libraries/utilities/mod6cnt/include/mod6cnt.h
//! \brief  Contains the public interface to odulo 6 counter (MOD6CNT)
//!         module routines
//!

#ifndef MOD6CNT_H
#define MOD6CNT_H


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
//! \defgroup MOD6CNT MOD6CNT
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


//! \brief Defines the angle generator (ANGLE_COMP) object
//!
typedef struct _MOD6CNT_Obj_
{
    uint32_t counter;
    uint16_t counterMax;
    bool     flagCount;
} MOD6CNT_Obj;


//! \brief Defines the MOD6CNT handle
//!
typedef struct _MOD6CNT_Obj_  *MOD6CNT_Handle;


//
// the function prototypes
//

//! \brief     Gets the modulo 6 counter
//! \param[in] handle  The modulo 6 counter (MOD6CNT) handle
//! \return    The modulo 6 counter
static inline uint16_t MOD6CNT_getCounter(MOD6CNT_Handle handle)
{
    MOD6CNT_Obj *obj = (MOD6CNT_Obj *)handle;

    return((uint16_t)obj->counter);
} // end of MOD6CNT_getCounter() function


//! \brief     Sets the count flag
//! \param[in] handle  The modulo 6 counter (MOD6CNT) handle
//! \param[in] flagCount The count flag
//! \return    N/A
static inline void
MOD6CNT_setFlagCount(MOD6CNT_Handle handle, const bool flagCount)
{
    MOD6CNT_Obj *obj = (MOD6CNT_Obj *)handle;

    obj->flagCount = flagCount;

    return;
} // end of MOD6CNT_setFlagCount() function


//! \brief     Sets the current counter value
//! \param[in] handle  The modulo 6 counter (MOD6CNT) handle
//! \param[in] counter The current counter value
//! \return    N/A
static inline void
MOD6CNT_setCounter(MOD6CNT_Handle handle, const uint16_t counter)
{
    MOD6CNT_Obj *obj = (MOD6CNT_Obj *)handle;

    obj->counter = counter;

    return;
} // end of MOD6CNT_setCounter() function


//! \brief     Sets the maximum count value
//! \param[in] handle  The modulo 6 counter (MOD6CNT) handle
//! \param[in] counterMax The maximum count value
//! \return    N/A
static inline void
MOD6CNT_setMaximumCount(MOD6CNT_Handle handle, const uint16_t counterMax)
{
    MOD6CNT_Obj *obj = (MOD6CNT_Obj *)handle;

    if(counterMax >= 1)
    {
        obj->counterMax = counterMax - 1;
    }

    return;
} // end of MOD6CNT_setMaximumCount() function


//! \brief     Initializes  modulo 6 counter (MOD6CNT) module
//! \param[in] pMemory   A pointer to the memory for the object
//! \param[in] numBytes  The number of bytes allocated for the object, bytes
//! \return    The  modulo 6 counter (MOD6CNT) object handle
extern MOD6CNT_Handle MOD6CNT_init(void *pMemory, const size_t numBytes);


//! \brief  Runs the modulo 6 counter
//! \param[in] handle       The modulo 6 counter (MOD6CNT) handle
//! \param[in] speedRef_Hz  The reference speed in Hz
static inline void
MOD6CNT_run(MOD6CNT_Handle handle, const float32_t speedRef_Hz)
{
    MOD6CNT_Obj *obj = (MOD6CNT_Obj *)handle;

    if(obj->flagCount == true)
    {
        if(speedRef_Hz >= 0.0f)
        {
            if(obj->counter == obj->counterMax)
            {
                obj->counter = 0;
            }
            else
            {
                obj->counter++;
            }
        }
        else
        {
            if(obj->counter == 0)
            {
                obj->counter = 5;
            }
            else
            {
                obj->counter--;
            }
        }
    }

    return;
} // end of MOD6CNT_run()

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

#endif // end of MOD6CNT_H definition

