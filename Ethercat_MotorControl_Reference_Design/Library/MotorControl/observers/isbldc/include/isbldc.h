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

#ifndef ISBLDC_H
#define ISBLDC_H

//! \file   \libraries\observers\isbldc\include\isbldc.h
//! \brief  Contains the public interface to the
//!         enhanced isbldc estimator
//!


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup ISBLDC ISBLDC
//! @{
//
//*****************************************************************************

// the includes
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/src/float/CLAmath.h"
#else
#include <math.h>
#endif

#include "libraries/math/include/math.h"

//modules
#include "userParams.h"

#include "gpio.h"


//-----------------------------------------------------------------------------
// enumerations
//-----------------------------------------------------------------------------
typedef enum
{
    PHS_A = 0,
    PHS_B = 1,
    PHS_C = 2
}ISBLDC_PHS_e;

typedef enum
{
    DIR_NEG = 0,
    DIR_POS = 1
}ISBLDC_DIR_e;

//-----------------------------------------------------------------------------
// Structure and object definitions
//-----------------------------------------------------------------------------
typedef struct _ISBLDC_Obj_
{
    MATH_Vec3   Vabcg;				// Contains real Va/b/c + Va/b/c_Offset, referenced to ground
    MATH_Vec3   VabcOffset;			// VaOffset
    MATH_Vec3   Vabcn;              // Va/Vb/Vc to neutral
    float32_t   VintPhase;          // V
    float32_t   thresholdMin;       // Minimum integration threshold
    float32_t   thresholdMax;       // Maximum integration threshold
    float32_t   thresholdSF;        // scale factor for calculate threshold
    float32_t   thresholdInt;		// Integration threshold where a commutate happens
    float32_t   bemfInt;            // V
    float32_t   speedScaler;        // Scaler converting 1/N cycles
    float32_t   speedInt_Hz;        // Hz

    uint32_t    timeStamp;          // Current Timestamp corresponding to a capture event
    uint32_t    timeStampBuf[5];    // Timestamp buffer
    uint32_t    timePeriod;         // Time Period
    uint32_t    intTimer;           // Interval timer

    uint16_t    commState;          // Input: Values 0 to 5

    ISBLDC_PHS_e  bemfPhase;        // What phase to sense (A/B/C)
    ISBLDC_DIR_e  bemfDirect;       // BEMF integrator count direction (POS/NEG)
    bool          bemfLockFlag;     // Flyback voltage lockout flag
	bool          commTrigFlag;     // Commutation trigger impulse
} ISBLDC_Obj;

typedef struct _ISBLDC_Obj_ *ISBLDC_Handle;    //  Handle to object INSTASPIN_BLDC


//******************************************************************************
// typedefs
typedef struct _BLDC_Obj_t_
{
    float32_t IdcRefSet;
    float32_t IdcStart;
    float32_t IdcAlign;
    float32_t IdcDelta;

    float32_t pwmDutySet;
    float32_t pwmDutyStart;
    float32_t pwmDutyAlign;
    float32_t pwmDutyDelta;

    float32_t IdcRef;
    float32_t IdcIn;
    float32_t IdcInFilter;
    float32_t IdcInBuff[3];

    float32_t pwmDutyOut;
    float32_t pwmDuty;
    float32_t commIndex;
    uint16_t  commState;
    uint16_t  commSampleCount;
    uint16_t  commSampleDelay;
    bool      commTrigFlag;
} BLDC_Obj;

//! \brief Defines the BLDC_Obj_t_ handle
//!
typedef struct _BLDC_Obj_t_ *BLDC_Handle;    //  Handle to object INSTASPIN_BLDC

//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------
// the function prototypes
//! \brief     Initializes the ISBLDC controller
//! \param[in] pMemory   A pointer to the memory for the ISBLDC controller object
//! \param[in] numBytes  The number of bytes allocated for the ISBLDC controller object, bytes
//! \return The ISBLDC controller (ISBLDC) object handle
extern ISBLDC_Handle ISBLDC_init(void *pMemory, const size_t numBytes);

//! \brief     Reset the ISBLDC controller
//! \param[in] handle      The ISBLDC controller handle
extern void ISBLDC_resetParams(ISBLDC_Handle handle);

//! \brief     Set the ISBLDC controller
//! \param[in] handle      The ISBLDC controller handle
//! \param[in] pUserParams  The pointer to the user parameters
extern void
ISBLDC_setParams(ISBLDC_Handle handle, const USER_Params *pUserParams,
                 const float32_t threshold_max, const float32_t threshold_min);

//! \brief     Resets the ISBLDC controller
//! \param[in] handle      The ISBLDC controller handle
extern void ISBLDC_resetState(ISBLDC_Handle handle);

//! \brief     Updates integral threshold
//! \param[in] handle   The ISBLDC controller handle
//! \param[in] speedRef The target speed
extern void
ISBLDC_updateThresholdInt(ISBLDC_Handle handle, const float32_t speedRef);


//! \brief     Gets commutation trig flag for BEMF
//! \param[in] handle The ISBLDC controller handle
static inline bool ISBLDC_getCommTrigFlag(ISBLDC_Handle handle)
{
    ISBLDC_Obj *obj = (ISBLDC_Obj *)handle;

    return(obj->commTrigFlag);
}

//! \brief     Gets the estimation speed
//! \param[in] handle The ISBLDC controller handle
static inline float32_t ISBLDC_getSpeedINT(ISBLDC_Handle handle)
{
    ISBLDC_Obj *obj = (ISBLDC_Obj *)handle;

    return(obj->speedInt_Hz);
}

//! \brief     Set the bemf threshold value
//! \param[in] handle The ISBLDC controller handle
//! \param[in] thresholdInt The bemf threshold value
static inline void
ISBLDC_setThresholdInt(ISBLDC_Handle handle, const float32_t thresholdInt)
{
    ISBLDC_Obj *obj = (ISBLDC_Obj *)handle;

    obj->thresholdInt = thresholdInt;

    return;
}

//! \brief     Set the bemf threshold value
//! \param[in] handle The ISBLDC controller handle
//! \param[in] thresholdInt The bemf threshold value
static inline void
ISBLDC_setVabcOffset(ISBLDC_Handle handle, const MATH_Vec3 *pVabcOffset)
{
    ISBLDC_Obj *obj = (ISBLDC_Obj *)handle;

    obj->VabcOffset.value[0] = pVabcOffset->value[0];
    obj->VabcOffset.value[1] = pVabcOffset->value[1];
    obj->VabcOffset.value[2] = pVabcOffset->value[2];

    return;
}

//! \brief     Set low pass filter parameters for BEMF
//! \param[in] handle The ISBLDC controller handle
static inline void
ISBLDC_setCommState(ISBLDC_Handle handle, uint16_t commState)
{
    ISBLDC_Obj *obj = (ISBLDC_Obj *)handle;

    obj->commState = commState;

    return;
}

//! \brief     Set low pass filter parameters for BEMF
//! \param[in] handle The ISBLDC controller handle
extern void  ISBLDC_setSpeedFilterParams(ISBLDC_Handle handle);


//! \brief     Runs the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] refValue    The reference value to the controller
extern void ISBLDC_full_run(ISBLDC_Handle handle, MATH_Vec3 *pVabcVec);


//! \brief     Runs the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] refValue    The reference value to the controller
#define ISBLDC_inline_run ISBLDC_run

static inline void ISBLDC_run(ISBLDC_Handle handle, MATH_Vec3 *pVabcVec)
{
    ISBLDC_Obj *obj = (ISBLDC_Obj *)handle;

    // State Filter decides integration direction and bemf phase to sense
    switch(obj->commState)
    {
        case 0:
            obj->bemfPhase = PHS_C;
            obj->bemfDirect = DIR_NEG;
            break;

        case 1:
            obj->bemfPhase = PHS_B;
            obj->bemfDirect = DIR_POS;
            break;

        case 2:
            obj->bemfPhase = PHS_A;
            obj->bemfDirect = DIR_NEG;
            break;

        case 3:
            obj->bemfPhase = PHS_C;
            obj->bemfDirect = DIR_POS;
            break;

        case 4:
            obj->bemfPhase = PHS_B;
            obj->bemfDirect = DIR_NEG;
            break;

        case 5:
            obj->bemfPhase = PHS_A;
            obj->bemfDirect = DIR_POS;
    }

    obj->Vabcg.value[0] = pVabcVec->value[0] - obj->VabcOffset.value[0];
    obj->Vabcg.value[1] = pVabcVec->value[1] - obj->VabcOffset.value[1];
    obj->Vabcg.value[2] = pVabcVec->value[2] - obj->VabcOffset.value[2];

    obj->Vabcn.value[0] = obj->Vabcg.value[0] -
                        ((obj->Vabcg.value[1] + obj->Vabcg.value[2]) * 0.5f);
    obj->Vabcn.value[1] = obj->Vabcg.value[1] -
                        ((obj->Vabcg.value[0] + obj->Vabcg.value[2]) * 0.5f);
    obj->Vabcn.value[2] = obj->Vabcg.value[2] -
                        ((obj->Vabcg.value[0] + obj->Vabcg.value[1]) * 0.5f);

    switch(obj->bemfPhase)
    {
        case PHS_A:
            // Point to phase voltage A to neutral
            obj->VintPhase = obj->Vabcn.value[0];
            break;

        case PHS_B:
            // Point to phase voltage B to neutral
            obj->VintPhase = obj->Vabcn.value[1];
            break;

        case PHS_C:
            // Point to phase voltage C to neutral
            obj->VintPhase = obj->Vabcn.value[2];
            break;
    }

    obj->commTrigFlag = false;

    if (obj->bemfDirect == DIR_POS)
    {
       // Integrate positive
        if(obj->bemfLockFlag == true)
        {
            if(obj->VintPhase < 0.0f)
            {
                obj->bemfLockFlag = false;
            }
        }
        else
        {
            obj->bemfInt += obj->VintPhase;     // Integrate the phase voltage

            if(obj->bemfInt > 0.0f)             // Only integrate positively
            {
                // Check to see if there is enough volt-seconds to commutate
                if(obj->bemfInt > obj->thresholdInt)
                {
                    obj->bemfLockFlag = true;
                    obj->commTrigFlag = true;
                    obj->bemfInt = 0.0f;

                }
            }
            else
            {
                obj->bemfInt = 0.0f;
            }
        }
    }
    else
    {
        // Integrate negative
        if(obj->bemfLockFlag == true)
        {
            if(obj->VintPhase > 0.0f)
            {
                obj->bemfLockFlag = false;
            }
        }
        else
        {
            obj->bemfInt += obj->VintPhase;       // Integrate the phase voltage

            if(obj->bemfInt < 0.0f)          // Only integrate negatively
            {
                // Check to see if there is enough volt-seconds to commutate
                if(obj->bemfInt < -obj->thresholdInt)
                {
                    obj->bemfLockFlag = true;
                    obj->commTrigFlag = true;
                    obj->bemfInt = 0.0f;

                }
            }
            else
            {
                obj->bemfInt = 0.0f;
            }
        }
    }

    // calculate speed
    obj->timeStamp++;

    if(obj->commTrigFlag == true)
    {

        obj->timePeriod = (obj->timeStamp       + obj->timeStampBuf[0] +
                           obj->timeStampBuf[1] + obj->timeStampBuf[2] +
                           obj->timeStampBuf[3] + obj->timeStampBuf[4] );

        obj->timeStampBuf[4] = obj->timeStampBuf[3];
        obj->timeStampBuf[3] = obj->timeStampBuf[2];
        obj->timeStampBuf[2] = obj->timeStampBuf[1];
        obj->timeStampBuf[1] = obj->timeStampBuf[0];
        obj->timeStampBuf[0] = obj->timeStamp;

        obj->speedInt_Hz = obj->speedScaler / ((float32_t)obj->timePeriod);

        obj->timeStamp = 0;
        obj->intTimer = 0;
    }

    obj->intTimer++;

    if(obj->intTimer > 20000)
    {
        obj->intTimer = 0;
        obj->speedInt_Hz = 0.0f;
    }

    return;
}

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
#endif

#endif  // end of ISBLDC_H defines

