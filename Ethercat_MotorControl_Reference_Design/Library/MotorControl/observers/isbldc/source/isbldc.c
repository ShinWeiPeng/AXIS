
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

//! \file   \libraries\observers\isbldc\source\isbldc.c
//! \brief  Portable C fixed point code.  These functions define the
//!         enhanced isbldc estimator
//!
// **************************************************************************
// the includes
#include "isbldc.h"

// **************************************************************************
// the defines

#pragma CODE_SECTION(ISBLDC_full_run, ".TI.ramfunc");

// **************************************************************************
// the globals


// **************************************************************************
// the functions

ISBLDC_Handle ISBLDC_init(void *pMemory, const size_t numBytes)
{
    ISBLDC_Handle handle;

    if(numBytes < sizeof(ISBLDC_Obj))
    {
        return((ISBLDC_Handle)NULL);
    }

    // assign the handle
    handle = (ISBLDC_Handle)pMemory;

    return(handle);
} // end of ISBLDC_init() function

//------------------------------------------------------------------------------
void ISBLDC_setParams(ISBLDC_Handle handle, const USER_Params *pUserParams,
                      const float32_t threshold_max, const float32_t threshold_min)
{
    ISBLDC_Obj *obj = (ISBLDC_Obj *)handle;

    obj->speedScaler = pUserParams->ctrlFreq_Hz;

    obj->timeStamp = 2000;
    obj->timeStampBuf[0] = 2000;
    obj->timeStampBuf[1] = 2000;
    obj->timeStampBuf[2] = 2000;
    obj->timeStampBuf[3] = 2000;
    obj->timeStampBuf[4] = 2000;

    obj->commState = 0;

    obj->bemfLockFlag = false;
    obj->commTrigFlag = false;

    obj->thresholdMax = pUserParams->dcBus_nominal_V * threshold_max;      //
    obj->thresholdMin = pUserParams->dcBus_nominal_V * threshold_min;      //

    obj->thresholdSF = (obj->thresholdMax - obj->thresholdMin) / pUserParams->maxFrequency_Hz;

    obj->thresholdInt = obj->thresholdMax;

    return;
}

//------------------------------------------------------------------------------
void ISBLDC_updateThresholdInt(ISBLDC_Handle handle, const float32_t speedRef)
{
    ISBLDC_Obj *obj = (ISBLDC_Obj *)handle;

    float32_t speedAbs = fabsf(speedRef);
    float32_t thresholdInt = obj->thresholdMax - obj->thresholdSF * speedAbs;

    if(thresholdInt < obj->thresholdMin)
    {
        obj->thresholdInt = obj->thresholdMin;
    }
    else if(thresholdInt > obj->thresholdMax)
    {
        obj->thresholdInt = obj->thresholdMax;
    }
    else
    {
        obj->thresholdInt = thresholdInt;
    }

    return;
}

//------------------------------------------------------------------------------
void ISBLDC_resetState(ISBLDC_Handle handle)
{
    ISBLDC_Obj *obj = (ISBLDC_Obj *)handle;

    obj->thresholdInt = obj->thresholdMax;
    obj->bemfLockFlag = false;
    obj->commTrigFlag = false;
    obj->bemfInt = 0.0f;
    obj->speedInt_Hz = 0.0f;

    obj->timeStamp = 2000;
    obj->timeStampBuf[0] = 2000;
    obj->timeStampBuf[1] = 2000;
    obj->timeStampBuf[2] = 2000;
    obj->timeStampBuf[3] = 2000;
    obj->timeStampBuf[4] = 2000;

    obj->thresholdInt = obj->thresholdMax;

    return;
}

//------------------------------------------------------------------------------
void ISBLDC_full_run(ISBLDC_Handle handle, MATH_Vec3 *pVabcVec)
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
}   // end of InstaSPIN_BLDC() function

// end of file
