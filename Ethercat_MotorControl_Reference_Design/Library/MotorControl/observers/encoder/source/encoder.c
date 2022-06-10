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

//! \file   \libraries\observers\encoder\source\encoder.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         encoder object
//!

// **************************************************************************
// the includes
#include "encoder.h"

// **************************************************************************
// the defines

#pragma CODE_SECTION(ENC_full_run, ".TI.ramfunc");

// **************************************************************************
// the globals


// **************************************************************************
// the functions

ENC_Handle ENC_init(void *pMemory, const size_t numBytes)
{
	ENC_Handle handle;

	if(numBytes < sizeof(ENC_Obj))
	{
		return((ENC_Handle)NULL);
	}

	// assign the handle
	handle = (ENC_Handle)pMemory;

	return(handle);
} // end of ENC_init() function

//------------------------------------------------------------------------------
void ENC_setHallGPIO(ENC_Handle handle, const uint16_t gpioHallU,
                     const uint16_t gpioHallV, const uint16_t gpioHallW)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->gpioHallU = gpioHallU;
    obj->gpioHallV = gpioHallV;
    obj->gpioHallW = gpioHallW;

    return;
}

//------------------------------------------------------------------------------
void ENC_setParams(ENC_Handle handle, const USER_Params *pUserParams)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->Ts_sec = pUserParams->ctrlPeriod_sec;

    obj->polePairs = pUserParams->motor_numPolePairs;
    obj->encLines = pUserParams->motor_numEncSlots;

    obj->mechanicalScaler = 0.25f / obj->encLines;

    obj->encState = ENC_IDLE;

    return;
}

//! \brief     Runs the ENC controller
//! \param[in] handle  The ENC controller handle
//! \param[in] pVabVec The reference value to the controller
//! \param[in] pIabVec The feedback value to the controller
void ENC_full_run(ENC_Handle handle)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    if(obj->encState == ENC_CALIBRATION_DONE)
    {
        obj->thetaMech_pu = obj->mechanicalScaler *
                (float32_t)EQEP_getPositionLatch(obj->qepHandle);

        obj->thetaMech_rad = obj->thetaMech_pu * MATH_TWO_PI;

        float32_t thetaElec_pu = obj->thetaMech_pu * obj->polePairs;

        obj->thetaElec_rad = (thetaElec_pu - ((int32_t)thetaElec_pu)) * MATH_TWO_PI;

        if(obj->thetaElec_rad >= MATH_PI)
        {
            obj->thetaElec_rad = obj->thetaElec_rad - MATH_TWO_PI;
        }
        else if(obj->thetaElec_rad <= -MATH_PI)
        {
            obj->thetaElec_rad = obj->thetaElec_rad + MATH_TWO_PI;
        }
    }
    else if(obj->encState == ENC_WAIT_FOR_INDEX)
    {
        if(EQEP_getInterruptStatus(obj->qepHandle) & EQEP_INT_INDEX_EVNT_LATCH)
        {
            EQEP_setInitialPosition(obj->qepHandle, EQEP_getPositionLatch(obj->qepHandle));

            EQEP_setPositionInitMode(obj->qepHandle, EQEP_INIT_RISING_INDEX);

            obj->encState = ENC_CALIBRATION_DONE;
        }
    }
    else    // obj->encState == ENC_ALIGNMENT
    {
        // during alignment, reset the current shaft position to zero
        EQEP_setPosition(obj->qepHandle, 0);

        // Reset pos cnt for QEP
        EQEP_clearInterruptStatus(obj->qepHandle, EQEP_INT_INDEX_EVNT_LATCH);

        // reset poscnt init on index
        EQEP_setPositionInitMode(obj->qepHandle, EQEP_INIT_DO_NOTHING);

    }

    return;
}


//----------------------------------------------------------------

// end of file
