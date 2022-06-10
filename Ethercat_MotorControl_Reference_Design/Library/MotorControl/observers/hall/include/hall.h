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

#ifndef _HALL_H_
#define _HALL_H_

//! \file   \libraries\observers\hall\include\hall.h
//! \brief  Contains the public interface to the 
//!         encoder object
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
//! \defgroup HALL HALL
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

// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

// State machine typedef for HALL status
typedef enum
{
    HALL_IDLE             = 0,
    HALL_ALIGNMENT        = 1,
    HALL_CALIBRATION      = 2,
    HALL_RUN              = 3
} HALL_Status_e;

#ifdef HALL_CAL
#define CAL_BUF_NUM         13
#endif  //HALL_CAL


//! \brief Defines the HALL controller object
//!
typedef struct _HALL_Obj_
{
    float32_t capScaler;           // Scaler converting 1/N CPU cycles
    float32_t pwmScaler;           // Scaler converting 1/N CPU cycles
    float32_t speedCAP_Hz;         // Hz
    float32_t speedPWM_Hz;         // Hz
    float32_t speedHall_Hz;        // Hz
    float32_t speedSwitch_Hz;      // Hz
    uint32_t  timeStampCAP;        //
    uint32_t  timeCountPWM;        //
    uint32_t  timeCount[2];        //
    uint32_t  timeCountMax;        //

    float32_t thetaBuff[7];         // 1~6 are valid value
    float32_t thetaHall_rad;        //
    float32_t thetaDelta_rad;       //
    float32_t thetaDelta_pu;        //
    float32_t thetaHall_pu;         //
    uint16_t  gpioHallU;            // the GPIO for Hall U
    uint16_t  gpioHallV;            // the GPIO for Hall V
    uint16_t  gpioHallW;            // the GPIO for Hall W
    uint16_t  hallIndex;            //
    uint16_t  hallIndexPrev;        //
    uint16_t  hallPrev[7];          //
    uint16_t  hallDirection;        //
    HALL_Status_e hallStatus;       //

#ifdef HALL_CAL
    float32_t thetaCalBuff[7];               //
    float32_t thetaIndexBuff[CAL_BUF_NUM];   //
    uint16_t  hallIndexNow;                  //
    uint16_t  hallIndexPre;                  //
    uint16_t  hallIndexFlag;                 //
#endif  //HALL_CAL
} HALL_Obj;


//! \brief Defines the HALL handle
//!
typedef struct _HALL_Obj_ *HALL_Handle;

// **************************************************************************
// the function prototypes

//! \brief     Initializes the HALL controller
//! \param[in] pMemory   A pointer to the memory for the HALL controller object
//! \param[in] numBytes  The number of bytes allocated for the HALL controller object, bytes
//! \return The HALL controller (HALL) object handle
extern HALL_Handle HALL_init(void *pMemory, const size_t numBytes);

//! \brief     Sets the hall estimator parameters
//! \param[in] handle      The HALL controller handle
extern void HALL_setParams(HALL_Handle handle, const USER_Params *pUserParams);

//! \brief     Sets the angle buffer value for hall estimator
//! \param[in] handle      The HALL controller handle
extern void HALL_setAngleBuf(HALL_Handle handle, const float32_t *ptrAngleBuf);

//! \brief     Resets the hall estimator parameters
//! \param[in] handle      The HALL controller handle
extern void HALL_resetParams(HALL_Handle handle);

//! \brief     Sets GPIO for hall sensors input
//! \param[in] handle  the HALL Handle
extern void HALL_setGPIOs(HALL_Handle handle, const uint16_t gpioHallU,
                   const uint16_t gpioHallV, const uint16_t gpioHallW);

//! \brief     Gets the feedback speed from hall estimator
//! \param[in] handle  the ENC Handle
static inline float32_t HALL_getSpeed_Hz(HALL_Handle handle)
{
    HALL_Obj *obj = (HALL_Obj *)handle;

    return(obj->speedHall_Hz);
}

//! \brief     Sets a value to the speed for the hall estimator
//! \param[in] handle  the ENC Handle
//! \param[in] speedHall_Hz  the speed value, Hz
static inline void HALL_setSpeed_Hz(HALL_Handle handle, const float32_t speedHall_Hz)
{
    HALL_Obj *obj = (HALL_Obj *)handle;

    obj->speedHall_Hz = speedHall_Hz;

    return;
}

//! \brief     Sets a value to the time stamp for the hall estimator
//! \param[in] handle  the ENC Handle
//! \param[in] timeStamp  the time stamp value, clock cycles
static inline void HALL_setTimeStamp(HALL_Handle handle, const uint32_t timeStamp)
{
    HALL_Obj *obj = (HALL_Obj *)handle;

    obj->timeStampCAP = timeStamp;

    return;
}

//! \brief     Gets the angle from the hall estimator, rad
//! \param[in] handle  the ENC Handle
static inline float32_t HALL_getAngle_rad(HALL_Handle handle)
{
    HALL_Obj *obj = (HALL_Obj *)handle;

    return(obj->thetaHall_rad);
}

//! \brief     Gets the hall sensors input GPIO state
//! \param[in] handle  the ENC Handle
static inline uint16_t HALL_getInputState(HALL_Handle handle)
{
    HALL_Obj *obj = (HALL_Obj *)handle;

    uint32_t hallState = 0;

    hallState = GPIO_readPin(obj->gpioHallU);
    hallState += GPIO_readPin(obj->gpioHallV)<<1;
    hallState += GPIO_readPin(obj->gpioHallW)<<2;

    return(hallState & 0x0007);
}


#ifdef HALL_CAL
//! \brief     Runs the HALL controller
//! \param[in] handle      The HALL controller handle
//! \param[in] refValue    The reference value to the controller
static inline void HALL_calibrateIndexAngle(HALL_Handle handle, float32_t angle)
{
    HALL_Obj *obj = (HALL_Obj *)handle;
    obj->hallIndexNow = HALL_getInputState(handle);

    if(obj->hallIndexNow != obj->hallIndexPre)
    {
        obj->hallIndexPre = obj->hallIndexNow;

        if(obj->hallIndexFlag == 0)
        {
            obj->thetaIndexBuff[obj->hallIndexNow] = angle;

            if(obj->hallIndexNow == 0x01)
            {
                obj->hallIndexFlag = 1;
            }
        }
        else
        {
            obj->thetaIndexBuff[6 + obj->hallIndexNow] = angle;

            if(obj->hallIndexNow == 0x01)
            {
                obj->hallIndexFlag = 0;
            }
        }

        obj->thetaCalBuff[obj->hallIndexNow] = 0.5f *
                (obj->thetaIndexBuff[obj->hallIndexNow] +
                 obj->thetaIndexBuff[6 + obj->hallIndexNow]);
    }

    return;
}
#endif  //HALL_CAL

//! \brief     Sets the force angle and index for next step of the hall estimator
//! \param[in] handle      The HALL controller handle
//! \param[in] refValue    The reference value to the controller
static inline void HALL_setForceAngleAndIndex(HALL_Handle handle, float32_t speedRef)
{
    HALL_Obj *obj = (HALL_Obj *)handle;
    obj->hallIndex = HALL_getInputState(handle);

    if(speedRef > 0.0f)
    {
        obj->hallIndexPrev = obj->hallPrev[obj->hallIndex];
        obj->hallDirection = 0;

        obj->thetaHall_rad = obj->thetaBuff[obj->hallIndex] - obj->thetaDelta_rad;
    }
    else
    {
        obj->hallIndexPrev = obj->hallIndex;
        obj->hallDirection = 1;

        obj->thetaHall_rad = obj->thetaBuff[obj->hallIndex]  + obj->thetaDelta_rad;
    }

    if(obj->thetaHall_rad >= MATH_PI)
    {
        obj->thetaHall_rad = obj->thetaHall_rad - MATH_TWO_PI;
    }
    else if(obj->thetaHall_rad <= -MATH_PI)
    {
        obj->thetaHall_rad = obj->thetaHall_rad + MATH_TWO_PI;
    }

    return;
}

//! \brief     Runs the HALL controller
//! \param[in] handle  The HALL controller handle
//! \param[in] speedRef The reference speed value to the controller
#define HALL_calcAngle HALL_run

static inline void HALL_run(HALL_Handle handle, float32_t speedRef)
{
    HALL_Obj *obj = (HALL_Obj *)handle;

    obj->hallIndex = HALL_getInputState(handle);
    obj->timeCountPWM++;

    if(obj->hallIndex != obj->hallIndexPrev)
    {
        if(speedRef > 0.0f)
        {
            obj->thetaHall_rad = obj->thetaBuff[obj->hallIndex] + obj->thetaDelta_rad;
            obj->hallDirection = 0;
        }
        else
        {
            obj->thetaHall_rad = obj->thetaBuff[obj->hallIndex] - obj->thetaDelta_rad;
            obj->hallDirection = 1;
        }

        obj->hallIndexPrev = obj->hallIndex;

        float32_t speedCAP_Hz = obj->capScaler / ((float32_t)obj->timeStampCAP);
        float32_t speedPWM_Hz = obj->pwmScaler /
                ((float32_t)(obj->timeCountPWM + obj->timeCount[1] + obj->timeCount[0]));

        if(obj->hallDirection == 0)
        {
            obj->speedCAP_Hz = speedCAP_Hz;
            obj->speedPWM_Hz = speedPWM_Hz;
        }
        else
        {
            obj->speedCAP_Hz = -speedCAP_Hz;
            obj->speedPWM_Hz = -speedPWM_Hz;
        }

        if(speedPWM_Hz > obj->speedSwitch_Hz)
        {
            obj->speedHall_Hz = obj->speedCAP_Hz;
        }
        else
        {
            obj->speedHall_Hz = obj->speedPWM_Hz;
        }

        obj->timeCount[1] = obj->timeCount[0];
        obj->timeCount[0] = obj->timeCountPWM;
        obj->timeCountPWM = 0;
    }

    if(obj->timeCountPWM > obj->timeCountMax)
    {
        obj->speedCAP_Hz = 0.0f;
        obj->speedPWM_Hz = 0.0f;
        obj->speedHall_Hz = 0.0f;
        obj->timeCountPWM = 0;
    }

    if(obj->thetaHall_rad >= MATH_PI)
    {
        obj->thetaHall_rad = obj->thetaHall_rad - MATH_TWO_PI;
    }
    else if(obj->thetaHall_rad <= -MATH_PI)
    {
        obj->thetaHall_rad = obj->thetaHall_rad + MATH_TWO_PI;
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

#endif //end of _HALL_H_ definition

