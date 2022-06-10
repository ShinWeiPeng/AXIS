//#############################################################################
// FILE    : Spd_Observer.h
// TITLE   : Header file to be shared between example and library for CPU data.
// Version : 1.0
//
//  Group           : C2000
//  Target Family   : F2837x
//  Created on      : Mar 28, 2018
//  Author          : Ramesh Ramamoorthy
//#############################################################################
// $TI Release: MotorControl SDK v4.00.00.00 $
// $Release Date: Thu Feb 17 18:05:19 CST 2022 $
// $Copyright: Copyright (C) 2013-2017 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################


#ifndef SPD_OBSERVER_H
#define SPD_OBSERVER_H

#include "device.h"

typedef struct _SPD_OBSERVER_obj_
{
    float32_t  Ref;             // Input: reference set-point
    float32_t  Fbk;             // Input: feedback
    float32_t  Err;             // Error
    float32_t  Out;             // Output: controller output
    float32_t  Kp;              // Parameter: proportional loop gain
    float32_t  Ki;              // Parameter: integral gain
    float32_t  KiT;             // Parameter
    float32_t  Umax;            // Parameter: upper saturation limit
    float32_t  Umin;            // Parameter: lower saturation limit
    float32_t  up;              // Data: proportional term
    float32_t  ui;              // Data: integral term
    float32_t  IqMax;           // Parameter: Max current in Q axis
    float32_t  IqKf;            // Parameter: Gain of Iq error current
    float32_t  thetaMax;        // Parameter: maximum theta
} SPD_OBSERVER;

/*-----------------------------------------------------------------------------
Default initialization values for the SPD_OBSERVER objects
-----------------------------------------------------------------------------*/
#define SPD_OBSERVER_DEFAULTS {             \
        0.0f,       /*  Ref            */   \
        0.0f,       /*  Fbk            */   \
        0.0f,       /*  Err            */   \
        0.0f,       /*  Out            */   \
        0.0f,       /*  Kp             */   \
        0.0f,       /*  Ki             */   \
        0.0f,       /*  KiT            */   \
        1.0f,       /*  Umax           */   \
       -1.0f,       /*  Umin           */   \
        0.0f,       /*  up             */   \
        0.0f,       /*  ui             */   \
        0.0f,       /*  IqMax          */   \
        0.1f,       /*  IqKf           */   \
        0.1f        /*  thetaMax       */   \
}

// ***************************************
// extern functions
// ***************************************
static inline float32_t SPD_OBSERVER_run(SPD_OBSERVER * obs,
                                         float32_t theta, float32_t IqErr,
                                         float32_t Ts, float32_t thetaMax)
{
    float32_t  IqErrFF = __fsat(IqErr, obs->IqMax, -obs->IqMax) * obs->IqKf;

    obs->Ref  = theta;

    // error cal
    obs->Err = obs->Ref - obs->Fbk + IqErrFF;

    // roll in the error
    obs->Err = (obs->Err >  0.5f) ? (obs->Err - 1.0f) :
               (obs->Err < -0.5f) ? (obs->Err + 1.0f) : obs->Err;

    // P and I control
    obs->up  = obs->Kp * obs->Err;             // P control
    obs->ui += obs->Ki * obs->Err * Ts;        // I control
    obs->ui  = __fsat(obs->ui, obs->Umax, obs->Umin);

    // control output
    obs->Out = obs->up + obs->ui;
    obs->Out = __fsat(obs->Out, obs->Umax, obs->Umin);

    // Latest angle feedback estimation --> ( Fbk = integral of speed )
    obs->Fbk += obs->Out * thetaMax;

    // roll "Fbk" within -pi to pi
    obs->Fbk = (obs->Fbk >  1.0f) ? (obs->Fbk - 1.0f) :
               (obs->Fbk <  0.0f) ? (obs->Fbk + 1.0f) : obs->Fbk;

    return(obs->Out);
}

// ***************************************
// extern functions
// ***************************************
static inline float32_t runSpeedObserve(SPD_OBSERVER * obs, float32_t theta)
{
    obs->Ref  = theta;

    // error cal
    obs->Err = obs->Ref - obs->Fbk;

    // roll in the error
    obs->Err = (obs->Err >  0.5) ? (obs->Err - 1.0) :
               (obs->Err < -0.5) ? (obs->Err + 1.0) : obs->Err;

    // P and I control
    obs->up  = obs->Kp * obs->Err;             // P control
    obs->ui += obs->Ki * obs->Err;             // I control
    obs->ui  = __fsat(obs->ui, obs->Umax, obs->Umin);

    // control output
    obs->Out = obs->up + obs->ui;
    obs->Out = __fsat(obs->Out, obs->Umax, obs->Umin);

    // Latest angle feedback estimation --> ( Fbk = integral of speed )
    obs->Fbk += obs->Out * obs->thetaMax;

    // roll "Fbk" within -pi to pi
    obs->Fbk = (obs->Fbk >  1.0) ? (obs->Fbk - 1.0) :
               (obs->Fbk <  0.0) ? (obs->Fbk + 1.0) : obs->Fbk;

    return(obs->Out);
}

#endif // end of SPD_OBSERVER_H definition
