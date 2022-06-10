//#############################################################################
//
// FILE:   dclink_ss.h
//
// TITLE:  C28x DC-Link Single-Shunt Current Reconstruction library
//         (floating point)
//
//#############################################################################
// $TI Release: MotorControl SDK v4.00.00.00 $
// $Release Date: Thu Feb 17 18:05:18 CST 2022 $
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

#ifndef DCLINK_SS_H
#define DCLINK_SS_H

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
//! \defgroup DCLINK_SS DCLINK_SS
//! @{
//
//*****************************************************************************

#include "types.h"
#include "libraries/math/include/math.h"

//*****************************************************************************
//
//! \brief Defines unsigned integer three element vector
//
//*****************************************************************************
typedef struct _MATH_ui_Vec3_
{
    uint16_t value[3];
} MATH_ui_Vec3;

typedef MATH_ui_Vec3 MATH_ui_vec3;

//*****************************************************************************
//
//! \brief Defines unsigned integer two element vector
//
//*****************************************************************************
typedef struct _MATH_ui_Vec2_
{
    uint16_t value[2];
} MATH_ui_Vec2;

typedef MATH_ui_Vec2 MATH_ui_vec2;

//*****************************************************************************
//
//! \brief Defines the DC_Link Single-Shunt Object
//
//*****************************************************************************
typedef struct _DCLINK_SS_Obj_
{
    uint16_t minAvDuration;     //!< the minimum active voltage duration
    uint16_t sampleDelay;       //!< the sampling delay time(pwm tick)
    uint16_t sector;            //!< the sector value of present PWM cycle
    uint16_t sector_1;          //!< the sector value of last PWM cycle
    uint16_t pwmPeriod;         //!< PWM period value in PRD register

    float32_t SSTOffThrVs_pu;   //!< the Vs threshold value(PU) for sequence control off

    uint16_t flag_SST;          //!< the sector toggle flag for sequence control
                                //!< 0: formal half=compensation vector, latter half=measurement vector
                                //!< 1: formal half=measurement vector, latter half=compensation vector
    uint16_t flag_SST_1;        //!< the sector toggle flag for sequence control (one cycle delay)

    uint16_t vecArea;       //!< the output vector area
                            //!< 0: the area all currents are measurable,
                            //!< 1: Bar area that only one current is measurable
                            //!< 2: Star area that all currents are unmeasurable (low modulation area)
    uint16_t vecArea_1;     //!< the vector area (one cycle delay)

    bool flagEnableFullSample;      //!< the enable flag for full sampling
    bool flagEnableSequenceControl; //!< the enable flag for sequence control
    bool flagRunInHighModulation;   //!< the flag for checking if motor is running in high modulation

    MATH_vec3 I_A;              //!< the reconstructed three-phase current

 } DCLINK_SS_Obj;

//*****************************************************************************
//
//! \brief Defines the DCLINK Single-Shunt handle
//
//*****************************************************************************

typedef struct _DCLINK_SS_Obj_ *DCLINK_SS_Handle;

// **************************************************************************
// the function prototypes

//*****************************************************************************
//
//! \brief     Initializes the DC-Link Single-Shunt object
//
//! \param[in] *pMemory         Pointer in to the DC-Link Single-Shunt object
//
//! \param[in] numBytes         Size of the object
//
//! \return    The DC-Link Single-Shunt(DCLINK_SS) object handle
//
//*****************************************************************************
extern DCLINK_SS_Handle
DCLINK_SS_init(void *pMemory,const size_t numBytes);

//*****************************************************************************
//
//! \brief     Sets the initial conditions for single-shunt variables
//
//! \param[in] handle    The DC-Link Single-Shunt(DCLINK_SS) handle
//
//! \param[in] pwmPeriod The PWM period value
//
//! \param[in] offset    The dc-link offset voltage
//
//! \param[in] SSTOffThrDuty  The threshold output duty for sequence control off
//
//! \return    None
//
//*****************************************************************************
extern void
DCLINK_SS_setInitialConditions(DCLINK_SS_Handle handle,
                               const uint16_t pwmPeriod,
                               const float32_t SSTOffThrVs_pu);

//*****************************************************************************
//
//! \brief     Run the three-phase current reconstruction
//             with measured dc-link current
//
//! \param[in] handle    The DC-Link Single-Shunt(DCLINK_SS) handle
//
//! \param[in] pIdc1     A pointer to the currents measured at up count
//
//! \param[in] pIdc2     A pointer to the currents measured at down count
//
//! \return    None
//
//*****************************************************************************
static inline void
DCLINK_SS_runCurrentReconstruction(DCLINK_SS_Handle handle,
                                const MATH_vec2 *pIdc1, const MATH_vec2 *pIdc2)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;
    MATH_vec2 Idc;

    if(obj->flagEnableFullSample)
    {
        if(obj->vecArea_1 == 0)
        {
            Idc.value[0] = (pIdc1->value[1] + pIdc2->value[0])*0.5f;
            Idc.value[1] = (pIdc1->value[0] + pIdc2->value[1])*0.5f;
        }
        else // star area
        {
            if(obj->flag_SST_1 == 0) //measurement vector is on down count
            {
                Idc.value[0] = pIdc2->value[0];
                Idc.value[1] = pIdc2->value[1];
            }
            else //measurement vector is on up count
            {
                Idc.value[0] = pIdc1->value[1];
                Idc.value[1] = pIdc1->value[0];
            }
        }
    }
    else
    {
        if(obj->flag_SST_1 == 0) //measurement vector is on down count
        {
            Idc.value[0] = pIdc2->value[0];
            Idc.value[1] = pIdc2->value[1];
        }
        else //measurement vector is on up count
        {
            Idc.value[0] = pIdc1->value[1];
            Idc.value[1] = pIdc1->value[0];
        }
    }

    //
    //Sector    1st_Sample       2nd_Sample   Actual_Sector
    //  1       1,1,0 (-Ic)      0,1,0 (+Ib)      2
    //  2       1,0,1 (-Ib)      1,0,0 (+Ia)      6
    //  3       1,1,0 (-Ic)      1,0,0 (+Ia)      1
    //  4       0,1,1 (-Ia)      0,0,1 (+Ic)      4
    //  5       0,1,1 (-Ia)      0,1,0 (+Ib)      3
    //  6       1,0,1 (-Ib)      0,0,1 (+Ic)      5
    //
    switch(obj->sector_1)
    {
        case 1:
            obj->I_A.value[2] = -Idc.value[0];
            obj->I_A.value[1] = Idc.value[1];
            obj->I_A.value[0] = -obj->I_A.value[1] - obj->I_A.value[2];
            break;
        case 2:
            obj->I_A.value[1] = -Idc.value[0];
            obj->I_A.value[0] = Idc.value[1];
            obj->I_A.value[2] = -obj->I_A.value[0] - obj->I_A.value[1];
            break;
        case 3:
            obj->I_A.value[2] = -Idc.value[0];
            obj->I_A.value[0] = Idc.value[1];
            obj->I_A.value[1] = -obj->I_A.value[0] - obj->I_A.value[2];
            break;
        case 4:
            obj->I_A.value[0] = -Idc.value[0];
            obj->I_A.value[2] = Idc.value[1];
            obj->I_A.value[1] = -obj->I_A.value[0] - obj->I_A.value[2];
            break;
        case 5:
            obj->I_A.value[0] = -Idc.value[0];
            obj->I_A.value[1] = Idc.value[1];
            obj->I_A.value[2] = -obj->I_A.value[0] - obj->I_A.value[1];
            break;
        case 6:
            obj->I_A.value[1] = -Idc.value[0];
            obj->I_A.value[2] = Idc.value[1];
            obj->I_A.value[0] = -obj->I_A.value[1] - obj->I_A.value[2];
            break;
        default:
            break;
    }

    return;
} // end of DCLINK_SS_runCurrentReconstruction() function


//*****************************************************************************
//
//! \brief     Run the PWM phase shift compensation
//
//! \param[in] handle    The DC-Link Single-Shunt(DCLINK_SS) handle
//! \param[in] pVab_out  The pointer to the Vab reference voltage
//! \param[in] Vdc_V     The DC_bus voltage
//! \param[in] pPwmCMPA  The pointer to the PWM compare-A values
//! \param[in] pPwmCMPB  The pointer to the PWM compare-B values
//! \param[in] pUpSoc    The pointer to the up counter SOC trig values
//! \param[in] pDownSoc  The pointer to the down conter SOC trig values
//
//! \return    None
//
//*****************************************************************************
static inline void
DCLINK_SS_runPWMCompensation(DCLINK_SS_Handle handle,
                          const MATH_vec2 *pVab_out, const float32_t Vdc_V,
                          MATH_ui_vec3 *pPwmCMPA, MATH_ui_vec3 *pPwmCMPB,
                          MATH_ui_vec2 *pUpSoc, MATH_ui_vec2 *pDownSoc)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;
    uint16_t cmp[3];
    uint16_t swap;
    uint16_t sector = 0;
    MATH_ui_vec3 newCMPA;
    MATH_ui_vec3 newCMPB;
    int16_t dT1,dT2;
    int16_t half_dT1, half_dT2;
    uint16_t pwmPRD = obj->pwmPeriod;
    uint16_t index0 = 0;
    uint16_t index1 = 1;
    uint16_t index2 = 2;

    //
    // determine the sector of space vector modulation
    //
    float32_t Va_tmp = pVab_out->value[1] * 0.5f;
    float32_t Vb_tmp = MATH_SQRTTHREE_OVER_TWO * pVab_out->value[0];

    // Inverse clarke transformation
    float32_t Va = pVab_out->value[1];
    float32_t Vb = -Va_tmp + Vb_tmp;
    float32_t Vc = -Va_tmp - Vb_tmp;

    if (Va > 0.0f) sector = 1;
    if (Vb > 0.0f) sector = sector + 2;
    if (Vc > 0.0f) sector = sector + 4;

    obj->sector_1 = obj->sector;
    obj->sector = sector;

    //
    // get EPWMx CMPA values
    //
    cmp[0] = pPwmCMPA->value[0];
    cmp[1] = pPwmCMPA->value[1];
    cmp[2] = pPwmCMPA->value[2];

    //
    // find min, mid, max value of PWMx CMPA
    // cmp[0]=min(max_duty), cmp[1]=mid, cmp[2]=max(min_duty)
    //
    if(cmp[0] > cmp[1])
    {
        swap = cmp[0];
        cmp[0] = cmp[1];
        cmp[1] = swap;

        index0 = 1;
        index1 = 0;
    }

    if(cmp[1] > cmp[2])
    {
        swap = cmp[1];
        cmp[1] = cmp[2];
        cmp[2] = swap;

        index2 = index1;
        index1 = 2;
    }

    if(cmp[0] > cmp[1])
    {
        swap = cmp[0];
        cmp[0] = cmp[1];
        cmp[1] = swap;

        swap = index1;
        index1 = index0;
        index0 = swap;
    }

    //
    // set default newCMPA/B value with new arranged compare value
    //
    newCMPA.value[0] = newCMPB.value[0] = cmp[0];
    newCMPA.value[1] = newCMPB.value[1] = cmp[1];
    newCMPA.value[2] = newCMPB.value[2] = cmp[2];

    obj->flag_SST_1 = obj->flag_SST;

    //
    // sequence control for smooth injection at the sector transition
    //
    if(obj->flagEnableSequenceControl)
    {

        float32_t Vs = sqrtf((pVab_out->value[0]*pVab_out->value[0]) +
                             (pVab_out->value[1]*pVab_out->value[1]));
        float32_t highThr_V = obj->SSTOffThrVs_pu*Vdc_V;
        float32_t lowThr_V = (obj->SSTOffThrVs_pu-0.05f)*Vdc_V;

        //
        // check if sequence control should be disabled based on output modulation value,
        // because ADC should be sampled at down count only in case of high duty
        //
        if(Vs > highThr_V)
        {
            // if the output voltage is higher than threshold, stop sequence control
            obj->flagRunInHighModulation = true;
        }
        else if(Vs < lowThr_V)
        {
            // if the output voltage is higher than threshold, stop sequence control
            obj->flagRunInHighModulation = false;
        }

        if(obj->flagRunInHighModulation == false)
        {
            if((sector == 1) || (sector == 4) || (sector == 2)) //actual sector=2,4,6
            {
                // formal half = measurement vector, latter half = compensation vector
                obj->flag_SST = 1;
            }
            else
            {
                // formal half = compensation vector, latter half = measurement vector
                obj->flag_SST = 0;
            }
        }
        else
        {
            // formal half = compensation vector, latter half = measurement vector
            obj->flag_SST = 0;
        }
    }
    else
    {
        // formal half = compensation vector, latter half = measurement vector
        obj->flag_SST = 0;
    }

    //
    // PWM Phase Shift Compensation with Minimum Voltage Injection
    //
    dT1 = obj->minAvDuration - (cmp[2] - cmp[1]);
    dT2 = obj->minAvDuration - (cmp[1] - cmp[0]);

    obj->vecArea_1 = obj->vecArea;

    if(obj->flag_SST == 0) // inject measurable vector at DOWN counter
    {
        if(dT1 > 0) // the 1st active vector is unmeasurable area
        {
            if(dT2 > 0) // the 2nd active vector is also unmeasurable area (low modulation)
            {
                obj->vecArea = 2;

                // cmp[2](min duty) is shifted to the left
                newCMPA.value[2] = cmp[2] - dT1;
                newCMPB.value[2] = cmp[2] + dT1;

                // cmp[0](max duty) is shifted to the right
                newCMPA.value[0] = cmp[0] + dT2;
                newCMPB.value[0] = cmp[0] - dT2;
            }
            else // in case of 6-vector(bar-area), only 1st active vector is unmeasurable
            {
                obj->vecArea = 1;

                half_dT1 = dT1>>1;

                // cmp[2](min duty) is shifted to the left
                newCMPB.value[2] = __min(cmp[2] + half_dT1, pwmPRD);
                newCMPA.value[2] = cmp[2] - half_dT1;

                // cmp[1](mid duty) is shifted to the right
                newCMPB.value[1] = newCMPB.value[2] - obj->minAvDuration;
                newCMPA.value[1] = __min(cmp[1]*2 - newCMPB.value[1], pwmPRD);

                int16_t new_dT2 = obj->minAvDuration - (newCMPB.value[1] - cmp[0]);
                if(new_dT2 > 0)
                {
                    // cmp[0](max duty) is shifted to the right
                    newCMPB.value[0] = cmp[0] - new_dT2;
                    newCMPA.value[0] = cmp[0] + new_dT2;
                }
            } // end of if(dT2 > 0)
        } // end of if(dT1 > 0)
        else // the 1st active vector is measurable
        {
            if(dT2 > 0) // only the 2nd active vector is unmeasurable area(bar-area)
            {
                obj->vecArea = 1;

                half_dT2 = dT2>>1;

                // cmp[0](max duty) is shifted to the right
                newCMPB.value[0] = __max(cmp[0] - half_dT2, 0);
                newCMPA.value[0] = cmp[0] + half_dT2;

                // cmp[1](mid duty) is shifted to the left
                newCMPB.value[1] = newCMPB.value[0] + obj->minAvDuration;
                newCMPA.value[1] = __max(cmp[1]*2 - newCMPB.value[1], 0);

                int16_t new_dT1 = obj->minAvDuration - (cmp[2] - newCMPB.value[1]);
                if(new_dT1 > 0)
                {
                    // cmp[2](min duty) is shifted to the right
                    newCMPB.value[2] = cmp[2] + new_dT1;
                    newCMPA.value[2] = cmp[2] - new_dT1;
                }
            }
            else // all active vectors are measurable
            {
                obj->vecArea = 0;
            }
        }
    }
    else // inject measurable vector at UP counter (flag_SST = 1)
    {
        if(dT1 > 0) // the 2nd active vector is unmeasurable area
        {
            if(dT2 > 0) // the 1st active vector is also unmeasurable area (low modulation)
            {
                obj->vecArea = 2;

                // cmp[2](min duty) is shifted to the right
                newCMPA.value[2] = cmp[2] + dT1;
                newCMPB.value[2] = cmp[2] - dT1;

                // cmp[0](max duty) is shifted to the left
                newCMPA.value[0] = cmp[0] - dT2;
                newCMPB.value[0] = cmp[0] + dT2;
            }
            else // in case of 6-vector(bar-area), only 2nd active vector is unmeasurable
            {
                obj->vecArea = 1;

                half_dT1 = dT1>>1;

                // cmp[2](min duty) is shifted to the right
                newCMPA.value[2] = __min(cmp[2] + half_dT1, pwmPRD);
                newCMPB.value[2] = cmp[2] - half_dT1;

                // cmp[1](mid duty) is shifted to the left
                newCMPA.value[1] =  newCMPA.value[2] - obj->minAvDuration;
                newCMPB.value[1] = __min(cmp[1]*2 - newCMPA.value[1], pwmPRD);

                int16_t new_dT2 = obj->minAvDuration - (newCMPA.value[1] - cmp[0]);
                if(new_dT2 > 0)
                {
                    // cmp[0](max duty) is shifted to the left
                    newCMPA.value[0] = cmp[0] - new_dT2;
                    newCMPB.value[0] = cmp[0] + new_dT2;
                }
            } // end of if(dT2 > 0)
        } // end of if(dT1 > 0)
        else // the 2nd active vector is measurable
        {
            if(dT2 > 0) // only the 1st active vector is unmeasurable area(bar-area)
            {
                obj->vecArea = 1;

                half_dT2 = dT2>>1;

                // cmp[0](max duty) is shifted to the left
                newCMPA.value[0] = __max(cmp[0] - half_dT2, 0);
                newCMPB.value[0] = cmp[0] + half_dT2;

                // cmp[1](mid duty) is shifted to the right
                newCMPA.value[1] = newCMPA.value[0] + obj->minAvDuration;
                newCMPB.value[1] = __max(cmp[1]*2 - newCMPA.value[1], 0);

                int16_t new_dT1 = obj->minAvDuration - (cmp[2] - newCMPA.value[1]);
                if(new_dT1 > 0)
                {
                    // cmp[2](min duty) is shifted to the right
                    newCMPA.value[2] = cmp[2] + new_dT1;
                    newCMPB.value[2] = cmp[2] - new_dT1;
                }
            } // end of if(dT2 > 0)
            else // all active vectors are measurable
            {
                obj->vecArea = 0;
            }
        }
    }

    //
    // recalculate ADC SOC trigger points
    //
    pDownSoc->value[0] = __min((newCMPB.value[2] + newCMPB.value[1])>>1,
                               (newCMPB.value[2] - obj->sampleDelay));
    pDownSoc->value[1] = __min((newCMPB.value[1] + newCMPB.value[0])>>1,
                               (newCMPB.value[1] - obj->sampleDelay));

    pUpSoc->value[0] = __max((newCMPA.value[0] + newCMPA.value[1])>>1,
                               (newCMPA.value[0] + obj->sampleDelay));
    pUpSoc->value[1] = __max((newCMPA.value[2] + newCMPA.value[1])>>1,
                               (newCMPA.value[1] + obj->sampleDelay));

    //
    // revise CMPA/B with new compare values
    //
    pPwmCMPA->value[index0] = newCMPA.value[0];
    pPwmCMPB->value[index0] = newCMPB.value[0];
    pPwmCMPA->value[index1] = newCMPA.value[1];
    pPwmCMPB->value[index1] = newCMPB.value[1];
    pPwmCMPA->value[index2] = newCMPA.value[2];
    pPwmCMPB->value[index2] = newCMPB.value[2];

    return;
} // end of DCLINK_SS_runPWMCompensation() function


//*****************************************************************************
//
//! \brief     Gets the reconstructed phase-A current
//
//! \param[in] handle    The DC-Link Single-Shunt(DCLINK_SS) handle
//
//! \return    Phase-A current, A
//
//*****************************************************************************
static inline float32_t
DCLINK_SS_getIa(DCLINK_SS_Handle handle)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;

    return obj->I_A.value[0];
} // end of DCLINK_SS_getIa() function

//*****************************************************************************
//
//! \brief     Gets the reconstructed phase-B current
//
//! \param[in] handle    The DC-Link Single-Shunt(DCLINK_SS) handle
//
//! \return    Phase-B current, A
//
//*****************************************************************************
static inline float32_t
DCLINK_SS_getIb(DCLINK_SS_Handle handle)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;

    return obj->I_A.value[1];
} // end of DCLINK_SS_getIb() function

//*****************************************************************************
//
//! \brief     Gets the reconstructed phase-A current
//
//! \param[in] handle    The DC-Link Single-Shunt(DCLINK_SS) handle
//
//! \return    Phase-C current, A
//
//*****************************************************************************
static inline float32_t
DCLINK_SS_getIc(DCLINK_SS_Handle handle)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;

    return obj->I_A.value[2];
} // end of DCLINK_SS_getIc() function

//*****************************************************************************
//
//! \brief     Sets the initial conditions for single-shunt variables
//
//! \param[in] handle   The DC-Link Single-Shunt(DCLINK_SS) handle
//!
//! \param[in] minTime   The minimum active duration for dc link current measurement
//
//! \return    None
//
//*****************************************************************************
static inline void
DCLINK_SS_setMinAVDuration(DCLINK_SS_Handle handle, const uint16_t minTime)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;

    obj->minAvDuration = minTime;

    return;
} // end of DCLINK_SS_setMinAVDuration() function

//*****************************************************************************
//
//! \brief     Sets the SOC trigger delay for ADC sampling
//!
//! \param[in] handle  The DC-Link Single-Shunt (DCLINK_SS) handle
//!
//! \param[in] sampleDelay  The SOC trigger delay value
//!
//! \return    None
//
//*****************************************************************************
static inline void
DCLINK_SS_setSampleDelay(DCLINK_SS_Handle handle, const uint16_t sampleDelay)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;

    obj->sampleDelay = sampleDelay;

    return;
} // end of DCLINK_SS_setSampleDelay() function

//*****************************************************************************
//
//! \brief     Sets the flag of full sampling
//
//! \param[in] handle    The DC-Link Single-Shunt(DCLINK_SS) handle
//
//! \param[in] state    The desired flag state, on (1) or off (0)
//
//! \return    None
//
//*****************************************************************************
static inline void
DCLINK_SS_setFlag_enableFullSampling(DCLINK_SS_Handle handle, const bool state)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;

    obj->flagEnableFullSample = state;

    return;
} // end of DCLINK_SS_setFlag_enableFullSampling() function

//*****************************************************************************
//
//! \brief     Sets the flag of sequence control
//
//! \param[in] handle    The DC-Link Single-Shunt(DCLINK_SS) handle
//
//! \param[in] state    The desired flag state, on (1) or off (0)
//
//! \return    None
//
//*****************************************************************************
static inline void
DCLINK_SS_setFlag_enableSequenceControl(DCLINK_SS_Handle handle, const bool state)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;

    obj->flagEnableSequenceControl = state;

    return;
} // end of DCLINK_SS_setFlag_enableSequenceControl() function

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

#endif // end of DCLINK_SS_H defines
