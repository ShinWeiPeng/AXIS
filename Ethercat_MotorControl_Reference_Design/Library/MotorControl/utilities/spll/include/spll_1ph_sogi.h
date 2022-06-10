//#############################################################################
//
//  FILE:   spll_1ph_sogi.h
//
//  TITLE:  Orthogonal Signal Generator Software Phase Lock Loop (SPLL) 
//          for Single Phase Grid Module
//
//#############################################################################
// $TI Release: MotorControl SDK v4.00.00.00 $
// $Release Date: Thu Feb 17 18:05:20 CST 2022 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef SPLL_1PH_SOGI_H
#define SPLL_1PH_SOGI_H

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \defgroup SPLL_1PH_SOGI SPLL_1PH_SOGI
//! @{
//
//*****************************************************************************

//
// Included Files
//
#include <stdint.h>
#ifndef __TMS320C28XX_CLA__
#include <math.h>
#else
#include <CLAmath.h>
#endif

#include "libraries/math/include/math.h"

//
// Typedefs
//

//! \brief  Defines the SPLL_1PH_SOGI_OSG_COEFF structure
//!
typedef struct{
    float32_t osg_k;
    float32_t osg_x;
    float32_t osg_y;
    float32_t osg_b0;
    float32_t osg_b2;
    float32_t osg_a1;
    float32_t osg_a2;
    float32_t osg_qb0;
    float32_t osg_qb1;
    float32_t osg_qb2;
} SPLL_1PH_SOGI_OSG_COEFF;

//! \brief  Defines the SPLL_1PH_SOGI_LPF_COEFF structure
//!
typedef struct{
    float32_t b1;
    float32_t b0;
} SPLL_1PH_SOGI_LPF_COEFF;

//! \brief Defines the Orthogonal Signal Generator SPLL_1PH_SOGI
//!        structure
//!
//! \details The SPLL_1PH_SOGI can be used to generate the
//!          orthogonal signal from the sensed single phase grid voltage
//!          and use that information to provide phase of the grid voltage
//!
typedef struct{
    float32_t   u[3];       //!< AC input data buffer
    float32_t   osg_u[3];   //!< Orthogonal signal generator data buffer
    float32_t   osg_qu[3];  //!< Orthogonal signal generator quadrature data buffer
    float32_t   u_Q[2];     //!< Q-axis component
    float32_t   u_D[2];     //!< D-axis component
    float32_t   ylf[2];     //!< Loop filter data storage
    float32_t   fo;         //!< Output frequency of PLL(Hz)
    float32_t   fn;         //!< Nominal frequency (Hz)
    float32_t   theta;      //!< Angle output (0-2*pi)
    float32_t   cosine;     //!< Cosine value of the PLL angle
    float32_t   sine;       //!< Sine value of the PLL angle
    float32_t   delta_t;    //!< Inverse of the ISR rate at which module is called
    SPLL_1PH_SOGI_OSG_COEFF osg_coeff; //!< Orthogonal signal generator coefficient
    SPLL_1PH_SOGI_LPF_COEFF lpf_coeff; //!< Loop filter coeffcient structure
} SPLL_1PH_SOGI;

//! \brief Resets internal storage data of the module
//! \param *spll_obj The SPLL_1PH_SOGI structure pointer
//! \return None
//!
static inline void SPLL_1PH_SOGI_reset(SPLL_1PH_SOGI *spll_obj)
{
    spll_obj->u[0]=(float32_t)(0.0f);
    spll_obj->u[1]=(float32_t)(0.0f);
    spll_obj->u[2]=(float32_t)(0.0f);
    
    spll_obj->osg_u[0]=(float32_t)(0.0f);
    spll_obj->osg_u[1]=(float32_t)(0.0f);
    spll_obj->osg_u[2]=(float32_t)(0.0f);
    
    spll_obj->osg_qu[0]=(float32_t)(0.0f);
    spll_obj->osg_qu[1]=(float32_t)(0.0f);
    spll_obj->osg_qu[2]=(float32_t)(0.0f);
    
    spll_obj->u_Q[0]=(float32_t)(0.0f);
    spll_obj->u_Q[1]=(float32_t)(0.0f);
    
    spll_obj->u_D[0]=(float32_t)(0.0f);
    spll_obj->u_D[1]=(float32_t)(0.0f);
    
    spll_obj->ylf[0]=(float32_t)(0.0f);
    spll_obj->ylf[1]=(float32_t)(0.0f);
    
    spll_obj->fo=(float32_t)(0.0f);
    
    spll_obj->theta=(float32_t)(0.0f);
    
    spll_obj->sine=(float32_t)(0.0f);
    spll_obj->cosine=(float32_t)(0.0f);
}

//! \brief Calculates the SPLL_1PH_SOGI coefficient
//! \param *spll_obj The SPLL_1PH_SOGI structure
//! \return None
//!
static inline void SPLL_1PH_SOGI_coeff_calc(SPLL_1PH_SOGI *spll_obj)
{
    float32_t osgx,osgy,temp, wn;

    wn= spll_obj->fn * 2.0f * MATH_PI;
    spll_obj->osg_coeff.osg_k = 0.5f;
    osgx = 2.0f * 0.5f * wn * spll_obj->delta_t;
    spll_obj->osg_coeff.osg_x= osgx;
    osgy = wn * spll_obj->delta_t * wn * spll_obj->delta_t;
    spll_obj->osg_coeff.osg_y = osgy ;
    temp = 1.0f / (osgx + osgy + 4.0f);
    spll_obj->osg_coeff.osg_b0 = osgx * temp;
    spll_obj->osg_coeff.osg_b2 = -1.0f * spll_obj->osg_coeff.osg_b0;
    spll_obj->osg_coeff.osg_a1 = 2.0f * (4.0f - osgy) * temp;
    spll_obj->osg_coeff.osg_a2 = (osgx - osgy -4.0f ) * temp;
    spll_obj->osg_coeff.osg_qb0 = 0.5f * osgy * temp;
    spll_obj->osg_coeff.osg_qb1 = spll_obj->osg_coeff.osg_qb0 * 2.0f;
    spll_obj->osg_coeff.osg_qb2 = spll_obj->osg_coeff.osg_qb0;
}

//! \brief Configures the SPLL_1PH_SOGI module
//! \param *spll_obj The SPLL_1PH_SOGI structure
//! \param acFreq Nominal AC frequency for the SPLL Module
//! \param isrFrequency Frequency at which SPLL module is run
//! \param lpf_b0 B0 coefficient of LPF of SPLL
//! \param lpf_b1 B1 coefficient of LPF of SPLL
//! \return None
//!
static inline void SPLL_1PH_SOGI_config(SPLL_1PH_SOGI *spll_obj,
                         float32_t acFreq,
                         float32_t isrFrequency,
                         float32_t lpf_b0,
                         float32_t lpf_b1)
{
    spll_obj->fn = acFreq;
    spll_obj->delta_t = 1.0f / isrFrequency;

    SPLL_1PH_SOGI_coeff_calc(spll_obj);

    spll_obj->lpf_coeff.b0 = lpf_b0;
    spll_obj->lpf_coeff.b1 = lpf_b1;
}

//! \brief Run the SPLL_1PH_SOGI module
//! \param *spll_obj The SPLL_1PH_SOGI structure pointer
//! \param acValue AC grid voltage in per unit (pu)
//! \return None
//!
static inline void SPLL_1PH_SOGI_run(SPLL_1PH_SOGI *spll_obj,
                                     float32_t acValue)
{
    // Update the spll_obj->u[0] with the grid value
    spll_obj->u[0]=acValue;

    //
    // Orthogonal Signal Generator
    //
    spll_obj->osg_u[0]=(spll_obj->osg_coeff.osg_b0*
                       (spll_obj->u[0]-spll_obj->u[2])) +
                       (spll_obj->osg_coeff.osg_a1*spll_obj->osg_u[1]) +
                       (spll_obj->osg_coeff.osg_a2*spll_obj->osg_u[2]);

    spll_obj->osg_u[2]=spll_obj->osg_u[1];
    spll_obj->osg_u[1]=spll_obj->osg_u[0];

    spll_obj->osg_qu[0]=(spll_obj->osg_coeff.osg_qb0*spll_obj->u[0]) +
                        (spll_obj->osg_coeff.osg_qb1*spll_obj->u[1]) +
                        (spll_obj->osg_coeff.osg_qb2*spll_obj->u[2]) +
                        (spll_obj->osg_coeff.osg_a1*spll_obj->osg_qu[1]) +
                        (spll_obj->osg_coeff.osg_a2*spll_obj->osg_qu[2]);

    spll_obj->osg_qu[2]=spll_obj->osg_qu[1];
    spll_obj->osg_qu[1]=spll_obj->osg_qu[0];

    spll_obj->u[2]=spll_obj->u[1];
    spll_obj->u[1]=spll_obj->u[0];

    //
    // Park Transform from alpha beta to d-q axis
    //
    spll_obj->u_Q[0]=(spll_obj->cosine*spll_obj->osg_u[0]) +
                     (spll_obj->sine*spll_obj->osg_qu[0]);
    spll_obj->u_D[0]=(spll_obj->cosine*spll_obj->osg_qu[0]) -
                     (spll_obj->sine*spll_obj->osg_u[0]);

    //
    // Loop Filter
    //
    spll_obj->ylf[0]=spll_obj->ylf[1] +
                     (spll_obj->lpf_coeff.b0*spll_obj->u_Q[0]) +
                     (spll_obj->lpf_coeff.b1*spll_obj->u_Q[1]);
    spll_obj->ylf[1]=spll_obj->ylf[0];

    spll_obj->u_Q[1]=spll_obj->u_Q[0];

    //
    // VCO
    //
    spll_obj->fo = spll_obj->fn+spll_obj->ylf[0];

    spll_obj->theta = spll_obj->theta +
            spll_obj->fo * spll_obj->delta_t * MATH_TWO_PI;

    if(spll_obj->theta > MATH_TWO_PI)
    {
        spll_obj->theta=spll_obj->theta - MATH_TWO_PI;
    }


    spll_obj->sine=(float32_t)sinf(spll_obj->theta);
    spll_obj->cosine=(float32_t)cosf(spll_obj->theta);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif // extern "C"


#endif  // end of  _SPLL_1PH_SOGI_H_ definition

//
// End of File
//

