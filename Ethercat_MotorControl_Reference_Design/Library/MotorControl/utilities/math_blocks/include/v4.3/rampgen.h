//#############################################################################
// $TI Release: MotorControl SDK v4.00.00.00 $
// $Release Date: Thu Feb 17 18:05:20 CST 2022 $
// $Copyright:
// Copyright (C) 2017-2022 Texas Instruments Incorporated - http://www.ti.com/
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

/* =================================================================================
File name:        RAMPGEN.H  
===================================================================================*/

#ifndef __RAMPGEN_H__
#define __RAMPGEN_H__

typedef struct { float32_t  Freq; 		  // Input: Ramp frequency (pu)
				 float32_t  StepAngleMax; // Parameter: Maximum step angle (pu)
				 float32_t  Angle;		  // Variable: Step angle (pu)
				 float32_t  Gain;		  // Input: Ramp gain (pu)
				 float32_t  Out;  	 	  // Output: Ramp signal (pu)
				 float32_t  Offset;		  // Input: Ramp offset (pu)
	  	  	   } RAMPGEN;	            

/*------------------------------------------------------------------------------
      Object Initializers
------------------------------------------------------------------------------*/                       
#define RAMPGEN_DEFAULTS { \
        0, /* Freq */ \
        0, /* StepAngleMax */ \
        0, /* Angle */ \
        1, /* Gain */ \
        0, /* Out */ \
        1  /* Offset */ \
    }

/*------------------------------------------------------------------------------
	RAMP(Sawtooh) Generator Macro Definition
------------------------------------------------------------------------------*/                                               

static inline void fclRampGen(RAMPGEN * in)
{
	// Compute the angle rate
	in->Angle += in->StepAngleMax * in->Freq;

	// Saturate the angle rate within (-1,1)
	if(in->Angle > 1.0f)
	{
		in->Angle -= 1.0f;
	}
	else if(in->Angle < -1.0f)
	{
		in->Angle += 1.0f;
	}

	in->Out = in->Angle;
}


#define RG_MACRO(v)									\
													\
	/* Compute the angle rate */					\
	v.Angle += _IQmpy(v.StepAngleMax,v.Freq);		\
													\
    /* Saturate the angle rate within (-1,1) */		\
	if (v.Angle>_IQ(1.0))							\
		v.Angle -= _IQ(1.0);						\
	else if (v.Angle<_IQ(-1.0))						\
		v.Angle += _IQ(1.0);						\
		v.Out=v.Angle;

   /* Use the code snippet below if gain/offset needed.													\
	v.Out = _IQmpy(v.Angle,v.Gain) + v.Offset;		\
	if (v.Out>_IQ(1.0))								\
		v.Out -= _IQ(1.0);							\
	else if (v.Out<_IQ(-1.0))						\
		v.Out += _IQ(1.0);
*/

#define RG_INIT_MACRO(v, fb, Ys)                  \
    v.Angle=0.0;                                  \
    v.Freq=0;                                     \
    v.StepAngleMax=fb*Ts;                         \
    v.Out=0.0;                                    \

#endif // __RAMPGEN_H__
