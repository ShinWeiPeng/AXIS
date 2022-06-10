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
File name:       ACI_SE_CONST.H                     
===================================================================================*/
#ifndef __ACI_SE_CONST_H__
#define __ACI_SE_CONST_H__

typedef struct 	{ float32  Rr;				// Input: Rotor resistance (ohm) 
				  float32  Lr;				// Input: Rotor inductance (H) 	
				  float32  Tr;				// Variable: Rotor time constant 			
				  float32  fb;              // Input: Base electrical frequency (Hz)
				  float32  Wb;              // Variable: Base angular speed (rad/s)
                  float32  fc;              // Input: Cut-off frequency of lowpass filter (Hz)
                  float32  Tc;              // Variable: Time constant (sec)
				  float32  Ts;				// Input: Sampling period in sec 
			      float32  K1;				// Output: constant using in rotor flux calculation 
			      float32  K2;				// Output: constant using in rotor flux calculation 
			      float32  K3;				// Output: constant using in rotor flux calculation 
			      float32  K4;				// Output: constant using in stator current calculation 
				} ACISE_CONST;

/*-----------------------------------------------------------------------------
	Default initalizer for the ACISE_CONST object.
-----------------------------------------------------------------------------*/                     
#define ACISE_CONST_DEFAULTS {0,0,0,0,	\
                              0,0,0,0, 	\
 		          	          0,0,0,0, 	\
                             }

/*------------------------------------------------------------------------------
	ACI_SE_CONST macro definition  
------------------------------------------------------------------------------*/


#define PI 3.14159265358979  

#define ACISE_CONST_MACRO(v)				\
											\
/* Rotor time constant (sec) */				\
	v.Tr = v.Lr/v.Rr;						\
											\
/* Lowpass filter time constant (sec) */	\
	v.Tc = 1/(2*PI*v.fc);					\
											\
	v.Wb = 2*PI*v.fb;						\
	v.K1 = 1/(v.Wb*v.Tr);					\
	v.K2 = 1/(v.fb*v.Ts);					\
	v.K3 = v.Tc/(v.Tc+v.Ts);				\
	v.K4 = v.Ts/(v.Tc+v.Ts); 

	
#endif

