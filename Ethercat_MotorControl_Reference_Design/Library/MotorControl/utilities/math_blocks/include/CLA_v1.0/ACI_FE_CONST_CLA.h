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

#ifndef __ACI_FE_CONST_CLA_H__
#define __ACI_FE_CONST_CLA_H__

typedef struct 	{ float32  Rs; 				// Input: Stator resistance (ohm) 
				  float32  Rr;				// Input: Rotor resistance (ohm) 
			      float32  Ls;				// Input: Stator inductance (H) 	  			      
				  float32  Lr;				// Input: Rotor inductance (H) 			
				  float32  Lm;				// Input: Magnetizing inductance (H)
				  float32  Ib; 				// Input: Base phase current (amp) 
				  float32  Vb;				// Input: Base phase voltage (volt) 
				  float32  Ts;				// Input: Sampling period in sec 
				  float32  Tr;				// Parameter: Rotor time constant 
			      float32  K1;				// Output: constant using in rotor flux calculation  
			      float32  K2;				// Output: constant using in rotor flux calculation  
			      float32  K3;				// Output: constant using in rotor flux calculation 
			      float32  K4;				// Output: constant using in stator current calculation  
			      float32  K5;				// Output: constant using in stator current calculation  
			      float32  K6;				// Output: constant using in stator current calculation  
			      float32  K7;				// Output: constant using in stator current calculation  
			      float32  K8;				// Output: constant using in torque calculation  
				} ACI_FE_CONST_CLA;

/*------------------------------------------------------------------------------
	ACIFE_CONST MACRO Definition
------------------------------------------------------------------------------*/


#define ACI_FE_CONST_CLA_MACRO(v)				\
												\
/* Rotor time constant (sec)*/					\
   v.Tr = v.Lr/v.Rr;							\
   												\
   v.K1 = v.Tr/(v.Tr+v.Ts);						\
   v.K2 = v.Ts/(v.Tr+v.Ts);						\
   v.K3 = v.Lm/v.Lr;							\
   v.K4 = (v.Ls*v.Lr-v.Lm*v.Lm)/(v.Lr*v.Lm);	\
   v.K5 = v.Ib*v.Rs/v.Vb;						\
   v.K6 = v.Vb*v.Ts/(v.Lm*v.Ib);				\
   v.K7 = v.Lr/v.Lm;							\
   v.K8 = (v.Ls*v.Lr-v.Lm*v.Lm)/(v.Lm*v.Lm);

#define ACI_FE_CONST_INIT_MACRO(v)\
	  v.Rs=0;             \
	  v.Rr=0;             \
	  v.Ls=0;             \
	  v.Lr=0;             \
	  v.Lm=0;             \
	  v.Ib=0;             \
	  v.Vb=0;             \
	  v.Ts=0;             \
	  v.Tr=0;             \
	  v.K1=0;             \
	  v.K2=0;             \
	  v.K3=0;             \
	  v.K4=0;             \
	  v.K5=0;             \
	  v.K6=0;             \
	  v.K7=0;             \
	  v.K8=0;             \
	  

#endif





