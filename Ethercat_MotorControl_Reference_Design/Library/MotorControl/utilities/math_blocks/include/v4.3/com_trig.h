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
File name:       COM_TRIG.H  
===================================================================================*/


#include "PeripheralHeaderIncludes.h"

typedef  struct { Uint32 CmtnTrig;       	// Output: Commutation trigger output (0 or 0x00007FFF)       
                  _iq Va;                 	// Input: Motor phase a voltage referenced to GND (pu)  
                  _iq Vb;                 	// Input: Motor phase b voltage referenced to GND (pu)  
                  _iq Vc;                 	// Input: Motor phase c voltage referenced to GND (pu) 
                  _iq Neutral;            	// Variable: 3*Motor netural voltage (pu) 
                  Uint32 RevPeriod;      	// Variable: revolution time counter (Q0)        
                  Uint32 ZcTrig;         	// Variable: Zero-Crossing trig flag (0 or 0x00007FFF)  
                  Uint32 CmtnPointer;     	// Input: Commutation state pointer input (0,1,2,3,4,5)
                  _iq DebugBemf;         	// Variable: 3*Back EMF = 3*(vx=vn), x=a,b,c (pu)
                  Uint32 NoiseWindowCounter;// Variable: Noise windows counter (Q0) 
                  Uint32 Delay30DoneFlag;   // Variable: 30 Deg delay flag (0 or 0x0000000F) 
                  Uint32 NewTimeStamp;  	// Variable: Time stamp (Q0) 
                  Uint32 OldTimeStamp;  	// History: Previous time stamp (Q0) 
	              Uint32 VirtualTimer;    	// Input: Virtual timer (Q0) 
                  Uint32 CmtnDelay;      	// Variable: Time delay in terms of number of sampling time periods (Q0)    
                  Uint32 DelayTaskPointer; 	// Variable: Delay task pointer, see note below (0 or 1)
                  Uint32 NoiseWindowMax;  	// Variable: Maximum noise windows counter (Q0)
                  Uint32 CmtnDelayCounter; 	// Variable: Time delay counter (Q0) 
                  Uint32 NWDelta;      		// Variable: Noise windows delta (Q0)
                  Uint32 NWDelayThres;    	// Variable: Noise windows dynamic threshold (Q0)
		 	 	   int32 GPR1_COM_TRIG;		// Variable: Division reminder
		 	 	   int32 Tmp;				// Variable: Temp. variable
                } CMTN;

/*
Note: 
DelayTaskPointer = 0, branch for #COUNT_DWN
DelayTaskPointer = 1, branch for #CHK_TRIGGER
*/

/*-----------------------------------------------------------------------------
Default initalizer for the CMTN object.
-----------------------------------------------------------------------------*/                     
#define CMTN_DEFAULTS { 0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        1, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
              		   }

/*----------------------------------------------------------------------------------------------
	 CMTN_TRIG Macro Definition
----------------------------------------------------------------------------------------------*/

#define CMTN_TRIG_MACRO(v)																\
																						\
/* Always clear flags on entry*/														\
    v.CmtnTrig = 0;																		\
    v.ZcTrig = 0;																		\
        																				\
/* Neutral voltage calculation (3*motor Neutral voltage)*/								\
	v.Neutral = v.Va + v.Vb + v.Vc;														\
																						\
/* Commutation State table Tasks*/														\
/* State s1: current flows to motor windings from phase A->B, de-energized phase = C*/	\
   if (v.CmtnPointer == 0)																\
    {																					\
	  v.DebugBemf = _IQmpy(_IQ(3),v.Vc) - v.Neutral;									\
	  if (v.DebugBemf > 0)																\
	       v.NoiseWindowCounter = 0;													\
	  else   /*  Zero crossing Noise window processing*/								\
           NOISE_WINDOW_CNT_MACRO(v);													\
    }   /* else if-end: State s1*/ 														\
																						\
/* State s2: current flows to motor windings from phase A->C, de-energized phase = B*/ 	\
    else if (v.CmtnPointer == 1)  														\
	{																					\
	  v.DebugBemf = _IQmpy(_IQ(3),v.Vb) - v.Neutral;									\
	  if (v.DebugBemf < 0)																\
	       v.NoiseWindowCounter = 0;													\
	  else   /*  Zero crossing Noise window processing*/								\
           NOISE_WINDOW_CNT_MACRO(v);													\
    }   /* else if-end: State s2*/														\
																						\
/* State s3: current flows to motor windings from phase B->C, de-energized phase = A*/ 	\
    else if (v.CmtnPointer == 2)  														\
    {																					\
	  v.DebugBemf = _IQmpy(_IQ(3),v.Va) - v.Neutral;									\
	  if (v.DebugBemf > 0)																\
	       v.NoiseWindowCounter = 0;													\
	  else  /*  Zero crossing Noise window processing*/									\
           NOISE_WINDOW_CNT_MACRO(v);													\
    }   /* else if-end: State s3*/														\
																						\
/* State s4: current flows to motor windings from phase B->A, de-energized phase = C*/	\
    else if (v.CmtnPointer == 3)  														\
    {																					\
	  v.DebugBemf = _IQmpy(_IQ(3),v.Vc) - v.Neutral;									\
	  if (v.DebugBemf < 0)																\
	       v.NoiseWindowCounter = 0;													\
	  else   /*  Zero crossing Noise window processing*/								\
           NOISE_WINDOW_CNT_MACRO(v);													\
    }   /* else if-end: State s4*/														\
																						\
/* State s5: current flows to motor windings from phase C->A, de-energized phase = B*/	\
    else if (v.CmtnPointer == 4)														\
    {																					\
	  v.Delay30DoneFlag = 0;	       /* clear flag for delay calc in State 5*/		\
	  																					\
	  v.DebugBemf = _IQmpy(_IQ(3),v.Vb) - v.Neutral;									\
	  if (v.DebugBemf > 0)																\
	       v.NoiseWindowCounter = 0;													\
	  else   /*  Zero crossing Noise window processing */								\
           NOISE_WINDOW_CNT_MACRO(v);													\
    }   /* else if-end: State s5	 */													\
																						\
/* State s6: current flows to motor windings from phase C->B, de-energized phase = A*/	\
    else if (v.CmtnPointer == 5)  														\
    {																					\
	  v.DebugBemf = _IQmpy(_IQ(3),v.Va) - v.Neutral;									\
	  if (v.DebugBemf < 0)																\
	       v.NoiseWindowCounter = 0;													\
	  else   /*  Zero crossing Noise window processing*/								\
           NOISE_WINDOW_CNT_MACRO(v);													\
      DELAY_30DEG_MACRO(v);																\
    }   /* else if-end: State s6*/														\
																						\
/* Zero crossing to Commutation trigger delay*/											\
   v.CmtnTrig = 0;     /* Always clear flag on entry */									\
																						\
   if (v.DelayTaskPointer > 0)     /* v.DelayTaskPointer = 1 for #CHK_TRIGGER*/			\
   { 																					\
      if (v.ZcTrig != 0)																\
      {																					\
/* Substract NoiseWindowMax to compensate the advanced zero-crossing validation point */\
          v.CmtnDelayCounter = v.CmtnDelay - v.NoiseWindowMax;							\
          v.DelayTaskPointer = 0;     /* v.DelayTaskPointer = 0 for #COUNT_DWN*/		\
      }																					\
   }																					\
   else     /* v.DelayTaskPointer = 0 for #COUNT_DWN */									\
   {  																					\
       v.CmtnDelayCounter -= 1;															\
       if (v.CmtnDelayCounter == 0) 													\
       {																				\
          v.CmtnTrig = 0x00007FFF; /* Yes!- Set trigger. This is used */				\
/* as an input to "MOD6_CNTR" module that changes the commutation sequence.*/			\
						             													\
          v.DelayTaskPointer = 1;       /* v.DelayTaskPointer = 1 for #CHK_TRIGGER*/	\
       }    																			\
   }

/*----------------------------------------------------------------------------------------------
	 NOISE_WINDOW_CNT Macro Definition
----------------------------------------------------------------------------------------------*/

#define NOISE_WINDOW_CNT_MACRO(v)															\
   if (v.CmtnDelay >= v.NWDelayThres)      /* noise window is fixed Value*/					\
      v.NoiseWindowMax = v.NWDelayThres - v.NWDelta;										\
   else                                       /* noise window adjusted dynamically*/		\
      v.NoiseWindowMax = v.CmtnDelay - v.NWDelta;											\
																							\
   v.NoiseWindowCounter += 1;																\
																							\
   if (v.NoiseWindowCounter == v.NoiseWindowMax)  /* zc must occur max_noise_window times*/	\
   {																						\
     v.ZcTrig = 0x00007FFF;       /* Yes! Set trigger */									\
     v.NoiseWindowCounter = 0;																\
   }								

/*----------------------------------------------------------------------------------------------
	DELAY_30DEG Macro Definition
----------------------------------------------------------------------------------------------*/

#define DELAY_30DEG_MACRO(v)																\
/* Delay 30 deg calculator*/																\
   if (v.Delay30DoneFlag == 0)																\
   {  																						\
      v.OldTimeStamp = v.NewTimeStamp;														\
      v.NewTimeStamp = v.VirtualTimer; 														\
      v.Tmp = v.NewTimeStamp - v.OldTimeStamp; 												\
      																						\
      if (v.Tmp > 0) /* Period = NewTimeStamp - OldTimeStamp*/								\
          v.RevPeriod = v.Tmp;																\
      else       /* If Period is negative, allow "wrapping"  */								\
          v.RevPeriod = 0x00007FFF + v.Tmp;													\
																							\
      v.RevPeriod &= 0x0000FFFF;															\
																							\
      v.CmtnDelay = v.RevPeriod/12;                  /* Division quotient*/					\
      v.GPR1_COM_TRIG = v.RevPeriod - v.CmtnDelay*12;  /* Division reminder*/					\
      if (v.GPR1_COM_TRIG >= 6) 																\
           v.CmtnDelay += 1;     /* if Division reminder >= 6, rounding division quotient*/	\
      v.Delay30DoneFlag = 0x0000000F;  /* flag indicates "gone through" once*/				\
   }   /* if-end: v.Delay30DoneFlag == 0*/    
