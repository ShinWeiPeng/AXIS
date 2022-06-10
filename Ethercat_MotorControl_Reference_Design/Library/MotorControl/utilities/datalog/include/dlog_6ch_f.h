//#############################################################################
// $TI Release: MotorControl SDK v4.00.00.00 $
// $Release Date: Thu Feb 17 18:05:20 CST 2022 $
// $Copyright:
// Copyright (C) 2017-2020 Texas Instruments Incorporated - http://www.ti.com/
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

//----------------------------------------------------------------------------------
//	FILE:			dlog_6ch_f.h
//
//	Description:header file for data logging module
//
//	Version: 		1.0
//
//  Target:  		TMS320F2837x, TMS320F2838x, TMS320F28004x
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments � 2004-2017
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------


#ifndef DLOG_6CH_F_H
#define DLOG_6CH_F_H
#include "device.h"

#define DLOG_CH_NUM			6
#define DLOG_BUF_SIZE		512
#define DLOG_SCALER_NUM     10

//*********** Structure Definition ********//
typedef struct{
	float32_t *input_ptr[DLOG_CH_NUM];
	float32_t *output_ptr[DLOG_CH_NUM];
	float32_t refValue;	
	float32_t prevValue;
	uint16_t status;
	uint16_t preScalar;
    uint16_t skipCount;
	uint16_t size;
	uint16_t count;
}DLOG_6CH_F;

//*********** Function Declarations *******//
void DLOG_6CH_F_init(DLOG_6CH_F *v);
void DLOG_6CH_F_FUNC(DLOG_6CH_F *v);

#endif /* DLOG_6CH_F_H_ */
