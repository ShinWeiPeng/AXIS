/*
 ******************************************************************************
 * Software License Agreement
 *     Copyright (c) 2021 ASIX Electronics Corporation   All rights reserved.
 * (1) This software is owned by ASIX Electronics Corporation and is protected
 *     under all applicable laws, including copyright laws.
 * (2) This software is provided by ASIX Electronics Corporation, including
 *     modifications and/or derivative works of this software, are only authorized
 *     for use on ASIX products.
 * (3) Redistribution and use of this software without specific written permission
 *     is void and will automatically terminate your rights under this license.
 * (4) Redistribution of source code or in binary form must retain/reproduce the
 *     copyright notice above and the following disclaimer in the documentation or other
 *     materials provided with the distribution.
 *
 * DISCLAIMER
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ASIX MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
 * LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
 * TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER ASIX
 * ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
 * FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
 * ANY REASON RELATED TO THIS SOFTWARE, EVEN IF ASIX OR ITS AFFILIATES HAVE
 * BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * ASIX reserves the right, without notice, to make changes to this software
 * and to discontinue the availability of this software.
 ******************************************************************************
 */

#ifndef __PARAM_DEF_H__
#define __PARAM_DEF_H__

/* INCLUDE FILE DECLARATIONS */
extern int mapMemory[];

#define aEscDAS (200)
#define aEscPDO (220)
#define aEscCMD (240)
#define aEscSEG (260)
#define aEscPOS (300)
#define aEscSPD (320)
#define aEscAMP (340)
#define aEscIOP (400)
#define aEscPWM (420)
#define aEscUVW (440)
#define aEscADC (460)
#define aEscANG (480)
#define aEscABZ (500)

#ifdef MC_DBG_PARAM_ENABLE
#define aEscDBG (990)
#endif

/* das module (map.200.L) */
#define aDasN    (aEscDAS+0)
#define aDasCMD  (aEscDAS+1)
#define aDasMS   (aEscDAS+2)
#define aDasDT   (aEscDAS+3)
#define aDasMAX  (aEscDAS+4)
#define aDasSIZ  (aEscDAS+5)

#define aDasCH   (aEscDAS+10)
#define aDasV1   (aEscDAS+11)
#define aDasV2   (aEscDAS+12)
#define aDasV3   (aEscDAS+13)
#define aDasV4   (aEscDAS+14)
#define aDasV5   (aEscDAS+15)
#define aDasV6   (aEscDAS+16)
#define aDasV7   (aEscDAS+17)
#define aDasV8   (aEscDAS+18)
#define aDasV9   (aEscDAS+19)

#define  dasN    (mapMemory[aDasN])
#define  dasCMD  (mapMemory[aDasCMD])
#define  dasMS   (mapMemory[aDasMS])
#define  dasDT   (mapMemory[aDasDT])
#define  dasMAX  (mapMemory[aDasMAX])
#define  dasSIZ  (mapMemory[aDasSIZ])
#define  dasCH   (mapMemory[aDasCH])
#define  dasV1   (mapMemory[aDasV1])
#define  dasV2   (mapMemory[aDasV2])
#define  dasV3   (mapMemory[aDasV3])
#define  dasV4   (mapMemory[aDasV4])
#define  dasV5   (mapMemory[aDasV5])
#define  dasV6   (mapMemory[aDasV6])
#define  dasV7   (mapMemory[aDasV7])
#define  dasV8   (mapMemory[aDasV8])
#define  dasV9   (mapMemory[aDasV9])

/* pdo module (map.220.L) */
#define aPdoCMD  (aEscPDO+0)
#define aPdoSTS  (aEscPDO+1)
#define aPdoPOS  (aEscPDO+2)
#define aPdoVEL  (aEscPDO+3)
#define aPdoTRQ  (aEscPDO+4)
#define aPdoMXC  (aEscPDO+5)
#define aPdoANG  (aEscPDO+6)
#define aPdoSPD  (aEscPDO+7)
#define aPdoAMP  (aEscPDO+8)
#define aPdoOUT  (aEscPDO+9)

#define pdoCMD  (mapMemory[aPdoCMD])
#define pdoSTS  (mapMemory[aPdoSTS])

#define pdoPOS  (mapMemory[aPdoPOS])
#define pdoVEL  (mapMemory[aPdoVEL])
#define pdoTRQ  (mapMemory[aPdoTRQ])
#define pdoMXC  (mapMemory[aPdoMXC])

#define pdoANG  (mapMemory[aPdoANG])
#define pdoSPD  (mapMemory[aPdoSPD])
#define pdoAMP  (mapMemory[aPdoAMP])
#define pdoOUT  (mapMemory[aPdoOUT])

/* cmd module (map.240.L) */
#define aCmdCMD (aEscCMD+0)
#define aCmdPOS (aEscCMD+1)
#define aCmdVEL (aEscCMD+2)
#define aCmdACC (aEscCMD+3)
#define aCmdTRQ (aEscCMD+4)
#define aCmdMXC (aEscCMD+5)
#define aCmdSIM (aEscCMD+6)
#define aCmdON  (aEscCMD+9)

#define cmdCMD  (mapMemory[aCmdCMD])
typedef enum
{
	LC_ALL_OFF	    = 0,
	LC_RESET        = 1,
	LC_PWM_CTRL		= 2,
	LC_AMP_CTRL	    = 3,
	LC_SPD_CTRL	    = 4,
	LC_POS_CTRL	    = 5,
	LC_PP_CTRL	    = 6,
} LOOP_CTRL_E;

#define cmdPOS  (mapMemory[aCmdPOS])
#define cmdVEL  (mapMemory[aCmdVEL])
#define cmdACC  (mapMemory[aCmdACC])
#define cmdTRQ  (mapMemory[aCmdTRQ])
#define cmdMXC  (mapMemory[aCmdMXC])
#define cmdSIM  (mapMemory[aCmdSIM])
#define cmdON   (mapMemory[aCmdON])

/* seg module (map.260.L) */
#define aSegCMD (aEscSEG+0)
#define aSegPOS (aEscSEG+1)
#define aSegVEL (aEscSEG+2)
#define aSegMAX (aEscSEG+3)
#define aSegACC (aEscSEG+4)
#define aSegDEC (aEscSEG+5)
#define aSegSTS (aEscSEG+6)

#define aSegALM (aEscSEG+10)
#define aSegERR (aEscSEG+11)
#define aSegMXC (aEscSEG+12)
#define aSegMS  (aEscSEG+13)

#define  segCMD (mapMemory[aSegCMD])
#define  segPOS (mapMemory[aSegPOS])
#define  segVEL (mapMemory[aSegVEL])
#define  segMAX (mapMemory[aSegMAX])
#define  segACC (mapMemory[aSegACC])
#define  segDEC (mapMemory[aSegDEC])
#define  segSTS (mapMemory[aSegSTS])

#define  segALM (mapMemory[aSegALM])
#define  segERR (mapMemory[aSegERR])
#define  segMXC (mapMemory[aSegMXC])
#define  segMS  (mapMemory[aSegMS])

/* pos module (map.300.L) */
#define aPosCMD  (aEscPOS+0)
#define aPosERR  (aEscPOS+1)
#define aPosSUM  (aEscPOS+2)
#define aPosOUT  (aEscPOS+3)
#define aPosMXE  (aEscPOS+4)
#define aPosMXI  (aEscPOS+5)
#define aPosMXO  (aEscPOS+6)
#define aPosKP   (aEscPOS+7)
#define aPosKI   (aEscPOS+8)
#define aPosDIV  (aEscPOS+9)

#define  posCMD  (mapMemory[aPosCMD])
#define  posERR  (mapMemory[aPosERR])
#define  posSUM  (mapMemory[aPosSUM])
#define  posOUT  (mapMemory[aPosOUT])
#define  posMXE  (mapMemory[aPosMXE])
#define  posMXI  (mapMemory[aPosMXI])
#define  posMXO  (mapMemory[aPosMXO])
#define  posKP   (mapMemory[aPosKP])
#define  posKI   (mapMemory[aPosKI])
#define  posDIV  (mapMemory[aPosDIV])

/* spd module (map.320.L) */
#define aSpdCMD  (aEscSPD+0)
#define aSpdERR  (aEscSPD+1)
#define aSpdSUM  (aEscSPD+2)
#define aSpdOUT  (aEscSPD+3)
#define aSpdMXE  (aEscSPD+4)
#define aSpdMXI  (aEscSPD+5)
#define aSpdMXO  (aEscSPD+6)
#define aSpdKP   (aEscSPD+7)
#define aSpdKI   (aEscSPD+8)
#define aSpdFLT  (aEscSPD+9)
#define aSpdDIV  (aEscSPD+10)

#define  spdCMD  (mapMemory[aSpdCMD])
#define  spdERR  (mapMemory[aSpdERR])
#define  spdSUM  (mapMemory[aSpdSUM])
#define  spdOUT  (mapMemory[aSpdOUT])
#define  spdMXE  (mapMemory[aSpdMXE])
#define  spdMXI  (mapMemory[aSpdMXI])
#define  spdMXO  (mapMemory[aSpdMXO])
#define  spdKP   (mapMemory[aSpdKP])
#define  spdKI   (mapMemory[aSpdKI])
#define  spdFLT  (mapMemory[aSpdFLT])
#define  spdDIV  (mapMemory[aSpdDIV])

/* amp module (map.340.L) */
#define aAmpCMD (aEscAMP+0)
#define aAmpSD  (aEscAMP+1)
#define aAmpSQ  (aEscAMP+2)
#define aAmpLMT (aEscAMP+3)
#define aAmpMXE (aEscAMP+4)
#define aAmpMXI (aEscAMP+5)
#define aAmpMXO (aEscAMP+6)
#define aAmpKP  (aEscAMP+7)
#define aAmpKI  (aEscAMP+8)
#define aAmpFLT (aEscAMP+9)

#define aAmpV0  (aEscAMP+10)
#define aAmpKV  (aEscAMP+11)
#define aAmpDM  (aEscAMP+12)
#define aAmpD0  (aEscAMP+13)
#define aAmpERR (aEscAMP+14)
#define aAmpERR2 (aEscAMP+15)
#define aAmpDIV (aEscAMP+16)

#define  ampCMD (mapMemory[aAmpCMD])
#define  ampSD  (mapMemory[aAmpSD])
#define  ampSQ  (mapMemory[aAmpSQ])
#define  ampLMT (mapMemory[aAmpLMT])
#define  ampMXE (mapMemory[aAmpMXE])
#define  ampMXI (mapMemory[aAmpMXI])
#define  ampMXO (mapMemory[aAmpMXO])
#define  ampKP  (mapMemory[aAmpKP])
#define  ampKI  (mapMemory[aAmpKI])
#define  ampFLT (mapMemory[aAmpFLT])

#define  ampV0  (mapMemory[aAmpV0])
#define  ampKV  (mapMemory[aAmpKV])
#define  ampDM  (mapMemory[aAmpDM])
#define  ampD0  (mapMemory[aAmpD0])
#define  ampERR (mapMemory[aAmpERR])
#define  ampERR2 (mapMemory[aAmpERR2])
#define  ampDIV (mapMemory[aAmpDIV])

/* iop module (map.400.L) */
#define aIopANG  (aEscIOP+0)
#define aIopSPD  (aEscIOP+1)
#define aIopID   (aEscIOP+2)
#define aIopIQ   (aEscIOP+3)
#define aIopVD   (aEscIOP+4)
#define aIopVQ   (aEscIOP+5)
#define aIopPWR  (aEscIOP+6)
#define aIopTMP  (aEscIOP+7)
#define aIopI2T  (aEscIOP+8)
#define aIopALM  (aEscIOP+9)

#define aIopREL  (aEscIOP+10)
#define aIopIDX  (aEscIOP+11)
#define aIopABS  (aEscIOP+12)
#define aIopMOD  (aEscIOP+13)
#define aIopPWM  (aEscIOP+14)
#define aIopRST  (aEscIOP+15)
#define aIopVBUS (aEscIOP+16)
#define aIopTEMP (aEscIOP+17)

#define  iopANG  (mapMemory[aIopANG])
#define  iopSPD  (mapMemory[aIopSPD])
#define  iopID   (mapMemory[aIopID])
#define  iopIQ   (mapMemory[aIopIQ])
#define  iopVD   (mapMemory[aIopVD])
#define  iopVQ   (mapMemory[aIopVQ])
#define  iopPWR  (mapMemory[aIopPWR])
#define  iopTMP  (mapMemory[aIopTMP])
#define  iopI2T  (mapMemory[aIopI2T])
#define  iopALM  (mapMemory[aIopALM])

#define  iopREL  (mapMemory[aIopREL])
#define  iopIDX  (mapMemory[aIopIDX])
#define  iopABS  (mapMemory[aIopABS])
#define  iopMOD  (mapMemory[aIopMOD])
#define  iopPWM  (mapMemory[aIopPWM])
#define  iopRST  (mapMemory[aIopRST])
#define  iopVBUS  (mapMemory[aIopVBUS])
#define  iopTEMP  (mapMemory[aIopTEMP])

/* pwm module (map.420.L) */
#define aPwmU    (aEscPWM+0)
#define aPwmV    (aEscPWM+1)
#define aPwmW    (aEscPWM+2)
#define aPwmDEG  (aEscPWM+3)
#define aPwmCMD  (aEscPWM+4)
#define aPwmMS   (aEscPWM+5)
#define aPwmANG  (aEscPWM+6)
#define aPwmENB  (aEscPWM+7)
#define aPwmBRK  (aEscPWM+8)
#define aPwmMAX  (aEscPWM+10)
#define aPwmINV  (aEscPWM+11)
#define aPwmRST  (aEscPWM+12)
#define aPwmFLT  (aEscPWM+13)

#define  pwmU    (mapMemory[aPwmU])
#define  pwmV    (mapMemory[aPwmV])
#define  pwmW    (mapMemory[aPwmW])
#define  pwmDEG  (mapMemory[aPwmDEG])
#define  pwmCMD  (mapMemory[aPwmCMD])
#define  pwmMS   (mapMemory[aPwmMS])
#define  pwmANG  (mapMemory[aPwmANG])
#define  pwmENB  (mapMemory[aPwmENB])
#define  pwmBRK  (mapMemory[aPwmBRK])
#define  pwmMAX  (mapMemory[aPwmMAX])
#define  pwmINV  (mapMemory[aPwmINV])
#define  pwmRST  (mapMemory[aPwmRST])
#define  pwmFLT  (mapMemory[aPwmFLT])

/* uvw module (map.440.L) */
#define aUvwDEG  (aEscUVW+0)
#define aUvwSIN  (aEscUVW+1)
#define aUvwCOS  (aEscUVW+2)
#define aUvwRDY  (aEscUVW+3)
#define aUvwKI   (aEscUVW+4)
#define aUvwKO   (aEscUVW+5)
#define aUvwDEG0 (aEscUVW+6)

#define  uvwDEG   (mapMemory[aUvwDEG])
#define  uvwSIN   (mapMemory[aUvwSIN])
#define  uvwCOS   (mapMemory[aUvwCOS])
#define  uvwRDY   (mapMemory[aUvwRDY])
#define  uvwKI    (mapMemory[aUvwKI])
#define  uvwKO    (mapMemory[aUvwKO])
#define  uvwDEG0  (mapMemory[aUvwDEG0])

/* adc module (map.460.L) */
#define aAdcU   (aEscADC+0)
#define aAdcV   (aEscADC+1)
#define aAdcW   (aEscADC+2)
#define aAdcP   (aEscADC+3)
#define aAdcT   (aEscADC+4)
#define aAdcK0  (aEscADC+7)
#define aAdcK   (aEscADC+8)
#define aAdcSTD (aEscADC+9)

#define aAdcIU  (aEscADC+10)
#define aAdcIV  (aEscADC+11)
#define aAdcIW  (aEscADC+12)
#define aAdcSUM (aEscADC+13)
#define aAdcDLY (aEscADC+14)

#define  adcU    (mapMemory[aAdcU])
#define  adcV    (mapMemory[aAdcV])
#define  adcW    (mapMemory[aAdcW])
#define  adcP    (mapMemory[aAdcP])
#define  adcT    (mapMemory[aAdcT])
#define  adcK0   (mapMemory[aAdcK0])
#define  adcK    (mapMemory[aAdcK])
#define  adcSTD  (mapMemory[aAdcSTD])

#define  adcIU   (mapMemory[aAdcIU])
#define  adcIV   (mapMemory[aAdcIV])
#define  adcIW   (mapMemory[aAdcIW])
#define  adcSUM  (mapMemory[aAdcSUM])
#define  adcDLY  (mapMemory[aAdcDLY])

/* ang module (map.480.L) */
#define aAngABS0  (aEscANG+0)
#define aAngIDX0  (aEscANG+1)
#define aAngREL0  (aEscANG+2)
#define aAngABS   (aEscANG+3)
#define aAngREL   (aEscANG+4)
#define aAngPOLE  (aEscANG+5)
#define aAngGEAR  (aEscANG+6)
#define aAngINV   (aEscANG+7)
#define aAngKN    (aEscANG+8)
#define aAngKD    (aEscANG+9)

#define  angABS0  (mapMemory[aAngABS0])
#define  angIDX0  (mapMemory[aAngIDX0])
#define  angREL0  (mapMemory[aAngREL0])
#define  angABS   (mapMemory[aAngABS])
#define  angREL   (mapMemory[aAngREL])
#define  angPOLE  (mapMemory[aAngPOLE])
#define  angGEAR  (mapMemory[aAngGEAR])
#define  angINV   (mapMemory[aAngINV])
#define  angKN    (mapMemory[aAngKN])
#define  angKD    (mapMemory[aAngKD])

/* abz module (map.500.L) */
#define aAbzENB  (aEscABZ+0)
#define aAbzFLT  (aEscABZ+1)
#define aAbzINV  (aEscABZ+2)
#define aAbzOUT  (aEscABZ+3)
#define aAbzPRD  (aEscABZ+4)
#define aAbzINP  (aEscABZ+5)
#define aAbzRST  (aEscABZ+6)

#define  abzENB  (mapMemory[aAbzENB])
#define  abzFLT  (mapMemory[aAbzFLT])
#define  abzINV  (mapMemory[aAbzINV])
#define  abzOUT  (mapMemory[aAbzOUT])
#define  abzPRD  (mapMemory[aAbzPRD])
#define  abzINP  (mapMemory[aAbzINP])
#define  abzRST  (mapMemory[aAbzRST])

/* protection module (map.950.L) */
#define aPrtOVP     (950+0)
#define aPrtUVP     (950+1)
#define aPrtOTP     (950+2)
#define aPrtOCP     (950+3)
#define aPrtUBC     (950+4)
#define aOvpFLT     (950+5)
#define aOtpFLT     (950+6)
#define aI2cFLT     (950+7)
#define aOvpCNT     (950+8)
#define aUvpCNT     (950+9)
#define aOtpCNT     (950+10)

#define prtOVP      (mapMemory[aPrtOVP])
#define prtUVP      (mapMemory[aPrtUVP])
#define prtOTP      (mapMemory[aPrtOTP])
#define prtOCP      (mapMemory[aPrtOCP])
#define prtUBC      (mapMemory[aPrtUBC])
#define ovpFLT      (mapMemory[aOvpFLT])
#define otpFLT      (mapMemory[aOtpFLT])
#define i2cFLT      (mapMemory[aI2cFLT])
#define ovpCNT      (mapMemory[aOvpCNT])
#define uvpCNT      (mapMemory[aUvpCNT])
#define otpCNT      (mapMemory[aOtpCNT])

#ifdef MC_DBG_PARAM_ENABLE
/* Debug module (map.990.L) */
#define aCoeERR  (aEscDBG+0)
#define aPdiLOK  (aEscDBG+1)
#define aPdiERR  (aEscDBG+2)
#define aLdpERR  (aEscDBG+3)
#define aFoeERR  (aEscDBG+4)

#define coeERR   (mapMemory[aCoeERR])//CoE operation error
#define pdiLOK   (mapMemory[aPdiLOK])
#define pdiERR   (mapMemory[aPdiERR])//SPI PDI access error
#define ldpERR   (mapMemory[aLdpERR])//Load parameters error
#define foeERR   (mapMemory[aFoeERR])//FoE operation error
#endif

/* sys module (map.1000.L) */
#define sysCLK   (mapMemory[1000])
#define sys20K   (mapMemory[1001])
#define sysLOP   (mapMemory[1002])
#define sysKHZ   (mapMemory[1003])
#define sysON    (mapMemory[1004])
#define sysOFF   (mapMemory[1005])
#define powerON  (mapMemory[1009])

/* led module (map.1010.L) */
#define ledR     (mapMemory[1010])
#define ledO     (mapMemory[1011])
#define ledY     (mapMemory[1012])
#define ledG     (mapMemory[1013])
#define ledMS    (mapMemory[1014])
#define ledBITS  (mapMemory[1015])

#define sysVERS   (mapMemory[1020])
#define sysDATE   (mapMemory[1021])
#define romVERS   (mapMemory[1022])
#define romDATE   (mapMemory[1023])

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */


#endif /* __PARAM_DEF_H__ */

/* End of param_def.h */

