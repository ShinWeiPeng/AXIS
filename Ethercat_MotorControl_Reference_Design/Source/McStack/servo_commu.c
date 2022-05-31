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

/* INCLUDE FILE DECLARATIONS */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "servo_commu.h"
#include "das.h"

/* NAMING CONSTANT DECLARATIONS */

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */

/* LOCAL SUBPROGRAM DECLARATIONS */

/* LOCAL SUBPROGRAM BODIES */

/* EXPORTED SUBPROGRAM BODIES */
int mapMemory[MC_PARAM_TABLE_SIZE];
              

//*********************************************************
//              map|idx|sub|n|dat
// Frame: m0.sdo.idx.sub.L=d,d,d : ndot=4,nequ>0
//           m0.sdo.idx.sub.L       : ndot=4,nequ=0
//           m0.map.addr=d          : ndot=2,nequ>0
//           m0.map.addr            : ndot=2,nequ=0
//*********************************************************
static int    mapD,idxD,subD,lenD,datD[512],numD,chnD;
static int    nVar,nEqu,nDat,nDot,dots[5];
static char   sVar[11],fmtD;
static size_t nLine;
static char  *sLine;

static cmd_t mapTable[]=    
{
	{SC_MAP_CMD, "map"},         
	{SC_DAS_CMD, "das"},         
	{0,""}
};
      
static cmd_t lenTable[]=   
{
	{0x04, "L"},           //**4B=long
	{0x02, "W"},           //**2B=word
	{0x01, "B"},           //**1B=byte
	{0,""}
};

static cmd_t varTable[]=     
{    
	{aPdoPOS,"pdoPOS"},{aPdoVEL,"pdoVEL"},{aPdoTRQ,"pdoTRQ"},{aPdoMXC,"pdoMXC"},
	{aCmdCMD,"cmdCMD"},{aCmdMXC,"cmdMXC"},{aCmdSIM,"cmdSIM"},
	{aSegMAX,"segMAX"},{aSegACC,"segACC"},{aSegDEC,"segDEC"},		 
	{aPosMXE,"posMXE"},{aPosMXI,"posMXI"},{aPosMXO,"posMXO"},{aPosKP, "posKP"}, {aPosKI, "posKI"},{aPosDIV,"posDIV"},
	{aSpdMXE,"spdMXE"},{aSpdMXI,"spdMXI"},{aSpdMXO,"spdMXO"},{aSpdKP, "spdKP"}, {aSpdKI, "spdKI"},{aSpdFLT,"spdFLT"},{aSpdDIV, "spdDIV"},
	{aAmpMXE,"ampMXE"},{aAmpMXI,"ampMXI"},{aAmpMXO,"ampMXO"},{aAmpKP, "ampKP"}, {aAmpKI, "ampKI"},{aAmpFLT,"ampFLT"},
	{aAmpV0, "ampV0"}, {aAmpKV, "ampKV"}, {aAmpDM,"ampDM"},{aAmpERR, "ampERR"},{aAmpERR2, "ampERR2"},{aAmpDIV, "ampDIV"},
	{aIopREL,"iopREL"},{aIopIDX,"iopIDX"},{aIopABS,"iopABS"},{aIopMOD,"iopMOD"},{aIopPWM,"iopPWM"},
	{aPwmMAX,"pwmMAX"},{aPwmINV,"pwmINV"},{aPwmRST,"pwmRST"},{aPwmFLT,"pwmFLT"},
	{aUvwRDY,"uvwRDY"},{aUvwKI, "uvwKI"}, {aUvwKO, "uvwKO"}, {aUvwDEG0,"uvwDEG0"},
	{aAdcK,  "adcK"},  {aAdcK0, "adcK0"}, {aAdcSTD,"adcSTD"},{aAdcDLY,"adcDLY"},
	{aAngABS0,"angABS0"},{aAngIDX0,"angIDX0"},{aAngREL0,"angREL0"},{aAngABS,"angABS"},{aAngREL,"angREL"},
	{aAngPOLE,"angPOLE"},{aAngGEAR,"angGEAR"},{aAngINV, "angINV"}, {aAngKN, "angKN"}, {aAngKD, "angKD"},
	{aAbzENB,"abzENB"},{aAbzFLT,"abzFLT"},{aAbzINV,"abzINV"},{aAbzOUT,"abzOUT"},{aAbzPRD,"abzPRD"},
	{aPrtOVP,"prtOVP"},{aPrtUVP,"prtUVP"},{aPrtOTP,"prtOTP"},{aPrtOCP,"prtOCP"},{aOvpFLT,"ovpFLT"},{aOtpFLT,"otpFLT"},{aI2cFLT,"i2cFLT"},
	{1000,"sysCLK"},{1001,"sys20K"},{1002,"sysLOP"},{1003,"sysKHZ"},{1004,"sysON"},
	{1005,"sysOFF"},                                                {1009,"powerON"},
	{1010,"ledR"},  {1011,"ledG"},  {1012,"ledY"},  {1013,"ledB"},  {1014,"ledMS"},
	{1015,"ledBITS"},
	{1020,"sysVERS"},{1021,"sysDATE"},{1022,"romVERS"},{1023,"romDATE"},
	{0,""},
};

static char	*sEcho;                  
static int	nEcho=0, mxEcho=0;
/*
 * ----------------------------------------------------------------------------
 * Function Name: putC()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void putC(char c) 
{
	if (nEcho<mxEcho)
	{
		sEcho[nEcho++] = c;
	}
} /* End of putC() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: putS()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void putS(char *s)
{
	int i,n; 
	n = strlen(s); 
	for (i=0; i<n; i++)
	{
		putC(*s++);
	}
} /* End of putS() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: putERR()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void putERR(void)  
{
	putS("*** error !!");
} /* End of putERR() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: getFields()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void getFields(void)         
{    
	int i; 
	char *s;      
	nDot=nEqu=nDat=0; 
	s=(char *)sLine;
	for (i=0; i<nLine && nDot<5 && !nDat; i++,s++)
	{    
		if (*s=='.')
		{
			dots[nDot++] = i;
		}
		if (*s == '=')        
		{
			nEqu = i; 
			nDat = 1;
		}
	}                                  
	for (; i<nLine && nDat>0; i++,s++) 
	{
		if (*s==',')
		{
			nDat++;
		}
	}
} /* End of getFields() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: getField()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void getField(int n)  
{    
	int i,k; 
	char *s,*d; k=0;
	if (n<0)                
	{    
		n=-n;              
		if (nEqu<=0 || n>nDat)   
		{
			sVar[0]=0; 
			nVar=0; 
			return;
		}
		for (i=nEqu+1,k=1,s=sLine+i; i<nLine && k<n; i++,s++)
		{
			if (*s==',')
			{
				k++;
			}	
		}      
		for (; i<nLine && *s==' '; i++, s++);
		for (k=0,d=sVar; i<nLine && *s!=' ' && k<10; i++,k++)
		{
			if (*s==',') 
			{
				break;    
			}
			*d++=*s++;
		}
		*d=0;    
		nVar=k; 
		return;
	}                       
	if (n==0)   
	{
		i = 0;           
		k = dots[n];
	}
	else if (n<nDot) 
	{
		i=dots[n-1]+1; 
		k=dots[n];
	}
	else
	{    
		if (nEqu)   
		{
			i=dots[n-1]+1; 
			k=nEqu;
		}
		else        
		{
			i=dots[n-1]+1; 
			k=nLine;
		}
	}    
	nVar=k-i;          
	for (s=sLine+i; i<k && *s==' '; i++)       
	{
		s++;
	}     
	for (d=sVar; i<k && *s!=' '; i++) 
	{    
		*d++=*s++;
	}     
	*d=0;
} /* End of getField() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: getTable()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int getTable(cmd_t *tab)      
{    
	cmd_t *p;
	for (p=tab; p->code; p++)
	{
		if (strcmp(sVar,p->name)==0) 
		{
			return(p->code);
		}
	}
	return(0);
} /* End of getTable() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: getInteger()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int getInteger(void)      
{   
	int i, d, neg; 
	char c, *s; 
	s=sVar;

	if (s[0]=='0' && (s[1]=='X' || s[1]=='x'))
	{    
		for (s+=2, i=2, d=0; i<nVar; i++)
		{    
			c=*s++;
			if (c>='a')
			{
				c=c-'a'+10;
			}
			else if (c>='A') 
			{
				c=c-'A'+10;
			}
			else
			{
				c=c-'0';
			}
			if (c<0 || c>15) 
			{
				mapD=0; 
				return(0);
			}
			d=d*16+c;
		}
	} 
	else         
	{    
		if (*s=='-') 
		{
			neg=1; 
			i=1; 
			s++;
		}
		else         
		{
			neg=0; 
			i=0;
		}
		for (d=0; i<nVar; i++)
		{    
			c=*s++; 
			c-='0';
			if (c<0 || c>9)  
			{
				mapD=0; 
				return(0);
			}
			d=d*10+c;
		}    
		if (neg)
		{
			d=-d;
		}
	}    
	return(d);
} /* End of getInteger() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: getVariable()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void getVariable(void)    
{    
	int i,n;
	
	mapD=0;
	idxD=0;
	subD=0;
	lenD=0;
	chnD=0;
	fmtD='D';
	
	getFields();
	n=nDot;
  if (n==0)              
  {    
		dots[0]=(nEqu>0)?nEqu:nLine;
		getField(0); 
		n=getTable(varTable);
		if (n<=0)
		{			
			return;
		}
		mapD=SC_MAP_CMD; 
		idxD=n; 
		lenD=4; 
		subD=0; 
		n=0;
  }
  for (i=0; i<nDat; i++)  
  {
		getField(-(i+1)); 
		datD[i]=getInteger();
  }
  if (n<2 || n>4)
	{
		return;
	}

	getField(0); 
  if (nVar!=2 || sVar[0]!='m')
	{
		return;
	}
  chnD=sVar[1]-'0';   
  if (chnD<0 || chnD>9) 
	{
		return;
	}
	getField(1);  
  mapD=getTable(mapTable);
  getField(n);
  if (nVar>1)  
  {
		fmtD=sVar[1]; 
		sVar[1]=0; 
		nVar=1;
  }
  lenD=getTable(lenTable);
  getField(2);  
  idxD=getInteger();
  if (lenD>0) 
  {
		n--;
  }               
  if (n<3)
	{
		return;
	}
  getField(3);  
  subD=getInteger();
} /* End of getVariable() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: putH()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void putH(int d)        
{    
	int i,k;
	char c;
	
	if (lenD==1) 
	{
		k=2; 
		d<<=24;
	}  
	else if (lenD==2) 
	{
		k=4; 
		d<<=16;
	}  
	else              
	{
		k=8;
	}          
	for (i=0; i<k; i++,d<<=4)
	{    
		c=(d>>28)&0xf;
		if (c>9)
		{			
			c=c+'A'-10;
		}
		else
		{
			c=c+'0';
		}
		putC(c);
	}    
	putC(' ');
} /* End of putH() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: putD()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void putD(int d)         
{    
	int i,k,m;
	
	if (fmtD=='X') 
	{
		putC('0'); 
		putC('x'); 
		putH(d); 
		return;
	}
	if (fmtD=='H')                       
	{
		putH(d); 
		return;
	}
	if (d<0) 
	{
		d=-d; 
		putC('-');
	}
	for (m=0,k=1000000000; k>0; k/=10)
	{
		i=d/k; 
		if (i || m) 
		{
			putC('0'+i); 
			d=d%k; 
			m=1;
		}
	}
	if (m==0)
	{
		putC('0');
	}
	putC(' ');
} /* End of putD() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: getD()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int getD(int d)   
{    
	if (lenD==1) 
	{
		d&=0xff;
	}
	if (lenD==2) 
	{
		d&=0xffff; 
		if ((d&0x8000) && (fmtD!='U')) 
		{
			d-=0x10000;
		}
	}
	return(d);
} /* End of getD() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: getMAP()
 * Purpose:
 * Params:
 * Returns:
 * Note: esc.get("m0.map.<idxD>.<subD>.<lenD>", numD)
 * ----------------------------------------------------------------------------
 */
static int getMAP(void)           
{    
	int i,d;
	if (idxD<0 || idxD>=1024 || nDat>0)
	{
		return(0);
	}
	if (numD<1) 
	{
		numD=1;
	}  
	if (numD>512) 
	{
		numD=512;
	}
	if ((idxD+numD)>1024)
	{
		numD=1024-idxD;
	}
	for (i=0; i<numD; i++)
	{
		d=mapMemory[idxD++]; 
		datD[i]=d;
	}
	return(numD);
} /* End of getMAP() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: putMAP()
 * Purpose:
 * Params:
 * Returns:
 * Note: esc.put("m0.map.<idxD>.<subD>.<lenD>=<..nDat..>",numD)
 * ----------------------------------------------------------------------------
 */
static int putMAP(void)       
{    
	int i,k,d;
	if (idxD<0 || idxD>=1024 || nDat<1)
	{
		return(0);
	}
	if (numD<1) 
	{
		numD=nDat;
	}  
	if (numD>512) 
	{
		numD=512;
	}
	if ((idxD+numD)>1024)
	{
		numD=1024-idxD;
	}
	for (i=0,k=0; i<numD; i++)
	{    
		d=datD[k]; 
		if ((++k)>=nDat) 
		{
			k=0;
		}
		mapMemory[idxD++]=d;
	}    
	return(0);
} /* End of putMAP() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: putDAS()
 * Purpose:
 * Params:
 * Returns:
 * Note: esc.put("m0.das.<idxD>.<subD>.<lenD>=<..nDat..>",numD)
 * ----------------------------------------------------------------------------
 */
static int putDAS(void) 
{    
	int i,k,d,addr;
	
	if (nDat<1 || idxD<0 || subD<0) 
	{
		return(0);
	}
	if (numD<1) 
	{
		numD=nDat;
	} 
	if (numD>512) 
	{
		numD=512;
	}
	for (i=0,k=0; i<numD; i++,idxD++)
	{    
		d=datD[k];
		if ((++k)>=nDat) 
		{
			k=0;
		}
		if (subD>0) 
		{
			addr=idxD*dasCH+subD-1;
		}
		else
		{
			addr=idxD;
		}
		if (addr<dasSIZ) 
		{
			dasMemory[addr]=d;
		}
	}    
	return(0);
} /* End of putDAS() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: getDAS()
 * Purpose:
 * Params:
 * Returns:
 * Note: esc.get("m0.das.<idxD>.<subD>.<lenD>",numD)
 * ----------------------------------------------------------------------------
 */
static int getDAS(void) 
{    
	int i, addr;
	
	if (nDat>0 || idxD<0 || subD<0)
	{
     return(0);
	}
	
	if (numD<1) 
	{
		numD=1;
	}
	
	if (numD>512)
	{
		numD=512;
	}
	
	for (i=0; i<numD; i++, idxD++)
	{    
		if (subD>0)
		{
			addr=idxD*dasCH+subD-1;
		}
		else
		{
			addr=idxD;
		}
		if (addr<dasSIZ) 
		{
			datD[i]=dasMemory[addr];
		}
		else
		{
			datD[i]=0;
		}
	}    
	return(numD);
} /* End of getDAS() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: putCommand()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int putCommand(void)     
{
	switch (mapD)                    
	{
	case SC_MAP_CMD: 
		return(putMAP());
	
	case SC_DAS_CMD: 
		return(putDAS());
	} 
	putERR(); 
	return(0);
} /* End of putCommand() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: getCommand()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int getCommand(void)    
{
	switch (mapD)                     
	{
	case SC_MAP_CMD:
		return(getMAP());
	
  case SC_DAS_CMD:
		return(getDAS());
	} 
	putERR(); 
	return(0);
} /* End of getCommand() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: esc_doCommand()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
int esc_doCommand(char *echo,int mx,char *cmd,char *s,int n,int d)
{    
	sEcho=echo;
	nEcho=0;
	mxEcho=mx;
	nLine=n;
	sLine=s;
	numD=d;
	
	if (d>=0)
	{
		getVariable();
	}

	if (strcmp(cmd,"put")==0)
	{
		d=putCommand();
	}
	else if (strcmp(cmd,"get")==0)
	{
		d=getCommand();
	}
	else
	{
		putERR(); 
		return(nEcho);
	}
	
	if (d<0)
	{
		return(d);
	}
	
	for (int i=0; i<d; i++)
	{
		putD(getD(datD[i]));
	}
	return(nEcho);                
} /* End of esc_doCommand() */

/* End of servo_commu.c */
