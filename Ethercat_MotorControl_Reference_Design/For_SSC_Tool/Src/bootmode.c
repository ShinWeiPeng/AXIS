/*
* This source file is part of the EtherCAT Slave Stack Code licensed by Beckhoff Automation GmbH & Co KG, 33415 Verl, Germany.
* The corresponding license agreement applies. This hint shall not be removed.
*/

/**
\addtogroup ESM EtherCAT State Machine
@{
*/

/**
\file bootmode.c
\author EthercatSSC@beckhoff.com
\brief Implementation

\version 5.12.1

<br>Changes to version V4.20:<br>
V5.12 BOOT2: call BL_Start() from Init to Boot<br>
<br>Changes to version - :<br>
V4.20: File created
*/

/*--------------------------------------------------------------------------------------
------
------    Includes
------
--------------------------------------------------------------------------------------*/
#include "ecat_def.h"
#include "ecatslv.h"
#define _BOOTMODE_ 1
#include "bootmode.h"
#undef _BOOTMODE_
#include "foeappl.h"
#include "ecatfoe.h"

#define PARAMETERA_UPDATE_BASE_ADDR		0x38000
#define PARAMETERB_UPDATE_BASE_ADDR		0x78000
#define PARAMETER_UPDATE_MAX_FILE_SIZE	0x7FFF

#define FIRMWARE_UPDATE_BASE_ADDR		0x40000
#define FIRMWARE_UPDATE_MAX_PAGE_NUMBER	64
#define FIRMWARE_UPDATE_MAX_FILE_SIZE	0x37FFF
#define SIGNATURE_STR					"ASIX"
#define FIRMWARE_UPDATE_ENABLE			1
#define H2NL(x) ((x >> 24 & 0x000000FF) | (x >> 8 & 0x0000FF00) | ( x << 24 & 0xFF000000) | (x << 8 & 0x00FF0000))
#define TIMEOUT_FOR_JUMP_TO_LDROM		1000 //ms
#define FILE_IMAGE_TYPE		0
#define PARAM_IMAGE_TYPE	1

typedef struct _UPGRADE_RUNTIME_HEADER
{
	unsigned char	Signature[4];
	unsigned short	ImageType;
	unsigned short	Compress;
	unsigned long	ContentOffset;
	unsigned long	FileLen;
	unsigned long	Reserved0;
	unsigned long	Checksum32;	
	unsigned long	Reserved1[2];
} UPGRADE_RT_HEAD;

typedef struct _UPGRADE_FIRMWARE
{
	unsigned char	State;
	unsigned long	FileOffset;
	unsigned long	FileLength;
	unsigned int	NumberPageErase;
	unsigned int 	PageBeErased;
	int32_t UpgradeDownFlag ;
} UPGRADE_FIRMWARE;

typedef enum{
	checksum_fault = 0,
	checksum_success,
	parameter_address_empty,
}ECHECKSUM32;

UPGRADE_FIRMWARE	firmwareObj;
UPGRADE_RT_HEAD		firmwareHeader;
UINT16				timerForJumpToLdrom;


UINT32 firmwareAddr = FIRMWARE_UPDATE_BASE_ADDR+sizeof(UPGRADE_RT_HEAD);
UINT32 parameterAddr_A = PARAMETERA_UPDATE_BASE_ADDR + sizeof(UPGRADE_RT_HEAD);
UINT32 parameterAddr_B = PARAMETERB_UPDATE_BASE_ADDR + sizeof(UPGRADE_RT_HEAD);

static UINT32 ReadFlashHeader(UINT32 Addr,uint8_t *data);
static UINT32 FirmwareChecksum32(UINT32 Addr,UINT32 length);

/*ECATCHANGE_START(V5.12) BOOT2*/
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    State        Current state

 \brief Called from INIT to BOOT
*////////////////////////////////////////////////////////////////////////////////////////
void BL_Start( UINT8 State)
{
	
	if (State == STATE_BOOT)
	{
		firmwareObj.State = BL_STATE_START;
		firmwareObj.UpgradeDownFlag = 0;
	}
	else if (State == 0)//Polling
	{
	}
}
/*ECATCHANGE_END(V5.12) BOOT2*/

void BL_Init(void)
{
	firmwareObj.State = BL_STATE_IDLE;
	firmwareObj.FileOffset = 0;
	firmwareObj.FileLength = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**

\brief Called in the state transition from BOOT to Init
*////////////////////////////////////////////////////////////////////////////////////////
void BL_Stop(void)
{
	firmwareObj.UpgradeDownFlag = 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    password    download password

 \brief Dummy BL_StartDownload function
*////////////////////////////////////////////////////////////////////////////////////////
void BL_StartDownload(UINT32 password0)
{
	if (firmwareObj.State == BL_STATE_START)
	{
		firmwareObj.State = BL_STATE_START_DOWNLOAD;
		firmwareObj.FileOffset = 0;
		firmwareObj.FileLength = 0;
		SYS_UnlockReg();		/* Unlock register lock protect */
		FMC_Open();				/* Enable FMC ISP function */
		FMC_ENABLE_AP_UPDATE();	/* Enable FMC erase/program APROM */
		SYS_LockReg();			/* Lock protected registers */
	}
	printf("Start download firmware...\r\n");
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pData    Data pointer
 \param    Size    Data Length


 \return    FoE error code

 \brief Dummy BL_Data function
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 BL_Data(UINT16 *pData,UINT16 Size)
{
	unsigned long	dwordData, dataRead32;
	int				updateLen = 0;
	UINT16			writeSize = Size;
	unsigned char	*pSrcData = (unsigned char *)pData;
	static UINT16 parameterAddrChange = 0;
	

	if (firmwareObj.State < BL_STATE_START_DOWNLOAD)
	{
		return ECAT_FOE_ERRCODE_ACCESS;
	}
	else if (firmwareObj.State == BL_STATE_START_DOWNLOAD)
	{
		if (firmwareObj.FileOffset == 0)
		{
			memcpy((unsigned char *)&firmwareHeader, pSrcData, sizeof(UPGRADE_RT_HEAD));
			// printf("Signature %c%c%c%c\r\n", firmwareHeader.Signature[0],
			// 								 firmwareHeader.Signature[1],
			// 								 firmwareHeader.Signature[2],
			// 								 firmwareHeader.Signature[3]);

			/* Check the signature */
			if ((firmwareHeader.Signature[0] != 'A') | (firmwareHeader.Signature[1] != 'S') |
				(firmwareHeader.Signature[2] != 'I') | (firmwareHeader.Signature[3] != 'X'))
			{
				SYS_UnlockReg();
				FMC_Close();	/* Disable FMC ISP function */
				SYS_LockReg();
				return ECAT_FOE_ERRCODE_ILLEGAL;
			}
			/* Check the file size */
			firmwareObj.FileLength = H2NL(firmwareHeader.FileLen);
			if(firmwareHeader.ImageType == FILE_IMAGE_TYPE)
			{
				if (firmwareObj.FileLength > FIRMWARE_UPDATE_MAX_FILE_SIZE)
				{
					SYS_UnlockReg();
					FMC_Close();	/* Disable FMC ISP function */
					SYS_LockReg();
					return ECAT_FOE_ERRCODE_ILLEGAL;
				}
			}
			else if(firmwareHeader.ImageType == PARAM_IMAGE_TYPE)
			{
				if (firmwareObj.FileLength > PARAMETER_UPDATE_MAX_FILE_SIZE)
				{
					SYS_UnlockReg();
					FMC_Close();	/* Disable FMC ISP function */
					SYS_LockReg();
					return ECAT_FOE_ERRCODE_ILLEGAL;
				}
			}
			
			firmwareObj.NumberPageErase = firmwareObj.FileLength/4096;
			if (firmwareObj.FileLength % 4096)
			{
				
				firmwareObj.NumberPageErase += 1;
			}
			
//			printf("Page to be Erased %d\r\n", firmwareObj.NumberPageErase);
			firmwareObj.PageBeErased = 0;
		}
		firmwareObj.State = BL_STATE_DATA;
	}
	
	/* Erase the flash for firmware update */
	if (firmwareObj.NumberPageErase >  0)
	{
		SYS_UnlockReg();
		if(firmwareHeader.ImageType == FILE_IMAGE_TYPE)
		{
			if (FMC_Erase(FIRMWARE_UPDATE_BASE_ADDR+(firmwareObj.PageBeErased*4096)) < 0)
			{
					FMC_Erase(FIRMWARE_UPDATE_BASE_ADDR);
					FMC_Close();                       /* Disable FMC ISP function */
					SYS_LockReg();
					return ECAT_FOE_ERRCODE_DISKFULL;
			}
		}
		else if(firmwareHeader.ImageType == PARAM_IMAGE_TYPE)
		{
			UPGRADE_RT_HEAD		flashHeader32;
			uint8_t result = ReadFlashHeader(PARAMETERA_UPDATE_BASE_ADDR,(uint8_t*)&flashHeader32);
			if (result == 1 && FirmwareChecksum32(parameterAddr_A , H2NL(flashHeader32.FileLen)-sizeof(UPGRADE_RT_HEAD))== ~(flashHeader32.Checksum32))
			{
				if (FMC_Erase(PARAMETERB_UPDATE_BASE_ADDR+(firmwareObj.PageBeErased*4096)) < 0)
				{
						FMC_Close();                       /* Disable FMC ISP function */
						SYS_LockReg();
						return ECAT_FOE_ERRCODE_DISKFULL;
				}
				parameterAddrChange = 0;
			}
			else 
			{
				if (FMC_Erase(PARAMETERA_UPDATE_BASE_ADDR+(firmwareObj.PageBeErased*4096)) < 0)
				{
						FMC_Close();                       /* Disable FMC ISP function */
						SYS_LockReg();
						return ECAT_FOE_ERRCODE_DISKFULL;
				}
				parameterAddrChange = 1;
			}
		}
		SYS_LockReg();
		firmwareObj.NumberPageErase--;
		firmwareObj.PageBeErased++;
//		printf("Erased %d\r\n", firmwareObj.PageBeErased);
	}
	
	if ((firmwareObj.FileOffset+Size) > firmwareObj.FileLength)
	{
			SYS_UnlockReg();
			FMC_Erase(FIRMWARE_UPDATE_BASE_ADDR);	/* Erase the signature page */
			FMC_Close();							/* Disable FMC ISP function */
			SYS_LockReg();
			return ECAT_FOE_ERRCODE_DISKFULL;
	}
	
	SYS_UnlockReg();
	writeSize = Size;
	while (writeSize > 0)
	{
		if (writeSize >= 4)
		{
			dwordData = *(unsigned long *)pSrcData;
			updateLen = 4;
    }
		else
		{
			dwordData = 0xFFFFFFFF;
			updateLen = writeSize;
			memcpy((unsigned char *)&dwordData, pSrcData, updateLen);
		}
		/* Write data to flash, FMC support 4 bytes write */
		if(firmwareHeader.ImageType == FILE_IMAGE_TYPE)
		{
			FMC_Write(FIRMWARE_UPDATE_BASE_ADDR+firmwareObj.FileOffset, dwordData);
			dataRead32 = FMC_Read(FIRMWARE_UPDATE_BASE_ADDR+firmwareObj.FileOffset);
			if (dataRead32 != dwordData)
			{
				printf("FW Write %8lx, Readback: %8lx\r\n", dwordData, dataRead32);
				FMC_Erase(FIRMWARE_UPDATE_BASE_ADDR);	/* Erase the signature page */
				FMC_Close();							/* Disable FMC ISP function */
				SYS_LockReg();
				return ECAT_FOE_ERRCODE_ILLEGAL;
			}
		}
		else if(firmwareHeader.ImageType == PARAM_IMAGE_TYPE)
		{
			if(parameterAddrChange == 1)
			{
				FMC_Write(PARAMETERA_UPDATE_BASE_ADDR+firmwareObj.FileOffset, dwordData);
				dataRead32 = FMC_Read(PARAMETERA_UPDATE_BASE_ADDR+firmwareObj.FileOffset);
				if (dataRead32 != dwordData)
				{
					printf("ParameterA Write %8lx, Readback: %8lx\r\n", dwordData, dataRead32);
					FMC_Erase(PARAMETERA_UPDATE_BASE_ADDR);	/* Erase the signature page */
					FMC_Close();							/* Disable FMC ISP function */
					SYS_LockReg();
					return ECAT_FOE_ERRCODE_ILLEGAL;
				}
			}
			else
			{
				FMC_Write(PARAMETERB_UPDATE_BASE_ADDR+firmwareObj.FileOffset, dwordData);
				dataRead32 = FMC_Read(PARAMETERB_UPDATE_BASE_ADDR+firmwareObj.FileOffset);
				if (dataRead32 != dwordData)
				{
					printf("ParameterB Write %8lx, Readback: %8lx\r\n", dwordData, dataRead32);
					FMC_Erase(PARAMETERB_UPDATE_BASE_ADDR);	/* Erase the signature page */
					FMC_Close();							/* Disable FMC ISP function */
					SYS_LockReg();
					return ECAT_FOE_ERRCODE_ILLEGAL;
				}
			}
			
		}
		
		firmwareObj.FileOffset += updateLen;
		writeSize -= updateLen;
		pSrcData += updateLen;
	}
	SYS_LockReg();

	if (firmwareObj.FileOffset >= firmwareObj.FileLength)
	{	
		if(firmwareHeader.ImageType == FILE_IMAGE_TYPE)
		{
			printf("FW received completely, ");
			if (FirmwareChecksum32(firmwareAddr , firmwareObj.FileLength-sizeof(UPGRADE_RT_HEAD))!= ~firmwareHeader.Checksum32)
			{
				printf("but CRC error!!!\r\n\r\n");
				SYS_UnlockReg();
				FMC_Erase(FIRMWARE_UPDATE_BASE_ADDR);	/* Erase the signature page */
				FMC_Close();							/* Disable FMC ISP function */
				SYS_LockReg();
				return ECAT_FOE_ERRCODE_ILLEGAL;
			}
			printf("jump to LDROM after 1 second.\r\n");
			printf("LDROM will move received firmware to APROM later.\r\n\r\n");
			timerForJumpToLdrom = HW_GetTimer();
			firmwareObj.State = BL_STATE_FW_TRANSMIT_DONE;
			
		}
		else if (firmwareHeader.ImageType == PARAM_IMAGE_TYPE)
		{
			printf("Parameter received completely, ");
			if(parameterAddrChange == 1)
			{
				if (FirmwareChecksum32(parameterAddr_A , firmwareObj.FileLength-sizeof(UPGRADE_RT_HEAD))!= ~firmwareHeader.Checksum32)
				{
					printf("but CRC error!!!\r\n\r\n");
					SYS_UnlockReg();
					FMC_Erase(PARAMETERA_UPDATE_BASE_ADDR);	/* Erase the signature page */
					FMC_Close();							/* Disable FMC ISP function */
					SYS_LockReg();
					return ECAT_FOE_ERRCODE_ILLEGAL;
				}
				printf("and write in flash memory area A. \r\n");
				SYS_UnlockReg();
				FMC_Open();      
				FMC_ENABLE_AP_UPDATE();
				FMC_Erase(PARAMETERB_UPDATE_BASE_ADDR); 
				FMC_DISABLE_AP_UPDATE();
				FMC_Close();     
				SYS_LockReg();
			}
				
			else
			{
				if (FirmwareChecksum32(parameterAddr_B , firmwareObj.FileLength-sizeof(UPGRADE_RT_HEAD))!= ~firmwareHeader.Checksum32)
				{
					printf("but CRC error!!!\r\n\r\n");
					SYS_UnlockReg();
					FMC_Erase(PARAMETERB_UPDATE_BASE_ADDR);	/* Erase the signature page */
					FMC_Close();							/* Disable FMC ISP function */
					SYS_LockReg();
					return ECAT_FOE_ERRCODE_ILLEGAL;
				}
				printf("and write in flash memory area B. \r\n");
				SYS_UnlockReg();
				FMC_Open();      
				FMC_ENABLE_AP_UPDATE();
				FMC_Erase(PARAMETERA_UPDATE_BASE_ADDR); 
				FMC_DISABLE_AP_UPDATE();
				FMC_Close();     
				SYS_LockReg();
			}

			printf("jump To Reset.\r\n\r\n");
			timerForJumpToLdrom = HW_GetTimer();
			firmwareObj.State = BL_STATE_PARAMETER_TRANSMIT_DONE;
		}		
	}
	return 0;
}
static UINT32 ReadFlashHeader(UINT32 Addr,uint8_t *data)
{
	UINT32 i;
	UINT32 count = 8;
	uint32_t *pbuffer = (uint32_t*)data;
	UPGRADE_RT_HEAD *pHeader =(UPGRADE_RT_HEAD*)data;
	SYS_UnlockReg();
	for (i = 0; i < count; i++)
	{

		pbuffer[i] = FMC_Read(Addr);
		Addr += 4;
	}

	SYS_LockReg();
	if ((pHeader->Signature[0] != 'A') | (pHeader->Signature[1] != 'S') |
				(pHeader->Signature[2] != 'I') | (pHeader->Signature[3] != 'X'))
	{
		return(0);
	}
	return(1);
}
static UINT32 FirmwareChecksum32(UINT32 Addr,UINT32 length)
{
	UINT32 i;
	UINT32 count = (length >> 2);
	UINT32 cksum = 0, data32;
	unsigned char buffer[4];

	SYS_UnlockReg();
	for (i = 0; i < count; i++)
	{

		data32 = FMC_Read(Addr);
		cksum += data32;
		Addr += 4;
//		printf("cksum: %x, data32:%x, Addr:%x\r\n",cksum,data32,Addr);
		if(cksum < data32)
		{
			cksum++;
		}
	}
	if (length % 4)
	{
		data32 = FMC_Read(Addr);

		memcpy(buffer, (unsigned char *)&data32, 4);
		count = length % 4;
		data32 = 0;
		i = 0;
		while (count > 1)
		{
			data32 |= buffer[(count-1)];
			count--;
			data32 <<= 8;
		}
		data32 += buffer[i];
//		printf("convert long word %8lx\r\n", data32);
		cksum += data32;
		if (cksum < data32)
		{
			cksum++;
		}
	}

	SYS_LockReg();
	return (cksum);
} /* End of FirmwareChecksum32() */

void BL_JumpToLdrom(void)
{
	UINT16	tmp16;
	
	if (firmwareObj.State == BL_STATE_FW_TRANSMIT_DONE && firmwareObj.UpgradeDownFlag == 1)
	{
		tmp16 = HW_GetTimer();
		if (tmp16 >= timerForJumpToLdrom)
		{
			if ((tmp16 - timerForJumpToLdrom) >= TIMEOUT_FOR_JUMP_TO_LDROM)
			{
				firmwareObj.State = BL_STATE_IDLE;
				SYS_UnlockReg();
				FMC_Open();
				FMC_SetBootSource(1);
				FMC_Close();
				SYS_LockReg();
				NVIC_SystemReset();	
			}
		}
		else
		{
			timerForJumpToLdrom = tmp16;
		}
	}

	else if (firmwareObj.State == BL_STATE_PARAMETER_TRANSMIT_DONE && firmwareObj.UpgradeDownFlag == 1)
	{
		tmp16 = HW_GetTimer();
		if (tmp16 >= timerForJumpToLdrom)
		{
			if ((tmp16 - timerForJumpToLdrom) >= TIMEOUT_FOR_JUMP_TO_LDROM)
			{
				firmwareObj.State = BL_STATE_IDLE;
				SYS_UnlockReg();
				FMC_Open();
				FMC_SetBootSource(0);
				FMC_Close();
				SYS_LockReg();
				NVIC_SystemReset();	
			}
		}
		else
		{
			timerForJumpToLdrom = tmp16;
		}
	}
}

/** @} */

/** @} */
