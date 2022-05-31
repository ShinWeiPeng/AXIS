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

/**
\addtogroup AX58200 Foe implementation file
@{
*/

/**
\file AX58200_FoeAppl.c
\brief Implementation

\version 1.0.0.0
*/

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/

#include "ecat_def.h"
#include "ecatslv.h"
#include "ecatfoe.h"
#define _FOEAPPL_ 1
    #include "foeappl.h"
#undef _FOEAPPL_
#define _FOEAPPL_ 0

#include "bootmode.h"
#include "applInterface.h"
#include "AX58200_FoeAppl.h"
/*-----------------------------------------------------------------------------------------
------
------    internal Types and Defines
------
-----------------------------------------------------------------------------------------*/

#define	MAX_FILE_NAME_SIZE	16
/*-----------------------------------------------------------------------------------------
------
------    Module internal function declarations
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    Module internal variable definitions
------
-----------------------------------------------------------------------------------------*/
UINT32              nFileSize;
UINT32              nFileWriteOffset;
CHAR                aFileName[MAX_FILE_NAME_SIZE];
UINT16 MBXMEM       aFileData[(MAX_FILE_SIZE >> 1)];
const UINT16 HUGE   aFirmwareDownloadHeader[4] = {0x5841, 0x3835, 0x3032, 0x5F30}; // "AX58200_"

/*-----------------------------------------------------------------------------------------
------
------    Functions
------
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     pName         Pointer to the name of the file
 \param     nameSize      Length of the file name
 \param     pData         Pointer to the response data
 \param     password      Password for the file read

 \return size of the data to be sent, busy or an error code
            1..FOE_MAXBUSY-101 (size of data to be sent)
            FOE_MAXBUSY-100 (0%)    (busy)
            FOE_MAXBUSY (100%) (busy)
            ECAT_FOE_ERRCODE_NOTDEFINED (error)
            ECAT_FOE_ERRCODE_NOTFOUND (error)
            ECAT_FOE_ERRCODE_ACCESS    (error)
            ECAT_FOE_ERRCODE_DISKFULL (error)
            ECAT_FOE_ERRCODE_ILLEGAL (error)
            ECAT_FOE_ERRCODE_PACKENO (error)
            ECAT_FOE_ERRCODE_EXISTS    (error)
            ECAT_FOE_ERRCODE_NOUSER    (error)

 \brief    The function starts a file reading

*////////////////////////////////////////////////////////////////////////////////////////
UINT16 AX58200_FoeRead(UINT16 MBXMEM * pName, UINT16 nameSize, UINT32 password, UINT16 maxBlockSize, UINT16 *pData)
{
	UINT16	size = 0;
	UINT16	i = 0;
	CHAR	aReadFileName[MAX_FILE_NAME_SIZE];

	/* ECATCHANGE_START(V5.11) FOE2*/
	if ((nameSize + 1) > MAX_FILE_NAME_SIZE)
	{
		return ECAT_FOE_ERRCODE_DISKFULL;
	}

	/*Read requested file name to endianess conversion if required*/
	MBXSTRCPY(aReadFileName, pName, nameSize);
	aReadFileName[nameSize] = '\0';
    
	/* ECATCHANGE_END(V5.11) FOE2*/
	{
		/* for test only the written file name can be read */
		for (i = 0; i < nameSize; i++)
		{
			if (aReadFileName[i] != aFileName[i])
			{
				/* file name not found */
				return ECAT_FOE_ERRCODE_NOTFOUND;
			}
		}

		if (nFileSize >= (u16SendMbxSize - SIZEOF(TFOEHEADER) - MBX_HEADER_SIZE))
		{
			size = (u16SendMbxSize - SIZEOF(TFOEHEADER) - MBX_HEADER_SIZE);
		}
		else
		{
			size = (unsigned short) nFileSize;
		}
		MBXMEMCPY(pData, aFileData, size);
	}
	return size;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     pName         Pointer to the name of the file
 \param     nameSize      Length of the file name
 \param     password      Password for the file read

 \return okay, busy or an error code
            0 (okay)
            ECAT_FOE_ERRCODE_NOTDEFINED (error)
            ECAT_FOE_ERRCODE_NOTFOUND (error)
            ECAT_FOE_ERRCODE_ACCESS    (error)
            ECAT_FOE_ERRCODE_DISKFULL (error)
            ECAT_FOE_ERRCODE_ILLEGAL (error)
            ECAT_FOE_ERRCODE_PACKENO (error)
            ECAT_FOE_ERRCODE_EXISTS    (error)
            ECAT_FOE_ERRCODE_NOUSER    (error)

            (no busy response shall be returned by this function.
            If the slave stack requires some time to handle the incoming data the function FOE_Data() should return busy)

 \brief    The function starts a file writing

*////////////////////////////////////////////////////////////////////////////////////////
UINT16 AX58200_FoeWrite(UINT16 MBXMEM * pName, UINT16 nameSize, UINT32 password)
{
    if ((nameSize >= SIZEOF(aFirmwareDownloadHeader))
        &&(pName[0] == aFirmwareDownloadHeader[0])
        &&(pName[1] == aFirmwareDownloadHeader[1])
        &&(pName[2] == aFirmwareDownloadHeader[2])
		&&(pName[3] == aFirmwareDownloadHeader[3]))
    {
        if (bBootMode)
        {
            BL_StartDownload(password);
            return 0;
        }
        else
        {
            return ECAT_FOE_ERRCODE_BOOTSTRAPONLY;
        }
    }
    else if (bBootMode)
    {
        return ECAT_FOE_ERRCODE_NOTINBOOTSTRAP;
    }
    else if (nameSize < MAX_FILE_NAME_SIZE)
    {
        /* for test every file name can be written */
        MBXSTRCPY(aFileName, pName, nameSize);
        MBXSTRCPY(aFileName+nameSize, "\0", 1); //string termination
		
        nFileWriteOffset = 0;
        nFileSize = 0;
        return 0;
    }

    return ECAT_FOE_ERRCODE_DISKFULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     pData         Received file data
 \param 	Size          Length of received file data

 \return okay, busy or an error code
            0 (okay)
            FOE_MAXBUSY-100 (0%)    (busy)
            FOE_MAXBUSY (100%) (busy)
            ECAT_FOE_ERRCODE_NOTDEFINED (error)
            ECAT_FOE_ERRCODE_NOTFOUND (error)
            ECAT_FOE_ERRCODE_ACCESS    (error)
            ECAT_FOE_ERRCODE_DISKFULL (error)
            ECAT_FOE_ERRCODE_ILLEGAL (error)
            ECAT_FOE_ERRCODE_PACKENO (error)
            ECAT_FOE_ERRCODE_EXISTS    (error)
            ECAT_FOE_ERRCODE_NOUSER    (error)

 \brief    The function is called to write the next part of a file

*////////////////////////////////////////////////////////////////////////////////////////
UINT16 AX58200_FoeWriteData(UINT16 MBXMEM * pData, UINT16 Size, BOOL bDataFollowing)
{
	if (bBootMode)
	{
		return BL_Data(pData, Size);
	}
	else if ((nFileWriteOffset + Size) > MAX_FILE_SIZE)
	{
		return ECAT_FOE_ERRCODE_DISKFULL;
	}

	if (Size)
	{ 
		MBXMEMCPY(&aFileData[(nFileWriteOffset >> 1)], pData, Size);
	}
    
	if (Size == (u16ReceiveMbxSize - MBX_HEADER_SIZE - FOE_HEADER_SIZE))
	{
		/* FoE-Data services will follow */
		nFileWriteOffset += Size;
		return 0;
	}
	else
	{
        /* last part of the file is written */
        nFileSize = nFileWriteOffset + Size;
        nFileWriteOffset = 0;
        return FOE_ACKFINISHED;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     fileOffset    Offset of the next file data to be sent
 \param     pData         Buffer for file data to be sent

 \return size of the data to be sent, busy or an error code
            0 (all data was sent before)
            1..MAX_FILE_SIZE (size of data to be sent)
            FOE_MAXBUSY-100 (0%)    (busy)
            FOE_MAXBUSY (100%) (busy)
            ECAT_FOE_ERRCODE_NOTDEFINED (error)
            ECAT_FOE_ERRCODE_NOTFOUND (error)
            ECAT_FOE_ERRCODE_ACCESS    (error)
            ECAT_FOE_ERRCODE_DISKFULL (error)
            ECAT_FOE_ERRCODE_ILLEGAL (error)
            ECAT_FOE_ERRCODE_PACKENO (error)
            ECAT_FOE_ERRCODE_EXISTS    (error)
            ECAT_FOE_ERRCODE_NOUSER    (error)

 \brief    The function is called when the reading of a part of a file is acknowledged

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 AX58200_FoeReadData(UINT32 offset, UINT16 maxBlockSize, UINT16 *pData)
{
	if (offset < nFileSize)
    {
        /* send next part of the file */
        UINT32 size;
        UINT32 sendSize = nFileSize - offset;

        if (sendSize >= (u16SendMbxSize - SIZEOF(TFOEHEADER) - MBX_HEADER_SIZE))
        {
            size = (u16SendMbxSize - SIZEOF(TFOEHEADER) - MBX_HEADER_SIZE);
        }
        else
        {
            size = sendSize;
        }

        MBXMEMCPY(pData, &aFileData[(offset >> 1)], size);

        return ((UINT16) size);
    }
    else if (offset == nFileSize)
    {
        /* file transfer is finished */
        return 0; // size = 0
    }
    return ECAT_FOE_ERRCODE_ILLEGAL;
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     done          Indicates how much of the busy action is done
 \param     fileOffset    Offset of the next file data to be sent
 \param     pData         Buffer for file data to be sent

 \return size of the data to be sent, busy or an error code
            0 (all data was sent before)
            1..MAX_FILE_SIZE (size of data to be sent)
            FOE_MAXBUSY-100 (0%)    (busy)
            FOE_MAXBUSY (100%) (busy)
            ECAT_FOE_ERRCODE_NOTDEFINED (error)
            ECAT_FOE_ERRCODE_NOTFOUND (error)
            ECAT_FOE_ERRCODE_ACCESS    (error)
            ECAT_FOE_ERRCODE_DISKFULL (error)
            ECAT_FOE_ERRCODE_ILLEGAL (error)
            ECAT_FOE_ERRCODE_PACKENO (error)
            ECAT_FOE_ERRCODE_EXISTS    (error)
            ECAT_FOE_ERRCODE_NOUSER    (error)

 \brief    The function is called when the reading of a part of a file should be repeated

*////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     errorCode    error code

 \brief    The function is called when a file transfer is aborted from the other station

*////////////////////////////////////////////////////////////////////////////////////////
void AX58200_FoeError(UINT32 errorCode)
{
	if (nFileWriteOffset)
	{
		/* no file is stored */
		memset(aFileName, 0, MAX_FILE_NAME_SIZE);
		nFileWriteOffset = 0;
		nFileSize = 0;
	}
}

void AX58200_FoeInit(void)
{
	/* Assign related foe function pointers */
	pAPPL_FoeRead = AX58200_FoeRead;
    pAPPL_FoeReadData = AX58200_FoeReadData;
    pAPPL_FoeError = AX58200_FoeError;
    pAPPL_FoeWrite = AX58200_FoeWrite;
    pAPPL_FoeWriteData = AX58200_FoeWriteData;
}
/** @} */

