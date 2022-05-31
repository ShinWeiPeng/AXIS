/*
* This source file is part of the EtherCAT Slave Stack Code licensed by Beckhoff Automation GmbH & Co KG, 33415 Verl, Germany.
* The corresponding license agreement applies. This hint shall not be removed.
*/

/**
 * \addtogroup ESM EtherCAT State Machine
 * @{
 */

/**
\file bootmode.h
\author EthercatSSC@beckhoff.com
\brief Function prototypes for the Boot state

\version 5.11.1

<br>Changes to version V5.01:<br>
V5.11 ECAT10: change PROTO handling to prevent compiler errors<br>
<br>Changes to version - :<br>
V5.01 : Start file change log
 */

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecat_def.h"


#ifndef _BOOTMODE_H_
#define _BOOTMODE_H_

/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/

#endif //_BOOTMODE_H_

#if defined(_BOOTMODE_) && (_BOOTMODE_ == 1)
    #define PROTO
#else
    #define PROTO extern
#endif
typedef enum{
	BL_STATE_IDLE = 0,
	BL_STATE_START,
	BL_STATE_START_DOWNLOAD,
	BL_STATE_DATA,
	BL_STATE_FW_TRANSMIT_DONE,
   BL_STATE_PARAMETER_TRANSMIT_DONE,
} BL_STATE;
 /*-----------------------------------------------------------------------------------------
------
------    Global variables
------
-----------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------
------    Global functions
------
-----------------------------------------------------------------------------------------*/
PROTO    void BL_Start( UINT8 State);
PROTO   void BL_Stop(void);
PROTO    void BL_StartDownload(UINT32 password);
PROTO	UINT16 BL_Data(UINT16 *pData,UINT16 Size);
PROTO	void BL_Init(void);
PROTO	void BL_JumpToLdrom(void);



#undef PROTO
/** @}*/
