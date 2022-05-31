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

#ifndef __CONSOLE_H__
#define __CONSOLE_H__

/* INCLUDE FILE DECLARATIONS */

/* NAMING CONSTANT DECLARATIONS */
#define CLI_AUTHEN_ENABLE 0
#define CLI_CMD_HISTORY_ENABLE 1
#define CLI_MAX_CMD_HISTORY_NUM	10

#define CLI_MAX_MESSAGE_LEN 512//128
#define CLI_MAX_COMMAND_LEN 512//128
#define CLI_MAX_CMD_LEN 24
#define CLI_MAX_USERNAME_LEN 16
#define CLI_MAX_PASSWD_LEN 16
#define CLI_MAX_PROMPT_LEN 16
#define CLI_MAX_CMD_BUF_LEN 100

enum LLID_Ind 
{
    LL_UART,
    LL_TELNET,
    LL_MAX
};

enum _CONSOLE_STATE_IND {
    CLI_STATE_BLOCK=0,
    CLI_STATE_LOGIN,
    CLI_STATE_PASSWD,
    CLI_STATE_COMMAND,
    CLI_STATE_COMMAND_WAIT,	
    CLI_STATE_PASSWD_CHANGE1,
    CLI_STATE_PASSWD_CHANGE2,
    CLI_STATE_PASSWD_CHANGE3,
    CLI_STATE_MAX
};

/* TYPE DECLARATIONS */
#if (CLI_CMD_HISTORY_ENABLE)
typedef struct
{
	unsigned char AddIndex;
	unsigned char GetIndex;
	char Buf[CLI_MAX_CMD_HISTORY_NUM][CLI_MAX_COMMAND_LEN];
} CONSOLE_CmdHistory;
#endif

 // Add for linked list algorithm //
struct _CLILINK
{
	struct _CLILINK *pPre;
	struct _CLILINK *pNext;
	unsigned short WaitTime;
	unsigned short ReplyLen;
	unsigned char Buf[CLI_MAX_CMD_BUF_LEN];
};
typedef struct _CLILINK tsCLILINK;

typedef struct
{
    unsigned char (*PutChar)(unsigned char c); /* write one character */
    char (*GetChar)(void);   /* read one character */  
    unsigned char Privilege;
    unsigned char State;
    unsigned char PromptEnable;
    unsigned char LowLayerId;
    unsigned short BufIndex;
    unsigned short CursorPosition;	
    unsigned short Argc;
    char **Argv;
    char CmdBuf[CLI_MAX_COMMAND_LEN];
    char UserName[CLI_MAX_USERNAME_LEN];
    char Passwd[CLI_MAX_PASSWD_LEN];
    char PasswdNew[CLI_MAX_PASSWD_LEN];
    char PromptStr[CLI_MAX_PROMPT_LEN];
	unsigned char CmdId;
	tsCLILINK Cmd; // Add for linked list algorithm //
#if (CLI_CMD_HISTORY_ENABLE)
    CONSOLE_CmdHistory CmdHistory;
#endif
	void *pCmdTable;
	unsigned short CmdTableSize;	
	unsigned short Timer;		
} CONSOLE_Inst;

typedef int (*CmdPtr)(CONSOLE_Inst *pInst, int argc, char **argv);
typedef int (*HelpPtr)(CONSOLE_Inst *pInst);

typedef struct
{
    char Cmd[CLI_MAX_CMD_LEN];
    CmdPtr CmdFunc;
    HelpPtr Help;
    unsigned char Level;
} CONSOLE_CmdEntry;

typedef struct
{
    char Name[CLI_MAX_USERNAME_LEN];
    char Passwd[CLI_MAX_PASSWD_LEN];
    unsigned char Level;
} CONSOLE_Account;

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */ 
void CONSOLE_Init(void);
void CONSOLE_Task(void);
short CONSOLE_PutMessage(CONSOLE_Inst *pInst, char *fmt, ...);
short CONSOLE_ChangeUsername(CONSOLE_Inst *pInst, unsigned char *username);
#if (INCLUDE_TELNET_SERVER)
CONSOLE_Inst *CONSOLE_GetInstance(unsigned char id);
#endif
short CONSOLE_TimeTick(void);

#endif /* __CONSOLE_H__ */

/* End of console.h */
