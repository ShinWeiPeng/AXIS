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
#include "NuMicro.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "ax_uart.h"
#include "console.h"
#include "clicmd.h"
#if (INCLUDE_TELNET_SERVER)
#include "telnet.h"
#endif

/* NAMING CONSTANT DECLARATIONS */
#if (INCLUDE_TELNET_SERVER)
#define MAX_INST_NUM 2
#else
#define MAX_INST_NUM 1
#endif
#define USERNAME_STR "username: "
#define PASSWD_STR   "password: "
#define CONSOLE_QUIT "quit"
#define CONSOLE_HELP "help"
#define CONSOLE_PASSWD "passwd"
#define CONSOLE_REBOOT "reboot"
#define CONSOLE_PROMPT "uart> "

#define CTRL_C  0x03
#define BACKSP  0x08
#define DELCHAR 0x7F
#define SPACE   0x20
#define BELL    0x07
#define CLI_CR  0x0d
#define CLI_LF  0x0a
#define CLI_ECHO_ON "\010 \010"
#define CLI_MAX_ARGS 30
#define CLI_CLOSED -1
#define CLI_COMMAND_DONE 10
#define ESCAPE	0x1B
/* MACRO DECLARATIONS */
#define IsSpace(x) ((x == SPACE) ? 1 : 0)

/* GLOBAL VARIABLES DECLARATIONS */

/* LOCAL VARIABLES DECLARATIONS */
static CONSOLE_Inst console_Instance[MAX_INST_NUM];
static char pBuf[CLI_MAX_MESSAGE_LEN];

/* LOCAL SUBPROGRAM DECLARATIONS */
static short console_PutString(CONSOLE_Inst *pInst, char *strData, short len);
static void console_ClearLine(CONSOLE_Inst *pInst);
static void console_HelpMessage(CONSOLE_Inst *pInst, unsigned char privilege);
static void console_ParseLine(char *pLine, short numArgvs, short *pArgc, char *argv[]);
static short console_CollectCommand(CONSOLE_Inst *pInst, char buf[], short size, short echo);
static short console_Authenticate(CONSOLE_Inst *pInst);
static short console_CommandProcess(CONSOLE_Inst *pInst, char *pLine, unsigned char privilege);
static void console_CmdExecute(CONSOLE_Inst *pInst, short argc, char *argv[], unsigned char privilege);
static void console_LoadUserAccount(void);
static short console_ChangePasswd(CONSOLE_Inst *pInst);
#if (CLI_CMD_HISTORY_ENABLE)
static unsigned char console_EscapeSequenceProcess(CONSOLE_Inst *pInst, char buf[]);
static unsigned char console_AddCommandHistory(CONSOLE_Inst *pInst, char buf[]);
static unsigned char console_GetCommandHistory(CONSOLE_Inst *pInst, char buf[], unsigned char indexinc);
#endif
/* LOCAL SUBPROGRAM BODIES */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CONSOLE_ChangeUsername()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
short CONSOLE_ChangeUsername(CONSOLE_Inst *pInst, unsigned char *username)
{
    CONSOLE_Account *account;
    unsigned char ulen, plen, i;

    ulen = strlen((char *)pInst->UserName);
    plen = strlen((char *)username);

    for (i=0, account=&CLICMD_userTable[0]; i < MAX_USER_ACCOUNT; i++, account++)
    {
        if ((ulen == strlen(account->Name)) && (memcmp(account->Name, pInst->UserName, ulen)==0))
        {
            strcpy(account->Name, (char *)username);
            strcpy(pInst->UserName, (char *)username);
            /* Update the configuration */
            return 1;
        }
    }

    return -1;
} /* End of CONSOLE_ChangeUsername() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_ChangePasswd()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static short console_ChangePasswd(CONSOLE_Inst *pInst)
{
    CONSOLE_Account *account;
    unsigned char ulen, plen, i;

    ulen = strlen(pInst->UserName);
    plen = strlen(pInst->Passwd);

    for (i=0, account=&CLICMD_userTable[0]; i < MAX_USER_ACCOUNT; i++, account++)
    {
        if ((ulen == strlen(account->Name)) &&
            (memcmp(account->Name, pInst->UserName, ulen)==0) &&
            (plen == strlen(account->Passwd)) &&
            (memcmp(account->Passwd, pInst->Passwd, plen)==0))
        {
            ulen = strlen(pInst->PasswdNew);
            plen = strlen(pInst->CmdBuf);
       	    if ((ulen == plen) && (memcmp(pInst->PasswdNew, pInst->CmdBuf, ulen)==0))
            {
                strcpy(account->Passwd, pInst->PasswdNew);
                /* Update the configuration */

                return 1;
            }
            else
                return -1;
        }
    }

    return -1;
} /* End of console_ChangePasswd() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_LoadUserAccount()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void console_LoadUserAccount(void)
{
    strcpy(CLICMD_userTable[0].Name, "admin");
    strcpy(CLICMD_userTable[0].Passwd, "admin");
    CLICMD_userTable[0].Level = 5;
} /* End of console_LoadUserAccount() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_PutString()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static short console_PutString(CONSOLE_Inst *pInst, char *strData, short len)
{
    char *pStr=strData;

    if (!pStr)
        return -1;

    while (len-->0)
    {
        pInst->PutChar(*pStr);
        pStr++;
    }

    return 1;
} /* End of console_PutString() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_ParseLine()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void console_ParseLine(char *pLine, short numArgvs, short *pArgc, char *argv[])
{
    short argc;

    /* for each argument */
    for (argc = 0; argc < (numArgvs - 1); argc++)
    {
        /* Skip leading white space */
        while (IsSpace(*pLine))
        {
            pLine++;
        }

        /* if end of line. */
        if( *pLine == '\0')
        {
            break;
        }

        /* record the start of the argument */
        argv[argc] = pLine;

        /* find the end of the argument */
        while (*pLine != '\0' && !IsSpace(*pLine))
        {
            pLine++;
        }

        /* null terminate argument */
        if (*pLine != '\0')
        {
            *pLine = '\0';
            pLine++;
        }
    }

    /* null terminate list of arguments */
    argv[argc] = 0;
    *pArgc = argc;
} /* End of console_ParseLine() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_HelpMessage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void console_HelpMessage(CONSOLE_Inst *pInst, unsigned char privilege)
{
    CONSOLE_CmdEntry *pCmd;
    unsigned char i;

    CONSOLE_PutMessage(pInst, "help\r\n");
//    CONSOLE_PutMessage(pInst, "quit\r\n");
    CONSOLE_PutMessage(pInst, "reboot\r\n");
#if CLI_AUTHEN_ENABLE
    CONSOLE_PutMessage(pInst, "Usage: passwd\r\n");
    CONSOLE_PutMessage(pInst, "       Old Password: \r\n");
    CONSOLE_PutMessage(pInst, "       New Password: \r\n");
    CONSOLE_PutMessage(pInst, "       Re-enter New Password: \r\n");
#endif
    for (i = 0, pCmd = (CONSOLE_CmdEntry*)pInst->pCmdTable; i < pInst->CmdTableSize; i++, pCmd++)
    {
        if (pCmd->Level >= privilege && pCmd->Help)
        {
            pCmd->Help(pInst);
        }
    }
} /* End of console_HelpMessage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_Authenticate()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static short console_Authenticate(CONSOLE_Inst *pInst)
{
    CONSOLE_Account *account;
    unsigned char ulen, plen, i;

    ulen = strlen(pInst->UserName);
    plen = strlen(pInst->Passwd);

    for (i=0, account=&CLICMD_userTable[0]; i < MAX_USER_ACCOUNT; i++, account++)
    {
        if ((ulen == strlen(account->Name)) &&
            (memcmp(account->Name, pInst->UserName, ulen)==0) &&
            (plen == strlen(account->Passwd)) &&
            (memcmp(account->Passwd, pInst->Passwd, plen)==0))
        {
#if (INCLUDE_TELNET_SERVER)
       	    if (pInst->LowLayerId == LL_TELNET )
    	        CONSOLE_PutMessage(pInst, "Successful login through telnet\r\n");
#endif
      	    pInst->Privilege = account->Level;
            return 1;
        }
    }

    return -1;
} /* End of console_Authenticate() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_CommandProcess()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static short console_CommandProcess(CONSOLE_Inst *pInst, char *pLine, unsigned char privilege)
{
    char *argv[CLI_MAX_ARGS+1];
    short argc;

    if (!strncmp(pLine, CONSOLE_HELP, strlen(CONSOLE_HELP)))
    {
        console_HelpMessage(pInst, privilege);
        return 1;
    }
#if (CLI_AUTHEN_ENABLE)
    else if (!strncmp(pLine, CONSOLE_PASSWD, strlen(CONSOLE_PASSWD)))
    {
        pInst->State = CLI_STATE_PASSWD_CHANGE1;
        return 1;
    }
    else if (!strncmp(pLine, CONSOLE_QUIT, strlen(CONSOLE_QUIT)))
    {
        return -1;
    }
#endif
    console_ParseLine(pLine, (sizeof(argv)/sizeof(argv[0])), &argc, argv);
    console_CmdExecute(pInst, argc, argv, privilege);

    return 1;
} /* End of console_CommandProcess() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_CmdExecute()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void console_CmdExecute(CONSOLE_Inst *pInst, short argc, char* argv[], unsigned char privilege)
{
    unsigned char i;
    CONSOLE_CmdEntry *pCmd;

    if (argc < 1)
    {
        return;
    }

    /* check the command in command list */
    for (i = 0, pCmd = (CONSOLE_CmdEntry*)pInst->pCmdTable; i < pInst->CmdTableSize; i++, pCmd++)
    {
        if (strcmp(argv[0], pCmd->Cmd) == 0)
        {
            if (pCmd->Level >= privilege)
            {
                if (pCmd->CmdFunc == NULL)
                {
                     return;
                }

                pInst->Argc = argc;
                pInst->Argv = (char **)&argv[0];
                if (pCmd->CmdFunc(pInst, pInst->Argc, pInst->Argv) > 0)
                {
										if (pInst->State != CLI_STATE_COMMAND_WAIT)
                    CONSOLE_PutMessage(pInst, "Ok\r\n");
                }
                else
                {
                    CONSOLE_PutMessage(pInst, "Error\r\n");
                }
                return;
            }
            else
                break;
       }
    }

    CONSOLE_PutMessage(pInst, "Unknown command %s\r\n", argv[0]);
    return;
}  /* End of console_CmdExecute() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_CollectCommand()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static short console_CollectCommand(CONSOLE_Inst *pInst, char buf[], short size, short echo)
{
    short c;
    char tmp;

    c = pInst->GetChar();

    switch (c)
    {
        case CLI_CLOSED :  /* telnet quit */
            return -1;

        case CTRL_C:
            return -5;  /* user abort, return -1, means nothing */

        case BACKSP:
        case DELCHAR:
            if (pInst->BufIndex == 0 || (pInst->CursorPosition < pInst->BufIndex))
            {
                tmp = BELL;
                pInst->PutChar(tmp);
                return 0;
            }
            /* if echo flag do back space */
            if(echo)
                console_PutString(pInst, CLI_ECHO_ON, strlen(CLI_ECHO_ON));

            if( pInst->BufIndex != 0 )  /* get rid of one char from buf */
                pInst->BufIndex -= 1;

            break;

        case 21:
            /* ^U */
            console_ClearLine(pInst);
            return 0;

        case '\r':
            buf[pInst->BufIndex] = '\0';  /* NULL char would not be counted */
            pInst->CursorPosition = pInst->BufIndex;
#if (CLI_CMD_HISTORY_ENABLE)
			console_AddCommandHistory(pInst, buf);
#endif
            tmp = '\r';
            pInst->PutChar(tmp);
            tmp = '\n';
            pInst->PutChar(tmp);
            return CLI_COMMAND_DONE;

        case '\n':
#if (INCLUDE_TELNET_SERVER)
            if (pInst->LowLayerId == LL_TELNET)
            {
                buf[pInst->BufIndex] = '\0';  /* NULL char would not be counted */
				pInst->CursorPosition = pInst->BufIndex;
#if (CLI_CMD_HISTORY_ENABLE)
				console_AddCommandHistory(pInst, buf);
#endif
                tmp = '\r';
                pInst->PutChar(tmp);
                tmp = '\n';
                pInst->PutChar(tmp);
                return CLI_COMMAND_DONE;
            }
            else
#endif
                return 0;
        default:
	            if ( c < SPACE || c >= 0x80) /* if want printable ascii only */
                return 0;

 			if (pInst->BufIndex > pInst->CursorPosition)
			{
                buf[pInst->CursorPosition] = c;
                pInst->CursorPosition++;
                tmp = (char)c;
                if (echo)
                    pInst->PutChar(tmp); /* for passwd no echo */
			}
            else if (pInst->BufIndex <= (size - 1))
            {
                buf[pInst->BufIndex] = c;
                pInst->BufIndex++;
				pInst->CursorPosition = pInst->BufIndex;
                tmp = (char)c;
                if (echo)
                    pInst->PutChar(tmp); /* for passwd no echo */
                if (pInst->BufIndex == size)
                     return CLI_COMMAND_DONE;
            }
            else
            {
                tmp = BELL;                   /* buffer full, but no cr/lf */
                pInst->PutChar(tmp); /* send bell, notify user */
            }
            break;
#if (CLI_CMD_HISTORY_ENABLE)
		case ESCAPE:
			console_EscapeSequenceProcess(pInst, buf);
		break;
#endif
    }

    return 1;
} /* End of console_CollectCommand() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_ClearLine()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void console_ClearLine(CONSOLE_Inst *pInst)
{
    pBuf[0] = '\r';
    memset(&pBuf[1], ' ', 79);
    pBuf[80] = '\r';
    console_PutString(pInst, pBuf, 81);

} /* End of console_ClearLine() */

#if (CLI_CMD_HISTORY_ENABLE)
/*
 * ----------------------------------------------------------------------------
 * Function Name: console_EscapeSequenceProcess()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static unsigned char console_EscapeSequenceProcess(CONSOLE_Inst *pInst, char buf[])
{
	unsigned short len;
	unsigned char escseq[4];

	escseq[0] = ESCAPE;

	/* get second character */
	do {
		escseq[1] = pInst->GetChar();
	} while (!escseq[1]);

	if (escseq[1] == '[')/* for Control Sequence Introducer(CSI) */
	{
		/* get third character */
		do {
			escseq[2] = pInst->GetChar();
		} while (!escseq[2]);

		if (escseq[2] == 'A')/* if up arrow key has pressed */
		{
			len = console_GetCommandHistory(pInst, buf, 1);
			if (len)/* get command string and increase the command index */
			{
				while (pInst->CursorPosition < pInst->BufIndex)
				{
					console_PutString(pInst, " ", 1);
					pInst->CursorPosition++;
				}

				/* clear command */
				while (pInst->BufIndex--)
   					console_PutString(pInst, CLI_ECHO_ON, strlen(CLI_ECHO_ON));

				/* shown selected command from history */
				pInst->BufIndex = len;
				pInst->CursorPosition = pInst->BufIndex;
				console_PutString(pInst, buf, pInst->BufIndex);
			}
		}
		else if (escseq[2] == 'B')/* down arrow key has pressed */
		{
			len = console_GetCommandHistory(pInst, buf, 0);
			if (len)/* get command string and decrease the command index */
			{
				while (pInst->CursorPosition < pInst->BufIndex)
				{
					console_PutString(pInst, " ", 1);
					pInst->CursorPosition++;
				}

				/* clear command */
				while (pInst->BufIndex--)
   					console_PutString(pInst, CLI_ECHO_ON, strlen(CLI_ECHO_ON));

				/* shown selected command from history */
				pInst->BufIndex = len;
				pInst->CursorPosition = pInst->BufIndex;
				console_PutString(pInst, buf, pInst->BufIndex);
			}
		}
		else if (escseq[2] == 'C')/* right arrow key has pressed */
		{
			if (pInst->CursorPosition < pInst->BufIndex)
			{
				console_PutString(pInst, &buf[pInst->CursorPosition], 1);
				pInst->CursorPosition ++;
			}
		}
		else if (escseq[2] == 'D')/* left arrow key has pressed */
		{
			if (pInst->CursorPosition)
			{
				pInst->CursorPosition --;
				console_PutString(pInst, "\b", strlen("\b"));
			}
		}
		return 1;
	}

	return 0;
} /* End of console_EscapeSequenceProcess() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_AddCommandHistory()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static unsigned char console_AddCommandHistory(CONSOLE_Inst *pInst, char buf[])
{
	unsigned char i, tmp;

	/* ignore the invalid command string */
	if (buf[0] == '\0')
		return 0;

	/* add the new command string into command history table */
	for (tmp=0; tmp<CLI_MAX_CMD_HISTORY_NUM; tmp++)
	{
		if (pInst->CmdHistory.Buf[tmp][0] != '\0' && !strcmp(pInst->CmdHistory.Buf[tmp], buf))
			break;
	}

	if (tmp != CLI_MAX_CMD_HISTORY_NUM)
	{
		pInst->CmdHistory.GetIndex = tmp ? (tmp-1):(CLI_MAX_CMD_HISTORY_NUM-1);
		return 0;
	}

	i = pInst->CmdHistory.AddIndex;
	strcpy(pInst->CmdHistory.Buf[i], buf);

	/* change the index of get command history in order point to latest added command string */
	pInst->CmdHistory.GetIndex = i ? (i-1):(CLI_MAX_CMD_HISTORY_NUM-1);

	if (i == (CLI_MAX_CMD_HISTORY_NUM-1))
		i = 0;
	else
		i++;

	pInst->CmdHistory.AddIndex = i;

	return 1;
} /* End of console_AddCommandHistory() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: console_GetCommandHistory()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static unsigned char console_GetCommandHistory(CONSOLE_Inst *pInst, char buf[], unsigned char indexinc)
{
	unsigned char i, j;

	/* get command string in order from command history table by index */
	i = pInst->CmdHistory.GetIndex;

	for (j=0; j<CLI_MAX_CMD_HISTORY_NUM; j++)
	{
		if (indexinc)/* index increment direction */
		{
			if (i == (CLI_MAX_CMD_HISTORY_NUM-1))
				i = 0;
			else
				i++;
		}
		else		/* index decrement direction */
		{
			if (i == 0)
				i = (CLI_MAX_CMD_HISTORY_NUM-1);
			else
				i--;

		}

		if (pInst->CmdHistory.Buf[i][0] != '\0')
		{
			strcpy(buf, pInst->CmdHistory.Buf[i]);
			break;
		}
	}

	pInst->CmdHistory.GetIndex = i;

	if (j == CLI_MAX_CMD_HISTORY_NUM)/* the command history table is emptied */
		return 0;
	else
		return (strlen(pInst->CmdHistory.Buf[i]));
} /* End of console_GetCommandHistory() */
#endif
/* EXPORTED SUBPROGRAM BODIES */

#if (INCLUDE_TELNET_SERVER)
/*
 * ----------------------------------------------------------------------------
 * Function Name: CONSOLE_GetInstance()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
CONSOLE_Inst *CONSOLE_GetInstance(unsigned char id)
{
    unsigned char i;

    for (i= 0; i < MAX_INST_NUM; i++)
    {
        if (console_Instance[i].LowLayerId == id)
            return (CONSOLE_Inst *)&console_Instance[i];
    }

    return (CONSOLE_Inst *)0;

} /* End of CONSOLE_GetInstance() */
#endif

/*
 * ----------------------------------------------------------------------------
 * Function Name: CONSOLE_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void CONSOLE_Init(void)
{
    CONSOLE_Inst *pInst;

    AX_UART_Init();

    pInst = &console_Instance[LL_UART];
    pInst->State = CLI_STATE_LOGIN;
    pInst->PromptEnable = 1;
    pInst->Privilege = 5;
    pInst->BufIndex = 0;
    pInst->CursorPosition = 0;
    pInst->GetChar = AX_UART_GetChar;
    pInst->PutChar = AX_UART_PutChar;
    pInst->LowLayerId = LL_UART;
    memset(pInst->UserName, 0, CLI_MAX_USERNAME_LEN);
    memset(pInst->Passwd, 0, CLI_MAX_PASSWD_LEN);
    memset(pInst->CmdBuf, 0, CLI_MAX_COMMAND_LEN);
    strcpy(pInst->PromptStr, CONSOLE_PROMPT);
    console_LoadUserAccount();
#if (CLI_CMD_HISTORY_ENABLE)
	pInst->CmdHistory.AddIndex = 0;
	pInst->CmdHistory.GetIndex = 0;
	for (pBuf[0]=0; pBuf[0]<CLI_MAX_CMD_HISTORY_NUM; pBuf[0]++)
		pInst->CmdHistory.Buf[pBuf[0]][0] = '\0';
#endif
	pInst->pCmdTable = CLICMD_GetCmdTable();
	pInst->CmdTableSize = CLICMD_GetCmdTableSize();

#if (INCLUDE_TELNET_SERVER)
    pInst = &console_Instance[LL_TELNET];
    pInst->State = CLI_STATE_BLOCK;
    pInst->PromptEnable = 1;
    pInst->Privilege = 5;
    pInst->BufIndex = 0;
    pInst->CursorPosition = 0;
    pInst->GetChar = TELNET_GetChar;
    pInst->PutChar = TELNET_PutChar;
    pInst->LowLayerId = LL_TELNET;
    memset(pInst->UserName, 0, CLI_MAX_USERNAME_LEN);
    memset(pInst->Passwd, 0, CLI_MAX_PASSWD_LEN);
    memset(pInst->CmdBuf, 0, CLI_MAX_COMMAND_LEN);
    strcpy(pInst->PromptStr, "telnet> ");
#if (CLI_CMD_HISTORY_ENABLE)
	pInst->CmdHistory.AddIndex = 0;
	pInst->CmdHistory.GetIndex = 0;
	for (pBuf[0]=0; pBuf[0]<CLI_MAX_CMD_HISTORY_NUM; pBuf[0]++)
		pInst->CmdHistory.Buf[pBuf[0]][0] = '\0';
#endif
	pInst->pCmdTable = CLICMD_GetCmdTable();
	pInst->CmdTableSize = CLICMD_GetCmdTableSize();

#endif

} /* End of CONSOLE_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CONSOLE_Task
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void CONSOLE_Task(void)
{
    CONSOLE_Inst *pInst;
    short retCode;
    unsigned char i;

    for (i = 0; i < MAX_INST_NUM; i++)
    {
        retCode=0;
        pInst = &console_Instance[i];

        switch(pInst->State)
        {
            case CLI_STATE_LOGIN:
#if (!CLI_AUTHEN_ENABLE)
                pInst->State = CLI_STATE_COMMAND;
#else
                if (pInst->PromptEnable == 1)
                {
                    CONSOLE_PutMessage(pInst, "%s", USERNAME_STR);
                    pInst->PromptEnable = 0;
                    pInst->BufIndex = 0;
                    pInst->CursorPosition = pInst->BufIndex;
                    memset(pInst->UserName, 0, CLI_MAX_USERNAME_LEN);
                }
                retCode = console_CollectCommand(pInst, pInst->UserName, CLI_MAX_USERNAME_LEN, 1);
                if (retCode == CLI_COMMAND_DONE)
                {
                    pInst->PromptEnable = 1;
                    pInst->State = CLI_STATE_PASSWD;
                }
                else if (retCode < 0)
                {
                    pInst->PromptEnable = 1;
#if (INCLUDE_TELNET_SERVER)
                    if (pInst->LowLayerId == LL_TELNET)
                    {
                        pInst->State = CLI_STATE_BLOCK;
                        /* Notify telnet to close connection */
                        TELNET_NotifyClose();
                    }
#endif
                }
#endif
                break;
            case CLI_STATE_PASSWD:
                if (pInst->PromptEnable == 1)
                {
                    CONSOLE_PutMessage(pInst, "%s", PASSWD_STR);
                    memset(pInst->Passwd, 0, CLI_MAX_PASSWD_LEN);
                    pInst->PromptEnable = 0;
                    pInst->BufIndex = 0;
                    pInst->CursorPosition = pInst->BufIndex;
                }
                retCode = console_CollectCommand(pInst, pInst->Passwd, CLI_MAX_PASSWD_LEN, 0);
                if (retCode == CLI_COMMAND_DONE)
                {
                    pInst->PromptEnable = 1;
                    if (console_Authenticate(pInst) > 0)
                    {
                        pInst->State = CLI_STATE_COMMAND;
                    }
                    else
                    {
#if (INCLUDE_EVENT_DETECT)
						GEVENT_SetAuthFailEvent(1);//*** Add for authentication fail detect ***
#endif
                        CONSOLE_PutMessage(pInst, "Authentication failed!\r\n\r\n");
                        pInst->State = CLI_STATE_LOGIN;
                    }
                }
                else if (retCode < 0)
                {
                    pInst->PromptEnable = 1;
#if (INCLUDE_TELNET_SERVER)
                    if (pInst->LowLayerId == LL_TELNET)
                    {
                        pInst->State = CLI_STATE_BLOCK;
                        /* Notify telnet to close connection */
                        TELNET_NotifyClose();
                    }
                    else
#endif
                        pInst->State = CLI_STATE_LOGIN;
                }
                break;
            case CLI_STATE_COMMAND:
                if (pInst->PromptEnable == 1)
                {
                    CONSOLE_PutMessage(pInst, "%s", pInst->PromptStr);
                    memset(pInst->CmdBuf, 0, CLI_MAX_COMMAND_LEN);
                    pInst->PromptEnable = 0;
                    pInst->BufIndex = 0;
                    pInst->CursorPosition = pInst->BufIndex;
                }

                retCode = console_CollectCommand(pInst, pInst->CmdBuf, CLI_MAX_COMMAND_LEN, 1);
                if (retCode == CLI_COMMAND_DONE)
                {
                    pInst->PromptEnable = 1;
                    if (console_CommandProcess(pInst, pInst->CmdBuf, pInst->Privilege) < 0)
                    {
#if (INCLUDE_TELNET_SERVER)
                        if (pInst->LowLayerId == LL_TELNET)
                        {
                            pInst->State = CLI_STATE_BLOCK;
                            /* Notify telnet to close connection */
                            TELNET_NotifyClose();
                        }
                        else
#endif
                        {
                            pInst->State = CLI_STATE_LOGIN;
                        }
                    }
                }
                else if (retCode < 0)
                {
                    pInst->PromptEnable = 1;
#if (INCLUDE_TELNET_SERVER)
                    if (pInst->LowLayerId == LL_TELNET)
                    {
                        pInst->State = CLI_STATE_BLOCK;
                        /* Notify telnet to close connection */
                        TELNET_NotifyClose();
                    }
                    else
#endif
                        pInst->State = CLI_STATE_LOGIN;
                }
                break;
				/* for task type command */
			case CLI_STATE_COMMAND_WAIT:
				console_CommandProcess(pInst, pInst->CmdBuf, pInst->Privilege);

				break;
            case CLI_STATE_BLOCK:
                /* instance has been block */
                break;
            case CLI_STATE_PASSWD_CHANGE1:
                if (pInst->PromptEnable == 1)
                {
                    CONSOLE_PutMessage(pInst, "Old Password: ");
                    memset(pInst->Passwd, 0, CLI_MAX_PASSWD_LEN);
                    pInst->PromptEnable = 0;
                    pInst->BufIndex = 0;
                    pInst->CursorPosition = pInst->BufIndex;
                }
                retCode = console_CollectCommand(pInst, pInst->Passwd, CLI_MAX_PASSWD_LEN, 0);
                if (retCode == CLI_COMMAND_DONE)
                {
                    pInst->PromptEnable = 1;
                    pInst->State = CLI_STATE_PASSWD_CHANGE2;
                }
                else if (retCode < 0)
                {
                    pInst->PromptEnable = 1;
                    pInst->State = CLI_STATE_COMMAND;
                }
                break;

            case CLI_STATE_PASSWD_CHANGE2:
                if (pInst->PromptEnable == 1)
                {
                    CONSOLE_PutMessage(pInst, "New Password: ");
                    memset(pInst->PasswdNew, 0, CLI_MAX_PASSWD_LEN);
                    pInst->PromptEnable = 0;
                    pInst->BufIndex = 0;
                    pInst->CursorPosition = pInst->BufIndex;
                }
                retCode = console_CollectCommand(pInst, pInst->PasswdNew, CLI_MAX_PASSWD_LEN, 0);
                if (retCode == CLI_COMMAND_DONE)
                {
                    pInst->PromptEnable = 1;
                    pInst->State = CLI_STATE_PASSWD_CHANGE3;
                }
                else if (retCode < 0)
                {
                    pInst->PromptEnable = 1;
                    pInst->State = CLI_STATE_COMMAND;
                }
                break;

            case CLI_STATE_PASSWD_CHANGE3:
                if (pInst->PromptEnable == 1)
                {
                    CONSOLE_PutMessage(pInst, "Re-enter New Password: ");
                    memset(pInst->CmdBuf, 0, CLI_MAX_PASSWD_LEN);
                    pInst->PromptEnable = 0;
                    pInst->BufIndex = 0;
                    pInst->CursorPosition = pInst->BufIndex;
                }
                retCode = console_CollectCommand(pInst, pInst->CmdBuf, CLI_MAX_PASSWD_LEN, 0);
                if (retCode == CLI_COMMAND_DONE)
                {
                    pInst->PromptEnable = 1;
                    if (console_ChangePasswd(pInst) < 0)
                    {
                        CONSOLE_PutMessage(pInst, "Failed to change password\r\n");
                    }
                    else
                    {
                        CONSOLE_PutMessage(pInst, "Success to change password\r\n");
                    }
                    pInst->State = CLI_STATE_COMMAND;
                }
                else if (retCode < 0)
                {
                    pInst->PromptEnable = 1;
                    pInst->State = CLI_STATE_COMMAND;
                }
                break;
            default:
                CONSOLE_PutMessage(pInst, "Unknow state was detected, reset to LOGIN state\r\n");
                pInst->PromptEnable = 1;
                pInst->State = CLI_STATE_LOGIN;
                break;
        } /* switch */
    } /* for */

} /* End of CONSOLE_Task() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CONSOLE_PutMessage()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
short CONSOLE_PutMessage(CONSOLE_Inst *pInst, char *fmt, ...)
{
    short ret;
    va_list args;

    va_start(args, fmt);
    ret = vsprintf(pBuf,fmt,args);  /* process fmt & args into buf */
    console_PutString(pInst, pBuf, ret);
    va_end(args);

    return ret;
} /* End of CONSOLE_PutMessage() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: CONSOLE_TimeTick()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
short CONSOLE_TimeTick(void)
{
	uint8_t i;

	for (i=0; i<MAX_INST_NUM; i++)
	{
		if (console_Instance[i].Timer)
		{
			console_Instance[i].Timer--;
		}
	}

	return 1;
} /* End of CONSOLE_TimeTick() */

/* End of console.c */
