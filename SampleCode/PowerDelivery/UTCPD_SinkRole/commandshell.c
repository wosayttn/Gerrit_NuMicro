/**************************************************************************//**
 * @file     commandshell.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 18/07/16 10:25a $
 * @brief
 *           Transmit and receive data from PC hyper-terminal through RS232 interface.
 * @note
 *
 * Copyright (c) 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include "NuMicro.h"
#include <string.h>
#include "utcpdlib.h"

#define RXBUFSIZE   32

#if (CONFIG_COMMAND_SHELL == 1)
extern void pd_backup_snk_pdo(int port);
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;

    if (UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))
    {
        //printf("\nInput:");

        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0))
        {
            /* Push char to buffer with "g_u32comRtail" pointer */
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);

            printf("%c", u8InChar);
            while( UART_GET_TX_EMPTY(UART0) == 0);

            if(u8InChar == 0x0D)
            {
                g_bWait = FALSE;
                u8InChar = 0;         /* Change 0x0D to 0x0 (NULL) */
            }

            /* Check if buffer full */
            if(g_u32comRbytes < (RXBUFSIZE - 1))
            {
                /* Enqueue the character */
                if(u8InChar != 0x08)
                {
                    g_u8RecData[g_u32comRtail] = u8InChar;
                    g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                    g_u32comRbytes++;
                }
                else
                {
                    if(g_u32comRtail != 0)
                        g_u32comRtail--;
                }

            }
            else
            {
                /* Input char over buffer size, error recovery,  */
                g_bWait = FALSE;
                break;
            }
        }
        //printf("\nTransmission Test:");
    }
#if 0
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_THREINT_Msk))
    {
        /* pop char from buffer with "g_u32comRhead" pointer */
        uint32_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp)
        {
            u8InChar = g_u8RecData[g_u32comRhead];
            while(UART_IS_TX_FULL(UART0));  /* Wait Tx is not full to transmit data */
            UART_WRITE(UART0, u8InChar);

            //g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            //g_u32comRbytes--;
        }
    }
#endif
    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk | UART_INTSTS_BUFERRINT_Msk));
    }
}

//#define     NULL    0

const static char szCommandShellHelp[] =
{
    "PD Command Shell Command List -\n"
    "<HELP>\n"     			/* command help                             */
    "<HRESET>\n"  			/* Hard Reset                               */
    "<LSRCCAPS>\n"          /* (SRC/SNK Only), List source capabilities */
    "<REQ_PDO>\n"           /* (SNK Only), Request Voltage              */
    "\n\n"
};



const static char *PDCommads[] =
{
    "HELP",            "HRESET",         "LSRCCAPS",	        "REQ_PDO",         NULL,
};

enum  EnumerateCommandList
{
    HELP,               HRESET,            LSRCCAPS,           REQ_PDO,
};

#define CHAR char
#define INT int


static CHAR  *get_token(CHAR *szString, CHAR *pcSepCharList, INT nMaxLen)
{
    CHAR    *pcParsePtr = szString;
    CHAR    *pcPtr;
    INT     nLen = 0;

    if (pcParsePtr == NULL)
        return NULL;

    while (*pcParsePtr)
    {
        pcPtr = pcSepCharList;
        while (*pcPtr)
        {
            if (*pcParsePtr == *pcPtr)
            {
                *pcParsePtr++ = 0;
                return pcParsePtr;
            }
            pcPtr++;
        }
        pcParsePtr++;
        if (++nLen >= nMaxLen)
            return NULL;        /* string too long!! */
    }

    return pcParsePtr;        /* reach end of the input string */
}
char _szCommandLine[RXBUFSIZE];
char *szCmd;
char *szArgum1;
char *szArgum2;

int32_t ParsingCommand(void)
{

    uint8_t nIdx = 0;
    uint8_t u8InChar = 0xFF;
    //pop char from buffer with "g_u32comRhead" pointer */
    uint32_t tmp;
    tmp = g_u32comRtail;

    printf("\n");
    while(g_u32comRhead != tmp)
    {
        u8InChar = g_u8RecData[g_u32comRhead];
        _szCommandLine[nIdx] = u8InChar;
        nIdx = nIdx + 1;
        while(UART_IS_TX_FULL(UART0));  /* Wait Tx is not full to transmit data */
        UART_WRITE(UART0, u8InChar);
        g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
        g_u32comRbytes--;
    }
    printf("\n");
    /*- get command tokens */
    szCmd = &_szCommandLine[0];
    szArgum1 = get_token(szCmd, " \t\r\n", 16);
    szArgum2 = get_token(szArgum1, " \t\r\n", 32);

    for(nIdx = 0; nIdx < sizeof(PDCommads) / sizeof(PDCommads[0]); nIdx = nIdx + 1)
    {
        if (PDCommads[nIdx] == NULL)
            return -1;      /* command not found */

        if (!strcmp(PDCommads[nIdx], szCmd))
        {
            printf("Ret = %d\n", nIdx);
            return (int32_t)nIdx;
        }
    }
    printf("Ret\n");
    return -1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
extern int pd_snk_pdo_cnt;
void UART_Commandshell(int port)
{
    /**
      *      Using a RS232 cable to connect UART0 and PC.
      **/
    do
    {
        int32_t nIdx;

        if(g_bWait == 0)
        {
            nIdx = ParsingCommand();

            switch(nIdx)
            {
            case HELP:
                printf("%s", szCommandShellHelp);
                break;
            case HRESET:
                printf("Issue Hard Reset\n");
                pd_dpm_request(port, DPM_REQUEST_HARD_RESET_SEND);
                break;
            case LSRCCAPS:
            {
                int32_t u32SrcCnt, u32SrcArray[7], i;
                if( (UTCPD_TC_get_cc_state(port) == PD_CC_UFP_ATTACHED) || (UTCPD_TC_get_cc_state(port) == PD_CC_DFP_ATTACHED) )
                    UTCPD_PE_get_src_caps(port, &u32SrcArray[0], &u32SrcCnt);
                printf("================================\n");
                for(i = 0; i < u32SrcCnt; i = i + 1)
                {
                    if( (u32SrcArray[i]&PDO_TYPE_MASK) == PDO_TYPE_FIXED)
                        printf("Fixed PDO[%d] = %dmV, %dmA\n", i, PDO_FIXED_VOLTAGE(u32SrcArray[i]), PDO_FIXED_CURRENT(u32SrcArray[i]));
                    else if( (u32SrcArray[i]&PDO_TYPE_MASK) == PDO_TYPE_AUGMENTED)
                        printf("AUG PDO[%d] = Max %dmV, Min = %dmV, Max %dmA\n", i, PDO_AUG_MAX_VOLTAGE(u32SrcArray[i]), PDO_AUG_MIN_VOLTAGE(u32SrcArray[i]), PDO_AUG_MAX_CURRENT(u32SrcArray[i]));
                }
                printf("================================\n\n");
            }
            break;
            case REQ_PDO:
            {
                int i;
                int32_t req_volt, req_curr;
                int32_t u32SrcCnt, u32SrcArray[7];
                UTCPD_PE_get_src_caps(port, &u32SrcArray[0], &u32SrcCnt);
                req_volt = atoi(szArgum1);
                req_curr = atoi(szArgum2);
                printf("Req %d %d\n", req_volt, req_curr);
                pd_backup_snk_pdo(port);
                for(i = 0; i < u32SrcCnt; i = i + 1)
                {
                    if( (u32SrcArray[i]&PDO_TYPE_MASK) == PDO_TYPE_FIXED)
                    {
                        if( (PDO_FIXED_VOLTAGE(u32SrcArray[i]) == req_volt) &&  (PDO_FIXED_CURRENT(u32SrcArray[i]) >= req_curr))
                        {
                            uint32_t flags = pd_snk_pdo[0] & (uint32_t)PDO_FIXED_FLAGS_MASK;
                            /* Back up original SNK PDOs, It will be restored until event: UTCPD_PD_SNK_REC_ACCEPT */
                            pd_snk_pdo[0] = PDO_FIXED(req_volt, req_curr, flags);
                            for(i = 1; i < pd_snk_pdo_cnt; i = i + 1)
                            {
                                pd_snk_pdo[i] = 0x0;
                            }
                            pd_request_source_voltage(port, req_volt);
                            break;
                        }
                    }
                    else if( (u32SrcArray[i]&PDO_TYPE_MASK) == PDO_TYPE_AUGMENTED )
                    {
                        printf("The chip does not support PPS requests yet\n");
                    }
                }

                if(i >= u32SrcCnt )
                {
                    printf("Requested PDO mismatched\n");
                }
            }
            break;

            default:
                printf("Unknown command\n");
            }
            g_bWait = 1;
        }
    }
    while(0);
}
#endif
