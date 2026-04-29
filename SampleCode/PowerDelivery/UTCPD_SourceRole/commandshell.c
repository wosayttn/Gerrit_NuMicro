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

#if (CONFIG_NPD_TCPC == 1)
#define UTCPD_BASE  0
#endif

extern uint32_t pd_src_pdo[];

#define RXBUFSIZE   32

#if (CONFIG_COMMAND_SHELL == 1)
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


#if (NPD48_FPGA == 1)
#define DEBUG_PORT   UART0
#else
#define DEBUG_PORT   UART1
#endif



/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#if (NPD48_FPGA == 1)
void UART0_IRQHandler(void)
{
    UART_TEST_HANDLE();
}
#else
void UART1_IRQHandler(void)
{
    UART_TEST_HANDLE();
}
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
CODE_LDROM void UART_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;

    if (UART_GET_INT_FLAG(DEBUG_PORT,UART_INTSTS_RDAINT_Msk))
    {
        //DBG_PRINTF("\nInput:");

        /* Get all the input characters */
        while(UART_IS_RX_READY(DEBUG_PORT))
        {
            /* Push char to buffer with "g_u32comRtail" pointer */
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(DEBUG_PORT);

            DBG_PRINTF("%c", u8InChar);
            while( UART_GET_TX_EMPTY(DEBUG_PORT) == 0);

            if(u8InChar == 0x0D)
            {
                g_bWait = FALSE;
                u8InChar = 0;         /* Change 0x0D to 0x0 (NULL) */
            }

            /* Check if buffer full */
            if(g_u32comRbytes < (RXBUFSIZE-1))
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
                    if(g_u32comRtail!=0)
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
        //DBG_PRINTF("\nTransmission Test:");
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
    if(DEBUG_PORT->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(DEBUG_PORT, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

//#define     NULL    0

const static char szCommandShellHelp[] =
{
#if 0
    "PD Command Shell Command List -\n"
    "<HELP>\n"    /* command help */
    "<HRESET>\n"  /* Hard Reset */
    "<PR_SWAP>\n" /* Power Role Swap, dual role only */
    "<DR_SWAP>\n" /* Data Role Swap */
    "<FR_SWAP>\n" /* Fast Role Swap, current source role and dual role only */
    "<VCS_SWAP>\n" /* VCONN Source Swap */
    "<GET_SRC_CAPS>\n"  /* (Current SNK Role Only)  Get SRC Capabilities */
    "<GET_SNK_CAPS>\n"  /* (Current SRC Role Only)  Get SNK Capabilities */
//    "<ADD>              (SNK Only)               20V                 \n"
//    "<SUB>              (SNK Only)               5V                  \n"
    "<VDM>\n"           /* TBD */
    "<SNKEN>\n"         /* (SRC/SNK Only) */
    "<SNKDIS>\n"        /* (SRC/SNK Only) */
    "<SRCEN>\n"         /* (SRC/SNK Only) */
    "<SRCDIS>\n"        /* (SRC/SNK Only) */
    "<PROLE>\n"         /* (SRC/SNK Only) */
    "<DROLE>\n"         /* (SRC/SNK Only) */
    "<SRCVC>\n"         /* (SRC/SNK Only) */
    "<DUMP0>\n"         /* (Debug Purpose)*/
    "<DUMP1>\n"         /* (Debug Purpose)*/
    "<LSRCCAPS>\n"      /* (SRC/SNK Only), List source capabilities */
    "<LSNKCAPS>\n"      /* (SRC/SNK Only), List sink capabilities   */
    "<LSRCPDO>\n"       /* (SRC/SNK Only), List source PDO we provided  */
    "<LSNKPDO>\n"       /* (SRC/SNK Only), List sink PDO we prodived   */
    "<LCONSTATE>\n"     /* (SRC/SNK Only), List connection state    */
    "<REQ_FIX>\n"       /* (SNK Only), Request Fixed PDO        */
    "<REQ_PPS>\n"       /* (SNK Only), Request PPS PDO          */
    "\n\n"
#endif
};



const static char *PDCommads[] =
{
    "HELP",             "HRESET",           "PR_SWAP",            "DR_SWAP",          "FR_SWAP",
    "VCS_SWAP",         "GET_SRC_CAPS",     "GET_SNK_CAPS",     //"ADD",              "SUB",
    "VDM",              "SNKEN",            "SNKDIS",           "SRCEN",            "SRCDIS",
    "PROLE",            "DROLE",            "SRCVC",            "DUMP0",            "DUMP1",
    "LSRCCAPS",         "LSNKCAPS",         "LSRCPDO",         "LSNKPDO",
    "LCONSTATE",        "REQ_FIX",          "REQ_PPS",  NULL,
};

enum  EnumerateCommandList
{
    HELP,                           HRESET,              PR_SWAP,                   DR_SWAP,                    FR_SWAP,
    VCS_SWAP,           GET_SRC_CAPS,              GET_SNK_CAPS,      //ADD,                SUB,
    VDM,                SNKEN,               SNKDIS,            SRCEN,              SRCDIS,
    PROLE,              DROLE,               SRCVC,             DUMP0,              DUMP1,
    LSRCCAPS,           LSNKCAPS,             LSRCPDO,          LSNKPDO,
    LCONSTATE,         REQ_FIX,                      REQ_PPS,
};


CODE_LDROM int32_t ParsingCommand(void)
{
    char szCmd[RXBUFSIZE];
    uint8_t nIdx = 0;
    uint8_t u8InChar = 0xFF;
    //pop char from buffer with "g_u32comRhead" pointer */
    volatile uint32_t tmp;
    tmp = g_u32comRtail;

    DBG_PRINTF("\n");
    while(g_u32comRhead != tmp)
    {

        u8InChar = g_u8RecData[g_u32comRhead];
        szCmd[nIdx] = u8InChar;
        nIdx = nIdx + 1;
#if (NPD48_FPGA == 1)
        while(UART_IS_TX_FULL(UART0));  /* Wait Tx is not full to transmit data */
        UART_WRITE(UART0, u8InChar);
#else
        while(UART_IS_TX_FULL(DEBUG_PORT));  /* Wait Tx is not full to transmit data */
        UART_WRITE(DEBUG_PORT, u8InChar);
#endif
        g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
        g_u32comRbytes--;
    }
    DBG_PRINTF("\n");

    for(nIdx=0; nIdx<sizeof(PDCommads)/sizeof(PDCommads[0]); nIdx=nIdx+1)
    {
        if (PDCommads[nIdx] == NULL)
            return -1;      /* command not found */

        if (!strcmp(PDCommads[nIdx], szCmd))
        {
            //DBG_PRINTF("Ret = %d\n", nIdx);
            return (int32_t)nIdx;
        }
    }
    //DBG_PRINTF("Ret = -1\n");
    return -1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
//#ifdef DBG_FRS
//extern uint32_t au32State[2];
extern int pd_snk_pdo_cnt;
//#endif
#if (CONFIG_DUMP_REGISTER == 1)
extern uint32_t Reg0[];
extern uint32_t Reg1[];
#endif

void cpu_dump(uint32_t start_addr, uint32_t end_addr)
{
    uint32_t  u32Addr;
    int val;
    int port = 0;
    uint32_t  u32Line = 0;
    DBG_PRINTF("\n[CPU DUMP: 0x%x ~ 0x%x]\n", start_addr, end_addr);
    for (u32Addr = start_addr; u32Addr < end_addr; u32Addr += 1)
    {
        //u32Data = inp32(u32Addr);
        tcpc_addr_read(port, NULL, u32Addr, &val);
        if (u32Addr % 16 == 0)
        {
            DBG_PRINTF("\n");
            DBG_PRINTF("0x%08x --     ", u32Line*16);
            u32Line = u32Line + 1;
        }
        if ( (u32Addr % 4 == 0) && (u32Addr % 16 != 0) )
        {
            DBG_PRINTF(" | ");
        }
        DBG_PRINTF("0x%02x ", (uint8_t)val);
    }
    DBG_PRINTF("\n\n");
}



CODE_LDROM void UART_Commandshell(int port)
{
    /*
            Using a RS232 cable to connect UART0 and PC.
            UART0 is set to debug port. UART0 is enable RDA and RLS interrupt.
            When inputting char to terminal screen, RDA interrupt will happen and
            UART0 will print the received char on screen.
    */
    do
    {
        int32_t nIdx;

        if(g_bWait == 0)        /* It will be set to 0 until "Enter" */
        {
            nIdx = ParsingCommand();

            switch(nIdx)
            {
            case HELP:
                DBG_PRINTF("%s", szCommandShellHelp);
                break;
            case HRESET:
                DBG_PRINTF("Issue Hard Reset\n");
                pd_dpm_request(port, DPM_REQUEST_HARD_RESET_SEND);
                break;
            case PR_SWAP:
                DBG_PRINTF("Issue Power Role Swap\n");
                pd_dpm_request(port, DPM_REQUEST_PR_SWAP);
                break;
            case DR_SWAP:
                DBG_PRINTF("Issue Data Role Swap\n");
                pd_dpm_request(port, DPM_REQUEST_DR_SWAP);
                break;
#if 0
            case FR_SWAP:
                DBG_PRINTF("Issue Fast Role Swap\n");

                outp32(UTCPD0_BASE+UTCPD_VNDIS, TXFRSIS); /* Clear TXFRXIS First, otherwise, FRS signal can't send out */
//                                  au32State[0] = 0xFFFFFFFF;
//                                  au32State[1] = 0xFFFFFFFF;
                pd_dpm_request(port, DPM_REQUEST_FR_SWAP);
                /* Issue FRS TX */
                outp32(UTCPD0_BASE+UTCPD_FRSRXCTL, inp32(UTCPD0_BASE+UTCPD_FRSRXCTL) | FRSTX);
                break;
#else
            case FR_SWAP:
                break;
#endif
            case VCS_SWAP:
                DBG_PRINTF("Issue VCONN Source Swap\n");
                if(tc_is_vconn_src(port) == 0)
                    DBG_PRINTF("We are sinking VCONN now\n");
                else
                    DBG_PRINTF("We are sourcing VCONN now\n");
                pd_dpm_request(port, DPM_REQUEST_VCONN_SWAP);
                break;
            case GET_SRC_CAPS:
                DBG_PRINTF("Get SRC Capabilities\n");
                if(pd_get_power_role(port) == PD_ROLE_SOURCE)
                    pd_dpm_request(port, DPM_REQUEST_GET_SRC_CAPS);
                else
                    pd_dpm_request(port, DPM_REQUEST_SOURCE_CAP);
                break;
            case GET_SNK_CAPS:
                DBG_PRINTF("Get SNK Capabilities\n");
                pd_dpm_request(port, DPM_REQUEST_GET_SNK_CAPS);
                break;
//                              case ADD:
//                                  pd_snk_pdo_cnt = 2;
//                                  pd_dpm_request(port, DPM_REQUEST_NEW_POWER_LEVEL);
//                              break;
//                              case SUB:
//                                  pd_snk_pdo_cnt = 1;
//                                  pd_dpm_request(port, DPM_REQUEST_NEW_POWER_LEVEL);
//                              break;
            case VDM:
                pd_dpm_request(port, DPM_REQUEST_VDM);
                break;
            case SNKEN:
                board_vbus_sink_enable(port, 1);
                break;
            case SNKDIS:
                board_vbus_sink_enable(port, 0);
                break;
            case SRCEN:
                pd_set_power_supply_ready(port);
                break;
            case SRCDIS:
                pd_power_supply_reset(port);
                break;
            case PROLE:
                //buck_boost_information(0);
                if (pd_get_power_role(port) == PD_ROLE_SOURCE)
                    DBG_PRINTF("Power Role = SRC\n");
                else
                    DBG_PRINTF("Power Role = SNK\n");
                break;
            case DROLE:
                if (pd_get_data_role(port) == PD_ROLE_DFP)
                    DBG_PRINTF("Data Role = DFP\n");
                else
                    DBG_PRINTF("Data Role = UFP\n");
                break;
            case SRCVC:
                if (!tc_check_vconn_swap(port) && tc_is_vconn_src(port) < 1)
                    DBG_PRINTF("VCONN Off\n");
                else
                    DBG_PRINTF("VCONN On\n");
                break;
#if (CONFIG_DUMP_REGISTER == 1)
            case DUMP0:
                cpu_dump(Reg0, Reg0+0x180/4);
                break;
            case DUMP1:
                cpu_dump(Reg1, Reg1+0x180/4);
                break;
#else
            case DUMP0:
                //ADC_SET_CONVERSION_DATA(port, 1, 0x770);
                cpu_dump(UTCPD_BASE, UTCPD_BASE+0x100);
                break;
            case DUMP1:
                cpu_dump(UTCPD_BASE+0x100, UTCPD_BASE+0x200);
                //ADC_SET_CONVERSION_DATA(port, 1, 0x780);
                //cpu_dump(UTCPD_BASE+0x100, UTCPD_BASE+0x200);
                //ADC_SET_CONVERSION_DATA(port, 1, 0x781);
                break;
#endif

            case LSRCCAPS:
            {
                int32_t u32SrcCnt, u32SrcArray[7], i;
                if( (UTCPD_TC_get_cc_state(port) == PD_CC_UFP_ATTACHED) || (UTCPD_TC_get_cc_state(port) == PD_CC_DFP_ATTACHED) )
                    UTCPD_PE_get_src_caps(port, &u32SrcArray[0], &u32SrcCnt);
                DBG_PRINTF("================================\n");
                for(i=0; i<u32SrcCnt; i=i+1)
                    printf("PDO[%d] = 0x%x\n", i, u32SrcArray[i]);
                DBG_PRINTF("================================\n\n");
            }
            break;
            case LSNKCAPS:
            {
                int32_t u32SnkCnt, u32SnkArray[7], i;
                if( (UTCPD_TC_get_cc_state(port) == PD_CC_UFP_ATTACHED) || (UTCPD_TC_get_cc_state(port) == PD_CC_DFP_ATTACHED) )
                    UTCPD_PE_get_snk_caps(port, &u32SnkArray[0], &u32SnkCnt);
                DBG_PRINTF("================================\n");
                for(i=0; i<u32SnkCnt; i=i+1)
                    DBG_PRINTF("PDO[%d] = 0x%x\n", i, u32SnkArray[i]);
                DBG_PRINTF("================================\n\n");
            }
            break;
            case LSRCPDO:
            {
                uint32_t i;
                for( i = 0; i<pd_src_pdo_cnt; i = i+1)
                    DBG_PRINTF("SPR PDO[%d] = 0x%x\n", i, pd_src_pdo[i]);
                DBG_PRINTF("--------------------\n");
                for( i = 0; i<pd_src_epr_pdo_cnt; i = i+1)
                    DBG_PRINTF("EPR PDO[%d] = 0x%x\n", i, pd_src_epr_pdo[i]);
                DBG_PRINTF("--------------------\n");
            }
            break;

            case LSNKPDO:
                break;

            case LCONSTATE:
            {
                int32_t u32SrcCnt, u32SrcArray[7];
                int32_t u32SnkCnt, u32SnkArray[7];
                if( (UTCPD_TC_get_cc_state(port) == PD_CC_UFP_ATTACHED) || (UTCPD_TC_get_cc_state(port) == PD_CC_DFP_ATTACHED) )
                {
                    UTCPD_PE_get_src_caps(port, &u32SrcArray[0], &u32SrcCnt);
                    UTCPD_PE_get_snk_caps(port, &u32SnkArray[0], &u32SnkCnt);
                    if(u32SrcCnt != 0)
                        DBG_PRINTF("PD Contract Established\n");
                    else
                        DBG_PRINTF("USBC Connected, But PD Contract is fail\n");
                }
                else
                    DBG_PRINTF("USBC Disconnected\n");
            }
            break;
            case REQ_FIX:
            {
                static uint32_t u32ReqIndex=0;
                int32_t u32Req_mv=0;
                int32_t u32SrcCnt, u32SrcArray[7];

                UTCPD_PE_get_src_caps(port, &u32SrcArray[0], &u32SrcCnt);
                do
                {
                    if( (u32SrcArray[u32ReqIndex] & PDO_TYPE_MASK) == PDO_TYPE_FIXED)
                    {
                        u32Req_mv = ((u32SrcArray[u32ReqIndex] & 0xFFC00) >> 10)*50;
                        /* Reqest Fixed PDO */
                        pd_request_source_voltage(port, u32Req_mv);
                    }
                    else
                    {
                        //Find next FIX PDO
                        DBG_PRINTF("Not Fixed PDO, Plese use command REQ_PPS to request PPS PDO\n");
                    }
                    /* Next Request PDO */
                    u32ReqIndex = u32ReqIndex + 1;
                    if(u32ReqIndex >= u32SrcCnt)
                        u32ReqIndex = 0;
                }
                while(u32Req_mv == 0);

            }
            break;
            case REQ_PPS:
            {
                static uint32_t u32ReqIndex = 0, i;
                static uint32_t u32ReqVol = 0;
                uint32_t u32Req_mv;
                uint32_t u32minmv =0, u32maxmv = 0, u32curr=0;
                int32_t u32SrcCnt, u32SrcArray[7];

                UTCPD_PE_get_src_caps(port, &u32SrcArray[0], &u32SrcCnt);
                for(i=0; i< u32SrcCnt; i=i+1)
                {
                    if( (u32SrcArray[i] & PDO_TYPE_AUGMENTED) == PDO_TYPE_AUGMENTED )
                    {
                        DBG_PRINTF("PPS IDx = %d\n", i);
                        u32maxmv = PDO_AUG_MAX_VOLTAGE(u32SrcArray[i]);
                        u32minmv = PDO_AUG_MIN_VOLTAGE(u32SrcArray[i]);
                        u32curr = PDO_AUG_MAX_CURRENT(u32SrcArray[i]);
                        DBG_PRINTF("PPS MAX = %d\n", u32maxmv);
                        DBG_PRINTF("PPS Min = %d\n", u32minmv);
                        DBG_PRINTF("PPS Cur = %d\n", u32curr);
                        u32ReqIndex = i+1;
                        if( (u32ReqVol >= u32maxmv) || (u32ReqVol< u32minmv) )
                        {
                            u32Req_mv = u32minmv;
                            DBG_PRINTF("Req mv = %d\n", u32Req_mv);
                        }
                        break;
                    }
                }
                if(u32maxmv == 0)
                {
                    DBG_PRINTF("Can't find PPS PDO\n");
                    break;
                }



                if( (u32SrcArray[u32ReqIndex-1] & PDO_TYPE_AUGMENTED) == PDO_TYPE_AUGMENTED)
                {
                    /* Reqest Fixed PDO */
                    DBG_PRINTF("req pps %d mv\n", u32Req_mv);
                    pd_request_source_voltage(port, u32Req_mv);
                }
                else
                    DBG_PRINTF("Not Fixed PDO, Plese use command REQ_PPS %d\n", u32ReqIndex);
                /* Next Request PDO */
                u32Req_mv = u32Req_mv + 20;
                if(u32Req_mv > u32maxmv)
                    u32Req_mv = u32minmv;

#if 0
                /* Request first FIX PDO */
                UUTCPD_PE_get_src_caps(port, &u32SrcArray[0], &u32SrcCnt);
                u32Req_mv = ((u32SrcArray[0] & 0xFFC00) >> 10)*50;
                /* Reqest Fixed PDO */
                pd_request_source_voltage(port, u32Req_mv);
#endif
            }
            break;


            default:
                DBG_PRINTF("Unknown command\n");
            }
            g_bWait = 1;
        }
    }
    while(0);
}
#endif
