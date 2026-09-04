/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use CANFD Monitor mode to listen to CAN bus communication test.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

CANFD_FD_MSG_T   g_sRxFIFO0MsgFrame[3];
CANFD_FD_MSG_T   g_sRxFIFO1MsgFrame[3];
uint8_t g_u8RxFIFO0RcvOk = 0;
uint8_t g_u8RxFIFO1RcvOk = 0;
uint8_t g_u8RxFIFO0MsgIndex = 0;
uint8_t g_u8RxFIFO1MsgIndex = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_MonitorMode_Init(uint32_t u32NormBitRate,uint32_t u32DataBitRate);
uint32_t Get_CANFD_NominalBitRate(CANFD_T *psCanfd);
uint32_t Get_CANFD_DataBitRate(CANFD_T *psCanfd);
void CANFD_ShowMsg(CANFD_FD_MSG_T * sRxMsg);
void CANFD00_IRQHandler(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  ISR to handle CAN FD Line 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD00_IRQHandler(void)
{
    /*Rx FIFO 0 New Message Interrupt */
    if(CANFD0->IR & CANFD_IR_RF0N_Msk )
    {
        g_u8RxFIFO0RcvOk = 1;
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0N_Msk);
    }
    /*Rx FIFO 1 New Message Interrupt */
    if(CANFD0->IR & CANFD_IR_RF1N_Msk )
    {
        g_u8RxFIFO1RcvOk = 1;
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1N_Msk);
    }
    /* Rx FIFO 0 Message Lost Interrupt */
    if(CANFD0->IR & CANFD_IR_RF0L_Msk)
    {
        printf("Rx FIFO 0 Message Lost(Standard ID)\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0L_Msk);
    }
    /* Rx FIFO 1 Message Lost Interrupt */
    if(CANFD0->IR & CANFD_IR_RF1L_Msk)
    {
        printf("Rx FIFO 1 Message Lost(Extended ID)\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1L_Msk);
    }

}

/*---------------------------------------------------------------------------*/
/*  Show Message Function                                                    */
/*---------------------------------------------------------------------------*/
void CANFD_ShowMsg(CANFD_FD_MSG_T * sRxMsg)
{
    uint8_t u8Cnt;
    /* Show the message information */
    if(sRxMsg->eIdType == eCANFD_SID)
        printf("Rx buf 0: ID = 0x%08X(11-bit),DLC = %d\n", sRxMsg->u32Id,sRxMsg->u32DLC);
    else
        printf("Rx buf 1: ID = 0x%08X(29-bit),DLC = %d\n", sRxMsg->u32Id,sRxMsg->u32DLC);

    printf("Message Data : ");
    for (u8Cnt = 0; u8Cnt < sRxMsg->u32DLC; u8Cnt++)
    {
        printf("%02u ,", sRxMsg->au8Data[u8Cnt]);
    }
    printf("\n\n");
}

/*---------------------------------------------------------------------------*/
/*  Get the CANFD interface Nominal bit rate Function                        */
/*---------------------------------------------------------------------------*/
uint32_t Get_CANFD_NominalBitRate(CANFD_T *psCanfd)
{
    uint32_t u32BitRate = 0;
    uint32_t u32CanClk  = 0;
    uint32_t u32CanDiv  = 0;
    uint8_t  u8Tq = 0;
    uint8_t  u8NtSeg1 = 0;
    uint8_t  u8NtSeg2 = 0;
    if(CLK_GetModuleClockSource(CANFD0_MODULE) == (CLK_CLKSEL0_CANFD0SEL_HCLK >> CLK_CLKSEL0_CANFD0SEL_Pos) )
        u32CanClk = CLK_GetHCLKFreq();
    else
        u32CanClk = CLK_GetHXTFreq();

    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD0DIV_Msk) >> CLK_CLKDIV5_CANFD0DIV_Pos) + 1;
    u32CanClk = u32CanClk / u32CanDiv;
    u8Tq = ((psCanfd->NBTP & CANFD_NBTP_NBRP_Msk) >> CANFD_NBTP_NBRP_Pos)+1 ;
    u8NtSeg1 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG1_Msk) >> CANFD_NBTP_NTSEG1_Pos);
    u8NtSeg2 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG2_Msk) >> CANFD_NBTP_NTSEG2_Pos);
    u32BitRate = u32CanClk / u8Tq / (u8NtSeg1+u8NtSeg2+3);

    return u32BitRate;
}

/*---------------------------------------------------------------------------*/
/*  Get the CANFD interface Data bit rate Function                           */
/*---------------------------------------------------------------------------*/
uint32_t Get_CANFD_DataBitRate(CANFD_T *psCanfd)
{
    uint32_t u32BitRate = 0;
    uint32_t u32CanClk  = 0;
    uint32_t u32CanDiv  = 0;
    uint8_t  u8Tq = 0;
    uint8_t  u8NtSeg1 = 0;
    uint8_t  u8NtSeg2 = 0;
    if(CLK_GetModuleClockSource(CANFD0_MODULE) == (CLK_CLKSEL0_CANFD0SEL_HCLK >> CLK_CLKSEL0_CANFD0SEL_Pos) )
        u32CanClk = CLK_GetHCLKFreq();
    else
        u32CanClk = CLK_GetHXTFreq();

    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD0DIV_Msk) >> CLK_CLKDIV5_CANFD0DIV_Pos) + 1;
    u32CanClk = u32CanClk / u32CanDiv;
    u8Tq = ((psCanfd->DBTP & CANFD_DBTP_DBRP_Msk) >> CANFD_DBTP_DBRP_Pos)+1 ;
    u8NtSeg1 = ((psCanfd->DBTP & CANFD_DBTP_DTSEG1_Msk) >> CANFD_DBTP_DTSEG1_Pos);
    u8NtSeg2 = ((psCanfd->DBTP & CANFD_DBTP_DTSEG2_Msk) >> CANFD_DBTP_DTSEG2_Pos);
    u32BitRate = u32CanClk / u8Tq / (u8NtSeg1+u8NtSeg2+3);

    return u32BitRate;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select CAN FD0 clock source is HXT */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HXT, CLK_CLKDIV5_CANFD0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set CAN FD0 multi-function pins */
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(SYS_GPC_MFP1_PC4MFP_Msk | SYS_GPC_MFP1_PC5MFP_Msk)) |
                    (SYS_GPC_MFP1_PC4MFP_CANFD0_RXD | SYS_GPC_MFP1_PC5MFP_CANFD0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init CAN FD                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_MonitorMode_Init(uint32_t u32NormBitRate,uint32_t u32DataBitRate)
{
    CANFD_FD_T sCANFD_Config;

    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = u32NormBitRate;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = u32DataBitRate;
    CANFD_Open(CANFD0, &sCANFD_Config);
    NVIC_EnableIRQ(CANFD00_IRQn);
    printf("CANFD monitoring Nominal baud rate(bps): %d\n",Get_CANFD_NominalBitRate(CANFD0));
    printf("CANFD monitoring Data baud rate(bps): %d\n",Get_CANFD_DataBitRate(CANFD0));
    /*Enable the Bus Monitoring Mode */
    CANFD0->CCCR |= CANFD_CCCR_MON_Msk;

    /* Non-matching Frames with Extended ID and Standard ID are stored in Rx FIFO0 or Rx FIFO1, reject all remote frames with 11-bit standard IDs and 29-bit extended IDs */
    CANFD_SetGFC(CANFD0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO1, 1, 1);
    /* Enable RX FIFO New message, Message lost interrupt using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_RF0NE_Msk |CANFD_IE_RF0LE_Msk | CANFD_IE_RF1NE_Msk|CANFD_IE_RF1LE_Msk), 0, 0, 0);
    /* CAN FD Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("\n\nCPU @ %dHz\n",SystemCoreClock);
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                      CANFD Monitor Mode Sample Code                         |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                              |\n");
    printf("|    The sample code needs three boards. Use CANFD Monitor mode to listen to  |\n");
    printf("|    CAN Bus communication test.The sample code must be set up on the node of |\n");
    printf("|    CAN communication transmission. Users can use one of the sample codes    |\n");
    printf("|    ' CANFD_CANFD_TxRx ' or ' CANFD_CANFD_TxRxINT ' as the CAN communication |\n");
    printf("|    transmission network.                                                    |\n");
    printf("|    Note: Users need to confirm whether the transmission rate matches.       |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");

    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                              Pin Configure                                  |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                                                         CAN_TXD(Any board)  |\n");
    printf("|                                 CAN BUS                 CAN_RXD(Any board)  |\n");
    printf("|                                   ||    CAN_H   |-----------|               |\n");
    printf("|                                   || <--------->|           |<------        |\n");
    printf("|                                   ||            |    CAN    |CAN_TX         |\n");
    printf("|  CAN0_TXD(PC5)                    ||    CAN_L   |Transceiver|               |\n");
    printf("|  CAN0_RXD(PC4)                    || <--------->|           |------>        |\n");
    printf("|          |-----------|   CAN_H    ||            |           |CAN_RX         |\n");
    printf("|  ------> |           |<---------> ||            |-----------|               |\n");
    printf("|  CAN0_TX |   CAN     |            ||                                        |\n");
    printf("|          |Transceiver|            ||                    CAN_TXD(Any board)  |\n");
    printf("|  <------ |           |   CAN_L    ||                    CAN_RXD(Any board)  |\n");
    printf("|  CAN0_RX |           |<---------> ||    CAN_H   |-----------|               |\n");
    printf("|          |-----------|            || <--------->|           |<------        |\n");
    printf("|                                   ||            |    CAN    |CAN_TX         |\n");
    printf("|                                   ||    CAN_L   |Transceiver|               |\n");
    printf("|                                   || <--------->|           |------>        |\n");
    printf("|                                   ||            |           |CAN_RX         |\n");
    printf("|                                   ||            |-----------|               |\n");
    printf("|                                                                             |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");

    /* CANFD interface initialization*/
    CANFD_MonitorMode_Init(1000000,4000000);

    while(1)
    {
        if(g_u8RxFIFO0RcvOk == 1)
        {
            if(g_u8RxFIFO0MsgIndex > 2)
                g_u8RxFIFO0MsgIndex = 0;
            /* Receive the Rx FIFO0 message(Standard ID) */
            CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxFIFO0MsgFrame[g_u8RxFIFO0MsgIndex]);
            g_u8RxFIFO0MsgIndex++;
            g_u8RxFIFO0RcvOk = 0;
        }
        if(g_u8RxFIFO0RcvOk == 0 && g_u8RxFIFO0MsgIndex != 0)
        {
            CANFD_ShowMsg(&g_sRxFIFO0MsgFrame[g_u8RxFIFO0MsgIndex - 1]);
            g_u8RxFIFO0MsgIndex--;
        }
        if(g_u8RxFIFO1RcvOk == 1)
        {
            if(g_u8RxFIFO1MsgIndex > 2)
                g_u8RxFIFO1MsgIndex = 0;
            /* Receive the Rx FIFO1 message(Extended ID) */
            CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxFIFO1MsgFrame[g_u8RxFIFO1MsgIndex]);
            g_u8RxFIFO1MsgIndex++;
            g_u8RxFIFO1RcvOk = 0;
        }
        if(g_u8RxFIFO1RcvOk == 0 && g_u8RxFIFO1MsgIndex != 0)
        {
            CANFD_ShowMsg(&g_sRxFIFO1MsgFrame[g_u8RxFIFO1MsgIndex - 1]);
            g_u8RxFIFO1MsgIndex--;
        }
    }
}

