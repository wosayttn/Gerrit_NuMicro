/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This example demonstrates how to recover transmission from the bus-off state.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

#define CANFD_OP_MODE   CANFD_OP_CAN_FD_MODE

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
CANFD_FD_MSG_T      g_sRxMsgFrame;
CANFD_FD_MSG_T      g_sTxMsgFrame;
uint32_t            g_TxData;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_ShowRecvMessage(void);
void CANFD_RxTest(void);
void CANFD_TxTest(void);
void CANFD_TxRxINTTest(void);
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenType);
void CANFD_BusOffRecovery(CANFD_T *psCanfd);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  ISR to handle CAN FD Line 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD00_IRQHandler(void)
{
    if (CANFD0->IR & CANFD_IR_EW_Msk)
    {
        printf("Error warning flag is set.\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_EW_Msk);
    }

    if (CANFD0->IR & CANFD_IR_EP_Msk)
    {
        printf("Error passive flag is set.\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_EP_Msk);
    }

    if (CANFD0->IR & CANFD_IR_BO_Msk)
    {
        if (CANFD0->PSR & CANFD_PSR_BO_Msk)
        {
            printf("\nBus-Off occurred. Press any key to start the bus-off recovery procedure.\n");
            getchar();

            CANFD_BusOffRecovery(CANFD0);
        }
        else
        {
            /* The Bus-Off state change does not indicate a Bus-Off event; it only clears the BO flag */
            CANFD_ClearStatusFlag(CANFD0, CANFD_IR_BO_Msk);
        }
    }

    if (CANFD0->IR & CANFD_IR_RF1N_Msk)
    {
        /* Receive the Rx Fifo1 buffer */
        CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxMsgFrame);

        CANFD_ShowRecvMessage();
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));

        /* Clear the Interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1N_Msk);
    }
}

void CANFD_BusOffRecovery(CANFD_T *psCanfd)
{
    /* CAN FD0 run to initial mode */
    CANFD_RunToNormal(psCanfd, FALSE);

    /* Cancel all transmit requests */
    psCanfd->TXBCR = 0xFFFFFFFF;

    /* Clear all interrupt flag */
    CANFD_ClearStatusFlag(psCanfd, 0xFFFFFFFF);

    /* CAN FD0 run to normal mode */
    CANFD_RunToNormal(psCanfd, TRUE);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL and set core clock to 144MHz*/
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HCLK, CLK_CLKDIV1_CANFD0(1));

    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /* Debug UART clock setting */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PC multi-function pins for CANFD RXD and TXD */
    SET_CANFD0_RXD_PC4();
    SET_CANFD0_TXD_PC5();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN FD Function Tx Test                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxTest(void)
{
    g_TxData = 0;

    while (1)
    {
        /* Standard ID =0x111, Data lenght 8 bytes */
        CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 0);

        /* Transmit packets with a 100ms cycle time */
        CLK_SysTickDelay(100000);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                 Send CAN FD Message Function                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenType)
{
    uint8_t u8Cnt;

    /* Set the ID Number */
    psTxMsg->u32Id = u32Id;
    /* Set the frame type */
    psTxMsg->eIdType = eIdType;
    /* Set FD frame format attribute */
    psTxMsg->bFDFormat = 1;
    /* Set the bitrate switch attribute */
    psTxMsg->bBitRateSwitch = 1;

    /* Set data length */
    if (u8LenType == 0)      psTxMsg->u32DLC = 8;
    else if (u8LenType == 1) psTxMsg->u32DLC = 12;
    else if (u8LenType == 2) psTxMsg->u32DLC = 16;
    else if (u8LenType == 3) psTxMsg->u32DLC = 20;
    else if (u8LenType == 4) psTxMsg->u32DLC = 24;
    else if (u8LenType == 5) psTxMsg->u32DLC = 32;
    else if (u8LenType == 6) psTxMsg->u32DLC = 48;
    else if (u8LenType == 7) psTxMsg->u32DLC = 64;

    /* use message buffer 0 */
    if (eIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    printf("Data Message(%02u bytes) : ", psTxMsg->u32DLC);

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++)
    {
        g_TxData++;
        psTxMsg->au8Data[u8Cnt] = u8Cnt + (g_TxData % 3);
        printf("%02u,", psTxMsg->au8Data[u8Cnt]);
    }

    printf("\n");

    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                   CAN FD Function Rx Test                                               */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_RxTest(void)
{
    printf("CAN FD bus reception :\n");

    while (1) {}
}

/*---------------------------------------------------------------------------------------------------------*/
/*                             Show the CAN FD Message Function                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_ShowRecvMessage(void)
{
    uint8_t u8Cnt;

    if (g_sRxMsgFrame.eIdType == eCANFD_SID)
        printf("Rx FIFO1(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
    else
        printf("Rx FIFO1(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

    printf("Message Data(%02u bytes) : ", g_sRxMsgFrame.u32DLC);

    for (u8Cnt = 0; u8Cnt <  g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02u ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                     Init CAN FD0                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|    CAN0_TXD(PC5)                         CAN_TXD(Any board)   |\n");
    printf("|    CAN0_RXD(PC4)                         CAN_RXD(Any board)   |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|   CAN0_TX|   CANFD   | CANFD_H |    CANFD  |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|   CAN0_RX|           | CANFD_L |           |CAN_RX            |\n");
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");

    /* Use defined configuration */
    sCANFD_Config.sElemSize.u32UserDef = 0;

    /* Get the CAN FD configuration value */
#if (CANFD_OP_MODE == CANFD_OP_CAN_FD_MODE)
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 2000000;
#else
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
#endif

    /* Open the CAN FD feature */
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* Note: About TLE9351VSJ High speed CAN FD transceiver */
    /* Due to TX/RX propagation delay, user should adjust the 'Transmitter Delay Compensation' setting */
    //CANFD0->DBTP |= CANFD_DBTP_TDC_Msk;
    //CANFD0->TDCR = 0x300;

    /* receive 0x110~0x11F in CAN FD0 rx fifo1 buffer by setting index 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO1_STD_MASK(0x110, 0x7F0));

    /* Reject Non-Matching Standard ID and Extended ID Filter(RX fifo1) */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);

    /* Enable interrupt using interrupt line 0 */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF1NE_Msk | CANFD_IE_BOE_Msk | CANFD_IE_EWE_Msk | CANFD_IE_EPE_Msk), 0, 0, 0);

    /* Enable CANFD0 IRQ00 Handler */
    NVIC_EnableIRQ(CANFD00_IRQn);

    /* CAN FD0 Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                           CAN FD Tx Rx Interrupt Function Test                                           */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* CAN FD interface initialization */
    CANFD_Init();

    printf("+----------------------------------------------------------------------------+\n");
    printf("|                     CAN FD Function Test                                   |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                                        |\n");
    printf("|  [0] Master(CAN FD transmitter)    [1] Slave(CAN FD receiver)              |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Attempt to trigger a bus-off state after communication is established     |\n");
    printf("+----------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if (u8Item == '0')
    {
        CANFD_TxTest();
    }
    else
    {
        CANFD_RxTest();
    }

    printf("CAN FD Sample Code End.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /* Init Debug UART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    /* print a note to terminal */
    printf("\n CAN FD mode transmission test\r\n");

    CANFD_TxRxINTTest();

    while (1) {}
}

/*** (C) COPYRIGHT 2026 Nuvoton Technology Corp. ***/
