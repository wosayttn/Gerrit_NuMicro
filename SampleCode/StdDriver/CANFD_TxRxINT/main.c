/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    An example of interrupt control using CAN / CAN FD bus communication.
 *
 *           Set OP_MODE to CANFD_OP_CAN_FD_MODE to enable CAN FD mode,
 *           or CANFD_OP_CAN_MODE to use classic CAN mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Mode selection: set OP_MODE to select operating mode                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define OP_MODE  CANFD_OP_CAN_FD_MODE  /* Options: CANFD_OP_CAN_FD_MODE  or  CANFD_OP_CAN_MODE */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
CANFD_FD_MSG_T      g_sRxMsgFrame;
CANFD_FD_MSG_T      g_sTxMsgFrame;
volatile uint8_t    g_u8RxCompleteFlag         = 0;  /* FIFO1 in CAN FD mode, FIFO0 in CAN mode */
volatile uint8_t    g_u8BusOffFlag             = 0;
volatile uint32_t   g_u32BusOffRecoveryCounter = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_Init(void);
void CANFD_TxRxINTTest(void);
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenParam);
void CANFD_ShowRecvMessage(void);
void CANFD_RxTest(void);
void CANFD_TxTest(void);
uint8_t CANFD_BusOffRecovery(void);
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD00_IRQHandler(void)
{
    uint32_t u32IntStatus = CANFD0->IR;

    printf("IR =0x%08X \n", u32IntStatus);

    /* Check Error Warning status -- common */
    if (u32IntStatus & CANFD_IR_EW_Msk)
    {
        printf("Error warning flag is set.\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_EW_Msk);
    }

    /* Check Error Passive status -- common */
    if (u32IntStatus & CANFD_IR_EP_Msk)
    {
        printf("Error passive flag is set.\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_EP_Msk);
    }

    /* Check Bus-Off status -- common */
    if (u32IntStatus & CANFD_IR_BO_Msk)
    {
        if (CANFD0->PSR & CANFD_PSR_BO_Msk)
        {
            printf("Bus-Off detected! Recovery will be triggered on next transmission attempt.\n");
            g_u8BusOffFlag = 1;
        }

        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_BO_Msk);
    }

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)

    /* CAN FD mode: receive from FIFO1 */
    if (u32IntStatus & CANFD_IR_RF1N_Msk)
    {
        CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxMsgFrame);
        g_u8RxCompleteFlag = 1;
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1N_Msk);
    }

#else

    /* CAN mode: receive from FIFO0 */
    if (u32IntStatus & CANFD_IR_RF0N_Msk)
    {
        CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
        g_u8RxCompleteFlag = 1;
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0N_Msk);
    }

#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock and Multi-function I/O                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL and set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HCLK, CLK_CLKDIV1_CANFD0(1));

    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /* Debug UART clock setting */
    SetDebugUartCLK();

    /* Init I/O Multi-function */
    SetDebugUartMFP();

    /* Set PC multi-function pins for CANFD RXD and TXD */
    SET_CANFD0_RXD_PC4();
    SET_CANFD0_TXD_PC5();
}

/*---------------------------------------------------------------------------------------------------------*/
/* Initialize CAN / CAN FD peripheral                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  CAN0_TXD(PC5)                         CAN_TXD(Any board)     |\n");
    printf("|  CAN0_RXD(PC4)                         CAN_RXD(Any board)     |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|  CAN0_TX |Transceiver|  CAN_H  |Transceiver|CAN_TX            |\n");
    printf("|          |           |         |           |                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|  CAN0_RX |           |  CAN_L  |           |CAN_RX            |\n");
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");

    sCANFD_Config.sElemSize.u32UserDef = 0;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* CAN FD mode: FD operation, nominal 1 Mbps, data 2 Mbps */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 2000000;
#else
    /* CAN mode: classic operation, no data phase bitrate */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
#endif

    /* Common: set nominal bitrate and open peripheral */
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    CANFD_Open(CANFD0, &sCANFD_Config);

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("CAN FD Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
    printf("CAN FD Data bit rate(bps): %d\n", CANFD_GetDataBitRate(CANFD0));
#else
    printf("CAN Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
#endif

    /* Set SID / XID receive filters -- FIFO1 for CAN FD, FIFO0 for CAN */
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO1_STD_MASK(0x110, 0x7F0));
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO1_STD_MASK(0x22F, 0x7FF));
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO1_STD_MASK(0x333, 0x7FF));
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO1_EXT_MASK_LOW(0x220),   CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFF0));
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO1_EXT_MASK_LOW(0x3333),  CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO1_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
#else
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(0x110, 0x7F0));
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO0_STD_MASK(0x22F, 0x7FF));
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO0_STD_MASK(0x333, 0x7FF));
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO0_EXT_MASK_LOW(0x220),   CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFF0));
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO0_EXT_MASK_LOW(0x3333),  CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO0_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
#endif

    /* Common: reject non-matching frames */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);

    /* Enable FIFO new-message, Bus-Off, Error Warning, Error Passive interrupts on line 0 */
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    CANFD_EnableInt(CANFD0, (CANFD_IE_RF1NE_Msk | CANFD_IE_BOE_Msk | CANFD_IE_EWE_Msk | CANFD_IE_EPE_Msk), 0, 0, 0);
#else
    CANFD_EnableInt(CANFD0, (CANFD_IE_RF0NE_Msk | CANFD_IE_BOE_Msk | CANFD_IE_EWE_Msk | CANFD_IE_EPE_Msk), 0, 0, 0);
#endif

    /* Common: enable IRQ and run */
    NVIC_EnableIRQ(CANFD00_IRQn);
    CANFD_RunToNormal(CANFD0, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Bus-Off recovery: reset to init mode, cancel pending Tx, re-enter Normal mode                          */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CANFD_BusOffRecovery(void)
{
    printf("Starting Bus-Off recovery sequence...\n");

    /* Enter init mode */
    CANFD_RunToNormal(CANFD0, FALSE);

    /* Cancel all pending transmit requests */
    CANFD0->TXBCR = 0xFFFFFFFF;

    /* Clear all interrupt flags */
    CANFD_ClearStatusFlag(CANFD0, 0xFFFFFFFF);

    /* Back to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);

    /* 50 ms settling delay */
    CLK_SysTickDelay(50000);

    if (CANFD0->PSR & CANFD_PSR_BO_Msk)
    {
        printf("Bus-Off recovery failed. Still in Bus-Off state.\n");
        return 0;   /* Recovery failed */
    }
    else
    {
        g_u8BusOffFlag = 0;
        g_u32BusOffRecoveryCounter++;
        printf("Bus-Off recovery completed. Recovery count: %u\n", g_u32BusOffRecoveryCounter);
        return 1;   /* Recovery successful */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Transmit a message                                                                                      */
/*  u8LenParam: CAN FD mode - DLC index (0~7 maps to 8/12/16/20/24/32/48/64 bytes)                       */
/*              CAN mode    - DLC value directly (bytes)                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenParam)
{
    uint8_t u8Cnt;

    /* Check Bus-Off state before transmitting -- common */
    if (g_u8BusOffFlag)
    {
        printf("Bus-Off state detected. Starting recovery process...\n");

        if (CANFD_BusOffRecovery())
            printf("Bus-Off recovery successful. Proceeding with transmission.\n");
        else
        {
            printf("Bus-Off recovery failed. Cannot transmit.\n");
            return;
        }
    }

    /* Set ID and type -- common */
    psTxMsg->u32Id   = u32Id;
    psTxMsg->eIdType = eIdType;
    psTxMsg->eFrmType = eCANFD_DATA_FRM;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* CAN FD mode: enable FD format and bitrate switching, map index to DLC */
    psTxMsg->bFDFormat      = 1;
    psTxMsg->bBitRateSwitch = 1;

    if (u8LenParam == 0) psTxMsg->u32DLC = 8;
    else if (u8LenParam == 1) psTxMsg->u32DLC = 12;
    else if (u8LenParam == 2) psTxMsg->u32DLC = 16;
    else if (u8LenParam == 3) psTxMsg->u32DLC = 20;
    else if (u8LenParam == 4) psTxMsg->u32DLC = 24;
    else if (u8LenParam == 5) psTxMsg->u32DLC = 32;
    else if (u8LenParam == 6) psTxMsg->u32DLC = 48;
    else if (u8LenParam == 7) psTxMsg->u32DLC = 64;

#else
    /* CAN mode: classic frame, u8LenParam is the actual byte count */
    psTxMsg->bBitRateSwitch = 0;
    psTxMsg->u32DLC         = u8LenParam;
#endif

    g_u8RxCompleteFlag = 0;

    /* Print ID -- common */
    if (eIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* CAN FD mode: fill and print full payload before transmitting */
    printf("Data Message(%02u bytes) : ", psTxMsg->u32DLC);

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++)
    {
        psTxMsg->au8Data[u8Cnt] = u8Cnt;
        printf("%02u,", psTxMsg->au8Data[u8Cnt]);
    }

    printf("\n");
#else

    /* CAN mode: fill payload without printing */
    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;

#endif

    /* Transmit via message buffer 0 -- common */
    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
        printf("Failed to transmit message\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* Display received message                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_ShowRecvMessage(void)
{
    uint8_t u8Cnt;

    if (g_sRxMsgFrame.eIdType == eCANFD_SID)
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
        printf("Rx FIFO1(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

#else
        printf("Rx FIFO0(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
#endif
    else
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
        printf("Rx FIFO1(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

#else
        printf("Rx FIFO0(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
#endif

    printf("Message Data(%02u bytes) : ", g_sRxMsgFrame.u32DLC);

    for (u8Cnt = 0; u8Cnt < g_sRxMsgFrame.u32DLC; u8Cnt++)
        printf("%02u ,", g_sRxMsgFrame.au8Data[u8Cnt]);

    printf("\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* Tx Test (Master): interactive menu to send messages                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxTest(void)
{
    uint8_t u8Item;

    do
    {
        printf("\n");
        printf("+-----------------------------------------------------------+\n");
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
        printf("|             CAN FD Tx Function Test (Master)              |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| [1] Standard ID = 0x111   ( Data length  8 bytes )        |\n");
        printf("| [2] Standard ID = 0x113   ( Data length 12 bytes )        |\n");
        printf("| [3] Standard ID = 0x22F   ( Data length 16 bytes )        |\n");
        printf("| [4] Standard ID = 0x333   ( Data length 20 bytes )        |\n");
        printf("| [5] Extended ID = 0x221   ( Data length 24 bytes )        |\n");
        printf("| [6] Extended ID = 0x227   ( Data length 32 bytes )        |\n");
        printf("| [7] Extended ID = 0x3333  ( Data length 48 bytes )        |\n");
        printf("| [8] Extended ID = 0x44444 ( Data length 64 bytes )        |\n");
#else
        printf("|               CAN Tx Function Test (Master)               |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| [1] Standard ID = 0x111   ( Data length 8 bytes )         |\n");
        printf("| [2] Standard ID = 0x22F   ( Data length 8 bytes )         |\n");
        printf("| [3] Standard ID = 0x333   ( Data length 8 bytes )         |\n");
        printf("| [4] Extended ID = 0x221   ( Data length 8 bytes )         |\n");
        printf("| [5] Extended ID = 0x3333  ( Data length 8 bytes )         |\n");
        printf("| [6] Extended ID = 0x44444 ( Data length 8 bytes )         |\n");
#endif
        printf("| Select ID number and master will send message to slave ...|\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| Quit                                              - [ESC] |\n");
        printf("+-----------------------------------------------------------+\n\n");

        u8Item = getchar();

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)

        switch (u8Item)
        {
            case '1':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111,   0);
                break;

            case '2':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x113,   1);
                break;

            case '3':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F,   2);
                break;

            case '4':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333,   3);
                break;

            case '5':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221,   4);
                break;

            case '6':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x227,   5);
                break;

            case '7':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333,  6);
                break;

            case '8':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 7);
                break;

            default:
                break;
        }

#else

        switch (u8Item)
        {
            case '1':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111,   8);
                break;

            case '2':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F,   8);
                break;

            case '3':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333,   8);
                break;

            case '4':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221,   8);
                break;

            case '5':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333,  8);
                break;

            case '6':
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);
                break;

            default:
                break;
        }

#endif
    } while (u8Item != 27);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Rx Test (Slave): wait for messages via interrupt                                                        */
/*  CAN FD mode expects 8 messages; CAN mode expects 6 messages                                           */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_RxTest(void)
{
    uint8_t u8Cnt = 0;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("Start CAN FD bus reception :\n");
    const uint8_t u8TotalMsg = 8;
#else
    printf("Start CAN bus reception :\n");
    const uint8_t u8TotalMsg = 6;
#endif

    do
    {
        while (!g_u8RxCompleteFlag) {}

        CANFD_ShowRecvMessage();
        g_u8RxCompleteFlag = 0;
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
        u8Cnt++;
    } while (u8Cnt < u8TotalMsg);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Tx / Rx Interrupt Function Test                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* Peripheral initialization */
    CANFD_Init();

    printf("+----------------------------------------------------------------------------+\n");
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("|                       CAN FD Function Test                                 |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                             |\n");
    printf("|    The sample code needs two boards. One is master(CAN FD transmitter) and |\n");
    printf("|    the other is slave(CAN FD receiver). Master will send 8 messages with   |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if        |\n");
    printf("|    received data is correct after getting 8 messages data.                 |\n");
#else
    printf("|                       CAN Function Test                                    |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                             |\n");
    printf("|    The sample code needs two boards. One is master(CAN transmitter) and    |\n");
    printf("|    the other is slave(CAN receiver). Master will send 6 messages with      |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if        |\n");
    printf("|    received data is correct after getting 6 messages data.                 |\n");
#endif
    printf("|    Bus-Off recovery feature is enabled for error handling.                 |\n");
    printf("|  Please select Master or Slave test                                        |\n");
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("|  [0] Master(CAN FD transmitter)    [1] Slave(CAN FD receiver)              |\n");
#else
    printf("|  [0] Master(CAN transmitter)       [1] Slave(CAN receiver)                 |\n");
#endif
    printf("+----------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if (u8Item == '0')
        CANFD_TxTest();
    else
        CANFD_RxTest();

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("CAN FD Sample Code End.\n");
#else
    printf("CAN Sample Code End.\n");
#endif

    if (g_u32BusOffRecoveryCounter > 0)
        printf("Total Bus-Off recovery cycles: %u\n", g_u32BusOffRecoveryCounter);
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

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("\nCAN FD bus communication example\r\n");
#else
    printf("\nCAN bus communication example\r\n");
#endif

    CANFD_TxRxINTTest();

    while (1) {}
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
