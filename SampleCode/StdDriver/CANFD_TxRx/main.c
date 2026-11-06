/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive CAN / CAN FD messages through CAN interface.
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
volatile uint8_t   g_u8BusOffFlag             = 0;
volatile uint32_t  g_u32BusOffRecoveryCounter = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_Init(void);
void CANFD_TxRxTest(void);
uint8_t CANFD_BusOffRecovery(void);
uint8_t CANFD_CheckBusOffStatus(void);
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

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
    printf("|  CAN0_TXD(PC5)                         CAN_TXD(Any board)    |\n");
    printf("|  CAN0_RXD(PC4)                         CAN_RXD(Any board)    |\n");
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

    /* Common: nominal bitrate and open peripheral */
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    CANFD_Open(CANFD0, &sCANFD_Config);

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("CAN FD Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
    printf("CAN FD Data bit rate(bps): %d\n", CANFD_GetDataBitRate(CANFD0));
#else
    printf("CAN Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
#endif

    /* Receive Standard IDs in dedicated Rx buffer 0 -- same IDs for both modes */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_BUFFER_STD(0x111, 0));
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_BUFFER_STD(0x22F, 0));
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_BUFFER_STD(0x333, 0));

    /* Receive Extended IDs in dedicated Rx buffer 1 -- same IDs for both modes */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_BUFFER_EXT_LOW(0x222,   1), CANFD_RX_BUFFER_EXT_HIGH(0x222,   1));
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_BUFFER_EXT_LOW(0x3333,  1), CANFD_RX_BUFFER_EXT_HIGH(0x3333,  1));
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_BUFFER_EXT_LOW(0x44444, 1), CANFD_RX_BUFFER_EXT_HIGH(0x44444, 1));

    /* Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Check Bus-Off status and clear related interrupt flags                                                  */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CANFD_CheckBusOffStatus(void)
{
    uint32_t u32IntStatus = CANFD0->IR;

    /* Check Bus-Off status */
    if (u32IntStatus & CANFD_IR_BO_Msk)
    {
        if (CANFD0->PSR & CANFD_PSR_BO_Msk)
        {
            printf("Bus-Off detected!\n");
            g_u8BusOffFlag = 1;
        }

        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_BO_Msk);
        return 1;   /* Bus-Off detected */
    }

    /* Check Error Warning status */
    if (u32IntStatus & CANFD_IR_EW_Msk)
    {
        printf("Error warning flag is set.\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_EW_Msk);
    }

    /* Check Error Passive status */
    if (u32IntStatus & CANFD_IR_EP_Msk)
    {
        printf("Error passive flag is set.\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_EP_Msk);
    }

    return 0;   /* No Bus-Off detected */
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
/* Tx / Rx Test                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRxTest(void)
{
    uint8_t u8Item;
    uint8_t u8Cnt;
    CANFD_FD_MSG_T sRxMsgFrame;
    CANFD_FD_MSG_T sTxMsgFrame;

    /* Peripheral initialization */
    CANFD_Init();

    printf("+--------------------------------------------------------------------------+\n");
    printf("|                             Function Test                                |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  Description :                                                           |\n");
    printf("|    The sample code needs two boards. One is master(transmitter)          |\n");
    printf("|    and the other is slave(receiver). Master will send 6 messages with    |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if      |\n");
    printf("|    received data is correct after getting 6 messages data.               |\n");
    printf("|    Bus-Off recovery feature is enabled for error handling.               |\n");
    printf("|  Please select Master or Slave test                                      |\n");
    printf("|  [0] Master(transmitter)    [1] Slave(receiver)                          |\n");
    printf("+--------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if (u8Item == '0')
    {
        uint8_t u8TxTestNum = 0;

        /* Send 6 messages with different ID and data size */
        for (u8TxTestNum = 0; u8TxTestNum < 6; u8TxTestNum++)
        {
            /* Check Bus-Off status before each transmission */
            CANFD_CheckBusOffStatus();

            if (g_u8BusOffFlag)
            {
                printf("Bus-Off state detected. Starting recovery process...\n");

                if (CANFD_BusOffRecovery())
                    printf("Bus-Off recovery successful. Proceeding with transmission.\n");
                else
                {
                    printf("Bus-Off recovery failed. Skipping transmission.\n");
                    continue;
                }
            }

            printf("Start to Transmitter :\n");

            /* Set the ID Number -- same IDs for both modes */
            if (u8TxTestNum == 0) sTxMsgFrame.u32Id = 0x111;
            else if (u8TxTestNum == 1) sTxMsgFrame.u32Id = 0x22F;
            else if (u8TxTestNum == 2) sTxMsgFrame.u32Id = 0x333;
            else if (u8TxTestNum == 3) sTxMsgFrame.u32Id = 0x222;
            else if (u8TxTestNum == 4) sTxMsgFrame.u32Id = 0x3333;
            else if (u8TxTestNum == 5) sTxMsgFrame.u32Id = 0x44444;

            /* Set ID type: first 3 are Standard, last 3 are Extended */
            sTxMsgFrame.eIdType = (u8TxTestNum < 3) ? eCANFD_SID : eCANFD_XID;

            /* Set frame type -- common */
            sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
            /* CAN FD: enable FD format and bitrate switching; larger DLC */
            sTxMsgFrame.bFDFormat      = 1;
            sTxMsgFrame.bBitRateSwitch = 1;

            if (u8TxTestNum == 0 || u8TxTestNum == 3) sTxMsgFrame.u32DLC = 16;
            else if (u8TxTestNum == 1 || u8TxTestNum == 4) sTxMsgFrame.u32DLC = 32;
            else if (u8TxTestNum == 2 || u8TxTestNum == 5) sTxMsgFrame.u32DLC = 64;

#else
            /* CAN: classic frame, no bitrate switching; smaller DLC */
            sTxMsgFrame.bBitRateSwitch = 0;

            if (u8TxTestNum == 0 || u8TxTestNum == 3) sTxMsgFrame.u32DLC = 2;
            else if (u8TxTestNum == 1 || u8TxTestNum == 4) sTxMsgFrame.u32DLC = 4;
            else if (u8TxTestNum == 2 || u8TxTestNum == 5) sTxMsgFrame.u32DLC = 8;

#endif

            /* Fill payload -- common */
            for (u8Cnt = 0; u8Cnt < sTxMsgFrame.u32DLC; u8Cnt++)
                sTxMsgFrame.au8Data[u8Cnt] = u8Cnt + u8TxTestNum;

            if (u8TxTestNum < 3)
                printf("Send to transmit message 0x%08x (11-bit)\n", sTxMsgFrame.u32Id);
            else
                printf("Send to transmit message 0x%08x (29-bit)\n", sTxMsgFrame.u32Id);

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
            /* CAN FD: also print the full payload before transmitting */
            printf("Data Message : ");

            for (u8Cnt = 0; u8Cnt < sTxMsgFrame.u32DLC; u8Cnt++)
                printf("%02u,", sTxMsgFrame.au8Data[u8Cnt]);

            printf("\n\n");
#endif

            /* Transmit via message buffer 0 -- common */
            if (CANFD_TransmitTxMsg(CANFD0, 0, &sTxMsgFrame) != eCANFD_TRANSMIT_SUCCESS)
            {
                printf("Failed to transmit message\n");

                if (CANFD_CheckBusOffStatus())
                    printf("Transmission failed due to Bus-Off condition.\n");
            }
            else
            {
                CANFD_CheckBusOffStatus();
                printf("Message transmitted successfully.\n");
            }
        }

        printf("\n Transmit Done\n");

        if (g_u32BusOffRecoveryCounter > 0)
            printf("Total Bus-Off recovery cycles: %u\n", g_u32BusOffRecoveryCounter);
    }
    else
    {
        uint8_t u8ErrFlag   = 0;
        uint8_t u8RxTestNum = 0;
        uint8_t u8RxTempLen = 0;

        printf("Start to Bus Receiver :\n");

        /* Receive 6 messages with different ID and data size */
        do
        {
            /* Check Rx dedicated buffer 0 (Standard ID frames) */
            if (CANFD_ReadRxBufMsg(CANFD0, 0, &sRxMsgFrame) == eCANFD_RECEIVE_SUCCESS)
            {
                printf("Rx buf 0: Received message 0x%08X (11-bit)\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for (u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02u ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if (sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                        u8ErrFlag = 1;
                }

                printf(" \n\n");

                /* Check Standard ID value */
                if ((sRxMsgFrame.u32Id != 0x111) && (sRxMsgFrame.u32Id != 0x22F) && (sRxMsgFrame.u32Id != 0x333))
                    u8ErrFlag = 1;

                /* Check expected DLC */
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)

                if (u8RxTestNum == 0) u8RxTempLen = 16;
                else if (u8RxTestNum == 1) u8RxTempLen = 32;
                else if (u8RxTestNum == 2) u8RxTempLen = 64;

#else

                if (u8RxTestNum == 0) u8RxTempLen = 2;
                else if (u8RxTestNum == 1) u8RxTempLen = 4;
                else if (u8RxTestNum == 2) u8RxTempLen = 8;

#endif

                if ((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_SID))
                    u8ErrFlag = 1;

                if (u8ErrFlag == 1)
                {
                    printf("STD ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;
            }

            /* Check Rx dedicated buffer 1 (Extended ID frames) */
            if (CANFD_ReadRxBufMsg(CANFD0, 1, &sRxMsgFrame) == eCANFD_RECEIVE_SUCCESS)
            {
                printf("Rx buf 1: Received message 0x%08X (29-bit)\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for (u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02u ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if (sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                        u8ErrFlag = 1;
                }

                printf(" \n\n");

                /* Check Extended ID value */
                if ((sRxMsgFrame.u32Id != 0x222) && (sRxMsgFrame.u32Id != 0x3333) && (sRxMsgFrame.u32Id != 0x44444))
                    u8ErrFlag = 1;

                /* Check expected DLC */
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)

                if (u8RxTestNum == 3) u8RxTempLen = 16;
                else if (u8RxTestNum == 4) u8RxTempLen = 32;
                else if (u8RxTestNum == 5) u8RxTempLen = 64;

#else

                if (u8RxTestNum == 3) u8RxTempLen = 2;
                else if (u8RxTestNum == 4) u8RxTempLen = 4;
                else if (u8RxTestNum == 5) u8RxTempLen = 8;

#endif

                if ((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_XID))
                    u8ErrFlag = 1;

                if (u8ErrFlag == 1)
                {
                    printf("EXT ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;
            }
        } while (u8RxTestNum < 6);

        printf("\n Receive OK & Check OK\n");

        if (g_u32BusOffRecoveryCounter > 0)
            printf("Total Bus-Off recovery cycles: %u\n", g_u32BusOffRecoveryCounter);
    }
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

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("+----------------------------------------+\n");
    printf("|      CAN FD mode transmission test     |\n");
    printf("+----------------------------------------+\n");
#else
    printf("+---------------------------------------+\n");
    printf("|        CAN mode transmission test     |\n");
    printf("+---------------------------------------+\n");
#endif

    CANFD_TxRxTest();

    while (1) {}
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
