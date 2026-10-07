/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive CAN message through CAN interface.
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
volatile uint32_t  g_u32BusOffRecoveryCounter = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CAN_Init(void);
void CAN_TxRxTest(void);
uint8_t CAN_BusOffRecovery(void);
uint8_t CAN_CheckBusOffStatus(void);

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
/* Init CAN                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+---------------------------------------------------------------+\n");
    printf("|                      Pin Configure                            |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|   CAN_TXD                              CAN_TXD(Any board)     |\n");
    printf("|   CAN_RXD                              CAN_RXD(Any board)     |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|    CAN_TX|    CAN    |  CAN_H  |    CAN    |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|    CAN_RX|           |  CAN_L  |           |CAN_RX            |\n");
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");

    /* Get the CAN configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
    /* Open the CAN feature */
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* Receive 0x111 in CAN rx message buffer 0 by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_BUFFER_STD(0x111, 0));
    /* Receive 0x22F in CAN rx message buffer 0 by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_BUFFER_STD(0x22F, 0));
    /* Receive 0x333 in CAN rx message buffer 0 by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_BUFFER_STD(0x333, 0));

    /* Receive 0x222 (29-bit id) in CAN rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_BUFFER_EXT_LOW(0x222, 1), CANFD_RX_BUFFER_EXT_HIGH(0x222, 1));
    /* Receive 0x3333 (29-bit id) in CAN rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_BUFFER_EXT_LOW(0x3333, 1), CANFD_RX_BUFFER_EXT_HIGH(0x3333, 1));
    /* Receive 0x44444 (29-bit id) in CAN rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_BUFFER_EXT_LOW(0x44444, 1), CANFD_RX_BUFFER_EXT_HIGH(0x44444, 1));
    /* CAN Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN Bus-Off Status Check Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CAN_CheckBusOffStatus(void)
{
    uint32_t u32IntStatus;

    /* Read interrupt status register */
    u32IntStatus = CANFD0->IR;

    /* Check Bus-Off status */
    if (u32IntStatus & CANFD_IR_BO_Msk)
    {
        if (CANFD0->PSR & CANFD_PSR_BO_Msk)
        {
            printf("Bus-Off detected!\n");
        }

        /* Clear Bus-Off interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_BO_Msk);
        /* Bus-Off detected */
        return 1;
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

    /* No Bus-Off detected */
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN Bus-Off Recovery Function                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CAN_BusOffRecovery(void)
{
    printf("Starting CAN Bus-Off recovery sequence...\n");

    /* CAN FD run to initial mode */
    CANFD_RunToNormal(CANFD0, FALSE);

    /* Cancel all transmit requests */
    CANFD0->TXBCR = 0xFFFFFFFF;

    /* Clear all interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, 0xFFFFFFFF);

    /* CAN FD run to normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);

    /* 50ms delay after recovery process */
    CLK_SysTickDelay(50000);

    /* Check if recovery was successful by verifying Bus-Off status */
    if (CANFD0->PSR & CANFD_PSR_BO_Msk)
    {
        /* Still in Bus-Off state, recovery failed */
        printf("CAN Bus-Off recovery failed. Still in Bus-Off state.\n");
        /* Recovery failed */
        return 0;
    }
    else
    {
        g_u32BusOffRecoveryCounter++;
        printf("CAN Bus-Off recovery completed. Recovery count: %u\n", g_u32BusOffRecoveryCounter);
        /* Recovery successful */
        return 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN Function Test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxRxTest(void)
{
    uint8_t u8Item;
    uint8_t u8Cnt;
    uint8_t u8ErrFlag = 0;
    uint8_t u8TxTestNum = 0;
    uint8_t u8RxTestNum = 0;
    uint8_t u8RxTempLen = 0;
    CANFD_FD_MSG_T      sRxMsgFrame;
    CANFD_FD_MSG_T      sTxMsgFrame;

    /* CAN interface initialization */
    CAN_Init();

    printf("+--------------------------------------------------------------------------+\n");
    printf("|                             CAN Function Test                            |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  Description :                                                           |\n");
    printf("|    The sample code needs two boards. One is master(CAN transmitter)      |\n");
    printf("|    and the other is slave(CAN receiver). Master will send 6 messages     |\n");
    printf("|    with different sizes of data and ID to the slave. Slave will check if |\n");
    printf("|    received data is correct after getting 6 messages data.               |\n");
    printf("|    Bus-Off recovery feature is enabled for error handling.               |\n");
    printf("|  Please select Master or Slave test                                      |\n");
    printf("|  [0] Master(CAN transmitter)    [1] Slave(CAN receiver)                  |\n");
    printf("+--------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if (u8Item == '0')
    {
        /* Send 6 messages with different ID and data size */
        for (u8TxTestNum = 0; u8TxTestNum < 6 ; u8TxTestNum++)
        {
            /* Check if CAN is in Bus-Off state before transmitting */
            if (CAN_CheckBusOffStatus())
            {
                printf("CAN is in Bus-Off state. Starting recovery process...\n");

                if (CAN_BusOffRecovery())
                {
                    printf("Bus-Off recovery successful. Proceeding with transmission.\n");
                }
                else
                {
                    printf("Bus-Off recovery failed. Skipping transmission.\n");
                    continue; /* Skip this transmission and try next */
                }
            }

            printf("Start to CAN Bus Transmitter :\n");

            /* Set the ID Number */
            if (u8TxTestNum == 0)      sTxMsgFrame.u32Id = 0x111;
            else if (u8TxTestNum == 1) sTxMsgFrame.u32Id = 0x22F;
            else if (u8TxTestNum == 2) sTxMsgFrame.u32Id = 0x333;
            else if (u8TxTestNum == 3) sTxMsgFrame.u32Id = 0x222;
            else if (u8TxTestNum == 4) sTxMsgFrame.u32Id = 0x3333;
            else if (u8TxTestNum == 5) sTxMsgFrame.u32Id = 0x44444;

            /*Set the ID type*/
            if (u8TxTestNum < 3)
                sTxMsgFrame.eIdType = eCANFD_SID;
            else
                sTxMsgFrame.eIdType = eCANFD_XID;

            /*Set the frame type*/
            sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;
            /* Set the bitrate switch */
            sTxMsgFrame.bBitRateSwitch = 0;

            /* Set the data length */
            if(u8TxTestNum == 0  ||  u8TxTestNum == 3)     sTxMsgFrame.u32DLC = 2;
            else if(u8TxTestNum == 1 || u8TxTestNum == 4)  sTxMsgFrame.u32DLC = 4;
            else if(u8TxTestNum == 2 || u8TxTestNum == 5)  sTxMsgFrame.u32DLC = 8;

            for(u8Cnt = 0; u8Cnt < sTxMsgFrame.u32DLC; u8Cnt++) sTxMsgFrame.au8Data[u8Cnt] = u8Cnt + u8TxTestNum;

            if (u8TxTestNum < 3)
                printf("Send to transmit message 0x%08x (11-bit)\n", sTxMsgFrame.u32Id);
            else
                printf("Send to transmit message 0x%08x (29-bit)\n", sTxMsgFrame.u32Id);

            /* Use message buffer 0 */
            if (CANFD_TransmitTxMsg(CANFD0, 0, &sTxMsgFrame) != eCANFD_TRANSMIT_SUCCESS)
            {
                printf("Failed to transmit message\n");

                /* Check if failure was due to bus-off condition */
                if (CAN_CheckBusOffStatus())
                {
                    printf("Transmission failed due to Bus-Off condition.\n");
                }
            }
            else
            {
                /* Check for any error conditions after successful transmission */
                CAN_CheckBusOffStatus();
                printf("Message transmitted successfully.\n");
            }

        }

        printf("\n Transmit Done\n");

        if (g_u32BusOffRecoveryCounter > 0)
        {
            printf("Total Bus-Off recovery cycles: %u\n", g_u32BusOffRecoveryCounter);
        }
    }
    else
    {
        printf("Start to CAN Bus Receiver :\n");

        /* Receive  6 messages with different ID and data size */
        do
        {
            /* Check for any received messages on CAN message buffer 0 */
            if (CANFD_ReadRxBufMsg(CANFD0, 0, &sRxMsgFrame) == eCANFD_RECEIVE_SUCCESS)
            {
                printf("Rx buf 0: Received message 0x%08X (11-bit)\r\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for (u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02d ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if (sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                    {
                        u8ErrFlag = 1;
                    }
                }

                printf(" \n\n");

                /* Check Standard ID number */
                if ((sRxMsgFrame.u32Id != 0x111) && (sRxMsgFrame.u32Id != 0x22F) && (sRxMsgFrame.u32Id != 0x333))
                {
                    u8ErrFlag = 1;
                }

                if(u8RxTestNum == 0)      u8RxTempLen = 2;
                else if(u8RxTestNum == 1) u8RxTempLen = 4;
                else if(u8RxTestNum == 2) u8RxTempLen = 8;

                /* Check Data length */
                if ((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_SID))
                {
                    u8ErrFlag = 1;
                }

                if (u8ErrFlag == 1)
                {
                    printf("CAN STD ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;
            }

            /* Check for any received messages on CAN message buffer 1 */
            if (CANFD_ReadRxBufMsg(CANFD0, 1, &sRxMsgFrame) == eCANFD_RECEIVE_SUCCESS)
            {

                printf("Rx buf 1: Received message 0x%08X (29-bit)\r\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for (u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02d ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if (sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                    {
                        u8ErrFlag = 1;
                    }
                }

                printf(" \n\n");

                /* Check Extend ID number */
                if ((sRxMsgFrame.u32Id  != 0x222) && (sRxMsgFrame.u32Id  != 0x3333) && (sRxMsgFrame.u32Id != 0x44444))
                {
                    u8ErrFlag = 1;
                }

                if(u8RxTestNum == 3)      u8RxTempLen = 2;
                else if(u8RxTestNum == 4) u8RxTempLen = 4;
                else if(u8RxTestNum == 5) u8RxTempLen = 8;

                /* Check Data length */
                if ((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_XID))
                {
                    u8ErrFlag = 1;
                }

                if (u8ErrFlag == 1)
                {
                    printf("CAN EXD ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;

            }
        }
        while (u8RxTestNum < 6);

        printf("\n Receive OK & Check OK\n");

        if (g_u32BusOffRecoveryCounter > 0)
        {
            printf("Total Bus-Off recovery cycles: %u\n", g_u32BusOffRecoveryCounter);
        }
    }
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
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    CAN mode transmission test          |\n");
    printf("+----------------------------------------+\n");

    /* CAN sample function */
    CAN_TxRxTest();

    while (1) {}
}
