/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    An example of interrupt control using CAN FD bus communication.
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
CANFD_FD_MSG_T      g_sRxMsgFrame;
CANFD_FD_MSG_T      g_sTxMsgFrame;
volatile uint8_t   g_u8RxFIFO1CompleteFlag = 0;

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

/*---------------------------------------------------------------------------------------------------------*/
/*  ISR to handle CAN FD Line 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD00_IRQHandler(void)
{
    printf("IR =0x%08X \n", CANFD0->IR);
    /*Clear the Interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF1N_Msk);
    /* Receive the Rx FIFO1 buffer */
    CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxMsgFrame);
    g_u8RxFIFO1CompleteFlag = 1;
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
/* CAN FD Function Test Menu (Master)                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TestItem(void)
{
    printf("\n");
    printf("+------------------------------------------------------------+\n");
    printf("|              CAN FD Tx Function Test (Master)              |\n");
    printf("+------------------------------------------------------------+\n");
    printf("| [1] Standard ID = 0x111           ( Data length  8 bytes ) |\n");
    printf("| [2] Standard ID = 0x113           ( Data length 12 bytes ) |\n");
    printf("| [3] Standard ID = 0x22F           ( Data length 16 bytes ) |\n");
    printf("| [4] Standard ID = 0x333           ( Data length 20 bytes ) |\n");
    printf("| [5] Extended ID = 0x220           ( Data length 24 bytes ) |\n");
    printf("| [6] Extended ID = 0x227           ( Data length 32 bytes ) |\n");
    printf("| [7] Extended ID = 0x3333          ( Data length 48 bytes ) |\n");
    printf("| [8] Extended ID = 0x44444         ( Data length 64 bytes ) |\n");
    printf("| Select ID number and master will send message to slave ... |\n");
    printf("+------------------------------------------------------------+\n");
    printf("| Quit                                               - [ESC] |\n");
    printf("+------------------------------------------------------------+\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN FD Function Tx Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxTest(void)
{
    uint8_t u8Item;

    do
    {
        CANFD_TestItem();
        u8Item = getchar();

        switch (u8Item)
        {

        case '1':
            /* Standard ID = 0x111, Data length 8 bytes */
            CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 0);
            break;

        case '2':
            /* Standard ID = 0x113, Data length 12 bytes */
            CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x113, 1);
            break;

        case '3':
            /* Standard ID = 0x22F, Data length 16 bytes */
            CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 2);
            break;

        case '4':
            /* Standard ID = 0x333, Data length 20 bytes */
            CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 3);
            break;

        case '5':
            /* Extend ID = 0x220, Data length 24 bytes */
            CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x220, 4);
            break;

        case '6':
            /* Extend ID = 0x227, Data length 32 bytes */
            CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x227, 5);
            break;

        case '7':
            /* Extend ID = 0x3333, Data length 48 bytes */
            CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 6);
            break;

        case '8':
            /* Extend ID = 0x44444, Data length 64 bytes */
            CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 7);
            break;

        default:
            break;

        }
    }
    while (u8Item != 27);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Send CAN FD Message Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenType)
{
    uint8_t u8Cnt;

    /* Set the ID number */
    psTxMsg->u32Id = u32Id;
    /* Set the ID type */
    psTxMsg->eIdType = eIdType;
    /*Set FD frame format attribute */
    psTxMsg->bFDFormat = 1;
    /*Set the bitrate switch attribute*/
    psTxMsg->bBitRateSwitch = 1;

    /*Set data length*/
    if (u8LenType == 0)      psTxMsg->u32DLC = 8;
    else if (u8LenType == 1) psTxMsg->u32DLC = 12;
    else if (u8LenType == 2) psTxMsg->u32DLC = 16;
    else if (u8LenType == 3) psTxMsg->u32DLC = 20;
    else if (u8LenType == 4) psTxMsg->u32DLC = 24;
    else if (u8LenType == 5) psTxMsg->u32DLC = 32;
    else if (u8LenType == 6) psTxMsg->u32DLC = 48;
    else if (u8LenType == 7) psTxMsg->u32DLC = 64;

    g_u8RxFIFO1CompleteFlag = 0;

    /* Use message buffer 0 */
    if (eIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    printf("Data Message(%02d bytes) : ", psTxMsg->u32DLC);

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++)
    {
        psTxMsg->au8Data[u8Cnt] = u8Cnt;
        printf("%02d,", psTxMsg->au8Data[u8Cnt]);
    }

    printf("\n");

    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN FD Function Rx Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_RxTest(void)
{
    uint8_t u8Cnt = 0;

    printf("Start CAN FD bus reception :\n");

    do
    {
        while(!g_u8RxFIFO1CompleteFlag);
        CANFD_ShowRecvMessage();
        g_u8RxFIFO1CompleteFlag = 0;
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
        u8Cnt++;
    }
    while (u8Cnt < 8);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Show the CAN FD Message Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_ShowRecvMessage(void)
{
    uint8_t u8Cnt;

    if (g_sRxMsgFrame.eIdType == eCANFD_SID)
        printf("Rx FIFO1(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
    else
        printf("Rx FIFO1(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

    printf("Message Data(%02d bytes) : ", g_sRxMsgFrame.u32DLC);

    for (u8Cnt = 0; u8Cnt <  g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02d ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init CAN FD                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+-------------------------------------------------------------+\n");
    printf("|                        Pin Configure                        |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|      CAN_TXD                            CAN_TXD(Any board)  |\n");
    printf("|      CAN_RXD                            CAN_RXD(Any board)  |\n");
    printf("|         |-----------|  CANBUS  |-----------|                |\n");
    printf("|  ------>|           |<-------->|           |<------         |\n");
    printf("|   CAN_TX|   CANFD   |  CAN_H   |   CANFD   |CAN_TX          |\n");
    printf("|         |Transceiver|          |Transceiver|                |\n");
    printf("|  <------|           |<-------->|           |------>         |\n");
    printf("|   CAN_RX|           |  CAN_L   |           |CAN_RX          |\n");
    printf("|         |-----------|          |-----------|                |\n");
    printf("+-------------------------------------------------------------+\n\n");

    /*Get the CAN FD configuration value*/
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 4000000;

    /*Open the CAN FD feature*/
    CANFD_Open(CANFD0, &sCANFD_Config);

    NVIC_EnableIRQ(CANFD00_IRQn);

    /* Receive 0x110~0x11F (11-bit id) in CAN FD rx FIFO1 buffer by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO1_STD_MASK(0x110, 0x7F0));
    /* Receive 0x22F (11-bit id) in CAN FD rx FIFO1 buffer by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO1_STD_MASK(0x22F, 0x7FF));
    /* Receive 0x333 (11-bit id) in CAN FD rx FIFO1 buffer by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO1_STD_MASK(0x333, 0x7FF));

    /* Receive 0x220~0x22F (29-bit id) in CAN FD rx FIFO1 buffer by setting mask 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO1_EXT_MASK_LOW(0x220), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFF0));
    /* Receive 0x3333 (29-bit id) in CAN FD rx FIFO1 buffer by setting mask 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO1_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Receive 0x44444 (29-bit id) in CAN FD rx FIFO1 buffer by setting mask 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO1_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX FIFO1 new message interrupt using interrupt line 0 */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF1NE_Msk), 0, 0, 0);
    /* CAN FD Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                           CAN FD Tx Rx Interrupt Function Test                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* CAN FD interface initialization*/
    CANFD_Init();

    printf("+----------------------------------------------------------------------------+\n");
    printf("|                     CAN FD Function Test                                   |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                             |\n");
    printf("|    The sample code needs two boards. One is master(CAN FD transmitter) and |\n");
    printf("|    the other is slave(CAN FD receiver). Master will send 8 messages with   |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if        |\n");
    printf("|    received data is correct after getting 8 messages data.                 |\n");
    printf("|  Please select Master or Slave test                                        |\n");
    printf("|  [0] Master(CAN FD transmitter)    [1] Slave(CAN FD receiver)              |\n");
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

    printf("\n CAN FD mode transmission test\r\n");

    CANFD_TxRxINTTest();

    while (1) {}
}
