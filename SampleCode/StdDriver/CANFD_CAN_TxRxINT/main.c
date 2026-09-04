/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    An example of interrupt control using CAN bus communication.
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
volatile uint8_t   g_u8RxFIFO0CompleteFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CAN_ShowRecvMessage(void);
void CAN_RxTest(void);
void CAN_TxTest(void);
void CAN_TxRxINTTest(void);
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len);

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN Line 0 interrupt event                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD00_IRQHandler(void)
{
    printf("IR =0x%08X \n", CANFD0->IR);
    /*Clear the Interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF1N_Msk);
    /* Receive the Rx FIFO0 buffer */
    CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
    g_u8RxFIFO0CompleteFlag = 1;
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
/* CAN Function Test Menu (Master)                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TestItem(void)
{
    printf("\n");
    printf("+------------------------------------------------------------+\n");
    printf("|                CAN Tx Function Test (Master)               |\n");
    printf("+------------------------------------------------------------+\n");
    printf("| [1] Standard ID = 0x111            ( Data length 8 bytes ) |\n");
    printf("| [2] Standard ID = 0x22F            ( Data length 8 bytes ) |\n");
    printf("| [3] Standard ID = 0x333            ( Data length 8 bytes ) |\n");
    printf("| [4] Extended ID = 0x221            ( Data length 8 bytes ) |\n");
    printf("| [5] Extended ID = 0x3333           ( Data length 8 bytes ) |\n");
    printf("| [6] Extended ID = 0x44444          ( Data length 8 bytes ) |\n");
    printf("| Select ID number and master will send message to slave ... |\n");
    printf("+------------------------------------------------------------+\n");
    printf("| Quit                                               - [ESC] |\n");
    printf("+------------------------------------------------------------+\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN Function Tx Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxTest(void)
{
    uint8_t u8Item;

    do
    {
        CAN_TestItem();
        u8Item = getchar();

        switch (u8Item)
        {

        case '1':
            /* Standard ID = 0x111, Data length 8 bytes */
            CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 8);
            break;

        case '2':
            /* Standard ID = 0x22F, Data length 8 bytes */
            CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 8);
            break;

        case '3':
            /* Standard ID = 0x333, Data length 8 bytes */
            CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 8);
            break;

        case '4':
            /* Extend ID = 0x221, Data length 8 bytes */
            CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221, 8);
            break;

        case '5':
            /* Extend ID = 0x3333, Data length 8 bytes */
            CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 8);
            break;

        case '6':
            /* Extend ID = 0x44444, Data length 8 bytes */
            CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);
            break;

        default:
            break;

        }
    }
    while (u8Item != 27);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Send CAN Message Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len)
{
    uint8_t u8Cnt;

    /* Set the ID number */
    psTxMsg->u32Id = u32Id;
    /* Set the ID type */
    psTxMsg->eIdType = eIdType;
    /* Set the frame type */
    psTxMsg->eFrmType = eCANFD_DATA_FRM;
    /* Set the bitrate switch attribute */
    psTxMsg->bBitRateSwitch = 0;
    /* Set data length */
    psTxMsg->u32DLC = u8Len;

    for(u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;

    g_u8RxFIFO0CompleteFlag = 0;

    /* Use message buffer 0 */
    if (eIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    if(CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN Function Rx Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_RxTest(void)
{
    uint8_t u8Cnt = 0;

    printf("Start CAN bus reception :\n");

    do
    {
        while(!g_u8RxFIFO0CompleteFlag);
        CAN_ShowRecvMessage();
        g_u8RxFIFO0CompleteFlag = 0;
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
        u8Cnt++;
    }
    while(u8Cnt < 6);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Show the CAN Message Function                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_ShowRecvMessage(void)
{
    uint8_t u8Cnt;

    if (g_sRxMsgFrame.eIdType == eCANFD_SID)
        printf("Rx FIFO0(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
    else
        printf("Rx FIFO0(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

    printf("Message Data(%02d bytes) : ", g_sRxMsgFrame.u32DLC);

    for (u8Cnt = 0; u8Cnt <  g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02d ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init CAN                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+-------------------------------------------------------------+\n");
    printf("|                        Pin Configure                        |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|      CAN_TXD                            CAN_TXD(Any board)  |\n");
    printf("|      CAN_RXD                            CAN_RXD(Any board)  |\n");
    printf("|         |-----------|  CANBUS  |-----------|                |\n");
    printf("|  ------>|           |<-------->|           |<------         |\n");
    printf("|   CAN_TX|    CAN    |  CAN_H   |    CAN    |CAN_TX          |\n");
    printf("|         |Transceiver|          |Transceiver|                |\n");
    printf("|  <------|           |<-------->|           |------>         |\n");
    printf("|   CAN_RX|           |  CAN_L   |           |CAN_RX          |\n");
    printf("|         |-----------|          |-----------|                |\n");
    printf("+-------------------------------------------------------------+\n\n");

    /* Get the CAN configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;

    /* Open the CAN feature */
    CANFD_Open(CANFD0, &sCANFD_Config);

    NVIC_EnableIRQ(CANFD00_IRQn);

    /* Receive 0x110~0x11F in CAN rx FIFO0 buffer by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(0x110, 0x7F0));
    /* Receive 0x22F in CAN rx FIFO0 buffer by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO0_STD_MASK(0x22F, 0x7FF));
    /* Receive 0x333 in CAN rx FIFO0 buffer by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO0_STD_MASK(0x333, 0x7FF));

    /* Receive 0x220~0x22F (29-bit id) in CAN rx FIFO0 buffer by setting mask 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO0_EXT_MASK_LOW(0x220), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFF0));
    /* Receive 0x3333 (29-bit id) in CAN rx FIFO0 buffer by setting mask 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO0_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Receive 0x44444 (29-bit id) in CAN rx FIFO0 buffer by setting mask 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO0_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter(RX FIFO0) */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX FIFO0 new message interrupt using interrupt line 0 */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk), 0, 0, 0);
    /* CAN Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN Tx Rx Interrupt Function Test                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* CAN interface initialization */
    CAN_Init();

    printf("+----------------------------------------------------------------------------+\n");
    printf("|                              CAN Function Test                             |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                             |\n");
    printf("|    The sample code needs two boards. One is master(CAN transmitter) and    |\n");
    printf("|    the other is slave(CAN receiver). Master will send 6 messages with      |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if        |\n");
    printf("|    received data is correct after getting 6 messages data.                 |\n");
    printf("|  Please select Master or Slave test                                        |\n");
    printf("|  [0] Master(CAN transmitter)    [1] Slave(CAN receiver)                    |\n");
    printf("+----------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if (u8Item == '0')
    {
        CAN_TxTest();
    }
    else
    {
        CAN_RxTest();
    }

    printf("CAN Sample Code End.\n");
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

    printf("\n CAN mode transmission test\r\n");

    CAN_TxRxINTTest();

    while (1) {}
}
