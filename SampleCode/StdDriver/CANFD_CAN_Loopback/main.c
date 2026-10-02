/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use CAN mode function to do internal loopback test.
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
void CANFD_TEST_HANDLE(void);
void CANFD00_IRQHandler(void);

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN Line0 interrupt event                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

void CANFD00_IRQHandler(void)
{
    CANFD_TEST_HANDLE();
}
/*---------------------------------------------------------------------------------------------------------*/
/* CAN Callback function                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TEST_HANDLE(void)
{
    printf("IR =0x%08X \n", CANFD0->IR);
    /*Clear the Interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF0N_Msk);
    /*Receive the Rx FIFO0 buffer */
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

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN Tx Rx Function Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_CAN_TxRx_Test(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eFrameIdType, uint32_t u32Id, uint8_t u8Len)
{
    uint8_t u8Cnt;
    /*Set the ID number*/
    psTxMsg->u32Id = u32Id;
    /* Set the ID type */
    psTxMsg->eIdType = eFrameIdType;
    /* Set the frame type */
    psTxMsg->eFrmType = eCANFD_DATA_FRM;
    /*Set FD frame format attribute */
    psTxMsg->bFDFormat = 0;
    /*Set the bitrate switch attribute*/
    psTxMsg->bBitRateSwitch = 0;
    /* Set data length */
    psTxMsg->u32DLC = u8Len;

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;

    g_u8RxFIFO0CompleteFlag = 0;

    /* Use message buffer 0 */
    if (eFrameIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }

    /* Wait the Rx FIFO0 received message */
    while (!g_u8RxFIFO0CompleteFlag)
    {
    }

    printf("Rx FIFO0 : Received message 0x%08X\n", g_sRxMsgFrame.u32Id);
    printf("Message Data : ");

    for (u8Cnt = 0; u8Cnt <  g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02d ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
    memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  CAN Function Test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_CAN_Loopback(void)
{
    uint8_t u8Loop;
    CANFD_FD_T sCANFD_Config;

    /* Get the CAN configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    /* Enable internal loopback mode */
    sCANFD_Config.sBtConfig.bEnableLoopBack = TRUE;
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

    for (u8Loop = 1  ; u8Loop < 8; u8Loop++)
    {
        CANFD_CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x110 + u8Loop, u8Loop);
    }

    CANFD_CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 8);
    CANFD_CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x333, 8);

    for (u8Loop = 1 ; u8Loop < 8; u8Loop++)
    {
        CANFD_CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x220 + u8Loop, u8Loop);
    }

    CANFD_CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 8);
    CANFD_CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);

    NVIC_DisableIRQ(CANFD00_IRQn);
    CANFD_Close(CANFD0);
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

    printf("\n CAN Mode Loopback example\r\n");
    /* CAN Loopback Test */
    CANFD_CAN_Loopback();
    printf("\n CAN Mode Loopback Test Done\r\n");

    while (1) {}
}
