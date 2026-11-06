/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    An example of interrupt control using CAN/CAN FD bus communication.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

#define OP_MODE  CANFD_OP_CAN_FD_MODE  /* Options: CANFD_OP_CAN_FD_MODE  or  CANFD_OP_CAN_MODE */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

CANFD_FD_MSG_T      g_sRxMsgFrame;
CANFD_FD_MSG_T      g_sTxMsgFrame;
volatile uint8_t   g_u8RxFifoCompleteFlag = 0;
volatile uint8_t   g_u8BusOffFlag = 0;
volatile uint32_t  g_u32BusOffRecoveryCounter = 0;

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
uint8_t CANFD_BusOffRecovery(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD0_IRQ0_IRQHandler(void)
{
    uint32_t u32IntStatus;

    u32IntStatus = CANFD0->IR;
    printf("IR =0x%08X \n", u32IntStatus);

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

    /* Check Bus-Off status */
    if (u32IntStatus & CANFD_IR_BO_Msk)
    {
        if (CANFD0->PSR & CANFD_PSR_BO_Msk)
        {
            printf("Bus-Off detected! Recovery will be triggered on next transmission attempt.\n");
            g_u8BusOffFlag = 1;
        }

        /* Always clear Bus-Off interrupt flag to prevent repeated interrupts */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_BO_Msk);
    }

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* Check Rx FIFO1 New Message interrupt */
    if (u32IntStatus & CANFD_IR_RF1N_Msk)
    {
        /* Receive the Rx Fifo1 buffer */
        CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxMsgFrame);
        g_u8RxFifoCompleteFlag = 1;
        /* Clear Rx FIFO1 New Message interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1N_Msk);
    }
#else
    /* Check Rx FIFO0 New Message interrupt */
    if (u32IntStatus & CANFD_IR_RF0N_Msk)
    {
        /* Receive the Rx Fifo0 buffer */
        CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
        g_u8RxFifoCompleteFlag = 1;
        /* Clear Rx FIFO0 New Message interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0N_Msk);
    }
#endif
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HCLK, CLK_CLKDIV4_CANFD0(1));
    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting*/
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    UartDebugMFP();

    /* Set PC multi-function pins for CAN FD0 RXD and TXD */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_CAN0_RXD;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_CAN0_TXD;

    /* Lock protected registers */
    SYS_LockReg();
}


/*---------------------------------------------------------------------------------------------------------*/
/*  CAN FD Function Test Menu (Master)                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TestItem(void)
{
    printf("\n");
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("+-----------------------------------------------------------+\n");
    printf("|             CAN FD Tx Function Test (Master)              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Standard ID = 0x111           ( Data lenght  8 bytes )|\n");
    printf("| [2] Standard ID = 0x113           ( Data lenght 12 bytes )|\n");
    printf("| [3] Standard ID = 0x22F           ( Data lenght 16 bytes )|\n");
    printf("| [4] Standard ID = 0x333           ( Data lenght 20 bytes )|\n");
    printf("| [5] Extended ID = 0x221           ( Data lenght 24 bytes )|\n");
    printf("| [6] Extended ID = 0x227           ( Data lenght 32 bytes )|\n");
    printf("| [7] Extended ID = 0x3333          ( Data lenght 48 bytes )|\n");
    printf("| [8] Extended ID = 0x44444         ( Data lenght 64 bytes )|\n");
    printf("| Select ID number and master will send message to slave ...|\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
#else
    printf("+-----------------------------------------------------------+\n");
    printf("|              CAN Tx Function Test (Master)                |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Standard ID = 0x111    (Date Lenght 8bytes)           |\n");
    printf("| [2] Standard ID = 0x22F    (Date Lenght 8bytes)           |\n");
    printf("| [3] Standard ID = 0x333    (Date Lenght 8bytes)           |\n");
    printf("| [4] Extended ID = 0x221    (Date Lenght 8bytes)           |\n");
    printf("| [5] Extended ID = 0x3333   (Date Lenght 8bytes)           |\n");
    printf("| [6] Extended ID = 0x44444  (Date Lenght 8bytes)           |\n");
    printf("| Select ID number and master will send message to slave ...|\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
#endif
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN FD Function Tx Test                                                     */
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
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
                /* Standard ID = 0x111, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 0);
#else
                /* Standard ID = 0x111, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 8);
#endif
                break;

            case '2':
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
                /* Standard ID = 0x113, Data length 12 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x113, 1);
#else
                /* Standard ID = 0x22F, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 8);
#endif
                break;

            case '3':
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
                /* Standard ID = 0x22F, Data length 16 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 2);
#else
                /* Standard ID = 0x333, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 8);
#endif
                break;

            case '4':
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
                /* Standard ID = 0x333, Data length 20 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 3);
#else
                /* Extended ID = 0x221, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221, 8);
#endif
                break;

            case '5':
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
                /* Extended ID = 0x221, Data length 24 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221, 4);
#else
                /* Extended ID = 0x3333, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 8);
#endif
                break;

            case '6':
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
                /* Extended ID = 0x227, Data length 32 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x227, 5);
#else
                /* Extended ID = 0x44444, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);
#endif
                break;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
            case '7':
                /* Extended ID = 0x3333, Data length 48 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 6);
                break;

            case '8':
                /* Extended ID = 0x44444, Data length 64 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 7);
                break;
#endif

            default:
                break;
        }

    } while (u8Item != 27);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN FD Send Message Function                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenType)
{
    uint8_t u8Cnt;

    /* Check if CAN FD is in Bus-Off state before transmitting */
    if (g_u8BusOffFlag)
    {
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
        printf("CAN FD is in Bus-Off state. Starting recovery process...\n");
#else
        printf("CAN is in Bus-Off state. Starting recovery process...\n");
#endif

        if (CANFD_BusOffRecovery())
        {
            printf("Bus-Off recovery successful. Proceeding with transmission.\n");
        }
        else
        {
            printf("Bus-Off recovery failed. Cannot transmit.\n");
            return;
        }
    }

    /* Set the ID number */
    psTxMsg->u32Id = u32Id;
    /* Set the frame type */
    psTxMsg->eIdType = eIdType;
    /* Set the frame format attribute */
    psTxMsg->eFrmType = eCANFD_DATA_FRM;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* Set FD frame format attribute */
    psTxMsg->bFDFormat = 1;
    /* Set the bitrate switch attribute */
    psTxMsg->bBitRateSwitch = 1;

    /* Set data length */
    if      (u8LenType == 0) psTxMsg->u32DLC = 8;
    else if (u8LenType == 1) psTxMsg->u32DLC = 12;
    else if (u8LenType == 2) psTxMsg->u32DLC = 16;
    else if (u8LenType == 3) psTxMsg->u32DLC = 20;
    else if (u8LenType == 4) psTxMsg->u32DLC = 24;
    else if (u8LenType == 5) psTxMsg->u32DLC = 32;
    else if (u8LenType == 6) psTxMsg->u32DLC = 48;
    else if (u8LenType == 7) psTxMsg->u32DLC = 64;
#else
    psTxMsg->bBitRateSwitch = 0;
    psTxMsg->u32DLC = u8LenType;
#endif

    g_u8RxFifoCompleteFlag = 0;

    /* use message buffer 0 */
    if (eIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("Data Message(%02u bytes) : ", psTxMsg->u32DLC);

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++)
    {
        psTxMsg->au8Data[u8Cnt] = u8Cnt;
        printf("%02u,", psTxMsg->au8Data[u8Cnt]);
    }

    printf("\n");
#else
    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;
#endif

    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN FD Function Rx Test                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_RxTest(void)
{
    uint8_t u8Cnt = 0;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("Start CAN FD bus reception :\n");
#else
    printf("Start CAN bus reception :\n");
#endif

    do
    {
        while (!g_u8RxFifoCompleteFlag) {}

        CANFD_ShowRecvMessage();
        g_u8RxFifoCompleteFlag = 0;
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
        u8Cnt++;
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    } while (u8Cnt < 8);
#else
    } while (u8Cnt < 6);
#endif
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             Show the CAN FD Received Message Function                                   */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_ShowRecvMessage(void)
{
    uint8_t u8Cnt;

    if (g_sRxMsgFrame.eIdType == eCANFD_SID)
        printf("Rx FIFO1(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
    else
        printf("Rx FIFO1(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

    printf("Message Data(%02u bytes) : ", g_sRxMsgFrame.u32DLC);

    for (u8Cnt = 0; u8Cnt < g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02u ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*                                 CAN FD Bus-Off Recovery Function                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CANFD_BusOffRecovery(void)
{
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("Starting CAN FD Bus-Off recovery sequence...\n");
#else
    printf("Starting CAN Bus-Off recovery sequence...\n");
#endif

    /* CAN FD0 run to initial mode */
    CANFD_RunToNormal(CANFD0, FALSE);

    /* Cancel all transmit requests */
    CANFD0->TXBCR = 0xFFFFFFFF;

    /* Clear all interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, 0xFFFFFFFF);

    /* CAN FD0 run to normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);

    /* 50ms delay after recovery process */
    CLK_SysTickDelay(50000);

    /* Check if recovery was successful by verifying Bus-Off status */
    if (CANFD0->PSR & CANFD_PSR_BO_Msk)
    {
        /* Still in Bus-Off state, recovery failed */
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
        printf("CAN FD Bus-Off recovery failed. Still in Bus-Off state.\n");
#else
        printf("CAN Bus-Off recovery failed. Still in Bus-Off state.\n");
#endif
        /* Recovery failed */
        return 0;
    }
    else
    {
        /* Recovery successful, clear Bus-Off flag */
        g_u8BusOffFlag = 0;
        g_u32BusOffRecoveryCounter++;
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
        printf("CAN FD Bus-Off recovery completed. Recovery count: %u\n", g_u32BusOffRecoveryCounter);
#else
        printf("CAN Bus-Off recovery completed. Recovery count: %u\n", g_u32BusOffRecoveryCounter);
#endif
        /* Recovery successful */
        return 1;
    }
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
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("|    CAN0_TXD(PC5)                         CAN_TXD(Any board)   |\n");
    printf("|    CAN0_RXD(PC4)                         CAN_RXD(Any board)   |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|   CAN0_TX|   CANFD   | CANFD_H |    CANFD  |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|   CAN0_RX|           | CANFD_L |           |CAN_RX            |\n");
#else
    printf("|  CAN0_TXD(PC5)                         CAN_TXD(Any board)     |\n");
    printf("|  CAN0_RXD(PC4)                         CAN_RXD(Any board)     |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|   CAN0_TX|   CAN     |  CAN_H  |    CAN    |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|   CAN0_RX|           |  CAN_L  |           |CAN_RX            |\n");
#endif
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* Get the CAN FD configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 4000000;
#else
    /* Get the CAN configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
#endif

    /* Open the CAN FD feature */
    CANFD_Open(CANFD0, &sCANFD_Config);

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
	printf("CAN FD Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
	printf("CAN FD Data bit rate(bps): %d\n", CANFD_GetDataBitRate(CANFD0));
#else
	printf("CAN Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
#endif

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* receive 0x110~0x11F in CAN FD0 rx fifo1 buffer by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO1_STD_MASK(0x110, 0x7F0));
    /* receive 0x22F in CAN FD0 rx fifo1 buffer by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO1_STD_MASK(0x22F, 0x7FF));
    /* receive 0x333 in CAN FD0 rx fifo1 buffer by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO1_STD_MASK(0x333, 0x7FF));

    /* receive 0x220~0x22F (29-bit id) in CAN FD0 rx fifo1 buffer by setting mask 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO1_EXT_MASK_LOW(0x220), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFF0));
    /* receive 0x3333 (29-bit id) in CAN FD0 rx fifo1 buffer by setting mask 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO1_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* receive 0x44444 (29-bit id) in CAN FD0 rx fifo1 buffer by setting mask 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO1_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter (RX fifo1) */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX fifo1 new message interrupt, Bus-Off interrupt, Error Warning and Error Passive interrupts using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_RF1NE_Msk | CANFD_IE_BOE_Msk | CANFD_IE_EWE_Msk | CANFD_IE_EPE_Msk), 0, 0, 0);
#else
    /* receive 0x110~0x11F in CAN rx fifo0 buffer by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(0x110, 0x7F0));
    /* receive 0x220 ~ 0x22F in CAN rx fifo0 buffer by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO0_STD_MASK(0x22F, 0x7FF));
    /* receive 0x333 in CAN rx fifo0 buffer by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO0_STD_MASK(0x333, 0x7FF));

    /* receive 0x220 (29-bit id) in CAN rx fifo0 buffer by setting mask 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO0_EXT_MASK_LOW(0x220), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFF0));
    /* receive 0x3333 (29-bit id) in CAN rx fifo0 buffer by setting mask 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO0_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* receive 0x44444 (29-bit id) in CAN rx fifo0 buffer by setting mask 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO0_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter (RX fifo0) */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX fifo0 new message interrupt, Bus-Off interrupt, Error Warning and Error Passive interrupts using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_RF0NE_Msk | CANFD_IE_BOE_Msk | CANFD_IE_EWE_Msk | CANFD_IE_EPE_Msk), 0, 0, 0);
#endif

    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                           CAN FD Tx Rx Interrupt Function Test                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* CAN FD interface initialization */
    CANFD_Init();

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("+----------------------------------------------------------------------------+\n");
    printf("|                     CAN FD Function Test                                   |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                             |\n");
    printf("|    The sample code needs two boards. One is master(CAN FD transmitter) and |\n");
    printf("|    the other is slave(CAN FD receiver). Master will send 8 messages with   |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if        |\n");
    printf("|    received data is correct after getting 8 messages data.                 |\n");
    printf("|    Bus-Off recovery feature is enabled for error handling.                 |\n");
    printf("|  Please select Master or Slave test                                        |\n");
    printf("|  [0] Master(CAN FD transmitter)    [1] Slave(CAN FD receiver)              |\n");
    printf("+----------------------------------------------------------------------------+\n\n");
#else
    printf("+--------------------------------------------------------------------------+\n");
    printf("|                      CAN Function Test                                   |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  Description :                                                           |\n");
    printf("|    The sample code needs two boards. One is master(CAN transmitter) and  |\n");
    printf("|    the other is slave(CAN receiver). Master will send 6 messages with    |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if      |\n");
    printf("|    received data is correct after getting 6 messages data.               |\n");
    printf("|    Bus-Off recovery feature is enabled for error handling.               |\n");
    printf("|  Please select Master or Slave test                                      |\n");
    printf("|  [0] Master(CAN transmitter)    [1] Slave(CAN receiver)                  |\n");
    printf("+--------------------------------------------------------------------------+\n\n");
#endif

    u8Item = getchar();

    if (u8Item == '0')
    {
        CANFD_TxTest();
    }
    else
    {
        CANFD_RxTest();
    }

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("CAN FD Sample Code End.\n");
#else
    printf("CAN Sample Code End.\n");
#endif

    if (g_u32BusOffRecoveryCounter > 0)
    {
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

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART for printf */
    UartDebugInit();

    /* print a note to terminal */
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("\nCAN FD mode transmission test\r\n");
#else
    printf("\nCAN bus communication example\r\n");
#endif

    CANFD_TxRxINTTest();

    while (1) {}
}
