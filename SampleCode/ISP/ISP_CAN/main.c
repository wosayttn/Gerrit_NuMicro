/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/* Add implementations to fix linker warnings from the newlib-nano C library in VSCode-GCC14.3.1 */
void _close(void) {}
void _lseek(void) {}
void _read_r(void) {}
void _write_r(void) {}


#define GPIO_SETMODE(port, pin, u32Mode) ((port)->MODE = ((port)->MODE & ~(0x3ul << ((pin) << 1))) | ((u32Mode) << ((pin) << 1)))

#define CAN_BAUD_RATE                     500000
#define Master_ISP_ID                     0x487
#define Device0_ISP_ID                    0x784
#define CAN_ISP_DtatLength                0x08
#define CAN_RETRY_COUNTS                  0x1fffffff

#define CMD_READ_CONFIG                   0xA2000000
#define CMD_RUN_APROM                     0xAB000000
#define CMD_GET_DEVICEID                  0xB1000000

#define SCB_VTOR_ADDR                     0xE000ED08UL
#define LDROM_ADDR                        0x0F100000

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
/* Declare a CAN message structure */
typedef struct
{
    uint32_t  Address;
    uint32_t  Data;
} STR_CANMSG_ISP_T;

static CANFD_FD_MSG_T g_sRxMsgFrame;
static volatile uint8_t s_u8CANPackageFlag = 0, s_u8CANAckFlag = 0;

void CANFD00_IRQHandler(void);
void SYS_Init(void);
void CAN_Package_ACK(CANFD_T *psCanfd);
void CAN_Init(void);
void ProcessHardFault(void);
void SH_Return(void);
void SendChar_ToUART(int ch);
uint32_t CAN_Parsing_MSG(uint8_t *u8pMsg);
/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD00_IRQHandler(void)
{
    uint32_t u32IIDRstatus;
    u32IIDRstatus = CANFD0->IR;

    /**************************/
    /* Status Change interrupt*/
    /**************************/
    if (u32IIDRstatus & CANFD_IR_RF0N_Msk)
    {
        /*Clear the Interrupt flag */
        CANFD0->IR |= (CANFD_IR_TOO_Msk | CANFD_IR_RF0N_Msk);
        CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
        s_u8CANPackageFlag = 1;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRC48MEN_Msk;

    /* Wait for HIRC clock ready */
    while ((CLK->STATUS & CLK_STATUS_HIRC48MSTB_Msk) != CLK_STATUS_HIRC48MSTB_Msk);

    /* Switch RMC access cycle to maximum value for safe */
    RMC->CYCCTL = 4;

    /* Select HCLK clock source as HIRC48 and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK0SEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC48M;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLK0DIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Switch RMC access cycle to suitable value base on HCLK */
    RMC->CYCCTL = 3;

    SystemCoreClock = __HIRC48; // HCLK
    CyclesPerUs = SystemCoreClock / 1000000; // For SYS_SysTickDelay()
    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HCLK, CLK_CLKDIV5_CANFD0(1));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Tx Msg by Normal Mode Function (With Message RAM)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Package_ACK(CANFD_T *psCanfd)
{
    CANFD_FD_MSG_T  sTxMsgFrame;
    s_u8CANAckFlag = 1;
    /* Send a 11-bit Standard Identifier message */
    sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;
    sTxMsgFrame.eIdType  = eCANFD_SID;
    sTxMsgFrame.u32Id    = Device0_ISP_ID;
    sTxMsgFrame.u32DLC   = CAN_ISP_DtatLength;

    sTxMsgFrame.au32Data[0] = g_sRxMsgFrame.au32Data[0];
    sTxMsgFrame.au32Data[1] = g_sRxMsgFrame.au32Data[1];

    if (CANFD_TransmitTxMsg(psCanfd, 0, &sTxMsgFrame) == FALSE)    // Configure Msg RAM and send the Msg in the RAM
    {
        return;
    }

}

void CAN_Init(void)
{
    CANFD_FD_T sCANFD_Config;
    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /* Set PC multi-function pins for CAN RXD and TXD */
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_CANFD0_RXD;
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_CANFD0_TXD;

    /* Enable PC clock */
    CLK->AHBCLK0 |=  CLK_AHBCLK0_GPCCKEN_Msk;

    /* Set CAN transceiver to high speed mode */
    GPIO_SETMODE(PC, 11, GPIO_MODE_OUTPUT);
    PC11 = 0;

    /*Get the CAN configuration value*/

    sCANFD_Config.sBtConfig.bFDEn = FALSE;
    sCANFD_Config.sBtConfig.bBitRateSwitch = FALSE;
    sCANFD_Config.sBtConfig.bEnableLoopBack = FALSE;
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = CAN_BAUD_RATE;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
    /* CAN FD Standard message ID elements as 12 elements    */
    sCANFD_Config.sElemSize.u32SIDFC = 12;
    /* CAN FD Extended message ID elements as 10 elements    */
    sCANFD_Config.sElemSize.u32XIDFC = 10;
    /* CAN FD TX Buffer elements as 3 elements    */
    sCANFD_Config.sElemSize.u32TxBuf = 3;
    /* CAN FD RX Buffer elements as 3 elements    */
    sCANFD_Config.sElemSize.u32RxBuf = 3;
    /* CAN FD RX FIFO0 elements as 3 elements    */
    sCANFD_Config.sElemSize.u32RxFifo0 = 3;
    /* CAN FD RX FIFO1 elements as 3 elements    */
    sCANFD_Config.sElemSize.u32RxFifo1 = 3;
    /* CAN FD TX Event FOFI elements as 3 elements    */
    sCANFD_Config.sElemSize.u32TxEventFifo = 3;
    /* Set the Standard ID Filter element address     */
    sCANFD_Config.sMRamStartAddr.u32SIDFC_FLSSA =  0;
    /* Set the Extend ID Filter element address       */
    sCANFD_Config.sMRamStartAddr.u32XIDFC_FLESA =  0x30;
    /* Set the TX Buffer element address              */
    sCANFD_Config.sMRamStartAddr.u32TXBC_TBSA   =  0x80;
    /* Set the RX Buffer element address              */
    sCANFD_Config.sMRamStartAddr.u32RXBC_RBSA   =  0x158;
    /* Set the RX FIFO0 element address               */
    sCANFD_Config.sMRamStartAddr.u32RXF0C_F0SA  =  0x230;
    /* Set the RX FIFO1 element address               */
    sCANFD_Config.sMRamStartAddr.u32RXF1C_F1SA  =  0x308;
    /* Set the TX Event FOFI element address           */
    sCANFD_Config.sMRamStartAddr.u32TXEFC_EFSA  =  0x3E0;


    /*Open the CAN feature*/
    CANFD_Open(CANFD0, &sCANFD_Config);

    NVIC_EnableIRQ(CANFD00_IRQn);

    /* Set CAN reveive message */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(Master_ISP_ID, 0x7FF));

    /* Enable RX fifo0 new message interrupt using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk), 0, 0, 0);

    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Enable FMC ISP AP CFG function & clear ISPFF */
    RMC->ISPCTL |= RMC_ISPCTL_ISPEN_Msk | RMC_ISPCTL_APUEN_Msk | RMC_ISPCTL_CFGUEN_Msk | RMC_ISPCTL_ISPFF_Msk;

    /* Init CAN port */
    CAN_Init();

    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1)
    {
        if (s_u8CANPackageFlag == 1)
        {
            break;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

    /* state of update program */
    while (1)
    {
        if (s_u8CANPackageFlag)
        {
            STR_CANMSG_ISP_T *psISPCanMsg = (STR_CANMSG_ISP_T *)&g_sRxMsgFrame.au32Data[0];
            s_u8CANPackageFlag = 0;

            if (psISPCanMsg->Address == CMD_GET_DEVICEID)
            {
                psISPCanMsg->Data = SYS->PDID;
            }
            else if (psISPCanMsg->Address == CMD_READ_CONFIG)
            {
                psISPCanMsg->Data = RMC_Read(psISPCanMsg->Data);
            }
            else if (psISPCanMsg->Address == CMD_RUN_APROM)
            {
                break;
            }
            else
            {
                RMC_Write(psISPCanMsg->Address, psISPCanMsg->Data);
                psISPCanMsg->Data = RMC_Read(psISPCanMsg->Address);
            }

            /* send CAN FD ISP Package (ACK) */
            CAN_Package_ACK(CANFD0);
        }
    }

_APROM:
    RMC_SetVectorPageAddr(RMC_APROM_BASE);
    RMC_SET_APROM_BOOT();
    NVIC_SystemReset();

    /* Trap the CPU */
    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Empty functions for reduce code size to fit  into LDROM & solve the functions are not be defined.      */
/*---------------------------------------------------------------------------------------------------------*/
void ProcessHardFault()
{}

void SH_Return()
{}

void SendChar_ToUART(int ch)
{
    (void)ch;
}
