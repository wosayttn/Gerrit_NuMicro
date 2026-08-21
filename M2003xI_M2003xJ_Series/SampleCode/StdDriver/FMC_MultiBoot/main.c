/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a multi-boot system to boot from different applications in APROM.
 *           4 APROM code are implemented in this sample code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved..
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if !defined(__ICCARM__) && !defined(__GNUC__)
extern uint32_t Image$$RO$$Base;
#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 40 MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_HCLKDIV_HCLK(1));

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV_DIV2 | CLK_PCLKDIV_PCLK1DIV_DIV2);

    /* Set core clock to 40MHz */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL_UART0SEL_HIRC, CLK_UARTDIV_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while(1)
    {
        if ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            return (UART0->DAT);
        }
    }
}

/*
 * @returns     Send value from UART debug port
 * @details     Send a target char to UART debug port .
 */
static void SendChar_ToUART(int ch)
{
    while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    UART0->DAT = ch;
    if(ch == '\n')
    {
        while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
        UART0->DAT = '\r';
    }
}

static void PutString(char *str)
{
    while (*str != '\0')
    {
        SendChar_ToUART(*str++);
    }
}

static void PutHexNumber(uint32_t u32Num)
{
    int32_t i = 28;
    uint32_t Digit;
    PutString("0x");
    do
    {
        Digit = (u32Num >> i) & 0x0F;
        if(Digit != 0)
            break;
        i = i - 4;
    }
    while(i!=0);

    while (i >= 0)
    {
        Digit =  (u32Num >> i) & 0x0F;
        if(Digit < 10)
            SendChar_ToUART('0'+Digit);
        else
            SendChar_ToUART('A'+Digit-10);
        i = i - 4;
    }
}

int32_t main(void)
{
    uint8_t u8Ch;
    uint32_t u32Data;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC_Open();

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            FMC_MultiBoot, RO=0x0, ICE download algorithm to APROM
            FMC_Boot0, RO=0x2000, ICE download algorithm to APROM
            FMC_Boot1, RO=0x4000, ICE download algorithm to APROM
            FMC_Boot2, RO=0x6000, ICE download algorithm to APROM
        2. Reset MCU to execute FMC_MultiBoot.
    */

    PutString("\n\n");
    PutString("+---------------------------+\n");
    PutString("|   Multi-Boot Sample Code  |\n");
    PutString("+---------------------------+\n");
#if defined(__BASE__)
    PutString("Boot from 0x0\n");
#endif
#if defined(__BOOT0__)
    PutString("Boot from 0x2000\n");
#endif
#if defined(__BOOT1__)
    PutString("Boot from 0x4000\n");
#endif
#if defined(__BOOT2__)
    PutString("Boot from 0x6000\n");
#endif

    u32Data = FMC_GetVECMAP();
    PutString("VECMAP = ");
    PutHexNumber(u32Data);
    PutString("\n");

    PutString("Select one boot image: \n");
#if defined(__ARMCC_VERSION)
    PutString("[0] Boot 0, base = 0x2000\n");
#endif
    PutString("[1] Boot 1, base = 0x4000\n");
#if defined(__ARMCC_VERSION)
    PutString("[2] Boot 2, base = 0x6000\n");
#endif
    PutString("[Others] Boot, base = 0x0\n");

    u8Ch = GetChar();

    switch (u8Ch)
    {
    case '0':
        FMC_SetVectorPageAddr(0x2000);
        break;

    case '1':
        FMC_SetVectorPageAddr(0x4000);
        break;

    case '2':
        FMC_SetVectorPageAddr(0x6000);
        break;

    default:
        FMC_SetVectorPageAddr(0x0);
        break;
    }
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_SetVectorPageAddr failed!\n");
        while (1);
    }

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    //NVIC_SystemReset();

    /* Disable ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    PutString("\nDone\n");

    while (SYS->PDID) __WFI();

}

/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/
