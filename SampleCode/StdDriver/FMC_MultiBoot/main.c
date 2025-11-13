/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a multi-boot system to boot from different applications in APROM.
 *           A LDROM code and 4 APROM code are implemented in this sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#if !defined(__ICCARM__) && !defined(__GNUC__)
extern uint32_t Image$$RO$$Base;
#endif

void SYS_Init(void)
{
#if defined(__LDROM__)
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk ) | CLK_CLKSEL0_HCLKSEL_HIRC ;

    /* Enable UART0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk ;

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_UART0SEL_Msk) | CLK_CLKSEL2_UART0SEL_HIRC;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
#else

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
#endif
}
void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
#if defined(__LDROM__)
    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
#else
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
#endif
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
#ifdef __GNUC__                        /* for GNU C compiler */
/**
 * @brief       Hard fault handler
 * @param[in]   stack pointer points to the dumped registers in SRAM
 * @return      None
 * @details     Replace while(1) at the end of this function with chip reset if WDT is not enabled for end product
 */
void Hard_Fault_Handler(uint32_t stack[])
{
    PutString("In Hard Fault Handler\n");
    while(1);
}
#endif

int32_t main(void)
{
    uint8_t u8Ch;
    uint32_t u32Data;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC_Open();

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff.
        The sample code didn't use stanard C library for UART message due to the size of LDROM.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            FMC_MultiBoot, RO=0x0, ICE download algorithm to APROM
            FMC_Boot0, RO=0x2000, ICE download algorithm to APROM
            FMC_Boot1, RO=0x4000, ICE download algorithm to APROM
            FMC_Boot2, RO=0x6000, ICE download algorithm to APROM
            FMC_BootLD, RO=0x0F100000. ICE download algorithm to LDROM
        2. Reset MCU to execute FMC_MultiBoot.
    */
    PutString("\n+---------------------------+\n");
    PutString("|   Multi-Boot Sample Code  |\n");
    PutString("+---------------------------+\n");

#if defined(__BASE__)
    PutString("Boot from 0\n");
#elif defined(__BOOT0__)
    PutString("Boot from 0x2000\n");
#elif defined(__BOOT1__)
    PutString("Boot from 0x4000\n");
#elif defined(__BOOT2__)
    PutString("Boot from 0x6000\n");
#elif defined(__LDROM__)
    PutString("Boot from 0xF100000\n");
#endif

    u32Data = FMC_GetVECMAP();
    PutString("VECMAP = ");
    PutHexNumber(u32Data);
    PutString("\n");

    PutString("Select one boot image: \n");
#if defined(__BASE__)
    PutString("[0] Boot 0, base = 0x2000\n");
    PutString("[1] Boot 1, base = 0x4000\n");
    PutString("[2] Boot 2, base = 0x6000\n");
    PutString("[3] Boot 3, base = 0xF100000\n");
#elif defined(__BOOT0__)
    PutString("[1] Boot 1, base = 0x4000\n");
    PutString("[2] Boot 2, base = 0x6000\n");
    PutString("[3] Boot 3, base = 0xF100000\n");
    PutString("[Others] Boot, base = 0x0\n");
#elif defined(__BOOT1__)
    PutString("[0] Boot 0, base = 0x2000\n");
    PutString("[2] Boot 2, base = 0x6000\n");
    PutString("[3] Boot 3, base = 0xF100000\n");
    PutString("[Others] Boot, base = 0x0\n");
#elif defined(__BOOT2__)
    PutString("[0] Boot 0, base = 0x2000\n");
    PutString("[1] Boot 1, base = 0x4000\n");
    PutString("[3] Boot 3, base = 0xF100000\n");
    PutString("[Others] Boot, base = 0x0\n");
#elif defined(__LDROM__)
    PutString("[0] Boot 0, base = 0x2000\n");
    PutString("[1] Boot 1, base = 0x4000\n");
    PutString("[2] Boot 2, base = 0x6000\n");
    PutString("[Others] Boot, base = 0x0\n");
#endif
    u8Ch = GetChar();

    switch (u8Ch)
    {
#if defined(__BASE__)
    case '0':
        FMC_SetVectorPageAddr(0x2000);
        break;
    case '1':
        FMC_SetVectorPageAddr(0x4000);
        break;
    case '2':
        FMC_SetVectorPageAddr(0x6000);
        break;
    case '3':
        FMC_SetVectorPageAddr(0xF100000);
        break;
#elif defined(__BOOT0__)
    case '1':
        FMC_SetVectorPageAddr(0x4000);
        break;
    case '2':
        FMC_SetVectorPageAddr(0x6000);
        break;
    case '3':
        FMC_SetVectorPageAddr(0xF100000);
        break;
    default:
        FMC_SetVectorPageAddr(0x0);
        break;
#elif defined(__BOOT1__)
    case '0':
        FMC_SetVectorPageAddr(0x2000);
        break;
    case '2':
        FMC_SetVectorPageAddr(0x6000);
        break;
    case '3':
        FMC_SetVectorPageAddr(0xF100000);
        break;
    default:
        FMC_SetVectorPageAddr(0x0);
        break;
#elif defined(__BOOT2__)
    case '0':
        FMC_SetVectorPageAddr(0x2000);
        break;
    case '1':
        FMC_SetVectorPageAddr(0x4000);
        break;
    case '3':
        FMC_SetVectorPageAddr(0xF100000);
        break;
    default:
        FMC_SetVectorPageAddr(0x0);
        break;
#elif defined(__LDROM__)
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
#endif

    }
    if (g_FMC_i32ErrCode != 0)
    {
        PutString("Set Vector failed!\n");
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

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
