/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a multi-boot system to boot from different applications in APROM.
 *           A LDROM code and 4 APROM code are implemented in this sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#if !defined(__ICCARM__) && !defined(__GNUC__)
extern uint32_t Image$$RO$$Base;
#endif

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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

    /* Unlock protected registers to operate RMC ISP function */
    SYS_UnlockReg();

    /* Enable RMC ISP function. Before using RMC function, it should unlock system register first. */
    RMC_Open();

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        The sample code didn't use stanard C library for UART message due to the size of LDROM.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            RMC_MultiBoot, RO=0x0, ICE download algorithm to APROM
            RMC_Boot0, RO=0x4000, ICE download algorithm to APROM
            RMC_Boot1, RO=0x8000, ICE download algorithm to APROM
            RMC_Boot2, RO=0xC000, ICE download algorithm to APROM
            RMC_Boot3, RO=0x10000, ICE download algorithm to APROM
            RMC_BootLD, RO=0x100000. ICE download algorithm to LDROM
        2. Reset MCU to execute RMC_MultiBoot.
    */

    PutString("\n\n");
    PutString("+---------------------------+\n");
    PutString("|   Multi-Boot Sample Code  |\n");
    PutString("+---------------------------+\n");
#if defined(__BASE__)
    PutString("Boot from 0\n");
#endif
#if defined(__BOOT0__)
    PutString("Boot from 0x4000\n");
#endif
#if defined(__BOOT1__)
    PutString("Boot from 0x8000\n");
#endif
#if defined(__BOOT2__)
    PutString("Boot from 0xC000\n");
#endif
#if defined(__BOOT3__)
    PutString("Boot from 0x10000\n");
#endif
#if defined( __LDROM__ )
#if defined(__ICCARM__) || defined(__ARMCC_VERSION  )
    PutString("Boot from 0xF100000\n");
#endif
#endif

    u32Data = RMC_GetVECMAP();
    PutString("VECMAP = ");
    PutHexNumber(u32Data);
    PutString("\n");

    PutString("Select one boot image: \n");

    PutString("[0] Boot 0, base = 0x4000\n");
    PutString("[1] Boot 1, base = 0x8000\n");
    PutString("[2] Boot 2, base = 0xC000\n");
    PutString("[3] Boot 3, base = 0x10000\n");
#if defined(__ICCARM__) || defined(__ARMCC_VERSION  )
    PutString("[4] Boot 4, base = 0xF100000\n");
#endif
    PutString("[Others] Boot, base = 0x0\n");

    u8Ch = GetChar();

    switch (u8Ch)
    {
    case '0':
        RMC_SetVectorPageAddr(0x4000);
        break;

    case '1':
        RMC_SetVectorPageAddr(0x8000);
        break;

    case '2':
        RMC_SetVectorPageAddr(0xC000);
        break;

    case '3':
        RMC_SetVectorPageAddr(0x10000);
        break;
#if defined(__ICCARM__) || defined(__ARMCC_VERSION  )
    case '4':
        RMC_SetVectorPageAddr(0xF100000);
        break;
#endif
    default:
        RMC_SetVectorPageAddr(0x0);
        break;
    }
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_SetVectorPageAddr failed!\n");
        while (1);
    }

    /* Reset CPU only to reset to new vector page */
//    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    NVIC_SystemReset();

    /* Disable ISP function */
//    RMC_Close();

    /* Lock protected registers */
//    SYS_LockReg();

//    PutString("\nDone\n");

//    while (SYS->PDID) __WFI();

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
