/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 18/07/10 11:21a $
 * @brief
 *           Demonstrate how to cause WDT time-out reset system event while WDT time-out reset delay period expired.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint32_t g_u32WDTINTCounts;
volatile uint8_t g_u8IsWDTWakeupINT;

/**
 * @brief       IRQ Handler for WDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT.
 */
void WDT_IRQHandler(void)
{
    if(g_u32WDTINTCounts < 10)
        WDT_RESET_COUNTER();

    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();

        g_u32WDTINTCounts++;
    }

    if(WDT_GET_TIMEOUT_WAKEUP_FLAG() == 1)
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();

        g_u8IsWDTWakeupINT = 1;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch WDT clock source to LIRC */
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable WDT clock */
    CLK_EnableModuleClock(WDT_MODULE);

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
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    WDT Time-out Wake-up Sample Code    |\n");
    printf("+----------------------------------------+\n\n");
    /* To check if system has been reset by WDT time-out reset or not */
    if(WDT_GET_RESET_FLAG() == 1)
    {
        WDT_CLEAR_RESET_FLAG();
        printf("*** System has been reset by WDT time-out event ***\n\n");
        while(1);
    }

    printf("# WDT Settings:\n");
    printf("    - Clock source is LIRC                  \n");
    printf("    - Time-out interval is 2^14 * WDT clock \n");
    printf("      (around 0.4266 ~ 0.4270 s)            \n");
    printf("    - Interrupt enable                      \n");
    printf("    - Wake-up function enable               \n");
    printf("    - Reset function enable                 \n");
    printf("    - Reset delay period is 18 * WDT clock  \n");
    printf("# System will generate a WDT time-out interrupt event after 0.4266 ~ 0.4270 s, \n");
    printf("    and will be wake up from power-down mode also.\n");
    printf("    (Use PA.0 high/low period time to check WDT time-out interval)\n");
    printf("# When WDT interrupt counts large than 10, \n");
    printf("    we will not reset WDT counter value and system will be reset immediately by WDT time-out reset signal.\n");

    /* Use PA.0 to check WWDT compare match interrupt period time */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);

    PA0 = 1;

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Configure WDT settings and start WDT counting */
    g_u32WDTINTCounts = g_u8IsWDTWakeupINT = 0;
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, TRUE, TRUE);

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    while(1)
    {
        /* System enter to Power-down */
        /* To program PWRCTL register, it needs to disable register protection first. */
        SYS_UnlockReg();
        printf("\nSystem enter to power-down mode ...\n");
        /* To check if all the debug messages are finished */
        while(IsDebugFifoEmpty() == 0);

        CLK_PowerDown();

        /* Check if WDT time-out interrupt and wake-up occurred or not */
        while(g_u8IsWDTWakeupINT == 0);

        g_u8IsWDTWakeupINT = 0;
        PA0 ^= 1;

        printf("System has been waken up done. WDT interrupt counts: %d.\n\n", g_u32WDTINTCounts);
    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
