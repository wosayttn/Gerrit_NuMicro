/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 20/08/12 3:32p $
 * @brief
 *           Demonstrate how to cause WDT time-out reset system event while WDT time-out reset delay period expired.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
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
 * @brief       IRQ Handler for WDT and WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT and WWDT, declared in startup_M471.s.
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
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 96MHz from PLL */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable WDT module clock */
    CLK_EnableModuleClock(WDT_MODULE);

    /* Switch WDT clock source to LIRC */
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

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
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_130CLK, TRUE, TRUE);

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

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
