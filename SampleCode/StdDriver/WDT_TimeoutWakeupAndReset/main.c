/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement WDT time-out interrupt event to wake up system and generate time-out reset system event while WDT time-out reset delay period expired.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8IsINTEvent;
static volatile uint32_t g_u32WakeupCounts;

#define WDT            WDT0
#define WDT_IRQHandler WDT0_IRQHandler

void WDT_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);

/**
 * @brief       IRQ Handler for WDT Interrupt
 * @param       None
 * @return      None
 * @details     The WDT_IRQHandler is default IRQ of WDT.
 */
void WDT_IRQHandler(void)
{
    if (g_u32WakeupCounts < 10)
    {
        WDT_RESET_COUNTER(WDT);
    }

    if (WDT_GET_TIMEOUT_INT_FLAG(WDT) == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG(WDT);

        g_u8IsINTEvent = 1;
    }

    if (WDT_GET_TIMEOUT_WAKEUP_FLAG(WDT) == 1)
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG(WDT);

        g_u32WakeupCounts++;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    SYS_UnlockReg();

    /* Enable clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Set Debug Uart Clock */
    SetDebugUartCLK();

    /* Set Debug Uart I/O */
    SetDebugUartMFP();

    /* Enable WDT module clock */
    CLK_EnableModuleClock(WDT0_MODULE);
    CLK_SetModuleClock(WDT0_MODULE, CLK_CLKSEL1_WDT0SEL_LIRC, 0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug Uart*/
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    WDT Time-out Wake-up Sample Code    |\n");
    printf("+----------------------------------------+\n\n");

    /* To check if system has been reset by WDT time-out reset or not */
    if (WDT_GET_RESET_FLAG(WDT) == 1)
    {
        WDT_CLEAR_RESET_FLAG(WDT);
        printf("*** System has been reset by WDT time-out event ***\n\n");

        while (1) {}
    }

    printf("# WDT Settings:\n");
    printf("    - Clock source is LIRC                  \n");
    printf("    - Time-out interval is 2^14 * WDT clock \n");
    printf("      (around 0.427 s)                      \n");
    printf("    - Interrupt enable                      \n");
    printf("    - Wake-up function enable               \n");
    printf("    - Reset function enable                 \n");
    printf("    - Reset delay period is 18 * WDT clock  \n");
    printf("# System will generate a WDT time-out interrupt event after 0.427 s, \n");
    printf("    and will be wake up from power-down mode also.\n");
    printf("    (Use PA.2 high/low period time to check WDT time-out interval)\n");
    printf("# When WDT interrupt counts large than 10, \n");
    printf("    we will not reset WDT counter value and system will be reset immediately by WDT time-out reset signal.\n");

    /* Use PA.2 to check time-out interrupt period time */
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 1;

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT0_IRQn);

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Configure WDT settings and start WDT counting */
    g_u8IsINTEvent = g_u32WakeupCounts = 0;
    WDT_Open(WDT, WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, TRUE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt(WDT);

    while (1)
    {
        /* System enter to Power-down */
        /* To program PWRCTL register, it needs to disable register protection first. */
        SYS_UnlockReg();
        printf("\nSystem enter to power-down mode ...\n");
        /* To check if all the debug messages are finished */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (!UART_IS_TX_EMPTY(DEBUG_PORT))
            if (--u32TimeOutCnt == 0) break;

        CLK_PowerDown();

        /* Check if WDT time-out interrupt and wake-up occurred or not */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (g_u8IsINTEvent == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for WDT interrupt time-out!\n");
                break;
            }
        }

        PA2 ^= 1;

        g_u8IsINTEvent = 0;
        printf("System has been waken up done. WDT wake-up counts: %d.\n\n", g_u32WakeupCounts);
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/