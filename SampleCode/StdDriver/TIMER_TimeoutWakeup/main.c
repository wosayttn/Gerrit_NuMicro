/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use timer to wake up system from Power-down mode periodically.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

volatile uint8_t g_u8SlvPWRDNWK = 0, g_u8SlvTMRWK = 0;

void TIMER0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1;

    // Clear interrupt flag
    TIMER_ClearIntFlag(TIMER0);

    /* Get timer wake-up interrupt status flag */
    g_u8SlvTMRWK = TIMER_GetWakeupFlag(TIMER0);
    // Clear timer wake up flag
    TIMER_ClearWakeupFlag(TIMER0);

    while (TIMER_GetWakeupFlag(TIMER0) || TIMER_GetIntFlag(TIMER0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TIMER0 IntFlag time-out!\n");
            break;
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Power-down Mode Wake-up IRQ Handler                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1;

    /* Get system power down mode wake-up interrupt status flag */
    g_u8SlvPWRDNWK = CLK->PWRCTL & CLK_PWRCTL_PDWKIF_Msk;
    /* Clear system power down wake-up interrupt flag */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;

    /* Wait until the interrupt flag is cleared */
    while (CLK->PWRCTL & CLK_PWRCTL_PDWKIF_Msk)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for System wake-up Flag clear time-out!\n");
            break;
        }
    }
}

void EnterToPowerDown(uint32_t u32PDMode)
{
    printf("Entering power-down mode...\n");
    /* Waiting for UART printf finish*/
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;

    /* Enable Power-down mode wake-up interrupt */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIEN_Msk;
    NVIC_EnableIRQ(PWRWU_IRQn);

    CLK_SetPowerDownMode(u32PDMode);   //Power down

    /* Set system Power-down enabled and Power-down entry condition */
    CLK_PowerDown();
    /* Lock protected registers */
    SYS_LockReg();
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable LIRC*/
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_LIRC, 0);
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1;
    int i = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("Timer power down/wake up sample code\n");

    /* Output selected clock to CKO*/
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 3, 1);

    /* Initial Timer0 to periodic mode with 1Hz, since system is fast (144MHz)
       and timer is slow (32KHz), and following function calls all modified timer's
       CTL register, so add extra delay between each function call and make sure the
       setting take effect */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    CLK_SysTickDelay(50);
    /* Enable timer wake up system */
    TIMER_EnableWakeup(TIMER0);
    CLK_SysTickDelay(50);
    /* Enable Timer0 interrupt */
    TIMER_EnableInt(TIMER0);
    CLK_SysTickDelay(50);
    NVIC_EnableIRQ(TIMER0_IRQn);
    /* Start Timer0 counting */
    TIMER_Start(TIMER0);
    CLK_SysTickDelay(50);

    while (1)
    {
        u32TimeOutCnt = SystemCoreClock >> 1;
        g_u8SlvPWRDNWK = g_u8SlvTMRWK = 0;
        EnterToPowerDown(CLK_PMUCTL_PDMSEL_NPD1);

        /* Waiting for system wake-up and timer wake-up finish */
        while ((g_u8SlvPWRDNWK == 0) || (g_u8SlvTMRWK == 0))
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait wake-up finish time-out!\n");
                break;
            }
        }

        printf("Wake %d\n", i++);
        CLK_SysTickDelay(1000000);
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
