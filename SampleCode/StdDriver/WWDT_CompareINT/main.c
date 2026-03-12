/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to reload the WWDT counter value.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8WWDTINTCounts;

#define WWDT_PORT   (0)
#if (WWDT_PORT != 0)
    #define WWDT            WWDT1
    #define WWDT_IRQHandler WWDT1_IRQHandler
    #define WWDT_IRQn       WWDT1_IRQn
#else
    #define WWDT            WWDT0
    #define WWDT_IRQHandler WWDT0_IRQHandler
    #define WWDT_IRQn       WWDT0_IRQn
#endif

void WWDT_IRQHandler(void);
void SYS_Init(void);


/**
 * @brief       IRQ Handler for WWDT Interrupt
 * @param       None
 * @return      None
 * @details     The WWDT_IRQHandler is default IRQ of WWDT.
 */
void WWDT_IRQHandler(void)
{
    if (WWDT_GET_INT_FLAG(WWDT) == 1)
    {
        /* Clear WWDT compare match interrupt flag */
        WWDT_CLEAR_INT_FLAG(WWDT);

        PA2 ^= 1;

        g_u8WWDTINTCounts++;

        if (g_u8WWDTINTCounts < 10)
        {
            /* To reload the WWDT counter value to 0x3F */
            WWDT_RELOAD_COUNTER(WWDT);
        }

        printf("WWDT compare match interrupt occurred. (%d)\n", g_u8WWDTINTCounts);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable WWDT module clock */
#if (WWDT_PORT != 0)
    CLK_EnableModuleClock(WWDT1_MODULE);
    CLK_SetModuleClock(WWDT1_MODULE, CLK_CLKSEL1_WWDT1SEL_HCLK_DIV2048, 0);
#else
    CLK_EnableModuleClock(WWDT0_MODULE);
    CLK_SetModuleClock(WWDT0_MODULE, CLK_CLKSEL1_WWDT0SEL_HCLK_DIV2048, 0);
#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    double dPeriodTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug Uart*/
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    WWDT Compare Match Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /* To check if system has been reset by WWDT time-out reset or not */
    if (WWDT_GET_RESET_FLAG(WWDT) == 1)
    {
        printf("*** System has been reset by WWDT time-out reset event. [WWDT_CTL: 0x%08X] ***\n\n", WWDT->CTL);
        WWDT_CLEAR_RESET_FLAG(WWDT);

        while (1) {}
    }

    dPeriodTime = (((double)(1000000 * 2048) / (double)(SystemCoreClock / 2)) * 1024) * 32;

    printf("# WWDT Settings: \n");
    printf("    - Clock source is PCLK0/2048 (%d Hz)    \n", (SystemCoreClock / 2) / 2048);
    printf("    - WWDT counter prescale period is 1024, \n");
    printf("        and max WWDT time-out period is 1024 * (64 * WWDT_CLK)\n");
    printf("    - Interrupt enable                      \n");
    printf("    - Window Compare value is 32            \n");
    printf("# System will generate first WWDT compare match interrupt event after %u ms.\n", (uint32_t)(dPeriodTime / 1000));
    printf("    1.) use PA.2 high/low period to check WWDT compare match interrupt period time\n");
    printf("    2.) reload WWDT counter value to avoid WWDT time-out reset system occurred\n");
    printf("        when interrupt counts less than 11.\n");
    printf("    3.) do not reload WWDT counter value to generate WWDT time-out reset system event\n");
    printf("        when interrupt counts large than 10.\n\n");

    /* Use PA.2 to check WWDT compare match interrupt period time */
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 1;

    /* Enable WWDT NVIC */
    NVIC_EnableIRQ(WWDT_IRQn);

    g_u8WWDTINTCounts = 0;

    /*
        Max time-out period is 1024*(64*WWDT_CLK);
        WWDT compare value is 32;
        Enable WWDT compare match interrupt;
    */
    /* Note: WWDT_CTL register can be written only once after chip is powered on or reset */
    WWDT_Open(WWDT, WWDT_PRESCALER_1024, 32, TRUE);

    printf("[WWDT_CTL: 0x%08X]\n\n", WWDT->CTL);

    while (1) {}
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/