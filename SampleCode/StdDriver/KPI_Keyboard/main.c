/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to set scan key board by KPI.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

KPI_KEY_T queue[512] = {0};

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
    /* Use SystemCoreClockUpdate() to calculate and update SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Select KPI module clock source as HIRC and KPI module clock divider as 1 */
    CLK_EnableModuleClock(KPI0_MODULE);
    CLK_SetModuleClock(KPI0_MODULE, CLK_KPISEL_KPI0SEL_HIRC, CLK_KPIDIV_KPI0DIV(1));
    /* KPI Multi-function pin */
    SET_KPI_ROW0_PC5();
    SET_KPI_ROW1_PC4();
    SET_KPI_ROW2_PC3();
    SET_KPI_ROW3_PC2();
    SET_KPI_ROW4_PC1();
    SET_KPI_ROW5_PC0();
    SET_KPI_COL0_PA6();
    SET_KPI_COL1_PA7();
    SET_KPI_COL2_PC6();
    SET_KPI_COL3_PC7();
    SET_KPI_COL4_PC8();
    SET_KPI_COL5_PB6();
    SET_KPI_COL6_PB5();
    SET_KPI_COL7_PB4();
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    KPI_KEY_T key;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------------------+\n");
    printf("|          Keypad Interface Sample Code           |\n");
    printf("+-------------------------------------------------+\n\n");
    printf("  Key array board is necessary for this sample code\n");
    KPI_Open(6, 2, queue, 512);
    KPI->STATUS = KPI->STATUS;
    NVIC_EnableIRQ(KPI_IRQn);

    while (1)
    {
        if (KPI_kbhit())
        {
            key = KPI_GetKey();
            printf("%d, %d, %s\n", key.x, key.y, (key.st == KPI_PRESS) ? "PRESS" : "RELEASE");
        }
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
