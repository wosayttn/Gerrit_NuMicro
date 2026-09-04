/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Show how to use RMC ISP APIs config/erase XOM region.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define XOMR0_Base    0x10000

extern int32_t LibXOMAdd(uint32_t a, uint32_t b);

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

int32_t main(void)
{
    int32_t i32Status;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|  RMC XOM config & erase  Sample Code   |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable RMC ISP function and enable APROM active. Before using RMC function, it should unlock system register first. */
    RMC_Open();

    RMC_ENABLE_AP_UPDATE();

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", RMC_Read(RMC_USER_CONFIG_0));

    printf("XOM Status = 0x%x\n", RMC->XOMSTS);
    printf("Any key to continue...\n");
    getchar();

    if((RMC->XOMSTS & 0x1) != 0x1)
    {
        /* Config XOMR0 */
        if(RMC_GetXOMState(XOMR0) == 0)
        {
            i32Status = RMC_ConfigXOM(XOMR0, XOMR0_Base, 1);
            if(i32Status)
                printf("XOMR0 Config fail...\n");
            else
                printf("XOMR0 Config OK...\n");
        }

        printf("\nAny key to reset chip to enable XOM regions...\n");
        getchar();

        /* Reset chip to enable XOM region. */
        SYS_ResetChip();
        while(1) {}
    }

    /* Run XOM function */
    printf("\n");
    printf(" 100 + 200 = %d\n", LibXOMAdd(100, 200));

    printf("\nXOMR0 active success....\n");
    printf("\nAny key to Erase XOM...\n");
    getchar();

    if((RMC->XOMSTS & 0x1) == 0x1)
    {
        /* Erase XOMR0 region */
        if(RMC_EraseXOM(XOMR0) == 0)
            printf("Erase XOMR0....OK\n");
        else
            printf("Erase XOMR0....Fail\n");

        printf("\nAny key to reset chip...\n");
        getchar();

        /* Reset chip to finish erase XOM region. */
        SYS_ResetChip();
    }

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/


