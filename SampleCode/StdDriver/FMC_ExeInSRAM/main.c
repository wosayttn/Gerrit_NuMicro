/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demo how to place code to execute in SRAM to program embedded flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

int32_t FlashAccess_OnSRAM(void);
void SysTick_Handler_SRAM(void);

/* Declare new vector table placed in SRAM */
static uint32_t s_au32SRAM_VECTOR_TABLE[256] __ALIGNED(256);

int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

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

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    memcpy((void *)s_au32SRAM_VECTOR_TABLE, (void *)SCB->VTOR, sizeof(s_au32SRAM_VECTOR_TABLE));
    s_au32SRAM_VECTOR_TABLE[16 + SysTick_IRQn] = (uint32_t)SysTick_Handler_SRAM;
    /* Set vector table offset */
    SCB->VTOR = (uint32_t)s_au32SRAM_VECTOR_TABLE;

    printf("+------------------------------------------------+\n");
    printf("|      FMC Code Execute in SRAM Sample Code      |\n");
    printf("+------------------------------------------------+\n");
    printf("Will branch to SRAM address: 0x%08X\n", (uint32_t)FlashAccess_OnSRAM);
    /* SysTick used for test interrupts in SRAM */
    SysTick_Config(SystemCoreClock / 1000);

    if (FlashAccess_OnSRAM())
    {
        printf("Flash access return error !\n");
    }
    else
    {
        printf("Flash access return ok.\n");
    }

    printf("\nEnd of FMC Sample Code\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
