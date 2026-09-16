/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate Cache parity error detection.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

volatile uint32_t g_u32ParityErrorFlag = 0U;

void CACHE_IRQHandler(void)
{
    if (CACHE_GetParityErrorStatus())
    {
        printf("[CACHE_IRQ] Cache Parity Error Detected at Address: 0x%08X.\n", CACHE_GetParityErrorAddr());

        /* Clear Cache parity error flag */
        CACHE_ClrParityErrorStatus();
        g_u32ParityErrorFlag = 1;
        printf("Disable CACHE to prevent further parity errors.\n");
        CACHE_Disable();
    }
}

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
    uint8_t u8IntEnable;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("+------------------------------------------------------------------+\n");
    printf("|        CACHE check parity status and parity error address        |\n");
    printf("+------------------------------------------------------------------+\n");
    printf("Terminal will print error address when parity error occurs.\n");
    printf("Enable interrupt to check CACHE parity status ?\n");
    printf("  [0] No\n");
    printf("  [1] Yes\n");
    printf("  Select: ");
    u8IntEnable = getchar() - '0';
    printf("Interrupt is %s\n\n", u8IntEnable ? "enabled" : "disabled");

    CACHE_Enable();
    CACHE_EnableParityCheck();

    if (u8IntEnable)
    {
        /* Enable Cache interrupt */
        CACHE_EnableParityCheckInt();
        NVIC_EnableIRQ(CACHE_IRQn);
    }
    else
    {
        /* Disable Cache interrupt */
        CACHE_DisableParityCheckInt();
        NVIC_DisableIRQ(CACHE_IRQn);
    }

    while (1)
    {
        if (u8IntEnable == 0)
        {
            if (CACHE_GetParityErrorStatus())
            {
                printf("Cache Parity Error Detected at Address: 0x%08X.\n", CACHE_GetParityErrorAddr());

                /* Clear Cache parity error flag */
                CACHE_ClrParityErrorStatus();
                g_u32ParityErrorFlag = 1;
            }
        }

        if (g_u32ParityErrorFlag)
        {
            printf("Press [ESC] to exit or any key to continue ...\n");

            if (getchar() == 0x1B) /* ESC */
            {
                goto lexit;
            }
        }
    }

lexit:
    NVIC_DisableIRQ(CACHE_IRQn);
    CACHE_DisableParityCheckInt();
    CACHE_DisableParityCheck();
    CACHE_Disable();
    printf("End of CACHE parity error check sample code.\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
