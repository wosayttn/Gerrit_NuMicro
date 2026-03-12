/******************************************************************************
* @file     main.c
* @version  V1.00
* @brief    RNG_EntropyPoll code for NuMicro M3351
*
* SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void)
{
    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL0 144MHz clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable CRYPTO0 module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Enable TRNG0 module clock */
    CLK_EnableModuleClock(TRNG_MODULE);

    /* Enable UART0 module clock */
    SetDebugUartCLK();

    /* Set UART0 MFP */
    SetDebugUartMFP();
}

int32_t main(void)
{
    int32_t i, n;
    uint8_t au8Buf[32];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    //Keep unlock for RNG_Open setting
    //SYS_LockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("RNG_EntropyPoll Sample.\r\n");

    /* Initial Random Number Generator */
    if (RNG_Open() < 0)
    {
        printf("RNG_Open fail.\r\n");

        while (1);
    }

    do
    {
        /* Get entropy */
        n = RNG_EntropyPoll((uint32_t *)(au8Buf), 32);

        if (n < 0)
        {
            printf("Entropy poll fail. return code = %d\n", n);
            break;
        }

        for (i = 0; i < n; i++)
        {
            printf("%02x", au8Buf[i]);
        }

        printf("\n");

    } while (1);


}
