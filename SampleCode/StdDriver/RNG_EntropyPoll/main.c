/******************************************************************************
* @file     main.c
* @version  V1.00
* @brief    Template for NuMicro M3351
*
* SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

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

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

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
    int32_t i, n;
    uint8_t au8Buf[32];
	
	  /* Unlock protected registers */
    SYS_UnlockReg();
		
		/* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
		
	  /* Lock protected registers */
    SYS_LockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

      /* Initial Random Number Generator */
    RNG_Open();

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

        CLK_SysTickDelay(100000);
    } while (1);

    
}
