/******************************************************************************
* @file     main.c
* @version  V1.00
* @brief    mbedTLS_AES code for NuMicro M3351
*
* SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "common.h"
#include "mbedtls/aes.h"
#include "mbedtls/gcm.h"
#include "mbedtls/ccm.h"


#define MBEDTLS_EXIT_SUCCESS    0
#define MBEDTLS_EXIT_FAILURE    -1
#define AES_BLOCK_SIZE         (16)
#define TEST_AES
#define TEST_GCM
#define TEST_CCM

volatile uint32_t g_u32Ticks = 0;

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

void SysTick_Handler()
{
    g_u32Ticks++;
}

int32_t main(void)
{
    int  i32Ret = MBEDTLS_EXIT_SUCCESS;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    SysTick_Config(SystemCoreClock / 1000);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("MBEDTLS AES self test ...\n");

#ifdef MBEDTLS_AES_ALT
    printf("Hardware Accelerator Enabled.\n");
#else
    printf("Pure software crypto running.\n");
#endif

#ifdef TEST_AES
    g_u32Ticks = 0;
    i32Ret = mbedtls_aes_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);
#endif

#ifdef TEST_GCM
    g_u32Ticks = 0;
    i32Ret |= mbedtls_gcm_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);
#endif

#ifdef TEST_CCM
    g_u32Ticks = 0;
    i32Ret |= mbedtls_ccm_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);
#endif

    if (i32Ret != 0)
    {
        printf("Test fail!\n");
    }

    printf("Test Done!\n");

    while (1);


}
