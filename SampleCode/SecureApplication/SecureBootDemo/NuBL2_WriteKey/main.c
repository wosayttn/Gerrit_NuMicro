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

#define ROTPK_BASE          FMC_SOTP_BASE
#define SOTP_FSBC_BASE      (FMC_SOTP_BASE + 224) /*!< Fast Secure Boot Configuration (FSBC) \hideinitializer */

#define BOOT_KEY0_OFFSET    27
#define BOOT_KEY1_OFFSET    59
#define ECDSA_PUB_KEY_LEN   91

extern const unsigned char ecdsa_pub_key[];

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
    uint32_t au32WriteKey[8] = { 0 },
             au32KeyOffset[] = { BOOT_KEY0_OFFSET, BOOT_KEY1_OFFSET };
    uint32_t u32ROTPK_Addr;
    int32_t  i, j, k;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_CFG_UPDATE();

    printf("Write secure boot key\n");

    for (k = 0; k < 2; k++)
    {
        u32ROTPK_Addr = ROTPK_BASE + (k * 0x20);
        printf("Write key%d to 0x%08X\n", k, u32ROTPK_Addr);

        for (i = 0 ; i < 8; i++)
        {
            // SOTP write key order is the same as ecdsa_pub_key byte order
            j = au32KeyOffset[k] + i * 4;
            au32WriteKey[i] = (ecdsa_pub_key[j]) | (ecdsa_pub_key[j + 1] << 8) | (ecdsa_pub_key[j + 2] << 16) | (ecdsa_pub_key[j + 3] << 24);
            //printf("Write au32WriteKey[%d]: 0x%08X\n", i, au32WriteKey[i]);
            FMC_Write(u32ROTPK_Addr + (i * 4), au32WriteKey[i]);
        }
    }
    
    /* Fast Secure Boot Configuration (FSBC)
     *   FSBC[0] = 0, full secure boot verify will be apply to boot instead of fast secure boot verify.
     *   FSBC[10:8]:
     *      b110 = Only ROTPK 0 Existed
     *      b100 = ROTPK 0, 1 Existed
     *      b000 = ROTPK 0, 1, 2 Existed
     *      Others = Only ROTPK 0 Existed 
     */
    FMC_Write(SOTP_FSBC_BASE, 0xFFFFFEFF);  // Only ROTPK 0 Existed

    if(FMC_Write(FMC_SOTP_LOCK, 0x0) != FMC_OK)
        printf("  Write SOTP LOCK fail !\n");
    else
        printf("  Write SOTP LOCK.\n");

    printf("Done\n");
    /* Infinite loop to prevent from exiting */
    while (1)
    {
        // Add custom routine here.
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
