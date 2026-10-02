/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use FMC Read-All-One ISP command to check
 *           specified FMC flash region are all 0xFFFFFFFF or not.
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
    uint32_t u32RetCode = 0;
    uint32_t u32Data;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+----------------------------------------+\n");
    printf("|      FMC Read All One Sample Code      |\n");
    printf("+----------------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers to operate FMC ISP function */

    FMC_Open();                        /* Enable FMC ISP function */

    u32Data = FMC_ReadCID();           /* Read company ID. Should be 0xDA. */
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* Read product ID. */
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_USER_CONFIG_0));
    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_USER_CONFIG_1));

    FMC_ENABLE_LD_UPDATE();            /* Enable LDROM update. */

    FMC_Erase(FMC_LDROM_BASE);         /* Erase LDROM page 0. */

    printf("\n  Check LDROM is all one after erase\n");
    /* Run and check flash contents are all 0xFFFFFFFF. */
    u32RetCode = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);

    if (u32RetCode == READ_ALLONE_YES)           /* Return value READ_ALLONE_YES means all flash contents are 0xFFFFFFFF */
        printf("    Check READ_ALLONE_YES success.\n");    /* FMC_CheckAllOne() READ_ALLONE_YES passed on LDROM page 0. */
    else
        printf("    Check READ_ALLONE_YES failed!\n");     /* FMC_CheckAllOne() READ_ALLONE_YES failed on LDROM page 0. */

    FMC_Write8Bytes(FMC_LDROM_BASE, 0, 0xFFFFFFFF);        /* Program a 0 to LDROM to make it not all 0xFFFFFFFF. */
    printf("  Check LDROM is not all one after write\n");
    /* Run and check flash contents are not all 0xFFFFFFFF. */
    u32RetCode = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);

    if (u32RetCode == READ_ALLONE_NOT)
        printf("    Check READ_ALLONE_NOT success.\n");   /* FMC_CheckAllOne() READ_ALLONE_NOT passed on LDROM page 0. */
    else
        printf("    Check READ_ALLONE_NOT failed!\n");    /* FMC_CheckAllOne() READ_ALLONE_NOT failed on LDROM page 0. */

    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    printf("\nEnd of FMC Read All One Sample Code\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
