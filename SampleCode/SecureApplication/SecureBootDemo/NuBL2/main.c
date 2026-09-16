/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to generate the first booting image: NuBL2.
 *           NuBL2 will authenticate NuBL32 and NuBL33 then jump to execute in NuBL32.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <arm_cmse.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "NuBL2.h"

void ActiveSecureConceal(uint32_t u32NuBL32Base);
void SYS_Init(void);
void UART_Init(void);

/*
 * Secure Conceal is used to hide user boot code from CPU instruction fetch and data access.
 *
 * After all coding and testing are finished, follow __RO_BASE and __RO_SIZE in NuBL2.scatter and
 * use ICP Programming Tool to set Base address, Page count and Enable of Secure Conceal Function.
 * Call ActiveSecureConceal when exit NuBL2 if user want to hide NuBL2.
 *
 * [Note] If Secure Conceal Function is enabled, Secure Conceal Region cannot be updated again.
 *        It needs whole chip erase to disable Secure Conceal Function.
 */
uint32_t ConfigSecureConceal(void)
{
    SYS_UnlockReg();
    FMC_Open();

    if (FMC_Read(FMC_USER_CONFIG_6) != 0xFFFFFFFF)
    {
        return 1;
    }
    else
    {
        printf("\nSecure Conceal Function is disabled.\n* Please read comment about ConfigSecureConceal function.\n");
        printf("\nPress 'y' to config and enable Secure Conceal Function.\n");

        if (getchar() == 'y')
        {
            printf("Apply Secure Conceal Function setting:\n");
            printf("  CONFIG4 = 0x%08X, CONFIG5 = 0x%08X and CONFIG6 = 0x0\n",
                   (uint32_t)NUBL2_FW_IMG_BASE, (uint32_t)(NUBL2_FW_IMG_SIZE / FMC_FLASH_PAGE_SIZE));
            FMC_WriteConfig(FMC_USER_CONFIG_4, NUBL2_FW_IMG_BASE);
            FMC_WriteConfig(FMC_USER_CONFIG_5, (NUBL2_FW_IMG_SIZE / FMC_FLASH_PAGE_SIZE));
            FMC_WriteConfig(FMC_USER_CONFIG_6, 0);
            printf("Please wait chip reset to take effect.\n");
            UART_WAIT_TX_EMPTY(DEBUG_PORT);
            SYS_ResetChip();

            while (1);
        }
    }

    return 0;
}

// Check root of trust public key (SOTP) status
static int32_t CheckROTPKStatus(void)
{
    if (FMC_IS_SOTP_LOCK())
    {
        printf("SOTP is locked. NuBL2 execution via secure boot.\n\n");
    }
    else
    {
        printf("SOTP is not locked ! NuBL2 execution via direct boot.\n\n");
    }

    return 0;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(ISP_MODULE);
    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);
    /* Reset CRYPTO module */
    SYS_ResetModule(CRPT_RST);
    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32NuBL32Base, u32NuBL33Base;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("\n\nCPU Clock: %d Hz\n", SystemCoreClock);
    printf("Flash Non-secure Base Address: 0x%08X\n", SCU_GET_FLASH_NS_BASE());
    printf("+------------------------------------------+\n");
    printf("|    SecureBootDemo - NuBL2 Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    CheckROTPKStatus();

    /* Authenticate NuBL32 FW */
    if (VerifyNuBL3x(NUBL32_FW_IMG_BASE, NUBL32_FW_IMG_SIZE, &u32NuBL32Base) != 0)
    {
        printf("\n\nNuBL2 verified NuBL32 FAIL !\n");
        goto ErrorExit;
    }

    printf("\nNuBL2 verified NuBL32 FW PASS.\n");

    /* Authenticate NuBL33 FW */
    if (VerifyNuBL3x(NUBL33_FW_IMG_BASE, NUBL33_FW_IMG_SIZE, &u32NuBL33Base) != 0)
    {
        printf("\n\nNuBL2 verified NuBL33 FAIL !\n");
        goto ErrorExit;
    }

    printf("\nNuBL2 verified NuBL33 FW PASS.\n");

    /* Jump to execute NuBL32 FW */
    printf("\nPress any key to execute NuBL32@0x%08X ...\n", u32NuBL32Base);
    getchar();

    if (ConfigSecureConceal())
    {
        printf("\nSecure Conceal Function is now enabled.\n");
        printf("Secure Conceal Region will be inaccessible after activation until the chip is reset.\n");
        UART_WAIT_TX_EMPTY(DEBUG_PORT);
    }

    ActiveSecureConceal(u32NuBL32Base);

ErrorExit:
    printf("Halt here\n");

    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
