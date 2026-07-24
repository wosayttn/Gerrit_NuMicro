/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Change system clock to different PLL frequency and output system clock from CLKO pin.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SIGNATURE   0x125ab234
#define FLAG_ADDR   0x20001FFC

int32_t pi(void);
void SYS_PLL_Test(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PI_NUM  256
static int32_t g_ai32f[PI_NUM + 1];
static uint32_t g_au32piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

static int32_t g_ai32piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for (; b - c;)
        g_ai32f[b++] = a / 5;

    i = 0;

    for (; (void)(d = 0), g = c * 2; c -= 14, g_ai32piResult[i++] = e + d / a, e = d % a)
    {
        if (i == 19)
            break;

        for (b = c; (void)(d += g_ai32f[b] * a), (void)(g_ai32f[b] = d % --g), (void)(d /= g--), --b; d *= b);
    }

    i32Err = 0;

    for (i = 0; i < 19; i++)
    {
        if (g_au32piTbl[i] != (uint32_t)g_ai32piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

static uint32_t g_au32PllSetting[] =
{
    50000000,   /* PLL = 50MHz */
    72000000,   /* PLL = 72MHz */
    96000000,   /* PLL = 96MHz */
    144000000,  /* PLL = 144MHz */
};

void SYS_PLL_Test(void)
{
    uint32_t  u32Idx;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PLL clock configuration test                                                                            */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n-------------------------[ Test PLL ]-----------------------------\n");

    for (u32Idx = 0; u32Idx < sizeof(g_au32PllSetting) / sizeof(g_au32PllSetting[0]) ; u32Idx++)
    {
        /* Select HCLK clock source from PLL */
        CLK_SetCoreClock(g_au32PllSetting[u32Idx]);

        printf("  Change system clock to %d Hz ...................... ", SystemCoreClock);

        /* Output selected clock to CKO, CKO Clock = HCLK / 2^(1 + 1) */
        CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 1, FALSE);

        CLK_SysTickDelay(5000);

        if (pi())
        {
            printf("[FAIL]\n");
        }
        else
        {
            printf("[OK]\n");
        }

        /* Disable CLKO clock */
        CLK_DisableCKO();
    }
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PB multi-function pin for CLKO(PB.14) */
    SET_CLKO_PB14();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32data, u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|       System Driver Sample Code       |\n");
    printf("+---------------------------------------+\n");

    if (M32(FLAG_ADDR) == SIGNATURE)
    {
        printf("  CPU Reset success!\n");
        M32(FLAG_ADDR) = 0;
        printf("  Press any key to continue ...\n\n");
        getchar();
    }

    /* Read Part Device ID */
    printf("Product ID 0x%x\n", SYS->PDID);

    /* Get reset source from last operation */
    u32data = SYS_GetResetSrc();
    printf("Reset Source 0x%x\n", u32data);

    /* Clear reset source */
    SYS_ClearResetSrc(u32data);

    /* Unlock protected registers for HCLK clock source settings */
    SYS_UnlockReg();

    /* Check if the write-protected registers are unlocked before HCLK clock source setting and CPU Reset */
    if (SYS_IsRegLocked() == 0)
    {
        printf("Protected Address is Unlocked\n");
    }

    /* Run PLL Test */
    SYS_PLL_Test();

    /* Write a signature work to SRAM to check if it is reset by software */
    M32(FLAG_ADDR) = SIGNATURE;
    printf("\n\n  >>> Reset CPU <<<\n");

    /* Wait for message send out */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power down mode and HW will also clear PLLSTB bit in CLK_STATUS register */
    CLK_DisablePLL();

    /* Reset CPU */
    SYS_ResetCPU();
}
