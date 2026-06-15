/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the methods of detecting reset abnormalities.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t s_u32ResetFlagCount = 0;

void SysTick_Handler(void);
void SYS_Init(void);

/*--------------------------------------------------------------------------------------------------------*/
/*  SysTick IRQ Handler                                                                                   */
/*--------------------------------------------------------------------------------------------------------*/
NVT_ITCM void SysTick_Handler(void)
{
    if (s_u32ResetFlagCount > 0)
    {
        if (s_u32ResetFlagCount >= 5)
        {
            printf("Reset Flag Count = %d\n", s_u32ResetFlagCount);
            printf("Reset abnormalities happened!!! The same reset event occurs repeatedly for a fixed period of time.\n");
        }

        /* Clear spare reigster 0 */
        RTC_WRITE_SPARE_REGISTER(RTC, 0, 0);

        /* Clear reset flag count */
        s_u32ResetFlagCount = 0;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 220MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable RTC module clock */
    CLK_EnableModuleClock(RTC0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32SPRData;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Generate interrupt each 1 s */
    CLK_EnableSysTick(CLK_STSEL_ST0SEL_HIRC_DIV2, __HIRC >> 1);

    /* Enable spare register */
    RTC_EnableSpareAccess();

    /* Restore spare register data */
    u32SPRData = RTC_READ_SPARE_REGISTER(RTC, 0);

    s_u32ResetFlagCount = u32SPRData;

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+\n");
    printf("|   SYS Reset Detection Sample Code   |\n");
    printf("+-------------------------------------+\n\n");

    printf("Check UART message to confirm if any reset abnormalities have occurred.\n\n");

    while (1)
    {
        /* Get reset source is from reset pin reset */
        if (SYS_IS_RSTPIN_RST())
        {
            /* Clear reset source flag */
            SYS_ClearResetSrc(SYS_RSTSTS_PINRF_Msk);

            s_u32ResetFlagCount++;

            RTC_WRITE_SPARE_REGISTER(RTC, 0, s_u32ResetFlagCount);
        }

        /* Get reset sources are from Chip reset and CPU reset */
        while (SYS_IS_POR_RST() && SYS_IS_CPU_RST())
        {
            printf("Reset abnormalities happened! Different reset events occur at the same time.\n");
        }
    }
}
