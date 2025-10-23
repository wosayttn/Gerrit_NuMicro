/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Dual bank firmware update application sample
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "NuDB_common.h"
#include "xmodem.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t  s_u32DbLength;             /* dual bank program remaining length       */
static volatile uint32_t  s_u32DbAddr;               /* dual bank program current flash address  */
static volatile uint32_t  s_u32TickCnt;              /* timer ticks - 100 ticks per second       */

/*---------------------------------------------------------------------------------------------------------*/
/* Global Functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len)
{
    uint32_t  u32Idx, u32Data = 0UL;

    for (u32Idx = 0; u32Idx < u32Len; u32Idx += 4)
    {
        u32Data += M32(u32Start + u32Idx);
    }

    u32Data = 0xFFFFFFFF - u32Data + 1UL;

    return u32Data;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupt Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void SysTick_Handler(void)
{
    /* Increase timer tick */
    s_u32TickCnt++;

    /* Calculate CRC32 value, just to consume CPU time  */
    FuncCrc32(DTCM_BASE, 0x100);
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(ISP0_MODULE);
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

void StartTimer0(void)
{
    volatile int32_t i32Timeout = 1000;

    CLK_EnableModuleClock(TMR0_MODULE);     /* Enable TIMER0 clock          */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, MODULE_NoMsk);

    TIMER_Stop(TIMER0);
    TIMER_ClearIntFlag(TIMER0);
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);
    TIMER_SET_PRESCALE_VALUE(TIMER0, ((__HIRC / 1000000) << TIMER_CTL_PSC_Pos));
    TIMER_ResetCounter(TIMER0);
    TIMER_Start(TIMER0);

    while (TIMER_IS_ACTIVE(TIMER0) == 0)
    {
        if (i32Timeout-- < 0)
        {
            printf("Timer0 active timeout !\n");
            break;
        }
    }
}

int main()
{
    uint32_t u32ch;
    int32_t  i32Err;
    /* CPU executing in which Bank */
    uint32_t  u32ExecBank = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /* Init Debug UART for print message */
    InitDebugUart();

    /* Disable register write-protection function */
    SYS_UnlockReg();
    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
#ifdef NEW_APP
        printf("\n [New Firmware] BANK%d APP processing ...\n", u32ExecBank);
#else
        printf("\n BANK%d APP processing ...\n", u32ExecBank);
#endif
        printf("\n Download new FW to Bank%d ? [y/n]\n", u32ExecBank ^ 1);
        u32ch = (uint32_t)getchar();

        if (u32ch == 'y')
        {
            printf("\n Bank%d processing, download data to Bank%d.\n", u32ExecBank, u32ExecBank ^ 1);

            SysTick_Config(1000);
            StartTimer0();

            /* Dual bank background program address */
            s_u32DbAddr   = FMC_APROM_BANK_SIZE * (u32ExecBank ^ 1) + APP_BASE;
            /* Dual bank background length */
            s_u32DbLength = APP_SIZE;

            SysTick_Config(1000);
            StartTimer0();

            i32Err = XmodemRecv(s_u32DbAddr);

            if (i32Err < 0)
            {
                printf("Xmodem transfer fail !\n");

                while (1);
            }
            else
            {
                printf("Xomdem transfer done.\n");
                printf("Total trnasfer size is %d\n", i32Err);
            }

            printf("\n Firmware download completed.\n");
        }
        else
        {
            printf("\n Reset from BANK%d Loader \n", u32ExecBank);
            /* Remap to Loader */
            FMC_SetVectorPageAddr(FMC_APROM_BASE);
            UART_WAIT_TX_EMPTY(DEBUG_PORT);
            SYS_ResetCPU();
        }
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
