/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate the RTC function and displays current time to the UART console.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32RTCTInt = 0;

void RTC_IRQHandler(void);
void SYS_Init(void);

/**
 * @brief       IRQ Handler for RTC Interrupt
 * @param       None
 * @return      None
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in startup_m3351.c.
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC tick interrupt occurred */
    if (RTC_GET_TICK_INT_FLAG(RTC) == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG(RTC);

        g_u32RTCTInt = 1;

        PA2 ^= 1;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Enable DebugUART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for DebugUART RXD and TXD */
    SetDebugUartMFP();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable RTC module clock */
    CLK_EnableModuleClock(RTC_MODULE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_RTC_TIME_DATA_T sInitTime, sReadRTC;
    uint32_t u32Sec;
    uint8_t u8IsNewDateTime = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DebugUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------+\n");
    printf("|    RTC Time Display Sample Code   |\n");
    printf("+-----------------------------------+\n\n");

    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Open RTC and start counting */
    sInitTime.u32Year       = 2025;
    sInitTime.u32Month      = 10;
    sInitTime.u32Day        = 10;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 12;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_FRIDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    if (RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        goto lexit;
    }

    /* Enable RTC tick interrupt, one RTC tick is 1 second */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    printf("# Showing RTC date/time on UART0.\n\n");
    printf("1.) Use PA.2 to check tick period time is one second or not.\n");
    printf("2.) Show RTC date/time and change date/time after 5 seconds:\n");

    /* Use PA.2 to check tick period time */
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 1;

    u32Sec = 0;
    g_u32RTCTInt = 0;

    while (1)
    {
        if (g_u32RTCTInt == 1)
        {
            g_u32RTCTInt = 0;

            /* Read current RTC date/time */
            RTC_GetDateAndTime(&sReadRTC);
            printf("    %d/%02d/%02d %02d:%02d:%02d\n",
                   sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

            if (u32Sec == sReadRTC.u32Second)
            {
                printf("\nRTC time is incorrect.\n");
                return -1;
            }

            u32Sec = sReadRTC.u32Second;

            if (u8IsNewDateTime == 0)
            {
                if (u32Sec == (sInitTime.u32Second + 5))
                {
                    printf("\n");
                    printf("3.) Update new date/time to 2025/12/24 11:12:13.\n");

                    u8IsNewDateTime = 1;
                    RTC_SetDate(2025, 12, 24, RTC_WEDNESDAY);
                    RTC_SetTime(11, 12, 13, RTC_CLOCK_24, 0);
                }
            }
        }
    }

lexit:

    while (1) {}
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/