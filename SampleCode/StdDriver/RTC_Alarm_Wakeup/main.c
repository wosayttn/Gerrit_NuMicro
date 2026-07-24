/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use RTC alarm interrupt event to wake up system.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8IsRTCAlarmINT = 0;

void RTC_IRQHandler(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
int IsDebugFifoEmpty(void)
{
    return ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) != 0U);
}

/**
 * @brief       IRQ Handler for RTC Interrupt
 * @param       None
 * @return      None
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in startup_m3351.c.
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC alarm interrupt occurred */
    if (RTC_GET_ALARM_INT_FLAG(RTC) == 1)
    {
        /* Clear RTC alarm interrupt flag */
        RTC_CLEAR_ALARM_INT_FLAG(RTC);

        g_u8IsRTCAlarmINT++;
    }

    if (RTC_GET_TICK_INT_FLAG(RTC) == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG(RTC);
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
    S_RTC_TIME_DATA_T sWriteRTC, sReadRTC;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DebugUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+\n");
    printf("|    RTC Alarm Wake-up Sample Code    |\n");
    printf("+-------------------------------------+\n\n");

    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Open RTC */
    sWriteRTC.u32Year       = 2025;
    sWriteRTC.u32Month      = 10;
    sWriteRTC.u32Day        = 10;
    sWriteRTC.u32DayOfWeek  = RTC_FRIDAY;
    sWriteRTC.u32Hour       = 23;
    sWriteRTC.u32Minute     = 59;
    sWriteRTC.u32Second     = 50;
    sWriteRTC.u32TimeScale  = RTC_CLOCK_24;

    if (RTC_Open(&sWriteRTC) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        goto lexit;
    }

    /* Set RTC alarm date/time */
    sWriteRTC.u32Year       = 2025;
    sWriteRTC.u32Month      = 10;
    sWriteRTC.u32Day        = 10;
    sWriteRTC.u32DayOfWeek  = RTC_FRIDAY;
    sWriteRTC.u32Hour       = 23;
    sWriteRTC.u32Minute     = 59;
    sWriteRTC.u32Second     = 55;
    RTC_SetAlarmDateAndTime(&sWriteRTC);

    /* Enable RTC alarm interrupt and wake-up function will be enabled also */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    printf("# Set RTC current date/time: 2025/10/10 23:59:50.\n");
    printf("# Set RTC alarm date/time:   2025/10/10 23:59:55.\n");
    printf("# Wait system waken-up by RTC alarm interrupt event.\n");

    g_u8IsRTCAlarmINT = 0;

    /* System enter to Power-down */
    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();
    printf("\nSystem enter to power-down mode ...\n");
    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (IsDebugFifoEmpty() == 0)
        if (--u32TimeOutCnt == 0) break;

    CLK_PowerDown();

    while (g_u8IsRTCAlarmINT == 0) {}

    /* Read current RTC date/time */
    RTC_GetDateAndTime(&sReadRTC);
    printf("System has been waken-up and current date/time is:\n");
    printf("    %d/%02d/%02d %02d:%02d:%02d\n",
           sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);


    printf("\n\n");
    printf("# Set next RTC alarm date/time: 2025/10/11 00:00:05.\n");
    printf("# Wait system waken-up by RTC alarm interrupt event.\n");
    RTC_SetAlarmDate(2025, 10, 11);
    RTC_SetAlarmTime(0, 0, 5, RTC_CLOCK_24, 0);

    g_u8IsRTCAlarmINT = 0;

    /* System enter to Power-down */
    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();
    printf("\nSystem enter to power-down mode ...\n");
    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (IsDebugFifoEmpty() == 0)
        if (--u32TimeOutCnt == 0) break;

    CLK_PowerDown();

    while (g_u8IsRTCAlarmINT == 0) {}

    /* Read current RTC date/time */
    RTC_GetDateAndTime(&sReadRTC);
    printf("System has been waken-up and current date/time is:\n");
    printf("    %d/%02d/%02d %02d:%02d:%02d\n",
           sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

lexit:

    while (1) {}
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/