/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate the RTC alarm function. It sets an alarm 10 seconds after execution.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8IsAlarm = FALSE;

void RTC_AlarmHandle(void);
void RTC_IRQHandler(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* RTC Alarm Handle                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_AlarmHandle(void)
{
    printf(" Alarm!!\n");
    g_u8IsAlarm = TRUE;
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

        RTC_AlarmHandle();
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
    S_RTC_TIME_DATA_T sInitTime, sCurTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DebugUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+----------------------------------+\n");
    printf("|    RTC Alarm Test Sample Code    |\n");
    printf("+----------------------------------+\n\n");

    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Time Setting */
    sInitTime.u32Year       = 2025;
    sInitTime.u32Month      = 10;
    sInitTime.u32Day        = 10;
    sInitTime.u32Hour       = 13;
    sInitTime.u32Minute     = 0;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_FRIDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    if (RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        goto lexit;
    }

    printf("\nRTC Alarm Test (Alarm after 10 seconds)\n\n");

    g_u8IsAlarm = FALSE;

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf("Current Time: %d/%02d/%02d %02d:%02d:%02d\n",
           sCurTime.u32Year, sCurTime.u32Month, sCurTime.u32Day,
           sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

    /* The alarm time setting */
    sCurTime.u32Second = sCurTime.u32Second + 10;

    /* Set the alarm time */
    RTC_SetAlarmDateAndTime(&sCurTime);

    /* Clear RTC alarm interrupt flag */
    RTC_CLEAR_ALARM_INT_FLAG(RTC);

    /* Enable RTC Alarm Interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

    while (g_u8IsAlarm == FALSE) {}

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf("Current Time: %d/%02d/%02d %02d:%02d:%02d\n",
           sCurTime.u32Year, sCurTime.u32Month, sCurTime.u32Day,
           sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

    /* Disable RTC Alarm Interrupt */
    RTC_DisableInt(RTC_INTEN_ALMIEN_Msk);
    NVIC_DisableIRQ(RTC_IRQn);

    printf("\nRTC Alarm Test End !!\n");

lexit:

    while (1) {}
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/