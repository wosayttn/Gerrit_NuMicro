/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC alarm function. It sets an alarm 10 seconds
 *           after execution
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t   g_bAlarm  = FALSE;

/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    /* To check if RTC alarm interrupt occurred */
    if(RTC_GET_ALARM_INT_FLAG() == 1)
    {
        /* Clear RTC alarm interrupt flag */
        RTC_CLEAR_ALARM_INT_FLAG();

        g_bAlarm = TRUE;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable LXT */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Waiting for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable RTC clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{
    S_RTC_TIME_DATA_T sInitTime;
    S_RTC_TIME_DATA_T sCurTime;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|          M2U51 RTC Sample Code         |\n");
    printf("+----------------------------------------+\n");
    /* Time Setting */
    sInitTime.u32Year       = 2025;
    sInitTime.u32Month      = 5;
    sInitTime.u32Day        = 1;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_THURSDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    if(RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        while(1);
    }

    printf("\n RTC Alarm Test (Alarm after 10 seconds)\n\n");

    g_bAlarm = FALSE;

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf(" Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,
           sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

    /* The alarm time setting */
    sCurTime.u32Second = sCurTime.u32Second + 10;

    /* Set the alarm time */
    RTC_SetAlarmDateAndTime(&sCurTime);

    /* Clear interrupt status */
    RTC_CLEAR_ALARM_INT_FLAG();

    /* Enable RTC Alarm Interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    NVIC_EnableIRQ(RTC_IRQn);

    while(!g_bAlarm);

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf(" Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,
           sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

    /* Disable RTC Alarm Interrupt */
    RTC_DisableInt(RTC_INTEN_ALMIEN_Msk);
    NVIC_DisableIRQ(RTC_IRQn);

    printf("\n RTC Alarm Test End !!\n");

    while(1);

}



/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/



