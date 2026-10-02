/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC function and displays current time to the
 *           UART console
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

volatile uint32_t  g_u32TICK = 0;

/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{

    if (RTC_GET_TICK_INT_FLAG())        /* tick interrupt occurred */
    {
        S_RTC_TIME_DATA_T sCurTime;

        RTC_CLEAR_TICK_INT_FLAG() ;

        /* Get the current time */
        RTC_GetDateAndTime(&sCurTime);

        printf(" Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

        g_u32TICK++;
    }

}

void Delay(uint32_t ucnt)
{
    volatile uint32_t i = ucnt;

    while (i--);
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

    SYS_Init();

    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+\n");
    printf("|     RTC Time Display Sample Code    |\n");
    printf("+-------------------------------------+\n\n");

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

    printf("RTC Time Display Test (Exit after 5 seconds)\n\n");

    /* Set Tick Period */
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    /* Enable RTC Tick Interrupt */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

    g_u32TICK = 0;
    while(g_u32TICK < 5);

    printf("\n RTC Time Display Test End !!\n");

    /* Disable RTC Tick Interrupt */
    RTC_DisableInt(RTC_INTEN_TICKIEN_Msk);
    NVIC_DisableIRQ(RTC_IRQn);

    while(1);

}






/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/



