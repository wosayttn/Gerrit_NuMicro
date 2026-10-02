/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate the RTC function and displays current time to the UART console.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32RTCTInt = 0;

/**
 * @brief       IRQ Handler for RTC Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in startup_M471.s.
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG() == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG();

        g_u32RTCTInt = 1;

        PA2 ^= 1;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);  /* 32K (LXT) Enabled */

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Waiting for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as 96MHz from PLL */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable RTC module clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_RTC_TIME_DATA_T sInitTime, sReadRTC;
    uint32_t u32Sec;
    uint8_t u8IsNewDateTime = 0;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------+\n");
    printf("|    RTC Time Display Sample Code   |\n");
    printf("+-----------------------------------+\n\n");

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Open RTC and start counting */
    sInitTime.u32Year       = 2020;
    sInitTime.u32Month      = 5;
    sInitTime.u32Day        = 1;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_MONDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;
    if(RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        while(1);
    }

    /* Enable RTC tick interrupt, one RTC tick is 1 second */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    printf("# Showing RTC date/time on UART0.\n\n");
    printf("1.) Use PA.2 to check tick period time is one second or not.\n");
    printf("2.) Show RTC date/time and change date/time after 5 seconds:\n");

    /* Use PA.2 to check tick period time */
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE2_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE2_Pos);
    PA2 = 1;

    u32Sec = 0;
    g_u32RTCTInt = 0;
    while(1)
    {
        if(g_u32RTCTInt == 1)
        {
            g_u32RTCTInt = 0;

            /* Read current RTC date/time */
            RTC_GetDateAndTime(&sReadRTC);
            printf("    %d/%02d/%02d %02d:%02d:%02d\n",
                   sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

            if(u32Sec == sReadRTC.u32Second)
            {
                printf("\nRTC time is incorrect.\n");
                while(1) {}
            }

            u32Sec = sReadRTC.u32Second;

            if(u8IsNewDateTime == 0)
            {
                if(u32Sec == (sInitTime.u32Second + 5))
                {
                    printf("\n");
                    printf("3.) Update new date/time to 2020/05/15 11:12:13.\n");

                    u8IsNewDateTime = 1;
                    RTC_SetDate(2020, 5, 15, RTC_MONDAY);
                    RTC_SetTime(11, 12, 13, RTC_CLOCK_24, RTC_AM);
                }
            }
        }
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
