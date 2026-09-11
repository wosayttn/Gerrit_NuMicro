/****************************************************************************
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to trigger DAC by timer.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

uint16_t g_au16Sine[] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                         4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                         3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
                         639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
                         238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876
                        };

const uint32_t g_u32ArraySize = sizeof(g_au16Sine) / sizeof(uint16_t);
static uint32_t g_u32Index = 0;

void DAC_IRQHandler(void)
{
    if (DAC_GET_INT_FLAG(DAC0, 0))
    {
        if (g_u32Index == g_u32ArraySize)
            g_u32Index = 0;
        else
        {
            DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index++]);
            /* Clear the DAC conversion complete finish flag */
            DAC_CLR_INT_FLAG(DAC0, 0);
        }
    }

    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable DAC module clock */
    CLK_EnableModuleClock(DAC_MODULE);

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select timer 0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB12 multi-function pin for DAC voltage output */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~SYS_GPB_MFP3_PB12MFP_Msk) | SYS_GPB_MFP3_PB12MFP_DAC0_OUT;

    /* Set PB.12 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE12_Msk) ;

    /* Disable digital input path of analog pin DAC01_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 12));

    /* Set PA multi-function pins for UART0 RXD=PA.4 and TXD=PA.5 */
    SYS->GPA_MFP1 &= ~(SYS_GPA_MFP1_PA5MFP_Msk | SYS_GPA_MFP1_PA4MFP_Msk);
    SYS->GPA_MFP1 |= SYS_GPA_MFP1_PA5MFP_UART0_TXD | SYS_GPA_MFP1_PA4MFP_UART0_RXD;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* This sample code uses semihost as the debug port */
    printf("+----------------------------------------------------------+\n");
    printf("|            DAC Driver Sample Code                        |\n");
    printf("+----------------------------------------------------------+\n");
    printf("Hit any key to start triggering DAC by TIMER.\n");
    getchar();
    printf("Code started ...\n");

    /* Set the timer 0 trigger,DAC and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_TIMER0_TRIGGER);

    /* The DAC conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index]);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the DAC interrupt.  */
    DAC_ENABLE_INT(DAC0, 0);
    NVIC_EnableIRQ(DAC_IRQn);

    /* Configure Timer 0 frequency to 1000Hz and start it. */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_DAC);
    TIMER_Start(TIMER0);

    while (1);

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
