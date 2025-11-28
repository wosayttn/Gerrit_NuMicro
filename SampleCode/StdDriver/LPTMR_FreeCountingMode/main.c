/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use the LPTMR LPTM0_EXT pin to demonstrate timer free counting mode
 *           function. And displays the measured input frequency to
 *           UART console.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void LPTMR0_IRQHandler(void)
{
    static int cnt = 0;
    static uint32_t t0, t1;

    LPTMR_ClearCaptureIntFlag(LPTMR0);

    if(cnt == 0)
    {
        t0 = LPTMR_GetCaptureData(LPTMR0);
        cnt++;
    }
    else if(cnt == 1)
    {
        t1 = LPTMR_GetCaptureData(LPTMR0);
        cnt++;
        if(t0 >= t1)
        {
            /* over run, drop this data and do nothing */
        }
        else
        {
            /* LPTMR0 clock source = HIRC */
            printf("Input frequency is %luHz\n", (__HIRC) / (t1 - t0));
        }
    }
    else
    {
        cnt = 0;
    }
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select LPTMR0 clock source */
    CLK_SetModuleClock(LPTMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);

    /* Enable LPTMR0 clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set LPTMR0 capture pin */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(SYS_GPB_MFP3_PB15MFP_Msk)) |
                    (SYS_GPB_MFP3_PB15MFP_LPTM0_EXT);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main(void)
{
    int volatile i;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nThis sample code demonstrate LPTMR0 free counting mode.\n");
    printf("Please connect input source with LPTMR0 capture pin PB.15, press any key to continue\n");
    getchar();

    /* Give a dummy target frequency here. Will over write capture resolution with macro */
    LPTMR_Open(LPTMR0, LPTMR_PERIODIC_MODE, 1000000);

    /* Update prescale to set proper resolution. */
    LPTMR_SET_PRESCALE_VALUE(LPTMR0, 0);

    /* Set compare value as large as possible, so don't need to worry about counter overrun too frequently. */
    LPTMR_SET_CMP_VALUE(LPTMR0, LPTMR_CMP_MAX_VALUE);

    /* Configure LPTMR0 free counting mode, capture value on rising edge */
    LPTMR_EnableCapture(LPTMR0, LPTMR_CAPTURE_FREE_COUNTING_MODE, LPTMR_CAPTURE_EVENT_RISING);

    /* Start LPTMR0 */
    LPTMR_Start(LPTMR0);

    /* Enable timer interrupt */
    LPTMR_EnableCaptureInt(LPTMR0);
    NVIC_EnableIRQ(LPTMR0_IRQn);

    while(1);
}
