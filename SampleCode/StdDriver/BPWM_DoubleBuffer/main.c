/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Change duty cycle and period of output waveform by BPWM Double Buffer function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       BPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
void BPWM0_IRQHandler(void)
{
    static int i8Toggle = 0;

    // Update BPWM0 channel 0 period and duty
    if (i8Toggle == 0)
    {
        BPWM_SET_CNR(BPWM0, 0, 99);
        BPWM_SET_CMR(BPWM0, 0, 39);
    }
    else
    {
        BPWM_SET_CNR(BPWM0, 0, 399);
        BPWM_SET_CMR(BPWM0, 0, 199);
    }

    i8Toggle ^= 1;
    // Clear channel 0 period interrupt flag
    BPWM_ClearPeriodIntFlag(BPWM0, 0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40 MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable BPWM module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PA.0 multi-function pin for BPWM0 channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0 channel 0(PA.0)\n");
    printf("\nUse double buffer feature.\n");

    /*
        BPWM0 channel 0 waveform of this sample shown below:

        |<-        CNR + 1  clk     ->|  CNR + 1 = 399 + 1 CLKs
                       |<-CMR+1 clk ->|  CMR + 1 = 199 + 1 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                               |<CMR+1>|  CMR + 1 = 39 + 1 CLKs

      __                ______________          _______
        |______200_____|     200      |____60__|   40  |_____BPWM waveform

    */

    /*
      Configure BPWM0 channel 0 init period and duty.
      Period is PCLK / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 40 MHz / (1 * (199 + 1)) = 200000 Hz
      Duty ratio = (100) / (199 + 1) = 50%
    */
    /* BPWM0 channel 0 frequency is 200000 Hz, duty 50% */
    BPWM_ConfigOutputChannel(BPWM0, 0, 200000, 50);

    /* Enable output of BPWM0 channel 0 */
    BPWM_EnableOutput(BPWM0, BPWM_CH_0_MASK);

    /* Enable BPWM0 channel 0 period interrupt, use channel 0 to measure time. */
    BPWM_EnablePeriodInt(BPWM0, 0, 0);
    NVIC_EnableIRQ(BPWM0_IRQn);

    /* Start */
    BPWM_Start(BPWM0, BPWM_CH_0_MASK);

    while (1);

}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
