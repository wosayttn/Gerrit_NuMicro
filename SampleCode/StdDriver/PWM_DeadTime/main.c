/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 18/07/19 2:16p $
 * @brief    Demonstrate how to use PWM Dead Zone function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       PWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM0 interrupt event
 */
void PWM0P0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    /* Channel 0 frequency is 6,000 Hz, every 1 second enter this IRQ handler 6,000 times. */
    if(++cnt == 6000)
    {
        if(out)
            PWM_EnableOutput(PWM0, 0x3F);
        else
            PWM_DisableOutput(PWM0, 0x3F);

        out ^= 1;
        cnt = 0;
    }
    /* Clear channel 0 period interrupt flag */
    PWM_ClearPeriodIntFlag(PWM0, 0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for PWM0 Channel0~5 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA5MFP_Msk) | SYS_GPA_MFPL_PA5MFP_PWM0_CH0;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA4MFP_Msk) | SYS_GPA_MFPL_PA4MFP_PWM0_CH1;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA3MFP_Msk) | SYS_GPA_MFPL_PA3MFP_PWM0_CH2;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA2MFP_Msk) | SYS_GPA_MFPL_PA2MFP_PWM0_CH3;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA1MFP_Msk) | SYS_GPA_MFPL_PA1MFP_PWM0_CH4;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_PWM0_CH5;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output PWM0 channel 0~5 with different \n");
    printf("  frequency and duty, enable dead time function of all PWM0 pairs.\n");
    printf("  And also enable/disable PWM output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PA.5), PWM0_CH1(PA.4), PWM0_CH2(PA.3), PWM0_CH3(PA.2), PWM0_CH4(PA.1), PWM0_CH5(PA.0)\n");

    /* Set Pwm mode as complementary mode */
    PWM_ENABLE_COMPLEMENTARY_MODE(PWM0);

    /* PWM0 channel 0 frequency is 6,000 Hz, duty 30% */
    PWM_ConfigOutputChannel(PWM0, 0, 6000, 30);
    SYS_UnlockReg();
    PWM_EnableDeadZone(PWM0, 0, 100);
    SYS_LockReg();

    /* PWM0 channel 2 frequency is 3,000 Hz, duty 50% */
    PWM_ConfigOutputChannel(PWM0, 2, 3000, 50);
    SYS_UnlockReg();
    PWM_EnableDeadZone(PWM0, 2, 200);
    SYS_LockReg();

    /* PWM0 channel 4 frequency is 1,000 Hz, duty 50% */
    PWM_ConfigOutputChannel(PWM0, 4, 1000, 50);
    SYS_UnlockReg();
    PWM_EnableDeadZone(PWM0, 4, 400);
    SYS_LockReg();

    /* Enable output of PWM0 channel 0~5 */
    PWM_EnableOutput(PWM0, 0x3F);

    /* Enable PWM0 channel 0 period interrupt, use channel 0 to measure time. */
    PWM_EnablePeriodInt(PWM0, 0, 0);
    NVIC_EnableIRQ(PWM0_P0_IRQn);

    /* Start */
    PWM_Start(PWM0, 0x3F);

    while(1);
}
