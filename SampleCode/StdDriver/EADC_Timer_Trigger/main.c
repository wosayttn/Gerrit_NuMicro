/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to trigger EADC by Timer.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC00_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = 1000;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);/* Clear the A/D ADINT0 interrupt flag */

    /*Confirm that the Flag has been cleared.*/
    while (EADC_GET_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk) != 0)
    {
        if ((--u32TimeOutCnt) == 0)
            break;
    }

    g_u32AdcIntFlag = 1;
    g_u32COVNUMFlag++;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select EADC peripheral clock source(maximum clock frequency = divider 4) */
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_HCLK, CLK_CLKDIV0_EADC0(10));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Enable GPIOB module clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Select timer 0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PB.2 to input mode */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    /* Configure the PB.2 ADC analog input pins. */
    SET_EADC0_CH2_PB2();
    /* Disable the PB.2 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);

    /* Set PB.0 - PB.1 to input mode */
    GPIO_SetMode(PB, BIT0 | BIT1, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.1 ADC analog input pins. */
    SET_EADC0_CH0_PB0();
    SET_EADC0_CH1_PB1();
    /* Disable the PB.0 - PB.1 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1);


}

void TIMER0_Init()
{
    /* Set timer0 periodic time-out period is 1s if timer clock is 12 MHz */
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_EADC);
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
}

void EADC_FunctionTest()
{
    uint8_t  u8Option;
    uint32_t u32ChannelNum;
    uint32_t u32COVNUMFlag = 0;
    int32_t  i32ConversionData[6] = {0};

    printf("\n");
    printf("+-------------------------------------------+\n");
    printf("|           Timer trigger mode test         |\n");
    printf("+-------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    while (1)
    {
        printf("Select test items:\n");
        printf("  [1] Basic EADC conversion (channel 0 only)\n");
        printf("  [2] Basic EADC conversion (channel 1 only)\n");
        printf("  Other keys: exit EADC test\n");

        u8Option = getchar();

        if (u8Option == '1')
            u32ChannelNum = 0;
        else if (u8Option == '2')
            u32ChannelNum = 1;
        else
            break;  /* exit while loop */

        /* Set the EADC and enable the A/D converter */
        EADC_Open(EADC0, 0);

        /* To decrease the effect of electrical random noise */
        EADC_Calibration(EADC0);

        /* Configure the sample module 0 for analog input channel and enable Timer0 trigger source */
        EADC_ConfigSampleModule(EADC0, 0, EADC_TIMER0_TRIGGER, u32ChannelNum);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

        /* Enable the sample module 0 interrupt.  */
        EADC_ENABLE_INT(EADC0, BIT0);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);
        NVIC_EnableIRQ(EADC00_IRQn);

        printf("Conversion result of channel 2:\n");

        /* Reset the ADC indicator and enable Timer0 counter */
        g_u32AdcIntFlag = 0;
        g_u32COVNUMFlag = 0;
        TIMER_Start(TIMER0);

        while (1)
        {
            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            while (g_u32AdcIntFlag == 0);

            /* Reset the EADC interrupt indicator */
            g_u32AdcIntFlag = 0;

            /* Get the conversion result of the sample module 0 */
            u32COVNUMFlag = g_u32COVNUMFlag - 1;
            i32ConversionData[u32COVNUMFlag] = EADC_GET_CONV_DATA(EADC0, 0);
            printf("    0x%X (%d)\n", i32ConversionData[u32COVNUMFlag], i32ConversionData[u32COVNUMFlag]);

            if (g_u32COVNUMFlag > 5)
                break;
        }

        /* Disable Timer0 counter */
        TIMER_Stop(TIMER0);

        /* Disable the ADINT0 interrupt */
        EADC_DISABLE_INT(EADC0, BIT0);
    }
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Init TIMER0 for EADC */
    TIMER0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable Timer0 IP clock */
    CLK_DisableModuleClock(TMR0_MODULE);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
