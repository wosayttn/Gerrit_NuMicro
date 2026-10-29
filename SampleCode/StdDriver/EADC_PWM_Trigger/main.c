/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to trigger EADC by PWM.
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
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
    g_u32AdcIntFlag = 1;
    g_u32COVNUMFlag++;

    /*Confirm that the Flag has been cleared.*/
    while (EADC_GET_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk) != 0)
    {
        if ((--u32TimeOutCnt) == 0)
            break;
    }

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
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_HCLK, CLK_CLKDIV0_EADC0(15));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Enable GPIOB module clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Select PWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

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

    /* Set PA multi-function pins for PWM0 Channel 0 */
    SET_PWM0_CH0_PA5();

}


void PWM0_Init()
{
    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 10);

    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 0, 1000);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 0, 2000);

    /* PWM period point trigger ADC enable */
    PWM_EnableADCTrigger(PWM0, 0, PWM_TRIGGER_ADC_EVEN_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, BIT0, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, BIT0);
}

void EADC_FunctionTest()
{
    uint8_t  u8Option;
    uint32_t u32ChannelNum;
    uint8_t  u8Index = 0;
    int32_t  ai32ConversionData[6] = {0};
    uint32_t u32COVNUMFlag = 0;

    printf("\n");
    printf("+---------------------------------------------------+\n");
    printf("|               PWM trigger mode test              |\n");
    printf("+---------------------------------------------------+\n");

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

        /* Configure the sample module 0 for analog input channel and enable PWM0 trigger source */
        EADC_ConfigSampleModule(EADC0, 0, EADC_PWM0TG0_TRIGGER, u32ChannelNum);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

        /* Enable the sample module 0 interrupt */
        EADC_ENABLE_INT(EADC0, BIT0);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);
        NVIC_EnableIRQ(EADC00_IRQn);

        printf("Conversion result of channel 2:\n");

        /* Reset the EADC indicator and enable PWM0 channel 0 counter */
        g_u32AdcIntFlag = 0;
        g_u32COVNUMFlag = 0;
        PWM_Start(PWM0, BIT0); //PWM0 channel 0 counter start running.

        while (1)
        {
            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            while (g_u32AdcIntFlag == 0);

            /* Reset the ADC interrupt indicator */
            g_u32AdcIntFlag = 0;

            /* Get the conversion result of the sample module 0 */
            u32COVNUMFlag = g_u32COVNUMFlag - 1;
            ai32ConversionData[u32COVNUMFlag] = EADC_GET_CONV_DATA(EADC0, 0);

            if (g_u32COVNUMFlag > 5)
                break;
        }

        /* Disable PWM0 channel 0 counter */
        PWM_ForceStop(PWM0, BIT0); //PWM0 counter stop running.

        /* Disable sample module 0 interrupt */
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);

        for (u8Index = 0; (u8Index) < 6; u8Index++)
            printf("                                0x%X (%d)\n", ai32ConversionData[u8Index], ai32ConversionData[u8Index]);

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

    /* Init PWM for EADC */
    PWM0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Disable PWM0 IP clock */
    CLK_DisableModuleClock(PWM0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
