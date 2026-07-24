/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use ADINT interrupt to do the EADC continuous scan conversion.
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
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
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
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_HCLK, CLK_CLKDIV0_EADC0(12));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();
    /* Set PB.2 - PB.5 to input mode */
    GPIO_SetMode(PB, BIT2 | BIT3 | BIT4 | BIT5, GPIO_MODE_INPUT);
    /* Configure the PB.2 - PB.5 ADC analog input pins. */
    SET_EADC0_CH2_PB2();
    SET_EADC0_CH3_PB3();
    SET_EADC0_CH4_PB4();
    SET_EADC0_CH5_PB5();
    /* Disable the PB.2 - PB.5 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2 | BIT3 | BIT4 | BIT5);

    /* Set PB.0 - PB.1 to input mode */
    GPIO_SetMode(PB, BIT0 | BIT1, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.1 ADC analog input pins. */
    SET_EADC0_CH0_PB0();
    SET_EADC0_CH1_PB1();
    /* Disable the PB.0 - PB.1 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1);

}


void EADC_FunctionTest()
{
    uint8_t  u8Option, u32SAMPLECount = 0;
    int32_t  ai32ConversionData[8] = {0};

    /* The Maximum EADC clock frequency is 36 MHz.
     * Hence, we set PLL to 144 MHz, HCLK 144 MHz and PCLK1 to 72 MHz.
     * EADC clock source is from HCLK/12 = 6 MHz.
     */
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    while (1)
    {
        printf("\n\nSelect test items:\n");
        printf("  [1] continuous scan mode test (channel 2, 3, 4 and 5)\n");
        printf("  Other keys: exit \n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Set input mode as single-end and enable the A/D converter */
            EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

            /* To decrease the effect of electrical random noise */
            EADC_Calibration(EADC0);

            /* Configure the sample 4 module for analog input channel 2 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 4, EADC_ADINT0_TRIGGER, 2);
            /* Configure the sample 5 module for analog input channel 3 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 5, EADC_ADINT0_TRIGGER, 3);
            /* Configure the sample 6 module for analog input channel 4 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 6, EADC_ADINT0_TRIGGER, 4);
            /* Configure the sample 7 module for analog input channel 5 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 7, EADC_ADINT0_TRIGGER, 5);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 7 interrupt */
            EADC_ENABLE_INT(EADC0, BIT0);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT7);
            NVIC_EnableIRQ(EADC00_IRQn);

            /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC0, BIT7);

            CLK_Idle();

            /* Disable the sample module 7 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT7);

            /* Wait conversion done */
            while (EADC_GET_DATA_VALID_FLAG(EADC0, (BIT7 | BIT6 | BIT5 | BIT4)) != (BIT7 | BIT6 | BIT5 | BIT4));

            /* Get the conversion result of the sample module */
            for (u32SAMPLECount = 0; u32SAMPLECount < 4; u32SAMPLECount++)
                ai32ConversionData[u32SAMPLECount] = EADC_GET_CONV_DATA(EADC0, (u32SAMPLECount + 4));

            printf("Conversion result of channel %d: 0x%X (%d)\n", 2, ai32ConversionData[0], ai32ConversionData[0]);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 3, ai32ConversionData[1], ai32ConversionData[1]);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 4, ai32ConversionData[2], ai32ConversionData[2]);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 5, ai32ConversionData[3], ai32ConversionData[3]);

        }
        else
            return;
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

#if !(defined(DEBUG_ENABLE_SEMIHOST))
    /* Init Debug UART for printf */
    InitDebugUart();
#endif

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

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
