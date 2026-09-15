/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use ADINT interrupt to do the EADC continuous scan conversion.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;


void EADC0_INT0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/1 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable EADC peripheral clock */
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_HIRC, CLK_CLKDIV0_EADC0(2));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB.0 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT0|BIT1|BIT2|BIT3, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.3 ADC analog input pins. */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB0MFP_Msk | SYS_GPB_MFP0_PB1MFP_Msk | SYS_GPB_MFP0_PB2MFP_Msk | SYS_GPB_MFP0_PB3MFP_Msk)) |
                    (SYS_GPB_MFP0_PB0MFP_EADC0_CH0 | SYS_GPB_MFP0_PB1MFP_EADC0_CH1 | SYS_GPB_MFP0_PB2MFP_EADC0_CH2 | SYS_GPB_MFP0_PB3MFP_EADC0_CH3);
    /* Disable the PB.0 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT1|BIT2|BIT3);

    /* Set PB.14 - PB.15 to input mode */
    GPIO_SetMode(PB, BIT14|BIT15, GPIO_MODE_INPUT);
    /* Configure the PB.14 - PB.15 ADC analog input pins. */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(SYS_GPB_MFP3_PB14MFP_Msk | SYS_GPB_MFP3_PB15MFP_Msk  )) |
                    (SYS_GPB_MFP3_PB14MFP_EADC0_CH14 | SYS_GPB_MFP3_PB15MFP_EADC0_CH15);
    /* Disable the PB.14 - PB.15 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT14|BIT15);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void EADC_FunctionTest()
{
    uint8_t  u8Option, u32SAMPLECount = 0;
    int32_t  i32ConversionData[8] = {0};

    /* The Maximum EADC clock frequency is 36 MHz.
     * Hence, we set PLL to 72 MHz, HCLK 72 MHz and PCLK to 72 MHz.
     * EADC clock source is from PCLK/2 = 36 MHz.
     */
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (channel pair 7: channel 14 and 15)\n");
        printf("  Other keys: exit continuous scan mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end and enable the A/D converter */
            EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

            /* Configure the sample 4 module for analog input channel 0 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 4, EADC_ADINT0_TRIGGER, 0);
            /* Configure the sample 5 module for analog input channel 1 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 5, EADC_ADINT0_TRIGGER, 1);
            /* Configure the sample 6 module for analog input channel 2 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 6, EADC_ADINT0_TRIGGER, 2);
            /* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 7, EADC_ADINT0_TRIGGER, 3);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 7 interrupt */
            EADC_ENABLE_INT(EADC, BIT0);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);
            NVIC_EnableIRQ(EADC0_INT0_IRQn);

            /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC, BIT7);

            __WFI();

            /* Disable the sample module 7 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);

            /* Wait conversion done */
            while(EADC_GET_DATA_VALID_FLAG(EADC, (BIT7|BIT6|BIT5|BIT4)) != (BIT7|BIT6|BIT5|BIT4));

            /* Get the conversion result of the sample module */
            for(u32SAMPLECount = 0; u32SAMPLECount < 4; u32SAMPLECount++)
                i32ConversionData[u32SAMPLECount] = EADC_GET_CONV_DATA(EADC, (u32SAMPLECount + 4));

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 4; g_u32COVNUMFlag++)
                printf("Conversion result of channel %d: 0x%X (%d)\n", g_u32COVNUMFlag, i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential and enable the A/D converter */
            EADC_Open(EADC, EADC_CTL_DIFFEN_DIFFERENTIAL);

            /* Configure the sample module 5 for analog input channel 14 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 5, EADC_ADINT0_TRIGGER, 14);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 5 interrupt */
            EADC_ENABLE_INT(EADC, BIT0);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT5);
            NVIC_EnableIRQ(EADC0_INT0_IRQn);

            /* Reset the ADC indicator and trigger sample module 5 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC, BIT5);

            __WFI();

            /* Disable the sample module 5 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT5);

            /* Wait conversion done */
            while(EADC_GET_DATA_VALID_FLAG(EADC, BIT5) != BIT5);

            /* Get the conversion result of the sample module */
            i32ConversionData[0] = EADC_GET_CONV_DATA(EADC, 5);
            printf("Conversion result of channel pair 7 (channel 14/15): 0x%X (%d)\n", i32ConversionData[0], i32ConversionData[0]);
        }
        else
            return;
    }
}


int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while(1);
}
