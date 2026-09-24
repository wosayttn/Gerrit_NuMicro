/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use ADINT interrupt to do the ADC Single-cycle scan conversion.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------*/
/* Define global variables and constants        */
/*----------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;

/*----------------------------------------------*/
/* ADC interrupt handler                        */
/*----------------------------------------------*/
void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
}

void SYS_Init(void)
{
    /*------------------------------------------*/
    /* Init System Clock                        */
    /*------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* The Maximum ADC clock frequency is 16 MHz.
     * Hence, we set PLL to 32 MHz, HCLK = PLL/1, and PCLK1 = HCLK/1 = 32 MHz.
     * ADC clock source is from PCLK1/2 = 16 MHz.
     */
    /* Set PLL to 32MHz */
    CLK_EnablePLL((uint32_t)NULL, FREQ_32MHZ);

    /* Switch HCLK clock source to PLL and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_HCLKDIV_HCLK(1));

    /* Set PCLK0 = HCLK/1 and PCLK1 = HCLK/1 */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV_DIV1 | CLK_PCLKDIV_PCLK1DIV_DIV1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL_UART0SEL_HIRC, CLK_UARTDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set ADC clock source and divider */
    CLK_SetModuleClock(ADC0_MODULE, (uint32_t)NULL, CLK_ADCDIV_ADC0(2));

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC0_MODULE);

    /*----------------------------------*/
    /* Init I/O Multi-function          */
    /*----------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set PB.0 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT0|BIT1|BIT2|BIT3, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.3 ADC analog input pins. */
    SET_ADC0_CH0_PB0();
    SET_ADC0_CH1_PB1();
    SET_ADC0_CH2_PB2();
    SET_ADC0_CH3_PB3();
    /* Disable the PB.0 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT1|BIT2|BIT3);

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

void ADC_FunctionTest()
{
    uint8_t  u8Option, u32ChannelCount = 0;
    int32_t  i32ConversionData[8] = {0};

    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  Other keys: exit single-cycle scan mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end, Single-cycle scan mode, and select channel 0~3 */
            ADC_Open(ADC, 0, ADC_ADCR_ADMD_SINGLE_CYCLE, BIT0|BIT1|BIT2|BIT3);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Enable the sample module interrupt */
            ADC_ENABLE_INT(ADC, ADC_ADF_INT);  /* Enable sample module A/D interrupt. */
            NVIC_EnableIRQ(ADC_IRQn);

            /* Reset the ADC indicator and trigger sample module to start A/D conversion */
            g_u32AdcIntFlag = 0;
            ADC_START_CONV(ADC);

            /* Wait conversion done */
            while(g_u32AdcIntFlag == 0);

            /* Wait conversion data become valid */
            while(ADC_IS_DATA_VALID(ADC, 0) == 0);
            while(ADC_IS_DATA_VALID(ADC, 1) == 0);
            while(ADC_IS_DATA_VALID(ADC, 2) == 0);
            while(ADC_IS_DATA_VALID(ADC, 3) == 0);

            /* Get the conversion result of the sample module */
            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
                i32ConversionData[u32ChannelCount] = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
                printf("Conversion result of channel %d: 0x%x (%d)\n", u32ChannelCount, i32ConversionData[u32ChannelCount], i32ConversionData[u32ChannelCount]);
        }
        else
            return;
    }
}

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to print message */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    /* Disable ADC Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("Exit ADC sample code\n");

    /* Got nowhere to go, just loop forever */
    while (1);
}

/*** (C) COPYRIGHT 2016-2026 Nuvoton Technology Corp. ***/
