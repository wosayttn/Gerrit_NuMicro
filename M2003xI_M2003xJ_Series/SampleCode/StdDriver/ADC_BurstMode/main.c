/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Perform A/D Conversion with ADC burst mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------*/
/* Define global variables and constants        */
/*----------------------------------------------*/
#define CONV_TOTAL_COUNT    20

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

    /* Set PB.2 to input mode */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    /* Configure the PB.2 ADC analog input pins. */
    SET_ADC0_CH2_PB2();
    /* Disable the PB.2 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);

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
    uint8_t  u8Option;
    uint32_t u32ConvCount;
    int32_t  i32ConversionData[CONV_TOTAL_COUNT];

    printf("+----------------------------------------------------------------------+\n");
    printf("|                     ADC burst mode sample code                       |\n");
    printf("+----------------------------------------------------------------------+\n");

    for(u32ConvCount = 0; u32ConvCount < CONV_TOTAL_COUNT; u32ConvCount++)
    {
        i32ConversionData[u32ConvCount] = 0;
    }

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  Other keys: exit burst mode test\n");
        u8Option = getchar();

        if(u8Option == '1')
        {
            /* Set input mode as single-end, burst mode, and select channel 2 */
            ADC_Open(ADC, 0, ADC_ADCR_ADMD_BURST, BIT2);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
            u32ConvCount = 0;
            ADC_START_CONV(ADC);

            while(1)
            {
                /* Wait ADC conversion completed */
                while (ADC_GET_INT_FLAG(ADC, ADC_ADF_INT)==0);
                ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* clear ADF interrupt flag */

                /* Get the conversion result until VALIDF turns to 0 */
                while(ADC->ADSR0 & ADC_ADSR0_VALIDF_Msk)
                {
                    /* Get the conversion result from ADC channel 0 always */
                    i32ConversionData[u32ConvCount++] = ADC_GET_CONVERSION_DATA(ADC, 0);
                    if(u32ConvCount == CONV_TOTAL_COUNT)
                        break;
                }

                if(u32ConvCount == CONV_TOTAL_COUNT)
                    break;
            }

            /* Stop A/D conversion */
            ADC_STOP_CONV(ADC);

            /* Show the conversion result */
            for(u32ConvCount = 0; u32ConvCount < CONV_TOTAL_COUNT; u32ConvCount++)
            {
                printf("Conversion result of channel 2 [#%02d]: 0x%x (%d)\n", u32ConvCount+1, i32ConversionData[u32ConvCount], i32ConversionData[u32ConvCount]);
            }

            /* Clear remaining data in FIFO that got before stop ADC */
            while (ADC_IS_DATA_VALID(ADC, 0))
            {
                i32ConversionData[0] = ADC_GET_CONVERSION_DATA(ADC, 0);
            }
        }
        else
            return;

        printf("\n");
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

    printf("Exit ADC sample code\n");

    /* Got nowhere to go, just loop forever */
    while (1);
}

/*** (C) COPYRIGHT 2016-2026 Nuvoton Technology Corp. ***/
