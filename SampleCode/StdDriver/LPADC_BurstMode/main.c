/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Perform A/D Conversion with LPADC burst mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define CONV_TOTAL_COUNT    20


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
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

    /* Set HCLK1 to HIRC/1 */
    CLK_SetModuleClock(HCLK1_MODULE, CLK_CLKSEL0_HCLK1SEL_HIRC, LPSCC_CLKDIV0_HCLK1(1));

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Set PCLK2 to HCLK1/2 */
    LPSCC->CLKDIV0 = (LPSCC->CLKDIV0 & ~(LPSCC_CLKDIV0_APB2DIV_Msk)) |
                     (LPSCC_CLKDIV0_APB2DIV_DIV2);

    /* LPADC clock source is PCLK2 = 24MHz, set divider to 2, LPADC clock is 24/2 MHz */
    CLK_SetModuleClock(LPADC0_MODULE, LPSCC_CLKSEL0_LPADC0SEL_PCLK2, LPSCC_CLKDIV0_LPADC0(2));

    /* Enable LPADC module clock */
    CLK_EnableModuleClock(LPADC0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB.2 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT2|BIT3, GPIO_MODE_INPUT);

    /* Configure the PB.2 - PB.3 LPADC analog input pins. */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB2MFP_Msk | SYS_GPB_MFP0_PB3MFP_Msk)) |
                    (SYS_GPB_MFP0_PB2MFP_LPADC0_CH2 | SYS_GPB_MFP0_PB3MFP_LPADC0_CH3);

    /* Disable the PB.2 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2|BIT3);

    /* Lock protected registers */
    SYS_LockReg();
}

void LPADC_FunctionTest()
{
    uint8_t  u8Option;
    uint32_t u32ConvCount;
    int32_t  i32ConversionData[CONV_TOTAL_COUNT];

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                   LPADC burst mode sample code                       |\n");
    printf("+----------------------------------------------------------------------+\n");

    for(u32ConvCount = 0; u32ConvCount < CONV_TOTAL_COUNT; u32ConvCount++)
    {
        i32ConversionData[u32ConvCount] = 0;
    }

    /* Enable LPADC converter */
    LPADC_POWER_ON(LPADC0);

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only)\n");
        printf("  Other keys: exit burst mode test\n");
        u8Option = getchar();

        if(u8Option == '1')
        {
            /* Set input mode as single-end, burst mode, and select channel 2 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_BURST, BIT2);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

            /* Reset the LPADC interrupt indicator and trigger sample module to start A/D conversion */
            u32ConvCount = 0;
            LPADC_START_CONV(LPADC0);

            while(1)
            {
                /* Wait LPADC conversion completed */
                while (LPADC_GET_INT_FLAG(LPADC0, LPADC_ADF_INT)==0);
                LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT); /* clear ADF interrupt flag */

                /* Get the conversion result until VALIDF turns to 0 */
                while(LPADC0->ADSR0 & LPADC_ADSR0_VALIDF_Msk)
                {
                    /* Get the conversion result from LPADC channel 0 always */
                    i32ConversionData[u32ConvCount++] = LPADC_GET_CONVERSION_DATA(LPADC0, 0);
                    if(u32ConvCount == CONV_TOTAL_COUNT)
                        break;
                }

                if(u32ConvCount == CONV_TOTAL_COUNT)
                    break;
            }

            /* Stop A/D conversion */
            LPADC_STOP_CONV(LPADC0);

            /* Show the conversion result */
            for(u32ConvCount = 0; u32ConvCount < CONV_TOTAL_COUNT; u32ConvCount++)
            {
                printf("Conversion result of channel 2 [#%d]: 0x%X (%d)\n", u32ConvCount+1, i32ConversionData[u32ConvCount], i32ConversionData[u32ConvCount]);
            }

            /* Clear remaining data in FIFO that got before stop LPADC */
            while (LPADC_IS_DATA_VALID(LPADC0, 0))
            {
                i32ConversionData[0] = LPADC_GET_CONVERSION_DATA(LPADC0, 0);
            }
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential, burst mode, and select channel 2 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_DIFFERENTIAL, LPADC_ADCR_ADMD_BURST, BIT2);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

            /* Reset the LPADC interrupt indicator and trigger sample module to start A/D conversion */
            u32ConvCount = 0;
            LPADC_START_CONV(LPADC0);

            while(1)
            {
                /* Wait LPADC conversion completed */
                while (LPADC_GET_INT_FLAG(LPADC0, LPADC_ADF_INT)==0);
                LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT); /* clear ADF interrupt flag */

                /* Get the conversion result until VALIDF turns to 0 */
                while(LPADC0->ADSR0 & LPADC_ADSR0_VALIDF_Msk)
                {
                    /* Get the conversion result from LPADC channel 0 always */
                    i32ConversionData[u32ConvCount++] = LPADC_GET_CONVERSION_DATA(LPADC0, 0);
                    if(u32ConvCount == CONV_TOTAL_COUNT)
                        break;
                }

                if(u32ConvCount == CONV_TOTAL_COUNT)
                    break;
            }

            /* Stop A/D conversion */
            LPADC_STOP_CONV(LPADC0);

            /* Show the conversion result */
            for(u32ConvCount = 0; u32ConvCount < CONV_TOTAL_COUNT; u32ConvCount++)
            {
                printf("Conversion result of channel pair 1 [#%d]: 0x%X (%d)\n", u32ConvCount+1, i32ConversionData[u32ConvCount], i32ConversionData[u32ConvCount]);
            }

            /* Clear remaining data in FIFO that got before stop LPADC */
            while (LPADC_IS_DATA_VALID(LPADC0, 0))
            {
                i32ConversionData[0] = LPADC_GET_CONVERSION_DATA(LPADC0, 0);
            }
        }
        else
            return;

        printf("\n");
    }
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

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* LPADC function test */
    LPADC_FunctionTest();

    /* Disable LPADC IP clock */
    CLK_DisableModuleClock(LPADC0_MODULE);

    printf("Exit LPADC sample code\n");

    while(1);
}
