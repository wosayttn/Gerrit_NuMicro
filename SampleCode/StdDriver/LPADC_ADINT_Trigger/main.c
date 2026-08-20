/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use ADINT interrupt to do the LPADC Single-cycle scan conversion.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32LpadcIntFlag;


/*---------------------------------------------------------------------------------------------------------*/
/* LPADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LPADC0_IRQHandler(void)
{
    g_u32LpadcIntFlag = 1;
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT); /* Clear the A/D interrupt flag */
}

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

    /* Set PB.0 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT0|BIT1|BIT2|BIT3, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.3 LPADC analog input pins. */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB0MFP_Msk | SYS_GPB_MFP0_PB1MFP_Msk | SYS_GPB_MFP0_PB2MFP_Msk | SYS_GPB_MFP0_PB3MFP_Msk)) |
                    (SYS_GPB_MFP0_PB0MFP_LPADC0_CH0 | SYS_GPB_MFP0_PB1MFP_LPADC0_CH1 | SYS_GPB_MFP0_PB2MFP_LPADC0_CH2 | SYS_GPB_MFP0_PB3MFP_LPADC0_CH3);
    /* Disable the PB.0 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT1|BIT2|BIT3);

    /* Lock protected registers */
    SYS_LockReg();
}

void LPADC_FunctionTest()
{
    uint8_t  u8Option, u32ChannelCount = 0;
    int32_t  i32ConversionData[8] = {0};

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Enable LPADC converter */
    LPADC_POWER_ON(LPADC0);

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit single-cycle scan mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end, Single-cycle scan mode, and select channel 0~3 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE_CYCLE, BIT0|BIT1|BIT2|BIT3);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

            /* Enable the sample module interrupt */
            LPADC_ENABLE_INT(LPADC0, LPADC_ADF_INT);
            NVIC_EnableIRQ(LPADC0_IRQn);

            /* Reset the LPADC indicator and trigger sample module to start A/D conversion */
            g_u32LpadcIntFlag = 0;
            LPADC_START_CONV(LPADC0);

            __WFI();

            /* Wait conversion done */
            while(g_u32LpadcIntFlag == 0);

            /* Wait conversion data become valid */
            while(LPADC_IS_DATA_VALID(LPADC0, 0) == 0);
            while(LPADC_IS_DATA_VALID(LPADC0, 1) == 0);
            while(LPADC_IS_DATA_VALID(LPADC0, 2) == 0);
            while(LPADC_IS_DATA_VALID(LPADC0, 3) == 0);

            /* Get the conversion result of the sample module */
            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
                i32ConversionData[u32ChannelCount] = LPADC_GET_CONVERSION_DATA(LPADC0, u32ChannelCount);

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData[u32ChannelCount], i32ConversionData[u32ChannelCount]);
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential, Single-cycle scan mode, and select channel 0 and 2 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_DIFFERENTIAL, LPADC_ADCR_ADMD_SINGLE_CYCLE, BIT0|BIT2);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

            /* Enable the sample module interrupt */
            LPADC_ENABLE_INT(LPADC0, LPADC_ADF_INT);
            NVIC_EnableIRQ(LPADC0_IRQn);

            /* Reset the LPADC indicator and trigger sample module to start A/D conversion */
            g_u32LpadcIntFlag = 0;
            LPADC_START_CONV(LPADC0);

            __WFI();

            /* Wait conversion done */
            while(g_u32LpadcIntFlag == 0);

            /* Wait conversion data become valid */
            while(LPADC_IS_DATA_VALID(LPADC0, 0) == 0);
            while(LPADC_IS_DATA_VALID(LPADC0, 2) == 0);

            /* Get the conversion result of the sample module */
            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount += 2)
                i32ConversionData[u32ChannelCount] = LPADC_GET_CONVERSION_DATA(LPADC0, u32ChannelCount);

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount += 2)
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData[u32ChannelCount], i32ConversionData[u32ChannelCount]);
        }
        else
            return ;
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
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* LPADC function test */
    LPADC_FunctionTest();

    /* Disable LPADC IP clock */
    CLK_DisableModuleClock(LPADC0_MODULE);

    /* Disable LPADC Interrupt */
    NVIC_DisableIRQ(LPADC0_IRQn);

    printf("Exit LPADC sample code\n");

    while(1);
}
