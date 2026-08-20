/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Monitor the conversion result of channel 2 by the digital compare function.
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
volatile uint32_t g_u32LpadcCmp0IntFlag;
volatile uint32_t g_u32LpadcCmp1IntFlag;


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

    /* Set PB.2 to input mode */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);

    /* Configure the PB.2 LPADC analog input pins.  */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB2MFP_Msk)) |
                    (SYS_GPB_MFP0_PB2MFP_LPADC0_CH2);

    /* Disable the PB.2 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);

    /* Lock protected registers */
    SYS_LockReg();
}

void LPADC_FunctionTest()
{
    uint32_t u32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|          LPADC compare function (result monitor) sample code         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 2.\n");

    /* Enable LPADC converter */
    LPADC_POWER_ON(LPADC0);

    /* Set input mode as single-end, Single mode, and select channel 2 */
    LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT2);

    /* Enable LPADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 0: channel 2 is less than 0x800; match count is 5.\n");
    LPADC_ENABLE_CMP0(LPADC0, 2, LPADC_ADCMPR_CMPCOND_LESS_THAN, 0x800, 5);

    /* Enable LPADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 1: channel 2 is greater than or equal to 0x800; match count is 5.\n");
    LPADC_ENABLE_CMP1(LPADC0, 2, LPADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 5);

    /* Clear the A/D interrupt flag for safe */
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

    /* Enable the sample module interrupt */
    LPADC_ENABLE_INT(LPADC0, LPADC_ADF_INT);
    NVIC_EnableIRQ(LPADC0_IRQn);

    /* Clear the LPADC comparator 0 interrupt flag for safe */
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_CMP0_INT);
    /* Enable LPADC comparator 0 interrupt */
    LPADC_ENABLE_INT(LPADC0, LPADC_CMP0_INT);

    /* Clear the LPADC comparator 1 interrupt flag for safe */
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_CMP1_INT);
    /* Enable LPADC comparator 1 interrupt */
    LPADC_ENABLE_INT(LPADC0, LPADC_CMP1_INT);

    /* Reset the LPADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32LpadcIntFlag = 0;
    g_u32LpadcCmp0IntFlag = 0;
    g_u32LpadcCmp1IntFlag = 0;
    LPADC_START_CONV(LPADC0);

    /* Wait LPADC compare interrupt */
    while(1)
    {
        if (g_u32LpadcIntFlag == 1)
        {
            u32ConversionData = LPADC_GET_CONVERSION_DATA(LPADC0, 2);
            printf("Conversion result of channel 2: 0x%03X (%d)\n", u32ConversionData, u32ConversionData);

            if ((g_u32LpadcCmp0IntFlag == 1) || (g_u32LpadcCmp1IntFlag == 1))
                break;

            g_u32LpadcIntFlag = 0;
            LPADC_START_CONV(LPADC0);
        }
    }

    /* Disable LPADC comparator interrupt */
    LPADC_DISABLE_INT(LPADC0, LPADC_CMP0_INT);
    LPADC_DISABLE_INT(LPADC0, LPADC_CMP1_INT);
    /* Disable compare function */
    LPADC_DISABLE_CMP0(LPADC0);
    LPADC_DISABLE_CMP1(LPADC0);

    if(g_u32LpadcCmp0IntFlag == 1)
    {
        printf("Comparator 0 interrupt occurs.\nThe conversion result of channel 2 is less than 0x800\n");
    }
    else
    {
        printf("Comparator 1 interrupt occurs.\nThe conversion result of channel 2 is greater than or equal to 0x800\n");
    }
}

void LPADC0_IRQHandler(void)
{
    if(LPADC_GET_INT_FLAG(LPADC0, LPADC_CMP0_INT))
    {
        g_u32LpadcCmp0IntFlag = 1;
        LPADC_CLR_INT_FLAG(LPADC0, LPADC_CMP0_INT);    /* Clear the A/D compare flag 0 */
    }

    if(LPADC_GET_INT_FLAG(LPADC0, LPADC_CMP1_INT))
    {
        g_u32LpadcCmp1IntFlag = 1;
        LPADC_CLR_INT_FLAG(LPADC0, LPADC_CMP1_INT);    /* Clear the A/D compare flag 1 */
    }

    g_u32LpadcIntFlag = 1;
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);
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

    /* Disable External Interrupt */
    NVIC_DisableIRQ(LPADC0_IRQn);

    printf("Exit LPADC sample code\n");

    while(1);
}
