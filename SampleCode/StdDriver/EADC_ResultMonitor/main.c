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

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcCmp0IntFlag;
volatile uint32_t g_u32AdcCmp1IntFlag;


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
    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|   EADC compare function (result monitor) sample code  |\n");
    printf("+-------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 2.\n");

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Configure the sample module 0 for analog input channel 2 and ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 0, EADC_ADINT0_TRIGGER, 2);

    /* Enable EADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 0 : channel 2 is less than 0x800; match count is 5.\n");
    EADC_ENABLE_CMP0(EADC, 0, EADC_CMP_CMPCOND_LESS_THAN, 0x800, 5);

    /* Enable EADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 1 : channel 2 is greater than or equal to 0x800; match count is 5.\n");
    EADC_ENABLE_CMP1(EADC, 0, EADC_CMP_CMPCOND_GREATER_OR_EQUAL, 0x800, 5);

    /* Enable sample module 0 for ADINT0 */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0);
    /* Enable ADINT0 interrupt */
    EADC_ENABLE_INT(EADC, BIT0);

    /* Clear the A/D ADINT3 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF3_Msk);
    /* Enable sample module 0 for ADINT3 */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 3, BIT0);
    /* Enable ADINT3 interrupt */
    EADC_ENABLE_INT(EADC, BIT3);
    NVIC_EnableIRQ(EADC0_INT3_IRQn);

    /* Clear the EADC comparator 0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADCMPF0_Msk);
    /* Enable ADC comparator 0 interrupt */
    EADC_ENABLE_CMP_INT(EADC, 0);

    /* Clear the EADC comparator 1 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADCMPF1_Msk);
    /* Enable ADC comparator 1 interrupt */
    EADC_ENABLE_CMP_INT(EADC, 1);

    /* Reset the EADC interrupt indicator and trigger sample module 0 to start A/D conversion */
    g_u32AdcCmp0IntFlag = 0;
    g_u32AdcCmp1IntFlag = 0;
    EADC_START_CONV(EADC, BIT0);

    /* Wait EADC compare interrupt */
    while((g_u32AdcCmp0IntFlag == 0) && (g_u32AdcCmp1IntFlag == 0));

    /* Disable the sample module 0 interrupt */
    EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0);

    /* Disable ADC comparator interrupt */
    EADC_DISABLE_CMP_INT(EADC, 0);
    EADC_DISABLE_CMP_INT(EADC, 1);
    /* Disable compare function */
    EADC_DISABLE_CMP0(EADC);
    EADC_DISABLE_CMP1(EADC);

    if(g_u32AdcCmp0IntFlag == 1)
    {
        printf("Comparator 0 interrupt occurs.\nThe conversion result of channel 2 is less than 0x800\n");
    }
    else
    {
        printf("Comparator 1 interrupt occurs.\nThe conversion result of channel 2 is greater than or equal to 0x800\n");
    }
}

void EADC0_INT3_IRQHandler(void)
{
    if(EADC_GET_INT_FLAG(EADC, EADC_STATUS2_ADCMPF0_Msk))
    {
        g_u32AdcCmp0IntFlag = 1;
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADCMPF0_Msk);/* Clear the A/D compare flag 0 */
    }

    if(EADC_GET_INT_FLAG(EADC, EADC_STATUS2_ADCMPF1_Msk))
    {
        g_u32AdcCmp1IntFlag = 1;
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADCMPF1_Msk);/* Clear the A/D compare flag 1 */
    }
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF3_Msk);
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
    NVIC_DisableIRQ(EADC0_INT3_IRQn);

    printf("Exit EADC sample code\n");

    while(1);
}
