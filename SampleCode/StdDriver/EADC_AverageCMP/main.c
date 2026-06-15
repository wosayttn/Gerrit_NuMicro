/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to compare average conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;
volatile uint32_t g_u32AdcCmpIntFlag;

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

void EADC03_IRQHandler(void)
{
    if (EADC_GET_INT_FLAG(EADC0, EADC_STATUS2_ADCMPF0_Msk))
    {
        g_u32AdcCmpIntFlag = 1;
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADCMPF0_Msk);/* Clear the A/D compare flag 0 */
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

    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

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
    uint8_t  u8Option;
    int32_t  i32ConversionData, i32Target;
    uint32_t u32ChannelNum;
    uint32_t u32SampleModuleNum = 1; /* Use Sample Module 1 */
    uint32_t u32IntNum = 0;

    uint32_t u32IntMask = (BIT0 << u32IntNum);
    uint32_t u32ModuleMask = (BIT0 << u32SampleModuleNum);

    i32Target = 0x80;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|        EADC average monitored by compare function sample code        |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC0, 0);

    /* To decrease the effect of electrical random noise */
    EADC_Calibration(EADC0);

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

        /* Configure the sample module(u32SampleModuleNum) for analog input channel(u32ChannelNum) and software trigger source */
        EADC_ConfigSampleModule(EADC0, u32SampleModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

        /* Set sample module external sampling time to 10 */
        EADC_SetExtendSampleTime(EADC0, u32SampleModuleNum, 10);

        /* Enable Accumulate feature */
        EADC_ENABLE_ACU(EADC0, u32SampleModuleNum, EADC_MCTL1_ACU_32);

        /* Enable Average feature */
        EADC_ENABLE_AVG(EADC0, u32SampleModuleNum);

        /* Clear the A/D ADINT0 and EADC comparator 0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk | EADC_STATUS2_ADCMPF0_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC0, u32IntMask);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);

        /* Enable EADC comparator 0. Compare condition: conversion result > i32Target; match Count=1 */
        EADC_ENABLE_CMP0(EADC0, u32SampleModuleNum, EADC_CMP_CMPCOND_GREATER_OR_EQUAL, i32Target, 1);
        /* Enable ADC comparator 0 interrupt */
        EADC_ENABLE_CMP_INT(EADC0, 0);

        NVIC_EnableIRQ(EADC00_IRQn);
        NVIC_EnableIRQ(EADC03_IRQn);

        /* Reset the interrupt indicator and trigger sample module to start A/D conversion */
        g_u32AdcIntFlag = 0;
        g_u32AdcCmpIntFlag = 0;
        EADC_START_CONV(EADC0, u32ModuleMask);

        /* Wait interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
        while (g_u32AdcIntFlag == 0);

        /* Disable Average feature */
        EADC_DISABLE_ACU(EADC0, u32SampleModuleNum);

        /* Disable Accumulate feature */
        EADC_DISABLE_AVG(EADC0, u32SampleModuleNum);

        /* Disable ADC comparator interrupt */
        EADC_DISABLE_CMP_INT(EADC0, 0);

        /* Disable compare function */
        EADC_DISABLE_CMP0(EADC0);

        /* Disable the sample module interrupt. */
        EADC_DISABLE_INT(EADC0, u32IntMask);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);
        NVIC_DisableIRQ(EADC00_IRQn);
        NVIC_DisableIRQ(EADC03_IRQn);
        /* Get the conversion result of the sample module */
        i32ConversionData = EADC_GET_CONV_DATA(EADC0, u32SampleModuleNum);

        /* EADC oversampling mode or averaging mode can not be monitored by compare and window compare functions */
        /* Implement software compare to replace hardware compare */
        printf("Conversion result of channel %u: 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);

        if (g_u32AdcCmpIntFlag == 1)
        {
            printf("The conversion result of channel %u is >= 0x%03X\n", u32ChannelNum, i32Target);
        }
        else
        {
            printf("The conversion result of channel %u is < 0x%03X\n", u32ChannelNum, i32Target);
        }
    }   /* end of while(1) */
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

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
