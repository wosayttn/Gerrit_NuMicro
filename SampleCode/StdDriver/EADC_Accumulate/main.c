/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to get accumulate conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void EADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Enable APLL1 200MHz clock for maximum EADC clock frequency */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, FREQ_200MHZ, CLK_APLL1_SELECT);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select EADC peripheral clock source(maximum clock frequency = divider 1) */
    CLK_SetModuleClock(EADC0_MODULE, CLK_EADCSEL_EADC0SEL_APLL1_DIV2, CLK_EADCDIV_EADC0DIV(10));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

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


/*---------------------------------------------------------------------------------------------------------*/
/* @brief Left shift Accumulate raw data back to correct conversion result                                 */
/* @param[in] accu_raw_result  Raw data of Accumulate conversion                                           */
/* @param[in] accu_count       Conversion count of Accumulate conversion.                                  */
/*                             Valid values are 1, 2, 4, 8, 16, 32, 64, 128, 256.                          */
/* @return The correct conversion result of Accumulate conversion                                          */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t EADC_Accumulate_LeftShift(uint32_t u32AccuRawResult, uint32_t u32AccuCount)
{
    switch (u32AccuCount)
    {
        case 1:
            return u32AccuRawResult;

        case 2:
            return u32AccuRawResult;

        case 4:
            return u32AccuRawResult;

        case 8:
            return u32AccuRawResult;

        case 16:
            return u32AccuRawResult;

        case 32:
            return (u32AccuRawResult << 1);

        case 64:
            return (u32AccuRawResult << 2);

        case 128:
            return (u32AccuRawResult << 3);

        case 256:
            return (u32AccuRawResult << 4);

        default:
            printf("*** Error! Wrong parameter %u for Accumulate.\n\n", u32AccuCount);
            return u32AccuRawResult;
    }
}

void EADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData;

    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;

    u32IntNum = 0;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 1;   /* Use Sample Module 1 */

    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);

    printf("\n");
    printf("+---------------------------------------------------+\n");
    printf("|            EADC Accumulate sample code            |\n");
    printf("+---------------------------------------------------+\n");

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

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

        /* Configure the sample module for analog input channel and software trigger source. */
        EADC_ConfigSampleModule(EADC0, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

        /* Set sample module external sampling time to 0 */
        EADC_SetExtendSampleTime(EADC0, u32ModuleNum, 0);

        /* Enable Accumulate feature */
        EADC_ENABLE_ACU(EADC0, u32ModuleNum, EADC_MCTL1_ACU_32);

        /* Disable Average feature */
        EADC_DISABLE_AVG(EADC0, u32ModuleNum);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC0, u32IntMask);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);
        NVIC_EnableIRQ(EADC00_IRQn);

        /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
        g_u32AdcIntFlag = 0;
        EADC_START_CONV(EADC0, u32ModuleMask);

        /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
        while (g_u32AdcIntFlag == 0);

        /* Get the conversion result of the sample module */
        i32ConversionData = EADC_GET_CONV_DATA(EADC0, u32ModuleNum);
        printf("Conversion result of channel %u accumulated 32 times : 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);

        i32ConversionData = EADC_Accumulate_LeftShift(i32ConversionData, 32);
        printf("Conversion result of channel %u after left shifted   : 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);

        printf("The average that calculated by software is            0x%X (%d)\n\n", (int)(i32ConversionData / 32), (int)(i32ConversionData / 32));

        /* Disable Accumulate feature */
        EADC_DISABLE_ACU(EADC0, u32ModuleNum);

        /* Disable the sample module interrupt. */
        EADC_DISABLE_INT(EADC0, u32IntMask);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);
        NVIC_DisableIRQ(EADC00_IRQn);
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

    /* Init Debug UART for printf */
    InitDebugUart();

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

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
