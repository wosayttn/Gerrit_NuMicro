/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to trigger EADC by BPWM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
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
    uint32_t u32TimeOutCnt = 1000;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
    g_u32AdcIntFlag = 1;
    g_u32COVNUMFlag++;

    /*Confirm that the Flag has been cleared.*/
    while (EADC_GET_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk) != 0)
    {
        if ((--u32TimeOutCnt) == 0)
            break;
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

    /* Enable GPIOB module clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Select BPWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL2_BPWM0SEL_PCLK0, 0);

    /* Enable BPWM0 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

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

    /* Set PA multi-function pins for BPWM Channel0 */
    SET_BPWM0_CH0_PA0();
}

void BPWM0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init BPWM0                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set BPWM0 timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 1);

    /* Center-aligned type */
    BPWM_SET_ALIGNED_TYPE(BPWM0, BPWM_CH_0_MASK, BPWM_CENTER_ALIGNED);

    /* Set BPWM0 timer duty */
    BPWM_SET_CMR(BPWM0, 0, 108);

    /* Set BPWM0 timer period */
    BPWM_SET_CNR(BPWM0, 0, 216);

    /* BPWM period point trigger ADC enable */
    BPWM_EnableADCTrigger(BPWM0, 0, BPWM_TRIGGER_ADC_EVEN_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, 0x1, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);

    /* Enable output of BPWM0 channel 0 */
    BPWM_EnableOutput(BPWM0, 0x1);
}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    int32_t  ai32ConversionData[6] = {0};
    int32_t  i32Temp;
    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;

    u32IntNum = 0;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 1;   /* Use Sample Module 1 */

    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      EADC BPWM trigger sample code                   |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

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

        uint8_t u8Option;
        u8Option = getchar();

        if (u8Option == '1')
            u32ChannelNum = 0;
        else if (u8Option == '2')
            u32ChannelNum = 1;
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and enable BPWM0 trigger source */
        EADC_ConfigSampleModule(EADC0, u32ModuleNum, EADC_BPWM0TG_TRIGGER, u32ChannelNum);

        /* Set sample module external sampling time to 10 */
        EADC_SetExtendSampleTime(EADC0, u32ModuleNum, 10);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC0, u32IntMask);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);
        NVIC_EnableIRQ(EADC00_IRQn);

        /* Reset the EADC indicator and enable BPWM0 channel 0 counter */
        g_u32AdcIntFlag = 0;
        g_u32COVNUMFlag = 0;
        BPWM_Start(BPWM0, BIT0);

        while (1)
        {
            /* Wait EADC interrupt (g_u32AdcIntFlag will be set at EADC_INT0_IRQHandler() function) */
            while (g_u32AdcIntFlag == 0);

            /* Reset the EADC interrupt indicator */
            g_u32AdcIntFlag = 0;

            /* Get the conversion result of the sample module 0 */
            i32Temp = EADC_GET_CONV_DATA(EADC0, u32ModuleNum);
            ai32ConversionData[g_u32COVNUMFlag - 1] = i32Temp;

            if (g_u32COVNUMFlag >= 6)
                break;
        }

        /* Disable BPWM0 channel 0 counter */
        BPWM_ForceStop(BPWM0, BIT0);  /* BPWM0 counter stop running. */

        printf("Conversion result of channel %u:\n", u32ChannelNum);

        for (g_u32COVNUMFlag = 0; g_u32COVNUMFlag < 6; g_u32COVNUMFlag++)
        {
            i32Temp = ai32ConversionData[g_u32COVNUMFlag];
            printf("                                0x%X (%d)\n", i32Temp, i32Temp);
        }

        printf("\n");

        /* Disable the ADINTx interrupt */
        EADC_DISABLE_INT(EADC0, u32IntMask);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);
        NVIC_DisableIRQ(EADC00_IRQn);
    }   /* End of while(1) */

    /* Disable the A/D converter */
    EADC_Close(EADC0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Init BPWM0 for EADC */
    BPWM0_Init();

    printf("\nSystem clock rate: %u Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable BPWM0 IP clock */
    CLK_DisableModuleClock(BPWM0_MODULE);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}
