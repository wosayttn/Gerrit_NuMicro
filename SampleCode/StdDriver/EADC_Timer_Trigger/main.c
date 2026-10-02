/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger EADC by Timer.
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

    /* Select timer 0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

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

void TIMER0_Init()
{
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_EADC);
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
}

void EADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData[6] = {0};

    printf("\n");
    printf("+-------------------------------------------+\n");
    printf("|           Timer trigger mode test         |\n");
    printf("+-------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 7: channel 14 and 15)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end and enable the A/D converter */
            EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

            /* Configure the sample module 0 for analog input channel 2 and enable Timer0 trigger source */
            EADC_ConfigSampleModule(EADC, 0, EADC_TIMER0_TRIGGER, 2);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 0 interrupt.  */
            EADC_ENABLE_INT(EADC, BIT0);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0);
            NVIC_EnableIRQ(EADC0_INT0_IRQn);

            printf("Conversion result of channel 2:\n");

            /* Reset the ADC indicator and enable Timer0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            TIMER_Start(TIMER0);

            while(1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while(g_u32AdcIntFlag == 0);

                /* Reset the EADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the sample module 0 */
                i32ConversionData[g_u32COVNUMFlag - 1] = EADC_GET_CONV_DATA(EADC, 0);
                printf("    0x%X (%d)\n", i32ConversionData[g_u32COVNUMFlag-1], i32ConversionData[g_u32COVNUMFlag-1]);

                if(g_u32COVNUMFlag > 6)
                    break;
            }

            /* Disable Timer0 counter */
            TIMER_Stop(TIMER0);

            /* Disable the ADINT0 interrupt */
            EADC_DISABLE_INT(EADC, BIT0);
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential and enable the A/D converter */
            EADC_Open(EADC, EADC_CTL_DIFFEN_DIFFERENTIAL);

            /* Configure the sample module 0 for analog input channel 14 and enable Timer0 trigger source */
            EADC_ConfigSampleModule(EADC, 0, EADC_TIMER0_TRIGGER, 14);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 0 interrupt.  */
            EADC_ENABLE_INT(EADC, BIT0);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0);
            NVIC_EnableIRQ(EADC0_INT0_IRQn);

            printf("Conversion result of channel pair 7 (channel 14/15):\n");

            /* Reset the EADC indicator and enable Timer0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            TIMER_Start(TIMER0);

            while(1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while(g_u32AdcIntFlag == 0);

                /* Reset the ADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the sample module 0 */
                i32ConversionData[g_u32COVNUMFlag - 1] = EADC_GET_CONV_DATA(EADC, 0);
                printf("    0x%X (%d)\n", i32ConversionData[g_u32COVNUMFlag-1], i32ConversionData[g_u32COVNUMFlag-1]);

                if(g_u32COVNUMFlag > 6)
                    break;
            }

            /* Disable Timer0 counter */
            TIMER_Stop(TIMER0);

            /* Disable the ADINT0 interrupt */
            EADC_DISABLE_INT(EADC, BIT0);
        }
        else
            return;
    }
}

void EADC0_INT0_IRQHandler(void)
{
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);/* Clear the A/D ADINT0 interrupt flag */
    g_u32AdcIntFlag = 1;
    g_u32COVNUMFlag++;
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init TIMER0 for EADC */
    TIMER0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable Timer0 IP clock */
    CLK_DisableModuleClock(TMR0_MODULE);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while(1);
}
