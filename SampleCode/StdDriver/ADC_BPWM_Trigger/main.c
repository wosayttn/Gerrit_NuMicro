/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger ADC by BPWM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*------------------------------------------------------*/
/* Define global variables and constants                */
/*------------------------------------------------------*/
#define DATA_NUMBER     (6)
volatile uint32_t g_u32AdcIntFlag;


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* ADC clock source is 16 MHz from HIRC, set divider to 1, ADC clock is 16/1 MHz */
    CLK_SetModuleClock(ADC0_MODULE, CLK_CLKSEL1_ADC0SEL_HIRC, CLK_CLKDIV_ADC0(1));

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC0_MODULE);

    /* Enable BPWM0 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /*----------------------------------------------*/
    /* Init I/O Multi-function                      */
    /*----------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB.2 to input mode */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    /* Configure the PB.2 to ADC analog input pins. */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk)) |
                    (SYS_GPB_MFPL_PB2MFP_ADC0_CH2);
    /* Disable the PB.2 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);

    /* Set PA multi-function pins for BPWM0 Channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA0MFP_BPWM0_CH0);

    /* Lock protected registers */
    SYS_LockReg();
}

void BPWM0_Init()
{
    /* Set BPWM0 timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 10);

    /* Set up counter type */
    BPWM0->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;

    /* Set BPWM0 timer duty */
    BPWM_SET_CMR(BPWM0, 0, 1000);

    /* Set BPWM0 timer period */
    BPWM_SET_CNR(BPWM0, 0, 2000);

    /* BPWM period point trigger ADC enable */
    BPWM_EnableADCTrigger(BPWM0, 0, BPWM_TRIGGER_ADC_EVEN_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, BIT0, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);

    /* Enable output of BPWM0 channel 0 */
    BPWM_EnableOutput(BPWM0, BIT0);
}

void ADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData[DATA_NUMBER] = {0};
    uint32_t u32CovNum, i;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADC trigger by BPWM test                        |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get %d conversion result from the specified channel.\n", DATA_NUMBER);

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set ADC to Single mode, and select channel 2 */
            ADC_Open(ADC, (uint32_t)NULL, ADC_ADCR_ADMD_SINGLE, BIT2);

            /* Configure the sample module and enable BPWM0 trigger source */
            ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_BPWM, 0);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Enable the sample module interrupt */
            ADC_ENABLE_INT(ADC, ADC_ADF_INT);  /* Enable sample module A/D interrupt. */
            NVIC_EnableIRQ(ADC0_INT0_IRQn);

            printf("Conversion result of channel 2:\n");

            /* Reset the ADC indicator and enable BPWM0 channel 0 counter */
            g_u32AdcIntFlag = 0;
            u32CovNum = 0;
            BPWM_Start(BPWM0, BPWM_CH_0_MASK); /* BPWM0 channel 0 counter start running. */

            while(1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while(g_u32AdcIntFlag == 0);

                /* Reset the ADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of ADC channel 2 */
                i32ConversionData[u32CovNum] = ADC_GET_CONVERSION_DATA(ADC, 2);
                u32CovNum++;

                if(u32CovNum >= DATA_NUMBER)
                    break;
            }

            /* Disable BPWM0 channel 0 counter */
            BPWM_ForceStop(BPWM0, BIT0);  /* BPWM0 counter stop running. */

            for(i = 0; i < DATA_NUMBER; i++)
                printf("                                0x%X (%d)\n", i32ConversionData[i], i32ConversionData[i]);
        }
        else
            return ;
    }
}

void ADC0_INT0_IRQHandler(void)
{
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
    g_u32AdcIntFlag = 1;
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

    /* Init BPWM for ADC */
    BPWM0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    /* Disable BPWM0 IP clock */
    CLK_DisableModuleClock(BPWM0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC0_INT0_IRQn);

    printf("Exit ADC sample code\n");

    while(1);
}
