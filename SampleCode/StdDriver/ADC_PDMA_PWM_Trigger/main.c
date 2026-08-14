/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger ADC by PWM and transfer conversion data by PDMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*------------------------------------------------------*/
/* Define global variables and constants                */
/*------------------------------------------------------*/
uint32_t g_u32COVNUMFlag = 0;
volatile uint32_t g_u32IsTestOver = 0;
int16_t  g_i32ConversionData[6] = {0};
uint32_t g_u32SampleModuleNum = 0;


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

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB.2 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT2|BIT3, GPIO_MODE_INPUT);
    /* Configure the PB.2 - PB.3 ADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3);
    /* Disable the PB.2 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2|BIT3);

    /* Lock protected registers */
    SYS_LockReg();
}

void PWM0_Init()
{
    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 0);

    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 0, 108);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 0, 216);

    /* PWM period point trigger ADC enable */
    PWM_EnableADCTrigger(PWM0, 0, PWM_TRIGGER_ADC_EVEN_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, BIT0, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, BIT0);
}

void PDMA_Init()
{
    /* Configure PDMA peripheral mode form ADC to memory */
    /* Open Channel 1 */
    PDMA_Open(PDMA, BIT1);

    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA_SetTransferCnt(PDMA, 1, PDMA_WIDTH_16, 6);

    /* Set source address as ADC PDMA Current Transfer Data register (no increment) and destination address as g_i32ConversionData array (increment) */
    PDMA_SetTransferAddr(PDMA, 1, (uint32_t)&ADC->ADPDMA, PDMA_SAR_FIX, (uint32_t)g_i32ConversionData, PDMA_DAR_INC);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, 1, PDMA_ADC0_RX, FALSE, 0);

    /* Set PDMA as single request type for ADC */
    PDMA_SetBurstType(PDMA, 1, PDMA_REQ_SINGLE, PDMA_BURST_4);

    PDMA_EnableInt(PDMA, 1, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
}

void ReloadPDMA()
{
    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA_SetTransferCnt(PDMA, 1, PDMA_WIDTH_16, 6);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, 1, PDMA_ADC0_RX, FALSE, 0);
}

void ADC_FunctionTest()
{
    uint8_t  u8Option;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|   PWM trigger mode and transfer ADC conversion data by PDMA test     |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    while(1)
    {
        /* reload PDMA configuration for next transmission */
        ReloadPDMA();

        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set ADC to Single mode, and select channel 2 */
            ADC_Open(ADC, (uint32_t)NULL, ADC_ADCR_ADMD_SINGLE, BIT2);

            /* Configure the sample module and enable PWM0 trigger source */
            ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_PWM, 0);

            /* ADC enable PDMA transfer */
            ADC_ENABLE_PDMA(ADC);

            printf("Conversion result of channel 2:\n");

            /* Enable PWM0 channel 0 counter */
            PWM_Start(PWM0, PWM_CH_0_MASK);

            while(1)
            {
                /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
                while(g_u32IsTestOver == 0);
                break;
            }
            g_u32IsTestOver = 0;

            /* Disable PWM0 channel 0 counter */
            PWM_ForceStop(PWM0, BIT0);  /* PWM0 counter stop running. */

            for(g_u32COVNUMFlag = 0; g_u32COVNUMFlag < 6; g_u32COVNUMFlag++)
            {
                printf("                                0x%X (%d)\n", g_i32ConversionData[g_u32COVNUMFlag], g_i32ConversionData[g_u32COVNUMFlag]);
            }
        }
        else
            return ;
    }
}

void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF1_Msk)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown PDMA interrupt !!\n");
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

    /* Init PWM for ADC */
    PWM0_Init();

    /* Init PDMA for ADC */
    PDMA_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    /* Disable PWM0 IP clock */
    CLK_DisableModuleClock(PWM0_MODULE);

    /* Disable PDMA clock source */
    CLK_DisableModuleClock(PDMA0_MODULE);

    /* Disable PDMA Interrupt */
    NVIC_DisableIRQ(PDMA0_IRQn);

    printf("Exit ADC sample code\n");

    while(1);
}
