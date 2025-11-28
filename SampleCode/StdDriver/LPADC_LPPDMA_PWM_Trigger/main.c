/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger LPADC by PWM and transfer conversion data by LPPDMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32IsTestOver = 0;

/* M2L31: Because LPPDMA only can access LPSRAM,
   the g_i32ConversionData[] MUST be allocated at LPSRAM area 0x28000000 ~ 0x28001FFF (8 KB).
 */
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
volatile uint32_t g_i32ConversionData[6] __attribute__((section(".lpSram")));
#else
volatile uint32_t g_i32ConversionData[6] __attribute__((section(".ARM.__at_0x28000000")));
#endif

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

    /* Select PWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0SEL_PCLK0, 0);

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Enable LPPDMA clock source */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Enable LPSRAM clock source */
    /* LPPDMA only can access LPSRAM and cannot access normal SRAM. */
    CLK_EnableModuleClock(LPSRAM_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB.2 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT2|BIT3, GPIO_MODE_INPUT);

    /* Configure the PB.2 - PB.3 LPADC analog input pins.  */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB2MFP_Msk | SYS_GPB_MFP0_PB3MFP_Msk)) |
                    (SYS_GPB_MFP0_PB2MFP_LPADC0_CH2 | SYS_GPB_MFP0_PB3MFP_LPADC0_CH3);

    /* Disable the PB.2 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2|BIT3);

    /* Lock protected registers */
    SYS_LockReg();
}

void PWM0_Init()
{
    uint32_t pwm_ch = 0;

    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, pwm_ch, 0);

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, pwm_ch, 108);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, pwm_ch, 216);

    /* PWM period point trigger LPADC enable */
    PWM_EnableADCTrigger(PWM0, pwm_ch, PWM_TRIGGER_ADC_EVEN_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, BIT0 << pwm_ch, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, BIT0 << pwm_ch);
}

void ReloadLPPDMA()
{
    /* transfer width is half word(16 bit) and transfer count is 6 */
    LPPDMA_SetTransferCnt(LPPDMA0, 1, LPPDMA_WIDTH_32, 6);

    /* Select PDMA request source as LPADC RX */
    LPPDMA_SetTransferMode(LPPDMA0, 1, LPPDMA_LPADC0_RX, FALSE, 0);
}

void LPPDMA_Init()
{
    /* Configure LPPDMA peripheral mode form LPADC to memory */
    /* Open LPPDMA Channel 1 */
    LPPDMA_Open(LPPDMA0, BIT1);

    ReloadLPPDMA();

    /* Set source address as LPADC LPPDMA Current Transfer Data register (no increment) and destination address as g_i32ConversionData array (increment) */
    LPPDMA_SetTransferAddr(LPPDMA0, 1, (uint32_t)&(LPADC0->ADPDMA), LPPDMA_SAR_FIX, (uint32_t)g_i32ConversionData, LPPDMA_DAR_INC);

    LPPDMA_SetBurstType(LPPDMA0, 1, LPPDMA_REQ_SINGLE, LPPDMA_BURST_1);

    LPPDMA_CLR_TD_FLAG(LPPDMA0, LPPDMA_TDSTS_TDIF1_Msk);
    LPPDMA_EnableInt(LPPDMA0, 1, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA0_IRQn);
}

void LPADC_FunctionTest()
{
    uint8_t  u8Option;
    uint32_t i;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|   PWM trigger mode and transfer LPADC conversion data by LPPDMA test |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    /* Enable LPADC converter */
    LPADC_POWER_ON(LPADC0);

    while(1)
    {
        /* reload LPPDMA configuration for next transmission */
        ReloadLPPDMA();

        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only (channel 2 and 3))\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end, Single mode, and select channel 2 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT2);

            /* Configure the sample module and enable PWM0 trigger source */
            LPADC_EnableHWTrigger(LPADC0, LPADC_ADCR_TRGS_PWM, 0);

            /* LPADC enable LPPDMA transfer */
            LPADC_ENABLE_LPPDMA(LPADC0);

            printf("Conversion result of channel 2:\n");

            /* Enable PWM0 channel 0 counter */
            PWM_Start(PWM0, PWM_CH_0_MASK);

            /* Wait LPPDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
            while(g_u32IsTestOver == 0);

            g_u32IsTestOver = 0;

            /* Disable PWM0 channel 0 counter */
            PWM_ForceStop(PWM0, BIT0);  /* PWM0 counter stop running. */

            for(i = 0; i < 6; i++)
                printf("                                0x%X (%d)\n", g_i32ConversionData[i], g_i32ConversionData[i] & 0xFFF);
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential, Single mode, and select channel 2 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_DIFFERENTIAL, LPADC_ADCR_ADMD_SINGLE, BIT2);

            /* Configure the sample module and enable PWM0 trigger source */
            LPADC_EnableHWTrigger(LPADC0, LPADC_ADCR_TRGS_PWM, 0);

            /* LPADC enable LPPDMA transfer */
            LPADC_ENABLE_LPPDMA(LPADC0);

            printf("Conversion result of channel 2:\n");

            /* Enable PWM0 channel 0 counter */
            PWM_Start(PWM0, PWM_CH_0_MASK);

            /* Wait LPPDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
            while(g_u32IsTestOver == 0);

            g_u32IsTestOver = 0;

            /* Disable EPWM0 channel 0 counter */
            PWM_ForceStop(PWM0, BIT0);  /* PWM0 counter stop running. */

            for(i = 0; (i) < 6; i++)
                printf("                                0x%X (%d)\n", g_i32ConversionData[i], g_i32ConversionData[i] & 0xFFF);
        }
        else
            return ;
    }
}

void LPPDMA0_IRQHandler(void)
{
    uint32_t status = LPPDMA_GET_INT_STATUS(LPPDMA0);

    if(status & LPPDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(LPPDMA_GET_ABORT_STS(LPPDMA0) & LPPDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;
        LPPDMA_CLR_ABORT_FLAG(LPPDMA0, LPPDMA_ABTSTS_ABTIF1_Msk);
    }
    else if(status & LPPDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(LPPDMA_GET_TD_STS(LPPDMA0) & LPPDMA_TDSTS_TDIF1_Msk)
            g_u32IsTestOver = 1;
        LPPDMA_CLR_TD_FLAG(LPPDMA0, LPPDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown LPPDMA interrupt !!\n");
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

    /* Init PWM for LPADC */
    PWM0_Init();

    /* Init LPPDMA for LPADC */
    LPPDMA_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* LPADC function test */
    LPADC_FunctionTest();

    /* Disable LPADC IP clock */
    CLK_DisableModuleClock(LPADC0_MODULE);

    /* Disable PWM0 IP clock */
    CLK_DisableModuleClock(PWM0_MODULE);

    /* Disable LPPDMA clock source */
    CLK_DisableModuleClock(LPPDMA0_MODULE);

    /* Disable LPPDMA Interrupt */
    NVIC_DisableIRQ(LPPDMA0_IRQn);

    printf("Exit LPADC sample code\n");

    while(1);
}
