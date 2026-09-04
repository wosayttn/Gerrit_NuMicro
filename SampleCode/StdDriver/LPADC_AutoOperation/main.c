/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to enable LPADC Auto-operation mode
 *           to convert when chip enters power-down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*--------------------------------------------------------------------------*/
/* Define global variables and constants                                    */
/*--------------------------------------------------------------------------*/
volatile uint32_t g_u32LPPdmaIntFlag = 0;
volatile uint32_t g_u32WakeupCount = 0;

/* M2L31: Because LPPDMA only can access LPSRAM,
   the g_i32ConversionData[] MUST be allocated at LPSRAM area 0x28000000 ~ 0x28001FFF (8 KB).
 */
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
volatile uint32_t g_i32ConversionData[10] __attribute__((section(".lpSram")));
#else
volatile uint32_t g_i32ConversionData[10] __attribute__((section(".ARM.__at_0x28000000")));
#endif

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

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

    /* LPADC clock source is PCLK2 = 6MHz, set divider to 2, LPADC clock is 6/2 MHz */
    CLK_SetModuleClock(LPADC0_MODULE, LPSCC_CLKSEL0_LPADC0SEL_PCLK2, LPSCC_CLKDIV0_LPADC0(2));

    /* Enable LPADC module clock */
    CLK_EnableModuleClock(LPADC0_MODULE);

    /* Select LPTMR0 module clock source as LIRC */
    CLK_SetModuleClock(LPTMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_LIRC, 0);

    /* Enable LPTMR0 module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

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

    /* Set PB.2 to input mode */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    /* Configure the PB.2 LPADC analog input pins.  */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB2MFP_Msk)) |
                    (SYS_GPB_MFP0_PB2MFP_LPADC0_CH2);
    /* Disable the PB.2 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);

    /* Set LPTMR0 PWM mode output pins */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB5MFP_Msk)) |
                    (SYS_GPB_MFP1_PB5MFP_LPTM0);

    /* Clock output HCLK to PB14 */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(SYS_GPB_MFP3_PB14MFP_Msk)) |
                    (SYS_GPB_MFP3_PB14MFP_CLKO);
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 0, 1);

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

void LPTPWM_Init(void)
{
    uint32_t freq;

    LPTPWM_ENABLE_PWM_MODE(LPTMR0);
    LPTPWM_ENABLE_OUTPUT(LPTMR0, LPTPWM_CH0);

    /* Set LPTMR0 PWM output frequency is 1 Hz, duty 50% in up count type */
    freq = LPTPWM_ConfigOutputFreqAndDuty(LPTMR0, 1, 50);
    if (freq != 1)
    {
        printf("LPTPWM: Set the frequency %d different from the user !\n", freq);
    }

    /* LPTMR0 PWM mode trigger all Low Power IPs except LPPDMA */
    LPTPWM_EnableTrigger(LPTMR0, LPTMR_PWMTRGCTL_TRGEN_Msk, LPTPWM_TRIGGER_AT_PERIOD_POINT);

    /* Enable LPTMR clock in power-down mode */
    LPTMR_EnableWakeup(LPTMR0);
}

void LPPDMA0_IRQHandler(void)
{
    uint32_t status = LPPDMA_GET_INT_STATUS(LPPDMA0);

    if(status & LPPDMA_INTSTS_WKF_Msk)          /* wakeup */
    {
        LPPDMA0->INTSTS = LPPDMA_INTSTS_WKF_Msk;
    }

    if(status & LPPDMA_INTSTS_ABTIF_Msk)        /* abort */
    {
        if(LPPDMA_GET_ABORT_STS(LPPDMA0) & LPPDMA_ABTSTS_ABTIF1_Msk)
            g_u32LPPdmaIntFlag = 2;
        LPPDMA_CLR_ABORT_FLAG(LPPDMA0, LPPDMA_ABTSTS_ABTIF1_Msk);
    }
    else if(status & LPPDMA_INTSTS_TDIF_Msk)    /* done */
    {
        if(LPPDMA_GET_TD_STS(LPPDMA0) & LPPDMA_TDSTS_TDIF1_Msk)
            g_u32LPPdmaIntFlag = 1;
        LPPDMA_CLR_TD_FLAG(LPPDMA0, LPPDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("LPPDMA0_IRQHandler: unknown LPPDMA interrupt !!\n");
}

void LPPDMA_Init()
{
    uint32_t lppdma_ch = 1;

    /* Configure LPPDMA peripheral mode form LPADC to LPSRAM */
    /* Open Channel 1 */
    LPPDMA_Open(LPPDMA0, BIT0 << lppdma_ch);

    /* Transfer count is 5, transfer width is 32 bits */
    LPPDMA_SetTransferCnt(LPPDMA0, lppdma_ch, LPPDMA_WIDTH_32, 5);

    /* Set source address is LPADC0->ADPDMA, destination address is g_i32ConversionData[] on LPSRAM */
    LPPDMA_SetTransferAddr(LPPDMA0, lppdma_ch, (uint32_t)&(LPADC0->ADPDMA), LPPDMA_SAR_FIX,
                           (uint32_t)g_i32ConversionData, LPPDMA_DAR_INC);
    /* Request source is LPADC */
    LPPDMA_SetTransferMode(LPPDMA0, lppdma_ch, LPPDMA_LPADC0_RX, FALSE, 0);

    /* Transfer type is burst transfer and burst size is 1 */
    LPPDMA_SetBurstType(LPPDMA0, lppdma_ch, LPPDMA_REQ_SINGLE, LPPDMA_BURST_1);

    /* Enable interrupt */
    LPPDMA_CLR_TD_FLAG(LPPDMA0, LPPDMA_TDSTS_TDIF0_Msk << lppdma_ch);
    LPPDMA_EnableInt(LPPDMA0, lppdma_ch, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA0_IRQn);
}

void LPADC_Init(void)
{
    /* Enable LPADC converter */
    LPADC_POWER_ON(LPADC0);

    /* Set input mode as single-end, Single-cycle scan mode, and select channel 2 */
    LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT2);

    /* Enable LPADC Auto-operation and set it can be triggered by LPTMR0 */
    LPADC_SelectAutoOperationMode(LPADC0, LPADC_AUTOCTL_TRIGSEL_LPTMR0);

    /* Set LPADC to trigger LPPDMA */
    LPADC_ENABLE_LPPDMA(LPADC0);
}

void AutoOperation_FunctionTest()
{
    uint32_t i;

    g_u32WakeupCount = 0;

    /* Start LPTMR counting */
    LPTPWM_START_COUNTER(LPTMR0);

    while(1)
    {
        LPPDMA_Init();

        printf("Power down and wait LPPDMA wake up CPU ...\n\n");
        UART_WAIT_TX_EMPTY(UART0);

        /* Clear wake-up status flag */
        CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        g_u32LPPdmaIntFlag = 0;

        SYS_UnlockReg();
        CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_NPD2);
        CLK_PowerDown();
        SYS_LockReg();

        printf("Wakeup %d times !!\n", ++g_u32WakeupCount);

        /* Waiting for LPPDMA transfer done. g_u32LPPdmaIntFlag is set by LPPDMA interrupt handler */
        while (g_u32LPPdmaIntFlag == 0);

        /* Check transfer result */
        if (g_u32LPPdmaIntFlag == 1)
            printf("LPPDMA trasnfer LPADC register done.\n");
        else if (g_u32LPPdmaIntFlag == 2)
            printf("LPPDMA trasnfer LPADC register abort...\n");

        printf("Only the first 5 values are valid LPADC conversion results.\n");
        for(i=0; i<10; i++)
        {
            printf("    [%2d]: %08X.\n", i, g_i32ConversionData[i]);
        }
    }   /* end of while(1) */
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------+\n");
    printf("|   LPADC Auto-operation mode sample code   |\n");
    printf("+-------------------------------------------+\n");
    printf("    * Initial LPTMR0, LPPDMA, and LPADC to Auto-operation mode.\n");
    printf("    * CPU enter power-down mode.\n");
    printf("    * LPTMR0 PWM mode output 1 Hz 50%% duty to LPTM0 (PB5).\n");
    printf("          and trigger LPADC0 channel 2 (PB2) at period point.\n");
    printf("    * LPADC0 trigger LPPDMA to move data from LPADC0 to LPSRAM.\n");
    printf("    * LPPDMA wakeup CPU after 5 data be moved.\n");
    printf("    * CPU can access LPSRAM to get 5 LPADC conversion result.\n");
    printf("    *** Check the HCLK output on PB14 can monitor power-down period.\n");

    LPTPWM_Init();

    LPADC_Init();

    LPPDMA_Init();

    AutoOperation_FunctionTest();

    printf("Exit LPADC Auto-operation sample code\n");

    while(1);
}
