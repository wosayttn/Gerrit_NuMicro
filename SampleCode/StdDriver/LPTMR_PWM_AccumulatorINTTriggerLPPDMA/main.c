/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate Low Power TIMER PWM accumulator interrupt to trigger LPPDMA transfer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void LPPDMA0_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32IsTestOver = 0;
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
uint32_t u32UpdatedPeriod __attribute__((section(".lpSram")));
#else
uint32_t u32UpdatedPeriod __attribute__((section(".ARM.__at_0x28000000")));
#endif

/**
 * @brief       LPPDMA IRQ Handler
 * @param       None
 * @return      None
 * @details     The Low Power PDMA default IRQ, declared in startup_M2L31.s.
 */
void LPPDMA0_IRQHandler(void)
{
    uint32_t u32Status = LPPDMA_GET_INT_STATUS(LPPDMA0);

    if(u32Status & LPPDMA_INTSTS_ABTIF_Msk)        /* abort */
    {
        if(LPPDMA_GET_ABORT_STS(LPPDMA0) & LPPDMA_ABTSTS_ABTIF0_Msk)
            g_u32IsTestOver = 2;
        LPPDMA_CLR_ABORT_FLAG(LPPDMA0, LPPDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(u32Status & LPPDMA_INTSTS_TDIF_Msk)   /* done */
    {
        if(LPPDMA_GET_TD_STS(LPPDMA0) & LPPDMA_TDSTS_TDIF0_Msk)
            g_u32IsTestOver = 1;
        LPPDMA_CLR_TD_FLAG(LPPDMA0, LPPDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Low Power TIMER module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Enable LPPDMA module clock */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Select Low Power TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);

    /* Set Low Power Timer0 PWM CH0(TM0) pin */
    SYS->GPB_MFP1 |= SYS_GPB_MFP1_PB5MFP_LPTM0;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32InitPeriod, u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------------------------------------------+\n");
    printf("|    Low Power Timer PWM Accumulator Inerrupt Trigger LPPDMA Sample Code    |\n");
    printf("+---------------------------------------------------------------------------+\n\n");

    printf("  This sample code demonstrate Low Power Timer0 PWM accumulator interrupt trigger LPPDMA.\n");
    printf("  When accumulator interrupt happens, Low Power Timer0 PWM period will be updated to (Initial Period x 2) by LPPDMA.\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n");
    printf("  Output frequency will be updated from 18kHz to 9kHz, and duty cycle from 50%% to 25%%.\n");

    printf("\n\nPress any key to start Low Power Timer0 PWM.\n\n");
    getchar();

    /* Change Timer to PWM counter mode */
    LPTPWM_ENABLE_PWM_MODE(LPTMR0);

    /* Set Low Power Timer0 PWM mode as independent mode */
    //LPTPWM_ENABLE_INDEPENDENT_MODE(LPTMR0);

    /* Set Low Power Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    LPTPWM_ConfigOutputFreqAndDuty(LPTMR0, 18000, 50);
    u32InitPeriod = LPTPWM_GET_PERIOD(LPTMR0);

    /* Set Timer0 PWM down count type */
    //LPTPWM_SET_COUNTER_TYPE(LPTMR0, LPTPWM_DOWN_COUNT);

    /* Enable output of Low Power Timer0 PWM_CH0 */
    LPTPWM_ENABLE_OUTPUT(LPTMR0, LPTPWM_CH0);

    /* Enable Low Power Timer0 PWM accumulator function, interrupt count 10, accumulator source select to zero point */
    LPTPWM_EnableAcc(LPTMR0, 10, LPTPWM_IFA_PERIOD_POINT);

    /* Enable Low Power Timer0 PWM accumulator interrupt trigger LPPDMA */
    LPTPWM_EnableAccLPPDMA(LPTMR0);

    /* Enable Low Power Timer0 PWM interrupt */
    NVIC_EnableIRQ(TMR0_IRQn);

    /*--------------------------------------------------------------------------------------*/
    /* Configure LPPDMA peripheral mode form memory to Low Power TIMER PWM                              */
    /*--------------------------------------------------------------------------------------*/
    /* Open LPPDMA Channel 0 */
    LPPDMA_Open(LPPDMA0, BIT0);

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    LPPDMA_SetTransferCnt(LPPDMA0, 0, LPPDMA_WIDTH_16, 1);

    /* Set updated period vaule */
    u32UpdatedPeriod = ((u32InitPeriod + 1) * 2) - 1;

    /* Set source address as u32UpdatedPeriod(no increment) and destination address as Low Power Timer0 PWM period register(no increment) */
    LPPDMA_SetTransferAddr(LPPDMA0, 0, (uint32_t)&u32UpdatedPeriod, LPPDMA_SAR_FIX, (uint32_t)&(LPTMR0->PWMPERIOD), LPPDMA_DAR_FIX);

    /* Select LPPDMA request source as LPPDMA_TMR0(Low Power Timer0 PWM accumulator interrupt) */
    LPPDMA_SetTransferMode(LPPDMA0, 0, LPPDMA_LPTMR0, FALSE, 0);

    /* Set LPPDMA as single request type for Low Power Timer0 PWM */
    LPPDMA_SetBurstType(LPPDMA0, 0, LPPDMA_REQ_SINGLE, LPPDMA_BURST_1);

    /* Enable LPPDMA interrupt */
    LPPDMA_EnableInt(LPPDMA0, 0, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA0_IRQn);

    /* Start Low Power Timer0 PWM counter */
    LPTPWM_START_COUNTER(LPTMR0);

    g_u32IsTestOver = 0;

    /* Wait for LPPDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while (g_u32IsTestOver != 1)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for LPPDMA transfer done time-out!\n");
            return -1;
        }
    }

    printf("Low Power Timer0 PWM period register is updated from %d to %d\n\n", u32InitPeriod, LPTPWM_GET_PERIOD(LPTMR0));

    printf("Press any key to stop Low Power Timer0 PWM.\n\n");
    getchar();

    /* Disable LPPDMA function */
    LPPDMA_Close(LPPDMA0);
    NVIC_DisableIRQ(LPPDMA0_IRQn);

    /* Stop Low Power Timer0 PWM */
    LPTPWM_STOP_COUNTER(LPTMR0);

    /* Wait until Low Power Timer0 PWM Stop */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((LPTMR0->PWMCNT & LPTMR_PWMCNT_CNT_Msk) != 0)
        if(--u32TimeOutCnt == 0) break;

    if(u32TimeOutCnt == 0)
        printf("Wait for Low Power Timer PWM stop time-out!\n");
    else
        printf("Low Power Timer0 PWM has STOP.\n");

    while(1) {}
}
