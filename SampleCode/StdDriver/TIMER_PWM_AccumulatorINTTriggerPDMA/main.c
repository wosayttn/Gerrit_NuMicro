/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate TIMER PWM accumulator interrupt to trigger PDMA transfer.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define PDMA_CH       0UL

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32IsTestOver = 0;
__attribute__((aligned(4))) static uint32_t s_u32UpdatedPeriod;

void PDMA0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)       /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA0) & (PDMA_ABTSTS_ABTIF0_Msk << PDMA_CH))
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA0, (PDMA_ABTSTS_ABTIF0_Msk << PDMA_CH));
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)  /* done */
    {
        if (PDMA_GET_TD_STS(PDMA0) & (PDMA_TDSTS_TDIF0_Msk << PDMA_CH))
            g_u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA0, (PDMA_TDSTS_TDIF0_Msk << PDMA_CH));
    }
    else
        printf("unknown interrupt %x !!\n", u32Status);

    while (PDMA_GET_INT_STATUS(PDMA0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA0 IntFlag time-out!\n");
            break;
        }
    }
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();
    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPB_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set Timer0 PWM CH0(TM0) pin */
    SET_TM0_PB5();
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32InitPeriod, u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("+---------------------------------------------------------------+\n");
    printf("|    Timer PWM Accumulator Inerrupt Trigger PDMA Sample Code    |\n");
    printf("+---------------------------------------------------------------+\n\n");

    printf("  This sample code demonstrate Timer0 PWM accumulator interrupt trigger PDMA.\n");
    printf("  When accumulator interrupt happens, Timer0 PWM period will be updated to (Initial Period x 2) by PDMA.\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n");
    printf("  Output frequency will be updated from 18kHz to 9kHz, and duty cycle from 50%% to 25%%.\n");

    printf("\n\nPress any key to start Timer0 PWM.\n\n");
    getchar();

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);
    u32InitPeriod = TPWM_GET_PERIOD(TIMER0);

    /* Enable output of Timer0 PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Enable Timer0 PWM accumulator function, interrupt count 10, accumulator source select to period point */
    TPWM_EnableAcc(TIMER0, 100, TPWM_IFA_PERIOD_POINT);

    /* Enable Timer0 PWM accumulator interrupt trigger PDMA */
    TPWM_EnableAccPDMA(TIMER0);

    /* Enable Timer0 PWM interrupt */
    NVIC_EnableIRQ(TIMER0_IRQn);

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form memory to TIMER PWM                              */
    /*--------------------------------------------------------------------------------------*/
    /* Open PDMA Channel 0 */
    PDMA_Open(PDMA0, (BIT0 << PDMA_CH));

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, PDMA_CH, PDMA_WIDTH_16, 1);

    /* Set updated period vaule */
    s_u32UpdatedPeriod = ((u32InitPeriod + 1) * 2) - 1;

    /* Set source address as s_u32UpdatedPeriod(no increment) and destination address as Timer0 PWM period register(no increment) */
    PDMA_SetTransferAddr(PDMA0, PDMA_CH, (uint32_t)&s_u32UpdatedPeriod, PDMA_SAR_FIX, (uint32_t) & (TIMER0->PWMPERIOD), PDMA_DAR_FIX);

    /* Select PDMA request source as PDMA_TMR0(Timer0 PWM accumulator interrupt) */
    PDMA_SetTransferMode(PDMA0, PDMA_CH, PDMA_TMR0, FALSE, 0);

    /* Set PDMA as single request type for Timer0 PWM */
    PDMA_SetBurstType(PDMA0, PDMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_1);

    /* Enable PDMA interrupt */
    PDMA_EnableInt(PDMA0, PDMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    /* Start Timer0 PWM counter */
    TPWM_START_COUNTER(TIMER0);

    g_u32IsTestOver = 0;

    /* Wait for PDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver != 1)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            return -1;
        }
    }

    printf("Timer0 PWM period register is updated from %d to %d\n\n", u32InitPeriod, TPWM_GET_PERIOD(TIMER0));

    printf("Press any key to stop Timer0 PWM.\n\n");
    getchar();

    /* Disable PDMA function */
    PDMA_Close(PDMA0);
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Stop Timer0 PWM */
    TPWM_STOP_COUNTER(TIMER0);

    /* Wait until Timer0 PWM Stop */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while ((TIMER0->PWMCNT & TIMER_PWMCNT_CNT_Msk) != 0)
        if (--u32TimeOutCnt == 0) break;

    if (u32TimeOutCnt == 0)
        printf("Wait for Timer PWM stop time-out!\n");
    else
        printf("Timer0 PWM has STOP.\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
