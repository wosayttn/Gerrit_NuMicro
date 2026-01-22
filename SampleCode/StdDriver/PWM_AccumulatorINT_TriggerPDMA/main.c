/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate PWM accumulator interrupt trigger PDMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define PWM_OUT_CH    0UL
#define PDMA_CH       0UL

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
__attribute__((aligned(4))) static uint16_t g_au16Period[2] = {31999, 15999};
static volatile uint32_t g_u32IsTestOver = 0;

void PDMA0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA0) & (PDMA_ABTSTS_ABTIF0_Msk << PDMA_CH))
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA0, (PDMA_ABTSTS_ABTIF0_Msk << PDMA_CH));
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)     /* done */
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
            printf("Wait for PDMA IntFlag time-out!\n");
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
    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPC_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for PWM */
    SET_PWM1_CH0_PC5();
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32NewCNR = 0;
    uint32_t u32TimeOutCnt = 0;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|              PWM AccumulatorINT TriggerPDMA Sample Code               |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code demonstrate PWM1 channel 0 accumulator interrupt trigger PDMA.\n");
    printf("  When accumulator interrupt happens, PWM1 channel 0 (PC5) period register will be updated \n");
    printf("  to g_u32Count array content, 31999(0x7CFF), by PDMA.\n");

    printf("\n\nPress any key to start PWM1 channel 0.(PC5)\n");
    getchar();

    /*--------------------------------------------------------------------------------------*/
    /* Set the PWM1 Channel 0 as PWM output function.                                     */
    /*--------------------------------------------------------------------------------------*/

    /* Set PWM1 channel 0 output configuration */
    PWM_ConfigOutputChannel(PWM1, PWM_OUT_CH, 300, 30);

    /* Enable PWM Output path for PWM1 channel 0 */
    PWM_EnableOutput(PWM1, PWM_CH_0_MASK);

    /* Enable PWM1 channel 0 accumulator, interrupt count 50, accumulator source select to zero point */
    PWM_EnableAcc(PWM1, PWM_OUT_CH, 50, PWM_IFA_ZERO_POINT);

    /* Enable PWM1 channel 0 accumulator interrupt */
    PWM_EnableAccInt(PWM1, PWM_OUT_CH);

    /* Enable PWM1 channel 0 accumulator interrupt trigger PDMA */
    PWM_EnableAccPDMA(PWM1, PWM_OUT_CH);

    /* Enable Timer for PWM1 channel 0 */
    PWM_Start(PWM1, PWM_CH_0_MASK);

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form memory to PWM                                   */
    /*--------------------------------------------------------------------------------------*/
    /* Open Channel 0 */
    PDMA_Open(PDMA0, (BIT0 << PDMA_CH));

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, PDMA_CH, PDMA_WIDTH_16, 1);

    /* Set source address as g_u32Count array(increment) and destination address as PWM1 channel 0 period register(no increment) */
    PDMA_SetTransferAddr(PDMA0, PDMA_CH, (uint32_t)&g_au16Period[0], PDMA_SAR_INC, (uint32_t) & (PWM1->PERIOD[0]), PDMA_DAR_FIX);

    /* Select PDMA request source as PWM1_CH0_TX(PWM1 channel 0 accumulator interrupt) */
    PDMA_SetTransferMode(PDMA0, PDMA_CH, PDMA_PWM1_CH0_TX, FALSE, 0);

    /* Set PDMA as single request type for PWM */
    PDMA_SetBurstType(PDMA0, PDMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_1);

    PDMA_EnableInt(PDMA0, PDMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    g_u32IsTestOver = 0;

    /* Wait for PDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver != 1)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            goto lexit;
        }
    }

    u32NewCNR = PWM_GET_CNR(PWM1, PWM_OUT_CH);
    printf("\n\nPWM1 channel0 period register is updated to %d(0x%x)\n", u32NewCNR, u32NewCNR);
    printf("Press any key to stop PWM1 channel 0.\n");
    getchar();

    /* Set PWM1 channel 0 loaded value as 0 */
    PWM_Stop(PWM1, PWM_CH_0_MASK);

    /* Wait until PWM1 channel 0 Timer Stop */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while ((PWM1->CNT[PWM_OUT_CH] & PWM_CNT_CNT_Msk) != 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM stop time-out!\n");
            break;
        }
    }

lexit:

    /* Disable Timer for PWM1 channel 0 */
    PWM_ForceStop(PWM1, PWM_CH_0_MASK);

    /* Disable PWM Output path for PWM1 channel 0 */
    PWM_DisableOutput(PWM1, PWM_CH_0_MASK);

    /* Disable PDMA NVIC */
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Close PDMA */
    PDMA_Close(PDMA0);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
