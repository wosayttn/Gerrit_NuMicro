/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate EPWM accumulator interrupt trigger PDMA.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint16_t s_au16Period[2] = {31999, 15999};
static volatile uint32_t s_u32IsTestOver = 0;


void PDMA_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       PDMA IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PDMA interrupt event
 */
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA);

    if(u32Status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF0_Msk)
            s_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(u32Status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF0_Msk)
            s_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable EPWM1 module clock */
    CLK_EnableModuleClock(EPWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* EPWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, 0);

    /* Reset EPWM1 module */
    SYS_ResetModule(EPWM1_RST);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PC multi-function pins for EPWM1 Channel 0 */
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_EPWM1_CH0;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCount;
    uint32_t u32NewCNR = 0;
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          EPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code demonstrates EPWM1 channel 0 accumulator interrupt trigger PDMA.\n");
    printf("  When accumulator interrupt happens, EPWM1 channel 0 period register will be updated \n");
    printf("  to g_u32Count array content, 31999(0x7CFF), by PDMA.\n");

    printf("\n\nPress any key to start EPWM1 channel 0.\n");
    getchar();

    /*--------------------------------------------------------------------------------------*/
    /* Set the EPWM1 Channel 0 as EPWM output function.                                       */
    /*--------------------------------------------------------------------------------------*/

    /* Assume EPWM output frequency is 300Hz and duty ratio is 30%, user can calculate EPWM settings as follows. (up counter type)
       duty ratio = (CMR)/(CNR+1)
       cycle time = CNR+1
       High level = CMR
       EPWM clock source frequency = PCLK = 36000000
       (CNR+1) = EPWM clock source frequency/prescaler/EPWM output frequency
               = 36000000/2/300 = 60000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 59999
       duty ratio = 30% ==> (CMR)/(CNR+1) = 30%
       CMR = 18000
       Prescale value is 1 : prescaler= 2
    */

    /* Set EPWM1 channel 0 output configuration */
    EPWM_ConfigOutputChannel(EPWM1, 0, 300, 30);

    /* Enable EPWM Output path for EPWM1 channel 0 */
    EPWM_EnableOutput(EPWM1, EPWM_CH_0_MASK);

    /* Enable EPWM1 channel 0 accumulator, interrupt count 50, accumulator source select to zero point */
    EPWM_EnableAcc(EPWM1, 0, 50, EPWM_IFA_ZERO_POINT);

    /* Enable EPWM1 channel 0 accumulator interrupt */
    EPWM_EnableAccInt(EPWM1, 0);

    /* Enable EPWM1 channel 0 accumulator interrupt trigger PDMA */
    EPWM_EnableAccPDMA(EPWM1, 0);

    /* Enable Timer for EPWM1 channel 0 */
    EPWM_Start(EPWM1, EPWM_CH_0_MASK);

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form memory to EPWM                                    */
    /*--------------------------------------------------------------------------------------*/
    /* Open Channel 0 */
    PDMA_Open(PDMA0, BIT0);

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, 1);

    /* Set source address as g_u32Count array(increment) and destination address as EPWM1 channel 0 period register(no increment) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)&s_au16Period[0], PDMA_SAR_INC, (uint32_t) & (EPWM1->PERIOD[0]), PDMA_DAR_FIX);

    /* Select PDMA request source as EPWM1_CH0_TX(EPWM1 channel 0 accumulator interrupt) */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_EPWM1_CH0_TX, FALSE, 0);

    /* Set PDMA as single request type for EPWM */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_1);

    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    s_u32IsTestOver = 0;

    /* Wait for PDMA transfer done */
    u32TimeOutCount = SystemCoreClock;
    while(s_u32IsTestOver != 1)
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nTimeout is happened, please check if it. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    u32NewCNR = EPWM_GET_CNR(EPWM1, 0);
    printf("\n\nEPWM1 channel 0 period register is updated to %d(0x%x) and frequency becomes 562 Hz.\n", u32NewCNR, u32NewCNR);
    printf("Press any key to stop EPWM1 channel 0.\n");
    getchar();

    /* Set EPWM1 channel 0 loaded value as 0 */
    EPWM_Stop(EPWM1, EPWM_CH_0_MASK);

    /* Wait until EPWM1 channel 0 Timer Stop */
    u32TimeOutCount = SystemCoreClock;
    while((EPWM1->CNT[0] & EPWM_CNT0_CNT_Msk) != 0)
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nTimeout is happened, please check if it. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    /* Disable Timer for EPWM1 channel 0 */
    EPWM_ForceStop(EPWM1, EPWM_CH_0_MASK);

    /* Disable EPWM Output path for EPWM1 channel 0 */
    EPWM_DisableOutput(EPWM1, EPWM_CH_0_MASK);

    /* Disable PDMA NVIC */
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Close PDMA */
    PDMA_Close(PDMA0);

    printf("End the EPWM test.\n");

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
