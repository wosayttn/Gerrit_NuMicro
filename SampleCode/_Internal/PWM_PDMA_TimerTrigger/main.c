/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    PWM change freq/duty by TMR0 trigger PDMA to modify PWM PERIOD/CMPDAT(Ping-Pong buffer
 *           by scatter-gather mode).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

uint16_t g_au16SrcArrayPWMPERIOD[2];
uint16_t g_au16SrcArrayPWMCMPDAT[2];
uint32_t g_u32DMAConfig = 0;

typedef struct dma_desc_t
{
    uint32_t u32Ctl;
    uint32_t u32Src;
    uint32_t u32Dest;
    uint32_t u32Offset;
} DMA_DESC_T;

DMA_DESC_T DMA_DESC[4];


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);
    SYS_ResetModule(PDMA_RST);
    /* Enable TMR0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    SYS_ResetModule(TMR0_RST);
    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);
    SYS_ResetModule(PWM0_RST);
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
    SYS_ResetModule(UART0_RST);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for PWM0 Channel 0,1 */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB5MFP_PWM0_CH0 | SYS_GPB_MFPL_PB4MFP_PWM0_CH1);

    Uart0DefaultMPF();
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
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+----------------------------------------------------------------------------+ \n");
    printf("|       PWM PDMA TimerTrigger Sample, check PB5 & PB4 for PWM waveform       | \n");
    printf("|    PWM0 channel 0(PB5) between freq 1MHz, duty 25/freq 100KHz, duty 75     | \n");
    printf("|    PWM0 channel 1(PB4) between freq 1MHz, duty 25/freq 100KHz, duty 2.5    | \n");
    printf("+----------------------------------------------------------------------------+ \n");

    /* PWM0 channel 0/1 defalt frequency is 1000000Hz, duty 25%. */
    PWM_ConfigOutputChannel(PWM0, 0, 1000000, 25);
    PWM_ConfigOutputChannel(PWM0, 1, 1000000, 25);
    g_au16SrcArrayPWMCMPDAT[0] = PWM_GET_CMR(PWM0, 0);
    g_au16SrcArrayPWMPERIOD[0] = PWM_GET_CNR(PWM0, 0);

    /* PWM0 channel 0/1 change frequency is 100000Hz; Channel 0 changes duty to 75%. */
    g_au16SrcArrayPWMPERIOD[1] = g_au16SrcArrayPWMPERIOD[0] * 10;                         // Frequency is 1MHz/10 = 100KHz
    g_au16SrcArrayPWMCMPDAT[1] = 75 * (g_au16SrcArrayPWMPERIOD[1] + 1UL) / 100UL; // Duty is 75%

    /* Enable output of PWM0 channel 0/1 */
    PWM_EnableOutput(PWM0, 0x3);

    /* Set PDMA will transfer data by looped around two descriptor tables from two different source to the same destination buffer in sequence.
    And operation sequence will be table 1 -> table 2-> table 1 -> table 2 -> table 1 -> ... -> until PDMA configuration doesn't be reloaded. */

    /*--------------------------------------------------------------------------------------------------
      PDMA transfer configuration:

        Channel 0/1 operation mode = scatter-gather mode
        Channel 0 first scatter-gather descriptor table = DMA_DESC[0]
        Channel 1 first scatter-gather descriptor table = DMA_DESC[2]
        Channel 0/1 request source = PDMA_TMR0(DMA Connect to TMR0)

      Transmission flow:
                                            loop around
           ------------------------                             -----------------------
          |                        | ------------------------> |                       |
          |  DMA_DESC[0/2]         |                           |  DMA_DESC[1/3]        |
          |  (Descriptor table 1)  |                           |  (Descriptor table 2) |
          |                        | <-----------------------  |                       |
           ------------------------                             -----------------------

        Note: The configuration of each table in SRAM need to be reloaded after transmission finished.
    --------------------------------------------------------------------------------------------------*/

    /* Open Channel 0/1 */
    PDMA_Open(PDMA, 0x3);

    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as DMA Connect to TMR0 */
    PDMA_SetTransferMode(PDMA, 0, PDMA_TMR0, TRUE, (uint32_t)&DMA_DESC[0]);
    PDMA_SetTransferMode(PDMA, 1, PDMA_TMR0, TRUE, (uint32_t)&DMA_DESC[2]);

    /* Scatter-Gather descriptor table configuration in SRAM */
    g_u32DMAConfig = \
                     (0 << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is 1. */ \
                     PDMA_WIDTH_16 |   /* Transfer width is 16 bits. */ \
                     PDMA_SAR_FIX |    /* Source increment size is fixed (no increment). */ \
                     PDMA_DAR_FIX |    /* Destination increment size is fixed (no increment). */ \
                     PDMA_REQ_SINGLE | /* Transfer type is single transfer type. */ \
                     PDMA_BURST_1 |    /* Burst size is 1. No effect in single transfer type. */ \
                     PDMA_OP_SCATTER;  /* Operation mode is scatter-gather mode. */

    /*------------------------------------------------------------------------------------------------------
      PDMA Channel 0 Descriptor table 1 configuration:

             g_au16SrcArrayPWMPERIOD0       transfer 1 times    PWM0_BASE+0x30(PWMPERIOD)
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   16bits                                            16bits

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[1](Descriptor table 2)

        Transfer count = 1
        Transfer width = 16 bits
        Source address = (uint32_t)&g_au16SrcArrayPWMPERIOD[0]
        Source address increment size = fixed address(no increment)
        Destination address = (uint32_t)&(PWM0->PERIOD[0])
        Destination address increment size = fixed address(no increment)
        Transfer type = single transfer

        Total transfer length = 1 * 16 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[0].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[0].u32Src = (uint32_t)&g_au16SrcArrayPWMPERIOD[0]; /* Ping-Pong buffer 1 */
    /* Configure destination address */
    DMA_DESC[0].u32Dest = (uint32_t) & (PWM0->PERIOD[0]);
    /* Configure next descriptor table address */
    DMA_DESC[0].u32Offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA); /* Next operation table is table 2. */

    /*------------------------------------------------------------------------------------------------------
      PDMA Channel 0 Descriptor table 2 configuration:

             g_au16SrcArrayPWMPERIOD1      transfer 1 times    PWM0_BASE+0x30(PWMPERIOD)
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   16bits                                            16bits

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[0](Descriptor table 1)

        Transfer count = 1
        Transfer width = 16 bits
        Source address = (uint32_t)&g_au16SrcArrayPWMPERIOD[1]
        Source address increment size = fixed address(no increment)
        Destination address = (uint32_t)&(PWM0->PERIOD[0])
        Destination address increment size = fixed address(no increment)
        Transfer type = single transfer

        Total transfer length = 1 * 16 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[1].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[1].u32Src = (uint32_t)&g_au16SrcArrayPWMPERIOD[1]; /* Ping-Pong buffer 2 */
    /* Configure destination address */
    DMA_DESC[1].u32Dest = (uint32_t) & (PWM0->PERIOD[0]);
    /* Configure next descriptor table address */
    DMA_DESC[1].u32Offset = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA); /* Next operation table is table 1. */

    /*------------------------------------------------------------------------------------------------------
      PDMA Channel 1 Descriptor table 1 configuration:

             g_au32SrcArrayPWMCMPDAT0       transfer 1 times    PWM0_BASE+0x50(PWMCMPDAT)
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   16bits                                            16bits

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[3](Descriptor table 2)

        Transfer count = 1
        Transfer width = 16 bits
        Source address = (uint32_t)&g_au16SrcArrayPWMCMPDAT[0]
        Source address increment size = fixed address(no increment)
        Destination address = (uint32_t)&(PWM0->CMPDAT[0])
        Destination address increment size = fixed address(no increment)
        Transfer type = single transfer

        Total transfer length = 1 * 16 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[2].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[2].u32Src = (uint32_t)&g_au16SrcArrayPWMCMPDAT[0]; /* Ping-Pong buffer 1 */
    /* Configure destination address */
    DMA_DESC[2].u32Dest = (uint32_t) & (PWM0->CMPDAT[0]);
    /* Configure next descriptor table address */
    DMA_DESC[2].u32Offset = (uint32_t)&DMA_DESC[3] - (PDMA->SCATBA); /* Next operation table is table 2. */

    /*------------------------------------------------------------------------------------------------------
    PDMA Channel 1 Descriptor table 2 configuration:

         g_au32SrcArrayPWMCMPDAT1      transfer 1 times    PWM0_BASE+0x50(PWMCMPDAT)
         ---------------------------   ----------------->  ---------------------------
        |            [0]            |                     |            [0]            |
         ---------------------------                       ---------------------------
         \                         /                       \                         /
               16bits                                            16bits

    Operation mode = scatter-gather mode
    Next descriptor table = DMA_DESC[0](Descriptor table 1)

    Transfer count = 1
    Transfer width = 16 bits
    Source address = (uint32_t)&g_au16SrcArrayPWMCMPDAT[1]
    Source address increment size = fixed address(no increment)
    Destination address = (uint32_t)&(PWM0->CMPDAT[0])
    Destination address increment size = fixed address(no increment)
    Transfer type = single transfer

    Total transfer length = 1 * 16 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[3].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[3].u32Src = (uint32_t)&g_au16SrcArrayPWMCMPDAT[1]; /* Ping-Pong buffer 2 */
    /* Configure destination address */
    DMA_DESC[3].u32Dest = (uint32_t) & (PWM0->CMPDAT[0]);
    /* Configure next descriptor table address */
    DMA_DESC[3].u32Offset = (uint32_t)&DMA_DESC[2] - (PDMA->SCATBA); /* Next operation table is table 1. */

    /* Set Timer0 operation, every 1 sec timeout trigger PDMA*/
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_SetTriggerSource(TIMER0, TIMER_TRGSEL_TIMEOUT_EVENT);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_PDMA);

    /* Start PWM0 channel 0/1 */
    PWM_Start(PWM0, 0x3);

    /* Start PDMA operation */
    PDMA_Trigger(PDMA, 0);
    PDMA_Trigger(PDMA, 1);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    while (1);
}
