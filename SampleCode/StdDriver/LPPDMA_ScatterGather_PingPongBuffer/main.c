/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Use LPPDMA0 to implement Ping-Pong buffer by scatter-gather mode(memory to memory).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t LPPDMA_TEST_COUNT = 50;

typedef struct dma_desc_t
{
    uint32_t u32Ctl;
    uint32_t u32Src;
    uint32_t u32Dest;
    uint32_t u32Offset;
} DMA_DESC_T;

#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
__attribute__((section(".lpSram"))) uint32_t g_au32SrcArray0[1] = {0x55555555};
__attribute__((section(".lpSram"))) uint32_t g_au32SrcArray1[1] = {0xAAAAAAAA};
__attribute__((section(".lpSram"))) uint32_t g_au32DestArray[1];
__attribute__((section(".lpSram"))) DMA_DESC_T DMA_DESC[2];
#else
__attribute__ ((section(".ARM.__at_0x28000000"))) uint32_t g_au32SrcArray0[1] = {0x55555555};
__attribute__ ((section(".ARM.__at_0x28000004"))) uint32_t g_au32SrcArray1[1] = {0xAAAAAAAA};
__attribute__ ((section(".ARM.__at_0x28000008"))) uint32_t g_au32DestArray[1];
__attribute__ ((section(".ARM.__at_0x28000010"))) DMA_DESC_T DMA_DESC[2];
#endif

uint32_t volatile g_u32IsTestOver = 0;
uint32_t volatile g_u32TransferredCount = 0;
uint32_t g_u32DMAConfig = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M2L31.s.
 */
void LPPDMA0_IRQHandler(void)
{
    /* Check channel transfer done status */
    if (LPPDMA_GET_TD_STS(LPPDMA0) == LPPDMA_TDSTS_TDIF2_Msk)
    {
        /* When finished a descriptor table then g_u32TransferredCount increases 1 */
        g_u32TransferredCount++;

        /* Check if LPPDMA has finished LPPDMA_TEST_COUNT tasks */
        if (g_u32TransferredCount >= LPPDMA_TEST_COUNT)
        {
            /* Set LPPDMA into idle state by Descriptor table */
            DMA_DESC[0].u32Ctl &= ~LPPDMA_DSCT_CTL_OPMODE_Msk;
            DMA_DESC[1].u32Ctl &= ~LPPDMA_DSCT_CTL_OPMODE_Msk;
            g_u32IsTestOver = 1;
        }
        /* Clear transfer done flag of channel 2 */
        LPPDMA_CLR_TD_FLAG(LPPDMA0,LPPDMA_TDSTS_TDIF2_Msk);
    }
}

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

    /* Set HCLK1 to HIRC */
    CLK_SetModuleClock(HCLK1_MODULE, CLK_CLKSEL0_HCLK1SEL_HIRC, LPSCC_CLKDIV0_HCLK1(1));

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------------------+ \n");
    printf("|     LPPDMA Driver Ping-Pong Buffer Sample Code (Scatter-gather)       | \n");
    printf("+-----------------------------------------------------------------------+ \n");

    /* This sample will transfer data by looped around two descriptor tables from two different source to the same destination buffer in sequence.
       And operation sequence will be table 1 -> table 2-> table 1 -> table 2 -> table 1 -> ... -> until LPPDMA configuration doesn't be reloaded. */

    /*--------------------------------------------------------------------------------------------------
      LPPDMA transfer configuration:

        Channel = 2
        Operation mode = scatter-gather mode
        First scatter-gather descriptor table = DMA_DESC[0]
        Request source = LPPDMA_MEM(memory to memory)

        Transmission flow:

                                            loop around
                                      LPPDMA_TEST_COUNT/2 times
           ------------------------                             -----------------------
          |                        | ------------------------> |                       |
          |  DMA_DESC[0]           |                           |  DMA_DESC[1]          |
          |  (Descriptor table 1)  |                           |  (Descriptor table 2) |
          |                        | <-----------------------  |                       |
           ------------------------                             -----------------------

        Note: The configuration of each table in LPSRAM need to be reloaded after transmission finished.
    --------------------------------------------------------------------------------------------------*/

    /* Open Channel 2 */
    LPPDMA_Open(LPPDMA0,1 << 2);

    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as memory to memory */
    LPPDMA_SetTransferMode(LPPDMA0,2, LPPDMA_MEM, TRUE, (uint32_t)&DMA_DESC[0]);


    /* Scatter-Gather descriptor table configuration in LPSRAM */
    g_u32DMAConfig = \
                     (0 << LPPDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is 1 */ \
                     LPPDMA_WIDTH_32 |  /* Transfer width is 32 bits(one word) */ \
                     LPPDMA_SAR_FIX |   /* Source increment size is fixed(no increment) */ \
                     LPPDMA_DAR_FIX |   /* Destination increment size is fixed(no increment) */ \
                     LPPDMA_REQ_BURST | /* Transfer type is burst transfer type */ \
                     LPPDMA_BURST_1 |   /* Burst size is 128. No effect in single transfer type */ \
                     LPPDMA_OP_SCATTER; /* Operation mode is scatter-gather mode */
    /*-----------------------------------------------------------------------------------------------------------------------------------------------------------
       Note: LPPDMA_REQ_BURST is only supported in memory-to-memory transfer mode.
             LPPDMA transfer type should be set as LPPDMA_REQ_SINGLE in memory-to-peripheral and peripheral-to-memory transfer mode,
             then above code will be modified as follows:
             g_u32DMAConfig = (0 << LPPDMA_DSCT_CTL_TXCNT_Pos) | LPPDMA_WIDTH_32 | LPPDMA_SAR_FIX | LPPDMA_DAR_FIX | LPPDMA_BURST_1 | LPPDMA_REQ_SINGLE | LPPDMA_OP_SCATTER;
    -----------------------------------------------------------------------------------------------------------------------------------------------------------*/

    /*------------------------------------------------------------------------------------------------------
      Descriptor table 1 configuration:

             g_au32SrcArray0               transfer 1 times    g_au32DestArray
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   32bits(one word)                                  32bits(one word)

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[1](Descriptor table 2)
        transfer done and table empty interrupt = enable

        Transfer count = 1
        Transfer width = 32 bits(one word)
        Source address = g_au32SrcArray0
        Source address increment size = fixed address(no increment)
        Destination address = au8DestArray0
        Destination address increment size = fixed address(no increment)
        Transfer type = burst transfer

        Total transfer length = 1 * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[0].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[0].u32Src = (uint32_t)g_au32SrcArray0; /* Ping-Pong buffer 1 */
    /* Configure destination address */
    DMA_DESC[0].u32Dest = (uint32_t)&g_au32DestArray[0];
    /* Configure next descriptor table address */
    DMA_DESC[0].u32Offset = (uint32_t)&DMA_DESC[1] - (LPPDMA0->SCATBA); /* next operation table is table 2 */

    /*------------------------------------------------------------------------------------------------------
      Descriptor table 2 configuration:

             g_au32SrcArray1               transfer 1 times    g_au32DestArray
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   32bits(one word)                                  32bits(one word)

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[0](Descriptor table 1)
        transfer done and table empty interrupt = enable

        Transfer count = 1
        Transfer width = 32 bits(one word)
        Source address = g_au32SrcArray1
        Source address increment size = fixed address(no increment)
        Destination address = au8DestArray0
        Destination address increment size = fixed address(no increment)
        Transfer type = burst transfer

        Total transfer length = 1 * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[1].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[1].u32Src = (uint32_t)g_au32SrcArray1; /* Ping-Pong buffer 2 */
    /* Configure destination address */
    DMA_DESC[1].u32Dest = (uint32_t)&g_au32DestArray[0];
    /* Configure next descriptor table address */
    DMA_DESC[1].u32Offset = (uint32_t)&DMA_DESC[0] - (LPPDMA0->SCATBA); /* next operation table is table 1 */


    /* Enable transfer done interrupt */
    LPPDMA_EnableInt(LPPDMA0,2, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA0_IRQn);
    g_u32IsTestOver = 0;

    /* Start LPPDMA operation */
    LPPDMA_Trigger(LPPDMA0,2);

    while(1)
    {
        if(g_u32IsTestOver == 1)
        {
            g_u32IsTestOver = 0;
            printf("test done...\n");

            /* Close LPPDMA channel */
            LPPDMA_Close(LPPDMA0);
        }
    }
}

