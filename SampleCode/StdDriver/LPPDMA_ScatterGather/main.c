/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Use LPPDMA0 channel 2 to transfer data from memory to memory by scatter-gather mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t LPPDMA_TEST_LENGTH = 64;

typedef struct dma_desc_t
{
    uint32_t u32Ctl;
    uint32_t u32Src;
    uint32_t u32Dest;
    uint32_t u32Offset;
} DMA_DESC_T;

#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
uint8_t au8SrcArray[256] __attribute__((section(".lpSram")));
uint8_t au8DestArray0[256] __attribute__((section(".lpSram")));
uint8_t au8DestArray1[256] __attribute__((section(".lpSram")));
DMA_DESC_T DMA_DESC[2] __attribute__((section(".lpSram")));
#else
uint8_t au8SrcArray[256] __attribute__ ((section(".ARM.__at_0x28000000")));
uint8_t au8DestArray0[256] __attribute__ ((section(".ARM.__at_0x28000100")));
uint8_t au8DestArray1[256] __attribute__ ((section(".ARM.__at_0x28000200")));
DMA_DESC_T DMA_DESC[2] __attribute__ ((section(".ARM.__at_0x28000300")));
#endif

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
    uint32_t u32Src, u32Dst0, u32Dst1;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------------------+ \n");
    printf("|    LPPDMA Memory to Memory Driver Sample Code (Scatter-gather)        | \n");
    printf("+-----------------------------------------------------------------------+ \n");

    u32Src = (uint32_t)au8SrcArray;
    u32Dst0 = (uint32_t)au8DestArray0;
    u32Dst1 = (uint32_t)au8DestArray1;

    /* This sample will transfer data by finished two descriptor table in sequence.(descriptor table 1 -> descriptor table 2) */

    /*----------------------------------------------------------------------------------
      LPPDMA transfer configuration:

        Channel = 2
        Operation mode = scatter-gather mode
        First scatter-gather descriptor table = DMA_DESC[0]
        Request source = LPPDMA_MEM(memory to memory)

        Transmission flow:
           ------------------------      -----------------------
          |                        |    |                       |
          |  DMA_DESC[0]           | -> |  DMA_DESC[1]          | -> transfer done
          |  (Descriptor table 1)  |    |  (Descriptor table 2) |
          |                        |    |                       |
           ------------------------      -----------------------

    ----------------------------------------------------------------------------------*/

    /* Open Channel 2 */
    LPPDMA_Open(LPPDMA0,1 << 2);
    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as memory to memory */
    LPPDMA_SetTransferMode(LPPDMA0,2, LPPDMA_MEM, 1, (uint32_t)&DMA_DESC[0]);

    /*------------------------------------------------------------------------------------------------------

                          au8SrcArray                         au8DestArray0
                          ---------------------------   -->   ---------------------------
                        /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                         |      |      |      |      |       |      |      |      |      |
      LPPDMA_TEST_LENGTH |            ...            |       |            ...            | LPPDMA_TEST_LENGTH
                         |      |      |      |      |       |      |      |      |      |
                        \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                          ---------------------------         ---------------------------
                          \                         /         \                         /
                                32bits(one word)                     32bits(one word)

      Descriptor table 1 configuration:

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[1](Descriptor table 2)
        transfer done and table empty interrupt = disable

        Transfer count = LPPDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray0
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = LPPDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[0].u32Ctl =
        ((LPPDMA_TEST_LENGTH - 1) << LPPDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is LPPDMA_TEST_LENGTH */ \
        LPPDMA_WIDTH_32 |   /* Transfer width is 32 bits(one word) */ \
        LPPDMA_SAR_INC |    /* Source increment size is 32 bits(one word) */ \
        LPPDMA_DAR_INC |    /* Destination increment size is 32 bits(one word) */ \
        LPPDMA_REQ_BURST |  /* Transfer type is burst transfer type */ \
        LPPDMA_BURST_128 |  /* Burst size is 128. No effect in single transfer type */ \
        LPPDMA_TBINTDIS_DISABLE |   /* Disable transfer done and table empty interrupt */ \
        LPPDMA_OP_SCATTER;  /* Operation mode is scatter-gather mode */

    /* Configure source address */
    DMA_DESC[0].u32Src = u32Src;
    /* Configure destination address */
    DMA_DESC[0].u32Dest = u32Dst0;
    /* Configure next descriptor table address */
    DMA_DESC[0].u32Offset = (uint32_t)&DMA_DESC[1] - (LPPDMA0->SCATBA); /* next descriptor table is table 2 */


    /*------------------------------------------------------------------------------------------------------

                          au8DestArray0                       au8DestArray1
                          ---------------------------   -->   ---------------------------
                        /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                         |      |      |      |      |       |      |      |      |      |
      LPPDMA_TEST_LENGTH |            ...            |       |            ...            | LPPDMA_TEST_LENGTH
                         |      |      |      |      |       |      |      |      |      |
                        \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                          ---------------------------         ---------------------------
                          \                         /         \                         /
                                32bits(one word)                     32bits(one word)

      Descriptor table 2 configuration:

        Operation mode = basic mode
        transfer done and table empty interrupt = enable

        Transfer count = LPPDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8DestArray0
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray1
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = LPPDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[1].u32Ctl =
        ((LPPDMA_TEST_LENGTH - 1) << LPPDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is LPPDMA_TEST_LENGTH */ \
        LPPDMA_WIDTH_32 |   /* Transfer width is 32 bits(one word) */ \
        LPPDMA_SAR_INC |    /* Source increment size is 32 bits(one word) */ \
        LPPDMA_DAR_INC |    /* Destination increment size is 32 bits(one word) */ \
        LPPDMA_REQ_BURST |  /* Transfer type is burst transfer type */ \
        LPPDMA_BURST_128 |  /* Burst size is 128. No effect in single transfer type */ \
        LPPDMA_OP_BASIC;    /* Operation mode is basic mode */

    DMA_DESC[1].u32Src = u32Dst0;
    DMA_DESC[1].u32Dest = u32Dst1;
    DMA_DESC[1].u32Offset = 0; /* No next operation table. No effect in basic mode */


    /* Generate a software request to trigger transfer with LPPDMA channel 2 */
    LPPDMA_Trigger(LPPDMA0,2);

    /* Waiting for transfer done */
    while(LPPDMA_IS_CH_BUSY(LPPDMA0,2));

    printf("test done...\n");

    /* Close Channel 2 */
    LPPDMA_Close(LPPDMA0);

    while(1);
}

