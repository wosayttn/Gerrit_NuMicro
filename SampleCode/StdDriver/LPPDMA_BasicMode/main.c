/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use LPPDMA0 channel 2 to transfer data from memory to memory.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t LPPDMA_TEST_LENGTH = 64;
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
uint8_t au8SrcArray[256] __attribute__((section(".lpSram")));
uint8_t au8DestArray[256] __attribute__((section(".lpSram")));
#else
uint8_t au8SrcArray[256] __attribute__ ((section(".ARM.__at_0x28000000")));
uint8_t au8DestArray[256] __attribute__ ((section(".ARM.__at_0x28000100")));
#endif
uint32_t volatile g_u32IsTestOver = 0;

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
    uint32_t u32Status = LPPDMA_GET_INT_STATUS(LPPDMA0);

    if(u32Status & LPPDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        /* Check if channel 2 has abort error */
        if(LPPDMA_GET_ABORT_STS(LPPDMA0) & LPPDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;
        /* Clear abort flag of channel 2 */
        LPPDMA_CLR_ABORT_FLAG(LPPDMA0,LPPDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(u32Status & LPPDMA_INTSTS_TDIF_Msk)      /* done */
    {
        /* Check transmission of channel 2 has been transfer done */
        if(LPPDMA_GET_TD_STS(LPPDMA0) & LPPDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;
        /* Clear transfer done flag of channel 2 */
        LPPDMA_CLR_TD_FLAG(LPPDMA0,LPPDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
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

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable LPPDMA clock source */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init()
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
    printf("+------------------------------------------------------+ \n");
    printf("|    LPPDMA Memory to Memory Driver Sample Code        | \n");
    printf("+------------------------------------------------------+ \n");


    /*------------------------------------------------------------------------------------------------------

                          au8SrcArray                         au8DestArray
                          ---------------------------   -->   ---------------------------
                        /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                         |      |      |      |      |       |      |      |      |      |
      LPPDMA_TEST_LENGTH |            ...            |       |            ...            | LPPDMA_TEST_LENGTH
                         |      |      |      |      |       |      |      |      |      |
                        \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                          ---------------------------         ---------------------------
                          \                         /         \                         /
                                32bits(one word)                     32bits(one word)

      LPPDMA transfer configuration:

        Channel = 2
        Operation mode = basic mode
        Request source = LPPDMA_MEM(memory to memory)
        transfer done and table empty interrupt = enable

        Transfer count = LPPDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = LPPDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/

    /* Open Channel 2 */
    LPPDMA_Open(LPPDMA0,1 << 2);
    /* Transfer count is LPPDMA_TEST_LENGTH, transfer width is 32 bits(one word) */
    LPPDMA_SetTransferCnt(LPPDMA0,2, LPPDMA_WIDTH_32, LPPDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    LPPDMA_SetTransferAddr(LPPDMA0,2, (uint32_t)au8SrcArray, LPPDMA_SAR_INC, (uint32_t)au8DestArray, LPPDMA_DAR_INC);
    /* Request source is memory to memory */
    LPPDMA_SetTransferMode(LPPDMA0,2, LPPDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    LPPDMA_SetBurstType(LPPDMA0,2, LPPDMA_REQ_BURST, LPPDMA_BURST_4);

    /* Enable interrupt */
    LPPDMA_EnableInt(LPPDMA0,2, LPPDMA_INT_TRANS_DONE);

    /* Enable NVIC for LPPDMA */
    NVIC_EnableIRQ(LPPDMA0_IRQn);
    g_u32IsTestOver = 0;

    /* Generate a software request to trigger transfer with LPPDMA channel 2  */
    LPPDMA_Trigger(LPPDMA0,2);

    /* Waiting for transfer done */
    while(g_u32IsTestOver == 0);

    /* Check transfer result */
    if(g_u32IsTestOver == 1)
        printf("test done...\n");
    else if(g_u32IsTestOver == 2)
        printf("target abort...\n");

    /* Close channel 2 */
    LPPDMA_Close(LPPDMA0);

    while(1);
}
