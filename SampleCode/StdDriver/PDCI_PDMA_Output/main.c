/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate PDCI Mode0 data transfer with PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT 22

/* 5b Table for PDMA transfer */
volatile uint32_t g_au32FiveBTable_GROUP0[TEST_COUNT] =
{
    0x00000000, 0x01010101, 0x02020202, 0x03030303, 0x04040404, 0x05050505, 0x06060606, 0x07070707, 0x08080808, 0x09090909, 0x0A0A0A0A,
    0x0B0B0B0B, 0x0C0C0C0C, 0x0D0D0D0D, 0x0E0E0E0E, 0x0F0F0F0F, 0x10101010, 0x1F1F1F1F, 0x1A1A1A1A, 0x15151515, 0x19191919, 0x14141414
};

/* Function prototype declaration */
void SYS_Init(void);
void PDCI_Init(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_180MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable PDCI module clock */
    CLK_EnableModuleClock(PDCI_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Configure PDCI related multi-function pins for group 0. */
    SET_PDCI_G0_TX0_PB5();
    SET_PDCI_G0_TX1_PB4();
    SET_PDCI_G0_TX2_PB3();
    SET_PDCI_G0_TX3_PB2();

    /* Enable PDCI I/O schmitt trigger */
    GPIO_ENABLE_SCHMITT_TRIGGER(PB, BIT2 | BIT3 | BIT4 | BIT5);
}

void PDCI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init PDCI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure PDCI bit clock rate 2MHz, set 64 bits preamble, set the bit time period of logic 0 is same as logic 1. */
    PDCI_SetBitClock(2000000);
    PDCI_PREAMBLE_BIT(PDCI_PREAMBLE_64);
    PDCI_BITWIDTH_ADJUST(PDCI_BITWIDTH_1);

    /* Set the dummy level to high and the dummy delay time of group 0 to 1000us. Enable PDCI group 0 channels. */
    PDCI_DUMMY_LEVEL(PDCI_DUM_LVL_HIGH);
    PDCI_SetDummyDelayPeriod(PDCI_GROUP_0, 1000);
    PDCI_ENABLE_GROUP0();
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+--------------------------------------------------+\n");
    printf("|              PDCI + PDMA Sample Code             |\n");
    printf("+--------------------------------------------------+\n");
    printf("PDCI output pins:\n");
    printf("    G0_TX0(PB5), G0_TX1(PB4), G0_TX2(PB3), G0_TX3(PB2)\n");
    printf("Press any key to start transmission ...");
    getchar();
    printf("\n");

    /* Init PDCI */
    PDCI_Init();

    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Open Channel 0 */
    PDMA_Open(PDMA0, 1 << 0);
    /* Transfer count is TEST_COUNT, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_32, TEST_COUNT);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32FiveBTable_GROUP0, PDMA_SAR_INC, (uint32_t)&PDCI->TXDATG0, PDMA_DAR_FIX);
    /* Request source is PDMA_PDCI_TX0 */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_PDCI_TX0, FALSE, 0);
    /* Transfer type is single transfer and burst size is 4 (No effect in single request type) */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_4);

    /* Enable PDMA function */
    PDCI_ENABLE_DMA();

    /* Enable PDCI controller */
    PDCI_ENABLE();

    /* Wait until one frame transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!PDCI_GetIntFlag(PDCI_FTXD_INT_MASK))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDCI interrupt time-out!\n");
            break;
        }
    }

    /* Disable PDMA function */
    PDCI_DISABLE_DMA();

    printf("\nExit PDCI driver sample code.\n");

    while(1);
}
