/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate PDCI Mode0 data transfer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT 22

volatile uint32_t g_au32FrameTransmitDoneIntFlag = 0;
volatile uint32_t g_u32CountCH0 = 0, g_u32CountCH1 = 0, g_u32CountCH2 = 0, g_u32CountCH3 = 0;

/* 5b Table for FIFO transfer */
volatile uint8_t g_au8FiveBTable_CH0[TEST_COUNT] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x1F, 0x1A, 0x15, 0x19, 0x14
};
volatile uint8_t g_au8FiveBTable_CH1[TEST_COUNT] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x1F, 0x1A, 0x15, 0x19, 0x14
};
volatile uint8_t g_au8FiveBTable_CH2[TEST_COUNT] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x1F, 0x1A, 0x15, 0x19, 0x14
};
volatile uint8_t g_au8FiveBTable_CH3[TEST_COUNT] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x1F, 0x1A, 0x15, 0x19, 0x14
};

/* Function prototype declaration */
void TMR0_IRQHandler(void);
void PDCI_IRQHandler(void);
void SYS_Init(void);
void PDCI_Init(void);

void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        if(PDCI_GET_CH_EMPTY_FLAG() & BIT0)
        {
            outp8((uint32_t)&(PDCI->TXDATG0) + 0x0, g_au8FiveBTable_CH0[g_u32CountCH0++]);
        }
        if(PDCI_GET_CH_EMPTY_FLAG() & BIT1)
        {
            outp8((uint32_t)&(PDCI->TXDATG0) + 0x1, g_au8FiveBTable_CH1[g_u32CountCH1++]);
        }
        if(PDCI_GET_CH_EMPTY_FLAG() & BIT2)
        {
            outp8((uint32_t)&(PDCI->TXDATG0) + 0x2, g_au8FiveBTable_CH2[g_u32CountCH2++]);
        }
        if(PDCI_GET_CH_EMPTY_FLAG() & BIT3)
        {
            outp8((uint32_t)&(PDCI->TXDATG0) + 0x3, g_au8FiveBTable_CH3[g_u32CountCH3++]);
        }
    }
}

void PDCI_IRQHandler(void)
{
    if((PDCI->INTSTS & PDCI_INTSTS_FTXDIF_Msk) && (PDCI->INTEN & PDCI_INTEN_FTXDIEN_Msk))
    {
        /* Check group 0 transmit done status */
        if(PDCI_GetStatus(PDCI_G0TXDF_MASK))
        {
            /* Clear group 0 transmit data flag */
            PDCI_ClearStatus(PDCI_G0TXDF_MASK);
        }

        /* Disable Timer0 NVIC */
        NVIC_DisableIRQ(TMR0_IRQn);
        /* Stop Timer0 counting */
        TIMER_Stop(TIMER0);

        g_au32FrameTransmitDoneIntFlag += 1;
    }
}

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

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

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

    /* Configure PDCI bit clock rate 2MHz, set 64 bits preamble, set the bit time period of logic 0 is 1.5 times logic 1. */
    PDCI_SetBitClock(2000000);
    PDCI_PREAMBLE_BIT(PDCI_PREAMBLE_64);
    PDCI_BITWIDTH_ADJUST(PDCI_BITWIDTH_15);

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
    printf("|                 PDCI Sample Code                 |\n");
    printf("+--------------------------------------------------+\n");
    printf("PDCI output pins:\n");
    printf("    G0_TX0(PB5), G0_TX1(PB4), G0_TX2(PB3), G0_TX3(PB2)\n");
    printf("Press any key to start transmission ...");
    getchar();
    printf("\n");

    /* Init PDCI */
    PDCI_Init();

    /* Open Timer0 in periodic mode, enable interrupt */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 2000000);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Enable frame transmit done interrupt */
    PDCI_EnableInt(PDCI_FTXD_INT_MASK);

    /* Enable PDCI NVIC */
    NVIC_EnableIRQ(PDCI_IRQn);

    /* PDCI interrupt has higher frequency then TMR0 interrupt. */
    NVIC_SetPriority(TMR0_IRQn, 3);
    NVIC_SetPriority(PDCI_IRQn, 2);

    PDCI->TXDATG0 = (g_au8FiveBTable_CH0[g_u32CountCH0++] << PDCI_TXDATG0_CH0_TXDAT_Pos) | (g_au8FiveBTable_CH1[g_u32CountCH1++] << PDCI_TXDATG0_CH1_TXDAT_Pos) |
                    (g_au8FiveBTable_CH2[g_u32CountCH2++] << PDCI_TXDATG0_CH2_TXDAT_Pos) | (g_au8FiveBTable_CH3[g_u32CountCH3++] << PDCI_TXDATG0_CH3_TXDAT_Pos);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* Enable PDCI controller */
    PDCI_ENABLE();

    /* Wait until one frame transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_au32FrameTransmitDoneIntFlag == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDCI interrupt time-out!\n");
            break;
        }
    }

    printf("\nExit PDCI driver sample code.\n");

    while(1);
}
