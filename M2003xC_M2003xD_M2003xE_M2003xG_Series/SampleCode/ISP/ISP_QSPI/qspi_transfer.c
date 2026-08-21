/**************************************************************************//**
 * @file     qspi_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    QSPI ISP slave sample file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define TEST_COUNT 16

uint32_t spi_rcvbuf[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

volatile uint8_t bSpiDataReady = 0;

void QSPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure QSPI0 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    QSPI0->CTL = QSPI_SLAVE | QSPI_CTL_TXNEG_Msk | QSPI_CTL_SPIEN_Msk;
    /* Configure QSPI0 as a low level active device. */
    QSPI0->SSCTL = 0;
    /* Set IP clock divider. SPI peripheral clock rate = f_PCLK0 */
    QSPI0->CLKDIV = 0;
    /* Set TX FIFO threshold and enable FIFO mode. */
    QSPI0->FIFOCTL = (QSPI0->FIFOCTL & ~(QSPI_FIFOCTL_TXTH_Msk | QSPI_FIFOCTL_RXTH_Msk)) |
                     (4 << QSPI_FIFOCTL_TXTH_Pos) |
                     (4 << QSPI_FIFOCTL_RXTH_Pos);
    /* Enable slave selection signal active interrupt flag */
    QSPI0->SSCTL |= QSPI_SSCTL_SSACTIEN_Msk;
    QSPI_WRITE_TX(QSPI0, 0xFFFFFFFF);    /* Dummy Write to prevent TX under run */
    NVIC_EnableIRQ(QSPI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  QSPI0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void QSPI0_IRQHandler(void)
{
    uint32_t *_response_buff;
    _response_buff = (uint32_t *)response_buff; // in isp_user.c

    if (QSPI0->STATUS & QSPI_STATUS_SSACTIF_Msk)
    {
        QSPI0->STATUS |= QSPI_STATUS_SSACTIF_Msk;
        QSPI0->FIFOCTL |= (QSPI_FIFOCTL_RXFBCLR_Msk | QSPI_FIFOCTL_TXFBCLR_Msk);
        g_u32TxDataCount = 0;
        g_u32RxDataCount = 0;

        // Active
        while (!(QSPI0->STATUS & QSPI_STATUS_SSINAIF_Msk))
        {
            /* Check TX FULL flag and TX data count */
            if ((QSPI_GET_TX_FIFO_FULL_FLAG(QSPI0) == 0) && (g_u32TxDataCount < TEST_COUNT))
            {
                QSPI_WRITE_TX(QSPI0, _response_buff[g_u32TxDataCount]);    /* Write to TX FIFO */
                g_u32TxDataCount++;
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                SysTick->LOAD = 1000 * CyclesPerUs;
                SysTick->VAL   = (0x00);
                SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            }

            /* Check RX EMPTY flag */
            if (QSPI_GET_RX_FIFO_EMPTY_FLAG(QSPI0) == 0)
            {
                g_u32RxDataCount &= 0x0F;
                spi_rcvbuf[g_u32RxDataCount++] = QSPI_READ_RX(QSPI0);    /* Read RX FIFO */
#ifdef ReadyPin
                // If hardware flow control pin is used, the slave side needs to pull this pin to high status before exiting irq.
                if ((g_u32RxDataCount == 1) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900))
                {
                    ReadyPin = 1;
                }
#endif
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                break;
            }
        }

        if (QSPI0->STATUS & QSPI_STATUS_SSINAIF_Msk)
        {
            QSPI0->STATUS |= QSPI_STATUS_SSINAIF_Msk;

            if ((g_u32RxDataCount == 16) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900))
            {
                bSpiDataReady = 1;
            }

            spi_rcvbuf[0] &= 0x000000FF;
            g_u32TxDataCount = 0;
            g_u32RxDataCount = 0;

            if (QSPI_GET_TX_FIFO_FULL_FLAG(QSPI0) == 0)
            {
                QSPI_WRITE_TX(QSPI0, 0xFFFFFFFF);    /* Write to TX FIFO */
            }
        }
    }
    else
    {
    }
}
