/**************************************************************************//**
 * @file     usci_spi_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    USPI ISP slave sample file
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

void USPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set USCI_SPI protocol */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;
    USPI0->CTL = 1UL << USPI_CTL_FUNMODE_Pos;
    /* Configure USPI0 as a slave, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI0->PROTCTL &= ~(USPI_PROTCTL_SCLKMODE_Msk | USPI_PROTCTL_SLAVE_Msk);
    USPI0->PROTCTL |= (USPI_SLAVE | USPI_MODE_0);
    USPI0->LINECTL &= ~USPI_LINECTL_DWIDTH_Msk;
    /* Configure USPI0 as a low level active device. */
    USPI0->CTLIN0 |= USPI_CTLIN0_ININV_Msk;
    /* Eable SPI Protocol. */
    USPI0->PROTCTL |=  USPI_PROTCTL_PROTEN_Msk;
    /* Enable slave selection signal active interrupt flag */
    USPI0->PROTIEN |= USPI_PROTIEN_SSACTIEN_Msk;
    USPI_WRITE_TX(USPI0, 0xFFFF);    /* Dummy Write to prevent TX under run */
    NVIC_EnableIRQ(USCI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  SPI0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    uint16_t *_response_buff;
    _response_buff = (uint16_t *)response_buff; // in isp_user.c

    if (USPI0->PROTSTS & USPI_PROTSTS_SSACTIF_Msk)
    {
        USPI0->PROTSTS |= USPI_PROTSTS_SSACTIF_Msk;
        USPI0->BUFCTL |= (USPI_BUFCTL_RXCLR_Msk | USPI_BUFCTL_TXCLR_Msk);
        g_u32TxDataCount = 0;
        g_u32RxDataCount = 0;

        // Active
        while (!(USPI0->PROTSTS & USPI_PROTSTS_SSINAIF_Msk))
        {
            /* Check TX FULL flag and TX data count */
            if ((USPI_GET_TX_FULL_FLAG(USPI0) == 0) && (g_u32TxDataCount < 32))
            {

                if(g_u32TxDataCount == 0)
                    USPI_WRITE_TX(USPI0, 0xFFFF);

                while((USPI_GET_TX_FULL_FLAG(USPI0) != 0));

                if(g_u32TxDataCount & 0x1)
                    USPI_WRITE_TX(USPI0, _response_buff[g_u32TxDataCount - 1]);    /* Write to TX FIFO */
                else
                    USPI_WRITE_TX(USPI0, _response_buff[g_u32TxDataCount + 1]);    /* Write to TX FIFO */

                g_u32TxDataCount++;
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                SysTick->LOAD = 1000 * CyclesPerUs;
                SysTick->VAL   = (0x00);
                SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            }

            /* Check RX EMPTY flag */
            if (USPI_GET_RX_EMPTY_FLAG(USPI0) == 0)
            {
                g_u32RxDataCount &= 0x1F;

                if(g_u32RxDataCount & 0x1)
                    ((uint16_t *)spi_rcvbuf)[g_u32RxDataCount - 1] = USPI_READ_RX(USPI0);    /* Read RX FIFO */
                else
                    ((uint16_t *)spi_rcvbuf)[g_u32RxDataCount + 1] = USPI_READ_RX(USPI0);    /* Read RX FIFO */

                g_u32RxDataCount++;
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                break;
            }
        }

        if (USPI0->PROTSTS & USPI_PROTSTS_SSINAIF_Msk)
        {
            USPI0->PROTSTS |= USPI_PROTSTS_SSINAIF_Msk;

            if ((g_u32RxDataCount == 32) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900))
            {
                bSpiDataReady = 1;
            }

            spi_rcvbuf[0] &= 0x000000FF;
            g_u32TxDataCount = 0;
            g_u32RxDataCount = 0;

            if (USPI_GET_TX_FULL_FLAG(USPI0) == 0)
            {
                USPI_WRITE_TX(USPI0, 0xFFFF);    /* Write to TX FIFO */
            }
        }
    }
    else
    {
    }
}
