/*************************************************************************//**
 * @file     isr.c
 * @version  V1.00
 * @brief    ISR source file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "config.h"

//------------------------------------------------------------------------------
extern volatile uint8_t aPCMBuffer_Full[2];
extern void PDMA_ResetTxSGTable(uint8_t id);
//------------------------------------------------------------------------------
volatile uint8_t u8PCMBuffer_Playing = 0;

//------------------------------------------------------------------------------
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA_PORT);

    if (u32Status & PDMA_INTSTS_TDIF_Msk)   /* done */
    {
        if (PDMA_GET_TD_STS(PDMA_PORT) & (1u << I2S_TX_DMA_CH))
        {
            if (aPCMBuffer_Full[u8PCMBuffer_Playing ^ 1] != 1)
                printf("underflow!!\n");

            aPCMBuffer_Full[u8PCMBuffer_Playing] = 0;       /* Set empty flag */
            PDMA_ResetTxSGTable(u8PCMBuffer_Playing);
            u8PCMBuffer_Playing ^= 1;
            PDMA_CLR_TD_FLAG(PDMA_PORT, (1u << I2S_TX_DMA_CH));
        }
    }

    //else if (u32Status & 0x400)    /* Timeout */
    //{
    //    //PDMA_CLR_TMOUT_FLAG(PDMA_PORT, (1u << I2S_TX_DMA_CH));
    //    printf("PDMA Timeout!!\n");
    //}
}
