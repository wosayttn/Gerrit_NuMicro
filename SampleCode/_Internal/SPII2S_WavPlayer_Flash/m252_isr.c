/******************************************************************************
 * @file     M252_isr.c
 * @brief    M252 series ISR source file
 *
 * @note
 * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"

extern volatile uint8_t aPCMBuffer_Full[2];
extern volatile uint8_t u8PCMBufferTargetIdx;
extern volatile uint8_t u8PCMBufferI2STargetIdx;
void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA);

    if (u32Status & PDMA_INTSTS_TDIF_Msk)   /* done */
    {
        if (PDMA_GET_TD_STS(PDMA) & (PDMA_TDSTS_TDIF0_Msk << PDMA_CODEC_TX_CH))
        {
            u8PCMBufferTargetIdx ^= 1;
            aPCMBuffer_Full[u8PCMBufferTargetIdx] = 0;       //set empty flag
            PDMA_CLR_TD_FLAG(PDMA, (PDMA_TDSTS_TDIF0_Msk << PDMA_CODEC_TX_CH));
        }

        if (PDMA_GET_TD_STS(PDMA) & (PDMA_TDSTS_TDIF0_Msk << PDMA_FLASH_RX_CH))
        {
            g_au8PDMAFlashRXFlag = 1;
            PDMA_CLR_TD_FLAG(PDMA, (PDMA_TDSTS_TDIF0_Msk << PDMA_FLASH_RX_CH));
        }
    }
    else if (u32Status & PDMA_INTSTS_REQTOF1_Msk)    /* Timeout */
    {
        PDMA_CLR_TMOUT_FLAG(PDMA, PDMA_CODEC_TX_CH);
    }
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
