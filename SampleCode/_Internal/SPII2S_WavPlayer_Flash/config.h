/******************************************************************************
 * @file     config.h
 * @brief    Sample code configuration file
 *
 * @note
 * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "NuMicro.h"
/*---------------------------------------------------------------------------*/
/* Define                                                                    */
/*---------------------------------------------------------------------------*/
#define PCM_BUFFER_SIZE 8192
#define WAVFile_address 0x2000

#define PDMA_CODEC_TX_CH   0
#define PDMA_CODEC_RX_CH   3
#define PDMA_FLASH_TX_CH   1
#define PDMA_FLASH_RX_CH   2


typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t NEXT;
} DESC_TABLE_T;

extern DESC_TABLE_T g_asDescTableCodec_TX[2], g_asDescTableCodec_RX[2], g_asDescTableFlash_TX[2], g_asDescTableFlash_RX[2];
extern volatile uint8_t g_au8PDMAFlashTXFlag, g_au8PDMAFlashRXFlag;
extern volatile uint8_t u8PCMBufferTargetIdx;

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
extern uint8_t aPCMBuffer[2][PCM_BUFFER_SIZE];
extern volatile uint8_t aPCMBuffer_Full[2];

void PDMA_Flash_Init(uint8_t *u8DataBuffer);
void WAVPlayer(void);
void InternalCODEC_Setup(void);
int ReadFlashPlay(uint32_t FlashAddr);

#endif

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
