/**************************************************************************//**
 * @file     wavplayer.c
 * @brief    M252 I2S Driver Sample Code
 *
 * @note
 * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "NuMicro.h"
#include "config.h"
#include "spiflash_drv.h"
#include "codec_drv.h"

/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/

uint32_t aWavHeader[11];
uint8_t bAudioPlaying = 0;
uint16_t count_frame = 0;
volatile uint8_t u8PCMBufferTargetIdx = 0;
volatile uint8_t u8PCMBufferI2STargetIdx = 0;

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
int ReadFlashPlay(uint32_t FlashAddr)
{
    uint32_t u32WavSamplingRate, AudioFormat;
    int PlayFrameSize, ChannelNum, WavRealSize/*, I2SBit, I2SBitFormat*/;

    /* Clear default parameter*/
    count_frame = 0;
    bAudioPlaying = 0;
    aPCMBuffer_Full[0] = 0;
    aPCMBuffer_Full[1] = 0;

    /* Parsing WAV file header */
    memset(aWavHeader, 0, sizeof(aWavHeader));
    SpiFlash_NormalRead(FlashAddr, (uint8_t *)&aWavHeader[0], 44);

    /* Read sampling rate from WAV header */
    u32WavSamplingRate = aWavHeader[6];

    /* Check if the WAV file is correct */
    if (aWavHeader[0] != 0x46464952 | aWavHeader[2] != 0x45564157)
    {
        printf("WAV File ERROR, please store Wav file at 0x%08x\n", FlashAddr);
        return -1;
    }

    /* Read WAV file total size from WAV header */
    PlayFrameSize = aWavHeader[1] / (PCM_BUFFER_SIZE);
    WavRealSize = aWavHeader[1] / 1024;

    /* Read Channel number from WAV header */
    ChannelNum = (aWavHeader[5] & 0x00030000) >> 16;
    AudioFormat = ChannelNum == 1 ? SPII2S_MONO : SPII2S_STEREO;

    printf("wav fie pre-stored in internal SPI Flash address from 0x%08x\n", FlashAddr);
    printf("wav: sampling rate = %d\n", u32WavSamplingRate);
    printf("wav: Data size = %d KB\n", WavRealSize);
    printf("wav: Channel Num = %d\n", ChannelNum);

    /* SPII2S setting */
    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, u32WavSamplingRate, SPII2S_DATABIT_16, AudioFormat, SPII2S_FORMAT_I2S);
    SPII2S_EnableMCLK(SPI0, 12000000);

    /* Initialize NAU88L25 CODEC */
    NAU8822_Setup(u32WavSamplingRate, AudioFormat);

    u8PCMBufferTargetIdx = 0;

    while (1)
    {
        /* Set ping-pong buffer to transmit data */
        if ((aPCMBuffer_Full[0] == 1) && (aPCMBuffer_Full[1] == 1))         //all buffers are full, wait
        {
            if (!bAudioPlaying)
            {
                bAudioPlaying = 1;
                SPII2S_ENABLE_TXDMA(SPI0);
                SPII2S_ENABLE_TX(SPI0);
                printf("Start Playing ...\n");
            }

            while ((aPCMBuffer_Full[0] == 1) && (aPCMBuffer_Full[1] == 1));
        }

        PDMA_Flash_Init(&aPCMBuffer[u8PCMBufferTargetIdx][0]);
        /* Read WAV data from internal flash */
        SpiFlash_DMA_NormalRead(FlashAddr + 78);

        /* Move address pointer to next address */
        FlashAddr = FlashAddr + PCM_BUFFER_SIZE;
        count_frame++;

        /* Set buffer full flag */
        aPCMBuffer_Full[u8PCMBufferTargetIdx] = 1;

        if (bAudioPlaying)
        {
            /* Make sure the current ping-pong buffer is empty */
            /* Break if the last audio data played */
            if (count_frame >= (PlayFrameSize + 1))
            {
                goto stop;
            }
        }

        /* Switch to another buffer */
        if (!bAudioPlaying)
            u8PCMBufferTargetIdx ^= 1;
    }

stop:
    SPII2S_DISABLE_TX(SPI0);
    SPII2S_DISABLE_TXDMA(SPI0);

    printf("Play finish... enter while 1\n");
    return 0;
}
