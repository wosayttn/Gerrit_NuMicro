/**************************************************************************//**
 * @file     config.h
 * @version  V3.00
 * @brief    I2S MP3 recorder sample configuration header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

/*---------------------------------------------------------------------------*/
/* Define                                                                    */
/*---------------------------------------------------------------------------*/
#ifndef USE_SDH
    #define USE_SDH             1
#endif

#define USE_I2S                 1

#if (USE_I2S == 1)
    #define I2S_PORT            I2S0
    #define I2S_IRQ             MP3_CONCAT3(I2S, 0, _IRQn)
    #define I2C_PORT            I2C3
#else
    #define SPI_PORT            SPI0
    #define SPI_IRQ             MP3_CONCAT3(SPI, 0, _IRQn)
    #define I2C_PORT            I2C1
#endif

#define SPIM_PORT               SPIM0

#define PDMA_PORT               PDMA0
#define PDMA_IRQ                MP3_CONCAT3(PDMA, 0, _IRQn)
#define I2S_TX_DMA_CH           2

/* Defined when online recording; undefined when offline recording. */
#define REC_IN_RT

#define MP3_CAT2_INNER(a, b)    a##b
#define MP3_CONCAT2(a, b)       MP3_CAT2_INNER(a, b)

#define MP3_CAT3_INNER(a, b, c) a##b##c
#define MP3_CONCAT3(a, b, c)    MP3_CAT3_INNER(a, b, c)

/*
For shine MP3 encoder, the following cases are supported when audio format is mono mode.
If user wants to implement stereo mode, the sampling rate and bit rate can reach 48 kHz and 192 kbps.
 --------------------------------------
| Sampling rate (Hz) | Bit rate (kbps) |
|--------------------------------------|
|        32000       |      <= 128     |
|--------------------------------------|
|        16000       |      <=  64     |
|--------------------------------------|
|         8000       |      <=  32     |
 --------------------------------------
*/
#define REC_FORMAT              I2S_MONO    /* The record audio format. */
#define REC_SAMPLE_RATE         16000       /* The record sampling rate. */
#define REC_BIT_RATE            64          /* The record bit rate. */

#define PLAY_FORMAT             REC_FORMAT  /* The play audio format. Must be the same with REC_FORMAT */

#if (USE_SDH == 1)
    #define MP3_FILE            "0:\\recorder.mp3"
#else
    #define MP3_FILE            "3:\\recorder.mp3"
#endif

#define PCM_BUFFER_SIZE         2304
#define FILE_IO_BUFFER_SIZE     4096

struct mp3Header
{
    unsigned int sync : 11;
    unsigned int version : 2;
    unsigned int layer : 2;
    unsigned int protect : 1;
    unsigned int bitrate : 4;
    unsigned int samfreq : 2;
    unsigned int padding : 1;
    unsigned int private : 1;
    unsigned int channel : 2;
    unsigned int mode : 2;
    unsigned int copy : 1;
    unsigned int original : 1;
    unsigned int emphasis : 2;
};

struct AudioInfoObject
{
    unsigned int playFileSize;
    unsigned int mp3FileEndFlag;
    unsigned int mp3SampleRate;
    unsigned int mp3BitRate;
    unsigned int mp3Channel;
    unsigned int mp3PlayTime;
    unsigned int mp3Playing;
};

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

extern volatile uint32_t g_au32PcmBuff1[PCM_BUFFER_SIZE];
extern volatile uint32_t g_au32PcmBuff2[PCM_BUFFER_SIZE];

extern volatile uint32_t g_u32RecordDone;
extern volatile uint32_t g_u32BuffPos;
extern volatile uint32_t g_u32BuffPos1, g_u32BuffPos2;
extern volatile uint32_t g_u32WriteSDToggle;
extern volatile uint32_t g_u32ErrorFlag;

extern int32_t g_ai32PCMBuffer[2][PCM_BUFFER_SIZE];
extern volatile uint8_t g_au8PCMBuffer_Full[2];
extern volatile uint8_t g_u8PCMBuffer_Playing;

void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data);
void NAU8822_ConfigSampleRate(uint32_t u32SampleRate);
void NAU8822_Setup(void);

void Recorder_Init(void);
void MP3Recorder(void);
int32_t Write_MP3(long bytes, void *buffer);

int32_t mp3CountV1L3Headers(unsigned char *pu8Bytes, size_t size);
void PDMA_Init(void);
void MP3Player(void);

#endif
