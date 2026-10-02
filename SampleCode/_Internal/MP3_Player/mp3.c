/*
 * libmad - MPEG audio decoder library
 * Copyright (C) 2000-2004 Underbit Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * $Id: minimad.c,v 1.4 2004/01/23 09:41:32 rob Exp $
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "mad.h"

#if (USE_SDH == 1)
    #define MP3_FILE    "0:\\test.mp3"
#elif (USE_USBH == 1)
    #define MP3_FILE    "3:\\test.mp3"
#endif

/*
 * This is perhaps the simplest example use of the MAD high-level API.
 * Standard input is mapped into memory via mmap(), then the high-level API
 * is invoked with three callbacks: input, output, and error. The output
 * callback converts MAD's high-resolution PCM samples to 16 bits, then
 * writes them to standard output in little-endian, stereo-interleaved
 * format.
 */


struct mad_stream   Stream;
struct mad_frame    Frame;
struct mad_synth    Synth;

FIL             mp3FileObject;
FILINFO         Finfo;
size_t          ReadSize;
size_t          Remaining;
size_t          ReturnSize;

// I2S PCM buffer x2
// Otherwise, each buffer is not aligned to a full cache line and is not padded
signed int aPCMBuffer[2][PCM_BUFFER_SIZE] = {0};
// File IO buffer for MP3 library
unsigned char MadInputBuffer[FILE_IO_BUFFER_SIZE + MAD_BUFFER_GUARD];
// buffer full flag x2
volatile uint8_t aPCMBuffer_Full[2] = {0, 0};
// audio information structure
struct AudioInfoObject audioInfo;

// Parse MP3 header and get some informations
void MP3_ParseHeaderInfo(uint8_t *pFileName)
{
    FRESULT res;
    uint32_t fptr = 0;

    res = f_open(&mp3FileObject, (void *)pFileName, FA_OPEN_EXISTING | FA_READ);

    if (res == FR_OK)
    {
        printf("file is opened!!\r\n");
        f_stat((void *)pFileName, &Finfo);
        audioInfo.playFileSize = Finfo.fsize;

        while (1)
        {
            res = f_read(&mp3FileObject, (char *)(&MadInputBuffer[0]), FILE_IO_BUFFER_SIZE, &ReturnSize);

            //parsing MP3 header
            mp3CountV1L3Headers((unsigned char *)(&MadInputBuffer[0]), ReturnSize);

            if (audioInfo.mp3SampleRate != 0)
                // Got the header and sampling rate
                break;

            // ID3 may too long, try to parse following data
            // but only forward file point to half of buffer to prevent the header is
            // just right at the boundry of buffer
            fptr += FILE_IO_BUFFER_SIZE / 2;

            if (fptr >= audioInfo.playFileSize)
                // Fail to find header
                break;

            f_lseek(&mp3FileObject, fptr);
        }
    }
    else
    {
        //printf("Open File Error\r\n");
        return;
    }

    f_close(&mp3FileObject);

    printf("====[MP3 Info]======\r\n");
    printf("FileSize = %d\r\n", audioInfo.playFileSize);
    printf("SampleRate = %d\r\n", audioInfo.mp3SampleRate);
    printf("BitRate = %d\r\n", audioInfo.mp3BitRate);
    printf("Channel = %d\r\n", audioInfo.mp3Channel);
    printf("=====================\r\n");
}

// Enable I2S TX with PDMA function
void StartPlay(void)
{
    printf("Start playing ...\n");

    PDMA_Init();

#if (USE_I2S == 1)
    I2S_ENABLE_TXDMA(I2S_PORT);
    I2S_ENABLE_TX(I2S_PORT);
#else
    SPII2S_CLR_TX_FIFO(SPI_PORT);
    SPII2S_ENABLE_TX(SPI_PORT);
    SPII2S_ENABLE_TXDMA(SPI_PORT);
#endif

    // enable sound output
    audioInfo.mp3Playing = 1;
}

// Disable I2S TX with PDMA function
void StopPlay(void)
{
#if (USE_I2S == 1)
    I2S_DISABLE_TXDMA(I2S_PORT);
    I2S_DISABLE_TX(I2S_PORT);
#else
    SPII2S_DISABLE_TXDMA(SPI_PORT);
    SPII2S_DISABLE_TX(SPI_PORT);
#endif

    PDMA_Close(PDMA_PORT);

    // disable sound output
    audioInfo.mp3Playing = 0;

    printf("Stop ...\n");
}

// MP3 decode player
void MP3Player(void)
{
    FRESULT res;
    uint8_t *ReadStart;
    uint8_t *GuardPtr;
    volatile uint8_t u8PCMBufferTargetIdx = 0;
    volatile uint32_t pcmbuf_idx, i;
    volatile unsigned int Mp3FileOffset = 0;
    uint16_t sampleL, sampleR;

    pcmbuf_idx = 0;
    u8PCMBuffer_Playing = 0;
    memset((void *)&audioInfo, 0, sizeof(audioInfo));
    memset((void *)MadInputBuffer, 0, sizeof(MadInputBuffer));
    memset((void *)aPCMBuffer, 0, sizeof(aPCMBuffer));
    memset((void *)aPCMBuffer_Full, 0, sizeof(aPCMBuffer_Full));

    /* Parse MP3 header */
    MP3_ParseHeaderInfo((uint8_t *)MP3_FILE);

    /* First the structures used by libmad must be initialized. */
    mad_stream_init(&Stream);
    mad_frame_init(&Frame);
    mad_synth_init(&Synth);

    /* Open MP3 file */
    res = f_open(&mp3FileObject, MP3_FILE, FA_OPEN_EXISTING | FA_READ);

    if (res != FR_OK)
    {
        //printf("Open file error \r\n");
        return;
    }

#if (!NAU8822)
    /* Reset NAU88L25 codec */
    NAU88L25_Reset();
#endif

    audioInfo.mp3SampleRate = ((audioInfo.mp3SampleRate == 0) ? 48000 : audioInfo.mp3SampleRate);

#if (USE_I2S == 1)
    /* Open I2S interface and set to slave mode, stereo channel, I2S format */
    I2S_Open(I2S_PORT, I2S_MODE_SLAVE, audioInfo.mp3SampleRate, I2S_DATABIT_16, I2S_DISABLE_MONO, I2S_FORMAT_I2S);

    /* Set JK-EN low to enable phone jack on NuMaker board. */
    SET_GPIO_PD1();
    GPIO_SetMode(PD, BIT1, GPIO_MODE_OUTPUT);
    PD1 = 0;

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S_PORT, 12000000);

    I2S_PORT->CTL0 |= I2S_CTL0_ORDER_Msk;

#else

    SPII2S_Open(SPI_PORT, SPII2S_MODE_SLAVE, audioInfo.mp3SampleRate, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK (MCLK can provide clock source to NAU8822) */
    SPII2S_EnableMCLK(SPI_PORT, 12000000);

    SPI_PORT->I2SCTL |= SPI_I2SCTL_ORDER_Msk;

    SPII2S_SET_MONO_RX_CHANNEL(SPI_PORT, SPII2S_MONO_LEFT);       //NAU8822 will store data in left channel
#endif

#if NAU8822
    /* Initialize NAU8822 codec */
    NAU8822_Setup();

    /* Configure NAU8822 to specific sample rate */
    NAU8822_ConfigSampleRate(audioInfo.mp3SampleRate);
#else
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(20000);
    NAU88L25_Setup();

    /* Configure NAU88L25 to specific sample rate */
    NAU88L25_ConfigSampleRate(audioInfo.mp3SampleRate);
#endif

    while (1)
    {
        if (Stream.buffer == NULL || Stream.error == MAD_ERROR_BUFLEN)
        {
            if (Stream.next_frame != NULL)
            {
                /* Get the remaining frame */
                Remaining = Stream.bufend - Stream.next_frame;
                memmove(MadInputBuffer, Stream.next_frame, Remaining);
                ReadStart = MadInputBuffer + Remaining;
                ReadSize = FILE_IO_BUFFER_SIZE - Remaining;
            }
            else
            {
                ReadSize = FILE_IO_BUFFER_SIZE,
                ReadStart = MadInputBuffer,
                Remaining = 0;
            }

            /* read the file */
            res = f_read(&mp3FileObject, ReadStart, ReadSize, &ReturnSize);

            if (res != FR_OK)
            {
                printf("Stop !(%x)\n\r", res);
                goto stop;
            }

            if (f_eof(&mp3FileObject))
            {
                if (ReturnSize == 0)
                    goto stop;
            }

            /* if the file is over */
            if (ReadSize > ReturnSize)
            {
                GuardPtr = ReadStart + ReturnSize;
                memset(GuardPtr, 0, MAD_BUFFER_GUARD);
                ReturnSize += MAD_BUFFER_GUARD;
            }

            Mp3FileOffset = Mp3FileOffset + ReturnSize;
            /* Pipe the new buffer content to libmad's stream decoder
                     * facility.
            */
            mad_stream_buffer(&Stream, MadInputBuffer, ReturnSize + Remaining);
            Stream.error = (enum  mad_error)0;
        }

        /* decode a frame from the mp3 stream data */
        if (mad_frame_decode(&Frame, &Stream))
        {
            if (MAD_RECOVERABLE(Stream.error))
            {
                /*if(Stream.error!=MAD_ERROR_LOSTSYNC ||
                   Stream.this_frame!=GuardPtr)
                {
                }*/
                continue;
            }
            else
            {
                /* the current frame is not full, need to read the remaining part */
                if (Stream.error == MAD_ERROR_BUFLEN)
                {
                    continue;
                }
                else
                {
                    printf("Something error!!\n");

                    /* play the next file */
                    audioInfo.mp3FileEndFlag = 1;
                    goto stop;
                }
            }
        }

        /* Once decoded the frame is synthesized to PCM samples. No errors
        * are reported by mad_synth_frame();
        */
        mad_synth_frame(&Synth, &Frame);

        //
        // decode finished, try to copy pcm data to audio buffer
        //

        if (audioInfo.mp3Playing)
        {
            //if next buffer is still full (playing), wait until it's empty
            if (aPCMBuffer_Full[u8PCMBufferTargetIdx] == 1)
                while (aPCMBuffer_Full[u8PCMBufferTargetIdx]);
        }
        else
        {
            if ((aPCMBuffer_Full[0] == 1) && (aPCMBuffer_Full[1] == 1))         //all buffers are full, wait
            {
                StartPlay();
            }
        }

        for (i = 0; i < (uint32_t)Synth.pcm.length; i++)
        {
            /* Get the left/right samples */
            sampleL = (Synth.pcm.samples[0][i] >> 1);
            sampleR = (Synth.pcm.samples[1][i] >> 1);

            /* Fill PCM data to I2S(PDMA) buffer */
            aPCMBuffer[u8PCMBufferTargetIdx][pcmbuf_idx++] = (sampleR | (sampleL << 16));

            /* Need change buffer ? */
            if (pcmbuf_idx >= PCM_BUFFER_SIZE)
            {
                aPCMBuffer_Full[u8PCMBufferTargetIdx] = 1;      //set full flag
                u8PCMBufferTargetIdx ^= 1;

                pcmbuf_idx = 0;

                //printf("change to ==>%d ..\n", u8PCMBufferTargetIdx);
                /* if next buffer is still full (playing), wait until it's empty */
                if ((aPCMBuffer_Full[u8PCMBufferTargetIdx] == 1) && (audioInfo.mp3Playing))
                    while (aPCMBuffer_Full[u8PCMBufferTargetIdx]) {}
            }
        }
    }

stop:

    printf("Exit MP3\r\n");

    mad_synth_finish(&Synth);
    mad_frame_finish(&Frame);
    mad_stream_finish(&Stream);

    f_close(&mp3FileObject);
    StopPlay();
}
