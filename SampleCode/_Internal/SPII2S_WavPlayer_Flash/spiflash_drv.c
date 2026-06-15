/**************************************************************************//**
 * @file        spiflash_drv.c
 * @brief       SPIM SPI Flash driver
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "spiflash_drv.h"
#include "config.h"
//#include "incbin-main/incbin.h"
#include "wav_sample.h"

#define SPI_FLASH_PORT  QSPI0

//INCBIN(TestMusic, "../16000 (mp3cut.net).wav");

volatile uint8_t g_au8PDMAFlashTXFlag, g_au8PDMAFlashRXFlag;

uint16_t SpiFlash_ReadMidDid(void)
{
    uint8_t u8RxData[6], u8IDCnt = 0;

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x90, Read Manufacturer/Device ID
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x90);

    // send 24-bit '0', dummy
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // receive 16-bit
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    while (!QSPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
        u8RxData[u8IDCnt ++] = QSPI_READ_RX(SPI_FLASH_PORT);

    return ((u8RxData[4] << 8) | u8RxData[5]);
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    QSPI_ClearRxFIFO(SPI_FLASH_PORT);
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    uint8_t u8Val;

    QSPI_ClearRxFIFO(SPI_FLASH_PORT);

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

    // read status
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    u8Val = QSPI_READ_RX(SPI_FLASH_PORT);
    u8Val = QSPI_READ_RX(SPI_FLASH_PORT);

    return u8Val;
}

uint8_t SpiFlash_ReadStatusReg2(void)
{
    uint8_t u8Val;

    QSPI_ClearRxFIFO(SPI_FLASH_PORT);

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x35, Read status register
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x35);

    // read status
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    u8Val = QSPI_READ_RX(SPI_FLASH_PORT);
    u8Val = QSPI_READ_RX(SPI_FLASH_PORT);

    return u8Val;
}

void SpiFlash_WriteStatusReg(uint8_t u8Value1, uint8_t u8Value2)
{
    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x01);

    // write status
    QSPI_WRITE_TX(SPI_FLASH_PORT, u8Value1);
    QSPI_WRITE_TX(SPI_FLASH_PORT, u8Value2);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

void SpiFlash_WaitReady(void)
{
    volatile uint8_t u8ReturnValue;

    do
    {
        u8ReturnValue = SpiFlash_ReadStatusReg();
        u8ReturnValue = u8ReturnValue & 1;
    } while (u8ReturnValue != 0); // check the BUSY bit
}

void SpiFlash_NormalPageProgram(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt = 0;

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);


    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 16) & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 8)  & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    // write data
    while (1)
    {
        if (!QSPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT))
        {
            QSPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[u32Cnt++]);

            if (u32Cnt > 255) break;
        }
    }

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    QSPI_ClearRxFIFO(SPI_FLASH_PORT);
}

void SpiFlash_NormalRead(uint32_t StartAddress, uint8_t *u8DataBuffer, uint32_t u32Len)
{
    uint32_t u32Cnt;

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x03, Read data
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x03);

    // send 24-bit start address
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 16) & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 8)  & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // clear RX buffer
    QSPI_ClearRxFIFO(SPI_FLASH_PORT);

    // read data
    for (u32Cnt = 0; u32Cnt < u32Len; u32Cnt++)
    {
        QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

        while (QSPI_IS_BUSY(SPI_FLASH_PORT));

        u8DataBuffer[u32Cnt] = QSPI_READ_RX(SPI_FLASH_PORT);
    }

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

void SpiFlash_DMA_NormalRead(uint32_t StartAddress)
{
    g_au8PDMAFlashTXFlag = 0;
    g_au8PDMAFlashRXFlag = 0;

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x03, Read data
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x03);

    // send 24-bit start address
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 16) & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 8)  & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // clear RX buffer
    QSPI_ClearRxFIFO(SPI_FLASH_PORT);

    // read data
    QSPI_TRIGGER_TX_RX_PDMA(QSPI0);

    while (g_au8PDMAFlashRXFlag == 0);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    QSPI_DISABLE_TX_RX_PDMA(QSPI0);
}

#define FLASHPAGESIZE 256

void SpiFlash_WriteMusic(void)
{
    uint32_t TEST_LENGTH, TEST_NUMBER;
    uint32_t u32FlashAddress, u32PageNumber;
    uint16_t u16ID;

    TEST_LENGTH = FLASHPAGESIZE;

    //TEST_NUMBER = ((gTestMusicSize / TEST_LENGTH));
    TEST_NUMBER = ((WAV_SAMPLE_LENGTH / TEST_LENGTH));

    if ((u16ID = SpiFlash_ReadMidDid()) != 0xEF14)
    {
        printf("Wrong ID, 0x%x\n", u16ID);

        while (1);
    }
    else
    {
        printf("Flash found: W25X16 ...\n");
    }

    printf("Erase chip ...");

    /* Erase SPI flash */
    SpiFlash_ChipErase();

    /* Wait ready */
    SpiFlash_WaitReady();

    printf("[OK]\n");

    printf("Start to normal write data to Flash ...");
    /* Program SPI flash */
    u32FlashAddress = 0;

    /* page program */
    for (u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page program */
        //SpiFlash_NormalPageProgram(u32FlashAddress, (uint8_t *)&gTestMusicData[u32PageNumber * TEST_LENGTH]);
        SpiFlash_NormalPageProgram(u32FlashAddress, (uint8_t *)&gWAV_SAMPLE[u32PageNumber * TEST_LENGTH]);
        SpiFlash_WaitReady();
        u32FlashAddress += 0x100;
    }

    printf("[Write Page %d done]\n", TEST_NUMBER);
}

void SpiFlash_Open()
{
    uint32_t u32Freq;

    u32Freq = QSPI_Open(SPI_FLASH_PORT, QSPI_MASTER, QSPI_MODE_0, 8, 4000000);

    QSPI_DisableAutoSS(SPI_FLASH_PORT);

    printf("SPI Freq is %dHz\n", u32Freq);
}
