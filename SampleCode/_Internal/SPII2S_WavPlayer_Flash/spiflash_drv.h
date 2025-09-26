/**************************************************************************//**
 * @file        spiflash_drv.h
 * @brief       SPIM SPI Flash driver
 *
 * @note
 * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __SPIFLASH_DRV_H__
#define __SPIFLASH_DRV_H__

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
uint16_t SpiFlash_ReadMidDid(void);
void SpiFlash_ChipErase(void);
uint8_t SpiFlash_ReadStatusReg(void);
uint8_t SpiFlash_ReadStatusReg2(void);
void SpiFlash_WriteStatusReg(uint8_t u8Value1, uint8_t u8Value2);
void SpiFlash_WaitReady(void);
void SpiFlash_NormalPageProgram(uint32_t StartAddress, uint8_t *u8DataBuffer);
void SpiFlash_NormalRead(uint32_t StartAddress, uint8_t *u8DataBuffer, uint32_t u32Len);
void SpiFlash_DMA_NormalRead(uint32_t StartAddress);

void SpiFlash_Open(void);
void SpiFlash_WriteMusic(void);

#endif /* __SPIFLASH_DRV_H__ */
