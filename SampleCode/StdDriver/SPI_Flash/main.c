/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Access SPI flash through SPI interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Fast
#define SlewRateMode    1
// <c1> Enable SPI Optimize
// <i> Use FIFO mechanism for QSPI RX to maximize throughput
#define ENABLE_SPI_OPTIMIZE
// </c>
// *** <<< end of configuration section >>> ***

#define TEST_NUMBER     1   /* page numbers */
#define TEST_LENGTH     256 /* length */

#define SPI_FLASH_PORT  SPI0
#define SPI_FLASH_IO_TIMEOUT     (SystemCoreClock)
#define SPI_FLASH_READY_TIMEOUT  (SystemCoreClock * 8UL)

//------------------------------------------------------------------------------
uint8_t g_au8SrcArray[TEST_LENGTH];
uint8_t g_au8DestArray[TEST_LENGTH];

//------------------------------------------------------------------------------
static int32_t SpiFlash_WaitIdle(void)
{
    uint32_t u32Timeout = SPI_FLASH_IO_TIMEOUT;

    while (SPI_IS_BUSY(SPI_FLASH_PORT))
    {
        if (u32Timeout-- == 0UL)
        {
            return -1;
        }
    }

    return 0;
}

int32_t SpiFlash_ReadMidDid(uint16_t *pu16MidDid)
{
    uint8_t u8RxData[6], u8IDCnt = 0;

    SPI_ClearRxFIFO(SPI_FLASH_PORT);

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x90, Read Manufacturer/Device ID
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x90);

    // send 24-bit '0', dummy
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // receive 16-bit
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    while (!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
    {
        if (u8IDCnt >= sizeof(u8RxData))
        {
            SPI_ClearRxFIFO(SPI_FLASH_PORT);
            return -1;
        }

        u8RxData[u8IDCnt++] = SPI_READ_RX(SPI_FLASH_PORT);
    }

    if (u8IDCnt != sizeof(u8RxData))
    {
        return -1;
    }

    *pu16MidDid = ((u8RxData[4] << 8) | u8RxData[5]);
    return 0;
}

int32_t SpiFlash_ChipErase(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    SPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI0);
    return 0;
}

int32_t SpiFlash_ReadStatusReg(uint8_t *pu8Status)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

    // read status
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    SPI_READ_RX(SPI_FLASH_PORT);

    *pu8Status = (SPI_READ_RX(SPI_FLASH_PORT) & 0xff);
    return 0;
}

int32_t SpiFlash_WriteStatusReg(uint8_t u8Value)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x01);

    // write status
    SPI_WRITE_TX(SPI_FLASH_PORT, u8Value);

    // wait tx finish
    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
    return 0;
}

int32_t SpiFlash_WaitReady(void)
{
    uint8_t u8ReturnValue;
    uint32_t u32Timeout = SPI_FLASH_READY_TIMEOUT;

    do
    {
        if (SpiFlash_ReadStatusReg(&u8ReturnValue) < 0)
        {
            return -1;
        }

        if (u32Timeout-- == 0UL)
        {
            return -1;
        }

        u8ReturnValue &= 1U;
    } while (u8ReturnValue != 0); // check the BUSY bit

    return 0;
}

int32_t SpiFlash_NormalPageProgram(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt = 0;
    uint32_t u32Timeout = SPI_FLASH_IO_TIMEOUT;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);


    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    // write data
    while (u32Cnt < TEST_LENGTH)
    {
        if (!SPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT))
        {
            SPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[u32Cnt++]);
            u32Timeout = SPI_FLASH_IO_TIMEOUT;
        }
        else if (u32Timeout-- == 0UL)
        {
            SPI_SET_SS_HIGH(SPI_FLASH_PORT);
            return -1;
        }
    }

    // wait tx finish
    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
    return 0;
}

int32_t SpiFlash_NormalRead(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
#if !defined(ENABLE_SPI_OPTIMIZE)
    uint32_t u32Cnt;
#else
    uint32_t u32RxDataWord = 64;
    uint32_t u32RxTmp;
    uint32_t u32TxDataCount = 0;
    uint32_t u32RxDataCount = 0;
    uint32_t u32Timeout = SPI_FLASH_READY_TIMEOUT;
#endif

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x03, Read data
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x03);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // clear RX buffer
    SPI_ClearRxFIFO(SPI_FLASH_PORT);

#ifdef ENABLE_SPI_OPTIMIZE
    SPI_SET_DATA_WIDTH(SPI_FLASH_PORT, 32);
    SPI_ENABLE_BYTE_REORDER(SPI_FLASH_PORT);

    while (u32RxDataCount < 256)
    {
        uint32_t u32Progress = 0U;

        /* Check TX FULL flag */
        if ((SPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT) == 0) && (u32TxDataCount < u32RxDataWord))
        {
            SPI_WRITE_TX(SPI_FLASH_PORT, 0xFFFFFFFF);
            u32TxDataCount++;
            u32Progress = 1U;
        }

        /* Check RX EMPTY flag */
        if (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT) == 0)
        {
            u32RxTmp = SPI_READ_RX(SPI_FLASH_PORT);
            memcpy(&u8DataBuffer[u32RxDataCount], &u32RxTmp, 4);
            u32RxDataCount += 4;
            u32Progress = 1U;
        }

        if (u32Progress != 0U)
        {
            u32Timeout = SPI_FLASH_READY_TIMEOUT;
        }
        else if (u32Timeout-- == 0UL)
        {
            SPI_DISABLE_BYTE_REORDER(SPI_FLASH_PORT);
            SPI_SET_DATA_WIDTH(SPI_FLASH_PORT, 8);
            SPI_SET_SS_HIGH(SPI_FLASH_PORT);
            return -1;
        }
    }

    SPI_DISABLE_BYTE_REORDER(SPI_FLASH_PORT);
    SPI_SET_DATA_WIDTH(SPI_FLASH_PORT, 8);
#else

    // read data
    for (u32Cnt = 0; u32Cnt < 256; u32Cnt++)
    {
        SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

        if (SpiFlash_WaitIdle() < 0)
        {
            SPI_SET_SS_HIGH(SPI_FLASH_PORT);
            return -1;
        }

        u8DataBuffer[u32Cnt] = SPI_READ_RX(SPI_FLASH_PORT);
    }

#endif

    // wait tx finish
    if (SpiFlash_WaitIdle() < 0)
    {
        SPI_SET_SS_HIGH(SPI_FLASH_PORT);
        return -1;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
    return 0;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set UART Default MPF */
    UartDebugMFP();

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_SPI0_CLK | SYS_GPB_MFPH_PB15MFP_SPI0_SS;

    /* Enable SPI0 clock pin (PB14) schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN14_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

#if (SlewRateMode == 0)
    /* Enable SPI0 I/O normal slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_NORMAL);
#elif (SlewRateMode == 1)
    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_HIGH);
#elif (SlewRateMode == 2)
    /* Enable SPI0 I/O fast slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_FAST);
#endif
}

/* Main */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t u32Error = 0;
    uint16_t u16ID;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART */
    UartDebugInit();

    /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 20MHz */
    if (SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8, 200000) == 0U)
    {
        printf("SPI_Open failed.\n");
        goto sample_exit;
    }

    /* Disable auto SS function, control SS signal manually. */
    SPI_DisableAutoSS(SPI_FLASH_PORT);

    printf("\n+------------------------------------------------------------------------+\n");
    printf("|                      SPI Sample with SPI Flash                         |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf(" NOTE: This sample uses bounded waits for demo recovery only. Add application-specific timeout and authentication policy for production use.\n");

    if (SpiFlash_ReadMidDid(&u16ID) < 0)
    {
        printf("Read flash ID timed out.\n");
        goto sample_exit;
    }

    if (u16ID == 0xEF13)
        printf("Flash found: W25Q80 ...\n");
    else if (u16ID == 0xEF14)
        printf("Flash found: W25Q16 ...\n");
    else if (u16ID == 0xEF15)
        printf("Flash found: W25Q32 ...\n");
    else if (u16ID == 0xEF16)
        printf("Flash found: W25Q64 ...\n");
    else if (u16ID == 0xEF17)
        printf("Flash found: W25Q128 ...\n");
    else
    {
        printf("Wrong ID, 0x%x\n", u16ID);
        goto sample_exit;
    }

    printf("Erase chip ...");

    /* Erase SPI flash */
    if (SpiFlash_ChipErase() < 0)
    {
        printf("[FAIL] timeout\n");
        goto sample_exit;
    }

    /* Wait ready */
    if (SpiFlash_WaitReady() < 0)
    {
        printf("[FAIL] timeout\n");
        goto sample_exit;
    }

    printf("[OK]\n");

    /* init source data buffer */
    for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        g_au8SrcArray[u32ByteCount] = u32ByteCount;
    }

    printf("Start to normal write data to Flash ...");
    /* Program SPI flash */
    u32FlashAddress = 0;

    for (u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page program */
        if (SpiFlash_NormalPageProgram(u32FlashAddress, g_au8SrcArray) < 0)
        {
            printf("[FAIL] timeout\n");
            goto sample_exit;
        }

        if (SpiFlash_WaitReady() < 0)
        {
            printf("[FAIL] timeout\n");
            goto sample_exit;
        }

        u32FlashAddress += 0x100;
    }

    printf("[OK]\n");

    /* clear destination data buffer */
    for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        g_au8DestArray[u32ByteCount] = 0;
    }

    printf("Normal Read & Compare ...");

    /* Read SPI flash */
    u32FlashAddress = 0;

    for (u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page read */
        if (SpiFlash_NormalRead(u32FlashAddress, g_au8DestArray) < 0)
        {
            printf("[FAIL] timeout\n");
            goto sample_exit;
        }

        u32FlashAddress += 0x100;

        for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
        {
            if (g_au8DestArray[u32ByteCount] != g_au8SrcArray[u32ByteCount])
                u32Error ++;
        }
    }

    if (u32Error == 0)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

sample_exit:
    SPI_Close(SPI_FLASH_PORT);

    while (1) {}
}
