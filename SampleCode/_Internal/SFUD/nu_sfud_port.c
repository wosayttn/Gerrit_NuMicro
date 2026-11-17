/*
 * This file is part of the Serial Flash Universal Driver Library.
 *
 * Copyright (C) 2016-2018, Armink, <armink.ztl@gmail.com>
 * Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2016-04-23
 */

/* MISRA C:2012 Rule 17.1 deviation: the external SFUD logging ABI requires
    variadic `sfud_log_debug` and `sfud_log_info` implementations. */
#include "stdarg.h"
#include "NuMicro.h"
#include "sfud.h"
#include "nu_sfud_port.h"

//------------------------------------------------------------------------------
// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Faster0 <3=> Faster1
#define SlewRateMode        2
// *** <<< end of configuration section >>> ***

#define SPI_FLASH_PORT           (SPI0)
#define QSPI_FLASH_PORT          (QSPI0)
#define SPIM_FLASH_PORT          (SPIM0)
#define USPI_FLASH_PORT          (USPI0)
#define LPSPI_FLASH_PORT         (LPSPI0)

#define SFUD_FLASH_RETRY_TIMES   (30000UL)  /* 100 us x 30000 = 3 s */
#define SFUD_GPIO_PORT(port)     (((__PC() & NS_OFFSET) != 0U) ? (port##_NS) : (port##_S))

//------------------------------------------------------------------------------
static char log_buf[256];

//------------------------------------------------------------------------------
void sfud_log_debug(const char *file, const long line, const char *format, ...);
void sfud_log_info(const char *format, ...);

static void SPI_SS_Low(void *spi);
static void SPI_SS_High(void *spi);
static void SPI_Write_Tx(void *spi, uint8_t data);
static uint8_t SPI_Read_Rx(void *spi);
bool SPI_Is_Busy(void *spi);
bool SPI_Tx_Fifo_empty(void *spi);

static void QSPI_SS_Low(void *spi);
static void QSPI_SS_High(void *spi);
static void QSPI_Write_Tx(void *spi, uint8_t data);
static uint8_t QSPI_Read_Rx(void *spi);
bool QSPI_Is_Busy(void *spi);
bool QSPI_Tx_Fifo_empty(void *spi);
uint8_t QSPI_Flash_ReadStatusReg(QSPI_T *qspi);
uint8_t QSPI_Flash_ReadStatusReg2(QSPI_T *qspi);
void QSPI_Flash_WriteStatusReg(QSPI_T *qspi, uint8_t u8Value1, uint8_t u8Value2);
int32_t QSPI_Flash_WaitReady(QSPI_T *qspi);
void QSPI_Flash_EnableQEBit(QSPI_T *qspi);
void QSPI_Flash_DisableQEBit(QSPI_T *qspi);

static void USPI_SS_Low(void *spi);
static void USPI_SS_High(void *spi);
static void USPI_Write_Tx(void *spi, uint8_t data);
static uint8_t USPI_Read_Rx(void *spi);
bool USPI_Is_Busy(void *spi);
bool USPI_Tx_Fifo_empty(void *spi);
static sfud_err USPI_BurstTransfer(const sfud_spi *spi, const uint8_t *write_buf,
                                   size_t write_size, uint8_t *read_buf, size_t read_size);

static void LPSPI_SS_Low(void *spi);
static void LPSPI_SS_High(void *spi);
static void LPSPI_Write_Tx(void *spi, uint8_t data);
static uint8_t LPSPI_Read_Rx(void *spi);
static bool LPSPI_Is_Busy(void *spi);
static bool LPSPI_Tx_Fifo_empty(void *spi);

static sfud_err SFUD_WaitWhileBusy(const sfud_spi *spi)
{
    uint32_t u32RetryTimes = 1000UL;

    while (spi->isbusy(spi->user_module) == true)
    {
        if (u32RetryTimes == 0UL)
        {
            return SFUD_ERR_TIMEOUT;
        }

        u32RetryTimes--;
    }

    return SFUD_SUCCESS;
}

//------------------------------------------------------------------------------
static void spi_lock(const sfud_spi *spi)
{
    (void)spi;

    __disable_irq();
}

static void spi_unlock(const sfud_spi *spi)
{
    (void)spi;

    __enable_irq();
}

//------------------------------------------------------------------------------
// SPI API
//------------------------------------------------------------------------------
/**
 * SPI write data then read data
 */
static sfud_err spi_write_read(const sfud_spi *spi, const uint8_t *write_buf,
                               size_t write_size, uint8_t *read_buf, size_t read_size)
{
    sfud_err result = SFUD_SUCCESS;
    uint8_t send_data, read_data;

    if ((write_size != 0U) && (write_buf == NULL))
    {
        return SFUD_ERR_WRITE;
    }

    if ((read_size != 0U) && (read_buf == NULL))
    {
        return SFUD_ERR_READ;
    }

    /**
     * add your spi write and read code
     */
    // /CS: active
    spi->ss_low(spi->user_module);

    /* Start reading and writing data */
    for (size_t i = 0U; i < (write_size + read_size); i++)
    {
        /* First write the data from the buffer to the SPI bus; after writing, send a dummy (0xFF) to the SPI bus */
        if ((i < write_size) && (write_buf != NULL))
        {
            send_data = *write_buf++;

            if (spi->sel_dir_out != NULL)
            {
                spi->sel_dir_out(spi->user_module);
            }
        }
        else
        {
            send_data = SFUD_DUMMY_DATA;

            if (spi->sel_dir_in != NULL)
            {
                spi->sel_dir_in(spi->user_module);
            }
        }

        /* Send data */
        result = SFUD_WaitWhileBusy(spi);

        if (result != SFUD_SUCCESS)
        {
            goto exit;
        }

        spi->tx(spi->user_module, send_data);
        /* Receive data */
        result = SFUD_WaitWhileBusy(spi);

        if (result != SFUD_SUCCESS)
        {
            goto exit;
        }

        read_data = spi->rx(spi->user_module);

        /* After sending the data from the write buffer, read the data from the SPI bus into the read buffer */
        if ((read_buf != NULL) && (i >= write_size))
        {
            *read_buf++ = read_data;
        }
    }

exit:
    // /CS: de-active
    spi->ss_high(spi->user_module);

    return result;
}

void SPI_SS_Low(void *spi)
{
    SPI_SET_SS_LOW((SPI_T *)spi);
}

void SPI_SS_High(void *spi)
{
    SPI_SET_SS_HIGH((SPI_T *)spi);
}

void SPI_Write_Tx(void *spi, uint8_t data)
{
    SPI_WRITE_TX((SPI_T *)spi, data);
}

uint8_t SPI_Read_Rx(void *spi)
{
    return (uint8_t)SPI_READ_RX((SPI_T *)spi);
}

bool SPI_Is_Busy(void *spi)
{
    return (SPI_IS_BUSY((SPI_T *)spi) != 0U);
}

bool SPI_Tx_Fifo_empty(void *spi)
{
    return (SPI_GET_TX_FIFO_EMPTY_FLAG((SPI_T *)spi) != 0U);
}

//------------------------------------------------------------------------------
// QSPI API
//------------------------------------------------------------------------------
#ifdef SFUD_USING_QSPI
static void D2D3_SwitchToNormalMode(void)
{
    SYS->GPA_MFP1 = SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk);
    GPIO_SetMode(SFUD_GPIO_PORT(PA), BIT4, GPIO_MODE_OUTPUT);
    GPIO_SetMode(SFUD_GPIO_PORT(PA), BIT5, GPIO_MODE_OUTPUT);
    SFUD_GPIO_PORT(PA)->DOUT |= (BIT4 | BIT5);
}

static void D2D3_SwitchToQuadMode(void)
{
    SET_QSPI0_MOSI1_PA4();
    SET_QSPI0_MISO1_PA5();
}

static sfud_err QSPI_WaitBusy(QSPI_T *qspi)
{
    uint32_t u32TimeOutCnt = SystemCoreClock;

    while (QSPI_IS_BUSY(qspi) != 0U)
    {
        if (--u32TimeOutCnt == 0U)
        {
            return SFUD_ERR_TIMEOUT;
        }
    }

    return SFUD_SUCCESS;
}

static sfud_err QSPI_SetOutputLines(QSPI_T *qspi, uint8_t u8Lines)
{
    sfud_err result = SFUD_SUCCESS;

    QSPI_DISABLE_DUAL_MODE(qspi);
    QSPI_DISABLE_QUAD_MODE(qspi);

    switch (u8Lines)
    {
        case 1U:
            break;

        case 2U:
            QSPI_ENABLE_DUAL_OUTPUT_MODE(qspi);
            break;

        case 4U:
            D2D3_SwitchToQuadMode();
            QSPI_ENABLE_QUAD_OUTPUT_MODE(qspi);
            break;

        default:
            result = SFUD_ERR_READ;
            break;
    }

    return result;
}

static sfud_err QSPI_SetInputLines(QSPI_T *qspi, uint8_t u8Lines)
{
    sfud_err result = SFUD_SUCCESS;

    QSPI_DISABLE_DUAL_MODE(qspi);
    QSPI_DISABLE_QUAD_MODE(qspi);

    switch (u8Lines)
    {
        case 1U:
            break;

        case 2U:
            QSPI_ENABLE_DUAL_INPUT_MODE(qspi);
            break;

        case 4U:
            D2D3_SwitchToQuadMode();
            QSPI_ENABLE_QUAD_INPUT_MODE(qspi);
            break;

        default:
            result = SFUD_ERR_READ;
            break;
    }

    return result;
}

/**
 * Read flash data using the line widths selected by SFUD.
 */
static sfud_err qspi_read(const struct __sfud_spi *spi, uint32_t addr,
                          sfud_qspi_read_cmd_format *qspi_read_cmd_format,
                          uint8_t *read_buf, size_t read_size)
{
    QSPI_T *qspi;
    sfud_err result = SFUD_SUCCESS;
    uint32_t u32Cnt;
    uint32_t u32DummyBits;
    uint32_t u32DummyBytes;
    bool bUseQuad;

    if ((spi == NULL) || (spi->user_module == NULL) ||
            (qspi_read_cmd_format == NULL) ||
            ((read_size != 0U) && (read_buf == NULL)))
    {
        return SFUD_ERR_READ;
    }

    if ((qspi_read_cmd_format->address_size != 24U) &&
            (qspi_read_cmd_format->address_size != 32U))
    {
        return SFUD_ERR_READ;
    }

    if (qspi_read_cmd_format->alternate_bytes_lines != 0U)
    {
        return SFUD_ERR_READ;
    }

    qspi = (QSPI_T *)spi->user_module;
    bUseQuad = ((qspi_read_cmd_format->instruction_lines == 4U) ||
                (qspi_read_cmd_format->address_lines == 4U) ||
                (qspi_read_cmd_format->data_lines == 4U));

    /* Winbond QE is only required when a Quad phase is actually used. */
    if (bUseQuad == true)
    {
        QSPI_Flash_EnableQEBit(qspi);
    }

    QSPI_DISABLE_DUAL_MODE(qspi);
    QSPI_DISABLE_QUAD_MODE(qspi);
    D2D3_SwitchToNormalMode();

    QSPI_SET_SS_LOW(qspi);

    result = QSPI_SetOutputLines(qspi, qspi_read_cmd_format->instruction_lines);

    if (result != SFUD_SUCCESS)
    {
        goto exit;
    }

    QSPI_WRITE_TX(qspi, qspi_read_cmd_format->instruction);
    result = QSPI_WaitBusy(qspi);

    if (result != SFUD_SUCCESS)
    {
        goto exit;
    }

    result = QSPI_SetOutputLines(qspi, qspi_read_cmd_format->address_lines);

    if (result != SFUD_SUCCESS)
    {
        goto exit;
    }

    if (qspi_read_cmd_format->address_size == 32U)
    {
        QSPI_WRITE_TX(qspi, (addr >> 24) & 0xFFU);
    }

    QSPI_WRITE_TX(qspi, (addr >> 16) & 0xFFU);
    QSPI_WRITE_TX(qspi, (addr >> 8) & 0xFFU);
    QSPI_WRITE_TX(qspi, addr & 0xFFU);

    /* SFUD describes dummy time in clock cycles. The dummy phase follows
       the address line width for the read formats used by SFUD v1.1.0. */
    u32DummyBits = (uint32_t)qspi_read_cmd_format->dummy_cycles *
                   (uint32_t)qspi_read_cmd_format->address_lines;

    if ((u32DummyBits & 0x7UL) != 0UL)
    {
        result = SFUD_ERR_READ;
        goto exit;
    }

    u32DummyBytes = u32DummyBits / 8UL;

    for (u32Cnt = 0UL; u32Cnt < u32DummyBytes; u32Cnt++)
    {
        QSPI_WRITE_TX(qspi, 0x00U);
    }

    result = QSPI_WaitBusy(qspi);

    if (result != SFUD_SUCCESS)
    {
        goto exit;
    }

    QSPI_ClearRxFIFO(qspi);

    result = QSPI_SetInputLines(qspi, qspi_read_cmd_format->data_lines);

    if (result != SFUD_SUCCESS)
    {
        goto exit;
    }

    for (u32Cnt = 0UL; u32Cnt < read_size; u32Cnt++)
    {
        QSPI_WRITE_TX(qspi, SFUD_DUMMY_DATA);
        result = QSPI_WaitBusy(qspi);

        if (result != SFUD_SUCCESS)
        {
            goto exit;
        }

        read_buf[u32Cnt] = (uint8_t)QSPI_READ_RX(qspi);
    }

exit:
    QSPI_SET_SS_HIGH(qspi);
    QSPI_DISABLE_DUAL_MODE(qspi);
    QSPI_DISABLE_QUAD_MODE(qspi);
    D2D3_SwitchToNormalMode();

    return result;
}

uint8_t QSPI_Flash_ReadStatusReg(QSPI_T *qspi)
{
    uint8_t u8Val;

    QSPI_DISABLE_DUAL_MODE(qspi);
    QSPI_DISABLE_QUAD_MODE(qspi);
    D2D3_SwitchToNormalMode();
    QSPI_ClearRxFIFO(qspi);

    QSPI_SET_SS_LOW(qspi);
    QSPI_WRITE_TX(qspi, 0x05U);
    QSPI_WRITE_TX(qspi, 0x00U);
    (void)QSPI_WaitBusy(qspi);
    QSPI_SET_SS_HIGH(qspi);

    (void)QSPI_READ_RX(qspi);
    u8Val = (uint8_t)QSPI_READ_RX(qspi);

    return u8Val;
}

uint8_t QSPI_Flash_ReadStatusReg2(QSPI_T *qspi)
{
    uint8_t u8Val;

    QSPI_DISABLE_DUAL_MODE(qspi);
    QSPI_DISABLE_QUAD_MODE(qspi);
    D2D3_SwitchToNormalMode();
    QSPI_ClearRxFIFO(qspi);

    QSPI_SET_SS_LOW(qspi);
    QSPI_WRITE_TX(qspi, 0x35U);
    QSPI_WRITE_TX(qspi, 0x00U);
    (void)QSPI_WaitBusy(qspi);
    QSPI_SET_SS_HIGH(qspi);

    (void)QSPI_READ_RX(qspi);
    u8Val = (uint8_t)QSPI_READ_RX(qspi);

    return u8Val;
}

void QSPI_Flash_WriteStatusReg(QSPI_T *qspi, uint8_t u8Value1, uint8_t u8Value2)
{
    QSPI_DISABLE_DUAL_MODE(qspi);
    QSPI_DISABLE_QUAD_MODE(qspi);
    D2D3_SwitchToNormalMode();

    QSPI_SET_SS_LOW(qspi);
    QSPI_WRITE_TX(qspi, 0x06U);
    (void)QSPI_WaitBusy(qspi);
    QSPI_SET_SS_HIGH(qspi);

    QSPI_SET_SS_LOW(qspi);
    QSPI_WRITE_TX(qspi, 0x01U);
    QSPI_WRITE_TX(qspi, u8Value1);
    QSPI_WRITE_TX(qspi, u8Value2);
    (void)QSPI_WaitBusy(qspi);
    QSPI_SET_SS_HIGH(qspi);
}

int32_t QSPI_Flash_WaitReady(QSPI_T *qspi)
{
    uint8_t u8ReturnValue;
    uint32_t u32TimeOutCnt = SystemCoreClock;

    do
    {
        if (--u32TimeOutCnt == 0U)
        {
            return -1;
        }

        u8ReturnValue = QSPI_Flash_ReadStatusReg(qspi);
        u8ReturnValue &= 0x01U;
    } while (u8ReturnValue != 0U);

    return 0;
}

void QSPI_Flash_EnableQEBit(QSPI_T *qspi)
{
    uint8_t u8Status1 = QSPI_Flash_ReadStatusReg(qspi);
    uint8_t u8Status2 = QSPI_Flash_ReadStatusReg2(qspi);

    if ((u8Status2 & 0x02U) != 0U)
    {
        return;
    }

    u8Status2 |= 0x02U;
    QSPI_Flash_WriteStatusReg(qspi, u8Status1, u8Status2);
    (void)QSPI_Flash_WaitReady(qspi);
}

void QSPI_Flash_DisableQEBit(QSPI_T *qspi)
{
    uint8_t u8Status1 = QSPI_Flash_ReadStatusReg(qspi);
    uint8_t u8Status2 = QSPI_Flash_ReadStatusReg2(qspi);

    if ((u8Status2 & 0x02U) == 0U)
    {
        return;
    }

    u8Status2 &= (uint8_t)~0x02U;
    QSPI_Flash_WriteStatusReg(qspi, u8Status1, u8Status2);
    (void)QSPI_Flash_WaitReady(qspi);
}
#endif

void QSPI_SS_Low(void *spi)
{
    QSPI_SET_SS_LOW((QSPI_T *)spi);
}

void QSPI_SS_High(void *spi)
{
    QSPI_SET_SS_HIGH((QSPI_T *)spi);
}

void QSPI_Write_Tx(void *spi, uint8_t data)
{
    QSPI_WRITE_TX((QSPI_T *)spi, data);
}

uint8_t QSPI_Read_Rx(void *spi)
{
    return (uint8_t)QSPI_READ_RX((QSPI_T *)spi);
}

bool QSPI_Is_Busy(void *spi)
{
    return (QSPI_IS_BUSY((QSPI_T *)spi) != 0U);
}

bool QSPI_Tx_Fifo_empty(void *spi)
{
    return (QSPI_GET_TX_FIFO_EMPTY_FLAG((QSPI_T *)spi) != 0U);
}

//------------------------------------------------------------------------------
// SPIM API
//------------------------------------------------------------------------------
static sfud_err _SFUD_SPIM_ConvertWriteResult(int32_t i32Ret)
{
    if (i32Ret == SPIM_OK)
    {
        return SFUD_SUCCESS;
    }

    if (i32Ret == SPIM_ERR_TIMEOUT)
    {
        return SFUD_ERR_TIMEOUT;
    }

    return SFUD_ERR_WRITE;
}

static sfud_err _SFUD_SPIM_ConvertReadResult(int32_t i32Ret)
{
    if (i32Ret == SPIM_OK)
    {
        return SFUD_SUCCESS;
    }

    if (i32Ret == SPIM_ERR_TIMEOUT)
    {
        return SFUD_ERR_TIMEOUT;
    }

    return SFUD_ERR_READ;
}

#ifdef SFUD_USING_QSPI
static int32_t _SFUD_SPIM_LineToPhase(uint8_t u8Lines, uint32_t *pu32Phase)
{
    int32_t i32Result = 0;

    if (pu32Phase == NULL)
    {
        return -1;
    }

    switch (u8Lines)
    {
        case 1U:
            *pu32Phase = PHASE_NORMAL_MODE;
            break;

        case 2U:
            *pu32Phase = PHASE_DUAL_MODE;
            break;

        case 4U:
            *pu32Phase = PHASE_QUAD_MODE;
            break;

        default:
            i32Result = -1;
            break;
    }

    return i32Result;
}

#ifdef ENABLE_SPIM_DMA_READ
/**
 * @brief Check whether an SFUD SPIM fast-read request can use SPIM DMA.
 *
 * SPIM DMA requires the flash address, SRAM buffer address and transfer size
 * to be 8-byte aligned. When D-Cache is enabled, use the stricter 32-byte
 * buffer/size alignment required by CMSIS cache maintenance operations.
 */
static bool _SFUD_SPIM_CanUseDMA(uint32_t u32Addr,
                                 const uint8_t *pu8Buf,
                                 size_t u32Size)
{
    uintptr_t uBufAddr;

    if ((pu8Buf == NULL) || (u32Size == 0U))
    {
        return false;
    }

#if (SIZE_MAX > UINT32_MAX)

    if (u32Size > (size_t)UINT32_MAX)
    {
        return false;
    }

#endif

    uBufAddr = (uintptr_t)pu8Buf;

    /* SPIM DMA hardware constraint: address, SRAM buffer and size are 8-byte aligned. */
    if (((u32Addr & 0x7UL) != 0UL) ||
            ((uBufAddr & 0x7UL) != 0UL) ||
            ((((uint32_t)u32Size) & 0x7UL) != 0UL))
    {
        return false;
    }

#if (NVT_DCACHE_ON == 1)

    /* CMSIS D-Cache by-address maintenance operates on a 32-byte boundary. */
    if (((uBufAddr & 0x1FUL) != 0UL) ||
            ((((uint32_t)u32Size) & 0x1FUL) != 0UL))
    {
        return false;
    }

#endif

    return true;
}
#endif /* ENABLE_SPIM_DMA_READ */

static sfud_err spim_read(const struct __sfud_spi *spi, uint32_t addr,
                          sfud_qspi_read_cmd_format *qspi_read_cmd_format,
                          uint8_t *read_buf, size_t read_size)
{
    SPIM_T *spim;
    SPIM_PHASE_T sPhase = {0U};

    if ((spi == NULL) || (spi->user_module == NULL) ||
            (qspi_read_cmd_format == NULL) ||
            ((read_size != 0U) && (read_buf == NULL)))
    {
        return SFUD_ERR_READ;
    }

#if (SIZE_MAX > UINT32_MAX)

    if (read_size > (size_t)UINT32_MAX)
    {
        return SFUD_ERR_READ;
    }

#endif

    if ((qspi_read_cmd_format->address_size != 24U) &&
            (qspi_read_cmd_format->address_size != 32U))
    {
        return SFUD_ERR_READ;
    }

    if (_SFUD_SPIM_LineToPhase(qspi_read_cmd_format->instruction_lines, &sPhase.u32CMDPhase) != 0)
    {
        return SFUD_ERR_READ;
    }

    if (_SFUD_SPIM_LineToPhase(qspi_read_cmd_format->address_lines, &sPhase.u32AddrPhase) != 0)
    {
        return SFUD_ERR_READ;
    }

    if (_SFUD_SPIM_LineToPhase(qspi_read_cmd_format->data_lines, &sPhase.u32DataPhase) != 0)
    {
        return SFUD_ERR_READ;
    }

    if (qspi_read_cmd_format->alternate_bytes_lines != 0U)
    {
        return SFUD_ERR_READ;
    }

    spim = (SPIM_T *)spi->user_module;

    sPhase.u32CMDCode = qspi_read_cmd_format->instruction;
    sPhase.u32CMDWidth = PHASE_WIDTH_8;
    sPhase.u32CMDDTR = PHASE_DISABLE_DTR;

    sPhase.u32AddrWidth = (qspi_read_cmd_format->address_size == 32U) ?
                          PHASE_WIDTH_32 : PHASE_WIDTH_24;
    sPhase.u32AddrDTR = PHASE_DISABLE_DTR;

    sPhase.u32ByteOrder = PHASE_ORDER_MODE0;
    sPhase.u32DataDTR = PHASE_DISABLE_DTR;
    sPhase.u32RDQS = PHASE_DISABLE_RDQS;
    sPhase.u32DcNum = qspi_read_cmd_format->dummy_cycles;
    sPhase.u32ContRdEn = PHASE_DISABLE_CONT_READ;

    /* SPIM phase mode natively supports 1/2/4/8 bits. SFUD v1.1.0
       qspi_read only describes 1/2/4-line fast-read transactions, so Octal
       mode is intentionally not synthesized here. */
#ifdef ENABLE_SPIM_DMA_READ

    if (_SFUD_SPIM_CanUseDMA(addr, read_buf, read_size))
    {
        int32_t i32Ret;
        uint32_t u32Is4ByteAddr = (qspi_read_cmd_format->address_size == 32U) ?
                                  SPIM_OP_ENABLE : SPIM_OP_DISABLE;

        /* Program the SPIM Page Read phase table from the same SFUD descriptor
           used by the normal phase-I/O fallback path. */
        SPIM_DMADMM_InitPhase(spim, &sPhase, SPIM_CTL0_OPMODE_PAGEREAD);

#if (NVT_DCACHE_ON == 1)
        /* Destination buffer is cache-line aligned by _SFUD_SPIM_CanUseDMA().
           Clean/invalidate before DMA so dirty destination lines cannot later
           overwrite DMA data in SRAM. */
        SCB_CleanInvalidateDCache_by_Addr((void *)read_buf, (int32_t)read_size);
#endif

        i32Ret = SPIM_DMA_Read(spim,
                               addr,
                               u32Is4ByteAddr,
                               (uint32_t)read_size,
                               read_buf,
                               qspi_read_cmd_format->instruction,
                               SPIM_OP_ENABLE);

#if (NVT_DCACHE_ON == 1)

        if (i32Ret == SPIM_OK)
        {
            /* Discard cached destination lines so the CPU observes DMA data. */
            SCB_InvalidateDCache_by_Addr((void *)read_buf, (int32_t)read_size);
        }

#endif

        return _SFUD_SPIM_ConvertReadResult(i32Ret);
    }

#endif /* ENABLE_SPIM_DMA_READ */

    /* DMA disabled or request does not satisfy DMA/cache alignment rules. */
    SPIM_IO_ReadByPhase(spim, &sPhase, addr, read_buf, (uint32_t)read_size);

    return SFUD_SUCCESS;
}
#endif /* SFUD_USING_QSPI */

/**
 * @brief  SFUD raw SPI write-then-read transaction.
 *
 * The whole write/read sequence is kept under one active SPIM_SS period.
 * SPIM_WriteData() and SPIM_ReadData() intentionally do not control SS.
 */
static sfud_err spim_write_read(const sfud_spi *spi,
                                const uint8_t *write_buf,
                                size_t write_size,
                                uint8_t *read_buf,
                                size_t read_size)
{
    SPIM_T *spim;
    sfud_err eResult = SFUD_SUCCESS;
    int32_t i32Ret;

    if ((spi == NULL) || (spi->user_module == NULL))
    {
        return SFUD_ERR_NOT_FOUND;
    }

    if ((write_size != 0U) && (write_buf == NULL))
    {
        return SFUD_ERR_WRITE;
    }

    if ((read_size != 0U) && (read_buf == NULL))
    {
        return SFUD_ERR_READ;
    }

    if ((write_size == 0U) && (read_size == 0U))
    {
        return SFUD_SUCCESS;
    }

    spim = (SPIM_T *)spi->user_module;

    /* SFUD command/status/SFDP/program/erase transactions use legacy
       1-1-1 SPI through wr(). Multi-line read is handled by qspi_read(). */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    if (write_size != 0U)
    {
        i32Ret = SPIM_WriteData(spim, write_buf, (uint32_t)write_size, SPIM_BITMODE_1);
        eResult = _SFUD_SPIM_ConvertWriteResult(i32Ret);
    }

    if ((eResult == SFUD_SUCCESS) && (read_size != 0U))
    {
        i32Ret = SPIM_ReadData(spim, read_buf, (uint32_t)read_size, SPIM_BITMODE_1);
        eResult = _SFUD_SPIM_ConvertReadResult(i32Ret);
    }

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    return eResult;
}

//------------------------------------------------------------------------------
// USPI API
//------------------------------------------------------------------------------
static sfud_err USPI_ReadWriteByte(USPI_T *uspi, uint8_t u8TxData, uint8_t *pu8RxData)
{
    uint32_t u32TimeOutCnt = SystemCoreClock;

    if (pu8RxData == NULL)
    {
        return SFUD_ERR_READ;
    }

    while ((uspi->BUFSTS & USPI_BUFSTS_TXEMPTY_Msk) == 0U)
    {
        if (--u32TimeOutCnt == 0U)
        {
            return SFUD_ERR_TIMEOUT;
        }
    }

    uspi->TXDAT = u8TxData;
    u32TimeOutCnt = SystemCoreClock;

    while ((uspi->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) != 0U)
    {
        if (--u32TimeOutCnt == 0U)
        {
            return SFUD_ERR_TIMEOUT;
        }
    }

    *pu8RxData = (uint8_t)uspi->RXDAT;
    return SFUD_SUCCESS;
}

static sfud_err USPI_BurstTransfer(const sfud_spi *spi, const uint8_t *write_buf,
                                   size_t write_size, uint8_t *read_buf, size_t read_size)
{
    USPI_T *uspi;
    uint32_t u32i;
    uint32_t u32TimeOutCnt;
    uint8_t u8TxData;
    uint8_t u8RxData;
    sfud_err result = SFUD_SUCCESS;

    if ((spi == NULL) || (spi->user_module == NULL))
    {
        return SFUD_ERR_NOT_FOUND;
    }

    if (((write_size != 0U) && (write_buf == NULL)) ||
            ((read_size != 0U) && (read_buf == NULL)))
    {
        return SFUD_ERR_READ;
    }

    uspi = (USPI_T *)spi->user_module;
    uspi->PROTSTS = USPI_PROTSTS_TXENDIF_Msk;
    uspi->BUFCTL |= USPI_BUFCTL_RXRST_Msk;

    spi->ss_low(spi->user_module);

    for (u32i = 0U; u32i < (write_size + read_size); u32i++)
    {
        u8TxData = ((write_buf != NULL) && (u32i < write_size)) ?
                   write_buf[u32i] : (uint8_t)SFUD_DUMMY_DATA;

        result = USPI_ReadWriteByte(uspi, u8TxData, &u8RxData);

        if (result != SFUD_SUCCESS)
        {
            goto exit;
        }

        if ((read_buf != NULL) && (u32i >= write_size))
        {
            read_buf[u32i - write_size] = u8RxData;
        }
    }

    u32TimeOutCnt = SystemCoreClock;

    while ((uspi->PROTSTS & USPI_PROTSTS_BUSY_Msk) != 0U)
    {
        if (--u32TimeOutCnt == 0U)
        {
            result = SFUD_ERR_TIMEOUT;
            break;
        }
    }

exit:
    spi->ss_high(spi->user_module);
    return result;
}

void USPI_SS_Low(void *spi)
{
    USPI_T *uspi = (USPI_T *)spi;

    uspi->PROTCTL |= USPI_PROTCTL_SS_Msk;
}

void USPI_SS_High(void *spi)
{
    USPI_T *uspi = (USPI_T *)spi;

    uspi->PROTCTL &= ~USPI_PROTCTL_SS_Msk;
}

void USPI_Write_Tx(void *spi, uint8_t data)
{
    USPI_T *uspi = (USPI_T *)spi;

    /* Clear the interrupt flag of master */
    uspi->PROTSTS = USPI_PROTSTS_TXENDIF_Msk;

    while ((uspi->BUFSTS & USPI_BUFSTS_TXEMPTY_Msk) == 0U) {}

    uspi->TXDAT = data;

    /* Check the BUSY flag */
    while ((uspi->PROTSTS & USPI_PROTSTS_BUSY_Msk) != 0U) {}
}

uint8_t USPI_Read_Rx(void *spi)
{
    USPI_T *uspi = (USPI_T *)spi;
    uint8_t u8RdData = 0U;

    /* Reset SPI RX */
    uspi->BUFCTL |= USPI_BUFCTL_RXRST_Msk;

    uspi->TXDAT = 0U;

    while ((uspi->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) != 0U) {}

    u8RdData = (uint8_t)uspi->RXDAT;

    /* Check the BUSY flag */
    while ((uspi->PROTSTS & USPI_PROTSTS_BUSY_Msk) != 0U) {}

    return u8RdData;
}

bool USPI_Is_Busy(void *spi)
{
    USPI_T *uspi = (USPI_T *)spi;

    return (USPI_IS_BUSY(uspi) != 0U);
}

bool USPI_Tx_Fifo_empty(void *spi)
{
    USPI_T *uspi = (USPI_T *)spi;

    return (USPI_GET_TX_EMPTY_FLAG(uspi) != 0U);
}

//------------------------------------------------------------------------------
// LPSPI API
//------------------------------------------------------------------------------
static void LPSPI_SS_Low(void *spi)
{
    LPSPI_SET_SS_LOW((LPSPI_T *)spi);
}

static void LPSPI_SS_High(void *spi)
{
    LPSPI_SET_SS_HIGH((LPSPI_T *)spi);
}

static void LPSPI_Write_Tx(void *spi, uint8_t data)
{
    LPSPI_WRITE_TX((LPSPI_T *)spi, data);
}

static uint8_t LPSPI_Read_Rx(void *spi)
{
    return (uint8_t)LPSPI_READ_RX((LPSPI_T *)spi);
}

static bool LPSPI_Is_Busy(void *spi)
{
    return (LPSPI_IS_BUSY((LPSPI_T *)spi) != 0U);
}

static bool LPSPI_Tx_Fifo_empty(void *spi)
{
    return (LPSPI_GET_TX_FIFO_EMPTY_FLAG((LPSPI_T *)spi) != 0U);
}

//------------------------------------------------------------------------------
/* about 100 microsecond delay */
static void retry_delay_100us(void)
{
    CLK_SysTickDelay(100);
}

sfud_err sfud_spi_port_init(sfud_flash *flash)
{
    sfud_err result = SFUD_SUCCESS;
    uint32_t u32SlewRate = GPIO_SLEWCTL_FAST1;
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel != 0U)
    {
        SYS_UnlockReg();
    }

    switch (flash->index)
    {
        case SFUD_WINBOND_DEV_IDX0:
        {
            /* Enable SPI0 module clock */
            CLK_EnableModuleClock(SPI0_MODULE);

            /* Select SPI0 module clock source as PCLK1 */
            CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_PCLK0, 0UL);

            SYS_ResetModule(SYS_SPI0RST);

            /* Setup SPI0 multi-function pins */
            SET_SPI0_MOSI_PA0();
            SET_SPI0_MISO_PA1();
            SET_SPI0_CLK_PA2();
            SET_SPI0_SS_PA3();

            SYS->GPA_MFP1 =  SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk);
            GPIO_SetMode(SFUD_GPIO_PORT(PA), BIT4, GPIO_MODE_OUTPUT);
            GPIO_SetMode(SFUD_GPIO_PORT(PA), BIT5, GPIO_MODE_OUTPUT);
            SFUD_GPIO_PORT(PA)->DOUT |= (BIT4 | BIT5);

            /* Enable SPI0 clock pin schmitt trigger */
            SFUD_GPIO_PORT(PA)->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

#if (SlewRateMode == 0)
            /* Enable SPI0 I/O normal slew rate */
            u32SlewRate = GPIO_SLEWCTL_NORMAL;
#elif (SlewRateMode == 1)
            /* Enable QSPI0 I/O high slew rate */
            u32SlewRate = GPIO_SLEWCTL_HIGH;
#elif (SlewRateMode == 2)
            /* Enable QSPI0 I/O fast0 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST0;
#elif (SlewRateMode == 3)
            /* Enable QSPI0 I/O fast1 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST1;
#endif

            /* Enable SPI0 I/O normal slew rate */
            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PA), BIT0 | BIT1 | BIT2 | BIT3, u32SlewRate);

            /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
            (void)SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8U, 2000000U);

            /* Disable auto SS function, control SS signal manually. */
            SPI_DisableAutoSS(SPI_FLASH_PORT);

            flash->spi.wr = spi_write_read;
            flash->spi.lock = spi_lock;
            flash->spi.unlock = spi_unlock;

            flash->spi.user_module = (void *)SPI_FLASH_PORT;
            flash->spi.ss_low = SPI_SS_Low;
            flash->spi.ss_high = SPI_SS_High;
            flash->spi.tx = SPI_Write_Tx;
            flash->spi.rx = SPI_Read_Rx;
            flash->spi.isbusy = SPI_Is_Busy;
            flash->spi.tx_fifo_empty = SPI_Tx_Fifo_empty;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = SFUD_FLASH_RETRY_TIMES;
            break;
        }

        case SFUD_WINBOND_DEV_IDX1:
        {
            /* Enable QSPI0 module clock */
            CLK_EnableModuleClock(QSPI0_MODULE);

            /* Select QSPI0 module clock source as PCLK0 */
            CLK_SetModuleClock(QSPI0_MODULE, CLK_QSPISEL_QSPI0SEL_PCLK0, 0UL);

            SYS_ResetModule(SYS_QSPI0RST);

            /* Setup QSPI0 multi-function pins */
            SET_QSPI0_MOSI0_PA0();
            SET_QSPI0_MISO0_PA1();
            SET_QSPI0_CLK_PA2();
            SET_QSPI0_SS_PA3();
            SET_QSPI0_MOSI1_PA4();
            SET_QSPI0_MISO1_PA5();

            /* Enable QSPI0 clock pin (PA2) schmitt trigger */
            SFUD_GPIO_PORT(PA)->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

#if (SlewRateMode == 0)
            /* Enable QSPI0 I/O normal slew rate */
            u32SlewRate = GPIO_SLEWCTL_NORMAL;
#elif (SlewRateMode == 1)
            /* Enable QSPI0 I/O high slew rate */
            u32SlewRate = GPIO_SLEWCTL_HIGH;
#elif (SlewRateMode == 2)
            /* Enable QSPI0 I/O fast0 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST0;
#elif (SlewRateMode == 3)
            /* Enable QSPI0 I/O fast1 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST1;
#endif

            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PA), BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5, u32SlewRate);

#ifdef SFUD_USING_QSPI
            D2D3_SwitchToNormalMode();
#endif
            /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, QSPI Mode-0 timing, clock is 2MHz */
            (void)QSPI_Open(QSPI_FLASH_PORT, QSPI_MASTER, QSPI_MODE_0, 8U, 2000000U);

            /* Disable auto SS function, control SS signal manually. */
            QSPI_DisableAutoSS(QSPI_FLASH_PORT);

            /* set the interfaces and data */
            flash->spi.wr = spi_write_read;
#ifdef SFUD_USING_QSPI
            flash->spi.qspi_read = qspi_read;
#endif
            flash->spi.lock = spi_lock;
            flash->spi.unlock = spi_unlock;

            flash->spi.user_module = (void *)QSPI_FLASH_PORT;
            flash->spi.ss_low = QSPI_SS_Low;
            flash->spi.ss_high = QSPI_SS_High;
            flash->spi.tx = QSPI_Write_Tx;
            flash->spi.rx = QSPI_Read_Rx;
            flash->spi.isbusy = QSPI_Is_Busy;
            flash->spi.tx_fifo_empty = QSPI_Tx_Fifo_empty;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = SFUD_FLASH_RETRY_TIMES;
            break;
        }

        case SFUD_WINBOND_DEV_IDX2:
        {
            /* Enable SPIM module clock */
            CLK_EnableModuleClock(USCI0_MODULE);

            SYS_ResetModule(SYS_USCI0RST);

            /* Set USCI0_SPI multi-function pins */
            SET_USCI0_CTL0_PB0();
            SET_USCI0_CLK_PA11();
            SET_USCI0_DAT0_PA10();
            SET_USCI0_DAT1_PA9();

            /* USCI_SPI clock pin enable schmitt trigger */
            SFUD_GPIO_PORT(PA)->SMTEN |= GPIO_SMTEN_SMTEN11_Msk;

#if (SlewRateMode == 0)
            /* Enable SPIM I/O normal slew rate */
            u32SlewRate = GPIO_SLEWCTL_NORMAL;
#elif (SlewRateMode == 1)
            /* Enable SPIM I/O high slew rate */
            u32SlewRate = GPIO_SLEWCTL_HIGH;
#elif (SlewRateMode == 2)
            /* Enable SPIM I/O fast0 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST0;
#elif (SlewRateMode == 3)
            /* Enable SPIM I/O fast1 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST1;
#endif
            /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PA), BIT9 | BIT10 | BIT11, u32SlewRate);
            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PB), BIT0, u32SlewRate);

            /* Set USCI_SPI0 clock rate = 2MHz */
            (void)USPI_Open(USPI_FLASH_PORT, USPI_MASTER, USPI_MODE_0, 8U, 2000000U);

            USPI_DisableAutoSS(USPI_FLASH_PORT);

            flash->spi.wr = USPI_BurstTransfer;
            flash->spi.lock = spi_lock;
            flash->spi.unlock = spi_unlock;

            flash->spi.user_module = (void *)USPI_FLASH_PORT;
            flash->spi.ss_low = USPI_SS_Low;
            flash->spi.ss_high = USPI_SS_High;
            flash->spi.tx = USPI_Write_Tx;
            flash->spi.rx = USPI_Read_Rx;
            flash->spi.isbusy = USPI_Is_Busy;
            flash->spi.tx_fifo_empty = USPI_Tx_Fifo_empty;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = SFUD_FLASH_RETRY_TIMES;
            break;
        }

        case SFUD_WINBOND_DEV_IDX3:
        {
            /* Enable SPIM module clock */
            CLK_EnableModuleClock(SPIM0_MODULE);

            /* Init SPIM multi-function pins */
            SET_SPIM0_CLK_PH13();
            SET_SPIM0_MISO_PJ4();
            SET_SPIM0_MOSI_PJ3();
            SET_SPIM0_D2_PJ5();
            SET_SPIM0_D3_PJ6();
            SET_SPIM0_SS_PJ7();

            SFUD_GPIO_PORT(PH)->SMTEN |= GPIO_SMTEN_SMTEN13_Msk;

            SFUD_GPIO_PORT(PJ)->SMTEN |= (GPIO_SMTEN_SMTEN3_Msk |
                                          GPIO_SMTEN_SMTEN4_Msk |
                                          GPIO_SMTEN_SMTEN5_Msk |
                                          GPIO_SMTEN_SMTEN6_Msk |
                                          GPIO_SMTEN_SMTEN7_Msk);

#if (SlewRateMode == 0)
            /* Enable SPIM I/O normal slew rate */
            u32SlewRate = GPIO_SLEWCTL_NORMAL;
#elif (SlewRateMode == 1)
            /* Enable SPIM I/O high slew rate */
            u32SlewRate = GPIO_SLEWCTL_HIGH;
#elif (SlewRateMode == 2)
            /* Enable SPIM I/O fast0 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST0;
#elif (SlewRateMode == 3)
            /* Enable SPIM I/O fast1 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST1;
#endif
            /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PH), BIT13, u32SlewRate);

            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PJ), BIT3, u32SlewRate);
            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PJ), BIT4, u32SlewRate);
            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PJ), BIT5, u32SlewRate);
            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PJ), BIT6, u32SlewRate);
            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PJ), BIT7, u32SlewRate);

            SPIM_SET_FLASH_MODE(SPIM0);
            /* Enable DLL. */
            (void)SPIM_INIT_DLL(SPIM0);

            /* Set SPIM clock as HCLK divided by 1 */
            SPIM_SET_CLOCK_DIVIDER(SPIM0, 1U);
            SPIM_SET_RXCLKDLY_RDDLYSEL(SPIM0, 3U);

            /* SFUD owns Flash reset, JEDEC/SFDP discovery, write protection and
               4-byte address policy. Configure controller state only. */
            SPIM_SET_SS_ACTLVL(SPIM0, SPIM_OP_DISABLE);
            SPIM_SET_DTR_MODE(SPIM0, SPIM_OP_DISABLE);
            SPIM_SET_SS_EN(SPIM0, SPIM_OP_DISABLE);

            /* set the interfaces and data */
            flash->spi.wr = spim_write_read;
#ifdef SFUD_USING_QSPI
            flash->spi.qspi_read = spim_read;
#endif
            flash->spi.lock = NULL;
            flash->spi.unlock = NULL;
            flash->spi.user_module = (void *)SPIM_FLASH_PORT;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = SFUD_FLASH_RETRY_TIMES;
            break;
        }

        case SFUD_WINBOND_DEV_IDX4:
        {
            /* Enable SPI0 module clock */
            CLK_EnableModuleClock(LPSPI0_MODULE);

            /* Select SPI0 module clock source as PCLK1 */
            CLK_SetModuleClock(LPSPI0_MODULE, CLK_LPSPISEL_LPSPI0SEL_PCLK4, 0UL);

            /* Setup SPI0 multi-function pins */
            SET_LPSPI0_MOSI_PA0();
            SET_LPSPI0_MISO_PA1();
            SET_LPSPI0_CLK_PA2();
            SET_LPSPI0_SS_PA3();

            SYS->GPA_MFP1 =  SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk);
            GPIO_SetMode(SFUD_GPIO_PORT(PA), BIT4, GPIO_MODE_OUTPUT);
            GPIO_SetMode(SFUD_GPIO_PORT(PA), BIT5, GPIO_MODE_OUTPUT);
            SFUD_GPIO_PORT(PA)->DOUT |= (BIT4 | BIT5);

            /* Enable SPI0 clock pin schmitt trigger */
            SFUD_GPIO_PORT(PA)->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

#if (SlewRateMode == 0)
            /* Enable SPI0 I/O normal slew rate */
            u32SlewRate = GPIO_SLEWCTL_NORMAL;
#elif (SlewRateMode == 1)
            /* Enable QSPI0 I/O high slew rate */
            u32SlewRate = GPIO_SLEWCTL_HIGH;
#elif (SlewRateMode == 2)
            /* Enable QSPI0 I/O fast0 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST0;
#elif (SlewRateMode == 3)
            /* Enable QSPI0 I/O fast1 slew rate */
            u32SlewRate = GPIO_SLEWCTL_FAST1;
#endif

            /* Enable SPI0 I/O normal slew rate */
            GPIO_SetSlewCtl(SFUD_GPIO_PORT(PA), BIT0 | BIT1 | BIT2 | BIT3, u32SlewRate);

            /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
            (void)LPSPI_Open(LPSPI_FLASH_PORT, LPSPI_MASTER, LPSPI_MODE_0, 8U, 2000000U);

            /* Disable auto SS function, control SS signal manually. */
            LPSPI_DisableAutoSS(LPSPI_FLASH_PORT);

            flash->spi.wr = spi_write_read;
            flash->spi.lock = spi_lock;
            flash->spi.unlock = spi_unlock;

            flash->spi.user_module = (void *)LPSPI_FLASH_PORT;
            flash->spi.ss_low = LPSPI_SS_Low;
            flash->spi.ss_high = LPSPI_SS_High;
            flash->spi.tx = LPSPI_Write_Tx;
            flash->spi.rx = LPSPI_Read_Rx;
            flash->spi.isbusy = LPSPI_Is_Busy;
            flash->spi.tx_fifo_empty = LPSPI_Tx_Fifo_empty;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = SFUD_FLASH_RETRY_TIMES;
            break;
        }

        default:
        {
            result = SFUD_ERR_NOT_FOUND;
            break;
        }
    }

    if (u32RegLockLevel != 0U)
    {
        SYS_LockReg();
    }

    return result;
}

/**
 * This function is print debug info.
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 */
void sfud_log_debug(const char *file, const long line, const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    (void)printf("[SFUD](%s:%ld) ", file, line);
    /* must use vprintf to print */
    (void)vsnprintf(log_buf, sizeof(log_buf), format, args);
    (void)printf("%s\n", log_buf);
    va_end(args);
}

/**
 * This function is print routine info.
 *
 * @param format output format
 * @param ... args
 */
void sfud_log_info(const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    (void)printf("[SFUD]");
    /* must use vprintf to print */
    (void)vsnprintf(log_buf, sizeof(log_buf), format, args);
    (void)printf("%s\n", log_buf);
    va_end(args);
}
