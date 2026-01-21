/*
 * This file is part of the Serial Flash Universal Driver Library.
 *
 * Copyright (c) 2016-2018, Armink, <armink.ztl@gmail.com>
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

#include "stdarg.h"
#include "NuMicro.h"
#include "sfud.h"

//------------------------------------------------------------------------------
// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Faster0 <3=> Faster1
#define SlewRateMode        2
// *** <<< end of configuration section >>> ***

#define SPI_FLASH_PORT      SPI0
#define QSPI_FLASH_PORT     QSPI0
#define SPIM_FLASH_PORT     SPIM0
#define USPI_FLASH_PORT     USPI0
#define LPSPI_FLASH_PORT    LPSPI0
#define CHAR_BIT_SIZE       8

//------------------------------------------------------------------------------
static char log_buf[256];

//------------------------------------------------------------------------------
void sfud_log_debug(const char *file, const long line, const char *format, ...);

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
void QSPI_Flash_QuadFastRead(QSPI_T *qspi, uint32_t u32StartAddress, uint8_t *u8DataBuffer);

void SPIM_Write_Tx(void *spi, uint8_t data);
uint8_t SPIM_Read_Rx(void *spi);
void SPIM_SS_Low(void *spi);
void SPIM_SS_High(void *spi);
bool SPIM_Is_Busy(void *spi);
void SPIM_Switch_Output(void *spi);
void SPIM_Switch_Input(void *spi);

static void USPI_SS_Low(void *spi);
static void USPI_SS_High(void *spi);
static void USPI_Write_Tx(void *spi, uint8_t data);
static uint8_t USPI_Read_Rx(void *spi);
bool USPI_Is_Busy(void *spi);
bool USPI_Tx_Fifo_empty(void *spi);
static sfud_err USPI_BurstTransfer(const sfud_spi *spi, const uint8_t *write_buf,
                                   size_t write_size, uint8_t *read_buf, size_t read_size);

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

    if (write_size)
    {
        SFUD_ASSERT(write_buf);
    }

    if (read_size)
    {
        SFUD_ASSERT(read_buf);
    }

    /**
     * add your spi write and read code
     */
    // /CS: active
    spi->ss_low(spi->user_module);

    /* Start reading and writing data */
    for (size_t i = 0, retry_times; i < write_size + read_size; i++)
    {
        /* First write the data from the buffer to the SPI bus; after writing, send a dummy (0xFF) to the SPI bus */
        if ((i < write_size) && (write_buf != NULL))
        {
            send_data = *write_buf++;

            if (spi->sel_dir_out)
            {
                spi->sel_dir_out(spi->user_module);
            }
        }
        else
        {
            send_data = SFUD_DUMMY_DATA;

            if (spi->sel_dir_in)
            {
                spi->sel_dir_in(spi->user_module);
            }
        }

        /* Send data */
        retry_times = 1000;

        while (spi->isbusy(spi->user_module))
        {
            SFUD_RETRY_PROCESS(NULL, retry_times, result);
        }

        if (result != SFUD_SUCCESS)
        {
            goto exit;
        }

        spi->tx(spi->user_module, send_data);
        /* Receive data */
        retry_times = 1000;

        while (spi->isbusy(spi->user_module))
        {
            SFUD_RETRY_PROCESS(NULL, retry_times, result);
        }

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
    return SPI_READ_RX((SPI_T *)spi);
}

bool SPI_Is_Busy(void *spi)
{
    return SPI_IS_BUSY((SPI_T *)spi);
}

bool SPI_Tx_Fifo_empty(void *spi)
{
    return SPI_GET_TX_FIFO_EMPTY_FLAG((SPI_T *)spi);
}

//------------------------------------------------------------------------------
// QSPI API
//------------------------------------------------------------------------------
#ifdef SFUD_USING_QSPI
void D2D3_SwitchToNormalMode(void)
{
    SYS->GPA_MFP1 =  SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk);
    GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT);
    PA4 = 1;
    PA5 = 1;
}

void D2D3_SwitchToQuadMode(void)
{
    SET_QSPI0_MOSI1_PA4();
    SET_QSPI0_MISO1_PA5();
}

__STATIC_INLINE void wait_Qspi_IS_Busy(QSPI_T *qspi)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (QSPI_IS_BUSY(qspi))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for QSPI time-out!\n");
            break;
        }
    }
}

/**
 * read flash data by QSPI
 */
static sfud_err qspi_read(const struct __sfud_spi *spi, uint32_t addr,
                          sfud_qspi_read_cmd_format *qspi_read_cmd_format, uint8_t *read_buf, size_t read_size)
{
    QSPI_T *qspi = (QSPI_T *)spi->user_module;
    sfud_err result = SFUD_SUCCESS;
    uint32_t u32Cnt;

    // enable quad mode
    QSPI_Flash_EnableQEBit(qspi);

    // /CS: active
    QSPI_SET_SS_LOW(qspi);

    // Command: 0xEB, Fast Read quad data
    QSPI_WRITE_TX(qspi, qspi_read_cmd_format->instruction);
    wait_Qspi_IS_Busy(qspi);

    // enable SPI quad IO mode and set direction to input
    D2D3_SwitchToQuadMode();

    // enable SPI quad IO mode and set direction
    QSPI_ENABLE_QUAD_OUTPUT_MODE(qspi);

    if (qspi_read_cmd_format->address_size == 4)
    {
        QSPI_WRITE_TX(qspi, (addr >> 24) & 0xFF);
    }

    // send 24-bit start address
    QSPI_WRITE_TX(qspi, (addr >> 16) & 0xFF);
    QSPI_WRITE_TX(qspi, (addr >> 8) & 0xFF);
    QSPI_WRITE_TX(qspi, addr & 0xFF);

    // dummy byte
    for (u32Cnt = 0; u32Cnt <= (qspi_read_cmd_format->dummy_cycles / 2); u32Cnt++)
    {
        QSPI_WRITE_TX(qspi, 0x00);
    }

    wait_Qspi_IS_Busy(qspi);
    QSPI_ENABLE_QUAD_INPUT_MODE(qspi);

    // clear RX buffer
    QSPI_ClearRxFIFO(qspi);

    // read data
    for (u32Cnt = 0; u32Cnt < read_size; u32Cnt++)
    {
        QSPI_WRITE_TX(qspi, SFUD_DUMMY_DATA);
        wait_Qspi_IS_Busy(qspi);
        read_buf[u32Cnt] = (uint8_t)QSPI_READ_RX(qspi);
    }

    // wait tx finish
    wait_Qspi_IS_Busy(qspi);

    // /CS: de-active
    QSPI_SET_SS_HIGH(qspi);

    QSPI_DISABLE_QUAD_MODE(qspi);

    D2D3_SwitchToNormalMode();

    // disable quad mode
    QSPI_Flash_DisableQEBit(qspi);

    /**
     * add your qspi read flash data code
     */

    return result;
}

uint8_t QSPI_Flash_ReadStatusReg(QSPI_T *qspi)
{
    uint8_t u8Val;

    QSPI_ClearRxFIFO(qspi);

    // /CS: active
    QSPI_SET_SS_LOW(qspi);

    // send Command: 0x05, Read status register
    QSPI_WRITE_TX(qspi, 0x05);

    // read status
    QSPI_WRITE_TX(qspi, 0x00);

    // wait tx finish
    wait_Qspi_IS_Busy(qspi);

    // /CS: de-active
    QSPI_SET_SS_HIGH(qspi);

    // skip first rx data
    u8Val = (uint8_t)QSPI_READ_RX(qspi);
    u8Val = (uint8_t)QSPI_READ_RX(qspi);

    return u8Val;
}

uint8_t QSPI_Flash_ReadStatusReg2(QSPI_T *qspi)
{
    uint8_t u8Val;

    QSPI_ClearRxFIFO(qspi);

    // /CS: active
    QSPI_SET_SS_LOW(qspi);

    // send Command: 0x35, Read status register
    QSPI_WRITE_TX(qspi, 0x35);

    // read status
    QSPI_WRITE_TX(qspi, 0x00);

    // wait tx finish
    wait_Qspi_IS_Busy(qspi);

    // /CS: de-active
    QSPI_SET_SS_HIGH(qspi);

    // skip first rx data
    u8Val = (uint8_t)QSPI_READ_RX(qspi);
    u8Val = (uint8_t)QSPI_READ_RX(qspi);

    return u8Val;
}

void QSPI_Flash_WriteStatusReg(QSPI_T *qspi, uint8_t u8Value1, uint8_t u8Value2)
{
    // /CS: active
    QSPI_SET_SS_LOW(qspi);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(qspi, 0x06);

    // wait tx finish
    wait_Qspi_IS_Busy(qspi);

    // /CS: de-active
    QSPI_SET_SS_HIGH(qspi);

    ///////////////////////////////////////

    // /CS: active
    QSPI_SET_SS_LOW(qspi);

    // send Command: 0x01, Write status register
    QSPI_WRITE_TX(qspi, 0x01);

    // write status
    QSPI_WRITE_TX(qspi, u8Value1);
    QSPI_WRITE_TX(qspi, u8Value2);

    // wait tx finish
    wait_Qspi_IS_Busy(qspi);

    // /CS: de-active
    QSPI_SET_SS_HIGH(qspi);
}

int32_t QSPI_Flash_WaitReady(QSPI_T *qspi)
{
    volatile uint8_t u8ReturnValue;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    do
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for QSPI time-out!\n");
            return -1;
        }

        u8ReturnValue = QSPI_Flash_ReadStatusReg(qspi);
        u8ReturnValue = u8ReturnValue & 1;
    } while (u8ReturnValue != 0); // check the BUSY bit

    return 0;
}

void QSPI_Flash_EnableQEBit(QSPI_T *qspi)
{
    uint8_t u8Status1 = QSPI_Flash_ReadStatusReg(qspi);
    uint8_t u8Status2 = QSPI_Flash_ReadStatusReg2(qspi);

    u8Status2 |= 0x2;
    QSPI_Flash_WriteStatusReg(qspi, u8Status1, u8Status2);
    QSPI_Flash_WaitReady(qspi);
}

void QSPI_Flash_DisableQEBit(QSPI_T *qspi)
{
    uint8_t u8Status1 = QSPI_Flash_ReadStatusReg(qspi);
    uint8_t u8Status2 = QSPI_Flash_ReadStatusReg2(qspi);

    u8Status2 &= ~0x2;
    QSPI_Flash_WriteStatusReg(qspi, u8Status1, u8Status2);
    QSPI_Flash_WaitReady(qspi);
}
#endif

void QSPI_SS_Low(void *spi)
{
				    QSPI_T *qspi = (QSPI_T *)spi;
	
    QSPI_SET_SS_LOW(qspi);
}

void QSPI_SS_High(void *spi)
{
			    QSPI_T *qspi = (QSPI_T *)spi;
	
    QSPI_SET_SS_HIGH(qspi);
}

void QSPI_Write_Tx(void *spi, uint8_t data)
{
			    QSPI_T *qspi = (QSPI_T *)spi;
	
    QSPI_WRITE_TX(qspi, data);
}

uint8_t QSPI_Read_Rx(void *spi)
{
		    QSPI_T *qspi = (QSPI_T *)spi;
	
    return QSPI_READ_RX(qspi);
}

bool QSPI_Is_Busy(void *spi)
{
		    QSPI_T *qspi = (QSPI_T *)spi;
	
    return (bool)QSPI_IS_BUSY(qspi);
}

bool QSPI_Tx_Fifo_empty(void *spi)
{
	    QSPI_T *qspi = (QSPI_T *)spi;
	
    return (bool)QSPI_GET_TX_FIFO_EMPTY_FLAG(qspi);
}

//------------------------------------------------------------------------------
// SPIM API
//------------------------------------------------------------------------------

static sfud_err spim_read(const struct __sfud_spi *spi, uint32_t addr, sfud_qspi_read_cmd_format *qspi_read_cmd_format,
                          uint8_t *read_buf, size_t read_size)
{
    SPIM_T *spim = (SPIM_T *)spi->user_module;
    sfud_err result = SFUD_SUCCESS;
    /* 0xEB: CMD_DMA_FAST_QUAD_READ Command Phase Table */
    //SPIM_PHASE_T sWbEBhRdCMD =
    //{
    //    CMD_DMA_FAST_QUAD_READ,                                                    // Command Code
    //    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                       // Command Phase
    //    PHASE_QUAD_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                        // Address Phase
    //    PHASE_QUAD_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, PHASE_DISABLE_RDQS, // Data Phase
    //    4,                                                                         // Dummy Cycle Phase
    //    PHASE_ENABLE_CONT_READ, PHASE_QUAD_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR, // Read Mode Phase
    //};

    (void)spi;

    // enable quad mode
    SPIM_SetQuadEnable(spim, SPIM_OP_ENABLE, SPIM_BITMODE_1);    /* Enable SPI flash quad mode */

#ifndef ENABLE_SPIM_DMA_READ

#if (NVT_DCACHE_ON == 1)
    // Invalidate the data cache for the buffer to ensure data consistency
    SCB_InvalidateDCache_by_Addr((uint32_t *)&read_buf, read_size);
#endif

    SPIM_IO_Read(spim, addr, qspi_read_cmd_format->address_size > 24,
                 read_size, read_buf, CMD_DMA_FAST_QUAD_READ,
                 SPIM_BITMODE_1, SPIM_BITMODE_4, SPIM_BITMODE_4,
                 qspi_read_cmd_format->dummy_cycles);
    // Perform a Phase IO Read operation from the SPI flash to the buffer
    //SPIM_IO_ReadByPhase(spim, &sWbEBhRdCMD, addr, read_buf, read_size);

#else
    SPIM_SET_DCNUM(4);
    SPIM_DMA_Read(spim, addr, 0, read_size, read_buf, CMD_DMA_FAST_QUAD_READ, 1);
#endif

    return result;
}

void SPIM_Write_Tx(void *spi, uint8_t data)
{
    SPIM_T *spim = (SPIM_T *)spi;

    spim->TX[0] = data;

    /* Switch to Normal mode */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);
    /* Set data width */
    SPIM_SET_DATA_WIDTH(spim, SPIM_DWIDTH_8);
    /* Set burst data number */
    SPIM_SET_BURST_DATA(spim, SPIM_BURSTNUM_1);

    /* Wait until transfer complete */
    (void)SPIM_WaitOpDone(spim, SPIM_OP_ENABLE);
}

uint8_t SPIM_Read_Rx(void *spi)
{
    SPIM_T *spim = (SPIM_T *)spi;

    return spim->RX[0];
}

void SPIM_SS_High(void *spi)
{
    SPIM_T *spim = (SPIM_T *)spi;

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
}

void SPIM_SS_Low(void *spi)
{
    SPIM_T *spim = (SPIM_T *)spi;

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);
}

bool SPIM_Is_Busy(void *spi)
{
    SPIM_T *spim = (SPIM_T *)spi;

    return (bool)SPIM_IS_BUSY(spim);
}

void SPIM_Switch_Output(void *spi)
{
    SPIM_T *spim = (SPIM_T *)spi;

    SPIM_ENABLE_SING_OUTPUT_MODE(spim);     /* 1-bit, Output. */
}

void SPIM_Switch_Input(void *spi)
{
    SPIM_T *spim = (SPIM_T *)spi;

    SPIM_ENABLE_SING_INPUT_MODE(spim);      /* 1-bit, Input.  */
}

//------------------------------------------------------------------------------
// USPI API
//------------------------------------------------------------------------------
static uint8_t USPI_ReadWriteByte(USPI_T *uspi, uint8_t txData)
{
    while ((uspi->BUFSTS & USPI_BUFSTS_TXEMPTY_Msk) == 0U)
    {
        /* wait */
    }

    uspi->TXDAT = txData;

    while ((uspi->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) != 0U)
    {
        /* wait */
    }

    return (uint8_t)uspi->RXDAT;
}

static sfud_err USPI_BurstTransfer(const sfud_spi *spi, const uint8_t *write_buf,
                                   size_t write_size, uint8_t *read_buf, size_t read_size)
{
    USPI_T *uspi = (USPI_T *)spi->user_module;
    volatile uint32_t i;
    uint8_t  txData;
    uint8_t  rxData;
    sfud_err result = SFUD_SUCCESS;

    uspi->PROTSTS = USPI_PROTSTS_TXENDIF_Msk;
    uspi->BUFCTL |= USPI_BUFCTL_RXRST_Msk;

    // /CS: active
    spi->ss_low(spi->user_module);

    for (i = 0U; i < (write_size + read_size); i++)
    {
        if ((write_buf != NULL) && (i < write_size))
        {
            /* command/address phase */
            txData = write_buf[i];
        }
        else
        {
            txData = SFUD_DUMMY_DATA;
        }

        rxData = USPI_ReadWriteByte(uspi, txData);

        if ((read_buf != NULL) && (i >= write_size))
        {
            read_buf[i - write_size] = rxData;
        }
    }

    while ((uspi->PROTSTS & USPI_PROTSTS_BUSY_Msk) != 0U)
    {
        /* wait */
    }

    // /CS: Inactive
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

    while ((uspi->BUFSTS & USPI_BUFSTS_TXEMPTY_Msk) == 0) {}

    uspi->TXDAT = data;

    /* Check the BUSY flag */
    while (uspi->PROTSTS & USPI_PROTSTS_BUSY_Msk) {}
}

uint8_t USPI_Read_Rx(void *spi)
{
    USPI_T *uspi = (USPI_T *)spi;
    uint8_t u8RdData = 0;

    /* Reset SPI RX */
    uspi->BUFCTL |= USPI_BUFCTL_RXRST_Msk;

    uspi->TXDAT = 0;

    while (uspi->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) {}

    u8RdData = uspi->RXDAT;

    /* Check the BUSY flag */
    while (uspi->PROTSTS & USPI_PROTSTS_BUSY_Msk) {}

    return u8RdData;
}

bool USPI_Is_Busy(void *spi)
{
    USPI_T *uspi = (USPI_T *)spi;

    return (bool)USPI_IS_BUSY(uspi);
}

bool USPI_Tx_Fifo_empty(void *spi)
{
    USPI_T *uspi = (USPI_T *)spi;

    return (bool)USPI_GET_TX_EMPTY_FLAG(uspi);
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

    if (u32RegLockLevel)
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
            CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_PCLK0, MODULE_NoMsk);

					            SYS_ResetModule(SYS_SPI0RST);
					
            /* Setup SPI0 multi-function pins */
            SET_SPI0_MOSI_PA0();
            SET_SPI0_MISO_PA1();
            SET_SPI0_CLK_PA2();
            SET_SPI0_SS_PA3();

            SYS->GPA_MFP1 =  SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk);
            GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);
            GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT);
            PA4 = 1;
            PA5 = 1;

            /* Enable SPI0 clock pin schmitt trigger */
            PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

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
            GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, u32SlewRate);

            /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
            SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8, 2000000);

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
            flash->retry.times = 1000;//100ms timeout
        }
        break;

        case SFUD_WINBOND_DEV_IDX1:
        {
            /* Enable QSPI0 module clock */
            CLK_EnableModuleClock(QSPI0_MODULE);

            /* Select QSPI0 module clock source as PCLK0 */
            CLK_SetModuleClock(QSPI0_MODULE, CLK_QSPISEL_QSPI0SEL_PCLK0, MODULE_NoMsk);

            SYS_ResetModule(SYS_QSPI0RST);

            /* Setup QSPI0 multi-function pins */
            SET_QSPI0_MOSI0_PA0();
            SET_QSPI0_MISO0_PA1();
            SET_QSPI0_CLK_PA2();
            SET_QSPI0_SS_PA3();
            SET_QSPI0_MOSI1_PA4();
            SET_QSPI0_MISO1_PA5();

            /* Enable QSPI0 clock pin (PA2) schmitt trigger */
            PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

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

            GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5, u32SlewRate);

#ifdef SFUD_USING_QSPI
            D2D3_SwitchToNormalMode();
#endif
            /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, QSPI Mode-0 timing, clock is 2MHz */
            QSPI_Open(QSPI_FLASH_PORT, QSPI_MASTER, QSPI_MODE_0, 8, 2000000);

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
            flash->retry.times = 1000;//100ms timeout
        }
        break;

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
            PA->SMTEN |= GPIO_SMTEN_SMTEN11_Msk;

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
            GPIO_SetSlewCtl(PA, BIT9 | BIT10 | BIT11, u32SlewRate);
            GPIO_SetSlewCtl(PB, BIT0, u32SlewRate);

            /* Set USCI_SPI0 clock rate = 2MHz */
            USPI_Open(USPI_FLASH_PORT, USPI_MASTER, USPI_MODE_0, 8, 2000000);

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
            flash->retry.times = 1000;//100ms timeout
        }
        break;

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

            PH->SMTEN |= (GPIO_SMTEN_SMTEN13_Msk);

            PJ->SMTEN |= (GPIO_SMTEN_SMTEN3_Msk |
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
            GPIO_SetSlewCtl(PH, BIT13, u32SlewRate);

            GPIO_SetSlewCtl(PJ, BIT3, u32SlewRate);
            GPIO_SetSlewCtl(PJ, BIT4, u32SlewRate);
            GPIO_SetSlewCtl(PJ, BIT5, u32SlewRate);
            GPIO_SetSlewCtl(PJ, BIT6, u32SlewRate);
            GPIO_SetSlewCtl(PJ, BIT7, u32SlewRate);

            /* Set SPIM clock as HCLK divided by 1 */
            SPIM_SET_CLOCK_DIVIDER(SPIM0, 1);
            SPIM_SET_RXCLKDLY_RDDLYSEL(SPIM0, 3);

            SPIM_InitFlash(SPIM0, SPIM_OP_ENABLE);

            /* set the interfaces and data */
            flash->spi.wr = spi_write_read;
#ifdef SFUD_USING_QSPI
            flash->spi.qspi_read = spim_read;
#endif
            flash->spi.lock = spi_lock;
            flash->spi.unlock = spi_unlock;

            flash->spi.user_module = (void *)SPIM_FLASH_PORT;
            flash->spi.ss_low = SPIM_SS_Low;
            flash->spi.ss_high = SPIM_SS_High;
            flash->spi.sel_dir_out = SPIM_Switch_Output;
            flash->spi.sel_dir_in = SPIM_Switch_Input;
            flash->spi.tx = SPIM_Write_Tx;
            flash->spi.rx = SPIM_Read_Rx;
            flash->spi.isbusy = SPIM_Is_Busy;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = 1000;//100ms timeout
        }
        break;

        case SFUD_WINBOND_DEV_IDX4:
        {
            /* Enable SPI0 module clock */
            CLK_EnableModuleClock(LPSPI0_MODULE);

            /* Select SPI0 module clock source as PCLK1 */
            CLK_SetModuleClock(LPSPI0_MODULE, CLK_LPSPISEL_LPSPI0SEL_PCLK4, MODULE_NoMsk);

            /* Setup SPI0 multi-function pins */
            SET_LPSPI0_MOSI_PA0();
            SET_LPSPI0_MISO_PA1();
            SET_LPSPI0_CLK_PA2();
            SET_LPSPI0_SS_PA3();

            SYS->GPA_MFP1 =  SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk);
            GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);
            GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT);
            PA4 = 1;
            PA5 = 1;

            /* Enable SPI0 clock pin schmitt trigger */
            PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

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
            GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, u32SlewRate);

            /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
            LPSPI_Open(LPSPI_FLASH_PORT, LPSPI_MASTER, LPSPI_MODE_0, 8, 2000000);

            /* Disable auto SS function, control SS signal manually. */
            LPSPI_DisableAutoSS(LPSPI_FLASH_PORT);

            flash->spi.wr = spi_write_read;
            flash->spi.lock = spi_lock;
            flash->spi.unlock = spi_unlock;

            flash->spi.user_module = (void *)LPSPI_FLASH_PORT;
            flash->spi.ss_low = SPI_SS_Low;
            flash->spi.ss_high = SPI_SS_High;
            flash->spi.tx = SPI_Write_Tx;
            flash->spi.rx = SPI_Read_Rx;
            flash->spi.isbusy = SPI_Is_Busy;
            flash->spi.tx_fifo_empty = SPI_Tx_Fifo_empty;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = 1000;//100ms timeout
        }
        break;

        default:
            result = SFUD_ERR_NOT_FOUND;
            break;
    }

    if (u32RegLockLevel)
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
    printf("[SFUD](%s:%ld) ", file, line);
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\n", log_buf);
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
    printf("[SFUD]");
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\n", log_buf);
    va_end(args);
}
