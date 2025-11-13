/**************************************************************************//**
 * @file     spi.c
 * @version  V3.00
 * @brief    M2U51 series SPI driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPI_Driver SPI Driver
  @{
*/

/** @addtogroup SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  This function make SPI module be ready to transfer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32MasterSlave Decides the SPI module is operating in master mode or in slave mode. (SPI_SLAVE, SPI_MASTER)
  * @param[in]  u32SPIMode Decides the transfer timing. (SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3)
  * @param[in]  u32DataWidth Decides the data width of a SPI transaction.
  * @param[in]  u32BusClock The expected frequency of SPI bus clock in Hz.
  * @return Actual frequency of SPI peripheral clock.
  * @details By default, the SPI transfer sequence is MSB first, the slave selection signal is active low and the automatic
  *          slave selection function is disabled.
  *          In Slave mode, the u32BusClock shall be NULL and the SPI clock divider setting will be 0.
  *          The actual clock rate may be different from the target SPI clock rate.
  *          For example, if the SPI source clock rate is 12 MHz and the target SPI bus clock rate is 7 MHz, the
  *          actual SPI clock rate will be 6MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, SPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
  * @note   If u32BusClock >= SPI peripheral clock source, DIVIDER will be set to 0.
  * @note   In slave mode, the SPI peripheral clock rate will be equal to APB clock rate.
  */
uint32_t SPI_Open(SPI_T *spi,
                  uint32_t u32MasterSlave,
                  uint32_t u32SPIMode,
                  uint32_t u32DataWidth,
                  uint32_t u32BusClock)
{
    uint32_t u32ClkSrc = 0U, u32Div, u32HCLKFreq, u32RetValue=0U;

    /* Disable I2S mode */
//    spi->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;

    if(u32DataWidth == 32U)
    {
        u32DataWidth = 0U;
    }

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    if(u32MasterSlave == SPI_MASTER)
    {
        /* Default setting: slave selection signal is active low; disable automatic slave selection function. */
        spi->SSCTL = SPI_SS_ACTIVE_LOW;

        /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
        spi->CTL = u32MasterSlave | (u32DataWidth << SPI_CTL_DWIDTH_Pos) | (u32SPIMode) | SPI_CTL_SPIEN_Msk;

        if(u32BusClock >= u32HCLKFreq)
        {
            /* Select PCLK as the clock source of SPI */
            if(spi == SPI0)
            {
                CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK1;
            }
            else if(spi == SPI1)
            {
                CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;
            }
            else
            {
                CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI2SEL_Msk)) | CLK_CLKSEL2_SPI2SEL_PCLK1;
            }
        }

        /* Check clock source of SPI */
        if(spi == SPI0)
        {
            if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PCLK1)
            {
                /* Clock source is PCLK1 */
                u32ClkSrc = CLK_GetPCLK1Freq();
            }
            else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_MIRC)
            {
                u32ClkSrc = __MIRC; /* Clock source is MIRC */
            }
            else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_HIRC)
            {
                u32ClkSrc = __HIRC; /* Clock source is HIRC */
            }
            else
            {
                u32ClkSrc = 0; /* Clock source is reserved */
            }
        }
        else if(spi == SPI1)
        {
            if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PCLK0)
            {
                /* Clock source is PCLK0 */
                u32ClkSrc = CLK_GetPCLK0Freq();
            }
            else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_MIRC)
            {
                u32ClkSrc = __MIRC; /* Clock source is MIRC */
            }
            else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_HIRC)
            {
                u32ClkSrc = __HIRC; /* Clock source is HIRC */
            }
            else
            {
                u32ClkSrc = 0; /* Clock source is reserved */
            }
        }
        else    //(spi == SPI2)
        {
            if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_PCLK1)
            {
                /* Clock source is PCLK1 */
                u32ClkSrc = CLK_GetPCLK1Freq();
            }
            else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_MIRC)
            {
                u32ClkSrc = __MIRC; /* Clock source is MIRC */
            }
            else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_HIRC)
            {
                u32ClkSrc = __HIRC; /* Clock source is HIRC */
            }
            else
            {
                u32ClkSrc = 0; /* Clock source is reserved */
            }
        }
        if(u32BusClock >= u32HCLKFreq)
        {
            /* Set DIVIDER = 0 */
            spi->CLKDIV = 0U;
            /* Return master peripheral clock rate */
            u32RetValue = u32ClkSrc;
        }
        else if(u32BusClock >= u32ClkSrc)
        {
            /* Set DIVIDER = 0 */
            spi->CLKDIV = 0U;
            /* Return master peripheral clock rate */
            u32RetValue = u32ClkSrc;
        }
        else if(u32BusClock == 0U)
        {
            /* Set DIVIDER to the maximum value 0xFF. f_spi = f_spi_clk_src / (DIVIDER + 1) */
            spi->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
            /* Return master peripheral clock rate */
            u32RetValue = (u32ClkSrc / (0xFFU + 1U));
        }
        else
        {
            u32Div = (((u32ClkSrc * 10U) / u32BusClock + 5U) / 10U) - 1U; /* Round to the nearest integer */
            if(u32Div > 0xFFU)
            {
                u32Div = 0xFFU;
                spi->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
                /* Return master peripheral clock rate */
                u32RetValue = (u32ClkSrc / (0xFFU + 1U));
            }
            else
            {
                spi->CLKDIV = (spi->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (u32Div << SPI_CLKDIV_DIVIDER_Pos);
                /* Return master peripheral clock rate */
                u32RetValue = (u32ClkSrc / (u32Div + 1U));
            }
        }
    }
    else     /* For slave mode, force the SPI peripheral clock rate to equal APB clock rate. */
    {
        /* Default setting: slave selection signal is low level active. */
        spi->SSCTL = SPI_SS_ACTIVE_LOW;

        /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
        spi->CTL = u32MasterSlave | (u32DataWidth << SPI_CTL_DWIDTH_Pos) | (u32SPIMode) | SPI_CTL_SPIEN_Msk;

        /* Set DIVIDER = 0 */
        spi->CLKDIV = 0U;

        /* Select PCLK as the clock source of SPI */
        if(spi == SPI0)
        {
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK1;
            /* Return slave peripheral clock rate */
            u32RetValue = CLK_GetPCLK1Freq();
        }
        else if(spi == SPI1)
        {
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;
            /* Return slave peripheral clock rate */
            u32RetValue = CLK_GetPCLK0Freq();
        }
        else // (spi == SPI2)
        {
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI2SEL_Msk)) | CLK_CLKSEL2_SPI2SEL_PCLK1;
            /* Return slave peripheral clock rate */
            u32RetValue = CLK_GetPCLK1Freq();
        }
    }

    return u32RetValue;
}

/**
  * @brief  Disable SPI controller.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  * @details This function will reset SPI controller.
  */
void SPI_Close(SPI_T *spi)
{
    if(spi == SPI0)
    {
        /* Reset SPI */
        SYS->IPRST1 |= SYS_IPRST1_SPI0RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_SPI0RST_Msk;
    }
    else if(spi == SPI1)
    {
        /* Reset SPI */
        SYS->IPRST1 |= SYS_IPRST1_SPI1RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_SPI1RST_Msk;
    }
    else    //(spi == SPI2)
    {
        /* Reset SPI */
        SYS->IPRST1 |= SYS_IPRST1_SPI2RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_SPI2RST_Msk;
    }
}

/**
  * @brief  Clear RX FIFO buffer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  * @details This function will clear SPI RX FIFO buffer. The RXEMPTY (SPI_STATUS[8]) will be set to 1.
  */
void SPI_ClearRxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
}

/**
  * @brief  Clear TX FIFO buffer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  * @details This function will clear SPI TX FIFO buffer. The TXEMPTY (SPI_STATUS[16]) will be set to 1.
  * @note The TX shift register will not be cleared.
  */
void SPI_ClearTxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_TXFBCLR_Msk;
}

/**
  * @brief  Disable the automatic slave selection function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  * @details This function will disable the automatic slave selection function and set slave selection signal to inactive state.
  */
void SPI_DisableAutoSS(SPI_T *spi)
{
    spi->SSCTL &= ~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk);
}

/**
  * @brief  Enable the automatic slave selection function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32SSPinMask Specifies slave selection pins. (SPI_SS)
  * @param[in]  u32ActiveLevel Specifies the active level of slave selection signal. (SPI_SS_ACTIVE_HIGH, SPI_SS_ACTIVE_LOW)
  * @return None
  * @details This function will enable the automatic slave selection function. Only available in Master mode.
  *          The slave selection pin and the active level will be set in this function.
  */
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    spi->SSCTL = (spi->SSCTL & (~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS_Msk))) | (u32SSPinMask | u32ActiveLevel | SPI_SSCTL_AUTOSS_Msk);
}

/**
  * @brief  Set the SPI bus clock.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32BusClock The expected frequency of SPI bus clock in Hz.
  * @return Actual frequency of SPI bus clock.
  * @details This function is only available in Master mode. The actual clock rate may be different from the target SPI bus clock rate.
  *          For example, if the SPI source clock rate is 12 MHz and the target SPI bus clock rate is 7 MHz, the actual SPI bus clock
  *          rate will be 6 MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, SPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
  * @note   If u32BusClock >= SPI peripheral clock source, DIVIDER will be set to 0.
  */
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock)
{
    uint32_t u32ClkSrc, u32HCLKFreq;
    uint32_t u32Div, u32RetValue;

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    if(u32BusClock >= u32HCLKFreq)
    {
        /* Select PCLK as the clock source of SPI */
        if(spi == SPI0)
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK1;
        else if(spi == SPI1)
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;
        else
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI2SEL_Msk)) | CLK_CLKSEL2_SPI2SEL_PCLK1;
    }

    /* Check clock source of SPI */
    if(spi == SPI0)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PCLK1)
        {
            /* Clock source is PCLK1 */
            u32ClkSrc = CLK_GetPCLK1Freq();
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_MIRC)
        {
            u32ClkSrc = __MIRC; /* Clock source is MIRC */
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_HIRC)
        {
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
        }
        else
        {
            u32ClkSrc = 0; /* Clock source is reserved */
        }
    }
    else if(spi == SPI1)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PCLK0)
        {
            /* Clock source is PCLK0 */
            u32ClkSrc = CLK_GetPCLK0Freq();
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_MIRC)
        {
            u32ClkSrc = __MIRC; /* Clock source is MIRC */
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_HIRC)
        {
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
        }
        else
        {
            u32ClkSrc = 0; /* Clock source is reserved */
        }
    }
    else    //(spi == SPI2)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_PCLK1)
        {
            /* Clock source is PCLK1 */
            u32ClkSrc = CLK_GetPCLK1Freq();
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_MIRC)
        {
            u32ClkSrc = __MIRC; /* Clock source is MIRC */
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_HIRC)
        {
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
        }
        else
        {
            u32ClkSrc = 0; /* Clock source is reserved */
        }
    }

    if(u32BusClock >= u32HCLKFreq)
    {
        /* Set DIVIDER = 0 */
        spi->CLKDIV = 0U;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    }
    else if(u32BusClock >= u32ClkSrc)
    {
        /* Set DIVIDER = 0 */
        spi->CLKDIV = 0U;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    }
    else if(u32BusClock == 0U)
    {
        /* Set DIVIDER to the maximum value 0xFF. f_spi = f_spi_clk_src / (DIVIDER + 1) */
        spi->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
        /* Return master peripheral clock rate */
        u32RetValue = (u32ClkSrc / (0xFFU + 1U));
    }
    else
    {
        u32Div = (((u32ClkSrc * 10U) / u32BusClock + 5U) / 10U) - 1U; /* Round to the nearest integer */
        if(u32Div > 0x1FFU)
        {
            u32Div = 0x1FFU;
            spi->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
            /* Return master peripheral clock rate */
            u32RetValue = (u32ClkSrc / (0xFFU + 1U));
        }
        else
        {
            spi->CLKDIV = (spi->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (u32Div << SPI_CLKDIV_DIVIDER_Pos);
            /* Return master peripheral clock rate */
            u32RetValue = (u32ClkSrc / (u32Div + 1U));
        }
    }

    return u32RetValue;
}

/**
  * @brief  Configure FIFO threshold setting.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32TxThreshold Decides the TX FIFO threshold. It could be 0 ~ 3. If data width is 8~16 bits, it could be 0 ~ 7.
  * @param[in]  u32RxThreshold Decides the RX FIFO threshold. It could be 0 ~ 3. If data width is 8~16 bits, it could be 0 ~ 7.
  * @return None
  * @details Set TX FIFO threshold and RX FIFO threshold configurations.
  */
void SPI_SetFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    spi->FIFOCTL = (spi->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk)) |
                   (u32TxThreshold << SPI_FIFOCTL_TXTH_Pos) |
                   (u32RxThreshold << SPI_FIFOCTL_RXTH_Pos);
}

/**
  * @brief  Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return Actual SPI bus clock frequency in Hz.
  * @details This function will calculate the actual SPI bus clock rate according to the SPInSEL and DIVIDER settings. Only available in Master mode.
  */
uint32_t SPI_GetBusClock(SPI_T *spi)
{
    uint32_t u32Div;
    uint32_t u32ClkSrc;

    /* Get DIVIDER setting */
    u32Div = (spi->CLKDIV & SPI_CLKDIV_DIVIDER_Msk) >> SPI_CLKDIV_DIVIDER_Pos;

    /* Check clock source of SPI */
    if(spi == SPI0)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PCLK1)
        {
            /* Clock source is PCLK1 */
            u32ClkSrc = CLK_GetPCLK1Freq();
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_MIRC)
        {
            u32ClkSrc = __MIRC; /* Clock source is MIRC */
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_HIRC)
        {
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
        }
        else
        {
            u32ClkSrc = 0; /* Clock source is reserved */
        }
    }
    else if(spi == SPI1)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PCLK0)
        {
            /* Clock source is PCLK0 */
            u32ClkSrc = CLK_GetPCLK0Freq();
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_MIRC)
        {
            u32ClkSrc = __MIRC; /* Clock source is MIRC */
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_HIRC)
        {
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
        }
        else
        {
            u32ClkSrc = 0; /* Clock source is reserved */
        }
    }
    else    //(spi == SPI2)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_PCLK1)
        {
            /* Clock source is PCLK1 */
            u32ClkSrc = CLK_GetPCLK1Freq();
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_MIRC)
        {
            u32ClkSrc = __MIRC; /* Clock source is MIRC */
        }
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_HIRC)
        {
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
        }
        else
        {
            u32ClkSrc = 0; /* Clock source is reserved */
        }
    }

    /* Return SPI bus clock rate */
    return (u32ClkSrc / (u32Div + 1U));
}

/**
  * @brief  Enable interrupt function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt enable bit.
  *                     This parameter decides which interrupts will be enabled. It is combination of:
  *                       - \ref SPI_UNIT_INT_MASK
  *                       - \ref SPI_SSACT_INT_MASK
  *                       - \ref SPI_SSINACT_INT_MASK
  *                       - \ref SPI_SLVUR_INT_MASK
  *                       - \ref SPI_SLVBE_INT_MASK
  *                       - \ref SPI_TXUF_INT_MASK
  *                       - \ref SPI_FIFO_TXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXOV_INT_MASK
  *                       - \ref SPI_FIFO_RXTO_INT_MASK
  *
  * @return None
  * @details Enable SPI related interrupts specified by u32Mask parameter.
  */
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask)
{
    /* Enable unit transfer interrupt flag */
    if((u32Mask & SPI_UNIT_INT_MASK) == SPI_UNIT_INT_MASK)
    {
        spi->CTL |= SPI_CTL_UNITIEN_Msk;
    }

    /* Enable slave selection signal active interrupt flag */
    if((u32Mask & SPI_SSACT_INT_MASK) == SPI_SSACT_INT_MASK)
    {
        spi->SSCTL |= SPI_SSCTL_SSACTIEN_Msk;
    }

    /* Enable slave selection signal inactive interrupt flag */
    if((u32Mask & SPI_SSINACT_INT_MASK) == SPI_SSINACT_INT_MASK)
    {
        spi->SSCTL |= SPI_SSCTL_SSINAIEN_Msk;
    }

    /* Enable slave TX under run interrupt flag */
    if((u32Mask & SPI_SLVUR_INT_MASK) == SPI_SLVUR_INT_MASK)
    {
        spi->SSCTL |= SPI_SSCTL_SLVURIEN_Msk;
    }

    /* Enable slave bit count error interrupt flag */
    if((u32Mask & SPI_SLVBE_INT_MASK) == SPI_SLVBE_INT_MASK)
    {
        spi->SSCTL |= SPI_SSCTL_SLVBEIEN_Msk;
    }

    /* Enable slave TX underflow interrupt flag */
    if((u32Mask & SPI_TXUF_INT_MASK) == SPI_TXUF_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Enable TX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_TXTH_INT_MASK) == SPI_FIFO_TXTH_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Enable RX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_RXTH_INT_MASK) == SPI_FIFO_RXTH_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Enable RX overrun interrupt flag */
    if((u32Mask & SPI_FIFO_RXOV_INT_MASK) == SPI_FIFO_RXOV_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Enable RX time-out interrupt flag */
    if((u32Mask & SPI_FIFO_RXTO_INT_MASK) == SPI_FIFO_RXTO_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_RXTOIEN_Msk;
    }
}

/**
  * @brief  Disable interrupt function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt bit.
  *                     This parameter decides which interrupts will be disabled. It is combination of:
  *                       - \ref SPI_UNIT_INT_MASK
  *                       - \ref SPI_SSACT_INT_MASK
  *                       - \ref SPI_SSINACT_INT_MASK
  *                       - \ref SPI_SLVUR_INT_MASK
  *                       - \ref SPI_SLVBE_INT_MASK
  *                       - \ref SPI_TXUF_INT_MASK
  *                       - \ref SPI_FIFO_TXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXOV_INT_MASK
  *                       - \ref SPI_FIFO_RXTO_INT_MASK
  *
  * @return None
  * @details Disable SPI related interrupts specified by u32Mask parameter.
  */
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask)
{
    /* Disable unit transfer interrupt flag */
    if((u32Mask & SPI_UNIT_INT_MASK) == SPI_UNIT_INT_MASK)
    {
        spi->CTL &= ~SPI_CTL_UNITIEN_Msk;
    }

    /* Disable slave selection signal active interrupt flag */
    if((u32Mask & SPI_SSACT_INT_MASK) == SPI_SSACT_INT_MASK)
    {
        spi->SSCTL &= ~SPI_SSCTL_SSACTIEN_Msk;
    }

    /* Disable slave selection signal inactive interrupt flag */
    if((u32Mask & SPI_SSINACT_INT_MASK) == SPI_SSINACT_INT_MASK)
    {
        spi->SSCTL &= ~SPI_SSCTL_SSINAIEN_Msk;
    }

    /* Disable slave TX under run interrupt flag */
    if((u32Mask & SPI_SLVUR_INT_MASK) == SPI_SLVUR_INT_MASK)
    {
        spi->SSCTL &= ~SPI_SSCTL_SLVURIEN_Msk;
    }

    /* Disable slave bit count error interrupt flag */
    if((u32Mask & SPI_SLVBE_INT_MASK) == SPI_SLVBE_INT_MASK)
    {
        spi->SSCTL &= ~SPI_SSCTL_SLVBEIEN_Msk;
    }

    /* Disable slave TX underflow interrupt flag */
    if((u32Mask & SPI_TXUF_INT_MASK) == SPI_TXUF_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Disable TX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_TXTH_INT_MASK) == SPI_FIFO_TXTH_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Disable RX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_RXTH_INT_MASK) == SPI_FIFO_RXTH_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Disable RX overrun interrupt flag */
    if((u32Mask & SPI_FIFO_RXOV_INT_MASK) == SPI_FIFO_RXOV_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Disable RX time-out interrupt flag */
    if((u32Mask & SPI_FIFO_RXTO_INT_MASK) == SPI_FIFO_RXTO_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTOIEN_Msk;
    }
}

/**
  * @brief  Get interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be read. It is combination of:
  *                       - \ref SPI_UNIT_INT_MASK
  *                       - \ref SPI_SSACT_INT_MASK
  *                       - \ref SPI_SSINACT_INT_MASK
  *                       - \ref SPI_SLVUR_INT_MASK
  *                       - \ref SPI_SLVBE_INT_MASK
  *                       - \ref SPI_TXUF_INT_MASK
  *                       - \ref SPI_FIFO_TXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXOV_INT_MASK
  *                       - \ref SPI_FIFO_RXTO_INT_MASK
  *
  * @return Interrupt flags of selected sources.
  * @details Get SPI related interrupt flags specified by u32Mask parameter.
  */
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0U, u32TmpVal;

    u32TmpVal = spi->STATUS & SPI_STATUS_UNITIF_Msk;
    /* Check unit transfer interrupt flag */
    if((u32Mask & SPI_UNIT_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_UNIT_INT_MASK;
    }

    u32TmpVal = spi->STATUS & SPI_STATUS_SSACTIF_Msk;
    /* Check slave selection signal active interrupt flag */
    if((u32Mask & SPI_SSACT_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_SSACT_INT_MASK;
    }

    u32TmpVal = spi->STATUS & SPI_STATUS_SSINAIF_Msk;
    /* Check slave selection signal inactive interrupt flag */
    if((u32Mask & SPI_SSINACT_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_SSINACT_INT_MASK;
    }

    u32TmpVal = spi->STATUS & SPI_STATUS_SLVURIF_Msk;
    /* Check slave TX under run interrupt flag */
    if((u32Mask & SPI_SLVUR_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_SLVUR_INT_MASK;
    }

    u32TmpVal = spi->STATUS & SPI_STATUS_SLVBEIF_Msk;
    /* Check slave bit count error interrupt flag */
    if((u32Mask & SPI_SLVBE_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_SLVBE_INT_MASK;
    }

    u32TmpVal = spi->STATUS & SPI_STATUS_TXUFIF_Msk;
    /* Check slave TX underflow interrupt flag */
    if((u32Mask & SPI_TXUF_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_TXUF_INT_MASK;
    }

    u32TmpVal = spi->STATUS & SPI_STATUS_TXTHIF_Msk;
    /* Check TX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_TXTH_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_FIFO_TXTH_INT_MASK;
    }

    u32TmpVal = spi->STATUS & SPI_STATUS_RXTHIF_Msk;
    /* Check RX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_RXTH_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_FIFO_RXTH_INT_MASK;
    }

    u32TmpVal = spi->STATUS & SPI_STATUS_RXOVIF_Msk;
    /* Check RX overrun interrupt flag */
    if((u32Mask & SPI_FIFO_RXOV_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_FIFO_RXOV_INT_MASK;
    }

    u32TmpVal = spi->STATUS & SPI_STATUS_RXTOIF_Msk;
    /* Check RX time-out interrupt flag */
    if((u32Mask & SPI_FIFO_RXTO_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= SPI_FIFO_RXTO_INT_MASK;
    }

    return u32IntFlag;
}

/**
  * @brief  Clear interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be cleared. It could be the combination of:
  *                       - \ref SPI_UNIT_INT_MASK
  *                       - \ref SPI_SSACT_INT_MASK
  *                       - \ref SPI_SSINACT_INT_MASK
  *                       - \ref SPI_SLVUR_INT_MASK
  *                       - \ref SPI_SLVBE_INT_MASK
  *                       - \ref SPI_TXUF_INT_MASK
  *                       - \ref SPI_FIFO_RXOV_INT_MASK
  *                       - \ref SPI_FIFO_RXTO_INT_MASK
  *
  * @return None* @details Clear SPI related interrupt flags specified by u32Mask parameter.
  */
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask)
{
    if(u32Mask & SPI_UNIT_INT_MASK)
    {
        spi->STATUS = SPI_STATUS_UNITIF_Msk; /* Clear unit transfer interrupt flag */
    }

    if(u32Mask & SPI_SSACT_INT_MASK)
    {
        spi->STATUS = SPI_STATUS_SSACTIF_Msk; /* Clear slave selection signal active interrupt flag */
    }

    if(u32Mask & SPI_SSINACT_INT_MASK)
    {
        spi->STATUS = SPI_STATUS_SSINAIF_Msk; /* Clear slave selection signal inactive interrupt flag */
    }

    if(u32Mask & SPI_SLVUR_INT_MASK)
    {
        spi->STATUS = SPI_STATUS_SLVURIF_Msk; /* Clear slave TX under run interrupt flag */
    }

    if(u32Mask & SPI_SLVBE_INT_MASK)
    {
        spi->STATUS = SPI_STATUS_SLVBEIF_Msk; /* Clear slave bit count error interrupt flag */
    }

    if(u32Mask & SPI_TXUF_INT_MASK)
    {
        spi->STATUS = SPI_STATUS_TXUFIF_Msk; /* Clear slave TX underflow interrupt flag */
    }

    if(u32Mask & SPI_FIFO_RXOV_INT_MASK)
    {
        spi->STATUS = SPI_STATUS_RXOVIF_Msk; /* Clear RX overrun interrupt flag */
    }

    if(u32Mask & SPI_FIFO_RXTO_INT_MASK)
    {
        spi->STATUS = SPI_STATUS_RXTOIF_Msk; /* Clear RX time-out interrupt flag */
    }
}

/**
  * @brief  Get SPI status.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related sources.
  *                     Each bit corresponds to a source.
  *                     This parameter decides which flags will be read. It is combination of:
  *                       - \ref SPI_BUSY_MASK
  *                       - \ref SPI_RX_EMPTY_MASK
  *                       - \ref SPI_RX_FULL_MASK
  *                       - \ref SPI_TX_EMPTY_MASK
  *                       - \ref SPI_TX_FULL_MASK
  *                       - \ref SPI_TXRX_RESET_MASK
  *                       - \ref SPI_SPIEN_STS_MASK
  *                       - \ref SPI_SSLINE_STS_MASK
  *
  * @return Flags of selected sources.
  * @details Get SPI related status specified by u32Mask parameter.
  */
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32Flag = 0U, u32TmpValue;

    u32TmpValue = spi->STATUS & SPI_STATUS_BUSY_Msk;
    /* Check busy status */
    if((u32Mask & SPI_BUSY_MASK) && (u32TmpValue))
    {
        u32Flag |= SPI_BUSY_MASK;
    }

    u32TmpValue = spi->STATUS & SPI_STATUS_RXEMPTY_Msk;
    /* Check RX empty flag */
    if((u32Mask & SPI_RX_EMPTY_MASK) && (u32TmpValue))
    {
        u32Flag |= SPI_RX_EMPTY_MASK;
    }

    u32TmpValue = spi->STATUS & SPI_STATUS_RXFULL_Msk;
    /* Check RX full flag */
    if((u32Mask & SPI_RX_FULL_MASK) && (u32TmpValue))
    {
        u32Flag |= SPI_RX_FULL_MASK;
    }

    u32TmpValue = spi->STATUS & SPI_STATUS_TXEMPTY_Msk;
    /* Check TX empty flag */
    if((u32Mask & SPI_TX_EMPTY_MASK) && (u32TmpValue))
    {
        u32Flag |= SPI_TX_EMPTY_MASK;
    }

    u32TmpValue = spi->STATUS & SPI_STATUS_TXFULL_Msk;
    /* Check TX full flag */
    if((u32Mask & SPI_TX_FULL_MASK) && (u32TmpValue))
    {
        u32Flag |= SPI_TX_FULL_MASK;
    }

    u32TmpValue = spi->STATUS & SPI_STATUS_TXRXRST_Msk;
    /* Check TX/RX reset flag */
    if((u32Mask & SPI_TXRX_RESET_MASK) && (u32TmpValue))
    {
        u32Flag |= SPI_TXRX_RESET_MASK;
    }

    u32TmpValue = spi->STATUS & SPI_STATUS_SPIENSTS_Msk;
    /* Check SPIEN flag */
    if((u32Mask & SPI_SPIEN_STS_MASK) && (u32TmpValue))
    {
        u32Flag |= SPI_SPIEN_STS_MASK;
    }

    u32TmpValue = spi->STATUS & SPI_STATUS_SSLINE_Msk;
    /* Check SPIx_SS line status */
    if((u32Mask & SPI_SSLINE_STS_MASK) && (u32TmpValue))
    {
        u32Flag |= SPI_SSLINE_STS_MASK;
    }

    return u32Flag;
}

/*@}*/ /* end of group SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SPI_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
