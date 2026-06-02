/**************************************************************************//**
 * @file     llsi.c
 * @version  V3.00
 * @brief    LED Light Strip Interface(LLSI) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"

#define LLSI_MODULE_NULL ((LLSI_T *)0)
#define LLSI_CONFIG_NULL ((S_LLSI_CONFIG_T *)0)
#define LLSI_TIME_INFO_NULL ((S_LLSI_TIME_INFO_T *)0)

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LLSI_Driver LLSI Driver
  @{
*/

/** @addtogroup LLSI_EXPORTED_FUNCTIONS LLSI Exported Functions
  @{
*/

/**
  * @brief  This function make LLSI module be ready to transfer.
  * @param[in]  llsi The pointer of the specified LLSI module.
  * @param[in]  u32LLSIMode Decides the transfer mode. (LLSI_MODE_SW, LLSI_MODE_PDMA)
  * @param[in]  u32OutputFormat Decides the output format of LLSI transaction. (LLSI_FORMAT_RGB, LLSI_FORMAT_GRB)
  * @param[in]  u32BusClock The expected frequency of LLSI bus clock in Hz.
  * @param[in]  u32TransferTimeNsec The expected period of LLSI data transfer time in nano second.
  * @param[in]  u32T0HTimeNsec The expected period of LLSI T0H data time in nano second.
  * @param[in]  u32T1HTimeNsec The expected period of LLSI T1H data time in nano second.
  * @param[in]  u32ResetTimeNsec The expected period of LLSI reset command time in nano second.
  * @param[in]  u32PCNT Decides a frame size.
  * @param[in]  u32IDOS Decides the idle output state. (LLSI_IDLE_LOW, LLSI_IDLE_HIGH)
  * @return None
  * @details By default, the LLSI uses software mode for transmission, the output format is RGB, and the idle state is low.
  *          The actual clock rate may be different from the target LLSI clock rate.
  *          For example, if the LLSI source clock rate is 12 MHz and the target LLSI bus clock rate is 7 MHz, the
  *          actual LLSI clock rate will be 6 MHz.
  * @note If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note If u32BusClock >= PCLK clock frequency, DIVIDER will be set to 0.
  */
void LLSI_Open(LLSI_T *llsi,
               uint32_t u32LLSIMode,
               uint32_t u32OutputFormat,
               uint32_t u32BusClock,
               uint32_t u32TransferTimeNsec,
               uint32_t u32T0HTimeNsec,
               uint32_t u32T1HTimeNsec,
               uint32_t u32ResetTimeNsec,
               uint32_t u32PCNT,
               uint32_t u32IDOS)
{
    uint32_t u32PCLKFreq;
    uint32_t u32Tmp1;
    uint32_t u32Tmp2;
    uint32_t u32Div;
    uint32_t u32Period;
    uint32_t u32T0H;
    uint32_t u32T1H;
    uint32_t u32ResetPeriod;

    if (llsi == LLSI_MODULE_NULL)
    {
        return;
    }

    /* Get PCLK clock frequency */
    if ((llsi == LLSI0) || (llsi == LLSI2) || (llsi == LLSI4) || (llsi == LLSI6) || (llsi == LLSI8))
    {
        u32PCLKFreq = CLK_GetPCLK0Freq();
    }
    else if ((llsi == LLSI1) || (llsi == LLSI3) || (llsi == LLSI5) || (llsi == LLSI7) || (llsi == LLSI9))
    {
        u32PCLKFreq = CLK_GetPCLK1Freq();
    }
    else
    {
        return;
    }

    /* Default setting: software mode, RGB format, idle ouput low. */
    llsi->CTL = (u32LLSIMode & LLSI_CTL_LLSIMODE_Msk) | (u32OutputFormat & LLSI_CTL_OFDEF_Msk);
    llsi->PCNT = (u32PCNT & LLSI_PCNT_PCNT_Msk);
    llsi->OCTL = (u32IDOS & LLSI_OCTL_IDOS_Msk);

    if (u32BusClock >= u32PCLKFreq)
    {
        /* Set DIVIDER = 0 */
        u32Div = 0UL;
        llsi->CLKDIV = 0UL;
    }
    else if (u32BusClock == 0UL)
    {
        /* Set DIVIDER to the maximum value 0xFF. f_llsi = f_llsi_clk_src / (DIVIDER + 1) */
        u32Div = 0xFFU;
        llsi->CLKDIV |= LLSI_CLKDIV_DIVIDER_Msk;
    }
    else
    {
        u32Div = ((((u32PCLKFreq * 10U) / u32BusClock) + 5U) / 10U) - 1U; /* Round to the nearest integer */

        if (u32Div > 0xFFU)
        {
            u32Div = 0xFFU;
            llsi->CLKDIV |= LLSI_CLKDIV_DIVIDER_Msk;
        }
        else
        {
            llsi->CLKDIV = (llsi->CLKDIV & (~LLSI_CLKDIV_DIVIDER_Msk)) | (u32Div << (uint32_t)LLSI_CLKDIV_DIVIDER_Pos);
        }
    }

    u32Tmp1 = (u32PCLKFreq * 10U) / 1000000U;
    u32Tmp2 = 1000U * (u32Div + 1U);

    u32Period = ((u32Tmp1 * u32TransferTimeNsec / u32Tmp2) + 5U) / 10U; /* Round to the nearest integer */

    if (u32Period > 0xFFU)
    {
        llsi->PERIOD |= LLSI_PERIOD_PERIOD_Msk;
    }
    else
    {
        llsi->PERIOD = (llsi->PERIOD & (~LLSI_PERIOD_PERIOD_Msk)) | (u32Period << (uint32_t)LLSI_PERIOD_PERIOD_Pos);
    }

    u32T0H = ((u32Tmp1 * u32T0HTimeNsec / u32Tmp2) + 5U) / 10U; /* Round to the nearest integer */

    if (u32T0H > 0xFFU)
    {
        llsi->DUTY |= LLSI_DUTY_T0H_Msk;
    }
    else
    {
        llsi->DUTY = (llsi->DUTY & (~LLSI_DUTY_T0H_Msk)) | (u32T0H << (uint32_t)LLSI_DUTY_T0H_Pos);
    }

    u32T1H = ((u32Tmp1 * u32T1HTimeNsec / u32Tmp2) + 5U) / 10U; /* Round to the nearest integer */

    if (u32T1H > 0xFFU)
    {
        llsi->DUTY |= LLSI_DUTY_T1H_Msk;
    }
    else
    {
        llsi->DUTY = (llsi->DUTY & (~LLSI_DUTY_T1H_Msk)) | (u32T1H << (uint32_t)LLSI_DUTY_T1H_Pos);
    }

    u32ResetPeriod = ((u32Tmp1 * u32ResetTimeNsec / u32Tmp2) + 5U) / 10U; /* Round to the nearest integer */

    if (u32ResetPeriod > 0xFFFFU)
    {
        llsi->RSTPERIOD |= LLSI_RSTPERIOD_RSTPERIOD_Msk;
    }
    else
    {
        llsi->RSTPERIOD = (llsi->RSTPERIOD & (~LLSI_RSTPERIOD_RSTPERIOD_Msk)) | (u32ResetPeriod << (uint32_t)LLSI_RSTPERIOD_RSTPERIOD_Pos);
    }

    /* LLSI_Open not Enable LLSI Tx */
}

/**
  * @brief  This function makes the LLSI module ready by applying its configuration.
  * @param[in]  llsi The pointer of the specified LLSI module.
  * @param[in]  sLLSIConfig Specify the LLSI information for Setting. It includes:
  *             u32LLSIMode Decides the transfer mode. (LLSI_MODE_SW, LLSI_MODE_PDMA)
  *             u32OutputFormat Decides the output format of LLSI transaction. (LLSI_FORMAT_RGB, LLSI_FORMAT_GRB)
  *             u32BusClock The expected frequency of LLSI bus clock in Hz.
  *             u32TransferTimeNsec The expected period of LLSI data transfer time in nano second.
  *             u32T0HTimeNsec The expected period of LLSI T0H data time in nano second.
  *             u32T1HTimeNsec The expected period of LLSI T1H data time in nano second.
  *             u32ResetTimeNsec The expected period of LLSI reset command time in nano second.
  *             u32PCNT Decides a frame size.
  *             u32IDOS Decides the idle output state. (LLSI_IDLE_LOW, LLSI_IDLE_HIGH)
  * @return None
  */
void LLSI_OpenbyConfig(LLSI_T *llsi, const S_LLSI_CONFIG_T *sLLSIConfig)
{
    if ((llsi == LLSI_MODULE_NULL) || (sLLSIConfig == LLSI_CONFIG_NULL))
    {
        return;
    }

    LLSI_Open(llsi, sLLSIConfig->u32LLSIMode, sLLSIConfig->u32OutputFormat, sLLSIConfig->sTimeInfo.u32BusClock,
              sLLSIConfig->sTimeInfo.u32TransferTimeNsec, sLLSIConfig->sTimeInfo.u32T0HTimeNsec,
              sLLSIConfig->sTimeInfo.u32T1HTimeNsec, sLLSIConfig->sTimeInfo.u32ResetTimeNsec,
              sLLSIConfig->u32PCNT, sLLSIConfig->u32IDOS);
}

/**
  * @brief  Disable LLSI controller.
  * @param[in]  llsi The pointer of the specified LLSI module.
  * @return None
  * @details Clear LLSIEN bit of LLSI_CTL register to disable LLSI transfer control.
  */
void LLSI_Close(LLSI_T *llsi)
{
    if (llsi == LLSI_MODULE_NULL)
    {
        return;
    }

    llsi->CTL &= ~LLSI_CTL_LLSIEN_Msk;
}

/**
  * @brief  Get current LLSI time information
  * @param[in]  llsi The pointer of the specified LLSI module.
  * @param[out]  sPt The returned pointer is specified the current LLSI value. It includes:
  *                  u32BusClock: Bus clock
  *                  u32TransferTimeNsec: Transfer time in nano second
  *                  u32T0HTimeNsec: T0H time in nano second
  *                  u32T1HTimeNsec: T1H time in nano second
  *                  u32ResetTimeNsec: Reset time in nano second
  * @return None
  * @details This API is used to get the current LLSI time information.
  */
void LLSI_GetTimeInfo(const LLSI_T *llsi, S_LLSI_TIME_INFO_T *sPt)
{
    uint32_t u32PCLKFreq = 0UL;
    uint32_t u32Tmp;
    uint32_t u32Div;
    uint32_t u32Period;
    uint32_t u32T0H;
    uint32_t u32T1H;
    uint32_t u32ResetPeriod;

    if ((llsi == LLSI_MODULE_NULL) || (sPt == LLSI_TIME_INFO_NULL))
    {
        return;
    }

    /* Get PCLK clock frequency */
    if ((llsi == LLSI0) || (llsi == LLSI2) || (llsi == LLSI4) || (llsi == LLSI6) || (llsi == LLSI8))
    {
        u32PCLKFreq = CLK_GetPCLK0Freq();
    }
    else if ((llsi == LLSI1) || (llsi == LLSI3) || (llsi == LLSI5) || (llsi == LLSI7) || (llsi == LLSI9))
    {
        u32PCLKFreq = CLK_GetPCLK1Freq();
    }
    else
    {
        return;
    }

    /* Get time data */
    u32Div = (llsi->CLKDIV & LLSI_CLKDIV_DIVIDER_Msk) >> (uint32_t)LLSI_CLKDIV_DIVIDER_Pos;
    u32Period = (llsi->PERIOD & LLSI_PERIOD_PERIOD_Msk) >> (uint32_t)LLSI_PERIOD_PERIOD_Pos;
    u32T0H = (llsi->DUTY & LLSI_DUTY_T0H_Msk) >> (uint32_t)LLSI_DUTY_T0H_Pos;
    u32T1H = (llsi->DUTY & LLSI_DUTY_T1H_Msk) >> (uint32_t)LLSI_DUTY_T1H_Pos;
    u32ResetPeriod = (llsi->RSTPERIOD & LLSI_RSTPERIOD_RSTPERIOD_Msk) >> (uint32_t)LLSI_RSTPERIOD_RSTPERIOD_Pos;

    /* Compute LLSI time information */
    sPt->u32BusClock = u32PCLKFreq / (u32Div + 1U);

    u32Tmp = u32PCLKFreq / 1000U;
    sPt->u32TransferTimeNsec = u32Period * 1000000U / u32Tmp * (u32Div + 1U);
    sPt->u32T0HTimeNsec =  u32T0H * 1000000U / u32Tmp * (u32Div + 1U);
    sPt->u32T1HTimeNsec =  u32T1H * 1000000U / u32Tmp * (u32Div + 1U);
    sPt->u32ResetTimeNsec =  u32ResetPeriod * 1000000U / u32Tmp * (u32Div + 1U);
}

/**
  * @brief  Configure FIFO threshold setting.
  * @param[in]  llsi The pointer of the specified LLSI module.
  * @param[in]  u32TxThreshold Decides the TX FIFO threshold. It could be 0 ~ 3.
  * @return None
  * @details Set TX FIFO threshold configuration.
  */
void LLSI_SetFIFO(LLSI_T *llsi, uint32_t u32TxThreshold)
{
    if (llsi == LLSI_MODULE_NULL)
    {
        return;
    }

    llsi->CTL = (llsi->CTL & ~LLSI_CTL_TXTH_Msk) | ((u32TxThreshold << ((uint32_t)LLSI_CTL_TXTH_Pos)) & LLSI_CTL_TXTH_Msk);
}

/**
  * @brief  Enable interrupt function.
  * @param[in]  llsi The pointer of the specified LLSI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt enable bit.
  *                     This parameter decides which interrupts will be enabled. It is combination of:
  *                       - \ref LLSI_UNDFL_INT_MASK
  *                       - \ref LLSI_FEND_INT_MASK
  *                       - \ref LLSI_RSTC_INT_MASK
  *                       - \ref LLSI_EMP_INT_MASK
  *                       - \ref LLSI_FUL_INT_MASK
  *                       - \ref LLSI_TXTH_INT_MASK
  *
  * @return None
  * @details Enable LLSI related interrupts specified by u32Mask parameter.
  */
void LLSI_EnableInt(LLSI_T *llsi, uint32_t u32Mask)
{
    if (llsi == LLSI_MODULE_NULL)
    {
        return;
    }

    /* Enable underflow interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_UNDFL_INT_MASK) == (uint32_t)LLSI_UNDFL_INT_MASK)
    {
        llsi->CTL |= LLSI_CTL_UNDFLINTEN_Msk;
    }

    /* Enable frame end interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_FEND_INT_MASK) == (uint32_t)LLSI_FEND_INT_MASK)
    {
        llsi->CTL |= LLSI_CTL_FENDINTEN_Msk;
    }

    /* Enable reset command interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_RSTC_INT_MASK) == (uint32_t)LLSI_RSTC_INT_MASK)
    {
        llsi->CTL |= LLSI_CTL_RSTCINTEN_Msk;
    }

    /* Enable FIFO empty interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_EMP_INT_MASK) == (uint32_t)LLSI_EMP_INT_MASK)
    {
        llsi->CTL |= LLSI_CTL_EMPINTEN_Msk;
    }

    /* Enable FIFO full interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_FUL_INT_MASK) == (uint32_t)LLSI_FUL_INT_MASK)
    {
        llsi->CTL |= LLSI_CTL_FULINTEN_Msk;
    }

    /* Enable TX threshold interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_TXTH_INT_MASK) == (uint32_t)LLSI_TXTH_INT_MASK)
    {
        llsi->CTL |= LLSI_CTL_TXTHIEN_Msk;
    }
}

/**
  * @brief  Disable interrupt function.
  * @param[in]  llsi The pointer of the specified LLSI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt bit.
  *                     This parameter decides which interrupts will be disabled. It is combination of:
  *                       - \ref LLSI_UNDFL_INT_MASK
  *                       - \ref LLSI_FEND_INT_MASK
  *                       - \ref LLSI_RSTC_INT_MASK
  *                       - \ref LLSI_EMP_INT_MASK
  *                       - \ref LLSI_FUL_INT_MASK
  *                       - \ref LLSI_TXTH_INT_MASK
  *
  * @return None
  * @details Disable LLSI related interrupts specified by u32Mask parameter.
  */
void LLSI_DisableInt(LLSI_T *llsi, uint32_t u32Mask)
{
    if (llsi == LLSI_MODULE_NULL)
    {
        return;
    }

    /* Disable underflow interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_UNDFL_INT_MASK) == (uint32_t)LLSI_UNDFL_INT_MASK)
    {
        llsi->CTL &= ~LLSI_CTL_UNDFLINTEN_Msk;
    }

    /* Disable frame end interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_FEND_INT_MASK) == (uint32_t)LLSI_FEND_INT_MASK)
    {
        llsi->CTL &= ~LLSI_CTL_FENDINTEN_Msk;
    }

    /* Disable reset command interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_RSTC_INT_MASK) == (uint32_t)LLSI_RSTC_INT_MASK)
    {
        llsi->CTL &= ~LLSI_CTL_RSTCINTEN_Msk;
    }

    /* Disable FIFO empty interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_EMP_INT_MASK) == (uint32_t)LLSI_EMP_INT_MASK)
    {
        llsi->CTL &= ~LLSI_CTL_EMPINTEN_Msk;
    }

    /* Disable FIFO full interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_FUL_INT_MASK) == (uint32_t)LLSI_FUL_INT_MASK)
    {
        llsi->CTL &= ~LLSI_CTL_FULINTEN_Msk;
    }

    /* Disable TX FIFO threshold interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_TXTH_INT_MASK) == (uint32_t)LLSI_TXTH_INT_MASK)
    {
        llsi->CTL &= ~LLSI_CTL_TXTHIEN_Msk;
    }
}

/**
  * @brief  Get interrupt flag.
  * @param[in]  llsi The pointer of the specified LLSI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be read. It is combination of:
  *                       - \ref LLSI_UNDFL_INT_MASK
  *                       - \ref LLSI_FEND_INT_MASK
  *                       - \ref LLSI_RSTC_INT_MASK
  *                       - \ref LLSI_EMP_INT_MASK
  *                       - \ref LLSI_FUL_INT_MASK
  *                       - \ref LLSI_TXTH_INT_MASK
  *
  * @return Interrupt flags of selected sources.
  * @details Get LLSI related interrupt flags specified by u32Mask parameter.
  */
uint32_t LLSI_GetIntFlag(const LLSI_T *llsi, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0UL;

    /* Check underflow interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_UNDFL_INT_MASK) && (llsi->STATUS & LLSI_STATUS_UNDFLIF_Msk))
    {
        u32IntFlag |= (uint32_t)LLSI_UNDFL_INT_MASK;
    }

    /* Check frame end interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_FEND_INT_MASK) && (llsi->STATUS & LLSI_STATUS_FENDIF_Msk))
    {
        u32IntFlag |= (uint32_t)LLSI_FEND_INT_MASK;
    }

    /* Check reset command interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_RSTC_INT_MASK) && (llsi->STATUS & LLSI_STATUS_RSTCIF_Msk))
    {
        u32IntFlag |= (uint32_t)LLSI_RSTC_INT_MASK;
    }

    /* Check FIFO empty interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_EMP_INT_MASK) && (llsi->STATUS & LLSI_STATUS_EMPIF_Msk))
    {
        u32IntFlag |= (uint32_t)LLSI_EMP_INT_MASK;
    }

    /* Check FIFO full interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_FUL_INT_MASK) && (llsi->STATUS & LLSI_STATUS_FULIF_Msk))
    {
        u32IntFlag |= (uint32_t)LLSI_FUL_INT_MASK;
    }

    /* Check TX FIFO threshold interrupt flag */
    if ((u32Mask & (uint32_t)LLSI_TXTH_INT_MASK) && (llsi->STATUS & LLSI_STATUS_TXTHIF_Msk))
    {
        u32IntFlag |= (uint32_t)LLSI_TXTH_INT_MASK;
    }

    return u32IntFlag;
}

/**
  * @brief  Clear interrupt flag.
  * @param[in]  llsi The pointer of the specified LLSI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be cleared. It could be the combination of:
  *                       - \ref LLSI_UNDFL_INT_MASK
  *                       - \ref LLSI_FEND_INT_MASK
  *                       - \ref LLSI_RSTC_INT_MASK
  *
  * @return None
  * @details Clear LLSI related interrupt flags specified by u32Mask parameter.
  */
void LLSI_ClearIntFlag(LLSI_T *llsi, uint32_t u32Mask)
{
    if (u32Mask & (uint32_t)LLSI_UNDFL_INT_MASK)
    {
        llsi->STATUS = LLSI_STATUS_UNDFLIF_Msk; /* Clear underflow interrupt flag */
    }

    if (u32Mask & (uint32_t)LLSI_FEND_INT_MASK)
    {
        llsi->STATUS = LLSI_STATUS_FENDIF_Msk; /* Clear frame end interrupt flag */
    }

    if (u32Mask & (uint32_t)LLSI_RSTC_INT_MASK)
    {
        llsi->STATUS = LLSI_STATUS_RSTCIF_Msk; /* Clear reset command interrupt flag */
    }
}

/** @} end of group LLSI_EXPORTED_FUNCTIONS */
/** @} end of group LLSI_Driver */
/** @} end of group Standard_Driver */

