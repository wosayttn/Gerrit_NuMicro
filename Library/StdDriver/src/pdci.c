/**************************************************************************//**
 * @file     pdci.c
 * @version  V3.00
 * @brief    Parallel Dimming Communication Interface(PDCI) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup PDCI_Driver PDCI Driver
  @{
*/


/** @addtogroup PDCI_EXPORTED_FUNCTIONS PDCI Exported Functions
  @{
*/

/**
  * @brief      Set the PDCI bit clock
  * @param[in]  u32BitClock The expected frequency of PDCI bit clock in Hz.
  * @return     Actual frequency of PDCI bit clock
  * @details    The actual clock rate may be different from the target PDCI bit clock rate.
  *             For example, if the system clock rate is 160 MHz and the target PDCI bit clock rate is 3 MHz, the actual PDCI bit clock
  *             rate will be 2 MHz.
  * \hideinitializer
  */
uint32_t PDCI_SetBitClock(uint32_t u32BitClock)
{
    uint32_t u32HCLKFreq, u32Div, u32RetValue;

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    u32Div = ((u32HCLKFreq * 10UL) / u32BitClock + 5UL) / 10UL; /* Round to the nearest integer */

    PDCI->CTL = (PDCI->CTL & (~PDCI_CTL_BTDIV_Msk)) | (u32Div << PDCI_CTL_BTDIV_Pos);

    /* Return PDCI bit clock rate */
    u32RetValue = u32HCLKFreq / u32Div;

    return u32RetValue;
}

/**
  * @brief      Get the actual frequency of PDCI bit clock
  * @return     Actual PDCI bit frequency in Hz
  * @details    This API will calculate the actual PDCI bit clock rate according to the BTDIV setting.
  * \hideinitializer
  */
uint32_t PDCI_GetBitClock(void)
{
    uint32_t u32HCLKFreq, u32Div;

    /* Get BTDIV setting */
    u32Div = (PDCI->CTL & PDCI_CTL_BTDIV_Msk) >> PDCI_CTL_BTDIV_Pos;

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    /* Return PDCI bit clock rate */
    return (u32HCLKFreq / u32Div);
}

/**
  * @brief      Set the PDCI first narrow width of data 1
  * @param[in]  u32NarrowWidthPct The percentage of the first narrow width.
  * @return     Actual percentage of the first narrow width
  * @details    This API will set the narrow bit clock based on the BTDIV setting and the expected narrow width percentage.
 */
uint32_t PDCI_SetNarrowWidthPct(uint32_t u32NarrowWidthPct)
{
    uint32_t u32Div, u32NarrowDiv;

    /* Get BTDIV setting */
    u32Div = (PDCI->CTL & PDCI_CTL_BTDIV_Msk) >> PDCI_CTL_BTDIV_Pos;

    /* Calculate narrow bit clock divider */
    u32NarrowDiv = (u32Div * u32NarrowWidthPct) / 100UL;

    PDCI->CTL = (PDCI->CTL & (~PDCI_CTL_NBTDIV_Msk)) | (u32NarrowDiv << PDCI_CTL_NBTDIV_Pos);

    /* Return the PDCI first narrow width percentage */
    return (u32NarrowDiv * 100UL / u32Div);
}

/**
  * @brief      Get the PDCI first narrow width of data 1
  * @return     The first narrow width in Hz
  * @details    This API will calculate the actual PDCI first narrow width of data 1 according to the BTDIV and NBTDIV settings.
  * \hideinitializer
  */
uint32_t PDCI_GetFirstNarrowWidth(void)
{
    uint32_t u32Div, u32NarrowDiv;

    /* Get BTDIV setting */
    u32Div = (PDCI->CTL & PDCI_CTL_BTDIV_Msk) >> PDCI_CTL_BTDIV_Pos;

    /* Get NBTDIV setting */
    u32NarrowDiv = (PDCI->CTL & PDCI_CTL_NBTDIV_Msk) >> PDCI_CTL_NBTDIV_Pos;

    /* Return the first narrow width */
    return (PDCI_GetBitClock() * u32Div) / u32NarrowDiv;
}

/**
  * @brief      Set the PDCI receive data threshold
  * @param[in]  u32RxThrPct The percentage of the receive data threshold. It is recommended to set to 15~20%.
  * @return     Actual percentage of the receive data threshold
  * @details    This API will set the receive data threshold based on the BTDIV setting and the expected receive data threshold percentage.
 */
uint32_t PDCI_SetRxThreshold(uint32_t u32RxThrPct)
{
    uint32_t u32Div, u32RxThr;

    /* Get BTDIV setting */
    u32Div = (PDCI->CTL & PDCI_CTL_BTDIV_Msk) >> PDCI_CTL_BTDIV_Pos;

    /* Calculate receive data threshold */
    u32RxThr = (u32Div * u32RxThrPct / 100UL) - 1UL;

    PDCI->RXCTL0 = (PDCI->RXCTL0 & (~PDCI_RXCTL0_RXTHRES_Msk)) | (u32RxThr << PDCI_RXCTL0_RXTHRES_Pos);

    /* Return the PDCI receive data threshold percentage */
    return ((u32RxThr + 1UL) * 100UL / u32Div);
}

/**
  * @brief      Set the dummy delay time period of each group
  * @param[in]  u32ChGroup PDCI channel group selection, valid values are:
  *                        - \ref PDCI_GROUP_0
  *                        - \ref PDCI_GROUP_1
  *                        - \ref PDCI_GROUP_2
  *                        - \ref PDCI_GROUP_3
  *                        - \ref PDCI_GROUP_4
  *                        - \ref PDCI_GROUP_5
  * @param[in]  u32DumDelay The expected PDCI dummy delay period in microsecond.
  * @return     Actual dummy delay time period in microsecond
  * @details    This API is used to set each group dummy delay time period.
  * \hideinitializer
  */
uint32_t PDCI_SetDummyDelayPeriod(uint32_t u32ChGroup, uint32_t u32DumDelay)
{
    uint32_t i, u32BitNum;

    u32BitNum = ((PDCI_GetBitClock() / 8UL) * u32DumDelay) / 1000000UL;

    for(i = 0UL; i < (uint32_t)PDCI_CHANNEL_NUM; i += 4UL)
    {
        if(u32ChGroup == i)
        {
            if(u32ChGroup <= PDCI_GROUP_3)
            {
                outp8((uint32_t)&(PDCI->DNUM0) + (i >> 2UL), u32BitNum);
            }
            else
            {
                outp8((uint32_t)&(PDCI->DNUM1) + ((i >> 2UL) - 4UL), u32BitNum);
            }
            break;
        }
    }

    /* Return PDCI dummy delay time period */
    return ((8UL * 1000000UL) / PDCI_GetBitClock()) * u32BitNum;
}

/**
  * @brief      Write K-Code data
  * @param[in]  u8Kcode1 The first K-Code data.
  * @param[in]  u8Kcode2 The second K-Code data.
  * @param[in]  u8Kcode3 The third K-Code data.
  * @param[in]  u8Kcode4 The fourth K-Code data.
  * @return     None
  * @details    This API is used to write the first to fourth K-Code data.
  * \hideinitializer
  */
void PDCI_WriteKcodeData(uint8_t u8Kcode1, uint8_t u8Kcode2, uint8_t u8Kcode3, uint8_t u8Kcode4)
{
    uint8_t u8Kcodes[] = {u8Kcode1, u8Kcode2, u8Kcode3, u8Kcode4};
    uint32_t i;

    for(i = 0UL; i < 4UL; i++)
        outp8((uint32_t)&(PDCI->KCODE) + i, u8Kcodes[i]);
}

/**
  * @brief      Configure PDCI IO settings
  * @param[in]  u8SrcGroup The source group IO, valid values are:
  *                        - \ref PDCI_IO_GROUP_0
  *                        - \ref PDCI_IO_GROUP_1
  *                        - \ref PDCI_IO_GROUP_2
  *                        - \ref PDCI_IO_GROUP_3
  *                        - \ref PDCI_IO_GROUP_4
  *                        - \ref PDCI_IO_GROUP_5
  * @param[in]  u8DstGroup The destination group IO, valid values are:
  *                        - \ref PDCI_IO_GROUP_0
  *                        - \ref PDCI_IO_GROUP_1
  *                        - \ref PDCI_IO_GROUP_2
  *                        - \ref PDCI_IO_GROUP_3
  *                        - \ref PDCI_IO_GROUP_4
  *                        - \ref PDCI_IO_GROUP_5
  * @return     None
  * @details    This API configures the source IO group of the destination IO group.
  * \hideinitializer
  */
void PDCI_ConfigIO(uint8_t u8SrcGroup, uint8_t u8DstGroup)
{
    uint8_t u8Offset, u8Shift, u8CurrentValue;
    uint32_t u32Addr;

    u8Offset = u8SrcGroup / 2UL;
    u8Shift = (u8SrcGroup % 2UL) ? 0x4 : 0x0;

    u32Addr = (uint32_t)&(PDCI->CONFIGIO) + u8Offset;

    u8CurrentValue = inp8(u32Addr);
    u8CurrentValue = u8CurrentValue & ~(0xF << u8Shift) | (u8DstGroup << u8Shift);

    outp8(u32Addr, u8CurrentValue);
}

/**
  * @brief      Enable interrupt function
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt enable bit.
  *                     This parameter decides which interrupts will be enabled. It is combination of:
  *                     - \ref PDCI_FTXD_INT_MASK
  *                     - \ref PDCI_TXUND_INT_MASK
  *                     - \ref PDCI_FEMPT_INT_MASK
  *                     - \ref PDCI_RXFULL_INT_MASK
  *                     - \ref PDCI_RXDONE_INT_MASK
  *                     - \ref PDCI_RXOV_INT_MASK
  *                     - \ref PDCI_RXFAIL_INT_MASK
  *                     - \ref PDCI_RXCRC_INT_MASK
  *                     - \ref PDCI_RXCRCA_INT_MASK
  * @return     None
  * @details    This API is used to enable PDCI related interrupts specified by u32Mask parameter.
  * \hideinitializer
  */
void PDCI_EnableInt(uint32_t u32Mask)
{
    /* Enable frame transmit done interrupt flag */
    if((u32Mask & PDCI_FTXD_INT_MASK) == PDCI_FTXD_INT_MASK)
    {
        PDCI->INTEN |= PDCI_INTEN_FTXDIEN_Msk;
    }

    /* Enable transmit data under run interrupt flag */
    if((u32Mask & PDCI_TXUND_INT_MASK) == PDCI_TXUND_INT_MASK)
    {
        PDCI->INTEN |= PDCI_INTEN_TXUNDIEN_Msk;
    }

    /* Enable FIFO empty interrupt flag */
    if((u32Mask & PDCI_FEMPT_INT_MASK) == PDCI_FEMPT_INT_MASK)
    {
        PDCI->INTEN |= PDCI_INTEN_FEMPTIEN_Msk;
    }

    /* Enable receiver FIFO full interrupt flag */
    if((u32Mask & PDCI_RXFULL_INT_MASK) == PDCI_RXFULL_INT_MASK)
    {
        PDCI->RXCTL0 |= PDCI_RXCTL0_RXFIEN_Msk;
    }

    /* Enable receiver transfer done interrupt flag */
    if((u32Mask & PDCI_RXDONE_INT_MASK) == PDCI_RXDONE_INT_MASK)
    {
        PDCI->RXCTL0 |= PDCI_RXCTL0_RXDIEN_Msk;
    }

    /* Enable receiver FIFO overflow interrupt flag */
    if((u32Mask & PDCI_RXOV_INT_MASK) == PDCI_RXOV_INT_MASK)
    {
        PDCI->RXCTL0 |= PDCI_RXCTL0_RXOVIEN_Msk;
    }

    /* Enable receiver failure interrupt flag */
    if((u32Mask & PDCI_RXFAIL_INT_MASK) == PDCI_RXFAIL_INT_MASK)
    {
        PDCI->RXCTL0 |= PDCI_RXCTL0_RXFAILIEN_Msk;
    }

    /* Enable receiver CRC error interrupt flag */
    if((u32Mask & PDCI_RXCRC_INT_MASK) == PDCI_RXCRC_INT_MASK)
    {
        PDCI->RXCTL0 |= PDCI_RXCTL0_RXCRCIEN_Msk;
    }

    /* Enable receiver CRC address error interrupt flag */
    if((u32Mask & PDCI_RXCRCA_INT_MASK) == PDCI_RXCRCA_INT_MASK)
    {
        PDCI->RXCTL0 |= PDCI_RXCTL0_RXCRCAIEN_Msk;
    }
}

/**
  * @brief      Disable interrupt function
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt enable bit.
  *                     This parameter decides which interrupts will be disabled. It is combination of:
  *                     - \ref PDCI_FTXD_INT_MASK
  *                     - \ref PDCI_TXUND_INT_MASK
  *                     - \ref PDCI_FEMPT_INT_MASK
  *                     - \ref PDCI_RXFULL_INT_MASK
  *                     - \ref PDCI_RXDONE_INT_MASK
  *                     - \ref PDCI_RXOV_INT_MASK
  *                     - \ref PDCI_RXFAIL_INT_MASK
  *                     - \ref PDCI_RXCRC_INT_MASK
  *                     - \ref PDCI_RXCRCA_INT_MASK
  * @return     None
  * @details    This API is used to disable PDCI related interrupts specified by u32Mask parameter.
  * \hideinitializer
  */
void PDCI_DisableInt(uint32_t u32Mask)
{
    /* Disable frame transmit done interrupt flag */
    if((u32Mask & PDCI_FTXD_INT_MASK) == PDCI_FTXD_INT_MASK)
    {
        PDCI->INTEN &= ~PDCI_INTEN_FTXDIEN_Msk;
    }

    /* Disable transmit data under run interrupt flag */
    if((u32Mask & PDCI_TXUND_INT_MASK) == PDCI_TXUND_INT_MASK)
    {
        PDCI->INTEN &= ~PDCI_INTEN_TXUNDIEN_Msk;
    }

    /* Disable FIFO empty interrupt flag */
    if((u32Mask & PDCI_FEMPT_INT_MASK) == PDCI_FEMPT_INT_MASK)
    {
        PDCI->INTEN &= ~PDCI_INTEN_FEMPTIEN_Msk;
    }

    /* Disable receiver FIFO full interrupt flag */
    if((u32Mask & PDCI_RXFULL_INT_MASK) == PDCI_RXFULL_INT_MASK)
    {
        PDCI->RXCTL0 &= ~PDCI_RXCTL0_RXFIEN_Msk;
    }

    /* Disable receiver transfer done interrupt flag */
    if((u32Mask & PDCI_RXDONE_INT_MASK) == PDCI_RXDONE_INT_MASK)
    {
        PDCI->RXCTL0 &= ~PDCI_RXCTL0_RXDIEN_Msk;
    }

    /* Disable receiver FIFO overflow interrupt flag */
    if((u32Mask & PDCI_RXOV_INT_MASK) == PDCI_RXOV_INT_MASK)
    {
        PDCI->RXCTL0 &= ~PDCI_RXCTL0_RXOVIEN_Msk;
    }

    /* Disable receiver failure interrupt flag */
    if((u32Mask & PDCI_RXFAIL_INT_MASK) == PDCI_RXFAIL_INT_MASK)
    {
        PDCI->RXCTL0 &= ~PDCI_RXCTL0_RXFAILIEN_Msk;
    }

    /* Disable receiver CRC error interrupt flag */
    if((u32Mask & PDCI_RXCRC_INT_MASK) == PDCI_RXCRC_INT_MASK)
    {
        PDCI->RXCTL0 &= ~PDCI_RXCTL0_RXCRCIEN_Msk;
    }

    /* Disable receiver CRC address error interrupt flag */
    if((u32Mask & PDCI_RXCRCA_INT_MASK) == PDCI_RXCRCA_INT_MASK)
    {
        PDCI->RXCTL0 &= ~PDCI_RXCTL0_RXCRCAIEN_Msk;
    }
}

/**
  * @brief      Get interrupt flag
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be read. It is combination of:
  *                     - \ref PDCI_FTXD_INT_MASK
  *                     - \ref PDCI_TXUND_INT_MASK
  *                     - \ref PDCI_FEMPT_INT_MASK
  * @return     Interrupt flags of selected sources
  * @details    This API is used to get PDCI related interrupt flags specified by u32Mask parameter.
  * \hideinitializer
  */
uint32_t PDCI_GetIntFlag(uint32_t u32Mask)
{
    uint32_t u32IntStatus;
    uint32_t u32IntFlag = 0UL;

    u32IntStatus = PDCI->INTSTS;

    /* Check frame transmit done interrupt flag */
    if((u32Mask & PDCI_FTXD_INT_MASK) && (u32IntStatus & PDCI_INTSTS_FTXDIF_Msk))
    {
        u32IntFlag |= PDCI_FTXD_INT_MASK;
    }

    /* Check transmit data under run interrupt flag */
    if((u32Mask & PDCI_TXUND_INT_MASK) && (u32IntStatus & PDCI_INTSTS_TXUNDIF_Msk))
    {
        u32IntFlag |= PDCI_TXUND_INT_MASK;
    }

    /* Check FIFO empty interrupt flag */
    if((u32Mask & PDCI_FEMPT_INT_MASK) && (u32IntStatus & PDCI_INTSTS_FEMPTIF_Msk))
    {
        u32IntFlag |= PDCI_FEMPT_INT_MASK;
    }

    return u32IntFlag;
}

/**
  * @brief      Get PDCI status
  * @param[in]  u32Mask The combination of all related sources.
  *                     Each bit corresponds to a related source.
  *                     This parameter decides which flags will be read. It is combination of:
  *                     - \ref PDCI_G0TXUND_MASK
  *                     - \ref PDCI_G1TXUND_MASK
  *                     - \ref PDCI_G2TXUND_MASK
  *                     - \ref PDCI_G3TXUND_MASK
  *                     - \ref PDCI_G4TXUND_MASK
  *                     - \ref PDCI_G5TXUND_MASK
  *                     - \ref PDCI_G0TXDF_MASK
  *                     - \ref PDCI_G1TXDF_MASK
  *                     - \ref PDCI_G2TXDF_MASK
  *                     - \ref PDCI_G3TXDF_MASK
  *                     - \ref PDCI_G4TXDF_MASK
  *                     - \ref PDCI_G5TXDF_MASK
  * @return     Flags of selected sources
  * @details    This API is used to get PDCI related status specified by u32Mask parameter.
  * \hideinitializer
  */
uint32_t PDCI_GetStatus(uint32_t u32Mask)
{
    uint32_t u32TmpStatus;
    uint32_t u32Flag = 0UL;

    u32TmpStatus = PDCI->INTSTS;

    /* Check group 0 transmit data under run status */
    if((u32Mask & PDCI_G0TXUND_MASK) && (u32TmpStatus & PDCI_INTSTS_G0TXUND_Msk))
    {
        u32Flag |= PDCI_G0TXUND_MASK;
    }

    /* Check group 1 transmit data under run status */
    if((u32Mask & PDCI_G1TXUND_MASK) && (u32TmpStatus & PDCI_INTSTS_G1TXUND_Msk))
    {
        u32Flag |= PDCI_G1TXUND_MASK;
    }

    /* Check group 2 transmit data under run status */
    if((u32Mask & PDCI_G2TXUND_MASK) && (u32TmpStatus & PDCI_INTSTS_G2TXUND_Msk))
    {
        u32Flag |= PDCI_G2TXUND_MASK;
    }

    /* Check group 3 transmit data under run status */
    if((u32Mask & PDCI_G3TXUND_MASK) && (u32TmpStatus & PDCI_INTSTS_G3TXUND_Msk))
    {
        u32Flag |= PDCI_G3TXUND_MASK;
    }

    /* Check group 4 transmit data under run status */
    if((u32Mask & PDCI_G4TXUND_MASK) && (u32TmpStatus & PDCI_INTSTS_G4TXUND_Msk))
    {
        u32Flag |= PDCI_G4TXUND_MASK;
    }

    /* Check group 5 transmit data under run status */
    if((u32Mask & PDCI_G5TXUND_MASK) && (u32TmpStatus & PDCI_INTSTS_G5TXUND_Msk))
    {
        u32Flag |= PDCI_G5TXUND_MASK;
    }

    /* Check group 0 transmit done status */
    if((u32Mask & PDCI_G0TXDF_MASK) && (u32TmpStatus & PDCI_INTSTS_G0TXDF_Msk))
    {
        u32Flag |= PDCI_G0TXDF_MASK;
    }

    /* Check group 1 transmit done status */
    if ((u32Mask & PDCI_G1TXDF_MASK) && (u32TmpStatus & PDCI_INTSTS_G1TXDF_Msk))
    {
        u32Flag |= PDCI_G1TXDF_MASK;
    }

    /* Check group 2 transmit done status */
    if ((u32Mask & PDCI_G2TXDF_MASK) && (u32TmpStatus & PDCI_INTSTS_G2TXDF_Msk))
    {
        u32Flag |= PDCI_G2TXDF_MASK;
    }

    /* Check group 3 transmit done status */
    if ((u32Mask & PDCI_G3TXDF_MASK) && (u32TmpStatus & PDCI_INTSTS_G3TXDF_Msk))
    {
        u32Flag |= PDCI_G3TXDF_MASK;
    }

    /* Check group 4 transmit done status */
    if ((u32Mask & PDCI_G4TXDF_MASK) && (u32TmpStatus & PDCI_INTSTS_G4TXDF_Msk))
    {
        u32Flag |= PDCI_G4TXDF_MASK;
    }

    /* Check group 5 transmit done status */
    if ((u32Mask & PDCI_G5TXDF_MASK) && (u32TmpStatus & PDCI_INTSTS_G5TXDF_Msk))
    {
        u32Flag |= PDCI_G5TXDF_MASK;
    }

    return u32Flag;
}

/**
  * @brief      Clear PDCI status
  * @param[in]  u32Mask The combination of all related sources.
  *                     Each bit corresponds to a related source.
  *                     This parameter decides which flags will be cleared. It is combination of:
  *                     - \ref PDCI_G0TXUND_MASK
  *                     - \ref PDCI_G1TXUND_MASK
  *                     - \ref PDCI_G2TXUND_MASK
  *                     - \ref PDCI_G3TXUND_MASK
  *                     - \ref PDCI_G4TXUND_MASK
  *                     - \ref PDCI_G5TXUND_MASK
  *                     - \ref PDCI_G0TXDF_MASK
  *                     - \ref PDCI_G1TXDF_MASK
  *                     - \ref PDCI_G2TXDF_MASK
  *                     - \ref PDCI_G3TXDF_MASK
  *                     - \ref PDCI_G4TXDF_MASK
  *                     - \ref PDCI_G5TXDF_MASK
  * @return     None
  * @details    This API is used to clear PDCI related status specified by u32Mask parameter.
  * \hideinitializer
  */
void PDCI_ClearStatus(uint32_t u32Mask)
{
    if(u32Mask & PDCI_G0TXUND_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G0TXUND_Msk; /* Clear group 0 transmit data under run status */
    }

    if(u32Mask & PDCI_G1TXUND_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G1TXUND_Msk; /* Clear group 1 transmit data under run status */
    }

    if(u32Mask & PDCI_G2TXUND_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G2TXUND_Msk; /* Clear group 2 transmit data under run status */
    }

    if(u32Mask & PDCI_G3TXUND_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G3TXUND_Msk; /* Clear group 3 transmit data under run status */
    }

    if(u32Mask & PDCI_G4TXUND_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G4TXUND_Msk; /* Clear group 4 transmit data under run status */
    }

    if(u32Mask & PDCI_G5TXUND_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G5TXUND_Msk; /* Clear group 5 transmit data under run status */
    }

    if(u32Mask & PDCI_G0TXDF_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G0TXDF_Msk; /* Clear group 0 transmit data flag status */
    }

    if(u32Mask & PDCI_G1TXDF_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G1TXDF_Msk; /* Clear group 1 transmit data flag status */
    }

    if(u32Mask & PDCI_G2TXDF_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G2TXDF_Msk; /* Clear group 2 transmit data flag status */
    }

    if(u32Mask & PDCI_G3TXDF_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G3TXDF_Msk; /* Clear group 3 transmit data flag status */
    }

    if(u32Mask & PDCI_G4TXDF_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G4TXDF_Msk; /* Clear group 4 transmit data flag status */
    }

    if(u32Mask & PDCI_G5TXDF_MASK)
    {
        PDCI->INTSTS = PDCI_INTSTS_G5TXDF_Msk; /* Clear group 5 transmit data flag status */
    }
}

/**
  * @brief      Get RX interrupt flag
  * @param[in]  u32ChNum The specified channel number. It could be 0 ~ 23.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be read. It is combination of:
  *                     - \ref PDCI_RXFULL_INT_MASK
  *                     - \ref PDCI_RXDONE_INT_MASK
  *                     - \ref PDCI_RXOV_INT_MASK
  *                     - \ref PDCI_RXFAIL_INT_MASK
  *                     - \ref PDCI_RXCRC_INT_MASK
  *                     - \ref PDCI_RXCRCA_INT_MASK
  * @return     Interrupt flags of selected sources
  * @details    This API is used to get PDCI related interrupt flags specified by u32ChNum and u32Mask parameter.
  * \hideinitializer
  */
uint32_t PDCI_GetRxIntFlag(uint32_t u32ChNum, uint32_t u32Mask)
{
    uint32_t u32IntStatus;
    uint32_t u32IntFlag = 0UL;

    if((u32Mask == PDCI_RXCRC_INT_MASK) || (u32Mask == PDCI_RXCRCA_INT_MASK))
    {
        if(u32ChNum < 16UL)
            u32IntStatus = PDCI->RXCRCSTS0;
        else if((16UL <= u32ChNum) && (u32ChNum < 24UL))
        {
            u32IntStatus = PDCI->RXCRCSTS1;
            u32ChNum = u32ChNum - 16UL;
        }
    }
    else
    {
        if(u32ChNum < 8UL)
        {
            u32IntStatus = PDCI->RXG01STS;
        }
        else if((8UL <= u32ChNum) && (u32ChNum < 16UL))
        {
            u32IntStatus = PDCI->RXG23STS;
            u32ChNum = u32ChNum - 8UL;
        }
        else if((16UL <= u32ChNum) && (u32ChNum < 24UL))
        {
            u32IntStatus = PDCI->RXG45STS;
            u32ChNum = u32ChNum - 16UL;
        }
    }

    /* Check receiver data FIFO full interrupt flag */
    if((u32Mask & PDCI_RXFULL_INT_MASK) && (u32IntStatus & (1UL << u32ChNum)))
    {
        u32IntFlag |= PDCI_RXFULL_INT_MASK;
    }

    /* Check receiver transfer done interrupt flag */
    if((u32Mask & PDCI_RXDONE_INT_MASK) && (u32IntStatus & (1UL << (u32ChNum + 8UL))))
    {
        u32IntFlag |= PDCI_RXDONE_INT_MASK;
    }

    /* Check receiver FIFO overflow interrupt flag */
    if((u32Mask & PDCI_RXOV_INT_MASK) && (u32IntStatus & (1UL << (u32ChNum + 16UL))))
    {
        u32IntFlag |= PDCI_RXOV_INT_MASK;
    }

    /* Check receiver fail interrupt flag */
    if((u32Mask & PDCI_RXFAIL_INT_MASK) && (u32IntStatus & (1UL << (u32ChNum + 24UL))))
    {
        u32IntFlag |= PDCI_RXFAIL_INT_MASK;
    }

    /* Check receiver CRC error interrupt flag */
    if((u32Mask & PDCI_RXCRC_INT_MASK) && (u32IntStatus & (1UL << u32ChNum)))
    {
        u32IntFlag |= PDCI_RXCRC_INT_MASK;
    }

    /* Check receiver CRC address error interrupt flag */
    if((u32Mask & PDCI_RXCRCA_INT_MASK) && (u32IntStatus & (1UL << (u32ChNum + 16UL))))
    {
        u32IntFlag |= PDCI_RXCRCA_INT_MASK;
    }

    return u32IntFlag;
}

/**
  * @brief      Clear RX interrupt flag
  * @param[in]  u32ChNum The specified channel number. It could be 0 ~ 23.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be read. It is combination of:
  *                     - \ref PDCI_RXFULL_INT_MASK
  *                     - \ref PDCI_RXDONE_INT_MASK
  *                     - \ref PDCI_RXOV_INT_MASK
  *                     - \ref PDCI_RXFAIL_INT_MASK
  *                     - \ref PDCI_RXCRC_INT_MASK
  *                     - \ref PDCI_RXCRCA_INT_MASK
  * @return     None
  * @details    This API is used to clear RX interrupt flag specified by u32ChNum and u32Mask parameter.
  * \hideinitializer
  */
void PDCI_ClearRxIntFlag(uint32_t u32ChNum, uint32_t u32Mask)
{
    if(u32ChNum < 8UL)
    {
        if(u32Mask & PDCI_RXFULL_INT_MASK)
        {
            PDCI->RXG01STS = 1UL << (u32ChNum + 0UL); /* Clear receiver 0~7 data FIFO full status */
        }

        if(u32Mask & PDCI_RXDONE_INT_MASK)
        {
            PDCI->RXG01STS = 1UL << (u32ChNum + 8UL); /* Clear receiver 0~7 transfer done status */
        }

        if(u32Mask & PDCI_RXOV_INT_MASK)
        {
            PDCI->RXG01STS = 1UL << (u32ChNum + 16UL); /* Clear receiver 0~7 FIFO overflow status */
        }

        if(u32Mask & PDCI_RXFAIL_INT_MASK)
        {
            PDCI->RXG01STS = 1UL << (u32ChNum + 24UL); /* Clear receiver 0~7 failure status */
        }

        if(u32Mask & PDCI_RXCRC_INT_MASK)
        {
            PDCI->RXCRCSTS0 = 1UL << (u32ChNum + 0UL); /* Clear receiver 0~7 CRC error status */
        }

        if(u32Mask & PDCI_RXCRCA_INT_MASK)
        {
            PDCI->RXCRCSTS0 = 1UL << (u32ChNum + 16UL); /* Clear receiver 0~7 CRC address error status */
        }
    }
    else if((8UL <= u32ChNum) && (u32ChNum < 16UL))
    {
        if(u32Mask & PDCI_RXFULL_INT_MASK)
        {
            PDCI->RXG23STS = 1UL << (u32ChNum - 8UL + 0UL); /* Clear receiver 8~15 data FIFO full status */
        }

        if(u32Mask & PDCI_RXDONE_INT_MASK)
        {
            PDCI->RXG23STS = 1UL << (u32ChNum - 8UL + 8UL); /* Clear receiver 8~15 transfer done status */
        }

        if(u32Mask & PDCI_RXOV_INT_MASK)
        {
            PDCI->RXG23STS = 1UL << (u32ChNum - 8UL + 16UL); /* Clear receiver 8~15 FIFO overflow status */
        }

        if(u32Mask & PDCI_RXFAIL_INT_MASK)
        {
            PDCI->RXG23STS = 1UL << (u32ChNum - 8UL + 24UL); /* Clear receiver 8~15 failure status */
        }

        if(u32Mask & PDCI_RXCRC_INT_MASK)
        {
            PDCI->RXCRCSTS0 = 1UL << (u32ChNum - 8UL + 8UL); /* Clear receiver 8~15 CRC error status */
        }

        if(u32Mask & PDCI_RXCRCA_INT_MASK)
        {
            PDCI->RXCRCSTS0 = 1UL << (u32ChNum - 8UL + 24UL); /* Clear receiver 8~15 CRC address error status */
        }
    }
    else if((16UL <= u32ChNum) && (u32ChNum < 24UL))
    {
        if(u32Mask & PDCI_RXFULL_INT_MASK)
        {
            PDCI->RXG45STS = 1UL << (u32ChNum - 16UL + 0UL); /* Clear receiver 16~23 data FIFO full status */
        }

        if(u32Mask & PDCI_RXDONE_INT_MASK)
        {
            PDCI->RXG45STS = 1UL << (u32ChNum - 16UL + 16UL); /* Clear receiver 16~23 transfer done status */
        }

        if(u32Mask & PDCI_RXOV_INT_MASK)
        {
            PDCI->RXG45STS = 1UL << (u32ChNum - 16UL + 16UL); /* Clear receiver 16~23 FIFO overflow status */
        }

        if(u32Mask & PDCI_RXFAIL_INT_MASK)
        {
            PDCI->RXG45STS = 1UL << (u32ChNum - 16UL + 24UL); /* Clear receiver 16~23 failure status */
        }

        if(u32Mask & PDCI_RXCRC_INT_MASK)
        {
            PDCI->RXCRCSTS1 = 1UL << (u32ChNum - 16UL + 0UL); /* Clear receiver 16~23 CRC error status */
        }

        if(u32Mask & PDCI_RXCRCA_INT_MASK)
        {
            PDCI->RXCRCSTS1 = 1UL << (u32ChNum - 16UL + 16UL); /* Clear receiver 16~23 CRC address error status */
        }
    }
}


/*@}*/ /* end of group PDCI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PDCI_Driver */

/*@}*/ /* end of group Standard_Driver */
