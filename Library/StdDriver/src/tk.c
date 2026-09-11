/**************************************************************************//**
 * @file     tk.c
 * @version  V3.00
 * @brief    Touch key driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "tk.h"

static volatile uint32_t *TK_GetCCBDReg(uint32_t u32TKNum)
{
    /* cppcheck-suppress constVariable */
    static volatile uint32_t * const s_apu32CCBDReg[] =
    {
        &TK->CCBD0,
        &TK->CCBD1,
        &TK->CCBD2,
        &TK->CCBD3,
        &TK->CCBD4,
        &TK->CCBD5
    };


    uint32_t u32RegIndex;

    if (u32TKNum < 17UL)
    {
        u32RegIndex = u32TKNum / 4UL;
    }
    else
    {
        u32RegIndex = 5UL;
    }

    return s_apu32CCBDReg[u32RegIndex];
}

static volatile uint32_t *TK_GetREFCBDReg(uint32_t u32TKNum)
{
    /* cppcheck-suppress constVariable */
    static volatile uint32_t * const s_apu32REFCBDReg[] =
    {
        &TK->REFCBD0,
        &TK->REFCBD1,
        &TK->REFCBD2,
        &TK->REFCBD3,
        &TK->REFCBD4,
        &TK->REFCBD5
    };

    uint32_t u32RegIndex;

    if (u32TKNum < 17UL)
    {
        u32RegIndex = u32TKNum / 4UL;
    }
    else
    {
        u32RegIndex = 5UL;
    }

    return s_apu32REFCBDReg[u32RegIndex];
}

static volatile uint32_t *TK_GetTHCReg(uint32_t u32TKNum)
{
    /* cppcheck-suppress constVariable */
    static volatile uint32_t * const s_apu32THCReg[] =
    {
        &TK->THC01,
        &TK->THC23,
        &TK->THC45,
        &TK->THC67,
        &TK->THC89,
        &TK->THC1011,
        &TK->THC1213,
        &TK->THC1415,
        &TK->THC16,
        &TK->THC17
    };

    uint32_t u32RegIndex;

    if (u32TKNum < 17UL)
    {
        u32RegIndex = u32TKNum / 2UL;
    }
    else
    {
        u32RegIndex = 9UL;
    }

    return s_apu32THCReg[u32RegIndex];
}


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup TK_Driver TK Driver
  @{
*/


/** @addtogroup TK_EXPORTED_FUNCTIONS TK Exported Functions
  @{
*/


/**
 * @brief Enable touch key function
 * @param None
 * @return None
 * @note This function will enable touch key function and initial idle and polarity state as GND first for all scan keys
 * \hideinitializer
 */

void TK_Open(void)
{
    TK->SCANC |= TK_SCANC_TK_EN_Msk;

    /* Set idle and polarity state as GND */
    TK->IDLSC = 0UL;
    TK->IDLSC1 = 0UL;
    TK->POLSEL = 0UL;
    if( ((SYS->PDID & 0x01925000UL) == 0x01925000UL) || ((SYS->PDID & 0x01D23140UL) == 0x01D23140UL) || ((SYS->PDID & 0x01F31000UL) == 0x01F31000UL) )
    {
        //for M258G || TC8260 || M2L31
        TK->IDLSC1 = 0UL;
        TK->POLSEL1 = 0UL;
    }
    TK->POLC &= ~(TK_POLC_IDLS16_Msk | TK_POLC_POL16_Msk);
}

/**
 * @brief Disable touch key function
 * @param None
 * @return None
 * \hideinitializer
 */
void TK_Close(void)
{
    TK->SCANC &= ~TK_SCANC_TK_EN_Msk;
}

/**
 * @brief Set touch key scan mode
 * @param[in] u32Mode Single ,periodic or all key scan mode
 *              - \ref TK_SCAN_MODE_SINGLE
 *              - \ref TK_SCAN_MODE_PERIODIC
 *              - \ref TK_SCAN_MODE_ALL_KEY
 *              - \ref TK_SCAN_MODE_PERIODIC_ALL_KEY
 * @return None
 * @details This function is used to set touch key scan mode.
 * @note If touch key controller sets as periodic mode, touch key will be trigger scan by Timer0. So Timer0 must be enabled and operated in periodic mode.
 *       If touch key controller sets as single scan mode, touch key can be trigger scan by calling TK_START_SCAN().
 * \hideinitializer
 */
void TK_SetScanMode(uint32_t u32Mode)
{
    TK->SCANC &= ~TK_SCANC_TRG_EN_Msk;
    TK->REFC &= ~TK_REFC_SCAN_ALL_Msk;

    if (u32Mode == TK_SCAN_MODE_PERIODIC)
    {
        /* Periodic trigger source is configured by TK_TriggerMode(). */
    }
    else if (u32Mode == TK_SCAN_MODE_ALL_KEY)
    {
        TK->REFC |= u32Mode;
    }
    else if (u32Mode == TK_SCAN_MODE_PERIODIC_ALL_KEY)
    {
        /* Periodic trigger source is configured by TK_TriggerMode(). */
        TK->REFC |= TK_REFC_SCAN_ALL_Msk;
    }
    else
    {
        /* Single scan mode requires no additional configuration. */
    }
}

/**
 * @brief Configure touch key trigger source
 * @param[in] u32Src Trigger TK Source
 *              - \ref TK_SCAN_TRIGGER_SOURCE_TMR0
 *              - \ref TK_SCAN_TRIGGER_SOURCE_TMR1
 *              - \ref TK_SCAN_TRIGGER_SOURCE_TMR2
 *              - \ref TK_SCAN_TRIGGER_SOURCE_TMR3
 *              - \ref TK_SCAN_TRIGGER_SOURCE_LPTMR0
 *              - \ref TK_SCAN_TRIGGER_SOURCE_LPTMR1
 *              - \ref TK_SCAN_TRIGGER_SOURCE_TICKTMR0
 *              - \ref TK_SCAN_TRIGGER_SOURCE_TICKTMR2
 * @return None
 */
void TK_TriggerMode(uint32_t u32Src)
{
    TK->SCANC = (TK->SCANC & ~TK_SCANC_TRG_EN_Msk) | u32Src;
}

/**
 * @brief Configure touch key scan sensitivity
 * @param[in] u32PulseWidth Sensing pulse width
 *              - \ref TK_SENSE_PULSE_1
 *              - \ref TK_SENSE_PULSE_2
 *              - \ref TK_SENSE_PULSE_4
 *              - \ref TK_SENSE_PULSE_8
 *              - \ref TK_SENSE_PULSE_250NS
 *              - \ref TK_SENSE_PULSE_500NS
 * @param[in] u32SenseCnt Sensing count
 *              - \ref TK_SENSE_CNT_128
 *              - \ref TK_SENSE_CNT_255
 *              - \ref TK_SENSE_CNT_511
 *              - \ref TK_SENSE_CNT_1023
 *              - \ref TK_SENSE_CNT_8
 *              - \ref TK_SENSE_CNT_16
 *              - \ref TK_SENSE_CNT_32
 *              - \ref TK_SENSE_CNT_64
 * @param[in] u32AVCCHSel voltage selection
 *              - \ref TK_AVCCH_1_DIV_16
 *              - \ref TK_AVCCH_1_DIV_8
 *              - \ref TK_AVCCH_3_DIV_16
 *              - \ref TK_AVCCH_1_DIV_4
 *              - \ref TK_AVCCH_5_DIV_16
 *              - \ref TK_AVCCH_3_DIV_8
 *              - \ref TK_AVCCH_7_DIV_16
 *              - \ref TK_AVCCH_1_DIV_2
 * @return None
 * @details This function is used to configure touch key scan sensitivity.
 * \hideinitializer
 */
void TK_ConfigSensitivity(uint32_t u32PulseWidth, uint32_t u32SenseCnt, uint32_t u32AVCCHSel)
{
    TK->REFC = (TK->REFC & ~(TK_REFC_SENSET_Msk | TK_REFC_PULSET_Msk)) | (u32PulseWidth | u32SenseCnt);
    TK_SET_AVCCH(u32AVCCHSel);
}

/**
 * @brief Set touch key capacitor bank polarity
 * @param[in] u32CapBankPolSel capacitor bank polarity selection
 *              - \ref TK_CAP_BANK_POL_SEL_GND
 *              - \ref TK_CAP_BANK_POL_SEL_AVCCH
 *              - \ref TK_CAP_BANK_POL_SEL_VDD
 * @return None
 * @details This function is used to set touch key capacitor bank polarity.
 * \hideinitializer
 */
void TK_SetCapBankPol(uint32_t u32CapBankPolSel)
{
    TK->POLC = (TK->POLC & ~TK_POLC_POL_CAP_Msk) | (u32CapBankPolSel << TK_POLC_POL_CAP_Pos);
}

/**
 * @brief Configure touch key polarity
 * @param[in] u32Mask Combination of touch keys which need to be configured
 * @param[in] u32PolSel touch key polarity selection
 *              - \ref TK_TKn_POL_SEL_GND
 *              - \ref TK_TKn_POL_SEL_AVCCH
 *              - \ref TK_TKn_POL_SEL_VDD
 * @return None
 * @details This function is used to configure touch key polarity.
 * \hideinitializer
 */
void TK_SetTkPol(uint32_t u32Mask, uint32_t u32PolSel)
{
    uint32_t i;

    /* TK0 ~ TK15 Polarity Sel */
    for (i = 0 ; i < 16UL ; i++)
    {
        if (((1UL << i) & u32Mask) != 0UL)
        {
            TK->POLSEL = (TK->POLSEL & ~(TK_POLSEL_POL0_Msk << (i * 2UL))) | (u32PolSel << (i * 2UL));
        }
    }

    /* TK16's Polarity Sel is special */
    i = 16UL;
    if (((1UL << i) & u32Mask) != 0UL)
    {
        TK->POLC = (TK->POLC & ~(TK_POLC_POL16_Msk << 2U)) | (u32PolSel << 2U);
    }

    /* TK17 ~ TK26 Polarity Sel */
    for (i = 17UL ; i < 26UL ; i++)
    {
        if (((1UL << i) & u32Mask) != 0UL)
        {
            TK->POLSEL1 = (TK->POLSEL1 & ~(TK_POLSEL_POL0_Msk << ((i - 17UL) * 2UL))) | (u32PolSel << ((i - 17UL) * 2UL));
        }
    }
}

/**
 * @brief Enable the polarity of specified touch key(s)
 * @param[in] u32Mask Combination of enabled scan keys. Each bit corresponds to a touch key
 *                           Bit 0 represents touch key 0, bit 1 represents touch key 1...
 * @return None
 * @details This function is used to enable the polarity of specified touch key(s).
 * \hideinitializer
 */
void TK_EnableTkPolarity(uint32_t u32Mask)
{
    TK->POLC |= ((u32Mask & 0x1FFFFUL) << TK_POLC_POLEN0_Pos);
    if( ((SYS->PDID & 0x01925000UL) == 0x01925000UL) || ((SYS->PDID & 0x01D23140UL) == 0x01D23140UL) || ((SYS->PDID & 0x01F31000UL) == 0x01F31000UL) )
    {
        //for M258G || TC8260 || M2L31
        TK->POLC1 |= (u32Mask >> 17U);
    }
}

/**
 * @brief Disable the polarity of specified touch key(s)
 * @param[in] u32Mask Combination of enabled scan keys. Each bit corresponds to a touch key
 *                           Bit 0 represents touch key 0, bit 1 represents touch key 1...
 * @return None
 * @details This function is used to disable the polarity of specified touch key(s).
 * \hideinitializer
 */
void TK_DisableTkPolarity(uint32_t u32Mask)
{
    TK->POLC &= ~((u32Mask & 0x1FFFFUL) << TK_POLC_POLEN0_Pos);
    if( ((SYS->PDID & 0x01925000UL) == 0x01925000UL) || ((SYS->PDID & 0x01D23140UL) == 0x01D23140UL) || ((SYS->PDID & 0x01F31000UL) == 0x01F31000UL) )
    {
        //for M258G || TC8260 || M2L31
        TK->POLC1 &= ~(u32Mask >> 17U);
    }
}

/**
 * @brief Set complement capacitor bank data of specified touch key
 * @param[in] u32TKNum Touch key number. The valid value is 0~16.
 * @param[in] u32CapData Complement capacitor bank data. The valid value is 0~0xFF.
 * @return None
 * @details This function is used to set complement capacitor bank data of specified touch key.
 * \hideinitializer
 */
void TK_SetCompCapBankData(uint32_t u32TKNum, uint32_t u32CapData)
{
    volatile uint32_t *pu32Reg;
    uint32_t u32Shift;
    uint32_t u32MaskField;
    uint32_t u32CapField;

    if (u32TKNum > 17UL)
    {
        return;
    }

    pu32Reg = TK_GetCCBDReg(u32TKNum);

    if (u32TKNum < 17UL)
    {
        u32Shift = (u32TKNum % 4UL) * 8UL;
    }
    else
    {
        u32Shift = 0UL;
    }

    u32MaskField = (uint32_t)TK_CCBD0_CCBD0_Msk << u32Shift;
    u32CapField = (((u32CapData & 0xFFUL) << u32Shift) & u32MaskField);

    *pu32Reg = (*pu32Reg & (uint32_t)(~u32MaskField)) | u32CapField;
}

/**
 * @brief Set complement capacitor bank data of reference touch key
 * @param[in] u32CapData Complement capacitor bank data. The valid value is 0~0xFF.
 * @return None
 * @details This function is used to set complement capacitor bank data of reference touch key.
 * \hideinitializer
 */
void TK_SetRefKeyCapBankData(uint32_t u32CapData)
{
    uint32_t u32CapField;

    u32CapField =
        (((u32CapData & 0xFFUL) << (uint32_t)TK_CCBD4_CCBD_ALL_Pos)
         & (uint32_t)TK_CCBD4_CCBD_ALL_Msk);

    TK->CCBD4 = (TK->CCBD4 & (uint32_t)(~TK_CCBD4_CCBD_ALL_Msk))
                | u32CapField;
}

/**
  * @brief      Set reference capacitor bank data of specified touch key
  * @param[in]  u32TKNum: Touch key number. The valid value is 0~25.
  * @param[in]  u32CapData: Complement capacitor bank data. The valid value is 0~0xFF.
  * @return     None
  * @details    This function is used to set complement capacitor bank data of reference touch key.
  */

void TK_SetRefCapBankData(uint32_t u32TKNum, uint32_t u32CapData)
{
    volatile uint32_t *pu32Reg;
    uint32_t u32Shift;
    uint32_t u32MaskField;
    uint32_t u32CapField;

    if (u32TKNum > 17UL)
    {
        return;
    }

    pu32Reg = TK_GetREFCBDReg(u32TKNum);

    if (u32TKNum < 17UL)
    {
        u32Shift = (u32TKNum % 4UL) * 8UL;
    }
    else
    {
        u32Shift = 0UL;
    }

    u32MaskField = (uint32_t)TK_REFCBD0_CBD0_Msk << u32Shift;
    u32CapField = (((u32CapData & 0xFFUL) << u32Shift) & u32MaskField);

    *pu32Reg = (*pu32Reg & (uint32_t)(~u32MaskField)) | u32CapField;
}

/**
 * @brief Set high and low threshold of specified touch key for threshold control interrupt
 * @param[in] u32TKNum Touch key number. The valid value is 0~16.
 * @param[in] u32HighLevel High level for touch key threshold control. The valid value is 0~0xFF.
 * @return None
 * @details This function is used to set high and low threshold of specified touch key for threshold control interrupt.
 * \hideinitializer
 */
void TK_SetScanThreshold(uint32_t u32TKNum, uint32_t u32HighLevel)
{
    volatile uint32_t *pu32Reg;
    uint32_t u32Shift;
    uint32_t u32MaskField;
    uint32_t u32ThresholdField;

    if (u32TKNum > 17UL)
    {
        return;
    }

    pu32Reg = TK_GetTHCReg(u32TKNum);

    if (u32TKNum < 17UL)
    {
        u32Shift = (uint32_t)TK_THC01_HTH0_Pos
                   + ((u32TKNum & 0x1UL) * 16UL);
    }
    else
    {
        u32Shift = (uint32_t)TK_THC01_HTH0_Pos;
    }

    u32MaskField = (uint32_t)TK_THC01_HTH0_Msk << u32Shift;
    u32ThresholdField =
        (((u32HighLevel & 0xFFUL) << u32Shift) & u32MaskField);

    *pu32Reg = (*pu32Reg & (uint32_t)(~u32MaskField))
               | u32ThresholdField;
}

/**
 * @brief Enable touch key scan interrupt
 * @param[in] u32Msk Interrupt type selection.
 *              - \ref TK_INT_EN_SCAN_COMPLETE
 *              - \ref TK_INT_EN_SCAN_COMPLETE_LEVEL_TH
 * @return None
 * @details This function is used to enable touch key scan interrupt.
 * @note It need disable the enabled interrupt type first by TK_DisableInt() before to change enabled interrupt type.
 * \hideinitializer
 */
void TK_EnableInt(uint32_t u32Msk)
{
    TK->INTEN |= u32Msk;
}

/**
 * @brief Disable touch key scan interrupt
 * @param[in] u32Msk Interrupt type selection.
 *              - \ref TK_INT_EN_SCAN_COMPLETE
 *              - \ref TK_INT_EN_SCAN_COMPLETE_LEVEL_TH
 * @return None
 * @details This function is used to disable touch key scan interrupt.
* @note It need disable the enabled interrupt type first by TK_DisableInt() before to change enabled interrupt type.
 * \hideinitializer
 */
void TK_DisableInt(uint32_t u32Msk)
{
    TK->INTEN &= ~u32Msk;
}

/**
  * @brief      To disable all channels
  * @param[in]  None
  * @return     None
  * @details    This function is used to disable all channels for key scan.
  */
void TK_DisableAllChannel(void)
{
    TK->SCANC &= ~(0x1FFFFUL);
    if( ((SYS->PDID & 0x01925000UL) == 0x01925000UL) || ((SYS->PDID & 0x01D23140UL) == 0x01D23140UL) || ((SYS->PDID & 0x01F31000UL) == 0x01F31000UL) )
    {
        //for M258G || TC8260 || M2L31
        TK->SCANC1 &= ~(0x1FUL);
    }
}


/**
  * @brief      To clear all interrupts that were caused if the conversion TK data over the high threshold.
  * @param[in]  None
  * @return     None
  * @details    This function is used to clear all interrupts that were caused if the conversion TK data over the high threshold.
  */
void TK_ClearTKIF(void)
{
    TK->STA |= 0x1FFFFC3UL;
    if( ((SYS->PDID & 0x01925000UL) == 0x01925000UL) || ((SYS->PDID & 0x01D23140UL) == 0x01D23140UL) || ((SYS->PDID & 0x01F31000UL) == 0x01F31000UL) )
    {
        //for M258G || TC8260 || M2L31
        TK->STA1 |= 0x1FUL;
    }
}

/**
  * @brief      To enable scan all function to wake up system by any touch keys as low power mode
  * @param[in]  u8RefcbAll: The value co-works with u8CcbAll to make conversion all enabled touch keys' data - TKDATALL close to 0.
  * @param[in]  u8CcbAll: The value co-works with u8RefcbAll to make conversion all enabled touch keys' data - TKDATALL close to 0.
  * @param[in]  u8HThAll: Threshold to wake up system by any touch keys as low power mode
  * @return     None
  * @details    The u8RefcbAll and u8CcbAll are the calibration values was generate by calibration flow
  *             The flow makes the TKDATALL close to 0 after scan all enabled all touch keys.
  */
void TK_EnableScanAll(uint8_t u8RefcbAll, uint8_t u8CcbAll, uint8_t u8HThAll)
{
    TK->REFC |= TK_REFC_SCAN_ALL_Msk;
    TK->REFCBD4 = (TK->REFCBD4 & (~TK_REFCBD4_CBD_ALL_Msk)) | ((uint32_t)u8RefcbAll << TK_REFCBD4_CBD_ALL_Pos);
    TK->CCBD4 = (TK->CCBD4 & (~TK_CCBD4_CCBD_ALL_Msk)) | ((uint32_t)u8CcbAll << TK_CCBD4_CCBD_ALL_Pos);
    TK->THC16 = (TK->THC16 & (~TK_THC16_HTH_ALL_Msk))  | ((uint32_t)u8HThAll << TK_THC16_HTH_ALL_Pos);
}

/**
  * @brief      To disable scan all function to wake up system by any touch keys as low power mode
  * @param[in]  None
  * @return     None
  */
void TK_DisableScanAll(void)
{
    TK->REFC &= ~TK_REFC_SCAN_ALL_Msk;
}

/**
  * @brief      To assign a TK channel as reference channel
  * @param[in]  u32TKChanBitMsk: Channel mask
  * @return     None
  * @details    The reference channel's voltage will compare with TK channel's as operation
  *
  */
void TK_SetReferenceChannel(uint32_t u32TKChanBitMsk)  /* Single bit can be set */
{
    if(u32TKChanBitMsk <= 0x10000UL)
    {
        TK->REFC = (TK->REFC & 0xFFFE0000UL) | u32TKChanBitMsk;
        TK->REFC1 = 0UL;
    }
    else
    {
        TK->REFC = 0UL;
        TK->REFC1 = (u32TKChanBitMsk >> 17U);
    }
}

/**
  * @brief      To assign enabled TK channels mask
  * @param[in]  u32TKChanBitMsk: Enabled channels mask
  * @return     None
  * @details
  *
  */
void TK_EnableChannel(uint32_t u32TKChanBitMsk)     /* Multiple bits can be set */
{
    TK->SCANC =  (TK->SCANC & ~0x1FFFFUL) | (u32TKChanBitMsk & 0x1FFFFUL);
    TK->SCANC1 = u32TKChanBitMsk >> 17U;
}

/**
  * @brief      To assign disabled TK channels mask
  * @param[in]  u32TKChanBitMsk: Disable channels mask
  * @return     None
  * @details
  *
  */
void TK_DisableChannel(uint32_t u32TKChanBitMsk)    /* Multiple bits can be set */
{
    if ((u32TKChanBitMsk & 0x1FFFFUL) != 0UL)
    {
        TK->SCANC &= ~(u32TKChanBitMsk & 0x1FFFFUL);
    }
    if ((u32TKChanBitMsk >> 17U) != 0UL)
    {
        TK->SCANC1 &= ~(u32TKChanBitMsk >> 17U);
    }
}

/**
  * @brief      Clear the specifiy TK channel mask interrupt flags
  * @param[in]  u32TKChanBitMsk: Clear TK channels mask
  * @return     None
  * @details
  *
  */
void TK_ClearTKIFBitMask(uint32_t u32TKChanBitMsk)          /* Multiple bits can be set */
{
    if ((u32TKChanBitMsk & 0x1FFFFUL) != 0UL)
    {
        TK->STA = (u32TKChanBitMsk & 0x1FFFFUL) << 8U; /* Write one clear */
    }
    if ((u32TKChanBitMsk >> 17U) != 0UL)
    {
        TK->STA1 = u32TKChanBitMsk >> 17U;         /* Write one clear */
    }
}

/**
  * @brief      Checking if the specifiy TK channel number interrupt flag occurs
  * @param[in]  u8TKNum: Disable channels number. Value from 0 ~ max TK number
  * @return     None
  * @details
  *
  */
uint32_t TK_CheckTKIF(uint8_t u8TKNum)
{
    uint32_t u32Ret = 0UL;
    uint32_t u32TkNum = (uint32_t)u8TKNum;

    if (u32TkNum <= 16UL)
    {
        if ((TK->STA & (1UL << (u32TkNum + 8UL))) != 0UL)
        {
            u32Ret = 1UL;
        }
    }
    else if (u32TkNum <= 25UL)
    {
        if ((TK->STA1 & (1UL << (u32TkNum - 17UL))) != 0UL)
        {
            u32Ret = 1UL;
        }
    }
    else
    {
        /* Invalid touch-key number. */
    }

    return u32Ret;
}

/*@}*/ /* end of group TK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group TK_Driver */

/*@}*/ /* end of group Standard_Driver */
