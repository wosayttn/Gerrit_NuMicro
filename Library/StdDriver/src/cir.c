/**************************************************************************//**
 * @file     cir.c
 * @version  V1.00
 * @brief    M471 series CIR driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CIR_Driver CIR Driver
  @{
*/


/** @addtogroup CIR_EXPORTED_FUNCTIONS CIR Exported Functions
  @{
*/

/**
  * @brief      Disable Error Bypass
  *
  * @param[in]  cir         The pointer of the specified CIR module. 
  *
  * @return     None
  *
  * @details    This receiving bits will be dropped as error occurrence.
  *
  * \hideinitializer
  */
void CIR_DisableErrorBypass(CIR_T *cir)  
{	
    cir->CTL &= ~CIR_CTL_ERRBYP_Msk;    
}


/**
  * @brief      Enable Error Bypass
  *
  * @param[in]  cir         The pointer of the specified CIR module. 
  *
  * @return     None
  *
  * @details    This receiving bits will keep into DATAx buffer as error occurrence.
  *
  * \hideinitializer
  */
void CIR_EnableErrorBypass(CIR_T *cir)  
{	
    cir->CTL |= CIR_CTL_ERRBYP_Msk;    
}


/**
  * @brief      Get the enabled interrupt mask
  *
  * @param[in]  cir         The pointer of the specified CIR module. 
  *
  * @return     The enabled interrupt mask
  *
  * \hideinitializer
  */
uint32_t CIR_GetEnabledIntMask(const CIR_T *cir)  
{	
    return (cir->INTCTL);    
}


/**
  * @brief      Get current latched timer value
  *
  * @param[in]  cir         The pointer of the specified CIR module. 
  *
  * @return     The latched counter value between two consecutive rising edges or two consecutive falling edges
  *
  * \hideinitializer
  */
uint32_t CIR_GetLatchedTimerValue(const CIR_T *cir)  
{
    return (cir->LTVR & CIR_LTVR_LTV_Msk);
}


/**
  * @brief      Get data in CIR buffer
  *
  * @param[in]  cir         The pointer of the specified CIR module. 
  *
  * @return     None
  *
  * \hideinitializer
  */
void CIR_GetData(const CIR_T *cir, uint32_t *u32Data0, uint32_t *u32Data1)  
{	
    *u32Data0 = cir->DATA0;
    *u32Data1 = cir->DATA1;
}




/**
  *   @brief Disable CIR function.
  *
  *   @param[in]    cir           The pointer of the specified CIR module.
  *
  *   @return       None
  */
void CIR_Close(CIR_T *cir)
{
    cir->CTL &= ~CIR_CTL_CNTEN_Msk;
}


/**
  * @brief      Enable CIR function
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @return     None
  */
void CIR_Open(CIR_T *cir)
{
    cir->CTL |= CIR_CTL_CNTEN_Msk;
}


/* CIR CLK Source */
#define CIR_SRC_HXT                     (0UL)
#define CIR_SRC_LXT                     (1UL)
#define CIR_SRC_TM0                     (2UL)
#define CIR_SRC_LIRC                    (3UL)
#define CIR_SRC_HIRC                    (4UL)
/**
  * @brief Set CIR prescaler
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u32Prescaler      The prescaler for CIR
  *                               - \ref CIR_PRESCALER_1           : Clock divider is equal to 1. CIR sampling clock is equal to source clock/1.
  *                               - \ref CIR_PRESCALER_2           : Clock divider is equal to 2. CIR sampling clock is equal to source clock/2.
  *                               - \ref CIR_PRESCALER_4           : Clock divider is equal to 4. CIR sampling clock is equal to source clock/4.
  *                               - \ref CIR_PRESCALER_8           : Clock divider is equal to 8. CIR sampling clock is equal to source clock/8.
  *                               - \ref CIR_PRESCALER_16          : Clock divider is equal to 16. CIR sampling clock is equal to source clock/16.
  *                               - \ref CIR_PRESCALER_32          : Clock divider is equal to 32. CIR sampling clock is equal to source clock/32.
  *                               - \ref CIR_PRESCALER_64          : Clock divider is equal to 64. CIR sampling clock is equal to source clock/64.
  *                               - \ref CIR_PRESCALER_128         : Clock divider is equal to 128. CIR sampling clock is equal to source clock/128.
  *
  * @return     The frequency of CIR sampling clock.
  *
  * @details     The function is used to set CIR sampling clock.
  */
int32_t CIR_SetClockPrescaler(CIR_T *cir, uint32_t u32Prescaler)
{
    uint32_t u32SampleClk;
    uint32_t u32Src;
    uint32_t u32PrescalerValue;
    uint32_t u32Divider;

    const uint32_t au32Clk[] = {__HXT, __LXT, 0UL, 0UL, 0UL, __LIRC, 0UL, __HIRC};

    cir->CTL = (cir->CTL & ~CIR_CTL_PSCALER_Msk) | u32Prescaler;

    u32Src = (CLK->CLKSEL2 & CLK_CLKSEL2_CIR0SEL_Msk) >> CLK_CLKSEL2_CIR0SEL_Pos;
    u32PrescalerValue = u32Prescaler >> CIR_CTL_PSCALER_Pos;
    u32Divider = 1UL << u32PrescalerValue;

    switch(u32Src)
    {
        case CIR_SRC_HIRC:
        {
            u32SampleClk = __HIRC / u32Divider;
            break;
        }

        case CIR_SRC_LIRC:
        {
            u32SampleClk = __LIRC / u32Divider;
            break;
        }

        case CIR_SRC_TM0:
        {
            uint32_t u32Clk;
            uint32_t u32TmrPsc;
            uint32_t u32TmrCmp;

            u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR0SEL_Msk) >> CLK_CLKSEL1_TMR0SEL_Pos;

            if(u32Src == 2UL)
            {
                u32Clk = CLK_GetPCLK0Freq();
            }
            else
            {
                u32Clk = au32Clk[u32Src];
            }

            u32TmrPsc = TIMER0->CTL & TIMER_CTL_PSC_Msk;
            u32TmrCmp = TIMER0->CMP & TIMER_CMP_CMPDAT_Msk;

            if(u32TmrCmp == 0UL)
            {
                return -1;
            }

            u32SampleClk = u32Clk / (u32TmrCmp * (u32TmrPsc + 1UL));
            u32SampleClk /= u32Divider;
            break;
        }

        case CIR_SRC_LXT:
        {
            u32SampleClk = __LXT / u32Divider;
            break;
        }

        case CIR_SRC_HXT:
        {
            u32SampleClk = __HXT / u32Divider;
            break;
        }

        default:
        {
            return -1;
        }
    }

    return (int32_t)u32SampleClk;
}

/**
  * @brief Set CIR Debounce Sampling Selection
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u32DebounceSmplSel Debounce Sampling Selection
  *                                - \ref CIR_DEBOUNCE_DISABLE      : Debounce filter disable
  *                                - \ref CIR_DEBOUNCE_TWO_SMPL     : Debounce filter enable with two samples matched
  *                                - \ref CIR_DEBOUNCE_THREE_SMPL   : Debounce filter enable with three samples matched
  *                                - \ref CIR_DEBOUNCE_FOUR_SMPL    : Debounce filter enable with four samples matched
  *
  * @return     None
  */
void CIR_SetDebounce(CIR_T *cir, uint32_t u32DebounceSmplSel)
{
    cir->CTL = (cir->CTL & ~CIR_CTL_DBSEL_Msk) | u32DebounceSmplSel;
}

/**
  * @brief Set CIR input pattern type and polarity
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u32InputType      Input pattern type
  *                               - \ref CIR_POSITIVE_EDGE         : Standardized Positive Edge Mode
  *                               - \ref CIR_NEGATIVE_EDGE         : Standardized Negative Edge Mode
  *                               - \ref CIR_FLEXIBLE_POSITIVE_EDGE : Flexible Positive Edge Mode
  *
  * @param[in]  u32Inverse        Polarity
  *                               - \ref CIR_NORMAL                : Bypass signal
  *                               - \ref CIR_INVERSE               : Inverse signal
  *
  * @return None
  */
void CIR_SetInputType(CIR_T *cir, uint32_t u32InputType, uint32_t u32Inverse)
{
    cir->CTL = (cir->CTL & ~(CIR_CTL_PATTYP_Msk | CIR_CTL_POLINV_Msk)) |
               (u32InputType | u32Inverse);
}


/**
  * @brief      This function disables the interrupts of CIR module
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u32Mask           The combination of all related interrupt enable bits.
  *                               Each bit corresponds to an interrupt bit.
  *                               This parameter decides which interrupts will be disabled. It could be combination of following elements
  *                               - \ref CIR_INTCTL_SPMIEN_Msk    : Special Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_D1PMIEN_Msk   : Data1 Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_D0PMIEN_Msk   : Data0 Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_HPMIEN_Msk    : Header Pattern Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_RBUFIEN_Msk   : Receive Buffer Full Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_DRECIEN_Msk   : Data Receive Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_PERRIEN_Msk   : Pattern Error Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_CMPMIEN_Msk   : Compare Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_EPMIEN_Msk    : End Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_RBMIEN_Msk    : Receive Bit Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_PDWKIEN_Msk   : Power Down Wake Up Interrupt Enable Mask
  *
  * @return       None
  *
  * @details      The function is used to disable CIR interrupt events.
  */
void CIR_DisableInt(CIR_T *cir, uint32_t u32Mask)
{
    cir->INTCTL &= ~u32Mask;
}

/**
  * @brief      This function enables the interrupts of CIR module.
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u32Mask           The combination of all related interrupt enable bits.
  *                               Each bit corresponds to a interrupt bit.
  *                               This parameter decides which interrupts will be enabled. It could be combination of following elements
  *                               - \ref CIR_INTCTL_SPMIEN_Msk    : Special Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_D1PMIEN_Msk   : Data1 Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_D0PMIEN_Msk   : Data0 Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_HPMIEN_Msk    : Header Pattern Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_RBUFIEN_Msk   : Receive Buffer Full Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_DRECIEN_Msk   : Data Receive Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_PERRIEN_Msk   : Pattern Error Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_CMPMIEN_Msk   : Compare Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_EPMIEN_Msk    : End Pattern Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_RBMIEN_Msk    : Receive Bit Match Interrupt Enable Mask
  *                               - \ref CIR_INTCTL_PDWKIEN_Msk   : Power Down Wake Up Interrupt Enable Mask
  *    @return  None
  *
  *    @details The function is used to enable CIR interrupt events.
  */
void CIR_EnableInt(CIR_T *cir, uint32_t u32Mask)
{
    cir->INTCTL |= u32Mask;
}


/**
  * @brief      This function get the CIR interrupt flag
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @return     Interrupt flags.
  *
  * @details    Clear CIR interrupt flag when interrupt flag is set. These flags are written "1" to clear status
  */
uint32_t CIR_GetIntFlag(const CIR_T *cir)
{
    return cir->STATUS;
}


/**
  * @brief      This function clears the CIR interrupt flag
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u32Mask           The combination of all related interrupt status bits.
  *                               Each bit corresponds to a interrupt status bit.
  *                               This parameter decides which interrupts will be cleared. It could be combination of following element.
  *                               - \ref CIR_STATUS_SPMF_Msk    : Special Pattern Match Interrupt Status
  *                               - \ref CIR_STATUS_D1PMF_Msk   : Data1 Pattern Match Interrupt Status
  *                               - \ref CIR_STATUS_D0PMF_Msk   : Data0 Pattern Match Interrupt Status
  *                               - \ref CIR_STATUS_HPMF_Msk    : Header Pattern Pattern Match Interrupt Status
  *                               - \ref CIR_STATUS_RBUFF_Msk   : Receive Buffer Full Interrupt Status
  *                               - \ref CIR_STATUS_DRECF_Msk   : Data Receive Interrupt Status
  *                               - \ref CIR_STATUS_RERRF_Msk   : Pattern Error Interrupt Status
  *                               - \ref CIR_STATUS_COMPMF_Msk  : Compare Match Interrupt Status
  *                               - \ref CIR_STATUS_EPMF_Msk    : End Pattern Match Interrupt Status
  *                               - \ref CIR_STATUS_RBMF_Msk    : Receive Bit Match Interrupt Status
  *                               - \ref CIR_STATUS_PDWKF_Msk   : Power Down Wake Up Interrupt Status
  * @return     None
  *
  * @details    Clear CIR interrupt flag when interrupt flag is set. These flags are written "1" clear
  */
void CIR_ClearIntFlag(CIR_T *cir, uint32_t u32Mask)
{
    cir->STATUS = u32Mask;
}


/**
  * @brief      This function is use to set the CIR upper and lower bounds
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u32Pattern        Pattern type.
  *                               - \ref CIR_HEADER_PAT    :   Header Pattern
  *                               - \ref CIR_DATA1_PAT     :   Data1 Pattern
  *                               - \ref CIR_DATA0_PAT     :   Data0 Pattern
  *                               - \ref CIR_SPECIAL_PAT   :   Special Pattern
  *                               - \ref CIR_END_PAT       :   End Pattern
  * @param[in]  u32HBound         Upper bound. The field is 11 bits. It is invalid if CIR_END_PAT pattern.
  *
  * @param[in]  u32LBound         Low bound. The field is 11 bits.
  *
  * @return     None
  */
void CIR_SetPatternBoundary(CIR_T *cir, uint32_t u32Pattern, uint32_t u32HBound, uint32_t u32LBound)
{
    if(u32Pattern == CIR_HEADER_PAT)
    {
        cir->HDBOUND = (u32HBound << CIR_HDBOUND_HBOUND_Pos) | u32LBound;
    }
    else if(u32Pattern == CIR_DATA0_PAT)
    {
        cir->D0BOUND = (u32HBound << CIR_D0BOUND_HBOUND_Pos) | u32LBound;
    }
    else if(u32Pattern == CIR_DATA1_PAT)
    {
        cir->D1BOUND = (u32HBound << CIR_D1BOUND_HBOUND_Pos) | u32LBound;
    }
    else if(u32Pattern == CIR_SPECIAL_PAT)
    {
        cir->SPBOUND = (u32HBound << CIR_SPBOUND_HBOUND_Pos) | u32LBound;
    }
    else if(u32Pattern == CIR_END_PAT)
    {
        cir->ENDBOUND = u32LBound;
    }
    else
    {
        /* Invalid pattern: no register is updated. */
    }
}

/**
  * @brief      This function is use to get the CIR upper and lower bounds
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u32Pattern        Pattern type.
  *                               - \ref CIR_HEADER_PAT    :   Header Pattern
  *                               - \ref CIR_DATA1_PAT     :   Data1 Pattern
  *                               - \ref CIR_DATA0_PAT     :   Data0 Pattern
  *                               - \ref CIR_SPECIAL_PAT   :   Special Pattern
  *                               - \ref CIR_END_PAT       :   End Pattern
  * @param[in]  u32HBound         Upper bound. The field is 11 bits. It is invalid if CIR_END_PAT pattern.
  * @param[in]  u32LBound         Low bound. The field is 11 bits.
  *
  * @return     None
  */
void CIR_GetPatternBoundary(const CIR_T *cir, uint32_t u32Pattern, uint32_t *pu32HBound, uint32_t *pu32LBound)
{
    uint32_t u32Bound = 0UL;

    if(u32Pattern == CIR_HEADER_PAT)
    {
        u32Bound = cir->HDBOUND;
    }
    else if(u32Pattern == CIR_DATA0_PAT)
    {
        u32Bound = cir->D0BOUND;
    }
    else if(u32Pattern == CIR_DATA1_PAT)
    {
        u32Bound = cir->D1BOUND;
    }
    else if(u32Pattern == CIR_SPECIAL_PAT)
    {
        u32Bound = cir->SPBOUND;
    }
    else if(u32Pattern == CIR_END_PAT)
    {
        u32Bound = cir->ENDBOUND;
    }
    else
    {
        /* Invalid pattern: return zero bounds. */
    }

    *pu32HBound = u32Bound >> 16U;
    *pu32LBound = u32Bound & 0xFFFFUL;
}

/**
  * @brief      It is used to enable the receiving bits count matched function.
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u32RecvBitCount   Specified receciving bit count.
  *
  * @return     None
  *
  * @details    CIR will raise the CIR_RBMF_FLAG flag if receiving bits count meets the specified bits count.
  */
void CIR_EnableRecvBitCountMatch(CIR_T *cir, uint32_t u32RecvBitCount)
{
    cir->RDBC = (cir->RDBC & ~CIR_RDBC_RBITCMP_Msk)\
                | (CIR_RDBC_BCCMEN_Msk | (u32RecvBitCount << CIR_RDBC_RBITCMP_Pos));
}


/**
  * @brief      It is used to disable the receiving bits count matched function.
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @return     None
  *
  * @details    CIR didn't raise the CIR_RBMF_FLAG flag if receiving bits count meets the specific bits count.
  */
void CIR_DisableRecvBitCountMatch(CIR_T *cir)
{
    cir->RDBC &= ~CIR_RDBC_BCCMEN_Msk;
}

/**
  * @brief      This function is used to clear received bit counts
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @return     None
  *
  * @details    Clear CIR received bit counts
  */
void CIR_ClearReceivedBitLength(CIR_T *cir)
{
    cir->RDBC = (cir->RDBC & ~CIR_RDBC_RBITCNT_Msk) | 0x1UL;
}


/**
  * @brief      This function is used to enable data compare match function.
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @param[in]  u8ExpectedData    Expected data.
  *
  * @param[in]  u8CmpBitCount     Comparing data bit count. The value will be 0 ~ 7.
  *                               If the field is set to 3, the CIR->DATA0[3:0] will be compared with the u8ExpecedData[3:0].
  *
  * @return     None
  *
  * @details    Enable CIR to wake up system if IR's coming data meeting with the specified data.
  */
void CIR_EnableDataCmpWakeup(CIR_T *cir, uint8_t u8ExpectedData, uint8_t u8CmpBitCount)
{
    cir->CMPCTL = (cir->CMPCTL & ~(CIR_CMPCTL_CMPDAT_Msk | CIR_CMPCTL_CMPVALID_Msk))\
                  | ((CIR_CMPCTL_DCMPEN_Msk | (uint32_t)u8ExpectedData) | ((uint32_t)u8CmpBitCount << CIR_CMPCTL_CMPVALID_Pos));
}

/**
  * @brief      This function is used to disable data compare match function.
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @return     None
  *
  * @details    Disable CIR to wake up system if IR's coming data meeting with the specified data.
  */
void CIR_DisableDataCmpWakeup(CIR_T *cir)
{
    cir->CMPCTL &= ~CIR_CMPCTL_DCMPEN_Msk;
}

/**
  * @brief      This function is used to clear data filed and bit count for next receiving.
  *
  * @param[in]  cir               The pointer of the specified CIR module.
  *
  * @return     None
  *
  * @details    Bit count will be cleared automatically after disable CNTEN then enable CIREN
  */
void CIR_ClearDataFieldBitCount(CIR_T *cir)
{
    cir->CTL &= ~CIR_CTL_CNTEN_Msk;   /* !!!! To clear CIREN bit then set CNTEN, also clear receive bit count */
    cir->DATA0 = 0x1UL;
    cir->DATA1 = 0x1UL;
    cir->CTL |= CIR_CTL_CNTEN_Msk;
}
/*@}*/ /* end of group CIR_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group CIR_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


