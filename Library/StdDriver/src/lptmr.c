/**************************************************************************//**
 * @file     lptmr.c
 * @brief    LPTMR Controller (Low Power Timer) driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPTMR_Driver LPTMR Driver
  @{
*/

/** @addtogroup LPTMR_EXPORTED_FUNCTIONS LPTMR Exported Functions
  @{
*/

/**
  * @brief      Open LPTMR with Operate Mode and Frequency
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Mode     Operation mode. Possible options are
  *                         - \ref LPTMR_ONESHOT_MODE
  *                         - \ref LPTMR_PERIODIC_MODE
  *                         - \ref LPTMR_TOGGLE_MODE
  *                         - \ref LPTMR_CONTINUOUS_MODE
  * @param[in]  u32Freq     Target working frequency
  *
  * @return     Real lptmr working frequency
  *
  * @details    This API is used to configure lptmr to operate in specified mode and frequency.
  *             If lptmr cannot work in target frequency, a closest frequency will be chose and returned.
  * @note       After calling this API, LPTMR is \b NOT running yet. But could start lptmr running be calling
  *             \ref LPTMR_Start macro or program registers directly.
  */
uint32_t LPTMR_Open(LPTMR_T *lptmr, uint32_t u32Mode, uint32_t u32Freq)
{
    uint32_t u32Clk;
    uint32_t u32Cmpr;
    uint32_t u32Prescale;
    uint32_t u32Denominator;

    u32Cmpr = 0UL;
    u32Prescale = 0UL;

    if (lptmr == (LPTMR_T *)NULL)
    {
        return 0UL;
    }

    if (u32Freq == 0UL)
    {
        return 0UL;
    }

    u32Clk = LPTMR_GetModuleClock(lptmr);

    if (u32Clk == 0UL)
    {
        return 0UL;
    }

    /*
     * The fastest possible LPTMR frequency is u32Clk / 2.
     * The minimum valid compare value is 2.
     */
    if (u32Freq > (u32Clk / 2UL))
    {
        u32Cmpr = 2UL;
    }
    else
    {
        /*
         * Calculate the required total counter ticks.
         *
         * CMP is a 24-bit register. When the required count exceeds
         * 0x00FFFFFF, use the prescaler to reduce the CMP value.
         */
        u32Cmpr = u32Clk / u32Freq;

        u32Prescale = u32Cmpr >> 24U;

        if (u32Prescale > 0UL)
        {
            u32Cmpr /= (u32Prescale + 1UL);
        }
        else
        {
            /* No prescaling is required. */
        }

        /*
         * Defensive limitation for the 24-bit CMP register.
         * Under the current calculation, this condition normally
         * should not occur.
         */
        if (u32Cmpr > 0x00FFFFFFUL)
        {
            u32Cmpr = 0x00FFFFFFUL;
        }
        else
        {
            /* CMP value is within the 24-bit range. */
        }

        /*
         * Avoid programming an invalid CMP value.
         * Normally u32Cmpr cannot be less than 2 because the maximum
         * frequency case has already been handled above.
         */
        if (u32Cmpr < 2UL)
        {
            u32Cmpr = 2UL;
        }
        else
        {
            /* CMP value is valid. */
        }
    }

    lptmr->CTL = u32Mode | u32Prescale;
    lptmr->CMP = u32Cmpr;

    /*
     * Maximum denominator:
     *
     * 0x00FFFFFF × 256 = 0xFFFFFF00
     *
     * Therefore, the multiplication does not overflow uint32_t,
     * assuming the prescaler is an 8-bit direct value as used by
     * the original implementation.
     */
    u32Denominator = u32Cmpr * (u32Prescale + 1UL);

    if (u32Denominator == 0UL)
    {
        return 0UL;
    }

    return u32Clk / u32Denominator;
}

/**
  * @brief      Stop LPTMR Counting
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This API stops lptmr counting and disable all lptmr interrupt function.
  */
void LPTMR_Close(LPTMR_T *lptmr)
{
    lptmr->CTL = 0UL;
    lptmr->EXTCTL = 0UL;
}


/**
  * @brief      Create a specify Delay Time
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Usec     Delay period in micro seconds. Valid values are between 100~1000000 (100 micro second ~ 1 second).
  *
  * @retval     0                 Delay success, target delay time reached
  * @retval     LPTMR_TIMEOUT_ERR Delay function execute failed due to timer stop working
  *
  * @details    This API is used to create a delay loop for u32usec micro seconds by using lptmr one-shot mode.
  * @note       This API overwrites the register setting of the lptmr used to count the delay time.
  * @note       This API use polling mode. So there is no need to enable interrupt for the lptmr module used to generate delay.
  */
int32_t LPTMR_Delay(LPTMR_T *lptmr, uint32_t u32Usec)
{
    uint32_t u32Clk;
    uint32_t u32UsecReq;
    uint32_t u32Prescale;
    uint32_t u32Cmpr;
    uint32_t u32Delay;
    uint32_t u32DelayReload;
    uint32_t u32Cntr;
    uint64_t u64TotalCount;
    uint64_t u64Delay;

    if (lptmr == (LPTMR_T *)NULL)
    {
        return LPTMR_TIMEOUT_ERR;
    }

    u32Clk = LPTMR_GetModuleClock(lptmr);

    if (u32Clk == 0UL)
    {
        return LPTMR_TIMEOUT_ERR;
    }

    /*
     * Use a local copy because the requested delay may need
     * to be limited to the supported range.
     */
    u32UsecReq = u32Usec;

    /* Clear current LPTMR configuration. */
    lptmr->CTL = 0UL;
    lptmr->EXTCTL = 0UL;

    /*
     * The minimum supported delay depends on the LPTMR clock.
     */
    if (u32Clk <= 1000000UL)
    {
        if (u32UsecReq < 1000UL)
        {
            u32UsecReq = 1000UL;
        }
    }
    else
    {
        if (u32UsecReq < 100UL)
        {
            u32UsecReq = 100UL;
        }
    }

    if (u32UsecReq > 1000000UL)
    {
        u32UsecReq = 1000000UL;
    }

    /*
     * Calculate the number of LPTMR input clock cycles:
     *
     * total count = delay(us) * clock(Hz) / 1,000,000
     *
     * A 64-bit intermediate prevents multiplication overflow
     * and avoids precision loss caused by dividing u32Clk first.
     */
    u64TotalCount = ((uint64_t)u32UsecReq * (uint64_t)u32Clk)
                    / 1000000ULL;

    /*
     * CMP is 24 bits and PSC is 8 bits.
     *
     * PSC divides the input clock by (PSC + 1).
     * Use the bits above bit 23 to determine the prescaler.
     */
    u32Prescale = (uint32_t)(u64TotalCount >> 24U);

    /*
     * u64TotalCount cannot exceed UINT32_MAX because:
     *
     * u32UsecReq <= 1,000,000 us
     * u32Clk     <= UINT32_MAX Hz
     *
     * Therefore, u32Prescale cannot exceed 0xFF.
     */
    if (u32Prescale > 0xFFUL)
    {
        u32Prescale = 0xFFUL;
    }

    if (u32Prescale > 0UL)
    {
        u64TotalCount /= ((uint64_t)u32Prescale + 1ULL);
    }

    /*
     * CMP is a 24-bit register.
     */
    if (u64TotalCount > 0x00FFFFFFULL)
    {
        u32Cmpr = 0x00FFFFFFUL;
    }
    else
    {
        u32Cmpr = (uint32_t)u64TotalCount;
    }

    /*
     * Avoid programming CMP with zero.
     * Under the supported delay and clock conditions,
     * this should normally not occur.
     */
    if (u32Cmpr == 0UL)
    {
        u32Cmpr = 1UL;
    }

    lptmr->CMP = u32Cmpr;

    /*
     * PSC occupies CTL[7:0], so u32Prescale can be directly
     * combined with the control bits.
     */
    lptmr->CTL = LPTMR_CTL_CNTEN_Msk
                 | LPTMR_ONESHOT_MODE
                 | u32Prescale;

    /*
     * Wait for more than one LPTMR input clock period so that
     * ACTSTS has enough time to become active.
     */
    u32Delay = (SystemCoreClock / u32Clk) + 1UL;

    while (u32Delay > 0UL)
    {
        __NOP();
        u32Delay--;
    }

    /*
     * The LPTMR counter should change within:
     *
     * CPU clocks per LPTMR clock
     * multiplied by the prescaler division ratio.
     */
    u64Delay = ((uint64_t)SystemCoreClock / (uint64_t)u32Clk)
               * ((uint64_t)u32Prescale + 1ULL);

    if (u64Delay > 0xFFFFFFFFULL)
    {
        u32DelayReload = 0xFFFFFFFFUL;
    }
    else
    {
        u32DelayReload = (uint32_t)u64Delay;
    }

    /*
     * Ensure the timeout counter is not initialized to zero.
     */
    if (u32DelayReload == 0UL)
    {
        u32DelayReload = 1UL;
    }

    u32Delay = u32DelayReload;
    u32Cntr = lptmr->CNT;

    while ((lptmr->CTL & LPTMR_CTL_ACTSTS_Msk) != 0UL)
    {
        if (u32Cntr == lptmr->CNT)
        {
            /*
             * Countdown avoids overflow caused by an incrementing
             * timeout counter.
             */
            if (u32Delay == 0UL)
            {
                return LPTMR_TIMEOUT_ERR;
            }

            u32Delay--;
        }
        else
        {
            /*
             * The timer is still counting. Reload the timeout
             * counter and monitor the next counter transition.
             */
            u32Cntr = lptmr->CNT;
            u32Delay = u32DelayReload;
        }
    }

    return 0L;
}

/**
  * @brief      Enable LPTMR Capture Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32CapMode  LPTMR capture mode. Could be
  *                         - \ref LPTMR_CAPTURE_FREE_COUNTING_MODE
  *                         - \ref LPTMR_CAPTURE_COUNTER_RESET_MODE
  * @param[in]  u32Edge     LPTMR capture trigger edge. Possible values are
  *                         - \ref LPTMR_CAPTURE_EVENT_FALLING
  *                         - \ref LPTMR_CAPTURE_EVENT_RISING
  *                         - \ref LPTMR_CAPTURE_EVENT_FALLING_RISING
  *                         - \ref LPTMR_CAPTURE_EVENT_RISING_FALLING
  *                         - \ref LPTMR_CAPTURE_EVENT_GET_LOW_PERIOD
  *                         - \ref LPTMR_CAPTURE_EVENT_GET_HIGH_PERIOD
  *
  * @return     None
  *
  * @details    This API is used to enable lptmr capture function with specify capture trigger edge \n
  *             to get current counter value or reset counter value to 0.
  * @note       LPTMR frequency should be configured separately by using \ref LPTMR_Open API, or program registers directly.
  */
void LPTMR_EnableCapture(LPTMR_T *lptmr, uint32_t u32CapMode, uint32_t u32Edge)
{
    lptmr->EXTCTL = (lptmr->EXTCTL & ~(LPTMR_EXTCTL_CAPFUNCS_Msk | LPTMR_EXTCTL_CAPEDGE_Msk)) |
                    u32CapMode | u32Edge | LPTMR_EXTCTL_CAPEN_Msk;
}


/**
  * @brief      Disable LPTMR Capture Function
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This API is used to disable the lptmr capture function.
  */
void LPTMR_DisableCapture(LPTMR_T *lptmr)
{
    lptmr->EXTCTL &= ~LPTMR_EXTCTL_CAPEN_Msk;
}


/**
  * @brief      Enable LPTMR Counter Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Edge     Detection edge of counter pin. Could be ether
  *                         - \ref LPTMR_COUNTER_EVENT_FALLING
  *                         - \ref LPTMR_COUNTER_EVENT_RISING
  *
  * @return     None
  *
  * @details    This function is used to enable the lptmr counter function with specify detection edge.
  * @note       LPTMR compare value should be configured separately by using \ref LPTMR_SET_CMP_VALUE macro or program registers directly.
  * @note       While using event counter function, \ref LPTMR_TOGGLE_MODE cannot set as lptmr operation mode.
  */
void LPTMR_EnableEventCounter(LPTMR_T *lptmr, uint32_t u32Edge)
{
    lptmr->EXTCTL = (lptmr->EXTCTL & ~LPTMR_EXTCTL_CNTPHASE_Msk) | u32Edge;
    lptmr->CTL |= LPTMR_CTL_EXTCNTEN_Msk;
}


/**
  * @brief      Disable LPTMR Counter Function
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This API is used to disable the lptmr event counter function.
  */
void LPTMR_DisableEventCounter(LPTMR_T *lptmr)
{
    lptmr->CTL &= ~LPTMR_CTL_EXTCNTEN_Msk;
}


/**
  * @brief      Get LPTMR Clock Frequency
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     LPTMR clock frequency
  *
  * @details    This API is used to get the lptmr clock frequency.
  * @note       This API cannot return correct clock rate if lptmr source is from external clock input.
  */
uint32_t LPTMR_GetModuleClock(const LPTMR_T *lptmr)
{
    uint32_t u32Src;
    uint32_t u32Clk;
    const uint32_t au32Clk[] = {__HIRC, __MIRC, __LXT, __LIRC, 0UL, 0UL, 0UL, 0UL};

    if(lptmr == LPTMR0)
    {
        u32Src = (LPSCC->CLKSEL0 & LPSCC_CLKSEL0_LPTMR0SEL_Msk) >> LPSCC_CLKSEL0_LPTMR0SEL_Pos;
    }
    else if(lptmr == LPTMR1)
    {
        u32Src = (LPSCC->CLKSEL0 & LPSCC_CLKSEL0_LPTMR1SEL_Msk) >> LPSCC_CLKSEL0_LPTMR1SEL_Pos;
    }
    else
    {
        /* Unsupported LPTMR instance */
        return 0U;
    }

    u32Clk = au32Clk[u32Src];

    return u32Clk;
}


/**
  * @brief This function is used to select the interrupt source used to trigger other modules.
  * @param[in] lptmr The base address of LPTMR module
  * @param[in] u32Src Selects the interrupt source to trigger other modules. Could be:
  *              - \ref LPTMR_TRGSRC_TIMEOUT_EVENT
  *              - \ref LPTMR_TRGSRC_CAPTURE_EVENT
  * @return None
  */
void LPTMR_SetTriggerSource(LPTMR_T *lptmr, uint32_t u32Src)
{
    lptmr->TRGCTL = (lptmr->TRGCTL & ~LPTMR_TRGCTL_TRGSSEL_Msk) | u32Src;
}


/**
  * @brief This function is used to set modules trigger by lptmr interrupt
  * @param[in] lptmr The base address of LPTMR module
  * @param[in] u32Mask The mask of modules (Low power IPs and LPPDMA) trigger by lptmr. Is the combination of
  *             - \ref LPTMR_TRGEN
  *             - \ref LPTMR_TRG_TO_LPPDMA
  * @return None
  */
void LPTMR_SetTriggerTarget(LPTMR_T *lptmr, uint32_t u32Mask)
{
    lptmr->TRGCTL = (lptmr->TRGCTL & ~(LPTMR_TRGCTL_TRGEN_Msk | LPTMR_TRGCTL_TRGLPPDMA_Msk)) | u32Mask;
}


/**
  * @brief      Select LPTMR Capture Source
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module.
  * @param[in]  u32Src      LPTMR capture source. Possible values are
  *                         - \ref LPTMR_CAPTURE_FROM_EXTERNAL
  *                         - \ref LPTMR_CAPTURE_FROM_ACMP0
  *                         - \ref LPTMR_CAPTURE_FROM_ACMP1
  *                         - \ref LPTMR_CAPTURE_FROM_ACMP2
  *
  * @return     None
  *
  * @details    This API is used to select LPTMR capture source from Tx_EXT or internal signal.
  */
void LPTMR_CaptureSelect(LPTMR_T *lptmr, uint32_t u32Src)
{
    if (u32Src == LPTMR_CAPTURE_FROM_EXTERNAL)
    {
        lptmr->CTL = (lptmr->CTL & ~(LPTMR_CTL_CAPSRC_Msk)) |
                     (LPTMR_CAPSRC_TMX_EXT);
    }
    else
    {
        lptmr->CTL = (lptmr->CTL & ~(LPTMR_CTL_CAPSRC_Msk)) |
                     (LPTMR_CAPSRC_INTERNAL);
        lptmr->EXTCTL = (lptmr->EXTCTL & ~(LPTMR_EXTCTL_INTERCAPSEL_Msk)) |
                        (u32Src);
    }
}


/**
  * @brief      Reset LPTMR Counter
  *
  * @param[in]  lptmr The base address of Timer module
  *
  * @return     Reset success or not
  * @retval     0 Timer reset success
  * @retval     LPTMR_TIMEOUT_ERR Timer reset failed
  *
  * @details    This function is used to reset current counter value and internal prescale counter value.
  */
int32_t LPTMR_ResetCounter(LPTMR_T *lptmr)
{
    uint32_t u32Delay;

    lptmr->CNT |= LPTMR_CNT_RSTACT_Msk;
    /* Takes 2~3 ECLKs to reset timer counter */
    u32Delay = (SystemCoreClock / LPTMR_GetModuleClock(lptmr)) * 3UL;
    while(((lptmr->CNT & LPTMR_CNT_RSTACT_Msk) == LPTMR_CNT_RSTACT_Msk) && (--u32Delay))
    {
        __NOP();
    }
    return ((u32Delay > 0UL) ? 0L : LPTMR_TIMEOUT_ERR);
}

/**
  * @brief      Enable Capture Input Noise Filter Function
  *
  * @param[in]  lptmr           The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @param[in]  u32FilterCount  Noise filter counter. Valid values are between 0~7.
  *
  * @param[in]  u32ClkSrcSel    Noise filter counter clock source, could be one of following source
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_1
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_2
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_4
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_8
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_16
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_32
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_64
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_128
  *
  * @return     None
  *
  * @details    This function is used to enable capture input noise filter function.
  */
void LPTMR_EnableCaptureInputNoiseFilter(LPTMR_T *lptmr, uint32_t u32FilterCount, uint32_t u32ClkSrcSel)
{
    lptmr->CAPNF = ( ((lptmr)->CAPNF & ~(LPTMR_CAPNF_CAPNFEN_Msk | LPTMR_CAPNF_CAPNFCNT_Msk | LPTMR_CAPNF_CAPNFSEL_Msk))
                     | (LPTMR_CAPNF_CAPNFEN_Msk | (u32FilterCount << LPTMR_CAPNF_CAPNFCNT_Pos) | (u32ClkSrcSel << LPTMR_CAPNF_CAPNFSEL_Pos)) );
}

/**
  * @brief      Disable Capture Input Noise Filter Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable capture input noise filter function.
  */
void LPTMR_DisableCaptureInputNoiseFilter(LPTMR_T *lptmr)
{
    lptmr->CAPNF &= ~LPTMR_CAPNF_CAPNFEN_Msk;
}

/*@}*/ /* end of group LPTMR_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group LPTMR_Driver */

/*@}*/ /* end of group Standard_Driver */
