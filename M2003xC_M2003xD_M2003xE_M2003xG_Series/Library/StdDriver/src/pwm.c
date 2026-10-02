/**************************************************************************//**
 * @file     pwm.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 19/02/14 4:03p $
 * @brief    PWM driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup PWM_Driver PWM Driver
  @{
*/


/** @addtogroup PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief Configure PWM capture and get the nearest unit time.
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32UnitTimeNsec The unit time of counter
 * @param[in] u32CaptureEdge The condition to latch the counter. This parameter is not used
 * @return The nearest unit time in nano second.
 * @details This function is used to Configure PWM capture and get the nearest unit time.
 */
uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge)
{
    uint32_t u32PWMClockSrc;
    uint32_t u32NearestUnitTimeNsec = 0U;
    uint32_t u32LoopStop = 0U;
    uint32_t u32Prescale = 1U;
    uint16_t u16CNR = 0xFFFFU;

    (void)u32CaptureEdge;

    /* clock source is from PCLK */
    SystemCoreClockUpdate();
    u32PWMClockSrc = SystemCoreClock;

    if (u32PWMClockSrc == 0UL)
    {
        return 0UL;
    }

    u32PWMClockSrc /= 1000UL;
    while ((u32Prescale <= 0x1000U) && (u32LoopStop == 0U))
    {
        const uint64_t u64ScaleValue = ((uint64_t)1000000U * (uint64_t)u32Prescale);
        const uint64_t u64Candidate = u64ScaleValue / (uint64_t)u32PWMClockSrc;

        u32NearestUnitTimeNsec = (uint32_t)u64Candidate;

        if (u32NearestUnitTimeNsec < u32UnitTimeNsec)
        {
            if (u32Prescale == 0x1000U)  /* limit to the maximum unit time(nano second) */
            {
                u32LoopStop = 1U;
            }
            else
            {
                const uint32_t u32NextPrescale = u32Prescale + 1U;
                const uint64_t u64NextScaleValue = ((uint64_t)1000000U * (uint64_t)u32NextPrescale);

                if (u64NextScaleValue <= ((uint64_t)u32NearestUnitTimeNsec * (uint64_t)u32PWMClockSrc))
                {
                    u32LoopStop = 1U;
                }
                else
                {
                    u32Prescale++;
                }
            }
        }
        else
        {
            u32LoopStop = 1U;
        }
    }

    /* convert to real register value, every two channels share a prescaler */
    {
        const uint32_t u32RealPrescale = u32Prescale - 1U;
        PWM_SET_PRESCALER(pwm, u32ChannelNum, (uint16_t)u32RealPrescale);
    }

    /* set PWM to down count type(edge aligned) */
    (pwm)->CTL1 = ((pwm)->CTL1 & ~(PWM_CTL1_CNTTYPE0_Msk << ((u32ChannelNum >> 1UL) << 2UL))) | (1UL << ((u32ChannelNum >> 1UL) << 2UL));
    PWM_SET_CNR(pwm, u32ChannelNum, u16CNR);

    return (u32NearestUnitTimeNsec);
}

/**
 * @brief This function Configure PWM generator and get the nearest frequency in edge aligned(up counter type) auto-reload mode
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Frequency Target generator frequency
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second
 * @note Since every two channels, (0 & 1), (2 & 3), shares a prescaler. Call this API to configure PWM frequency may affect
 *       existing frequency of other channel.
 * @note This function is used for initial stage.
 *       To change duty cycle later, it should get the configured period value and calculate the new comparator value.
 */
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle)
{
    uint32_t u32PWMClockSrc;
    uint32_t u32Period;
    uint32_t u32LoopStop = 0U;
    uint32_t u32Prescale = 1U;
    uint16_t u16CNR = 0xFFFFU;

    if (u32Frequency == 0UL)
    {
        return 0UL;
    }

    /* clock source is from PCLK */
    SystemCoreClockUpdate();
    u32PWMClockSrc = SystemCoreClock;

    while ((u32Prescale < 0xFFFU) && (u32LoopStop == 0U)) /* prescale could be 0~0xFFF */
    {
        if (u32PWMClockSrc < u32Frequency)
        {
            u16CNR = 1U;
            u32LoopStop = 1U;
        }
        else
        {
            const uint32_t u32CandidatePeriod = (u32PWMClockSrc / u32Frequency) / u32Prescale;

            /* If target value is larger than CNR, need to use a larger prescaler */
            if (u32CandidatePeriod > 0x10000UL)
            {
                u32Prescale++;
            }
            else
            {
                u16CNR = (uint16_t)u32CandidatePeriod;
                if (u16CNR != 0U)
                {
                    u32LoopStop = 1U;
                }
                else
                {
                    u32Prescale++;
                }
            }
        }
    }

    if (u16CNR == 0U)
    {
        u16CNR = 1U;
    }

    {
        const uint32_t u32PrescaleValue = u32Prescale;
        const uint32_t u32CnrValue = (uint32_t)u16CNR;

        /*store return value here 'cos we're gonna change u16Prescale & u16CNR to the real value to fill into register */
        u32Period = u32PWMClockSrc / (u32PrescaleValue * u32CnrValue);
    }

    {
        const uint32_t u32RealPrescale = u32Prescale - 1U;
        const uint32_t u32RealCNR = (uint32_t)u16CNR - 1U;
        const uint32_t u32CmpValue = u32RealCNR + 1UL;

        /* convert to real register value, every two channels share a prescaler */
        PWM_SET_PRESCALER(pwm, u32ChannelNum, (uint16_t)u32RealPrescale);
        /* set PWM to up counter type (edge aligned) */
        (pwm)->CTL1 = ((pwm)->CTL1 & ~(PWM_CTL1_CNTTYPE0_Msk << ((u32ChannelNum >> 1UL) << 2UL))) | (0UL << ((u32ChannelNum >> 1UL) << 2UL));

        PWM_SET_CNR(pwm, u32ChannelNum, (uint16_t)u32RealCNR);
        PWM_SET_CMR(pwm, u32ChannelNum, (u32DutyCycle * u32CmpValue) / 100UL);
    }

    (pwm)->WGCTL0 = ((pwm)->WGCTL0 & ~(((uint32_t)PWM_WGCTL0_PRDPCTL0_Msk | (uint32_t)PWM_WGCTL0_ZPCTL0_Msk) << (u32ChannelNum << 1U))) | \
                    (((uint32_t)PWM_OUTPUT_HIGH) << (((uint32_t)u32ChannelNum << 1U) + (uint32_t)PWM_WGCTL0_ZPCTL0_Pos));
    (pwm)->WGCTL1 = ((pwm)->WGCTL1 & ~(((uint32_t)PWM_WGCTL1_CMPDCTL0_Msk | (uint32_t)PWM_WGCTL1_CMPUCTL0_Msk) << (u32ChannelNum << 1U))) | \
                    (((uint32_t)PWM_OUTPUT_LOW) << (((uint32_t)u32ChannelNum << 1U) + (uint32_t)PWM_WGCTL1_CMPUCTL0_Pos));

    return(u32Period);
}

/**
 * @brief Start PWM module
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to start PWM module.
 */
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t i;

    for (i = 0UL; i < (uint32_t)PWM_CHANNEL_NUM; i ++)
    {
        if ((u32ChannelMask & (1UL << i)) != 0UL)
        {
            (pwm)->CNTEN |= (1UL << ((i >> 1UL) << 1UL));
        }
    }
}

/**
 * @brief Stop PWM module
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to stop PWM module.
 */
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t i;

    for (i = 0UL; i < (uint32_t)PWM_CHANNEL_NUM; i ++)
    {
        if ((u32ChannelMask & (1UL << i)) != 0UL)
        {
            (pwm)->PERIOD[((i >> 1UL) << 1UL)] = 0UL;
        }
    }
}

/**
 * @brief Stop PWM generation immediately by clear channel enable bit
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to stop PWM generation immediately by clear channel enable bit.
 */
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t i;

    for (i = 0UL; i < (uint32_t)PWM_CHANNEL_NUM; i ++)
    {
        if ((u32ChannelMask & (1UL << i)) != 0UL)
        {
            (pwm)->CNTEN &= ~(1UL << ((i >> 1UL) << 1UL));
        }
    }
}

/**
 * @brief Enable selected channel to trigger ADC
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Condition The condition to trigger ADC. Combination of following conditions:
 *                  - \ref PWM_TRIGGER_ADC_EVEN_ZERO_POINT
 *                  - \ref PWM_TRIGGER_ADC_EVEN_PERIOD_POINT
 *                  - \ref PWM_TRIGGER_ADC_EVEN_ZERO_OR_PERIOD_POINT
 *                  - \ref PWM_TRIGGER_ADC_EVEN_COMPARE_UP_COUNT_POINT
 *                  - \ref PWM_TRIGGER_ADC_EVEN_COMPARE_DOWN_COUNT_POINT
 *                  - \ref PWM_TRIGGER_ADC_ODD_ZERO_POINT
 *                  - \ref PWM_TRIGGER_ADC_ODD_PERIOD_POINT
 *                  - \ref PWM_TRIGGER_ADC_ODD_ZERO_OR_PERIOD_POINT
 *                  - \ref PWM_TRIGGER_ADC_ODD_COMPARE_UP_COUNT_POINT
 *                  - \ref PWM_TRIGGER_ADC_ODD_COMPARE_DOWN_COUNT_POINT
 *                  - \ref PWM_TRIGGER_ADC_CH_0_FREE_COMPARE_UP_COUNT_POINT
 *                  - \ref PWM_TRIGGER_ADC_CH_0_FREE_COMPARE_DOWN_COUNT_POINT
 *                  - \ref PWM_TRIGGER_ADC_CH_2_FREE_COMPARE_UP_COUNT_POINT
 *                  - \ref PWM_TRIGGER_ADC_CH_2_FREE_COMPARE_DOWN_COUNT_POINT
 *                  - \ref PWM_TRIGGER_ADC_CH_4_FREE_COMPARE_UP_COUNT_POINT
 *                  - \ref PWM_TRIGGER_ADC_CH_4_FREE_COMPARE_DOWN_COUNT_POINT
 * @return None
 * @details This function is used to enable selected channel to trigger ADC.
 */
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if (u32ChannelNum < 4UL)
    {
        const uint32_t u32Shift = (u32ChannelNum << 3U);

        (pwm)->ADCTS0 &= ~(((uint32_t)PWM_ADCTS0_TRGSEL0_Msk) << u32Shift);
        (pwm)->ADCTS0 |= ((((uint32_t)PWM_ADCTS0_TRGEN0_Msk) | u32Condition) << u32Shift);
    }
    else
    {
        const uint32_t u32Shift = ((u32ChannelNum - 4UL) << 3U);

        (pwm)->ADCTS1 &= ~(((uint32_t)PWM_ADCTS1_TRGSEL4_Msk) << u32Shift);
        (pwm)->ADCTS1 |= ((((uint32_t)PWM_ADCTS1_TRGEN4_Msk) | u32Condition) << u32Shift);
    }
}

/**
 * @brief Disable selected channel to trigger ADC
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to disable selected channel to trigger ADC.
 */
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum)
{
    if (u32ChannelNum < 4UL)
    {
        const uint32_t u32Shift = (u32ChannelNum << 3U);
        (pwm)->ADCTS0 &= ~(((uint32_t)PWM_ADCTS0_TRGEN0_Msk) << u32Shift);
    }
    else
    {
        const uint32_t u32Shift = ((u32ChannelNum - 4UL) << 3U);
        (pwm)->ADCTS1 &= ~(((uint32_t)PWM_ADCTS1_TRGEN4_Msk) << u32Shift);
    }
}

/**
 * @brief Clear selected channel trigger ADC flag
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Condition This parameter is not used
 * @return None
 * @details This function is used to clear selected channel trigger ADC flag.
 */
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    (void)u32Condition;
    (pwm)->STATUS = ((uint32_t)PWM_STATUS_ADCTRG0_Msk << u32ChannelNum);
}

/**
 * @brief Get selected channel trigger ADC flag
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @retval 0 The specified channel trigger ADC to start of conversion flag is not set
 * @retval 1 The specified channel trigger ADC to start of conversion flag is set
 * @details This function is used to get PWM trigger ADC to start of conversion flag for specified channel.
 */
uint32_t PWM_GetADCTriggerFlag(const PWM_T *pwm, uint32_t u32ChannelNum)
{
    return ((((pwm)->STATUS & ((uint32_t)PWM_STATUS_ADCTRG0_Msk << u32ChannelNum)) != 0UL) ? 1U : 0U);
}

/**
 * @brief This function enable fault brake of selected channel(s)
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 * @param[in] u32LevelMask Output high or low while fault brake occurs, each bit represent the level of a channel
 *                         while fault brake occurs. Bit 0 represents channel 0, bit 1 represents channel 1...
 * @param[in] u32BrakeSource Fault brake source, could be one of following source
 *                  - \ref PWM_FB_EDGE_ACMP0
 *                  - \ref PWM_FB_EDGE_ACMP1
 *                  - \ref PWM_FB_EDGE_BKP0
 *                  - \ref PWM_FB_EDGE_BKP1
 *                  - \ref PWM_FB_EDGE_SYS_CSS
 *                  - \ref PWM_FB_EDGE_SYS_BOD
 *                  - \ref PWM_FB_EDGE_SYS_COR
 *                  - \ref PWM_FB_LEVEL_ACMP0
 *                  - \ref PWM_FB_LEVEL_ACMP1
 *                  - \ref PWM_FB_LEVEL_BKP0
 *                  - \ref PWM_FB_LEVEL_BKP1
 *                  - \ref PWM_FB_LEVEL_SYS_CSS
 *                  - \ref PWM_FB_LEVEL_SYS_BOD
 *                  - \ref PWM_FB_LEVEL_SYS_COR
 * @return None
 * @details This function is used to enable fault brake of selected channel(s).
 *          The write-protection function should be disabled before using this function.
 */
void PWM_EnableFaultBrake(PWM_T *pwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource)
{
    uint32_t i;

    for(i = 0UL; i < (uint32_t)PWM_CHANNEL_NUM; i ++)
    {
        if (((u32ChannelMask & ((uint32_t)1UL << (uint32_t)i)) != 0UL))
        {
            if ((u32BrakeSource == PWM_FB_EDGE_SYS_BOD) || \
                    (u32BrakeSource == PWM_FB_EDGE_SYS_COR) || \
                    (u32BrakeSource == PWM_FB_LEVEL_SYS_BOD) || \
                    (u32BrakeSource == PWM_FB_LEVEL_SYS_COR))
            {
                (pwm)->BRKCTL[i >> 1UL] |= (u32BrakeSource & (PWM_BRKCTL_SYSEBEN_Msk | PWM_BRKCTL_SYSLBEN_Msk));
                (pwm)->FAILBRK |= (u32BrakeSource & 0xBUL);
            }
            else
            {
                (pwm)->BRKCTL[i >> 1UL] |= u32BrakeSource;
            }
        }

        if (((u32LevelMask & ((uint32_t)1UL << (uint32_t)i)) != 0UL))
        {
            if ((i & 0x1UL) == 0UL)
            {
                /* set brake action as high level for even channel */
                (pwm)->BRKCTL[i >> 1UL] &= ~PWM_BRKCTL_BRKAEVEN_Msk;
                (pwm)->BRKCTL[i >> 1UL] |= ((3UL) << PWM_BRKCTL_BRKAEVEN_Pos);
            }
            else
            {
                /* set brake action as high level for odd channel */
                (pwm)->BRKCTL[i >> 1UL] &= ~PWM_BRKCTL_BRKAODD_Msk;
                (pwm)->BRKCTL[i >> 1UL] |= ((3UL) << PWM_BRKCTL_BRKAODD_Pos);
            }
        }
        else
        {
            if ((i & 0x1UL) == 0UL)
            {
                /* set brake action as low level for even channel */
                (pwm)->BRKCTL[i >> 1UL] &= ~PWM_BRKCTL_BRKAEVEN_Msk;
                (pwm)->BRKCTL[i >> 1UL] |= ((2UL) << PWM_BRKCTL_BRKAEVEN_Pos);
            }
            else
            {
                /* set brake action as low level for odd channel */
                (pwm)->BRKCTL[i >> 1UL] &= ~PWM_BRKCTL_BRKAODD_Msk;
                (pwm)->BRKCTL[i >> 1UL] |= ((2UL) << PWM_BRKCTL_BRKAODD_Pos);
            }
        }
    }
}

/**
 * @brief Enable capture of selected channel(s)
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to enable capture of selected channel(s).
 */
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask)
{
    (pwm)->CAPINEN |= u32ChannelMask;
    (pwm)->CAPCTL |= u32ChannelMask;
}

/**
 * @brief Disable capture of selected channel(s)
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to disable capture of selected channel(s).
 */
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask)
{
    (pwm)->CAPINEN &= ~u32ChannelMask;
    (pwm)->CAPCTL &= ~u32ChannelMask;
}

/**
 * @brief Enables PWM output generation of selected channel(s)
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Set bit 0 to 1 enables channel 0 output, set bit 1 to 1 enables channel 1 output...
 * @return None
 * @details This function is used to enable PWM output generation of selected channel(s).
 */
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask)
{
    (pwm)->POEN |= u32ChannelMask;
}

/**
 * @brief Disables PWM output generation of selected channel(s)
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Set bit 0 to 1 disables channel 0 output, set bit 1 to 1 disables channel 1 output...
 * @return None
 * @details This function is used to disable PWM output generation of selected channel(s).
 */
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask)
{
    (pwm)->POEN &= ~u32ChannelMask;
}

/**
 * @brief Enables PDMA transfer of selected channel for PWM capture
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 *                - PWM1 : PWM Group 1
 * @param[in] u32ChannelNum PWM channel number.
 * @param[in] u32RisingFirst The capture order is rising, falling first. Every two channels share the same setting. Valid values are TRUE and FALSE.
 * @param[in] u32Mode Captured data transferred by PDMA interrupt type. It could be either
 *              - \ref PWM_CAPTURE_PDMA_RISING_LATCH
 *              - \ref PWM_CAPTURE_PDMA_FALLING_LATCH
 *              - \ref PWM_CAPTURE_PDMA_RISING_FALLING_LATCH
 * @return None
 * @details This function is used to enable PDMA transfer of selected channel(s) for PWM capture.
 * @note This function can only selects even or odd channel of pairs to do PDMA transfer.
 */
void PWM_EnablePDMA(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32RisingFirst, uint32_t u32Mode)
{
    uint32_t u32IsOddCh;
    uint32_t u32Shift;

    u32IsOddCh = u32ChannelNum & 0x1U;
    u32Shift = ((u32ChannelNum >> 1U) << 3U);
    (pwm)->PDMACTL = ((pwm)->PDMACTL & ~((PWM_PDMACTL_CHSEL0_1_Msk | PWM_PDMACTL_CAPORD0_1_Msk | PWM_PDMACTL_CAPMOD0_1_Msk) << u32Shift)) | \
                     ((((uint32_t)u32IsOddCh << (uint32_t)PWM_PDMACTL_CHSEL0_1_Pos) | ((uint32_t)u32RisingFirst << (uint32_t)PWM_PDMACTL_CAPORD0_1_Pos) | \
                       (uint32_t)u32Mode | (uint32_t)PWM_PDMACTL_CHEN0_1_Msk) << u32Shift);
}

/**
 * @brief Disables PDMA transfer of selected channel for PWM capture
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 *                - PWM1 : PWM Group 1
 * @param[in] u32ChannelNum PWM channel number.
 * @return None
 * @details This function is used to enable PDMA transfer of selected channel(s) for PWM capture.
 */
void PWM_DisablePDMA(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->PDMACTL &= ~(PWM_PDMACTL_CHEN0_1_Msk << ((u32ChannelNum >> 1) << 3));
}

/**
 * @brief Enable Dead zone of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Duration Dead zone length in PWM clock count, valid values are between 0~0xFFF, but 0 means there is no Dead zone.
 * @return None
 * @details This function is used to enable Dead zone of selected channel.
 *          The write-protection function should be disabled before using this function.
 * @note Every two channels share the same setting.
 */
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration)
{
    /* every two channels share the same setting */
    (pwm)->DTCTL[(u32ChannelNum) >> 1UL] &= ~PWM_DTCTL_DTCNT_Msk;
    (pwm)->DTCTL[(u32ChannelNum) >> 1UL] |= PWM_DTCTL_DTEN_Msk | u32Duration;
}

/**
 * @brief Disable Dead zone of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to disable Dead zone of selected channel.
 *          The write-protection function should be disabled before using this function.
 */
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum)
{
    /* every two channels shares the same setting */
    (pwm)->DTCTL[(u32ChannelNum) >> 1UL] &= ~PWM_DTCTL_DTEN_Msk;
}

/**
 * @brief Enable capture interrupt of selected channel.
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref PWM_CAPTURE_INT_RISING_LATCH
 *              - \ref PWM_CAPTURE_INT_FALLING_LATCH
 * @return None
 * @details This function is used to enable capture interrupt of selected channel.
 */
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge)
{
    (pwm)->CAPIEN |= ((uint32_t)u32Edge << u32ChannelNum);
}

/**
 * @brief Disable capture interrupt of selected channel.
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref PWM_CAPTURE_INT_RISING_LATCH
 *              - \ref PWM_CAPTURE_INT_FALLING_LATCH
 * @return None
 * @details This function is used to disable capture interrupt of selected channel.
 */
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge)
{
    (pwm)->CAPIEN &= ~((uint32_t)u32Edge << u32ChannelNum);
}

/**
 * @brief Clear capture interrupt of selected channel.
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref PWM_CAPTURE_INT_RISING_LATCH
 *              - \ref PWM_CAPTURE_INT_FALLING_LATCH
 * @return None
 * @details This function is used to clear capture interrupt of selected channel.
 */
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge)
{
    (pwm)->CAPIF = ((uint32_t)u32Edge << u32ChannelNum);
}

/**
 * @brief Get capture interrupt of selected channel.
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @retval 0 No capture interrupt
 * @retval 1 Rising edge latch interrupt
 * @retval 2 Falling edge latch interrupt
 * @retval 3 Rising and falling latch interrupt
 * @details This function is used to get capture interrupt of selected channel.
 */
uint32_t PWM_GetCaptureIntFlag(const PWM_T *pwm, uint32_t u32ChannelNum)
{
    const uint32_t u32FallingMask = ((uint32_t)PWM_CAPIF_CFLIF0_Msk << u32ChannelNum);
    const uint32_t u32RisingMask = ((uint32_t)PWM_CAPIF_CRLIF0_Msk << u32ChannelNum);
    const uint32_t u32FallingStatus = ((((pwm)->CAPIF & u32FallingMask) != 0UL) ? 1U : 0U);
    const uint32_t u32RisingStatus = ((((pwm)->CAPIF & u32RisingMask) != 0UL) ? 1U : 0U);

    return ((u32FallingStatus << 1U) | u32RisingStatus);
}

/**
 * @brief Enable duty interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32IntDutyType Duty interrupt type, could be either
 *              - \ref PWM_DUTY_INT_DOWN_COUNT_MATCH_CMP
 *              - \ref PWM_DUTY_INT_UP_COUNT_MATCH_CMP
 * @return None
 * @details This function is used to enable duty interrupt of selected channel.
 */
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType)
{
    (pwm)->INTEN0 |= (u32IntDutyType << u32ChannelNum);
}

/**
 * @brief Disable duty interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to disable duty interrupt of selected channel.
 */
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN0 &= ~((PWM_DUTY_INT_DOWN_COUNT_MATCH_CMP | PWM_DUTY_INT_UP_COUNT_MATCH_CMP) << u32ChannelNum);
}

/**
 * @brief Clear duty interrupt flag of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to clear duty interrupt flag of selected channel.
 */
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTSTS0 = (((uint32_t)PWM_INTSTS0_CMPUIF0_Msk | (uint32_t)PWM_INTSTS0_CMPDIF0_Msk) << u32ChannelNum);
}

/**
 * @brief Get duty interrupt flag of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Duty interrupt flag of specified channel
 * @retval 0 Duty interrupt did not occur
 * @retval 1 Duty interrupt occurred
 * @details This function is used to get duty interrupt flag of selected channel.
 */
uint32_t PWM_GetDutyIntFlag(const PWM_T *pwm, uint32_t u32ChannelNum)
{
    const uint32_t u32Mask = (((uint32_t)PWM_INTSTS0_CMPDIF0_Msk | (uint32_t)PWM_INTSTS0_CMPUIF0_Msk) << u32ChannelNum);
    return ((((pwm)->INTSTS0 & u32Mask) != 0UL) ? 1U : 0U);
}

/**
 * @brief This function enable fault brake interrupt
 * @param[in] pwm The pointer of the specified PWM module
 * @param[in] u32BrakeSource Fault brake source.
 *              - \ref PWM_FB_EDGE
 *              - \ref PWM_FB_LEVEL
 * @return None
 * @details This function is used to enable fault brake interrupt.
 *          The write-protection function should be disabled before using this function.
 * @note Every two channels share the same setting.
 */
void PWM_EnableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource)
{
    const uint32_t u32Mask = 0x7UL << u32BrakeSource;

    (pwm)->INTEN1 |= u32Mask;
}

/**
 * @brief This function disable fault brake interrupt
 * @param[in] pwm The pointer of the specified PWM module
 * @param[in] u32BrakeSource Fault brake source.
 *              - \ref PWM_FB_EDGE
 *              - \ref PWM_FB_LEVEL
 * @return None
 * @details This function is used to disable fault brake interrupt.
 *          The write-protection function should be disabled before using this function.
 * @note Every two channels share the same setting.
 */
void PWM_DisableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource)
{
    const uint32_t u32Mask = 0x7UL << u32BrakeSource;

    (pwm)->INTEN1 &= ~u32Mask;
}

/**
 * @brief This function clear fault brake interrupt of selected source
 * @param[in] pwm The pointer of the specified PWM module
 * @param[in] u32BrakeSource Fault brake source.
 *              - \ref PWM_FB_EDGE
 *              - \ref PWM_FB_LEVEL
 * @return None
 * @details This function is used to clear fault brake interrupt of selected source.
 *          The write-protection function should be disabled before using this function.
 */
void PWM_ClearFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource)
{
    const uint32_t u32Mask = 0x3fUL << u32BrakeSource;

    (pwm)->INTSTS1 = u32Mask;
}

/**
 * @brief This function get fault brake interrupt flag of selected source
 * @param[in] pwm The pointer of the specified PWM module
 * @param[in] u32BrakeSource Fault brake source, could be either
 *              - \ref PWM_FB_EDGE
 *              - \ref PWM_FB_LEVEL
 * @return Fault brake interrupt flag of specified source
 * @retval 0 Fault brake interrupt did not occurred
 * @retval 1 Fault brake interrupt occurred
 * @details This function is used to get fault brake interrupt flag of selected source.
 */
uint32_t PWM_GetFaultBrakeIntFlag(const PWM_T *pwm, uint32_t u32BrakeSource)
{
    return (((pwm)->INTSTS1 & (0x3fU << u32BrakeSource)) ? 1U : 0U);
}

/**
 * @brief Enable period interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32IntPeriodType Period interrupt type. This parameter is not used.
 * @return None
 * @details This function is used to enable period interrupt of selected channel.
 */
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType)
{
    (void)u32IntPeriodType;
    (pwm)->INTEN0 |= ((uint32_t)PWM_INTEN0_PIEN0_Msk << ((u32ChannelNum >> 1UL) << 1UL));
}

/**
 * @brief Disable period interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to disable period interrupt of selected channel.
 */
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN0 &= ~((uint32_t)PWM_INTEN0_PIEN0_Msk << ((u32ChannelNum >> 1UL) << 1UL));
}

/**
 * @brief Clear period interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to clear period interrupt of selected channel.
 */
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTSTS0 = ((uint32_t)PWM_INTSTS0_PIF0_Msk << ((u32ChannelNum >> 1UL) << 1UL));
}

/**
 * @brief Get period interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Period interrupt flag of specified channel
 * @retval 0 Period interrupt did not occur
 * @retval 1 Period interrupt occurred
 * @details This function is used to get period interrupt of selected channel.
 */
uint32_t PWM_GetPeriodIntFlag(const PWM_T *pwm, uint32_t u32ChannelNum)
{
    const uint32_t u32Mask = ((uint32_t)PWM_INTSTS0_PIF0_Msk << ((u32ChannelNum >> 1UL) << 1UL));
    return ((((pwm)->INTSTS0 & u32Mask) != 0UL) ? 1U : 0U);
}

/**
 * @brief Enable zero interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to enable zero interrupt of selected channel.
 */
void PWM_EnableZeroInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN0 |= ((uint32_t)PWM_INTEN0_ZIEN0_Msk << ((u32ChannelNum >> 1UL) << 1UL));
}

/**
 * @brief Disable zero interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to disable zero interrupt of selected channel.
 */
void PWM_DisableZeroInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN0 &= ~((uint32_t)PWM_INTEN0_ZIEN0_Msk << ((u32ChannelNum >> 1UL) << 1UL));
}

/**
 * @brief Clear zero interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to clear zero interrupt of selected channel.
 */
void PWM_ClearZeroIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTSTS0 = ((uint32_t)PWM_INTSTS0_ZIF0_Msk << ((u32ChannelNum >> 1UL) << 1UL));
}

/**
 * @brief Get zero interrupt of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Zero interrupt flag of specified channel
 * @retval 0 Zero interrupt did not occur
 * @retval 1 Zero interrupt occurred
 * @details This function is used to get zero interrupt of selected channel.
 */
uint32_t PWM_GetZeroIntFlag(const PWM_T *pwm, uint32_t u32ChannelNum)
{
    const uint32_t u32Mask = ((uint32_t)PWM_INTSTS0_ZIF0_Msk << ((u32ChannelNum >> 1UL) << 1UL));
    return ((((pwm)->INTSTS0 & u32Mask) != 0UL) ? 1U : 0U);
}

/**
 * @brief Enable load mode of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32LoadMode PWM counter loading mode.
 *              - \ref PWM_LOAD_MODE_IMMEDIATE
 *              - \ref PWM_LOAD_MODE_WINDOW
 *              - \ref PWM_LOAD_MODE_CENTER
 * @return None
 * @details This function is used to enable load mode of selected channel.
 */
void PWM_EnableLoadMode(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32LoadMode)
{
    (pwm)->CTL0 |= ((uint32_t)u32LoadMode << u32ChannelNum);
}

/**
 * @brief Disable load mode of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32LoadMode PWM counter loading mode.
 *              - \ref PWM_LOAD_MODE_IMMEDIATE
 *              - \ref PWM_LOAD_MODE_WINDOW
 *              - \ref PWM_LOAD_MODE_CENTER
 * @return None
 * @details This function is used to disable load mode of selected channel.
 */
void PWM_DisableLoadMode(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32LoadMode)
{
    (pwm)->CTL0 &= ~((uint32_t)u32LoadMode << u32ChannelNum);
}

/**
 * @brief Set PWM clock source
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32ClkSrcSel PWM external clock source.
 *              - \ref PWM_CLKSRC_PWM_CLK
 *              - \ref PWM_CLKSRC_TIMER0
 *              - \ref PWM_CLKSRC_TIMER1
 *              - \ref PWM_CLKSRC_TIMER2
 *              - \ref PWM_CLKSRC_TIMER3
 * @return None
 * @details This function is used to set PWM clock source.
 * @note Every two channels share the same setting.
 * @note If the clock source of PWM counter is selected from TIMERn interrupt events, the TRGPWM(TIMERn_TRGCTL[1], n=0,1..3) bit must be set as 1.
 */
void PWM_SetClockSource(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel)
{
    const uint32_t u32Shift = ((u32ChannelNum >> 1UL) << 3U);

    (pwm)->CLKSRC &= ~((uint32_t)PWM_CLKSRC_ECLKSRC0_Msk << u32Shift);
    (pwm)->CLKSRC |= ((uint32_t)u32ClkSrcSel << u32Shift);
}

/**
 * @brief Enable PWM brake noise filter function
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32BrakePinNum Brake pin selection. Valid values are 0 or 1.
 * @param[in] u32ClkCnt SYNC Edge Detector Filter Count. This controls the counter number of edge detector
 * @param[in] u32ClkDivSel SYNC Edge Detector Filter Clock Selection.
 *              - \ref PWM_NF_CLK_DIV_1
 *              - \ref PWM_NF_CLK_DIV_2
 *              - \ref PWM_NF_CLK_DIV_4
 *              - \ref PWM_NF_CLK_DIV_8
 *              - \ref PWM_NF_CLK_DIV_16
 *              - \ref PWM_NF_CLK_DIV_32
 *              - \ref PWM_NF_CLK_DIV_64
 *              - \ref PWM_NF_CLK_DIV_128
 * @return None
 * @details This function is used to enable PWM brake noise filter function.
 */
void PWM_EnableBrakeNoiseFilter(PWM_T *pwm, uint32_t u32BrakePinNum, uint32_t u32ClkCnt, uint32_t u32ClkDivSel)
{
    const uint32_t u32Shift = (u32BrakePinNum << 3U);
    const uint32_t u32FilterMask = ((uint32_t)PWM_BNF_BRK0FCNT_Msk | (uint32_t)PWM_BNF_BRK0NFSEL_Msk);
    const uint32_t u32FilterValue = (((uint32_t)u32ClkCnt << (uint32_t)PWM_BNF_BRK0FCNT_Pos) |
                                    ((uint32_t)u32ClkDivSel << (uint32_t)PWM_BNF_BRK0NFSEL_Pos) |
                                    (uint32_t)PWM_BNF_BRK0NFEN_Msk);

    (pwm)->BNF = ((pwm)->BNF & ~(u32FilterMask << u32Shift)) | (u32FilterValue << u32Shift);
}

/**
 * @brief Disable PWM brake noise filter function
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 
 * @param[in] u32BrakePinNum Brake pin selection. Valid values are 0 or 1.
 * @return None
 * @details This function is used to disable PWM brake noise filter function.
 */
void PWM_DisableBrakeNoiseFilter(PWM_T *pwm, uint32_t u32BrakePinNum)
{
    (pwm)->BNF &= ~((uint32_t)PWM_BNF_BRK0NFEN_Msk << (u32BrakePinNum << 3U));
}

/**
 * @brief Enable PWM brake pin inverse function
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32BrakePinNum Brake pin selection. Valid values are 0 or 1.
 * @return None
 * @details This function is used to enable PWM brake pin inverse function.
 */
void PWM_EnableBrakePinInverse(PWM_T *pwm, uint32_t u32BrakePinNum)
{
    (pwm)->BNF |= ((uint32_t)PWM_BNF_BRK0PINV_Msk << (u32BrakePinNum << 3U));
}

/**
 * @brief Disable PWM brake pin inverse function
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32BrakePinNum Brake pin selection. Valid values are 0 or 1.
 * @return None
 * @details This function is used to disable PWM brake pin inverse function.
 */
void PWM_DisableBrakePinInverse(PWM_T *pwm, uint32_t u32BrakePinNum)
{
    (pwm)->BNF &= ~((uint32_t)PWM_BNF_BRK0PINV_Msk << ((uint32_t)u32BrakePinNum * (uint32_t)PWM_BNF_BRK1NFEN_Pos));
}

/**
 * @brief Set PWM brake pin source
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32BrakePinNum Brake pin selection. Valid values are 0 or 1.
 * @param[in] u32SelAnotherModule Select to another module. Valid values are TRUE or FALSE.
 * @return None
 * @details This function is used to set PWM brake pin source.
 * @note This function is only supported in M45xD/M45xC.
 */
void PWM_SetBrakePinSource(PWM_T *pwm, uint32_t u32BrakePinNum, uint32_t u32SelAnotherModule)
{
    const uint32_t u32Shift = (u32BrakePinNum << 3U);

    (pwm)->BNF = ((pwm)->BNF & ~((uint32_t)PWM_BNF_BK0SRC_Msk << u32Shift)) |
                 (((uint32_t)u32SelAnotherModule) << ((uint32_t)PWM_BNF_BK0SRC_Pos + u32Shift));
}

/**
 * @brief Get the time-base counter reached its maximum value flag of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Count to max interrupt flag of specified channel
 * @retval 0 Count to max interrupt did not occur
 * @retval 1 Count to max interrupt occurred
 * @details This function is used to get the time-base counter reached its maximum value flag of selected channel.
 */
uint32_t PWM_GetWrapAroundFlag(const PWM_T *pwm, uint32_t u32ChannelNum)
{
    return (((pwm)->STATUS & (PWM_STATUS_CNTMAX0_Msk << u32ChannelNum)) ? 1U : 0U);
}

/**
 * @brief Clear the time-base counter reached its maximum value flag of selected channel
 * @param[in] pwm The pointer of the specified PWM module
 *                - PWM0 : PWM Group 0
 
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to clear the time-base counter reached its maximum value flag of selected channel.
 */
void PWM_ClearWrapAroundFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->STATUS = (PWM_STATUS_CNTMAX0_Msk << u32ChannelNum);
}

/**
 * @brief This macro enable PWM trigger ECAP window mode
 * @param[in] pwm The pointer of the specified PWM module
 * @param[in] u32Ecap ECAP number
 * @param[in] u32ChannelDuty PWM channel duty selection
 * @return None
 * @details This macro is used to enable PWM trigger ECAP window mode.
 * \hideinitializer
 */
void PWM_ENABLE_ECAP_WINDOW(PWM_T *pwm, uint32_t u32Ecap, uint32_t u32ChannelDuty)
{
    const uint32_t u32Shift = 8UL * u32Ecap;

    pwm->WECAP &= ~(PWM_WECAP_TRGWECSEL0_Msk << u32Shift);
    pwm->WECAP |= ((u32ChannelDuty & 0x0FU) << u32Shift);
    pwm->WECAP |= (PWM_WECAP_TRGWEC0_Msk << u32Shift);
}

/**
 * @brief This macro disable PWM trigger ECAP window mode
 * @param[in] pwm The pointer of the specified PWM module
 * @param[in] u32Ecap ECAP number
 * @return None
 * @details This macro is used to disable PWM trigger ECAP window mode.
 * \hideinitializer
 */
void PWM_DISABLE_ECAP_WINDOW(PWM_T *pwm, uint32_t u32Ecap)
{
    const uint32_t u32Shift = 8UL * u32Ecap;

    pwm->WECAP &= ~(PWM_WECAP_TRGWEC0_Msk << u32Shift);
}


/*@}*/ /* end of group PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PWM_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
