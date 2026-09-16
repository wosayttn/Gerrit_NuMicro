/**************************************************************************//**
 * @file     adc.c
 * @version  V1.00
 * @brief    M2U51 Series ADC Driver Source File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup ADC_Driver ADC Driver
    @{
*/

int32_t g_ADC_i32ErrCode = 0;   /*!< ADC global error code */

/** @addtogroup ADC_EXPORTED_FUNCTIONS ADC Exported Functions
    @{
*/

/**
  * @brief This API configures ADC module to be ready for convert the input from selected channel
  *
  * @param[in] adc          The pointer of the specified ADC module
  * @param[in] u32InputMode Decides the ADC analog input mode. M2U51 don't support it.
  * @param[in] u32OpMode    Decides the ADC operation mode. Valid values are:
  *                         - \ref ADC_ADCR_ADMD_SINGLE         :Single mode.
  *                         - \ref ADC_ADCR_ADMD_BURST          :Burst mode.
  *                         - \ref ADC_ADCR_ADMD_SINGLE_CYCLE   :Single cycle scan mode.
  *                         - \ref ADC_ADCR_ADMD_CONTINUOUS     :Continuous scan mode.
  * @param[in] u32ChMask    Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1..., bit 15 is channel 15.
  *
  * @return    None
  *
  * @note ADC can only convert 1 channel at a time. If more than 1 channels are enabled, only channel
  *       with smallest number will be convert.
  * @note This API does not turn on ADC power nor does trigger ADC conversion.
  */
void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask)
{
    adc->ADCR = (adc->ADCR & (~(ADC_ADCR_ADMD_Msk))) | (u32OpMode);
    adc->ADCHER = (adc->ADCHER & ~(ADC_ADCHER_CHEN_Msk)) | (u32ChMask);
    return;
}

/**
  * @brief Disable ADC module
  *
  * @param[in] adc The pointer of the specified ADC module
  *
  * @return None
  */
void ADC_Close(ADC_T *adc)
{
    SYS->IPRST1 |= SYS_IPRST1_ADC0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADC0RST_Msk;
    return;
}

/**
  * @brief Configure the hardware trigger condition and enable hardware trigger
  *
  * @param[in] adc       The pointer of the specified ADC module
  * @param[in] u32Source Decides the hardware trigger source. Valid values are:
  *                       - \ref ADC_ADCR_TRGS_STADC            :A/D conversion is started by external STADC pin.
  *                       - \ref ADC_ADCR_TRGS_TIMER            :A/D conversion is started by Timer.
  *                       - \ref ADC_ADCR_TRGS_BPWM             :A/D conversion is started by BPWM.
  *                       - \ref ADC_ADCR_TRGS_PWM              :A/D conversion is started by PWM.
  *                       - \ref ADC_ADCR_TRGS_ACMP0            :A/D conversion is started by ACMP0.
  *                       - \ref ADC_ADCR_TRGS_ACMP1            :A/D conversion is started by ACMP1.
  * @param[in] u32Param  While ADC trigger by external pin, this parameter is used to set trigger condition.
  *                      Valid values are:
  *                       - \ref ADC_ADCR_TRGCOND_LOW_LEVEL     :STADC Low level active
  *                       - \ref ADC_ADCR_TRGCOND_HIGH_LEVEL    :STADC High level active
  *                       - \ref ADC_ADCR_TRGCOND_FALLING_EDGE  :STADC Falling edge active
  *                       - \ref ADC_ADCR_TRGCOND_RISING_EDGE   :STADC Rising edge active
  *                      While ADC trigger by ACMP0 or ACMP1, this parameter is used to set trigger condition.
  *                      Valid values are:
  *                       - \ref ADC_ADCR_ACMPTES_BOTH_EDGE     :ACMP Both edge active
  *                       - \ref ADC_ADCR_ACMPTES_RISING_EDGE   :ACMP Rising edge active
  *                       - \ref ADC_ADCR_ACMPTES_FALLING_EDGE  :ACMP Falling edge active
  *                      While ADC trigger by other source, this parameter is unused.
  *
  * @return None
  *
  * @note Software should disable TRGEN and ADST before change TRGS.
  */
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param)
{
    /* Software should clear TRGEN bit and ADST bit before changing TRGS bits. */
    adc->ADCR &= ~(ADC_ADCR_TRGEN_Msk | ADC_ADCR_ADST_Msk);

    if(u32Source == ADC_ADCR_TRGS_STADC)
    {
        adc->ADCR = (adc->ADCR & ~(ADC_ADCR_TRGS_Msk | ADC_ADCR_TRGCOND_Msk)) |
                    ((u32Source) | (u32Param) | ADC_ADCR_TRGEN_Msk);
    }
    else if ((u32Source == ADC_ADCR_TRGS_ACMP0) || (u32Source == ADC_ADCR_TRGS_ACMP1))
    {
        adc->ADCR = (adc->ADCR & ~(ADC_ADCR_TRGS_Msk | ADC_ADCR_ACMPTES_Msk)) |
                    ((u32Source) | (u32Param) | ADC_ADCR_TRGEN_Msk);
    }
    else
    {
        adc->ADCR = (adc->ADCR & ~(ADC_ADCR_TRGS_Msk)) |
                    ((u32Source) | ADC_ADCR_TRGEN_Msk);
    }
    return;
}

/**
  * @brief Disable hardware trigger ADC function.
  *
  * @param[in] adc The pointer of the specified ADC module
  *
  * @return None
  */
void ADC_DisableHWTrigger(ADC_T *adc)
{
    /* Software should clear TRGEN bit and ADST bit before changing TRGS bits. */
    adc->ADCR &= ~(ADC_ADCR_TRGEN_Msk | ADC_ADCR_ADST_Msk);

    adc->ADCR &= ~(ADC_ADCR_TRGS_Msk | ADC_ADCR_TRGCOND_Msk | ADC_ADCR_ACMPTES_Msk);
    return;
}

/**
  * @brief Enable the interrupt(s) selected by u32Mask parameter.
  *
  * @param[in] adc      The pointer of the specified ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be enabled.
  *                     - \ref ADC_ADF_INT    :ADC convert complete interrupt
  *                     - \ref ADC_CMP0_INT   :ADC comparator 0 interrupt
  *                     - \ref ADC_CMP1_INT   :ADC comparator 1 interrupt
  *
  * @return None
  */
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask)
{
    if((u32Mask) & ADC_ADF_INT)
    {
        adc->ADCR |= ADC_ADCR_ADIE_Msk;
    }
    if((u32Mask) & ADC_CMP0_INT)
    {
        adc->ADCMPR[0] |= ADC_ADCMPR_CMPIE_Msk;
    }
    if((u32Mask) & ADC_CMP1_INT)
    {
        adc->ADCMPR[1] |= ADC_ADCMPR_CMPIE_Msk;
    }

    return;
}

/**
  * @brief Disable the interrupt(s) selected by u32Mask parameter.
  *
  * @param[in] adc      The pointer of the specified ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be disabled.
  *                     - \ref ADC_ADF_INT     :ADC convert complete interrupt
  *                     - \ref ADC_CMP0_INT    :ADC comparator 0 interrupt
  *                     - \ref ADC_CMP1_INT    :ADC comparator 1 interrupt
  *
  * @return None
  */
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask)
{
    if((u32Mask) & ADC_ADF_INT)
    {
        adc->ADCR &= ~ADC_ADCR_ADIE_Msk;
    }
    if((u32Mask) & ADC_CMP0_INT)
    {
        adc->ADCMPR[0] &= ~ADC_ADCMPR_CMPIE_Msk;
    }
    if((u32Mask) & ADC_CMP1_INT)
    {
        adc->ADCMPR[1] &= ~ADC_ADCMPR_CMPIE_Msk;
    }

    return;
}

/**
  * @brief Set ADC extend sample time.
  *
  * @param[in] adc                  The pointer of the specified ADC module.
  * @param[in] u32ModuleNum         Decides the sample module number, valid value are 0.
  * @param[in] u32ExtendSampleTime  Decides the extend sampling time, the range is from 0~16367 ADC clock.
  *                                 EXTSMPT will be set to 16367 if write a value more than 16367 in the register.
  *
  * @return None
  *
  * @details When A/D converting at high conversion rate, the sampling time of analog input voltage may not enough if input channel loading is heavy,
  *          user can extend A/D sampling time after trigger source is coming to get enough sampling time.
  */
void ADC_SetExtendSampleTime(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32ExtendSampleTime)
{
    adc->ESMPCTL = (adc->ESMPCTL & ~(ADC_ESMPCTL_EXTSMPT_Msk)) |
                   (u32ExtendSampleTime << ADC_ESMPCTL_EXTSMPT_Pos);
    return;
}

/**
  * @brief  Select and configure automatic operation function
  *
  * @param[in] adc          The pointer of the specified ADC module
  * @param[in] u32TrigSel   The ADC automatic operation trigger source:
  *                         - \ref ADC_AUTOCTL_TRIGSEL_SOFTWARE
  *                         - \ref ADC_AUTOCTL_TRIGSEL_TIMER0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_TIMER1
  *                         - \ref ADC_AUTOCTL_TRIGSEL_TIMER2
  *                         - \ref ADC_AUTOCTL_TRIGSEL_TIMER3
  *                         - \ref ADC_AUTOCTL_TRIGSEL_WKIOA0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_WKIOB0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_WKIOC0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_WKIOD0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_ACMP0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_ACMP1
  *
  * @return  None
  *
  * @details The function is used to set Automatic Operation relative setting.
  */
void ADC_SelectAutoOperationMode(ADC_T *adc, uint32_t u32TrigSel)
{
    /* TRIGSEL cannot be changed when TRIGEN is 1. */
    adc->AUTOCTL &= ~(ADC_AUTOCTL_TRIGEN_Msk);

    if (u32TrigSel == ADC_AUTOCTL_TRIGSEL_SOFTWARE)
    {
        adc->AUTOSTRG = (adc->AUTOSTRG & ~(ADC_AUTOSTRG_SWTRIG_Msk));
    }
    else
    {
        adc->AUTOCTL = (adc->AUTOCTL & ~(ADC_AUTOCTL_TRIGSEL_Msk)) | (u32TrigSel);
    }

    /* Automatic Operation Trigger Enable */
    adc->AUTOCTL |= (ADC_AUTOCTL_TRIGEN_Msk);

    /* Automatic Operation Mode Enable */
    adc->AUTOCTL |= (ADC_AUTOCTL_AUTOEN_Msk);

    return;
}

/**
  * @brief Configure the automatic operation mode hardware trigger condition and enable hardware trigger
  *
  * @param[in] adc          The pointer of the specified ADC module
  * @param[in] u32Source    Decides the automatic operation mode hardware trigger source. Valid values are:
  *                         - \ref ADC_AUTOCTL_TRIGSEL_TIMER0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_TIMER1
  *                         - \ref ADC_AUTOCTL_TRIGSEL_TIMER2
  *                         - \ref ADC_AUTOCTL_TRIGSEL_TIMER3
  *                         - \ref ADC_AUTOCTL_TRIGSEL_WKIOA0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_WKIOB0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_WKIOC0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_WKIOD0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_ACMP0
  *                         - \ref ADC_AUTOCTL_TRIGSEL_ACMP1
  * @param[in] u32Param     This parameter is unused.
  *
  * @return None
  *
  * @note TRIGSEL cannot be changed when TRIGEN is 1.
  */
void ADC_AutoEnableHWTrigger(ADC_T *adc,
                             uint32_t u32Source,
                             uint32_t u32Param)
{
    /* TRIGSEL cannot be changed when TRIGEN is 1. */
    adc->AUTOCTL &= ~(ADC_AUTOCTL_TRIGEN_Msk);

    adc->AUTOCTL = (adc->AUTOCTL & ~(ADC_AUTOCTL_TRIGSEL_Msk)) | (u32Source);

    /* Automatic Operation Trigger Enable */
    adc->AUTOCTL |= (ADC_AUTOCTL_TRIGEN_Msk);
    return;
}

/**
  * @brief Disable automatic operation mode hardware trigger ADC function.
  *
  * @param[in] adc The pointer of the specified ADC module
  *
  * @return None
  */
void ADC_AutoDisableHWTrigger(ADC_T *adc)
{
    /* TRIGSEL cannot be changed when TRIGEN is 1. */
    adc->AUTOCTL &= ~(ADC_AUTOCTL_TRIGEN_Msk);

    adc->AUTOCTL &= ~(ADC_AUTOCTL_TRIGSEL_Msk);
    return;
}

/**
  * @brief Enable the automatic operation mode wakeup source selected by u32Mask parameter.
  *
  * @param[in] adc      The pointer of the specified ADC module
  * @param[in] u32Mask  The combination of wakeup enable bits listed below. Each bit
  *                     corresponds to a wakeup source. This parameter decides which
  *                     wakeup source will be enabled.
  *                     - \ref ADC_AUTOCTL_ADWK     :ADC convert complete wakeup
  *                     - \ref ADC_AUTOCTL_CMP0WK   :ADC comparator 0 wakeup
  *                     - \ref ADC_AUTOCTL_CMP1WK   :ADC comparator 1 wakeup
  *
  * @return None
  */
void ADC_AutoEnableWakeup(ADC_T *adc, uint32_t u32Mask)
{
    adc->AUTOCTL |= (u32Mask);
    return;
}

/**
  * @brief Disable the automatic operation mode wakeup source selected by u32Mask parameter.
  *
  * @param[in] adc      The pointer of the specified ADC module
  * @param[in] u32Mask  The combination of wakeup enable bits listed below. Each bit
  *                     corresponds to a wakeup source. This parameter decides which
  *                     wakeup source will be disabled.
  *                     - \ref ADC_AUTOCTL_ADWK     :ADC convert complete wakeup
  *                     - \ref ADC_AUTOCTL_CMP0WK   :ADC comparator 0 wakeup
  *                     - \ref ADC_AUTOCTL_CMP1WK   :ADC comparator 1 wakeup
  *
  * @return None
  */
void ADC_AutoDisableWakeup(ADC_T *adc, uint32_t u32Mask)
{
    adc->AUTOCTL &= ~(u32Mask);
    return;
}

/*@}*/ /* end of group ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ADC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
