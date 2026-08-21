/**************************************************************************//**
 * @file     adc.c
 * @version  V1.00
 * @brief    M2003J Series ADC Driver Source File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
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
  * @param[in] adc          The pointer of the specified ADC module
  * @param[in] u32InputMode Decides the ADC analog input mode. M2003J don't support it.
  * @param[in] u32OpMode    Decides the ADC operation mode. Valid values are:
  *                         - \ref ADC_ADCR_ADMD_SINGLE         :Single mode.
  *                         - \ref ADC_ADCR_ADMD_BURST          :Burst mode.
  *                         - \ref ADC_ADCR_ADMD_SINGLE_CYCLE   :Single cycle scan mode.
  *                         - \ref ADC_ADCR_ADMD_CONTINUOUS     :Continuous scan mode.
  * @param[in] u32ChMask Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1..., bit 29 is channel 29.
  * @return  None
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
    adc->ADCHER = (adc->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (u32ChMask);
    return;
}


/**
  * @brief Disable ADC module
  * @param[in] adc The pointer of the specified ADC module
  * @return None
  */
void ADC_Close(ADC_T *adc)
{
    SYS->ADCRST |= SYS_ADCRST_ADC0RST_Msk;
    SYS->ADCRST &= ~SYS_ADCRST_ADC0RST_Msk;
    return;
}


/**
  * @brief Configure the hardware trigger condition and enable hardware trigger
  * @param[in] adc        The pointer of the specified ADC module
  * @param[in] u32Source  Decides the hardware trigger source. Valid values are:
  *                       - \ref ADC_ADCR_TRGS_STADC            :A/D conversion is started by external STADC pin.
  *                       - \ref ADC_ADCR_TRGS_TIMER            :A/D conversion is started by Timer.
  *                       - \ref ADC_ADCR_TRGS_BPWM             :A/D conversion is started by BPWM.
  * @param[in] u32Param   While ADC trigger by BPWM or Timer, this parameter is unused.
  *                       While ADC trigger by external pin, this parameter is used to set trigger condition.
  *                       Valid values are:
  *                       - \ref ADC_ADCR_TRGCOND_LOW_LEVEL     :STADC Low level active
  *                       - \ref ADC_ADCR_TRGCOND_HIGH_LEVEL    :STADC High level active
  *                       - \ref ADC_ADCR_TRGCOND_FALLING_EDGE  :STADC Falling edge active
  *                       - \ref ADC_ADCR_TRGCOND_RISING_EDGE   :STADC Rising edge active
  * @return None
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
        adc->ADCR = (adc->ADCR & ~(ADC_ADCR_TRGS_Msk | ADC_ADCR_TRGCOND_Msk | ADC_ADCR_TRGEN_Msk)) |
                    ((u32Source) | (u32Param) | ADC_ADCR_TRGEN_Msk);
    }
    else
    {
        adc->ADCR = (adc->ADCR & ~(ADC_ADCR_TRGS_Msk | ADC_ADCR_TRGCOND_Msk | ADC_ADCR_TRGEN_Msk)) |
                    ((u32Source) | ADC_ADCR_TRGEN_Msk);
    }
    return;
}


/**
  * @brief Disable hardware trigger ADC function.
  * @param[in] adc The pointer of the specified ADC module
  * @return None
  */
void ADC_DisableHWTrigger(ADC_T *adc)
{
    /* Software should clear TRGEN bit and ADST bit before changing TRGS bits. */
    adc->ADCR &= ~(ADC_ADCR_TRGEN_Msk | ADC_ADCR_ADST_Msk);

    adc->ADCR &= ~(ADC_ADCR_TRGS_Msk | ADC_ADCR_TRGCOND_Msk | ADC_ADCR_TRGEN_Msk);
    return;
}


/**
  * @brief Enable the interrupt(s) selected by u32Mask parameter.
  * @param[in] adc      The pointer of the specified ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be enabled.
  *                     - \ref ADC_ADF_INT    :ADC convert complete interrupt
  *                     - \ref ADC_CMP0_INT   :ADC comparator 0 interrupt
  *                     - \ref ADC_CMP1_INT   :ADC comparator 1 interrupt
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
        adc->ADCMPR[0] |= ADC_ADCMPR0_CMPIE_Msk;
    }
    if((u32Mask) & ADC_CMP1_INT)
    {
        adc->ADCMPR[1] |= ADC_ADCMPR1_CMPIE_Msk;
    }

    return;
}


/**
  * @brief Disable the interrupt(s) selected by u32Mask parameter.
  * @param[in] adc      The pointer of the specified ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be disabled.
  *                     - \ref ADC_ADF_INT     :ADC convert complete interrupt
  *                     - \ref ADC_CMP0_INT    :ADC comparator 0 interrupt
  *                     - \ref ADC_CMP1_INT    :ADC comparator 1 interrupt
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
        adc->ADCMPR[0] &= ~ADC_ADCMPR0_CMPIE_Msk;
    }
    if((u32Mask) & ADC_CMP1_INT)
    {
        adc->ADCMPR[1] &= ~ADC_ADCMPR1_CMPIE_Msk;
    }

    return;
}


/**
  * @brief Set ADC extend sample time.
  * @param[in] adc                  The pointer of the specified ADC module.
  * @param[in] u32ModuleNum         Decides the sample module number, valid value are 0.
  * @param[in] u32ExtendSampleTime  Decides the extend sampling time, the range is from 0 ~ 16367 (0x3FEF) ADC clock.
  * @return None
  * @details When A/D converting at high conversion rate, the sampling time of analog input voltage may not enough if input channel loading is heavy,
  *          user can extend A/D sampling time after trigger source is coming to get enough sampling time.
  */
void ADC_SetExtendSampleTime(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32ExtendSampleTime)
{
    adc->ESMPCTL = (adc->ESMPCTL & ~(ADC_ESMPCTL_EXTSMPT_Msk)) |
                   (u32ExtendSampleTime << ADC_ESMPCTL_EXTSMPT_Pos);
}


/**
  * @brief Wait ADC power on ready.
  * @param[in] adc  The pointer of the specified ADC module.
  * @return    0  ADC is not ready
  * @return    1  ADC is ready
  * @details When ADEN set to 1 to enable ADC, the ADC need a stable time. The ADCRDY will keep low until ADC stable.
  *          The ADC power ready time depends on STBSEL.
  * @note This function sets g_ADC_i32ErrCode to ADC_TIMEOUT_ERR if ADCRDY is not set to 1.
  */
uint32_t ADC_WaitPowerOnReady(ADC_T *adc)
{
    uint32_t u32DelayCount = SystemCoreClock >> 1;  /* 500ms timeout */

    g_ADC_i32ErrCode = 0;

    while((adc->ADSR0 & ADC_ADSR0_ADCRDY_Msk)==0)
    {
        u32DelayCount--;
        if(u32DelayCount == 0)
        {
            g_ADC_i32ErrCode = ADC_TIMEOUT_ERR;
            return 0;
        }
    }
    return 1;
}


/*@}*/ /* end of group ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ADC_Driver */

/*@}*/ /* end of group Standard_Driver */
