/**************************************************************************//**
 * @file     swotg.c
 * @version  V3.00
 * @brief    Software otg sample file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "swotg.h"
#include "massstorage.h"
#include "usbh_lib.h"

uint32_t gu32ChnMask = 0;
static uint32_t s_gu32BuiltInBandGapValue = 0;

static int32_t isConnectedUSBRole(const int16_t *cc_sum);
static int32_t _eadc_enabled(EADC_T *eadc, uint32_t channel, uint32_t enabled);
static int32_t _eadc_enable(EADC_T *eadc, uint32_t channel);
static int32_t _eadc_disable(EADC_T *eadc, uint32_t channel);
static uint32_t _eadc_convert(EADC_T *eadc, uint32_t channel);
static int16_t _eadc_get_vref(EADC_T *eadc);
static int32_t _eadc_get_convert(EADC_T *eadc, uint32_t channel, uint32_t *value);
int16_t _eadc_get_voltage(EADC_T *eadc, uint32_t channel);
void _eadc_init_get_VBG(EADC_T *eadc);

void swotg_init(EADC_T *eadc);
void swotg_worker(EADC_T *eadc);

void delay_us(int usec);
void USBH_Process();

static int32_t isConnectedUSBRole(const int16_t *cc_sum)
{
    /* Determine the USB role based on CC1+CC2 voltage. */
    for(E_USB_ROLE role = evUSB_ROLE_DEVICE; role < evUSB_ROLE_NONE; role++)
    {
        /* Check if CC1+CC2 voltage is within the threshold range. */
        if((cc_sum[role] >= s_swotg_ccx_threshold[role][0]) &&
           (cc_sum[role] <= s_swotg_ccx_threshold[role][1]))
        {
            /* Found the connected USB role */
            return role;
        }
    }

    /* No connected USB role */
    return evUSB_ROLE_NONE;
}

static int32_t _eadc_enabled(EADC_T *eadc, uint32_t channel, uint32_t enabled)
{
    if(channel >= CONFIG_MAX_CHN_NUM)
        return -1;

    int32_t i32Err;

    if(enabled)
    {
        if(gu32ChnMask == 0)
        {
            /* Set input mode as single-end and enable the A/D converter */
            i32Err = EADC_Open(eadc, EADC_CTL_DIFFEN_SINGLE_END);

            /* Check EADC global error code. */
            if(i32Err != 0)
            {
                if(i32Err == EADC_CAL_ERR)
                {
                    printf("EADC has calibration error.\n");
                    return i32Err;
                }
                else if(i32Err == EADC_CLKDIV_ERR)
                {
                    printf("EADC clock frequency is configured error.\n");
                    return i32Err;
                }
                else
                {
                    printf("EADC has operation error.\n");
                    return i32Err;
                }
            }
        }

        switch(channel)
        {
            case EADC_CH_AVDD_DIV4:
                /* Enable AVDD/4 */
                SYS->IVSCTL |= SYS_IVSCTL_AVDDDIV4EN_Msk;
                break;

            case EADC_CH_VBG:
                /* Force to enable internal voltage band-gap. */
                SYS_UnlockReg();
                SYS->VREFCTL |= SYS_VREFCTL_VBGFEN_Msk;
                SYS_LockReg();
                break;

            case EADC_CH_VTEMP:
                /* Enable temperature sensor */
                SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;
                break;

            default:
                break;
        }

        gu32ChnMask |= (1 << channel);
    }
    else
    {
        gu32ChnMask &= ~(0x1 << channel);

        switch(channel)
        {
            case EADC_CH_AVDD_DIV4:
                /* Disable AVDD/4 */
                SYS->IVSCTL &= ~SYS_IVSCTL_AVDDDIV4EN_Msk;
                break;

            case EADC_CH_VBG:
                /* Force to enable internal voltage band-gap. */
                SYS_UnlockReg();
                SYS->VREFCTL &= ~SYS_VREFCTL_VBGFEN_Msk;
                SYS_LockReg();
                break;

            case EADC_CH_VTEMP:
                /* Disable temperature sensor */
                SYS->IVSCTL &= ~SYS_IVSCTL_VTEMPEN_Msk;
                break;

            default:
                break;
        }

        if(gu32ChnMask == 0)
        {
            EADC_Close(eadc);
        }
    }

    return 0;
}

static int32_t _eadc_enable(EADC_T *eadc, uint32_t channel)
{
    return _eadc_enabled(eadc, channel, TRUE);
}

static int32_t _eadc_disable(EADC_T *eadc, uint32_t channel)
{
    return _eadc_enabled(eadc, channel, FALSE);
}

static uint32_t _eadc_convert(EADC_T *eadc, uint32_t channel)
{
    uint32_t u32ConvValue, u32ModuleNum;

    if(_eadc_enabled(eadc, channel, TRUE) != 0)
    {
        return 0xFFFFFFFF;
    }

    u32ModuleNum = channel;

    /* Configure the sample module for analog input channel and software trigger source. */
    EADC_ConfigSampleModule(eadc, u32ModuleNum, EADC_SOFTWARE_TRIGGER, channel);

    /* Set sample module external sampling time to 0xF */
    EADC_SetExtendSampleTime(eadc, u32ModuleNum, CONFIG_EXT_SMPL_TIME);

    /* Enable Accumulate feature */
    EADC_ENABLE_ACU(eadc, u32ModuleNum, CONFIG_SMPL_MODULE_ACU_TIMES);

    /* Enable Average feature */
    EADC_ENABLE_AVG(eadc, u32ModuleNum);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(eadc, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module interrupt. */
    EADC_ENABLE_INT(eadc, (1 << CONFIG_CONV_INTSEL));
    EADC_ENABLE_SAMPLE_MODULE_INT(eadc, CONFIG_CONV_INTSEL, (1 << u32ModuleNum));

    EADC_START_CONV(eadc, (1 << u32ModuleNum));
    while(EADC_GET_INT_FLAG(eadc, (1 << CONFIG_CONV_INTSEL)) == 0);

    /* Disable the sample module interrupt. */
    EADC_DISABLE_INT(eadc, (1 << CONFIG_CONV_INTSEL));

    /* Disable Average feature */
    EADC_DISABLE_AVG(eadc, u32ModuleNum);

    /* Disable Accumulate feature */
    EADC_DISABLE_ACU(eadc, u32ModuleNum);

    u32ConvValue = EADC_GET_CONV_DATA(eadc, u32ModuleNum);

    // printf("    >> EADC0 Channel %d Conversion Result: %d\n", channel, u32ConvValue);
    //printf("u32ConvValue: %08x\n", u32ConvValue);

    return u32ConvValue;
}

static int16_t _eadc_get_vref(EADC_T *eadc)
{
    uint32_t u32VBG;

    u32VBG = _eadc_convert(eadc, EADC_CH_VBG); // VBG Channel

    /* Calculate band-gap voltage in mV */
    return ((3072 * s_gu32BuiltInBandGapValue) / u32VBG);
}

static int32_t _eadc_get_convert(EADC_T *eadc, uint32_t channel, uint32_t *value)
{
    int32_t ret = 1;

    if(channel >= CONFIG_MAX_CHN_NUM)
    {
        *value = 0xFFFFFFFF;
        ret = -1;
        goto exit_eadc_convert;
    }

    if((gu32ChnMask & (1 << channel)) == 0)
    {
        *value = 0xFFFFFFFF;
        ret = -2;
        goto exit_eadc_convert;
    }

    *value = _eadc_convert(eadc, channel);

    ret = 0;

exit_eadc_convert:
    return ret;
}

int16_t _eadc_get_voltage(EADC_T *eadc, uint32_t channel)
{
    uint32_t value = 0;
    int16_t vref = 0, voltage = 0;

    /*get the reference voltage*/
    if((vref = _eadc_get_vref(eadc)) == 0)
    {
        goto _voltage_exit;
    }

    /*read the value and convert to voltage*/
    if(_eadc_get_convert(eadc, channel, &value) != 0)
    {
        goto _voltage_exit;
    }

    voltage = value * vref / (1 << 12); // 12-bit resolution

_voltage_exit:
    return voltage;
}

void _eadc_init_get_VBG(EADC_T *eadc)
{
    EADC_Open(eadc, EADC_CTL_DIFFEN_SINGLE_END);

    /* Configure the sample module for analog input channel and software trigger source. */
    EADC_ConfigSampleModule(eadc, EADC_CH_VBG, EADC_SOFTWARE_TRIGGER, EADC_CH_VBG);

    /* Set sample module external sampling time to 0xF */
    EADC_SetExtendSampleTime(eadc, EADC_CH_VBG, CONFIG_EXT_SMPL_TIME);

    /* Enable Accumulate feature */
    EADC_ENABLE_ACU(eadc, EADC_CH_VBG, CONFIG_SMPL_MODULE_ACU_TIMES);

    /* Enable Average feature */
    EADC_ENABLE_AVG(eadc, EADC_CH_VBG);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(eadc, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module interrupt. */
    EADC_ENABLE_INT(eadc, (1 << CONFIG_CONV_INTSEL));
    EADC_ENABLE_SAMPLE_MODULE_INT(eadc, CONFIG_CONV_INTSEL, (1 << EADC_CH_VBG));

    EADC_START_CONV(eadc, (1 << EADC_CH_VBG));
    while(EADC_GET_INT_FLAG(eadc, (1 << CONFIG_CONV_INTSEL)) == 0);

    /* Disable the sample module interrupt. */
    EADC_DISABLE_INT(eadc, (1 << CONFIG_CONV_INTSEL));

    /* Disable Average feature */
    EADC_DISABLE_AVG(eadc, EADC_CH_VBG);

    /* Disable Accumulate feature */
    EADC_DISABLE_ACU(eadc, EADC_CH_VBG);

    s_gu32BuiltInBandGapValue = EADC_GET_CONV_DATA(eadc, EADC_CH_VBG);
    // printf("    >>>>>>>>>> Built-in Band-Gap Value: %d\n", s_gu32BuiltInBandGapValue);

    EADC_Close(eadc);
}

void swotg_init(EADC_T *eadc)
{
    if(eadc != SWOTG_ADC_MODULE)
    {
        printf("Error: Invalid EADC module for SWOTG!\n");
        return;
    }
    SYS_UnlockReg();

    /* Enable EADC0 module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Set EADC0 clock divider as 12 */
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_PLL_DIV2, CLK_CLKDIV0_EADC0(12));

    /* Set multi-function pins for EADC0 channels. */
    SET_EADC0_CH6_PB6();    //!< DEF_ADC_CC1_CHANNEL
    SET_EADC0_CH7_PB7();    //!< DEF_ADC_CC2_CHANNEL

    /* Disable digital input path of EADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT6 | BIT7);

    SET_GPIO_PA7();
    GPIO_SetMode(PA, BIT7, GPIO_MODE_OUTPUT);   //!< DEF_MOS_G_S_PIN

    _eadc_init_get_VBG(eadc);

    SYS_LockReg();
}

void swotg_worker(EADC_T *eadc)
{
    int32_t i32err = 0;

    /* Enable EADC channel for CC1 and CC2 */
    i32err = _eadc_enable(eadc, DEF_ADC_CC1_CHANNEL);
    i32err |= _eadc_enable(eadc, DEF_ADC_CC2_CHANNEL);
    if(i32err)
    {
        printf("Failed to enable EADC channel for USB OTG Type-C detection!\n");
        goto fail_init;
    }

    /* Sample CC1 and CC2 voltages periodically. */
    do
    {
        int16_t i16cc_sum[2] = {0};

        /* Sample CC1 and CC2 voltages under both USB roles. */
        for(E_USB_ROLE role = evUSB_ROLE_DEVICE; role < evUSB_ROLE_NONE; role++)
        {
            int16_t i16cc1_mv, i16cc2_mv;

            /* Set MOS_G_S pin to select USB role */
            DEF_MOS_G_S_PIN = role;
            delay_us(1000);

            /* Read CC1 and CC2 voltages */
            i16cc1_mv = _eadc_get_voltage(eadc, DEF_ADC_CC1_CHANNEL); // CC1
            i16cc2_mv = _eadc_get_voltage(eadc, DEF_ADC_CC2_CHANNEL); // CC2

            /* Sum CC1 and CC2 voltages */
            i16cc_sum[role] = i16cc1_mv + i16cc2_mv;

            // printf("%d: cc1:%04d + cc2:%04d = %04d(mv)\n", role, i16cc1_mv, i16cc2_mv, i16cc_sum[role]);
        }

        /* Determine the USB role */
        E_USB_ROLE role = (E_USB_ROLE)isConnectedUSBRole(i16cc_sum);

        switch(role)
        {
            case evUSB_ROLE_DEVICE:
                /* Unlock protected registers */
                SYS_UnlockReg();
//                SYS->USBPHY &= ~(SYS_USBPHY_HSUSBEN_Msk | SYS_USBPHY_HSUSBACT_Msk);
//                SYS->USBPHY = SYS_USBPHY_HSUSBEN_Msk | (0x1 << SYS_USBPHY_HSUSBROLE_Pos) | SYS_USBPHY_SBO_Msk;
//                delay_us(20);
//                SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

                SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk)) | (0x1u << SYS_USBPHY_HSUSBROLE_Pos);
                SYS->USBPHY &= ~SYS_USBPHY_HSUSBACT_Msk;
                SYS->USBPHY |= (SYS_USBPHY_HSUSBEN_Msk);
                delay_us(20);
                SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

                /* Lock protected registers */
                SYS_LockReg();

                if(HSOTG_GET_STATUS(HSOTG_STATUS_ASHOST_Msk))   /* A-device */
                {
                    printf("A-device (HSOTG_STATUS: 0x%x)\n", HSOTG->STATUS);
                    USBH_Process();
                }

                break;

            case evUSB_ROLE_HOST:
                /* Unlock protected registers */
                SYS_UnlockReg();
//                SYS->USBPHY &= ~(SYS_USBPHY_HSUSBEN_Msk | SYS_USBPHY_HSUSBACT_Msk);
//                SYS->USBPHY = SYS_USBPHY_HSUSBEN_Msk | (0x0 << SYS_USBPHY_HSUSBROLE_Pos) | SYS_USBPHY_SBO_Msk;
//                delay_us(20);
//                SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

                SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk)) | (0x0u << SYS_USBPHY_HSUSBROLE_Pos);
                SYS->USBPHY &= ~SYS_USBPHY_HSUSBACT_Msk;
                SYS->USBPHY |= (SYS_USBPHY_HSUSBEN_Msk);
                delay_us(20);
                SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

                /* Lock protected registers */
                SYS_LockReg();

                if(HSOTG_GET_STATUS(HSOTG_STATUS_ASPERI_Msk))   /* B-device */
                {
                    printf("B-device (HSOTG_STATUS: 0x%x)\n", HSOTG->STATUS);
                    HSUSBD_Open(&gsHSInfo, MSC_ClassRequest, NULL);
                    MSC_Init();
                    NVIC_EnableIRQ(USBD20_IRQn);

//                    /* Start transaction */
//                    while(1)
//                    {
//                        if(HSUSBD_IS_ATTACHED())
//                        {
//                            HSUSBD_Start();
//                            break;
//                        }
//                    }

                    while(1)
                    {
                        if(HSOTG_GET_STATUS(HSOTG_STATUS_BVLD_Msk) == 0)
                            break;
                        if(g_hsusbd_Configured)
                            MSC_ProcessCmd();
                    }
                    printf("break-B (HSOTG_STATUS: 0x%x)\n", HSOTG->STATUS);
                }

                break;

            case evUSB_ROLE_NONE:
            default:
                /* Unlock protected registers */
                SYS_UnlockReg();
//                SYS->USBPHY &= ~(SYS_USBPHY_HSUSBEN_Msk | SYS_USBPHY_HSUSBACT_Msk);
//                SYS->USBPHY = SYS_USBPHY_HSUSBEN_Msk | (0x2 << SYS_USBPHY_HSUSBROLE_Pos) | SYS_USBPHY_SBO_Msk;
//                delay_us(20);
//                SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

//                SYS->USBPHY &= ~SYS_USBPHY_HSUSBACT_Msk;
                SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk)) | (0x2u << SYS_USBPHY_HSUSBROLE_Pos);
                SYS->USBPHY &= ~SYS_USBPHY_HSUSBACT_Msk;
                SYS->USBPHY |= (SYS_USBPHY_HSUSBEN_Msk);
                delay_us(20);
                SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

                /* Lock protected registers */
                SYS_LockReg();
                break;
        }
    }
    while(0);

    return;

fail_init:
    /* Disable EADC channels */
    _eadc_disable(eadc, DEF_ADC_CC1_CHANNEL);
    _eadc_disable(eadc, DEF_ADC_CC2_CHANNEL);
}
