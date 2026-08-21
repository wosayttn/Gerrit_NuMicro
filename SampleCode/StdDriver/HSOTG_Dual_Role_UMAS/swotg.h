/***************************************************************************//**
 * @file     swotg.h
 * @version  V3.00
 * @brief    Software otg header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __SWOTG_H__
#define __SWOTG_H__

#include "NuMicro.h"

#define DEF_ADC_CC1_CHANNEL             (6)         // CC1
#define DEF_ADC_CC2_CHANNEL             (7)         // CC2
#define DEF_MOS_G_S_PIN                 PA7         // MOS Gate Switch Control Pin

#define CONFIG_CONV_INTSEL              (0)
#define CONFIG_EXT_SMPL_TIME            (0xff)
#define CONFIG_SMPL_MODULE_ACU_TIMES    (EADC_MCTL1_ACU_2)

#define SWOTG_ADC_MODULE                (EADC0)

#define SWOTG_MODE                      (1)

enum
{
    EADC_CH_0,
    EADC_CH_1,
    EADC_CH_2,
    EADC_CH_3,
    EADC_CH_4,
    EADC_CH_5,
    EADC_CH_6,
    EADC_CH_7,
    EADC_CH_8,
    EADC_CH_9,
    EADC_CH_10,
    EADC_CH_11,
    EADC_CH_12,
    EADC_CH_13,
    EADC_CH_14,
    EADC_CH_15,
    EADC_CH_VBG,        // 16, Band-gap voltage
    EADC_CH_VTEMP,      // 17, Internal Temperature sensor
    EADC_CH_AVDD_DIV4,  // 18, AVDD/4
    EADC_CH_NUM
};

#define CONFIG_MAX_CHN_NUM  EADC_CH_NUM

typedef enum
{
    evUSB_ROLE_DEVICE = 0,
    evUSB_ROLE_HOST,
    evUSB_ROLE_NONE
} E_USB_ROLE;

static const uint16_t s_swotg_ccx_threshold[evUSB_ROLE_NONE][2] =
{
    /* Min / Max CC1+CC2  */
    {3200,   3800},  /* evUSB_ROLE_DEVICE */
    {400,    600},   /* evUSB_ROLE_HOST */
};

void swotg_init(EADC_T *eadc);
void swotg_worker(EADC_T *eadc);

#endif /* __SWOTG_H__ */
