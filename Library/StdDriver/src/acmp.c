/**************************************************************************//**
 * @file     acmp.c
 * @version  V1.00
 * @brief    M471 series Analog Comparator(ACMP) driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup ACMP_Driver ACMP Driver
  @{
*/


/** @addtogroup ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/


/**
  * @brief  Configure the specified ACMP module
  *
  * @param[in]  acmp The pointer of the specified ACMP module
  * @param[in]  u32ChNum Comparator number.
  * @param[in]  u32NegSrc Comparator negative input selection.  Including:
  *                  - \ref ACMP_CTL_NEGSEL_PIN
  *                  - \ref ACMP_CTL_NEGSEL_CRV
  *                  - \ref ACMP_CTL_NEGSEL_VBG
  *                  - \ref ACMP_CTL_NEGSEL_DAC
  * @param[in]  u32HysSel The hysteresis function option. Including:
  *                  - \ref ACMP_CTL_HYSTERESIS_40MV
  *                  - \ref ACMP_CTL_HYSTERESIS_20MV
  *                  - \ref ACMP_CTL_HYSTERESIS_DISABLE
  *
  * @return     None
  *
  * @details    Configure hysteresis function, select the source of negative input and enable analog comparator.
  */
void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysSel)
{
    uint32_t cal_val = 0;

    acmp->CTL[0] |= ACMP_CTL_ACMPEN_Msk;
    acmp->CTL[1] |= ACMP_CTL_ACMPEN_Msk;
    cal_val = inp32(ACMP01_BASE+0xFF0);
    if ( ((cal_val & 0x0F000000) == 0x0F000000) ||
         ((cal_val & 0x000F0000) == 0x000F0000) ||
         ((cal_val & 0x00000F00) == 0x00000F00) ||
         ((cal_val & 0x0000000F) == 0x0000000F)
       )
    {
        ACMP_Calibration(acmp);
    }

    acmp->CTL[u32ChNum] = (acmp->CTL[u32ChNum] & (~(ACMP_CTL_NEGSEL_Msk | ACMP_CTL_HYSSEL_Msk | ACMP_CTL_POSSEL_Msk))) | (u32NegSrc | u32HysSel | ACMP_CTL_POSSEL_P0 | ACMP_CTL_ACMPEN_Msk);
}

/**
  * @brief  Close analog comparator
  *
  * @param[in]  acmp The pointer of the specified ACMP module
  * @param[in]  u32ChNum Comparator number.
  *
  * @return     None
  *
  * @details  This function will clear ACMPEN bit of ACMP_CTL register to disable analog comparator.
  */
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum)
{
    acmp->CTL[u32ChNum] &= (~ACMP_CTL_ACMPEN_Msk);
}


/**
  * @brief  Calibrate the specified ACMP module
  *
  * @param[in]  acmp The pointer of the specified ACMP module
  *
  * @return     None
  *
  * @details    Calibrate the ACMP module.
  */
void ACMP_Calibration(ACMP_T *acmp)
{
    uint32_t cal_val = 0;
    uint32_t i, mask;

    /* Configure acmp. Enable acmp and select CRV voltage as the source of ACMP negative input. */
    acmp->CTL[0] = (acmp->CTL[0] & (~(ACMP_CTL_NEGSEL_Msk | ACMP_CTL_HYSSEL_Msk | ACMP_CTL_POSSEL_Msk))) |
                   (ACMP_CTL_NEGSEL_CRV | ACMP_CTL_HYSTERESIS_DISABLE | ACMP_CTL_POSSEL_P0 | ACMP_CTL_ACMPEN_Msk);
    acmp->CTL[1] = (acmp->CTL[1] & (~(ACMP_CTL_NEGSEL_Msk | ACMP_CTL_HYSSEL_Msk | ACMP_CTL_POSSEL_Msk))) |
                   (ACMP_CTL_NEGSEL_CRV | ACMP_CTL_HYSTERESIS_DISABLE | ACMP_CTL_POSSEL_P0 | ACMP_CTL_ACMPEN_Msk);

    /* Select P1 as ACMP positive input channel */
    ACMP_SELECT_P(acmp, 0, ACMP_CTL_POSSEL_P1);
    ACMP_SELECT_P(acmp, 1, ACMP_CTL_POSSEL_P1);

    /* Enable CRV */
    acmp->VREF = (ACMP_VREF_CRV1EN_Msk | (0x0 << ACMP_VREF_CRV1SEL_Pos)) |
                 (ACMP_VREF_CRV0EN_Msk | (0x0 << ACMP_VREF_CRV0SEL_Pos));
    /* Do ACMP calibration */
    acmp->CALCTL |= (ACMP_CALCTL_CALTRG0_Msk | ACMP_CALCTL_CALTRG1_Msk);
    while (((acmp->CALSR) & ACMP_CALSR_DONE0_Msk) == 0);
    while (((acmp->CALSR) & ACMP_CALSR_DONE1_Msk) == 0);
    cal_val |= (inp32(ACMP01_BASE+0xFF0) & 0x00FF00FF);

    /* If user must trigger calibration twice or more times,
       the second trigger has to wait at least 300us after
       the previous calibration is done. */
    CLK_SysTickDelay(300);

    /* Disable CRV */
    acmp->VREF = 0;
    /* Do ACMP calibration */
    acmp->CALCTL |= (ACMP_CALCTL_CALTRG0_Msk | ACMP_CALCTL_CALTRG1_Msk);
    while (((acmp->CALSR) & ACMP_CALSR_DONE0_Msk) == 0);
    while (((acmp->CALSR) & ACMP_CALSR_DONE1_Msk) == 0);
    cal_val |= (inp32(ACMP01_BASE+0xFF0) & 0xFF00FF00);

    /* Replace 0xF with 0x0 since 0xF is invalid calibration value. */
    for (i=0; i<8; i++)
    {
        mask = 0xF << (4*i);
        if (((cal_val & mask) >> (4*i)) == 0xF)
        {
            cal_val = (cal_val & ~mask);
        }
    }

    outp32(ACMP01_BASE+0xFF0, cal_val);
    return;
}

/*@}*/ /* end of group ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ACMP_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
