/**************************************************************************//**
 * @file     opa.h
 * @version  V3.00
 * @brief    M2L31 series OPA driver header file
 *f
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __OPA_H__
#define __OPA_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup OPA_Driver OPA Driver
  @{
*/

/** @addtogroup OPA_EXPORTED_CONSTANTS OPA Exported Constants
  @{
*/
#define OPA_MODE_POSCHEN_DISABLE        (0x00UL << OPA_MODE_POSCHEN_Pos)
#define OPA_MODE_POSCHEN_P0             (0x01UL << OPA_MODE_POSCHEN_Pos)
#define OPA_MODE_POSCHEN_P1             (0x02UL << OPA_MODE_POSCHEN_Pos)
#define OPA_MODE_POSCHEN_DAC0_OUT       (0x04UL << OPA_MODE_POSCHEN_Pos)
#define OPA_MODE_POSCHEN_DAC1_OUT       (0x08UL << OPA_MODE_POSCHEN_Pos)
#define OPA_MODE_POSCHEN_OPA1_INT_OUT   (0x10UL << OPA_MODE_POSCHEN_Pos)
#define OPA_MODE_POSCHEN_OPA0_INT_OUT   (0x10UL << OPA_MODE_POSCHEN_Pos)

#define OPA_MODE_NEGCHEN_DISABLE        (0x00UL << OPA_MODE_NEGCHEN_Pos)
#define OPA_MODE_NEGCHEN_N0             (0x01UL << OPA_MODE_NEGCHEN_Pos)
#define OPA_MODE_NEGCHEN_N1             (0x02UL << OPA_MODE_NEGCHEN_Pos)
#define OPA_MODE_NEGCHEN_OPA1_INT_OUT   (0x04UL << OPA_MODE_NEGCHEN_Pos)
#define OPA_MODE_NEGCHEN_OPA0_INT_OUT   (0x04UL << OPA_MODE_NEGCHEN_Pos)
#define OPA_MODE_NEGCHEN_VF_INT         (0x08UL << OPA_MODE_NEGCHEN_Pos)

#define OPA_MODE_SWSEL_N0               (0x0UL << OPA_MODE_SWSEL_Pos)
#define OPA_MODE_SWSEL_AVSS             (0x1UL << OPA_MODE_SWSEL_Pos)
#define OPA_MODE_SWSEL_OPA1_INT_OUT     (0x2UL << OPA_MODE_SWSEL_Pos)
#define OPA_MODE_SWSEL_OPA0_INT_OUT     (0x2UL << OPA_MODE_SWSEL_Pos)

#define OPA_CALIBRATION_CLK_1K          (0UL)   /*!< OPA calibration clock select 1 KHz  \hideinitializer */
#define OPA_CALIBRATION_RV_1_2_AVDD     (0UL)   /*!< OPA calibration reference voltage select 1/2 AVDD  \hideinitializer */
#define OPA_CALIBRATION_RV_H_L_VCM      (1UL)   /*!< OPA calibration reference voltage select from high vcm to low vcm \hideinitializer */


/* Definitions for calibration return codes and timeout. */
#define OPA_CAL_RET_OK              (0L)   /* Calibration passed */
#define OPA_CAL_RET_PMOS_FAIL       (-1L)  /* PMOS calibration failed */
#define OPA_CAL_RET_NMOS_FAIL       (-2L)  /* NMOS calibration failed */
#define OPA_CAL_RET_BOTH_FAIL       (-3L)  /* PMOS & NMOS calibration failed */
#define OPA_CAL_RET_NULL_PTR        (-4L)  /* NULL pointer error */
#define OPA_CAL_RET_TIMEOUT         (-5L)  /* Timeout waiting for calibration done */
#define OPA_CAL_RET_PARAM_INVALID   (-6L)  /* Invalid parameter */

/* [SEC] Loop-based timeout to avoid an infinite busy-wait.
   Adjust this value based on the worst-case calibration latency and CPU speed. */
#define OPA_CAL_TIMEOUT_LOOP_COUNT  (1000000UL) 

/* [SAST] The strictest constraint is 2. */
#define OPA_OPA_NUM_SHIFT_MAX       (2U)

typedef enum {
    OPA_OPEN_LOOP = 0,
    OPA_GAIN_1,
    OPA_GAIN_2,
    OPA_GAIN_4,
    OPA_GAIN_8,
    OPA_GAIN_16,
    OPA_GAIN_32,
    OPA_GAIN_RESERVED
} E_NONINVERTING_GAIN;

typedef enum {
    OPA_GAIN_MINUS_1 = 2,
    OPA_GAIN_MINUS_3,
    OPA_GAIN_MINUS_7,
    OPA_GAIN_MINUS_15,
    OPA_GAIN_MINUS_31,
    OPA_GAIN_MINUS_RESERVED
} E_INVERTING_GAIN;

/*@}*/ /* end of group OPA_EXPORTED_CONSTANTS */

/** @addtogroup OPA_EXPORTED_FUNCTIONS OPA Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define OPA functions prototype                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static inline int32_t OPA_Calibration(OPA_T *opa, uint32_t u32OpaNum, uint32_t u32ClockSel, uint32_t u32RefVol);

/**
  * @brief This macro is used to power on the OPA circuit
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return None
  * @details This macro will set OPx_EN (x=0,1,2) bit of OPACR register to power on the OPA circuit.
  * @note Remember to enable HIRC clock while power on the OPA circuit.
  * \hideinitializer
  */
#define OPA_POWER_ON(opa, u32OpaNum) ((opa)->CTL |= (1UL<<(OPA_CTL_OPEN0_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to power down the OPA circuit
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return None
  * @details This macro will clear OPx_EN (x=0,1,2) bit of OPACR register to power down the OPA circuit.
  * \hideinitializer
  */
#define OPA_POWER_DOWN(opa, u32OpaNum) ((opa)->CTL &= ~(1UL<<(OPA_CTL_OPEN0_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to enable the OPA Schmitt trigger buffer
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return None
  * @details This macro will set OPSCHx_EN (x=0,1,2) bit of OPACR register to enable the OPA Schmitt trigger buffer.
  * \hideinitializer
  */
#define OPA_ENABLE_SCH_TRIGGER(opa, u32OpaNum) ((opa)->CTL |= (1UL<<(OPA_CTL_OPDOEN0_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to disable the OPA Schmitt trigger buffer
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return None
  * @details This macro will clear OPSCHx_EN (x=0,1,2) bit of OPACR register to disable the OPA Schmitt trigger buffer.
  * \hideinitializer
  */
#define OPA_DISABLE_SCH_TRIGGER(opa, u32OpaNum) ((opa)->CTL &= ~(1UL<<(OPA_CTL_OPDOEN0_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to enable OPA Schmitt trigger digital output interrupt
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return None
  * @details This macro will set OPDIEx (x=0,1,2) bit of OPACR register to enable the OPA Schmitt trigger digital output interrupt.
  * \hideinitializer
  */
#define OPA_ENABLE_INT(opa, u32OpaNum) ((opa)->CTL |= (1UL<<(OPA_CTL_OPDOIEN0_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to disable OPA Schmitt trigger digital output interrupt
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return None
  * @details This macro will clear OPDIEx (x=0,1,2) bit of OPACR register to disable the OPA Schmitt trigger digital output interrupt.
  * \hideinitializer
  */
#define OPA_DISABLE_INT(opa, u32OpaNum) ((opa)->CTL &= ~(1UL<<(OPA_CTL_OPDOIEN0_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to enable OPA output
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return None
  * @details This macro will set OUTOE bit of OPA_MODE register to enable the OPA output.
  * \hideinitializer
  */
#define OPA_ENABLE_OUTPUT(opa, u32OpaNum) *((__IO uint32_t *) (&((opa)->MODE0) + (u32OpaNum))) |= OPA_MODE_OUTOE_Msk

/**
  * @brief This macro is used to enable OPA output
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return None
  * @details This macro will clear OUTOE bit of OPA_MODE register to disable the OPA output.
  * \hideinitializer
  */
#define OPA_DISABLE_OUTPUT(opa, u32OpaNum) *((__IO uint32_t *) (&((opa)->MODE0) + (u32OpaNum))) &= (~OPA_MODE_OUTOE_Msk)

/**
  * @brief This macro is used to get OPA digital output state
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return  OPA digital output state
  * @details This macro will return the OPA digital output value.
  * \hideinitializer
  */
#define OPA_GET_DIGITAL_OUTPUT(opa, u32OpaNum) (((opa)->STATUS & (OPA_STATUS_OPDO0_Msk<<(u32OpaNum)))?1UL:0UL)

/**
  * @brief This macro is used to set OPA gain
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @param[in] opa_gain The gain setting. 3'b000 ~ 3'b110.
  * @return None
  * @details This macro will set OPA gain.
  * \hideinitializer
  */
#define OPA_SET_GAIN(opa, u32OpaNum, opa_gain) *((__IO uint32_t *) (&((opa)->MODE0) + (u32OpaNum))) = (*((__IO uint32_t *) (&((opa)->MODE0) + (u32OpaNum)))&(~OPA_MODE_GAIN_Msk)) | (opa_gain << OPA_MODE_GAIN_Pos)

/**
  * @brief This macro is used to get OPA interrupt flag
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @retval     0 OPA interrupt does not occur.
  * @retval     1 OPA interrupt occurs.
  * @details This macro will return the OPA interrupt flag.
  * \hideinitializer
  */
#define OPA_GET_INT_FLAG(opa, u32OpaNum) (((opa)->STATUS & (OPA_STATUS_OPDOIF0_Msk<<(u32OpaNum)))?1UL:0UL)

/**
  * @brief This macro is used to clear OPA interrupt flag
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @return   None
  * @details This macro will write 1 to OPDFx (x=0,1,2) bit of OPASR register to clear interrupt flag.
  * \hideinitializer
  */
#define OPA_CLR_INT_FLAG(opa, u32OpaNum) ((opa)->STATUS = (OPA_STATUS_OPDOIF0_Msk<<(u32OpaNum)))


/**
  * @brief This function is used to configure and start OPA calibration
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1; 2 for OPA2.
  * @param[in] u32ClockSel Select OPA calibration clock
  *                 - \ref OPA_CALIBRATION_CLK_1K
  * @param[in] u32RefVol Select OPA reference voltage
  *                 - \ref OPA_CALIBRATION_RV_1_2_AVDD
  *                 - \ref OPA_CALIBRATION_RV_H_L_VCM
  * @retval      0 PMOS and NMOS calibration successfully.
  * @retval     -1 only PMOS calibration failed.
  * @retval     -2 only NMOS calibration failed.
  * @retval     -3 PMOS and NMOS calibration failed.
  */
static inline int32_t OPA_Calibration(OPA_T *opa,
                                        uint32_t u32OpaNum,
                                        uint32_t u32ClockSel,
                                        uint32_t u32RefVol)
{
    uint32_t u32CALResult;
    uint32_t u32TrigMask;
    uint32_t u32RefMask;
    int32_t  i32Ret = OPA_CAL_RET_OK;

    (void)u32ClockSel;

    /* [SEC] Basic input validation to avoid NULL dereference. */
    if (opa == (OPA_T *)NULL)
    {
        return OPA_CAL_RET_NULL_PTR;
    }

    /* [SAST] Constrain channel index so that all shift amounts are provably < 32. */
    if (u32OpaNum >= OPA_OPA_NUM_SHIFT_MAX)
    {
        return OPA_CAL_RET_PARAM_INVALID; 
    }

    /* Compute per-channel masks explicitly. */
    u32TrigMask = (uint32_t)(OPA_CALCTL_CALTRG0_Msk << u32OpaNum);
    u32RefMask  = (uint32_t)(OPA_CALCTL_CALRVS0_Msk << u32OpaNum);

    /* Clear calibration clock selection (even if unused). */
    opa->CALCTL &=
        (uint32_t)~(uint32_t)(OPA_CALCTL_CALCLK0_Msk << (u32OpaNum << 1UL));


    /* Set reference voltage selection. */
    opa->CALCTL =
        (opa->CALCTL & (uint32_t)~u32RefMask) |
        (uint32_t)(((uint32_t)u32RefVol << OPA_CALCTL_CALRVS0_Pos) << u32OpaNum);


    /* Trigger calibration */
    opa->CALCTL |= u32TrigMask;

    /* Wait calibration complete */
    {
        /* [SEC] Add a timeout to avoid an infinite loop if hardware/clock is abnormal. */
        uint32_t u32Timeout = OPA_CAL_TIMEOUT_LOOP_COUNT;

        while (((opa->CALCTL & u32TrigMask) != 0UL) && (u32Timeout > 0UL))
        {
            u32Timeout--;
        }

        if (u32Timeout == 0UL)
        {
            return OPA_CAL_RET_TIMEOUT;
        }
    }

    /* extract result bits cleanly */
    u32CALResult =
        (opa->CALST >> (u32OpaNum * 4UL)) &
        (OPA_CALST_CALNS0_Msk | OPA_CALST_CALPS0_Msk);

    /* [SAST] Use bit-based decoding to avoid duplicate-branch warnings and reduce dependence
       on macro values when configuration is incomplete (cppcheck unknown macros). */
    {
        uint32_t u32NmosFail;
        uint32_t u32PmosFail;
        uint32_t u32FailBits;
        u32NmosFail = (u32CALResult & (uint32_t)OPA_CALST_CALNS0_Msk);
        u32PmosFail = (u32CALResult & (uint32_t)OPA_CALST_CALPS0_Msk);

        /* Normalize to 1-bit flags. */
        u32FailBits = 0UL;
        if (u32NmosFail != 0UL) { u32FailBits |= 1UL; }
        if (u32PmosFail != 0UL) { u32FailBits |= 2UL; }
       
        /* Decode failure combinations. */
        if (u32FailBits == 0UL)
        {
            i32Ret = OPA_CAL_RET_OK;
        }
        else if (u32FailBits == 1UL)
        {
            i32Ret = OPA_CAL_RET_NMOS_FAIL;
        }
        else if (u32FailBits == 2UL)
        {
            i32Ret = OPA_CAL_RET_PMOS_FAIL;
        }
        else
        {
            i32Ret = OPA_CAL_RET_BOTH_FAIL;
        }
    }
    return i32Ret;
}

/**
  * @brief This macro is used to generate asynchronous reset signals to OPA controller
  * @param    None
  * @return   None
  * \hideinitializer
  */
#define OPA_Reset() \
do { \
    LPSCC->IPRST0 |= (LPSCC_IPRST0_OPARST_Msk); \
    LPSCC->IPRST0 &= ~(LPSCC_IPRST0_OPARST_Msk); \
} while(0)


/*@}*/ /* end of group OPA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group OPA_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __OPA_H__ */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
