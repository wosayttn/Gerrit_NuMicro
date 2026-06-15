/**************************************************************************//**
 * @file     psio.h
 * @version  V1.00
 * @brief    PSIO driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __PSIO_H__
#define __PSIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup PSIO_Driver PSIO Driver
  @{
*/

/** @addtogroup PSIO_EXPORTED_CONSTANTS PSIO Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Operation Mode Constant Definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PSIO_SC0                0x00000000UL            /*!<PSIO Slot Controller 0  \hideinitializer */
#define PSIO_SC1                0x00000001UL            /*!<PSIO Slot Controller 1  \hideinitializer */
#define PSIO_SC2                0x00000002UL            /*!<PSIO Slot Controller 2  \hideinitializer */
#define PSIO_SC3                0x00000003UL            /*!<PSIO Slot Controller 3  \hideinitializer */

#define PSIO_INT0               0x00000000UL            /*!<PSIO Interrupt 0  \hideinitializer */
#define PSIO_INT1               0x00000001UL            /*!<PSIO Interrupt 1  \hideinitializer */

#define PSIO_SLOT_DISABLE       0x00000000UL            /*!<PSIO Slot disable  \hideinitializer */
#define PSIO_SLOT0              0x00000001UL            /*!<PSIO Slot 0  \hideinitializer */
#define PSIO_SLOT1              0x00000002UL            /*!<PSIO Slot 1  \hideinitializer */
#define PSIO_SLOT2              0x00000003UL            /*!<PSIO Slot 2  \hideinitializer */
#define PSIO_SLOT3              0x00000004UL            /*!<PSIO Slot 3  \hideinitializer */
#define PSIO_SLOT4              0x00000005UL            /*!<PSIO Slot 4  \hideinitializer */
#define PSIO_SLOT5              0x00000006UL            /*!<PSIO Slot 5  \hideinitializer */
#define PSIO_SLOT6              0x00000007UL            /*!<PSIO Slot 6  \hideinitializer */
#define PSIO_SLOT7              0x00000008UL            /*!<PSIO Slot 7  \hideinitializer */

#define PSIO_PIN0               0x00000000UL            /*!<PSIO Pin 0  \hideinitializer */
#define PSIO_PIN1               0x00000001UL            /*!<PSIO Pin 1  \hideinitializer */
#define PSIO_PIN2               0x00000002UL            /*!<PSIO Pin 2  \hideinitializer */
#define PSIO_PIN3               0x00000003UL            /*!<PSIO Pin 3  \hideinitializer */
#define PSIO_PIN4               0x00000004UL            /*!<PSIO Pin 4  \hideinitializer */
#define PSIO_PIN5               0x00000005UL            /*!<PSIO Pin 5  \hideinitializer */
#define PSIO_PIN6               0x00000006UL            /*!<PSIO Pin 6  \hideinitializer */
#define PSIO_PIN7               0x00000007UL            /*!<PSIO Pin 7  \hideinitializer */

#define PSIO_CP0                0x00000000UL            /*!<PSIO Check Point 0  \hideinitializer */
#define PSIO_CP1                0x00000001UL            /*!<PSIO Check Point 1  \hideinitializer */
#define PSIO_CP2                0x00000002UL            /*!<PSIO Check Point 2  \hideinitializer */
#define PSIO_CP3                0x00000003UL            /*!<PSIO Check Point 3  \hideinitializer */
#define PSIO_CP4                0x00000004UL            /*!<PSIO Check Point 4  \hideinitializer */
#define PSIO_CP5                0x00000005UL            /*!<PSIO Check Point 5  \hideinitializer */
#define PSIO_CP6                0x00000006UL            /*!<PSIO Check Point 6  \hideinitializer */
#define PSIO_CP7                0x00000007UL            /*!<PSIO Check Point 7  \hideinitializer */

#define PSIO_PIN_ENABLE         0x00000001UL            /*!<PSIO Pin Enable   \hideinitializer */
#define PSIO_PIN_DISABLE        0x00000000UL            /*!<PSIO Pin Disable  \hideinitializer */

#define PSIO_SW_TRIGGER         ((uint32_t)((uint32_t)0x0UL << PSIO_SCCT_SCCTL_TRIGSRC_Pos))            /*!<PSIO Software Trigger        \hideinitializer */
#define PSIO_FALLING_TRIGGER    ((uint32_t)((uint32_t)0x1UL << PSIO_SCCT_SCCTL_TRIGSRC_Pos))            /*!<PSIO Falling Edge Trigger    \hideinitializer */
#define PSIO_RISING_TRIGGER     ((uint32_t)((uint32_t)0x2UL << PSIO_SCCT_SCCTL_TRIGSRC_Pos))            /*!<PSIO Rising Edge Trigger     \hideinitializer */
#define PSIO_BOTH_EDGE_TRIGGER  ((uint32_t)((uint32_t)0x3UL << PSIO_SCCT_SCCTL_TRIGSRC_Pos))            /*!<PSIO Both Edge Trigger       \hideinitializer */

#define PSIO_REPEAT_ENABLE      0x00000001UL            /*!<PSIO Repeat Mode Enable     \hideinitializer */
#define PSIO_REPEAT_DISABLE     0x00000000UL            /*!<PSIO Repeat Mode Disable    \hideinitializer */

#define PSIO_INPUT_MODE         0x00000000UL            /*!<PSIO Input Mode             \hideinitializer */
#define PSIO_OUTPUT_MODE        0x00000001UL            /*!<PSIO Output Mode            \hideinitializer */
#define PSIO_OPENDRAIN_MODE     0x00000002UL            /*!<PSIO Open-drain Mode        \hideinitializer */
#define PSIO_QUASI_MODE         0x00000003UL            /*!<PSIO Quasi Mode             \hideinitializer */

#define PSIO_LOW_LEVEL          0x00000000UL            /*!<PSIO Low Level              \hideinitializer */
#define PSIO_HIGH_LEVEL         0x00000001UL            /*!<PSIO High Level             \hideinitializer */
#define PSIO_LAST_OUTPUT        0x00000002UL            /*!<PSIO Last Output            \hideinitializer */
#define PSIO_Toggle             0x00000003UL            /*!<PSIO Toggle                 \hideinitializer */

#define PSIO_SWITCH_P0          0x00000000UL            /*!<PSIO Switch Point 0         \hideinitializer */
#define PSIO_SWITCH_P1          0x00000001UL            /*!<PSIO Switch Point 1         \hideinitializer */

#define PSIO_LSB                (0x0UL << PSIO_GNCT_DATCTL_ORDER_Pos)             /*!<PSIO Data LSB               \hideinitializer */
#define PSIO_MSB                (0x1UL << PSIO_GNCT_DATCTL_ORDER_Pos)             /*!<PSIO Data MSB               \hideinitializer */

#define PSIO_DEPTH1             0x00000000UL            /*!<PSIO Data Depth 1       \hideinitializer */
#define PSIO_DEPTH2             0x00000001UL            /*!<PSIO Data Depth 2       \hideinitializer */
#define PSIO_DEPTH3             0x00000002UL            /*!<PSIO Data Depth 3       \hideinitializer */
#define PSIO_DEPTH4             0x00000003UL            /*!<PSIO Data Depth 4       \hideinitializer */

#define PSIO_NO_ACTION          0x00000000UL            /*!<PSIO no action                  \hideinitializer */
#define PSIO_OUT_LOW            0x00000000UL            /*!<PSIO Output Low                 \hideinitializer */
#define PSIO_OUT_HIGH           0x00000001UL            /*!<PSIO Output High                \hideinitializer */
#define PSIO_OUT_BUFFER         0x00000002UL            /*!<PSIO Output form Buffer         \hideinitializer */
#define PSIO_OUT_TOGGLE         0x00000003UL            /*!<PSIO Output Toggle              \hideinitializer */
#define PSIO_IN_BUFFER          0x00000004UL            /*!<PSIO Input to Buffer            \hideinitializer */
#define PSIO_IN_STATUS          0x00000005UL            /*!<PSIO Input to Status            \hideinitializer */
#define PSIO_IN_STATUS_UPDATE   0x00000006UL            /*!<PSIO Input to Status and Update \hideinitializer */

/** @} end of group PSIO_EXPORTED_CONSTANTS */

/** @addtogroup PSIO_EXPORTED_STRUCTS PSIO Exported Structs
  @{
*/
/**
 * @brief PSIO check point configuration.
 *
 * CKPTxSLT valid values:
 * - PSIO_SLOT_DISABLE
 * - PSIO_SLOT0 ~ PSIO_SLOT7
 *
 * CKPTxACT valid values:
 * - PSIO_NO_ACTION
 * - PSIO_OUT_LOW
 * - PSIO_OUT_HIGH
 * - PSIO_OUT_BUFFER
 * - PSIO_OUT_TOGGLE
 * - PSIO_IN_BUFFER
 * - PSIO_IN_STATUS
 * - PSIO_IN_STATUS_UPDATE
 */
typedef struct
{
    unsigned CKPT0SLT : 4;   ///< Check point 0 slot selection.
    unsigned CKPT1SLT : 4;   ///< Same as CKPT0SLT.
    unsigned CKPT2SLT : 4;   ///< Same as CKPT0SLT.
    unsigned CKPT3SLT : 4;   ///< Same as CKPT0SLT.
    unsigned CKPT4SLT : 4;   ///< Same as CKPT0SLT.
    unsigned CKPT5SLT : 4;   ///< Same as CKPT0SLT.
    unsigned CKPT6SLT : 4;   ///< Same as CKPT0SLT.
    unsigned CKPT7SLT : 4;   ///< Same as CKPT0SLT.

    unsigned CKPT0ACT : 4;   ///< Check point 0 action.
    unsigned CKPT1ACT : 4;   ///< Same as CKPT0ACT.
    unsigned CKPT2ACT : 4;   ///< Same as CKPT0ACT.
    unsigned CKPT3ACT : 4;   ///< Same as CKPT0ACT.
    unsigned CKPT4ACT : 4;   ///< Same as CKPT0ACT.
    unsigned CKPT5ACT : 4;   ///< Same as CKPT0ACT.
    unsigned CKPT6ACT : 4;   ///< Same as CKPT0ACT.
    unsigned CKPT7ACT : 4;   ///< Same as CKPT0ACT.
} S_PSIO_CP_CONFIG;

#define PSIO_PDMACTL_OPINEN_Msk    (PSIO_PDMACTL_OPIN0EN_Msk | PSIO_PDMACTL_OPIN1EN_Msk | PSIO_PDMACTL_OPIN2EN_Msk | PSIO_PDMACTL_OPIN3EN_Msk | \
                                    PSIO_PDMACTL_OPIN4EN_Msk | PSIO_PDMACTL_OPIN5EN_Msk | PSIO_PDMACTL_OPIN6EN_Msk | PSIO_PDMACTL_OPIN7EN_Msk)
#define PSIO_PDMACTL_IPINEN_Msk    (PSIO_PDMACTL_IPIN0EN_Msk | PSIO_PDMACTL_IPIN1EN_Msk | PSIO_PDMACTL_IPIN2EN_Msk | PSIO_PDMACTL_IPIN3EN_Msk | \
                                    PSIO_PDMACTL_IPIN4EN_Msk | PSIO_PDMACTL_IPIN5EN_Msk | PSIO_PDMACTL_IPIN6EN_Msk | PSIO_PDMACTL_IPIN7EN_Msk)

#define PSIO_IS_VALID_SC(u32SC)                 ((uint32_t)(u32SC) <= PSIO_SC3)
#define PSIO_IS_VALID_INT(u32Int)               (((uint32_t)(u32Int) == PSIO_INT0) || ((uint32_t)(u32Int) == PSIO_INT1))
#define PSIO_IS_VALID_SLOT(u32Slot)             ((uint32_t)(u32Slot) <= PSIO_SLOT7)
#define PSIO_IS_VALID_CP_SLOT(u32Slot)          ((uint32_t)(u32Slot) <= PSIO_SLOT7)
#define PSIO_IS_VALID_PIN(u32Pin)               ((uint32_t)(u32Pin) <= PSIO_PIN7)
#define PSIO_IS_VALID_CP(u32CheckPoint)         ((uint32_t)(u32CheckPoint) <= PSIO_CP7)
#define PSIO_IS_VALID_SWITCH_POINT(u32SwPoint)  ((uint32_t)(u32SwPoint) <= PSIO_SWITCH_P1)
#define PSIO_IS_VALID_IO_MODE(u32IOMode)        ((uint32_t)(u32IOMode) <= PSIO_QUASI_MODE)
#define PSIO_IS_VALID_PIN_LEVEL(u32Level)       ((uint32_t)(u32Level) <= PSIO_Toggle)
#define PSIO_IS_VALID_PIN_ENABLE(u32PinEn)      (((uint32_t)(u32PinEn) == PSIO_PIN_ENABLE) || ((uint32_t)(u32PinEn) == PSIO_PIN_DISABLE))
#define PSIO_IS_VALID_REPEAT(u32Repeat)         (((uint32_t)(u32Repeat) == PSIO_REPEAT_ENABLE) || ((uint32_t)(u32Repeat) == PSIO_REPEAT_DISABLE))
#define PSIO_IS_VALID_CP_ACTION(u32Action)      ((uint32_t)(u32Action) <= PSIO_IN_STATUS_UPDATE)

/** @} end of group PSIO_EXPORTED_STRUCTS */

/** @addtogroup PSIO_EXPORTED_FUNCTIONS PSIO Exported Functions
  @{
*/

/**
 * @brief       Enable specified PSIO interrupt
 *
 * @param[in]   psio        The pointer of the specified PSIO module
 * @param[in]   u32IntSel   Interrupt type select
 *                - \ref PSIO_INTEN_CON0IE_Msk
 *                - \ref PSIO_INTEN_CON1IE_Msk
 *                - \ref PSIO_INTEN_MISMATIE_Msk
 *                - \ref PSIO_INTEN_TERRIE_Msk
 *                - \ref PSIO_INTEN_SC0IE_Msk
 *                - \ref PSIO_INTEN_SC1IE_Msk
 *                - \ref PSIO_INTEN_SC2IE_Msk
 *                - \ref PSIO_INTEN_SC3IE_Msk
 *
 * @return      None
 *
 * @details     This macro enable specified PSIO interrupt.
 * \hideinitializer
 */
#define PSIO_ENABLE_INT(psio, u32IntSel)    ((psio)->INTEN |= (u32IntSel))

/**
 * @brief       Disable specified PSIO interrupt
 *
 * @param[in]   psio        The pointer of the specified PSIO module
 * @param[in]   u32IntSel  Interrupt type select
 *                - \ref PSIO_INTEN_CON0IE_Msk
 *                - \ref PSIO_INTEN_CON1IE_Msk
 *                - \ref PSIO_INTEN_MISMATIE_Msk
 *                - \ref PSIO_INTEN_TERRIE_Msk
 *                - \ref PSIO_INTEN_SC0IE_Msk
 *                - \ref PSIO_INTEN_SC1IE_Msk
 *                - \ref PSIO_INTEN_SC2IE_Msk
 *                - \ref PSIO_INTEN_SC3IE_Msk
 *
 * @return      None
 *
 * @details     This macro disable specified PSIO interrupt.
 * \hideinitializer
 */
#define PSIO_DISABLE_INT(psio, u32IntSel)    ((psio)->INTEN &= ~(u32IntSel))

/**
 * @brief       Get specified interrupt flag/status
 *
 * @param[in]   psio            The pointer of the specified PSIO module
 * @param[in]   u32IntTypeFlag  Interrupt Type Flag, Valid values are
 *                - \ref PSIO_INTSTS_CON0IF_Msk
 *                - \ref PSIO_INTSTS_CON1IF_Msk
 *                - \ref PSIO_INTSTS_MISMATIF_Msk
 *                - \ref PSIO_INTSTS_TERRIF_Msk
 *                - \ref PSIO_INTSTS_SC0IF_Msk
 *                - \ref PSIO_INTSTS_SC1IF_Msk
 *                - \ref PSIO_INTSTS_SC2IF_Msk
 *                - \ref PSIO_INTSTS_SC3IF_Msk
 *
 * @return      0 The specified interrupt is not happened.
 *              1 The specified interrupt is happened.
 *
 * @details     This macro get specified interrupt flag or interrupt indicator status.
 * \hideinitializer
 */
#define PSIO_GET_INT_FLAG(psio, u32IntTypeFlag)    (((psio)->INTSTS & (u32IntTypeFlag))?1:0)

/**
 * @brief       Clear specified interrupt flag/status
 *
 * @param[in]   psio            The pointer of the specified PSIO module
 * @param[in]   u32IntTypeFlag Interrupt Type Flag, Valid values are
 *                - \ref PSIO_INTSTS_CON0IF_Msk
 *                - \ref PSIO_INTSTS_CON1IF_Msk
 *                - \ref PSIO_INTSTS_MISMATIF_Msk
 *                - \ref PSIO_INTSTS_TERRIF_Msk
 *                - \ref PSIO_INTSTS_SC0IF_Msk
 *                - \ref PSIO_INTSTS_SC1IF_Msk
 *                - \ref PSIO_INTSTS_SC2IF_Msk
 *                - \ref PSIO_INTSTS_SC3IF_Msk
 *
 * @return      None
 *
 * @details     This macro clear specified interrupt flag or interrupt indicator status.
 * \hideinitializer
 */
#define PSIO_CLEAR_INT_FLAG(psio, u32IntTypeFlag)    ((psio)->INTSTS = (u32IntTypeFlag))

/**
 * @brief       Get specified transfer status
 *
 * @param[in]   psio        The pointer of the specified PSIO module
 * @param[in]   u32Status   Transfer status, Valid values are
 *                - \ref PSIO_TRANSTS_INFULL0_Msk
 *                - \ref PSIO_TRANSTS_INOVER0_Msk
 *                - \ref PSIO_TRANSTS_OUTEPY0_Msk
 *                - \ref PSIO_TRANSTS_OUTUF0_Msk
 *                - \ref PSIO_TRANSTS_INFULL1_Msk
 *                - \ref PSIO_TRANSTS_INOVER1_Msk
 *                - \ref PSIO_TRANSTS_OUTEPY1_Msk
 *                - \ref PSIO_TRANSTS_OUTUF1_Msk
 *                - \ref PSIO_TRANSTS_INFULL2_Msk
 *                - \ref PSIO_TRANSTS_INOVER2_Msk
 *                - \ref PSIO_TRANSTS_OUTEPY2_Msk
 *                - \ref PSIO_TRANSTS_OUTUF2_Msk
 *                - \ref PSIO_TRANSTS_INFULL3_Msk
 *                - \ref PSIO_TRANSTS_INOVER3_Msk
 *                - \ref PSIO_TRANSTS_OUTEPY3_Msk
 *                - \ref PSIO_TRANSTS_OUTUF3_Msk
 *                - \ref PSIO_TRANSTS_INFULL4_Msk
 *                - \ref PSIO_TRANSTS_INOVER4_Msk
 *                - \ref PSIO_TRANSTS_OUTEPY4_Msk
 *                - \ref PSIO_TRANSTS_OUTUF4_Msk
 *                - \ref PSIO_TRANSTS_INFULL5_Msk
 *                - \ref PSIO_TRANSTS_INOVER5_Msk
 *                - \ref PSIO_TRANSTS_OUTEPY5_Msk
 *                - \ref PSIO_TRANSTS_OUTUF5_Msk
 *                - \ref PSIO_TRANSTS_INFULL6_Msk
 *                - \ref PSIO_TRANSTS_INOVER6_Msk
 *                - \ref PSIO_TRANSTS_OUTEPY6_Msk
 *                - \ref PSIO_TRANSTS_OUTUF6_Msk
 *                - \ref PSIO_TRANSTS_INFULL7_Msk
 *                - \ref PSIO_TRANSTS_INOVER7_Msk
 *                - \ref PSIO_TRANSTS_OUTEPY7_Msk
 *                - \ref PSIO_TRANSTS_OUTUF7_Msk
 *
 * @return      0 The specified status is not happened.
 *              1 The specified status is happened.
 *
 * @details     This macro get specified transfer status.
 * \hideinitializer
 */
#define PSIO_GET_TRANSFER_STATUS(psio, u32Status)    (((psio)->TRANSTS & (u32Status))?1:0)

/**
 * @brief       Clear specified transfer status
 *
 * @param[in]   psio        The pointer of the specified PSIO module
 * @param[in]   u32Status   Transfer status, Valid values are
 *                - \ref PSIO_TRANSTS_INOVER0_Msk
 *                - \ref PSIO_TRANSTS_OUTUF0_Msk
 *                - \ref PSIO_TRANSTS_INOVER1_Msk
 *                - \ref PSIO_TRANSTS_OUTUF1_Msk
 *                - \ref PSIO_TRANSTS_INOVER2_Msk
 *                - \ref PSIO_TRANSTS_OUTUF2_Msk
 *                - \ref PSIO_TRANSTS_INOVER3_Msk
 *                - \ref PSIO_TRANSTS_OUTUF3_Msk
 *                - \ref PSIO_TRANSTS_INOVER4_Msk
 *                - \ref PSIO_TRANSTS_OUTUF4_Msk
 *                - \ref PSIO_TRANSTS_INOVER5_Msk
 *                - \ref PSIO_TRANSTS_OUTUF5_Msk
 *                - \ref PSIO_TRANSTS_INOVER6_Msk
 *                - \ref PSIO_TRANSTS_OUTUF6_Msk
 *                - \ref PSIO_TRANSTS_INOVER7_Msk
 *                - \ref PSIO_TRANSTS_OUTUF7_Msk
 *
 * @return      None
 *
 * @details     This macro clear specified transfer status.
 * \hideinitializer
 */
#define PSIO_CLEAR_TRANSFER_STATUS(psio, u32Status)    ((psio)->TRANSTS = (u32Status))

/**
 * @brief       Get specified input status state
 *
 * @param[in]   psio        The pointer of the specified PSIO module
 * @param[in]   u32Status   Transfer input status state, Valid values are
 *                - \ref PSIO_ISSTS_VALID0_Msk
 *                - \ref PSIO_ISSTS_INSTSOV0_Msk
 *                - \ref PSIO_ISSTS_VALID1_Msk
 *                - \ref PSIO_ISSTS_INSTSOV1_Msk
 *                - \ref PSIO_ISSTS_VALID2_Msk
 *                - \ref PSIO_ISSTS_INSTSOV2_Msk
 *                - \ref PSIO_ISSTS_VALID3_Msk
 *                - \ref PSIO_ISSTS_INSTSOV3_Msk
 *                - \ref PSIO_ISSTS_VALID4_Msk
 *                - \ref PSIO_ISSTS_INSTSOV4_Msk
 *                - \ref PSIO_ISSTS_VALID5_Msk
 *                - \ref PSIO_ISSTS_INSTSOV5_Msk
 *                - \ref PSIO_ISSTS_VALID6_Msk
 *                - \ref PSIO_ISSTS_INSTSOV6_Msk
 *                - \ref PSIO_ISSTS_VALID7_Msk
 *                - \ref PSIO_ISSTS_INSTSOV7_Msk
 *
 * @return      0 The specified status is not happened.
 *              1 The specified status is happened.
 *
 * @details     This macro get input status state.
 * \hideinitializer
 */
#define PSIO_GET_INPUT_STATUS_STATE(psio, u32Status)    (((psio)->ISSTS & (u32Status))?1:0)

/**
 * @brief       Clear specified input status state
 *
 * @param[in]   psio        The pointer of the specified PSIO module
 * @param[in]   u32Status   Transfer input status state, Valid values are
 *                - \ref PSIO_ISSTS_INSTSOV0_Msk
 *                - \ref PSIO_ISSTS_INSTSOV1_Msk
 *                - \ref PSIO_ISSTS_INSTSOV2_Msk
 *                - \ref PSIO_ISSTS_INSTSOV3_Msk
 *                - \ref PSIO_ISSTS_INSTSOV4_Msk
 *                - \ref PSIO_ISSTS_INSTSOV5_Msk
 *                - \ref PSIO_ISSTS_INSTSOV6_Msk
 *                - \ref PSIO_ISSTS_INSTSOV7_Msk
 *
 * @return      None
 *
 * @details     This macro clear input status state.
 * \hideinitializer
 */
#define PSIO_CLEAR_INPUT_STATUS_STATE(psio, u32Status)    ((psio)->ISSTS = (u32Status))

/**
 * @brief       Set PSIO PDMA control input
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller. Valid values are
 *                - \ref PSIO_SC0
 *                - \ref PSIO_SC1
 *                - \ref PSIO_SC2
 *                - \ref PSIO_SC3
 * @param[in]   u32InPin  The selected input pin
 *                - \ref PSIO_PDMACTL_IPIN0EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN1EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN2EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN3EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN4EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN5EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN6EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN7EN_Msk
 *
 * @return      None
 *
 * @details     This macro set PSIO input with PDMA.
 * \hideinitializer
 */
#define PSIO_SET_PDMA_INPUT(psio, u32SC, u32InPin)    ((psio)->PDMACTL = (((psio)->PDMACTL & ~PSIO_PDMACTL_INSCSEL_Msk & ~PSIO_PDMACTL_IPINEN_Msk) \
                                                                          | ((((uint32_t)(u32SC)) << PSIO_PDMACTL_INSCSEL_Pos) & PSIO_PDMACTL_INSCSEL_Msk) \
                                                                          | ((uint32_t)(u32InPin) & PSIO_PDMACTL_IPINEN_Msk)))

/**
 * @brief       Clear PSIO PDMA control input
 *
 * @param[in]   psio        The pointer of the specified PSIO module
 * @param[in]   u32InPin    The selected input pin
 *                - \ref PSIO_PDMACTL_IPIN0EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN1EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN2EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN3EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN4EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN5EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN6EN_Msk
 *                - \ref PSIO_PDMACTL_IPIN7EN_Msk
 *
 * @return      None
 *
 * @details     This macro clear PSIO input with PDMA.
 * \hideinitializer
 */
#define PSIO_CLEAR_PDMA_INPUT(psio, u32InPin)    ((psio)->PDMACTL &= ~(u32InPin))

/**
 * @brief       Set PSIO PDMA control output
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller. Valid values are
 *                - \ref PSIO_SC0
 *                - \ref PSIO_SC1
 *                - \ref PSIO_SC2
 *                - \ref PSIO_SC3
 * @param[in]   u32OutPin  The selected output pin
 *                - \ref PSIO_PDMACTL_OPIN0EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN1EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN2EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN3EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN4EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN5EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN6EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN7EN_Msk
 *
 * @return      None
 *
 * @details     This macro set PSIO output with PDMA.
 * \hideinitializer
 */
#define PSIO_SET_PDMA_OUTPUT(psio, u32SC, u32OutPin)    ((psio)->PDMACTL = (((psio)->PDMACTL & ~PSIO_PDMACTL_OUTSCSEL_Msk & ~PSIO_PDMACTL_OPINEN_Msk) \
                                                                            | ((((uint32_t)(u32SC)) << PSIO_PDMACTL_OUTSCSEL_Pos) & PSIO_PDMACTL_OUTSCSEL_Msk) \
                                                                            | ((uint32_t)(u32OutPin) & PSIO_PDMACTL_OPINEN_Msk)))

/**
 * @brief       Clear PSIO PDMA control output
 *
 * @param[in]   psio        The pointer of the specified PSIO module
 * @param[in]   u32OutPin   The selected output pin
 *                - \ref PSIO_PDMACTL_OPIN0EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN1EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN2EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN3EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN4EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN5EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN6EN_Msk
 *                - \ref PSIO_PDMACTL_OPIN7EN_Msk
 *
 * @return      None
 *
 * @details     This macro clear PSIO output with PDMA.
 * \hideinitializer
 */
#define PSIO_CLEAR_PDMA_OUTPUT(psio, u32OutPin)    ((psio)->PDMACTL &= ~(u32OutPin))

/**
 * @brief       Set slot controller trigger source
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller. Valid values are
 *                - \ref PSIO_SC0
 *                - \ref PSIO_SC1
 *                - \ref PSIO_SC2
 *                - \ref PSIO_SC3
 * @param[in]   u32SrcType  The selected trigger source type
 *                - \ref PSIO_SW_TRIGGER
 *                - \ref PSIO_FALLING_TRIGGER
 *                - \ref PSIO_RISING_TRIGGER
 *                - \ref PSIO_BOTH_EDGE_TRIGGER
 *
 * @return      None
 *
 * @details     This macro set slot controller trigger source.
 * \hideinitializer
 */
#define PSIO_SET_TRIGSRC(psio, u32SC, u32SrcType)     ((psio)->SCCT[(u32SC)].SCCTL = (((psio)->SCCT[(u32SC)].SCCTL & ~PSIO_SCCT_SCCTL_TRIGSRC_Msk) | ((uint32_t)(u32SrcType) & PSIO_SCCT_SCCTL_TRIGSRC_Msk)))

/**
 * @brief       Start PSIO slot controller
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller. Valid values are
 *                - \ref PSIO_SC0
 *                - \ref PSIO_SC1
 *                - \ref PSIO_SC2
 *                - \ref PSIO_SC3
 *
 * @return      None
 *
 * @details     This macro start PSIO slot controller.
 * \hideinitializer
 */
#define PSIO_START_SC(psio, u32SC)    ((psio)->SCCT[(u32SC)].SCCTL |= PSIO_SCCT_SCCTL_START_Msk)

/**
 * @brief       Stop PSIO slot controller
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller. Valid values are
 *                - \ref PSIO_SC0
 *                - \ref PSIO_SC1
 *                - \ref PSIO_SC2
 *                - \ref PSIO_SC3
 *
 * @return      None
 *
 * @details     This macro stop PSIO slot controller.
 * \hideinitializer
 */
#define PSIO_STOP_SC(psio, u32SC)    ((psio)->SCCT[(u32SC)].SCCTL |= PSIO_SCCT_SCCTL_STOP_Msk)

/**
 * @brief       Get PSIO busy flag
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller. Valid values are
 *                - \ref PSIO_SC0
 *                - \ref PSIO_SC1
 *                - \ref PSIO_SC2
 *                - \ref PSIO_SC3
 *
 * @return      0 The busy flag is not happened.
 *              1 The busy flag is happened.
 *
 * @details     This macro get PSIO busy flag.
 * \hideinitializer
 */
#define PSIO_GET_BUSY_FLAG(psio, u32SC)    (((psio)->SCCT[(u32SC)].SCCTL & PSIO_SCCT_SCCTL_BUSY_Msk)?1:0)

/**
 * @brief       Get PSIO idle flag
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller. Valid values are
 *                - \ref PSIO_SC0
 *                - \ref PSIO_SC1
 *                - \ref PSIO_SC2
 *                - \ref PSIO_SC3
 *
 * @return      0 The idle flag is not happened.
 *              1 The idle flag is happened.
 *
 * @details     This macro get PSIO idle flag.
 * \hideinitializer
 */
#define PSIO_GET_IDLE_FLAG(psio, u32SC)    (((psio)->SCCT[(u32SC)].SCCTL & PSIO_SCCT_SCCTL_IDLE_Msk)?1:0)

/**
* @brief       Set PSIO idle flag
*
* @param[in]   psio    The pointer of the specified PSIO module
* @param[in]   u32SC   The selected slot controller. Valid values are
*                - \ref PSIO_SC0
*                - \ref PSIO_SC1
*                - \ref PSIO_SC2
*                - \ref PSIO_SC3
*
* @return      None
*
* @details     This macro sets the IDLE flag by software.
* \hideinitializer
*/
#define PSIO_SET_IDLE_FLAG(psio, u32SC)    ((psio)->SCCT[(u32SC)].SCCTL |= PSIO_SCCT_SCCTL_IDLE_Msk)

/**
* @brief       Set PSIO slot tick count
*
* @param[in]   psio    The pointer of the specified PSIO module
* @param[in]   u32SC   The selected slot controller. Valid values are
*                - \ref PSIO_SC0
*                - \ref PSIO_SC1
*                - \ref PSIO_SC2
*                - \ref PSIO_SC3
* @param[in]   u32Slot  The selected slot. Valid values are
*                - \ref PSIO_SLOT0
*                - \ref PSIO_SLOT1
*                - \ref PSIO_SLOT2
*                - \ref PSIO_SLOT3
*                - \ref PSIO_SLOT4
*                - \ref PSIO_SLOT5
*                - \ref PSIO_SLOT6
*                - \ref PSIO_SLOT7
* @param[in]   u32Cnt  The slot tick count. Valid values are 0x0~0xF
*
* @return      None
*
* @details     This macro set PSIO slot tick count.
* \hideinitializer
*/
#define PSIO_SCSLOT_SET_SLOT(psio, u32SC, u32Slot, u32Cnt)                 \
    do {                                                                   \
        if (((u32Slot) >= PSIO_SLOT0) && ((u32Slot) <= PSIO_SLOT7))        \
        {                                                                  \
            (psio)->SCCT[(u32SC)].SCSLOT =                                 \
                                                                           ((psio)->SCCT[(u32SC)].SCSLOT &                            \
                                                                            ~(PSIO_SCCT_SCSLOT_SLOT0CNT_Msk <<                        \
                                                                              (((u32Slot) - 1U) * PSIO_SCCT_SCSLOT_SLOT1CNT_Pos))) | \
                                                                           (((u32Cnt) & 0xFUL) <<                                     \
                                                                            (((u32Slot) - 1U) * PSIO_SCCT_SCSLOT_SLOT1CNT_Pos));      \
        }                                                                  \
    } while (0)

/**
 * @brief       Set PSIO all slot tick count
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller. Valid values are
 *                - \ref PSIO_SC0
 *                - \ref PSIO_SC1
 *                - \ref PSIO_SC2
 *                - \ref PSIO_SC3
 * @param[in]   u32Cnt  The slot tick count. Valid values are 0x0~0xF
 *
 * @return      None
 *
 * @details     This macro set PSIO all slot tick count.
 * \hideinitializer
 */
#define PSIO_SCSLOT_SET_ALL_SLOT(psio, u32SC, u32Cnt) ((psio)->SCCT[(u32SC)].SCSLOT= \
                                                                                     ((u32Cnt&0xF)<<PSIO_SCCT_SCSLOT_SLOT0CNT_Pos)|((u32Cnt&0xF)<<PSIO_SCCT_SCSLOT_SLOT1CNT_Pos)\
                                                                                     |((u32Cnt&0xF)<<PSIO_SCCT_SCSLOT_SLOT2CNT_Pos)|((u32Cnt&0xF)<<PSIO_SCCT_SCSLOT_SLOT3CNT_Pos)\
                                                                                     |((u32Cnt&0xF)<<PSIO_SCCT_SCSLOT_SLOT4CNT_Pos)|((u32Cnt&0xF)<<PSIO_SCCT_SCSLOT_SLOT5CNT_Pos)\
                                                                                     |((u32Cnt&0xF)<<PSIO_SCCT_SCSLOT_SLOT6CNT_Pos)|((u32Cnt&0xF)<<PSIO_SCCT_SCSLOT_SLOT7CNT_Pos))

/**
 * @brief       Enable Pin function
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin      The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 *
 * @return      None
 *
 * @details     This function is used to Enable this Pin.
 * \hideinitializer
 */
#define PSIO_ENABLE_PIN(psio, u32Pin) ((psio)->GNCT[(u32Pin)].GENCTL = \
                                                                       ((psio)->GNCT[(u32Pin)].GENCTL & ~PSIO_GNCT_GENCTL_PINEN_Msk)|PSIO_GNCT_GENCTL_PINEN_Msk)

/**
 * @brief       Disable Pin function
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin      The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 *
 * @return      None
 *
 * @details     This function is used to disable this Pin.
 * \hideinitializer
 */
#define PSIO_DISABLE_PIN(psio, u32Pin) ((psio)->GNCT[(u32Pin)].GENCTL = \
                                                                        ((psio)->GNCT[(u32Pin)].GENCTL & ~PSIO_GNCT_GENCTL_PINEN_Msk))

/**
 * @brief       Set specified pin data width
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin  The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 * @param[in]   u32InWidth  The input data width. Valid values are 0~32
 * @param[in]   u32OutWidth The output data width. Valid values are 0~32
 *
 * @return      None
 *
 * @details     This macro set in/out data width.
 * \hideinitializer
 */
#define PSIO_SET_WIDTH(psio, u32Pin, u32InWidth, u32OutWidth) (((psio)->GNCT[(u32Pin)].DATCTL) = \
                                                                                                 (((psio)->GNCT[(u32Pin)].DATCTL & ~PSIO_GNCT_DATCTL_INDATWD_Msk & ~PSIO_GNCT_DATCTL_OUTDATWD_Msk) \
                                                                                                  | ((((uint32_t)((u32InWidth) == 0U ? 0U : ((u32InWidth) - 1U))) << PSIO_GNCT_DATCTL_INDATWD_Pos) & PSIO_GNCT_DATCTL_INDATWD_Msk) \
                                                                                                  | ((((uint32_t)((u32OutWidth) == 0U ? 0U : ((u32OutWidth) - 1U))) << PSIO_GNCT_DATCTL_OUTDATWD_Pos) & PSIO_GNCT_DATCTL_OUTDATWD_Msk)))

/**
 * @brief       Set specified pin data order
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin  The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 * @param[in]   u32Order  The data order. Valid values are
 *                - \ref PSIO_LSB
 *                - \ref PSIO_MSB
 *
 * @return      None
 *
 * @details     This macro set data order.
 * \hideinitializer
 */
#define PSIO_SET_ORDER(psio, u32Pin, u32Order) (((psio)->GNCT[(u32Pin)].DATCTL) = \
                                                                                  (((psio)->GNCT[(u32Pin)].DATCTL & ~PSIO_GNCT_DATCTL_ORDER_Msk) | ((uint32_t)(u32Order) & PSIO_GNCT_DATCTL_ORDER_Msk)))

/**
 * @brief       Set specified pin output data depth
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin  The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 * @param[in]   u32Depth  The data depth. Valid values are
 *                - \ref PSIO_DEPTH1
 *                - \ref PSIO_DEPTH2
 *                - \ref PSIO_DEPTH3
 *                - \ref PSIO_DEPTH4
 *
 * @return      None
 *
 * @details     This macro set output data order.
 * \hideinitializer
 */
#define PSIO_SET_OUTPUT_DEPTH(psio, u32Pin, u32Depth) ((psio)->GNCT[(u32Pin)].DATCTL = \
                                                                                       (((psio)->GNCT[(u32Pin)].DATCTL & ~PSIO_GNCT_DATCTL_OUTDEPTH_Msk) | ((((uint32_t)(u32Depth)) << PSIO_GNCT_DATCTL_OUTDEPTH_Pos) & PSIO_GNCT_DATCTL_OUTDEPTH_Msk)))

/**
 * @brief       Set specified pin input data depth
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin  The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 * @param[in]   u32Depth  The data depth. Valid values are
 *                - \ref PSIO_DEPTH1
 *                - \ref PSIO_DEPTH2
 *                - \ref PSIO_DEPTH3
 *                - \ref PSIO_DEPTH4
 *
 * @return      None
 *
 * @details     This macro set input data order.
 * \hideinitializer
 */
#define PSIO_SET_INPUT_DEPTH(psio, u32Pin, u32Depth) ((psio)->GNCT[(u32Pin)].DATCTL = \
                                                                                      (((psio)->GNCT[(u32Pin)].DATCTL & ~PSIO_GNCT_DATCTL_INDEPTH_Msk) | ((((uint32_t)(u32Depth)) << PSIO_GNCT_DATCTL_INDEPTH_Pos) & PSIO_GNCT_DATCTL_INDEPTH_Msk)))

/**
 * @brief       Get specified pin input status
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin  The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 *
 * @return      The specified pin input status
 *
 * @details     This macro gets and clears the specified pin input status.
 * \hideinitializer
 */
#define PSIO_GET_INPUT_STATUS(psio, u32Pin) (((psio)->GNCT[(u32Pin)].INSTS) & 0xFFUL)

/**
 * @brief       Get specified pin input data
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin  The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 *
 * @return      The specified pin input data
 *
 * @details     This macro gets and clears the specified pin input data.
 * \hideinitializer
 */
#define PSIO_GET_INPUT_DATA(psio, u32Pin) ((psio)->GNCT[(u32Pin)].INDAT)

/**
* @brief       Set specified pin output data
*
* @param[in]   psio    The pointer of the specified PSIO module
* @param[in]   u32Pin  The selected Pin. Valid values are
*                - \ref PSIO_PIN0
*                - \ref PSIO_PIN1
*                - \ref PSIO_PIN2
*                - \ref PSIO_PIN3
*                - \ref PSIO_PIN4
*                - \ref PSIO_PIN5
*                - \ref PSIO_PIN6
*                - \ref PSIO_PIN7
* @param[in]   u32Data  The output data
*
* @return      None
*
* @details     This macro set specified pin output data.
* \hideinitializer
*/
#define PSIO_SET_OUTPUT_DATA(psio, u32Pin, u32Data) ((psio)->GNCT[(u32Pin)].OUTDAT = (uint32_t)(u32Data))

/**
* @brief       Set specified pin check point and slot link
*
* @param[in]   psio    The pointer of the specified PSIO module
* @param[in]   u32Pin  The selected Pin. Valid values are
*                - \ref PSIO_PIN0
*                - \ref PSIO_PIN1
*                - \ref PSIO_PIN2
*                - \ref PSIO_PIN3
*                - \ref PSIO_PIN4
*                - \ref PSIO_PIN5
*                - \ref PSIO_PIN6
*                - \ref PSIO_PIN7
* @param[in]   u32CheckPoint   The selected check point. Valid values are
*                - \ref PSIO_CP0
*                - \ref PSIO_CP1
*                - \ref PSIO_CP2
*                - \ref PSIO_CP3
*                - \ref PSIO_CP4
*                - \ref PSIO_CP5
*                - \ref PSIO_CP6
*                - \ref PSIO_CP7
* @param[in]   u32Slot  The selected slot. Valid values are
*                - \ref PSIO_SLOT0
*                - \ref PSIO_SLOT1
*                - \ref PSIO_SLOT2
*                - \ref PSIO_SLOT3
*                - \ref PSIO_SLOT4
*                - \ref PSIO_SLOT5
*                - \ref PSIO_SLOT6
*                - \ref PSIO_SLOT7
*
* @return      None
*
* @details     This macro used to link check point and slot.
* \hideinitializer
*/
#define PSIO_SET_CHECKPOINT(psio, u32Pin, u32CheckPoint, u32Slot)    \
    do                                                               \
    {                                                                \
        if (((psio) != (PSIO_T *)0) && PSIO_IS_VALID_PIN(u32Pin)     \
                && PSIO_IS_VALID_CP(u32CheckPoint) && PSIO_IS_VALID_CP_SLOT(u32Slot)) \
        {                                                            \
            (psio)->GNCT[(u32Pin)].CPCTL0 = (((psio)->GNCT[(u32Pin)].CPCTL0 & ~(PSIO_GNCT_CPCTL0_CKPT0_Msk << ((u32CheckPoint) * PSIO_GNCT_CPCTL0_CKPT1_Pos))) \
                                             | ((((uint32_t)(u32Slot)) << ((u32CheckPoint) * PSIO_GNCT_CPCTL0_CKPT1_Pos)) & (PSIO_GNCT_CPCTL0_CKPT0_Msk << ((u32CheckPoint) * PSIO_GNCT_CPCTL0_CKPT1_Pos)))); \
        }                                                            \
    } while (0)

/**
 * @brief       Clear specified pin check point and slot link
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin  The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 * @param[in]   u32CheckPoint   The selected check point. Valid values are
 *                - \ref PSIO_CP0
 *                - \ref PSIO_CP1
 *                - \ref PSIO_CP2
 *                - \ref PSIO_CP3
 *                - \ref PSIO_CP4
 *                - \ref PSIO_CP5
 *                - \ref PSIO_CP6
 *                - \ref PSIO_CP7
 *
 * @return      None
 *
 * @details     This macro used to clear the link of check point and slot.
 * \hideinitializer
 */
#define PSIO_CLEAR_CHECKPOINT(psio, u32Pin, u32CheckPoint)           \
    do                                                               \
    {                                                                \
        if (((psio) != (PSIO_T *)0) && PSIO_IS_VALID_PIN(u32Pin)     \
                && PSIO_IS_VALID_CP(u32CheckPoint))                  \
        {                                                            \
            (psio)->GNCT[(u32Pin)].CPCTL0 = (psio)->GNCT[(u32Pin)].CPCTL0 & ~(PSIO_GNCT_CPCTL0_CKPT0_Msk << ((u32CheckPoint) * PSIO_GNCT_CPCTL0_CKPT1_Pos)); \
        }                                                            \
    } while (0)

/**
 * @brief       Set specified pin action of check point
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin  The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 * @param[in]   u32CheckPoint   The selected check point. Valid values are
 *                - \ref PSIO_CP0
 *                - \ref PSIO_CP1
 *                - \ref PSIO_CP2
 *                - \ref PSIO_CP3
 *                - \ref PSIO_CP4
 *                - \ref PSIO_CP5
 *                - \ref PSIO_CP6
 *                - \ref PSIO_CP7
 * @param[in]   u32Action  The selected action. Valid values are
 *                - \ref PSIO_OUT_LOW
 *                - \ref PSIO_OUT_HIGH
 *                - \ref PSIO_OUT_BUFFER
 *                - \ref PSIO_OUT_TOGGLE
 *                - \ref PSIO_IN_BUFFER
 *                - \ref PSIO_IN_STATUS
 *                - \ref PSIO_IN_STATUS_UPDATE
 *
 * @return      None
 *
 * @details     This macro used to set specified pin action of check point.
 * \hideinitializer
 */
#define PSIO_SET_ACTION(psio, u32Pin, u32CheckPoint, u32Action)      \
    do                                                               \
    {                                                                \
        if (((psio) != (PSIO_T *)0) && PSIO_IS_VALID_PIN(u32Pin)     \
                && PSIO_IS_VALID_CP(u32CheckPoint) && PSIO_IS_VALID_CP_ACTION(u32Action)) \
        {                                                            \
            (psio)->GNCT[(u32Pin)].CPCTL1 = (((psio)->GNCT[(u32Pin)].CPCTL1 & ~(PSIO_GNCT_CPCTL1_CKPT0ACT_Msk << ((u32CheckPoint) * PSIO_GNCT_CPCTL1_CKPT1ACT_Pos))) \
                                             | ((((uint32_t)(u32Action) & 0x7UL) << ((u32CheckPoint) * PSIO_GNCT_CPCTL1_CKPT1ACT_Pos)) & (PSIO_GNCT_CPCTL1_CKPT0ACT_Msk << ((u32CheckPoint) * PSIO_GNCT_CPCTL1_CKPT1ACT_Pos)))); \
        }                                                            \
    } while (0)

/*---------------------------------------------------------------------------------------------------------*/
/* inline functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief       Set interrupt control
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller
 * @param[in]   u32Int  The interrupt type. Valid values are
 *                - \ref PSIO_INT0
 *                - \ref PSIO_INT1
 * @param[in]   u32Slot  The selected slot. Valid values are
 *                - \ref PSIO_SLOT0
 *                - \ref PSIO_SLOT1
 *                - \ref PSIO_SLOT2
 *                - \ref PSIO_SLOT3
 *                - \ref PSIO_SLOT4
 *                - \ref PSIO_SLOT5
 *                - \ref PSIO_SLOT6
 *                - \ref PSIO_SLOT7
 *
 * @return      None
 *
 * @details     This function is used to set the selected slot controller, interrupt type and slot.
 * \hideinitializer
 */
static inline void PSIO_SET_INTCTL(PSIO_T *psio, uint32_t u32SC, uint32_t u32Int, uint32_t u32Slot)
{
    if ((psio == (PSIO_T *)0) || !PSIO_IS_VALID_SC(u32SC) || !PSIO_IS_VALID_INT(u32Int) || !PSIO_IS_VALID_SLOT(u32Slot))
    {
        return;
    }

    if (u32Int == PSIO_INT0)
    {
        (psio)->INTCTL = (((psio)->INTCTL & ~PSIO_INTCTL_CONI0SS_Msk & ~PSIO_INTCTL_CONI0SCS_Msk)
                          | ((((uint32_t)u32SC) << PSIO_INTCTL_CONI0SCS_Pos) & PSIO_INTCTL_CONI0SCS_Msk)
                          | ((((uint32_t)u32Slot) << PSIO_INTCTL_CONI0SS_Pos) & PSIO_INTCTL_CONI0SS_Msk));
    }
    else
    {
        (psio)->INTCTL = (((psio)->INTCTL & ~PSIO_INTCTL_CONI1SS_Msk & ~PSIO_INTCTL_CONI1SCS_Msk)
                          | ((((uint32_t)u32SC) << PSIO_INTCTL_CONI1SCS_Pos) & PSIO_INTCTL_CONI1SCS_Msk)
                          | ((((uint32_t)u32Slot) << PSIO_INTCTL_CONI1SS_Pos) & PSIO_INTCTL_CONI1SS_Msk));
    }
}

/**
 * @brief       Clear interrupt control setting
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Int  The interrupt type. Valid values are
 *                - \ref PSIO_INT0
 *                - \ref PSIO_INT1
 *
 * @return      None
 *
 * @details     This function is used to clear the selected slot controller, interrupt type and slot.
 * \hideinitializer
 */
static inline void PSIO_CLEAR_INTCTL(PSIO_T *psio, uint32_t u32Int)
{
    if ((psio == (PSIO_T *)0) || !PSIO_IS_VALID_INT(u32Int))
    {
        return;
    }

    if (u32Int == PSIO_INT0)
    {
        (psio)->INTCTL = ((psio)->INTCTL & ~PSIO_INTCTL_CONI0SS_Msk & ~PSIO_INTCTL_CONI0SCS_Msk);
    }
    else
    {
        (psio)->INTCTL = ((psio)->INTCTL & ~PSIO_INTCTL_CONI1SS_Msk & ~PSIO_INTCTL_CONI1SCS_Msk);
    }
}

/**
 * @brief       Set Slot controller control
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32SC   The selected slot controller
 * @param[in]   u32InitSlot  The selected initial slot of the repeat period. Valid values are
 *                - \ref PSIO_SLOT_DISABLE : No use
 *                - \ref PSIO_SLOT0
 *                - \ref PSIO_SLOT1
 *                - \ref PSIO_SLOT2
 *                - \ref PSIO_SLOT3
 *                - \ref PSIO_SLOT4
 *                - \ref PSIO_SLOT5
 *                - \ref PSIO_SLOT6
 *                - \ref PSIO_SLOT7
 * @param[in]   u32EndSlot  The selected end slot of the repeat period. Valid values are
 *                - \ref PSIO_SLOT_DISABLE : No use
 *                - \ref PSIO_SLOT0
 *                - \ref PSIO_SLOT1
 *                - \ref PSIO_SLOT2
 *                - \ref PSIO_SLOT3
 *                - \ref PSIO_SLOT4
 *                - \ref PSIO_SLOT5
 *                - \ref PSIO_SLOT6
 *                - \ref PSIO_SLOT7
 * @param[in]   u32LoopCnt The slot period loop count. Valid values are
 *                - 0x0 : Disable
 *                - 0x1~0x3E : Repeat slot 0x2~0x3F times
 *                - 0x3F : Loop until stop PSIO slot controller
 * @param[in]   u32Repeat Repeat mode Enable/Disable. Valid values are
 *                - \ref PSIO_REPEAT_ENABLE
 *                - \ref PSIO_REPEAT_DISABLE
 *
 * @return      None
 *
 * @details     This function is used to set the slot controller loop and repeat configuration.
 * \hideinitializer
 */
static inline void PSIO_SET_SCCTL(PSIO_T *psio, uint32_t u32SC, uint32_t u32InitSlot, uint32_t u32EndSlot, uint32_t u32LoopCnt, uint32_t u32Repeat)
{
    if ((psio == (PSIO_T *)0) || !PSIO_IS_VALID_SC(u32SC) || !PSIO_IS_VALID_SLOT(u32InitSlot) || !PSIO_IS_VALID_SLOT(u32EndSlot) || !PSIO_IS_VALID_REPEAT(u32Repeat))
    {
        return;
    }

    (psio)->SCCT[(u32SC)].SCCTL = ((psio)->SCCT[(u32SC)].SCCTL & ~PSIO_SCCT_SCCTL_INISLOT_Msk & ~PSIO_SCCT_SCCTL_ENDSLOT_Msk & ~PSIO_SCCT_SCCTL_SPLCNT_Msk)
                                  | ((((uint32_t)u32InitSlot) << PSIO_SCCT_SCCTL_INISLOT_Pos) & PSIO_SCCT_SCCTL_INISLOT_Msk)
                                  | ((((uint32_t)u32EndSlot) << PSIO_SCCT_SCCTL_ENDSLOT_Pos) & PSIO_SCCT_SCCTL_ENDSLOT_Msk)
                                  | ((((uint32_t)u32LoopCnt) << (uint32_t)PSIO_SCCT_SCCTL_SPLCNT_Pos) & PSIO_SCCT_SCCTL_SPLCNT_Msk);

    if (u32Repeat == PSIO_REPEAT_ENABLE)
    {
        (psio)->SCCT[(u32SC)].SCCTL |= PSIO_SCCT_SCCTL_REPEAT_Msk;
    }
    else
    {
        (psio)->SCCT[(u32SC)].SCCTL &= ~PSIO_SCCT_SCCTL_REPEAT_Msk;
    }
}

/**
 * @brief       Set Pin general control
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin      The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 * @param[in]   u32PinEn    The selected Pin enable/disable. Valid values are
 *                - \ref PSIO_PIN_ENABLE
 *                - \ref PSIO_PIN_DISABLE
 * @param[in]   u32SC       The selected slot controller for check point. Valid values are
 *                - \ref PSIO_SC0
 *                - \ref PSIO_SC1
 *                - \ref PSIO_SC2
 *                - \ref PSIO_SC3
 * @param[in]   u32IOMode   The pin I/O mode. Valid values are
 *                - \ref PSIO_INPUT_MODE
 *                - \ref PSIO_OUTPUT_MODE
 *                - \ref PSIO_OPENDRAIN_MODE
 *                - \ref PSIO_QUASI_MODE
 * @param[in]   u32PinInit  The pin initial status. Valid values are
 *                - \ref PSIO_LOW_LEVEL
 *                - \ref PSIO_HIGH_LEVEL
 *                - \ref PSIO_LAST_OUTPUT
 *                - \ref PSIO_Toggle
 * @param[in]   u32PinInterval The pin interval status. Valid values are
 *                - \ref PSIO_LOW_LEVEL
 *                - \ref PSIO_HIGH_LEVEL
 *                - \ref PSIO_LAST_OUTPUT
 *                - \ref PSIO_Toggle
 *
 * @return      None
 *
 * @details     This function is used to set the general control.
 * \hideinitializer
 */
static inline void PSIO_SET_GENCTL(PSIO_T *psio, uint32_t u32Pin, uint32_t u32PinEn, uint32_t u32SC, uint32_t u32IOMode, uint32_t u32PinInit, uint32_t u32PinInterval)
{
    if ((psio == (PSIO_T *)0) || !PSIO_IS_VALID_PIN(u32Pin) || !PSIO_IS_VALID_PIN_ENABLE(u32PinEn)
            || !PSIO_IS_VALID_SC(u32SC) || !PSIO_IS_VALID_IO_MODE(u32IOMode)
            || !PSIO_IS_VALID_PIN_LEVEL(u32PinInit) || !PSIO_IS_VALID_PIN_LEVEL(u32PinInterval))
    {
        return;
    }

    (psio)->GNCT[u32Pin].GENCTL = ((psio)->GNCT[(u32Pin)].GENCTL & ~PSIO_GNCT_GENCTL_SCSEL_Msk & ~PSIO_GNCT_GENCTL_IOMODE_Msk
                                   & ~PSIO_GNCT_GENCTL_INITIAL_Msk & ~PSIO_GNCT_GENCTL_INTERVAL_Msk)
                                  | ((((uint32_t)u32SC) << PSIO_GNCT_GENCTL_SCSEL_Pos) & PSIO_GNCT_GENCTL_SCSEL_Msk)
                                  | ((((uint32_t)u32IOMode) << PSIO_GNCT_GENCTL_IOMODE_Pos) & PSIO_GNCT_GENCTL_IOMODE_Msk)
                                  | ((((uint32_t)u32PinInit) << PSIO_GNCT_GENCTL_INITIAL_Pos) & PSIO_GNCT_GENCTL_INITIAL_Msk)
                                  | ((((uint32_t)u32PinInterval) << PSIO_GNCT_GENCTL_INTERVAL_Pos) & PSIO_GNCT_GENCTL_INTERVAL_Msk);

    if (u32PinEn == PSIO_PIN_ENABLE)
    {
        (psio)->GNCT[(u32Pin)].GENCTL |= PSIO_GNCT_GENCTL_PINEN_Msk;
    }
    else
    {
        (psio)->GNCT[(u32Pin)].GENCTL &= ~PSIO_GNCT_GENCTL_PINEN_Msk;
    }
}

/**
 * @brief       Set Pin mode switch
 *
 * @param[in]   psio    The pointer of the specified PSIO module
 * @param[in]   u32Pin      The selected Pin. Valid values are
 *                - \ref PSIO_PIN0
 *                - \ref PSIO_PIN1
 *                - \ref PSIO_PIN2
 *                - \ref PSIO_PIN3
 *                - \ref PSIO_PIN4
 *                - \ref PSIO_PIN5
 *                - \ref PSIO_PIN6
 *                - \ref PSIO_PIN7
 * @param[in]   u32SwPoint  The switch point. Valid values are
 *                - \ref PSIO_SWITCH_P0
 *                - \ref PSIO_SWITCH_P1
 * @param[in]   u32SwMode   The switch mode. Valid values are
 *                - \ref PSIO_INPUT_MODE
 *                - \ref PSIO_OUTPUT_MODE
 *                - \ref PSIO_OPENDRAIN_MODE
 *                - \ref PSIO_QUASI_MODE
 * @param[in]   u32SwCP     The switch I/O mode at which point. Valid values are
 *                - \ref PSIO_CP0
 *                - \ref PSIO_CP1
 *                - \ref PSIO_CP2
 *                - \ref PSIO_CP3
 *                - \ref PSIO_CP4
 *                - \ref PSIO_CP5
 *                - \ref PSIO_CP6
 *                - \ref PSIO_CP7
 * @return      None
 *
 * @details     This function is used to set the pin mode switch.
 * \hideinitializer
 */
static inline void PSIO_SWITCH_MODE(PSIO_T *psio, uint32_t u32Pin, uint32_t u32SwPoint, uint32_t u32SwMode, uint32_t u32SwCP)
{
    if ((psio == (PSIO_T *)0) || !PSIO_IS_VALID_PIN(u32Pin) || !PSIO_IS_VALID_SWITCH_POINT(u32SwPoint)
            || !PSIO_IS_VALID_IO_MODE(u32SwMode) || !PSIO_IS_VALID_CP(u32SwCP))
    {
        return;
    }

    if (u32SwPoint == PSIO_SWITCH_P0)
    {
        (psio)->GNCT[(u32Pin)].GENCTL = ((psio)->GNCT[(u32Pin)].GENCTL & ~PSIO_GNCT_GENCTL_MODESW0_Msk & ~PSIO_GNCT_GENCTL_SW0CP_Msk)
                                        | ((((uint32_t)u32SwMode) << PSIO_GNCT_GENCTL_MODESW0_Pos) & PSIO_GNCT_GENCTL_MODESW0_Msk)
                                        | ((((uint32_t)(u32SwCP + 1UL)) << (uint32_t)PSIO_GNCT_GENCTL_SW0CP_Pos) & PSIO_GNCT_GENCTL_SW0CP_Msk);
    }
    else
    {
        (psio)->GNCT[(u32Pin)].GENCTL = ((psio)->GNCT[(u32Pin)].GENCTL & ~PSIO_GNCT_GENCTL_MODESW1_Msk & ~PSIO_GNCT_GENCTL_SW1CP_Msk)
                                        | ((((uint32_t)u32SwMode) << PSIO_GNCT_GENCTL_MODESW1_Pos) & PSIO_GNCT_GENCTL_MODESW1_Msk)
                                        | ((((uint32_t)(u32SwCP + 1UL)) << (uint32_t)PSIO_GNCT_GENCTL_SW1CP_Pos) & PSIO_GNCT_GENCTL_SW1CP_Msk);
    }
}

/**
* @brief       Set specified pin check point and slot link, and pin action of check point
*
* @param[in]   psio    The pointer of the specified PSIO module
* @param[in]   u32Pin  The selected Pin. Valid values are
*                - \ref PSIO_PIN0
*                - \ref PSIO_PIN1
*                - \ref PSIO_PIN2
*                - \ref PSIO_PIN3
*                - \ref PSIO_PIN4
*                - \ref PSIO_PIN5
*                - \ref PSIO_PIN6
*                - \ref PSIO_PIN7
* @param[in]   sConfig   The selected check point configurations.
*
* @return      None
*
* @details     This macro used to link check point and slot, and set pin action of check point.
* \hideinitializer
*/
static inline void PSIO_SET_CP_CONFIG(PSIO_T *psio, uint32_t u32Pin, const S_PSIO_CP_CONFIG *sConfig)
{
    uint32_t u32Cpctl0;
    uint32_t u32Cpctl1;
    uint32_t u32Shift;

    if ((psio == (PSIO_T *)0) || !PSIO_IS_VALID_PIN(u32Pin) || (sConfig == (const S_PSIO_CP_CONFIG *)0)
            || !PSIO_IS_VALID_CP_SLOT(sConfig->CKPT0SLT) || !PSIO_IS_VALID_CP_SLOT(sConfig->CKPT1SLT)
            || !PSIO_IS_VALID_CP_SLOT(sConfig->CKPT2SLT) || !PSIO_IS_VALID_CP_SLOT(sConfig->CKPT3SLT)
            || !PSIO_IS_VALID_CP_SLOT(sConfig->CKPT4SLT) || !PSIO_IS_VALID_CP_SLOT(sConfig->CKPT5SLT)
            || !PSIO_IS_VALID_CP_SLOT(sConfig->CKPT6SLT) || !PSIO_IS_VALID_CP_SLOT(sConfig->CKPT7SLT)
            || !PSIO_IS_VALID_CP_ACTION(sConfig->CKPT0ACT) || !PSIO_IS_VALID_CP_ACTION(sConfig->CKPT1ACT)
            || !PSIO_IS_VALID_CP_ACTION(sConfig->CKPT2ACT) || !PSIO_IS_VALID_CP_ACTION(sConfig->CKPT3ACT)
            || !PSIO_IS_VALID_CP_ACTION(sConfig->CKPT4ACT) || !PSIO_IS_VALID_CP_ACTION(sConfig->CKPT5ACT)
            || !PSIO_IS_VALID_CP_ACTION(sConfig->CKPT6ACT) || !PSIO_IS_VALID_CP_ACTION(sConfig->CKPT7ACT))
    {
        return;
    }

    u32Shift = (uint32_t)PSIO_GNCT_CPCTL0_CKPT1_Pos;
    u32Cpctl0 = ((((uint32_t)(sConfig->CKPT0SLT & 0xFUL)) << (0UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT1SLT & 0xFUL)) << (1UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT2SLT & 0xFUL)) << (2UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT3SLT & 0xFUL)) << (3UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT4SLT & 0xFUL)) << (4UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT5SLT & 0xFUL)) << (5UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT6SLT & 0xFUL)) << (6UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT7SLT & 0xFUL)) << (7UL * u32Shift)));

    u32Shift = (uint32_t)PSIO_GNCT_CPCTL1_CKPT1ACT_Pos;
    u32Cpctl1 = ((((uint32_t)(sConfig->CKPT0ACT & 0x7UL)) << (0UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT1ACT & 0x7UL)) << (1UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT2ACT & 0x7UL)) << (2UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT3ACT & 0x7UL)) << (3UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT4ACT & 0x7UL)) << (4UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT5ACT & 0x7UL)) << (5UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT6ACT & 0x7UL)) << (6UL * u32Shift)) |
                 (((uint32_t)(sConfig->CKPT7ACT & 0x7UL)) << (7UL * u32Shift)));

    (psio)->GNCT[(u32Pin)].CPCTL0 = u32Cpctl0;
    (psio)->GNCT[(u32Pin)].CPCTL1 = u32Cpctl1;
}

/** @} end of group PSIO_EXPORTED_FUNCTIONS */
/** @} end of group PSIO_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __PSIO_H__ */
