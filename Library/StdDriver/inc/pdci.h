/**************************************************************************//**
 * @file     pdci.h
 * @version  V3.00
 * @brief    Parallel Dimming Communication Interface(PDCI) driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PDCI_H__
#define __PDCI_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup PDCI_Driver PDCI Driver
  @{
*/

/** @addtogroup PDCI_EXPORTED_CONSTANTS PDCI Exported Constants
  @{
*/
#define PDCI_BITWIDTH_1               (0UL)                      /*!< The bit time period of logic 0 is same as logic 1 \hideinitializer */
#define PDCI_BITWIDTH_15              (PDCI_CTL_BWADJ_Msk)       /*!< The bit time period of logic 0 is 1.5 times logic 1 \hideinitializer */

#define PDCI_PREAMBLE_64              (0UL)                      /*!< PDCI preamble is 64 bits \hideinitializer */
#define PDCI_PREAMBLE_60              (10UL)                     /*!< PDCI preamble is 60 bits \hideinitializer */
#define PDCI_PREAMBLE_48              (9UL)                      /*!< PDCI preamble is 48 bits \hideinitializer */
#define PDCI_PREAMBLE_40              (8UL)                      /*!< PDCI preamble is 40 bits \hideinitializer */
#define PDCI_PREAMBLE_32              (7UL)                      /*!< PDCI preamble is 32 bits \hideinitializer */
#define PDCI_PREAMBLE_20              (6UL)                      /*!< PDCI preamble is 20 bits \hideinitializer */
#define PDCI_PREAMBLE_16              (5UL)                      /*!< PDCI preamble is 16 bits \hideinitializer */
#define PDCI_PREAMBLE_10              (4UL)                      /*!< PDCI preamble is 10 bits \hideinitializer */
#define PDCI_PREAMBLE_8               (3UL)                      /*!< PDCI preamble is 8 bits \hideinitializer */
#define PDCI_PREAMBLE_5               (2UL)                      /*!< PDCI preamble is 5 bits \hideinitializer */
#define PDCI_PREAMBLE_4               (1UL)                      /*!< PDCI preamble is 4 bits \hideinitializer */

#define PDCI_DUM_LVL_LOW              (0UL)                      /*!< PDCI dummy level is low \hideinitializer */
#define PDCI_DUM_LVL_HIGH             (PDCI_CTL_DUMLVL_Msk)      /*!< PDCI dummy level is high \hideinitializer */

#define PDCI_TAIL_LVL_LOW             (0UL)                      /*!< PDCI tail level is low \hideinitializer */
#define PDCI_TAIL_LVL_HIGH            (PDCI_CTL_TAILVL_Msk)      /*!< PDCI tail level is high \hideinitializer */

#define PDCI_GROUP_0                  (0UL)                      /*!< PDCI group 0 mask \hideinitializer */
#define PDCI_GROUP_1                  (4UL)                      /*!< PDCI group 1 mask \hideinitializer */
#define PDCI_GROUP_2                  (8UL)                      /*!< PDCI group 2 mask \hideinitializer */
#define PDCI_GROUP_3                  (12UL)                     /*!< PDCI group 3 mask \hideinitializer */
#define PDCI_GROUP_4                  (16UL)                     /*!< PDCI group 4 mask \hideinitializer */
#define PDCI_GROUP_5                  (20UL)                     /*!< PDCI group 5 mask \hideinitializer */

#define PDCI_CHANNEL_NUM              (24UL)                     /*!< PDCI total channel number \hideinitializer */
#define PDCI_GROUP_NUM                (6UL)                      /*!< PDCI total group number \hideinitializer */

#define PDCI_G0TXUND_MASK             (0x0001UL)                 /*!< PDCI group 0 transmit data under run mask \hideinitializer */
#define PDCI_G1TXUND_MASK             (0x0002UL)                 /*!< PDCI group 1 transmit data under run mask \hideinitializer */
#define PDCI_G2TXUND_MASK             (0x0004UL)                 /*!< PDCI group 2 transmit data under run mask \hideinitializer */
#define PDCI_G3TXUND_MASK             (0x0008UL)                 /*!< PDCI group 3 transmit data under run mask \hideinitializer */
#define PDCI_G4TXUND_MASK             (0x0010UL)                 /*!< PDCI group 4 transmit data under run mask \hideinitializer */
#define PDCI_G5TXUND_MASK             (0x0020UL)                 /*!< PDCI group 5 transmit data under run mask \hideinitializer */
#define PDCI_G0TXDF_MASK              (0x0100UL)                 /*!< PDCI group 0 transmit done mask \hideinitializer */
#define PDCI_G1TXDF_MASK              (0x0200UL)                 /*!< PDCI group 1 transmit done mask \hideinitializer */
#define PDCI_G2TXDF_MASK              (0x0400UL)                 /*!< PDCI group 2 transmit done mask \hideinitializer */
#define PDCI_G3TXDF_MASK              (0x0800UL)                 /*!< PDCI group 3 transmit done mask \hideinitializer */
#define PDCI_G4TXDF_MASK              (0x1000UL)                 /*!< PDCI group 4 transmit done mask \hideinitializer */
#define PDCI_G5TXDF_MASK              (0x2000UL)                 /*!< PDCI group 5 transmit done mask \hideinitializer */

#define PDCI_FTXD_INT_MASK            (0x001UL)                  /*!< Frame transmit done interrupt mask \hideinitializer */
#define PDCI_TXUND_INT_MASK           (0x002UL)                  /*!< Transmit data under run interrupt mask \hideinitializer */
#define PDCI_FEMPT_INT_MASK           (0x004UL)                  /*!< FIFO empty interrupt mask \hideinitializer */

#define PDCI_RXFULL_INT_MASK          (0x010UL)                  /*!< Receiver FIFO full interrupt mask \hideinitializer */
#define PDCI_RXDONE_INT_MASK          (0x020UL)                  /*!< Receiver transfer done interrupt mask \hideinitializer */
#define PDCI_RXOV_INT_MASK            (0x040UL)                  /*!< Receiver FIFO overflow interrupt mask \hideinitializer */
#define PDCI_RXFAIL_INT_MASK          (0x080UL)                  /*!< Receiver failure interrupt mask \hideinitializer */
#define PDCI_RXCRC_INT_MASK           (0x100UL)                  /*!< Receiver CRC error interrupt mask \hideinitializer */
#define PDCI_RXCRCA_INT_MASK          (0x200UL)                  /*!< Receiver CRC address error interrupt mask \hideinitializer */

#define PDCI_KCODE_SOURCE_SRAM        (0UL)                      /*!< The content of K-code in PDCI packet stored in SRAM \hideinitializer */
#define PDCI_KCODE_SOURCE_REGISTER    (PDCI_CTL2_KCODES_Msk)     /*!< The content of K-code in PDCI packet stored in PDCI_KCODE register \hideinitializer */

#define PDCI_MODE_0                   (0UL)                      /*!< 5-bit data mode for entire packet \hideinitializer */
#define PDCI_MODE_1                   (1UL)                      /*!< 4-bit data mode + 5-bit KCODEm (m= 1~4) in PDCI_KCODE register \hideinitializer */
#define PDCI_MODE_2                   (2UL)                      /*!< 4-bit data mode for entire packet \hideinitializer */
#define PDCI_MODE_3                   (3UL)                      /*!< Data only mode \hideinitializer */

#define PDCI_IO_GROUP_0               (0UL)                      /*!< PDCI IO group 0 */
#define PDCI_IO_GROUP_1               (1UL)                      /*!< PDCI IO group 1 */
#define PDCI_IO_GROUP_2               (2UL)                      /*!< PDCI IO group 2 */
#define PDCI_IO_GROUP_3               (3UL)                      /*!< PDCI IO group 3 */
#define PDCI_IO_GROUP_4               (4UL)                      /*!< PDCI IO group 4 */
#define PDCI_IO_GROUP_5               (5UL)                      /*!< PDCI IO group 5 */


/*@}*/ /* end of group PDCI_EXPORTED_CONSTANTS */


/** @addtogroup PDCI_EXPORTED_FUNCTIONS PDCI Exported Functions
  @{
*/

/**
  * @brief      Enable PDCI controller
  * @return     None
  * @details    This macro is used to enable Biphase Mask Coding function.
  * \hideinitializer
  */
#define PDCI_ENABLE()    (PDCI->CTL |= PDCI_CTL_PDCIEN_Msk)

/**
  * @brief      Disable PDCI controller
  * @return     None
  * @details    This macro is used to disable Biphase Mask Coding function.
  * \hideinitializer
  */
#define PDCI_DISABLE()    (PDCI->CTL &= ~PDCI_CTL_PDCIEN_Msk)

/**
  * @brief      Set the bit width adjustment
  * @param[in]  u32BitAdj PDCI bit time period adjustment selection, valid values are:
  *                       - \ref PDCI_BITWIDTH_1
  *                       - \ref PDCI_BITWIDTH_15
  * @return     None
  * @details    This macro is used to set bit width adjustment.
  * \hideinitializer
  */
#define PDCI_BITWIDTH_ADJUST(u32BitAdj)    (PDCI->CTL = (PDCI->CTL & ~PDCI_CTL_BWADJ_Msk) | (u32BitAdj))

/**
  * @brief      Set the bit number of preamble
  * @param[in]  u32PreamBit PDCI preamble bit number selection, valid values are:
  *                         - \ref PDCI_PREAMBLE_64
  *                         - \ref PDCI_PREAMBLE_60
  *                         - \ref PDCI_PREAMBLE_48
  *                         - \ref PDCI_PREAMBLE_40
  *                         - \ref PDCI_PREAMBLE_32
  *                         - \ref PDCI_PREAMBLE_20
  *                         - \ref PDCI_PREAMBLE_16
  *                         - \ref PDCI_PREAMBLE_10
  *                         - \ref PDCI_PREAMBLE_8
  *                         - \ref PDCI_PREAMBLE_5
  *                         - \ref PDCI_PREAMBLE_4
  * @return     None
  * @details    This macro is used to set the bit number of preamble.
  * \hideinitializer
  */
#define PDCI_PREAMBLE_BIT(u32PreamBit)    (PDCI->CTL2 = (PDCI->CTL2 & ~PDCI_CTL2_PRENUM_Msk) | ((u32PreamBit) << PDCI_CTL2_PRENUM_Pos))

/**
  * @brief      Set the dummy bit level
  * @param[in]  u32DumLvl PDCI dummy bit level selection, valid values are:
  *                       - \ref PDCI_DUM_LVL_LOW
  *                       - \ref PDCI_DUM_LVL_HIGH
  * @return     None
  * @details    This macro is used to set dummy bit level.
  * \hideinitializer
  */
#define PDCI_DUMMY_LEVEL(u32DumLvl)    (PDCI->CTL = (PDCI->CTL & ~PDCI_CTL_DUMLVL_Msk) | (u32DumLvl))

/**
  * @brief      Enable PDMA function
  * @return     None
  * @details    This macro is used to enable PDMA function.
  * \hideinitializer
  */
#define PDCI_ENABLE_DMA()    (PDCI->CTL |= PDCI_CTL_DMAEN_Msk)

/**
  * @brief      Disable PDMA function
  * @return     None
  * @details    This macro is used to disable PDMA function.
  * \hideinitializer
  */
#define PDCI_DISABLE_DMA()    (PDCI->CTL &= ~PDCI_CTL_DMAEN_Msk)

/**
  * @brief      Set the tail bit level
  * @param[in]  u32TailLvl PDCI tail bit level selection, valid values are:
  *                       - \ref PDCI_TAIL_LVL_LOW
  *                       - \ref PDCI_TAIL_LVL_HIGH
  * @return     None
  * @details    This macro is used to set tail bit level.
  * \hideinitializer
  */
#define PDCI_TAIL_LEVEL(u32TailLvl)    (PDCI->CTL = (PDCI->CTL & ~PDCI_CTL_TAILVL_Msk) | (u32TailLvl))

/**
  * @brief      Set the bit number of tail
  * @param[in]  u32TailBit Decides the PDCI tail bit number. It could be 0 ~ 7.
  * @return     None
  * @details    This macro is used to set the bit number of tail.
  * \hideinitializer
  */
#define PDCI_TAIL_BIT(u32TailBit)    (PDCI->CTL2 = (PDCI->CTL2 & ~PDCI_CTL2_TAILNUM_Msk) | ((u32TailBit) << PDCI_CTL2_TAILNUM_Pos))

/**
  * @brief      Set the KCODE source
  * @param[in]  u32KcodeSrc PDCI KCODE source selection, valid values are:
  *                       - \ref PDCI_KCODE_SOURCE_SRAM
  *                       - \ref PDCI_KCODE_SOURCE_REGISTER
  * @return     None
  * @details    This macro is used to set the KCODE source.
  * \hideinitializer
  */
#define PDCI_KCODE_SRC_SEL(u32Mode)    (PDCI->CTL2 = (PDCI->CTL2 & ~PDCI_CTL2_KCODES_Msk) | (u32Mode))

/**
  * @brief      Enable CRC function
  * @return     None
  * @details    This macro is used to enable CRC function.
  * \hideinitializer
  */
#define PDCI_ENABLE_CRC()    (PDCI->CTL2 |= PDCI_CTL2_CRCEN_Msk)

/**
  * @brief      Disable CRC function
  * @return     None
  * @details    This macro is used to disable CRC function.
  * \hideinitializer
  */
#define PDCI_DISABLE_CRC()    (PDCI->CTL2 &= ~PDCI_CTL2_CRCEN_Msk)

/**
  * @brief      Enable 4-bit to 5-bit transfer function
  * @return     None
  * @details    This macro is used to enable B4 to B5 transfer.
  * \hideinitializer
  */
#define PDCI_ENABLE_B4B5_TRANSFER()    (PDCI->CTL2 |= PDCI_CTL2_B4B5EN_Msk)

/**
  * @brief      Disable 4-bit to 5-bit transfer function
  * @return     None
  * @details    This macro is used to disable B4 to B5 transfer.
  * \hideinitializer
  */
#define PDCI_DISABLE_B4B5_TRANSFER()   (PDCI->CTL2 &= ~PDCI_CTL2_B4B5EN_Msk)

/**
  * @brief      Enable two channel line mode
  * @return     None
  * @details    This macro is used to enable two channel line mode.
  * \hideinitializer
  */
#define PDCI_ENABLE_TWO_CAHNNEL()    (PDCI->CTL2 |= PDCI_CTL2_TWOCHEN_Msk)

/**
  * @brief      Disable two channel line mode
  * @return     None
  * @details    This macro is used to disable two channel line mode mode.
  * \hideinitializer
  */
#define PDCI_DISABLE_TWO_CAHNNEL()    (PDCI->CTL2 &= ~PDCI_CTL2_TWOCHEN_Msk)

/**
  * @brief      Set the PDCI transfer sequence with MSB first.
  * @return     None
  * @details    Set MSB bit of PDCI_CTL2 register to set the PDCI transfer sequence with MSB first.
  * \hideinitializer
  */
#define PDCI_SET_MSB_FIRST()    (PDCI->CTL2 |= PDCI_CTL2_MSB_Msk)

/**
  * @brief      Set the PDCI transfer sequence with LSB first.
  * @return     None
  * @details    Clear MSB bit of PDCI_CTL2 register to set the PDCI transfer sequence with LSB first.
  * \hideinitializer
  */
#define PDCI_SET_LSB_FIRST()    (PDCI->CTL2 &= ~PDCI_CTL2_MSB_Msk)

/**
  * @brief      Enable zero 3-bit after EOP
  * @return     None
  * @details    This macro is used to enable zero 3-bit after EOP.
  * \hideinitializer
  */
#define PDCI_ENABLE_ZERO3B()    (PDCI->CTL2 |= PDCI_CTL2_ZERO3B_Msk)

/**
  * @brief      Disable zero 3-bit after EOP
  * @return     None
  * @details    This macro is used to disable zero 3-bit after EOP.
  * \hideinitializer
  */
#define PDCI_DISABLE_ZERO3B()    (PDCI->CTL2 &= ~PDCI_CTL2_ZERO3B_Msk)

/**
  * @brief      Set the packet mode
  * @param[in]  u32Mode PDCI packet mode selection, valid values are:
  *                       - \ref PDCI_MODE_0
  *                       - \ref PDCI_MODE_1
  *                       - \ref PDCI_MODE_2
  *                       - \ref PDCI_MODE_3
  * @return     None
  * @details    This macro is used to set the packet mode.
  * \hideinitializer
  */
#define PDCI_MODE_SEL(u32Mode)    (PDCI->CTL2 = (PDCI->CTL2 & ~PDCI_CTL2_MODES_Msk) | ((u32Mode) << PDCI_CTL2_MODES_Pos))

/**
  * @brief      Enable Half Duplex Mode
  * @return     None
  * @details    This macro is used to enable Half Duplex mode.
  * \hideinitializer
  */
#define PDCI_ENABLE_HALF_DUPLEX()    (PDCI->CTL2 |= PDCI_CTL2_HDMODE_Msk)

/**
  * @brief      Disable Half Duplex Mode
  * @return     None
  * @details    This macro is used to disable Half Duplex mode.
  * \hideinitializer
  */
#define PDCI_DISABLE_HALF_DUPLEX()   (PDCI->CTL2 &= ~PDCI_CTL2_HDMODE_Msk)

/**
  * @brief      Enable PDCI group 0 channels
  * @return     None
  * @details    This macro is used to enable PDCI channel 0~3.
  * \hideinitializer
  */
#define PDCI_ENABLE_GROUP0()    (PDCI->CTL |= PDCI_CTL_G0CHEN_Msk)

/**
  * @brief      Disable PDCI group 0 channels
  * @return     None
  * @details    This macro is used to disable PDCI channel 0~3.
  * \hideinitializer
  */
#define PDCI_DISABLE_GROUP0()    (PDCI->CTL &= ~PDCI_CTL_G0CHEN_Msk)

/**
  * @brief      Get the PDCI group 0 transfer state.
  * @retval     0 The transfer of PDCI group 0 is done.
  * @retval     1 The transfer of PDCI group 0 is not done.
  * @details    This macro will return the transfer state of PDCI group 0.
  * \hideinitializer
  */
#define PDCI_GROUP0_IS_DONE()    ((PDCI->CTL & PDCI_CTL_G0CHEN_Msk) >> PDCI_CTL_G0CHEN_Pos)

/**
  * @brief      Enable PDCI group 1 channels
  * @return     None
  * @details    This macro is used to enable PDCI channel 4~7.
  * \hideinitializer
  */
#define PDCI_ENABLE_GROUP1()    (PDCI->CTL |= PDCI_CTL_G1CHEN_Msk)

/**
  * @brief      Disable PDCI group 1 channels
  * @return     None
  * @details    This macro is used to disable PDCI channel 4~7.
  * \hideinitializer
  */
#define PDCI_DISABLE_GROUP1()    (PDCI->CTL &= ~PDCI_CTL_G1CHEN_Msk)

/**
  * @brief      Get the PDCI group 1 transfer state.
  * @retval     0 The transfer of PDCI group 1 is done.
  * @retval     1 The transfer of PDCI group 1 is not done.
  * @details    This macro will return the transfer state of PDCI group 1.
  * \hideinitializer
  */
#define PDCI_GROUP1_IS_DONE()    ((PDCI->CTL & PDCI_CTL_G1CHEN_Msk) >> PDCI_CTL_G1CHEN_Pos)

/**
  * @brief      Enable PDCI group 2 channels
  * @return     None
  * @details    This macro is used to enable PDCI channel 8~11.
  * \hideinitializer
  */
#define PDCI_ENABLE_GROUP2()    (PDCI->CTL |= PDCI_CTL_G2CHEN_Msk)

/**
  * @brief      Disable PDCI group 2 channels
  * @return     None
  * @details    This macro is used to disable PDCI channel 8~11.
  * \hideinitializer
  */
#define PDCI_DISABLE_GROUP2()    (PDCI->CTL &= ~PDCI_CTL_G2CHEN_Msk)

/**
  * @brief      Get the PDCI group 2 transfer state.
  * @retval     0 The transfer of PDCI group 2 is done.
  * @retval     1 The transfer of PDCI group 2 is not done.
  * @details    This macro will return the transfer state of PDCI group 2.
  * \hideinitializer
  */
#define PDCI_GROUP2_IS_DONE()    ((PDCI->CTL & PDCI_CTL_G2CHEN_Msk) >> PDCI_CTL_G2CHEN_Pos)

/**
  * @brief      Enable PDCI group 3 channels
  * @return     None
  * @details    This macro is used to enable PDCI channel 12~15.
  * \hideinitializer
  */
#define PDCI_ENABLE_GROUP3()    (PDCI->CTL |= PDCI_CTL_G3CHEN_Msk)

/**
  * @brief      Disable PDCI group 3 channels
  * @return     None
  * @details    This macro is used to disable PDCI channel 12~15.
  * \hideinitializer
  */
#define PDCI_DISABLE_GROUP3()    (PDCI->CTL &= ~PDCI_CTL_G3CHEN_Msk)

/**
  * @brief      Get the PDCI group 3 transfer state.
  * @retval     0 The transfer of PDCI group 3 is done.
  * @retval     1 The transfer of PDCI group 3 is not done.
  * @details    This macro will return the transfer state of PDCI group 3.
  * \hideinitializer
  */
#define PDCI_GROUP3_IS_DONE()    ((PDCI->CTL & PDCI_CTL_G3CHEN_Msk) >> PDCI_CTL_G3CHEN_Pos)

/**
  * @brief      Enable PDCI group 4 channels
  * @return     None
  * @details    This macro is used to enable PDCI channel 16~19.
  * \hideinitializer
  */
#define PDCI_ENABLE_GROUP4()    (PDCI->CTL |= PDCI_CTL_G4CHEN_Msk)

/**
  * @brief      Disable PDCI group 4 channels
  * @return     None
  * @details    This macro is used to disable PDCI channel 16~19.
  * \hideinitializer
  */
#define PDCI_DISABLE_GROUP4()    (PDCI->CTL &= ~PDCI_CTL_G4CHEN_Msk)

/**
  * @brief      Get the PDCI group 4 transfer state.
  * @retval     0 The transfer of PDCI group 4 is done.
  * @retval     1 The transfer of PDCI group 4 is not done.
  * @details    This macro will return the transfer state of PDCI group 4.
  * \hideinitializer
  */
#define PDCI_GROUP4_IS_DONE()    ((PDCI->CTL & PDCI_CTL_G4CHEN_Msk) >> PDCI_CTL_G4CHEN_Pos)

/**
  * @brief      Enable PDCI group 5 channels
  * @return     None
  * @details    This macro is used to enable PDCI channel 20~23.
  * \hideinitializer
  */
#define PDCI_ENABLE_GROUP5()    (PDCI->CTL |= PDCI_CTL_G5CHEN_Msk)

/**
  * @brief      Disable PDCI group 5 channels
  * @return     None
  * @details    This macro is used to disable PDCI channel 20~23.
  * \hideinitializer
  */
#define PDCI_DISABLE_GROUP5()    (PDCI->CTL &= ~PDCI_CTL_G5CHEN_Msk)

/**
  * @brief      Get the PDCI group 5 transfer state.
  * @retval     0 The transfer of PDCI group 5 is done.
  * @retval     1 The transfer of PDCI group 5 is not done.
  * @details    This macro will return the transfer state of PDCI group 5.
  * \hideinitializer
  */
#define PDCI_GROUP5_IS_DONE()    ((PDCI->CTL & PDCI_CTL_G5CHEN_Msk) >> PDCI_CTL_G5CHEN_Pos)

/**
  * @brief      Get channel's FIFO empty flag
  * @return     Which channel's FIFO is empty
  * @details    This macro will return which channel's FIFO is empty.
  * \hideinitializer
  */
#define PDCI_GET_CH_EMPTY_FLAG()    (PDCI->CHEMPTY)

/**
  * @brief      Enable PDCI RX channel 0 ~ 23
  * @param[in]  u32ChNum Decides the PDCI RX channel number. It could be 0 ~ 23.
  * @return     None
  * @details    This macro is used to enable PDCI RX channel 0 ~ 23.
  * \hideinitializer
  */
#define PDCI_ENABLE_RX_CHANNEL(u32ChNum)    (PDCI->RXCTL1 |= 1 << (u32ChNum))

/**
  * @brief      Disable PDCI RX channel 0 ~ 23
  * @param[in]  u32ChNum Decides the PDCI RX channel number. It could be 0 ~ 23.
  * @return     None
  * @details    This macro is used to disable PDCI RX channel 0 ~ 23.
  * \hideinitializer
  */
#define PDCI_DISABLE_RX_CHANNEL(u32ChNum)    (PDCI->RXCTL1 &= ~(1 << (u32ChNum)))

/**
  * @brief      Enable RX PDMA function
  * @return     None
  * @details    This macro is used to enable RX PDMA function.
  * \hideinitializer
  */
#define PDCI_ENABLE_RX_DMA()    (PDCI->RXCTL0 |= PDCI_RXCTL0_RXPDMAEN_Msk)

/**
  * @brief      Disable RX PDMA function
  * @return     None
  * @details    This macro is used to disable RX PDMA function.
  * \hideinitializer
  */
#define PDCI_DISABLE_RX_DMA()    (PDCI->RXCTL0 &= ~PDCI_RXCTL0_RXPDMAEN_Msk)

/**
  * @brief      Enable OSPB data format
  * @return     None
  * @details    This macro is used to enable OSPB data format.
  * \hideinitializer
  */
#define PDCI_ENABLE_OSPBMODE()    (PDCI->RXCTL0 |= PDCI_RXCTL0_OSPBMODE_Msk)

/**
  * @brief      Disable OSPB data format
  * @return     None
  * @details    This macro is used to disable OSPB data format.
  * \hideinitializer
  */
#define PDCI_DISABLE_OSPBMODE()    (PDCI->RXCTL0 &= ~PDCI_RXCTL0_OSPBMODE_Msk)

/**
  * @brief      Set RX Address Counter
  * @param[in]  u32RxAddrCnt The value to set for the RX address counter. It could be 0 ~ 8.
  * @return     None
  * @details    This macro is used to set the RX address counter.
  * \hideinitializer
  */
#define PDCI_SET_RX_ADDR_CNT(u32RxAddrCnt)    (PDCI->RXCNT = (PDCI->RXCNT & ~PDCI_RXCNT_RXADDRC_Msk) | ((u32RxAddrCnt) << PDCI_RXCNT_RXADDRC_Pos))


uint32_t PDCI_SetBitClock(uint32_t u32BitClock);
uint32_t PDCI_GetBitClock(void);
uint32_t PDCI_SetNarrowWidthPct(uint32_t u32NarrowWidth);
uint32_t PDCI_GetFirstNarrowWidth(void);
uint32_t PDCI_SetRxThreshold(uint32_t u32RxThrPct);
uint32_t PDCI_SetDummyDelayPeriod(uint32_t u32ChGroup, uint32_t u32DumDelay);
void PDCI_WriteKcodeData(uint8_t u8Kcode1, uint8_t u8Kcode2, uint8_t u8Kcode3, uint8_t u8Kcode4);
void PDCI_ConfigIO(uint8_t u8SrcGroup, uint8_t u8DstGroup);
void PDCI_EnableInt(uint32_t u32Mask);
void PDCI_DisableInt(uint32_t u32Mask);
uint32_t PDCI_GetIntFlag(uint32_t u32Mask);
uint32_t PDCI_GetStatus(uint32_t u32Mask);
void PDCI_ClearStatus(uint32_t u32Mask);
uint32_t PDCI_GetRxIntFlag(uint32_t u32ChNum, uint32_t u32Mask);
void PDCI_ClearRxIntFlag(uint32_t u32ChNum, uint32_t u32Mask);


/*@}*/ /* end of group PDCI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PDCI_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __PDCI_H__ */
