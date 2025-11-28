/**************************************************************************//**
 * @file     spi.h
 * @version  V3.00
 * @brief    M2U51 series SPI driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPI_Driver SPI Driver
  @{
*/

/** @addtogroup SPI_EXPORTED_CONSTANTS SPI Exported Constants
  @{
*/

#define SPI_MODE_0        (SPI_CTL_TXNEG_Msk)                             /*!< CLKPOL=0; RXNEG=0; TXNEG=1 \hideinitializer */
#define SPI_MODE_1        (SPI_CTL_RXNEG_Msk)                             /*!< CLKPOL=0; RXNEG=1; TXNEG=0 \hideinitializer */
#define SPI_MODE_2        (SPI_CTL_CLKPOL_Msk | SPI_CTL_RXNEG_Msk)        /*!< CLKPOL=1; RXNEG=1; TXNEG=0 \hideinitializer */
#define SPI_MODE_3        (SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk)        /*!< CLKPOL=1; RXNEG=0; TXNEG=1 \hideinitializer */

#define SPI_SLAVE         (SPI_CTL_SLAVE_Msk)                             /*!< Set as slave \hideinitializer */
#define SPI_MASTER        (0x0U)                                          /*!< Set as master \hideinitializer */

#define SPI_SS                (SPI_SSCTL_SS_Msk)                          /*!< Set SS \hideinitializer */
#define SPI_SS_ACTIVE_HIGH    (SPI_SSCTL_SSACTPOL_Msk)                    /*!< SS active high \hideinitializer */
#define SPI_SS_ACTIVE_LOW     (0x0U)                                      /*!< SS active low \hideinitializer */

/* SPI Interrupt Mask */
#define SPI_UNIT_INT_MASK                (0x001U)                          /*!< Unit transfer interrupt mask \hideinitializer */
#define SPI_SSACT_INT_MASK               (0x002U)                          /*!< Slave selection signal active interrupt mask \hideinitializer */
#define SPI_SSINACT_INT_MASK             (0x004U)                          /*!< Slave selection signal inactive interrupt mask \hideinitializer */
#define SPI_SLVUR_INT_MASK               (0x008U)                          /*!< Slave under run interrupt mask \hideinitializer */
#define SPI_SLVBE_INT_MASK               (0x010U)                          /*!< Slave bit count error interrupt mask \hideinitializer */
#define SPI_TXUF_INT_MASK                (0x040U)                          /*!< Slave TX underflow interrupt mask \hideinitializer */
#define SPI_FIFO_TXTH_INT_MASK           (0x080U)                          /*!< FIFO TX threshold interrupt mask \hideinitializer */
#define SPI_FIFO_RXTH_INT_MASK           (0x100U)                          /*!< FIFO RX threshold interrupt mask \hideinitializer */
#define SPI_FIFO_RXOV_INT_MASK           (0x200U)                          /*!< FIFO RX overrun interrupt mask \hideinitializer */
#define SPI_FIFO_RXTO_INT_MASK           (0x400U)                          /*!< FIFO RX time-out interrupt mask \hideinitializer */

/* SPI Status Mask */
#define SPI_BUSY_MASK                    (0x01U)                           /*!< Busy status mask \hideinitializer */
#define SPI_RX_EMPTY_MASK                (0x02U)                           /*!< RX empty status mask \hideinitializer */
#define SPI_RX_FULL_MASK                 (0x04U)                           /*!< RX full status mask \hideinitializer */
#define SPI_TX_EMPTY_MASK                (0x08U)                           /*!< TX empty status mask \hideinitializer */
#define SPI_TX_FULL_MASK                 (0x10U)                           /*!< TX full status mask \hideinitializer */
#define SPI_TXRX_RESET_MASK              (0x20U)                           /*!< TX or RX reset status mask \hideinitializer */
#define SPI_SPIEN_STS_MASK               (0x40U)                           /*!< SPIEN status mask \hideinitializer */
#define SPI_SSLINE_STS_MASK              (0x80U)                           /*!< SPIx_SS line status mask \hideinitializer */

/* SPI AUTOCTL Trigger Source */
#define SPI_AUTOCTL_TRIGSEL_TMR0      (0UL<<SPI_AUTOCTL_TRIGSEL_Pos)       /*!< SPI Automatic Operation Trigger Source Select is LPTMR0  \hideinitializer */
#define SPI_AUTOCTL_TRIGSEL_TMR1      (1UL<<SPI_AUTOCTL_TRIGSEL_Pos)       /*!< SPI Automatic Operation Trigger Source Select is LPTMR1  \hideinitializer */
#define SPI_AUTOCTL_TRIGSEL_TMR2      (2UL<<SPI_AUTOCTL_TRIGSEL_Pos)       /*!< SPI Automatic Operation Trigger Source Select is TTMR0  \hideinitializer */
#define SPI_AUTOCTL_TRIGSEL_TMR3      (3UL<<SPI_AUTOCTL_TRIGSEL_Pos)       /*!< SPI Automatic Operation Trigger Source Select is TTMR1  \hideinitializer */
#define SPI_AUTOCTL_TRIGSEL_WKIOA0    (4UL<<SPI_AUTOCTL_TRIGSEL_Pos)       /*!< SPI Automatic Operation Trigger Source Select is WKIOA0  \hideinitializer */
#define SPI_AUTOCTL_TRIGSEL_WKIOB0    (5UL<<SPI_AUTOCTL_TRIGSEL_Pos)       /*!< SPI Automatic Operation Trigger Source Select is WKIOB0  \hideinitializer */
#define SPI_AUTOCTL_TRIGSEL_WKIOC0    (6UL<<SPI_AUTOCTL_TRIGSEL_Pos)       /*!< SPI Automatic Operation Trigger Source Select is WKIOC0  \hideinitializer */
#define SPI_AUTOCTL_TRIGSEL_WKIOD0    (7UL<<SPI_AUTOCTL_TRIGSEL_Pos)       /*!< SPI Automatic Operation Trigger Source Select is WKIOD0  \hideinitializer */

/*@}*/ /* end of group SPI_EXPORTED_CONSTANTS */

/** @addtogroup SPI_EXPORTED_MACROS SPI Exported Macros
  @{
*/

/**
  * @brief      Clear the unit transfer interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Write 1 to UNITIF bit of SPI_STATUS register to clear the unit transfer interrupt flag.
  * \hideinitializer
  */
#define SPI_CLR_UNIT_TRANS_INT_FLAG(spi)   ((spi)->STATUS = SPI_STATUS_UNITIF_Msk)

/**
  * @brief      Trigger RX PDMA function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set RXPDMAEN bit of SPI_PDMACTL register to enable RX PDMA transfer function.
  * \hideinitializer
  */
#define SPI_TRIGGER_RX_PDMA(spi)   ((spi)->PDMACTL |= SPI_PDMACTL_RXPDMAEN_Msk)

/**
  * @brief      Trigger TX PDMA function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set TXPDMAEN bit of SPI_PDMACTL register to enable TX PDMA transfer function.
  * \hideinitializer
  */
#define SPI_TRIGGER_TX_PDMA(spi)   ((spi)->PDMACTL |= SPI_PDMACTL_TXPDMAEN_Msk)

/**
  * @brief      Trigger TX and RX PDMA function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set TXPDMAEN bit and RXPDMAEN bit of SPI_PDMACTL register to enable TX and RX PDMA transfer function.
  * \hideinitializer
  */
#define SPI_TRIGGER_TX_RX_PDMA(spi)   ((spi)->PDMACTL |= (SPI_PDMACTL_TXPDMAEN_Msk | SPI_PDMACTL_RXPDMAEN_Msk))

/**
  * @brief      Disable RX PDMA transfer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear RXPDMAEN bit of SPI_PDMACTL register to disable RX PDMA transfer function.
  * \hideinitializer
  */
#define SPI_DISABLE_RX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI_PDMACTL_RXPDMAEN_Msk )

/**
  * @brief      Disable TX PDMA transfer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear TXPDMAEN bit of SPI_PDMACTL register to disable TX PDMA transfer function.
  * \hideinitializer
  */
#define SPI_DISABLE_TX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI_PDMACTL_TXPDMAEN_Msk )

/**
  * @brief      Disable TX and RX PDMA transfer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear TXPDMAEN bit and RXPDMAEN bit of SPI_PDMACTL register to disable TX and RX PDMA transfer function.
  * \hideinitializer
  */
#define SPI_DISABLE_TX_RX_PDMA(spi) ( (spi)->PDMACTL &= ~(SPI_PDMACTL_TXPDMAEN_Msk | SPI_PDMACTL_RXPDMAEN_Msk) )

/**
  * @brief      Get the count of available data in RX FIFO.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     The count of available data in RX FIFO.
  * @details    Read RXCNT (SPI_STATUS[27:24]) to get the count of available data in RX FIFO.
  * \hideinitializer
  */
#define SPI_GET_RX_FIFO_COUNT(spi)   (((spi)->STATUS & SPI_STATUS_RXCNT_Msk) >> SPI_STATUS_RXCNT_Pos)

/**
  * @brief      Get the RX FIFO empty flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 RX FIFO is not empty.
  * @retval     1 RX FIFO is empty.
  * @details    Read RXEMPTY bit of SPI_STATUS register to get the RX FIFO empty flag.
  * \hideinitializer
  */
#define SPI_GET_RX_FIFO_EMPTY_FLAG(spi)   (((spi)->STATUS & SPI_STATUS_RXEMPTY_Msk)>>SPI_STATUS_RXEMPTY_Pos)

/**
  * @brief      Get the TX FIFO empty flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 TX FIFO is not empty.
  * @retval     1 TX FIFO is empty.
  * @details    Read TXEMPTY bit of SPI_STATUS register to get the TX FIFO empty flag.
  * \hideinitializer
  */
#define SPI_GET_TX_FIFO_EMPTY_FLAG(spi)   (((spi)->STATUS & SPI_STATUS_TXEMPTY_Msk)>>SPI_STATUS_TXEMPTY_Pos)

/**
  * @brief      Get the TX FIFO full flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 TX FIFO is not full.
  * @retval     1 TX FIFO is full.
  * @details    Read TXFULL bit of SPI_STATUS register to get the TX FIFO full flag.
  * \hideinitializer
  */
#define SPI_GET_TX_FIFO_FULL_FLAG(spi)   (((spi)->STATUS & SPI_STATUS_TXFULL_Msk)>>SPI_STATUS_TXFULL_Pos)

/**
  * @brief      Get the datum read from RX register.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     Data in RX register.
  * @details    Read SPI_RX register to get the received datum.
  * \hideinitializer
  */
#define SPI_READ_RX(spi)   ((spi)->RX)

/**
  * @brief      Write datum to TX register.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32TxData The datum which user attempt to transfer through SPI bus.
  * @return     None.
  * @details    Write u32TxData to SPI_TX register.
  * \hideinitializer
  */
#define SPI_WRITE_TX(spi, u32TxData)   ((spi)->TX = (u32TxData))

/**
  * @brief      Set SPIx_SS pin to high state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIx_SS pin to high state.
  * \hideinitializer
  */
#define SPI_SET_SS_HIGH(spi)   ((spi)->SSCTL = ((spi)->SSCTL & (~SPI_SSCTL_AUTOSS_Msk)) | (SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS_Msk))

/**
  * @brief      Set SPIx_SS pin to low state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIx_SS pin to low state.
  * \hideinitializer
  */
#define SPI_SET_SS_LOW(spi)   ((spi)->SSCTL = ((spi)->SSCTL & (~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SSACTPOL_Msk))) | SPI_SSCTL_SS_Msk)

/**
  * @brief      Enable Byte Reorder function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Enable Byte Reorder function. The suspend interval depends on the setting of SUSPITV (SPI_CTL[7:4]).
  * \hideinitializer
  */
#define SPI_ENABLE_BYTE_REORDER(spi)   ((spi)->CTL |=  SPI_CTL_REORDER_Msk)

/**
  * @brief      Disable Byte Reorder function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear REORDER bit field of SPI_CTL register to disable Byte Reorder function.
  * \hideinitializer
  */
#define SPI_DISABLE_BYTE_REORDER(spi)   ((spi)->CTL &= ~SPI_CTL_REORDER_Msk)

/**
  * @brief      Set the length of suspend interval.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32SuspCycle Decides the length of suspend interval. It could be 0 ~ 15.
  * @return     None.
  * @details    Set the length of suspend interval according to u32SuspCycle.
  *             The length of suspend interval is ((u32SuspCycle + 0.5) * the length of one SPI bus clock cycle).
  * \hideinitializer
  */
#define SPI_SET_SUSPEND_CYCLE(spi, u32SuspCycle)   ((spi)->CTL = ((spi)->CTL & ~SPI_CTL_SUSPITV_Msk) | ((u32SuspCycle) << SPI_CTL_SUSPITV_Pos))

/**
  * @brief      Set the SPI transfer sequence with LSB first.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set LSB bit of SPI_CTL register to set the SPI transfer sequence with LSB first.
  * \hideinitializer
  */
#define SPI_SET_LSB_FIRST(spi)   ((spi)->CTL |= SPI_CTL_LSB_Msk)

/**
  * @brief      Set the SPI transfer sequence with MSB first.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear LSB bit of SPI_CTL register to set the SPI transfer sequence with MSB first.
  * \hideinitializer
  */
#define SPI_SET_MSB_FIRST(spi)   ((spi)->CTL &= ~SPI_CTL_LSB_Msk)

/**
  * @brief      Set the data width of a SPI transaction.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Width The bit width of one transaction.
  * @return     None.
  * @details    The data width can be 8 ~ 32 bits.
  * \hideinitializer
  */
#define SPI_SET_DATA_WIDTH(spi, u32Width)   ((spi)->CTL = ((spi)->CTL & ~SPI_CTL_DWIDTH_Msk) | (((u32Width)&0x1F) << SPI_CTL_DWIDTH_Pos))

/**
  * @brief      Get the SPI busy state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 SPI controller is not busy.
  * @retval     1 SPI controller is busy.
  * @details    This macro will return the busy state of SPI controller.
  * \hideinitializer
  */
#define SPI_IS_BUSY(spi)   ( ((spi)->STATUS & SPI_STATUS_BUSY_Msk)>>SPI_STATUS_BUSY_Pos )

/**
  * @brief      Enable SPI controller.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set SPIEN (SPI_CTL[0]) to enable SPI controller.
  * \hideinitializer
  */
#define SPI_ENABLE(spi)   ((spi)->CTL |= SPI_CTL_SPIEN_Msk)

/**
  * @brief      Disable SPI controller.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear SPIEN (SPI_CTL[0]) to disable SPI controller.
  * \hideinitializer
  */
#define SPI_DISABLE(spi)   ((spi)->CTL &= ~SPI_CTL_SPIEN_Msk)

/**
  * @brief      Set Auto Operation RX transfer count.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Tcnt The transfer count specified in RX phase.
  * @return     None.
  * @details    Set RX transfer count (SPI_AUTOCTL[23:16]).
  * \hideinitializer
  */
#define SPI_SET_AUTO_RX_TCNT(spi, u32Tcnt)   ((spi)->AUTOCTL &= ~SPI_AUTOCTL_TCNT_Msk); \
                                                 ((spi)->AUTOCTL |= ((u32Tcnt&0xFF) << SPI_AUTOCTL_TCNT_Pos))

/**
  * @brief      Enable RX TCNT count match wake up.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set CNTWKEN (SPI_AUTOCTL[10]) to enable RX TCNT count match wake up function.
  * \hideinitializer
  */
#define SPI_ENABLE_AUTO_CNT_WAKEUP(spi)   ((spi)->AUTOCTL |= SPI_AUTOCTL_CNTWKEN_Msk)

/**
  * @brief      Disable RX TCNT count match wake up.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear CNTWKEN (SPI_AUTOCTL[10]) to disable RX TCNT count match wake up function.
  * \hideinitializer
  */
#define SPI_DISABLE_AUTO_CNT_WAKEUP(spi)   ((spi)->AUTOCTL &= ~SPI_AUTOCTL_CNTWKEN_Msk)

/**
  * @brief      Enable Software Trigger for Auto Operation.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set SWTRIG (SPI_AUTOCTL[9], Write Only bit) to enable Software Trigger in Auto Operation Mode.
  * \hideinitializer
  */
#define SPI_ENABLE_AUTO_SW_TRIG(spi)   ((spi)->AUTOCTL |= SPI_AUTOCTL_SWTRIG_Msk)

/**
  * @brief      Disable Software Trigger for Auto Operation.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear SWTRIG (SPI_AUTOCTL[9], Write Only bit) to disable Software Trigger in Auto Operation Mode.
  * \hideinitializer
  */
#define SPI_DISABLE_AUTO_SW_TRIG(spi)   ((spi)->AUTOCTL &= ~SPI_AUTOCTL_SWTRIG_Msk)

/**
  * @brief      Enable SPI Auto Operation Mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set AUTOEN (SPI_AUTOCTL[8]) to enable Auto Operation Mode.
  * \hideinitializer
  */
#define SPI_ENABLE_AUTO(spi)   ((spi)->AUTOCTL |= SPI_AUTOCTL_AUTOEN_Msk)

/**
  * @brief      Disable SPI Auto Operation Mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear AUTOEN (SPI_AUTOCTL[8]) to disable Auto Operation Mode.
  * \hideinitializer
  */
#define SPI_DISABLE_AUTO(spi)   ((spi)->AUTOCTL &= ~SPI_AUTOCTL_AUTOEN_Msk)

/**
  * @brief      Enable Slave Selection wake up.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set SSWKEN (SPI_AUTOCTL[7]) to enable Slave Selection wake up function. This bit is not related to Auto Operation.
  * \hideinitializer
  */
#define SPI_ENABLE_SS_WAKEUP(spi)   ((spi)->AUTOCTL |= SPI_AUTOCTL_SSWKEN_Msk)

/**
  * @brief      Disable Slave Selection wake up.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear SSWKEN (SPI_AUTOCTL[7]) to disable Slave Selection wake up function. This bit is not related to Auto Operation.
  * \hideinitializer
  */
#define SPI_DISABLE_SS_WAKEUP(spi)   ((spi)->AUTOCTL &= ~SPI_AUTOCTL_SSWKEN_Msk)

/**
  * @brief      Enable RX function in Auto Opertion TX phase.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set FULLRXEN (SPI_AUTOCTL[6]) to enable RX function in Auto Operation TX phase.
  * \hideinitializer
  */
#define SPI_ENABLE_AUTO_FULLRX(spi)   ((spi)->AUTOCTL |= SPI_AUTOCTL_FULLRXEN_Msk)

/**
  * @brief      Disable RX function in Auto Opertion TX phase.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear FULLRXEN (SPI_AUTOCTL[6]) to disable RX function in Auto Operation TX phase.
  * \hideinitializer
  */
#define SPI_DISABLE_AUTO_FULLRX(spi)   ((spi)->AUTOCTL &= ~SPI_AUTOCTL_FULLRXEN_Msk)

/**
  * @brief      Enable RX TCNT count match Interrupt.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set CNTIEN (SPI_AUTOCTL[5]) to enable RX TCNT count match interrpu up function.
  * \hideinitializer
  */
#define SPI_ENABLE_AUTO_CNT_INT(spi)   ((spi)->AUTOCTL |= SPI_AUTOCTL_CNTIEN_Msk)

/**
  * @brief      Disable RX TCNT count match Interrupt.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear CNTIEN (SPI_AUTOCTL[5]) to disable RX TCNT count match interrpu up function.
  * \hideinitializer
  */
#define SPI_DISABLE_AUTO_CNT_INT(spi)   ((spi)->AUTOCTL &= ~SPI_AUTOCTL_CNTIEN_Msk)

/**
  * @brief      Enable Trigger function in Auto Opertion Mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set TRIGEN (SPI_AUTOCTL[4]) to enable Trigger function for Trigger source from LPTMR0/1, TTMR0/1 and LPGPIO0/1/2/3 
  *				in Auto Operation Mode.
  * \hideinitializer
  */
#define SPI_ENABLE_AUTO_TRIG(spi)   ((spi)->AUTOCTL |= SPI_AUTOCTL_TRIGEN_Msk)

/**
  * @brief      Disable Trigger function in Auto Opertion Mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear TRIGEN (SPI_AUTOCTL[4]) to disable Trigger function for Trigger source from LPTMR0/1, TTMR0/1 and LPGPIO0/1/2/3 
  *				in Auto Operation Mode.
  * \hideinitializer
  */
#define SPI_DISABLE_AUTO_TRIG(spi)   ((spi)->AUTOCTL &= ~SPI_AUTOCTL_TRIGEN_Msk)

/**
  * @brief      Set Trigger Source Selection for Auto Operation Mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32TrigSrc The triggered source specified in Auto Operation Mode.
  *                        This parameter could be only one of these following selections:
  *                          - \ref SPI_AUTOCTL_TRIGSEL_LPTMR0
  *                          - \ref SPI_AUTOCTL_TRIGSEL_LPTMR1
  *                          - \ref SPI_AUTOCTL_TRIGSEL_TTMR0
  *                          - \ref SPI_AUTOCTL_TRIGSEL_TTMR1
  *                          - \ref SPI_AUTOCTL_TRIGSEL_WKIOA0
  *                          - \ref SPI_AUTOCTL_TRIGSEL_WKIOB0
  *                          - \ref SPI_AUTOCTL_TRIGSEL_WKIOC0
  *                          - \ref SPI_AUTOCTL_TRIGSEL_WKIOD0
  * @return     None.
  * @details    Set Trigger Souce Selection (SPI_AUTOCTL[3:0]).
  * \hideinitializer
  */
#define SPI_SET_AUTO_TRIG_SOURCE(spi, u32TrigSrc)   ((spi)->AUTOCTL &= ~SPI_AUTOCTL_TRIGSEL_Msk); \
                                                        ((spi)->AUTOCTL |= ((u32TrigSrc&0x0F) << SPI_AUTOCTL_TRIGSEL_Pos))

/**
  * @brief      Get TCNT count match wake up flag in Auto Operation Mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 System is not woken up by CNT match in Auto Operation Mode.
  * @retval     1 System is woken up by CNT match in Auto Operation Mode.
  * @details    Read CNTWKF (SPI_AUTOSTS[3]) register to get the CNTWK flag.
  * \hideinitializer
  */
#define SPI_GET_AUTO_CNTWK_FLAG(spi)   (((spi)->AUTOSTS & SPI_AUTOSTS_CNTWKF_Msk)>>SPI_AUTOSTS_CNTWKF_Pos)

/**
  * @brief      Clear TCNT count match wake up flag in Auto Operation Mode
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Write 1 to CNTWKF bit of SPI_AUTOSTATUS register to clear CNTWK flag.
  * \hideinitializer
  */
#define SPI_CLR_AUTO_CNTWK_FLAG(spi)   ((spi)->AUTOSTS = SPI_AUTOSTS_CNTWKF_Msk)

/**
  * @brief      Get Auto Busy flag in Auto Operation Mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 No more request from triiger source during Auto Operation Mode
  * @retval     1 One more request from triiger source during Auto Operation Mode
  * @details    Read AOBUSY (SPI_AUTOSTS[2]) register to get the AOBUSY flag.
  * \hideinitializer
  */
#define SPI_GET_AUTO_AOBUSY_FLAG(spi)   (((spi)->AUTOSTS & SPI_AUTOSTS_AOBUSY_Msk)>>SPI_AUTOSTS_AOBUSY_Pos)

/**
  * @brief      Clear Auto Busy flag in Auto Operation Mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Write 1 to AOBUSY bit of SPI_AUTOSTATUS register to clear AOBUSY flag.
  * \hideinitializer
  */
#define SPI_CLR_AUTO_AOBUSY_FLAG(spi)   ((spi)->AUTOSTS = SPI_AUTOSTS_AOBUSY_Msk)

/**
  * @brief      Get SS wake up flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 System is not woken up by Slave Select flag.
  * @retval     1 System is woken up by Slave Select flag.
  * @details    Read SSWKF (SPI_AUTOSTS[1]) register to get the SSWKF flag.
  * \hideinitializer
  */
#define SPI_GET_SSWK_FLAG(spi)   (((spi)->AUTOSTS & SPI_AUTOSTS_SSWKF_Msk)>>SPI_AUTOSTS_SSWKF_Pos)

/**
  * @brief      Clear SS wake up flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Write 1 to SSWKF bit of SPI_AUTOSTATUS register to clear SSWKF flag.
  * \hideinitializer
  */
#define SPI_CLR_SSWK_FLAG(spi)   ((spi)->AUTOSTS = SPI_AUTOSTS_SSWKF_Msk)

/**
  * @brief      Get TCNT count match interrupt flag in Auto Operation Mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 RX CNT is not matched in Auto Operation Mode.
  * @retval     1 RX CNT is matched in Auto Operation Mode.
  * @details    Read CNTIF (SPI_AUTOSTS[0]) register to get the CNTIF flag.
  * \hideinitializer
  */
#define SPI_GET_AUTO_CNT_INT_FLAG(spi)   (((spi)->AUTOSTS & SPI_AUTOSTS_CNTIF_Msk)>>SPI_AUTOSTS_CNTIF_Pos)

/**
  * @brief      Clear TCNT count match interrupt flag in Auto Operation Mode
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Write 1 to CNTIF bit of SPI_AUTOSTATUS register to clear CNTIF flag.
  * \hideinitializer
  */
#define SPI_CLR_AUTO_CNT_INT_FLAG(spi)   ((spi)->AUTOSTS = SPI_AUTOSTS_CNTIF_Msk)

/*@}*/ /* end of group SPI_EXPORTED_MACROS */


/** @addtogroup SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/
/* Function prototype declaration */
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_SetFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);

uint32_t SPII2S_Open(SPI_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat);
void SPII2S_Close(SPI_T *i2s);
void SPII2S_EnableInt(SPI_T *i2s, uint32_t u32Mask);
void SPII2S_DisableInt(SPI_T *i2s, uint32_t u32Mask);
uint32_t SPII2S_EnableMCLK(SPI_T *i2s, uint32_t u32BusClock);
void SPII2S_DisableMCLK(SPI_T *i2s);
void SPII2S_SetFIFO(SPI_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold);


/*@}*/ /* end of group SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SPI_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
