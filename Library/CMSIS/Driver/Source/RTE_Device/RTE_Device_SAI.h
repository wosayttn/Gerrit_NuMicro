/**************************************************************************//**
 * @file     RTE_Device_SAI_I2S.h
 * @version  V1.00
 * @brief    RTE Device Configuration for I2S
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
/*
  CMSIS Driver Instance | Nuvoton Hardware Resource
  :---------------------|:--------------------------
  Driver_SAI0           | SPI0
*/

#ifndef __RTE_DEVICE_SAI_I2S_H
#define __RTE_DEVICE_SAI_I2S_H

// <e> SAI0 [SPII2S0]
// <i> Configuration settings for Driver_SAI0 in component ::CMSIS Driver:SAI
#define RTE_SAI0                            1

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI0_RX_PDMA                    0
#define RTE_SAI0_RX_PDMA_PORT               0
#define RTE_SAI0_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI0_TX_PDMA                    0
#define RTE_SAI0_TX_PDMA_PORT               0
#define RTE_SAI0_TX_PDMA_CHANNEL            1

// </e>

#endif /* __RTE_DEVICE_SAI_I2S_H */
