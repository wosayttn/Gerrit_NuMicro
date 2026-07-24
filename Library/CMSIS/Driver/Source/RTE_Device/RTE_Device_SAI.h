/**************************************************************************//**
 * @file     RTE_Device_SAI_I2S.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton M5531 I2S
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
/*
  CMSIS Driver Instance | Nuvoton Hardware Resource
  :---------------------|:--------------------------
  Driver_SAI0           | I2S0
  Driver_SAI1           | I2S1
  Driver_SAI2           | SPI0
  Driver_SAI3           | SPI1
  Driver_SAI4           | SPI2
  Driver_SAI5           | SPI3
*/

#ifndef __RTE_DEVICE_SAI_I2S_H
#define __RTE_DEVICE_SAI_I2S_H

// <e> SAI0 (I2S0)
// <i> Configuration settings for Driver_SAI0 in component ::CMSIS Driver:SAI
#define RTE_SAI0                            1

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI0_RX_PDMA                    1
#define RTE_SAI0_RX_PDMA_PORT               0
#define RTE_SAI0_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI0_TX_PDMA                    1
#define RTE_SAI0_TX_PDMA_PORT               0
#define RTE_SAI0_TX_PDMA_CHANNEL            1

// </e>


// <e> SAI1 (I2S1)
// <i> Configuration settings for Driver_SAI1 in component ::CMSIS Driver:SAI
#define RTE_SAI1                            0

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI1_RX_PDMA                    0
#define RTE_SAI1_RX_PDMA_PORT               0
#define RTE_SAI1_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI1_TX_PDMA                    0
#define RTE_SAI1_TX_PDMA_PORT               0
#define RTE_SAI1_TX_PDMA_CHANNEL            1

// </e>

// <e> SAI2 [SPII2S0]
// <i> Configuration settings for Driver_SAI2 in component ::CMSIS Driver:SAI
#define RTE_SAI2                            1

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI2_RX_PDMA                    1
#define RTE_SAI2_RX_PDMA_PORT               0
#define RTE_SAI2_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI2_TX_PDMA                    1
#define RTE_SAI2_TX_PDMA_PORT               0
#define RTE_SAI2_TX_PDMA_CHANNEL            1

// </e>

// <e> SAI3 [SPII2S1]
// <i> Configuration settings for Driver_SAI3 in component ::CMSIS Driver:SAI
#define RTE_SAI3                            0

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI3_RX_PDMA                    0
#define RTE_SAI3_RX_PDMA_PORT               0
#define RTE_SAI3_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI3_TX_PDMA                    0
#define RTE_SAI3_TX_PDMA_PORT               0
#define RTE_SAI3_TX_PDMA_CHANNEL            1

// </e>

// <e> SAI4 [SPII2S2]
// <i> Configuration settings for Driver_SAI4 in component ::CMSIS Driver:SAI
#define RTE_SAI4                            0

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI4_RX_PDMA                    0
#define RTE_SAI4_RX_PDMA_PORT               0
#define RTE_SAI4_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI4_TX_PDMA                    0
#define RTE_SAI4_TX_PDMA_PORT               0
#define RTE_SAI4_TX_PDMA_CHANNEL            1

// </e>

// <e> SAI5 [SPII2S3]
// <i> Configuration settings for Driver_SAI5 in component ::CMSIS Driver:SAI
#define RTE_SAI5                            0

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI5_RX_PDMA                    0
#define RTE_SAI5_RX_PDMA_PORT               0
#define RTE_SAI5_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SAI5_TX_PDMA                    0
#define RTE_SAI5_TX_PDMA_PORT               0
#define RTE_SAI5_TX_PDMA_CHANNEL            1

// </e>

#endif /* __RTE_DEVICE_SAI_I2S_H */
