/**************************************************************************//**
 * @file     RTE_Device_ETH_MAC.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton M3351 ETH MAC
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_ETH_MAC_H
#define __RTE_DEVICE_ETH_MAC_H

// <e> ETH (Ethernet Interface) [Driver_ETH_MAC0]
// <i> Configuration settings for Driver_ETH_MAC0 in component ::CMSIS Driver:Ethernet MAC
#define RTE_ETH                         1

//   <e> MII (Media Independent Interface)
#define RTE_ETH_MII                     0

//     <o> ETH_MII_TX_CLK Pin <0=>PC3
#define RTE_ETH_MII_TX_CLK_PORT_ID      0
#if    (RTE_ETH_MII_TX_CLK_PORT_ID == 0)
    #define RTE_ETH_MII_TX_CLK_PORT         GPIOC
    #define RTE_ETH_MII_TX_CLK_PIN          3
#else
    #error "Invalid ETH_MII_TX_CLK Pin Configuration!"
#endif
//     <o> ETH_MII_TXD0 Pin <0=>PB12 <1=>PG13
#define RTE_ETH_MII_TXD0_PORT_ID        0
#if    (RTE_ETH_MII_TXD0_PORT_ID == 0)
    #define RTE_ETH_MII_TXD0_PORT           GPIOB
    #define RTE_ETH_MII_TXD0_PIN            12
#elif  (RTE_ETH_MII_TXD0_PORT_ID == 1)
    #define RTE_ETH_MII_TXD0_PORT           GPIOG
    #define RTE_ETH_MII_TXD0_PIN            13
#else
    #error "Invalid ETH_MII_TXD0 Pin Configuration!"
#endif
//     <o> ETH_MII_TXD1 Pin <0=>PB13 <1=>PG14
#define RTE_ETH_MII_TXD1_PORT_ID        0
#if    (RTE_ETH_MII_TXD1_PORT_ID == 0)
    #define RTE_ETH_MII_TXD1_PORT           GPIOB
    #define RTE_ETH_MII_TXD1_PIN            13
#elif  (RTE_ETH_MII_TXD1_PORT_ID == 1)
    #define RTE_ETH_MII_TXD1_PORT           GPIOG
    #define RTE_ETH_MII_TXD1_PIN            14
#else
    #error "Invalid ETH_MII_TXD1 Pin Configuration!"
#endif
//     <o> ETH_MII_TXD2 Pin <0=>PC2
#define RTE_ETH_MII_TXD2_PORT_ID        0
#if    (RTE_ETH_MII_TXD2_PORT_ID == 0)
    #define RTE_ETH_MII_TXD2_PORT           GPIOC
    #define RTE_ETH_MII_TXD2_PIN            2
#else
    #error "Invalid ETH_MII_TXD2 Pin Configuration!"
#endif
//     <o> ETH_MII_TXD3 Pin <0=>PB8 <1=>PE2
#define RTE_ETH_MII_TXD3_PORT_ID        0
#if    (RTE_ETH_MII_TXD3_PORT_ID == 0)
    #define RTE_ETH_MII_TXD3_PORT           GPIOB
    #define RTE_ETH_MII_TXD3_PIN            8
#elif  (RTE_ETH_MII_TXD3_PORT_ID == 1)
    #define RTE_ETH_MII_TXD3_PORT           GPIOE
    #define RTE_ETH_MII_TXD3_PIN            2
#else
    #error "Invalid ETH_MII_TXD3 Pin Configuration!"
#endif
//     <o> ETH_MII_TX_EN Pin <0=>PB11 <1=>PG11
#define RTE_ETH_MII_TX_EN_PORT_ID       0
#if    (RTE_ETH_MII_TX_EN_PORT_ID == 0)
    #define RTE_ETH_MII_TX_EN_PORT          GPIOB
    #define RTE_ETH_MII_TX_EN_PIN           11
#elif  (RTE_ETH_MII_TX_EN_PORT_ID == 1)
    #define RTE_ETH_MII_TX_EN_PORT          GPIOG
    #define RTE_ETH_MII_TX_EN_PIN           11
#else
    #error "Invalid ETH_MII_TX_EN Pin Configuration!"
#endif
//     <o> ETH_MII_RX_CLK Pin <0=>PA1
#define RTE_ETH_MII_RX_CLK_PORT_ID        0
#if    (RTE_ETH_MII_RX_CLK_PORT_ID == 0)
    #define RTE_ETH_MII_RX_CLK_PORT         GPIOA
    #define RTE_ETH_MII_RX_CLK_PIN          1
#else
    #error "Invalid ETH_MII_RX_CLK Pin Configuration!"
#endif
//     <o> ETH_MII_RXD0 Pin <0=>PC4
#define RTE_ETH_MII_RXD0_PORT_ID        0
#if    (RTE_ETH_MII_RXD0_PORT_ID == 0)
    #define RTE_ETH_MII_RXD0_PORT           GPIOC
    #define RTE_ETH_MII_RXD0_PIN            4
#else
    #error "Invalid ETH_MII_RXD0 Pin Configuration!"
#endif
//     <o> ETH_MII_RXD1 Pin <0=>PC5
#define RTE_ETH_MII_RXD1_PORT_ID        0
#if    (RTE_ETH_MII_RXD1_PORT_ID == 0)
    #define RTE_ETH_MII_RXD1_PORT           GPIOC
    #define RTE_ETH_MII_RXD1_PIN            5
#else
    #error "Invalid ETH_MII_RXD1 Pin Configuration!"
#endif
//     <o> ETH_MII_RXD2 Pin <0=>PB0 <1=>PH6
#define RTE_ETH_MII_RXD2_PORT_ID        0
#if    (RTE_ETH_MII_RXD2_PORT_ID == 0)
    #define RTE_ETH_MII_RXD2_PORT           GPIOB
    #define RTE_ETH_MII_RXD2_PIN            0
#elif  (RTE_ETH_MII_RXD2_PORT_ID == 1)
    #define RTE_ETH_MII_RXD2_PORT           GPIOH
    #define RTE_ETH_MII_RXD2_PIN            6
#else
    #error "Invalid ETH_MII_RXD2 Pin Configuration!"
#endif
//     <o> ETH_MII_RXD3 Pin <0=>PB1 <1=>PH7
#define RTE_ETH_MII_RXD3_PORT_ID        0
#if    (RTE_ETH_MII_RXD3_PORT_ID == 0)
    #define RTE_ETH_MII_RXD3_PORT           GPIOB
    #define RTE_ETH_MII_RXD3_PIN            1
#elif  (RTE_ETH_MII_RXD3_PORT_ID == 1)
    #define RTE_ETH_MII_RXD3_PORT           GPIOH
    #define RTE_ETH_MII_RXD3_PIN            7
#else
    #error "Invalid ETH_MII_RXD3 Pin Configuration!"
#endif
//     <o> ETH_MII_RX_DV Pin <0=>PA7
#define RTE_ETH_MII_RX_DV_PORT_ID       0
#if    (RTE_ETH_MII_RX_DV_PORT_ID == 0)
    #define RTE_ETH_MII_RX_DV_PORT          GPIOA
    #define RTE_ETH_MII_RX_DV_PIN           7
#else
    #error "Invalid ETH_MII_RX_DV Pin Configuration!"
#endif
//     <o> ETH_MII_RX_ER Pin <0=>PB10 <1=>PI10
#define RTE_ETH_MII_RX_ER_PORT_ID       0
#if    (RTE_ETH_MII_RX_ER_PORT_ID == 0)
    #define RTE_ETH_MII_RX_ER_PORT          GPIOB
    #define RTE_ETH_MII_RX_ER_PIN           10
#elif  (RTE_ETH_MII_RX_ER_PORT_ID == 1)
    #define RTE_ETH_MII_RX_ER_PORT          GPIOI
    #define RTE_ETH_MII_RX_ER_PIN           10
#else
    #error "Invalid ETH_MII_RX_ER Pin Configuration!"
#endif
//     <o> ETH_MII_CRS Pin <0=>PA0 <1=>PH2
#define RTE_ETH_MII_CRS_PORT_ID       0
#if    (RTE_ETH_MII_CRS_PORT_ID == 0)
    #define RTE_ETH_MII_CRS_PORT            GPIOA
    #define RTE_ETH_MII_CRS_PIN             0
#elif  (RTE_ETH_MII_CRS_PORT_ID == 1)
    #define RTE_ETH_MII_CRS_PORT            GPIOH
    #define RTE_ETH_MII_CRS_PIN             2
#else
    #error "Invalid ETH_MII_CRS Pin Configuration!"
#endif
//     <o> ETH_MII_COL Pin <0=>PA3 <1=>PH3
#define RTE_ETH_MII_COL_PORT_ID       0
#if    (RTE_ETH_MII_COL_PORT_ID == 0)
    #define RTE_ETH_MII_COL_PORT            GPIOA
    #define RTE_ETH_MII_COL_PIN             3
#elif  (RTE_ETH_MII_COL_PORT_ID == 1)
    #define RTE_ETH_MII_COL_PORT            GPIOH
    #define RTE_ETH_MII_COL_PIN             3
#else
    #error "Invalid ETH_MII_COL Pin Configuration!"
#endif

//   </e>

//   <e> RMII (Reduced Media Independent Interface)
#define RTE_ETH_RMII                    1

//     <o> ETH_RMII_TXD0 Pin <0=>PB12 <1=>PG13
#define RTE_ETH_RMII_TXD0_PORT_ID       0
#if    (RTE_ETH_RMII_TXD0_PORT_ID == 0)
    #define RTE_ETH_RMII_TXD0_PORT          GPIOB
    #define RTE_ETH_RMII_TXD0_PIN           12
#elif  (RTE_ETH_RMII_TXD0_PORT_ID == 1)
    #define RTE_ETH_RMII_TXD0_PORT          GPIOG
    #define RTE_ETH_RMII_TXD0_PIN           13
#else
    #error "Invalid ETH_RMII_TXD0 Pin Configuration!"
#endif
//     <o> ETH_RMII_TXD1 Pin <0=>PB13 <1=>PG14
#define RTE_ETH_RMII_TXD1_PORT_ID       0
#if    (RTE_ETH_RMII_TXD1_PORT_ID == 0)
    #define RTE_ETH_RMII_TXD1_PORT          GPIOB
    #define RTE_ETH_RMII_TXD1_PIN           13
#elif  (RTE_ETH_RMII_TXD1_PORT_ID == 1)
    #define RTE_ETH_RMII_TXD1_PORT          GPIOG
    #define RTE_ETH_RMII_TXD1_PIN           14
#else
    #error "Invalid ETH_RMII_TXD1 Pin Configuration!"
#endif
//     <o> ETH_RMII_TX_EN Pin <0=>PB11 <1=>PG11
#define RTE_ETH_RMII_TX_EN_PORT_ID      0
#if    (RTE_ETH_RMII_TX_EN_PORT_ID == 0)
    #define RTE_ETH_RMII_TX_EN_PORT         GPIOB
    #define RTE_ETH_RMII_TX_EN_PIN          11
#elif  (RTE_ETH_RMII_TX_EN_PORT_ID == 1)
    #define RTE_ETH_RMII_TX_EN_PORT         GPIOG
    #define RTE_ETH_RMII_TX_EN_PIN          11
#else
    #error "Invalid ETH_RMII_TX_EN Pin Configuration!"
#endif
//     <o> ETH_RMII_RXD0 Pin <0=>PC4
#define RTE_ETH_RMII_RXD0_PORT_ID       0
#if    (RTE_ETH_RMII_RXD0_PORT_ID == 0)
    #define RTE_ETH_RMII_RXD0_PORT          GPIOC
    #define RTE_ETH_RMII_RXD0_PIN           4
#else
    #error "Invalid ETH_RMII_RXD0 Pin Configuration!"
#endif
//     <o> ETH_RMII_RXD1 Pin <0=>PC5
#define RTE_ETH_RMII_RXD1_PORT_ID       0
#if    (RTE_ETH_RMII_RXD1_PORT_ID == 0)
    #define RTE_ETH_RMII_RXD1_PORT          GPIOC
    #define RTE_ETH_RMII_RXD1_PIN           5
#else
    #error "Invalid ETH_RMII_RXD1 Pin Configuration!"
#endif
//     <o> ETH_RMII_REF_CLK Pin <0=>PA1
#define RTE_ETH_RMII_REF_CLK_PORT_ID    0
#if    (RTE_ETH_RMII_REF_CLK_PORT_ID == 0)
    #define RTE_ETH_RMII_REF_CLK_PORT       GPIOA
    #define RTE_ETH_RMII_REF_CLK_PIN        1
#else
    #error "Invalid ETH_RMII_REF_CLK Pin Configuration!"
#endif
//     <o> ETH_RMII_CRS_DV Pin <0=>PA7
#define RTE_ETH_RMII_CRS_DV_PORT_ID     0
#if    (RTE_ETH_RMII_CRS_DV_PORT_ID == 0)
    #define RTE_ETH_RMII_CRS_DV_PORT        GPIOA
    #define RTE_ETH_RMII_CRS_DV_PIN         7
#else
    #error "Invalid ETH_RMII_CRS_DV Pin Configuration!"
#endif

//   </e>

//   <h> Management Data Interface
//     <o> ETH_MDC Pin <0=>PC1
#define RTE_ETH_MDI_MDC_PORT_ID         0
#if    (RTE_ETH_MDI_MDC_PORT_ID == 0)
    #define RTE_ETH_MDI_MDC_PORT            GPIOC
    #define RTE_ETH_MDI_MDC_PIN             1
#else
    #error "Invalid ETH_MDC Pin Configuration!"
#endif
//     <o> ETH_MDIO Pin <0=>PA2
#define RTE_ETH_MDI_MDIO_PORT_ID        0
#if    (RTE_ETH_MDI_MDIO_PORT_ID == 0)
    #define RTE_ETH_MDI_MDIO_PORT           GPIOA
    #define RTE_ETH_MDI_MDIO_PIN            2
#else
    #error "Invalid ETH_MDIO Pin Configuration!"
#endif
//   </h>

// </e>

#endif
