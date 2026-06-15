/**************************************************************************//**
 * @file     RTE_Device_MCI.h
 * @version  V1.00
 * @brief    RTE Device Configuration for SDH
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_MCI_H
#define __RTE_DEVICE_MCI_H

// <e> MCI0 [SDH0]
// <i> Configuration settings for Driver_MCI0 in component ::CMSIS Driver:MCI
#define RTE_MCI0                            1

// <o> SDH0_CD Pin <0=>Disable <1=>Enable
#define   RTE_MCI0_CD_PIN_EN                0

// </e>

// <e> MCI1 [SDH1]
// <i> Configuration settings for Driver_MCI1 in component ::CMSIS Driver:MCI
#define RTE_MCI1                            0

// <o> SDH1_CD Pin <0=>Disable <1=>Enable
#define RTE_MCI1_CD_PIN_EN                  0

// </e>

#endif /* __RTE_DEVICE_MCI_H */
