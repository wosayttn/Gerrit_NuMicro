/**************************************************************************//**
 * @file     RTE_Device_Flash.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton CMSIS Flash driver
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_FLASH_H
#define __RTE_DEVICE_FLASH_H

//  <h> Flash Region
//  <i> Specify the valid Flash Region that Flash Driver can read, write and erase.
//      <o> Start Offset <0x0-0x200000:0x2000>
//      <i> Configure start offset of Flash Region from 0x0 ~ 0x200000 and must align with 0x2000
#define RTE_FLASH_START_OFFSET      0x00100000

//      <o> Byte Size <0x2000-0x200000:0x2000>
//      <i> Configure total byte size of Flash Region from 0x2000 ~ 0x200000 and must align with 0x2000
#define RTE_FLASH_BYTE_SIZE         0x00100000
//  </h>

#endif  // __RTE_DEVICE_FLASH_H
