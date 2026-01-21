/**************************************************************************//**
 * @file     RTE_Device_USBH.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton M55M1 USB host
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_USBH_H
#define __RTE_DEVICE_USBH_H

// <e> USB HOST EHCI
#define RTE_USB_HOST_EHCI           1
//       <o> Configure Pin For Driving VBUS Active State <0=>Low <1=>High
#define RTE_HS_VBUS_ACTIVE          1
//     <o> Configure Pin for Overcurrent Detection Active State <0=>Low <1=>High
#define RTE_HS_OC_ACTIVE            0

// </e>

// <e> USB HOST OHCI Port 0
#define RTE_USB_HOST_OHCI0           1
//     <o> Configure Pin For Driving VBUS Active State <0=>Low <1=>High
#define RTE_FS_VBUS_PORT0_ACTIVE    1
//     <o> Configure Pin For Overcurrent Detection Active State <0=>Low <1=>High
#define RTE_FS_OC_PORT0_ACTIVE      0

// </e>

// <e> USB HOST OHCI Port 1
#define RTE_USB_HOST_OHCI1          1
//     <o> onfigure Pin For Driving VBUS Active State <0=>Low <1=>High
#define RTE_FS_VBUS_PORT1_ACTIVE    1
//     <o> Configure Pin For Overcurrent Detection Active State <0=>Low <1=>High
#define RTE_FS_OC_PORT1_ACTIVE      0
// </e>

#endif