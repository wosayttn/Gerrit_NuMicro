/******************************************************************************
 * @file     config.h
 * @version  V3.00
 * @brief    Define the device setting.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/* Just select one */
#define MEDIAKEY    0
#define JOYSTICK    0

#if MEDIAKEY
#define __HID__
#define __MEDIAKEY__
#elif JOYSTICK
#define __HID__
#define __JOYSTICK__
#endif
