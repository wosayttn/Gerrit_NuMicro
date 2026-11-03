/**************************************************************************//**
 * @file     chip_type.h
 * @version  V1.00
 * @brief    NuMicro peripheral access layer header file.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __CHIP_TYPE_H__
#define __CHIP_TYPE_H__

/* Select HIRC frequency for M2003G */
#define M2003G_HIRC_40MHZ   (0)
#define M2003G_HIRC_32MHZ   (1)
#define M2003G_HIRC_24MHZ   (2)

#define M2003G_HIRC         (M2003G_HIRC_40MHZ)

/* Select CHIP_TYPE for M2003 series */
#define CHIP_TYPE_M2003C    (0)
#define CHIP_TYPE_M2003G    (1)
#define CHIP_TYPE_M2003E    (2)
#define CHIP_TYPE_M2003D    (3)

#define CHIP_TYPE           (CHIP_TYPE_M2003G)

#endif  /* __CHIP_TYPE_H__ */
