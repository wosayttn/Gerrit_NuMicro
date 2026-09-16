/**************************************************************************//**
 * @file     cc.h
 * @brief    Minimal bare-metal compiler port for the lwIP target.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

#include <stdint.h>
#include <stdio.h>

#define LWIP_NO_STDINT_H                1

typedef uint8_t u8_t;
typedef int8_t s8_t;
typedef uint16_t u16_t;
typedef int16_t s16_t;
typedef uint32_t u32_t;
typedef int32_t s32_t;
typedef u32_t mem_ptr_t;
typedef u32_t sys_prot_t;

//#define BYTE_ORDER                      LITTLE_ENDIAN

#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT              __attribute__((packed))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x)            x

#define U16_F                           "u"
#define S16_F                           "d"
#define X16_F                           "x"
#define U32_F                           "u"
#define S32_F                           "d"
#define X32_F                           "x"

#define LWIP_PLATFORM_DIAG(x)           do { printf x; } while (0)
#define LWIP_PLATFORM_ASSERT(x)         do { printf("lwIP assertion: %s:%d\n", __FILE__, __LINE__); } while (0)

#endif /* LWIP_ARCH_CC_H */
