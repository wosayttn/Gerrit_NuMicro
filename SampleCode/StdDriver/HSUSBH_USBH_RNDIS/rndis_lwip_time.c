/**************************************************************************//**
 * @file     rndis_lwip_time.c
 * @brief    NO_SYS=1 lwIP millisecond time source.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdint.h>

#include "lwip/sys.h"

extern uint32_t get_ticks(void);

u32_t sys_now(void)
{
    return (u32_t)(get_ticks() * 10U);
}
