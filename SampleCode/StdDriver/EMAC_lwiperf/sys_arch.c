/*************************************************************************//**
 * @file     sys_arch.c
 * @version  V1.00
 * @brief    System functions for LwIP
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "lwip/tcpip.h"

static volatile uint32_t u32Jiffies = 0;

void NVT_ITCM TIMER0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Jiffies++;
    TIMER_ClearIntFlag(TIMER0);

    /* CPU read interrupt flag register to wait write(clear) instruction completement */
    u32Status = TIMER0->INTSTS;
    (void)u32Status;
}

u32_t sys_now(void)
{
    return u32Jiffies;
}

uint32_t sys_arch_protect(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

void sys_arch_unprotect(uint32_t mask)
{
    __set_PRIMASK(mask);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
