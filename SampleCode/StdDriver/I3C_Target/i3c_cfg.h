/**************************************************************************//**
 * @file     i3c_cfg.c
 * @version  V3.00
 * @brief    i3c_cfg header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/
#define DA_ASSIGNED_MODE        (0)

// <c1> Enable Interrupt Debug Log
// <i> Show detail interrupt status on UART.
//#define DGBINT          printf
//</c>

#ifndef DGBINT
#define DGBINT(...)
#endif
// *** <<< end of configuration section >>>    ***


#define DEVICE_CONTROLLER_ROLE  (0)
#define DEVICE_DMA_ENABLED      (1)


#define I3C0_CTR_DA         (0x4A)
#define I3C0_CTR_MID        (0x5A00UL)
#define I3C0_CTR_PID        (0xA13579BDUL)

#define I3C0_TGT_SA         (0x17)
#define I3C0_TGT_MID        (0xA501UL)
#define I3C0_TGT_PID        (0xB2468ACEUL)
#define I3C0_VT1_SA         (0x27)
#define I3C0_VT1_MID        (I3C0_TGT_MID + 1)
#define I3C0_VT1_PID        (I3C0_TGT_PID + 1)
#define I3C0_VT2_SA         (0x37)
#define I3C0_VT2_MID        (I3C0_TGT_MID + 2)
#define I3C0_VT2_PID        (I3C0_TGT_PID + 2)

extern const uint8_t        g_TgtDA[];
extern volatile uint32_t    g_I3CDevRx[I3C_DEVICE_RX_BUF_CNT], g_I3CDevTx[I3C_DEVICE_TX_BUF_CNT];
