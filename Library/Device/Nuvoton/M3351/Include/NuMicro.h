/**************************************************************************//**
 * @file     NuMicro.h
 * @version  V1.00
 * @brief    NuMicro Peripheral Access Layer Header File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/


#ifndef __NUMICRO_H__
#define __NUMICRO_H__

/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -------------------
*/

// <o> Debug UART port index <0-9>
#define DEBUG_PORT_UART_IDX     0

// <c1> Enable FPGA Emulation Mode
// <i> USE HIRC 12MHz
//#define FPGA_EMU
// </c>

// <c1> Enable Palladium Emulation Mode
// <i> USE HIRC 12MHz
#define PLDM_EMU
// </c>

/*
//-------- <<< End of Use Configuration Section >>> -------------------
*/

#if defined FPGA_EMU && defined PLDM_EMU
    #warning FPGA_EMU and PLDM_EMU are defined at the same time !
#endif

#include "M3351.h"

#endif  /* __NUMICRO_H__ */
