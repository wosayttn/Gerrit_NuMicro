/******************************************************************************
 * @file     usci_spi_transfer.h
 * @brief    USPI ISP slave header file
 * @version  1.0.0
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USPI_TRANS_H__
#define __USPI_TRANS_H__
#include <stdint.h>

extern volatile uint8_t bSpiDataReady;
extern uint32_t spi_rcvbuf[];

/*-------------------------------------------------------------*/
void USPI_Init(void);

#endif  /* __USPI_TRANS_H__ */
