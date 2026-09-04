/***************************************************************************//**
 * @file     spi_transfer.h
 * @version  V1.00
 * @brief    ISP tool SPI initialization header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __SPI_TRANS_H__
#define __SPI_TRANS_H__
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile uint8_t bSpiDataReady;
extern uint32_t spi_rcvbuf[];

/*-------------------------------------------------------------*/
void SPI_Init(void);

#ifdef __cplusplus
}
#endif

#endif  /* __SPI_TRANS_H__ */

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
