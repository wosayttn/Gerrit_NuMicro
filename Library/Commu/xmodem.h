/******************************************************************************
 * @file     xmodem.h
 * @version  V1.00
 * @brief    Xmodem transfer
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef _XMODEM_H
#define _XMODEM_H

#ifndef XMD_UART_PORT
    #define XMD_UART_PORT       DEBUG_PORT
#endif

/* Xmodem Standard Commands */
#define XMD_SOH                 0x01
#define XMD_STX                 0x02
#define XMD_EOT                 0x04
#define XMD_ACK                 0x06
#define XMD_NAK                 0x15
#define XMD_CAN                 0x18
#define XMD_CTRLZ               0x1A
#define XMD_MAX_TIMEOUT         0x600

/* Xmodem Status */
#define XMD_STS_SUCCESS         0
#define XMD_STS_USER_CANCEL     -1
#define XMD_STS_NAK             -2
#define XMD_STS_TIMEOUT         -3
#define XMD_STS_PACKET_NUM_ERR  -4
#define XMD_STS_WRITE_FAIL      -5
#define XMD_STS_BUF_OP_FAIL     -6

/* Xmodem Send Status (aliases for API readability) */
#define XMD_STS_SEND_USER_CANCEL    XMD_STS_USER_CANCEL
#define XMD_STS_SEND_NO_SYNC        XMD_STS_NAK
#define XMD_STS_SEND_XMIT_ERR       XMD_STS_PACKET_NUM_ERR
#define XMD_STS_SEND_EOT_ACK_FAIL   XMD_STS_WRITE_FAIL

#define MAXRETRANS              25

int32_t XmodemRecv(uint32_t u32DestAddr);
int32_t XmodemSend(const uint8_t *pu8SrcBuf, int32_t i32SrcSize);

#endif
