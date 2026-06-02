/******************************************************************************
 * @file     xmodem.c
 * @version  V1.00
 * @brief    Xmodem transfer
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "xmodem.h"

#define XMD_MAX_TRANS_SIZE      (1024 * 1024)

/* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
static uint8_t s_au8XmdBuf[1030];

/* To program data from Xmodem transfer */
static int32_t XMD_Write8Bytes(uint32_t u32Addr, uint32_t u32Data0, uint32_t u32Data1)
{
    uint32_t u32TimeOutCnt;

    FMC->ISPADDR = u32Addr;

    if ((u32Addr & (FMC_FLASH_PAGE_SIZE - 1U)) == 0U)
    {
        FMC->ISPCMD   = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPTRG   = FMC_ISPTRG_ISPGO_Msk;
        u32TimeOutCnt = FMC_TIMEOUT_ERASE;

        while (FMC->ISPTRG)
        {
            if (--u32TimeOutCnt == 0)
            {
                return -1;
            }
        }
    }

    FMC->MPDAT0 = u32Data0;
    FMC->MPDAT1 = u32Data1;
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM_64;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    u32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while (FMC->ISPTRG)
    {
        if (--u32TimeOutCnt == 0)
        {
            return -1;
        }
    }

    return 0;
}

static void XMD_putc(uint8_t u8Data)
{
    UART_T *psUart = XMD_UART_PORT;
    uint32_t u32TimeOutCnt;

    u32TimeOutCnt = FMC_TIMEOUT_ERASE;

    while (psUart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk)
    {
        if (--u32TimeOutCnt == 0)
        {
            return ;
        }
    }

    psUart->DAT = u8Data;

}

static int32_t XMD_getc(void)
{
    const UART_T *psUart = XMD_UART_PORT;
    uint32_t u32Ms = 0;

    /* Wait for 100ms */
    while (u32Ms < 100U)
    {
        SysTick->CTRL = 0;
        SysTick->LOAD = 1000U * CyclesPerUs; /* 1ms */
        SysTick->VAL  = (0x0UL);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
        {
            if ((psUart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != UART_FIFOSTS_RXEMPTY_Msk)
            {
                SysTick->CTRL = 0;
                return ((int32_t)psUart->DAT);
            }
        }

        u32Ms++;
    }

    SysTick->CTRL = 0;
    return -1; /* time-out */
}

static uint16_t crc16_ccitt(const uint8_t *pu8Buf, int32_t i32Len)
{
    uint16_t u16Crc = 0;
    int32_t  i32DataByteCnt = i32Len;
    const uint8_t *pu8Data = pu8Buf;

    while (i32DataByteCnt--)
    {
        int32_t i32Idx;
        u16Crc ^= (uint16_t)(*pu8Data++ << 8);

        for (i32Idx = 0; i32Idx < 8; i32Idx++)
        {
            if (u16Crc & 0x8000U)
            {
                u16Crc = (uint16_t)(u16Crc << 1) ^ (uint16_t)0x1021;
            }
            else
            {
                u16Crc = (uint16_t)(u16Crc << 1);
            }
        }
    }

    return u16Crc;
}

static int32_t check(int32_t i32IsCrc, const uint8_t *pu8Buf, int32_t i32Size)
{
    if (i32IsCrc)
    {
        uint16_t u16Crc = crc16_ccitt(pu8Buf, i32Size);
        uint16_t u16TargetCrc = (uint16_t)(pu8Buf[i32Size] << 8) + (uint16_t)pu8Buf[i32Size + 1];

        if (u16Crc == u16TargetCrc)
        {
            return 1;
        }
    }
    else
    {
        int32_t i32Idx;
        uint8_t u8TargetSum = 0;

        for (i32Idx = 0; i32Idx < i32Size; i32Idx++)
        {
            u8TargetSum += pu8Buf[i32Idx];
        }

        if (u8TargetSum == pu8Buf[i32Size])
        {
            return 1;
        }
    }

    return 0;
}

/**
  * @brief      Recive data from UART Xmodem transfer and program the data to flash.
  * @param[in]  u32DestAddr Destination address of flash to program.
  * @return     Recived data size if successful. Return < 0 when error.
  *
  * @details    This function is used to recieve UART data through Xmodem transfer.
  *             The received data will be programmed to flash packet by packet.
  */
int32_t XmodemRecv(uint32_t u32DestAddr)
{
    int32_t i32Err = 0;
    int32_t i32UseCrc = 0;
    char    cTryChar = 'C';
    uint8_t u8PacketNo = 1;
    int32_t i32Idx;
    int32_t i32BlkIdx;
    int32_t i32Retrans = MAXRETRANS;
    int32_t i32TransBytes = 0;
    int32_t i32Char;
    uint32_t au32WriteData[2];

    while (1)
    {
        int32_t i32BufSize;
        int32_t i32StartChar;
        int32_t i32PayloadReadSize;
        int32_t i32RejectPacket;

        i32StartChar = -1;
        i32BufSize = 0;

        for (i32Idx = 0; i32Idx < XMD_MAX_TIMEOUT; i32Idx++) /* set timeout period */
        {
            if (cTryChar)
            {
                XMD_putc(cTryChar);
            }

            i32Char = XMD_getc();

            if (i32Char >= 0)
            {
                switch (i32Char)
                {
                    case XMD_SOH:
                        i32BufSize = 128;
                        i32StartChar = i32Char;
                        break;

                    case XMD_STX:
                        i32BufSize = 1024;
                        i32StartChar = i32Char;
                        break;

                    case XMD_EOT:
                        XMD_putc(XMD_ACK);
                        return (i32Err == 0) ? i32TransBytes : i32Err; /* normal end */

                    case XMD_CAN:
                        XMD_putc(XMD_ACK);
                        return XMD_STS_USER_CANCEL; /* canceled by remote */

                    default:
                        break;
                }

                if (i32StartChar >= 0)
                {
                    break;
                }
            }
        }

        if (i32StartChar < 0)
        {
            if (cTryChar == 'C')
            {
                XMD_putc(XMD_CAN);
                XMD_putc(XMD_CAN);
                XMD_putc(XMD_CAN);
                return XMD_STS_TIMEOUT; /* too many retry error */
            }

            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            return XMD_STS_NAK; /* sync error */
        }

        if (cTryChar == 'C')
        {
            i32UseCrc = 1;
        }

        cTryChar = 0;
        s_au8XmdBuf[0] = (uint8_t)i32StartChar;

        i32PayloadReadSize = i32BufSize + (i32UseCrc ? 1 : 0) + 3;
        i32RejectPacket = 0;

        for (i32Idx = 1; i32Idx <= i32PayloadReadSize; i32Idx++)
        {
            i32Char = XMD_getc();

            if (i32Char < 0)
            {
                i32RejectPacket = 1;
                break;
            }

            s_au8XmdBuf[i32Idx] = (uint8_t)i32Char;
        }

        if (i32RejectPacket != 0)
        {
            XMD_putc(XMD_NAK);
            continue;
        }

        if (s_au8XmdBuf[1] != u8PacketNo)
        {
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            return XMD_STS_PACKET_NUM_ERR;
        }
        else
        {
            if (((s_au8XmdBuf[1] + s_au8XmdBuf[2]) == 0xFFU) && check(i32UseCrc, &s_au8XmdBuf[3], i32BufSize))
            {
                if (s_au8XmdBuf[1] == u8PacketNo)
                {
                    volatile int32_t i32RemainCount = XMD_MAX_TRANS_SIZE - i32TransBytes;

                    if (i32RemainCount > i32BufSize)
                    {
                        i32RemainCount = i32BufSize;
                    }

                    if (i32RemainCount > 0)
                    {
                        for (i32BlkIdx = 0; i32BlkIdx < (i32BufSize + 3) / 8; i32BlkIdx++)
                        {
                            if (memcpy((uint8_t *)au32WriteData, &s_au8XmdBuf[3 + (i32BlkIdx * 0x8)], 8) != (void *)au32WriteData)
                            {
                                break;
                            }

                            i32Err = XMD_Write8Bytes((u32DestAddr + (uint32_t)i32TransBytes) + ((uint32_t)i32BlkIdx * 0x8U), au32WriteData[0], au32WriteData[1]);

                            if (i32Err < 0)
                            {
                                continue;
                            }
                        }

                        i32TransBytes += i32RemainCount;
                    }

                    u8PacketNo++;
                    i32Retrans = MAXRETRANS + 1;
                }

                if (--i32Retrans <= 0)
                {
                    XMD_putc(XMD_CAN);
                    XMD_putc(XMD_CAN);
                    XMD_putc(XMD_CAN);
                    return XMD_STS_TIMEOUT; /* too many retry error */
                }

                XMD_putc(XMD_ACK);
                continue;
            }
        }

        XMD_putc(XMD_NAK);
    }
}

/**
  * @brief      Send data by UART Xmodem transfer.
  * @param[in]  pu8SrcBuf    Address of the source data to transfer.
  * @param[in]  i32SrcSize   Size of the total size to transfer.
  * @retval     Total transfer size when successfull.
  * @retval     XMD_STS_SEND_USER_CANCEL   Canceled by remote.
  * @retval     XMD_STS_SEND_NO_SYNC       No sync character received.
  * @retval     XMD_STS_SEND_XMIT_ERR      Transmit error.
  * @retval     XMD_STS_SEND_EOT_ACK_FAIL  EOT phase did not receive ACK.
  * @retval     XMD_STS_BUF_OP_FAIL        Buffer operation failed.
  * @details    This function is used to send UART data through Xmodem transfer.
  *
  */
int32_t XmodemSend(const uint8_t *pu8SrcBuf, int32_t i32SrcSize)
{
    int32_t i32UseCrc = -1;
    uint8_t u8PacketNo = 1;
    int32_t i32Idx;
    int32_t i32Char;
    int32_t i32Len = 0;
    int32_t i32Retry;

    while (1)
    {
        int32_t  i32SyncOk;

        i32SyncOk = 0;

        for (i32Retry = 0; i32Retry < 160; i32Retry++)
        {
            i32Char = XMD_getc();

            if (i32Char >= 0)
            {
                switch (i32Char)
                {
                    case 'C':
                        i32UseCrc = 1;
                        i32SyncOk = 1;
                        break;

                    case XMD_NAK:
                        i32UseCrc = 0;
                        i32SyncOk = 1;
                        break;

                    case XMD_CAN:
                        i32Char = XMD_getc();

                        if (i32Char == XMD_CAN)
                        {
                            XMD_putc(XMD_ACK);

                            return XMD_STS_SEND_USER_CANCEL;
                        }

                        break;

                    default:
                        break;
                }

                if (i32SyncOk != 0)
                {
                    break;
                }
            }
        }

        if (i32SyncOk == 0)
        {
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);

            return XMD_STS_SEND_NO_SYNC;
        }

        while (1)
        {
            int32_t  i32BufSize;

            i32BufSize = 128;
            s_au8XmdBuf[0] = XMD_SOH;
            s_au8XmdBuf[1] = u8PacketNo;
            s_au8XmdBuf[2] = (uint8_t)(~u8PacketNo);
            i32Char = i32SrcSize - i32Len;

            if (i32Char > i32BufSize)
            {
                i32Char = i32BufSize;
            }

            if (i32Char > 0)
            {
                int32_t i32PacketSentOk = 0;

                if ((memset(&s_au8XmdBuf[3], 0, (uint32_t)i32BufSize) != (void *)&s_au8XmdBuf[3]) ||
                        (memcpy(&s_au8XmdBuf[3], &pu8SrcBuf[i32Len], (uint32_t)i32Char) != (void *)&s_au8XmdBuf[3])
                   )
                {
                    return XMD_STS_BUF_OP_FAIL;
                }

                if (i32Char < i32BufSize)
                {
                    s_au8XmdBuf[3 + i32Char] = XMD_CTRLZ;  /* Pad XMD_CTRLZ if left data is not align with bufsz */
                }

                if (i32UseCrc)
                {
                    uint16_t u16Crc = crc16_ccitt(&s_au8XmdBuf[3], i32BufSize);
                    s_au8XmdBuf[i32BufSize + 3] = (uint8_t)((u16Crc >> 8) & 0xFFU);
                    s_au8XmdBuf[i32BufSize + 4] = (uint8_t)(u16Crc & 0xFFU);
                }
                else
                {
                    uint8_t u8Checksum = 0;

                    for (i32Idx = 3; i32Idx < (i32BufSize + 3); i32Idx++)
                    {
                        u8Checksum += s_au8XmdBuf[i32Idx];
                    }

                    s_au8XmdBuf[i32BufSize + 3] = u8Checksum;
                }

                for (i32Retry = 0; i32Retry < MAXRETRANS; i32Retry++)
                {
                    for (i32Idx = 0; i32Idx < (i32BufSize + 4 + (i32UseCrc ? 1 : 0)); i32Idx++)
                    {
                        XMD_putc(s_au8XmdBuf[i32Idx]);
                    }

                    i32Char = XMD_getc();

                    if (i32Char >= 0)
                    {
                        switch (i32Char)
                        {
                            case XMD_ACK:
                                u8PacketNo++;
                                i32Len += i32BufSize;
                                i32PacketSentOk = 1;
                                break;

                            case XMD_CAN:
                                i32Char = XMD_getc();

                                if (i32Char == XMD_CAN)
                                {
                                    XMD_putc(XMD_ACK);

                                    return XMD_STS_SEND_USER_CANCEL;
                                }

                                break;

                            case XMD_NAK:
                            default:
                                break;
                        }

                        if (i32PacketSentOk != 0)
                        {
                            break;
                        }
                    }
                }

                if (i32PacketSentOk != 0)
                {
                    continue;
                }

                XMD_putc(XMD_CAN);
                XMD_putc(XMD_CAN);
                XMD_putc(XMD_CAN);
                return XMD_STS_SEND_XMIT_ERR;
            }
            else
            {
                for (i32Retry = 0; i32Retry < 10; i32Retry++)
                {
                    XMD_putc(XMD_EOT);

                    i32Char = XMD_getc();

                    if (i32Char == XMD_ACK)
                    {
                        break;
                    }
                }

                return (i32Char == XMD_ACK) ? i32Len : XMD_STS_SEND_EOT_ACK_FAIL;
            }
        }
    }
}
