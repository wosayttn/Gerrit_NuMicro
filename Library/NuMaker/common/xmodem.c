/******************************************************************************
 * @file     xmodem.c
 * @version  V1.00
 * @brief    Xmodem transfer
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "xmodem.h"


#define XMD_MAX_TRANS_SIZE      (1024*1024)


/* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
static uint8_t s_au8XmdBuf[1030UL];

/* Wait until ISP trigger is cleared or timeout occurs. */
static int32_t XMD_WaitISPDone(uint32_t u32Timeout)
{
    uint32_t u32Remain;

    u32Remain = u32Timeout;

    while ((u32Remain > 0UL) && (RMC->ISPTRG != 0UL))
    {
        u32Remain--;
    }

    if (u32Remain == 0UL)
    {
        return -1L;
    }

    return 0L;
}

/* Clear ISP data buffer and check ISP fail flag. */
static int32_t XMD_ClearDataBuffer(void)
{
    int32_t i32Ret;

    RMC->ISPCMD  = RMC_ISPCMD_CLEAR_DATA_BUFFER;
    RMC->ISPADDR = 0x00000000UL;
    RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

    i32Ret = XMD_WaitISPDone(RMC_TIMEOUT_WRITE);

    if ((i32Ret == 0L) && ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL))
    {
        RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
        i32Ret = -1L;
    }

    return i32Ret;
}

/*
    To program data from Xmodem transfer.
*/
static int32_t XMD_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t i32Ret;

    /* Do not use standard I/O in driver or bootloader path. */
    RMC->ISPCTL &= (uint32_t)~RMC_ISPCTL_MPEN_Msk;

    /* Clear data buffer before loading data. */
    i32Ret = XMD_ClearDataBuffer();

    if (i32Ret == 0L)
    {
        RMC->ISPCMD  = RMC_ISPCMD_LOAD_DATA_BUFFER;
        RMC->ISPADDR = u32Addr;
        RMC->ISPDAT  = u32Data;
        RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

        i32Ret = XMD_WaitISPDone(RMC_TIMEOUT_WRITE);

        if ((i32Ret == 0L) && ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL))
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
            i32Ret = -1L;
        }
    }

    if (i32Ret == 0L)
    {
        RMC->ISPCMD  = RMC_ISPCMD_PROGRAM;
        RMC->ISPADDR = u32Addr;
        RMC->ISPDAT  = u32Data;
        RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

        i32Ret = XMD_WaitISPDone(RMC_TIMEOUT_WRITE);

        if ((i32Ret == 0L) && ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL))
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
            i32Ret = -1L;
        }
    }

    if (i32Ret != 0L)
    {
        /* [SEC] Best-effort cleanup after failure. */
        (void)XMD_ClearDataBuffer();

        if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
        }

        return -1L;
    }

    return 0L;
}

static void XMD_putc(uint8_t c)
{
    UART_T * const pUART = UART0; /* Pointer itself is not modified. */

    while ((pUART->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0UL)
    {
        /* Wait until TX FIFO is not full. */
    }
    pUART->DAT = c;

}

static int32_t XMD_getc(void) /* Use prototype form with named parameter list. */
{
    const UART_T * pUART = UART0;
    uint32_t u32ms = 0UL;

    /* Wait for 100 ms */
    while (u32ms < 100UL) /* Use unsigned constant to match uint32_t. */
    {
        SysTick->CTRL = 0UL;
        SysTick->LOAD = 1000UL * (uint32_t)CyclesPerUs; /* Use unsigned arithmetic for 1 ms. */
        SysTick->VAL  = 0UL;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
        {
           if( (pUART->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != UART_FIFOSTS_RXEMPTY_Msk )
           {
               SysTick->CTRL = 0;
               return ((int32_t)pUART->DAT);
           }
        }
        u32ms++;
    }

    SysTick->CTRL = 0UL;
    return -1L; /* time-out */
}
static uint16_t CalcCrc16Ccitt(const uint8_t *pu8Buf, int32_t i32Len)
{
    uint16_t u16Crc = 0;
    int32_t  i32DataByteCnt = i32Len;
    const uint8_t *pu8Data = pu8Buf;

    while (i32DataByteCnt > 0)
    {
        int32_t  i32Idx;
        uint16_t u16Data;

        u16Data = (uint16_t)(*pu8Data);
        pu8Data++;
        u16Crc ^= (u16Data << 8U);

        for (i32Idx = 0; i32Idx < 8; i32Idx++)
        {
            if ((u16Crc & 0x8000U) == 0x8000U)
            {
                u16Crc = (uint16_t)(u16Crc << 1) ^ (uint16_t)0x1021;
            }
            else
            {
                u16Crc = (uint16_t)(u16Crc << 1);
            }
        }
        i32DataByteCnt--;
    }

    return u16Crc;
}


static bool CheckPacketIntegrity(bool bIsCrc, const uint8_t *pu8Buf, int32_t i32Size)
{
    if (bIsCrc)
    {
        uint16_t u16Crc = CalcCrc16Ccitt(pu8Buf, i32Size);
        uint16_t u16TargetCrc = (uint16_t)pu8Buf[i32Size];

        u16TargetCrc = (u16TargetCrc << 8);
        u16TargetCrc |= (uint16_t)(pu8Buf[i32Size + 1]);

        if (u16Crc == u16TargetCrc)
        {
            return true;
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
            return true;
        }
    }

    return false;
}

/**
  * @brief      Recive data from UART Xmodem transfer and program the data to flash.
  * @param[in]  u32DestAddr Destination address of flash to program.
  * @return     Recived data size if successful. Return -1 when error.
  *
  * @details    This function is used to recieve UART data through Xmodem transfer.
  *             The received data will be programmed to flash packet by packet.
  */
int32_t Xmodem(uint32_t u32DestAddr)
{
    int32_t i32Err = 0;
    bool    bUseCrc = false;
    char    cTryChar = 'C';
    uint8_t u8PacketNo = 1;
    int32_t i32Idx;
    int32_t i32BlkIdx;
    int32_t i32Retrans = MAXRETRANS;
    int32_t i32TransBytes = 0;
    int32_t i32Char;
    uint32_t u32WriteData;

    for (;;)
    {
        int32_t i32BufSize;
        int32_t i32StartChar;
        int32_t i32PayloadReadSize;
        int32_t i32RejectPacket;

        i32StartChar = -1;
        i32BufSize = 0;

        for (i32Idx = 0; i32Idx < XMD_MAX_TIMEOUT; i32Idx++) /* set timeout period */
        {
            if (cTryChar != (char)0)
            {
                XMD_putc((uint8_t)cTryChar);
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
            bUseCrc = true;
        }

        cTryChar = (char)0;
        s_au8XmdBuf[0] = (uint8_t)i32StartChar;

        i32PayloadReadSize = i32BufSize + (bUseCrc ? 1 : 0) + 3;
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
            if (((s_au8XmdBuf[1] + s_au8XmdBuf[2]) == 0xFFU) && CheckPacketIntegrity(bUseCrc, &s_au8XmdBuf[3], i32BufSize))
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
                        for (i32BlkIdx = 0; i32BlkIdx < (i32BufSize + 3) / 4; i32BlkIdx++)
                        {
                            (void)memcpy((uint8_t *)&u32WriteData, &s_au8XmdBuf[3 + (i32BlkIdx * 0x4)], 4);

                            i32Err = XMD_Write((u32DestAddr + (uint32_t)i32TransBytes) + ((uint32_t)i32BlkIdx * 0x4U), u32WriteData);
													
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
  * @param[in]  src     Address of the source data to transfer.
  * @param[in]  srcsz   Size of the total size to transfer.
  * @retval     Total transfer size when successfull
  * @retval     -1  Canceled by remote
  * @retval     -2  No sync chararcter received.
  * @retval     -4  Transmit error.
  * @retval     -5  Unknown error.
  * @details    This function is used to send UART data through Xmodem transfer.
  *
  */
int32_t XmodemSend(const uint8_t *pu8SrcBuf, int32_t i32SrcSize)
{
    bool    bUseCrc = false;
    uint8_t u8PacketNo = 1;
    int32_t i32Idx;
    int32_t i32Char;
    int32_t i32Len = 0;
    int32_t i32Retry;

    for (;;)
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
                        bUseCrc = true;
                        i32SyncOk = 1;
                        break;

                    case XMD_NAK:
                        bUseCrc = false;
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

        for (;;)
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

                (void)memset(&s_au8XmdBuf[3], 0, (uint32_t)i32BufSize);
                (void)memcpy(&s_au8XmdBuf[3], &pu8SrcBuf[i32Len], (uint32_t)i32Char);

                if (i32Char < i32BufSize)
                {
                    s_au8XmdBuf[3 + i32Char] = XMD_CTRLZ;  /* Pad XMD_CTRLZ if left data is not align with bufsz */
                }

                if (bUseCrc)
                {
                    uint16_t u16Crc = CalcCrc16Ccitt(&s_au8XmdBuf[3], i32BufSize);
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
                    for (i32Idx = 0; i32Idx < (i32BufSize + 4 + (bUseCrc ? 1 : 0)); i32Idx++)
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

