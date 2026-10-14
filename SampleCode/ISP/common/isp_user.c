/***************************************************************************//**
 * @file     isp_user.c
 * @version  V1.00
 * @brief    ISP Command source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "targetdev.h"
#include "isp_user.h"

__ALIGNED(4) uint8_t g_au8ResponseBuff[CMD_RESP_PACKET_BYTES];
__ALIGNED(4) static uint8_t g_au8ApromBuf[FMC_FLASH_PAGE_SIZE];
uint32_t g_u32UpdateApromCmd = FALSE;
uint32_t g_u32ApromSize,
         g_u32DataFlashAddr = FMC_DATA_FLASH_BASE,
         g_u32DataFlashSize = FMC_DATA_FLASH_SIZE;

static uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++)
    {
        c += buf[i];
    }

    return (c);
}

uint16_t CalCheckSum(uint32_t u32Start, uint32_t u32Bytes)
{
    uint32_t u32i;
    uint16_t u16lcksum = 0;

    for (u32i = 0; u32i < u32Bytes; u32i += FMC_FLASH_PAGE_SIZE)
    {
        ReadData(u32Start + u32i, u32Start + u32i + FMC_FLASH_PAGE_SIZE, (uint32_t *)g_au8ApromBuf);

        if (u32Bytes - u32i >= FMC_FLASH_PAGE_SIZE)
        {
            u16lcksum += Checksum(g_au8ApromBuf, FMC_FLASH_PAGE_SIZE);
        }
        else
        {
            u16lcksum += Checksum(g_au8ApromBuf, u32Bytes - u32i);
        }
    }

    return u16lcksum;
}

int ParseCmd(uint8_t *pu8Buffer, uint8_t u8BufBytes)
{
    static uint32_t u32StartAddress, u32StartAddress_bak, u32TotalLen, u32TotalLen_bak, u32LastDataLen, u32PackNo = 1;
    uint32_t u32PageAddress;
    uint8_t *pu8Response;
    uint16_t u16Lcksum;
    uint32_t u32PacketCmd, u32SrcBytes, u32Lock;
    uint32_t *pu32Config;
    uint8_t *pu8Src;
    /* Session command and lock status for the multi-packet commands */
    static uint32_t s_u32SessionCmd , s_u32SessionLock;

    if (u8BufBytes < 8)
        return -1;

    pu8Response = g_au8ResponseBuff;
    pu8Src = pu8Buffer;
    u32SrcBytes = u8BufBytes;
    u32PacketCmd = inpw((uint32_t)pu8Src);
    outpw((uint32_t)(pu8Response + 4), 0);
    pu8Src += 8;
    u32SrcBytes -= 8;

    /* Read user config: When total user config size > CMD_RESP_DATA_BYTES,
     * only read CMD_RESP_DATA_BYTES bytes due to packet size limitation. */
    ReadData(FMC_USER_CONFIG_0, (FMC_USER_CONFIG_0 + CMD_RESP_DATA_BYTES), (uint32_t *)(uint32_t)(pu8Response + 8));
    pu32Config = (uint32_t *)(pu8Response + 8);
    /* Check SCRLOCK and ARLOCK status in the user configuration. */
    u32Lock = (((pu32Config[11] & 0xFF) != 0x5A) || ((pu32Config[13] & 0xFF) != 0x5A));

    if (u32PacketCmd == CMD_SYNC_PACKNO)
    {
        u32PackNo = inpw((uint32_t)pu8Src);
    }

    if ((u32PacketCmd) && (u32PacketCmd != CMD_RESEND_PACKET))
    {
        s_u32SessionCmd  = u32PacketCmd;
        s_u32SessionLock = u32Lock;
    }

    if (u32PacketCmd == CMD_GET_FWVER)
    {
        pu8Response[8] = FW_VERSION; /* FW_VERSION is defined in isp_user.h */
    }
    else if (u32PacketCmd == CMD_GET_DEVICEID)
    {
        outpw((uint32_t)(pu8Response + 8), SYS->PDID);
        goto out;
    }
    else if (u32PacketCmd == CMD_RUN_APROM || u32PacketCmd == CMD_RUN_LDROM || u32PacketCmd == CMD_RESET)
    {
        /* Clear POR and Reset Pin reset flag */
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);

        /* Set VECMAP */
        if (u32PacketCmd == CMD_RUN_APROM)
        {
            FMC_SetVectorPageAddr(FMC_APROM_BASE);
        }
        else
        {
            FMC_SetVectorPageAddr(FMC_LDROM_BASE);
        }

        if (u32PacketCmd == CMD_RESET)
        {
            SYS_ResetChip();
        }
        else
        {
            NVIC_SystemReset();
        }
    }
    else if (u32PacketCmd == CMD_CONNECT)
    {
        u32PackNo = 1;
        goto out;
    }
    else if ((u32PacketCmd == CMD_UPDATE_APROM) || (u32PacketCmd == CMD_ERASE_ALL))
    {
        EraseAP(FMC_APROM_BASE, g_u32ApromSize);

        if (u32PacketCmd == CMD_ERASE_ALL)   /* Erase data flash */
        {
            EraseAP(g_u32DataFlashAddr, g_u32DataFlashSize);
        }

        g_u32UpdateApromCmd = TRUE;
    }
    else if (u32PacketCmd == CMD_GET_FLASHMODE)
    {
        /* Return 1: APROM, 2: LDROM */
        outpw(pu8Response + 8, (FMC->ISPCTL & 0x2) ? 2 : 1);
    }

    if ((u32PacketCmd == CMD_UPDATE_APROM) || (u32PacketCmd == CMD_UPDATE_DATAFLASH))
    {
        if (u32PacketCmd == CMD_UPDATE_DATAFLASH)
        {
            u32StartAddress = g_u32DataFlashAddr;

            if (g_u32DataFlashSize > 0)
            {
                EraseAP(g_u32DataFlashAddr, g_u32DataFlashSize);
            }
            else
            {
                goto out;
            }
        }
        else
        {
            u32StartAddress = FMC_APROM_BASE;
        }

        u32TotalLen = inpw(pu8Src + 4);
        pu8Src += 8;
        u32SrcBytes -= 8;
        u32StartAddress_bak = u32StartAddress;
        u32TotalLen_bak = u32TotalLen;
    }
    else if (u32PacketCmd == CMD_UPDATE_CONFIG)
    {
        /* Because the ISP command uses internal FMC routines with fewer restrictions,
         * check whether the device is security-locked or the Flash is empty before updating the configuration. */
        if ((u32Lock) && (!g_u32UpdateApromCmd))
        {
            goto out;
        }

        /* Update user config: When total user config size >= CMD_RESP_DATA_BYTES, only update CMD_RESP_DATA_BYTES bytes first. */
        UpdateConfig(FMC_USER_CONFIG_0, CMD_RESP_DATA_BYTES, (uint32_t *)(uint32_t)pu8Src, (uint32_t *)(uint32_t)(pu8Response + 8));
        goto out;
    }
    else if (u32PacketCmd == CMD_READ_CONFIG)
    {
        /* First CMD_RESP_DATA_BYTES of user configs have been read into the response buffer at the beginning of ParseCmd. */
        goto out;
    }
    else if (u32PacketCmd == CMD_RESEND_PACKET)     /* for APROM and Data flash only */
    {
        u32StartAddress -= u32LastDataLen;
        u32TotalLen += u32LastDataLen;
        u32PageAddress = u32StartAddress & FMC_PAGE_ADDR_MASK;

        if (u32PageAddress >= FMC_USER_CONFIG_0)
        {
            goto out;
        }

        ReadData(u32PageAddress, u32StartAddress, (uint32_t *)g_au8ApromBuf);
        FMC_Erase_User(u32PageAddress);
        WriteData(u32PageAddress, u32StartAddress, (uint32_t *)g_au8ApromBuf);

        if ((u32StartAddress % FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE - u32LastDataLen))
        {
            FMC_Erase_User(u32PageAddress + FMC_FLASH_PAGE_SIZE);
        }

        goto out;
    }

    if ((s_u32SessionCmd  == CMD_UPDATE_APROM) || (s_u32SessionCmd  == CMD_UPDATE_DATAFLASH))
    {
        if (u32TotalLen < u32SrcBytes)
        {
            u32SrcBytes = u32TotalLen; /* Prevent last package from over writing */
        }

        u32TotalLen -= u32SrcBytes;
        WriteData(u32StartAddress, u32StartAddress + u32SrcBytes, (uint32_t *)(uint32_t)pu8Src);
        memset(pu8Src, 0, u32SrcBytes);
        ReadData(u32StartAddress, u32StartAddress + u32SrcBytes, (uint32_t *)(uint32_t)pu8Src);
        u32StartAddress += u32SrcBytes;
        u32LastDataLen = u32SrcBytes;

        if (u32TotalLen == 0)
        {
            u16Lcksum = CalCheckSum(u32StartAddress_bak, u32TotalLen_bak);
            outps(pu8Response + 8, u16Lcksum);
        }
    }
    else if (s_u32SessionCmd  == CMD_UPDATE_CONFIG)
    {
        /* Although this ISP command uses internal FMC routines with fewer restrictions,
         * check whether the device is security-locked or the Flash is empty before updating the configuration. */
        if ((s_u32SessionLock) && (!g_u32UpdateApromCmd))
        {
            /* Read CONFIG14-15 and CONFIG16-18 separately because their addresses are discontinuous. */
            ReadData(FMC_USER_CONFIG_14, (FMC_USER_CONFIG_14 + (2 * 4)), (uint32_t *)(uint32_t)(pu8Response + 8));
            ReadData(FMC_USER_CONFIG_16, (FMC_USER_CONFIG_16 + (3 * 4)), (uint32_t *)(uint32_t)(pu8Response + 8 + (2 * 4)));
            goto out;
        }

        /* Update CONFIG14-15 and CONFIG16-18 separately because their addresses are discontinuous. */
        UpdateConfig(FMC_USER_CONFIG_14, (2 * 4), (uint32_t *)(uint32_t)pu8Src, (uint32_t *)(uint32_t)(pu8Response + 8));
        UpdateConfig(FMC_USER_CONFIG_16, (3 * 4), (uint32_t *)(uint32_t)(pu8Src + (2 * 4)), (uint32_t *)(uint32_t)(pu8Response + 8 + (2 * 4)));
    }
    else if (s_u32SessionCmd  == CMD_READ_CONFIG)
    {
        /* When total user config size > CMD_RESP_DATA_BYTES, read left over bytes in subsequent packets. */
        /* Read CONFIG14-15 and CONFIG16-18 separately because their addresses are discontinuous. */
        ReadData(FMC_USER_CONFIG_14, (FMC_USER_CONFIG_14 + (2 * 4)), (uint32_t *)(uint32_t)(pu8Response + 8));
        ReadData(FMC_USER_CONFIG_16, (FMC_USER_CONFIG_16 + (3 * 4)), (uint32_t *)(uint32_t)(pu8Response + 8 + (2 * 4)));
    }

out:
    u16Lcksum = Checksum(pu8Buffer, u8BufBytes);
    outps((uint32_t)pu8Response, u16Lcksum);
    ++u32PackNo;
    outpw((uint32_t)(pu8Response + 4), u32PackNo);
    u32PackNo++;
    return 0;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
