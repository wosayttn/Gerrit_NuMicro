/**************************************************************************//**
 * @file     isp_user.c
 * @brief    ISP Command source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "string.h"
#include "NuMicro.h"
#include "isp_user.h"

__ALIGNED(4) uint8_t g_au8ResponseBuff[64];
__ALIGNED(4) static uint8_t g_au8ApromBuf[FMC_FLASH_PAGE_SIZE];
uint32_t g_u32UpdateApromCmd;
uint32_t g_u32ApromSize, g_u32DataFlashAddr, g_u32DataFlashSize;

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

static uint16_t CalCheckSum(uint32_t start, uint32_t len)
{
    int i;
    register uint16_t lcksum = 0;

    for (i = 0; i < len; i += FMC_FLASH_PAGE_SIZE)
    {
        ReadData(start + i, start + i + FMC_FLASH_PAGE_SIZE, (uint32_t *)g_au8ApromBuf);

        if (len - i >= FMC_FLASH_PAGE_SIZE)
        {
            lcksum += Checksum(g_au8ApromBuf, FMC_FLASH_PAGE_SIZE);
        }
        else
        {
            lcksum += Checksum(g_au8ApromBuf, len - i);
        }
    }

    return lcksum;
}

int ParseCmd(uint8_t *pu8Buffer, uint8_t u8len)
{
    static uint32_t u32StartAddress, u32StartAddress_bak, u32TotalLen, u32TotalLen_bak, u32LastDataLen, u32PackNo = 1;
    uint32_t u32PageAddress;
    uint8_t *pu8Response;
    uint16_t u16Lcksum;
    uint32_t u32Lcmd, u32srclen, u32i, u32Lsecurity;
    uint32_t *pu32Config;
    uint8_t *pu8Src;
    static uint32_t u32Gcmd;
    static uint32_t s_u32CfgCmd;        /* Config transaction in progress: UPDATE / READ / 0 = idle */
    static uint32_t s_u32CfgSecurity;   /* Security lock state sampled on the first Config packet */
    pu8Response = g_au8ResponseBuff;
    pu8Src = pu8Buffer;
    u32srclen = u8len;
    u32Lcmd = inpw((uint32_t)pu8Src);
    outpw((uint32_t)(pu8Response + 4), 0);
    pu8Src += 8;
    u32srclen -= 8;
    ReadData(Config0, Config0 + (14 * 4), (uint32_t *)(uint32_t)(pu8Response + 8)); /* Read config */
    pu32Config = (uint32_t *)(pu8Response + 8);
    u32Lsecurity = (((pu32Config[11] & 0xFF) != 0x5A) || ((pu32Config[13] & 0xFF) != 0x5A));

    if(u32Lcmd == CMD_SYNC_PACKNO)
    {
        u32PackNo = inpw((uint32_t)pu8Src);
    }

    if((u32Lcmd) && (u32Lcmd != CMD_RESEND_PACKET))
    {
        u32Gcmd = u32Lcmd;

        if((u32Lcmd != CMD_UPDATE_CONFIG) && (u32Lcmd != CMD_READ_CONFIG))
        {
            s_u32CfgCmd = 0;    /* A new command terminates a pending Config transaction */
        }
    }

    if(u32Lcmd == CMD_GET_FWVER)
    {
        pu8Response[8] = FW_VERSION; /* version 2.3 */
    }
    else if(u32Lcmd == CMD_GET_DEVICEID)
    {
        outpw((uint32_t)(pu8Response + 8), SYS->PDID);
        goto out;
    }
    else if(u32Lcmd == CMD_RUN_APROM || u32Lcmd == CMD_RUN_LDROM || u32Lcmd == CMD_RESET)
    {
        /* Clear POR and Reset Pin reset flag */
        SYS_CLEAR_RST_SOURCE(SYS_RSTSTS_PORF_Msk);
        SYS_CLEAR_RST_SOURCE(SYS_RSTSTS_PINRF_Msk);

        /* Set BS */
        if(u32Lcmd == CMD_RUN_APROM)
        {
            u32i = (FMC->ISPCTL & 0xFFFFFFFC);
        }
        else if(u32Lcmd == CMD_RUN_LDROM)
        {
            u32i = (FMC->ISPCTL & 0xFFFFFFFC);
            u32i |= 0x00000002;
        }
        else
        {
            u32i = (FMC->ISPCTL & 0xFFFFFFFE); /* ISP disable */
        }

        outpw(&FMC->ISPCTL, u32i);
        outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

        /* Trap the CPU */
        while(1);
    }
    else if(u32Lcmd == CMD_CONNECT)
    {
        u32PackNo = 1;
        goto out;
    }
    else if((u32Lcmd == CMD_UPDATE_APROM) || (u32Lcmd == CMD_ERASE_ALL))
    {
        EraseAP(FMC_APROM_BASE, (g_u32ApromSize < g_u32DataFlashAddr) ? g_u32ApromSize : g_u32DataFlashAddr);

        if(u32Lcmd == CMD_ERASE_ALL)    /* Erase data flash */
        {
            EraseAP(g_u32DataFlashAddr, g_u32DataFlashSize);
        }

        g_u32UpdateApromCmd = TRUE;
    }
    else if(u32Lcmd == CMD_GET_FLASHMODE)
    {
        /* Return 1: APROM, 2: LDROM */
        outpw(pu8Response + 8, (FMC->ISPCTL & 0x2) ? 2 : 1);
    }

    if((u32Lcmd == CMD_UPDATE_APROM) || (u32Lcmd == CMD_UPDATE_DATAFLASH))
    {
        if(u32Lcmd == CMD_UPDATE_DATAFLASH)
        {
            u32StartAddress = g_u32DataFlashAddr;

            if(g_u32DataFlashSize)
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
            u32StartAddress = 0;
        }

        u32TotalLen = inpw(pu8Src + 4);
        pu8Src += 8;
        u32srclen -= 8;
        u32StartAddress_bak = u32StartAddress;
        u32TotalLen_bak = u32TotalLen;
    }
    /* Config0-18 does not fit in one packet, so the transfer is split into two
     * packets. Both packets are handled here so that the command has a single
     * entry point and a well defined end of transaction. */
    else if((u32Lcmd == CMD_UPDATE_CONFIG) || (u32Lcmd == CMD_READ_CONFIG) ||
            ((u32Lcmd == 0) && (s_u32CfgCmd != 0)))
    {
        if(u32Lcmd != 0)
        {
            /* First packet: Config0 - Config13 */
            s_u32CfgCmd = u32Lcmd;              /* Arm the transaction, one more packet is expected */
            s_u32CfgSecurity = u32Lsecurity;

            if((u32Lcmd == CMD_UPDATE_CONFIG) &&
               !((u32Lsecurity) && (!g_u32UpdateApromCmd)))    /* security lock */
            {
                UpdateConfig(Config0, (14 * 4), (uint32_t *)(uint32_t)pu8Src, (uint32_t *)(uint32_t)(pu8Response + 8));
            }
            else
            {
                /* CMD_READ_CONFIG, or CMD_UPDATE_CONFIG rejected by the security
                 * lock: report the current values. */
                ReadData(Config0, Config0 + (14 * 4), (uint32_t *)(uint32_t)(pu8Response + 8));
            }
        }
        else
        {
            /* Second packet: Config14 - Config18 */
            uint32_t u32CfgCmd = s_u32CfgCmd;
            uint32_t u32Locked = ((s_u32CfgSecurity) && (!g_u32UpdateApromCmd));

            /* Disarm before doing the work: the transaction ends here, so a
             * following zero-command packet must not repeat this update. */
            s_u32CfgCmd = 0;

            if((u32CfgCmd == CMD_UPDATE_CONFIG) && (!u32Locked))
            {
                UpdateConfig(Config14, (2 * 4), (uint32_t *)(uint32_t)pu8Src, (uint32_t *)(uint32_t)(pu8Response + 8));
                UpdateConfig(Config16, (3 * 4), (uint32_t *)(uint32_t)(pu8Src + (2 * 4)), (uint32_t *)(uint32_t)(pu8Response + 8 + (2 * 4)));
            }
            else
            {
                /* CMD_READ_CONFIG, or CMD_UPDATE_CONFIG rejected by the security
                 * lock: report the current values. */
                ReadData(Config14, (Config14 + (2 * 4)), (uint32_t *)(uint32_t)(pu8Response + 8));
                ReadData(Config16, (Config16 + (3 * 4)), (uint32_t *)(uint32_t)(pu8Response + 8 + (2 * 4)));
            }
        }

        goto out;
    }
    else if(u32Lcmd == CMD_RESEND_PACKET)      /* for APROM and Data flash only */
    {
        u32StartAddress -= u32LastDataLen;
        u32TotalLen += u32LastDataLen;
        u32PageAddress = u32StartAddress & ~(FMC_FLASH_PAGE_SIZE - 1);

        if(u32PageAddress >= Config0)
        {
            goto out;
        }

        ReadData(u32PageAddress, u32StartAddress, (uint32_t *)g_au8ApromBuf);
        FMC_Erase_User(u32PageAddress);
        WriteData(u32PageAddress, u32StartAddress, (uint32_t *)g_au8ApromBuf);

        if((u32StartAddress % FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE - u32LastDataLen))
        {
            FMC_Erase_User(u32PageAddress + FMC_FLASH_PAGE_SIZE);
        }

        goto out;
    }

    if((u32Gcmd == CMD_UPDATE_APROM) || (u32Gcmd == CMD_UPDATE_DATAFLASH))
    {
        if(u32TotalLen < u32srclen)
        {
            u32srclen = u32TotalLen; /* Prevent last package from over writing */
        }

        u32TotalLen -= u32srclen;
        WriteData(u32StartAddress, u32StartAddress + u32srclen, (uint32_t *)(uint32_t)pu8Src);
        memset(pu8Src, 0, u32srclen);
        ReadData(u32StartAddress, u32StartAddress + u32srclen, (uint32_t *)(uint32_t)pu8Src);
        u32StartAddress += u32srclen;
        u32LastDataLen = u32srclen;

        if(u32TotalLen == 0)
        {
            u16Lcksum = CalCheckSum(u32StartAddress_bak, u32TotalLen_bak);
            outps(pu8Response + 8, u16Lcksum);
        }
    }

out:
    u16Lcksum = Checksum(pu8Buffer, u8len);
    outps((uint32_t)pu8Response, u16Lcksum);
    ++u32PackNo;
    outpw((uint32_t)(pu8Response + 4), u32PackNo);
    u32PackNo++;
    return 0;
}
