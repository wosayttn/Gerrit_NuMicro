/***************************************************************************//**
 * @file    diskio.c
 * @version V1.00
 * @brief   FatFs disk I/O glue for the SPI mode SD Card
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <limits.h>
#include <stdio.h>
#include "sdcard.h"
#include "diskio.h"

#define DEV_SD    0U

static DSTATUS s_u8DiskStatus = STA_NOINIT;

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize(BYTE pdrv)
{
    uint8_t u8Status;

    if (pdrv != DEV_SD)
        return STA_NOINIT;

    u8Status = SD_Initialize();

    if (u8Status == 0U)
    {
        s_u8DiskStatus = 0U;
        printf("SDCard Open success\n");
    }
    else
    {
        s_u8DiskStatus = STA_NOINIT;
        printf("SDCard Open failed (status=0x%02X)\n", u8Status);
    }

    return s_u8DiskStatus;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != DEV_SD)
        return STA_NOINIT;

    return s_u8DiskStatus;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if ((pdrv != DEV_SD) || (buff == NULL) || (count == 0U) ||
            (count > UCHAR_MAX))
    {
        return RES_PARERR;
    }

#if FF_LBA64

    if (sector > UINT_MAX)
    {
        return RES_PARERR;
    }

#endif

    if (s_u8DiskStatus & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    return (SD_ReadDisk(buff, (unsigned int)sector, (unsigned char)count) == 0U) ?
           RES_OK : RES_ERROR;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if ((pdrv != DEV_SD) || (buff == NULL) || (count == 0U) ||
            (count > UCHAR_MAX))
        return RES_PARERR;

#if FF_LBA64

    if (sector > UINT_MAX)
        return RES_PARERR;

#endif

    if (s_u8DiskStatus & STA_NOINIT)
        return RES_NOTRDY;

    return (SD_WriteDisk((unsigned char *)buff, (unsigned int)sector,
                         (unsigned char)count) == 0U) ? RES_OK : RES_ERROR;
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    unsigned int u32SectorCount;

    if (pdrv != DEV_SD)
        return RES_PARERR;

    if (s_u8DiskStatus & STA_NOINIT)
        return RES_NOTRDY;

    switch (cmd)
    {
        case CTRL_SYNC:
            return RES_OK;

        case GET_SECTOR_COUNT:
            if (buff == NULL)
                return RES_PARERR;

            u32SectorCount = SD_GetSectorCount();

            if (u32SectorCount == 0U)
                return RES_ERROR;

            *(LBA_t *)buff = (LBA_t)u32SectorCount;
            return RES_OK;

        case GET_SECTOR_SIZE:
            if (buff == NULL)
                return RES_PARERR;

            *(WORD *)buff = 512U;
            return RES_OK;

        case GET_BLOCK_SIZE:
            if (buff == NULL)
                return RES_PARERR;

            *(DWORD *)buff = 1U;
            return RES_OK;

        default:
            return RES_PARERR;
    }
}

#if !FF_FS_READONLY && !FF_FS_NORTC
DWORD get_fattime(void)
{
    return 0U;
}
#endif

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
