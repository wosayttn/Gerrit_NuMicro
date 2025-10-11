/****************************************************************************//**
 * @file    sdcard.h
 * @brief
 *          SD Card driver header file
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/


#ifndef __SDCARD_H__
#define __SDCARD_H__

/****************************************************************************/
/* Define                                                                   */
/****************************************************************************/

#define SD_TYPE_ERR     0x00
#define SD_TYPE_MMC     0x01
#define SD_TYPE_V1      0x02
#define SD_TYPE_V2      0x04
#define SD_TYPE_V2HC    0x06

#define CMD0    0
#define CMD1    1
#define CMD8    8
#define CMD9    9
#define CMD10   10
#define CMD12   12
#define CMD16   16
#define CMD17   17
#define CMD18   18
#define CMD23   23
#define CMD24   24
#define CMD25   25
#define CMD41   41
#define CMD55   55
#define CMD58   58
#define CMD59   59
#define CRC_ON_OFF                  59
#define MSD_DATA_OK                 0x05
#define MSD_DATA_CRC_ERROR          0x0B
#define MSD_DATA_WRITE_ERROR        0x0D
#define MSD_DATA_OTHER_ERROR        0xFF

#define MSD_RESPONSE_NO_ERROR       0x00
#define MSD_IN_IDLE_STATE           0x01
#define MSD_ERASE_RESET             0x02
#define MSD_ILLEGAL_COMMAND         0x04
#define MSD_COM_CRC_ERROR           0x08
#define MSD_ERASE_SEQUENCE_ERROR    0x10
#define MSD_ADDRESS_ERROR           0x20
#define MSD_PARAMETER_ERROR         0x40
#define MSD_RESPONSE_FAILURE        0xFF

/****************************************************************************/
/* Global variables                                                         */
/****************************************************************************/

extern unsigned char  SD_Type;

/****************************************************************************/
/* Functions                                                                */
/****************************************************************************/

unsigned char SD_SPI_ReadWriteByte(unsigned char data);
unsigned char SD_WaitReady(void);
unsigned char SD_GetResponse(unsigned char Response);
unsigned char SD_Initialize(void);
unsigned char SD_ReadDisk(unsigned char *buf, unsigned int sector, unsigned char cnt);
unsigned char SD_WriteDisk(unsigned char *buf, unsigned int sector, unsigned char cnt);
unsigned int  SD_GetSectorCount(void);
unsigned char SD_GetCID(unsigned char *cid_data);
unsigned char SD_GetCSD(unsigned char *csd_data);
unsigned char SD_CRC_OFF(void);

#endif  /* __SDCARD_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
