/**************************************************************************//**
 * @file    main.c
 * @brief
 *          SPI mode SD Card with FAT File System Sample Code
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "stdio.h"
#include "NuMicro.h"
#include "diskio.h"
#include "ff.h"

/****************************************************************************/
/* Define                                                                   */
/****************************************************************************/

#define HCLK_CLOCK      48000000
#define DRIVE_NUMBER    0       /* Physical drive number */
#define DRIVE_NAME      "0:"    /* The drive name string for drive number 0 */

/****************************************************************************/
/* Global variables                                                         */
/****************************************************************************/

FATFS g_FatFs[_DRIVES];    /* File system object for logical drive */
char  g_buff[512];          /* Buffer for read / write data */

/****************************************************************************/
/* Functions                                                                */
/****************************************************************************/

void SYS_Init(void)
{
    /*------------------------------*/
    /* Init System Clock            */
    /*------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*------------------------------*/
    /* Init I/O Multi-function      */
    /*------------------------------*/

    /* Set UART Default MPF */
    UartDebugMFP();
}

void put_rc(FRESULT rc)
{
    uint32_t i;
    const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");

    for (i = 0; (i != (UINT)rc) && *p; i++)
    {
        while (*p++) ;
    }

    printf(_T("rc=%u FR_%s\n"), (UINT)rc, p);
}

/*------------------------------*/
/*  Main Function               */
/*------------------------------*/
int32_t main(void)
{
    FILINFO Finfo;
    uint32_t freeCluster, fileCnt, dirCnt, fileSize;
    FATFS *fs;      /* Pointer to file system object */
    DIR dir;        /* Directory object */
    FRESULT res;
    char  *ptr2 = NULL;

    FIL file;
    UINT count;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART */
    UartDebugInit();

    printf("Core Clock = %d Hz\n\n", CLK_GetHCLKFreq());
    printf("+-------------------------------------------------------+\n");
    printf("|       M253 Series SPI + SD Card Sample Code           |\n");
    printf("+-------------------------------------------------------+\n");

    /* Initializes the physical disk drive */
    res = (FRESULT)disk_initialize(DRIVE_NUMBER);

    if (res != STA_OK)
    {
        put_rc(res);
        printf("\n\nInitialize SD card fail.\n");
    }

    /* Registers a work area to the FatFs module */
    res = f_mount(DRIVE_NUMBER, &g_FatFs[0]);

    if (res != FR_OK)
    {
        put_rc(res);
        printf("\n\nMount file system fail.\n");
    }

    /* Opens an exsisting directory and creates the directory object for subsequent calls */
    put_rc(f_opendir(&dir, DRIVE_NAME));
    fileSize = fileCnt = dirCnt = 0;

    /* Show all items in the directory */
    for (;;)
    {
        res = f_readdir(&dir, &Finfo);

        if ((res != FR_OK) || (!Finfo.fname[0]))
            break;

        if (Finfo.fattrib & AM_DIR)
        {
            dirCnt++;
        }
        else
        {
            fileCnt++;
            fileSize += Finfo.fsize;
        }

        printf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s\n",
               (Finfo.fattrib & AM_DIR) ? 'D' : '-',
               (Finfo.fattrib & AM_RDO) ? 'R' : '-',
               (Finfo.fattrib & AM_HID) ? 'H' : '-',
               (Finfo.fattrib & AM_SYS) ? 'S' : '-',
               (Finfo.fattrib & AM_ARC) ? 'A' : '-',
               (Finfo.fdate >> 9) + 1980,
               (Finfo.fdate >> 5) & 15,
               Finfo.fdate & 31,
               (Finfo.ftime >> 11),
               (Finfo.ftime >> 5) & 63,
               Finfo.fsize, Finfo.fname);
    }

    printf("%4u File(s), %10u bytes total\n", fileCnt, fileSize);

    printf("%4u Dir(s)", dirCnt);

    if (f_getfree(ptr2, (DWORD *)&freeCluster, &fs) == FR_OK)
        printf(",  %10u bytes free\n", freeCluster * fs->csize * 512);

    /* File read / write case */
    /* Open file and set it to readable / writable mode */
    res = f_open(&file, "test.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

    if (res == FR_OK)
    {
        /* Move file pointer to end of file (offset = file.fsize) */
        f_lseek(&file, file.fsize);
        /* Write something to file pointer */
        f_write(&file, "hello ", 6, &count);
        /* Move file pointer to begin of file (offset = 0) */
        f_lseek(&file, 0);
        /* Read something from file pointer */
        f_read(&file, g_buff, file.fsize, &count);
        printf("Read data from file test.txt : \n[%s]\n", g_buff);
        /* Close file */
        f_close(&file);
    }
    else
        printf("Open file fail. Return %d\n", res);

    while (1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
