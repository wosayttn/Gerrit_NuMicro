/***************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   SPI mode SD Card with FAT File System Sample Code
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "diskio.h"
#include "ff.h"

/****************************************************************************/
/* Define                                                                   */
/****************************************************************************/

#define DRIVE_NUMBER    0       /* Physical drive number */
#define DRIVE_NAME      "0:"    /* The drive name string for drive number 0 */
#define CMD_LINE_SIZE   128U
#define CMD_ARG_COUNT   4U

/****************************************************************************/
/* Global variables                                                         */
/****************************************************************************/

FATFS g_FatFs[FF_VOLUMES]; /* File system object for logical drive */
char  g_buff[512];          /* Buffer for read / write data */

static int32_t s_argc;
static char *s_argv[CMD_ARG_COUNT];
static uint8_t s_isMounted;

/* Command function prototypes */
static void cmd_mount(void);
static void cmd_unmount(void);
static void cmd_format(void);
static void cmd_write(void);
static void cmd_append(void);
static void cmd_create(void);
static void cmd_read(void);
static void cmd_delete(void);
static void cmd_rename(void);
static void cmd_copy(void);
static void cmd_mkdir(void);
static void cmd_rmdir(void);
static void cmd_find(void);
static void cmd_stat(void);
static void cmd_pwd(void);
static void cmd_chdir(void);
static void cmd_remount(void);
static void cmd_size(void);
static void cmd_help(void);

/* Command definitions structure */
typedef struct
{
    void (*func)(void);
    const char *cmd;
    const char *info;
} CMD_t;

static const CMD_t cmd_list[] =
{
    { cmd_mount,   "mount",   "Mount the drive" },
    { cmd_unmount, "unmount", "Unmount the drive" },
    { cmd_format,  "format",  "Format the drive" },
    { cmd_write,   "write",   "Write text: write <filename> (enter .end to finish)" },
    { cmd_append,  "append",  "Append text: append <filename> (enter .end to finish)" },
    { cmd_create,  "touch",   "Create empty file: touch <filename>" },
    { cmd_read,    "read",    "Read lines from a file: read <filename> [lines]" },
    { cmd_delete,  "rm",      "Delete a file: rm <filename>" },
    { cmd_rename,  "mv",      "Rename a file: mv <src> <dst>" },
    { cmd_copy,    "cp",      "Copy a file: cp <src> <dst>" },
    { cmd_mkdir,   "mkdir",   "Make a directory: mkdir <dirname>" },
    { cmd_rmdir,   "rmdir",   "Remove a directory: rmdir <dirname>" },
    { cmd_find,    "ls",      "List files in directory: ls [path]" },
    { cmd_stat,    "stat",    "Show file attributes: stat <path>" },
    { cmd_pwd,     "pwd",     "Print working directory" },
    { cmd_chdir,   "cd",      "Change working directory: cd <path>" },
    { cmd_remount, "remount", "Re-initialize SD card (after reinsert)" },
    { cmd_size,    "df",      "Show total and free SD card space" },
    { cmd_help,    "?",       "Show help" },
};

/****************************************************************************/
/* Functions                                                                */
/****************************************************************************/
void SYS_Init(void)
{
    /*------------------------------*/
    /* Init System Clock            */
    /*------------------------------*/

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
}

void put_rc(FRESULT rc)
{
    uint32_t i;
    const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0INVALID_PARAMETER\0");

    for (i = 0; (i != (UINT)rc) && *p; i++)
    {
        while (*p++) ;
    }

    printf(_T("rc=%u FR_%s\n"), (UINT)rc, p);
}

static uint8_t require_mounted(void)
{
    if (s_isMounted == 0U)
    {
        printf("Drive is not mounted. Use 'mount' or 'remount'.\n");
        return 0U;
    }

    return 1U;
}

static uint32_t read_console_line(char *buffer, uint32_t bufferSize)
{
    static uint8_t ignoreLineFeed;
    uint32_t length = 0U;
    int input;

    for (;;)
    {
        input = getchar();

        if (ignoreLineFeed != 0U)
        {
            ignoreLineFeed = 0U;

            if (input == '\n')
                continue;
        }

        if ((input == '\r') || (input == '\n'))
        {
            if (input == '\r')
                ignoreLineFeed = 1U;

            buffer[length] = '\0';
            printf("\n");
            return length;
        }

        if ((input == '\b') || (input == 0x7F))
        {
            if (length > 0U)
            {
                length--;
                printf("\b \b");
            }
        }
        else if (isprint((unsigned char)input) && (length < (bufferSize - 1U)))
        {
            buffer[length++] = (char)input;
            putchar(input);
        }
    }
}

static void cmd_mount(void)
{
    FRESULT res;

    if (s_isMounted != 0U)
    {
        printf("Drive is already mounted.\n");
        return;
    }

    /* opt=1 asks FatFs to initialize the disk. Avoid initializing it twice. */
    res = f_mount(&g_FatFs[0], DRIVE_NAME, 1U);

    if (res != FR_OK)
    {
        printf("Mount file system fail. ");
        put_rc(res);
        return;
    }

    s_isMounted = 1U;
    printf("Drive mounted.\n");
}

static void cmd_unmount(void)
{
    FRESULT res;

    if (s_isMounted == 0U)
    {
        printf("Drive is already unmounted.\n");
        return;
    }

    res = f_unmount(DRIVE_NAME);

    if (res != FR_OK)
    {
        printf("Unmount drive fail. ");
        put_rc(res);
        return;
    }

    s_isMounted = 0U;
    printf("Drive unmounted.\n");
}

static void cmd_format(void)
{
    const MKFS_PARM formatOptions = {FM_ANY, 0U, 0U, 0U, 0U};
    FRESULT res;
    DSTATUS diskStatus;

    (void)f_unmount(DRIVE_NAME);
    s_isMounted = 0U;
    diskStatus = disk_initialize(DRIVE_NUMBER);

    if (diskStatus != 0U)
    {
        printf("Initialize SD card fail (status=0x%02X).\n", diskStatus);
        return;
    }

    printf("Formatting drive...\n");
    res = f_mkfs(DRIVE_NAME, &formatOptions, g_buff, sizeof(g_buff));

    if (res != FR_OK)
    {
        printf("Format drive fail. ");
        put_rc(res);
        return;
    }

    printf("Format complete.\n");
    cmd_mount();
}

static void input_file_text(BYTE openMode, const char *operation)
{
    static const char lineEnding = '\n';
    FIL file;
    FRESULT res;
    FRESULT closeRes;
    uint32_t lineCount = 0U;
    uint32_t inputLength;
    UINT bytesWritten;

    res = f_open(&file, s_argv[1], openMode | FA_WRITE);

    if (res != FR_OK)
    {
        printf("Open %s fail. ", s_argv[1]);
        put_rc(res);
        return;
    }

    printf("Enter text. Type .end on a new line to finish.\n");

    for (;;)
    {
        printf("> ");
        inputLength = read_console_line(g_buff, sizeof(g_buff));

        if (strcmp(g_buff, ".end") == 0)
            break;

        res = f_write(&file, g_buff, (UINT)inputLength, &bytesWritten);

        if ((res == FR_OK) && (bytesWritten != inputLength))
            res = FR_DISK_ERR;

        if (res == FR_OK)
            res = f_write(&file, &lineEnding, 1U, &bytesWritten);

        if ((res == FR_OK) && (bytesWritten != 1U))
            res = FR_DISK_ERR;

        if (res != FR_OK)
            break;

        lineCount++;
    }

    closeRes = f_close(&file);

    if (res == FR_OK)
        res = closeRes;

    if (res == FR_OK)
    {
        printf("%s %lu line(s) to %s.\n", operation,
               (unsigned long)lineCount, s_argv[1]);
    }
    else
    {
        printf("%s %s fail. ", operation, s_argv[1]);
        put_rc(res);
    }
}

static void cmd_write(void)
{
    if (s_argc != 2)
    {
        printf("Usage: write <filename>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    input_file_text(FA_CREATE_ALWAYS, "Wrote");
}

static void cmd_append(void)
{
    if (s_argc != 2)
    {
        printf("Usage: append <filename>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    input_file_text(FA_OPEN_APPEND, "Appended");
}

static void cmd_create(void)
{
    FIL file;
    FRESULT res;

    if (s_argc != 2)
    {
        printf("Usage: touch <filename>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_open(&file, s_argv[1], FA_OPEN_ALWAYS | FA_WRITE);

    if (res == FR_OK)
        res = f_close(&file);

    if (res == FR_OK)
        printf("File %s is ready.\n", s_argv[1]);
    else
    {
        printf("Create %s fail. ", s_argv[1]);
        put_rc(res);
    }
}

static void cmd_read(void)
{
    FIL file;
    FRESULT res;
    char *endPtr;
    uint32_t lineLimit = 0U;
    uint32_t linesRead = 0U;

    if ((s_argc < 2) || (s_argc > 3))
    {
        printf("Usage: read <filename> [lines]\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    if (s_argc == 3)
    {
        lineLimit = (uint32_t)strtoul(s_argv[2], &endPtr, 10);

        if ((*s_argv[2] == '\0') || (*endPtr != '\0') || (lineLimit == 0U))
        {
            printf("Invalid line count: %s\n", s_argv[2]);
            return;
        }
    }

    res = f_open(&file, s_argv[1], FA_READ);

    if (res != FR_OK)
    {
        printf("Open %s fail. ", s_argv[1]);
        put_rc(res);
        return;
    }

    while (((lineLimit == 0U) || (linesRead < lineLimit)) &&
            (f_gets(g_buff, sizeof(g_buff), &file) != NULL))
    {
        printf("%s", g_buff);
        linesRead++;
    }

    if (f_error(&file) != 0)
    {
        printf("Read %s fail. ", s_argv[1]);
        put_rc((FRESULT)f_error(&file));
    }

    res = f_close(&file);

    if (res != FR_OK)
    {
        printf("Close %s fail. ", s_argv[1]);
        put_rc(res);
    }
}

static void cmd_delete(void)
{
    FRESULT res;

    if (s_argc != 2)
    {
        printf("Usage: rm <filename>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_unlink(s_argv[1]);

    if (res == FR_OK)
        printf("Deleted %s.\n", s_argv[1]);
    else
    {
        printf("Delete %s fail. ", s_argv[1]);
        put_rc(res);
    }
}

static void cmd_rename(void)
{
    FRESULT res;

    if (s_argc != 3)
    {
        printf("Usage: mv <src> <dst>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_rename(s_argv[1], s_argv[2]);

    if (res == FR_OK)
        printf("Renamed %s to %s.\n", s_argv[1], s_argv[2]);
    else
    {
        printf("Rename fail. ");
        put_rc(res);
    }
}

static void cmd_copy(void)
{
    FIL sourceFile = {0};
    FIL destFile = {0};
    FRESULT res;
    FRESULT closeRes;
    UINT bytesRead;
    UINT bytesWritten = 0U;
    uint32_t totalBytes = 0U;

    if (s_argc != 3)
    {
        printf("Usage: cp <src> <dst>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    if (strcmp(s_argv[1], s_argv[2]) == 0)
    {
        printf("Source and destination must be different.\n");
        return;
    }

    res = f_open(&sourceFile, s_argv[1], FA_READ);

    if (res != FR_OK)
    {
        printf("Open %s fail. ", s_argv[1]);
        put_rc(res);
        return;
    }

    res = f_open(&destFile, s_argv[2], FA_CREATE_ALWAYS | FA_WRITE);

    while (res == FR_OK)
    {
        res = f_read(&sourceFile, g_buff, sizeof(g_buff), &bytesRead);

        if ((res != FR_OK) || (bytesRead == 0U))
            break;

        bytesWritten = 0U;
        res = f_write(&destFile, g_buff, bytesRead, &bytesWritten);

        if ((res == FR_OK) && (bytesWritten != bytesRead))
            res = FR_DISK_ERR;

        totalBytes += bytesWritten;
    }

    if (destFile.obj.fs != NULL)
    {
        closeRes = f_close(&destFile);

        if (res == FR_OK)
            res = closeRes;
    }

    closeRes = f_close(&sourceFile);

    if (res == FR_OK)
        res = closeRes;

    if (res == FR_OK)
        printf("Copied %lu bytes from %s to %s.\n", (unsigned long)totalBytes,
               s_argv[1], s_argv[2]);
    else
    {
        printf("Copy %s to %s fail. ", s_argv[1], s_argv[2]);
        put_rc(res);
    }
}

static void cmd_mkdir(void)
{
    FRESULT res;

    if (s_argc != 2)
    {
        printf("Usage: mkdir <dirname>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_mkdir(s_argv[1]);

    if (res == FR_OK)
        printf("Directory %s created.\n", s_argv[1]);
    else
    {
        printf("Create directory %s fail. ", s_argv[1]);
        put_rc(res);
    }
}

static void cmd_rmdir(void)
{
    FRESULT res;

    if (s_argc != 2)
    {
        printf("Usage: rmdir <dirname>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_rmdir(s_argv[1]);

    if (res == FR_OK)
        printf("Directory %s removed.\n", s_argv[1]);
    else
    {
        printf("Remove directory %s fail. ", s_argv[1]);
        put_rc(res);
    }
}

static void cmd_find(void)
{
    DIR dir = {0};
    FILINFO info;
    FRESULT res;
    const char *path = (s_argc == 2) ? s_argv[1] : ".";

    if (s_argc > 2)
    {
        printf("Usage: ls [path]\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_opendir(&dir, path);

    while (res == FR_OK)
    {
        res = f_readdir(&dir, &info);

        if ((res != FR_OK) || (info.fname[0] == '\0'))
            break;

        printf("%c%c%c%c%c %10lu  %s\n",
               (info.fattrib & AM_DIR) ? 'D' : '-',
               (info.fattrib & AM_RDO) ? 'R' : '-',
               (info.fattrib & AM_HID) ? 'H' : '-',
               (info.fattrib & AM_SYS) ? 'S' : '-',
               (info.fattrib & AM_ARC) ? 'A' : '-',
               (unsigned long)info.fsize, info.fname);
    }

    if (dir.obj.fs != NULL)
        (void)f_closedir(&dir);

    if (res != FR_OK)
    {
        printf("List %s fail. ", path);
        put_rc(res);
    }
}

static void cmd_stat(void)
{
    FILINFO info;
    FRESULT res;

    if (s_argc != 2)
    {
        printf("Usage: stat <path>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_stat(s_argv[1], &info);

    if (res == FR_OK)
    {
        printf("Name: %s\nSize: %lu bytes\nAttributes: %c%c%c%c%c\n",
               info.fname, (unsigned long)info.fsize,
               (info.fattrib & AM_DIR) ? 'D' : '-',
               (info.fattrib & AM_RDO) ? 'R' : '-',
               (info.fattrib & AM_HID) ? 'H' : '-',
               (info.fattrib & AM_SYS) ? 'S' : '-',
               (info.fattrib & AM_ARC) ? 'A' : '-');
    }
    else
    {
        printf("Stat %s fail. ", s_argv[1]);
        put_rc(res);
    }
}

static void cmd_pwd(void)
{
    FRESULT res;

    if (s_argc != 1)
    {
        printf("Usage: pwd\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_getcwd(g_buff, sizeof(g_buff));

    if (res == FR_OK)
        printf("%s\n", g_buff);
    else
    {
        printf("Get current directory fail. ");
        put_rc(res);
    }
}

static void cmd_chdir(void)
{
    FRESULT res;

    if (s_argc != 2)
    {
        printf("Usage: cd <path>\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_chdir(s_argv[1]);

    if (res != FR_OK)
    {
        printf("Change directory to %s fail. ", s_argv[1]);
        put_rc(res);
    }
}

static void cmd_remount(void)
{
    (void)f_unmount(DRIVE_NAME);
    s_isMounted = 0U;
    cmd_mount();
}

static void cmd_size(void)
{
    FATFS *fs;
    FRESULT res;
    DWORD freeClusters;
    uint32_t totalKB;
    uint32_t freeKB;

    if (s_argc != 1)
    {
        printf("Usage: df\n");
        return;
    }

    if (require_mounted() == 0U)
        return;

    res = f_getfree(DRIVE_NAME, &freeClusters, &fs);

    if (res != FR_OK)
    {
        printf("Get drive space fail. ");
        put_rc(res);
        return;
    }

    totalKB = ((fs->n_fatent - 2U) * fs->csize) / (1024U / FF_MAX_SS);
    freeKB = (freeClusters * fs->csize) / (1024U / FF_MAX_SS);
    printf("Total: %lu KB\nFree : %lu KB\n", (unsigned long)totalKB,
           (unsigned long)freeKB);
}

static void cmd_help(void)
{
    uint32_t i;

    for (i = 0U; i < (sizeof(cmd_list) / sizeof(cmd_list[0])); i++)
        printf("%-9s - %s\n", cmd_list[i].cmd, cmd_list[i].info);
}

static void process_command(char *commandLine)
{
    uint32_t i;
    char *p = commandLine;

    s_argc = 0;

    while ((*p != '\0') && (s_argc < (int32_t)CMD_ARG_COUNT))
    {
        while (isspace((unsigned char) * p))
            p++;

        if (*p == '\0')
            break;

        s_argv[s_argc++] = p;

        while ((*p != '\0') && !isspace((unsigned char) * p))
            p++;

        if (*p != '\0')
            *p++ = '\0';
    }

    if (s_argc == 0)
        return;

    while (isspace((unsigned char) * p))
        p++;

    if (*p != '\0')
    {
        printf("Too many arguments (maximum %u).\n", CMD_ARG_COUNT - 1U);
        return;
    }

    for (i = 0U; i < (sizeof(cmd_list) / sizeof(cmd_list[0])); i++)
    {
        if (strcmp(s_argv[0], cmd_list[i].cmd) == 0)
        {
            cmd_list[i].func();
            return;
        }
    }

    printf("Unknown command: %s. Use '?' for help.\n", s_argv[0]);
}

/*------------------------------*/
/*  Main Function               */
/*------------------------------*/
int32_t main(void)
{
    char commandLine[CMD_LINE_SIZE];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("+-------------------------------------------------------+\n");
    printf("|            SPI SD Card FatFs Command Shell            |\n");
    printf("+-------------------------------------------------------+\n");

    cmd_mount();
    cmd_help();
    printf("\nfatfs> ");

    for (;;)
    {
        (void)read_console_line(commandLine, sizeof(commandLine));
        process_command(commandLine);
        printf("fatfs> ");
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
