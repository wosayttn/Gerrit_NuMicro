#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "Driver_MCI.h"
#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS2:Keil RTX5
#include "NuMicro.h"

#include "diskio.h"
#include "ff.h"

//------------------------------------------------------------------------------
extern ARM_DRIVER_MCI Driver_MCI0;   // MCI Driver exported by Driver_MCI.c

//------------------------------------------------------------------------------
#define MCI_DRV (&Driver_MCI0)
/* Application stack size */
#define APP_MAIN_STK_SZ (2048U)
#define CMD_LIST_SIZE (sizeof(cmd_list)/sizeof(cmd_list[0]))

//------------------------------------------------------------------------------
const osThreadAttr_t app_main_attr =
{
    .stack_size = APP_MAIN_STK_SZ
};

/* FatFS structures */
static FATFS fs;
static FIL fil;
static DIR dir;
static FILINFO fno;
static char cmd_line[300];
static char cwd[256] = "/";

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
    { cmd_write,   "write",   "Write demo data to a file: write <filename> [lines]" },
    { cmd_append,  "append",  "Append to file: append <filename>" },
    { cmd_create,  "touch",   "Create empty file: create <filename>" },
    { cmd_read,    "read",    "Read lines from a file: read <filename> [lines]" },
    { cmd_delete,  "rm",      "Delete a file" },
    { cmd_rename,  "mv",      "Rename a file: rename <src> <dst>" },
    { cmd_copy,    "cp",      "Copy a file: copy <src> <dst>" },
    { cmd_mkdir,   "mkdir",   "Make a directory" },
    { cmd_rmdir,   "rmdir",   "Remove a directory" },
    { cmd_find,    "ls",      "List files in directory" },
    { cmd_stat,    "stat",    "Show file attributes" },
    { cmd_pwd,     "pwd",     "Print working directory" },
    { cmd_chdir,   "cd",      "Change working directory" },
    { cmd_remount, "remount", "Re-initialize SD card (after reinsert)" },
    { cmd_size,    "df",      "Show total and free SD card space" },
    { cmd_help,    "?",       "Show help" },
};

// FatFs RTC Dummy Implementation
DWORD get_fattime(void)
{
    return 0;
}

static void cmd_mount(void)
{
    FRESULT res = f_mount(&fs, "", 1);
    printf(res == FR_OK ? "Drive mounted.\n" : "Mount failed (%d)\n", res);
}

static void cmd_unmount(void)
{
    FRESULT res = f_mount(NULL, "0:", 1);

    if (res == FR_OK || res == FR_INVALID_DRIVE || res == FR_NOT_ENABLED)
        printf("Unmount OK\n");
    else
        printf("Unmount failed: %d\n", res);
}

static void cmd_format(void)
{
    static uint8_t workbuf[FF_MAX_SS];
    MKFS_PARM opt = {FM_FAT, 0, 0, 0, 0};
    FRESULT res = f_mkfs("0:", &opt, workbuf, sizeof(workbuf));  // ???? "0:"
    printf(res == FR_OK ? "Drive formatted.\n" : "Format failed (%d)\n", res);
}

static void cmd_write(void)
{
    char *file = strtok(NULL, " ");

    if (!file)
    {
        printf("Usage: write <filename>\n");
        return;
    }

    FRESULT res = f_open(&fil, file, FA_WRITE | FA_CREATE_ALWAYS);

    if (res != FR_OK)
    {
        printf("Open failed: %d\n", res);
        return;
    }

    printf("Enter text lines to write to %s (end with empty line):\n", file);
    char input[256];
    UINT bw;

    while (1)
    {
        printf("> ");

        if (!fgets(input, sizeof(input), stdin)) break;

        size_t len = strcspn(input, "\r\n");
        input[len] = '\0';

        if (len == 0) break;  // empty line to stop

        strcat(input, "\r\n");
        f_write(&fil, input, strlen(input), &bw);
    }

    f_close(&fil);
    printf("Write complete.\n");
}

static void cmd_append(void)
{
    char *file = strtok(NULL, " ");

    if (!file)
    {
        printf("Usage: append <filename>\n");
        return;
    }

    FRESULT res = f_open(&fil, file, FA_OPEN_APPEND | FA_WRITE);

    if (res != FR_OK)
    {
        printf("Append open failed: %d\n", res);
        return;
    }

    printf("Enter text lines to append to %s (end with empty line):\n", file);
    char input[256];
    UINT bw;

    while (1)
    {
        printf("> ");

        if (!fgets(input, sizeof(input), stdin)) break;

        size_t len = strcspn(input, "\r\n");
        input[len] = '\0';

        if (len == 0) break;  // empty line to stop

        strcat(input, "\r\n");
        f_write(&fil, input, strlen(input), &bw);
    }

    f_close(&fil);
    printf("Append complete.\n");
}

static void cmd_create(void)
{
    char *file = strtok(NULL, " ");

    if (!file)
    {
        printf("Usage: create <filename>\n");
        return;
    }

    FIL newfile;
    FRESULT res = f_open(&newfile, file, FA_CREATE_ALWAYS | FA_WRITE);

    if (res == FR_OK)
    {
        f_close(&newfile);
        printf("File '%s' created.\n", file);
    }
    else
    {
        printf("Create failed (%d)\n", res);
    }
}

static void cmd_read(void)
{
    char *file = strtok(NULL, " ");
    char *nstr = strtok(NULL, " ");
    uint32_t limit = (nstr != NULL) ? atoi(nstr) : 0;

    if (!file)
    {
        printf("Usage: read <filename> [lines]\n");
        return;
    }

    char line[128];
    FRESULT res = f_open(&fil, file, FA_READ);

    if (res != FR_OK) return (void)printf("Open failed: %d\n", res);

    uint32_t lines = 0;

    while (f_gets(line, sizeof(line), &fil))
    {
        printf("%s", line);

        if (limit && ++lines >= limit) break;
    }

    f_close(&fil);
    printf("File closed.\n");
}

static void cmd_delete(void)
{
    char *path = strtok(NULL, " ");

    if (!path) return;

    FRESULT res = f_unlink(path);
    printf("Delete %s: %d\n", path, res);
}

static void cmd_rename(void)
{
    char *src = strtok(NULL, " ");
    char *dst = strtok(NULL, " ");

    if (!src || !dst) return;

    FRESULT res = f_rename(src, dst);
    printf("Rename: %d\n", res);
}

static void cmd_copy(void)
{
    char *src = strtok(NULL, " ");
    char *dst = strtok(NULL, " ");

    if (!src || !dst) return;

    FIL src_fil, dst_fil;
    char buf[128];
    UINT br, bw;
    FRESULT res = f_open(&src_fil, src, FA_READ);

    if (res != FR_OK) return (void)printf("Copy open src failed: %d\n", res);

    res = f_open(&dst_fil, dst, FA_CREATE_ALWAYS | FA_WRITE);

    if (res != FR_OK) return (void)printf("Copy open dst failed: %d\n", res);

    while (1)
    {
        f_read(&src_fil, buf, sizeof(buf), &br);

        if (br == 0) break;

        f_write(&dst_fil, buf, br, &bw);
    }

    f_close(&src_fil);
    f_close(&dst_fil);
    printf("Copy completed.\n");
}

static void cmd_mkdir(void)
{
    char *path = strtok(NULL, " ");

    if (!path) return;

    FRESULT res = f_mkdir(path);
    printf("mkdir %s: %d\n", path, res);
}

static void cmd_rmdir(void)
{
    char *path = strtok(NULL, " ");

    if (!path) return;

    FRESULT res = f_unlink(path);
    printf("rmdir %s: %d\n", path, res);
}

static void cmd_find(void)
{
    FRESULT res = f_opendir(&dir, cwd);

    if (res != FR_OK) return (void)printf("opendir failed: %d\n", res);

    printf("%-6s %-10s %-s\n", "Type", "Size", "Name");

    while (1)
    {
        res = f_readdir(&dir, &fno);

        if (res != FR_OK || fno.fname[0] == 0) break;

        printf("%-6s %-10lu %-s\n", (fno.fattrib & AM_DIR) ? "<DIR>" : "FILE",
               (unsigned long)fno.fsize, fno.fname);
    }

    f_closedir(&dir);
}

static void cmd_stat(void)
{
    char *path = strtok(NULL, " ");

    if (!path) return;

    FRESULT res = f_stat(path, &fno);

    if (res != FR_OK) return (void)printf("stat failed: %d\n", res);

    printf("Name: %s\nType: %s\nSize: %lu\nAttributes: 0x%02X\n",
           fno.fname, (fno.fattrib & AM_DIR) ? "Directory" : "File",
           (unsigned long)fno.fsize, fno.fattrib);
}

static void cmd_pwd(void)
{
    printf("%s\n", cwd);
}

static void cmd_chdir(void)
{
    char *path = strtok(NULL, " ");

    if (!path) return;

    if (f_chdir(path) == FR_OK)
    {
        f_getcwd(cwd, sizeof(cwd));
        printf("Changed directory to: %s\n", cwd);
    }
    else
    {
        printf("chdir failed\n");
    }
}

static void cmd_remount(void)
{
    ARM_DRIVER_MCI *mci = MCI_DRV;

    printf("Unmounting drive...\n");
    f_mount(NULL, "", 1);
    mci->Uninitialize();

    printf("Re-inserting SD card...\n");
    mci->Initialize(NULL);
    mci->PowerControl(ARM_POWER_FULL);
    //mci->CardPower(ARM_MCI_POWER_ON);
    int retries = 20;

    while (!mci->ReadCD() && --retries > 0)
    {
        osDelay(50);
    }

    if (retries == 0)
    {
        printf("SD card not detected.\n");
        return;
    }

    //mci->MediaInitialize();
    FRESULT res = f_mount(&fs, "", 1);

    if (res == FR_OK)
    {
        f_getcwd(cwd, sizeof(cwd));
        printf("Remount successful.\n");
    }
    else
    {
        printf("Remount failed: %d\n", res);
    }
}

//static void cmd_size(void) {
//  FATFS *fs_ptr;
//  DWORD free_clusters, total_sectors, free_sectors;
//
//  FRESULT res = f_getfree("0:", &free_clusters, &fs_ptr);
//  if (res != FR_OK) {
//    printf("Failed to get free space (%d)\n", res);
//    return;
//  }
//
//  total_sectors = (fs_ptr->n_fatent - 2) * fs_ptr->csize;
//  free_sectors  = free_clusters * fs_ptr->csize;
//
//  // ??? KB/MB
//  uint32_t sector_size = 512; // ?? sector ? 512 bytes
//  float total_mb = total_sectors * sector_size / 1024.0 / 1024.0;
//  float free_mb  = free_sectors * sector_size / 1024.0 / 1024.0;
//
//  printf("SD Card Size: Total: %.2f MB, Free: %.2f MB\n", total_mb, free_mb);
//}

static void print_human_readable_size(uint64_t bytes)
{
    const char *units[] = {"Bytes", "KB", "MB", "GB", "TB"};
    int unit_index = 0;
    double size = (double)bytes;

    while (size >= 1024.0 && unit_index < 4)
    {
        size /= 1024.0;
        unit_index++;
    }

    printf("%.2f %s", size, units[unit_index]);
}

static void cmd_size(void)
{
    FATFS *fs_ptr;
    DWORD free_clusters, total_sectors, free_sectors;
    uint32_t sector_size = 512; // ?? sector size ? 512 bytes

    FRESULT res = f_getfree("0:", &free_clusters, &fs_ptr);

    if (res != FR_OK)
    {
        printf("Failed to get free space (%d)\n", res);
        return;
    }

    total_sectors = (fs_ptr->n_fatent - 2) * fs_ptr->csize;
    free_sectors  = free_clusters * fs_ptr->csize;

    uint64_t total_bytes = (uint64_t)total_sectors * sector_size;
    uint64_t free_bytes  = (uint64_t)free_sectors  * sector_size;
    uint64_t used_bytes  = total_bytes - free_bytes;
    float usage_percent = total_bytes > 0 ? ((float)used_bytes / total_bytes) * 100.0f : 0.0f;

    printf("Filesystem Usage\n");
    printf("%-10s %-15s %-15s %-15s %-10s\n", "Drive", "Total", "Used", "Free", "Use%");
    printf("%-10s ", "0:");
    print_human_readable_size(total_bytes);
    printf("     ");
    print_human_readable_size(used_bytes);
    printf("     ");
    print_human_readable_size(free_bytes);
    printf("     %.1f%%\n", usage_percent);
}

static void cmd_help(void)
{
    for (int i = 0; i < (int)CMD_LIST_SIZE; i++)
    {
        printf("%-10s - %s\n", cmd_list[i].cmd, cmd_list[i].info);
    }
}

static void init_filesystem(void)
{
    ARM_DRIVER_MCI *mci = MCI_DRV;
    mci->Initialize(NULL);
    mci->PowerControl(ARM_POWER_FULL);

    f_mount(&fs, "", 1);
    f_getcwd(cwd, sizeof(cwd));
}

__NO_RETURN void app_main_thread(void *arg)
{
    (void)arg;
    init_filesystem();

    while (1)
    {
        printf("\nFatFS> ");

        int idx = 0;
        int ch;
        memset(cmd_line, 0, sizeof(cmd_line));

        while ((ch = getchar()) != '\n' && ch != '\r')
        {
            if ((ch == '\b' || ch == 127) && idx > 0)
            {
                idx--;
                printf("\b \b");
            }
            else if (ch >= 32 && ch < 127 && idx < (int)sizeof(cmd_line) - 1)
            {
                cmd_line[idx++] = (char)ch;
                putchar(ch);
            }
        }

        cmd_line[idx] = '\0';
        printf("\n");

        char *cmd = strtok(cmd_line, " ");

        if (!cmd) continue;

        int i;

        for (i = 0; i < (int)CMD_LIST_SIZE; i++)
        {
            if (strcasecmp(cmd, cmd_list[i].cmd) == 0)
            {
                cmd_list[i].func();
                break;
            }
        }

        if (i == CMD_LIST_SIZE)
        {
            printf("Unknown command\n");
        }
    }
}

__NO_RETURN static void sd_monitor_thread(void)
{
    ARM_DRIVER_MCI *mci = MCI_DRV;
    static int last_cd = -1;  // -1: ????,0: ??,1: ??

    for (;;)
    {
        int cd = mci->ReadCD();  // 0: no card, 1: card inserted

        if (cd != last_cd)
        {
            if (cd == 1)
            {
                printf("\n[SD] Card inserted. Mounting...\n");
                mci->Uninitialize();

                mci->Initialize(NULL);
                mci->PowerControl(ARM_POWER_FULL);

                FRESULT res = f_mount(&fs, "", 1);

                if (res == FR_OK)
                {
                    f_getcwd(cwd, sizeof(cwd));
                    printf("[SD] Mount successful.\n");
                }
                else
                {
                    printf("[SD] Mount failed: %d\n", res);
                }
            }
            else
            {
                printf("\n[SD] Card removed. Unmounting...\n");
                f_mount(NULL, "", 1);
            }

            last_cd = cd;
        }

        osDelay(500);  // ? 500ms ????
    }
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable SDH0 module clock source as HCLK and SDH0 module clock divider as 4 */
    CLK_EnableModuleClock(SDH0_MODULE);
    CLK_SetModuleClock(SDH0_MODULE, CLK_SDHSEL_SDH0SEL_HCLK0, CLK_SDHDIV_SDH0DIV(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set multi-function pin for SDH */
    /* CD: PB12(9), PD13(3) */
    //SET_SD0_nCD_PB12();
    SET_SD0_nCD_PD13();

    /* CLK: PB1(3), PE6(3) */
    //SET_SD0_CLK_PB1();
    SET_SD0_CLK_PE6();

    /* CMD: PB0(3), PE7(3) */
    //SET_SD0_CMD_PB0();
    SET_SD0_CMD_PE7();

    /* D0: PB2(3), PE2(3) */
    //SET_SD0_DAT0_PB2();
    SET_SD0_DAT0_PE2();

    /* D1: PB3(3), PE3(3) */
    //SET_SD0_DAT1_PB3();
    SET_SD0_DAT1_PE3();

    /* D2: PB4(3), PE4(3) */
    //SET_SD0_DAT2_PB4();
    SET_SD0_DAT2_PE4();

    /* D3: PB5(3)-, PE5(3) */
    //SET_SD0_DAT3_PB5();
    SET_SD0_DAT3_PE5();

    /* Lock protected registers */
    SYS_LockReg();
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();

    printf("\nCMSIS Driver SAI Test\n");

    osKernelInitialize();                       // Initialize CMSIS-RTOS2

    osThreadNew((osThreadFunc_t)app_main_thread, NULL, NULL);      // Create validation main thread
    osThreadNew((osThreadFunc_t)sd_monitor_thread, NULL, NULL);      // Create validation main thread

    osKernelStart();                            // Start thread execution

    for (;;) {}
}
