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
#define TEST_BLOCK_ADDR     0x00010000
#define TEST_BLOCK_SIZE     512
#define TEST_BLOCK_COUNT    1
#define BUFFER_SIZE         2048

//------------------------------------------------------------------------------
extern ARM_DRIVER_MCI Driver_MCI0;   // MCI Driver exported by Driver_MCI.c

//------------------------------------------------------------------------------
// FATFS and buffer
FATFS fs;
static uint8_t gau8Buffer[BUFFER_SIZE];

//------------------------------------------------------------------------------
// FatFs RTC Dummy Implementation
DWORD get_fattime(void)
{
    return 0;
}

// MCI Callback
void MCI_EventCallback(uint32_t event)
{
    if (event & ARM_MCI_EVENT_COMMAND_COMPLETE)   printf("Command complete.\n");

    if (event & ARM_MCI_EVENT_TRANSFER_COMPLETE)  printf("Transfer complete.\n");

    if (event & ARM_MCI_EVENT_TRANSFER_ERROR)     printf("Transfer error.\n");

    if (event & ARM_MCI_EVENT_COMMAND_ERROR)      printf("Command error.\n");
}

int32_t MCI_ReadWriteTest(ARM_DRIVER_MCI *mci, uint32_t block_addr, uint32_t block_count, uint32_t block_size)
{
    uint8_t write_buffer[512];
    uint8_t read_buffer[512];
    uint32_t response[4];
    int32_t status;
    uint32_t i;

    if (block_size > 512 || block_count != 1)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // === Initialize MCI Interface ===
    status = mci->Initialize(MCI_EventCallback);

    if (status != ARM_DRIVER_OK) return status;

    status = mci->PowerControl(ARM_POWER_FULL);

    if (status != ARM_DRIVER_OK) return status;

    // === Generate test data ===
    for (i = 0; i < block_size; i++)
    {
        write_buffer[i] = 0x55 + (uint8_t)(i & 0xFF);
    }

    // === CMD24 - Write Single Block ===
    status = mci->SetupTransfer(write_buffer, block_count, block_size, ARM_MCI_TRANSFER_WRITE);

    if (status != ARM_DRIVER_OK)
    {
        printf("SetupTransfer (Write) failed: %d\n", status);
        return status;
    }

    status = mci->SendCommand(24, block_addr,
                              ARM_MCI_RESPONSE_SHORT | ARM_MCI_RESPONSE_CRC | ARM_MCI_TRANSFER_DATA, response);

    if (status != ARM_DRIVER_OK)
    {
        printf("CMD24 failed: %d\n", status);
        return status;
    }

    // === CMD17 - Read Single Block ===
    status = mci->SetupTransfer(read_buffer, block_count, block_size, ARM_MCI_TRANSFER_READ);

    if (status != ARM_DRIVER_OK)
    {
        printf("SetupTransfer (Read) failed: %d\n", status);
        return status;
    }

    status = mci->SendCommand(17, block_addr,
                              ARM_MCI_RESPONSE_SHORT | ARM_MCI_RESPONSE_CRC | ARM_MCI_TRANSFER_DATA, response);

    if (status != ARM_DRIVER_OK)
    {
        printf("CMD17 failed: %d\n", status);
        return status;
    }

    // === Verify Data ===
    for (i = 0; i < block_size; i++)
    {
        if (read_buffer[i] != write_buffer[i])
        {
            printf("Mismatch at [%u]: write=0x%02X, read=0x%02X\n", i, write_buffer[i], read_buffer[i]);
            return ARM_DRIVER_ERROR;
        }
    }

    printf("MCI Read/Write Test PASSED\n");
    return ARM_DRIVER_OK;
}

// Test: Send basic SD commands CMD0, CMD8
// Test: Send a series of common SD commands
void MCI_Validation_Command(void)
{
    int32_t i32Ret;
    uint32_t response;
    uint8_t u8Is_SD_v2 = 0;

    printf("=== Nuvoton M55 SDH with Command ===\n");

    assert(Driver_MCI0.Initialize(MCI_EventCallback) == ARM_DRIVER_OK);

    assert(Driver_MCI0.PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);

    //assert(Driver_MCI0.Control(ARM_MCI_BUS_SPEED, 400000) == ARM_DRIVER_OK);
    printf("Bus Speed set to 400KHz for init %d\n", Driver_MCI0.Control(ARM_MCI_BUS_SPEED, 25000000));

    // CMD0 - GO_IDLE_STATE
    assert(Driver_MCI0.SendCommand(0, 0, ARM_MCI_RESPONSE_NONE, NULL) == ARM_DRIVER_OK);
    printf("CMD0 OK (Go Idle)\n");

    // CMD8 - Check SD version (Only SD 2.0+ responds)
    i32Ret = Driver_MCI0.SendCommand(8, 0x1AA, ARM_MCI_RESPONSE_SHORT | ARM_MCI_RESPONSE_CRC, &response);

    if (i32Ret == ARM_DRIVER_OK && (response & 0xFFF) == 0x1AA)
    {
        u8Is_SD_v2 = 1;  // SD 2.0 or above
        printf("[MCI] SD 2.0+ Card Detected\n");
    }
    else
    {
        printf("[MCI] SD 1.1 Card Detected (CMD8 Failed)\n");
    }

    // ACMD41 loop - Card initialization
    uint32_t arg = u8Is_SD_v2 ? (0x40FF8000) : (0x00FF8000); // HCS bit for SD2.0

    // CMD55 - APP_CMD (Prepare for ACMD)
    assert(Driver_MCI0.SendCommand(55, 0, ARM_MCI_RESPONSE_SHORT, &response) == ARM_DRIVER_OK);
    printf("CMD55 OK - Response: 0x%08X\n", response);

    // ACMD41 - SD_SEND_OP_COND (Start card initialization process)
    //assert(Driver_MCI0.SendCommand(41, arg, ARM_MCI_RESPONSE_SHORT, &response) == ARM_DRIVER_OK);
    Driver_MCI0.SendCommand(41, arg, ARM_MCI_RESPONSE_SHORT, &response);
    printf("1 ACMD41 OK - OCR: 0x%08X\n", response);

    // CMD55 - APP_CMD (Prepare for ACMD)
    assert(Driver_MCI0.SendCommand(55, 0, ARM_MCI_RESPONSE_SHORT, &response) == ARM_DRIVER_OK);
    printf("CMD55 OK - Response: 0x%08X\n", response);

    Driver_MCI0.SendCommand(41, arg, ARM_MCI_RESPONSE_SHORT, &response);
    printf("2 ACMD41 OK - OCR: 0x%08X\n", response);

    // CMD2 - ALL_SEND_CID (Get Card ID)
    assert(Driver_MCI0.SendCommand(2, 0, ARM_MCI_RESPONSE_LONG, &response) == ARM_DRIVER_OK);
    printf("CMD2 OK (CID received)\n");

    // CMD3 - SEND_RELATIVE_ADDR (Get RCA)
    assert(Driver_MCI0.SendCommand(3, 0, ARM_MCI_RESPONSE_SHORT, &response) == ARM_DRIVER_OK);
    printf("CMD3 OK - RCA: 0x%08X\n", response);
    uint32_t rca = response & 0xFFFF0000;

    // CMD7 - SELECT_CARD
    assert(Driver_MCI0.SendCommand(7, rca, ARM_MCI_RESPONSE_SHORT, &response) == ARM_DRIVER_OK);
    printf("CMD7 OK - Card selected\n");

    // CMD9 - SEND_CSD
    assert(Driver_MCI0.SendCommand(9, rca, ARM_MCI_RESPONSE_LONG, &response) == ARM_DRIVER_OK);
    printf("CMD9 OK (CSD received)\n");

    // CMD13 - SEND_STATUS
    assert(Driver_MCI0.SendCommand(13, rca, ARM_MCI_RESPONSE_SHORT, &response) == ARM_DRIVER_OK);
    printf("CMD13 OK - Card status: 0x%08X\n", response);

    // CMD16 - SET_BLOCKLEN (Usually 512 bytes)
    assert(Driver_MCI0.SendCommand(16, 512, ARM_MCI_RESPONSE_SHORT, &response) == ARM_DRIVER_OK);
    printf("CMD16 OK - Block length set to 512 bytes\n");

    // CMD17 - READ_SINGLE_BLOCK (Read block 0)
    assert(Driver_MCI0.SendCommand(17, 0, ARM_MCI_RESPONSE_SHORT | ARM_MCI_TRANSFER_DATA, &response) == ARM_DRIVER_OK);
    printf("CMD17 OK - Single block read\n");

    // CMD24 - WRITE_SINGLE_BLOCK (Write block 0)
    assert(Driver_MCI0.SendCommand(24, 0, ARM_MCI_RESPONSE_SHORT | ARM_MCI_TRANSFER_DATA, &response) == ARM_DRIVER_OK);
    printf("CMD24 OK - Single block write\n");

    // CMD12 - STOP_TRANSMISSION
    //assert(Driver_MCI0.SendCommand(12, 0, ARM_MCI_RESPONSE_SHORT, &response) == ARM_DRIVER_OK);
    Driver_MCI0.SendCommand(12, 0, ARM_MCI_RESPONSE_SHORT, &response);
    printf("CMD12 OK - Transmission stopped\n");

    assert(Driver_MCI0.PowerControl(ARM_POWER_OFF) == ARM_DRIVER_OK);
    assert(Driver_MCI0.Uninitialize() == ARM_DRIVER_OK);

    printf("[ SD Command Test Done ]\n");
}

//------------------------------------------------------------------------------
// Test: FatFs Write and Read
void Test_FatFs_WriteRead(void)
{
    FIL file;
    const char *text = "Hello, Nuvoton SDIO!!!!";
    UINT bw, br;

    printf("\n[ FatFs Write/Read Test Start ]\n");
    assert(f_open(&file, "test.txt", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK);
    assert(f_write(&file, text, strlen(text), &bw) == FR_OK);
    f_close(&file);
    printf("File write done.\n");

    memset(gau8Buffer, 0, BUFFER_SIZE);
    assert(f_open(&file, "test.txt", FA_READ) == FR_OK);
    assert(f_read(&file, gau8Buffer, BUFFER_SIZE - 1, &br) == FR_OK);
    f_close(&file);
    printf("File read done, content: %s\n", gau8Buffer);
    printf("[ FatFs Write/Read Test Done ]\n");
}

//------------------------------------------------------------------------------
// Test: FatFs File Delete
void Test_FatFs_Delete(void)
{
    printf("\n[ FatFs File Delete Test Start ]\n");

    if (f_unlink("test.txt") == FR_OK)
    {
        printf("File deleted.\n");
    }
    else
    {
        printf("File delete failed.\n");
    }

    printf("[ FatFs File Delete Test Done ]\n");
}

//------------------------------------------------------------------------------
// Test: FatFs Create / Delete Directory
void Test_FatFs_Dir(void)
{
    printf("\n[ FatFs Directory Test Start ]\n");
    assert(f_mkdir("testdir") == FR_OK);
    printf("Directory created.\n");
    assert(f_rmdir("testdir") == FR_OK);
    printf("Directory deleted.\n");
    printf("[ FatFs Directory Test Done ]\n");
}

//------------------------------------------------------------------------------
// Test: List Directory
void Test_ListDirectory(void)
{
    DIR dir;
    FILINFO fno;
    FRESULT res;

    printf("\n[ FatFs Directory Listing ]\n");
    res = f_opendir(&dir, "");

    if (res == FR_OK)
    {
        while (1)
        {
            res = f_readdir(&dir, &fno);

            if (res != FR_OK || fno.fname[0] == 0) break;

            printf("%s %s (%u bytes)\n",
                   (fno.fattrib & AM_DIR) ? "[DIR]" : "[FILE]",
                   fno.fname, fno.fsize);
        }

        f_closedir(&dir);
    }
    else
    {
        printf("Failed to open root directory\n");
    }
}

void MCI_Teardown(void)
{
    printf("\n[ MCI Teardown Start ]\n");
    f_mount(NULL, "", 0);
    assert(Driver_MCI0.PowerControl(ARM_POWER_OFF) == ARM_DRIVER_OK);
    assert(Driver_MCI0.Uninitialize() == ARM_DRIVER_OK);
    printf("[ MCI Teardown Done ]\n");
}

void MCI_Setup(void)
{
    printf("\n[ MCI Setup Start ]\n");

    assert(Driver_MCI0.Initialize(MCI_EventCallback) == ARM_DRIVER_OK);
    assert(Driver_MCI0.PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);

    printf("Bus Speed set to 400KHz for init %d\n", Driver_MCI0.Control(ARM_MCI_BUS_SPEED, 400000));

    assert(f_mount(&fs, "", 0) == FR_OK);

    printf("[ MCI Setup Done ]\n");
}

uint32_t SDH_INTSTS_Read(void)
{
    return SDH0->INTSTS;
}

void SD_Card_Init_Test(void)
{
    int32_t i32Ret;
    uint32_t response;
    uint8_t u8Is_SD_v2 = 0;
    uint8_t u8Is_SDHC = 0;
    uint8_t u8Is_MMC = 0;
    uint32_t timeout;
    uint32_t arg;

    printf("=== Nuvoton M55 SDH SD Card Init Test ===\n");

    assert(Driver_MCI0.Initialize(&MCI_EventCallback) == ARM_DRIVER_OK);
    assert(Driver_MCI0.PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    Driver_MCI0.Control(ARM_MCI_BUS_SPEED, 25000000);

    // CMD0 - GO_IDLE_STATE
    assert(Driver_MCI0.SendCommand(0, 0, ARM_MCI_RESPONSE_NONE, NULL) == ARM_DRIVER_OK);
    printf("CMD0 OK\n");

    // CMD8 - Check SD version
    i32Ret = Driver_MCI0.SendCommand(8, 0x1AA, ARM_MCI_RESPONSE_SHORT | ARM_MCI_RESPONSE_CRC, &response);

    if (i32Ret == ARM_DRIVER_OK && (response & 0xFFF) == 0x1AA)
    {
        u8Is_SD_v2 = 1;
        printf("SD 2.0+ Card Detected\n");
    }
    else
    {
        u8Is_SD_v2 = 0;
        printf("SD 1.x or MMC Detected (CMD8 Failed)\n");
    }

    timeout = 1000;
    arg = u8Is_SD_v2 ? 0x40FF8000 : 0x00FF8000;

    // Try ACMD41 loop (SD cards)
    do
    {
        i32Ret = Driver_MCI0.SendCommand(55, 0, ARM_MCI_RESPONSE_SHORT, &response);

        if (i32Ret != ARM_DRIVER_OK)
        {
            printf("CMD55 Retry...\n");
            continue;
        }

        i32Ret = Driver_MCI0.SendCommand(41, arg, ARM_MCI_RESPONSE_SHORT, &response);

        if (i32Ret != ARM_DRIVER_OK)
        {
            printf("ACMD41 Retry...\n");
            continue;
        }

        printf("ACMD41 OCR = 0x%08X\n", response);

        if (response & 0x00800000)
        {
            printf("Card Ready via ACMD41\n");

            if (response & 0x40000000)
            {
                u8Is_SDHC = 1;
                printf("Card is SDHC/SDXC\n");
            }
            else
            {
                printf("Card is SDSC\n");
            }

            goto finish_init;
        }
    } while (--timeout > 0);

    printf("ACMD41 Timeout or failed. Assume MMC and try CMD1.\n");

    // Try CMD1 for MMC
    timeout = 1000;

    do
    {
        i32Ret = Driver_MCI0.SendCommand(1, 0x00FF8000, ARM_MCI_RESPONSE_SHORT, &response);

        if (i32Ret == ARM_DRIVER_OK && (response & 0x80000000))
        {
            printf("MMC Card Ready via CMD1\n");
            u8Is_MMC = 1;
            goto finish_init;
        }
    } while (--timeout > 0);

    printf("Card not ready. Initialization failed.\n");
    return;

finish_init:
    // CMD2 - Get CID
    assert(Driver_MCI0.SendCommand(2, 0, ARM_MCI_RESPONSE_LONG, &response) == ARM_DRIVER_OK);
    printf("CMD2 OK (CID received)\n");

    // CMD3 - Get RCA
    assert(Driver_MCI0.SendCommand(3, 0, ARM_MCI_RESPONSE_SHORT | ARM_MCI_RESPONSE_CRC, &response) == ARM_DRIVER_OK);
    printf("CMD3 R6 RESP = 0x%08X\n", response);

    uint32_t rca = response & 0xFFFF0000;
    printf("CMD3 OK - RCA: 0x%08X\n", rca);

    // CMD7 - Select Card
    i32Ret = Driver_MCI0.SendCommand(7, rca, ARM_MCI_RESPONSE_SHORT | ARM_MCI_RESPONSE_CRC, &response);

    if (i32Ret != ARM_DRIVER_OK)
    {
        printf("CMD7 Failed. Possibly incorrect RCA: 0x%08X\n", rca);
        return;
    }

    // CMD7 busy polling (DAT0 should go high)
    timeout = 100000;
    Driver_MCI0.Control(0x80000000 /* User-defined: issue CLK8OEN */, 1);

    while (--timeout > 0)
    {
        if (SDH_INTSTS_Read() & (1 << 7)) break; // SDH_INTSTS_DAT0STS_Msk = bit 7
    }

    if (timeout == 0)
    {
        printf("CMD7 Timeout: DAT0 busy\n");
        return;
    }

    printf("CMD7 OK (Card selected)\n");

    // CMD13 - Polling card state only (ignore READY_FOR_DATA)
    timeout = 1000;

    do
    {
        i32Ret = Driver_MCI0.SendCommand(13, rca, ARM_MCI_RESPONSE_SHORT, &response);

        if (i32Ret != ARM_DRIVER_OK)
        {
            printf("CMD13 Failed\n");
            return;
        }

        uint32_t card_state = (response >> 9) & 0xF;
        printf("CMD13 Status: 0x%08X, State: %u\n", response, card_state);

        if (card_state == 4)
        {
            printf("Card is in TRAN state. Proceeding.\n");
            break;
        }
    } while (--timeout > 0);

    if (timeout == 0)
    {
        printf("CMD13 timeout waiting for TRAN state\n");
        return;
    }

    printf("[ SD Card Init Sequence Done ]\n");

    if (u8Is_MMC) printf("? Detected Card Type: MMC\n");
    else if (u8Is_SDHC) printf("? Detected Card Type: SDHC/SDXC\n");
    else printf("? Detected Card Type: SDSC\n");

    Driver_MCI0.PowerControl(ARM_POWER_OFF);
    Driver_MCI0.Uninitialize();
}

void MCI_Validation_FatFs(void)
{
    //SD_Card_Init_Test();
    //MCI_Validation_Command();

    // Assume Driver_MCI0 is properly initialized and powered up
    MCI_ReadWriteTest(&Driver_MCI0, 0x1000, 1, 512);

    printf("=== Nuvoton M55 SDH with FatFs Sample ===\n");

    MCI_Setup();

    Test_FatFs_WriteRead();
    Test_ListDirectory();
    Test_FatFs_Delete();
    Test_FatFs_Dir();

    MCI_Teardown();

    printf("Test complete.\n");
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

    printf("\nCMSIS Driver MCI Test\n");

    osKernelInitialize();                       // Initialize CMSIS-RTOS2

    osThreadNew((osThreadFunc_t)MCI_Validation_FatFs, NULL, NULL);      // Create validation main thread

    osKernelStart();                            // Start thread execution

    for (;;) {}
}
