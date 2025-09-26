#include "Driver_Storage.h"
#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS2:Keil RTX5
#include "NuMicro.h"

/* Storage driver instance */
extern ARM_DRIVER_STORAGE Driver_Storage0;
static ARM_DRIVER_STORAGE *StorageDev = &Driver_Storage0;

/* CMSIS-RTOS2 Thread Id */
osThreadId_t Storage_Thread_Id;

#define BUFFER_BYTE_SIZE     FMC_FLASH_PAGE_SIZE
uint32_t g_au32ReadBuf[BUFFER_BYTE_SIZE / 4],
         g_au32WriteBuf[BUFFER_BYTE_SIZE / 4];

/* Storage signal event */
void Storage_Callback(int32_t status, ARM_STORAGE_OPERATION operation)
{
    NVT_UNUSED(status);
    NVT_UNUSED(operation);
}

/* CMSIS-RTOS2 Thread */
void Storage_Thread(void *argument)
{
    int32_t  rc;
    uint32_t i;
    ARM_STORAGE_BLOCK sNextBlock;
    ARM_STORAGE_INFO  sStorageInfo;

    /* Query drivers capabilities */
    const ARM_STORAGE_CAPABILITIES capabilities = StorageDev->GetCapabilities();

    NVT_UNUSED(argument);
    StorageDev->GetInfo(&sStorageInfo);

    /* Initialize Storage device */
    if (capabilities.asynchronous_ops)
    {
        StorageDev->Initialize(&Storage_Callback);
    }
    else
    {
        StorageDev->Initialize(NULL);
    }

    /* Power-on Storage device */
    StorageDev->PowerControl(ARM_POWER_FULL);
    rc = StorageDev->GetNextBlock(NULL, &sNextBlock);

    while ((rc == ARM_DRIVER_OK) && ARM_STORAGE_VALID_BLOCK(&sNextBlock))
    {
        printf("\rTest 0x%08X ... ", (uint32_t)sNextBlock.addr);

        if (StorageDev->Erase(sNextBlock.addr, sNextBlock.size) < 0)
        {
            printf("Erase failed !\n");
            return ;
        }

        if (StorageDev->ReadData(sNextBlock.addr, g_au32ReadBuf, sNextBlock.size) < 0)
        {
            printf("ReadData failed !\n");
            return ;
        }

        for (i = 0; i < (sNextBlock.size / sStorageInfo.program_unit); i++)
        {
            if (g_au32ReadBuf[i] != 0xFFFFFFFF)
                printf("0x%08X\n", g_au32ReadBuf[i]);
        }

        /* Prepare write pattern */
        for (i = 0; i < (sNextBlock.size / sStorageInfo.program_unit); i++)
        {
            g_au32WriteBuf[i] = sNextBlock.addr + i * 4;
        }

        if (StorageDev->ProgramData(sNextBlock.addr, g_au32WriteBuf, sNextBlock.size) < 0)
        {
            printf("ProgramData failed !\n");
            return ;
        }

        if (StorageDev->ReadData(sNextBlock.addr, g_au32ReadBuf, sNextBlock.size) < 0)
        {
            printf("ReadData failed !\n");
            return ;
        }

        for (i = 0; i < (sNextBlock.size / sStorageInfo.program_unit); i++)
        {
            if (g_au32ReadBuf[i] != g_au32WriteBuf[i])
                printf("0x%08X != 0x%08X\n", g_au32ReadBuf[i], g_au32WriteBuf[i]);
        }

        printf("OK");

        /* Wait operation to be completed */
        if (capabilities.asynchronous_ops)
        {
            osThreadFlagsWait(1U, osFlagsWaitAny, 100U);
        }
        else
        {
            osDelay(100U);
        }

        rc = StorageDev->GetNextBlock(&sNextBlock, &sNextBlock);
    }

    /* Switch off gracefully */
    StorageDev->PowerControl(ARM_POWER_OFF);
    StorageDev->Uninitialize();

    printf("\nTest done\n");
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

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
    printf("\nCMSIS Driver Storage Test\n");

    osKernelInitialize();                       // Initialize CMSIS-RTOS2

    osThreadNew(Storage_Thread, NULL, NULL);      // Create validation main thread

    osKernelStart();                            // Start thread execution

    for (;;) {}
}
