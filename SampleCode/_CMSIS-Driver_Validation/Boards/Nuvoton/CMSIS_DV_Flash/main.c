#include "Driver_Flash.h"
#include "cmsis_os2.h"                  /* ARM::CMSIS:RTOS2:Keil RTX5 */
#include "NuMicro.h"

#define debug_printf(...)       //printf
#define WAIT_FLAG_TIMEOUT       100U    /* Timeout value in milliseconds if OS_TICK_FREQ is 1000 */

/* Flash driver instance */
extern ARM_DRIVER_FLASH Driver_Flash0;
static ARM_DRIVER_FLASH *flashDev = &Driver_Flash0;

/* CMSIS-RTOS2 Thread Id */
osThreadId_t Flash_Thread_Id;

#define BUF_BYTE_SIZE   FMC_FLASH_PAGE_SIZE

#if (NVT_DCACHE_ON == 1)
uint32_t g_au32ReadBuf [DCACHE_ALIGN_LINE_SIZE(BUF_BYTE_SIZE / 4)] __ALIGNED(DCACHE_LINE_SIZE);
uint32_t g_au32WriteBuf[DCACHE_ALIGN_LINE_SIZE(BUF_BYTE_SIZE / 4)] __ALIGNED(DCACHE_LINE_SIZE);
#else
uint32_t g_au32ReadBuf [BUF_BYTE_SIZE / 4];
uint32_t g_au32WriteBuf[BUF_BYTE_SIZE / 4];
#endif

/* Flash signal event */
void Flash_Callback(uint32_t event)
{
    if (event & ARM_FLASH_EVENT_READY)
    {
        /* The read/program/erase operation is completed */
        osThreadFlagsSet(Flash_Thread_Id, 1U);
    }

    if (event & ARM_FLASH_EVENT_ERROR)
    {
        /* The read/program/erase operation is completed with errors */
        /* Call debugger or replace with custom error handling */
        printf("Got error event !\n");
    }
}

/* CMSIS-RTOS2 Thread */
void Flash_Thread(void *argument)
{
    uint32_t i, u32StartAddr;
    uint32_t u32Flags;

    NVT_UNUSED(argument);
    /* Query drivers capabilities */
    const ARM_FLASH_CAPABILITIES capabilities = flashDev->GetCapabilities();
    const ARM_FLASH_INFO *psFlashInfo = flashDev->GetInfo();

    /* Initialize Flash device */
    if (capabilities.event_ready)
    {
        flashDev->Initialize(&Flash_Callback);
    }
    else
    {
        flashDev->Initialize(NULL);
    }

    /* Power-on Flash device */
    if (flashDev->PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK)
    {
        printf("Power on failed !\n");
        return ;
    }

    /* Loop through all sectors to test erase, read and program */
    for (u32StartAddr = 0; u32StartAddr < (psFlashInfo->sector_count * psFlashInfo->sector_size); u32StartAddr += psFlashInfo->sector_size)
    {
        printf("\rTest 0x%08X ... ", u32StartAddr);

        if (flashDev->EraseSector(u32StartAddr) != 0)
        {
            printf("EraseSector failed !\n");
            return ;
        }

        /* Wait operation to be completed */
        u32Flags = osThreadFlagsWait(1U, osFlagsWaitAny, WAIT_FLAG_TIMEOUT);

        if (u32Flags & BIT31)       /* Check if BIT31 is set */
        {
            /* An error occurred. Check the specific error code. */
            if (u32Flags == (uint32_t)osErrorTimeout)
            {
                printf("EraseSector timeout !\n");
            }
            else
            {
                /* Unexpected error occurred (e.g., osErrorResource, osErrorParameter, etc.) */
                printf("Unexpected error !\n");
            }
        }

        /* Read back to verify erased data is 0xFFFFFFFF */
        if (flashDev->ReadData(u32StartAddr, g_au32ReadBuf, (sizeof(g_au32ReadBuf) >> capabilities.data_width)) < 0)
        {
            printf("ReadData failed !\n");
            return ;
        }

        /* Wait operation to be completed */
        u32Flags = osThreadFlagsWait(1U, osFlagsWaitAny, WAIT_FLAG_TIMEOUT);

        if (u32Flags & BIT31)       /* Check if BIT31 is set */
        {
            /* An error occurred. Check the specific error code. */
            if (u32Flags == (uint32_t)osErrorTimeout)
            {
                printf("EraseSector timeout !\n");
            }
            else
            {
                /* Unexpected error occurred (e.g., osErrorResource, osErrorParameter, etc.) */
                printf("Unexpected error !\n");
            }
        }

        /* Verify erased data */
        for (i = 0; i < (sizeof(g_au32ReadBuf) >> capabilities.data_width); i++)
        {
            if (g_au32ReadBuf[i] != 0xFFFFFFFF)
            {
                printf("0x%08X\n", g_au32ReadBuf[i]);
            }
        }

        /* Prepare test data */
        for (i = 0; i < (sizeof(g_au32WriteBuf) >> capabilities.data_width); i++)
        {
            g_au32WriteBuf[i] = u32StartAddr + i * 4;
        }

        if (flashDev->ProgramData(u32StartAddr, g_au32WriteBuf, (sizeof(g_au32ReadBuf) >> capabilities.data_width)) < 0)
        {
            printf("ProgramData failed !\n");
            return ;
        }

        /* Wait operation to be completed */
        u32Flags = osThreadFlagsWait(1U, osFlagsWaitAny, WAIT_FLAG_TIMEOUT);

        if (u32Flags & BIT31)       /* Check if BIT31 is set */
        {
            /* An error occurred. Check the specific error code. */
            if (u32Flags == (uint32_t)osErrorTimeout)
            {
                printf("EraseSector timeout !\n");
            }
            else
            {
                /* Unexpected error occurred (e.g., osErrorResource, osErrorParameter, etc.) */
                printf("Unexpected error !\n");
            }
        }

        /* Read back to verify programmed data consistency */
        /* Read data taking data_width into account */
        if (flashDev->ReadData(u32StartAddr, g_au32ReadBuf, (sizeof(g_au32ReadBuf) >> capabilities.data_width)) < 0)
        {
            printf("ReadData failed !\n");
            return ;
        }

        /* Wait operation to be completed */
        if (capabilities.event_ready)
        {
            /* Wait operation to be completed */
            u32Flags = osThreadFlagsWait(1U, osFlagsWaitAny, WAIT_FLAG_TIMEOUT);

            if (u32Flags & BIT31)       /* Check if BIT31 is set */
            {
                /* An error occurred. Check the specific error code. */
                if (u32Flags == (uint32_t)osErrorTimeout)
                {
                    printf("EraseSector timeout !\n");
                }
                else
                {
                    /* Unexpected error occurred (e.g., osErrorResource, osErrorParameter, etc.) */
                    printf("Unexpected error !\n");
                }
            }
        }

        /* Verify programmed data */
        for (i = 0; i < (sizeof(g_au32ReadBuf) >> capabilities.data_width); i++)
        {
            if (g_au32ReadBuf[i] != g_au32WriteBuf[i])
            {
                printf("[Error] 0x%08X Read 0x%08X != Write 0x%08X\n", i, g_au32ReadBuf[i], g_au32WriteBuf[i]);
                return ;
            }
        }

        printf("OK");
    }

    /* Set invalid address (beyond flash memory range) for error handling tests */
    u32StartAddr = (psFlashInfo->sector_count * psFlashInfo->sector_size);
    /* Test read data with invalid address */
    printf("\nTest read invalid address ... ");
    if (flashDev->ReadData(u32StartAddr, g_au32ReadBuf, (sizeof(g_au32ReadBuf) >> capabilities.data_width)) != ARM_DRIVER_ERROR_PARAMETER)
    {
        printf("ReadData with invalid address not return ARM_DRIVER_ERROR_PARAMETER !\n");
        return ;
    }
    else
    {
        printf("OK");
    }

    /* Test program data with invalid address */
    printf("\nTest program invalid address ... ");
    if (flashDev->ProgramData(u32StartAddr, g_au32ReadBuf, (sizeof(g_au32ReadBuf) >> capabilities.data_width)) != ARM_DRIVER_ERROR_PARAMETER)
    {
        printf("ProgramData with invalid address not return ARM_DRIVER_ERROR_PARAMETER !\n");
        return ;
    }
    else
    {
        printf("OK");
    }

    /* Test erase sector with invalid address */
    printf("\nTest erase invalid address ... ");
    if (flashDev->EraseSector(u32StartAddr) != ARM_DRIVER_ERROR_PARAMETER)
    {
        printf("EraseSector with invalid address not return ARM_DRIVER_ERROR_PARAMETER !\n");
        return ;
    }
    else
    {
        printf("OK");
    }

    /* Switch off gracefully */
    flashDev->PowerControl(ARM_POWER_OFF);
    flashDev->Uninitialize();

    printf("\nTest done\n");
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*--------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*--------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable PDMA module clock if PDMA option is enabled */
    CLK_EnableModuleClock(PDMA0_MODULE);
    CLK_EnableModuleClock(PDMA1_MODULE);

    /*--------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*--------------------------------------------------------------------------------*/
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
    printf("\n\nCMSIS Driver Flash Test\n");

    osKernelInitialize();                       /* Initialize CMSIS-RTOS2 */

    /* Create validation thread and save its ID for thread flags signaling */
    Flash_Thread_Id = osThreadNew(Flash_Thread, NULL, NULL);

    osKernelStart();                            /* Start thread execution */

    for (;;) {}
}
