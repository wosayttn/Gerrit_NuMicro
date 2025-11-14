#include <stdio.h>
#include "Driver_Flash.h"
#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS2:Keil RTX5
#include "NuMicro.h"

#define debug_printf(...)   printf
//#define debug_printf(...)

/* Flash driver instance */
extern ARM_DRIVER_FLASH Driver_Flash0;
static ARM_DRIVER_FLASH *flashDev = &Driver_Flash0;

/* CMSIS-RTOS2 Thread Id */
osThreadId_t Flash_Thread_Id;

#define BUF_BYTE_SIZE   FMC_FLASH_PAGE_SIZE

uint32_t g_au32ReadBuf[BUF_BYTE_SIZE / 4],
         g_au32WriteBuf[BUF_BYTE_SIZE / 4];

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
        printf("Error !\n");
    }
}

/* CMSIS-RTOS2 Thread */
void Flash_Thread(void *argument)
{
    uint32_t i, u32StartAddr;

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
    flashDev->PowerControl(ARM_POWER_FULL);

    for (u32StartAddr = 0; u32StartAddr < (psFlashInfo->sector_count * psFlashInfo->sector_size); u32StartAddr += psFlashInfo->sector_size)
    {
        printf("\rTest 0x%08X ... ", u32StartAddr);

        if (flashDev->EraseSector(u32StartAddr) != 0)
        {
            printf("EraseSector failed !\n");
            return ;
        }

        if (flashDev->ReadData(u32StartAddr, g_au32ReadBuf, (sizeof(g_au32ReadBuf) >> capabilities.data_width)) < 0)
        {
            printf("ReadData failed !\n");
            return ;
        }

        for (i = 0; i < (sizeof(g_au32ReadBuf) >> capabilities.data_width); i++)
        {
            if (g_au32ReadBuf[i] != 0xFFFFFFFF)
                printf("0x%08X\n", g_au32ReadBuf[i]);
        }

        for (i = 0; i < (sizeof(g_au32WriteBuf) >> capabilities.data_width); i++)
        {
            g_au32WriteBuf[i] = u32StartAddr + i * 4;
        }

        if (flashDev->ProgramData(u32StartAddr, g_au32WriteBuf, (sizeof(g_au32WriteBuf) / psFlashInfo->program_unit)) < 0)
        {
            printf("ProgramData failed !\n");
            return ;
        }

        /* Read data taking data_width into account */
        if (flashDev->ReadData(u32StartAddr, g_au32ReadBuf, (sizeof(g_au32ReadBuf) >> capabilities.data_width)) < 0)
        {
            printf("ReadData failed !\n");
            return ;
        }

        for (i = 0; i < (sizeof(g_au32ReadBuf) >> capabilities.data_width); i++)
        {
            if (g_au32ReadBuf[i] != g_au32WriteBuf[i])
            {
                printf("[Error] 0x%08X Read 0x%08X != Write 0x%08X\n", i, g_au32ReadBuf[i], g_au32WriteBuf[i]);
                return ;
            }
        }

        printf("OK");

        /* Wait operation to be completed */
        if (capabilities.event_ready)
        {
            osThreadFlagsWait(1U, osFlagsWaitAny, 100U);
        }
        else
        {
            osDelay(100U);
        }
    }

    /* Test read data with invalid address */
    if (flashDev->ReadData(0x300000, g_au32ReadBuf, (sizeof(g_au32ReadBuf) >> capabilities.data_width)) != ARM_DRIVER_ERROR_PARAMETER)
    {
        printf("ReadData with invalid address not return ARM_DRIVER_ERROR_PARAMETER !\n");
        return ;
    }

    /* Test program data with invalid address */
    if (flashDev->ProgramData(0x300000, g_au32ReadBuf, (sizeof(g_au32ReadBuf) >> capabilities.data_width)) != ARM_DRIVER_ERROR_PARAMETER)
    {
        printf("ProgramData with invalid address not return ARM_DRIVER_ERROR_PARAMETER !\n");
        return ;
    }

    /* Test erase sector with invalid address */
    if (flashDev->EraseSector(0x300000) != ARM_DRIVER_ERROR_PARAMETER)
    {
        printf("EraseSector with invalid address not return ARM_DRIVER_ERROR_PARAMETER !\n");
        return ;
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set core clock to 144MHz */
    //CLK_SetCoreClock(FREQ_144MHZ);

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
    printf("\nCMSIS Driver Flash Test\n");

    osKernelInitialize();                       // Initialize CMSIS-RTOS2

    osThreadNew(Flash_Thread, NULL, NULL);      // Create validation main thread

    osKernelStart();                            // Start thread execution

    for (;;) {}
}
