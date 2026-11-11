/*************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    SFUD flash access sample.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"
#include "sfud.h"

#define SFUD_DEMO_TEST_BUFFER_SIZE      (1024UL)
#define SFUD_DEMO_ERASE_SIZE            (4096UL)
#define SFUD_DEMO_TEST_ADDR             (0x01000000UL)

static void SFUD_Demo(const sfud_flash *psFlash, uint32_t u32Addr, uint32_t u32Size, uint8_t *pu8Data);

#if (NVT_DCACHE_ON == 1)
    static uint8_t s_au8SfudDemoBuf[SFUD_DEMO_TEST_BUFFER_SIZE] __attribute__((aligned(32)));
#else
    static uint8_t s_au8SfudDemoBuf[SFUD_DEMO_TEST_BUFFER_SIZE] __attribute__((aligned(8)));
#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable GPIO Module clock */
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

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
}

/**
 * @brief      Run a destructive SFUD erase/program/read/verify test.
 * @param[in]  psFlash   Pointer to the initialized SFUD flash device.
 * @param[in]  u32Addr   Flash start address reserved for destructive testing.
 * @param[in]  u32Size   Number of bytes to program/read/verify.
 * @param[in]  pu8Data   Test data buffer.
 * @return     None
 *
 * @note       The erase operation uses SFUD_DEMO_ERASE_SIZE while the
 *             program/read verification uses u32Size bytes.
 */
static void SFUD_Demo(const sfud_flash *psFlash, uint32_t u32Addr, uint32_t u32Size, uint8_t *pu8Data)
{
    sfud_err result = SFUD_SUCCESS;
    uint32_t u32i;

    if ((psFlash == NULL) || (pu8Data == NULL) || (u32Size == 0UL))
    {
        printf("Invalid SFUD demo parameter.\r\n");
        return;
    }

    /* Prepare test pattern: 00, 01, ... FF, 00, 01, ... */
    for (u32i = 0UL; u32i < u32Size; u32i++)
    {
        pu8Data[u32i] = (uint8_t)(u32i & 0xFFUL);
    }

    /* Erase one 4-Kbyte sector reserved for this destructive test. */
    result = sfud_erase(psFlash, u32Addr, SFUD_DEMO_ERASE_SIZE);

    if (result != SFUD_SUCCESS)
    {
        printf("Erase the %s flash data failed. Start from 0x%08X.\r\n",
               psFlash->name, (unsigned int)u32Addr);
        return;
    }

    printf("Erase the %s flash data finish. Start from 0x%08X, size is %u.\r\n",
           psFlash->name,
           (unsigned int)u32Addr,
           (unsigned int)SFUD_DEMO_ERASE_SIZE);

    /* Program test data. */
    result = sfud_write(psFlash, u32Addr, u32Size, pu8Data);

    if (result != SFUD_SUCCESS)
    {
        printf("Write the %s flash data failed. Start from 0x%08X.\r\n",
               psFlash->name, (unsigned int)u32Addr);
        return;
    }

    printf("Write the %s flash data finish. Start from 0x%08X, size is %u.\r\n",
           psFlash->name,
           (unsigned int)u32Addr,
           (unsigned int)u32Size);

    /* Clear the buffer before reading data back. */
    (void)memset(pu8Data, 0, u32Size);

    result = sfud_read(psFlash, u32Addr, u32Size, pu8Data);

    if (result != SFUD_SUCCESS)
    {
        printf("Read the %s flash data failed. Start from 0x%08X.\r\n",
               psFlash->name, (unsigned int)u32Addr);
        return;
    }

    printf("Read the %s flash data success. Start from 0x%08X, size is %u. The data is:\r\n",
           psFlash->name,
           (unsigned int)u32Addr,
           (unsigned int)u32Size);
    printf("Offset (h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");

    for (u32i = 0UL; u32i < u32Size; u32i++)
    {
        if ((u32i % 16UL) == 0UL)
        {
            printf("[%08X] ", (unsigned int)(u32Addr + u32i));
        }

        printf("%02X ", pu8Data[u32i]);

        if ((((u32i + 1UL) % 16UL) == 0UL) || (u32i == (u32Size - 1UL)))
        {
            printf("\r\n");
        }
    }

    printf("\r\n");

    /* Verify read data. */
    for (u32i = 0UL; u32i < u32Size; u32i++)
    {
        uint8_t u8Expected = (uint8_t)(u32i & 0xFFUL);

        if (pu8Data[u32i] != u8Expected)
        {
            printf("Data verify failed at 0x%08X. Read = 0x%02X, expect = 0x%02X.\r\n",
                   (unsigned int)(u32Addr + u32i),
                   pu8Data[u32i],
                   u8Expected);
            return;
        }
    }

    printf("The %s flash test is success.\r\n\r\n", psFlash->name);
}

/*
 * This sample initializes the system clock and debug UART, then demonstrates
 * how to use SFUD to erase, program, read, and verify data on an external
 * serial flash device.
 *
 * The example prints progress and verification results through the debug UART.
 * Adjust the related configuration as needed for your application.
 */
int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("\n\n");
    printf("+-------------------------------------------------------------------------+\n");
    printf("|                       SFUD with Flash Sample Code                       |\n");
    printf("+-------------------------------------------------------------------------+\n");

    /* SFUD initialize */
    sfud_init();

    sfud_flash *psFlash = sfud_get_device(SFUD_WINBOND_DEV_IDX3);

    if ((psFlash != NULL) && (psFlash->init_ok == true))
    {
#ifdef SFUD_USING_QSPI
        /* Enable QSPI fast read mode with four data lines. */
        sfud_qspi_fast_read_enable(psFlash, 4);
#endif

        printf("\r\nDestructive SFUD test region: 0x%08X - 0x%08X\r\n",
               (unsigned int)SFUD_DEMO_TEST_ADDR,
               (unsigned int)(SFUD_DEMO_TEST_ADDR + SFUD_DEMO_ERASE_SIZE - 1UL));

        SFUD_Demo(psFlash,
                  SFUD_DEMO_TEST_ADDR,
                  sizeof(s_au8SfudDemoBuf),
                  s_au8SfudDemoBuf);
    }
    else
    {
        printf("SFUD flash device initialization failed.\r\n");
    }

    /* Got no where to go, just loop forever */
    while (1);
}
