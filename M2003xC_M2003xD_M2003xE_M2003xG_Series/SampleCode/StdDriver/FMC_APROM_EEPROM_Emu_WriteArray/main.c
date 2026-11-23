/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    EEPROM emulation sample using APROM via FMC ISP interface.
 *           This sample demonstrates how to initialize the EEPROM emulation
 *           module and perform read, write, erase, and data verification
 *           operations on embedded flash memory.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "eeprom_emu.h"

/* Define a test address (must NOT overwrite code area) */
#define TEST_ADDR   (0x003F80U)

/* Test data */
static uint8_t txBuf[16] =
{
    0x11U, 0x22U, 0x33U, 0x44U,
    0x55U, 0x66U, 0x77U, 0x88U,
    0xAAU, 0xBBU, 0xCCU, 0xDDU,
    0xEEU, 0xFFU, 0x12U, 0x34U
};

static uint8_t rxBuf[16];

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

static int VerifyData(uint8_t *a, uint8_t *b, uint32_t len)
{
    uint32_t i;

    for (i = 0U; i < len; i++)
    {
        if (a[i] != b[i])
        {
            return -1;
        }
    }
    return 0;
}


int32_t main(void)
{
    flash_status_t status;
    uint8_t readByte;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+---------------------------------------------------+\n");
    printf("|    FMC APROM EEPROM Emu WriteArray Sample code    |\n");
    printf("+---------------------------------------------------+\n");

    /* Init EEPROM Emulation */
    status = EEPROM_Emu_Init();
    if (status != FLASH_STATUS_OK)
    {
        printf("Init failed!\n");
        while (1);
    }

    printf("Init OK\n");

    /* Erase test region */
    status = EEPROM_Emu_Erase(TEST_ADDR, 512U);
    if (status != FLASH_STATUS_OK)
    {
        printf("Erase failed!\n");
        while (1);
    }

    printf("Erase OK\n");

    /* Write Array */
    status = EEPROM_Emu_WriteArray(TEST_ADDR + 0x3U, txBuf, sizeof(txBuf));
    if (status != FLASH_STATUS_OK)
    {
        printf("WriteArray failed!\n");
        while (1);
    }

    printf("WriteArray OK\n");

    /* Read Array */
    status = EEPROM_Emu_ReadArray(TEST_ADDR + 0x3U, rxBuf, sizeof(rxBuf));
    if (status != FLASH_STATUS_OK)
    {
        printf("ReadArray failed!\n");
        while (1);
    }

    /* Verify  */
    if (VerifyData(txBuf, rxBuf, sizeof(txBuf)) == 0)
    {
        printf("Verify OK\n");
    }
    else
    {
        printf("Verify FAIL\n");
        while (1);
    }

    /* Write / Read Single Byte */
    status = EEPROM_Emu_WriteByte(TEST_ADDR + 0x1U, 0x5AU);
    if (status != FLASH_STATUS_OK)
    {
        printf("WriteByte failed!\n");
        while (1);
    }

    status = EEPROM_Emu_ReadByte(TEST_ADDR + 0x1U, &readByte);
    if (status != FLASH_STATUS_OK)
    {
        printf("ReadByte failed!\n");
        while (1);
    }

    printf("ReadByte = 0x%02X\n", readByte);

    printf("All test passed\n");

    EEPROM_Emu_DeInit();

    printf("\nFMC Sample Code Completed.\n");

    while (1);

}

/*** (C) COPYRIGHT 2026 Nuvoton Technology Corp. ***/
