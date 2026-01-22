/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   CRC_Polynomial example for NuMicro M3351
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
uint32_t CRC_SWResult(uint32_t mode, uint32_t polynom, uint32_t seed, uint8_t *string, uint32_t count, int8_t IsWrite1sCOM, int8_t IsWriteRVS, int8_t IsCRC1sCOM, int8_t IsCRCRVS);

void SYS_Init(void)
{
    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL0 144MHz clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable CRC module clock */
    CLK_EnableModuleClock(CRC_MODULE);

    /* Enable UART0 module clock */
    SetDebugUartCLK();

    /* Set UART0 MFP */
    SetDebugUartMFP();


}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    const uint16_t u16CRCSrcPattern[] = {0x3231, 0x3433, 0x3635, 0x3837};
    uint32_t polynom = 0x8408;
    uint32_t seed = 0xFFFF;
    uint32_t IsWrite1sCOM = 1;
    uint32_t IsWriteRVS = 1;
    uint32_t IsCRC1sCOM = 1;
    uint32_t IsCRCRVS = 1;
    uint32_t i, u32HWChecksum = 0, u32SWChecksum = 0;
    uint32_t u32Attribute = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();


    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------+\n");
    printf("|    CRC-16 Mode Sample Code     |\n");
    printf("+-------------------------------------------+\n\n");

    printf("# Calculate [0x3231, 0x3433, 0x3635, 0x3837] CRC16 checksum value.\n");
    printf("    - Seed value is 0x%x             \n", seed);
    printf("    - Polynomial value is 0x%x       \n", polynom);
    printf("    - CPU write data length is 16-bit \n");
    printf("    - Checksum complement %s    \n", IsWrite1sCOM ? "Enable" : "Disable");
    printf("    - Checksum reverse %s       \n", IsWriteRVS ? "Enable" : "Disable");
    printf("    - Write data complement %s  \n", IsCRC1sCOM ? "Enable" : "Disable");
    printf("    - Write data reverse %s     \n", IsCRCRVS ? "Enable" : "Disable");

    u32SWChecksum = CRC_SWResult(CRC_16, polynom, seed, (uint8_t *)u16CRCSrcPattern, sizeof(u16CRCSrcPattern), IsWrite1sCOM, IsWriteRVS, IsCRC1sCOM, IsCRCRVS);

    printf("    - Checksum should be 0x%x        \n\n", u32SWChecksum);

    if (IsWrite1sCOM)
        u32Attribute |= CRC_WDATA_COM;

    if (IsWriteRVS)
        u32Attribute |= CRC_WDATA_RVS;

    if (IsCRC1sCOM)
        u32Attribute |= CRC_CHECKSUM_COM;

    if (IsCRCRVS)
        u32Attribute |= CRC_CHECKSUM_RVS;

    /* Configure CRC controller for CRC-16 CPU mode */
    CRC_Open(CRC_16, u32Attribute, seed, CRC_CPU_WDATA_16);

    CRC_SET_POLYNOMIAL(polynom);

    /* Start to execute CRC-16 operation */
    for (i = 0; i < sizeof(u16CRCSrcPattern) / sizeof(u16CRCSrcPattern[0]); i++)
    {
        CRC_WRITE_DATA((u16CRCSrcPattern[i]));
    }

    /* Get CRC-16 checksum value */
    u32HWChecksum = CRC_GetChecksum();

    printf("CRC H/W checksum is 0x%x ... %s.\n", u32HWChecksum, (u32HWChecksum == u32SWChecksum) ? "PASS" : "FAIL");

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC_MODULE);

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
