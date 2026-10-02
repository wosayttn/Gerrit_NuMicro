/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Implement CRC in CRC-CCITT mode and get the CRC checksum result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t CRC_SWResult(uint32_t mode, uint32_t polynom, uint32_t seed, uint8_t *string, uint32_t count, int8_t IsInput1sCOM, int8_t IsInputRVS, int8_t IsCRC1sCOM, int8_t IsCRCRVS);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CRC module clock */
    CLK_EnableModuleClock(CRC_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------+\n");
    printf("|    CRC-16 Polynomial Mode Sample Code     |\n");
    printf("+-------------------------------------------+\n\n");

    printf("# Calculate [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38] CRC16 checksum value.\n");
    printf("    - Seed value is 0x%x             \n",seed);
    printf("    - Polynomial value is 0x%x       \n",polynom);
    printf("    - CPU write data length is 16-bit \n");
    printf("    - Checksum complement %s    \n", IsWrite1sCOM?"Enable":"Disable");
    printf("    - Checksum reverse %s       \n", IsWriteRVS?"Enable":"Disable");
    printf("    - Write data complement %s  \n", IsCRC1sCOM?"Enable":"Disable");
    printf("    - Write data reverse %s     \n", IsCRCRVS?"Enable":"Disable");

    u32SWChecksum = CRC_SWResult(CRC_16, polynom, seed, (uint8_t *)u16CRCSrcPattern, sizeof(u16CRCSrcPattern), IsWrite1sCOM, IsWriteRVS, IsCRC1sCOM, IsCRCRVS);

    printf("    - Checksum should be 0x%x        \n\n", u32SWChecksum);

    if(IsWrite1sCOM)
        u32Attribute |= CRC_WDATA_COM;
    if(IsWriteRVS)
        u32Attribute |= CRC_WDATA_RVS;
    if(IsCRC1sCOM)
        u32Attribute |= CRC_CHECKSUM_COM;
    if(IsCRCRVS)
        u32Attribute |= CRC_CHECKSUM_RVS;

    /* Configure CRC controller for CRC-CCITT CPU mode */
    CRC_Open(CRC_16, u32Attribute, seed, CRC_CPU_WDATA_16);

    CRC_SET_POLYNOMIAL(polynom);

    /* Start to execute CRC-CCITT operation */
    for(i = 0; i < sizeof(u16CRCSrcPattern) / sizeof(u16CRCSrcPattern[0]); i++)
    {
        CRC_WRITE_DATA((u16CRCSrcPattern[i]));
    }

    /* Get CRC-CCITT checksum value */
    u32HWChecksum = CRC_GetChecksum();

    printf("CRC H/W checksum is 0x%x ... %s.\n", u32HWChecksum, (u32HWChecksum == u32SWChecksum) ? "PASS" : "FAIL");

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC_MODULE);

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
