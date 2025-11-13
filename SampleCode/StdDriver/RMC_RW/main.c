/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Show how to read/program embedded flash by ISP function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             0x20000
#define APROM_TEST_END              APROM_TEST_BASE+0x4000
#define TEST_PATTERN                0x5A5A5A5A

void SYS_Init(void);
int32_t FillDataPattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern);
int32_t VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern);
int32_t FlashTest(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern);

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

int32_t FillDataPattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        if (RMC_Write(u32Addr, u32Pattern) != 0)          /* Program flash */
        {
            printf("RMC_Write address 0x%x failed!\n", u32Addr);
            return -1;
        }
    }
    return 0;
}


int32_t  VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32Data;

    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32Data = RMC_Read(u32Addr);

        if (g_RMC_i32ErrCode != 0)
        {
            printf("RMC_Read address 0x%X failed!\n", u32Addr);
            return -1;
        }
        if(u32Data != u32Pattern)
        {
            printf("\nRMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data, u32Pattern);
            return -1;
        }
    }
    return 0;
}


int32_t  FlashTest(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += RMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        /* Write test pattern to fill the whole page */
        if(FillDataPattern(u32Addr, u32Addr + RMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all equal to test pattern */
        if(VerifyData(u32Addr, u32Addr + RMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");
            return -1;
        }
    }
    printf("\r    Flash Test Passed.            \n");
    return 0;
}

int32_t main(void)
{
    uint32_t i, u32Data;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|        RMC ISP Sample Code             |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable RMC ISP function. Before using RMC function, it should unlock system register first. */
    RMC_Open();

    u32Data = RMC_ReadCID();
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = RMC_ReadPID();
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    for(i = 0; i < 3; i++)
    {
        u32Data = RMC_ReadUID((uint8_t)i);
        if (g_RMC_i32ErrCode != 0)
        {
            printf("RMC_ReadUID %d failed!\n", i);
            goto lexit;
        }
        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);
    }

    for(i = 0; i < 4; i++)
    {
        u32Data = RMC_ReadUCID(i);
        if (g_RMC_i32ErrCode != 0)
        {
            printf("RMC_ReadUCID %d failed!\n", i);
            goto lexit;
        }
        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);
    }

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", RMC_Read(RMC_USER_CONFIG_0));
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_Read(RMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    printf("  User Config 1 ......................... [0x%08x]\n", RMC_Read(RMC_USER_CONFIG_1));
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_Read(RMC_USER_CONFIG_1) failed!\n");
        goto lexit;
    }

    printf("  User Config 2 ......................... [0x%08x]\n", RMC_Read(RMC_USER_CONFIG_2));
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_Read(RMC_USER_CONFIG_2) failed!\n");
        goto lexit;
    }

    printf("  User Config 3 ......................... [0x%08x]\n", RMC_Read(RMC_USER_CONFIG_3));
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_Read(RMC_USER_CONFIG_3) failed!\n");
        goto lexit;
    }

    printf("\nLDROM test =>\n");
    RMC_ENABLE_LD_UPDATE();
    if(FlashTest(RMC_LDROM_BASE, RMC_LDROM_BASE + RMC_LDROM_SIZE, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");
        goto lexit;
    }
    RMC_DISABLE_LD_UPDATE();

    printf("\nAPROM test =>\n");
    RMC_ENABLE_AP_UPDATE();
    if(FlashTest(APROM_TEST_BASE, APROM_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nAPROM test failed!\n");
        goto lexit;
    }
    RMC_DISABLE_AP_UPDATE();

lexit:

    /* Disable RMC ISP function */
    RMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nRMC Sample Code Completed.\n");

    while(1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/


