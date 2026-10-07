/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 24/02/05 4:40p $
 * @brief    Show how to multiple word (word line) program embedded flash by ISP function
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             0x20000
#define LDROM_TEST_BASE             RMC_LDROM_BASE

uint32_t    page_buff[RMC_FLASH_PAGE_SIZE / 4];

void FlashTest(uint32_t u32Addr)
{
    uint32_t i, maddr;
    int32_t result;

    printf("   - Erase ...");

    result = RMC_Erase(u32Addr);

    if(result < 0)
    {
        printf("     [FAILED] Erase failed %d!!\n", result);
        return;
    }
    printf("  [OK]\n");

    printf("   - Verify...");

    for(i = 0; i < RMC_FLASH_PAGE_SIZE; i += 4)
    {
        if(RMC_Read(u32Addr + i) != 0xFFFFFFFF)
        {
            printf("\n     [FAILED] Data mismatch at address 0x%x, expect: 0xFFFFFFFF, read: 0x%x!\n", u32Addr + i, RMC_Read(u32Addr + i));
            return;
        }
        if (g_RMC_i32ErrCode != 0)
        {
            printf("\n     [FAILED] RMC_Read address 0x%x failed!\n", u32Addr+i);
            return;
        }
    }
    printf("  [OK]\n");

    printf("   - Program...\n");

    for(maddr = u32Addr; maddr < u32Addr + RMC_FLASH_PAGE_SIZE; maddr += RMC_MULTI_WORD_PROG_MAX_LEN)
    {
        /* Prepare test pattern */
        for(i = 0; i < RMC_MULTI_WORD_PROG_MAX_LEN; i += 4)
            page_buff[i / 4] = maddr + i;

        result = RMC_WriteMultiple(maddr, page_buff, RMC_MULTI_WORD_PROG_MAX_LEN);

        if(result <= 0)
        {
            printf("\n     [FAILED] RMC_WriteMultiple failed: %d\n", result);
            return;
        }
        printf("       0x%x Programmed length %d...  [OK]\n", maddr, result);

    }

    printf("   - Verify...");

    for(i = 0; i < RMC_FLASH_PAGE_SIZE; i += 4)
        page_buff[i / 4] = u32Addr + i;

    for(i = 0; i < RMC_FLASH_PAGE_SIZE; i += 4)
    {
        if(RMC_Read(u32Addr + i) != page_buff[i / 4])
        {
            printf("\n     [FAILED] Data mismatch at address 0x%x, expect: 0x%x, read: 0x%x!\n", u32Addr + i, page_buff[i / 4], RMC_Read(u32Addr + i));
            return;
        }
        if (g_RMC_i32ErrCode != 0)
        {
            printf("\n     [FAILED] RMC_Read address 0x%x failed!\n", u32Addr + i);
            return;
        }
    }
    printf("[OK]\n\n");	
}

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

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+------------------------------------------------------+\n");
    printf("|     RMC_MultiWord (Word Line) Program Sample Code    |\n");
    printf("+------------------------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable RMC ISP function. Before using RMC function, it should unlock system register first. */
    RMC_Open();

    printf(" * Multiword (Word Line) program APROM 0x%x =>\n", APROM_TEST_BASE);

    RMC_ENABLE_AP_UPDATE();

    FlashTest(APROM_TEST_BASE);

    RMC_DISABLE_AP_UPDATE();

    printf(" * Multiword (Word Line) program LDROM 0x%x =>\n", (uint32_t) LDROM_TEST_BASE);

    RMC_ENABLE_LD_UPDATE();

    FlashTest(LDROM_TEST_BASE);

    RMC_DISABLE_LD_UPDATE();

    /* Disable RMC ISP function */
    RMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("RMC Sample Code Completed.\n");

    while(1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/


