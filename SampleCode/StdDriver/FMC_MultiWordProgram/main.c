/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a code and execute in SRAM to program embedded Flash.
 *           (Support KEIL MDK Only)
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t    page_buff[FMC_FLASH_PAGE_SIZE/4];

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 96MHz from PLL */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{
    uint32_t  i, addr, maddr;          /* temporary variables */

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\n");
    printf("+---------------------------------------------+\n");
    printf("|       Multi-word Program Sample             |\n");
    printf("+---------------------------------------------+\n");
    printf("Check FMC function execution address\n");
    printf("FMC_Erase: 0x%X, FMC_WriteMultiple: 0x%X, FMC_Read: 0x%X\n\n",
           (uint32_t)FMC_Erase, (uint32_t)FMC_WriteMultiple, (uint32_t)FMC_Read);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC_Open();

    FMC_ENABLE_AP_UPDATE();            /* Enable APROM erase/program */

    for (addr = 0x8000; addr < 0x10000; addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("Multiword program APROM page 0x%x =>\n", addr);

        if (FMC_Erase(addr) < 0)
        {
            printf("    Erase failed!!\n");
            goto err_out;
        }

        printf("    Program...\n");

        for (maddr = addr; maddr < addr + FMC_FLASH_PAGE_SIZE; maddr += FMC_MULTI_WORD_PROG_LEN)
        {
            /* Prepare test pattern */
            for (i = 0; i < FMC_MULTI_WORD_PROG_LEN; i+=4)
                page_buff[i/4] = maddr + i;

            i = FMC_WriteMultiple(maddr, page_buff, FMC_MULTI_WORD_PROG_LEN);
            if (i <= 0)
            {
                printf("FMC_WriteMultiple failed: %d\n", i);
                goto err_out;
            }
            printf("programmed length = %d\n", i);
        }
        printf("    [OK]\n");

        printf("    Verify...");

        for (i = 0; i < FMC_FLASH_PAGE_SIZE; i+=4)
            page_buff[i/4] = addr + i;

        for (i = 0; i < FMC_FLASH_PAGE_SIZE; i+=4)
        {
            if (FMC_Read(addr+i) != page_buff[i/4])
            {
                printf("\n[FAILED] Data mismatch at address 0x%x, expect: 0x%x, read: 0x%x!\n", addr+i, page_buff[i/4], FMC_Read(addr+i));
                goto err_out;
            }
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_Read address 0x%x failed!\n", addr+i);
                goto err_out;
            }
        }

        printf("[OK]\n");
    }

    printf("\n\nMulti-word program demo done.\n");
    while (1);

err_out:
    printf("\n\nERROR!\n");
    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
