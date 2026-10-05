/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Bank Remap sample code(Bank0 Loader).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"

void ResetCPU(void);
void SYS_Init(void);

void ResetCPU(void)
{
    SYS->RSTCTL |= SYS_RSTCTL_CPURST_Msk;
    SYS->RSTCTL &= ~SYS_RSTCTL_CPURST_Msk;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_HCLKDIV_HCLK(1));

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV_DIV2 | CLK_PCLKDIV_PCLK1DIV_DIV2);

    /* Set core clock to 40MHz */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Enable all GPIO clock */
    CLK->GPIOCTL |= CLK_GPIOCTL_GPIOACKEN_Msk | CLK_GPIOCTL_GPIOBCKEN_Msk | CLK_GPIOCTL_GPIOCCKEN_Msk | CLK_GPIOCTL_GPIODCKEN_Msk |
                    CLK_GPIOCTL_GPIOECKEN_Msk | CLK_GPIOCTL_GPIOFCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL_UART0SEL_HIRC, CLK_UARTDIV_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();
}

int main()
{
    uint8_t u8GetCh;
    uint32_t u32ExecBank, i;
    uint32_t u32Loader0ChkSum, u32Loader1ChkSum;
    uint32_t u32App0ChkSum, u32App1ChkSum;

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC_Open();

    /* Set Vector Table Offset Register */
    SCB->VTOR = LOADER_BASE;

    /* Enable APROM update */
    FMC_ENABLE_AP_UPDATE();

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08x  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n BANK%d Loader processing \n\n", u32ExecBank);

        u32Loader0ChkSum = FMC_GetChkSum(FMC_APROM_BASE, LOADER_SIZE);
        u32Loader1ChkSum = FMC_GetChkSum(FMC_APROM_BANK0_END, LOADER_SIZE);
        printf(" Loader0 checksum: 0x%08x \n Loader1 checksum: 0x%08x\n", u32Loader0ChkSum, u32Loader1ChkSum);

        u32App0ChkSum = FMC_GetChkSum(FMC_APROM_BASE + APP_BASE, APP_SIZE);
        u32App1ChkSum = FMC_GetChkSum(FMC_APROM_BANK0_END + APP_BASE, APP_SIZE);
        printf(" App0 checksum: 0x%08x \n App1 checksum: 0x%08x\n", u32App0ChkSum, u32App1ChkSum);

        if((u32ExecBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
        {
            printf("\n Create BANK%d Loader... \n",  u32ExecBank ^ 1);

            /* Erase loader region */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_BANK_SIZE * (u32ExecBank ^ 1) + i);
            }
            /* Create loader in the other bank */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += 8)
            {
                FMC_Write8Bytes(FMC_BANK_SIZE * (u32ExecBank ^ 1) + i, FMC_Read((FMC_BANK_SIZE * u32ExecBank) + i), FMC_Read((FMC_BANK_SIZE * u32ExecBank) + i + 4));
            }
            printf(" Create Bank%d Loader completed! \n", (u32ExecBank ^ 1));
        }

        if((u32ExecBank == 0) && ((FMC_CheckAllOne((FMC_APROM_BANK0_END + APP_BASE), APP_SIZE)) == READ_ALLONE_YES))
        {
            printf("\n Create BANK%d App... \n", u32ExecBank ^ 1);

            /* Erase app region */
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_BANK_SIZE * (u32ExecBank ^ 1) + i);
            }
            /* Create app in the other bank(just for test)*/
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += 8)
            {
                FMC_Write8Bytes(FMC_BANK_SIZE * (u32ExecBank ^ 1) + i, FMC_Read((FMC_BANK_SIZE * u32ExecBank) + i), FMC_Read((FMC_BANK_SIZE * u32ExecBank) + i + 4));
            }
            printf(" Create Bank%d App completed! \n", (u32ExecBank ^ 1));
        }

        printf("\n Execute BANK%d APP? [y/n] \n", u32ExecBank);
        u8GetCh = (uint8_t)getchar();

        if(u8GetCh == 'y')
        {
            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            ResetCPU();
        }
        else
        {
            printf("\n Remap to BANK%d Loader? [y/n] \n", u32ExecBank ^ 1);
            u8GetCh = (uint8_t)getchar();

            if(u8GetCh == 'y')
            {
                /* Remap Bank */
                printf("\n BANK%d Loader before remap \n", u32ExecBank);
                FMC_RemapBank(u32ExecBank ^ 1);
                printf("\n Remap completed!\n");
            }
            else
            {
                printf("\n Continue to execute BANK%d Loader? \n ", u32ExecBank);
            }
        }

    }
    while(1);

}
/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/