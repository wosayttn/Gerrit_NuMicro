/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Bank Remap sample code(Bank0 Loader).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"

void ResetCPU(void);
void SYS_Init(void);

void ResetCPU(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_CPURST_Msk;
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
}


int main()
{
    uint8_t u8GetCh;
    uint32_t u32ExecBank, i;
    uint32_t u32Loader0ChkSum, u32Loader1ChkSum;
    uint32_t u32App0ChkSum, u32App1ChkSum;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Enable ISP and APROM update */
    RMC_ENABLE_ISP();
    RMC_ENABLE_AP_UPDATE();

    /* Unlock protected registers */
    SYS_UnlockReg();


    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", RMC_GetVECMAP());
        printf("+------------------------+\n");

        u32ExecBank = (uint32_t)((RMC->ISPSTS & RMC_ISPSTS_FBS_Msk) >> RMC_ISPSTS_FBS_Pos);
        printf("\n BANK%d Loader processing \n\n", u32ExecBank);

        u32Loader0ChkSum = RMC_GetChkSum(RMC_APROM_BASE, LOADER_SIZE);
        u32Loader1ChkSum = RMC_GetChkSum(RMC_APROM_BANK0_END, LOADER_SIZE);
        printf(" Loader0 checksum: 0x%08x \n Loader1 checksum: 0x%08x\n", u32Loader0ChkSum, u32Loader1ChkSum);

        u32App0ChkSum = RMC_GetChkSum(RMC_APROM_BASE + APP_BASE, APP_SIZE);
        u32App1ChkSum = RMC_GetChkSum(RMC_APROM_BANK0_END + APP_BASE, APP_SIZE);
        printf(" App0 checksum: 0x%08x \n App1 checksum: 0x%08x\n", u32App0ChkSum, u32App1ChkSum);

        if((u32ExecBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
        {
            printf("\n Create BANK%d Loader... \n",  u32ExecBank ^ 1);

            /* Create loader in the other bank */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += 4)
            {
                RMC_Write(RMC_BANK_SIZE * (u32ExecBank ^ 1) + i, RMC_Read((RMC_BANK_SIZE * u32ExecBank) + i));
            }
            printf(" Create Bank%d Loader completed! \n", (u32ExecBank ^ 1));
        }

        if((u32ExecBank == 0) && ((RMC_CheckAllOne((RMC_APROM_BANK0_END + APP_BASE), APP_SIZE)) == READ_ALLONE_YES))
        {
            printf("\n Create BANK%d App... \n", u32ExecBank ^ 1);

            /* Create app in the other bank(just for test)*/
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += 4)
            {
                RMC_Write(RMC_BANK_SIZE * (u32ExecBank ^ 1) + i, RMC_Read((RMC_BANK_SIZE * u32ExecBank) + i));
            }
            printf(" Create Bank%d App completed! \n", (u32ExecBank ^ 1));
        }

        printf("\n Execute BANK%d APP? [y/n] \n", u32ExecBank);
        u8GetCh = (uint8_t)getchar();

        if(u8GetCh == 'y')
        {
            /* Remap to App */
            RMC_SetVectorPageAddr(APP_BASE);
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
                RMC_RemapBank(u32ExecBank ^ 1);
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
