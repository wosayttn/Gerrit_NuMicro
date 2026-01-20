/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Bank Remap sample code(Bank/Segment0 Loader).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"


#define FMC_SEGMENT_SIZE (FMC_APROM_SIZE>>1)


void SYS_Init(void);
int main();


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_180MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}


#if defined ( __ICCARM__ )
#pragma optimize=none
#endif
int main()
{
    uint8_t u8GetCh;
    uint32_t u32ExecBank, i;
    uint32_t u32Loader0ChkSum, u32Loader1ChkSum;
    uint32_t u32App0ChkSum, u32App1ChkSum;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Set Vector Table Offset Register */
    SCB->VTOR = LOADER_BASE;

    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n Bank/Segment%d Loader processing \n\n", u32ExecBank);

        u32Loader0ChkSum = FMC_GetChkSum(FMC_APROM_BASE, LOADER_SIZE);
        u32Loader1ChkSum = FMC_GetChkSum(FMC_SEGMENT_SIZE, LOADER_SIZE);
        printf(" Loader0 checksum: 0x%08x \n Loader1 checksum: 0x%08x\n", u32Loader0ChkSum, u32Loader1ChkSum);

        u32App0ChkSum = FMC_GetChkSum(FMC_APROM_BASE + APP_BASE, APP_SIZE);
        u32App1ChkSum = FMC_GetChkSum(FMC_SEGMENT_SIZE + APP_BASE, APP_SIZE);
        printf(" App0 checksum: 0x%08x \n App1 checksum: 0x%08x\n", u32App0ChkSum, u32App1ChkSum);

        if((u32ExecBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
        {
            printf("\n Create Bank/Segment%d Loader... \n",  u32ExecBank ^ 1);

            /* Erase loader region */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_SEGMENT_SIZE * (u32ExecBank ^ 1) + i);
            }
            /* Create loader in the other bank */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += 8)
            {
                FMC_Write8Bytes(FMC_SEGMENT_SIZE * (u32ExecBank ^ 1) + i,
                                FMC_Read((FMC_SEGMENT_SIZE * u32ExecBank) + i),
                                FMC_Read((FMC_SEGMENT_SIZE * u32ExecBank) + i + 4));
            }
            printf(" Create Bank/Segment%d Loader completed! \n", (u32ExecBank ^ 1));
        }

        if((u32ExecBank == 0) && ((FMC_CheckAllOne((FMC_SEGMENT_SIZE + APP_BASE), APP_SIZE)) == READ_ALLONE_YES))
        {
            printf("\n Create  Bank/Segment%d App... \n", u32ExecBank ^ 1);

            /* Erase app region */
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_SEGMENT_SIZE * (u32ExecBank ^ 1) + i);
            }
            /* Create app in the other bank(just for test) */
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += 8)
            {
                FMC_Write8Bytes(FMC_SEGMENT_SIZE * (u32ExecBank ^ 1) + i,
                                FMC_Read((FMC_SEGMENT_SIZE * u32ExecBank) + i),
                                FMC_Read((FMC_SEGMENT_SIZE * u32ExecBank) + i + 4));
            }
            printf(" Create Bank/Segment%d App completed! \n", (u32ExecBank ^ 1));
        }

        printf("\n Execute Bank/Segment%d APP? [y/n] \n", u32ExecBank);
        u8GetCh = (uint8_t)getchar();

        if(u8GetCh == 'y')
        {
            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
            {
              if(--u32TimeOutCnt == 0)
              {
                  printf("Wait for FMC not busy time-out!\n");
                  break;
              }
            }

            if(u32TimeOutCnt != 0)
            {
                SYS_ResetCPU();
            }
        }
        else
        {
            printf("\n Remap to Bank/Segment%d Loader? [y/n] \n", u32ExecBank ^ 1);
            u8GetCh = (uint8_t)getchar();

            if(u8GetCh == 'y')
            {
                /* Remap Bank */
                printf("\n Bank/Segment%d Loader before remap \n", u32ExecBank);
                FMC_RemapBank(u32ExecBank ^ 1);
                printf("\n Remap completed!\n");
            }
            else
            {
                printf("\n Continue to execute  Bank/Segment%d Loader? \n ", u32ExecBank);
            }
        }

    }
    while(1);

}
