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
#include "../common/inc/NuDB_common.h"
#include "xmodem.h"


#define CREATE_BANK1_APP   1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t  s_u32ExecBank;             /* CPU executing in which Bank/Segment                */
static volatile uint32_t  s_u32DbLength;             /* Dual bank/segment program remaining length         */
static volatile uint32_t  s_u32DbAddr;               /* Dual bank/segment program current flash address    */

/*---------------------------------------------------------------------------------------------------------*/
/* Global Functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void WDT0_IRQHandler(void);
void SYS_Init(void);
void Download(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupt Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void WDT0_IRQHandler(void)
{
    WDT_RESET_COUNTER(WDT0);

    if(WDT_GET_TIMEOUT_INT_FLAG(WDT0) == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG(WDT0);
    }


}

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

    /* Enable WDT0 module clock */
    CLK_EnableModuleClock(WDT0_MODULE);

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


void Download(void)
{
    int32_t  i32Err;
    uint32_t u32ExecBank = s_u32ExecBank;

    printf("\n Bank/Segment%d processing, download data to Bank/Segment%d APP Base.\n\n\n", u32ExecBank, u32ExecBank);

    /* Dual bank/segment background program address */
    s_u32DbAddr   = APP_BASE;

    /* Dual bank/segment background program length */
    s_u32DbLength = APP_SIZE;

    /* Use Xmodem to download firmware from PC */
    i32Err = Xmodem(s_u32DbAddr);

    if(i32Err < 0)
    {
        printf("\nXmodem transfer fail!\n");
        while(1);
    }
    else
    {
        printf("\nXomdem transfer done!\n");
        printf("Total transfer size is %d\n", i32Err);
    }

    printf("\n Firmware download completed!!\n");
}

#if defined ( __ICCARM__ )
#pragma optimize=none
#endif
int main()
{
    uint8_t u8GetCh;
    uint32_t i;
    uint32_t u32Loader0ChkSum, u32Loader1ChkSum;
    uint32_t u32App0ChkSum, u32App1ChkSum;
    uint32_t u32ExecBank;
    uint32_t u32TimeOutCnt;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Set Vector Table Offset Register */
    SCB->VTOR = LOADER_BASE;

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        /* Check CPU run at Bank/Segment 0 or 1 */
        s_u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n Bank/Segment%d Loader processing \n\n", s_u32ExecBank);

        /* Get loader CRC */
        u32Loader0ChkSum = FMC_GetChkSum(FMC_APROM_BASE, LOADER_SIZE);
        u32Loader1ChkSum = FMC_GetChkSum((FMC_APROM_END/2), LOADER_SIZE);
        printf(" Loader0 checksum: 0x%08x \n Loader1 checksum: 0x%08x\n", u32Loader0ChkSum, u32Loader1ChkSum);

        /* Get app CRC */
        u32App0ChkSum = FMC_GetChkSum(FMC_APROM_BASE + APP_BASE, APP_SIZE);
        u32App1ChkSum = FMC_GetChkSum((FMC_APROM_END/2) + APP_BASE, APP_SIZE);
        printf(" App0 checksum: 0x%08x \n App1 checksum: 0x%08x\n", u32App0ChkSum, u32App1ChkSum);

        /* Write firmware CRC in CRC base for following checking */
        printf("\n Firmware CRC in [0x%x] is [0x%x]\n", FW_CRC_BASE, FMC_Read(FW_CRC_BASE));
        if(FMC_Read(FW_CRC_BASE) == 0xFFFFFFFF)
        {
            FMC_Write8Bytes(FW_CRC_BASE, u32App0ChkSum, 0xFFFFFFFF);
            printf("\n Update Firmware CRC in [0x%x] is [0x%x]\n", FW_CRC_BASE, FMC_Read(FW_CRC_BASE));
        }

        /* Write backup firmware CRC in backup CRC base for following checking */
        printf("\n Backup Firmware CRC in [0x%x] is [0x%x]\n", BACKUP_FW_CRC_BASE, FMC_Read(BACKUP_FW_CRC_BASE));
        if(FMC_Read(BACKUP_FW_CRC_BASE) == 0xFFFFFFFF)
        {

            FMC_Write8Bytes(BACKUP_FW_CRC_BASE, u32App1ChkSum, 0xFFFFFFFF);
            printf("\n Update Firmware CRC in [0x%x] is [0x%x]\n", BACKUP_FW_CRC_BASE, FMC_Read(BACKUP_FW_CRC_BASE));
        }

        /* Create the other bank/segment loader for executing bank remap */
        if((s_u32ExecBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
        {
            printf("\n Create Bank/Segment%d Loader... \n",  s_u32ExecBank ^ 1);

            /* Erase loader region */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase((FMC_APROM_END/2) * (s_u32ExecBank ^ 1) + i);
            }
            /* Create loader in the other bank/segment */
            u32ExecBank = s_u32ExecBank;
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += 8)
            {
                FMC_Write8Bytes((FMC_APROM_END/2) * (u32ExecBank ^ 1) + i,
                                FMC_Read(((FMC_APROM_END/2) * u32ExecBank) + i),
                                FMC_Read(((FMC_APROM_END/2) * u32ExecBank) + i + 4));
            }
            printf(" Create Bank/Segment%d Loader completed! \n", (s_u32ExecBank ^ 1));
        }

#if CREATE_BANK1_APP
        if((s_u32ExecBank == 0) && ((FMC_CheckAllOne(((FMC_APROM_END/2) + APP_BASE), APP_SIZE)) == READ_ALLONE_YES))
        {
            printf("\n Create Bank/Segment%d App... \n", s_u32ExecBank ^ 1);

            /* Erase app region */
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase((FMC_APROM_END/2) * (s_u32ExecBank ^ 1) + i);
            }
            /* Create app in the other bank(just for test) */
            u32ExecBank = s_u32ExecBank;
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += 8)
            {
                FMC_Write8Bytes((FMC_APROM_END/2) * (u32ExecBank ^ 1) + i,
                                FMC_Read(((FMC_APROM_END/2) * u32ExecBank) + i),
                                FMC_Read(((FMC_APROM_END/2) * u32ExecBank) + i + 4));
            }
            printf(" Create Bank/Segment %d App completed! \n", (s_u32ExecBank ^ 1));
        }
#endif
        /* To check if system has been reset by WDT time-out reset or not */
        if(WDT_GET_RESET_FLAG(WDT0) == 1)
        {
            WDT_CLEAR_RESET_FLAG(WDT0);
            printf("\n === System reset by WDT time-out event === \n");
            printf(" Any key to remap back to backup FW\n");
            getchar();

            /* Remap to Bank/Segment 1 to execute backup firmware */
            FMC_RemapBank(1);
            s_u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
            printf("\n Bank/Segment %d Loader after remap  \n\n", s_u32ExecBank);

            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            SYS_ResetCPU();
        }

        printf("\n Execute Bank/Segment %d APP? [y/n] \n", s_u32ExecBank);
        u8GetCh = (uint8_t)getchar();
        printf("\n User select [%c] \n", u8GetCh);

        if(u8GetCh == 'y')
        {
            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while((FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)==FMC_ISPTRG_ISPGO_Msk)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for FMC operation finished time-out!\n");
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

            printf("\n Download new firmware? [y/n] \n");
            u8GetCh = (uint8_t)getchar();
            printf("\n User select [%c] \n", u8GetCh);

            if(u8GetCh == 'y')
            {
                /* Download new firmware */
                Download();
                printf("\n Any key to execute new firmware \n");
                getchar();
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
                /* Remap to Loader */
                FMC_SetVectorPageAddr(LOADER_BASE);
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

        }

    }
    while(1);

}
