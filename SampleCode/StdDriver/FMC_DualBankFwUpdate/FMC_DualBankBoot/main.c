/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date:
 * @brief
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"

#define FW_SWAP_BACK 0
#define WDT_EN 0

static uint32_t g_au32Bank0PageSum[BANK0_FW_SIZE / TMP_PAGE_SIZE];
static uint32_t g_au32Bank0PageSumInFlash[BANK0_FW_SIZE / TMP_PAGE_SIZE];

static uint32_t g_au32Bank1PageSum[BANK1_FW_SIZE / TMP_PAGE_SIZE];
static uint32_t g_au32Bank1PageSumInFlash[BANK1_FW_SIZE / TMP_PAGE_SIZE];

extern int IsDebugFifoEmpty(void);

void WDT_IRQHandler(void);
void SYS_Init(void);

/**
 * @brief       IRQ Handler for WDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT, declared in startup_M471.s.
 */
void WDT_IRQHandler(void)
{
    WDT_RESET_COUNTER();


    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }
}

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

    /* Enable WDT module clock */
    CLK_EnableModuleClock(WDT_MODULE);

    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

#define DUMP_NUM 4

static void DumpSwapInfo(uint32_t u32Offset)
{
    uint32_t u32i;

    for(u32i = 0; u32i < DUMP_NUM; u32i += 4)
    {
        printf("[0x%8x][0x%8x]\t", BANK0_FW_BASE + u32Offset + u32i, FMC_Read(BANK0_FW_BASE + u32Offset + u32i));
        printf("[0x%8x][0x%8x]\t", BANK1_FW_BASE + u32Offset + u32i, FMC_Read(BANK1_FW_BASE + u32Offset + u32i));
        printf("[0x%8x][0x%8x]\n", TMP_PAGE_BASE + u32i, FMC_Read(TMP_PAGE_BASE + u32i));
    }
}

static void BankSwap(void)
{
    uint32_t        u32i, u32j;

    FMC_Erase(TMP_PAGE_BASE);

    /* Swap Bank0 and Bank1 page by page */
    for(u32i = 0; u32i < BANK0_FW_SIZE; u32i += TMP_PAGE_SIZE)
    {
        printf("+-------------------------+\n");
        printf("|     Page[%d] Swap        |\n", u32i / TMP_PAGE_SIZE);
        printf("+-------------------------+\n\n");

        DumpSwapInfo(u32i);

        /* Swap step1: move data from Bank0 to tmp */
        for(u32j = 0; u32j < TMP_PAGE_SIZE; u32j += 4)
        {
            FMC_Write(TMP_PAGE_BASE + u32j, FMC_Read(BANK0_FW_BASE + u32i + u32j));
        }
        DumpSwapInfo(u32i);
        printf("Erase [0x%8x]\n", BANK0_FW_BASE + u32i);
        FMC_Erase(BANK0_FW_BASE + u32i);
        DumpSwapInfo(u32i);

        /* Swap step2: move from Bank1 to Bank0 */
        for(u32j = 0; u32j < TMP_PAGE_SIZE; u32j += 4)
        {
            FMC_Write(BANK0_FW_BASE + u32i + u32j, FMC_Read(BANK1_FW_BASE + u32i + u32j));
        }
        DumpSwapInfo(u32i);
        printf("Erase [0x%8x]\n", BANK1_FW_BASE + u32i);
        FMC_Erase(BANK1_FW_BASE + u32i);
        DumpSwapInfo(u32i);

        /* Swap step3: move from tmp to Bank1 */
        for(u32j = 0; u32j < TMP_PAGE_SIZE; u32j += 4)
        {
            FMC_Write(BANK1_FW_BASE + u32i + u32j, FMC_Read(TMP_PAGE_BASE + u32j));
        }
        DumpSwapInfo(u32i);
        printf("Erase [0x%8x]\n", TMP_PAGE_BASE);
        FMC_Erase(TMP_PAGE_BASE);
        DumpSwapInfo(u32i);

        printf("\n");
    }
}

static void BankSwapContinue(void)
{
    uint32_t    u32i, u32j, u32StartPage = 0;
    uint32_t    u32TmpSum = 0;

    /* Calculate current bank0, bank1 CRC */
    for(u32i = 0; u32i < BANK0_FW_SIZE / TMP_PAGE_SIZE; u32i++)
    {
        g_au32Bank0PageSum[u32i] = func_crc32(BANK0_FW_BASE + u32i * TMP_PAGE_SIZE, TMP_PAGE_SIZE);
        g_au32Bank1PageSum[u32i] = func_crc32(BANK1_FW_BASE + u32i * TMP_PAGE_SIZE, TMP_PAGE_SIZE);
    }

    /* calculate tmp CRC */
    u32TmpSum = func_crc32(TMP_PAGE_BASE, TMP_PAGE_SIZE);

    for(u32i = 0; u32i < BANK0_FW_SIZE / TMP_PAGE_SIZE; u32i++)
    {
        printf(" Bank0 page[%d][0x%8x][0x%8x]\n", u32i, g_au32Bank0PageSum[u32i], g_au32Bank0PageSumInFlash[u32i]);
        printf(" Bank1 page[%d][0x%8x][0x%8x]\n", u32i, g_au32Bank1PageSum[u32i], g_au32Bank1PageSumInFlash[u32i]);
        printf(" g_au32Bank1PageSumTmp[0x%8x]\n",  u32TmpSum);

        if((g_au32Bank0PageSum[u32i] == g_au32Bank0PageSumInFlash[u32i])
                && (g_au32Bank1PageSum[u32i] == g_au32Bank1PageSumInFlash[u32i]))
        {
            printf(" Re-start from step 1\n");
            u32StartPage = u32i;
            break;
        }
        else if((u32TmpSum == g_au32Bank0PageSumInFlash[u32i])
                && (g_au32Bank1PageSum[u32i] == g_au32Bank1PageSumInFlash[u32i])
                && (g_au32Bank0PageSum[u32i] != g_au32Bank0PageSumInFlash[u32i]))
        {
            printf(" Re-start from step 2\n");
            printf("Erase [0x%8x]\n", BANK0_FW_BASE + u32i * TMP_PAGE_SIZE);
            FMC_Erase(BANK0_FW_BASE + u32i * TMP_PAGE_SIZE);
            DumpSwapInfo(u32i * TMP_PAGE_SIZE);

            /* Swap step2: move from Bank1 to Bank0 */
            for(u32j = 0; u32j < TMP_PAGE_SIZE; u32j += 4)
            {
                FMC_Write(BANK0_FW_BASE + u32i * TMP_PAGE_SIZE + u32j, FMC_Read(BANK1_FW_BASE + u32i * TMP_PAGE_SIZE + u32j));
            }
            DumpSwapInfo(u32i * TMP_PAGE_SIZE);
            printf("Erase [0x%8x]\n", BANK1_FW_BASE + u32i * TMP_PAGE_SIZE);
            FMC_Erase(BANK1_FW_BASE + u32i * TMP_PAGE_SIZE);
            DumpSwapInfo(u32i * TMP_PAGE_SIZE);

            for(u32j = 0; u32j < TMP_PAGE_SIZE; u32j += 4)
            {
                FMC_Write(BANK1_FW_BASE + u32i * TMP_PAGE_SIZE + u32j, FMC_Read(TMP_PAGE_BASE + u32j));
            }
            DumpSwapInfo(u32i * TMP_PAGE_SIZE);

            u32StartPage = u32i + 1;
            break;
        }
        else if((u32TmpSum == g_au32Bank0PageSumInFlash[u32i])
                && (g_au32Bank0PageSum[u32i] == g_au32Bank1PageSumInFlash[u32i])
                && (g_au32Bank1PageSum[u32i] != g_au32Bank1PageSumInFlash[u32i]))
        {
            printf(" Re-start from step 3\n");
            printf("Erase [0x%8x]\n", BANK1_FW_BASE + u32i * TMP_PAGE_SIZE);
            FMC_Erase(BANK1_FW_BASE + u32i * TMP_PAGE_SIZE);
            DumpSwapInfo(u32i * TMP_PAGE_SIZE);

            /* Swap step3: move from tmp to Bank1 */
            for(u32j = 0; u32j < TMP_PAGE_SIZE; u32j += 4)
            {
                FMC_Write(BANK1_FW_BASE + u32i * TMP_PAGE_SIZE + u32j, FMC_Read(TMP_PAGE_BASE + u32j));
            }
            DumpSwapInfo(u32i * TMP_PAGE_SIZE);

            u32StartPage = u32i + 1;
            break;
        }
        else if((g_au32Bank0PageSum[u32i] == g_au32Bank1PageSumInFlash[u32i])
                && (g_au32Bank1PageSum[u32i] == g_au32Bank0PageSumInFlash[u32i]))
        {
            printf("Page[%d] has been completed!!!\n", u32i);
            u32StartPage = u32i + 1;
        }
        else if(g_au32Bank0PageSumInFlash[u32i] == g_au32Bank1PageSumInFlash[u32i])
        {
            u32StartPage = u32i + 1;
        }
        else
        {
            u32StartPage = u32i + 1;
            printf("Other condition!!!  page @[%d]\n", u32i);
        }
    }

    if(u32StartPage < BANK0_FW_SIZE / TMP_PAGE_SIZE)
    {
        FMC_Erase(TMP_PAGE_BASE);
        for(u32i = u32StartPage * TMP_PAGE_SIZE; u32i < BANK0_FW_SIZE; u32i += TMP_PAGE_SIZE)
        {
            printf("+-------------------------+\n");
            printf("|     Page[%d] Swap        |\n", u32i / TMP_PAGE_SIZE);
            printf("+-------------------------+\n\n");

            DumpSwapInfo(u32i);

            /* Swap step1: move data from Bank0 to tmp*/
            for(u32j = 0; u32j < TMP_PAGE_SIZE; u32j += 4)
            {
                FMC_Write(TMP_PAGE_BASE + u32j, FMC_Read(BANK0_FW_BASE + u32i + u32j));
            }
            DumpSwapInfo(u32i);
            printf("Erase [0x%8x]\n", BANK0_FW_BASE + u32i);
            FMC_Erase(BANK0_FW_BASE + u32i);
            DumpSwapInfo(u32i);

            /* Swap step2: move from Bank1 to Bank0*/
            for(u32j = 0; u32j < TMP_PAGE_SIZE; u32j += 4)
            {
                FMC_Write(BANK0_FW_BASE + u32i + u32j, FMC_Read(BANK1_FW_BASE + u32i + u32j));
            }
            DumpSwapInfo(u32i);
            printf("Erase [0x%8x]\n", BANK1_FW_BASE + u32i);
            FMC_Erase(BANK1_FW_BASE + u32i);
            DumpSwapInfo(u32i);

            /* Swap step3: move from tmp to Bank1*/
            for(u32j = 0; u32j < TMP_PAGE_SIZE; u32j += 4)
            {
                FMC_Write(BANK1_FW_BASE + u32i + u32j, FMC_Read(TMP_PAGE_BASE + u32j));
            }
            DumpSwapInfo(u32i);
            printf("Erase [0x%8x]\n", TMP_PAGE_BASE);
            FMC_Erase(TMP_PAGE_BASE);
            DumpSwapInfo(u32i);

            printf("\n");
        }
    }
}

/**
  * @brief    Check User Configuration CONFIG0 bit 6 IAP boot setting. If it's not boot with IAP
  *           mode, modify it and execute a chip reset to make it take effect.
  * @return   Is boot with IAP mode or not.
  * @retval   0   Success.
  * @retval   -1  Failed on reading or programming User Configuration.
  */
static int  set_IAP_boot_mode(void)
{
    uint32_t  au32Config[2];

    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");
        return -1;
    }

    /*
        CONFIG0[7:6]
        00 = Boot from LDROM with IAP mode.
        01 = Boot from LDROM without IAP mode.
        10 = Boot from APROM with IAP mode.
        11 = Boot from APROM without IAP mode.
    */
    if (au32Config[0] & 0x40)      /* Boot from APROM with IAP mode */
    {
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] &= ~0x40;
        FMC_WriteConfig(au32Config, 2);

        // Perform chip reset to make new User Config take effect
        SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    }
    return 0;
}

int32_t main(void)
{
    uint32_t  u32i, u32RoBase = 0x0;

    uint32_t  u32Ver0, u32Ver1, u32Crc0, u32Crc1, u32CrcGet0 = 0, u32CrcGet1 = 0;
    int32_t i32Ch;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*-----------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                       */
    /*-----------------------------------------------------------------------------------*/

    printf("+------------------------------------------+\n");
    printf("|     M471 FMC Dual Bank Sample Demo       |\n");
    printf("+------------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC_Open();

    FMC_ENABLE_AP_UPDATE();            /* Enable FMC erase/program APROM                 */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    set_IAP_boot_mode();

    printf("===== Reset: ===== \n");

    /* To check if system has been reset by WDT time-out reset or not */
    if(WDT_GET_RESET_FLAG() == 1)
    {
        WDT_CLEAR_RESET_FLAG();
        printf("=== System reset by WDT time-out event ===\n");
        printf("Any key to swap back to old FW\n");
        getchar();

        BankSwap();
        /* Set remmaping address */
        FMC->ISPADDR = BANK0_FW_BASE;
        /* Set VECMAP */
        FMC->ISPCMD = 0x2E;
        FMC->ISPTRG = 1;
        while(FMC->ISPTRG);
        /* CPU Reset */
        SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
        while(1) {}
    }

    if(u32RoBase == 0x0)
        printf("===== Boot code process ===== \n");
    else
        printf("===== Boot from [0x%x] ===== \n", u32RoBase);

    printf("[Prework] Press: \n");
    printf("[p] Execute in Bank0 FW buffer \n");
    printf("[x] Any key to continue \n");

    i32Ch = getchar();

    if(i32Ch == 'p')
    {
        /* Set remmaping address */
        FMC->ISPADDR = BANK0_FW_BASE;
        /* Set VECMAP */
        FMC->ISPCMD = FMC_ISPCMD_VECMAP;
        FMC->ISPTRG = 1;
        while(FMC->ISPTRG);
        /* CPU Reset */
        SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
    }

    u32Ver0 = FMC_Read(BANK0_FW_VER_BASE);
    u32Crc0 = FMC_Read(BANK0_FW_CRC_BASE);

    u32Ver1 = FMC_Read(BANK1_FW_VER_BASE);
    u32Crc1 = FMC_Read(BANK1_FW_CRC_BASE);

    /* Galculate Bank0 CRC */
    u32CrcGet0 = func_crc32(BANK0_FW_BASE, BANK0_FW_SIZE);
    u32CrcGet1 = func_crc32(BANK1_FW_BASE, BANK1_FW_SIZE);

    /* Swap page CRC data */
    if((u32CrcGet0 == u32Crc1) && (u32CrcGet1 == u32Crc0))
    {
        FMC_Erase(TMP_PAGE_BASE);
        /* Swap step1: move data from Bank0 to tmp*/
        for(u32i = 0; u32i < (BANK0_FW_SIZE / TMP_PAGE_SIZE + 2); u32i++)
        {
            FMC_Write(TMP_PAGE_BASE + u32i * 4, FMC_Read(BANK0_PAGE_CRC_BASE + u32i * 4));
        }
        /* Swap step2: move from Bank1 to Bank0*/
        FMC_Erase(BANK0_PAGE_CRC_BASE);
        for(u32i = 0; u32i < (BANK0_FW_SIZE / TMP_PAGE_SIZE + 2); u32i++)
        {
            FMC_Write(BANK0_PAGE_CRC_BASE + u32i * 4, FMC_Read(BANK1_PAGE_CRC_BASE + u32i * 4));
        }
        FMC_Erase(BANK1_PAGE_CRC_BASE);
        /* Swap step3: move from tmp to Bank1*/
        for(u32i = 0; u32i < (BANK0_FW_SIZE / TMP_PAGE_SIZE + 2); u32i++)
        {
            FMC_Write(BANK1_PAGE_CRC_BASE + u32i * 4, FMC_Read(TMP_PAGE_BASE + u32i * 4));
        }
        FMC_Erase(TMP_PAGE_BASE);

        FMC->ISPADDR = BANK0_FW_BASE;
        /* Set VECMAP */
        FMC->ISPCMD = 0x2E;
        FMC->ISPTRG = 1;
        while(FMC->ISPTRG);
        /* CPU Reset */
        SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
    }
    u32Ver0 = FMC_Read(BANK0_FW_VER_BASE);
    u32Crc0 = FMC_Read(BANK0_FW_CRC_BASE);
    printf("Bank0 FW ver[0x%x]  CheckSum[0x%8x]\n", u32Ver0, u32Crc0);

    u32Ver1 = FMC_Read(BANK1_FW_VER_BASE);
    u32Crc1 = FMC_Read(BANK1_FW_CRC_BASE);
    printf("Bank1 FW ver[0x%x]  CheckSum[0x%8x]\n", u32Ver1, u32Crc1);

    /* Galculate Bank0 CRC */
    u32CrcGet0 = func_crc32(BANK0_FW_BASE, BANK0_FW_SIZE);
    printf("Bank0 get CheckSum[0x%8x]\n", u32CrcGet0);
    /* Galculate Bank1 CRC */
    u32CrcGet1 = func_crc32(BANK1_FW_BASE, BANK1_FW_SIZE);
    printf("Bank1 get CheckSum[0x%8x]\n", u32CrcGet1);

    printf("============== \n");
    for(u32i = 0; u32i < BANK0_FW_SIZE / TMP_PAGE_SIZE; u32i++)
    {
        g_au32Bank0PageSumInFlash[u32i] = FMC_Read(BANK0_PAGE_CRC_BASE + u32i * 4);
        printf("Bank0 FW page[%d] check sum = 0x%8x\n", u32i, g_au32Bank0PageSumInFlash[u32i]);
    }
    printf("============== \n");

    for(u32i = 0; u32i < BANK1_FW_SIZE / TMP_PAGE_SIZE; u32i++)
    {
        g_au32Bank1PageSumInFlash[u32i] = FMC_Read(BANK1_PAGE_CRC_BASE + u32i * 4);
        printf("Bank1 FW page[%d] check sum = 0x%8x\n", u32i, g_au32Bank1PageSumInFlash[u32i]);
    }
    printf("============== \n");

    printf("Any key to continue \n");
    getchar();

    if(u32Ver0 == 0xFFFFFFFF)
    {
        printf("No firmware in executing address\n");
        while(1);
    }
    else
    {
        if(u32Ver1 == 0xFFFFFFFF)
        {
            printf("No new firmware exist\n");
            printf("\n");
            /* Set remmaping address */
            FMC->ISPADDR = BANK0_FW_BASE;
            /* Set VECMAP */
            FMC->ISPCMD = 0x2E;
            FMC->ISPTRG = 1;
            while(FMC->ISPTRG);
            /* CPU Reset */
            SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
        }
    }
    if((u32CrcGet0 == u32Crc0) && (u32CrcGet1 == u32Crc1))
    {
        if(u32Ver0 < u32Ver1)
        {
            printf("Bank1 FW is new, do bank0/bank1 swap \n");
            BankSwap();
            printf("Swap done!!! \n");
            /* Set remmaping address */
            FMC->ISPADDR = BOOT_BASE;
            /* Set VECMAP */
            FMC->ISPCMD = 0x2E;
            FMC->ISPTRG = 1;
            while(FMC->ISPTRG);
            /* CPU Reset */
            SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
        }
        else
        {
#if FW_SWAP_BACK
            printf("Swap back to old FW? [y/n]\n");
            i32Ch = getchar();
            if(i32Ch == 'y')
            {
                printf("Swap back to old FW\n");
                BankSwap();
                printf("Swap done!!! \n");
                /* Set remmaping address */
                FMC->ISPADDR = BOOT_BASE;
                /* Set VECMAP */
                FMC->ISPCMD = 0x2E;
                FMC->ISPTRG = 1;
                while(FMC->ISPTRG);
                /* CPU Reset */
                SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
            }
            else
#endif
            {
                NVIC_EnableIRQ(WDT_IRQn);

                /* Because of all bits can be written in WDT Control Register are write-protected;
                   To program it needs to disable register protection first. */
                SYS_UnlockReg();

#if WDT_EN
                /* Configure WDT settings and start WDT counting */
                WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, 1, 0);
#endif
                /* Enable WDT interrupt function */
                WDT_EnableInt();
                printf(" Execute new FW\n");
                /* Set remmaping address */
                FMC->ISPADDR = BANK0_FW_BASE;
                /* Set VECMAP */
                FMC->ISPCMD = 0x2E;
                FMC->ISPTRG = 1;
                while(FMC->ISPTRG);
                /* CPU Reset */
                SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
            }
        }
    }
    else
    {
        if(u32CrcGet0 == u32Crc0)
        {
            printf("Bank1 FW error, execute old FW in Bank0 \n");
            /* Set remmaping address */
            FMC->ISPADDR = BANK0_FW_BASE;
            /* Set VECMAP */
            FMC->ISPCMD = 0x2E;
            FMC->ISPTRG = 1;
            while(FMC->ISPTRG);
            /* CPU Reset */
            SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
        }
        else
        {
            printf("Swap not completed condition\n");
            BankSwapContinue();
            printf("Swap done!!! \n");
            /* Set remmaping address */
            FMC->ISPADDR = BOOT_BASE;
            /* Set VECMAP */
            FMC->ISPCMD = 0x2E;
            FMC->ISPTRG = 1;
            while(FMC->ISPTRG);
            /* CPU Reset */
            SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
        }
    }

    printf("\nDual Bank Boot Loader completed!.\n");

    while(1);
}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


