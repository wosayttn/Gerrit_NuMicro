/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Template for M5531 series MCU
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/*
 * This is a template project for M5531 series MCU.
 * Users can create their own application based on this project.
 *
 * This template uses internal RC as APLL0 clock source and UART to print messages.
 * Users may need to do extra system configuration according to their system design.
 *
 * I/D-Cache
 *   I-Cache are enabled by default for better performance.
 *   Users can define NVT_DCACHE_ON in project setting to enabld D-Cache.
 * Debug UART
 *   system_M5531.c has three weak functions as below to configure debug UART port.
 *     SetDebugUartMFP, SetDebugUartCLK and InitDebugUart
 *   Users can re-implement these functions according to system design.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

#define PDMA_CHAN  0
/* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
uint32_t g_au32TestSrcBuf[DCACHE_ALIGN_LINE_SIZE(256)] __ALIGNED(DCACHE_LINE_SIZE),
         g_au32TestDstBuf[DCACHE_ALIGN_LINE_SIZE(256)] __ALIGNED(DCACHE_LINE_SIZE);

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 220MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t  i;
    uint32_t u32Addr;
    uint32_t u32SrcSum, u32DstSum;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|         M5531 L1 Cache Test Code         |\n");
    printf("+------------------------------------------+\n");

    printf("+------------------------------------------+\n");
    printf("|           Cache test with PDMA           |\n");
    printf("+------------------------------------------+\n");
    /*
     *   CPU write g_au32TestSrcBuf => CPU write to g_au32TestSrcBuf (in DCache)
     *   CPU read g_au32TestDstBuf  => CPU read from g_au32TestDstBuf (in SRAM) to DCache
     *
     *   CleanDCache_by_Addr of g_au32TestSrcBuf => CPU flush DCache to g_au32TestSrcBuf (in SRAM)
     *     Addr and size must be 32 bytes alignment
     *
     *   g_au32TestSrcBuf (in SRAM) -> PDMA -> g_au32TestDstBuf (in SRAM)
     *
     *   CPU read g_au32TestDstBuf  => CPU read from g_au32TestDstBuf (in DCache)
     *     => CPU read old data
     *
     *   InvalidateDCache_by_Addr of g_au32TestDstBuf => CPU invalid g_au32TestDstBuf (in DCache)
     *     Addr and size must be 32 bytes alignment
     *
     *   CPU read g_au32TestDstBuf  => CPU read from g_au32TestDstBuf (in SRAM) to DCache
     *     => CPU read new data
     */
    SYS_ResetModule(SYS_PDMA0RST);

    memset(g_au32TestDstBuf, 0x0, sizeof(g_au32TestDstBuf));
    u32SrcSum = 0;

    for (i = 0; i < (sizeof(g_au32TestSrcBuf) / sizeof(g_au32TestSrcBuf[0])); i++)
    {
        g_au32TestSrcBuf[i] = i + 0xA5;
        u32SrcSum += g_au32TestSrcBuf[i];
    }

    printf("Check sum before PDMA transfer\n");
    u32DstSum = 0;

    for (i = 0; i < (sizeof(g_au32TestDstBuf) / sizeof(g_au32TestDstBuf[0])); i++)
        u32DstSum += g_au32TestDstBuf[i];

    printf("  u32DstSum: %d, u32SrcSum: %d\n", u32DstSum, u32SrcSum);

    /* Open Channel 0 */
    PDMA_Open(PDMA0, (1 << PDMA_CHAN));
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, PDMA_CHAN, PDMA_WIDTH_32, sizeof(g_au32TestSrcBuf) / 4);
    /* Set source address is g_au32TestSrcBuf, destination address is g_au32TestDstBuf, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA0, PDMA_CHAN, (uint32_t)g_au32TestSrcBuf, PDMA_SAR_INC, (uint32_t)g_au32TestDstBuf, PDMA_DAR_INC);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA0, PDMA_CHAN, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA0, PDMA_CHAN, PDMA_REQ_BURST, PDMA_BURST_4);

#if (NVT_DCACHE_ON == 1)
    /* Flush cached data of g_au32TestSrcBuf (Buffer addr and size must be 32 bytes alignment) */
    SCB_CleanDCache_by_Addr((void *)g_au32TestSrcBuf, sizeof(g_au32TestSrcBuf));
#endif

    /* Generate a software request to trigger transfer with PDMA channel 0 */
    PDMA_Trigger(PDMA0, PDMA_CHAN);

    while ((PDMA_GET_INT_STATUS(PDMA0) & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk)) == 0)
        ;

    if (PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF0_Msk)
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);

    if (PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF0_Msk)
    {
        /* Clear abort flag of channel 0 */
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
        printf("PDMA transfer abort !\n");
        goto Error_Exit;
    }

    PDMA_Close(PDMA0);

    printf("Check sum after PDMA transfer without invalidate DCache\n");
    u32DstSum = 0;

    for (i = 0; i < (sizeof(g_au32TestDstBuf) / sizeof(g_au32TestDstBuf[0])); i++)
        u32DstSum += g_au32TestDstBuf[i];

    if (u32DstSum != u32SrcSum)
        printf("  [OK] DCache work - u32DstSum: %d != u32SrcSum: %d\n", u32DstSum, u32SrcSum);
    else
        printf("  [Error] DCache not work ! u32DstSum: %d == u32SrcSum: %d\n", u32DstSum, u32SrcSum);

#if (NVT_DCACHE_ON == 1)
    /* Invalidate cached data of g_au32TestDstBuf (Buffer addr and size must be 32 bytes alignment) */
    SCB_InvalidateDCache_by_Addr((void *)g_au32TestDstBuf, sizeof(g_au32TestDstBuf));
    printf("Check sum after invalidate DCache\n");
    u32DstSum = 0;

    for (i = 0; i < (sizeof(g_au32TestDstBuf) / sizeof(g_au32TestDstBuf[0])); i++)
    {
        if (g_au32TestDstBuf[i] != g_au32TestSrcBuf[i])
        {
            printf("g_au32TestDstBuf[%d] 0x%08X != g_au32TestSrcBuf[%d] 0x%08X\n", i, g_au32TestDstBuf[i], i, g_au32TestSrcBuf[i]);
            goto Error_Exit;
        }

        u32DstSum += g_au32TestDstBuf[i];

    }

    if (u32DstSum == u32SrcSum)
        printf("  [OK] Invalid DCache work - u32DstSum: %d == u32SrcSum: %d\n", u32DstSum, u32SrcSum);
    else
        printf("  [Error] Invalid DCache not work ! u32DstSum: %d != u32SrcSum: %d\n", u32DstSum, u32SrcSum);

#endif

    printf("+------------------------------------------+\n");
    printf("|           Cache test with FMC            |\n");
    printf("+------------------------------------------+\n");
    SYS_UnlockReg();
    FMC_Open();         /* Enable FMC ISP function */
    FMC_ENABLE_LD_UPDATE();

    printf("Erase LDROM\n");

    if (FMC_Erase(FMC_LDROM_BASE) != 0)
    {
        printf("  Failed to erase LDROM !\n");
        goto Error_Exit;
    }

    printf("  [OK]\n");
    printf("Check LDROM all 0xFFFFFFFF\n");

    for (u32Addr = FMC_LDROM_BASE; u32Addr < FMC_LDROM_END; u32Addr += 4)
    {
        if (inp32(u32Addr) != 0xFFFFFFFF)
        {
            printf("  Read [0x%08X] 0x%08X != 0xFFFFFFFF\n", u32Addr, inp32(u32Addr));
            goto Error_Exit;
        }
    }

    printf("  [OK]\n");
    printf("Write LDROM\n");

    for (u32Addr = FMC_LDROM_BASE; u32Addr < FMC_LDROM_END; u32Addr += 4)
    {
        if (FMC_Write(u32Addr, u32Addr) != 0 || FMC_GET_FAIL_FLAG())
        {
            FMC_CLR_FAIL_FLAG();
            printf("  Write [0x%08X] failed !\n", u32Addr);
            goto Error_Exit;
        }
    }

    printf("  [OK]\n");
    printf("Check LDROM after write without invalidate DCache\n");

    for (u32Addr = FMC_LDROM_BASE; u32Addr < FMC_LDROM_END; u32Addr += 4)
    {
        if (inp32(u32Addr) != u32Addr)
        {
            printf("  [OK] DCache work - Read [0x%08X] 0x%08X != 0x%08X\n", u32Addr, inp32(u32Addr), u32Addr);
            break;
        }
    }

    if (u32Addr == FMC_LDROM_END)
        printf("  [Error] DCache not work !\n");

#if (NVT_DCACHE_ON == 1)
    /* Invalidate cached data of LDROM (Buffer addr and size must be 32 bytes alignment) */
    SCB_InvalidateDCache_by_Addr((void *)FMC_LDROM_BASE, FMC_LDROM_SIZE);
    printf("Check LDROM after invalidate DCache\n");

    for (u32Addr = FMC_LDROM_BASE; u32Addr < FMC_LDROM_END; u32Addr += 4)
    {
        if (inp32(u32Addr) != u32Addr)
        {
            printf("  [Error] Invalid DCache not work ! Read [0x%08X] 0x%08X != 0x%08X\n", u32Addr, inp32(u32Addr), u32Addr);
            goto Error_Exit;
        }
    }

    printf("  [OK] Invalid DCache work\n");
#endif

    printf("Done\n");
Error_Exit:

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
