/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show FMC read Flash IDs, erase, read, and write functions
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

#define TEST_BUF_SIZE       (4096)

/* Declare variable or buffer placed in NonCacheable region */
/* Use NVT_NONCACHEABLE_INIT to declare variables or buffer with initial value placed in NonCacheable region */
NVT_NONCACHEABLE_INIT   uint32_t g_u32TestVar = 123;
/* Use NVT_NONCACHEABLE to declare variables or buffer without initial value placed in NonCacheable region */
NVT_NONCACHEABLE        uint32_t g_au32PDMA_DstBuf[TEST_BUF_SIZE / 4] __ALIGNED(32);
NVT_NONCACHEABLE        uint32_t g_au32NonCacheableBuf[TEST_BUF_SIZE / 4];

/* Declare variables or buffer placed in DTCM region */
/* Use NVT_DTCM_INIT to declare variables or buffer with initial value placed in DTCM region */
NVT_DTCM_INIT           uint32_t g_u32DTCMTestVar = 456;
/* Use NVT_DTCM to declare variables or buffer without initial value placed in DTCM region */
NVT_DTCM             uint32_t g_au32DTCMBuf[TEST_BUF_SIZE / 4] __ALIGNED(32);

/* Declare variable or buffer placed in Cacheable region */
uint32_t g_au32CacheableBuf[DCACHE_ALIGN_LINE_SIZE(TEST_BUF_SIZE / 4)] __ALIGNED(DCACHE_LINE_SIZE);

void MemManage_Handler(void)
{
    uint32_t u32LR = 0;
    uint32_t *pu32SP;

    __ASM volatile("mov %0, lr\n" : "=r"(u32LR));

    if (u32LR & BIT2)
        pu32SP = (uint32_t *)__get_PSP();
    else
        pu32SP = (uint32_t *)__get_MSP();

    /* NOTE1: Disable MPU to allow simple return from MemManage_Handler().
              MemManage fault typically indicates code failure, and would
              be resolved by reset or terminating faulty thread in OS.
       NOTE2: Call ARM_MPU_Disable() will allow the code touch
              illegal address to be executed after return from
              HardFault_Handler(). If this line is comment out, this code
              will keep enter MemManage_Handler(). */
    ARM_MPU_Disable();
    printf("\n  Memory Fault !\n");

    if (SCB->CFSR & SCB_CFSR_IACCVIOL_Msk)
    {
        printf("  Instruction access violation flag is raised.");
        /* Check disassembly code of MemManage_Handler to get stack offset of return address */
        printf("  Fault address: 0x%08X\n", pu32SP[10]);
    }

    if (SCB->CFSR & SCB_CFSR_DACCVIOL_Msk)
    {
        printf("  Data access violation flag is raised.");

        if (SCB->CFSR & SCB_CFSR_MMARVALID_Msk)
            printf("  Fault address: 0x%08X\n", SCB->MMFAR);
    }

    /* Clear MemManage fault status register */
    SCB->CFSR = SCB_CFSR_MEMFAULTSR_Msk;
}

void SYS_Init(void)
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

    /* Enable External Xtal clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External Xtal clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

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

int32_t PDMA_Transfer(uint32_t u32Chan, uint32_t *pu32SrcBuf, uint32_t u32TranByteSize)
{
    /* Open Channel */
    PDMA_Open(PDMA0, (1 << u32Chan));
    /* Transfer count is u32TranByteSize / 4, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, u32Chan, PDMA_WIDTH_32, u32TranByteSize / 4);
    /* Set source address is pu32SrcBuf, destination address is g_au32PDMA_DstBuf, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA0, u32Chan, (uint32_t)pu32SrcBuf, PDMA_SAR_INC, (uint32_t)g_au32PDMA_DstBuf, PDMA_DAR_INC);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA0, u32Chan, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA0, u32Chan, PDMA_REQ_BURST, PDMA_BURST_4);

    /* Generate a software request to trigger transfer with PDMA channel */
    PDMA_Trigger(PDMA0, u32Chan);

    while ((PDMA_GET_INT_STATUS(PDMA0) & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk)) == 0)
        ;

    if (PDMA_GET_TD_STS(PDMA0) & (PDMA_TDSTS_TDIF0_Msk << u32Chan))
        PDMA_CLR_TD_FLAG(PDMA0, (PDMA_TDSTS_TDIF0_Msk << u32Chan));

    if (PDMA_GET_ABORT_STS(PDMA0) & (PDMA_ABTSTS_ABTIF0_Msk << u32Chan))
    {
        /* Clear abort flag of channel */
        PDMA_CLR_ABORT_FLAG(PDMA0, (PDMA_ABTSTS_ABTIF0_Msk << u32Chan));
        printf("PDMA transfer abort !\n");
        return -1;
    }

    PDMA_Close(PDMA0);
    return 0;
}

/* Declare function placed in ITCM region */
NVT_ITCM int32_t WriteTest(uint32_t *pu32BaseAddr, uint32_t u32ByteSize)
{
    uint8_t  au8TestPattern[] = { 0x55, 0xAA, 0x11, 0xFF };
    int32_t  i32RetCode = 0;
    uint32_t i, u32Offset, u32DataErrCnt;

    memset(g_au32PDMA_DstBuf, 0x0, sizeof(g_au32PDMA_DstBuf));

    for (i = 0; i < (sizeof(au8TestPattern) / sizeof(au8TestPattern[0])); i++)
    {
        memset(pu32BaseAddr, 0x0, u32ByteSize);
        /* Write data with CPU */
        /* Write back mode   : data is write to cache
         * Write through mode: data is write to cache and memory
         */
        printf("CPU write (addr + 0x%02X)\n", au8TestPattern[i]);

        for (u32Offset = 0; u32Offset < u32ByteSize; u32Offset += 4)
        {
            pu32BaseAddr[u32Offset / 4] = (u32Offset + au8TestPattern[i]);
        }

        /* Check with CPU */
        printf("CPU check ");

        for (u32Offset = 0; u32Offset < u32ByteSize; u32Offset += 4)
        {
            if (pu32BaseAddr[u32Offset / 4] != (u32Offset + au8TestPattern[i]))
            {
                printf("[Error] CPU read (0x%08X != 0x%08X) !\n", pu32BaseAddr[u32Offset / 4], (u32Offset + au8TestPattern[i]));
                return -1;
            }
        }

        printf("Pass\n");

        /* Check with PDMA */
        printf("PDMA transfer\n");

        if (PDMA_Transfer(0, pu32BaseAddr, u32ByteSize) != 0)
        {
            printf("[Error] Failed to do PDMA transfer !\n");
            return -2;
        }

        u32DataErrCnt = 0;

        for (u32Offset = 0; u32Offset < u32ByteSize; u32Offset += 4)
        {
            if (g_au32PDMA_DstBuf[u32Offset / 4] != pu32BaseAddr[u32Offset / 4])
            {
                if (u32DataErrCnt == 0)
                    printf("  Check (0x%08X != 0x%08X).\n", g_au32PDMA_DstBuf[u32Offset / 4], pu32BaseAddr[u32Offset / 4]);

                u32DataErrCnt++;
            }
        }

        if (u32DataErrCnt > 0)
        {
            printf("  [Error] Check PDMA copied data - Failed !\n");
            i32RetCode++;
        }
        else
            printf("  Check PDMA copied data - Pass.\n");
    }

    g_u32TestVar += i32RetCode;
    g_u32DTCMTestVar += g_u32TestVar;
    return i32RetCode;
}

extern uint32_t g_u32NonCacheableBase, g_u32NonCacheableLimit;

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|    M55M1 MPU NonCacheable Sample Code    |\n");
    printf("+------------------------------------------+\n");
    printf("Base: 0x%08X, Limit: 0x%08X\n", g_u32NonCacheableBase, g_u32NonCacheableLimit);
    printf("g_u32TestVar: %d and its address is 0x%08X.\n", g_u32TestVar, (uint32_t)&g_u32TestVar);
    printf("g_u32DTCMTestVarn: %d and its address is 0x%08X.\n", g_u32DTCMTestVar, (uint32_t)&g_u32DTCMTestVar);

    if ((uint32_t)WriteTest >= FMC_APROM_BASE)
        printf("[Error] WriteTest not in ITCM region (0x%08X) !\n", (uint32_t)WriteTest);

    printf("\nTest Cacheable Buffer - PDMA check should fail.\n");

    if (WriteTest(g_au32CacheableBuf, TEST_BUF_SIZE) == 0)
        printf("[Error] WriteTest did not fail !\n");

    printf("g_u32TestVar: %d, g_u32DTCMTestVar: %d\n", g_u32TestVar, g_u32DTCMTestVar);

    g_au32NonCacheableBuf[0] = 0x11223344;
    g_au32NonCacheableBuf[1] = 0x55667788;
    printf("Test unaligned access\n");
    printf("  Read ((uint32_t)g_au32NonCacheableBuf + 1) should return 0x88112233. (0x%08X)\n",
           *(uint32_t *)(((uint32_t)g_au32NonCacheableBuf) + 1));
    printf("  Read ((uint32_t)g_au32NonCacheableBuf + 2) should return 0x77881122. (0x%08X)\n",
           *(uint32_t *)(((uint32_t)g_au32NonCacheableBuf) + 2));
    printf("  Read ((uint32_t)g_au32NonCacheableBuf + 3) should return 0x66778811. (0x%08X)\n\n",
           *(uint32_t *)(((uint32_t)g_au32NonCacheableBuf) + 3));

    printf("\nTest NonCacheable Buffer - PDMA check should pass.\n");

    if (WriteTest(g_au32NonCacheableBuf, TEST_BUF_SIZE) != 0)
        printf("[Error] WriteTest did not pass !\n");

    printf("g_u32TestVar: %d, g_u32DTCMTestVar: %d\n", g_u32TestVar, g_u32DTCMTestVar);

    printf("\nTest DTCM Buffer - PDMA check should pass.\n");

    if (WriteTest(g_au32DTCMBuf, TEST_BUF_SIZE) != 0)
        printf("[Error] WriteTest did not pass !\n");

    printf("g_u32TestVar: %d, g_u32DTCMTestVar: %d\n", g_u32TestVar, g_u32DTCMTestVar);

    printf("\nDone\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
