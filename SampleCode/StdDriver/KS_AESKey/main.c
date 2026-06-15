/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demo to use the AES with Key Store.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define AES_BUFF_SIZE           1024
#define AES_CHANNEL             0
#define AES_ENCRYPT             1
#define AES_DECRYPT             0
#define KS_MEM_TYPE(eMemType)   ((eMemType == KS_SRAM ? "KS_SRAM" : "KS_Flash"))

static volatile int32_t s_i32AES_Done = FALSE, s_i32AES_Err = FALSE;

#if (NVT_DCACHE_ON == 1)
/**
 * When D-Cache is enabled, if the buffer is frequently accessed by the CPU and shared between the CPU and DMA,
 * the buffer size and address must be aligned with DCACHE_LINE_SIZE.
 * This ensures proper cache operation and prevents data corruption caused by cache coherence issues.
 *
 * Placing the buffer in the cacheable region improves CPU access performance, as the cache accelerates data read/write operations.
 * However, if the buffer is also accessed by DMA and is not aligned with DCACHE_LINE_SIZE, it may lead to data inconsistency
 * between the DMA and CPU, causing data corruption or unexpected behavior.
 *
 * If the buffer is primarily used by DMA and does not require frequent CPU access, it is recommended to declare the buffer
 * with NVT_NONCACHEABLE. This avoids cache coherence issues by placing the buffer in a non-cacheable region,
 * ensuring it is not affected by D-Cache and does not require additional cache maintenance.
 */
static uint8_t g_au8DataBuf[DCACHE_ALIGN_LINE_SIZE(32)] __ALIGNED(DCACHE_LINE_SIZE) =
{
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88,
    0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00
};
static uint8_t g_au8EncBuf[DCACHE_ALIGN_LINE_SIZE(AES_BUFF_SIZE)] __ALIGNED(DCACHE_LINE_SIZE);
static uint8_t g_au8DecBuf[DCACHE_ALIGN_LINE_SIZE(AES_BUFF_SIZE)] __ALIGNED(DCACHE_LINE_SIZE);
#else
static uint8_t g_au8DataBuf[] __ALIGNED(4) =
{
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88,
    0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00
};
static uint8_t g_au8EncBuf[AES_BUFF_SIZE] __ALIGNED(4);
static uint8_t g_au8DecBuf[AES_BUFF_SIZE] __ALIGNED(4);
#endif

int AES_Test(CRYPTO_T *pCrypto, KS_MEM_Type eMemType, int32_t keyIdx);
void DumpBuf(uint8_t *pu8Buf, uint32_t u32BufByteSize);
void SYS_Init(void);
void UART_Init(void);

NVT_ITCM void CRYPTO_IRQHandler(void)
{
    if (AES_GET_INT_FLAG(CRYPTO) & CRYPTO_INTSTS_AESIF_Msk)
    {
        s_i32AES_Done = TRUE;
    }

    if (AES_GET_INT_FLAG(CRYPTO) & CRYPTO_INTSTS_AESEIF_Msk)
    {
        s_i32AES_Err = TRUE;
    }

    AES_CLR_INT_FLAG(CRYPTO);
    ECC_Complete(CRYPTO);
}

void DumpBuf(uint8_t *pu8Buf, uint32_t u32BufByteSize)
{
    int nIdx, i, j;

    nIdx = 0;

    while (u32BufByteSize > 0)
    {
        j = u32BufByteSize;

        if (j > 16)
        {
            j = 16;
        }

        printf("0x%04X  ", nIdx);

        for (i = 0; i < j; i++)
            printf("%02x ", pu8Buf[nIdx + i]);

        for (; i < 16; i++)
            printf("   ");

        printf("  ");

        for (i = 0; i < j; i++)
        {
            if ((pu8Buf[nIdx + i] >= 0x20) && (pu8Buf[nIdx + i] < 127))
                printf("%c", pu8Buf[nIdx + i]);
            else
                printf(".");

            u32BufByteSize--;
        }


        nIdx += j;
        printf("\n");
    }

    printf("\n");
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(KS0_MODULE);
    CLK_EnableModuleClock(CRYPTO0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i32KeyIdx;
    KS_MEM_Type eMemType = KS_SRAM;

    /* AES encryption/decryption key: 0x7A29E38E_063FF08A_2F7A7F2A_93484D6F
       Note: Each word is stored in reverse order for Key Store. */
    uint32_t au32Key[4] = { 0x93484D6F, 0x2F7A7F2A, 0x063FF08A, 0x7A29E38E };

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|            KS AES Sample Code            |\n");
    printf("+------------------------------------------+\n");

    if (KS_Open() != KS_OK)
    {
        printf("KS is not ready !\n");

        while (1) {};
    }

    i32KeyIdx = KS_Write(eMemType, KS_META_128 | KS_META_AES, au32Key);

    if (i32KeyIdx < 0)
    {
        printf("Fail to write key to %s!\n", KS_MEM_TYPE(eMemType));
        printf("The remain size = %d, remain key count: %d\n",
               KS_GetRemainSize(eMemType), KS_GetRemainKeyCount(eMemType));

        if (KS_GetRemainSize(eMemType) == 0 || (KS_GetRemainKeyCount(eMemType) == 0))
        {
            printf("%s is full then we can use KS_EraseAll(eMemType) to erase all keys.\n", KS_MEM_TYPE(eMemType));

            if (KS_EraseAll(eMemType) == KS_OK)
                printf("Erase done. Please test again.\n");
        }

        goto lexit;
    }

    printf("%s remain size: %d\n", KS_MEM_TYPE(eMemType), KS_GetRemainSize(eMemType));
    printf("The stored key %d in %s:\n", i32KeyIdx, KS_MEM_TYPE(eMemType));
    DumpBuf((uint8_t *)au32Key, sizeof(au32Key));
    AES_Test(CRYPTO, eMemType, i32KeyIdx);
    printf("Done\n");

lexit:

    while (1) {};
}


int AES_Test(CRYPTO_T *pCrypto, KS_MEM_Type eMemType, int32_t keyIdx)
{
    uint32_t u32DataByteSize;
    /* AES IV: 0xF8C44B6F_BDF96B83_5547FF45_DE1FFC92
       Note: Each word is stored in reverse order for AES. */
    uint32_t au32IV[4] = { 0xDE1FFC92, 0x5547FF45, 0xBDF96B83, 0xF8C44B6F };
    uint32_t u32TimeOutCnt;

    /* Enable Crypto interrupt */
    NVIC_EnableIRQ(CRYPTO_IRQn);

    /* Original data */
    printf("The input data:\n");
    u32DataByteSize = sizeof(g_au8DataBuf);
    DumpBuf(g_au8DataBuf, u32DataByteSize);

    /*---------------------------------------
     *  AES-128 ECB mode encrypt
     *---------------------------------------*/
    AES_Open(pCrypto, AES_CHANNEL, AES_ENCRYPT, AES_MODE_CFB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);

    /* Use key in key store */
    AES_SetKey_KS(pCrypto, eMemType, keyIdx);

    /* Provide the IV key */
    AES_SetInitVect(pCrypto, AES_CHANNEL, au32IV);

    /* Prepare the source data and output buffer */
    if (u32DataByteSize & 0xf)
    {
        printf("Alignment Error!\n");
        printf("For AES CFB mode, Input data must be 16 bytes (128 bits) alignment.\n");
        return -1;
    }

    memset(g_au8EncBuf, 0, sizeof(g_au8EncBuf));
    memset(g_au8DecBuf, 0, sizeof(g_au8DecBuf));
#if (NVT_DCACHE_ON == 1)
    /* Clean the data cache for the input buffer to ensure data coherency if D-Cache is enabled. */
    SCB_CleanDCache_by_Addr(g_au8DataBuf, sizeof(g_au8DataBuf));
#endif  // (NVT_DCACHE_ON == 1)
    /* Set the input and output buffer address */
    AES_SetDMATransfer(pCrypto, AES_CHANNEL, (uint32_t)g_au8DataBuf, (uint32_t)g_au8EncBuf, (uint32_t)u32DataByteSize);

    AES_ENABLE_INT(pCrypto);
    s_i32AES_Done = FALSE;
    s_i32AES_Err = FALSE;

    /* Start AES encrypt */
    AES_Start(pCrypto, AES_CHANNEL, CRYPTO_DMA_ONE_SHOT);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!s_i32AES_Done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for AES encode time-out!\n");
            return -1;
        }
    }

    if (s_i32AES_Err)
    {
        printf("AES Encode Fail!\n");
        return -1;
    }

#if (NVT_DCACHE_ON == 1)
    /* Because g_au8EncBuf is transferred by AES DMA,
       invalidate g_au8EncBuf to ensure data coherency if D-Cache is enabled after AES DMA transfer done. */
    SCB_InvalidateDCache_by_Addr(g_au8EncBuf, sizeof(g_au8EncBuf));
#endif  // (NVT_DCACHE_ON == 1)
    printf("AES encrypt done. The output data:\n");
    DumpBuf(g_au8EncBuf, 16);

    /*---------------------------------------
     *  AES-128 ECB mode decrypt
     *---------------------------------------*/
    AES_Open(pCrypto, AES_CHANNEL, AES_DECRYPT, AES_MODE_CFB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);

    /* Use key in key store */
    AES_SetKey_KS(pCrypto, eMemType, keyIdx);

    /* Provide the IV key */
    AES_SetInitVect(pCrypto, AES_CHANNEL, au32IV);

#if (NVT_DCACHE_ON == 1)
    /* Clean the data cache for the input buffer to ensure data coherency if D-Cache is enabled. */
    SCB_CleanDCache_by_Addr(g_au8EncBuf, sizeof(g_au8EncBuf));
#endif  // (NVT_DCACHE_ON == 1)
    /* Set the input and output buffer address */
    AES_SetDMATransfer(pCrypto, AES_CHANNEL, (uint32_t)g_au8EncBuf, (uint32_t)g_au8DecBuf, (uint32_t)u32DataByteSize);

    AES_ENABLE_INT(pCrypto);
    s_i32AES_Done = FALSE;
    s_i32AES_Err = FALSE;

    /* Start AES decrypt */
    AES_Start(pCrypto, AES_CHANNEL, CRYPTO_DMA_ONE_SHOT);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!s_i32AES_Done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for AES decode time-out!\n");
            return -1;
        }
    }

    if (s_i32AES_Err)
    {
        printf("AES Decode Fail!\n");
        return -1;
    }

#if (NVT_DCACHE_ON == 1)
    /* Because g_au8DecBuf is transferred by AES DMA,
       invalidate g_au8DecBuf to ensure data coherency if D-Cache is enabled after AES DMA transfer done. */
    SCB_InvalidateDCache_by_Addr(g_au8DecBuf, sizeof(g_au8DecBuf));
#endif  // (NVT_DCACHE_ON == 1)
    printf("AES decrypt done. The output data:\n");
    DumpBuf(g_au8DecBuf, u32DataByteSize);

    if (memcmp((char *)g_au8DataBuf, (char *)g_au8DecBuf, (uint32_t)u32DataByteSize))
    {
        printf("[FAILED]\n");
        return -1;
    }


    return 0;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
