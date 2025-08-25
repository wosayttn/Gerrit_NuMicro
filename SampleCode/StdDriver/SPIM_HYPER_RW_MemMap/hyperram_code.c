/**************************************************************************//**
 * @file     hyperram_code.c
 * @version  V1.03
 * @brief    Collect of sub-routines running on SPIM flash.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "NuMicro.h"
#include "hyperram_code.h"

//------------------------------------------------------------------------------
#ifndef HRAM_DEBUG
    #define HRAM_DEBUG              0
#endif

#if HRAM_DEBUG
    #define HRAM_DBGMSG   printf
#else
    #define HRAM_DBGMSG(...)   do { } while (0) /* disable debug */
#endif

#define SPIM_HYPER_DIV              1       // Divider for SPIM HyperBus clock

#define HYPERRAM_CSM_TIME           4000    // Chip Select Maximum (tCSM) timing in nanoseconds
#define HYPERRAM_RD_LTCY            7       // Read latency for HyperRAM
#define HYPERRAM_WR_LTCY            7       // Write latency for HyperRAM
#define HYPERRAM_CSHI_CYCLE         4       // Chip Select High time in cycles
#define HYPER_RAM_RST_CNT           0xFF    // Reset count for HyperRAM
#define VERIFY_ERASE_PATTERN        0x0000  // Pattern used to verify erase operation
#define CSMAXLT_CIPHER_OFF          21      // Maximum low time for Chip Select when cipher is off
#define CSMAXLT_CIPHER_ON           54      // Maximum low time for Chip Select when cipher is on

#define DMM_VERIFY_MAX_RETRY        3       // Maximum number of retries for DMM verification
#define DLL_TRIM_MAX_RETRY          10      // Maximum number of retries for DLL trimming
#define DLL_TRIM_WINDOW_SIZE        5       // Size of the window for DLL trimming
#define DLL_TRIM_SCORE_PER_BLOCK    8       // Score assigned per block during DLL trimming
#define DLL_TRIM_MIN_VALID_SCORE    64      // Minimum valid score for DLL trimming to pass
#define DLL_TRIM_PASS_COUNT         3       // Number of passes required for DLL trimming to be considered successful
#define DLL_TRIM_JUMP_STEP_1        0x08    // First jump step value for DLL trimming
#define DLL_TRIM_JUMP_STEP_2        0x10    // Second jump step value for DLL trimming
#define DLL_TRIM_DEF_NUM            0x07    // Default trimming number for DLL
#define HRAM_TRIM_SAFE_OFFSET       0x10000 // Safe offset for HyperRAM trimming
#define HRAM_TRIM_SIZE              512     // Trim size must be a multiple of 8

//------------------------------------------------------------------------------
/**
  * @brief      SPIM Default Config HyperBus Access Module Parameters.
  * @param      spim
  * @param      u32CSM    Refer to the Hyper Device Specific Chip Select Maximum (tCSM) timing parameters.
  *                       The reference Winbond HyperRAM is 4000ns.
  * @param      u32AcctRD Initial Read Access Time 1 ~ 0x1F, Default Set 0x04
  * @param      u32AcctWR Initial Write Access Time 1 ~ 0x1F, Default Set 0x04
  * @return     None.
  */
void SPIM_Hyper_DefaultConfig(SPIM_T *spim, uint32_t u32CSM, uint32_t u32AcctRD, uint32_t u32AcctWR)
{
    uint32_t u32CoreFreq = (CLK_GetSCLKFreq() / 1000000);
    float fFreq = (float)((float)1000 / (float)u32CoreFreq);
    uint32_t u32DIV = SPIM_HYPER_GET_CLKDIV(spim);
    uint32_t u32CipherEn = SPIM_HYPER_GET_CIPHER(spim);
    uint32_t u32CSMAXLT = (uint32_t)((u32CSM / fFreq) -
                                     (2 * 8 * u32DIV) -
                                     (((!u32CipherEn) == SPIM_HYPER_OP_ENABLE) ? CSMAXLT_CIPHER_ON : CSMAXLT_CIPHER_OFF));

    /* Chip Select Setup Time 3.5 HCLK */
    SPIM_HYPER_SET_CSST(spim, SPIM_HYPER_CSST_3_5_HCLK);

    /* Chip Select Hold Time 3.5 HCLK */
    SPIM_HYPER_SET_CSH(spim, SPIM_HYPER_CSH_3_5_HCLK);

    /* Chip Select High between Transaction as 2 HCLK cycles */
    SPIM_HYPER_SET_CSHI(spim, HYPERRAM_CSHI_CYCLE);

    /* Chip Select Masximum low time HCLK */
    SPIM_HYPER_SET_CSMAXLT(spim, u32CSMAXLT);

    /* Initial Device RESETN Low Time 255 */
    SPIM_HYPER_SET_RSTNLT(spim, HYPER_RAM_RST_CNT);

    /* Initial Read Access Time Clock cycle*/
    SPIM_HYPER_SET_ACCTRD(spim, u32AcctRD);

    /* Initial Write Access Time Clock cycle*/
    SPIM_HYPER_SET_ACCTWR(spim, u32AcctWR);
}

/**
 * @brief    Erase HyperRAM
 *
 * @param    spim         SPIM device pointer
 * @param    u32StartAddr Erase start address
 * @param    u32EraseSize Erase size
 *
 * @return   None
 *
 * @note     This function is used to erase HyperRAM block
 */
void HyperRAM_Erase(SPIM_T *spim, uint32_t u32StartAddr, uint32_t u32EraseSize)
{
    uint32_t u32i;

    /* Erase Hyper RAM */
    for (u32i = 0; u32i < u32EraseSize; u32i += 2)
    {
        /* Erase Hyper RAM block */
        SPIM_HYPER_Write2Byte(spim, (u32StartAddr + u32i), VERIFY_ERASE_PATTERN);

        /* Read back check and erase fail */
        if (SPIM_HYPER_Read1Word(spim, u32StartAddr + u32i) != VERIFY_ERASE_PATTERN)
        {
            printf("Erase Hyper RAM fail!!\n");

            while (1);
        }
    }

    /* Check remain 1 byte */
    if (u32EraseSize % 2)
    {
        /* Erase remain 1 byte */
        SPIM_HYPER_Write1Byte(spim, (u32StartAddr + u32EraseSize - 1), 0x00);

        /* Read back check and erase fail */
        if ((SPIM_HYPER_Read1Word(spim, (u32StartAddr + u32EraseSize - 1)) >> 8) & 0xFF)
        {
            printf("Erase Remain HyperRAM fail, Read Data = %x !!\n",
                   (SPIM_HYPER_Read1Word(spim, (u32StartAddr + u32EraseSize - 1)) >> 8));

            while (1);
        }
    }
}

/**
 * @brief Invalidate DCache for HyperRAM address range.
 *
 * @param u32Addr Start address of the range to invalidate.
 * @param u32Size Size of the range to invalidate.
 */
static void HyperRAM_InvalidateDCacheByAddr(uint32_t u32Addr, uint32_t u32Size)
{
#if (NVT_DCACHE_ON == 1)
    uint32_t u32AlignedAddr = u32Addr & ~31UL;                                // align to 32 bytes
    uint32_t u32Offset = u32Addr - u32AlignedAddr;
    uint32_t u32AlignedSize = (u32Size + u32Offset + 31) & ~31UL;                // round up to multiple of 32

    SCB_InvalidateDCache_by_Addr((uint32_t *)u32AlignedAddr, (int32_t)u32AlignedSize);
#else
    (void)addr;
    (void)size;
#endif
}

/**
 * @brief Safely reads a 64-bit value from the specified address.
 *
 * This function performs a read operation from the given memory address
 * and stores the result in the provided pointer. It ensures that the read
 * operation is safe and handles any necessary checks or conditions.
 *
 * @param addr The address from which to read the 64-bit value.
 * @param pData Pointer to a variable where the read value will be stored.
 * @return Returns 0 on success, or a negative error code on failure.
 */
__STATIC_INLINE int SafeDMMRead64(uint32_t addr, uint64_t *pData)
{
    if (!pData || addr == 0)
        return -1;

    __IO uint64_t *pSrc = (__IO uint64_t *)addr;
    *pData = *pSrc;
    return 0;
}

/**
 * @brief Verifies the final read from the SPIM memory.
 *
 * This function compares the data read from the specified DMM address
 * with the expected data to ensure that the read operation was successful.
 *
 * @param spim Pointer to the SPIM_T structure that contains the configuration
 *             and state of the SPIM peripheral.
 * @param dmmAddr The address in the DMM (Dynamic Memory Map) from which
 *                the data is read.
 * @param expected Pointer to the expected data buffer that will be compared
 *                 against the data read from the DMM address.
 * @param size The number of bytes to compare between the expected data and
 *             the data read from the DMM address.
 *
 * @return Returns 0 if the read data matches the expected data, otherwise
 *         returns a non-zero value indicating a mismatch.
 */
static int VerifyFinalRead(SPIM_T *spim, uint32_t dmmAddr, uint8_t *expected, uint32_t size)
{
    SPIM_HYPER_EnterDirectMapMode(spim);

    for (uint32_t i = 0; i + 8 <= size; i += 8)
    {
        uint64_t val = 0;

        if (SafeDMMRead64(dmmAddr + i, &val) != 0 || memcmp(&expected[i], &val, 8) != 0)
        {
            SPIM_HYPER_ExitDirectMapMode(spim);
            return 0;
        }
    }

    SPIM_HYPER_ExitDirectMapMode(spim);
    return 1;
}

/**
 * @brief Generates a Pseudo-Random Binary Sequence (PRBS) of length 7.
 *
 * This function fills the provided buffer with a PRBS7 pattern. The PRBS7
 * is a sequence of bits that can be used for testing and validation of
 * communication systems and memory interfaces.
 *
 * @param buf Pointer to the buffer where the PRBS7 pattern will be stored.
 * @param size The size of the buffer. It should be at least 7 bytes to
 *             accommodate the PRBS7 pattern.
 *
 * @note The function does not check if the provided size is sufficient.
 *       Ensure that the buffer is large enough to hold the generated pattern.
 */
static void GenPRBS7Pattern(uint8_t *buf, uint32_t size)
{
    uint8_t prbs = 0x5A;

    for (uint32_t i = 0; i < size; i++)
    {
        buf[i] = prbs;
        prbs = (prbs >> 1) ^ ((prbs & 1) ? 0xB8 : 0x00);
    }
}

/**
 * @brief Generates a walking 1's pattern in the provided buffer.
 *
 * This function fills the specified buffer with a pattern where only one bit is set to 1
 * at a time, creating a "walking" effect. The pattern starts with the least significant bit
 * and moves towards the most significant bit, wrapping around when the end of the buffer is reached.
 *
 * @param buf Pointer to the buffer where the pattern will be generated.
 * @param size The size of the buffer in bytes.
 */
static void GenWalking1sPattern(uint8_t *buf, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        buf[i] = (1 << (i % 8));
    }
}

/**
 * @brief Generates the original pattern for the given buffer.
 *
 * This function fills the provided buffer with a specific pattern
 * based on the size specified. The pattern generation logic is
 * defined within the function implementation.
 *
 * @param buf Pointer to the buffer where the pattern will be stored.
 * @param size The size of the buffer, which determines how much
 *             data will be filled with the pattern.
 */
static void GenOriginalPattern(uint8_t *buf, uint32_t size)
{
    uint32_t val;

    for (uint32_t k = 0; k < size; k++)
    {
        val = (k & 0x0F) ^ (k >> 4) ^ (k >> 3);

        if (k & 0x01) val = ~val;

        buf[k] = ~(uint8_t)(val ^ (k << 3) ^ (k >> 2));
    }
}

/**
 * @brief Trims the DLL delay number for the specified SPIM instance.
 *
 * This function adjusts the delay number of the DLL (Delay Locked Loop)
 * for the HyperRAM interface to optimize performance and ensure stable
 * communication. It is essential to call this function during the initialization
 * phase of the SPIM configuration.
 *
 * @param spim Pointer to the SPIM_T structure that contains the configuration
 *             and state of the SPIM instance.
 */
void HyperRAM_TrimDLLDelayNumber(SPIM_T *spim)
{
    uint8_t u8RdDelay = 0;
    uint16_t u16Score[SPIM_HYPER_MAX_LATENCY] = {0};
    uint32_t u32SrcAddr = HRAM_TRIM_SAFE_OFFSET;
    uint32_t u32ReTrimMaxCnt = DLL_TRIM_MAX_RETRY;
    uint32_t u32DMMAddr = SPIM_HYPER_GET_DMMADDR(spim);
    /*
        SPIM DMA requires memory buffers to be 8-byte aligned.
        HRAM_TRIM_SIZE is in bytes and must be divisible by 8.
    */
    uint64_t au64TrimPattern[HRAM_TRIM_SIZE / 8] = {0};
    uint64_t au64VerfiyBuf[HRAM_TRIM_SIZE / 8] = {0};
    uint8_t *pu8TrimPattern = (uint8_t *)au64TrimPattern;
    uint8_t *pu8VerfiyBuf = (uint8_t *)au64VerfiyBuf;
    uint32_t u32Retry = 0, u32PatIdx = 0, u32Pass = 0, u32i = 0;

    void (*patternGenerators[])(uint8_t *, uint32_t) =
    {
        GenPRBS7Pattern, GenWalking1sPattern, GenOriginalPattern
    };
    const uint32_t u32NumPatterns = sizeof(patternGenerators) / sizeof(patternGenerators[0]);

    for (u32PatIdx = 0; u32PatIdx < u32NumPatterns; u32PatIdx++)
    {
        patternGenerators[u32PatIdx](pu8TrimPattern, HRAM_TRIM_SIZE);
        SPIM_HYPER_DMAWrite(spim, u32SrcAddr, pu8TrimPattern, HRAM_TRIM_SIZE);

        for (u32Retry = 0; u32Retry < u32ReTrimMaxCnt; u32Retry++)
        {
            for (u8RdDelay = 0; u8RdDelay < SPIM_HYPER_MAX_LATENCY; u8RdDelay++)
            {
                SPIM_HYPER_SetDLLDelayNum(spim, u8RdDelay);
                memset(pu8VerfiyBuf, 0, HRAM_TRIM_SIZE);

                HyperRAM_InvalidateDCacheByAddr((u32DMMAddr + u32SrcAddr), HRAM_TRIM_SIZE);

                for (u32Pass = 0; u32Pass < DLL_TRIM_PASS_COUNT; u32Pass++)
                {
                    uint32_t u32LoopAddr = u32Pass * 0x100;

                    for (u32i = 0; u32i + 8 <= HRAM_TRIM_SIZE;)
                    {
                        if ((u32LoopAddr + 8 > HRAM_TRIM_SIZE)) break;

                        SPIM_HYPER_DMARead(spim, u32SrcAddr + u32LoopAddr, &pu8VerfiyBuf[u32i], 8);

                        if (memcmp(&pu8TrimPattern[u32LoopAddr], &pu8VerfiyBuf[u32i], 8) == 0)
                        {
                            u16Score[u8RdDelay] += DLL_TRIM_SCORE_PER_BLOCK;
                        }

                        u32LoopAddr += ((u32i % 3) == 0) ? DLL_TRIM_JUMP_STEP_1 : DLL_TRIM_JUMP_STEP_2;
                        u32i += 8;
                    }
                }
            }
        }
    }

    HRAM_DBGMSG("DLL Delay Score Map:\r\n");

    for (u32i = 0; u32i < SPIM_HYPER_MAX_LATENCY; u32i++)
    {
        HRAM_DBGMSG("[%02d]=%03d%s", u32i, u16Score[u32i], (u32i % 4 == 3) ? "\r\n" : " ");
    }

    uint16_t maxScore = 0;

    for (u32i = 0; u32i < SPIM_HYPER_MAX_LATENCY; u32i++)
    {
        if (u16Score[u32i] > maxScore)
            maxScore = u16Score[u32i];
    }

    uint8_t bestIdxStart = 0, bestIdxLen = 0, curLen = 0, curStart = 0;

    for (u32i = 0; u32i < SPIM_HYPER_MAX_LATENCY; u32i++)
    {
        if (u16Score[u32i] == maxScore)
        {
            if (curLen == 0) curStart = u32i;

            curLen++;
        }
        else
        {
            if (curLen > bestIdxLen)
            {
                bestIdxLen = curLen;
                bestIdxStart = curStart;
            }

            curLen = 0;
        }
    }

    if (curLen > bestIdxLen)
    {
        bestIdxLen = curLen;
        bestIdxStart = curStart;
    }

    // Phase 2: DMM verify max-score range with retry, store valid delays
    uint8_t verifiedList[SPIM_HYPER_MAX_LATENCY] = {0};
    uint8_t verifiedCount = 0;

    for (u32i = bestIdxStart; u32i < bestIdxStart + bestIdxLen; u32i++)
    {
        if (SPIM_HYPER_SetDLLDelayNum(spim, u32i) != SPIM_HYPER_OK)
            continue;

        if (SPIM_HYPER_GET_DLLREADY(spim) != SPIM_HYPER_OP_ENABLE)
            continue;

        for (int retry = 0; retry < DMM_VERIFY_MAX_RETRY; retry++)
        {
            if (VerifyFinalRead(spim, u32DMMAddr + u32SrcAddr, pu8TrimPattern, HRAM_TRIM_SIZE))
            {
                verifiedList[verifiedCount++] = u32i;
                break;
            }
        }
    }

    if (verifiedCount > 0)
    {
        u8RdDelay = verifiedList[verifiedCount / 2];
        HRAM_DBGMSG("DLL Delay Verified from DMM: %d (mid of %d passes)\r\n", u8RdDelay, verifiedCount);
    }
    else
    {
        u8RdDelay = bestIdxStart + (bestIdxLen / 2);
        HRAM_DBGMSG("DMM Verify Failed. Fallback to mid of best region: %d\r\n", u8RdDelay);
    }

    // Backup fallback retry: u8RdDelay, u8RdDelay-1, u8RdDelay-2
    const uint8_t backupTry[] = { u8RdDelay, (uint8_t)(u8RdDelay - 1), (uint8_t)(u8RdDelay - 2) };

    for (u32i = 0; u32i < sizeof(backupTry); u32i++)
    {
        if (SPIM_HYPER_SetDLLDelayNum(spim, backupTry[u32i]) != SPIM_HYPER_OK)
            continue;

        if (SPIM_HYPER_GET_DLLREADY(spim) != SPIM_HYPER_OP_ENABLE)
            continue;

        for (int retry = 0; retry < DMM_VERIFY_MAX_RETRY; retry++)
        {
            if (VerifyFinalRead(spim, u32DMMAddr + u32SrcAddr, pu8TrimPattern, HRAM_TRIM_SIZE))
            {
                u8RdDelay = backupTry[u32i];
                HRAM_DBGMSG("DLL Delay Backup Applied (verified): %d \r\n", u8RdDelay);
                SPIM_HYPER_SetDLLDelayNum(spim, u8RdDelay);
                return;
            }
        }

        HRAM_DBGMSG("Backup delay %d verify failed.\r\n", backupTry[u32i]);
    }

    HRAM_DBGMSG("WARNING: DLL Delay fallback failed. Apply default delay DLL_TRIM_DEF_NUM\r\n");
    SPIM_HYPER_SetDLLDelayNum(spim, DLL_TRIM_DEF_NUM);
}

void HyperRAM_Init(SPIM_T *spim)
{
    HRAM_REG_T sHRAMReg;

    /* Enable SPIM Hyper Bus Mode */
    SPIM_HYPER_Init(spim, SPIM_HYPERRAM_MODE, SPIM_HYPER_DIV);

    /* Set R/W Latency Number */
    SPIM_Hyper_DefaultConfig(spim, HYPERRAM_CSM_TIME, HYPERRAM_RD_LTCY, HYPERRAM_WR_LTCY);

    /* Reset HyperRAM */
    SPIM_HYPER_Reset(spim);

    /* Trim DLL component delay stop number */
    HyperRAM_TrimDLLDelayNumber(spim);

    /* Read HyperRAM Configuration Register 0 */
    sHRAMReg.CONFIG0.u32REG = SPIM_HYPER_ReadHyperRAMReg(spim, SPIM_HYPER_HRAM_CONFIG_REG0);

    /* Set Drive Strength to 34ohms by default (u3DriveStrength = 0) */
    /* This default setting is based on Infineon S27KS0642 and Winbond W958D8NBYA. */
    /*
       If you encounter issues such as signal instability, timing errors, or read/write failures during testing,
       you may refer to the HyperRAM datasheet and adjust this setting accordingly.

       - u3DriveStrength defines the I/O output driver impedance.
       - Valid range: 0 ~ 7, each value corresponds to a specific drive strength (e.g., 34ohms, 46ohms, 67ohms... depending on the vendor).
       - Adjust this value to improve signal integrity based on layout, trace length, and memory vendor.
    */
    sHRAMReg.CONFIG0.u3DriveStrength = 0;

    /* Write the updated value back to HyperRAM Configuration Register 0 */
    SPIM_HYPER_WriteHyperRAMReg(spim, SPIM_HYPER_HRAM_CONFIG_REG0, sHRAMReg.CONFIG0.u32REG);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
