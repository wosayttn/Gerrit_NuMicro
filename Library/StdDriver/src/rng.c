/**************************************************************************//**
 * @file     rng.c
 * @version  V3.00
 * @brief    Show how to get true random number.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup RNG_Driver RNG Driver
  @{
*/

/** @addtogroup RNG_EXPORTED_FUNCTIONS RNG Exported Functions
  @{
*/

/**
 *  @brief      Open random number generator
 *
 *  @retval      0  Successful
 *  @retval     -1  Failed
 *
 *  @details    The function is used to disable rng interrupt.
 */
int32_t RNG_Open(void)
{
    int32_t i;
    int32_t timeout = 0x1000000;

    /* TRNG LDO Enable */
    TRNG->CTL |= TRNG_CTL_LDOEN_Msk;

    /* Waiting for ready */
    i = 0;
    while((TRNG->STS & (TRNG_STS_LDORDY_Msk)) != (TRNG_STS_LDORDY_Msk))
    {
        if(i++ > timeout)
        {
            /* LDO ready timeout */
            return -1;
        }
    }

    /* TRNG Enable */
    TRNG->CTL |= TRNG_CTL_NRST_Msk | TRNG_CTL_TRNGEN_Msk;

    /* Waiting for ready */
    i = 0;
    while((TRNG->STS & (TRNG_STS_TRNGRDY_Msk)) != (TRNG_STS_TRNGRDY_Msk))
    {
        if(i++ > timeout)
        {
            /* TRNG ready timeout */
            return -1;
        }
    }

    return 0;
}


/**
 *  @brief      Get random words
 *
 *  @param[in]  pu32Buf Buffer pointer to store the random number
 *
 *  @param[in]  i32WordCnt  Buffer size in word count. i32WordCnt must <= 8
 *
 *  @return     Word count of random number in buffer
 *
 *  @details    The function is used to generate random numbers
 */
int32_t RNG_Random(uint32_t *pu32Buf, int32_t i32WordCnt)
{
    int32_t  i32Idx = 0;
    int32_t  i32Remaining;
    uint8_t  u8Err = 0U;

    i32Remaining = i32WordCnt;

    while ((i32Remaining > 0) && (u8Err == 0U))
    {
        int32_t  i32Timeout = 0x10000L; /* Signed timeout counter */

        /* Start DRBG */
        TRNG->CTL |= (TRNG_CTL_MODE_DRBG | TRNG_CTL_START_Msk);

        /* Waiting for data valid flag (DVIF) */
        while (((TRNG->STS & TRNG_STS_DVIF_Msk) == 0UL) && (i32Timeout > 0L))
        {
            i32Timeout--;
        }

        if (i32Timeout == 0L)
        {
            u8Err = 1U;
        }
        else
        {
            uint32_t u32Word;

            for (u32Word = 0UL; (u32Word < 4UL) && (i32Remaining > 0); u32Word++)
            {
                pu32Buf[i32Idx] = TRNG->DATA[u32Word];
                i32Idx++;
                i32Remaining--;
            }
        }
    }

    if (u8Err != 0U)
    {
        return 0L;
    }

    return i32Idx;
}

/**
 *  @brief      To generate entropy from hardware entropy source (TRNG)
 *
 *  @param[in]  pu8Out     Output buffer for the entropy
 *  @param[in]  i32Len     Entropy length in bytes
 *  @retval     -1       Failed
 *  @retval     Others   The bytes in pu8Out buffer
 *
 *  @details    The function is used to generate entropy from TRNG.
 */
int32_t RNG_EntropyPoll(uint8_t* pu8Out, int32_t i32Len)
{
    uint32_t u32Remaining;
    uint32_t u32Written;
    uint32_t u32ByteIdx;

    u32Remaining = (uint32_t)i32Len;
    u32Written   = 0UL;

    if ((TRNG->STS & (TRNG_STS_LDORDY_Msk | TRNG_STS_TRNGRDY_Msk)) !=
            (TRNG_STS_LDORDY_Msk | TRNG_STS_TRNGRDY_Msk))
    {
        return -1L;
    }

    while (u32Remaining > 0UL)
    {
        uint32_t u32Entropy;
        uint32_t u32Timeout;

        /* Trigger entropy generation */
        TRNG->CTL |= TRNG_CTL_START_Msk;

        /* Wait for data valid flag */
        u32Timeout = (uint32_t)SystemCoreClock;
        while ((TRNG->STS & TRNG_STS_DVIF_Msk) == 0UL)
        {
            if (u32Timeout == 0UL)
            {
                return -1L;
            }

            u32Timeout--;
        }

        /* Read one 32-bit entropy word */
        u32Entropy = TRNG->DATA[0];

        /* Extract up to 4 bytes from the entropy word */
        for (u32ByteIdx = 0UL; (u32ByteIdx < 4UL) && (u32Remaining > 0UL); u32ByteIdx++)
        {
            uint32_t u32Shift;
            u32Shift = (u32ByteIdx * 8UL);
            pu8Out[u32Written] = (uint8_t)((u32Entropy >> u32Shift) & 0xFFUL);

            u32Written++;
            u32Remaining--;
        }
    }

    return i32Len;
}

/**@}*/ /* end of group RNG_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group RNG_Driver */

/**@}*/ /* end of group Standard_Driver */

