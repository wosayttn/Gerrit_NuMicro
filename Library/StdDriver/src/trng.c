/**************************************************************************//**
 * @file     trng.c
 * @version  V3.00
 * @brief    M2L31 series TRNG driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup TRNG_Driver TRNG Driver
  @{
*/

/** @addtogroup TRNG_EXPORTED_FUNCTIONS TRNG Exported Functions
  @{
*/

/**
  * @brief Initialize TRNG hardware.
  * @return  TRNG hardware enable success or failed.
  * @retval  0   Success
  * @retval  -1  Time-out. TRNG hardware may not be enabled.
  */
int32_t TRNG_Open(void)
{
    uint32_t u32TimeOutCount;

    /* Reset TRNG */
    SYS->IPRST1 |= SYS_IPRST1_TRNGRST_Msk;
    SYS->IPRST1 ^= SYS_IPRST1_TRNGRST_Msk;

    /* Enable TRNG Power */
    TRNG->CTL = TRNG_CTL_LDOEN_Msk;

    /* Waiting for ready */
    u32TimeOutCount = SystemCoreClock;
    while ((TRNG->STS & TRNG_STS_LDORDY_Msk) == 0UL)
    {
        if (--u32TimeOutCount == 0UL)
        {
            return -1; /* Time-out error */
        }
    }

    /* Eanble TRNG and release reset */
    TRNG->CTL = (TRNG_CTL_TRNGEN_Msk | TRNG_CTL_LDOEN_Msk | TRNG_CTL_NRST_Msk);

    /* Waiting for ready */
    u32TimeOutCount = SystemCoreClock;
    while ((TRNG->STS & TRNG_STS_TRNGRDY_Msk) == 0UL)
    {
        if (--u32TimeOutCount == 0UL)
        {
            return -1; /* Time-out error */
        }
    }

    TRNG->CTL = TRNG_CTL_TRNGEN_Msk | TRNG_CTL_LDOEN_Msk | TRNG_CTL_NRST_Msk | TRNG_CTL_INSTANT_Msk |
                TRNG_CTL_RESEED_Msk | TRNG_CTL_UPDATE_Msk | TRNG_CTL_START_Msk;

    /* Waiting for DVIF */
    u32TimeOutCount = SystemCoreClock;
    while ((TRNG->STS & TRNG_STS_DVIF_Msk) == 0UL)
    {
        if (--u32TimeOutCount == 0UL)
        {
            return -1; /* Time-out error */
        }
    }

    return 0;
}

/**
  * @brief   Generate a 32-bits random number word.
  * @param[out]  u32RndNum    The output 32-bits word random number.
  *
  * @return  Success or time-out.
  * @retval  0   Success
  * @retval  -1  Time-out. TRNG hardware may not be enabled.
  */
int32_t TRNG_GenWord(uint32_t *u32RndNum)
{
    uint32_t u32TimeOutCount;

    *u32RndNum = 0U;

    TRNG->CTL |= TRNG_CTL_START_Msk;

    for(u32TimeOutCount = SystemCoreClock; u32TimeOutCount > 0U; u32TimeOutCount--)
    {
        if (TRNG->STS & TRNG_STS_DVIF_Msk)
        {
            break;
        }
    }

    if (u32TimeOutCount == 0U)
    {
        return -1;
    }

    *u32RndNum = TRNG->DATA[0];

    return 0;
}

/**
  * @brief   Generate a big number in binary format.
  * @param[out]  u8BigNum  The output big number.
  * @param[in]   i32Len    Request bit length of the output big number. It must be multiple of 8.
  *
  * @return  Success or time-out.
  * @retval  0   Success
  * @retval  -1  Time-out. TRNG hardware may not be enabled.
  */
int32_t TRNG_GenBignum(uint8_t u8BigNum[], int32_t i32Len)
{
    uint32_t i;
    uint32_t j;
    uint32_t u32Reg;
    uint32_t u32TimeOutCount;
    uint32_t u32ByteLen;

    u32ByteLen = (uint32_t)i32Len / 8UL;

    for (i = 0U; i < u32ByteLen; i++)
    {
        /* Get 32 random bits */
        if((i & 0x3U) == 0U)
        {
            TRNG->CTL |= TRNG_CTL_START_Msk;

            /* Return fail when timeout */
            for(u32TimeOutCount = (CLK_GetHCLKFreq() / 100U); u32TimeOutCount > 0U; u32TimeOutCount--)
            {
                if(TRNG->STS & TRNG_STS_DVIF_Msk)
                {
                    break;
                }
            }

            if(u32TimeOutCount == 0U)
            {
                return -1;
            }

            u32Reg = TRNG->DATA[0];
            j = 0U;
        }

        u8BigNum[i] = (uint8_t)((u32Reg >> (j * 8U)) & 0xFFU);
        j++;
    }

    return 0;
}

/**
  * @brief   Generate a big number in hex format.
  * @param[out]  cBigNumHex  The output hex format big number.
  * @param[in]   i32Len      Request bit length of the output big number. It must be multiple of 8.
  *
  * @return  Success or time-out.
  * @retval  0   Success
  * @retval  -1  Time-out. TRNG hardware may not be enabled.
  */
int32_t TRNG_GenBignumHex(char cBigNumHex[], int32_t i32Len)
{
    uint32_t i;
    uint32_t j;
    uint32_t u32Reg;
    uint32_t u32TimeOutCount;
    uint32_t u32ByteLen;

    u32ByteLen = (uint32_t)i32Len / 8UL;

    for(i = 0U; i < u32ByteLen; i++)
    {
        /* Get 32 random bits */
        if((i & 0x3U) == 0U)
        {
            TRNG->CTL |= TRNG_CTL_START_Msk;

            /* Return fail when timeout */
            for(u32TimeOutCount = (CLK_GetHCLKFreq() / 100U); u32TimeOutCount > 0U; u32TimeOutCount--)
            {
                if(TRNG->STS & TRNG_STS_DVIF_Msk)
                {
                    break;
                }
            }

            if(u32TimeOutCount == 0U)
            {
                return -1;
            }

            u32Reg = TRNG->DATA[0];
            j = 0U;
        }

        uint32_t u32HighCh;
        uint32_t u32LowCh;

        u32HighCh = ((u32Reg >> (j * 8U)) & 0xF0U) >> 4;
        u32LowCh = (u32Reg >> (j * 8U)) & 0x0FU;
        j++;

        if(u32HighCh >= 0xaU)
        {
            cBigNumHex[i * 2U] = u32HighCh - 10U + 'a';
        }
        else
        {
            cBigNumHex[i * 2U] = u32HighCh + '0';
        }

        if(u32LowCh >= 0xaU)
        {
            cBigNumHex[(i * 2U) + 1U] = u32LowCh - 10U + 'a';
        }
        else
        {
            cBigNumHex[(i * 2U) + 1U] = u32LowCh + '0';
        }

    }

    return 0;
}

/*@}*/ /* end of group TRNG_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group TRNG_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

