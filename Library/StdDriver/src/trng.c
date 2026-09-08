/**************************************************************************//**
 * @file     trng.c
 * @version  V1.00
 * @brief    TRNG driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
/** @cond HIDDEN_SYMBOLS */
#if defined(ENABLE_DEBUG) && (ENABLE_DEBUG)
    #define TRNG_DBGMSG   printf
#else
    #define TRNG_DBGMSG(...)   do { } while (0)       /* disable debug */
#endif
/** @endcond HIDDEN_SYMBOLS */
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
  * @return  Success or initialize fail.
  * @retval  0   Success
  * @retval  -1  Initialize fail. TRNG macro may not work.
  */
int32_t TRNG_Open(void)
{
    uint32_t i;
    SYS_ResetModule(TRNG_RST);

    TRNG->CTL |= TRNG_CTL_LDOEN_Msk;

    /* Waiting for ready */
    while ((TRNG->STS & TRNG_STS_LDORDY_Msk) == 0UL)
    {
        TRNG_DBGMSG("Waiting for ready\n");
    };

    TRNG->CTL &= ~TRNG_CTL_NRST_Msk;

    CLK_SysTickDelay(1);

    TRNG->CTL |= (TRNG_CTL_NRST_Msk);

    TRNG->CTL |= (TRNG_CTL_TRNGEN_Msk);

    TRNG_DBGMSG("TRNG->STS0 0x%x \n", TRNG->STS);

    /* Waiting for ready */
    while ((TRNG->STS & TRNG_STS_TRNGRDY_Msk) == 0UL)
    {
    }

    for (i = 0UL; i < 3UL; i++)
    {
        TRNG_DBGMSG("TRNG->STS: loop%d  0x%x \n", i, TRNG->STS);
    }

    if ((TRNG->STS & (TRNG_STS_ESAPT_Msk | TRNG_STS_ESRCT_Msk | TRNG_STS_ESSUT_Msk)) \
            != (TRNG_STS_ESAPT_Msk | TRNG_STS_ESRCT_Msk | TRNG_STS_ESSUT_Msk))
    {
        TRNG_DBGMSG("Entropy source test fail!\n");
        return -1;
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
    uint32_t i;
    uint32_t u32Reg;
    uint32_t timeout;

    *u32RndNum = 0;
    u32Reg = TRNG->CTL;

    for (i = 0UL; i < 4UL; i++)
    {
        TRNG->CTL = TRNG_CTL_START_Msk | u32Reg;

        /* TRNG should generate one byte per 125*8 us */
        for (timeout = (CLK_GetHCLKFreq() / 100UL); timeout > 0UL; timeout--)
        {
            if ((TRNG->STS & TRNG_STS_DVIF_Msk) != 0UL)
            {
                break;
            }

            if ((TRNG->STS & TRNG_STS_ERRIF_Msk) != 0UL)
            {
                return -1;
            }

        }

        if (timeout == 0UL)
        {
            return -1;
        }

        *u32RndNum |= ((TRNG->DATA_OUT[0] & 0xFFUL) << (i * 8UL));

    }

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
    uint32_t u32Reg;
    uint32_t timeout;
    int32_t i;
    int32_t i32ByteLen;

    u32Reg = TRNG->CTL;
    i32ByteLen = i32Len / 8;

    for (i = 0; i < i32ByteLen; i++)
    {
        TRNG->CTL = TRNG_CTL_START_Msk | u32Reg;

        /* TRNG should generate one byte per 125*8 us */
        for (timeout = (CLK_GetHCLKFreq() / 100UL); timeout > 0UL; timeout--)
        {
            if ((TRNG->STS & TRNG_STS_DVIF_Msk) != 0UL)
            {
                break;
            }

            if ((TRNG->STS & TRNG_STS_ERRIF_Msk) != 0UL)
            {
                return -1;
            }
        }

        if (timeout == 0UL)
        {
            return -1;
        }

        u8BigNum[i] = (uint8_t)(TRNG->DATA_OUT[0] & 0xFFUL);
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
    uint32_t idx;
    uint32_t u32Reg;
    uint32_t timeout;
    int32_t i;
    int32_t i32ByteLen;

    u32Reg = TRNG->CTL;
    idx = 0;
    i32ByteLen = i32Len / 8;

    for (i = 0; i < i32ByteLen; i++)
    {
        uint32_t data;
        uint32_t u32CharCode;
        uint32_t u32HighNibble;
        uint32_t u32LowNibble;

        TRNG->CTL = TRNG_CTL_START_Msk | u32Reg;

        /* TRNG should generate one byte per 125*8 us */
        for (timeout = (CLK_GetHCLKFreq() / 100UL); timeout > 0UL; timeout--)
        {
            if ((TRNG->STS & TRNG_STS_DVIF_Msk) != 0UL)
            {
                break;
            }

            if ((TRNG->STS & TRNG_STS_ERRIF_Msk) != 0UL)
            {
                return -1;
            }
        }

        if (timeout == 0UL)
        {
            return -1;
        }

        data = TRNG->DATA_OUT[0] & 0xFFUL;
        u32HighNibble = (data >> 4UL) & 0xFUL;
        u32LowNibble = data & 0xFUL;

        if (u32HighNibble >= 0xAUL)
        {
            u32CharCode = (u32HighNibble - 10UL) + (uint32_t)'A';
            cBigNumHex[idx] = (char)u32CharCode;
            idx++;
        }
        else
        {
            u32CharCode = u32HighNibble + (uint32_t)'0';
            cBigNumHex[idx] = (char)u32CharCode;
            idx++;
        }

        if (u32LowNibble >= 0xAUL)
        {
            u32CharCode = (u32LowNibble - 10UL) + (uint32_t)'A';
            cBigNumHex[idx] = (char)u32CharCode;
            idx++;
        }
        else
        {
            u32CharCode = u32LowNibble + (uint32_t)'0';
            cBigNumHex[idx] = (char)u32CharCode;
            idx++;
        }
    }

    cBigNumHex[idx] = '\0';
    return 0;
}

/** @} end of group TRNG_EXPORTED_FUNCTIONS */
/** @} end of group TRNG_Driver */
/** @} end of group Standard_Driver */
