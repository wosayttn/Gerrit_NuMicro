/**************************************************************************//**
 * @file     prng.c
 * @version  V1.00
 * @brief    M471 series PNRG driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup PRNG_Driver PRNG Driver
  @{
*/

/** @addtogroup PRNG_EXPORTED_FUNCTIONS PRNG Exported Functions
  @{
*/

/**
  * @brief  Open PRNG function
  * @param[in]  prng   Reference to PRNG module.
  * @param[in]  u32KeySize is PRNG key size, including:
  *         - \ref PRNG_KEY_SIZE_64
  *         - \ref PRNG_KEY_SIZE_128
  *         - \ref PRNG_KEY_SIZE_192
  *         - \ref PRNG_KEY_SIZE_256
  * @param[in]  u32SeedReload is PRNG seed reload or not, including:
  *         - \ref PRNG_SEED_CONT
  *         - \ref PRNG_SEED_RELOAD
  * @param[in]  u32Seed  The new seed. Only valid when u32SeedReload is PRNG_SEED_RELOAD.
  * @return None
  */
void PRNG_Open(PRNG_T *prng, uint32_t u32KeySize, uint32_t u32SeedReload, uint32_t u32Seed)
{
    if (u32SeedReload)
    {
        prng->SEED = u32Seed;
    }

    prng->CTL =  (u32KeySize << PRNG_CTL_KEYSZ_Pos) |
                 (u32SeedReload << PRNG_CTL_SEEDRLD_Pos);
}

/**
  * @brief  Start to generate one PRNG key.
  * @param[in]  prng   Reference to PRNG module.
  * @return None
  */
void PRNG_Start(PRNG_T *prng)
{
    prng->CTL |= PRNG_CTL_START_Msk;
}

/**
  * @brief  Read the PRNG key.
  * @param[in]   prng        Reference to PRNG module.
  * @param[out]  u32RandKey  The key buffer to store newly generated PRNG key.
  * @return None
  */
void PRNG_Read(PRNG_T *prng, uint32_t u32RandKey[])
{
    uint32_t  i, wcnt;

    wcnt = (((prng->CTL & PRNG_CTL_KEYSZ_Msk) >> PRNG_CTL_KEYSZ_Pos) + 1U) * 2U;

    for (i = 0U; i < wcnt; i++)
    {
        u32RandKey[i] = prng->KEY[i];
    }

    prng->CTL &= ~PRNG_CTL_SEEDRLD_Msk;
}

/*@}*/ /* end of group PRNG_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PRNG_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

