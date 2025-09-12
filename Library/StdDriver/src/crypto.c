/**************************************************************************//**
 * @file     crypto.c
 * @version  V1.00
 * @brief    CRYPTO driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/** @cond HIDDEN_SYMBOLS */
#if ENABLE_DEBUG
    #define CRYPTO_DBGMSG   printf
#else
    #define CRYPTO_DBGMSG(...)   do { } while (0)       /* disable debug */
#endif
/** @endcond HIDDEN_SYMBOLS */


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CRYPTO_Driver CRYPTO Driver
  @{
*/

/** @cond HIDDEN_SYMBOLS */
static uint32_t g_AES_CTL[4];
static char  hex_char_tbl[] = "0123456789abcdef";
static void dump_ecc_reg(char *str, uint32_t volatile regs[], int32_t count);
static char get_Nth_nibble_char(uint32_t val32, uint32_t idx);
static char ch2hex(char ch);
static int  get_nibble_value(char c);
void  dump_buff_hex(uint8_t *pucBuff, int nBytes);
/** @endcond HIDDEN_SYMBOLS */



/** @addtogroup CRYPTO_EXPORTED_FUNCTIONS CRYPTO Exported Functions
  @{
*/
/**
  * @brief  Open PRNG function
  * @param[in]  crypto   Reference to Crypto module.
  * @param[in]  u32KeySize is PRNG key size, including:
  *         - \ref PRNG_KEY_SIZE_128
  *         - \ref PRNG_KEY_SIZE_163
  *         - \ref PRNG_KEY_SIZE_192
  *         - \ref PRNG_KEY_SIZE_224
  *         - \ref PRNG_KEY_SIZE_233
  *         - \ref PRNG_KEY_SIZE_255
  *         - \ref PRNG_KEY_SIZE_256
  *         - \ref PRNG_KEY_SIZE_283
  *         - \ref PRNG_KEY_SIZE_384
  *         - \ref PRNG_KEY_SIZE_409
  *         - \ref PRNG_KEY_SIZE_512
  *         - \ref PRNG_KEY_SIZE_521
  *         - \ref PRNG_KEY_SIZE_571
  * @param[in]  u32SeedReload is PRNG seed reload or not, including:
  *         - \ref PRNG_SEED_CONT
  *         - \ref PRNG_SEED_RELOAD
  * @param[in]  u32Seed  The new seed. Only valid when u32SeedReload is PRNG_SEED_RELOAD.
  * @return None
  */
void PRNG_Open(CRYPTO_T *crypto, uint32_t u32KeySize, uint32_t u32SeedReload, uint32_t u32Seed)
{
    if (u32SeedReload)
    {
        crypto->PRNG_SEED = u32Seed;
    }

    crypto->PRNG_CTL = (u32KeySize << CRYPTO_PRNG_CTL_KEYSZ_Pos) |
                       (u32SeedReload << CRYPTO_PRNG_CTL_SEEDRLD_Pos);
}

/**
  * @brief  Start to generate one PRNG key.
  * @param[in]  crypto   Reference to Crypto module.
  * @return None
  */
void PRNG_Start(CRYPTO_T *crypto)
{
    crypto->PRNG_CTL |= CRYPTO_PRNG_CTL_START_Msk;
}

/**
  * @brief  Read the PRNG key.
  * @param[in]   crypto        Reference to Crypto module.
  * @param[out]  u32RandKey  The key buffer to store newly generated PRNG key.
  * @return None
  */
void PRNG_Read(CRYPTO_T *crypto, uint32_t u32RandKey[])
{
    uint32_t  i, wcnt;

    wcnt = (((crypto->PRNG_CTL & CRYPTO_PRNG_CTL_KEYSZ_Msk) >> CRYPTO_PRNG_CTL_KEYSZ_Pos) + 1U) * 2U;

    for (i = 0U; i < wcnt; i++)
    {
        u32RandKey[i] = crypto->PRNG_KEY[i];
    }

    crypto->PRNG_CTL &= ~CRYPTO_PRNG_CTL_SEEDRLD_Msk;
}

/**
  * @brief  Open AES encrypt/decrypt function.
  * @param[in]  crypto         Reference to Crypto module.
  * @param[in]  u32Channel   AES channel. Must be 0~3.
  * @param[in]  u32EncDec    1: AES encode;  0: AES decode
  * @param[in]  u32OpMode    AES operation mode, including:
  *         - \ref AES_MODE_ECB
  *         - \ref AES_MODE_CBC
  *         - \ref AES_MODE_CFB
  *         - \ref AES_MODE_OFB
  *         - \ref AES_MODE_CTR
  *         - \ref AES_MODE_CBC_CS1
  *         - \ref AES_MODE_CBC_CS2
  *         - \ref AES_MODE_CBC_CS3
  * @param[in]  u32KeySize is AES key size, including:
  *         - \ref AES_KEY_SIZE_128
  *         - \ref AES_KEY_SIZE_192
  *         - \ref AES_KEY_SIZE_256
  * @param[in]  u32SwapType is AES input/output data swap control, including:
  *         - \ref AES_NO_SWAP
  *         - \ref AES_OUT_SWAP
  *         - \ref AES_IN_SWAP
  *         - \ref AES_IN_OUT_SWAP
  * @return None
  */
void AES_Open(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t u32EncDec,
              uint32_t u32OpMode, uint32_t u32KeySize, uint32_t u32SwapType)
{
    crypto->AES_CTL =  // (u32Channel << CRYPTO_AES_CTL_CHANNEL_Pos) |
        (u32EncDec << CRYPTO_AES_CTL_ENCRYPTO_Pos) |
        (u32OpMode << CRYPTO_AES_CTL_OPMODE_Pos) |
        (u32KeySize << CRYPTO_AES_CTL_KEYSZ_Pos) |
        (u32SwapType << CRYPTO_AES_CTL_OUTSWAP_Pos);
    g_AES_CTL[u32Channel] = crypto->AES_CTL;
}

/**
  * @brief  Start AES encrypt/decrypt
  * @param[in]  crypto        Reference to Crypto module.
  * @param[in]  u32Channel  AES channel. Must be 0~3.
  * @param[in]  u32DMAMode  AES DMA control, including:
  *         - \ref CRYPTO_DMA_ONE_SHOT   One shop AES encrypt/decrypt.
  *         - \ref CRYPTO_DMA_CONTINUE   Continuous AES encrypt/decrypt.
  *         - \ref CRYPTO_DMA_LAST       Last AES encrypt/decrypt of a series of AES_Start.
  * @return None
  */
void AES_Start(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t u32DMAMode)
{
    crypto->AES_CTL = g_AES_CTL[u32Channel];
    crypto->AES_CTL |= CRYPTO_AES_CTL_START_Msk | (u32DMAMode << CRYPTO_AES_CTL_DMALAST_Pos);
}

/**
  * @brief  Start AES encrypt/decrypt
  * @param[in]  crypto        Reference to Crypto module.
  * @param[in]  u32Channel  AES channel. Must be 0~3.
  * @param[in]  u32DMAMode  AES DMA control, including:
  *         - \ref CRYPTO_DMA_ONE_SHOT   One shop AES encrypt/decrypt.
  *         - \ref CRYPTO_DMA_CONTINUE   Continuous AES encrypt/decrypt.
  *         - \ref CRYPTO_DMA_LAST       Last AES encrypt/decrypt of a series of AES_Start.
  * @param[in]  ksel        0:           AES key is from Key Store SRAM
  *                         2:           AES key is from Key Store OTP
  * @param[in]  knum        Use Key Store OTP/SRAM key number "knum" as AES key
  * @return None
  */
void AES_Start_KS(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t u32DMAMode, int ksel, int knum)
{
    if (ksel == 0)
        crypto->AES_KSCTL = CRYPTO_AES_KSCTL_RSRC_Msk | knum;     /* from KS SRAM */
    else
        crypto->AES_KSCTL = (2 << CRYPTO_AES_KSCTL_RSSRC_Pos) | CRYPTO_AES_KSCTL_RSRC_Msk | knum; /* from KS OTP */

    crypto->AES_CTL = g_AES_CTL[u32Channel];
    crypto->AES_CTL |= CRYPTO_AES_CTL_START_Msk | (u32DMAMode << CRYPTO_AES_CTL_DMALAST_Pos);
}

/**
  * @brief  Set AES keys
  * @param[in]  crypto      Reference to Crypto module.
  * @param[in]  u32Channel  AES channel. Must be 0~3.
  * @param[in]  au32Keys    An word array contains AES keys.
  * @param[in]  u32KeySize is AES key size, including:
  *         - \ref AES_KEY_SIZE_128
  *         - \ref AES_KEY_SIZE_192
  *         - \ref AES_KEY_SIZE_256
  * @return None
  */
void AES_SetKey(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t au32Keys[], uint32_t u32KeySize)
{
    uint32_t  i, wcnt;
    void    *key_reg_addr;
    uint32_t *u32p_key_reg;

    key_reg_addr = (void *)((uint32_t)&crypto->AES_KEY[0] + (u32Channel * 0x3CUL));
    wcnt = 4UL + u32KeySize * 2UL;
    u32p_key_reg = (uint32_t *)key_reg_addr;

    for (i = 0U; i < wcnt; i++)
    {
        outpw(u32p_key_reg, au32Keys[i]);
        u32p_key_reg++;
    }
}



/**
  * @brief  Set AES initial vectors
  * @param[in]  crypto        Reference to Crypto module.
  * @param[in]  u32Channel    AES channel. Must be 0~3.
  * @param[in]  au32IV        A four entry word array contains AES initial vectors.
  * @return None
  */
void AES_SetInitVect(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t au32IV[])
{
    uint32_t  i;
    void   *iv_reg_addr;
    uint32_t *u32p_iv_reg;
    iv_reg_addr = (void *)((uint32_t)&crypto->AES_IV[0] + (u32Channel * 0x3CUL));
    u32p_iv_reg = (uint32_t *)iv_reg_addr;

    for (i = 0U; i < 4U; i++)
    {
        outpw(u32p_iv_reg, au32IV[i]);
        u32p_iv_reg++;
    }
}

/**
  * @brief  Set AES DMA transfer configuration.
  * @param[in]  crypto       Reference to Crypto module.
  * @param[in]  u32Channel   AES channel. Must be 0~3.
  * @param[in]  u32SrcAddr   AES DMA source address
  * @param[in]  u32DstAddr   AES DMA destination address
  * @param[in]  u32TransCnt  AES DMA transfer byte count
  * @return None
  */
void AES_SetDMATransfer(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t u32SrcAddr,
                        uint32_t u32DstAddr, uint32_t u32TransCnt)
{
    void *reg_addr;

    reg_addr = (void *)((uint32_t)&crypto->AES_SADDR + (u32Channel * 0x3CUL));
    outpw(reg_addr, u32SrcAddr);

    reg_addr = (void *)((uint32_t)&crypto->AES_DADDR + (u32Channel * 0x3CUL));
    outpw(reg_addr, u32DstAddr);

    reg_addr = (void *)((uint32_t)&crypto->AES_CNT + (u32Channel * 0x3CUL));
    outpw(reg_addr, u32TransCnt);
}


/**
  * @brief  Open SHA encrypt function.
  * @param[in]  crypto      Reference to Crypto module.
  * @param[in]  u32OpMode   SHA operation mode, including:
  *         - \ref SHA_MODE_SHA1
  *         - \ref SHA_MODE_SHA224
  *         - \ref SHA_MODE_SHA256
  *         - \ref SHA_MODE_SHA384
  *         - \ref SHA_MODE_SHA512
  * @param[in]  u32SwapType is SHA input/output data swap control, including:
  *         - \ref SHA_NO_SWAP
  *         - \ref SHA_OUT_SWAP
  *         - \ref SHA_IN_SWAP
  *         - \ref SHA_IN_OUT_SWAP
  * @param[in]  hmac_key_len   HMAC key byte count
  * @return None
  */
void SHA_Open(CRYPTO_T *crypto, uint32_t u32OpMode, uint32_t u32SwapType, uint32_t hmac_key_len)
{
    crypto->HMAC_CTL = (u32OpMode << CRYPTO_HMAC_CTL_OPMODE_Pos) |
                       (u32SwapType << CRYPTO_HMAC_CTL_OUTSWAP_Pos);

    if (hmac_key_len != 0UL)
    {
        crypto->HMAC_KEYCNT = hmac_key_len;
        crypto->HMAC_CTL |= (1 << 11); /* M55M1, migrate from M480LD HMACEN is CRYPTO_HMAC_CTL[11] */
    }
}

/**
  * @brief  Start SHA encrypt
  * @param[in]  crypto        Reference to Crypto module.
  * @param[in]  u32DMAMode    TDES DMA control, including:
  *         - \ref CRYPTO_DMA_ONE_SHOT   One shop SHA encrypt.
  *         - \ref CRYPTO_DMA_CONTINUE   Continuous SHA encrypt.
  *         - \ref CRYPTO_DMA_LAST       Last SHA encrypt of a series of SHA_Start.
  * @return None
  */
void SHA_Start(CRYPTO_T *crypto, uint32_t u32DMAMode)
{
    crypto->HMAC_CTL &= ~(0x7UL << CRYPTO_HMAC_CTL_DMALAST_Pos);
    crypto->HMAC_CTL |= CRYPTO_HMAC_CTL_START_Msk | (u32DMAMode << CRYPTO_HMAC_CTL_DMALAST_Pos);
}

/**
  * @brief  Set SHA DMA transfer
  * @param[in]  crypto       Reference to Crypto module.
  * @param[in]  u32SrcAddr   SHA DMA source address
  * @param[in]  u32TransCnt  SHA DMA transfer byte count
  * @return None
  */
void SHA_SetDMATransfer(CRYPTO_T *crypto, uint32_t u32SrcAddr, uint32_t u32TransCnt)
{
    crypto->HMAC_SADDR = u32SrcAddr;
    crypto->HMAC_DMACNT = u32TransCnt;
}

/**
  * @brief  Read the SHA digest.
  * @param[in]  crypto      Reference to Crypto module.
  * @param[out]  u32Digest  The SHA encrypt output digest.
  * @return None
  */
void SHA_Read(CRYPTO_T *crypto, uint32_t u32Digest[])
{
    uint32_t  i, wcnt;
    uint32_t  reg_addr;

    i = (crypto->HMAC_CTL & CRYPTO_HMAC_CTL_OPMODE_Msk) >> CRYPTO_HMAC_CTL_OPMODE_Pos;

    if (i == SHA_MODE_SHA1)
    {
        wcnt = 5UL;
    }
    else if (i == SHA_MODE_SHA224)
    {
        wcnt = 7UL;
    }
    else if (i == SHA_MODE_SHA256)
    {
        wcnt = 8UL;
    }
    else if (i == SHA_MODE_SHA384)
    {
        wcnt = 12UL;
    }
    else
    {
        /* SHA_MODE_SHA512 */
        wcnt = 16UL;
    }

    reg_addr = (uint32_t) & (crypto->HMAC_DGST[0]);

    for (i = 0UL; i < wcnt; i++)
    {
        u32Digest[i] = inpw(reg_addr);
        reg_addr += 4UL;
    }
}

/** @cond HIDDEN_SYMBOLS */



static char  ch2hex(char ch)
{
    if (ch <= '9')
    {
        ch = ch - '0';
    }
    else if ((ch <= 'z') && (ch >= 'a'))
    {
        ch = ch - 'a' + 10U;
    }
    else
    {
        ch = ch - 'A' + 10U;
    }

    return ch;
}


static int  __strlen(char *str)
{
    char  *p = str;
    int   len = 0;

    while (*p != 0)
    {
        p++;
        len++;

        if (len > 1024)   /* max. 4096 bits */
            break;
    }

    // printf("< %d >\n", len);
    return len;
}

void Hex2Reg(char input[], uint32_t volatile reg[])
{
    char      hex;
    int       si, ri;
    uint32_t  i, val32;

    si = (int)__strlen(input) - 1;
    ri = 0;

    // printf("<%d>\n", si);

    while (si >= 0)
    {
        val32 = 0UL;

        for (i = 0UL; (i < 8UL) && (si >= 0); i++)
        {
            hex = ch2hex(input[si]);
            val32 |= (uint32_t)hex << (i * 4UL);
            si--;
        }

        reg[ri++] = val32;
    }
}

void Hex2RegEx(char input[], uint32_t volatile reg[], int shift)
{
    uint32_t  hex, carry;
    int       si, ri;
    uint32_t  i, val32;

    si = (int)__strlen(input) - 1;
    ri = 0L;
    carry = 0UL;

    while (si >= 0)
    {
        val32 = 0UL;

        for (i = 0UL; (i < 8UL) && (si >= 0L); i++)
        {
            hex = (uint32_t)ch2hex(input[si]);
            hex <<= shift;

            val32 |= (uint32_t)((hex & 0xFUL) | carry) << (i * 4UL);
            carry = (hex >> 4UL) & 0xFUL;
            si--;
        }

        reg[ri++] = val32;
    }

    if (carry != 0UL)
    {
        reg[ri] = carry;
    }
}

/**
  * @brief  Extract specified nibble from an unsigned word in character format.
  *         For example:
  *                Suppose val32 is 0x786543210, get_Nth_nibble_char(val32, 3) will return a '3'.
  * @param[in]  val32   The input unsigned word
  * @param[in]  idx     The Nth nibble to be extracted.
  * @return  The nibble in character format.
  */
static char get_Nth_nibble_char(uint32_t val32, uint32_t idx)
{
    return hex_char_tbl[(val32 >> (idx * 4U)) & 0xfU ];
}


void Reg2Hex(int32_t count, uint32_t volatile reg[], char output[])
{
    int32_t    idx, ri;
    uint32_t   i;

    output[count] = 0U;
    idx = count - 1;

    for (ri = 0; idx >= 0; ri++)
    {
        for (i = 0UL; (i < 8UL) && (idx >= 0); i++)
        {
            output[idx] = get_Nth_nibble_char(reg[ri], i);
            idx--;
        }
    }
}

static int  get_nibble_value(char c)
{
    if ((c >= '0') && (c <= '9'))
    {
        c = c - '0';
    }

    if ((c >= 'a') && (c <= 'f'))
    {
        c = c - 'a' + (char)10;
    }

    if ((c >= 'A') && (c <= 'F'))
    {
        c = c - 'A' + (char)10;
    }

    return (int)c;
}

int ecc_strcmp(char *s1, char *s2)
{
    char  c1, c2;

    while (*s1 == '0') s1++;

    while (*s2 == '0') s2++;

    for (; *s1 || *s2; s1++, s2++)
    {
        if ((*s1 >= 'A') && (*s1 <= 'Z'))
            c1 = *s1 + 32;
        else
            c1 = *s1;

        if ((*s2 >= 'A') && (*s2 <= 'Z'))
            c2 = *s2 + 32;
        else
            c2 = *s2;

        if (c1 != c2)
            return 1;
    }

    return 0;
}



/** @} end of group CRYPTO_EXPORTED_FUNCTIONS */
/** @} end of group CRYPTO_Driver */
/** @} end of group Standard_Driver */
