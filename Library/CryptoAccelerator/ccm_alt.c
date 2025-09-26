/*
 *  NIST SP800-38C compliant CCM implementation
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  Copyright (c) 2022, Nuvoton Technology Corporation
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */

/*
 * Definition of CCM:
 * http://csrc.nist.gov/publications/nistpubs/800-38C/SP800-38C_updated-July20_2007.pdf
 * RFC 3610 "Counter with CBC-MAC (CCM)"
 *
 * Related:
 * RFC 5116 "An Interface and Algorithms for Authenticated Encryption"
 */

#include "common.h"

#if defined(MBEDTLS_CCM_C)

#include "mbedtls/ccm.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"
#include <string.h>

#if defined(MBEDTLS_SELF_TEST) && defined(MBEDTLS_AES_C)
    #if defined(MBEDTLS_PLATFORM_C)
        #include "mbedtls/platform.h"
    #else
        #include <stdio.h>
        #define mbedtls_printf printf
    #endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST && MBEDTLS_AES_C */

#if defined(MBEDTLS_CCM_ALT)

#define CCM_VALIDATE_RET( cond ) \
    MBEDTLS_INTERNAL_VALIDATE_RET( cond, MBEDTLS_ERR_CCM_BAD_INPUT )
#define CCM_VALIDATE( cond ) \
    MBEDTLS_INTERNAL_VALIDATE( cond )

#define CCM_ENCRYPT 1
#define CCM_DECRYPT 0




#define START       CRYPTO_AES_CTL_START_Msk
#define DMAEN       CRYPTO_AES_CTL_DMAEN_Msk
#define DMALAST     CRYPTO_AES_CTL_DMALAST_Msk
#define DMACC       CRYPTO_AES_CTL_DMACSCAD_Msk
#define START       CRYPTO_AES_CTL_START_Msk
#define FBIN        CRYPTO_AES_CTL_FBIN_Msk
#define FBOUT       CRYPTO_AES_CTL_FBOUT_Msk

#define GCM_MODE    (AES_MODE_GCM << CRYPTO_AES_CTL_OPMODE_Pos)
#define GHASH_MODE  (AES_MODE_GHASH << CRYPTO_AES_CTL_OPMODE_Pos)
#define CTR_MODE    (AES_MODE_CTR << CRYPTO_AES_CTL_OPMODE_Pos)

#define CCM_STATE__CLEAR                0
#define CCM_STATE__STARTED              (1 << 0)
#define CCM_STATE__LENGHTS_SET          (1 << 1)
#define CCM_STATE__AUTH_DATA_STARTED    (1 << 2)
#define CCM_STATE__AUTH_DATA_FINISHED   (1 << 3)
#define CCM_STATE__ERROR                (1 << 4)


#if (NVT_DCACHE_ON == 1)
    // DCache-line aligned buffer for improved performance when DCache is enabled
    uint8_t ccm_buf_array[DCACHE_ALIGN_LINE_SIZE(MAX_CCM_BUF)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t out_buf_array[DCACHE_ALIGN_LINE_SIZE(MAX_CCM_BUF + 16)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    // Standard buffer alignment when DCache is disabled
    __attribute__((aligned(4))) static uint8_t ccm_buf_array[MAX_CCM_BUF];
    __attribute__((aligned(4))) static uint8_t out_buf_array[MAX_CCM_BUF + 16];
#endif

int32_t ToBigEndian(uint8_t *pbuf, uint32_t u32Size)
{
    uint32_t i;
    uint8_t u8Tmp;
    uint32_t u32Tmp;

    /* pbuf must be word alignment */
    if ((uint32_t)pbuf & 0x3)
    {
        printf("The buffer must be 32-bit alignment.");
        return -1;
    }

    while (u32Size >= 4)
    {
        u8Tmp = *pbuf;
        *(pbuf) = *(pbuf + 3);
        *(pbuf + 3) = u8Tmp;

        u8Tmp = *(pbuf + 1);
        *(pbuf + 1) = *(pbuf + 2);
        *(pbuf + 2) = u8Tmp;

        u32Size -= 4;
        pbuf += 4;
    }

    if (u32Size > 0)
    {
        u32Tmp = 0;

        for (i = 0; i < u32Size; i++)
        {
            u32Tmp |= *(pbuf + i) << (24 - i * 8);
        }

        *((uint32_t *)pbuf) = u32Tmp;
    }

    return 0;
}


/*
    CCM input format must be block alignment. The block size is 16 bytes.

    ----------------------------------------------------------------------
     Block B0
          Formatting of the Control Information and the Nonce
    ----------------------------------------------------------------------
    First block B_0:
    0        .. 0        flags
    1        .. iv_len   nonce (aka iv)
    iv_len+1 .. 15       length


    flags:
    With flags as (bits):
    7        0
    6        add present?
    5 .. 3   (t - 2) / 2
    2 .. 0   q - 1

*/

int32_t CCMPacker(const uint8_t *iv, uint32_t ivlen, const uint8_t *A, uint32_t alen, const uint8_t *P, uint32_t plen, uint8_t *pbuf, uint32_t *psize, uint32_t tlen, int32_t enc)
{
    uint32_t i, j;
    uint32_t alen_aligned, plen_aligned;
    uint32_t u32Offset = 0;
    uint8_t u8Tmp;
    uint32_t q;


    /* Flags in B0
    *With flags as(bits) :
        7        0
        6        add present ?
        5 .. 3   (t - 2) / 2
        2 .. 0   q - 1, q = 15 - nlen
    */

    q = 15 - ivlen;
    u8Tmp = (q - 1) | ((tlen - 2) / 2 << 3) | ((alen > 0) ? 0x40 : 0);
    pbuf[0] = u8Tmp;            // flags

    for (i = 0; i < ivlen; i++) // N
        pbuf[i + 1] = iv[i];

    for (i = ivlen + 1, j = q - 1; i < 16; i++, j--)   // Q
    {
        if (j >= 4)
            pbuf[i] = 0;
        else
        {
            pbuf[i] = (plen >> j * 8) & 0xfful;
        }
    }

    u32Offset = 16;

    /* Formatting addition data */
    /* alen. It is limited to be smaller than 2^16-2^8 */
    if (alen > 0)
    {
        pbuf[u32Offset] = (alen >> 8) & 0xfful;
        pbuf[u32Offset + 1] = alen & 0xfful;

        for (i = 0; i < alen; i++)
            pbuf[u32Offset + i + 2] = A[i];

        alen_aligned = ((alen + 2 + 15) / 16) * 16;

        for (i = u32Offset + 2 + alen; i < alen_aligned; i++)
        {
            pbuf[i] = (enc) ? 0 : 0xff; // padding zero or 0xff
        }

        u32Offset += alen_aligned;
    }

    /* Formatting payload */
    if (plen > 0)
    {
        plen_aligned = ((plen + 15) / 16) * 16;

        for (i = 0; i < plen; i++)
        {
            pbuf[u32Offset + i] = P[i];
        }

        for (; i < plen_aligned; i++)
        {
            pbuf[u32Offset + i] = 0; // padding zero
        }

        u32Offset += plen_aligned;
    }


    /* Formatting Ctr0 */
    pbuf[u32Offset] = q - 1; // Flags

    for (i = 0; i < ivlen; i++) // N
    {
        pbuf[u32Offset + 1 + i] = iv[i];
    }

    for (; i < 16; i++)
    {
        pbuf[u32Offset + 1 + i] = 0; // padding zero to block alignment
    }

    *psize = u32Offset;

    return 0;
}


static int32_t _CCM(mbedtls_ccm_context *ctx, int32_t enc, const uint8_t *iv, uint32_t ivlen, const uint8_t *A, uint32_t alen, const uint8_t *P, uint32_t plen, uint8_t *buf, uint8_t *tag,
                    uint32_t tlen)
{
    uint32_t size, plen_aligned;
    int32_t timeout = 0x1000000;
    uint32_t *pu32;
    uint32_t key[8], i;

    if (ivlen > 16)
        return -1;

    for (i = 0; i < 8; i++)
    {
        key[i] = CRYPTO->AES_KEY[i];
    }

    /* Reset Crypto */
    SYS_UnlockReg();
    SYS->CRYPTORST |= SYS_CRYPTORST_CRYPTO0RST_Msk;
    SYS->CRYPTORST = 0;
    SYS_LockReg();

    for (i = 0; i < 8; i++)
    {
        CRYPTO->AES_KEY[i] = key[i];
    }


    AES_ENABLE_INT(CRYPTO);

    /* Prepare the blocked buffer for GCM */
    memset(ctx->ccm_buf, 0, MAX_CCM_BUF);
    CCMPacker(iv, ivlen, A, alen, P, plen, ctx->ccm_buf, &size, tlen, enc);

    ToBigEndian(ctx->ccm_buf, size + 16);

    plen_aligned = (plen & 0xful) ? ((plen + 15) / 16) * 16 : plen;

    if (ctx->keySize == 16)
    {
        CRYPTO->AES_CTL = (enc << CRYPTO_AES_CTL_ENCRYPTO_Pos) |
                          (AES_MODE_CCM << CRYPTO_AES_CTL_OPMODE_Pos) |
                          (AES_KEY_SIZE_128 << CRYPTO_AES_CTL_KEYSZ_Pos) |
                          (AES_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos);
    }
    else if (ctx->keySize == 24)
    {
        CRYPTO->AES_CTL = (enc << CRYPTO_AES_CTL_ENCRYPTO_Pos) |
                          (AES_MODE_CCM << CRYPTO_AES_CTL_OPMODE_Pos) |
                          (AES_KEY_SIZE_192 << CRYPTO_AES_CTL_KEYSZ_Pos) |
                          (AES_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos);
    }
    else
    {
        CRYPTO->AES_CTL = (enc << CRYPTO_AES_CTL_ENCRYPTO_Pos) |
                          (AES_MODE_CCM << CRYPTO_AES_CTL_OPMODE_Pos) |
                          (AES_KEY_SIZE_256 << CRYPTO_AES_CTL_KEYSZ_Pos) |
                          (AES_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos);
    }


    pu32 = (uint32_t *)&ctx->ccm_buf[size];
    CRYPTO->AES_IV[0] = pu32[0];
    CRYPTO->AES_IV[1] = pu32[1];
    CRYPTO->AES_IV[2] = pu32[2];
    CRYPTO->AES_IV[3] = pu32[3];


    /* Set bytes count of A */
    CRYPTO->AES_GCM_ACNT[0] = size - plen_aligned;
    CRYPTO->AES_GCM_ACNT[1] = 0;
    CRYPTO->AES_GCM_PCNT[0] = plen;
    CRYPTO->AES_GCM_PCNT[1] = 0;

#if (NVT_DCACHE_ON == 1)
    SCB_CleanDCache_by_Addr(ctx->ccm_buf, sizeof(ccm_buf_array));
#endif

    CRYPTO->AES_SADDR = (uint32_t)ctx->ccm_buf;
    CRYPTO->AES_DADDR = (uint32_t)ctx->out_buf;
    CRYPTO->AES_CNT   = size;

    /* Start AES Eecrypt */
    CRYPTO->AES_CTL |= CRYPTO_AES_CTL_START_Msk | (CRYPTO_DMA_ONE_SHOT << CRYPTO_AES_CTL_DMALAST_Pos);

    /* Waiting for AES calculation */
    while ((CRYPTO->INTSTS & CRYPTO_INTSTS_AESIF_Msk) == 0)
    {
        if (timeout-- < 0)
            return -1;
    }

    /* Clear flag */
    CRYPTO->INTSTS = CRYPTO_INTSTS_AESIF_Msk;
#if (NVT_DCACHE_ON == 1)
    SCB_InvalidateDCache_by_Addr(ctx->out_buf, sizeof(out_buf_array));
#endif
    memcpy(buf, ctx->out_buf, plen);

    if (tlen > 16)
    {
        tlen = 16;
    }

    memcpy(tag, &ctx->out_buf[plen_aligned], tlen);

    return 0;
}




/*
 * Initialize context
 */
void mbedtls_ccm_init(mbedtls_ccm_context *ctx)
{
    CCM_VALIDATE(ctx != NULL);
    memset(ctx, 0, sizeof(mbedtls_ccm_context));

    ctx->ccm_buf = &ccm_buf_array[0];
    ctx->out_buf = &out_buf_array[0];
}

int mbedtls_ccm_setkey(mbedtls_ccm_context *ctx,
                       mbedtls_cipher_id_t cipher,
                       const unsigned char *key,
                       unsigned int keybits)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    const mbedtls_cipher_info_t *cipher_info;

    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(key != NULL);

    uint32_t au32Buf[8];
    int32_t i, klen;

    cipher_info = mbedtls_cipher_info_from_values(cipher, keybits, MBEDTLS_MODE_ECB);

    if (cipher_info == NULL)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    if (cipher_info->block_size != 16)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    mbedtls_cipher_free(&ctx->cipher_ctx);

    if ((ret = mbedtls_cipher_setup(&ctx->cipher_ctx, cipher_info)) != 0)
        return (ret);

    if ((ret = mbedtls_cipher_setkey(&ctx->cipher_ctx, key, keybits,
                                     MBEDTLS_ENCRYPT)) != 0)
    {
        return (ret);
    }

    klen = keybits / 8;
    ctx->keySize = klen;
    memcpy(au32Buf, key, klen);
    ToBigEndian((uint8_t *)au32Buf, klen);

    for (i = 0; i < klen / 4; i++)
    {
        CRYPTO->AES_KEY[i] = au32Buf[i];
    }

    return (0);
}

/*
 * Free context
 */
void mbedtls_ccm_free(mbedtls_ccm_context *ctx)
{
    if (ctx == NULL)
        return;

    mbedtls_cipher_free(&ctx->cipher_ctx);
    mbedtls_platform_zeroize(ctx, sizeof(mbedtls_ccm_context));
}

/*
 * Macros for common operations.
 * Results in smaller compiled code than static inline functions.
 */

/*
 * Update the CBC-MAC state in y using a block in b
 * (Always using b as the source helps the compiler optimise a bit better.)
 */
#define UPDATE_CBC_MAC                                                      \
    for( i = 0; i < 16; i++ )                                               \
        y[i] ^= b[i];                                                       \
    \
    if( ( ret = mbedtls_cipher_update( &ctx->cipher_ctx, y, 16, y, &olen ) ) != 0 ) \
        return( ret );

/*
 * Encrypt or decrypt a partial block with CTR
 * Warning: using b for temporary storage! src and dst must not be b!
 * This avoids allocating one more 16 bytes buffer while allowing src == dst.
 */
#define CTR_CRYPT( dst, src, len  )                                            \
    if( ( ret = mbedtls_cipher_update( &ctx->cipher_ctx, ctr, 16, b, &olen ) ) != 0 )  \
        return( ret );                                                         \
    \
    for( i = 0; i < len; i++ )                                                 \
        dst[i] = src[i] ^ b[i];

/*
 * Authenticated encryption or decryption
 */
static int ccm_auth_crypt(mbedtls_ccm_context *ctx, int mode, size_t length,
                          const unsigned char *iv, size_t iv_len,
                          const unsigned char *add, size_t add_len,
                          const unsigned char *input, unsigned char *output,
                          unsigned char *tag, size_t tag_len)
{
    int ret;

    if (iv_len < 7 || iv_len > 13)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);


    ret = _CCM(ctx, mode, iv, iv_len, add, add_len, input, length, output, tag, tag_len);
    return (ret);
}

/*
 * Authenticated encryption
 */
int mbedtls_ccm_star_encrypt_and_tag(mbedtls_ccm_context *ctx, size_t length,
                                     const unsigned char *iv, size_t iv_len,
                                     const unsigned char *add, size_t add_len,
                                     const unsigned char *input, unsigned char *output,
                                     unsigned char *tag, size_t tag_len)
{
    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(iv != NULL);
    CCM_VALIDATE_RET(add_len == 0 || add != NULL);
    CCM_VALIDATE_RET(length == 0 || input != NULL);
    CCM_VALIDATE_RET(length == 0 || output != NULL);
    CCM_VALIDATE_RET(tag_len == 0 || tag != NULL);
    return (ccm_auth_crypt(ctx, CCM_ENCRYPT, length, iv, iv_len,
                           add, add_len, input, output, tag, tag_len));
}

int mbedtls_ccm_encrypt_and_tag(mbedtls_ccm_context *ctx, size_t length,
                                const unsigned char *iv, size_t iv_len,
                                const unsigned char *add, size_t add_len,
                                const unsigned char *input, unsigned char *output,
                                unsigned char *tag, size_t tag_len)
{
    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(iv != NULL);
    CCM_VALIDATE_RET(add_len == 0 || add != NULL);
    CCM_VALIDATE_RET(length == 0 || input != NULL);
    CCM_VALIDATE_RET(length == 0 || output != NULL);
    CCM_VALIDATE_RET(tag_len == 0 || tag != NULL);

    if (tag_len == 0)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    return (mbedtls_ccm_star_encrypt_and_tag(ctx, length, iv, iv_len, add,
                                             add_len, input, output, tag, tag_len));
}

/*
 * Authenticated decryption
 */
int mbedtls_ccm_star_auth_decrypt(mbedtls_ccm_context *ctx, size_t length,
                                  const unsigned char *iv, size_t iv_len,
                                  const unsigned char *add, size_t add_len,
                                  const unsigned char *input, unsigned char *output,
                                  const unsigned char *tag, size_t tag_len)
{
    int ret;
    unsigned char check_tag[16];
    unsigned char i;
    int diff;

    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(iv != NULL);
    CCM_VALIDATE_RET(add_len == 0 || add != NULL);
    CCM_VALIDATE_RET(length == 0 || input != NULL);
    CCM_VALIDATE_RET(length == 0 || output != NULL);
    CCM_VALIDATE_RET(tag_len == 0 || tag != NULL);

    if ((ret = ccm_auth_crypt(ctx, CCM_DECRYPT, length,
                              iv, iv_len, add, add_len,
                              input, output, check_tag, tag_len)) != 0)
    {
        return (ret);
    }

    /* Check tag in "constant-time" */
    for (diff = 0, i = 0; i < tag_len; i++)
        diff |= tag[i] ^ check_tag[i];

    if (diff != 0)
    {
        mbedtls_platform_zeroize(output, length);
        return (MBEDTLS_ERR_CCM_AUTH_FAILED);
    }

    return (0);
}

int mbedtls_ccm_auth_decrypt(mbedtls_ccm_context *ctx, size_t length,
                             const unsigned char *iv, size_t iv_len,
                             const unsigned char *add, size_t add_len,
                             const unsigned char *input, unsigned char *output,
                             const unsigned char *tag, size_t tag_len)
{
    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(iv != NULL);
    CCM_VALIDATE_RET(add_len == 0 || add != NULL);
    CCM_VALIDATE_RET(length == 0 || input != NULL);
    CCM_VALIDATE_RET(length == 0 || output != NULL);
    CCM_VALIDATE_RET(tag_len == 0 || tag != NULL);

    if (tag_len == 0)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    return (mbedtls_ccm_star_auth_decrypt(ctx, length, iv, iv_len, add,
                                          add_len, input, output, tag, tag_len));
}

static int ccm_calculate_first_block_if_ready(mbedtls_ccm_context *ctx)
{
    //int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char i;
    size_t len_left;//, olen;

    /* length calulcation can be done only after both
     * mbedtls_ccm_starts() and mbedtls_ccm_set_lengths() have been executed
     */
    if (!(ctx->state & CCM_STATE__STARTED) || !(ctx->state & CCM_STATE__LENGHTS_SET))
        return 0;

    /* CCM expects non-empty tag.
     * CCM* allows empty tag. For CCM* without tag, ignore plaintext length.
     */
    if (ctx->tag_len == 0)
    {
        if (ctx->mode == MBEDTLS_CCM_STAR_ENCRYPT || ctx->mode == MBEDTLS_CCM_STAR_DECRYPT)
        {
            ctx->plaintext_len = 0;
        }
        else
        {
            return (MBEDTLS_ERR_CCM_BAD_INPUT);
        }
    }

    /*
     * First block:
     * 0        .. 0        flags
     * 1        .. iv_len   nonce (aka iv)  - set by: mbedtls_ccm_starts()
     * iv_len+1 .. 15       length
     *
     * With flags as (bits):
     * 7        0
     * 6        add present?
     * 5 .. 3   (t - 2) / 2
     * 2 .. 0   q - 1
     */
    ctx->y[0] |= (ctx->add_len > 0) << 6;
    ctx->y[0] |= ((ctx->tag_len - 2) / 2) << 3;
    ctx->y[0] |= ctx->q - 1;

    for (i = 0, len_left = ctx->plaintext_len; i < ctx->q; i++, len_left >>= 8)
        ctx->y[15 - i] = MBEDTLS_BYTE_0(len_left);

    if (len_left > 0)
    {
        ctx->state |= CCM_STATE__ERROR;
        return (MBEDTLS_ERR_CCM_BAD_INPUT);
    }

    return (0);
}

int mbedtls_ccm_starts(mbedtls_ccm_context *ctx,
                       int mode,
                       const unsigned char *iv,
                       size_t iv_len)
{
    /* Also implies q is within bounds */
    if (iv_len < 7 || iv_len > 13)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    ctx->mode = mode;
    ctx->q = 16 - 1 - (unsigned char) iv_len;

    /*
     * Prepare counter block for encryption:
     * 0        .. 0        flags
     * 1        .. iv_len   nonce (aka iv)
     * iv_len+1 .. 15       counter (initially 1)
     *
     * With flags as (bits):
     * 7 .. 3   0
     * 2 .. 0   q - 1
     */
    memset(ctx->ctr, 0, 16);
    ctx->ctr[0] = ctx->q - 1;
    memcpy(ctx->ctr + 1, iv, iv_len);
    memset(ctx->ctr + 1 + iv_len, 0, ctx->q);
    ctx->ctr[15] = 1;

    /*
     * See ccm_calculate_first_block_if_ready() for block layout description
     */
    memcpy(ctx->y + 1, iv, iv_len);

    ctx->state |= CCM_STATE__STARTED;
    return ccm_calculate_first_block_if_ready(ctx);
}

int mbedtls_ccm_set_lengths(mbedtls_ccm_context *ctx,
                            size_t total_ad_len,
                            size_t plaintext_len,
                            size_t tag_len)
{
    /*
     * Check length requirements: SP800-38C A.1
     * Additional requirement: a < 2^16 - 2^8 to simplify the code.
     * 'length' checked later (when writing it to the first block)
     *
     * Also, loosen the requirements to enable support for CCM* (IEEE 802.15.4).
     */
    if (tag_len == 2 || tag_len > 16 || tag_len % 2 != 0)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    if (total_ad_len >= 0xFF00)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    ctx->plaintext_len = plaintext_len;
    ctx->add_len = total_ad_len;
    ctx->tag_len = tag_len;
    ctx->processed = 0;

    ctx->state |= CCM_STATE__LENGHTS_SET;
    return ccm_calculate_first_block_if_ready(ctx);
}

int mbedtls_ccm_update(mbedtls_ccm_context *ctx,
                       const unsigned char *input, size_t input_len,
                       unsigned char *output, size_t output_size,
                       size_t *output_len)
{
    return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
}

int mbedtls_ccm_update_ad(mbedtls_ccm_context *ctx,
                          const unsigned char *add,
                          size_t add_len)
{

    return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;

}

int mbedtls_ccm_finish(mbedtls_ccm_context *ctx,
                       unsigned char *tag, size_t tag_len)
{
    return (0);
}
#endif /* MBEDTLS_CCM_ALT */
#endif /* MBEDTLS_CCM_C */
