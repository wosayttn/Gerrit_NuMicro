/*
 *  NIST SP800-38C compliant CCM implementation
 *
 *  Copyright The Mbed TLS Contributors
 *  Copyright (c) 2026, Nuvoton Technology Corporation
 *  SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*
 * Definition of CCM:
 * http://csrc.nist.gov/publications/nistpubs/800-38C/SP800-38C_updated-July20_2007.pdf
 * RFC 3610 "Counter with CBC-MAC (CCM)"
 *
 * Related:
 * RFC 5116 "An Interface and Algorithms for Authenticated Encryption"
 */

#include "tf_psa_crypto_common.h"

#include "nvt_psa_ccm.h"

#include <string.h>

#include <psa/crypto.h>

#include "mbedtls/private/error_common.h"
#include "mbedtls/platform.h"
#include "mbedtls/platform_util.h"
#include "psa_crypto_core.h"
#include "mbedtls/constant_time.h"
#include "nvt_crpt.h"
//#include "ccm.h"
//#define NVT_PSA_AES_USE_NUMICRO
//#if defined(NVT_PSA_AES_USE_NUMICRO)
#include "NuMicro.h"

#define CCM_STATE__CLEAR                0
#define CCM_STATE__STARTED              (1 << 0)
#define CCM_STATE__LENGHTS_SET          (1 << 1)
#define CCM_STATE__AUTH_DATA_STARTED    (1 << 2)
#define CCM_STATE__AUTH_DATA_FINISHED   (1 << 3)
#define CCM_STATE__ERROR                (1 << 4)

#define NVT_CCM_MULTIPART_AAD(operation)  ((operation)->ccm_multipart_buffer)
#define NVT_CCM_MULTIPART_DATA(operation) \
    ((operation)->ccm_multipart_buffer + MAX_CCM_BUF)

static size_t nvt_psa_ccm_align_block(size_t length)
{
    return (length + 15u) & ~(size_t) 15u;
}

static int nvt_psa_ccm_multipart_size_supported(size_t aad_length,
                                                size_t data_length)
{
    if (aad_length > MAX_CCM_BUF - 2u || data_length > MAX_CCM_BUF)
    {
        return 0;
    }

    return 16u + nvt_psa_ccm_align_block(aad_length + 2u) +
           nvt_psa_ccm_align_block(data_length) <= MAX_CCM_BUF;
}

/*
 * Initialize context
 */
void nvt_psa_ccm_init(nvt_psa_ccm_operation_t *ctx)
{
    memset(ctx, 0, sizeof(nvt_psa_ccm_operation_t));
}

int nvt_psa_ccm_setkey(nvt_psa_ccm_operation_t *ctx,
                       mbedtls_cipher_id_t cipher,
                       const unsigned char *key,
                       unsigned int keybits)
{
    int i;

    /* Store the key to ctx */
    for (i = 0; i < keybits / 8; i++)
        ctx->key[i] = key[i];

    ctx->key_bits = keybits;

    return (0);
}

/*
 * Free context
 */
void nvt_psa_ccm_free(nvt_psa_ccm_operation_t *ctx)
{
    if (ctx == NULL)
        return;

    mbedtls_cipher_free(&ctx->cipher_ctx);

    if (ctx->ccm_multipart_buffer != NULL)
    {
        mbedtls_platform_zeroize(ctx->ccm_multipart_buffer,
                                 2u * MAX_CCM_BUF);
        mbedtls_free(ctx->ccm_multipart_buffer);
    }

    mbedtls_platform_zeroize(ctx, sizeof(nvt_psa_ccm_operation_t));
}



/*
 * Encrypt or decrypt a partial block with CTR
 */
static int nvt_psa_ccm_crypt(nvt_psa_ccm_operation_t *ctx,
                             size_t offset, size_t use_len,
                             const unsigned char *input,
                             unsigned char *output)
{
    size_t i;
    size_t olen = 0;
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char tmp_buf[16] = {0};

    if ((ret = mbedtls_cipher_update(&ctx->cipher_ctx, ctx->ctr, 16, tmp_buf,
                                     &olen)) != 0)
    {
        ctx->state |= CCM_STATE__ERROR;
        mbedtls_platform_zeroize(tmp_buf, sizeof(tmp_buf));
        return ret;
    }

    for (i = 0; i < use_len; i++)
        output[i] = input[i] ^ tmp_buf[offset + i];

    mbedtls_platform_zeroize(tmp_buf, sizeof(tmp_buf));
    return ret;
}

static void nvt_psa_ccm_clear_state(nvt_psa_ccm_operation_t *ctx)
{
    ctx->state = CCM_STATE__CLEAR;
    memset(ctx->y, 0, 16);
    memset(ctx->ctr, 0, 16);
}

static int ccm_calculate_first_block_if_ready(nvt_psa_ccm_operation_t *ctx)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char i;
    size_t len_left, olen;

    /* length calulcation can be done only after both
     * nvt_pas_ccm_starts() and nvt_pas_ccm_set_lengths() have been executed
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
     * 1        .. iv_len   nonce (aka iv)  - set by: nvt_pas_ccm_starts()
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

int nvt_psa_ccm_starts(nvt_psa_ccm_operation_t *ctx,
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

int nvt_psa_ccm_set_lengths_internal(nvt_psa_ccm_operation_t *ctx,
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

int nvt_psa_ccm_update_ad(nvt_psa_ccm_operation_t *ctx,
                          const unsigned char *add,
                          size_t add_len)
{
    nvt_aead_operation_t *operation = ctx;

    if (operation == NULL || operation->active == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (operation->multipart_nonce_set == 0u ||
            operation->multipart_body_started != 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (add == NULL && add_len != 0u)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (add_len > MAX_CCM_BUF - operation->multipart_aad_length)
    {
        return PSA_ERROR_INSUFFICIENT_MEMORY;
    }

    if (operation->multipart_lengths_set != 0u &&
            add_len > operation->multipart_aad_expected -
            operation->multipart_aad_length)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (add_len != 0u)
    {
        memcpy(NVT_CCM_MULTIPART_AAD(operation) +
               operation->multipart_aad_length, add, add_len);
        operation->multipart_aad_length += add_len;
    }

    return PSA_SUCCESS;

}

int nvt_psa_ccm_update(nvt_psa_ccm_operation_t *ctx,
                       const unsigned char *input, size_t input_len,
                       unsigned char *output, size_t output_size,
                       size_t *output_len)
{
    nvt_aead_operation_t *operation = ctx;

    if (output_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    *output_len = 0u;

    if (operation == NULL || operation->active == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (operation->multipart_nonce_set == 0u ||
            (input == NULL && input_len != 0u))
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (input_len > MAX_CCM_BUF - operation->multipart_data_length)
    {
        return PSA_ERROR_INSUFFICIENT_MEMORY;
    }

    if (operation->multipart_lengths_set != 0u &&
            input_len > operation->multipart_data_expected -
            operation->multipart_data_length)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    (void) output;
    (void) output_size;

    if (input_len != 0u)
    {
        memcpy(NVT_CCM_MULTIPART_DATA(operation) +
               operation->multipart_data_length, input, input_len);
        operation->multipart_data_length += input_len;
        operation->multipart_body_started = 1u;
    }

    return PSA_SUCCESS;
}

//int nvt_psa_ccm_finish( nvt_psa_ccm_operation_t *ctx,
//                        unsigned char *tag, size_t tag_len )
//{
//    return (0);
//}

/*
 * Authenticated encryption or decryption
 */
static int ccm_auth_crypt(nvt_psa_ccm_operation_t *ctx, int mode, size_t length,
                          const unsigned char *iv, size_t iv_len,
                          const unsigned char *add, size_t add_len,
                          const unsigned char *input, unsigned char *output,
                          unsigned char *tag, size_t tag_len)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

    size_t add_len_aligned, length_aligned;
    int32_t i, j;
    uint32_t keySizeOp;
    int32_t timeout;

    if ((ret = nvt_psa_ccm_starts(ctx, mode, iv, iv_len)) != 0)
        return (ret);

    /* The default value is 1 in nvt_pas_ccm_starts but we need 0 */
    ctx->ctr[15] = 0;


    if ((ret = nvt_psa_ccm_set_lengths_internal(ctx, add_len, length,
                                                tag_len)) != 0)
        return (ret);

    /* Check size of input block */
    add_len_aligned = ((add_len + 2) & 0xf) ? (((add_len + 2) & (~0xf)) + 16) : (add_len + 2);
    length_aligned = (length & 0xf) ? ((length & (~0xf)) + 16) : (length);

    if (16 + add_len_aligned + length_aligned > MAX_CCM_BUF)
        return MBEDTLS_ERR_CCM_BAD_INPUT;

    /* Prepare input block for hardware CCM */
    memset(ctx->ccm_buf, 0, MAX_CCM_BUF);
    memcpy(ctx->ccm_buf, ctx->y, 16);

    /* add len in big endian. and support only support 2^16 - 2^8 length */
    ctx->ccm_buf[16] = add_len >> 8;
    ctx->ccm_buf[17] = add_len & 0xff;

    memcpy(ctx->ccm_buf + 16 + 2, add, add_len);
    memset(ctx->ccm_buf + 16 + 2 + add_len, 0, add_len_aligned - add_len - 2); // padding 0
    memcpy(ctx->ccm_buf + 16 + add_len_aligned, input, length);
    memset(ctx->ccm_buf + 16 + add_len_aligned + length, 0, length_aligned - length); // padding 0

    /* Init CCM Hardware */

    /* Polling on INTSTS below; do not enable the AES IRQ, or the shared
     * CRPT_IRQHandler (which only clears the SHA flag) re-triggers forever. */

    /* Basic AES CCM Option */
    keySizeOp = (ctx->key_bits == 256) ? AES_KEY_SIZE_256 : (ctx->key_bits == 192) ? AES_KEY_SIZE_192 : AES_KEY_SIZE_128;

    CRPT->AES_CTL = (mode << CRPT_AES_CTL_ENCRPT_Pos) | (AES_MODE_CCM << CRPT_AES_CTL_OPMODE_Pos) |
                    (keySizeOp << CRPT_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRPT_AES_CTL_OUTSWAP_Pos) | CRPT_AES_CTL_KINSWAP_Msk;

    /* Set Key */
    for (i = 0, j = 0; i < ctx->key_bits / 8; i += 4)
    {
        CRPT->AES_KEY[j++] = ctx->key[i] | (ctx->key[i + 1] << 8) | (ctx->key[i + 2] << 16) | (ctx->key[i + 3] << 24);
    }

    /* Set Ctr0 */
    for (i = 0, j = 0; i < 16; i += 4)
    {
        CRPT->AES_IV[j++] = ctx->ctr[i] | (ctx->ctr[i + 1] << 8) | (ctx->ctr[i + 2] << 16) | (ctx->ctr[i + 3] << 24);
    }


    /* Length settings. we assume length < 2^32 here */
    CRPT->AES_GCM_ACNT[0] = 16 + add_len_aligned;
    CRPT->AES_GCM_ACNT[1] = 0;
    CRPT->AES_GCM_PCNT[0] = length;
    CRPT->AES_GCM_PCNT[1] = 0;

    /* Init DMA */
    CRPT->AES_SADDR = (uint32_t)ctx->ccm_buf;
    CRPT->AES_DADDR = (uint32_t)ctx->out_buf;
    CRPT->AES_CNT = 16 + add_len_aligned + length_aligned;

    /* Clear flag */
    CRPT->INTSTS = CRPT_INTSTS_AESIF_Msk;

    /* Start AES CCM */
    CRPT->AES_CTL |= CRPT_AES_CTL_START_Msk | (CRYPTO_DMA_ONE_SHOT << CRPT_AES_CTL_DMALAST_Pos);

    timeout = SystemCoreClock;

    while ((CRPT->INTSTS & CRPT_INTSTS_AESIF_Msk) == 0)
    {
        /* Check timeout */
        if (timeout-- <= 0)
            return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
    }

    /* output */
    if (output != NULL)
        memcpy(output, ctx->out_buf, length);

    if (tag != NULL)
        memcpy(tag, ctx->out_buf + length_aligned, tag_len);

    nvt_psa_ccm_clear_state(ctx);

    return (0);
}

/*
 * Authenticated encryption
 */
int nvt_psa_ccm_star_encrypt_and_tag(nvt_psa_ccm_operation_t *ctx, size_t length,
                                     const unsigned char *iv, size_t iv_len,
                                     const unsigned char *add, size_t add_len,
                                     const unsigned char *input, unsigned char *output,
                                     unsigned char *tag, size_t tag_len)
{
    return (ccm_auth_crypt(ctx, MBEDTLS_CCM_STAR_ENCRYPT, length, iv, iv_len,
                           add, add_len, input, output, tag, tag_len));
}

int nvt_psa_ccm_encrypt_and_tag(nvt_psa_ccm_operation_t *ctx, size_t length,
                                const unsigned char *iv, size_t iv_len,
                                const unsigned char *add, size_t add_len,
                                const unsigned char *input, unsigned char *output,
                                unsigned char *tag, size_t tag_len)
{
    return (ccm_auth_crypt(ctx, MBEDTLS_CCM_ENCRYPT, length, iv, iv_len,
                           add, add_len, input, output, tag, tag_len));
}

/*
 * Authenticated decryption
 */
static int nvt_psa_ccm_compare_tags(const unsigned char *tag1, const unsigned char *tag2, size_t tag_len)
{
    unsigned char i;
    int diff;

    /* Check tag in "constant-time" */
    for (diff = 0, i = 0; i < tag_len; i++)
        diff |= tag1[i] ^ tag2[i];

    if (diff != 0)
    {
        return (MBEDTLS_ERR_CCM_AUTH_FAILED);
    }

    return (0);
}

static int ccm_auth_decrypt(nvt_psa_ccm_operation_t *ctx, int mode, size_t length,
                            const unsigned char *iv, size_t iv_len,
                            const unsigned char *add, size_t add_len,
                            const unsigned char *input, unsigned char *output,
                            const unsigned char *tag, size_t tag_len)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char check_tag[16];

    if ((ret = ccm_auth_crypt(ctx, mode, length,
                              iv, iv_len, add, add_len,
                              input, output, check_tag, tag_len)) != 0)
    {
        return (ret);
    }

    if ((ret = nvt_psa_ccm_compare_tags(tag, check_tag, tag_len)) != 0)
    {
        mbedtls_platform_zeroize(output, length);
        return (ret);
    }

    return (0);
}

int nvt_psa_ccm_star_auth_decrypt(nvt_psa_ccm_operation_t *ctx, size_t length,
                                  const unsigned char *iv, size_t iv_len,
                                  const unsigned char *add, size_t add_len,
                                  const unsigned char *input, unsigned char *output,
                                  const unsigned char *tag, size_t tag_len)
{
    return ccm_auth_decrypt(ctx, MBEDTLS_CCM_STAR_DECRYPT, length,
                            iv, iv_len, add, add_len,
                            input, output, tag, tag_len);
}

int nvt_psa_ccm_auth_decrypt(nvt_psa_ccm_operation_t *ctx, size_t length,
                             const unsigned char *iv, size_t iv_len,
                             const unsigned char *add, size_t add_len,
                             const unsigned char *input, unsigned char *output,
                             const unsigned char *tag, size_t tag_len)
{
    return ccm_auth_decrypt(ctx, MBEDTLS_CCM_DECRYPT, length,
                            iv, iv_len, add, add_len,
                            input, output, tag, tag_len);
}


/* ===== PSA Wrapper Layer Functions ===== */

/**
 * \brief Setup operation for PSA interface
 */
psa_status_t nvt_psa_ccm_setup(nvt_aead_operation_t *operation,
                               const psa_key_attributes_t *attributes,
                               const uint8_t *key,
                               size_t key_length,
                               psa_algorithm_t alg,
                               uint8_t is_encrypt)
{
    size_t key_bits;
    size_t tag_length;

    if (operation == NULL || attributes == NULL || key == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (PSA_ALG_AEAD_WITH_SHORTENED_TAG(alg, 0) !=
            PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 0) ||
            psa_get_key_type(attributes) != PSA_KEY_TYPE_AES)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    key_bits = psa_get_key_bits(attributes);

    if ((key_bits != 128u && key_bits != 192u && key_bits != 256u) ||
            key_length != key_bits / 8u)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    tag_length = PSA_ALG_AEAD_GET_TAG_LENGTH(alg);

    if (tag_length < 4u || tag_length > 16u || (tag_length & 1u) != 0u)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    memset(operation, 0, sizeof(*operation));
    operation->ccm_multipart_buffer = mbedtls_calloc(1u, 2u * MAX_CCM_BUF);

    if (operation->ccm_multipart_buffer == NULL)
    {
        return PSA_ERROR_INSUFFICIENT_MEMORY;
    }

    memcpy(operation->key, key, key_length);
    operation->key_bits = (uint32_t) key_bits;
    operation->alg = alg;
    operation->key_type = PSA_KEY_TYPE_AES;
    operation->is_encrypt = is_encrypt;
    operation->tag_length = (uint8_t) tag_length;
    operation->active = 1u;
    return PSA_SUCCESS;
}

/**
 * \brief One-shot CCM encryption for PSA interface
 */
psa_status_t nvt_psa_ccm_encrypt(const psa_key_attributes_t *attributes,
                                 const uint8_t *key,
                                 size_t key_length,
                                 psa_algorithm_t alg,
                                 const uint8_t *nonce,
                                 size_t nonce_length,
                                 const uint8_t *additional_data,
                                 size_t additional_data_length,
                                 const uint8_t *plaintext,
                                 size_t plaintext_length,
                                 uint8_t *ciphertext,
                                 size_t ciphertext_size,
                                 size_t *ciphertext_length)
{
    int ret;
    nvt_aead_operation_t operation;
    uint8_t tag_length;
    uint8_t tag_buffer[16];

    /* Validate algorithm */
    if (PSA_ALG_AEAD_WITH_SHORTENED_TAG(alg, 0) !=
            PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 0))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    /* Get tag length from algorithm */
    tag_length = (uint8_t)PSA_ALG_AEAD_GET_TAG_LENGTH(alg);

    /* Check output buffer size: ciphertext + tag */
    if (ciphertext_size < plaintext_length ||
            tag_length > ciphertext_size - plaintext_length)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    /* Initialize context */
    nvt_psa_ccm_init(&operation);

    /* Set key */
    ret = nvt_psa_ccm_setkey(&operation,
                             MBEDTLS_CIPHER_ID_AES,
                             key,
                             (unsigned int)psa_get_key_bits(attributes));

    if (ret != 0)
    {
        nvt_psa_ccm_free(&operation);
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    /* Perform encryption with tag generation */
    if (PSA_ALG_AEAD_WITH_SHORTENED_TAG(alg, 0) ==
            PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 0))
    {
        /* Standard CCM */
        ret = nvt_psa_ccm_encrypt_and_tag(&operation,
                                          plaintext_length,
                                          nonce,
                                          nonce_length,
                                          additional_data,
                                          additional_data_length,
                                          plaintext,
                                          ciphertext,
                                          tag_buffer,
                                          tag_length);
    }

    //      else if (alg == PSA_ALG_CCM_STAR_NO_TAG)
    //      {
    //        /* CCM* without tag */
    //        ret = nvt_psa_ccm_star_encrypt_and_tag(&operation,
    //                                               plaintext_length,
    //                                               nonce,
    //                                               nonce_length,
    //                                               additional_data,
    //                                               additional_data_length,
    //                                               plaintext,
    //                                               ciphertext,
    //                                               NULL,
    //                                               0);
    //        tag_length = 0;
    //    }
    //      else
    //      {
    //        /* CCM* with tag */
    //        ret = nvt_psa_ccm_star_encrypt_and_tag(&operation,
    //                                               plaintext_length,
    //                                               nonce,
    //                                               nonce_length,
    //                                               additional_data,
    //                                               additional_data_length,
    //                                               plaintext,
    //                                               ciphertext,
    //                                               tag_buffer,
    //                                               tag_length);
    //    }

    if (ret != 0)
    {
        nvt_psa_ccm_free(&operation);
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    /* Append tag to ciphertext */
    if (tag_length > 0)
    {
        memcpy(ciphertext + plaintext_length, tag_buffer, tag_length);
    }

    /* Set output length (plaintext + tag) */
    *ciphertext_length = plaintext_length + tag_length;

    /* Clean up */
    nvt_psa_ccm_free(&operation);

    return PSA_SUCCESS;
}

/**
 * \brief One-shot CCM decryption for PSA interface
 */
psa_status_t nvt_psa_ccm_decrypt(const psa_key_attributes_t *attributes,
                                 const uint8_t *key,
                                 size_t key_length,
                                 psa_algorithm_t alg,
                                 const uint8_t *nonce,
                                 size_t nonce_length,
                                 const uint8_t *additional_data,
                                 size_t additional_data_length,
                                 const uint8_t *ciphertext,
                                 size_t ciphertext_length,
                                 uint8_t *plaintext,
                                 size_t plaintext_size,
                                 size_t *plaintext_length)
{
    int ret;
    nvt_aead_operation_t operation;
    size_t tag_length;
    size_t payload_length;
    const uint8_t *tag;

    (void) key_length;

    /* Validate algorithm */
    if (PSA_ALG_AEAD_WITH_SHORTENED_TAG(alg, 0) !=
            PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 0))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    /* Extract tag length from algorithm */
    tag_length = (size_t) PSA_ALG_AEAD_GET_TAG_LENGTH(alg);

    /* Input must contain at least the authentication tag */
    if (ciphertext_length < tag_length)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    payload_length = ciphertext_length - tag_length;

    /* Output buffer must fit plaintext only (without tag) */
    if (plaintext_size < payload_length)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    /* Initialize context */
    nvt_psa_ccm_init(&operation);

    /* Set key */
    ret = nvt_psa_ccm_setkey(&operation,
                             MBEDTLS_CIPHER_ID_AES,
                             key,
                             (unsigned int) psa_get_key_bits(attributes));

    if (ret != 0)
    {
        nvt_psa_ccm_free(&operation);
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    /* Split input into ciphertext body and trailing tag */
    tag = ciphertext + payload_length;

    /* Authenticate and decrypt */
    ret = nvt_psa_ccm_auth_decrypt(&operation,
                                   payload_length,
                                   nonce,
                                   nonce_length,
                                   additional_data,
                                   additional_data_length,
                                   ciphertext,
                                   plaintext,
                                   tag,
                                   tag_length);

    if (ret != 0)
    {
        nvt_psa_ccm_free(&operation);

        if (ret == MBEDTLS_ERR_CCM_AUTH_FAILED)
        {
            return PSA_ERROR_INVALID_SIGNATURE;
        }

        return PSA_ERROR_HARDWARE_FAILURE;
    }

    *plaintext_length = payload_length;

    /* Clean up */
    nvt_psa_ccm_free(&operation);

    return PSA_SUCCESS;
}

/**
 * \brief Verify tag for PSA interface
 */
psa_status_t nvt_psa_ccm_verify(nvt_aead_operation_t *operation,
                                uint8_t *plaintext,
                                size_t plaintext_size,
                                size_t *plaintext_length,
                                const uint8_t *tag,
                                size_t tag_length)
{
    psa_status_t status;
    int ret;

    if (plaintext_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    *plaintext_length = 0u;

    if (operation == NULL || operation->active == 0u ||
            operation->is_encrypt != 0u ||
            operation->multipart_nonce_set == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (tag == NULL || tag_length != operation->tag_length ||
            (plaintext == NULL && operation->multipart_data_length != 0u))
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (plaintext_size < operation->multipart_data_length)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    if (operation->multipart_lengths_set != 0u &&
            (operation->multipart_aad_length != operation->multipart_aad_expected ||
             operation->multipart_data_length != operation->multipart_data_expected))
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (!nvt_psa_ccm_multipart_size_supported(operation->multipart_aad_length,
                                              operation->multipart_data_length))
    {
        return PSA_ERROR_INSUFFICIENT_MEMORY;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    ret = nvt_psa_ccm_setkey(operation, MBEDTLS_CIPHER_ID_AES,
                             operation->key, operation->key_bits);

    if (ret == 0)
    {
        ret = nvt_psa_ccm_auth_decrypt(operation,
                                       operation->multipart_data_length,
                                       operation->multipart_nonce,
                                       operation->multipart_nonce_length,
                                       NVT_CCM_MULTIPART_AAD(operation),
                                       operation->multipart_aad_length,
                                       NVT_CCM_MULTIPART_DATA(operation),
                                       plaintext, tag, tag_length);
    }

    nvt_crpt_release();

    if (ret == MBEDTLS_ERR_CCM_AUTH_FAILED)
    {
        if (operation->multipart_data_length != 0u)
        {
            mbedtls_platform_zeroize(plaintext,
                                     operation->multipart_data_length);
        }

        return PSA_ERROR_INVALID_SIGNATURE;
    }

    if (ret != 0)
    {
        if (operation->multipart_data_length != 0u)
        {
            mbedtls_platform_zeroize(plaintext,
                                     operation->multipart_data_length);
        }

        return PSA_ERROR_HARDWARE_FAILURE;
    }

    *plaintext_length = operation->multipart_data_length;
    return PSA_SUCCESS;
}

psa_status_t nvt_psa_ccm_abort(nvt_aead_operation_t *operation)
{
    if (operation != NULL)
    {
        nvt_psa_ccm_free(operation);
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_psa_ccm_set_nonce(nvt_aead_operation_t *operation,
                                   const uint8_t *nonce,
                                   size_t nonce_length)
{
    if (operation == NULL || operation->active == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (operation->multipart_nonce_set != 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (nonce == NULL || nonce_length < 7u || nonce_length > 13u)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    memcpy(operation->multipart_nonce, nonce, nonce_length);
    operation->multipart_nonce_length = nonce_length;
    operation->multipart_nonce_set = 1u;
    return PSA_SUCCESS;
}

psa_status_t nvt_psa_ccm_set_lengths(nvt_aead_operation_t *operation,
                                     size_t additional_data_length,
                                     size_t plaintext_length)
{
    if (operation == NULL || operation->active == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (operation->multipart_lengths_set != 0u ||
            operation->multipart_aad_length != 0u ||
            operation->multipart_data_length != 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (additional_data_length >= 0xff00u ||
            !nvt_psa_ccm_multipart_size_supported(additional_data_length,
                                                  plaintext_length))
    {
        return PSA_ERROR_INSUFFICIENT_MEMORY;
    }

    operation->multipart_aad_expected = additional_data_length;
    operation->multipart_data_expected = plaintext_length;
    operation->multipart_lengths_set = 1u;
    return PSA_SUCCESS;
}

psa_status_t nvt_psa_ccm_finish(nvt_aead_operation_t *operation,
                                uint8_t *ciphertext,
                                size_t ciphertext_size,
                                size_t *ciphertext_length,
                                uint8_t *tag,
                                size_t tag_size,
                                size_t *tag_length)
{
    psa_status_t status;
    int ret;

    if (ciphertext_length == NULL || tag_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    *ciphertext_length = 0u;
    *tag_length = 0u;

    if (operation == NULL || operation->active == 0u ||
            operation->is_encrypt == 0u ||
            operation->multipart_nonce_set == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if ((ciphertext == NULL && operation->multipart_data_length != 0u) ||
            tag == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (ciphertext_size < operation->multipart_data_length ||
            tag_size < operation->tag_length)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    if (operation->multipart_lengths_set != 0u &&
            (operation->multipart_aad_length != operation->multipart_aad_expected ||
             operation->multipart_data_length != operation->multipart_data_expected))
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (!nvt_psa_ccm_multipart_size_supported(operation->multipart_aad_length,
                                              operation->multipart_data_length))
    {
        return PSA_ERROR_INSUFFICIENT_MEMORY;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    ret = nvt_psa_ccm_setkey(operation, MBEDTLS_CIPHER_ID_AES,
                             operation->key, operation->key_bits);

    if (ret == 0)
    {
        ret = nvt_psa_ccm_encrypt_and_tag(operation,
                                          operation->multipart_data_length,
                                          operation->multipart_nonce,
                                          operation->multipart_nonce_length,
                                          NVT_CCM_MULTIPART_AAD(operation),
                                          operation->multipart_aad_length,
                                          NVT_CCM_MULTIPART_DATA(operation),
                                          ciphertext, tag,
                                          operation->tag_length);
    }

    nvt_crpt_release();

    if (ret != 0)
    {
        if (operation->multipart_data_length != 0u)
        {
            mbedtls_platform_zeroize(ciphertext,
                                     operation->multipart_data_length);
        }

        mbedtls_platform_zeroize(tag, tag_size);
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    *ciphertext_length = operation->multipart_data_length;
    *tag_length = operation->tag_length;
    return PSA_SUCCESS;
}
