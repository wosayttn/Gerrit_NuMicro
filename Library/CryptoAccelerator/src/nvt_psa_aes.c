/*
 * Extracted AES helper for the MY_AES PSA driver.
 */

/*
 * Copyright The Mbed TLS Contributors
 * Copyright (c) 2026, Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "nvt_psa_aes.h"

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
    volatile nvt_psa_cmac_path_t nvt_psa_cmac_last_compute_path =
    NVT_PSA_CMAC_PATH_FAILED;
#endif

#include <string.h>

#include "mbedtls/private/aes.h"
#include "mbedtls/platform_util.h"
#include "nvt_crpt.h"

#include "mbedtls/platform.h"
#include "tf_psa_crypto_common.h"//to use PSA_DEBUG_LOG


#include "NuMicro.h"

#define NVT_PSA_AES_MAX_DMA_CHAIN_SIZE (NVT_PSA_AES_BLOCK_SIZE * 6u)

__ALIGNED(4) static uint8_t dma_in[NVT_PSA_AES_MAX_DMA_CHAIN_SIZE];
__ALIGNED(4) static uint8_t dma_out[NVT_PSA_AES_MAX_DMA_CHAIN_SIZE];

static uint32_t get32_le(const uint8_t *pos)
{
    return ((uint32_t) pos[0]) |
           ((uint32_t) pos[1] << 8) |
           ((uint32_t) pos[2] << 16) |
           ((uint32_t) pos[3] << 24);
}

static void set_key_regs(const nvt_psa_aes_operation_t *operation)
{
    size_t index;
    size_t words = (size_t) operation->key_bits / 32u;

    for (index = 0; index < words; index++)
    {
        CRPT->AES_KEY[index] = get32_le(operation->key + index * 4u);
    }
}

static uint32_t key_size_bits(uint32_t key_bits)
{
    if (key_bits == 192u)
    {
        return 1u << CRPT_AES_CTL_KEYSZ_Pos;
    }

    if (key_bits == 256u)
    {
        return 2u << CRPT_AES_CTL_KEYSZ_Pos;
    }

    return 0u;
}

/**
 * @brief Encrypt or decrypt one DMA chunk using the AES hardware engine.
 *
 * @param operation   Pointer to the active AES operation context.
 * @param opmode_bits Hardware OPMODE bits (e.g. AES_MODE_CBC << CRPT_AES_CTL_OPMODE_Pos).
 * @param input       Pointer to input data buffer.
 * @param output      Pointer to output data buffer.
 * @param data_size   Number of bytes to process, must be block-aligned.
 *
 * @return PSA_SUCCESS on success, otherwise an appropriate PSA error code.
 */
static psa_status_t hw_crypt(const nvt_psa_aes_operation_t *operation,
                             uint32_t opmode_bits,
                             const uint8_t *input,
                             uint8_t *output,
                             size_t data_size)
{
    int32_t timeout = 0x10000000;
    uint32_t control;

    if (data_size == 0u || data_size > NVT_PSA_AES_MAX_DMA_CHAIN_SIZE ||
            (data_size % NVT_PSA_AES_BLOCK_SIZE) != 0u)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    memcpy(dma_in, input, data_size);
    CRPT->AES_CTL = CRPT_AES_CTL_STOP_Msk;
    /* Polling on INTSTS below; do not enable the AES IRQ, or the shared
     * CRPT_IRQHandler (which only clears the SHA flag) re-triggers forever. */

    control = opmode_bits |
              key_size_bits(operation->key_bits) |
              (operation->is_encrypt != 0u ? CRPT_AES_CTL_ENCRPT_Msk : 0u) |
              (AES_IN_OUT_SWAP << CRPT_AES_CTL_OUTSWAP_Pos) |
              CRPT_AES_CTL_KINSWAP_Msk |
              CRPT_AES_CTL_DMAEN_Msk;

    CRPT->AES_IV[0] = get32_le(operation->iv + 0u);
    CRPT->AES_IV[1] = get32_le(operation->iv + 4u);
    CRPT->AES_IV[2] = get32_le(operation->iv + 8u);
    CRPT->AES_IV[3] = get32_le(operation->iv + 12u);

    set_key_regs(operation);

    CRPT->AES_SADDR = (uint32_t) dma_in;
    CRPT->AES_DADDR = (uint32_t) dma_out;
    CRPT->AES_CNT   = (uint32_t) data_size;
    CRPT->INTSTS    = CRPT_INTSTS_AESIF_Msk;
    CRPT->AES_CTL   = control | CRPT_AES_CTL_START_Msk;

    while ((CRPT->INTSTS & CRPT_INTSTS_AESIF_Msk) == 0u)
    {
        if (timeout-- <= 0)
        {
            return PSA_ERROR_HARDWARE_FAILURE;
        }
    }

    memcpy(output, dma_out, data_size);
    return PSA_SUCCESS;
}

/**
 * @brief Perform AES-ECB encryption or decryption on full blocks.
 *
 * @param operation  Pointer to the active AES operation context.
 * @param input      Pointer to input data buffer.
 * @param output     Pointer to output data buffer.
 * @param length     Number of bytes to process; must be a multiple of the block size.
 *
 * @return PSA_SUCCESS on success, otherwise an appropriate PSA error code.
 */
static psa_status_t aes_crypt_ecb(const nvt_psa_aes_operation_t *operation,
                                  const uint8_t *input,
                                  uint8_t *output,
                                  size_t length)
{
    psa_status_t status;
    size_t offset;

    for (offset = 0u; offset < length; offset += NVT_PSA_AES_BLOCK_SIZE)
    {
        status = hw_crypt(operation,
                          AES_MODE_ECB << CRPT_AES_CTL_OPMODE_Pos,
                          input + offset,
                          output + offset,
                          NVT_PSA_AES_BLOCK_SIZE);

        if (status != PSA_SUCCESS)
        {
            return status;
        }
    }

    return PSA_SUCCESS;
}

static void cmac_double(uint8_t value[NVT_PSA_AES_BLOCK_SIZE])
{
    uint8_t carry = (uint8_t)(value[0] >> 7);
    size_t index;

    for (index = 0u; index + 1u < NVT_PSA_AES_BLOCK_SIZE; index++)
    {
        value[index] = (uint8_t)((value[index] << 1) |
                                 (value[index + 1u] >> 7));
    }

    value[NVT_PSA_AES_BLOCK_SIZE - 1u] <<= 1;

    if (carry != 0u)
    {
        value[NVT_PSA_AES_BLOCK_SIZE - 1u] ^= 0x87u;
    }
}

psa_status_t nvt_psa_cmac_compute(
    const psa_key_attributes_t *attributes,
    const uint8_t *key,
    size_t key_length,
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *mac,
    size_t mac_size,
    size_t *mac_length)
{
    nvt_psa_aes_operation_t operation;
    uint8_t zero_block[NVT_PSA_AES_BLOCK_SIZE] = {0};
    uint8_t subkey1[NVT_PSA_AES_BLOCK_SIZE];
    uint8_t subkey2[NVT_PSA_AES_BLOCK_SIZE];
    uint8_t chaining[NVT_PSA_AES_BLOCK_SIZE] = {0};
    uint8_t block[NVT_PSA_AES_BLOCK_SIZE];
    size_t block_count;
    size_t remainder;
    size_t index;
    size_t block_index;
    psa_status_t status;

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
    nvt_psa_cmac_last_compute_path = NVT_PSA_CMAC_PATH_FAILED;
#endif

    if (mac_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    *mac_length = 0u;

    if (attributes == NULL || mac == NULL ||
            (key == NULL && key_length != 0u) ||
            (input == NULL && input_length != 0u))
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (psa_get_key_type(attributes) != PSA_KEY_TYPE_AES || alg != PSA_ALG_CMAC)
    {
#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
        nvt_psa_cmac_last_compute_path = NVT_PSA_CMAC_PATH_BUILTIN_FALLBACK;
#endif
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (mac_size < NVT_PSA_AES_BLOCK_SIZE)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    if (key_length != (psa_get_key_bits(attributes) + 7u) / 8u ||
            (key_length != 16u && key_length != 24u && key_length != 32u))
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    memset(&operation, 0, sizeof(operation));
    memcpy(operation.key, key, key_length);
    operation.key_bits = (uint32_t)(key_length * 8u);
    operation.key_length = (uint32_t) key_length;
    operation.alg = PSA_ALG_ECB_NO_PADDING;
    operation.is_encrypt = 1u;

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

    if (nvt_psa_test_force_hardware_failure != 0u)
    {
        status = PSA_ERROR_HARDWARE_FAILURE;
        goto cleanup;
    }

#endif

    status = hw_crypt(&operation,
                      AES_MODE_ECB << CRPT_AES_CTL_OPMODE_Pos,
                      zero_block, subkey1, sizeof(subkey1));

    if (status != PSA_SUCCESS)
    {
        goto cleanup;
    }

    cmac_double(subkey1);
    memcpy(subkey2, subkey1, sizeof(subkey2));
    cmac_double(subkey2);

    block_count = (input_length == 0u) ? 1u :
                  1u + (input_length - 1u) / NVT_PSA_AES_BLOCK_SIZE;
    remainder = input_length % NVT_PSA_AES_BLOCK_SIZE;

    for (block_index = 0u; block_index + 1u < block_count; block_index++)
    {
        for (index = 0u; index < NVT_PSA_AES_BLOCK_SIZE; index++)
        {
            block[index] = input[block_index * NVT_PSA_AES_BLOCK_SIZE + index] ^
                           chaining[index];
        }

        status = hw_crypt(&operation,
                          AES_MODE_ECB << CRPT_AES_CTL_OPMODE_Pos,
                          block, chaining, sizeof(chaining));

        if (status != PSA_SUCCESS)
        {
            goto cleanup;
        }
    }

    memset(block, 0, sizeof(block));

    if (input_length != 0u && remainder == 0u)
    {
        memcpy(block, input + (block_count - 1u) * NVT_PSA_AES_BLOCK_SIZE,
               sizeof(block));

        for (index = 0u; index < NVT_PSA_AES_BLOCK_SIZE; index++)
        {
            block[index] ^= subkey1[index];
        }
    }
    else
    {
        if (remainder != 0u)
        {
            memcpy(block, input + (block_count - 1u) * NVT_PSA_AES_BLOCK_SIZE,
                   remainder);
        }

        block[remainder] = 0x80u;

        for (index = 0u; index < NVT_PSA_AES_BLOCK_SIZE; index++)
        {
            block[index] ^= subkey2[index];
        }
    }

    for (index = 0u; index < NVT_PSA_AES_BLOCK_SIZE; index++)
    {
        block[index] ^= chaining[index];
    }

    status = hw_crypt(&operation,
                      AES_MODE_ECB << CRPT_AES_CTL_OPMODE_Pos,
                      block, mac, NVT_PSA_AES_BLOCK_SIZE);

    if (status == PSA_SUCCESS)
    {
        *mac_length = NVT_PSA_AES_BLOCK_SIZE;
#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
        nvt_psa_cmac_last_compute_path = NVT_PSA_CMAC_PATH_HARDWARE_AES;
#endif
    }

cleanup:
    mbedtls_platform_zeroize(&operation, sizeof(operation));
    mbedtls_platform_zeroize(subkey1, sizeof(subkey1));
    mbedtls_platform_zeroize(subkey2, sizeof(subkey2));
    mbedtls_platform_zeroize(chaining, sizeof(chaining));
    mbedtls_platform_zeroize(block, sizeof(block));
    nvt_crpt_release();
    return status;
}

/**
 * @brief Perform AES-CBC encryption or decryption on full blocks.
 *
 * The IV in \p operation is updated after each chunk so that successive
 * calls produce a correctly chained result.
 *
 * @param operation  Pointer to the active AES operation context (IV updated on return).
 * @param input      Pointer to input data buffer.
 * @param output     Pointer to output data buffer.
 * @param length     Number of bytes to process; must be a multiple of the block size.
 *
 * @return PSA_SUCCESS on success, otherwise an appropriate PSA error code.
 */
static psa_status_t aes_crypt_cbc(nvt_psa_aes_operation_t *operation,
                                  const uint8_t *input,
                                  uint8_t *output,
                                  size_t length)
{
    psa_status_t status;
    const uint8_t *input_ptr  = input;
    uint8_t       *output_ptr = output;
    size_t         remaining  = length;

    while (remaining > 0u)
    {
        size_t chunk = (remaining > NVT_PSA_AES_MAX_DMA_CHAIN_SIZE)
                       ? NVT_PSA_AES_MAX_DMA_CHAIN_SIZE : remaining;

        status = hw_crypt(operation,
                          AES_MODE_CBC << CRPT_AES_CTL_OPMODE_Pos,
                          input_ptr, output_ptr, chunk);

        if (status != PSA_SUCCESS)
        {
            return status;
        }

        /* Update IV: encrypt → last ciphertext block; decrypt → last ciphertext (= input) */
        if (operation->is_encrypt != 0u)
        {
            memcpy(operation->iv, output_ptr + chunk - NVT_PSA_AES_BLOCK_SIZE,
                   NVT_PSA_AES_BLOCK_SIZE);
        }
        else
        {
            memcpy(operation->iv, input_ptr + chunk - NVT_PSA_AES_BLOCK_SIZE,
                   NVT_PSA_AES_BLOCK_SIZE);
        }

        input_ptr  += chunk;
        output_ptr += chunk;
        remaining  -= chunk;
    }

    return PSA_SUCCESS;
}

/**
 * @brief Perform AES-CFB128 encryption or decryption.
 *
 * CFB128 uses the same key schedule for both encryption and decryption.
 * The IV in \p operation is updated after each chunk.
 *
 * @param operation  Pointer to the active AES operation context (IV updated on return).
 * @param input      Pointer to input data buffer.
 * @param output     Pointer to output data buffer.
 * @param length     Number of bytes to process.
 *
 * @return PSA_SUCCESS on success, otherwise an appropriate PSA error code.
 */
static psa_status_t aes_crypt_cfb128(nvt_psa_aes_operation_t *operation,
                                     const uint8_t *input,
                                     uint8_t *output,
                                     size_t length)
{
    psa_status_t status;
    const uint8_t *input_ptr  = input;
    uint8_t       *output_ptr = output;
    size_t         remaining  = length;

    while (remaining > 0u)
    {
        size_t chunk = (remaining > NVT_PSA_AES_MAX_DMA_CHAIN_SIZE)
                       ? NVT_PSA_AES_MAX_DMA_CHAIN_SIZE : remaining;

        status = hw_crypt(operation,
                          AES_MODE_CFB << CRPT_AES_CTL_OPMODE_Pos,
                          input_ptr, output_ptr, chunk);

        if (status != PSA_SUCCESS)
        {
            return status;
        }

        /* Update IV: always the last ciphertext block */
        if (operation->is_encrypt != 0u)
        {
            memcpy(operation->iv, output_ptr + chunk - NVT_PSA_AES_BLOCK_SIZE,
                   NVT_PSA_AES_BLOCK_SIZE);
        }
        else
        {
            memcpy(operation->iv, input_ptr + chunk - NVT_PSA_AES_BLOCK_SIZE,
                   NVT_PSA_AES_BLOCK_SIZE);
        }

        input_ptr  += chunk;
        output_ptr += chunk;
        remaining  -= chunk;
    }

    return PSA_SUCCESS;
}

/**
 * @brief Perform AES-OFB encryption or decryption.
 *
 * OFB mode is symmetric: the same operation is used for both encryption
 * and decryption. The IV in \p operation is updated after each chunk.
 *
 * @param operation  Pointer to the active AES operation context (IV updated on return).
 * @param input      Pointer to input data buffer.
 * @param output     Pointer to output data buffer.
 * @param length     Number of bytes to process.
 *
 * @return PSA_SUCCESS on success, otherwise an appropriate PSA error code.
 */
static psa_status_t aes_crypt_ofb(nvt_psa_aes_operation_t *operation,
                                  const uint8_t *input,
                                  uint8_t *output,
                                  size_t length)
{
    psa_status_t status;
    const uint8_t *input_ptr  = input;
    uint8_t       *output_ptr = output;
    size_t         remaining  = length;

    while (remaining > 0u)
    {
        size_t chunk = (remaining > NVT_PSA_AES_MAX_DMA_CHAIN_SIZE)
                       ? NVT_PSA_AES_MAX_DMA_CHAIN_SIZE : remaining;

        status = hw_crypt(operation,
                          AES_MODE_OFB << CRPT_AES_CTL_OPMODE_Pos,
                          input_ptr, output_ptr, chunk);

        if (status != PSA_SUCCESS)
        {
            return status;
        }

        /* Update IV: OFB keystream K = output XOR input, so next IV = K_last */
        {
            size_t i;
            const uint8_t *last_out = output_ptr + chunk - NVT_PSA_AES_BLOCK_SIZE;
            const uint8_t *last_in  = input_ptr  + chunk - NVT_PSA_AES_BLOCK_SIZE;

            for (i = 0u; i < NVT_PSA_AES_BLOCK_SIZE; i++)
            {
                operation->iv[i] = last_out[i] ^ last_in[i];
            }
        }

        input_ptr  += chunk;
        output_ptr += chunk;
        remaining  -= chunk;
    }

    return PSA_SUCCESS;
}

static int aes_ctr_increment_counter(uint8_t counter[NVT_PSA_AES_BLOCK_SIZE])
{
    size_t index = NVT_PSA_AES_BLOCK_SIZE;

    while (index > 0u)
    {
        index--;
        counter[index]++;

        if (counter[index] != 0u)
        {
            return 0;
        }
    }

    return 1;
}

static psa_status_t aes_crypt_ctr(nvt_psa_aes_operation_t *operation,
                                  const uint8_t *input,
                                  uint8_t *output,
                                  size_t length)
{
    static const uint8_t zero_block[NVT_PSA_AES_BLOCK_SIZE] = {0};
    nvt_psa_aes_operation_t counter_operation = *operation;
    size_t offset = 0u;

    counter_operation.is_encrypt = 1u;

    while (offset < length)
    {
        size_t available;
        size_t chunk;
        size_t index;
        psa_status_t status;

        if (operation->iv_off == NVT_PSA_AES_BLOCK_SIZE)
        {
            if (operation->ctr_exhausted != 0u)
            {
                return PSA_ERROR_INVALID_ARGUMENT;
            }

            memcpy(counter_operation.iv, operation->iv, sizeof(operation->iv));
            status = hw_crypt(&counter_operation,
                              AES_MODE_CTR << CRPT_AES_CTL_OPMODE_Pos,
                              zero_block,
                              operation->stream_block,
                              NVT_PSA_AES_BLOCK_SIZE);

            if (status != PSA_SUCCESS)
            {
                return status;
            }

            operation->ctr_exhausted =
                (uint8_t) aes_ctr_increment_counter(operation->iv);
            operation->iv_off = 0u;
        }

        available = NVT_PSA_AES_BLOCK_SIZE - operation->iv_off;
        chunk = (length - offset < available) ? length - offset : available;

        for (index = 0u; index < chunk; index++)
        {
            output[offset + index] = input[offset + index] ^
                                     operation->stream_block[operation->iv_off + index];
        }

        operation->iv_off += chunk;
        offset += chunk;

        if (operation->iv_off == NVT_PSA_AES_BLOCK_SIZE)
        {
            operation->iv_off = NVT_PSA_AES_BLOCK_SIZE;
        }
    }

    return PSA_SUCCESS;
}


/**
 * \brief  Initialize an AES operation context with key and algorithm settings.
 *
 *         Supported algorithms: PSA_ALG_ECB_NO_PADDING, PSA_ALG_CBC_NO_PADDING,
 *         PSA_ALG_CTR, PSA_ALG_CFB, PSA_ALG_OFB.
 *         Supported key sizes: 128, 192, 256 bits.
 *
 * \param operation    Context to initialize.  Must not be \c NULL.
 * \param attributes   Key attributes carrying type and bit-size.  Must not be \c NULL.
 * \param key          Raw AES key bytes.  Must not be \c NULL.
 * \param key_length   Length of \p key in bytes; must equal \p key_bits / 8.
 * \param alg          PSA algorithm identifier for this operation.
 * \param is_encrypt   Non-zero for encryption, zero for decryption.
 *
 * \return  \c PSA_SUCCESS on success.
 * \return  \c PSA_ERROR_NOT_SUPPORTED if \p alg is not one of the four supported modes.
 * \return  \c PSA_ERROR_INVALID_ARGUMENT if any pointer is \c NULL or key parameters
 *          are inconsistent.
 */
psa_status_t nvt_psa_aes_setup(nvt_psa_aes_operation_t *operation,
                               const psa_key_attributes_t *attributes,
                               const uint8_t *key,
                               size_t key_length,
                               psa_algorithm_t alg,
                               uint8_t is_encrypt)
{
    psa_key_type_t key_type;
    size_t key_bits;

    if (operation == NULL || attributes == NULL || key == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (alg != PSA_ALG_CBC_NO_PADDING &&
            alg != PSA_ALG_ECB_NO_PADDING &&
            alg != PSA_ALG_CTR &&
            alg != PSA_ALG_OFB &&
            alg != PSA_ALG_CFB)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    key_type = psa_get_key_type(attributes);
    key_bits = psa_get_key_bits(attributes);

    if (key_type != PSA_KEY_TYPE_AES ||
            (key_bits != 128u && key_bits != 192u && key_bits != 256u) ||
            key_length != key_bits / 8u)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    mbedtls_platform_zeroize(operation, sizeof(*operation));
    memcpy(operation->key, key, key_length);
    operation->alg = alg;
    operation->key_bits = (uint32_t) key_bits;
    operation->iv_off = 0u;
    operation->is_encrypt = is_encrypt;
    operation->active = 1u;
    operation->iv_set = (alg == PSA_ALG_ECB_NO_PADDING) ? 1u : 0u;

    if (alg == PSA_ALG_CTR)
    {
        operation->iv_off = NVT_PSA_AES_BLOCK_SIZE;
    }

    return PSA_SUCCESS;
}

/**
 * \brief  Set the initialization vector for a non-ECB AES operation.
 *
 *         Must be called after nvt_psa_aes_setup() and before the first
 *         nvt_psa_aes_update() call.  Not valid for ECB mode.
 *
 * \param operation    Active AES operation context.
 * \param iv           IV bytes.  Must not be \c NULL.
 * \param iv_length    Must equal \c NVT_PSA_AES_BLOCK_SIZE (16).
 *
 * \return  \c PSA_SUCCESS on success.
 * \return  \c PSA_ERROR_BAD_STATE if the context is not active.
 * \return  \c PSA_ERROR_INVALID_ARGUMENT if called for ECB mode, or if
 *          \p iv is \c NULL or \p iv_length != 16.
 */
psa_status_t nvt_psa_aes_set_iv(nvt_psa_aes_operation_t *operation,
                                const uint8_t *iv,
                                size_t iv_length)
{
    if (operation == NULL || operation->active == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (operation->alg == PSA_ALG_ECB_NO_PADDING)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (iv == NULL || iv_length != NVT_PSA_AES_BLOCK_SIZE)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    memcpy(operation->iv, iv, NVT_PSA_AES_BLOCK_SIZE);

    if (operation->alg == PSA_ALG_CTR)
    {
        mbedtls_platform_zeroize(operation->stream_block,
                                 sizeof(operation->stream_block));
        operation->iv_off = NVT_PSA_AES_BLOCK_SIZE;
        operation->ctr_exhausted = 0u;
    }

    operation->iv_set = 1u;
    return PSA_SUCCESS;
}

/**
 * \brief  Process input data blocks for the active AES operation.
 *
 *         Dispatches to the appropriate mode-specific hardware function:
 *         - ECB:  aes_crypt_ecb()    — block-by-block, no IV state.
 *         - CBC:  aes_crypt_cbc()    — IV updated on every call.
 *         - CFB:  aes_crypt_cfb128() — IV updated on every call.
 *         - OFB:  aes_crypt_ofb()    — IV updated on every call.
 *         - CTR:  aes_crypt_ctr()    — counter and partial keystream retained.
 *
 *         ECB and CBC require \p input_length to be a multiple of 16.
 *         CFB and OFB accept any length.
 *
 * \param operation      Active AES operation context.
 * \param input          Input data buffer.
 * \param input_length   Number of bytes to process.
 * \param output         Output data buffer; must hold at least \p input_length bytes.
 * \param output_size    Size of \p output in bytes.
 * \param output_length  Set to the number of bytes written to \p output.
 *
 * \return  \c PSA_SUCCESS on success.
 * \return  \c PSA_ERROR_BAD_STATE if the context is not active or IV has not been set.
 * \return  \c PSA_ERROR_INVALID_ARGUMENT if lengths are inconsistent.
 * \return  \c PSA_ERROR_BUFFER_TOO_SMALL if \p output_size < \p input_length.
 * \return  \c PSA_ERROR_HARDWARE_FAILURE if the hardware engine times out.
 */
psa_status_t nvt_psa_aes_update(nvt_psa_aes_operation_t *operation,
                                const uint8_t *input,
                                size_t input_length,
                                uint8_t *output,
                                size_t output_size,
                                size_t *output_length)
{
    if (operation == NULL || output_length == NULL || operation->active == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    *output_length = 0u;

    if (input_length == 0u)
    {
        return PSA_SUCCESS;
    }

    if (input == NULL || output == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (output_size < input_length)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    if ((operation->alg == PSA_ALG_ECB_NO_PADDING ||
            operation->alg == PSA_ALG_CBC_NO_PADDING) &&
            (input_length % NVT_PSA_AES_BLOCK_SIZE) != 0u)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (operation->alg != PSA_ALG_ECB_NO_PADDING && operation->iv_set == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    *output_length = input_length;

    if (operation->alg == PSA_ALG_ECB_NO_PADDING)
    {
        return aes_crypt_ecb(operation, input, output, input_length);
    }

    if (operation->alg == PSA_ALG_CBC_NO_PADDING)
    {
        return aes_crypt_cbc(operation, input, output, input_length);
    }

    if (operation->alg == PSA_ALG_CTR)
    {
        return aes_crypt_ctr(operation, input, output, input_length);
    }

    if (operation->alg == PSA_ALG_CFB)
    {
        return aes_crypt_cfb128(operation, input, output, input_length);
    }

    if (operation->alg == PSA_ALG_OFB)
    {
        return aes_crypt_ofb(operation, input, output, input_length);
    }

    return PSA_ERROR_NOT_SUPPORTED;
}

/**
 * \brief  Finalize an AES operation.
 *
 *         This driver produces no buffered output, so \p output_length is
 *         always set to zero.  Call nvt_psa_aes_abort() afterward to clear
 *         sensitive state.
 *
 * \param operation      Active AES operation context.
 * \param output         Ignored; may be \c NULL.
 * \param output_size    Ignored.
 * \param output_length  Set to 0 on return.
 *
 * \return  \c PSA_SUCCESS on success.
 * \return  \c PSA_ERROR_BAD_STATE if the context is not active.
 */
psa_status_t nvt_psa_aes_finish(nvt_psa_aes_operation_t *operation,
                                uint8_t *output,
                                size_t output_size,
                                size_t *output_length)
{
    if (operation == NULL || output_length == NULL || operation->active == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    (void) output;
    (void) output_size;
    *output_length = 0u;
    return PSA_SUCCESS;
}

/**
 * \brief  Abort an AES operation and zeroize all sensitive state.
 *
 *         Safe to call on a context in any state, including after a failed
 *         setup or update.  Passing \c NULL is a no-op.
 *
 * \param operation  AES operation context to clear.
 *
 * \return  Always \c PSA_SUCCESS.
 */
psa_status_t nvt_psa_aes_abort(nvt_psa_aes_operation_t *operation)
{
    if (operation != NULL)
    {
        mbedtls_platform_zeroize(operation, sizeof(*operation));
    }

    return PSA_SUCCESS;
}


/* NIST test vectors for AES ECB (all zeros key and plaintext stress test) */
/* Reference: aes.c self-test - 10000 iterations of ECB with zero key/plaintext */
static const unsigned char aes_test_ecb_enc[3][16] =
{
    /* 128-bit key expected output */
    {
        0xC3, 0x4C, 0x05, 0x2C, 0xC0, 0xDA, 0x8D, 0x73,
        0x45, 0x1A, 0xFE, 0x5F, 0x03, 0xBE, 0x29, 0x7F
    },
    /* 192-bit key expected output */
    {
        0xF3, 0xF6, 0x75, 0x2A, 0xE8, 0xD7, 0x83, 0x11,
        0x38, 0xF0, 0x41, 0x56, 0x06, 0x31, 0xB1, 0x14
    },
    /* 256-bit key expected output */
    {
        0x8B, 0x79, 0xEE, 0xCC, 0x93, 0xA0, 0xEE, 0x5D,
        0xFF, 0x30, 0xB4, 0xEA, 0x21, 0x63, 0x6D, 0xA4
    }
};


static const unsigned char aes_test_ecb_dec[3][16] =
{
    {
        0x44, 0x41, 0x6A, 0xC2, 0xD1, 0xF5, 0x3C, 0x58,
        0x33, 0x03, 0x91, 0x7E, 0x6B, 0xE9, 0xEB, 0xE0
    },

    {
        0x48, 0xE3, 0x1E, 0x9E, 0x25, 0x67, 0x18, 0xF2,
        0x92, 0x29, 0x31, 0x9C, 0x19, 0xF1, 0x5B, 0xA4
    },

    {
        0x05, 0x8C, 0xCF, 0xFD, 0xBB, 0xCB, 0x38, 0x2D,
        0x1F, 0x6F, 0x56, 0x58, 0x5D, 0x8A, 0x4A, 0xDE
    }

};


/* Test data structure */
struct
{
    uint32_t key_bits;
    const char *key_str;
    int expected_idx;
} key_configs[] =
{
    { 128, "128-bit", 0 },
    { 192, "192-bit", 1 },
    { 256, "256-bit", 2 }
};

/**
 * Print hex buffer
 */
static void print_hex(const unsigned char *data, size_t len, const char *prefix)
{
    printf("%s", prefix);

    for (size_t i = 0; i < len; i++)
    {
        printf("%02X ", data[i]);

        if ((i + 1) % 16 == 0 && i + 1 < len)
        {
            printf("\n         ");
        }
    }

    printf("\n");
}

/**
 * Test AES ECB encryption with 10000 iterations
 *
 * This test mimics the NIST reference implementation:
 * - Initialize all-zero key and plaintext
 * - Perform 10000 ECB encryptions
 * - Compare with expected test vectors
 */
static int test_aes_ecb_encrypt(uint32_t key_bits)
{
    int ret;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    nvt_psa_aes_operation_t operation;
    uint8_t key[32] = {0};
    uint8_t buf[16] = {0};
    uint8_t output[16];
    size_t output_len;
    int key_idx = (key_bits == 128) ? 0 : (key_bits == 192) ? 1 : 2;

    printf("\n=== AES ECB Stress Test (%u-bit key) ===\n", key_bits);

    /* Setup PSA attributes */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECB_NO_PADDING);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, key_bits);

    /* Initialize EC operation with all-zero key */
    ret = nvt_psa_aes_setup(&operation, &attributes, key, key_bits / 8,
                            PSA_ALG_ECB_NO_PADDING, 1u);

    if (ret != PSA_SUCCESS)
    {
        printf("ERROR: nvt_psa_aes_setup failed (0x%x)\n", ret);
        return 0;
    }

    /* Stress test: 10000 ECB encryptions with all-zero input */
    for (int i = 0; i < 10000; i++)
    {
        ret = nvt_psa_aes_update(&operation, buf, 16, output, 16, &output_len);

        if (ret != PSA_SUCCESS)
        {
            printf("ERROR: nvt_psa_aes_update failed at iteration %d (0x%x)\n", i, ret);
            nvt_psa_aes_abort(&operation);
            return 0;
        }

        memcpy(buf, output, 16);  /* Feedback for next iteration */
    }

    /* Finalize */
    ret = nvt_psa_aes_finish(&operation, NULL, 0, &output_len);

    if (ret != PSA_SUCCESS)
    {
        printf("ERROR: nvt_psa_aes_finish failed (0x%x)\n", ret);
        return 0;
    }

    /* Verify result */
    print_hex(buf, 16, "Result:   ");
    print_hex(aes_test_ecb_enc[key_idx], 16, "Expected: ");

    if (memcmp(buf, aes_test_ecb_enc[key_idx], 16) == 0)
    {
        printf("✓ PASS: ECB %u-bit encryption matches expected vector\n", key_bits);
        nvt_psa_aes_abort(&operation);
        return 1;
    }
    else
    {
        printf("✗ FAIL: ECB %u-bit encryption does NOT match\n", key_bits);
        nvt_psa_aes_abort(&operation);
        return 0;
    }
}

/**
 * Test AES ECB decryption with 10000 iterations
 *
 * This test mimics the NIST reference implementation:
 * - Initialize all-zero key and plaintext
 * - Perform 10000 ECB decryptions
 * - Compare with expected test vectors
 */
static int test_aes_ecb_decrypt(uint32_t key_bits)
{
    int ret;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    nvt_psa_aes_operation_t operation;
    uint8_t key[32] = {0};
    uint8_t ciphertext[16];
    uint8_t output[16];
    size_t output_len;
    int key_idx = (key_bits == 128) ? 0 : (key_bits == 192) ? 1 : 2;

    printf("\n=== AES ECB Decryption Stress Test (%u-bit key) ===\n", key_bits);

    /* Decrypt stress-test input starts from known ECB enc vector. It's all zero from NIST tests.*/
    memset(ciphertext, 0, 16);

    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECB_NO_PADDING);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, key_bits);

    ret = nvt_psa_aes_setup(&operation, &attributes, key, key_bits / 8,
                            PSA_ALG_ECB_NO_PADDING, 0u);

    if (ret != PSA_SUCCESS)
    {
        printf("ERROR: nvt_psa_aes_setup failed (0x%x)\n", ret);
        return 0;
    }

    /* Stress test: 10000 ECB decryptions */
    for (int i = 0; i < 10000; i++)
    {
        ret = nvt_psa_aes_update(&operation, ciphertext, 16, output, 16, &output_len);

        if (ret != PSA_SUCCESS)
        {
            printf("ERROR: nvt_psa_aes_update failed at iteration %d (0x%x)\n", i, ret);
            nvt_psa_aes_abort(&operation);
            return 0;
        }

        memcpy(ciphertext, output, 16);
    }

    ret = nvt_psa_aes_finish(&operation, NULL, 0, &output_len);

    if (ret != PSA_SUCCESS)
    {
        printf("ERROR: nvt_psa_aes_finish failed (0x%x)\n", ret);
        nvt_psa_aes_abort(&operation);
        return 0;
    }

    print_hex(output, 16, "Result:   ");
    print_hex(aes_test_ecb_dec[key_idx], 16, "Expected: ");

    if (memcmp(output, aes_test_ecb_dec[key_idx], 16) == 0)
    {
        printf("PASS: ECB %u-bit decrypt stress(10000) matches expected vector\n", key_bits);
        nvt_psa_aes_abort(&operation);
        return 1;
    }
    else
    {
        printf("FAIL: ECB %u-bit decrypt stress(1000) mismatch\n", key_bits);
        nvt_psa_aes_abort(&operation);
        return 0;
    }
}

/**
 * Main test entry point
 */
int nvt_aes_self_test(void)
{
    int passed = 0, total = 0;

    printf("=========================================\n");
    printf("  AES ECB Mode Verification Test Suite\n");
    printf("=========================================\n");

    /* Initialize crypto library */
    if (psa_crypto_init() != PSA_SUCCESS)
    {
        printf("ERROR: PSA crypto initialization failed\n");
        return 1;
    }

    /* Test all key sizes */
    for (int i = 0; i < 3; i++)
    {
        total++;

        if (test_aes_ecb_encrypt(key_configs[i].key_bits))
        {
            passed++;
        }

        total++;

        if (test_aes_ecb_decrypt(key_configs[i].key_bits))
        {
            passed++;
        }
    }

    /* Summary */
    printf("\n=========================================\n");
    printf("  Test Results: %d/%d PASSED\n", passed, total);
    printf("=========================================\n");

    return (passed == total) ? 0 : 1;
}