/*
 * Nuvoton transparent cipher driver implementation (ECB / CBC / OFB / CFB)
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

#include "nvt_cipher_driver_entrypoints.h"
#include "mbedtls/platform.h"
#include "mbedtls/private/crypto_builtin_primitives.h"
#include "tf_psa_crypto_common.h"
#include <psa/crypto.h>
#include "nvt_psa_aes.h"
#include "nvt_crpt.h"

#if defined(MBEDTLS_PSA_NVT_CIPHER_DRIVER_ENABLED)

/* -------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------- */

psa_status_t nvt_cipher_driver_init(void)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");
    return PSA_SUCCESS;
}

void nvt_cipher_driver_free(void)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");
}

/* -------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------- */

static int is_alg_supported(psa_algorithm_t alg)
{
    return (alg == PSA_ALG_ECB_NO_PADDING ||
            alg == PSA_ALG_CBC_NO_PADDING ||
            alg == PSA_ALG_CTR ||
            alg == PSA_ALG_OFB            ||
            alg == PSA_ALG_CFB);
}

/* -------------------------------------------------------
 * Multipart interface
 * ------------------------------------------------------- */

psa_status_t nvt_cipher_transparent_cipher_encrypt_setup(
    nvt_cipher_operation_t *operation,
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");

    if (!is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    return nvt_psa_aes_setup(operation, attributes, key, key_length, alg, 1u);
}

psa_status_t nvt_cipher_transparent_cipher_decrypt_setup(
    nvt_cipher_operation_t *operation,
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");

    if (!is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    return nvt_psa_aes_setup(operation, attributes, key, key_length, alg, 0u);
}

psa_status_t nvt_cipher_transparent_cipher_set_iv(
    nvt_cipher_operation_t *operation,
    const uint8_t *iv, size_t iv_length)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");
    return nvt_psa_aes_set_iv(operation, iv, iv_length);
}

psa_status_t nvt_cipher_transparent_cipher_update(
    nvt_cipher_operation_t *operation,
    const uint8_t *input, size_t input_length,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");
    psa_status_t status;

    if (operation == NULL)
    {
        return PSA_ERROR_BAD_STATE;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = nvt_psa_aes_update(operation,
                                input,
                                input_length,
                                output,
                                output_size,
                                output_length);
    nvt_crpt_release();
    return status;
}

psa_status_t nvt_cipher_transparent_cipher_finish(
    nvt_cipher_operation_t *operation,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");
    psa_status_t status;

    if (operation == NULL)
    {
        return PSA_ERROR_BAD_STATE;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = nvt_psa_aes_finish(operation, output, output_size, output_length);
    nvt_crpt_release();
    return status;
}

psa_status_t nvt_cipher_transparent_cipher_abort(
    nvt_cipher_operation_t *operation)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");
    return nvt_psa_aes_abort(operation);
}

/* -------------------------------------------------------
 * Single-shot interface (composed from multipart)
 * ------------------------------------------------------- */

static psa_status_t cipher_encrypt_unlocked(
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg,
    const uint8_t *iv, size_t iv_length,
    const uint8_t *input, size_t input_length,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");
    nvt_psa_aes_operation_t operation;
    psa_status_t status;
    size_t update_length = 0u;
    size_t finish_length = 0u;

    if (!is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    status = nvt_psa_aes_setup(&operation, attributes, key, key_length, alg, 1u);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    if (alg == PSA_ALG_CBC_NO_PADDING)
    {
        status = nvt_psa_aes_set_iv(&operation, iv, iv_length);

        if (status != PSA_SUCCESS)
        {
            (void) nvt_psa_aes_abort(&operation);
            return status;
        }
    }

    status = nvt_psa_aes_update(&operation,
                                input,
                                input_length,
                                output,
                                output_size,
                                &update_length);

    if (status != PSA_SUCCESS)
    {
        (void) nvt_psa_aes_abort(&operation);
        return status;
    }

    status = nvt_psa_aes_finish(&operation,
                                output + update_length,
                                output_size - update_length,
                                &finish_length);

    if (status == PSA_SUCCESS)
    {
        *output_length = update_length + finish_length;
    }

    (void) nvt_psa_aes_abort(&operation);
    return status;
}

static psa_status_t cipher_decrypt_unlocked(
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg,
    const uint8_t *input, size_t input_length,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    PSA_DEBUG_LOG("nvt-cipher", "\r\n");
    nvt_psa_aes_operation_t operation;
    psa_status_t status;
    size_t update_length = 0u;
    size_t finish_length = 0u;
    const uint8_t *ciphertext = input;
    size_t ciphertext_length = input_length;

    if (!is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    status = nvt_psa_aes_setup(&operation, attributes, key, key_length, alg, 0u);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    if (alg == PSA_ALG_CBC_NO_PADDING)
    {
        if (input == NULL || input_length < NVT_PSA_AES_BLOCK_SIZE)
        {
            (void) nvt_psa_aes_abort(&operation);
            return PSA_ERROR_INVALID_ARGUMENT;
        }

        status = nvt_psa_aes_set_iv(&operation, input, NVT_PSA_AES_BLOCK_SIZE);

        if (status != PSA_SUCCESS)
        {
            (void) nvt_psa_aes_abort(&operation);
            return status;
        }

        ciphertext = input + NVT_PSA_AES_BLOCK_SIZE;
        ciphertext_length = input_length - NVT_PSA_AES_BLOCK_SIZE;
    }

    status = nvt_psa_aes_update(&operation,
                                ciphertext,
                                ciphertext_length,
                                output,
                                output_size,
                                &update_length);

    if (status != PSA_SUCCESS)
    {
        (void) nvt_psa_aes_abort(&operation);
        return status;
    }

    status = nvt_psa_aes_finish(&operation,
                                output + update_length,
                                output_size - update_length,
                                &finish_length);

    if (status == PSA_SUCCESS)
    {
        *output_length = update_length + finish_length;
    }

    (void) nvt_psa_aes_abort(&operation);
    return status;
}

psa_status_t nvt_cipher_transparent_cipher_encrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg,
    const uint8_t *iv, size_t iv_length,
    const uint8_t *input, size_t input_length,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    psa_status_t status;

    if (!is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = cipher_encrypt_unlocked(attributes, key, key_length, alg, iv, iv_length,
                                     input, input_length, output, output_size, output_length);
    nvt_crpt_release();
    return status;
}

psa_status_t nvt_cipher_transparent_mac_compute(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *mac,
    size_t mac_size,
    size_t *mac_length)
{
    return nvt_psa_cmac_compute(attributes, key_buffer, key_buffer_size,
                                alg, input, input_length,
                                mac, mac_size, mac_length);
}

psa_status_t nvt_cipher_transparent_cipher_decrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg,
    const uint8_t *input, size_t input_length,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    psa_status_t status;

    if (!is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = cipher_decrypt_unlocked(attributes, key, key_length, alg,
                                     input, input_length, output, output_size, output_length);
    nvt_crpt_release();
    return status;
}

#endif /* MBEDTLS_PSA_NVT_CIPHER_DRIVER_ENABLED */
