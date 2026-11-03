/*
 * Nuvoton transparent AEAD driver entrypoints.
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

#include "nvt_aead_driver_entrypoints.h"
#include "mbedtls/platform.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/private/crypto_builtin_primitives.h"
#include "tf_psa_crypto_common.h"
#include <psa/crypto.h>
#include "nvt_psa_ccm.h"
#include "nvt_psa_gcm.h"
#include "nvt_crpt.h"

#if defined(MBEDTLS_PSA_NVT_AEAD_DRIVER_ENABLED)

psa_status_t nvt_aead_driver_init(void)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");
    return PSA_SUCCESS;
}

void nvt_aead_driver_free(void)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");
}

static int nvt_aead_is_ccm(psa_algorithm_t alg)
{
    return PSA_ALG_AEAD_WITH_SHORTENED_TAG(alg, 0) ==
           PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 0);
}

static int nvt_aead_is_gcm(psa_algorithm_t alg)
{
    return PSA_ALG_AEAD_WITH_SHORTENED_TAG(alg, 0) ==
           PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, 0);
}

static int nvt_aead_is_alg_supported(psa_algorithm_t alg)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");
    return nvt_aead_is_ccm(alg) || nvt_aead_is_gcm(alg);
}

//////psa_status_t nvt_aead_init_param_by_alg(nvt_aead_operation_t *operation,
//////                                                const psa_key_attributes_t *attributes,
//////                                                const uint8_t *key,
//////                                                size_t key_length,
//////                                                psa_algorithm_t alg)
//////{
//////    memset(operation, 0, sizeof(*operation));

//////    operation->common.alg = alg;
//////    operation->common.key_type = psa_get_key_type(attributes);
//////    operation->common.key_bits = psa_get_key_bits(attributes);
//////    operation->common.is_encrypt = 1u;
//////    operation->common.tag_length = (uint8_t) PSA_ALG_AEAD_GET_TAG_LENGTH(alg);
//////    operation->common.active = 1u;

//////    memcpy(operation->common.key, key, key_length);
//////    PSA_DEBUG_LOG("nvt-aead", "\r\n");

//////    if (nvt_aead_is_ccm(alg))
//////    {
//////        operation->common.ctx_type = NVT_AEAD_CTX_CCM;
//////            PSA_DEBUG_LOG("nvt-aead", "CCM supported.\r\n");
//////
//////        return PSA_SUCCESS;//nvt_psa_ccm_setup(operation, attributes, key, key_length, alg, 1u);
//////    }

//////    if (nvt_aead_is_gcm(alg))
//////    {
//////        operation->common.ctx_type = NVT_AEAD_CTX_GCM;
////////        psa_nvt_gcm_init(&operation->ctx.gcm.gcm);

////////        if (psa_nvt_gcm_setkey(&operation->ctx.gcm.gcm,
////////                               MBEDTLS_CIPHER_ID_AES,
////////                               key,
////////                               (unsigned int) operation->common.key_bits) != 0)
////////        {
////////            return PSA_ERROR_HARDWARE_FAILURE;
////////        }

////////        return PSA_SUCCESS;
//////              PSA_DEBUG_LOG("nvt-aead", "GCM is working on porting!\r\n");

//////              return PSA_ERROR_HARDWARE_FAILURE;//working on porting
//////    }

//////    return PSA_ERROR_NOT_SUPPORTED;
//////}


static psa_status_t nvt_aead_encrypt_locked(const psa_key_attributes_t *attributes,
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
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (!nvt_aead_is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (nvt_aead_is_gcm(alg))
    {
        return nvt_psa_gcm_encrypt(attributes,
                                   key,
                                   key_length,
                                   alg,
                                   nonce,
                                   nonce_length,
                                   additional_data,
                                   additional_data_length,
                                   plaintext,
                                   plaintext_length,
                                   ciphertext,
                                   ciphertext_size,
                                   ciphertext_length);
    }

    return nvt_psa_ccm_encrypt(attributes,
                               key,
                               key_length,
                               alg,
                               nonce,
                               nonce_length,
                               additional_data,
                               additional_data_length,
                               plaintext,
                               plaintext_length,
                               ciphertext,
                               ciphertext_size,
                               ciphertext_length);
}

static psa_status_t nvt_aead_decrypt_locked(const psa_key_attributes_t *attributes,
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
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (!nvt_aead_is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (nvt_aead_is_gcm(alg))
    {
        return nvt_psa_gcm_decrypt(attributes,
                                   key,
                                   key_length,
                                   alg,
                                   nonce,
                                   nonce_length,
                                   additional_data,
                                   additional_data_length,
                                   ciphertext,
                                   ciphertext_length,
                                   plaintext,
                                   plaintext_size,
                                   plaintext_length);
    }

    return nvt_psa_ccm_decrypt(attributes,
                               key,
                               key_length,
                               alg,
                               nonce,
                               nonce_length,
                               additional_data,
                               additional_data_length,
                               ciphertext,
                               ciphertext_length,
                               plaintext,
                               plaintext_size,
                               plaintext_length);
}

static psa_status_t nvt_aead_validate(const psa_key_attributes_t *attributes,
                                      const uint8_t *key, size_t key_length,
                                      psa_algorithm_t alg,
                                      const uint8_t *nonce, size_t nonce_length,
                                      const uint8_t *additional_data,
                                      size_t additional_data_length,
                                      size_t payload_length)
{
    size_t key_bits;
    size_t tag_length;
    size_t aad_blocks;
    size_t payload_blocks;

    if (attributes == NULL || key == NULL || nonce == NULL ||
            (additional_data == NULL && additional_data_length != 0U))
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (!nvt_aead_is_alg_supported(alg) ||
            psa_get_key_type(attributes) != PSA_KEY_TYPE_AES)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    key_bits = psa_get_key_bits(attributes);

    if ((key_bits != 128U && key_bits != 192U && key_bits != 256U) ||
            key_length != key_bits / 8U)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    tag_length = PSA_ALG_AEAD_GET_TAG_LENGTH(alg);

    if (nvt_aead_is_ccm(alg))
    {
        if (nonce_length < 7U || nonce_length > 13U ||
                tag_length < 4U || tag_length > 16U || (tag_length & 1U) != 0U)
        {
            return PSA_ERROR_INVALID_ARGUMENT;
        }

        if (additional_data_length == 0U ||
                additional_data_length > MAX_CCM_BUF - 2U ||
                payload_length > MAX_CCM_BUF)
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }

        aad_blocks = (additional_data_length + 2U + 15U) / 16U;
        payload_blocks = (payload_length + 15U) / 16U;

        if (1U + aad_blocks + payload_blocks > MAX_CCM_BUF / 16U)
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }
    }
    else
    {
        if (nonce_length == 0U ||
                (tag_length != 4U && tag_length != 8U &&
                 (tag_length < 12U || tag_length > 16U)))
        {
            return PSA_ERROR_INVALID_ARGUMENT;
        }

        if (nonce_length != 12U || additional_data_length > MAX_GCM_BUF ||
                payload_length > MAX_GCM_BUF)
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }

        aad_blocks = (additional_data_length + 15U) / 16U;
        payload_blocks = (payload_length + 15U) / 16U;

        if (2U + aad_blocks + payload_blocks > MAX_GCM_BUF / 16U)
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_aead_transparent_encrypt(const psa_key_attributes_t *attributes,
                                          const uint8_t *key, size_t key_length,
                                          psa_algorithm_t alg,
                                          const uint8_t *nonce, size_t nonce_length,
                                          const uint8_t *additional_data,
                                          size_t additional_data_length,
                                          const uint8_t *plaintext, size_t plaintext_length,
                                          uint8_t *ciphertext, size_t ciphertext_size,
                                          size_t *ciphertext_length)
{
    static const uint8_t empty_input = 0;
    psa_status_t status;
    size_t tag_length;

    if (ciphertext_length == NULL)
        return PSA_ERROR_INVALID_ARGUMENT;

    *ciphertext_length = 0U;

    if (ciphertext == NULL || (plaintext == NULL && plaintext_length != 0U))
        return PSA_ERROR_INVALID_ARGUMENT;

    status = nvt_aead_validate(attributes, key, key_length, alg, nonce, nonce_length,
                               additional_data, additional_data_length, plaintext_length);

    if (status != PSA_SUCCESS)
        return status;

    tag_length = PSA_ALG_AEAD_GET_TAG_LENGTH(alg);

    if (plaintext_length > ciphertext_size ||
            tag_length > ciphertext_size - plaintext_length)
        return PSA_ERROR_BUFFER_TOO_SMALL;

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
        return status;

    status = nvt_aead_encrypt_locked(attributes, key, key_length, alg, nonce, nonce_length,
                                     additional_data != NULL ? additional_data : &empty_input,
                                     additional_data_length,
                                     plaintext != NULL ? plaintext : &empty_input, plaintext_length,
                                     ciphertext, ciphertext_size, ciphertext_length);
    nvt_crpt_release();

    if (status != PSA_SUCCESS)
    {
        mbedtls_platform_zeroize(ciphertext, plaintext_length + tag_length);
        *ciphertext_length = 0U;
    }

    return status;
}

psa_status_t nvt_aead_transparent_decrypt(const psa_key_attributes_t *attributes,
                                          const uint8_t *key, size_t key_length,
                                          psa_algorithm_t alg,
                                          const uint8_t *nonce, size_t nonce_length,
                                          const uint8_t *additional_data,
                                          size_t additional_data_length,
                                          const uint8_t *ciphertext, size_t ciphertext_length,
                                          uint8_t *plaintext, size_t plaintext_size,
                                          size_t *plaintext_length)
{
    static const uint8_t empty_input = 0;
    uint8_t empty_output = 0;
    psa_status_t status;
    size_t tag_length;
    size_t payload_length;

    if (plaintext_length == NULL)
        return PSA_ERROR_INVALID_ARGUMENT;

    *plaintext_length = 0U;

    if (ciphertext == NULL)
        return PSA_ERROR_INVALID_ARGUMENT;

    if (!nvt_aead_is_alg_supported(alg))
        return PSA_ERROR_NOT_SUPPORTED;

    tag_length = PSA_ALG_AEAD_GET_TAG_LENGTH(alg);

    if (ciphertext_length < tag_length)
        return PSA_ERROR_INVALID_ARGUMENT;

    payload_length = ciphertext_length - tag_length;

    if (plaintext == NULL && payload_length != 0U)
        return PSA_ERROR_INVALID_ARGUMENT;

    status = nvt_aead_validate(attributes, key, key_length, alg, nonce, nonce_length,
                               additional_data, additional_data_length, payload_length);

    if (status != PSA_SUCCESS)
        return status;

    if (payload_length > plaintext_size)
        return PSA_ERROR_BUFFER_TOO_SMALL;

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
        return status;

    status = nvt_aead_decrypt_locked(attributes, key, key_length, alg, nonce, nonce_length,
                                     additional_data != NULL ? additional_data : &empty_input,
                                     additional_data_length, ciphertext, ciphertext_length,
                                     plaintext != NULL ? plaintext : &empty_output,
                                     plaintext_size, plaintext_length);
    nvt_crpt_release();

    if (status != PSA_SUCCESS)
    {
        if (payload_length != 0U)
            mbedtls_platform_zeroize(plaintext, payload_length);

        *plaintext_length = 0U;
    }

    return status;
}

psa_status_t nvt_aead_transparent_encrypt_setup(nvt_aead_operation_t *operation,
                                                const psa_key_attributes_t *attributes,
                                                const uint8_t *key,
                                                size_t key_length,
                                                psa_algorithm_t alg)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (!nvt_aead_is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (nvt_aead_is_gcm(alg))
    {
        return nvt_psa_gcm_setup(operation, attributes, key, key_length, alg, 1u);
    }

    return nvt_psa_ccm_setup(operation, attributes, key, key_length, alg, 1u);
}

psa_status_t nvt_aead_transparent_decrypt_setup(nvt_aead_operation_t *operation,
                                                const psa_key_attributes_t *attributes,
                                                const uint8_t *key,
                                                size_t key_length,
                                                psa_algorithm_t alg)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (!nvt_aead_is_alg_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (nvt_aead_is_gcm(alg))
    {
        return nvt_psa_gcm_setup(operation, attributes, key, key_length, alg, 0u);
    }

    return nvt_psa_ccm_setup(operation, attributes, key, key_length, alg, 0u);
}

psa_status_t nvt_aead_transparent_set_nonce(nvt_aead_operation_t *operation,
                                            const uint8_t *nonce,
                                            size_t nonce_length)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (nvt_aead_is_gcm(operation->alg))
    {
        return nvt_psa_gcm_set_nonce(operation, nonce, nonce_length);
    }

    return nvt_psa_ccm_set_nonce(operation, nonce, nonce_length);
}

psa_status_t nvt_aead_transparent_set_lengths(nvt_aead_operation_t *operation,
                                              size_t additional_data_length,
                                              size_t plaintext_length)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (nvt_aead_is_gcm(operation->alg))
    {
        return nvt_psa_gcm_set_lengths(operation, additional_data_length, plaintext_length);
    }

    return nvt_psa_ccm_set_lengths(operation, additional_data_length,
                                   plaintext_length);
}

psa_status_t nvt_aead_transparent_update_ad(nvt_aead_operation_t *operation,
                                            const uint8_t *input,
                                            size_t input_length)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (nvt_aead_is_gcm(operation->alg))
    {
        return nvt_psa_gcm_update_ad(operation, input, input_length);
    }

    return nvt_psa_ccm_update_ad(operation, input, input_length);
}

psa_status_t nvt_aead_transparent_update(nvt_aead_operation_t *operation,
                                         const uint8_t *input,
                                         size_t input_length,
                                         uint8_t *output,
                                         size_t output_size,
                                         size_t *output_length)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (nvt_aead_is_gcm(operation->alg))
    {
        return nvt_psa_gcm_update(operation,
                                  input,
                                  input_length,
                                  output,
                                  output_size,
                                  output_length);
    }

    return nvt_psa_ccm_update(operation,
                              input,
                              input_length,
                              output,
                              output_size,
                              output_length);
}

psa_status_t nvt_aead_transparent_finish(nvt_aead_operation_t *operation,
                                         uint8_t *ciphertext,
                                         size_t ciphertext_size,
                                         size_t *ciphertext_length,
                                         uint8_t *tag,
                                         size_t tag_size,
                                         size_t *tag_length)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (nvt_aead_is_gcm(operation->alg))
    {
        return nvt_psa_gcm_finish(operation,
                                  ciphertext,
                                  ciphertext_size,
                                  ciphertext_length,
                                  tag,
                                  tag_size,
                                  tag_length);
    }

    return nvt_psa_ccm_finish(operation,
                              ciphertext,
                              ciphertext_size,
                              ciphertext_length,
                              tag,
                              tag_size,
                              tag_length);
}

psa_status_t nvt_aead_transparent_verify(nvt_aead_operation_t *operation,
                                         uint8_t *plaintext,
                                         size_t plaintext_size,
                                         size_t *plaintext_length,
                                         const uint8_t *tag,
                                         size_t tag_length)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (nvt_aead_is_gcm(operation->alg))
    {
        return nvt_psa_gcm_verify(operation,
                                  plaintext,
                                  plaintext_size,
                                  plaintext_length,
                                  tag,
                                  tag_length);
    }

    return nvt_psa_ccm_verify(operation,
                              plaintext,
                              plaintext_size,
                              plaintext_length,
                              tag,
                              tag_length);
}

psa_status_t nvt_aead_transparent_abort(nvt_aead_operation_t *operation)
{
    PSA_DEBUG_LOG("nvt-aead", "\r\n");

    if (nvt_aead_is_gcm(operation->alg))
    {
        return nvt_psa_gcm_abort(operation);
    }

    return nvt_psa_ccm_abort(operation);
}

#endif /* MBEDTLS_PSA_NVT_AEAD_DRIVER_ENABLED */