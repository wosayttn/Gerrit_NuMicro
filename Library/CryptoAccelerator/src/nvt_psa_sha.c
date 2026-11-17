/*
 * Nuvoton PSA SHA helper for the M2354 CRPT engine.
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

#include "nvt_psa_sha.h"

#include <string.h>

#include "mbedtls/md.h"
#include "mbedtls/platform_util.h"
#include "nvt_crpt.h"
#include "NuMicro.h"

#define NVT_PSA_SHA_TIMEOUT (0x100000)

volatile nvt_psa_sha_compute_path_t nvt_psa_sha_last_compute_path =
    NVT_PSA_SHA_COMPUTE_FAILED;

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
    volatile nvt_psa_hmac_path_t nvt_psa_hmac_last_compute_path =
    NVT_PSA_HMAC_PATH_FAILED;
#endif

static uint32_t nvt_psa_sha_mode(psa_algorithm_t alg)
{
    switch (alg)
    {
        case PSA_ALG_SHA_1:
            return SHA_MODE_SHA1;

        case PSA_ALG_SHA_224:
            return SHA_MODE_SHA224;

        case PSA_ALG_SHA_256:
            return SHA_MODE_SHA256;

        case PSA_ALG_SHA_384:
            return SHA_MODE_SHA384;

        case PSA_ALG_SHA_512:
            return SHA_MODE_SHA512;

        default:
            return SHA_MODE_SHA256;
    }
}

static int nvt_psa_sha_algorithm_supported(psa_algorithm_t alg)
{
    return alg == PSA_ALG_SHA_1 || alg == PSA_ALG_SHA_224 ||
           alg == PSA_ALG_SHA_256 || alg == PSA_ALG_SHA_384 ||
           alg == PSA_ALG_SHA_512;
}

static psa_status_t nvt_psa_hash_software_compute(psa_algorithm_t alg,
                                                  const uint8_t *input,
                                                  size_t input_length,
                                                  uint8_t *output,
                                                  size_t output_size,
                                                  size_t *output_length)
{
    mbedtls_md_type_t md_type;
    const mbedtls_md_info_t *md_info;
    size_t hash_size;

    if (output_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    hash_size = PSA_HASH_LENGTH(alg);

    if (hash_size == 0u || output == NULL || output_size < hash_size)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    switch (alg)
    {
        case PSA_ALG_SHA_1:
            md_type = MBEDTLS_MD_SHA1;
            break;

        case PSA_ALG_SHA_224:
            md_type = MBEDTLS_MD_SHA224;
            break;

        case PSA_ALG_SHA_256:
            md_type = MBEDTLS_MD_SHA256;
            break;

        case PSA_ALG_SHA_384:
            md_type = MBEDTLS_MD_SHA384;
            break;

        case PSA_ALG_SHA_512:
            md_type = MBEDTLS_MD_SHA512;
            break;

        default:
            return PSA_ERROR_NOT_SUPPORTED;
    }

    md_info = mbedtls_md_info_from_type(md_type);

    if (md_info == NULL)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (mbedtls_md(md_info, input, input_length, output) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    *output_length = hash_size;
    return PSA_SUCCESS;
}

static void nvt_psa_sha_program_state(nvt_psa_sha_operation_t *operation)
{
    CRPT->HMAC_CTL = CRPT_HMAC_CTL_STOP_Msk;
    CRPT->HMAC_SADDR = (uint32_t) &operation->buffer[0];
    CRPT->HMAC_DMACNT = NVT_PSA_SHA_BLOCK_SIZE;
    operation->ctl = CRPT_HMAC_CTL_DMAEN_Msk |
                     (nvt_psa_sha_mode(operation->alg) << CRPT_HMAC_CTL_OPMODE_Pos) |
                     CRPT_HMAC_CTL_INSWAP_Msk |
                     CRPT_HMAC_CTL_OUTSWAP_Msk;
}

static psa_status_t nvt_psa_sha_submit(nvt_psa_sha_operation_t *operation,
                                       size_t length, uint32_t last)
{
    int32_t timeout = NVT_PSA_SHA_TIMEOUT;
    uint32_t control = operation->ctl | CRPT_HMAC_CTL_START_Msk;

    if (operation->isfirst)
    {
        if (last == 0u)
        {
            control |= CRPT_HMAC_CTL_DMACSCAD_Msk |
                       CRPT_HMAC_CTL_DMAFIRST_Msk;
        }
    }
    else
    {
        control |= CRPT_HMAC_CTL_DMACSCAD_Msk;
    }

    CRPT->HMAC_DMACNT = (uint32_t) length;
    CRPT->HMAC_SADDR = (uint32_t) &operation->buffer[0];
    SHA_CLR_INT_FLAG(CRPT);
    g_SHA_error = 0U;
    g_SHA_irq_status = 0U;
    g_SHA_error_status = 0U;
    g_SHA_done = 0;
    CRPT->HMAC_CTL = control | last;

    while (!g_SHA_done)
    {
        if (--timeout == 0)
        {
            return PSA_ERROR_HARDWARE_FAILURE;
        }
    }

    while ((CRPT->HMAC_STS & (CRPT_HMAC_STS_BUSY_Msk |
                              CRPT_HMAC_STS_DMABUSY_Msk)) != 0U)
    {
        if (--timeout == 0)
        {
            return PSA_ERROR_HARDWARE_FAILURE;
        }
    }

    if ((g_SHA_error != 0U) &&
            (((g_SHA_error_status |
               CRPT->HMAC_STS) &
              (CRPT_HMAC_STS_DMAERR_Msk | CRPT_HMAC_STS_KSERR_Msk)) != 0U))
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    operation->isfirst = 0;
    return PSA_SUCCESS;
}

psa_status_t nvt_psa_sha_init(nvt_psa_sha_operation_t *operation)
{
    if (operation == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    mbedtls_platform_zeroize(operation, sizeof(*operation));
    return PSA_SUCCESS;
}

psa_status_t nvt_psa_sha_free(nvt_psa_sha_operation_t *operation)
{
    if (operation != NULL)
    {
        mbedtls_platform_zeroize(operation, sizeof(*operation));
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_psa_sha_clone(const nvt_psa_sha_operation_t *source,
                               nvt_psa_sha_operation_t *target)
{
    if (source == NULL || target == NULL || source->active == 0u ||
            target->active != 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    *target = *source;
    return PSA_SUCCESS;
}

psa_status_t nvt_psa_sha_starts(nvt_psa_sha_operation_t *operation,
                                psa_algorithm_t alg)
{
    if (operation == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (!nvt_psa_sha_algorithm_supported(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    operation->alg = alg;
    operation->is224 = (alg == PSA_ALG_SHA_224) ? 1u : 0u;
    operation->isfirst = 1;
    operation->active = 1u;
    operation->buffer_len = 0u;
    CRPT->HMAC_CTL = CRPT_HMAC_CTL_STOP_Msk;
    /* Clear HMAC key count left by a prior HMAC run; it corrupts plain SHA. */
    CRPT->HMAC_KEYCNT = 0u;
    nvt_psa_sha_program_state(operation);
    return PSA_SUCCESS;
}

psa_status_t nvt_psa_sha_update(nvt_psa_sha_operation_t *operation,
                                const uint8_t *input,
                                size_t input_length)
{
    size_t offset = 0u;

    if (operation == NULL || operation->active == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    if (input_length == 0u)
    {
        return PSA_SUCCESS;
    }

    if (input == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    while (input_length > 0u)
    {
        size_t space;
        size_t chunk;
        psa_status_t status;

        if (operation->buffer_len == NVT_PSA_SHA_BLOCK_SIZE)
        {
            status = nvt_psa_sha_submit(operation,
                                        NVT_PSA_SHA_BLOCK_SIZE,
                                        0u);

            if (status != PSA_SUCCESS)
            {
                return status;
            }

            operation->buffer_len = 0u;
        }

        space = NVT_PSA_SHA_BLOCK_SIZE - operation->buffer_len;
        chunk = (input_length < space) ? input_length : space;
        memcpy(operation->buffer + operation->buffer_len, input + offset, chunk);
        operation->buffer_len += (uint32_t) chunk;
        offset += chunk;
        input_length -= chunk;
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_psa_sha_finish(nvt_psa_sha_operation_t *operation,
                                uint8_t *output,
                                size_t output_size,
                                size_t *output_length)
{
    size_t actual_hash_length;
    psa_status_t status;

    if (operation == NULL || output_length == NULL || operation->active == 0u)
    {
        return PSA_ERROR_BAD_STATE;
    }

    *output_length = 0u;
    actual_hash_length = PSA_HASH_LENGTH(operation->alg);

    if (output == NULL && output_size != 0u)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (output_size < actual_hash_length)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    status = nvt_psa_sha_submit(operation, operation->buffer_len,
                                CRPT_HMAC_CTL_DMALAST_Msk);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    memcpy(output, (uint8_t *) &CRPT->HMAC_DGST[0], actual_hash_length);
    *output_length = actual_hash_length;
    operation->buffer_len = 0u;
    return PSA_SUCCESS;
}

psa_status_t nvt_psa_sha_compute(psa_algorithm_t alg,
                                 const uint8_t *input,
                                 size_t input_length,
                                 uint8_t *output,
                                 size_t output_size,
                                 size_t *output_length)
{
    nvt_psa_sha_operation_t operation;
    psa_status_t status;

    nvt_psa_sha_last_compute_path = NVT_PSA_SHA_COMPUTE_FAILED;
    status = nvt_psa_sha_init(&operation);

    if (status == PSA_SUCCESS)
    {
        status = nvt_psa_sha_starts(&operation, alg);
    }

    if (status == PSA_SUCCESS)
    {
        status = nvt_psa_sha_update(&operation, input, input_length);
    }

    if (status == PSA_SUCCESS)
    {
        status = nvt_psa_sha_finish(&operation, output, output_size,
                                    output_length);
    }

    (void) nvt_psa_sha_free(&operation);

    if (status == PSA_ERROR_HARDWARE_FAILURE)
    {
        status = nvt_psa_hash_software_compute(alg,
                                               input,
                                               input_length,
                                               output,
                                               output_size,
                                               output_length);

        if (status == PSA_SUCCESS)
        {
            nvt_psa_sha_last_compute_path =
                NVT_PSA_SHA_COMPUTE_SOFTWARE_FALLBACK;
        }

        return status;
    }

    if (status == PSA_SUCCESS)
    {
        nvt_psa_sha_last_compute_path = NVT_PSA_SHA_COMPUTE_HARDWARE;
    }

    return status;
}

#define NVT_PSA_HMAC_DMA_SIZE 1024u
#define NVT_PSA_HMAC_TIMEOUT 0x100000

static __ALIGNED(32) uint8_t s_hmac_dma[NVT_PSA_HMAC_DMA_SIZE];

static int nvt_psa_hmac_mode(psa_algorithm_t hash_alg,
                             uint32_t *mode,
                             size_t *hash_size,
                             size_t *block_size)
{
    *hash_size = PSA_HASH_LENGTH(hash_alg);
    *block_size = PSA_HASH_BLOCK_LENGTH(hash_alg);

    switch (hash_alg)
    {
        case PSA_ALG_SHA_1:
            *mode = HMAC_MODE_SHA1;
            return 1;

        case PSA_ALG_SHA_224:
            *mode = HMAC_MODE_SHA224;
            return 1;

        case PSA_ALG_SHA_256:
            *mode = HMAC_MODE_SHA256;
            return 1;

        case PSA_ALG_SHA_384:
            *mode = HMAC_MODE_SHA384;
            return 1;

        case PSA_ALG_SHA_512:
            *mode = HMAC_MODE_SHA512;
            return 1;

        default:
            return 0;
    }
}

static psa_status_t nvt_psa_hmac_software_compute(psa_algorithm_t hash_alg,
                                                  const uint8_t *key,
                                                  size_t key_length,
                                                  const uint8_t *input,
                                                  size_t input_length,
                                                  uint8_t *mac,
                                                  size_t hash_size,
                                                  size_t *mac_length)
{
    mbedtls_md_type_t md_type;
    const mbedtls_md_info_t *md_info;

    switch (hash_alg)
    {
        case PSA_ALG_SHA_1:
            md_type = MBEDTLS_MD_SHA1;
            break;

        case PSA_ALG_SHA_224:
            md_type = MBEDTLS_MD_SHA224;
            break;

        case PSA_ALG_SHA_256:
            md_type = MBEDTLS_MD_SHA256;
            break;

        case PSA_ALG_SHA_384:
            md_type = MBEDTLS_MD_SHA384;
            break;

        case PSA_ALG_SHA_512:
            md_type = MBEDTLS_MD_SHA512;
            break;

        default:
            return PSA_ERROR_NOT_SUPPORTED;
    }

    md_info = mbedtls_md_info_from_type(md_type);

    if (md_info == NULL)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (mbedtls_md_hmac(md_info, key, key_length, input, input_length, mac) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    *mac_length = hash_size;
    return PSA_SUCCESS;
}

static psa_status_t nvt_psa_hmac_submit(uint32_t control, size_t length)
{
    int32_t timeout = NVT_PSA_HMAC_TIMEOUT;

    CRPT->HMAC_SADDR = (uint32_t) s_hmac_dma;
    CRPT->HMAC_DMACNT = (uint32_t) length;
    SHA_CLR_INT_FLAG(CRPT);
    g_SHA_error = 0U;
    g_SHA_irq_status = 0U;
    g_SHA_error_status = 0U;
    g_SHA_done = 0;
    CRPT->HMAC_CTL = control | CRPT_HMAC_CTL_START_Msk;

    while (!g_SHA_done)
    {
        if (--timeout == 0)
        {
            return PSA_ERROR_HARDWARE_FAILURE;
        }
    }

    while ((CRPT->HMAC_STS & (CRPT_HMAC_STS_BUSY_Msk |
                              CRPT_HMAC_STS_DMABUSY_Msk)) != 0U)
    {
        if (--timeout == 0)
        {
            return PSA_ERROR_HARDWARE_FAILURE;
        }
    }

    if ((g_SHA_error != 0U) &&
            (((g_SHA_error_status |
               CRPT->HMAC_STS) &
              (CRPT_HMAC_STS_DMAERR_Msk | CRPT_HMAC_STS_KSERR_Msk)) != 0U))
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_psa_hmac_compute(
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
    psa_algorithm_t hash_alg;
    psa_status_t status;
    uint32_t mode;
    size_t hash_size;
    size_t block_size;
    size_t key_aligned;
    size_t total_length;
    uint8_t digest[PSA_HASH_MAX_SIZE];
    uint32_t control;

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
    nvt_psa_hmac_last_compute_path = NVT_PSA_HMAC_PATH_FAILED;
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

    if (psa_get_key_type(attributes) != PSA_KEY_TYPE_HMAC ||
            !PSA_ALG_IS_HMAC(alg) || PSA_ALG_FULL_LENGTH_MAC(alg) != alg)
    {
#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
        nvt_psa_hmac_last_compute_path = NVT_PSA_HMAC_PATH_BUILTIN_FALLBACK;
#endif
        return PSA_ERROR_NOT_SUPPORTED;
    }

    hash_alg = PSA_ALG_HMAC_GET_HASH(alg);

    if (!nvt_psa_hmac_mode(hash_alg, &mode, &hash_size, &block_size) ||
            hash_size == 0u || block_size == 0u || key_length == 0u ||
            key_length > block_size)
    {
#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
        nvt_psa_hmac_last_compute_path = NVT_PSA_HMAC_PATH_BUILTIN_FALLBACK;
#endif
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (key_length != (psa_get_key_bits(attributes) + 7u) / 8u)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (mac_size < hash_size)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    key_aligned = (key_length + 3u) & ~(size_t) 3u;

    if (key_aligned >= NVT_PSA_HMAC_DMA_SIZE)
    {
#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
        nvt_psa_hmac_last_compute_path = NVT_PSA_HMAC_PATH_BUILTIN_FALLBACK;
#endif
        return PSA_ERROR_NOT_SUPPORTED;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    /*
     * Avoid programming the CRPT HMAC DMA on the exact staging-buffer edge.
     * Board tests show edge-length requests can still trip a hardware error,
     * so we keep one-byte safety slack and fall back to software instead.
     */
    if (input_length >= NVT_PSA_HMAC_DMA_SIZE - key_aligned)
    {
        nvt_crpt_release();
#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
        nvt_psa_hmac_last_compute_path = NVT_PSA_HMAC_PATH_SOFTWARE_FALLBACK;
#endif
        return nvt_psa_hmac_software_compute(hash_alg,
                                             key,
                                             key_length,
                                             input,
                                             input_length,
                                             mac,
                                             hash_size,
                                             mac_length);
    }

    total_length = key_aligned + input_length;
    memset(s_hmac_dma, 0, key_aligned);
    memcpy(s_hmac_dma, key, key_length);

    if (input_length != 0u)
    {
        memcpy(s_hmac_dma + key_aligned, input, input_length);
    }

    CRPT->HMAC_CTL = CRPT_HMAC_CTL_STOP_Msk;
    CRPT->HMAC_KEYCNT = (uint32_t) key_length;
    control = (mode << CRPT_HMAC_CTL_OPMODE_Pos) |
              CRPT_HMAC_CTL_DMAEN_Msk |
              CRPT_HMAC_CTL_INSWAP_Msk |
              CRPT_HMAC_CTL_OUTSWAP_Msk;

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

    if (nvt_psa_test_force_hardware_failure != 0u)
    {
        status = PSA_ERROR_HARDWARE_FAILURE;
    }
    else
#endif
    {
        status = nvt_psa_hmac_submit(control | CRPT_HMAC_CTL_DMALAST_Msk,
                                     total_length);
    }

    if (status == PSA_SUCCESS)
    {
        memcpy(digest, (const uint8_t *) &CRPT->HMAC_DGST[0], hash_size);
        memcpy(mac, digest, hash_size);
        *mac_length = hash_size;
#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
        nvt_psa_hmac_last_compute_path = NVT_PSA_HMAC_PATH_HARDWARE;
#endif
    }

    mbedtls_platform_zeroize(digest, sizeof(digest));
    mbedtls_platform_zeroize(s_hmac_dma, sizeof(s_hmac_dma));
    nvt_crpt_release();
    return status;
}
