/*
 * Nuvoton transparent hash driver implementation (skeleton)
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

#include "nvt_hash_driver_entrypoints.h"
#include "mbedtls/platform.h"
#include "mbedtls/private/crypto_builtin_primitives.h"
#include "tf_psa_crypto_common.h"
#include <psa/crypto.h>
#include "nvt_psa_sha.h"
#include "nvt_crpt.h"
#include "NuMicro.h"

#if defined(MBEDTLS_PSA_NVT_HASH_DRIVER_ENABLED)

psa_status_t nvt_hash_driver_init(void)
{
    PSA_DEBUG_LOG("nvt-hash",
                  "\r\n");
    SHA_ENABLE_INT(CRPT);
    NVIC_EnableIRQ(CRPT_IRQn);
    return PSA_SUCCESS;
}

void nvt_hash_driver_free(void)
{
    PSA_DEBUG_LOG("nvt-hash",
                  "\r\n");
}

psa_status_t nvt_hash_transparent_hash_compute(
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *hash,
    size_t hash_size,
    size_t *hash_length)
{
    psa_status_t status;

    PSA_DEBUG_LOG("nvt-hash",
                  "\r\n");
    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = nvt_psa_sha_compute(alg,
                                 input,
                                 input_length,
                                 hash,
                                 hash_size,
                                 hash_length);
    nvt_crpt_release();
    return status;
}

psa_status_t nvt_hash_transparent_mac_compute(
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
    return nvt_psa_hmac_compute(attributes, key_buffer, key_buffer_size,
                                alg, input, input_length,
                                mac, mac_size, mac_length);
}

psa_status_t nvt_hash_transparent_hash_setup(
    nvt_hash_operation_t *operation,
    psa_algorithm_t alg)
{
    PSA_DEBUG_LOG("nvt-hash",
                  "\r\n");
    return nvt_psa_sha_starts(operation, alg);
}

psa_status_t nvt_hash_transparent_hash_clone(
    const nvt_hash_operation_t *source_operation,
    nvt_hash_operation_t *target_operation)
{
    PSA_DEBUG_LOG("nvt-hash",
                  "\r\n");
    return nvt_psa_sha_clone(source_operation, target_operation);
}

psa_status_t nvt_hash_transparent_hash_update(
    nvt_hash_operation_t *operation,
    const uint8_t *input,
    size_t input_length)
{
    psa_status_t status;

    PSA_DEBUG_LOG("nvt-hash",
                  "\r\n");
    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = nvt_psa_sha_update(operation, input, input_length);
    nvt_crpt_release();
    return status;
}

psa_status_t nvt_hash_transparent_hash_finish(
    nvt_hash_operation_t *operation,
    uint8_t *hash,
    size_t hash_size,
    size_t *hash_length)
{
    psa_status_t status;

    PSA_DEBUG_LOG("nvt-hash",
                  "\r\n");
    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = nvt_psa_sha_finish(operation, hash, hash_size, hash_length);
    nvt_crpt_release();
    return status;
}

psa_status_t nvt_hash_transparent_hash_abort(
    nvt_hash_operation_t *operation)
{
    PSA_DEBUG_LOG("nvt-hash",
                  "\r\n");
    return nvt_psa_sha_free(operation);
}

#endif /* MBEDTLS_PSA_NVT_HASH_DRIVER_ENABLED */
