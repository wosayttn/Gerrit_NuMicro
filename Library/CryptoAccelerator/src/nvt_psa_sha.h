/*
 * Nuvoton PSA SHA helper extracted from the SHA-256 hardware implementation.
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

#ifndef TF_PSA_CRYPTO_NVT_PSA_SHA_H
#define TF_PSA_CRYPTO_NVT_PSA_SHA_H
#include "nvt_hash_driver_entrypoints.h"

typedef nvt_hash_operation_t nvt_psa_sha_operation_t ;

typedef enum
{
    NVT_PSA_SHA_COMPUTE_FAILED = 0,
    NVT_PSA_SHA_COMPUTE_HARDWARE,
    NVT_PSA_SHA_COMPUTE_SOFTWARE_FALLBACK
} nvt_psa_sha_compute_path_t;

extern volatile nvt_psa_sha_compute_path_t nvt_psa_sha_last_compute_path;

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
typedef enum
{
    NVT_PSA_HMAC_PATH_FAILED = 0,
    NVT_PSA_HMAC_PATH_HARDWARE,
    NVT_PSA_HMAC_PATH_BUILTIN_FALLBACK,
    NVT_PSA_HMAC_PATH_SOFTWARE_FALLBACK
} nvt_psa_hmac_path_t;

extern volatile nvt_psa_hmac_path_t nvt_psa_hmac_last_compute_path;
#endif

psa_status_t nvt_psa_sha_init(nvt_psa_sha_operation_t *operation);
psa_status_t nvt_psa_sha_free(nvt_psa_sha_operation_t *operation);
psa_status_t nvt_psa_sha_clone(const nvt_psa_sha_operation_t *source,
                               nvt_psa_sha_operation_t *target);
psa_status_t nvt_psa_sha_starts(nvt_psa_sha_operation_t *operation,
                                psa_algorithm_t alg);
psa_status_t nvt_psa_sha_update(nvt_psa_sha_operation_t *operation,
                                const uint8_t *input,
                                size_t input_length);
psa_status_t nvt_psa_sha_finish(nvt_psa_sha_operation_t *operation,
                                uint8_t *output,
                                size_t output_size,
                                size_t *output_length);
psa_status_t nvt_psa_sha_compute(psa_algorithm_t alg,
                                 const uint8_t *input,
                                 size_t input_length,
                                 uint8_t *output,
                                 size_t output_size,
                                 size_t *output_length);

psa_status_t nvt_psa_hmac_compute(
    const psa_key_attributes_t *attributes,
    const uint8_t *key,
    size_t key_length,
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *mac,
    size_t mac_size,
    size_t *mac_length);

#endif /* TF_PSA_CRYPTO_NVT_PSA_SHA_H */