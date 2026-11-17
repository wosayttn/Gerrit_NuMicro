/*
 * Nuvoton transparent hash driver entrypoints (skeleton)
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

#ifndef TF_PSA_CRYPTO_NVT_HASH_DRIVER_ENTRYPOINTS_H
#define TF_PSA_CRYPTO_NVT_HASH_DRIVER_ENTRYPOINTS_H

#include <psa/crypto_types.h>
#include <stddef.h>
#include <stdint.h>


#define NVT_PSA_SHA_BLOCK_SIZE 256
typedef struct
{
    psa_algorithm_t alg;
    uint8_t is224;
    uint8_t active;
    uint8_t docascade;
    int32_t isfirst;
    uint32_t ctl;
    uint32_t buffer_len;
    uint8_t buffer[NVT_PSA_SHA_BLOCK_SIZE];
} nvt_hash_operation_t;

psa_status_t nvt_hash_transparent_hash_compute(
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *hash,
    size_t hash_size,
    size_t *hash_length);

psa_status_t nvt_hash_transparent_mac_compute(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *mac,
    size_t mac_size,
    size_t *mac_length);

psa_status_t nvt_hash_transparent_hash_setup(
    nvt_hash_operation_t *operation,
    psa_algorithm_t alg);

psa_status_t nvt_hash_transparent_hash_clone(
    const nvt_hash_operation_t *source_operation,
    nvt_hash_operation_t *target_operation);

psa_status_t nvt_hash_transparent_hash_update(
    nvt_hash_operation_t *operation,
    const uint8_t *input,
    size_t input_length);

psa_status_t nvt_hash_transparent_hash_finish(
    nvt_hash_operation_t *operation,
    uint8_t *hash,
    size_t hash_size,
    size_t *hash_length);

psa_status_t nvt_hash_transparent_hash_abort(
    nvt_hash_operation_t *operation);

psa_status_t nvt_hash_driver_init(void);
void nvt_hash_driver_free(void);

#endif /* TF_PSA_CRYPTO_NVT_HASH_DRIVER_ENTRYPOINTS_H */
