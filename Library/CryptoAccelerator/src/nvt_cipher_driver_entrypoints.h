/*
 * Nuvoton transparent cipher driver entrypoints (skeleton)
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

#ifndef TF_PSA_CRYPTO_NVT_CIPHER_DRIVER_ENTRYPOINTS_H
#define TF_PSA_CRYPTO_NVT_CIPHER_DRIVER_ENTRYPOINTS_H

#include <psa/crypto_types.h>
#include <stddef.h>
#include <stdint.h>


#define NVT_PSA_AES_BLOCK_SIZE 16u

typedef struct
{
    psa_algorithm_t alg;
    uint32_t key_bits;
    uint32_t key_length;
    uint8_t key[32];
    uint8_t iv[NVT_PSA_AES_BLOCK_SIZE];
    uint8_t stream_block[NVT_PSA_AES_BLOCK_SIZE];
    size_t iv_off;
    uint8_t is_encrypt;
    uint8_t ctr_exhausted;
    uint8_t iv_set;
    uint8_t active;
} nvt_cipher_operation_t;


psa_status_t nvt_cipher_transparent_cipher_encrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg,
    const uint8_t *iv, size_t iv_length,
    const uint8_t *input, size_t input_length,
    uint8_t *output, size_t output_size, size_t *output_length);

psa_status_t nvt_cipher_transparent_cipher_decrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg,
    const uint8_t *input, size_t input_length,
    uint8_t *output, size_t output_size, size_t *output_length);

psa_status_t nvt_cipher_transparent_mac_compute(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *mac,
    size_t mac_size,
    size_t *mac_length);

psa_status_t nvt_cipher_transparent_cipher_encrypt_setup(
    nvt_cipher_operation_t *operation,
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg);

psa_status_t nvt_cipher_transparent_cipher_decrypt_setup(
    nvt_cipher_operation_t *operation,
    const psa_key_attributes_t *attributes,
    const uint8_t *key, size_t key_length,
    psa_algorithm_t alg);

psa_status_t nvt_cipher_transparent_cipher_set_iv(
    nvt_cipher_operation_t *operation,
    const uint8_t *iv, size_t iv_length);

psa_status_t nvt_cipher_transparent_cipher_update(
    nvt_cipher_operation_t *operation,
    const uint8_t *input, size_t input_length,
    uint8_t *output, size_t output_size, size_t *output_length);

psa_status_t nvt_cipher_transparent_cipher_finish(
    nvt_cipher_operation_t *operation,
    uint8_t *output, size_t output_size, size_t *output_length);

psa_status_t nvt_cipher_transparent_cipher_abort(
    nvt_cipher_operation_t *operation);

psa_status_t nvt_cipher_driver_init(void);
void nvt_cipher_driver_free(void);

#endif /* TF_PSA_CRYPTO_NVT_CIPHER_DRIVER_ENTRYPOINTS_H */
