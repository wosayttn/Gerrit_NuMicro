/*
 * Nuvoton transparent RSA PSA driver entry points (dispatched by
 * psa_sign_hash/psa_verify_hash/psa_asymmetric_encrypt/psa_asymmetric_decrypt).
 *
 * Handled on the CRPT RSA engine: SHA-256 PKCS#1 v1.5 and PSS (standard salt)
 * sign/verify, SHA-256 OAEP encrypt/decrypt, 1024/2048/3072/4096-bit keys.
 * Everything else returns PSA_ERROR_NOT_SUPPORTED and falls back to software.
 *
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

#ifndef TF_PSA_CRYPTO_NVT_RSA_DRIVER_ENTRYPOINTS_H
#define TF_PSA_CRYPTO_NVT_RSA_DRIVER_ENTRYPOINTS_H

#include <psa/crypto.h>
#include <stddef.h>
#include <stdint.h>

psa_status_t nvt_rsa_driver_init(void);
void nvt_rsa_driver_free(void);

psa_status_t nvt_rsa_transparent_sign_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    uint8_t *signature, size_t signature_size, size_t *signature_length);

psa_status_t nvt_rsa_transparent_verify_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    const uint8_t *signature, size_t signature_length);

/* salt is the OAEP label. */
psa_status_t nvt_rsa_transparent_asymmetric_encrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input, size_t input_length,
    const uint8_t *salt, size_t salt_length,
    uint8_t *output, size_t output_size, size_t *output_length);

psa_status_t nvt_rsa_transparent_asymmetric_decrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input, size_t input_length,
    const uint8_t *salt, size_t salt_length,
    uint8_t *output, size_t output_size, size_t *output_length);

#endif /* TF_PSA_CRYPTO_NVT_RSA_DRIVER_ENTRYPOINTS_H */
