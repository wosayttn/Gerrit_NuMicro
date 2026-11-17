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

#ifndef TF_PSA_CRYPTO_NVT_AEAD_DRIVER_ENTRYPOINTS_H
#define TF_PSA_CRYPTO_NVT_AEAD_DRIVER_ENTRYPOINTS_H

#include <psa/crypto_types.h>
#include <stddef.h>
#include <stdint.h>
#include "mbedtls/private/ccm.h"

#define NVT_PSA_CCM_MAX_DMA_SIZE         256u
#define MAX_GCM_BUF                      256
#define MAX_CCM_BUF                      NVT_PSA_CCM_MAX_DMA_SIZE
#define GCM_PBLOCK_SIZE  MAX_GCM_BUF     /* NOTE: GCM_PBLOCK_SIZE must be 16 bytes alignment. This value must > size of A */
typedef struct
{
    //CCM
    unsigned char y[16];
    unsigned char ctr[16];
    mbedtls_cipher_context_t cipher_ctx;
    size_t plaintext_len;
    size_t add_len;
    size_t tag_len;
    size_t processed;
    unsigned char q;
    unsigned char mode;
    int state;

    uint8_t ccm_buf[MAX_CCM_BUF];
    uint8_t key[32];
    uint32_t key_bits;

    psa_algorithm_t alg;
    psa_key_type_t key_type;
    uint8_t is_encrypt;
    uint8_t tag_length;
    uint8_t active;

    //GCM
    size_t keySize;       /* Key size: 128/192/256 */
    uint32_t basicOpt;      /* Basic option of AES controller */
    uint8_t  gcm_buf[MAX_GCM_BUF   + 16]; /* buffer for GCM DMA input */

    uint8_t  fb_buf[72];    /* feedback buffer for GCM DMA */
    uint8_t  fb_buf2[72];   /* feedback buffer 2 for GCM DMA */
    uint8_t  tag[16];       /* Tag */
    uint32_t gcm_buf_bytes; /* Bytes in gcm_buf */
    uint32_t firstFlag;     /* A flag for the first data block */
    uint32_t endFlag;       /* final block is done */
    uint64_t len;           /*!< The total length of the encrypted data. */

    //CCM and GCM SHARED
    //uint8_t  out_buf[MAX_CCM_BUF + 16];
    uint8_t  out_buf[MAX_GCM_BUF + 16 + 16]; /* buffer for GCM DMA output, take bigger one.*/

    /* Keep multipart fields after the DMA buffers to preserve their offsets. */
    uint8_t *ccm_multipart_buffer;
    uint8_t multipart_nonce[16];
    uint8_t *multipart_buffer;
    size_t multipart_nonce_length;
    size_t multipart_aad_length;
    size_t multipart_data_length;
    size_t multipart_aad_expected;
    size_t multipart_data_expected;
    uint8_t multipart_nonce_set;
    uint8_t multipart_lengths_set;
    uint8_t multipart_body_started;
} nvt_aead_operation_t;


psa_status_t nvt_aead_transparent_encrypt(const psa_key_attributes_t *attributes,
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
                                          size_t *ciphertext_length);

psa_status_t nvt_aead_transparent_decrypt(const psa_key_attributes_t *attributes,
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
                                          size_t *plaintext_length);

psa_status_t nvt_aead_transparent_encrypt_setup(nvt_aead_operation_t *operation,
                                                const psa_key_attributes_t *attributes,
                                                const uint8_t *key,
                                                size_t key_length,
                                                psa_algorithm_t alg);

psa_status_t nvt_aead_transparent_decrypt_setup(nvt_aead_operation_t *operation,
                                                const psa_key_attributes_t *attributes,
                                                const uint8_t *key,
                                                size_t key_length,
                                                psa_algorithm_t alg);

psa_status_t nvt_aead_transparent_set_nonce(nvt_aead_operation_t *operation,
                                            const uint8_t *nonce,
                                            size_t nonce_length);

psa_status_t nvt_aead_transparent_set_lengths(nvt_aead_operation_t *operation,
                                              size_t additional_data_length,
                                              size_t plaintext_length);

psa_status_t nvt_aead_transparent_update_ad(nvt_aead_operation_t *operation,
                                            const uint8_t *input,
                                            size_t input_length);

psa_status_t nvt_aead_transparent_update(nvt_aead_operation_t *operation,
                                         const uint8_t *input,
                                         size_t input_length,
                                         uint8_t *output,
                                         size_t output_size,
                                         size_t *output_length);

psa_status_t nvt_aead_transparent_finish(nvt_aead_operation_t *operation,
                                         uint8_t *ciphertext,
                                         size_t ciphertext_size,
                                         size_t *ciphertext_length,
                                         uint8_t *tag,
                                         size_t tag_size,
                                         size_t *tag_length);

psa_status_t nvt_aead_transparent_verify(nvt_aead_operation_t *operation,
                                         uint8_t *plaintext,
                                         size_t plaintext_size,
                                         size_t *plaintext_length,
                                         const uint8_t *tag,
                                         size_t tag_length);

psa_status_t nvt_aead_transparent_abort(nvt_aead_operation_t *operation);

psa_status_t nvt_aead_driver_init(void);
void nvt_aead_driver_free(void);

#endif /* TF_PSA_CRYPTO_NVT_AEAD_DRIVER_ENTRYPOINTS_H */