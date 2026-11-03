/*
 * Nuvoton PSA CCM high-level helper API.
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

#ifndef NVT_PSA_CCM_HL_H
#define NVT_PSA_CCM_HL_H

#include "nvt_aead_driver_entrypoints.h"
#include <stddef.h>
#include <stdint.h>
#include <psa/crypto.h>

//#define MAX_CCM_BUF     256

#define MBEDTLS_CCM_DECRYPT       0
#define MBEDTLS_CCM_ENCRYPT       1
#define MBEDTLS_CCM_STAR_DECRYPT  2
#define MBEDTLS_CCM_STAR_ENCRYPT  3

//marked 2026-07-03
/////* Bad input parameters to the function. */
////#define MBEDTLS_ERR_CCM_BAD_INPUT       -0x000D
/////* Authenticated decryption failed. */
////#define MBEDTLS_ERR_CCM_AUTH_FAILED     -0x000F

#ifdef __cplusplus
extern "C" {
#endif

//typedef struct nvt_psa_ccm_operation_t
//{
//    unsigned char y[16];
//    unsigned char ctr[16];
//    mbedtls_cipher_context_t cipher_ctx;
//    size_t plaintext_len;
//    size_t add_len;
//    size_t tag_len;
//    size_t processed;
//    unsigned char q;
//    unsigned char mode;
//    int state;

//    uint8_t ccm_buf[MAX_CCM_BUF];
//    uint8_t out_buf[MAX_CCM_BUF + 16];
//    uint8_t key[32];
//    uint32_t key_bits;
//} nvt_psa_ccm_operation_t;

typedef nvt_aead_operation_t nvt_psa_ccm_operation_t ;

psa_status_t nvt_psa_ccm_set_lengths(nvt_aead_operation_t *operation,
                                     size_t additional_data_length,
                                     size_t plaintext_length);

void nvt_psa_ccm_init(nvt_psa_ccm_operation_t *ctx);

int nvt_psa_ccm_setkey(nvt_psa_ccm_operation_t *ctx,
                       mbedtls_cipher_id_t cipher,
                       const unsigned char *key,
                       unsigned int keybits);

void nvt_psa_ccm_free(nvt_psa_ccm_operation_t *ctx);

int nvt_psa_ccm_starts(nvt_psa_ccm_operation_t *ctx,
                       int mode,
                       const unsigned char *iv,
                       size_t iv_len);

int nvt_psa_ccm_set_lengths_internal(nvt_psa_ccm_operation_t *ctx,
                                     size_t total_ad_len,
                                     size_t plaintext_len,
                                     size_t tag_len);

int nvt_psa_ccm_update_ad(nvt_psa_ccm_operation_t *ctx,
                          const unsigned char *add,
                          size_t add_len);

int nvt_psa_ccm_update(nvt_psa_ccm_operation_t *ctx,
                       const unsigned char *input,
                       size_t input_len,
                       unsigned char *output,
                       size_t output_size,
                       size_t *output_len);

//int nvt_psa_ccm_finish( nvt_psa_ccm_operation_t *ctx,
//                        unsigned char *tag,
//                        size_t tag_len );

int nvt_psa_ccm_star_encrypt_and_tag(nvt_psa_ccm_operation_t *ctx,
                                     size_t length,
                                     const unsigned char *iv,
                                     size_t iv_len,
                                     const unsigned char *add,
                                     size_t add_len,
                                     const unsigned char *input,
                                     unsigned char *output,
                                     unsigned char *tag,
                                     size_t tag_len);

int nvt_psa_ccm_encrypt_and_tag(nvt_psa_ccm_operation_t *ctx,
                                size_t length,
                                const unsigned char *iv,
                                size_t iv_len,
                                const unsigned char *add,
                                size_t add_len,
                                const unsigned char *input,
                                unsigned char *output,
                                unsigned char *tag,
                                size_t tag_len);

int nvt_psa_ccm_star_auth_decrypt(nvt_psa_ccm_operation_t *ctx,
                                  size_t length,
                                  const unsigned char *iv,
                                  size_t iv_len,
                                  const unsigned char *add,
                                  size_t add_len,
                                  const unsigned char *input,
                                  unsigned char *output,
                                  const unsigned char *tag,
                                  size_t tag_len);

int nvt_psa_ccm_auth_decrypt(nvt_psa_ccm_operation_t *ctx,
                             size_t length,
                             const unsigned char *iv,
                             size_t iv_len,
                             const unsigned char *add,
                             size_t add_len,
                             const unsigned char *input,
                             unsigned char *output,
                             const unsigned char *tag,
                             size_t tag_len);

/* ===== PSA Wrapper Layer Functions ===== */

/**
 * \brief Setup operation for PSA interface
 */
psa_status_t nvt_psa_ccm_setup(nvt_aead_operation_t *operation,
                               const psa_key_attributes_t *attributes,
                               const uint8_t *key,
                               size_t key_length,
                               psa_algorithm_t alg,
                               uint8_t is_encrypt);


/**
 * \brief One-shot CCM encryption for PSA interface
 */
psa_status_t nvt_psa_ccm_encrypt(const psa_key_attributes_t *attributes,
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


/**
 * \brief One-shot CCM decryption for PSA interface
 */
psa_status_t nvt_psa_ccm_decrypt(const psa_key_attributes_t *attributes,
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


/**
 * \brief Verify tag for PSA interface
 */
psa_status_t nvt_psa_ccm_verify(nvt_aead_operation_t *operation,
                                uint8_t *plaintext,
                                size_t plaintext_size,
                                size_t *plaintext_length,
                                const uint8_t *tag,
                                size_t tag_length);
/**
 * \brief Abort operation for PSA interface
 */
psa_status_t nvt_psa_ccm_abort(nvt_aead_operation_t *operation);

/**
* \brief Set nonce for PSA interface
*/
psa_status_t nvt_psa_ccm_set_nonce(nvt_aead_operation_t *operation,
                                   const uint8_t *nonce,
                                   size_t nonce_length);

/**
* \brief Finish operation and generate tag for PSA interface
*/
psa_status_t nvt_psa_ccm_finish(nvt_aead_operation_t *operation,
                                uint8_t *ciphertext,
                                size_t ciphertext_size,
                                size_t *ciphertext_length,
                                uint8_t *tag,
                                size_t tag_size,
                                size_t *tag_length);

#ifdef __cplusplus
}
#endif

#endif /* NVT_PSA_CCM_HL_H */
