/*
 * \file gcm.h
 *
 * \brief This file contains GCM definitions and functions.
 *
 * The Galois/Counter Mode (GCM) for 128-bit block ciphers is defined
 * in <em>D. McGrew, J. Viega, The Galois/Counter Mode of Operation
 * (GCM), Natl. Inst. Stand. Technol.</em>
 *
 * For more information on GCM, see <em>NIST SP 800-38D: Recommendation for
 * Block Cipher Modes of Operation: Galois/Counter Mode (GCM) and GMAC</em>.
 *
 */
/*
 * Copyright The Mbed TLS Contributors
 * Copyright (c) 2026, Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NVT_PSA_GCM_H
#define NVT_PSA_GCM_H

#include "nvt_aead_driver_entrypoints.h"
#include <stddef.h>
#include <stdint.h>
#include <psa/crypto.h>


typedef nvt_aead_operation_t nvt_psa_gcm_operation_t ;


/////**
//// * \brief          The GCM context structure.
//// */
////typedef struct mbedtls_gcm_context
////{
////    mbedtls_cipher_context_t MBEDTLS_PRIVATE(cipher_ctx);  /*!< The cipher context used. */
////    /* uint64_t MBEDTLS_PRIVATE(HL)[16]; */                /*!< not used: Precalculated HTable low. */
////    /* uint64_t MBEDTLS_PRIVATE(HH)[16]; */                /*!< not used: Precalculated HTable high. */
////    uint64_t MBEDTLS_PRIVATE(len);                         /*!< The total length of the encrypted data. */
////    /* uint64_t MBEDTLS_PRIVATE(add_len); */               /*!< not used: The total length of the additional data. */
////    /* unsigned char MBEDTLS_PRIVATE(base_ectr)[16]; */    /*!< not used: The first ECTR for tag. */
////    /* unsigned char MBEDTLS_PRIVATE(y)[16]; */            /*!< not used: The Y working value. */
////    /* unsigned char MBEDTLS_PRIVATE(buf)[16]; */          /*!< not used: The buf working value. */
////    int MBEDTLS_PRIVATE(mode);                             /*!< The operation to perform:
////                                               #MBEDTLS_GCM_ENCRYPT or
////                                               #MBEDTLS_GCM_DECRYPT. */


////    uint32_t keySize;       /* Key size: 128/192/256 */
////    /* uint32_t encDec; */  /* not used: 0: decrypt, 1: encrypt */
////    /* uint32_t opMode; */  /* not used: AES_MODE_ECB/CBC/CFB */
////    /* uint32_t iv[4]; */   /* not used: IV for next block cipher */
////    /* uint32_t keys[8]; */ /* not used: Cipher key */
////    uint32_t basicOpt;      /* Basic option of AES controller */
////    uint8_t  gcm_buf[MAX_GCM_BUF   +16]; /* buffer for GCM DMA input */
////    uint8_t  out_buf[MAX_GCM_BUF+16+16]; /* buffer for GCM DMA output */
////    uint8_t  fb_buf[72];    /* feedback buffer for GCM DMA */
////    uint8_t  fb_buf2[72];   /* feedback buffer 2 for GCM DMA */
////    uint8_t  tag[16];       /* Tag */
////    uint32_t gcm_buf_bytes; /* Bytes in gcm_buf */
////    uint32_t firstFlag;     /* A flag for the first data block */
////    uint32_t endFlag;       /* final block is done */

////    /* uint8_t *add; */     /* not used */
////    /* size_t addlen; */    /* not used */


////}
////mbedtls_gcm_context;
#ifdef __cplusplus
extern "C" {
#endif


void nvt_psa_gcm_init(nvt_psa_gcm_operation_t *ctx);

int nvt_psa_gcm_setkey(nvt_psa_gcm_operation_t *ctx,
                       mbedtls_cipher_id_t cipher,
                       const unsigned char *key,
                       unsigned int keybits);

void nvt_psa_gcm_free(nvt_psa_gcm_operation_t *ctx);

int nvt_psa_gcm_starts(nvt_psa_gcm_operation_t *ctx,
                       int mode,
                       const unsigned char *iv,
                       size_t iv_len);

psa_status_t nvt_psa_gcm_update_ad(nvt_aead_operation_t *operation,
                                   const uint8_t *input,
                                   size_t input_length);

psa_status_t nvt_psa_gcm_update(nvt_aead_operation_t *operation,
                                const uint8_t *input,
                                size_t input_length,
                                uint8_t *output,
                                size_t output_size,
                                size_t *output_length);

psa_status_t nvt_psa_gcm_finish(nvt_aead_operation_t *operation,
                                uint8_t *ciphertext,
                                size_t ciphertext_size,
                                size_t *ciphertext_length,
                                uint8_t *tag,
                                size_t tag_size,
                                size_t *tag_length);

int nvt_psa_gcm_crypt_and_tag(nvt_psa_gcm_operation_t *ctx,
                              int mode,
                              size_t length,
                              const unsigned char *iv,
                              size_t iv_len,
                              const unsigned char *add,
                              size_t add_len,
                              const unsigned char *input,
                              unsigned char *output,
                              size_t tag_len,
                              unsigned char *tag);

psa_status_t nvt_psa_gcm_setup(nvt_aead_operation_t *operation,
                               const psa_key_attributes_t *attributes,
                               const uint8_t *key,
                               size_t key_length,
                               psa_algorithm_t alg,
                               uint8_t is_encrypt);

psa_status_t nvt_psa_gcm_encrypt(const psa_key_attributes_t *attributes,
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

psa_status_t nvt_psa_gcm_decrypt(const psa_key_attributes_t *attributes,
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

psa_status_t nvt_psa_gcm_set_nonce(nvt_aead_operation_t *operation,
                                   const uint8_t *nonce,
                                   size_t nonce_length);

psa_status_t nvt_psa_gcm_set_lengths(nvt_aead_operation_t *operation,
                                     size_t additional_data_length,
                                     size_t plaintext_length);

psa_status_t nvt_psa_gcm_verify(nvt_aead_operation_t *operation,
                                uint8_t *plaintext,
                                size_t plaintext_size,
                                size_t *plaintext_length,
                                const uint8_t *tag,
                                size_t tag_length);

psa_status_t nvt_psa_gcm_abort(nvt_aead_operation_t *operation);

#ifdef __cplusplus
}
#endif
#endif /* NVT_PSA_GCM_H*/
