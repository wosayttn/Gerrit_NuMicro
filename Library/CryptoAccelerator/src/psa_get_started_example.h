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

/*
 * File: psa_get_started_example.h
 * Brief: Declarations and feature switches for PSA Crypto get-started examples.
 */

#ifndef PSA_GET_STARTED_EXAMPLE_H
#define PSA_GET_STARTED_EXAMPLE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Set each switch to 0/1 to disable/enable a specific get-started function. */
//#define PSA_GET_STARTED_ENABLE_IMPORT_A_KEY                       1
//#define PSA_GET_STARTED_ENABLE_SIGN_A_MESSAGE_USING_RSA           1
#define PSA_GET_STARTED_ENABLE_ENCRYPT_WITH_SYMMETRIC_CIPHERS     1
#define PSA_GET_STARTED_ENABLE_DECRYPT_WITH_SYMMETRIC_CIPHERS     1
#define PSA_GET_STARTED_ENABLE_HASH_A_MESSAGE                     1
#define PSA_GET_STARTED_ENABLE_VERIFY_A_HASH                      1
//#define PSA_GET_STARTED_ENABLE_GENERATE_RANDOM_VALUE              1
//#define PSA_GET_STARTED_ENABLE_DERIVE_A_KEY_HKDF                  1
#define PSA_GET_STARTED_ENABLE_AUTHENTICATE_AND_ENCRYPT_A_MESSAGE 1
#define PSA_GET_STARTED_ENABLE_AUTHENTICATE_AND_DECRYPT_A_MESSAGE 1
//#define PSA_GET_STARTED_ENABLE_GENERATE_AND_EXPORT_KEYS           1

/* Set each switch to 0/1 to disable/enable a specific get-started function. */
#if PSA_GET_STARTED_ENABLE_ENCRYPT_WITH_SYMMETRIC_CIPHERS
void encrypt_with_symmetric_ciphers(const uint8_t *key, size_t key_len);
#endif
#if PSA_GET_STARTED_ENABLE_DECRYPT_WITH_SYMMETRIC_CIPHERS
void decrypt_with_symmetric_ciphers(const uint8_t *key, size_t key_len);
#endif
#if PSA_GET_STARTED_ENABLE_HASH_A_MESSAGE
void hash_a_message(void);
#endif
#if PSA_GET_STARTED_ENABLE_VERIFY_A_HASH
void verify_a_hash(void);
#endif

#if PSA_GET_STARTED_ENABLE_AUTHENTICATE_AND_ENCRYPT_A_MESSAGE
void authenticate_and_encrypt_a_message(void);
void authenticate_and_encrypt_a_message_gcm(void);
#endif


#if PSA_GET_STARTED_ENABLE_AUTHENTICATE_AND_DECRYPT_A_MESSAGE
void authenticate_and_decrypt_a_message(void);
void authenticate_and_decrypt_a_message_gcm(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* PSA_GET_STARTED_EXAMPLE_H */
