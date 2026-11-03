
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
 * File: psa_get_started_example.c
 * Brief: PSA Crypto get-started examples for key management, hashing,
 *        symmetric cipher operations, AEAD, key derivation, and key export.
 * Note:  This file provides demonstration routines and is intended as
 *        reference code for PSA API usage patterns.
 */

#include "tf_psa_crypto_common.h"
#include "psa_get_started_example.h"
#include "mbedtls/platform.h"
#include "mbedtls/private/crypto_builtin_primitives.h"
#include "psa/crypto.h"

#define SOME_PLAINTEXT { \
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, \
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f \
    }

#define SOME_CIPHERTEXT { \
        0x76, 0x49, 0xab, 0xac, 0x81, 0x19, 0xb2, 0x46, \
        0xce, 0xe9, 0x8e, 0x9b, 0x12, 0xe9, 0x19, 0x7d \
    }

#define ENCRYPTED_WITH_IV { \
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, \
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f \
    }


#if PSA_GET_STARTED_ENABLE_IMPORT_A_KEY
/* Import an AES key into PSA, then destroy it after a basic lifecycle check. */
void import_a_key(const uint8_t *key, size_t key_len)
{
    psa_status_t status;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;

    printf("Import an AES key...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    /* Set key attributes */
    psa_set_key_usage_flags(&attributes, 0);
    psa_set_key_algorithm(&attributes, 0);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);

    /* Import the key */
    status = psa_import_key(&attributes, key, key_len, &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to import key\n");
        return;
    }

    printf("Imported a key\n");

    /* Free the attributes */
    psa_reset_key_attributes(&attributes);

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();
}
#endif


#if PSA_GET_STARTED_ENABLE_SIGN_A_MESSAGE_USING_RSA
/* Import an RSA private key and sign a precomputed hash with PKCS#1 v1.5 raw mode. */
void sign_a_message_using_rsa(const uint8_t *key, size_t key_len)
{
    psa_status_t status;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    uint8_t hash[32] = {0x50, 0xd8, 0x58, 0xe0, 0x98, 0x5e, 0xcc, 0x7f,
                        0x60, 0x41, 0x8a, 0xaf, 0x0c, 0xc5, 0xab, 0x58,
                        0x7f, 0x42, 0xc2, 0x57, 0x0a, 0x88, 0x40, 0x95,
                        0xa9, 0xe8, 0xcc, 0xac, 0xd0, 0xf6, 0x54, 0x5c
                       };
    uint8_t signature[PSA_SIGNATURE_MAX_SIZE] = {0};
    size_t signature_length;
    psa_key_id_t key_id;

    printf("Sign a message...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    /* Set key attributes */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH);
    psa_set_key_algorithm(&attributes, PSA_ALG_RSA_PKCS1V15_SIGN_RAW);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_RSA_KEY_PAIR);
    psa_set_key_bits(&attributes, 1024);

    /* Import the key */
    status = psa_import_key(&attributes, key, key_len, &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to import key\n");
        return;
    }

    /* Sign message using the key */
    status = psa_sign_hash(key_id, PSA_ALG_RSA_PKCS1V15_SIGN_RAW,
                           hash, sizeof(hash),
                           signature, sizeof(signature),
                           &signature_length);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to sign\n");
        return;
    }

    printf("Signed a message\n");

    /* Free the attributes */
    psa_reset_key_attributes(&attributes);

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();
}
#endif

#if PSA_GET_STARTED_ENABLE_ENCRYPT_WITH_SYMMETRIC_CIPHERS
/* Encrypt a single AES block using CBC mode without padding. */
void encrypt_with_symmetric_ciphers(const uint8_t *key, size_t key_len)
{
    enum
    {
        block_size = PSA_BLOCK_CIPHER_BLOCK_LENGTH(PSA_KEY_TYPE_AES),
    };
    psa_status_t status;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_algorithm_t alg = PSA_ALG_CBC_NO_PADDING;
    uint8_t plaintext[block_size] = SOME_PLAINTEXT;
    uint8_t iv[block_size];
    size_t iv_len;
    uint8_t output[block_size];
    size_t output_len;
    psa_key_id_t key_id;
    psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;

    printf("Encrypt with cipher...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    /* Import a key */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, alg);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    status = psa_import_key(&attributes, key, key_len, &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to import a key\n");
        return;
    }

    psa_reset_key_attributes(&attributes);

    /* Encrypt the plaintext */
    status = psa_cipher_encrypt_setup(&operation, key_id, alg);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to begin cipher operation\n");
        return;
    }

    status = psa_cipher_generate_iv(&operation, iv, sizeof(iv), &iv_len);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to generate IV\n");
        return;
    }

    status = psa_cipher_update(&operation, plaintext, sizeof(plaintext),
                               output, sizeof(output), &output_len);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to update cipher operation\n");
        return;
    }

    status = psa_cipher_finish(&operation, output + output_len,
                               sizeof(output) - output_len, &output_len);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to finish cipher operation\n");
        return;
    }

    printf("Encrypted plaintext\n");

    /* Clean up cipher operation context */
    psa_cipher_abort(&operation);

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();
}
#endif


#if PSA_GET_STARTED_ENABLE_DECRYPT_WITH_SYMMETRIC_CIPHERS
/* Decrypt a single AES-CBC block (no padding) using a provided IV. */
void decrypt_with_symmetric_ciphers(const uint8_t *key, size_t key_len)
{
    enum
    {
        block_size = PSA_BLOCK_CIPHER_BLOCK_LENGTH(PSA_KEY_TYPE_AES),
    };
    psa_status_t status;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_algorithm_t alg = PSA_ALG_CBC_NO_PADDING;
    psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;
    uint8_t ciphertext[block_size] = SOME_CIPHERTEXT;
    uint8_t iv[block_size] = ENCRYPTED_WITH_IV;
    uint8_t output[block_size];
    size_t output_len;
    psa_key_id_t key_id;

    printf("Decrypt with cipher...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    /* Import a key */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, alg);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    status = psa_import_key(&attributes, key, key_len, &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to import a key\n");
        return;
    }

    psa_reset_key_attributes(&attributes);

    /* Decrypt the ciphertext */
    status = psa_cipher_decrypt_setup(&operation, key_id, alg);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to begin cipher operation\n");
        return;
    }

    status = psa_cipher_set_iv(&operation, iv, sizeof(iv));

    if (status != PSA_SUCCESS)
    {
        printf("Failed to set IV\n");
        return;
    }

    status = psa_cipher_update(&operation, ciphertext, sizeof(ciphertext),
                               output, sizeof(output), &output_len);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to update cipher operation\n");
        return;
    }

    status = psa_cipher_finish(&operation, output + output_len,
                               sizeof(output) - output_len, &output_len);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to finish cipher operation\n");
        return;
    }

    printf("Decrypted ciphertext\n");

    /* Clean up cipher operation context */
    psa_cipher_abort(&operation);

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();
}
#endif



#if PSA_GET_STARTED_ENABLE_HASH_A_MESSAGE
/* Calculate the SHA-256 digest of a small input message. */
void hash_a_message(void)
{
    psa_status_t status;
    psa_algorithm_t alg = PSA_ALG_SHA_256;
    psa_hash_operation_t operation = PSA_HASH_OPERATION_INIT;
    unsigned char input[] = { 'a', 'b', 'c' };
    unsigned char actual_hash[PSA_HASH_MAX_SIZE];
    size_t actual_hash_len;

    printf("Hash a message...\t");
    //fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    /* Compute hash of message  */
    status = psa_hash_setup(&operation, alg);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to begin hash operation\n");
        return;
    }

    status = psa_hash_update(&operation, input, sizeof(input));

    if (status != PSA_SUCCESS)
    {
        printf("Failed to update hash operation\n");
        return;
    }

    status = psa_hash_finish(&operation, actual_hash, sizeof(actual_hash),
                             &actual_hash_len);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to finish hash operation\n");
        return;
    }

    printf("Hashed a message\n");

    for (int i = 0; i < actual_hash_len; i++)
        printf("0x%x, ", actual_hash[i]);

    printf("\n");

    /* Clean up hash operation context */
    psa_hash_abort(&operation);

    mbedtls_psa_crypto_free();

}
#endif

#if PSA_GET_STARTED_ENABLE_VERIFY_A_HASH
/* Recompute and verify the SHA-256 digest of a known input buffer. */
void verify_a_hash(void)
{

    psa_status_t status;
    psa_algorithm_t alg = PSA_ALG_SHA_256;
    psa_hash_operation_t operation = PSA_HASH_OPERATION_INIT;
    unsigned char input[] = { 'a', 'b', 'c' };
    unsigned char expected_hash[] =
    {
        0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde,
        0x5d, 0xae, 0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
        0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad
    };
    size_t expected_hash_len = PSA_HASH_LENGTH(alg);

    printf("Verify a hash...\t");
    printf("Expected hash...\t");

    for (int i = 0; i < expected_hash_len; i++)
        printf("0x%x, ", expected_hash[i]);

    printf("\n");
    //fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    /* Verify message hash */
    status = psa_hash_setup(&operation, alg);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to begin hash operation\n");
        return;
    }

    status = psa_hash_update(&operation, input, sizeof(input));

    if (status != PSA_SUCCESS)
    {
        printf("Failed to update hash operation\n");
        return;
    }

    status = psa_hash_verify(&operation, expected_hash, expected_hash_len);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to verify hash\n");
        return;
    }

    printf("Verified a hash\n");

    /* Clean up hash operation context */
    psa_hash_abort(&operation);

    mbedtls_psa_crypto_free();

}
#endif


#if PSA_GET_STARTED_ENABLE_GENERATE_RANDOM_VALUE
/* Generate a short buffer of cryptographically secure random bytes. */
void generate_random_value(void)
{
    psa_status_t status;
    uint8_t random[10] = { 0 };

    printf("Generate random...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    status = psa_generate_random(random, sizeof(random));

    if (status != PSA_SUCCESS)
    {
        printf("Failed to generate a random value\n");
        return;
    }

    printf("Generated random data\n");

    /* Clean up */
    mbedtls_psa_crypto_free();
}
#endif


#if PSA_GET_STARTED_ENABLE_DERIVE_A_KEY_HKDF
/* Derive a new AES key from a base secret using HKDF-SHA256. */
void derive_a_key_hkdf(void)
{

    psa_status_t status;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    static const unsigned char key[] =
    {
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
        0x0b
    };
    static const unsigned char salt[] =
    {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
        0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c
    };
    static const unsigned char info[] =
    {
        0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6,
        0xf7, 0xf8, 0xf9
    };
    psa_algorithm_t alg = PSA_ALG_HKDF(PSA_ALG_SHA_256);
    psa_key_derivation_operation_t operation =
        PSA_KEY_DERIVATION_OPERATION_INIT;
    size_t derived_bits = 128;
    size_t capacity = PSA_BITS_TO_BYTES(derived_bits);
    psa_key_id_t base_key;
    psa_key_id_t derived_key;

    printf("Derive a key (HKDF)...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    /* Import a key for use in key derivation. If such a key has already been
     * generated or imported, you can skip this part. */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DERIVE);
    psa_set_key_algorithm(&attributes, alg);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_DERIVE);
    status = psa_import_key(&attributes, key, sizeof(key), &base_key);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to import a key\n");
        return;
    }

    psa_reset_key_attributes(&attributes);

    /* Derive a key */
    status = psa_key_derivation_setup(&operation, alg);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to begin key derivation\n");
        return;
    }

    status = psa_key_derivation_set_capacity(&operation, capacity);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to set capacity\n");
        return;
    }

    status = psa_key_derivation_input_bytes(&operation,
                                            PSA_KEY_DERIVATION_INPUT_SALT,
                                            salt, sizeof(salt));

    if (status != PSA_SUCCESS)
    {
        printf("Failed to input salt (extract)\n");
        return;
    }

    status = psa_key_derivation_input_key(&operation,
                                          PSA_KEY_DERIVATION_INPUT_SECRET,
                                          base_key);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to input key (extract)\n");
        return;
    }

    status = psa_key_derivation_input_bytes(&operation,
                                            PSA_KEY_DERIVATION_INPUT_INFO,
                                            info, sizeof(info));

    if (status != PSA_SUCCESS)
    {
        printf("Failed to input info (expand)\n");
        return;
    }

    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_CTR);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    status = psa_key_derivation_output_key(&attributes, &operation,
                                           &derived_key);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to derive key\n");
        return;
    }

    psa_reset_key_attributes(&attributes);

    printf("Derived key\n");

    /* Clean up key derivation operation */
    psa_key_derivation_abort(&operation);

    /* Destroy the keys */
    psa_destroy_key(derived_key);
    psa_destroy_key(base_key);

    mbedtls_psa_crypto_free();
}
#endif


#if PSA_GET_STARTED_ENABLE_AUTHENTICATE_AND_ENCRYPT_A_MESSAGE
/* Perform authenticated encryption with AES-CCM for sample data. */
void authenticate_and_encrypt_a_message(void)
{

    psa_status_t status;
    //    static const uint8_t key[] = {
    //    0x40, 0x41, 0x42, 0x43, 0x44, 0x44, 0x46, 0x47,
    //    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F
    //    };

    //     static const uint8_t nonce[] = {
    //     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    //     0x18,0x19, 0x1A, 0x1B
    //    };
    //
    //    static const uint8_t additional_data[] = {
    //    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    //    0x08, 0x09, 0x0A, 0x0B, 0x10, 0x11, 0x12, 0x13,
    //    };

    //    static const uint8_t input_data[] = {
    //    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    //    0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
    //    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37
    //    };
    //    const char *key_str = "404142434445464748494a4b4c4d4e4f";
    //    const char *iv_str = "101112131415161718191A1B";
    //    const char *a_str = "000102030405060708090A0B0C0D0E0F10111213";
    //    const char *pt_str = "202122232425262728292A2B2C2D2E2F3031323334353637";
    //    const char *c_str = "E3B201A9F5B71A7A9B1CEAECCD97E70B6176AAD9A4428AA5484392FBC1B09951";


    static const uint8_t key[] =
    {
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
        0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF
    };
    static const uint8_t nonce[] =
    {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B
    };
    static const uint8_t additional_data[] =
    {
        0xEC, 0x46, 0xBB, 0x63, 0xB0, 0x25,
        0x20, 0xC3, 0x3C, 0x49, 0xFD, 0x70
    };
    static const uint8_t input_data[] =
    {
        0xB9, 0x6B, 0x49, 0xE2, 0x1D, 0x62, 0x17, 0x41,
        0x63, 0x28, 0x75, 0xDB, 0x7F, 0x6C, 0x92, 0x43,
        0xD2, 0xD7, 0xC2
    };
    uint8_t *output_data = NULL;
    size_t output_size = 0;
    size_t output_length = 0;
    size_t tag_length = 16;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;

    printf("Authenticate encrypt...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    output_size = sizeof(input_data) + tag_length;
    printf("output_size = %d \r\n", output_size);
    printf("tag_length = %d \r\n", tag_length);

    output_data = (uint8_t *)malloc(output_size);

    if (!output_data)
    {
        printf("Out of memory\n");
        return;
    }

    /* Import a key */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_CCM);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    status = psa_import_key(&attributes, key, sizeof(key), &key_id);
    psa_reset_key_attributes(&attributes);

    /* Authenticate and encrypt */
    status = psa_aead_encrypt(key_id, PSA_ALG_CCM,
                              nonce, sizeof(nonce),
                              additional_data, sizeof(additional_data),
                              input_data, sizeof(input_data),
                              output_data, output_size,
                              &output_length);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to authenticate and encrypt\n");
        return;
    }

    printf("Authenticated and encrypted\n");

    /* Clean up */
    free(output_data);

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();
}

/* Perform authenticated encryption with AES-GCM for sample data. */
void authenticate_and_encrypt_a_message_gcm(void)
{

    psa_status_t status;
    static const uint8_t key[] =  //16 byte
    {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };

    static const uint8_t nonce[] =  //13 byte
    {
        0x4D, 0x4D, 0x4D, 0x00, 0x00, 0xBC, 0x61, 0x4E,
        0x01, 0x23, 0x45, 0x67, 0x89
    };

    static const uint8_t additional_data[] =  //17 byte
    {
        0x30, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6,
        0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde,
        0xdf
    };

    static const uint8_t input_data[] =
    {
        0x01, 0x01, 0x10, 0x00, 0x11, 0x22, 0x33, 0x44,
        0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC,
        0xDD, 0xEE, 0xFF, 0x00, 0x00, 0x06, 0x5F, 0x1F,
        0x04, 0x00, 0x00, 0x7E, 0x1F, 0x04, 0xB0, 0x11,
        0x22
    };

    uint8_t *output_data = NULL;
    size_t output_size = 0;
    size_t output_length = 0;
    size_t tag_length = 16;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;

    printf("Authenticate encrypt...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    output_size = sizeof(input_data) + tag_length;
    printf("output_size = %d \r\n", output_size);
    printf("tag_length = %d \r\n", tag_length);

    output_data = (uint8_t *)malloc(output_size);

    if (!output_data)
    {
        printf("Out of memory\n");
        return;
    }

    /* Import a key */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_GCM);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    status = psa_import_key(&attributes, key, sizeof(key), &key_id);
    psa_reset_key_attributes(&attributes);

    /* Authenticate and encrypt */
    status = psa_aead_encrypt(key_id, PSA_ALG_GCM,
                              nonce, sizeof(nonce),
                              additional_data, sizeof(additional_data),
                              input_data, sizeof(input_data),
                              output_data, output_size,
                              &output_length);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to authenticate and encrypt\n");
        return;
    }

    printf("Authenticated and encrypted\n");

    /* Clean up */
    free(output_data);

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();
}
#endif


#if PSA_GET_STARTED_ENABLE_AUTHENTICATE_AND_DECRYPT_A_MESSAGE
/* Perform authenticated decryption with AES-CCM and verify the tag. */
void authenticate_and_decrypt_a_message(void)
{

    psa_status_t status;
    static const uint8_t key_data[] =
    {
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
        0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF
    };
    static const uint8_t nonce[] =
    {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B
    };
    static const uint8_t additional_data[] =
    {
        0xEC, 0x46, 0xBB, 0x63, 0xB0, 0x25,
        0x20, 0xC3, 0x3C, 0x49, 0xFD, 0x70
    };
    static const uint8_t input_data[] =
    {
        0x20, 0x30, 0xE0, 0x36, 0xED, 0x09, 0xA0, 0x45, 0xAF, 0x3C, 0xBA, 0xEE,
        0x0F, 0xC8, 0x48, 0xAF, 0xCD, 0x89, 0x54, 0xF4, 0xF6, 0x3F, 0x28, 0x9A,
        0xA1, 0xDD, 0xB2, 0xB8, 0x09, 0xCD, 0x7C, 0xE1, 0x46, 0xE9, 0x98
    };
    uint8_t *output_data = NULL;
    size_t output_size = 0;
    size_t output_length = 0;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;

    printf("Authenticate decrypt...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    output_size = sizeof(input_data);
    output_data = (uint8_t *)malloc(output_size);

    if (!output_data)
    {
        printf("Out of memory\n");
        return;
    }

    /* Import a key */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_CCM);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    status = psa_import_key(&attributes, key_data, sizeof(key_data), &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to import a key\n");
        return;
    }

    psa_reset_key_attributes(&attributes);

    /* Authenticate and decrypt */
    status = psa_aead_decrypt(key_id, PSA_ALG_CCM,
                              nonce, sizeof(nonce),
                              additional_data, sizeof(additional_data),
                              input_data, sizeof(input_data),
                              output_data, output_size,
                              &output_length);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to authenticate and decrypt %d\n", status);
        return;
    }

    printf("Authenticated and decrypted\n");

    /* Clean up */
    free(output_data);

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();

}

/* Perform authenticated decryption with AES-GCM and verify the tag. */
void authenticate_and_decrypt_a_message_gcm(void)
{
    psa_status_t status;
    static const uint8_t key_data[] =  //16 byte
    {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };

    static const uint8_t nonce[] =  //13 byte
    {
        0x4D, 0x4D, 0x4D, 0x00, 0x00, 0xBC, 0x61, 0x4E,
        0x01, 0x23, 0x45, 0x67, 0x89
    };

    static const uint8_t additional_data[] =  //17 byte
    {
        0x30, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6,
        0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde,
        0xdf
    };

    static const uint8_t input_data[] =
    {

        0xcb, 0xc2, 0xa7, 0x1a, 0x9a, 0x0e, 0xdd, 0xd3,
        0x9a, 0xc1, 0xa4, 0xd4, 0x30, 0xb4, 0x8a, 0xb4,
        0xe4, 0x68, 0x98, 0x69, 0x79, 0x4c, 0xb4, 0x8a,
        0x97, 0x43, 0x95, 0x77, 0x40, 0x66, 0x1f, 0x96,
        0x3c, 0xc6, 0x23, 0xff, 0xe4, 0x76, 0x19, 0xa2,
        0x4c, 0x31, 0x20, 0xd2, 0xc8, 0xfa, 0x7c, 0x1a,
        0x1e
    };

    uint8_t *output_data = NULL;
    size_t output_size = 0;
    size_t output_length = 0;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;

    printf("Authenticate decrypt...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    output_size = sizeof(input_data);
    output_data = (uint8_t *)malloc(output_size);

    if (!output_data)
    {
        printf("Out of memory\n");
        return;
    }

    /* Import a key */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_GCM);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    status = psa_import_key(&attributes, key_data, sizeof(key_data), &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to import a key\n");
        return;
    }

    psa_reset_key_attributes(&attributes);

    /* Authenticate and decrypt */
    status = psa_aead_decrypt(key_id, PSA_ALG_GCM,
                              nonce, sizeof(nonce),
                              additional_data, sizeof(additional_data),
                              input_data, sizeof(input_data),
                              output_data, output_size,
                              &output_length);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to authenticate and decrypt %d\n", status);
        return;
    }

    printf("Authenticated and decrypted\n");

    /* Clean up */
    free(output_data);

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();

}
#endif

#if PSA_GET_STARTED_ENABLE_GENERATE_AND_EXPORT_KEYS
/* Generate an ECC key pair and export its public key bytes. */
void generate_and_export_keys(void)
{

    enum
    {
        key_bits = 256,
    };
    psa_status_t status;
    size_t exported_length = 0;
    static uint8_t exported[PSA_KEY_EXPORT_ECC_PUBLIC_KEY_MAX_SIZE(key_bits)];
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;

    printf("Generate a key pair...\t");
    fflush(stdout);

    /* Initialize PSA Crypto */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

    /* Generate a key */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH);
    psa_set_key_algorithm(&attributes,
                          PSA_ALG_DETERMINISTIC_ECDSA(PSA_ALG_SHA_256));
    psa_set_key_type(&attributes,
                     PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attributes, key_bits);
    status = psa_generate_key(&attributes, &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to generate key\n");
        return;
    }

    psa_reset_key_attributes(&attributes);

    status = psa_export_public_key(key_id, exported, sizeof(exported),
                                   &exported_length);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to export public key %ld\n", status);
        return;
    }

    printf("Exported a public key\n");

    /* Destroy the key */
    psa_destroy_key(key_id);

    mbedtls_psa_crypto_free();

}
#endif

/* End of file psa_get_started_example.c */