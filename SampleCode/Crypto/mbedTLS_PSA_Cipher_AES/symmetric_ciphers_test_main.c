/*
 * Wrapper test program for symmetric_ciphers_test.c.
 *
 * This file keeps the original tutorial-style function unchanged and provides
 * a buildable entry point plus output verification using a known AES-128-CBC
 * test vector.
 */

#include "psa/crypto.h"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "psa/crypto.h"
#include "psa_crypto_core.h"

#include "psa_get_started_example.h"

#include "nvt_psa_aes.h"
#include "nvt_crpt.h"
static const uint8_t expected_plaintext[] =
{
    0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
    0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a
};

static uint8_t captured_output[32];
static size_t captured_output_length;




#define SOME_CIPHERTEXT { \
        0x76, 0x49, 0xab, 0xac, 0x81, 0x19, 0xb2, 0x46, \
        0xce, 0xe9, 0x8e, 0x9b, 0x12, 0xe9, 0x19, 0x7d \
    }

#define ENCRYPTED_WITH_IV { \
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, \
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f \
    }





psa_status_t mbedtls_psa_external_get_random(
    mbedtls_psa_external_random_context_t *context,
    uint8_t *output,
    size_t output_size,
    size_t *output_length)
{
    (void) context;
    return nvt_crpt_get_random(output, output_size, output_length);
}

void decrypt_with_symmetric_ciphers_2(const uint8_t *key, size_t key_len)
{
    enum
    {
        block_size = PSA_BLOCK_CIPHER_BLOCK_LENGTH(PSA_KEY_TYPE_AES),
    };

    static const uint8_t expected_plaintext[block_size] =
    {
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
        0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a
    };

    psa_status_t status;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_algorithm_t alg = PSA_ALG_CBC_NO_PADDING;
    psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;
    uint8_t ciphertext[block_size] = SOME_CIPHERTEXT;
    uint8_t iv[block_size] = ENCRYPTED_WITH_IV;
    uint8_t output[block_size];
    size_t update_output_len;
    size_t finish_output_len;
    size_t total_output_len;
    size_t index;
    psa_key_id_t key_id;

    printf("Decrypt with cipher...\t");
    fflush(stdout);

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto\n");
        return;
    }

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

    printf("Done import a key\n");

    psa_reset_key_attributes(&attributes);

    status = psa_cipher_decrypt_setup(&operation, key_id, alg);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to begin cipher operation\n");
        return;
    }

    printf("Done begin cipher operation\n");

    status = psa_cipher_set_iv(&operation, iv, sizeof(iv));

    if (status != PSA_SUCCESS)
    {
        printf("Failed to set IV\n");
        return;
    }

    printf("Done set IV\n");

    status = psa_cipher_update(&operation, ciphertext, sizeof(ciphertext),
                               output, sizeof(output), &update_output_len);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to update cipher operation\n");
        return;
    }

    printf("Done update cipher operation\n");

    status = psa_cipher_finish(&operation,
                               output + update_output_len,
                               sizeof(output) - update_output_len,
                               &finish_output_len);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to finish cipher operation\n");
        return;
    }

    total_output_len = update_output_len + finish_output_len;

    printf("Update_output_len=%u\n", (unsigned) update_output_len);
    printf("Finish_output_len=%u\n", (unsigned) finish_output_len);
    printf("Total_output_len=%u\n", (unsigned) total_output_len);
    printf("Final_output=");

    for (index = 0; index < total_output_len; index++)
    {
        printf("%02x", output[index]);
    }

    printf("\n");

    if (total_output_len != sizeof(expected_plaintext))
    {
        printf("Verification failed: expected %u plaintext bytes, got %u\n",
               (unsigned) sizeof(expected_plaintext),
               (unsigned) total_output_len);
    }
    else if (memcmp(output, expected_plaintext, sizeof(expected_plaintext)) != 0)
    {
        printf("Verification failed: decrypted plaintext mismatch\n");
        printf("Expected: ");

        for (index = 0; index < sizeof(expected_plaintext); index++)
        {
            printf("%02x", expected_plaintext[index]);
        }

        printf("\nActual  : ");

        for (index = 0; index < total_output_len; index++)
        {
            printf("%02x", output[index]);
        }

        printf("\n");
    }
    else
    {
        printf("Verification passed: AES-128-CBC decrypted plaintext matches the expected vector\n");
    }

    psa_cipher_abort(&operation);
    psa_destroy_key(key_id);
    mbedtls_psa_crypto_free();
}

static int encrypt_with_alg_and_pattern(psa_algorithm_t alg,
                                        const uint8_t *key,
                                        size_t key_len,
                                        const uint8_t *plaintext,
                                        size_t plaintext_len,
                                        const uint8_t *iv,
                                        size_t iv_len,
                                        const uint8_t *expected,
                                        size_t expected_len,
                                        const char *alg_name)
{
    psa_status_t status;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;
    psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;
    uint8_t output[64];
    size_t output_len = 0;
    size_t total_len = 0;

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("[%s] Failed to initialize PSA Crypto\n", alg_name);
        return 1;
    }

    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, alg);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);

    status = psa_import_key(&attributes, key, key_len, &key_id);
    psa_reset_key_attributes(&attributes);

    if (status != PSA_SUCCESS)
    {
        printf("[%s] Failed to import key\n", alg_name);
        mbedtls_psa_crypto_free();
        return 1;
    }

    status = psa_cipher_encrypt_setup(&operation, key_id, alg);

    if (status != PSA_SUCCESS)
    {
        printf("[%s] Failed to begin cipher operation\n", alg_name);
        psa_destroy_key(key_id);
        mbedtls_psa_crypto_free();
        return 1;
    }

    if (alg != PSA_ALG_ECB_NO_PADDING)
    {
        status = psa_cipher_set_iv(&operation, iv, iv_len);

        if (status != PSA_SUCCESS)
        {
            printf("[%s] Failed to set IV\n", alg_name);
            psa_cipher_abort(&operation);
            psa_destroy_key(key_id);
            mbedtls_psa_crypto_free();
            return 1;
        }
    }

    status = psa_cipher_update(&operation, plaintext, plaintext_len,
                               output, sizeof(output), &output_len);

    if (status != PSA_SUCCESS)
    {
        printf("[%s] Failed to update cipher operation\n", alg_name);
        psa_cipher_abort(&operation);
        psa_destroy_key(key_id);
        mbedtls_psa_crypto_free();
        return 1;
    }

    total_len += output_len;

    status = psa_cipher_finish(&operation, output + total_len,
                               sizeof(output) - total_len, &output_len);

    if (status != PSA_SUCCESS)
    {
        printf("[%s] Failed to finish cipher operation\n", alg_name);
        psa_cipher_abort(&operation);
        psa_destroy_key(key_id);
        mbedtls_psa_crypto_free();
        return 1;
    }

    total_len += output_len;

    psa_cipher_abort(&operation);
    psa_destroy_key(key_id);
    mbedtls_psa_crypto_free();

    if (total_len != expected_len ||
            memcmp(output, expected, expected_len) != 0)
    {
        size_t i;

        printf("[%s] Pattern mismatch\n", alg_name);
        printf("Expected: ");

        for (i = 0; i < expected_len; i++)
        {
            printf("%02x", expected[i]);
        }

        printf("\nActual  : ");

        for (i = 0; i < total_len; i++)
        {
            printf("%02x", output[i]);
        }

        printf("\n");
        return 1;
    }

    printf("[%s] Pattern verified\n", alg_name);
    return 0;
}

int encrypt_with_symmetric_ciphers_ecb_pattern(void)
{
    static const uint8_t key[16] =
    {
        0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
        0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
    };
    static const uint8_t plaintext[16] =
    {
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
        0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a
    };
    static const uint8_t expected[16] =
    {
        0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60,
        0xa8, 0x9e, 0xca, 0xf3, 0x24, 0x66, 0xef, 0x97
    };

    return encrypt_with_alg_and_pattern(PSA_ALG_ECB_NO_PADDING,
                                        key, sizeof(key),
                                        plaintext, sizeof(plaintext),
                                        NULL, 0,
                                        expected, sizeof(expected),
                                        "PSA_ALG_ECB_NO_PADDING");
}

int encrypt_with_symmetric_ciphers_cfb_pattern(void)
{
    static const uint8_t key[16] =
    {
        0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
        0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
    };
    static const uint8_t iv[16] =
    {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
    };
    static const uint8_t plaintext[16] =
    {
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
        0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a
    };
    static const uint8_t expected[16] =
    {
        0x3b, 0x3f, 0xd9, 0x2e, 0xb7, 0x2d, 0xad, 0x20,
        0x33, 0x34, 0x49, 0xf8, 0xe8, 0x3c, 0xfb, 0x4a
    };

    return encrypt_with_alg_and_pattern(PSA_ALG_CFB,
                                        key, sizeof(key),
                                        plaintext, sizeof(plaintext),
                                        iv, sizeof(iv),
                                        expected, sizeof(expected),
                                        "PSA_ALG_CFB");
}

int encrypt_with_symmetric_ciphers_ofb_pattern(void)
{
    static const uint8_t key[16] =
    {
        0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
        0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
    };
    static const uint8_t iv[16] =
    {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
    };
    static const uint8_t plaintext[16] =
    {
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
        0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a
    };
    static const uint8_t expected[16] =
    {
        0x3b, 0x3f, 0xd9, 0x2e, 0xb7, 0x2d, 0xad, 0x20,
        0x33, 0x34, 0x49, 0xf8, 0xe8, 0x3c, 0xfb, 0x4a
    };

    return encrypt_with_alg_and_pattern(PSA_ALG_OFB,
                                        key, sizeof(key),
                                        plaintext, sizeof(plaintext),
                                        iv, sizeof(iv),
                                        expected, sizeof(expected),
                                        "PSA_ALG_OFB");
}

static int test_aes_ctr_nist(void)
{
    static const uint8_t key[16] =
    {
        0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
        0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
    };
    static const uint8_t iv[16] =
    {
        0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
        0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
    };
    static const uint8_t plaintext[64] =
    {
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
        0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
        0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
        0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
        0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
        0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
        0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
        0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10
    };
    static const uint8_t expected_ciphertext[64] =
    {
        0x87, 0x4d, 0x61, 0x91, 0xb6, 0x20, 0xe3, 0x26,
        0x1b, 0xef, 0x68, 0x64, 0x99, 0x0d, 0xb6, 0xce,
        0x98, 0x06, 0xf6, 0x6b, 0x79, 0x70, 0xfd, 0xff,
        0x86, 0x17, 0x18, 0x7b, 0xb9, 0xff, 0xfd, 0xff,
        0x5a, 0xe4, 0xdf, 0x3e, 0xdb, 0xd5, 0xd3, 0x5e,
        0x5b, 0x4f, 0x09, 0x02, 0x0d, 0xb0, 0x3e, 0xab,
        0x1e, 0x03, 0x1d, 0xda, 0x2f, 0xbe, 0x03, 0xd1,
        0x79, 0x21, 0x70, 0xa0, 0xf3, 0x00, 0x9c, 0xee
    };
    static const size_t encrypt_chunks[] = {7U, 9U, 1U, 31U, 16U};
    static const size_t decrypt_chunks[] = {16U, 1U, 15U, 17U, 15U};
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;
    uint8_t output[sizeof(plaintext)];
    size_t offset;
    size_t total_length;
    size_t output_length;
    size_t finish_length;
    size_t index;
    psa_status_t status;
    int result = 1;

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("[AES-CTR] PSA initialization failed: %ld\n", (long) status);
        return 1;
    }

    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_CTR);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128U);
    status = psa_import_key(&attributes, key, sizeof(key), &key_id);
    psa_reset_key_attributes(&attributes);

    if (status != PSA_SUCCESS)
    {
        printf("[AES-CTR] Key import failed: %ld\n", (long) status);
        goto cleanup;
    }

    for (index = 0U; index < 2U; index++)
    {
        const uint8_t *input = (index == 0U) ? plaintext : expected_ciphertext;
        const uint8_t *expected = (index == 0U) ? expected_ciphertext : plaintext;
        const size_t *chunks = (index == 0U) ? encrypt_chunks : decrypt_chunks;
        size_t chunk_count = (index == 0U)
                             ? sizeof(encrypt_chunks) / sizeof(encrypt_chunks[0])
                             : sizeof(decrypt_chunks) / sizeof(decrypt_chunks[0]);

        status = (index == 0U)
                 ? psa_cipher_encrypt_setup(&operation, key_id, PSA_ALG_CTR)
                 : psa_cipher_decrypt_setup(&operation, key_id, PSA_ALG_CTR);

        if (status == PSA_SUCCESS)
        {
            status = psa_cipher_set_iv(&operation, iv, sizeof(iv));
        }

        if (status != PSA_SUCCESS)
        {
            printf("[AES-CTR] %s setup failed: %ld\n",
                   index == 0U ? "Encrypt" : "Decrypt", (long) status);
            goto cleanup;
        }

        offset = 0U;
        total_length = 0U;

        for (size_t chunk_index = 0U; chunk_index < chunk_count; chunk_index++)
        {
            status = psa_cipher_update(&operation,
                                       input + offset,
                                       chunks[chunk_index],
                                       output + total_length,
                                       sizeof(output) - total_length,
                                       &output_length);

            if (status != PSA_SUCCESS || output_length != chunks[chunk_index])
            {
                printf("[AES-CTR] Update %lu failed: status=%ld, output=%lu\n",
                       (unsigned long) chunk_index, (long) status,
                       (unsigned long) output_length);
                goto cleanup;
            }

            offset += chunks[chunk_index];
            total_length += output_length;
        }

        status = psa_cipher_finish(&operation,
                                   output + total_length,
                                   sizeof(output) - total_length,
                                   &finish_length);

        if (status != PSA_SUCCESS || finish_length != 0U ||
                total_length != sizeof(output) ||
                memcmp(output, expected, sizeof(output)) != 0)
        {
            printf("[AES-CTR] %s vector mismatch: status=%ld\n",
                   index == 0U ? "Encrypt" : "Decrypt", (long) status);
            goto cleanup;
        }

        psa_cipher_abort(&operation);
        printf("[PASS] AES-128-CTR %s NIST vector (multipart)\n",
               index == 0U ? "encrypt" : "decrypt");
    }

    result = 0;

cleanup:
    psa_cipher_abort(&operation);

    if (key_id != PSA_KEY_ID_NULL)
    {
        psa_destroy_key(key_id);
    }

    mbedtls_psa_crypto_free();
    return result;
}

static int test_aes_cmac_nist(void)
{
    static const uint8_t key_128[16] =
    {
        0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
        0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
    };
    static const uint8_t key_192[24] =
    {
        0x8e, 0x73, 0xb0, 0xf7, 0xda, 0x0e, 0x64, 0x52,
        0xc8, 0x10, 0xf3, 0x2b, 0x80, 0x90, 0x79, 0xe5,
        0x62, 0xf8, 0xea, 0xd2, 0x52, 0x2c, 0x6b, 0x7b
    };
    static const uint8_t key_256[32] =
    {
        0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe,
        0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
        0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7,
        0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4
    };
    static const uint8_t invalid_key_160[20] = {0};
    static const uint8_t message[64] =
    {
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
        0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
        0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
        0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
        0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
        0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
        0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
        0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10
    };
    static const uint8_t expected_empty[16] =
    {
        0xbb, 0x1d, 0x69, 0x29, 0xe9, 0x59, 0x37, 0x28,
        0x7f, 0xa3, 0x7d, 0x12, 0x9b, 0x75, 0x67, 0x46
    };
    static const uint8_t expected_one_block[16] =
    {
        0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
        0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7c
    };
    static const uint8_t expected_128_20_bytes[16] =
    {
        0x7d, 0x85, 0x44, 0x9e, 0xa6, 0xea, 0x19, 0xc8,
        0x23, 0xa7, 0xbf, 0x78, 0x83, 0x7d, 0xfa, 0xde
    };
    static const uint8_t expected_128_64_bytes[16] =
    {
        0x51, 0xf0, 0xbe, 0xbf, 0x7e, 0x3b, 0x9d, 0x92,
        0xfc, 0x49, 0x74, 0x17, 0x79, 0x36, 0x3c, 0xfe
    };
    static const uint8_t expected_192_empty[16] =
    {
        0xd1, 0x7d, 0xdf, 0x46, 0xad, 0xaa, 0xcd, 0xe5,
        0x31, 0xca, 0xc4, 0x83, 0xde, 0x7a, 0x93, 0x67
    };
    static const uint8_t expected_192_one_block[16] =
    {
        0x9e, 0x99, 0xa7, 0xbf, 0x31, 0xe7, 0x10, 0x90,
        0x06, 0x62, 0xf6, 0x5e, 0x61, 0x7c, 0x51, 0x84
    };
    static const uint8_t expected_192_20_bytes[16] =
    {
        0x3d, 0x75, 0xc1, 0x94, 0xed, 0x96, 0x07, 0x04,
        0x44, 0xa9, 0xfa, 0x7e, 0xc7, 0x40, 0xec, 0xf8
    };
    static const uint8_t expected_192_64_bytes[16] =
    {
        0xa1, 0xd5, 0xdf, 0x0e, 0xed, 0x79, 0x0f, 0x79,
        0x4d, 0x77, 0x58, 0x96, 0x59, 0xf3, 0x9a, 0x11
    };
    static const uint8_t expected_256_empty[16] =
    {
        0x02, 0x89, 0x62, 0xf6, 0x1b, 0x7b, 0xf8, 0x9e,
        0xfc, 0x6b, 0x55, 0x1f, 0x46, 0x67, 0xd9, 0x83
    };
    static const uint8_t expected_256_one_block[16] =
    {
        0x28, 0xa7, 0x02, 0x3f, 0x45, 0x2e, 0x8f, 0x82,
        0xbd, 0x4b, 0xf2, 0x8d, 0x8c, 0x37, 0xc3, 0x5c
    };
    static const uint8_t expected_256_20_bytes[16] =
    {
        0x15, 0x67, 0x27, 0xdc, 0x08, 0x78, 0x94, 0x4a,
        0x02, 0x3c, 0x1f, 0xe0, 0x3b, 0xad, 0x6d, 0x93
    };
    static const uint8_t expected_256_64_bytes[16] =
    {
        0xe1, 0x99, 0x21, 0x90, 0x54, 0x9f, 0x6e, 0xd5,
        0x69, 0x6a, 0x2c, 0x05, 0x6c, 0x31, 0x54, 0x10
    };
    static const struct
    {
        const uint8_t *key;
        size_t key_length;
        const char *name;
        const uint8_t *expected_empty;
        const uint8_t *expected_one_block;
        const uint8_t *expected_20_bytes;
        const uint8_t *expected_64_bytes;
    } key_cases[] =
    {
        {
            key_128, sizeof(key_128), "AES-128",
            expected_empty, expected_one_block, expected_128_20_bytes,
            expected_128_64_bytes
        },
        {
            key_192, sizeof(key_192), "AES-192",
            expected_192_empty, expected_192_one_block,
            expected_192_20_bytes, expected_192_64_bytes
        },
        {
            key_256, sizeof(key_256), "AES-256",
            expected_256_empty, expected_256_one_block,
            expected_256_20_bytes, expected_256_64_bytes
        }
    };
    static const struct
    {
        const uint8_t *input;
        size_t input_length;
        const char *name;
    } message_cases[] =
    {
        {NULL, 0u, "empty"},
        {message, 16u, "one-block"},
        {message, 20u, "20-byte"},
        {message, sizeof(message), "64-byte"}
    };
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    uint8_t actual[16];
    size_t actual_length;
    size_t key_index;
    size_t message_index;
    psa_status_t status;
    int result = 1;

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("[CMAC] PSA initialization failed: %ld\n", (long) status);
        return 1;
    }

    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attributes,
                          PSA_ALG_AT_LEAST_THIS_LENGTH_MAC(PSA_ALG_CMAC, 8u));
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 160u);
    status = psa_import_key(&attributes, invalid_key_160,
                            sizeof(invalid_key_160), &key_id);
    psa_reset_key_attributes(&attributes);

    if (status != PSA_ERROR_INVALID_ARGUMENT || key_id != PSA_KEY_ID_NULL)
    {
        printf("[FAIL] Invalid AES-CMAC key size was not rejected: status=%ld\n",
               (long) status);

        if (key_id != PSA_KEY_ID_NULL)
        {
            psa_destroy_key(key_id);
            key_id = PSA_KEY_ID_NULL;
        }

        goto cleanup;
    }

    printf("[PASS] Invalid AES-CMAC key size rejected at import\n");

    for (key_index = 0u;
            key_index < sizeof(key_cases) / sizeof(key_cases[0]); key_index++)
    {
        psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE |
                                PSA_KEY_USAGE_VERIFY_MESSAGE);
        psa_set_key_algorithm(&attributes,
                              PSA_ALG_AT_LEAST_THIS_LENGTH_MAC(PSA_ALG_CMAC, 8u));
        psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
        psa_set_key_bits(&attributes, key_cases[key_index].key_length * 8u);
        status = psa_import_key(&attributes, key_cases[key_index].key,
                                key_cases[key_index].key_length, &key_id);
        psa_reset_key_attributes(&attributes);

        if (status != PSA_SUCCESS)
        {
            printf("[CMAC] %s key import failed: %ld\n",
                   key_cases[key_index].name, (long) status);
            goto cleanup;
        }

        for (message_index = 0u;
                message_index < sizeof(message_cases) / sizeof(message_cases[0]);
                message_index++)
        {
            const uint8_t *expected = message_index == 0u ?
                                      key_cases[key_index].expected_empty :
                                      message_index == 1u ? key_cases[key_index].expected_one_block :
                                      message_index == 2u ? key_cases[key_index].expected_20_bytes :
                                      key_cases[key_index].expected_64_bytes;

            status = psa_mac_compute(
                         key_id, PSA_ALG_TRUNCATED_MAC(PSA_ALG_CMAC, 16u),
                         message_cases[message_index].input,
                         message_cases[message_index].input_length,
                         actual, sizeof(actual), &actual_length);

            if (status != PSA_SUCCESS || actual_length != sizeof(actual) ||
                    memcmp(actual, expected, sizeof(actual)) != 0)
            {
                printf("[FAIL] %s CMAC %s builtin oracle: status=%ld\n",
                       key_cases[key_index].name,
                       message_cases[message_index].name, (long) status);
                goto cleanup;
            }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

            if (nvt_psa_cmac_last_compute_path != NVT_PSA_CMAC_PATH_BUILTIN_FALLBACK)
            {
                printf("[FAIL] %s CMAC %s oracle did not use builtin fallback\n",
                       key_cases[key_index].name,
                       message_cases[message_index].name);
                goto cleanup;
            }

#endif
            printf("[PASS] %s CMAC %s builtin oracle vector\n",
                   key_cases[key_index].name,
                   message_cases[message_index].name);

            status = psa_mac_compute(key_id, PSA_ALG_CMAC,
                                     message_cases[message_index].input,
                                     message_cases[message_index].input_length,
                                     actual, sizeof(actual), &actual_length);

            if (status != PSA_SUCCESS || actual_length != sizeof(actual) ||
                    memcmp(actual, expected, sizeof(actual)) != 0)
            {
                printf("[FAIL] %s CMAC %s NIST vector: status=%ld\n",
                       key_cases[key_index].name,
                       message_cases[message_index].name, (long) status);
                printf("[CMAC] actual=");

                for (size_t byte_index = 0u; byte_index < sizeof(actual); byte_index++)
                {
                    printf("%02x", actual[byte_index]);
                }

                printf(" expected=");

                for (size_t byte_index = 0u; byte_index < sizeof(actual); byte_index++)
                {
                    printf("%02x", expected[byte_index]);
                }

                printf("\n");
                goto cleanup;
            }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

            if (nvt_psa_cmac_last_compute_path != NVT_PSA_CMAC_PATH_HARDWARE_AES)
            {
                printf("[FAIL] %s CMAC %s did not use CRPT AES\n",
                       key_cases[key_index].name,
                       message_cases[message_index].name);
                goto cleanup;
            }

            printf("[PATH] %s CMAC %s: software composition + hardware AES\n",
                   key_cases[key_index].name,
                   message_cases[message_index].name);
#endif
            printf("[PASS] %s CMAC %s NIST vector\n",
                   key_cases[key_index].name,
                   message_cases[message_index].name);
        }

        status = psa_mac_compute(key_id,
                                 PSA_ALG_TRUNCATED_MAC(PSA_ALG_CMAC, 8u),
                                 message, sizeof(message), actual,
                                 sizeof(actual), &actual_length);

        if (status != PSA_SUCCESS || actual_length != 8u ||
                memcmp(actual, key_cases[key_index].expected_64_bytes, 8u) != 0)
        {
            printf("[FAIL] %s truncated CMAC builtin fallback: status=%ld\n",
                   key_cases[key_index].name, (long) status);
            goto cleanup;
        }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

        if (nvt_psa_cmac_last_compute_path != NVT_PSA_CMAC_PATH_BUILTIN_FALLBACK)
        {
            printf("[FAIL] %s truncated CMAC did not use builtin fallback\n",
                   key_cases[key_index].name);
            goto cleanup;
        }

        printf("[PATH] %s truncated CMAC: builtin fallback\n",
               key_cases[key_index].name);
#endif
        printf("[PASS] %s truncated CMAC vector\n", key_cases[key_index].name);

        actual_length = 0u;
        status = psa_mac_compute(key_id, PSA_ALG_CMAC,
                                 message, 16u, actual, sizeof(actual) - 1u,
                                 &actual_length);

        if (status != PSA_ERROR_BUFFER_TOO_SMALL)
        {
            printf("[FAIL] %s CMAC too-small output status=%ld\n",
                   key_cases[key_index].name, (long) status);
            goto cleanup;
        }

        printf("[PASS] %s CMAC too-small output rejected\n",
               key_cases[key_index].name);

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

        if (key_index == 0u)
        {
            actual_length = 0u;
            nvt_psa_test_force_hardware_failure = 1u;
            status = psa_mac_compute(key_id, PSA_ALG_CMAC,
                                     message, 16u, actual, sizeof(actual),
                                     &actual_length);
            nvt_psa_test_force_hardware_failure = 0u;

            if (status != PSA_ERROR_HARDWARE_FAILURE ||
                    nvt_psa_cmac_last_compute_path != NVT_PSA_CMAC_PATH_FAILED)
            {
                printf("[FAIL] CMAC hardware error was not propagated: status=%ld\n",
                       (long) status);
                goto cleanup;
            }

            printf("[PASS] CMAC hardware error propagates without fallback\n");
        }

#endif

        psa_destroy_key(key_id);
        key_id = PSA_KEY_ID_NULL;
    }

    result = 0;

cleanup:
#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
    nvt_psa_test_force_hardware_failure = 0u;
#endif

    if (key_id != PSA_KEY_ID_NULL)
    {
        psa_destroy_key(key_id);
    }

    mbedtls_psa_crypto_free();
    return result;
}

int main_task(void)
{
    static const uint8_t key[] =
    {
        0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
        0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
    };

    printf("Run symmetric_ciphers_decrypt_test\n");
    decrypt_with_symmetric_ciphers(key, sizeof(key));

    printf("Run symmetric_ciphers_encrypt_test\n");
    encrypt_with_symmetric_ciphers(key, sizeof(key));

    printf("Run hash_a_message_test\n");
    hash_a_message();

    printf("Run verify_a_hash_test\n");
    verify_a_hash();

    printf("Run symmetric_ciphers_ecb_test\n");
    encrypt_with_symmetric_ciphers_ecb_pattern();
    printf("Done!\n");

    printf("Run symmetric_ciphers_cfb_test\n");
    encrypt_with_symmetric_ciphers_cfb_pattern();
    printf("Done!\n");

    printf("Run symmetric_ciphers_ofb_test\n");
    encrypt_with_symmetric_ciphers_ofb_pattern();
    printf("Done!\n");

    printf("Run symmetric_ciphers_ctr_test\n");
    test_aes_ctr_nist();
    printf("Done!\n");

    printf("Run symmetric_ciphers_cmac_test\n");

    if (test_aes_cmac_nist() != 0)
    {
        printf("[FAIL] CMAC regression suite failed; stopping sample\n");
        return 1;
    }

    printf("Done!\n");

    printf("Run authenticate_and_encrypt_a_message_gcm_test\n");
    authenticate_and_encrypt_a_message_gcm();
    printf("Done!\n");

    printf("Run authenticate_and_decrypt_a_message_gcm_test\n");
    authenticate_and_decrypt_a_message_gcm();
    printf("Done!\n");

    //    printf("nvt_aes_self_test\n");
    //    nvt_aes_self_test();
    //    printf("Done!\n");

    return 0;
}