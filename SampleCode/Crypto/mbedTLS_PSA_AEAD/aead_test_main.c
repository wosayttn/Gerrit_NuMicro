#include <stdio.h>
#include <string.h>
#include "psa/crypto.h"

static const uint8_t ccm_key[16] =
{
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf
};
static const uint8_t ccm_nonce[13] =
{
    0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00,
    0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5
};
static const uint8_t ccm_aad[8] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};
static const uint8_t ccm_plaintext[23] =
{
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e
};
static const uint8_t ccm_expected[31] =
{
    0x58, 0x8c, 0x97, 0x9a, 0x61, 0xc6, 0x63, 0xd2,
    0xf0, 0x66, 0xd0, 0xc2, 0xc0, 0xf9, 0x89, 0x80,
    0x6d, 0x5f, 0x6b, 0x61, 0xda, 0xc3, 0x84,
    0x17, 0xe8, 0xd1, 0x2c, 0xfd, 0xf9, 0x26, 0xe0
};

static const uint8_t gcm_key[16] = {0};
static const uint8_t gcm_nonce[12] = {0};
static const uint8_t gcm_aad[16] = {0};
static const uint8_t gcm_plaintext[16] = {0};
static const uint8_t gcm_expected[32] =
{
    0x03, 0x88, 0xda, 0xce, 0x60, 0xb6, 0xa3, 0x92,
    0xf3, 0x28, 0xc2, 0xb9, 0x71, 0xb2, 0xfe, 0x78,
    0xd2, 0x4e, 0x50, 0x3a, 0x1b, 0xb0, 0x37, 0x07,
    0x1c, 0x71, 0xb3, 0x5d, 0x98, 0x7b, 0x86, 0x57
};

static psa_status_t import_aes_key(const uint8_t *key,
                                   size_t key_length,
                                   psa_algorithm_t alg,
                                   psa_key_usage_t usage,
                                   psa_key_id_t *key_id)
{
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_status_t status;

    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, (uint32_t)(key_length * 8u));
    psa_set_key_usage_flags(&attributes, usage);
    psa_set_key_algorithm(&attributes, alg);
    status = psa_import_key(&attributes, key, key_length, key_id);
    psa_reset_key_attributes(&attributes);
    return status;
}

static int check_decrypt_failure(psa_key_id_t key_id,
                                 psa_algorithm_t alg,
                                 const uint8_t *nonce,
                                 size_t nonce_length,
                                 const uint8_t *aad,
                                 size_t aad_length,
                                 const uint8_t *ciphertext,
                                 size_t ciphertext_length,
                                 size_t payload_length)
{
    uint8_t bad_ciphertext[64];
    uint8_t plaintext[64];
    size_t plaintext_length = 99u;
    psa_status_t status;
    size_t index;

    if (ciphertext_length > sizeof(bad_ciphertext) ||
            payload_length > sizeof(plaintext))
    {
        return 1;
    }

    memcpy(bad_ciphertext, ciphertext, ciphertext_length);
    bad_ciphertext[ciphertext_length - 1u] ^= 1u;
    memset(plaintext, 0xa5, sizeof(plaintext));
    status = psa_aead_decrypt(key_id, alg, nonce, nonce_length,
                              aad, aad_length, bad_ciphertext,
                              ciphertext_length, plaintext, sizeof(plaintext),
                              &plaintext_length);

    if (status != PSA_ERROR_INVALID_SIGNATURE || plaintext_length != 0u)
    {
        printf("[FAIL] AEAD bad-tag status=%ld length=%lu\n",
               (long) status, (unsigned long) plaintext_length);
        return 1;
    }

    for (index = 0u; index < payload_length; index++)
    {
        if (plaintext[index] != 0u)
        {
            printf("[FAIL] AEAD plaintext was not cleared after bad tag\n");
            return 1;
        }
    }

    return 0;
}

static int test_ccm_known_answer(void)
{
    const psa_algorithm_t alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 8);
    uint8_t ciphertext[sizeof(ccm_expected)];
    uint8_t plaintext[sizeof(ccm_plaintext)];
    size_t ciphertext_length = 0u;
    size_t plaintext_length = 0u;
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    psa_status_t status;
    int result = 1;

    status = import_aes_key(ccm_key, sizeof(ccm_key), alg,
                            PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT,
                            &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] CCM key import: %ld\n", (long) status);
        return 1;
    }

    status = psa_aead_encrypt(key_id, alg, ccm_nonce, sizeof(ccm_nonce),
                              ccm_aad, sizeof(ccm_aad), ccm_plaintext,
                              sizeof(ccm_plaintext), ciphertext,
                              sizeof(ciphertext), &ciphertext_length);

    if (status != PSA_SUCCESS || ciphertext_length != sizeof(ccm_expected) ||
            memcmp(ciphertext, ccm_expected, sizeof(ccm_expected)) != 0)
    {
        printf("[FAIL] CCM RFC 3610 encrypt: status=%ld length=%lu\n",
               (long) status, (unsigned long) ciphertext_length);
        goto cleanup;
    }

    status = psa_aead_decrypt(key_id, alg, ccm_nonce, sizeof(ccm_nonce),
                              ccm_aad, sizeof(ccm_aad), ciphertext,
                              ciphertext_length, plaintext, sizeof(plaintext),
                              &plaintext_length);

    if (status != PSA_SUCCESS || plaintext_length != sizeof(ccm_plaintext) ||
            memcmp(plaintext, ccm_plaintext, sizeof(ccm_plaintext)) != 0)
    {
        printf("[FAIL] CCM RFC 3610 decrypt: status=%ld length=%lu\n",
               (long) status, (unsigned long) plaintext_length);
        goto cleanup;
    }

    if (check_decrypt_failure(key_id, alg, ccm_nonce, sizeof(ccm_nonce),
                              ccm_aad, sizeof(ccm_aad), ciphertext,
                              ciphertext_length, sizeof(ccm_plaintext)) != 0)
    {
        goto cleanup;
    }

    printf("[PASS] PSA AES-CCM RFC 3610 encrypt/decrypt and bad-tag rejection\n");
    result = 0;

cleanup:
    psa_destroy_key(key_id);
    return result;
}

static int test_ccm_multipart(void)
{
    const psa_algorithm_t alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 8);
    psa_aead_operation_t operation = PSA_AEAD_OPERATION_INIT;
    uint8_t ciphertext[sizeof(ccm_plaintext)];
    uint8_t tag[sizeof(ccm_expected) - sizeof(ccm_plaintext)];
    uint8_t plaintext[sizeof(ccm_plaintext)];
    size_t output_length = 0u;
    size_t ciphertext_length = 0u;
    size_t tag_length = 0u;
    size_t plaintext_length = 0u;
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    psa_status_t status;
    int result = 1;

    status = import_aes_key(ccm_key, sizeof(ccm_key), alg,
                            PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT,
                            &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] CCM multipart key import: %ld\n", (long) status);
        return 1;
    }

    status = psa_aead_encrypt_setup(&operation, key_id, alg);

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_nonce(&operation, ccm_nonce, sizeof(ccm_nonce));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_abort(&operation);
    }

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] CCM multipart abort: %ld\n", (long) status);
        psa_aead_abort(&operation);
        goto cleanup;
    }

    operation = (psa_aead_operation_t) PSA_AEAD_OPERATION_INIT;
    status = psa_aead_encrypt_setup(&operation, key_id, alg);

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_nonce(&operation, ccm_nonce, sizeof(ccm_nonce));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_lengths(&operation, sizeof(ccm_aad),
                                      sizeof(ccm_plaintext));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update_ad(&operation, ccm_aad, 3u);
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update_ad(&operation, ccm_aad + 3u,
                                    sizeof(ccm_aad) - 3u);
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update(&operation, ccm_plaintext, 9u, ciphertext,
                                 sizeof(ciphertext), &output_length);
    }

    if (status == PSA_SUCCESS && output_length != 0u)
    {
        status = PSA_ERROR_CORRUPTION_DETECTED;
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update(&operation, ccm_plaintext + 9u,
                                 sizeof(ccm_plaintext) - 9u, ciphertext,
                                 sizeof(ciphertext), &output_length);
    }

    if (status == PSA_SUCCESS && output_length != 0u)
    {
        status = PSA_ERROR_CORRUPTION_DETECTED;
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_finish(&operation, ciphertext, sizeof(ciphertext),
                                 &ciphertext_length, tag, sizeof(tag),
                                 &tag_length);
    }

    if (status != PSA_SUCCESS || ciphertext_length != sizeof(ccm_plaintext) ||
            tag_length != sizeof(tag) ||
            memcmp(ciphertext, ccm_expected, sizeof(ciphertext)) != 0 ||
            memcmp(tag, ccm_expected + sizeof(ciphertext), sizeof(tag)) != 0)
    {
        printf("[FAIL] CCM multipart encrypt: status=%ld ciphertext=%lu tag=%lu\n",
               (long) status, (unsigned long) ciphertext_length,
               (unsigned long) tag_length);
        psa_aead_abort(&operation);
        goto cleanup;
    }

    operation = (psa_aead_operation_t) PSA_AEAD_OPERATION_INIT;
    status = psa_aead_decrypt_setup(&operation, key_id, alg);

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_nonce(&operation, ccm_nonce, sizeof(ccm_nonce));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_lengths(&operation, sizeof(ccm_aad),
                                      sizeof(ccm_plaintext));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update_ad(&operation, ccm_aad, 3u);
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update_ad(&operation, ccm_aad + 3u,
                                    sizeof(ccm_aad) - 3u);
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update(&operation, ciphertext, 9u, plaintext,
                                 sizeof(plaintext), &output_length);
    }

    if (status == PSA_SUCCESS && output_length != 0u)
    {
        status = PSA_ERROR_CORRUPTION_DETECTED;
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update(&operation, ciphertext + 9u,
                                 sizeof(ccm_plaintext) - 9u, plaintext,
                                 sizeof(plaintext), &output_length);
    }

    if (status == PSA_SUCCESS && output_length != 0u)
    {
        status = PSA_ERROR_CORRUPTION_DETECTED;
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_verify(&operation, plaintext, sizeof(plaintext),
                                 &plaintext_length, tag, tag_length);
    }

    if (status != PSA_SUCCESS || plaintext_length != sizeof(ccm_plaintext) ||
            memcmp(plaintext, ccm_plaintext, sizeof(ccm_plaintext)) != 0)
    {
        printf("[FAIL] CCM multipart decrypt: status=%ld plaintext=%lu\n",
               (long) status, (unsigned long) plaintext_length);
        psa_aead_abort(&operation);
        goto cleanup;
    }

    printf("[PASS] PSA AES-CCM multipart split-AAD/data encrypt/decrypt\n");
    result = 0;

cleanup:
    psa_destroy_key(key_id);
    return result;
}

static int test_gcm_known_answer(void)
{
    const psa_algorithm_t alg = PSA_ALG_GCM;
    uint8_t ciphertext[sizeof(gcm_expected)];
    uint8_t plaintext[sizeof(gcm_plaintext)];
    size_t ciphertext_length = 0u;
    size_t plaintext_length = 0u;
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    psa_status_t status;
    int result = 1;

    status = import_aes_key(gcm_key, sizeof(gcm_key), alg,
                            PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT,
                            &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] GCM key import: %ld\n", (long) status);
        return 1;
    }

    status = psa_aead_encrypt(key_id, alg, gcm_nonce, sizeof(gcm_nonce),
                              gcm_aad, sizeof(gcm_aad), gcm_plaintext,
                              sizeof(gcm_plaintext), ciphertext,
                              sizeof(ciphertext), &ciphertext_length);

    if (status != PSA_SUCCESS || ciphertext_length != sizeof(gcm_expected) ||
            memcmp(ciphertext, gcm_expected, sizeof(gcm_expected)) != 0)
    {
        printf("[FAIL] GCM zero-input KAT encrypt: status=%ld length=%lu\n",
               (long) status, (unsigned long) ciphertext_length);
        goto cleanup;
    }

    status = psa_aead_decrypt(key_id, alg, gcm_nonce, sizeof(gcm_nonce),
                              gcm_aad, sizeof(gcm_aad), ciphertext,
                              ciphertext_length, plaintext, sizeof(plaintext),
                              &plaintext_length);

    if (status != PSA_SUCCESS || plaintext_length != sizeof(gcm_plaintext) ||
            memcmp(plaintext, gcm_plaintext, sizeof(gcm_plaintext)) != 0)
    {
        printf("[FAIL] GCM zero-input KAT decrypt: status=%ld length=%lu\n",
               (long) status, (unsigned long) plaintext_length);
        goto cleanup;
    }

    if (check_decrypt_failure(key_id, alg, gcm_nonce, sizeof(gcm_nonce),
                              gcm_aad, sizeof(gcm_aad), ciphertext,
                              ciphertext_length, sizeof(gcm_plaintext)) != 0)
    {
        goto cleanup;
    }

    printf("[PASS] PSA AES-GCM zero-input KAT encrypt/decrypt and bad-tag rejection\n");
    result = 0;

cleanup:
    psa_destroy_key(key_id);
    return result;
}

static int test_gcm_multipart(void)
{
    const psa_algorithm_t alg = PSA_ALG_GCM;
    psa_aead_operation_t operation = PSA_AEAD_OPERATION_INIT;
    uint8_t ciphertext[sizeof(gcm_plaintext)];
    uint8_t tag[sizeof(gcm_expected) - sizeof(gcm_plaintext)];
    uint8_t plaintext[sizeof(gcm_plaintext)];
    size_t output_length;
    size_t ciphertext_length = 0u;
    size_t tag_length = 0u;
    size_t plaintext_length = 0u;
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    psa_status_t status;
    int result = 1;

    status = import_aes_key(gcm_key, sizeof(gcm_key), alg,
                            PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT,
                            &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] GCM multipart key import: %ld\n", (long) status);
        return 1;
    }

    status = psa_aead_encrypt_setup(&operation, key_id, alg);

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_nonce(&operation, gcm_nonce, sizeof(gcm_nonce));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update_ad(&operation, gcm_aad, 5u);
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_abort(&operation);
    }

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] GCM multipart abort: %ld\n", (long) status);
        psa_aead_abort(&operation);
        goto cleanup;
    }

    operation = (psa_aead_operation_t) PSA_AEAD_OPERATION_INIT;
    status = psa_aead_encrypt_setup(&operation, key_id, alg);

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_nonce(&operation, gcm_nonce, sizeof(gcm_nonce));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_lengths(&operation, sizeof(gcm_aad),
                                      sizeof(gcm_plaintext));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update_ad(&operation, gcm_aad, 5u);
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update_ad(&operation, gcm_aad + 5u,
                                    sizeof(gcm_aad) - 5u);
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update(&operation, gcm_plaintext, 7u,
                                 ciphertext, sizeof(ciphertext), &output_length);
    }

    if (status == PSA_SUCCESS && output_length != 0u)
    {
        status = PSA_ERROR_CORRUPTION_DETECTED;
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update(&operation, gcm_plaintext + 7u,
                                 sizeof(gcm_plaintext) - 7u,
                                 ciphertext, sizeof(ciphertext), &output_length);
    }

    if (status == PSA_SUCCESS && output_length != 0u)
    {
        status = PSA_ERROR_CORRUPTION_DETECTED;
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_finish(&operation, ciphertext, sizeof(ciphertext),
                                 &ciphertext_length,
                                 tag, sizeof(tag),
                                 &tag_length);
    }

    if (status != PSA_SUCCESS || ciphertext_length != sizeof(gcm_plaintext) ||
            tag_length != sizeof(tag) ||
            memcmp(ciphertext, gcm_expected, sizeof(ciphertext)) != 0 ||
            memcmp(tag, gcm_expected + sizeof(ciphertext), sizeof(tag)) != 0)
    {
        printf("[FAIL] GCM multipart encrypt: status=%ld ciphertext=%lu tag=%lu\n",
               (long) status, (unsigned long) ciphertext_length,
               (unsigned long) tag_length);
        psa_aead_abort(&operation);
        goto cleanup;
    }

    operation = (psa_aead_operation_t) PSA_AEAD_OPERATION_INIT;
    status = psa_aead_decrypt_setup(&operation, key_id, alg);

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_nonce(&operation, gcm_nonce, sizeof(gcm_nonce));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_set_lengths(&operation, sizeof(gcm_aad),
                                      sizeof(gcm_plaintext));
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update_ad(&operation, gcm_aad, 5u);
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update_ad(&operation, gcm_aad + 5u,
                                    sizeof(gcm_aad) - 5u);
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update(&operation, ciphertext, 7u,
                                 plaintext, sizeof(plaintext), &output_length);
    }

    if (status == PSA_SUCCESS && output_length != 0u)
    {
        status = PSA_ERROR_CORRUPTION_DETECTED;
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_update(&operation, ciphertext + 7u,
                                 sizeof(gcm_plaintext) - 7u,
                                 plaintext, sizeof(plaintext), &output_length);
    }

    if (status == PSA_SUCCESS && output_length != 0u)
    {
        status = PSA_ERROR_CORRUPTION_DETECTED;
    }

    if (status == PSA_SUCCESS)
    {
        status = psa_aead_verify(&operation, plaintext, sizeof(plaintext),
                                 &plaintext_length,
                                 tag, tag_length);
    }

    if (status != PSA_SUCCESS || plaintext_length != sizeof(gcm_plaintext) ||
            memcmp(plaintext, gcm_plaintext, sizeof(gcm_plaintext)) != 0)
    {
        printf("[FAIL] GCM multipart decrypt: status=%ld plaintext=%lu\n",
               (long) status, (unsigned long) plaintext_length);
        psa_aead_abort(&operation);
        goto cleanup;
    }

    printf("[PASS] PSA AES-GCM multipart split-AAD/data encrypt/decrypt\n");
    result = 0;

cleanup:
    psa_destroy_key(key_id);
    return result;
}

static int test_ccm_unsupported_request_fallback(void)
{
    const psa_algorithm_t alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 8);
    uint8_t ciphertext[sizeof(ccm_plaintext) + 8u];
    uint8_t plaintext[sizeof(ccm_plaintext)];
    size_t ciphertext_length = 0u;
    size_t plaintext_length = 0u;
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    psa_status_t status;
    int result = 1;

    status = import_aes_key(ccm_key, sizeof(ccm_key), alg,
                            PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT,
                            &key_id);

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] CCM fallback key import: %ld\n", (long) status);
        return 1;
    }

    status = psa_aead_encrypt(key_id, alg, ccm_nonce, sizeof(ccm_nonce),
                              NULL, 0u, ccm_plaintext, sizeof(ccm_plaintext),
                              ciphertext, sizeof(ciphertext), &ciphertext_length);

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] CCM unsupported-request fallback encrypt: %ld\n",
               (long) status);
        goto cleanup;
    }

    status = psa_aead_decrypt(key_id, alg, ccm_nonce, sizeof(ccm_nonce),
                              NULL, 0u, ciphertext, ciphertext_length,
                              plaintext, sizeof(plaintext), &plaintext_length);

    if (status != PSA_SUCCESS || plaintext_length != sizeof(ccm_plaintext) ||
            memcmp(plaintext, ccm_plaintext, sizeof(ccm_plaintext)) != 0)
    {
        printf("[FAIL] CCM unsupported-request fallback decrypt: %ld\n",
               (long) status);
        goto cleanup;
    }

    printf("[PASS] PSA AES-CCM unsupported empty-AAD request falls back\n");
    result = 0;

cleanup:
    psa_destroy_key(key_id);
    return result;
}

int run_aead_tests(void)
{
    if (test_ccm_known_answer() != 0 ||
            test_ccm_multipart() != 0 ||
            test_gcm_known_answer() != 0 ||
            test_gcm_multipart() != 0 ||
            test_ccm_unsupported_request_fallback() != 0)
    {
        return 1;
    }

    return 0;
}