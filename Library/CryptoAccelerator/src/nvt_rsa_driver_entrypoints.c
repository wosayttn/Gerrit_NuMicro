/*
 * Nuvoton transparent RSA PSA driver entry points: implementation.
 * See nvt_rsa_driver_entrypoints.h for the supported scope.
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

#include "nvt_rsa_driver_entrypoints.h"

#if defined(MBEDTLS_PSA_NVT_RSA_DRIVER_ENABLED)

#include <string.h>

#include "nvt_crpt.h"
#include "nvt_psa_rsa.h"
#include "mbedtls/asn1.h"
#include "mbedtls/platform_util.h"

/* Raw big-endian key material extracted from PSA's DER key buffer. */
typedef struct
{
    uint8_t n[NVT_RSA_MAX_BYTES];
    uint8_t e[NVT_RSA_MAX_BYTES];
    uint8_t d[NVT_RSA_MAX_BYTES];
    uint8_t p[NVT_RSA_MAX_BYTES / 2];
    uint8_t q[NVT_RSA_MAX_BYTES / 2];
    size_t n_len;
    size_t e_len;
} nvt_rsa_raw_key_t;

/* Only touched while the CRPT busy guard is held. */
static nvt_rsa_raw_key_t s_key;

psa_status_t nvt_rsa_driver_init(void)
{
    return PSA_SUCCESS;
}

void nvt_rsa_driver_free(void)
{
}

static int is_hw_key_size(size_t n_len)
{
    return n_len == 128 || n_len == 256 || n_len == 384 || n_len == 512;
}

/* Reads the next DER INTEGER as unsigned big-endian bytes (leading zeros dropped). */
static int next_integer(unsigned char **p, const unsigned char *end,
                        const uint8_t **value, size_t *len)
{
    if (mbedtls_asn1_get_tag(p, end, len, MBEDTLS_ASN1_INTEGER) != 0)
    {
        return -1;
    }

    *value = *p;
    *p += *len;

    while (*len > 0 && **value == 0)
    {
        (*value)++;
        (*len)--;
    }

    return *len == 0 ? -1 : 0;
}

static int copy_padded(const uint8_t *value, size_t len, uint8_t *out, size_t size)
{
    if (len > size)
    {
        return -1;
    }

    memset(out, 0, size - len);
    memcpy(out + size - len, value, len);
    return 0;
}

/* Extracts raw n/e(/d/p/q) from PSA's PKCS#1 DER key buffer into s_key. The key
 * was already fully validated by psa_import_key(), so it is not re-checked here.
 * Unbalanced keys (|p| or |q| > n_len/2) and sizes the engine lacks return
 * NOT_SUPPORTED so software handles them. */
static psa_status_t load_key(const psa_key_attributes_t *attributes,
                             const uint8_t *key_buffer, size_t key_buffer_size,
                             int need_private)
{
    unsigned char *p = (unsigned char *) key_buffer;
    const unsigned char *end = key_buffer + key_buffer_size;
    const uint8_t *value;
    size_t len, half;
    int version;

    if (mbedtls_asn1_get_tag(&p, end, &len, MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE) != 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (psa_get_key_type(attributes) == PSA_KEY_TYPE_RSA_KEY_PAIR &&
            mbedtls_asn1_get_int(&p, end, &version) != 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (next_integer(&p, end, &value, &len) != 0 || !is_hw_key_size(len))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    s_key.n_len = len;
    half = len / 2;
    memcpy(s_key.n, value, len);

    if (next_integer(&p, end, &value, &len) != 0 || len > s_key.n_len)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    s_key.e_len = len;
    memcpy(s_key.e, value, len);

    if (!need_private)
    {
        return PSA_SUCCESS;
    }

    if (next_integer(&p, end, &value, &len) != 0 ||
            copy_padded(value, len, s_key.d, s_key.n_len) != 0 ||
            next_integer(&p, end, &value, &len) != 0 ||
            copy_padded(value, len, s_key.p, half) != 0 ||
            next_integer(&p, end, &value, &len) != 0 ||
            copy_padded(value, len, s_key.q, half) != 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    return PSA_SUCCESS;
}

static void wipe_key(void)
{
    mbedtls_platform_zeroize(&s_key, sizeof(s_key));
}

/* RFC 8017 standard salt: as long as the hash, shortened only if the key is too small. */
static size_t pss_standard_salt_len(size_t n_len)
{
    size_t room = n_len - 2 - NVT_RSA_HASH_BYTES;
    return room > NVT_RSA_HASH_BYTES ? NVT_RSA_HASH_BYTES : room;
}

static int is_hw_sign_alg(psa_algorithm_t alg)
{
    if (PSA_ALG_SIGN_GET_HASH(alg) != PSA_ALG_SHA_256)
    {
        return 0;
    }

    return PSA_ALG_IS_RSA_PKCS1V15_SIGN(alg) || PSA_ALG_IS_RSA_PSS_STANDARD_SALT(alg);
}

static int is_hw_crypt_alg(psa_algorithm_t alg)
{
    return PSA_ALG_IS_RSA_OAEP(alg) && PSA_ALG_RSA_OAEP_GET_HASH(alg) == PSA_ALG_SHA_256;
}

static psa_status_t sign_hash_locked(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash,
    uint8_t *signature, size_t signature_size, size_t *signature_length)
{
    size_t half;
    psa_status_t status = load_key(attributes, key_buffer, key_buffer_size, 1);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    if (signature_size < s_key.n_len)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    half = s_key.n_len / 2;

    if (PSA_ALG_IS_RSA_PKCS1V15_SIGN(alg))
    {
        return nvt_rsa_sign_pkcs1v15_sha256_protected(s_key.n, s_key.n_len, s_key.p, half,
                                                      s_key.q, half, s_key.d, s_key.n_len, hash,
                                                      signature, signature_size, signature_length);
    }

    return nvt_rsa_sign_pss_sha256_protected(s_key.n, s_key.n_len, s_key.p, half,
                                             s_key.q, half, s_key.d, s_key.n_len,
                                             hash, pss_standard_salt_len(s_key.n_len),
                                             signature, signature_size, signature_length);
}

psa_status_t nvt_rsa_transparent_sign_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    uint8_t *signature, size_t signature_size, size_t *signature_length)
{
    psa_status_t status;

    if (psa_get_key_type(attributes) != PSA_KEY_TYPE_RSA_KEY_PAIR || !is_hw_sign_alg(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (hash_length != NVT_RSA_HASH_BYTES)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = sign_hash_locked(attributes, key_buffer, key_buffer_size, alg, hash,
                              signature, signature_size, signature_length);
    wipe_key();
    nvt_crpt_release();
    return status;
}

static psa_status_t verify_hash_locked(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash,
    const uint8_t *signature, size_t signature_length)
{
    psa_status_t status = load_key(attributes, key_buffer, key_buffer_size, 0);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    if (signature_length != s_key.n_len)
    {
        return PSA_ERROR_INVALID_SIGNATURE;
    }

    if (PSA_ALG_IS_RSA_PKCS1V15_SIGN(alg))
    {
        return nvt_rsa_verify_pkcs1v15_sha256(s_key.n, s_key.n_len, s_key.e, s_key.e_len,
                                              hash, signature, signature_length);
    }

    return nvt_rsa_verify_pss_sha256(s_key.n, s_key.n_len, s_key.e, s_key.e_len,
                                     hash, pss_standard_salt_len(s_key.n_len),
                                     signature, signature_length);
}

psa_status_t nvt_rsa_transparent_verify_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    const uint8_t *signature, size_t signature_length)
{
    psa_status_t status;

    if (!PSA_KEY_TYPE_IS_RSA(psa_get_key_type(attributes)) || !is_hw_sign_alg(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (hash_length != NVT_RSA_HASH_BYTES)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = verify_hash_locked(attributes, key_buffer, key_buffer_size, alg, hash,
                                signature, signature_length);
    wipe_key();
    nvt_crpt_release();
    return status;
}

static psa_status_t encrypt_locked(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    const uint8_t *input, size_t input_length,
    const uint8_t *label, size_t label_length,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    psa_status_t status = load_key(attributes, key_buffer, key_buffer_size, 0);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    if (output_size < s_key.n_len)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    return nvt_rsa_oaep_encrypt_sha256(s_key.n, s_key.n_len, s_key.e, s_key.e_len,
                                       label, label_length, input, input_length,
                                       output, output_size, output_length);
}

psa_status_t nvt_rsa_transparent_asymmetric_encrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input, size_t input_length,
    const uint8_t *salt, size_t salt_length,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    psa_status_t status;

    if (!PSA_KEY_TYPE_IS_RSA(psa_get_key_type(attributes)) || !is_hw_crypt_alg(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = encrypt_locked(attributes, key_buffer, key_buffer_size, input, input_length,
                            salt, salt_length, output, output_size, output_length);
    wipe_key();
    nvt_crpt_release();
    return status;
}

static psa_status_t decrypt_locked(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    const uint8_t *input, size_t input_length,
    const uint8_t *label, size_t label_length,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    static uint8_t plain[NVT_RSA_MAX_BYTES];
    size_t half;
    size_t plain_length = 0;
    psa_status_t status = load_key(attributes, key_buffer, key_buffer_size, 1);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    if (input_length != s_key.n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    half = s_key.n_len / 2;
    /* Decode into scratch first so a short output buffer reports BUFFER_TOO_SMALL, not bad padding. */
    status = nvt_rsa_oaep_decrypt_sha256_protected(s_key.n, s_key.n_len, s_key.p, half,
                                                   s_key.q, half, s_key.d, s_key.n_len,
                                                   label, label_length, input, input_length,
                                                   plain, sizeof(plain), &plain_length);

    if (status == PSA_SUCCESS)
    {
        if (plain_length > output_size)
        {
            status = PSA_ERROR_BUFFER_TOO_SMALL;
        }
        else
        {
            memcpy(output, plain, plain_length);
            *output_length = plain_length;
        }
    }

    mbedtls_platform_zeroize(plain, sizeof(plain));
    return status;
}

psa_status_t nvt_rsa_transparent_asymmetric_decrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input, size_t input_length,
    const uint8_t *salt, size_t salt_length,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    psa_status_t status;

    if (psa_get_key_type(attributes) != PSA_KEY_TYPE_RSA_KEY_PAIR || !is_hw_crypt_alg(alg))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = decrypt_locked(attributes, key_buffer, key_buffer_size, input, input_length,
                            salt, salt_length, output, output_size, output_length);
    wipe_key();
    nvt_crpt_release();
    return status;
}

#endif /* MBEDTLS_PSA_NVT_RSA_DRIVER_ENABLED */
