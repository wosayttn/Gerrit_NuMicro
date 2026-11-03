/*
 * Nuvoton "transparent"-shaped ECC PSA entry points: implementation.
 * See nvt_ecc_driver_entrypoints.h for the API contract and disclaimers.
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

#include "nvt_ecc_driver_entrypoints.h"

#if defined(MBEDTLS_PSA_NVT_ECC_DRIVER_ENABLED)

#include "nvt_crpt.h"
#include "nvt_psa_ecc.h"
#include "NuMicro.h"
#include "mbedtls/private/ecp.h"
#include "psa_util_internal.h"

psa_status_t nvt_ecc_driver_init(void)
{
    ECC_ENABLE_INT(CRPT);
    NVIC_EnableIRQ(CRPT_IRQn);
    return PSA_SUCCESS;
}

void nvt_ecc_driver_free(void)
{
    ECC_DISABLE_INT(CRPT);
}

void nvt_ecc_driver_isr(void)
{
    /* ECC_DriverISR() checks the ECC status bits itself, so it is safe to
     * call unconditionally from the shared CRPT_IRQHandler. */
    ECC_DriverISR(CRPT);
}

/* Maps a PSA ECC key (family + bits) to the matching StdDriver curve id.
 * Returns CURVE_UNDEF for families/sizes the HW doesn't expose (including
 * Montgomery/Curve25519 and any family not in the NIST/Koblitz/Brainpool
 * set below - see nvt_psa_ecc.h for the full rationale). */
static E_ECC_CURVE curve_from_attributes(const psa_key_attributes_t *attributes)
{
    psa_ecc_family_t family = PSA_KEY_TYPE_ECC_GET_FAMILY(psa_get_key_type(attributes));
    size_t bits = psa_get_key_bits(attributes);

    if (family == PSA_ECC_FAMILY_SECP_R1)
    {
        switch (bits)
        {
            case 192:
                return CURVE_P_192;

            case 224:
                return CURVE_P_224;

            case 256:
                return CURVE_P_256;

            case 384:
                return CURVE_P_384;

            case 521:
                return CURVE_P_521;

            default:
                break;
        }
    }
    else if (family == PSA_ECC_FAMILY_SECP_K1)
    {
        switch (bits)
        {
            case 192:
                return CURVE_KO_192;

            case 224:
                return CURVE_KO_224;

            case 256:
                return CURVE_KO_256;

            default:
                break;
        }
    }
    else if (family == PSA_ECC_FAMILY_SECT_R1)
    {
        switch (bits)
        {
            case 163:
                return CURVE_B_163;

            case 233:
                return CURVE_B_233;

            case 283:
                return CURVE_B_283;

            case 409:
                return CURVE_B_409;

            case 571:
                return CURVE_B_571;

            default:
                break;
        }
    }
    else if (family == PSA_ECC_FAMILY_SECT_K1)
    {
        switch (bits)
        {
            case 163:
                return CURVE_K_163;

            case 233:
                return CURVE_K_233;

            case 283:
                return CURVE_K_283;

            case 409:
                return CURVE_K_409;

            case 571:
                return CURVE_K_571;

            default:
                break;
        }
    }
    else if (family == PSA_ECC_FAMILY_BRAINPOOL_P_R1)
    {
        switch (bits)
        {
            case 256:
                return CURVE_BP_256;

            case 384:
                return CURVE_BP_384;

            case 512:
                return CURVE_BP_512;

            default:
                break;
        }
    }

    return CURVE_UNDEF;
}

/* Rejects peer points that are not on the curve (invalid-curve attack).
 * Curves with no software group in this tree cannot be checked, so the
 * request is declined rather than computed on an unvalidated point. */
static psa_status_t check_peer_point(const psa_key_attributes_t *attributes,
                                     const uint8_t *point, size_t point_length)
{
    mbedtls_ecp_group_id id = mbedtls_ecc_group_from_psa(
                                  PSA_KEY_TYPE_ECC_GET_FAMILY(psa_get_key_type(attributes)),
                                  psa_get_key_bits(attributes));
    mbedtls_ecp_group group;
    mbedtls_ecp_point q;
    int ret;

    if (id == MBEDTLS_ECP_DP_NONE)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    mbedtls_ecp_group_init(&group);
    mbedtls_ecp_point_init(&q);
    ret = mbedtls_ecp_group_load(&group, id);

    if (ret == 0)
    {
        ret = mbedtls_ecp_point_read_binary(&group, &q, point, point_length);
    }

    if (ret == 0)
    {
        ret = mbedtls_ecp_check_pubkey(&group, &q);
    }

    mbedtls_ecp_point_free(&q);
    mbedtls_ecp_group_free(&group);

    return ret == 0 ? PSA_SUCCESS : PSA_ERROR_INVALID_ARGUMENT;
}

static psa_status_t sign_hash_unlocked(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    uint8_t *signature, size_t signature_size, size_t *signature_length)
{
    E_ECC_CURVE curve = curve_from_attributes(attributes);
    size_t n_bytes = nvt_ecc_curve_bytes(curve);

    /* The HW draws k from the TRNG, so deterministic ECDSA (RFC 6979) falls back to software. */
    if (!PSA_ALG_IS_RANDOMIZED_ECDSA(alg) || n_bytes == 0 ||
            !PSA_KEY_TYPE_IS_ECC_KEY_PAIR(psa_get_key_type(attributes)))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (key_buffer_size != n_bytes || signature_size < 2 * n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    return nvt_ecc_sign_hash(curve, key_buffer, key_buffer_size, hash, hash_length,
                             signature, signature_size, signature_length);
}

psa_status_t nvt_ecc_transparent_sign_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    uint8_t *signature, size_t signature_size, size_t *signature_length)
{
    psa_status_t status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = sign_hash_unlocked(attributes, key_buffer, key_buffer_size, alg,
                                hash, hash_length,
                                signature, signature_size, signature_length);
    nvt_crpt_release();
    return status;
}

static psa_status_t verify_hash_unlocked(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    const uint8_t *signature, size_t signature_length)
{
    E_ECC_CURVE curve = curve_from_attributes(attributes);
    size_t n_bytes = nvt_ecc_curve_bytes(curve);
    uint8_t qx[NVT_ECC_MAX_BYTES], qy[NVT_ECC_MAX_BYTES];
    size_t qx_len, qy_len;
    psa_status_t status;

    if (!PSA_ALG_IS_ECDSA(alg) || n_bytes == 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (signature_length != 2 * n_bytes)
    {
        return PSA_ERROR_INVALID_SIGNATURE;
    }

    if (PSA_KEY_TYPE_IS_ECC_KEY_PAIR(psa_get_key_type(attributes)))
    {
        if (key_buffer_size != n_bytes)
        {
            return PSA_ERROR_INVALID_ARGUMENT;
        }

        status = nvt_ecc_compute_public_key(curve, key_buffer, key_buffer_size,
                                            qx, sizeof(qx), &qx_len, qy, sizeof(qy), &qy_len);

        if (status != PSA_SUCCESS)
        {
            return status;
        }

        return nvt_ecc_verify_hash(curve, qx, qx_len, qy, qy_len,
                                   hash, hash_length, signature, signature_length);
    }

    if (key_buffer_size != 1U + 2U * n_bytes || key_buffer[0] != 0x04)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    return nvt_ecc_verify_hash(curve, key_buffer + 1, n_bytes,
                               key_buffer + 1 + n_bytes, n_bytes,
                               hash, hash_length, signature, signature_length);
}

psa_status_t nvt_ecc_transparent_verify_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    const uint8_t *signature, size_t signature_length)
{
    psa_status_t status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = verify_hash_unlocked(attributes, key_buffer, key_buffer_size, alg,
                                  hash, hash_length, signature, signature_length);
    nvt_crpt_release();
    return status;
}

static psa_status_t key_agreement_unlocked(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *peer_key, size_t peer_key_length,
    uint8_t *shared_secret, size_t shared_secret_size, size_t *shared_secret_length)
{
    E_ECC_CURVE curve = curve_from_attributes(attributes);
    size_t n_bytes = nvt_ecc_curve_bytes(curve);
    psa_status_t status;

    if (!PSA_ALG_IS_ECDH(alg) || n_bytes == 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (key_buffer_size != n_bytes ||
            peer_key_length != 1U + 2U * n_bytes || peer_key[0] != 0x04 ||
            shared_secret_size < n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = check_peer_point(attributes, peer_key, peer_key_length);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    return nvt_ecdh_key_agreement(curve, key_buffer, key_buffer_size,
                                  peer_key + 1, n_bytes,
                                  peer_key + 1 + n_bytes, n_bytes,
                                  shared_secret, shared_secret_size, shared_secret_length);
}

psa_status_t nvt_ecc_transparent_key_agreement(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *peer_key, size_t peer_key_length,
    uint8_t *shared_secret, size_t shared_secret_size, size_t *shared_secret_length)
{
    psa_status_t status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = key_agreement_unlocked(attributes, key_buffer, key_buffer_size, alg,
                                    peer_key, peer_key_length,
                                    shared_secret, shared_secret_size, shared_secret_length);
    nvt_crpt_release();
    return status;
}

psa_status_t nvt_ecc_transparent_generate_key_pair(
    const psa_key_attributes_t *attributes,
    uint8_t *key_buffer, size_t key_buffer_size, size_t *key_buffer_length,
    uint8_t *pubkey_buffer, size_t pubkey_buffer_size, size_t *pubkey_buffer_length)
{
    E_ECC_CURVE curve = curve_from_attributes(attributes);
    size_t n_bytes = nvt_ecc_curve_bytes(curve);
    size_t qx_len, qy_len;
    psa_status_t status;

    if (n_bytes == 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (key_buffer_size < n_bytes || pubkey_buffer_size < 1U + 2U * n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = nvt_ecc_generate_keypair(curve, key_buffer, key_buffer_size, key_buffer_length,
                                      pubkey_buffer + 1, n_bytes, &qx_len,
                                      pubkey_buffer + 1 + n_bytes, n_bytes, &qy_len);
    nvt_crpt_release();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    pubkey_buffer[0] = 0x04;
    *pubkey_buffer_length = 1U + qx_len + qy_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_ecc_transparent_generate_key(
    const psa_key_attributes_t *attributes,
    uint8_t *key_buffer, size_t key_buffer_size, size_t *key_buffer_length)
{
    uint8_t pubkey[1 + 2 * NVT_ECC_MAX_BYTES];
    size_t pubkey_length;

    if (!PSA_KEY_TYPE_IS_ECC_KEY_PAIR(psa_get_key_type(attributes)))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    return nvt_ecc_transparent_generate_key_pair(attributes,
                                                 key_buffer, key_buffer_size, key_buffer_length,
                                                 pubkey, sizeof(pubkey), &pubkey_length);
}

psa_status_t nvt_ecc_transparent_export_public_key(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    uint8_t *data, size_t data_size, size_t *data_length)
{
    E_ECC_CURVE curve = curve_from_attributes(attributes);
    size_t n_bytes = nvt_ecc_curve_bytes(curve);
    size_t qx_len, qy_len;
    psa_status_t status;

    /* Public keys are already in export format; the core copies them itself. */
    if (n_bytes == 0 || !PSA_KEY_TYPE_IS_ECC_KEY_PAIR(psa_get_key_type(attributes)))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (key_buffer_size != n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (data_size < 1U + 2U * n_bytes)
    {
        return PSA_ERROR_BUFFER_TOO_SMALL;
    }

    status = nvt_crpt_acquire();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    status = nvt_ecc_compute_public_key(curve, key_buffer, key_buffer_size,
                                        data + 1, n_bytes, &qx_len,
                                        data + 1 + n_bytes, n_bytes, &qy_len);
    nvt_crpt_release();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    data[0] = 0x04;
    *data_length = 1U + qx_len + qy_len;
    return PSA_SUCCESS;
}

#endif /* MBEDTLS_PSA_NVT_ECC_DRIVER_ENABLED */
