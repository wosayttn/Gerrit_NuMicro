/*
 * Nuvoton "transparent"-shaped ECC PSA entry points.
 *
 * Thin adapter layer wrapping the native operations in nvt_psa_ecc.c, shaped to
 * match the PSA transparent-driver call convention (psa_key_attributes_t +
 * exported-key-format key_buffer + psa_algorithm_t), mirroring how
 * nvt_cipher_driver_entrypoints.c wraps nvt_psa_aes.c.
 *
 * Registered as the `nvt_ecc` PSA transparent driver when
 * MBEDTLS_PSA_NVT_ECC_DRIVER_ENABLED is defined: psa_sign_hash() /
 * psa_verify_hash() / psa_raw_key_agreement() / psa_generate_key() /
 * psa_export_public_key() reach this code, and fall back to software
 * whenever a function here returns PSA_ERROR_NOT_SUPPORTED.
 *
 * Curve is derived from `attributes` (PSA_KEY_TYPE_ECC_GET_FAMILY + key bits)
 * - see nvt_psa_ecc.h for which curves/families are supported (all HW
 * Weierstrass/Koblitz-binary curves EXCEPT Curve25519 and SM2).
 *
 * Key formats accepted (both are PSA's standard raw export formats for a
 * Weierstrass curve, so no ASN.1/DER parsing is required):
 *  - private key (key_buffer): n_bytes big-endian scalar d (n_bytes = curve
 *    field/order width, e.g. 32 for P-256).
 *  - public key (key_buffer / peer_key): 1 + 2*n_bytes bytes =
 *    0x04 || X(n_bytes) || Y(n_bytes) (SEC1 uncompressed point encoding, as
 *    produced by psa_export_public_key).
 *
 * ECC still requires the IRQ path (unlike RSA/AES): the StdDriver
 * ECC_GenerateSignature/VerifySignature/GeneratePublicKey/GenerateSecretZ
 * functions busy-wait internally on a flag set only by ECC_DriverISR(), so:
 *  - psa_crypto_init() calls nvt_ecc_driver_init() (enables the IRQ).
 *  - The app's CRPT_IRQHandler() must call nvt_crpt_irq_handler() (nvt_crpt.h).
 *
 * key_agreement only accepts peer points it can validate in software
 * (curves mbedTLS knows); for HW-only curves use nvt_ecdh_key_agreement().
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

#ifndef TF_PSA_CRYPTO_NVT_ECC_DRIVER_ENTRYPOINTS_H
#define TF_PSA_CRYPTO_NVT_ECC_DRIVER_ENTRYPOINTS_H

#include <psa/crypto.h>
#include <stddef.h>
#include <stdint.h>

/* Enables the ECC interrupt (NVIC + CRPT). Called by psa_crypto_init(). */
psa_status_t nvt_ecc_driver_init(void);

/* Disables the ECC interrupt. */
void nvt_ecc_driver_free(void);

/* Superseded by nvt_crpt_irq_handler(), which also services SHA. */
void nvt_ecc_driver_isr(void);

psa_status_t nvt_ecc_transparent_sign_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    uint8_t *signature, size_t signature_size, size_t *signature_length);

psa_status_t nvt_ecc_transparent_verify_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *hash, size_t hash_length,
    const uint8_t *signature, size_t signature_length);

psa_status_t nvt_ecc_transparent_key_agreement(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *peer_key, size_t peer_key_length,
    uint8_t *shared_secret, size_t shared_secret_size, size_t *shared_secret_length);

/*
 * Generate a fresh keypair for the curve named by `attributes` (PSA_KEY_TYPE_ECC_GET_FAMILY +
 * key bits, same mapping as the other nvt_ecc_transparent_* functions). key_buffer receives the
 * private scalar (n_bytes); pubkey_buffer receives 0x04 || X(n_bytes) || Y(n_bytes) (SEC1
 * uncompressed point, PSA's standard public-key export format).
 */
psa_status_t nvt_ecc_transparent_generate_key_pair(
    const psa_key_attributes_t *attributes,
    uint8_t *key_buffer, size_t key_buffer_size, size_t *key_buffer_length,
    uint8_t *pubkey_buffer, size_t pubkey_buffer_size, size_t *pubkey_buffer_length);

/* PSA generate_key entry point: key_buffer receives the private scalar only. */
psa_status_t nvt_ecc_transparent_generate_key(
    const psa_key_attributes_t *attributes,
    uint8_t *key_buffer, size_t key_buffer_size, size_t *key_buffer_length);

/* PSA export_public_key entry point for ECC key pairs (0x04 || X || Y). */
psa_status_t nvt_ecc_transparent_export_public_key(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    uint8_t *data, size_t data_size, size_t *data_length);

#endif /* TF_PSA_CRYPTO_NVT_ECC_DRIVER_ENTRYPOINTS_H */
