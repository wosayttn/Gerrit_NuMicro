/*
 * Nuvoton native ECC HW accelerator: ECDSA sign/verify + ECDH, any curve
 * the HW engine supports EXCEPT Curve25519 and SM2 (see below).
 *
 * This is a NATIVE operation-level driver, analogous to nvt_psa_aes.c: it
 * has no lifecycle (init/free/isr) of its own. The IRQ lifecycle and the
 * PSA-shaped `nvt_ecc_transparent_*` wrappers live in
 * nvt_ecc_driver_entrypoints.c/h (analogous to nvt_cipher_driver_entrypoints.c
 * wrapping nvt_psa_aes.c). The wrappers are registered as the nvt_ecc PSA
 * transparent driver (MBEDTLS_PSA_NVT_ECC_DRIVER_ENABLED).
 *
 * Curve support: any E_ECC_CURVE the HW's ECC_GenerateSignature/
 * VerifySignature/GeneratePublicKey/GenerateSecretZ support (P-192/224/256/
 * 384/521, K-163/233/283/409/571, B-163/233/283/409/571, KO-192/256,
 * BP-256/384/512) - the per-curve field width and group order are read at
 * runtime via ECC_GetCurve() (Nuvoton's own verified curve table), so no
 * curve constants are hand-transcribed here. KO-224 is rejected: StdDriver's
 * table stores a truncated 113-bit order for it (true n is 225 bits).
 *
 * EXCLUDED: CURVE_25519 and CURVE_SM2_256 are rejected with
 * PSA_ERROR_NOT_SUPPORTED. Curve25519 is a Montgomery curve (X25519 key
 * agreement only, different point/key encoding and required scalar
 * clamping per RFC 7748 - not compatible with ECDSA; upstream mbedtls's own
 * mbedtls_ecdsa_can_do() makes the same exclusion). SM2 uses a different
 * signing equation and message pre-hash (ZA) than ECDSA, even though it
 * shares a similar prime-field curve shape; StdDriver exposes it separately
 * via SM2_Sign()/SM2_Verify(), not through this generic ECDSA path. Both
 * would need dedicated wrappers if ever required.
 *
 * All keys/hashes/signatures are raw big-endian byte buffers (PSA style),
 * sized per-curve (see nvt_ecc_curve_bytes()), up to NVT_ECC_MAX_BYTES.
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

#ifndef TF_PSA_CRYPTO_NVT_PSA_ECC_H
#define TF_PSA_CRYPTO_NVT_PSA_ECC_H

#include <psa/crypto_types.h>
#include <stddef.h>
#include <stdint.h>

#include "NuMicro.h" /* E_ECC_CURVE */

/* Largest supported field/order width: B-571/K-571 = 571 bits = 72 bytes. */
#define NVT_ECC_MAX_BYTES 72U

/* Returns the field/order byte width for a curve (e.g. 32 for CURVE_P_256),
 * or 0 if the curve is unknown to the HW or is CURVE_25519/CURVE_SM2_256. */
size_t nvt_ecc_curve_bytes(E_ECC_CURVE curve);

/* Generate a fresh keypair using the on-chip TRNG for the private key.
 * Buffers must be >= nvt_ecc_curve_bytes(curve); *_len receives the actual
 * width used. */
psa_status_t nvt_ecc_generate_keypair(E_ECC_CURVE curve,
                                      uint8_t *priv, size_t priv_size, size_t *priv_len,
                                      uint8_t *qx, size_t qx_size, size_t *qx_len,
                                      uint8_t *qy, size_t qy_size, size_t *qy_len);

/* Derive the public key (Qx, Qy) from an existing private key. */
psa_status_t nvt_ecc_compute_public_key(E_ECC_CURVE curve,
                                        const uint8_t *priv, size_t priv_len,
                                        uint8_t *qx, size_t qx_size, size_t *qx_len,
                                        uint8_t *qy, size_t qy_size, size_t *qy_len);

/*
 * ECDSA sign a pre-computed hash (deterministic-k is NOT used; a fresh
 * TRNG-sourced nonce k is generated for every call, per SEC1/FIPS 186-4).
 * hash_len may be > curve width (only the leftmost bytes are used) or less.
 * sig receives r || s (2 * nvt_ecc_curve_bytes(curve) bytes total).
 */
psa_status_t nvt_ecc_sign_hash(E_ECC_CURVE curve,
                               const uint8_t *priv, size_t priv_len,
                               const uint8_t *hash, size_t hash_len,
                               uint8_t *sig, size_t sig_size, size_t *sig_len);

/* ECDSA verify a pre-computed hash against sig = r || s. */
psa_status_t nvt_ecc_verify_hash(E_ECC_CURVE curve,
                                 const uint8_t *qx, size_t qx_len,
                                 const uint8_t *qy, size_t qy_len,
                                 const uint8_t *hash, size_t hash_len,
                                 const uint8_t *sig, size_t sig_len);

/* ECDH: compute the shared secret Z from our private key + peer's public key. */
psa_status_t nvt_ecdh_key_agreement(E_ECC_CURVE curve,
                                    const uint8_t *priv, size_t priv_len,
                                    const uint8_t *peer_qx, size_t peer_qx_len,
                                    const uint8_t *peer_qy, size_t peer_qy_len,
                                    uint8_t *shared_secret, size_t shared_secret_size,
                                    size_t *shared_secret_len);

/*
 * Hardware self-test: for every HW-supported curve (P-192..P-521, KO-192..
 * KO-256, B-163..B-571, K-163..K-571, BP-256..BP-512), generates a fresh
 * keypair and round-trips sign+verify plus a two-party ECDH agreement,
 * printing PASS/FAIL per curve/case. Mirrors nvt_aes_self_test()'s pattern.
 * PRECONDITION: caller must already have called nvt_ecc_driver_init() (see
 * nvt_ecc_driver_entrypoints.h) and wired nvt_ecc_driver_isr() into the
 * shared CRPT_IRQHandler - this function does not manage that lifecycle
 * itself. Returns 0 if everything passed, 1 otherwise.
 */
int nvt_ecc_self_test(void);

#endif /* TF_PSA_CRYPTO_NVT_PSA_ECC_H */
