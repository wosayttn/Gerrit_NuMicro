/*
 * Nuvoton native RSA HW accelerator: PKCS#1 v1.5 and PSS sign/verify,
 * OAEP encrypt/decrypt (SHA-256 / MGF1-SHA-256 throughout).
 *
 * This is a NATIVE operation-level driver, analogous to nvt_psa_aes.c: it is
 * fully self-contained (busy-waits by polling CRPT->INTSTS directly, no
 * NVIC/ISR wiring required - same pattern as nvt_psa_aes.c). The PSA
 * transparent driver entry points (`nvt_rsa_transparent_*`, dispatched by
 * psa_sign_hash() etc.) live in nvt_rsa_driver_entrypoints.c/h.
 *
 * Scope: RSA_MODE_NORMAL (modexp only, no blinding) for verify and non-CRT
 * sign, plus an optional RSA_MODE_CRT path for sign (see
 * nvt_rsa_sign_pkcs1v15_sha256_crt) that is ~3-4x faster for the private-key
 * operation. Any key size the HW engine supports (1024/2048/3072/4096-bit -
 * see NVT_RSA_MIN_BYTES/NVT_RSA_MAX_BYTES), EMSA-PKCS1-v1.5 encoding with
 * SHA-256 (RFC 8017). The RSA engine only performs the modular
 * exponentiation primitive; padding is done in software here to
 * interoperate with mbedtls's own PKCS#1 v1.5 sign/verify.
 *
 * IMPORTANT (M2354 TRM Table 6.36-7 "RSA Accelerator Support Situation"):
 * the HW's CRT mode is NOT supported at all for 1024-bit keys (only Normal
 * and SCAP-only combinations are) - all `*_crt` functions below reject
 * n_len == 128 (1024-bit) with PSA_ERROR_NOT_SUPPORTED. Non-CRT sign and
 * verify remain fully supported at 1024-bit.
 *
 * App precondition: CLK_EnableModuleClock(CRPT_MODULE) and
 * SYS_SetPSRAMPowerMode(SYS_SRAMPC1_RSA_Msk, SYS_SRAMPC1_SRAM_NORMAL) once at
 * init (RSA needs its SRAM banked in). No IRQ/NVIC setup is required.
 *
 * PSS/OAEP additionally depend on mbedtls_sha256() (software, from
 * tf-psa-crypto/drivers/builtin/src/sha256.c, already linked) for MGF1 and
 * the label/salt hashing - not the HW SHA engine, to avoid coupling this
 * self-contained driver to nvt_psa_sha.c's IRQ requirements.
 *
 * Note: neither mode includes RSA blinding countermeasures; treat
 * private-key operations as sensitive to power/timing side channels same as
 * any unblinded modexp implementation. The CRT path itself introduces no
 * additional P/Q timing asymmetry beyond what unblinded CRT-RSA always has
 * (the HW derives Dp/Dq/Rp/Rq/Cp/Cq internally from D, P, Q as scratch
 * values - the caller never computes or supplies them).
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

#ifndef TF_PSA_CRYPTO_NVT_PSA_RSA_H
#define TF_PSA_CRYPTO_NVT_PSA_RSA_H

#include <psa/crypto_types.h>
#include <stddef.h>
#include <stdint.h>

/* HW-supported RSA modulus sizes: 1024/2048/3072/4096-bit (128..512 bytes). */
#define NVT_RSA_MIN_BYTES 128U
#define NVT_RSA_MAX_BYTES 512U
#define NVT_RSA_HASH_BYTES 32U /* SHA-256 digest size */

/*
 * Sign a SHA-256 hash with the RSA private key (n, d). n_len selects the key
 * size and must be one of {128, 256, 384, 512} (1024/2048/3072/4096-bit);
 * n and d are big-endian, d zero-padded up to n_len internally if shorter.
 * sig_size must be >= n_len; on success *sig_length == n_len.
 */
psa_status_t nvt_rsa_sign_pkcs1v15_sha256(const uint8_t *n, size_t n_len,
                                          const uint8_t *d, size_t d_len,
                                          const uint8_t hash[NVT_RSA_HASH_BYTES],
                                          uint8_t *sig, size_t sig_size, size_t *sig_length);

/*
 * Same as nvt_rsa_sign_pkcs1v15_sha256() but uses the HW's CRT mode for the
 * private-key operation (~3-4x faster). p_len/q_len must each equal n_len/2
 * (the standard balanced-RSA factor size); d is still the full private
 * exponent - the HW derives Dp/Dq/Rp/Rq/Cp/Cq internally, nothing extra to
 * compute here. NOT supported for 1024-bit keys (n_len == 128) - returns
 * PSA_ERROR_NOT_SUPPORTED (see Table 6.36-7 in the M2354 TRM).
 */
psa_status_t nvt_rsa_sign_pkcs1v15_sha256_crt(const uint8_t *n, size_t n_len,
                                              const uint8_t *p, size_t p_len,
                                              const uint8_t *q, size_t q_len,
                                              const uint8_t *d, size_t d_len,
                                              const uint8_t hash[NVT_RSA_HASH_BYTES],
                                              uint8_t *sig, size_t sig_size, size_t *sig_length);

/*
 * SCAP (side-channel-protected blinding) variant of nvt_rsa_sign_pkcs1v15_sha256();
 * supported at every HW key size, but like the CRT variant still requires P/Q.
 */
psa_status_t nvt_rsa_sign_pkcs1v15_sha256_scap(const uint8_t *n, size_t n_len,
                                               const uint8_t *p, size_t p_len,
                                               const uint8_t *q, size_t q_len,
                                               const uint8_t *d, size_t d_len,
                                               const uint8_t hash[NVT_RSA_HASH_BYTES],
                                               uint8_t *sig, size_t sig_size, size_t *sig_length);

/* CRT + SCAP combined; same >= 2048-bit gating as the plain CRT variant. */
psa_status_t nvt_rsa_sign_pkcs1v15_sha256_crt_scap(const uint8_t *n, size_t n_len,
                                                   const uint8_t *p, size_t p_len,
                                                   const uint8_t *q, size_t q_len,
                                                   const uint8_t *d, size_t d_len,
                                                   const uint8_t hash[NVT_RSA_HASH_BYTES],
                                                   uint8_t *sig, size_t sig_size, size_t *sig_length);

/*
 * Verify a signature (sig_length == n_len) against a SHA-256 hash with the
 * RSA public key (n, e). e is big-endian with length e_len (commonly 3 bytes
 * for 65537), n is big-endian with length n_len (see nvt_rsa_sign_pkcs1v15_sha256).
 */
psa_status_t nvt_rsa_verify_pkcs1v15_sha256(const uint8_t *n, size_t n_len,
                                            const uint8_t *e, size_t e_len,
                                            const uint8_t hash[NVT_RSA_HASH_BYTES],
                                            const uint8_t *sig, size_t sig_length);

/*
 * RSASSA-PSS sign/verify with SHA-256 (hash + MGF1), per RFC 8017 section 9.1.
 * salt_len is commonly NVT_RSA_HASH_BYTES; both sides must agree on it.
 */
psa_status_t nvt_rsa_sign_pss_sha256(const uint8_t *n, size_t n_len,
                                     const uint8_t *d, size_t d_len,
                                     const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                                     uint8_t *sig, size_t sig_size, size_t *sig_length);

/* CRT-accelerated variant of nvt_rsa_sign_pss_sha256(); p_len/q_len must == n_len/2.
 * NOT supported for 1024-bit keys (n_len == 128) - returns PSA_ERROR_NOT_SUPPORTED. */
psa_status_t nvt_rsa_sign_pss_sha256_crt(const uint8_t *n, size_t n_len,
                                         const uint8_t *p, size_t p_len,
                                         const uint8_t *q, size_t q_len,
                                         const uint8_t *d, size_t d_len,
                                         const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                                         uint8_t *sig, size_t sig_size, size_t *sig_length);

psa_status_t nvt_rsa_verify_pss_sha256(const uint8_t *n, size_t n_len,
                                       const uint8_t *e, size_t e_len,
                                       const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                                       const uint8_t *sig, size_t sig_length);

/*
 * RSAES-OAEP encrypt/decrypt with SHA-256 (hash + MGF1), per RFC 8017 section
 * 7.1. label may be NULL (empty label, the common case) with label_len 0.
 * Decrypt failures (bad padding) all return the SAME PSA_ERROR_INVALID_PADDING
 * via a uniform, branch-minimized decode path - never distinguish *why*
 * decoding failed, to avoid a Manger's-attack-style padding oracle.
 */
psa_status_t nvt_rsa_oaep_encrypt_sha256(const uint8_t *n, size_t n_len,
                                         const uint8_t *e, size_t e_len,
                                         const uint8_t *label, size_t label_len,
                                         const uint8_t *msg, size_t msg_len,
                                         uint8_t *out, size_t out_size, size_t *out_len);

psa_status_t nvt_rsa_oaep_decrypt_sha256(const uint8_t *n, size_t n_len,
                                         const uint8_t *d, size_t d_len,
                                         const uint8_t *label, size_t label_len,
                                         const uint8_t *in, size_t in_len,
                                         uint8_t *msg, size_t msg_size, size_t *msg_len);

/* CRT-accelerated variant of nvt_rsa_oaep_decrypt_sha256(); p_len/q_len must == n_len/2.
 * NOT supported for 1024-bit keys (n_len == 128) - returns PSA_ERROR_NOT_SUPPORTED. */
psa_status_t nvt_rsa_oaep_decrypt_sha256_crt(const uint8_t *n, size_t n_len,
                                             const uint8_t *p, size_t p_len,
                                             const uint8_t *q, size_t q_len,
                                             const uint8_t *d, size_t d_len,
                                             const uint8_t *label, size_t label_len,
                                             const uint8_t *in, size_t in_len,
                                             uint8_t *msg, size_t msg_size, size_t *msg_len);

/* SCAP variant of nvt_rsa_oaep_decrypt_sha256_crt(); supported at every HW key size. */
psa_status_t nvt_rsa_oaep_decrypt_sha256_scap(const uint8_t *n, size_t n_len,
                                              const uint8_t *p, size_t p_len,
                                              const uint8_t *q, size_t q_len,
                                              const uint8_t *d, size_t d_len,
                                              const uint8_t *label, size_t label_len,
                                              const uint8_t *in, size_t in_len,
                                              uint8_t *msg, size_t msg_size, size_t *msg_len);

/* CRT + SCAP combined; same >= 2048-bit gating as the plain CRT variant. */
psa_status_t nvt_rsa_oaep_decrypt_sha256_crt_scap(const uint8_t *n, size_t n_len,
                                                  const uint8_t *p, size_t p_len,
                                                  const uint8_t *q, size_t q_len,
                                                  const uint8_t *d, size_t d_len,
                                                  const uint8_t *label, size_t label_len,
                                                  const uint8_t *in, size_t in_len,
                                                  uint8_t *msg, size_t msg_size, size_t *msg_len);

/*
 * Private-key operations in the side-channel-protected mode the HW allows for
 * this key size: CRT+SCAP at >= 2048-bit, SCAP at 1024-bit. Used by PSA dispatch.
 */
psa_status_t nvt_rsa_sign_pkcs1v15_sha256_protected(const uint8_t *n, size_t n_len,
                                                    const uint8_t *p, size_t p_len,
                                                    const uint8_t *q, size_t q_len,
                                                    const uint8_t *d, size_t d_len,
                                                    const uint8_t hash[NVT_RSA_HASH_BYTES],
                                                    uint8_t *sig, size_t sig_size, size_t *sig_length);

psa_status_t nvt_rsa_sign_pss_sha256_protected(const uint8_t *n, size_t n_len,
                                               const uint8_t *p, size_t p_len,
                                               const uint8_t *q, size_t q_len,
                                               const uint8_t *d, size_t d_len,
                                               const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                                               uint8_t *sig, size_t sig_size, size_t *sig_length);

psa_status_t nvt_rsa_oaep_decrypt_sha256_protected(const uint8_t *n, size_t n_len,
                                                   const uint8_t *p, size_t p_len,
                                                   const uint8_t *q, size_t q_len,
                                                   const uint8_t *d, size_t d_len,
                                                   const uint8_t *label, size_t label_len,
                                                   const uint8_t *in, size_t in_len,
                                                   uint8_t *msg, size_t msg_size, size_t *msg_len);

/*
 * Query whether a given (key-size, mode) combination is supported by the HW
 * engine, per M2354 TRM Table 6.36-7 - returns PSA_SUCCESS if supported,
 * PSA_ERROR_NOT_SUPPORTED otherwise. is_crt/is_scap are booleans (0 or 1);
 * e.g. (n_len=256, is_crt=1, is_scap=0) asks about plain CRT at 2048-bit.
 */
psa_status_t nvt_rsa_mode_supported(size_t n_len, int is_crt, int is_scap);

/*
 * Generates a fresh RSA keypair via the SOFTWARE mbedtls_rsa_gen_key()
 * (tf-psa-crypto/drivers/builtin/src/rsa.c, TRNG-seeded) - key generation is
 * infrequent/not performance-critical and correctness-sensitive, so it is
 * deliberately NOT reimplemented on the HW engine (see
 * /memories/repo/pqc-mbedtls4-porting.md). n/e/d/p/q are exported as plain
 * big-endian byte buffers via mbedtls_mpi_write_binary() - no ASN.1/DER
 * involved - ready to feed directly into the sign/verify/OAEP functions
 * above. nbits must be one of {1024, 2048, 3072, 4096}.
 */
psa_status_t nvt_rsa_generate_key_pair(size_t nbits, uint32_t exponent,
                                       uint8_t *n, size_t n_size, size_t *n_len,
                                       uint8_t *e, size_t e_size, size_t *e_len,
                                       uint8_t *d, size_t d_size, size_t *d_len,
                                       uint8_t *p, size_t p_size, size_t *p_len,
                                       uint8_t *q, size_t q_size, size_t *q_len);

/*
 * Hardware self-test: generates a key pair (software, TRNG-seeded) per
 * HW-supported key size {1024,2048,3072,4096}-bit, then round-trips
 * sign+verify and encrypt+decrypt through every mode the HW/size
 * combination supports (Normal always; CRT/SCAP/CRT_SCAP per
 * nvt_rsa_mode_supported()), printing PASS/FAIL per case. Mirrors
 * nvt_aes_self_test()'s pattern. Returns 0 if everything passed, 1 otherwise.
 */
int nvt_rsa_self_test(void);

#endif /* TF_PSA_CRYPTO_NVT_PSA_RSA_H */
