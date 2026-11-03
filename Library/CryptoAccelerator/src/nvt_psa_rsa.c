/*
 * Nuvoton native RSA HW accelerator: implementation.
 * See nvt_psa_rsa.h for the API contract and hardware preconditions.
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

#include "nvt_psa_rsa.h"

#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include <psa/crypto.h>
#include "mbedtls/private/sha256.h" /* mbedtls_sha256(): MGF1 + label/salt hashing */
#include "mbedtls/private/rsa.h"    /* mbedtls_rsa_gen_key(): software key generation */
#include "mbedtls/private/bignum.h" /* mbedtls_mpi_write_binary(): mpi -> raw bytes */

#define NVT_RSA_TIMEOUT (0x1000000)

/* CRT+SCAP gave wrong results with its work buffer 8-byte aligned and correct ones 32-byte aligned (chip runs). */
#define NVT_RSA_BUF_ALIGN 32


/* ASN.1 DER DigestInfo prefix for SHA-256 (RFC 8017 / well-known constant). */
static const uint8_t s_sha256_digest_info_prefix[19] =
{
    0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01,
    0x65, 0x03, 0x04, 0x02, 0x01, 0x05, 0x00, 0x04, 0x20
};
#define DIGEST_INFO_T_LEN (sizeof(s_sha256_digest_info_prefix) + NVT_RSA_HASH_BYTES) /* 51 */

/* Maps a modulus byte length to the HW engine's RSA_KEY_SIZE_* selector. */
static int key_size_code(size_t n_len, uint32_t *out_code)
{
    switch (n_len)
    {
        case 128:
            *out_code = RSA_KEY_SIZE_1024;
            return 0;

        case 256:
            *out_code = RSA_KEY_SIZE_2048;
            return 0;

        case 384:
            *out_code = RSA_KEY_SIZE_3072;
            return 0;

        case 512:
            *out_code = RSA_KEY_SIZE_4096;
            return 0;

        default:
            return -1;
    }
}

/*
 * M2354 TRM Table 6.36-7 "RSA Accelerator Support Situation": Normal and
 * SCAP work at every key size; CRT and CRT+SCAP need >= 2048-bit. Any mode
 * with CRT_BYP is not used by this driver.
 */
typedef enum
{
    NVT_RSA_MODE_NORMAL = 0,
    NVT_RSA_MODE_CRT,
    NVT_RSA_MODE_SCAP,
    NVT_RSA_MODE_CRT_SCAP
} nvt_rsa_mode_t;

static int rsa_mode_supported(size_t n_len, nvt_rsa_mode_t mode)
{
    switch (mode)
    {
        case NVT_RSA_MODE_NORMAL:
        case NVT_RSA_MODE_SCAP:
            return 1; /* Normal/SCAP-alone: supported at every HW key size. */

        case NVT_RSA_MODE_CRT:
        case NVT_RSA_MODE_CRT_SCAP:
            return n_len >= 256; /* 2048/3072/4096-bit. */

        default:
            return 0;
    }
}

psa_status_t nvt_rsa_mode_supported(size_t n_len, int is_crt, int is_scap)
{
    nvt_rsa_mode_t mode = is_crt ? (is_scap ? NVT_RSA_MODE_CRT_SCAP : NVT_RSA_MODE_CRT)
                          : (is_scap ? NVT_RSA_MODE_SCAP : NVT_RSA_MODE_NORMAL);
    return rsa_mode_supported(n_len, mode) ? PSA_SUCCESS : PSA_ERROR_NOT_SUPPORTED;
}

static const char s_hex_digits[] = "0123456789abcdef";

static void bin_to_hex(const uint8_t *bin, size_t len, char *hex)
{
    size_t i;

    for (i = 0; i < len; i++)
    {
        hex[2 * i]     = s_hex_digits[bin[i] >> 4];
        hex[2 * i + 1] = s_hex_digits[bin[i] & 0x0F];
    }

    hex[2 * len] = '\0';
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';

    if (c >= 'a' && c <= 'f') return c - 'a' + 10;

    if (c >= 'A' && c <= 'F') return c - 'A' + 10;

    return -1;
}

static int hex_to_bin_fixed(const char *hex, uint8_t *bin, size_t len)
{
    size_t i;

    if (strlen(hex) != 2 * len)
    {
        return -1;
    }

    for (i = 0; i < len; i++)
    {
        int hi = hex_nibble(hex[2 * i]);
        int lo = hex_nibble(hex[2 * i + 1]);

        if (hi < 0 || lo < 0)
        {
            return -1;
        }

        bin[i] = (uint8_t)((hi << 4) | lo);
    }

    return 0;
}

/* EM = 0x00 || 0x01 || PS(0xFF...) || 0x00 || DigestInfo(hash) ; |EM| = em_len */
static int emsa_pkcs1_v15_encode(const uint8_t hash[NVT_RSA_HASH_BYTES],
                                 uint8_t *em, size_t em_len)
{
    size_t ps_len;

    if (em_len < 3 + DIGEST_INFO_T_LEN)
    {
        return -1;
    }

    ps_len = em_len - 3 - DIGEST_INFO_T_LEN;

    em[0] = 0x00;
    em[1] = 0x01;
    memset(em + 2, 0xFF, ps_len);
    em[2 + ps_len] = 0x00;
    memcpy(em + 3 + ps_len, s_sha256_digest_info_prefix, sizeof(s_sha256_digest_info_prefix));
    memcpy(em + 3 + ps_len + sizeof(s_sha256_digest_info_prefix), hash, NVT_RSA_HASH_BYTES);
    return 0;
}

/* Constant-time comparison of the decoded EM against the expected encoding. */
static int emsa_pkcs1_v15_verify(const uint8_t *em, size_t em_len,
                                 const uint8_t hash[NVT_RSA_HASH_BYTES])
{
    uint8_t expected[NVT_RSA_MAX_BYTES];
    unsigned diff = 0;
    size_t i;

    if (emsa_pkcs1_v15_encode(hash, expected, em_len) != 0)
    {
        return -1;
    }

    for (i = 0; i < em_len; i++)
    {
        diff |= (unsigned)(em[i] ^ expected[i]);
    }

    return diff == 0 ? 0 : -1;
}

/* Self-contained completion wait: poll CRPT->INTSTS directly (the flag latches
 * without RSAIEN), mirroring nvt_psa_aes.c. Enabling the IRQ here would make
 * the shared CRPT_IRQHandler fire on completion. */
static psa_status_t rsa_run_and_wait(void)
{
    int32_t timeout;

    CRPT->INTSTS = CRPT_INTSTS_RSAIF_Msk | CRPT_INTSTS_RSAEIF_Msk;
    RSA_Start(CRPT);

    timeout = NVT_RSA_TIMEOUT;

    while ((RSA_GET_INT_FLAG(CRPT) & CRPT_INTSTS_RSAIF_Msk) == 0)
    {
        if (--timeout <= 0)
        {
            return PSA_ERROR_HARDWARE_FAILURE;
        }
    }

    if (CRPT->INTSTS & CRPT_INTSTS_RSAEIF_Msk)
    {
        RSA_CLR_INT_FLAG(CRPT);
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    RSA_CLR_INT_FLAG(CRPT);
    return PSA_SUCCESS;
}

/* Runs Output = Input ^ Exponent mod n (RSA_MODE_NORMAL) on the HW engine.
 * n_len selects the key size (1024/2048/3072/4096-bit); input/output are
 * n_len bytes each. */
static psa_status_t rsa_modexp(const uint8_t *n, size_t n_len,
                               const uint8_t *exponent, size_t exponent_len,
                               const uint8_t *input,
                               uint8_t *output)
{
    static __ALIGNED(NVT_RSA_BUF_ALIGN) RSA_BUF_NORMAL_T s_rsa_buf;
    static char n_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char exp_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char in_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char out_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static uint8_t exp_padded[NVT_RSA_MAX_BYTES];
    uint32_t key_size;
    psa_status_t status;

    if (key_size_code(n_len, &key_size) != 0 || exponent_len > n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    memset(exp_padded, 0, n_len - exponent_len);
    memcpy(exp_padded + (n_len - exponent_len), exponent, exponent_len);

    bin_to_hex(n, n_len, n_hex);
    bin_to_hex(exp_padded, n_len, exp_hex);
    bin_to_hex(input, n_len, in_hex);

    if (RSA_Open(CRPT, RSA_MODE_NORMAL, key_size, &s_rsa_buf, sizeof(s_rsa_buf), 0) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    RSA_SetKey(CRPT, exp_hex);
    RSA_SetDMATransfer(CRPT, in_hex, n_hex, 0, 0);

    status = rsa_run_and_wait();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    RSA_Read(CRPT, out_hex);

    if (hex_to_bin_fixed(out_hex, output, n_len) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    return PSA_SUCCESS;
}

/* Runs Output = Input ^ D mod n via RSA_MODE_CRT (P, Q supplied; the HW
 * derives Dp/Dq/Rp/Rq/Cp/Cq internally as scratch values - nothing extra to
 * compute here). p_len/q_len must each be n_len/2. ~3-4x faster than
 * rsa_modexp() for the private-key operation. */
static psa_status_t rsa_modexp_crt(const uint8_t *n, size_t n_len,
                                   const uint8_t *p, size_t p_len,
                                   const uint8_t *q, size_t q_len,
                                   const uint8_t *d, size_t d_len,
                                   const uint8_t *input,
                                   uint8_t *output)
{
    static __ALIGNED(NVT_RSA_BUF_ALIGN) RSA_BUF_CRT_T s_rsa_buf_crt;
    static char n_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char p_hex[NVT_RSA_MAX_BYTES + 1];
    static char q_hex[NVT_RSA_MAX_BYTES + 1];
    static char d_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char in_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char out_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static uint8_t d_padded[NVT_RSA_MAX_BYTES];
    uint32_t key_size;
    psa_status_t status;

    if (key_size_code(n_len, &key_size) != 0 || d_len > n_len ||
            p_len != n_len / 2 || q_len != n_len / 2)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    /* Unsupported combinations latch CTLERR instead of running. */
    if (!rsa_mode_supported(n_len, NVT_RSA_MODE_CRT))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    memset(d_padded, 0, n_len - d_len);
    memcpy(d_padded + (n_len - d_len), d, d_len);

    bin_to_hex(n, n_len, n_hex);
    bin_to_hex(p, p_len, p_hex);
    bin_to_hex(q, q_len, q_hex);
    bin_to_hex(d_padded, n_len, d_hex);
    bin_to_hex(input, n_len, in_hex);

    if (RSA_Open(CRPT, RSA_MODE_CRT, key_size, &s_rsa_buf_crt, sizeof(s_rsa_buf_crt), 0) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    RSA_SetKey(CRPT, d_hex);
    RSA_SetDMATransfer(CRPT, in_hex, n_hex, p_hex, q_hex);

    status = rsa_run_and_wait();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    RSA_Read(CRPT, out_hex);

    if (hex_to_bin_fixed(out_hex, output, n_len) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    return PSA_SUCCESS;
}

/* Runs Output = Input ^ Exponent mod n via RSA_MODE_SCAP (side-channel
 * protected blinding; P, Q still required - mirrors the StdDriver
 * CRYPTO_RSA_SCAP sample's own RSA_SetDMATransfer(Msg, N, P, Q) call shape
 * even though this is not the CRT mode). Same timing/perf tradeoff notes as
 * rsa_modexp()/rsa_modexp_crt() apply; this is strictly a SIDE-CHANNEL
 * hardening addition, not a performance one. */
static psa_status_t rsa_modexp_scap(const uint8_t *n, size_t n_len,
                                    const uint8_t *p, size_t p_len,
                                    const uint8_t *q, size_t q_len,
                                    const uint8_t *exponent, size_t exponent_len,
                                    const uint8_t *input,
                                    uint8_t *output)
{
    static __ALIGNED(NVT_RSA_BUF_ALIGN) RSA_BUF_SCAP_T s_rsa_buf_scap;
    static char n_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char p_hex[NVT_RSA_MAX_BYTES + 1];
    static char q_hex[NVT_RSA_MAX_BYTES + 1];
    static char exp_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char in_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char out_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static uint8_t exp_padded[NVT_RSA_MAX_BYTES];
    uint32_t key_size;
    psa_status_t status;

    if (key_size_code(n_len, &key_size) != 0 || exponent_len > n_len ||
            p_len != n_len / 2 || q_len != n_len / 2)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (!rsa_mode_supported(n_len, NVT_RSA_MODE_SCAP))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    memset(exp_padded, 0, n_len - exponent_len);
    memcpy(exp_padded + (n_len - exponent_len), exponent, exponent_len);

    bin_to_hex(n, n_len, n_hex);
    bin_to_hex(p, p_len, p_hex);
    bin_to_hex(q, q_len, q_hex);
    bin_to_hex(exp_padded, n_len, exp_hex);
    bin_to_hex(input, n_len, in_hex);

    if (RSA_Open(CRPT, RSA_MODE_SCAP, key_size, &s_rsa_buf_scap, sizeof(s_rsa_buf_scap), 0) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    RSA_SetKey(CRPT, exp_hex);
    RSA_SetDMATransfer(CRPT, in_hex, n_hex, p_hex, q_hex);

    status = rsa_run_and_wait();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    RSA_Read(CRPT, out_hex);

    if (hex_to_bin_fixed(out_hex, output, n_len) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    return PSA_SUCCESS;
}

/* Runs Output = Input ^ D mod n via RSA_MODE_CRT_SCAP (CRT acceleration +
 * side-channel-protected blinding combined). Same >= 2048-bit gating as plain CRT. */
static psa_status_t rsa_modexp_crt_scap(const uint8_t *n, size_t n_len,
                                        const uint8_t *p, size_t p_len,
                                        const uint8_t *q, size_t q_len,
                                        const uint8_t *d, size_t d_len,
                                        const uint8_t *input,
                                        uint8_t *output)
{
    static __ALIGNED(NVT_RSA_BUF_ALIGN) RSA_BUF_CRT_SCAP_T s_rsa_buf_crt_scap;
    static char n_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char p_hex[NVT_RSA_MAX_BYTES + 1];
    static char q_hex[NVT_RSA_MAX_BYTES + 1];
    static char d_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char in_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static char out_hex[2 * NVT_RSA_MAX_BYTES + 1];
    static uint8_t d_padded[NVT_RSA_MAX_BYTES];
    uint32_t key_size;
    psa_status_t status;

    if (key_size_code(n_len, &key_size) != 0 || d_len > n_len ||
            p_len != n_len / 2 || q_len != n_len / 2)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (!rsa_mode_supported(n_len, NVT_RSA_MODE_CRT_SCAP))
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    memset(d_padded, 0, n_len - d_len);
    memcpy(d_padded + (n_len - d_len), d, d_len);

    bin_to_hex(n, n_len, n_hex);
    bin_to_hex(p, p_len, p_hex);
    bin_to_hex(q, q_len, q_hex);
    bin_to_hex(d_padded, n_len, d_hex);
    bin_to_hex(input, n_len, in_hex);

    if (RSA_Open(CRPT, RSA_MODE_CRT_SCAP, key_size, &s_rsa_buf_crt_scap, sizeof(s_rsa_buf_crt_scap), 0) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    RSA_SetKey(CRPT, d_hex);
    RSA_SetDMATransfer(CRPT, in_hex, n_hex, p_hex, q_hex);

    status = rsa_run_and_wait();

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    RSA_Read(CRPT, out_hex);

    if (hex_to_bin_fixed(out_hex, output, n_len) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    return PSA_SUCCESS;
}

/* Side-channel-protected private-key modexp: CRT+SCAP where the HW allows it, SCAP otherwise. */
static psa_status_t rsa_modexp_protected(const uint8_t *n, size_t n_len,
                                         const uint8_t *p, size_t p_len,
                                         const uint8_t *q, size_t q_len,
                                         const uint8_t *d, size_t d_len,
                                         const uint8_t *input,
                                         uint8_t *output)
{
    if (rsa_mode_supported(n_len, NVT_RSA_MODE_CRT_SCAP))
    {
        return rsa_modexp_crt_scap(n, n_len, p, p_len, q, q_len, d, d_len, input, output);
    }

    return rsa_modexp_scap(n, n_len, p, p_len, q, q_len, d, d_len, input, output);
}

psa_status_t nvt_rsa_sign_pkcs1v15_sha256(const uint8_t *n, size_t n_len,
                                          const uint8_t *d, size_t d_len,
                                          const uint8_t hash[NVT_RSA_HASH_BYTES],
                                          uint8_t *sig, size_t sig_size, size_t *sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || d == NULL || hash == NULL || sig == NULL || sig_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_size < n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (emsa_pkcs1_v15_encode(hash, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp(n, n_len, d, d_len, em, sig);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *sig_length = n_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_sign_pkcs1v15_sha256_crt(const uint8_t *n, size_t n_len,
                                              const uint8_t *p, size_t p_len,
                                              const uint8_t *q, size_t q_len,
                                              const uint8_t *d, size_t d_len,
                                              const uint8_t hash[NVT_RSA_HASH_BYTES],
                                              uint8_t *sig, size_t sig_size, size_t *sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || p == NULL || q == NULL || d == NULL || hash == NULL ||
            sig == NULL || sig_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_size < n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (emsa_pkcs1_v15_encode(hash, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp_crt(n, n_len, p, p_len, q, q_len, d, d_len, em, sig);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *sig_length = n_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_sign_pkcs1v15_sha256_scap(const uint8_t *n, size_t n_len,
                                               const uint8_t *p, size_t p_len,
                                               const uint8_t *q, size_t q_len,
                                               const uint8_t *d, size_t d_len,
                                               const uint8_t hash[NVT_RSA_HASH_BYTES],
                                               uint8_t *sig, size_t sig_size, size_t *sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || p == NULL || q == NULL || d == NULL || hash == NULL ||
            sig == NULL || sig_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_size < n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (emsa_pkcs1_v15_encode(hash, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp_scap(n, n_len, p, p_len, q, q_len, d, d_len, em, sig);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *sig_length = n_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_sign_pkcs1v15_sha256_crt_scap(const uint8_t *n, size_t n_len,
                                                   const uint8_t *p, size_t p_len,
                                                   const uint8_t *q, size_t q_len,
                                                   const uint8_t *d, size_t d_len,
                                                   const uint8_t hash[NVT_RSA_HASH_BYTES],
                                                   uint8_t *sig, size_t sig_size, size_t *sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || p == NULL || q == NULL || d == NULL || hash == NULL ||
            sig == NULL || sig_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_size < n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (emsa_pkcs1_v15_encode(hash, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp_crt_scap(n, n_len, p, p_len, q, q_len, d, d_len, em, sig);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *sig_length = n_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_verify_pkcs1v15_sha256(const uint8_t *n, size_t n_len,
                                            const uint8_t *e, size_t e_len,
                                            const uint8_t hash[NVT_RSA_HASH_BYTES],
                                            const uint8_t *sig, size_t sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || e == NULL || e_len == 0 || hash == NULL || sig == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_length != n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp(n, n_len, e, e_len, sig, em);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    if (emsa_pkcs1_v15_verify(em, n_len, hash) != 0)
    {
        return PSA_ERROR_INVALID_SIGNATURE;
    }

    return PSA_SUCCESS;
}

/* ------------------------------------------------------------------------
 * PSS / OAEP support (MGF1-SHA-256).
 * ------------------------------------------------------------------------ */

/* Reads `len` random bytes from the on-chip TRNG (same sequence as used by
 * nvt_psa_ecc.c; duplicated here to keep this file self-contained). */
static int trng_random(uint8_t *out, size_t len)
{
    size_t i;

    if ((TRNG->CTL & TRNG_CTL_READY_Msk) == 0U)
    {
        int32_t timeout = 0x1000;

        CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;
        CLK->APBCLK1 |= CLK_APBCLK1_TRNGCKEN_Msk;
        CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;
        RTC->LXTCTL |= (RTC_LXTCTL_C32KSEL_Msk | RTC_LXTCTL_LIRC32KEN_Msk);

        TRNG->ACT |= TRNG_ACT_ACT_Msk;

        while ((TRNG->CTL & TRNG_CTL_READY_Msk) == 0U)
        {
            if (timeout-- <= 0)
            {
                return -1;
            }
        }

        TRNG->CTL = (0U << TRNG_CTL_CLKPSC_Pos);
    }

    {
        uint32_t base_ctl = TRNG->CTL;

        for (i = 0; i < len; i++)
        {
            int32_t timeout = 0x100000;
            TRNG->CTL = TRNG_CTL_TRNGEN_Msk | base_ctl;

            while ((TRNG->CTL & TRNG_CTL_DVIF_Msk) == 0U)
            {
                if (timeout-- <= 0)
                {
                    return -1;
                }
            }

            out[i] = (uint8_t) TRNG->DATA;
        }
    }

    return 0;
}

static void xor_buf(uint8_t *dst, const uint8_t *mask, size_t len)
{
    size_t i;

    for (i = 0; i < len; i++)
    {
        dst[i] ^= mask[i];
    }
}

/* MGF1 (RFC 8017 appendix B.2.1) using SHA-256. seed_len must be
 * <= NVT_RSA_MAX_BYTES (true for every seed used in this file: hLen for the
 * OAEP seed-mask direction, up to n_len-hLen-1 for the DB-mask direction). */
static void mgf1_sha256(const uint8_t *seed, size_t seed_len, uint8_t *mask, size_t mask_len)
{
    static uint8_t buf[NVT_RSA_MAX_BYTES + 4];
    uint8_t digest[NVT_RSA_HASH_BYTES];
    size_t generated = 0;
    uint32_t counter = 0;

    memcpy(buf, seed, seed_len);

    while (generated < mask_len)
    {
        size_t take = mask_len - generated;

        if (take > NVT_RSA_HASH_BYTES)
        {
            take = NVT_RSA_HASH_BYTES;
        }

        buf[seed_len]     = (uint8_t)(counter >> 24);
        buf[seed_len + 1] = (uint8_t)(counter >> 16);
        buf[seed_len + 2] = (uint8_t)(counter >> 8);
        buf[seed_len + 3] = (uint8_t)(counter);
        mbedtls_sha256(buf, seed_len + 4, digest, 0);

        memcpy(mask + generated, digest, take);
        generated += take;
        counter++;
    }
}

/* EMSA-PSS-ENCODE (RFC 8017 section 9.1.1). em_len == n_len; assumes the
 * modulus's top bit is set (true for any properly generated RSA key), so
 * emBits = 8*em_len - 1 and exactly the top bit of the first output byte is
 * cleared. */
static int emsa_pss_encode(const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                           uint8_t *em, size_t em_len)
{
    static uint8_t salt[NVT_RSA_MAX_BYTES];
    static uint8_t mprime[8 + NVT_RSA_HASH_BYTES + NVT_RSA_MAX_BYTES];
    uint8_t h[NVT_RSA_HASH_BYTES];
    uint8_t *db;
    size_t db_len;

    if (em_len < NVT_RSA_HASH_BYTES + salt_len + 2 || salt_len > NVT_RSA_MAX_BYTES)
    {
        return -1;
    }

    if (salt_len > 0 && trng_random(salt, salt_len) != 0)
    {
        return -1;
    }

    memset(mprime, 0, 8);
    memcpy(mprime + 8, hash, NVT_RSA_HASH_BYTES);
    memcpy(mprime + 8 + NVT_RSA_HASH_BYTES, salt, salt_len);
    mbedtls_sha256(mprime, 8 + NVT_RSA_HASH_BYTES + salt_len, h, 0);

    db_len = em_len - NVT_RSA_HASH_BYTES - 1;
    db = em; /* build DB directly in the front of EM, then mask in place */
    memset(db, 0, db_len - salt_len - 1);
    db[db_len - salt_len - 1] = 0x01;
    memcpy(db + db_len - salt_len, salt, salt_len);

    {
        static uint8_t dbmask[NVT_RSA_MAX_BYTES];
        mgf1_sha256(h, NVT_RSA_HASH_BYTES, dbmask, db_len);
        xor_buf(db, dbmask, db_len);
    }
    db[0] &= 0x7F;

    memcpy(em + db_len, h, NVT_RSA_HASH_BYTES);
    em[em_len - 1] = 0xBC;
    return 0;
}

/* EMSA-PSS-VERIFY (RFC 8017 section 9.1.2). Operates on public data
 * (message hash + signature), so unlike OAEP-decode this does not need the
 * uniform-error-path treatment (no secret is being unmasked here). */
static int emsa_pss_verify(const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                           const uint8_t *em, size_t em_len)
{
    static uint8_t db[NVT_RSA_MAX_BYTES];
    static uint8_t mprime[8 + NVT_RSA_HASH_BYTES + NVT_RSA_MAX_BYTES];
    uint8_t hprime[NVT_RSA_HASH_BYTES];
    const uint8_t *h;
    size_t db_len;
    size_t i;
    unsigned diff;

    if (em_len < NVT_RSA_HASH_BYTES + salt_len + 2 || salt_len > NVT_RSA_MAX_BYTES)
    {
        return -1;
    }

    if (em[em_len - 1] != 0xBC)
    {
        return -1;
    }

    db_len = em_len - NVT_RSA_HASH_BYTES - 1;
    h = em + db_len;

    if ((em[0] & 0x80) != 0)
    {
        return -1;
    }

    mgf1_sha256(h, NVT_RSA_HASH_BYTES, db, db_len);
    xor_buf(db, em, db_len);
    db[0] &= 0x7F;

    diff = 0;

    for (i = 0; i < db_len - salt_len - 1; i++)
    {
        diff |= db[i];
    }

    diff |= (unsigned)(db[db_len - salt_len - 1] ^ 0x01);

    if (diff != 0)
    {
        return -1;
    }

    memset(mprime, 0, 8);
    memcpy(mprime + 8, hash, NVT_RSA_HASH_BYTES);
    memcpy(mprime + 8 + NVT_RSA_HASH_BYTES, db + db_len - salt_len, salt_len);
    mbedtls_sha256(mprime, 8 + NVT_RSA_HASH_BYTES + salt_len, hprime, 0);

    diff = 0;

    for (i = 0; i < NVT_RSA_HASH_BYTES; i++)
    {
        diff |= (unsigned)(hprime[i] ^ h[i]);
    }

    return diff == 0 ? 0 : -1;
}

psa_status_t nvt_rsa_sign_pss_sha256(const uint8_t *n, size_t n_len,
                                     const uint8_t *d, size_t d_len,
                                     const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                                     uint8_t *sig, size_t sig_size, size_t *sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || d == NULL || hash == NULL || sig == NULL || sig_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_size < n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (emsa_pss_encode(hash, salt_len, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp(n, n_len, d, d_len, em, sig);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *sig_length = n_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_sign_pss_sha256_crt(const uint8_t *n, size_t n_len,
                                         const uint8_t *p, size_t p_len,
                                         const uint8_t *q, size_t q_len,
                                         const uint8_t *d, size_t d_len,
                                         const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                                         uint8_t *sig, size_t sig_size, size_t *sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || p == NULL || q == NULL || d == NULL || hash == NULL ||
            sig == NULL || sig_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_size < n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (emsa_pss_encode(hash, salt_len, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp_crt(n, n_len, p, p_len, q, q_len, d, d_len, em, sig);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *sig_length = n_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_verify_pss_sha256(const uint8_t *n, size_t n_len,
                                       const uint8_t *e, size_t e_len,
                                       const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                                       const uint8_t *sig, size_t sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || e == NULL || e_len == 0 || hash == NULL || sig == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_length != n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp(n, n_len, e, e_len, sig, em);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    if (emsa_pss_verify(hash, salt_len, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_SIGNATURE;
    }

    return PSA_SUCCESS;
}

/* RSAES-OAEP-ENCRYPT (RFC 8017 section 7.1.1). em_len == n_len. */
static int oaep_encode(const uint8_t *label, size_t label_len,
                       const uint8_t *msg, size_t msg_len,
                       uint8_t *em, size_t em_len)
{
    static uint8_t dbmask[NVT_RSA_MAX_BYTES];
    static uint8_t seedmask[NVT_RSA_HASH_BYTES];
    uint8_t *db;
    uint8_t *seed;
    size_t db_len;

    if (em_len < 2 * NVT_RSA_HASH_BYTES + 2 ||
            msg_len > em_len - 2 * NVT_RSA_HASH_BYTES - 2)
    {
        return -1;
    }

    db_len = em_len - NVT_RSA_HASH_BYTES - 1;
    seed = em + 1;
    db = em + 1 + NVT_RSA_HASH_BYTES;

    em[0] = 0x00;

    if (trng_random(seed, NVT_RSA_HASH_BYTES) != 0)
    {
        return -1;
    }

    mbedtls_sha256(label, label_len, db, 0); /* lHash at the front of DB */
    memset(db + NVT_RSA_HASH_BYTES, 0,
           db_len - NVT_RSA_HASH_BYTES - 1 - msg_len);
    db[db_len - msg_len - 1] = 0x01;
    memcpy(db + db_len - msg_len, msg, msg_len);

    mgf1_sha256(seed, NVT_RSA_HASH_BYTES, dbmask, db_len);
    xor_buf(db, dbmask, db_len);

    mgf1_sha256(db, db_len, seedmask, NVT_RSA_HASH_BYTES);
    xor_buf(seed, seedmask, NVT_RSA_HASH_BYTES);
    return 0;
}

/*
 * RSAES-OAEP-DECODE (RFC 8017 section 7.1.2 steps 3.c-3.g), written to
 * accumulate a single `bad` flag across every check (label hash mismatch,
 * leading byte, missing 0x01 delimiter) instead of returning as soon as one
 * check fails. This intentionally avoids branching on *which* check failed
 * so a caller/attacker cannot distinguish failure reasons via timing -
 * mirroring the standard mbedtls/RFC 8017 mitigation for Manger's attack.
 * `em` is consumed/overwritten as scratch space.
 */
static int oaep_decode(const uint8_t *label, size_t label_len,
                       uint8_t *em, size_t em_len,
                       uint8_t *msg, size_t msg_size, size_t *msg_len)
{
    static uint8_t lhash[NVT_RSA_HASH_BYTES];
    static uint8_t seedmask[NVT_RSA_HASH_BYTES];
    static uint8_t dbmask[NVT_RSA_MAX_BYTES];
    uint8_t *seed;
    uint8_t *db;
    size_t db_len;
    unsigned bad;
    unsigned found_delim;
    size_t delim_index;
    size_t i;

    if (em_len < 2 * NVT_RSA_HASH_BYTES + 2)
    {
        return -1;
    }

    db_len = em_len - NVT_RSA_HASH_BYTES - 1;
    seed = em + 1;
    db = em + 1 + NVT_RSA_HASH_BYTES;

    mgf1_sha256(db, db_len, seedmask, NVT_RSA_HASH_BYTES);
    xor_buf(seed, seedmask, NVT_RSA_HASH_BYTES);

    mgf1_sha256(seed, NVT_RSA_HASH_BYTES, dbmask, db_len);
    xor_buf(db, dbmask, db_len);

    mbedtls_sha256(label, label_len, lhash, 0);

    bad = (unsigned) em[0]; /* the leading Y byte must be 0x00 */

    for (i = 0; i < NVT_RSA_HASH_BYTES; i++)
    {
        bad |= (unsigned)(db[i] ^ lhash[i]);
    }

    found_delim = 0;
    delim_index = 0;

    for (i = NVT_RSA_HASH_BYTES; i < db_len; i++)
    {
        unsigned is_zero = (db[i] == 0x00) ? 1U : 0U;
        unsigned is_one  = (db[i] == 0x01) ? 1U : 0U;
        unsigned this_is_delim = (unsigned)(!found_delim) & is_one;
        unsigned invalid_here = (unsigned)(!found_delim) & (unsigned)(!(is_zero | is_one));

        bad |= invalid_here;
        delim_index = found_delim ? delim_index : (this_is_delim ? (i + 1) : delim_index);
        found_delim |= this_is_delim;
    }

    bad |= (found_delim == 0U) ? 1U : 0U;

    if (bad != 0U)
    {
        return -1;
    }

    {
        size_t mlen = db_len - delim_index;

        if (mlen > msg_size)
        {
            return -1;
        }

        memcpy(msg, db + delim_index, mlen);
        *msg_len = mlen;
    }

    return 0;
}

psa_status_t nvt_rsa_oaep_encrypt_sha256(const uint8_t *n, size_t n_len,
                                         const uint8_t *e, size_t e_len,
                                         const uint8_t *label, size_t label_len,
                                         const uint8_t *msg, size_t msg_len,
                                         uint8_t *out, size_t out_size, size_t *out_len)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];

    if (n == NULL || e == NULL || e_len == 0 || msg == NULL || out == NULL || out_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (label == NULL && label_len != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || out_size < n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (oaep_encode(label, label_len, msg, msg_len, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (rsa_modexp(n, n_len, e, e_len, em, out) != PSA_SUCCESS)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    *out_len = n_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_oaep_decrypt_sha256(const uint8_t *n, size_t n_len,
                                         const uint8_t *d, size_t d_len,
                                         const uint8_t *label, size_t label_len,
                                         const uint8_t *in, size_t in_len,
                                         uint8_t *msg, size_t msg_size, size_t *msg_len)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];

    if (n == NULL || d == NULL || in == NULL || msg == NULL || msg_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (label == NULL && label_len != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || in_len != n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (rsa_modexp(n, n_len, d, d_len, in, em) != PSA_SUCCESS)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    if (oaep_decode(label, label_len, em, n_len, msg, msg_size, msg_len) != 0)
    {
        return PSA_ERROR_INVALID_PADDING;
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_oaep_decrypt_sha256_crt(const uint8_t *n, size_t n_len,
                                             const uint8_t *p, size_t p_len,
                                             const uint8_t *q, size_t q_len,
                                             const uint8_t *d, size_t d_len,
                                             const uint8_t *label, size_t label_len,
                                             const uint8_t *in, size_t in_len,
                                             uint8_t *msg, size_t msg_size, size_t *msg_len)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];

    if (n == NULL || p == NULL || q == NULL || d == NULL || in == NULL ||
            msg == NULL || msg_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (label == NULL && label_len != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || in_len != n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (rsa_modexp_crt(n, n_len, p, p_len, q, q_len, d, d_len, in, em) != PSA_SUCCESS)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    if (oaep_decode(label, label_len, em, n_len, msg, msg_size, msg_len) != 0)
    {
        return PSA_ERROR_INVALID_PADDING;
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_oaep_decrypt_sha256_scap(const uint8_t *n, size_t n_len,
                                              const uint8_t *p, size_t p_len,
                                              const uint8_t *q, size_t q_len,
                                              const uint8_t *d, size_t d_len,
                                              const uint8_t *label, size_t label_len,
                                              const uint8_t *in, size_t in_len,
                                              uint8_t *msg, size_t msg_size, size_t *msg_len)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];

    if (n == NULL || p == NULL || q == NULL || d == NULL || in == NULL ||
            msg == NULL || msg_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (label == NULL && label_len != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || in_len != n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (rsa_modexp_scap(n, n_len, p, p_len, q, q_len, d, d_len, in, em) != PSA_SUCCESS)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    if (oaep_decode(label, label_len, em, n_len, msg, msg_size, msg_len) != 0)
    {
        return PSA_ERROR_INVALID_PADDING;
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_oaep_decrypt_sha256_crt_scap(const uint8_t *n, size_t n_len,
                                                  const uint8_t *p, size_t p_len,
                                                  const uint8_t *q, size_t q_len,
                                                  const uint8_t *d, size_t d_len,
                                                  const uint8_t *label, size_t label_len,
                                                  const uint8_t *in, size_t in_len,
                                                  uint8_t *msg, size_t msg_size, size_t *msg_len)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];

    if (n == NULL || p == NULL || q == NULL || d == NULL || in == NULL ||
            msg == NULL || msg_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (label == NULL && label_len != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || in_len != n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (rsa_modexp_crt_scap(n, n_len, p, p_len, q, q_len, d, d_len, in, em) != PSA_SUCCESS)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    if (oaep_decode(label, label_len, em, n_len, msg, msg_size, msg_len) != 0)
    {
        return PSA_ERROR_INVALID_PADDING;
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_sign_pkcs1v15_sha256_protected(const uint8_t *n, size_t n_len,
                                                    const uint8_t *p, size_t p_len,
                                                    const uint8_t *q, size_t q_len,
                                                    const uint8_t *d, size_t d_len,
                                                    const uint8_t hash[NVT_RSA_HASH_BYTES],
                                                    uint8_t *sig, size_t sig_size, size_t *sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || p == NULL || q == NULL || d == NULL || hash == NULL ||
            sig == NULL || sig_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_size < n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (emsa_pkcs1_v15_encode(hash, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp_protected(n, n_len, p, p_len, q, q_len, d, d_len, em, sig);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *sig_length = n_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_sign_pss_sha256_protected(const uint8_t *n, size_t n_len,
                                               const uint8_t *p, size_t p_len,
                                               const uint8_t *q, size_t q_len,
                                               const uint8_t *d, size_t d_len,
                                               const uint8_t hash[NVT_RSA_HASH_BYTES], size_t salt_len,
                                               uint8_t *sig, size_t sig_size, size_t *sig_length)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];
    psa_status_t status;

    if (n == NULL || p == NULL || q == NULL || d == NULL || hash == NULL ||
            sig == NULL || sig_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || sig_size < n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (emsa_pss_encode(hash, salt_len, em, n_len) != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = rsa_modexp_protected(n, n_len, p, p_len, q, q_len, d, d_len, em, sig);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *sig_length = n_len;
    return PSA_SUCCESS;
}

psa_status_t nvt_rsa_oaep_decrypt_sha256_protected(const uint8_t *n, size_t n_len,
                                                   const uint8_t *p, size_t p_len,
                                                   const uint8_t *q, size_t q_len,
                                                   const uint8_t *d, size_t d_len,
                                                   const uint8_t *label, size_t label_len,
                                                   const uint8_t *in, size_t in_len,
                                                   uint8_t *msg, size_t msg_size, size_t *msg_len)
{
    static uint8_t em[NVT_RSA_MAX_BYTES];

    if (n == NULL || p == NULL || q == NULL || d == NULL || in == NULL ||
            msg == NULL || msg_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (label == NULL && label_len != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_len < NVT_RSA_MIN_BYTES || n_len > NVT_RSA_MAX_BYTES || in_len != n_len)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (rsa_modexp_protected(n, n_len, p, p_len, q, q_len, d, d_len, in, em) != PSA_SUCCESS)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    if (oaep_decode(label, label_len, em, n_len, msg, msg_size, msg_len) != 0)
    {
        return PSA_ERROR_INVALID_PADDING;
    }

    return PSA_SUCCESS;
}

/* ------------------------------------------------------------------------
 * Key generation (software mbedtls_rsa_gen_key(), TRNG-seeded).
 * ------------------------------------------------------------------------ */

static int rsa_keygen_rng(void *ctx, unsigned char *buf, size_t len)
{
    (void) ctx;
    return trng_random(buf, len) == 0 ? 0 : -1;
}

psa_status_t nvt_rsa_generate_key_pair(size_t nbits, uint32_t exponent,
                                       uint8_t *n, size_t n_size, size_t *n_len,
                                       uint8_t *e, size_t e_size, size_t *e_len,
                                       uint8_t *d, size_t d_size, size_t *d_len,
                                       uint8_t *p, size_t p_size, size_t *p_len,
                                       uint8_t *q, size_t q_size, size_t *q_len)
{
    mbedtls_rsa_context ctx;
    size_t n_bytes = nbits / 8;
    size_t p_bytes = n_bytes / 2;
    size_t e_bytes;
    size_t d_bytes;

    if (n == NULL || e == NULL || d == NULL || p == NULL || q == NULL ||
            n_len == NULL || e_len == NULL || d_len == NULL || p_len == NULL || q_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (nbits != 1024 && nbits != 2048 && nbits != 3072 && nbits != 4096)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (n_size < n_bytes || p_size < p_bytes || q_size < p_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    mbedtls_rsa_init(&ctx);

    if (mbedtls_rsa_gen_key(&ctx, rsa_keygen_rng, NULL,
                            (unsigned int) nbits, (int) exponent) != 0)
    {
        mbedtls_rsa_free(&ctx);
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    e_bytes = mbedtls_mpi_size(&ctx.MBEDTLS_PRIVATE(E));
    d_bytes = mbedtls_mpi_size(&ctx.MBEDTLS_PRIVATE(D));

    if (e_size < e_bytes || d_size < d_bytes ||
            mbedtls_mpi_write_binary(&ctx.MBEDTLS_PRIVATE(N), n, n_bytes) != 0 ||
            mbedtls_mpi_write_binary(&ctx.MBEDTLS_PRIVATE(E), e, e_bytes) != 0 ||
            mbedtls_mpi_write_binary(&ctx.MBEDTLS_PRIVATE(D), d, d_bytes) != 0 ||
            mbedtls_mpi_write_binary(&ctx.MBEDTLS_PRIVATE(P), p, p_bytes) != 0 ||
            mbedtls_mpi_write_binary(&ctx.MBEDTLS_PRIVATE(Q), q, p_bytes) != 0)
    {
        mbedtls_rsa_free(&ctx);
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    mbedtls_rsa_free(&ctx);

    *n_len = n_bytes;
    *e_len = e_bytes;
    *d_len = d_bytes;
    *p_len = p_bytes;
    *q_len = p_bytes;
    return PSA_SUCCESS;
}

/* ------------------------------------------------------------------------
 * Self-test (mirrors nvt_psa_aes.c's nvt_aes_self_test() pattern).
 * ------------------------------------------------------------------------ */

static int s_selftest_pass;
static int s_selftest_total;

static void rsa_selftest_report(const char *name, int ok)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    s_selftest_total++;
    s_selftest_pass += ok ? 1 : 0;
}

/* Exercises sign+verify and OAEP encrypt+decrypt for one key size, across
 * every mode nvt_rsa_mode_supported() reports as available for that size. */
static void rsa_selftest_key_size(size_t nbits)
{
    static uint8_t n[NVT_RSA_MAX_BYTES], e[NVT_RSA_MAX_BYTES], d[NVT_RSA_MAX_BYTES];
    static uint8_t p[NVT_RSA_MAX_BYTES / 2], q[NVT_RSA_MAX_BYTES / 2];
    static uint8_t sig[NVT_RSA_MAX_BYTES];
    static uint8_t cipher[NVT_RSA_MAX_BYTES];
    static uint8_t plain[NVT_RSA_MAX_BYTES];
    uint8_t hash[NVT_RSA_HASH_BYTES];
    size_t n_len, e_len, d_len, p_len, q_len, sig_len, cipher_len, plain_len;
    const char message[] = "nvt RSA self-test message";
    char label[64];

    if (nvt_rsa_generate_key_pair(nbits, 65537U,
                                  n, sizeof(n), &n_len, e, sizeof(e), &e_len,
                                  d, sizeof(d), &d_len, p, sizeof(p), &p_len,
                                  q, sizeof(q), &q_len) != PSA_SUCCESS)
    {
        snprintf(label, sizeof(label), "RSA-%u keygen", (unsigned) nbits);
        rsa_selftest_report(label, 0);
        return;
    }

    mbedtls_sha256((const unsigned char *) message, strlen(message), hash, 0);

    /* Normal mode: always supported. */
    snprintf(label, sizeof(label), "RSA-%u PKCS1v15 sign/verify (Normal)", (unsigned) nbits);
    rsa_selftest_report(label,
                        nvt_rsa_sign_pkcs1v15_sha256(n, n_len, d, d_len, hash, sig, sizeof(sig), &sig_len) == PSA_SUCCESS &&
                        nvt_rsa_verify_pkcs1v15_sha256(n, n_len, e, e_len, hash, sig, sig_len) == PSA_SUCCESS);

    snprintf(label, sizeof(label), "RSA-%u OAEP encrypt/decrypt (Normal)", (unsigned) nbits);
    rsa_selftest_report(label,
                        nvt_rsa_oaep_encrypt_sha256(n, n_len, e, e_len, NULL, 0,
                                                    (const uint8_t *) message, strlen(message),
                                                    cipher, sizeof(cipher), &cipher_len) == PSA_SUCCESS &&
                        nvt_rsa_oaep_decrypt_sha256(n, n_len, d, d_len, NULL, 0, cipher, cipher_len,
                                                    plain, sizeof(plain), &plain_len) == PSA_SUCCESS &&
                        plain_len == strlen(message) && memcmp(plain, message, plain_len) == 0);

    /* CRT mode: only where nvt_rsa_mode_supported() reports it available. */
    if (nvt_rsa_mode_supported(n_len, 1, 0) == PSA_SUCCESS)
    {
        snprintf(label, sizeof(label), "RSA-%u PKCS1v15 sign (CRT) / verify", (unsigned) nbits);
        rsa_selftest_report(label,
                            nvt_rsa_sign_pkcs1v15_sha256_crt(n, n_len, p, p_len, q, q_len, d, d_len,
                                                             hash, sig, sizeof(sig), &sig_len) == PSA_SUCCESS &&
                            nvt_rsa_verify_pkcs1v15_sha256(n, n_len, e, e_len, hash, sig, sig_len) == PSA_SUCCESS);
    }
    else
    {
        snprintf(label, sizeof(label), "RSA-%u CRT correctly rejected as unsupported", (unsigned) nbits);
        rsa_selftest_report(label,
                            nvt_rsa_sign_pkcs1v15_sha256_crt(n, n_len, p, p_len, q, q_len, d, d_len,
                                                             hash, sig, sizeof(sig), &sig_len) == PSA_ERROR_NOT_SUPPORTED);
    }

    /* SCAP mode: per rsa_mode_supported(), currently expected at every size. */
    if (nvt_rsa_mode_supported(n_len, 0, 1) == PSA_SUCCESS)
    {
        snprintf(label, sizeof(label), "RSA-%u PKCS1v15 sign (SCAP) / verify", (unsigned) nbits);
        rsa_selftest_report(label,
                            nvt_rsa_sign_pkcs1v15_sha256_scap(n, n_len, p, p_len, q, q_len, d, d_len,
                                                              hash, sig, sizeof(sig), &sig_len) == PSA_SUCCESS &&
                            nvt_rsa_verify_pkcs1v15_sha256(n, n_len, e, e_len, hash, sig, sig_len) == PSA_SUCCESS);
    }

    /* CRT_SCAP mode: same gating as plain CRT. */
    if (nvt_rsa_mode_supported(n_len, 1, 1) == PSA_SUCCESS)
    {
        snprintf(label, sizeof(label), "RSA-%u PKCS1v15 sign (CRT+SCAP) / verify", (unsigned) nbits);
        rsa_selftest_report(label,
                            nvt_rsa_sign_pkcs1v15_sha256_crt_scap(n, n_len, p, p_len, q, q_len, d, d_len,
                                                                  hash, sig, sizeof(sig), &sig_len) == PSA_SUCCESS &&
                            nvt_rsa_verify_pkcs1v15_sha256(n, n_len, e, e_len, hash, sig, sig_len) == PSA_SUCCESS);
    }
}

int nvt_rsa_self_test(void)
{
    static const size_t s_key_sizes[] = {1024, 2048, 3072, 4096};
    size_t i;

    s_selftest_pass = 0;
    s_selftest_total = 0;

    printf("=========================================\n");
    printf("  RSA HW Accelerator Verification Suite\n");
    printf("=========================================\n");

    for (i = 0; i < sizeof(s_key_sizes) / sizeof(s_key_sizes[0]); i++)
    {
        rsa_selftest_key_size(s_key_sizes[i]);
    }

    printf("\n=========================================\n");
    printf("  Test Results: %d/%d PASSED\n", s_selftest_pass, s_selftest_total);
    printf("=========================================\n");

    return (s_selftest_pass == s_selftest_total) ? 0 : 1;
}
