/*
 * Nuvoton native ECC (P-256) HW accelerator: implementation.
 * See nvt_psa_ecc.h for the API contract and hardware preconditions.
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

/*
 * Nuvoton native ECC HW accelerator: implementation.
 * See nvt_psa_ecc.h for the API contract and hardware preconditions.
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

#include "nvt_psa_ecc.h"

#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include <psa/crypto.h>

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

/* Parses a hex string (any length <= 2*out_len, possibly odd) into a
 * big-endian buffer of out_len bytes, left-padding with zero bytes. Needed
 * because StdDriver's ECC_GetCurve()/ECC_*Signature/GeneratePublicKey/
 * GenerateSecretZ hex strings are not guaranteed to be a fixed 2*out_len
 * width for every curve (e.g. P-521's 521-bit order doesn't fill a whole
 * number of hex digits). */
static int hex_to_bin_leftpad(const char *hex, uint8_t *out, size_t out_len)
{
    size_t hexlen = strlen(hex);
    size_t nbytes = (hexlen + 1U) / 2U;
    size_t pad;
    size_t i;

    if (nbytes > out_len)
    {
        return -1;
    }

    pad = out_len - nbytes;
    memset(out, 0, pad);

    if ((hexlen & 1U) != 0U)
    {
        int v = hex_nibble(hex[0]);

        if (v < 0)
        {
            return -1;
        }

        out[pad] = (uint8_t) v;
        hex++;
        hexlen--;

        for (i = 0; i < hexlen / 2U; i++)
        {
            int hi = hex_nibble(hex[2 * i]);
            int lo = hex_nibble(hex[2 * i + 1]);

            if (hi < 0 || lo < 0)
            {
                return -1;
            }

            out[pad + 1 + i] = (uint8_t)((hi << 4) | lo);
        }
    }
    else
    {
        for (i = 0; i < hexlen / 2U; i++)
        {
            int hi = hex_nibble(hex[2 * i]);
            int lo = hex_nibble(hex[2 * i + 1]);

            if (hi < 0 || lo < 0)
            {
                return -1;
            }

            out[pad + i] = (uint8_t)((hi << 4) | lo);
        }
    }

    return 0;
}

/* Constant-time-ish big-endian unsigned compare: <0, 0, >0. Not
 * secret-independent timing critical here (order/curve constants are
 * public). */
static int compare_be(const uint8_t *a, const uint8_t *b, size_t len)
{
    size_t i;

    for (i = 0; i < len; i++)
    {
        if (a[i] != b[i])
        {
            return (a[i] < b[i]) ? -1 : 1;
        }
    }

    return 0;
}

static int is_all_zero(const uint8_t *a, size_t len)
{
    size_t i;
    uint8_t acc = 0;

    for (i = 0; i < len; i++)
    {
        acc |= a[i];
    }

    return acc == 0;
}

/* big-endian subtract: a = a - b, a >= b required, len bytes each. */
static void sub_be(uint8_t *a, const uint8_t *b, size_t len)
{
    int borrow = 0;
    int i;

    for (i = (int) len - 1; i >= 0; i--)
    {
        int diff = (int) a[i] - (int) b[i] - borrow;

        if (diff < 0)
        {
            diff += 256;
            borrow = 1;
        }
        else
        {
            borrow = 0;
        }

        a[i] = (uint8_t) diff;
    }
}

/* Reads `len` random bytes from the on-chip TRNG peripheral. */
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

/* Rejection-sample a scalar in [1, n-1] from the TRNG (n = order, n_bytes wide).
 * Bits above the order's top bit are cleared first, so each try succeeds with
 * probability >= 1/2 even when the order is far below 2^(8*n_bytes). */
static psa_status_t random_scalar_in_range(const uint8_t *order, size_t n_bytes, uint8_t *out)
{
    size_t top = 0;
    uint8_t mask = 0xFF;
    int attempt;

    while (top < n_bytes && order[top] == 0U)
    {
        top++;
    }

    if (top == n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    while ((mask >> 1) >= order[top])
    {
        mask >>= 1;
    }

    for (attempt = 0; attempt < 64; attempt++)
    {
        if (trng_random(out, n_bytes) != 0)
        {
            return PSA_ERROR_HARDWARE_FAILURE;
        }

        memset(out, 0, top);
        out[top] &= mask;

        if (is_all_zero(out, n_bytes))
        {
            continue;
        }

        if (compare_be(out, order, n_bytes) < 0)
        {
            return PSA_SUCCESS;
        }
    }

    return PSA_ERROR_INSUFFICIENT_ENTROPY;
}

/* Derive the SEC1 4.1.3-step-5 integer e from a hash: the leftmost
 * bitlen(n) bits of the hash, then reduced once modulo the group order. */
static void derive_e(const uint8_t *order, size_t n_bytes,
                     const uint8_t *hash, size_t hash_len, uint8_t *e)
{
    size_t use_len = (hash_len > n_bytes) ? n_bytes : hash_len;
    size_t pad = n_bytes - use_len;
    size_t order_bits = 8U * n_bytes;
    size_t i;

    memset(e, 0, pad);
    memcpy(e + pad, hash, use_len);

    for (i = 0; i < n_bytes && order[i] == 0U; i++)
    {
        order_bits -= 8U;
    }

    if (i < n_bytes)
    {
        uint8_t top = order[i];

        while ((top & 0x80U) == 0U)
        {
            order_bits--;
            top <<= 1;
        }
    }

    /* Only possible when use_len == n_bytes, so the shift is 0..7 bits. */
    if (8U * use_len > order_bits)
    {
        unsigned shift = (unsigned)(8U * use_len - order_bits);
        uint8_t carry = 0U;

        for (i = 0; i < n_bytes; i++)
        {
            uint8_t b = e[i];
            e[i] = (uint8_t)((b >> shift) | carry);
            carry = (uint8_t)(b << (8U - shift));
        }
    }

    if (compare_be(e, order, n_bytes) >= 0)
    {
        sub_be(e, order, n_bytes);
    }
}

/* Curve25519 (Montgomery/X25519) and SM2 use different math/encodings than
 * ECDSA - see the header comment for why they are excluded here. */
static int curve_supported(E_ECC_CURVE curve)
{
    return curve != CURVE_25519 && curve != CURVE_SM2_256;
}

/* Bit length of a big-endian hex string, ignoring leading zeros. */
static size_t hex_bit_length(const char *hex)
{
    int top;
    size_t bits;

    while (*hex == '0')
    {
        hex++;
    }

    if (*hex == '\0')
    {
        return 0U;
    }

    top = hex_nibble(*hex);
    bits = 4U * (strlen(hex) - 1U);

    while (top > 0)
    {
        bits++;
        top >>= 1;
    }

    return bits;
}

/* Reads the field/order width (bytes) and, if order != NULL, the group
 * order (left-padded to n_bytes) via StdDriver's own curve table. */
static int curve_info(E_ECC_CURVE curve, size_t *n_bytes, uint8_t *order)
{
    ECC_CURVE info;

    if (!curve_supported(curve))
    {
        return -1;
    }

    if (ECC_GetCurve(CRPT, curve, &info) != 0)
    {
        return -1;
    }

    *n_bytes = ((size_t) info.key_len + 7U) / 8U;

    if (*n_bytes == 0U || *n_bytes > NVT_ECC_MAX_BYTES)
    {
        return -1;
    }

    /* Cofactor <= 4 puts the order within ~2 bits of the field size. A shorter
     * table order is corrupt (StdDriver's KO-224 entry is truncated to 113 bits)
     * and would yield weak keys and a non-invertible modulus. */
    if (hex_bit_length(info.Eorder) + 3U < (size_t) info.key_len)
    {
        return -1;
    }

    if (order != NULL && hex_to_bin_leftpad(info.Eorder, order, *n_bytes) != 0)
    {
        return -1;
    }

    return 0;
}

size_t nvt_ecc_curve_bytes(E_ECC_CURVE curve)
{
    size_t n_bytes;

    if (curve_info(curve, &n_bytes, NULL) != 0)
    {
        return 0;
    }

    return n_bytes;
}

psa_status_t nvt_ecc_compute_public_key(E_ECC_CURVE curve,
                                        const uint8_t *priv, size_t priv_len,
                                        uint8_t *qx, size_t qx_size, size_t *qx_len,
                                        uint8_t *qy, size_t qy_size, size_t *qy_len)
{
    uint8_t order[NVT_ECC_MAX_BYTES];
    char d_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char qx_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char qy_hex[2 * NVT_ECC_MAX_BYTES + 1];
    size_t n_bytes;

    if (priv == NULL || qx == NULL || qy == NULL || qx_len == NULL || qy_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (curve_info(curve, &n_bytes, order) != 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (priv_len != n_bytes || qx_size < n_bytes || qy_size < n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (is_all_zero(priv, n_bytes) || compare_be(priv, order, n_bytes) >= 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    bin_to_hex(priv, n_bytes, d_hex);

    if (ECC_GeneratePublicKey(CRPT, curve, d_hex, qx_hex, qy_hex) < 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    if (hex_to_bin_leftpad(qx_hex, qx, n_bytes) != 0 ||
            hex_to_bin_leftpad(qy_hex, qy, n_bytes) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    *qx_len = n_bytes;
    *qy_len = n_bytes;
    return PSA_SUCCESS;
}

psa_status_t nvt_ecc_generate_keypair(E_ECC_CURVE curve,
                                      uint8_t *priv, size_t priv_size, size_t *priv_len,
                                      uint8_t *qx, size_t qx_size, size_t *qx_len,
                                      uint8_t *qy, size_t qy_size, size_t *qy_len)
{
    uint8_t order[NVT_ECC_MAX_BYTES];
    size_t n_bytes;
    psa_status_t status;

    if (priv == NULL || priv_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (curve_info(curve, &n_bytes, order) != 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (priv_size < n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = random_scalar_in_range(order, n_bytes, priv);

    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *priv_len = n_bytes;

    return nvt_ecc_compute_public_key(curve, priv, n_bytes, qx, qx_size, qx_len, qy, qy_size, qy_len);
}

psa_status_t nvt_ecc_sign_hash(E_ECC_CURVE curve,
                               const uint8_t *priv, size_t priv_len,
                               const uint8_t *hash, size_t hash_len,
                               uint8_t *sig, size_t sig_size, size_t *sig_len)
{
    uint8_t order[NVT_ECC_MAX_BYTES];
    uint8_t e[NVT_ECC_MAX_BYTES];
    uint8_t k[NVT_ECC_MAX_BYTES];
    char d_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char e_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char k_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char r_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char s_hex[2 * NVT_ECC_MAX_BYTES + 1];
    size_t n_bytes;
    int attempt;

    if (priv == NULL || hash == NULL || sig == NULL || sig_len == NULL || hash_len == 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (curve_info(curve, &n_bytes, order) != 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (priv_len != n_bytes || sig_size < 2 * n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (is_all_zero(priv, n_bytes) || compare_be(priv, order, n_bytes) >= 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    derive_e(order, n_bytes, hash, hash_len, e);
    bin_to_hex(priv, n_bytes, d_hex);
    bin_to_hex(e, n_bytes, e_hex);

    for (attempt = 0; attempt < 8; attempt++)
    {
        psa_status_t status = random_scalar_in_range(order, n_bytes, k);

        if (status != PSA_SUCCESS)
        {
            return status;
        }

        bin_to_hex(k, n_bytes, k_hex);

        if (ECC_GenerateSignature(CRPT, curve, e_hex, d_hex, k_hex, r_hex, s_hex) < 0)
        {
            return PSA_ERROR_HARDWARE_FAILURE;
        }

        if (hex_to_bin_leftpad(r_hex, sig, n_bytes) != 0 ||
                hex_to_bin_leftpad(s_hex, sig + n_bytes, n_bytes) != 0)
        {
            return PSA_ERROR_HARDWARE_FAILURE;
        }

        /* r == 0 or s == 0 is invalid per FIPS 186-4; retry with a new k
         * (astronomically unlikely for a properly random k). */
        if (!is_all_zero(sig, n_bytes) && !is_all_zero(sig + n_bytes, n_bytes))
        {
            *sig_len = 2 * n_bytes;
            return PSA_SUCCESS;
        }
    }

    return PSA_ERROR_GENERIC_ERROR;
}

psa_status_t nvt_ecc_verify_hash(E_ECC_CURVE curve,
                                 const uint8_t *qx, size_t qx_len,
                                 const uint8_t *qy, size_t qy_len,
                                 const uint8_t *hash, size_t hash_len,
                                 const uint8_t *sig, size_t sig_len)
{
    uint8_t order[NVT_ECC_MAX_BYTES];
    uint8_t e[NVT_ECC_MAX_BYTES];
    char qx_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char qy_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char e_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char r_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char s_hex[2 * NVT_ECC_MAX_BYTES + 1];
    size_t n_bytes;

    if (qx == NULL || qy == NULL || hash == NULL || sig == NULL || hash_len == 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (curve_info(curve, &n_bytes, order) != 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (qx_len != n_bytes || qy_len != n_bytes || sig_len != 2 * n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    /* Reject the degenerate r=0 / s=0 signatures outright (never valid). */
    if (is_all_zero(sig, n_bytes) || is_all_zero(sig + n_bytes, n_bytes))
    {
        return PSA_ERROR_INVALID_SIGNATURE;
    }

    derive_e(order, n_bytes, hash, hash_len, e);
    bin_to_hex(qx, n_bytes, qx_hex);
    bin_to_hex(qy, n_bytes, qy_hex);
    bin_to_hex(e, n_bytes, e_hex);
    bin_to_hex(sig, n_bytes, r_hex);
    bin_to_hex(sig + n_bytes, n_bytes, s_hex);

    if (ECC_VerifySignature(CRPT, curve, e_hex, qx_hex, qy_hex, r_hex, s_hex) < 0)
    {
        return PSA_ERROR_INVALID_SIGNATURE;
    }

    return PSA_SUCCESS;
}

psa_status_t nvt_ecdh_key_agreement(E_ECC_CURVE curve,
                                    const uint8_t *priv, size_t priv_len,
                                    const uint8_t *peer_qx, size_t peer_qx_len,
                                    const uint8_t *peer_qy, size_t peer_qy_len,
                                    uint8_t *shared_secret, size_t shared_secret_size,
                                    size_t *shared_secret_len)
{
    uint8_t order[NVT_ECC_MAX_BYTES];
    char d_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char qx_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char qy_hex[2 * NVT_ECC_MAX_BYTES + 1];
    char z_hex[2 * NVT_ECC_MAX_BYTES + 1];
    size_t n_bytes;

    if (priv == NULL || peer_qx == NULL || peer_qy == NULL ||
            shared_secret == NULL || shared_secret_len == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (curve_info(curve, &n_bytes, order) != 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    if (priv_len != n_bytes || peer_qx_len != n_bytes || peer_qy_len != n_bytes ||
            shared_secret_size < n_bytes)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    if (is_all_zero(priv, n_bytes) || compare_be(priv, order, n_bytes) >= 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    bin_to_hex(priv, n_bytes, d_hex);
    bin_to_hex(peer_qx, n_bytes, qx_hex);
    bin_to_hex(peer_qy, n_bytes, qy_hex);

    if (ECC_GenerateSecretZ(CRPT, curve, d_hex, qx_hex, qy_hex, z_hex) < 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    if (hex_to_bin_leftpad(z_hex, shared_secret, n_bytes) != 0)
    {
        return PSA_ERROR_HARDWARE_FAILURE;
    }

    *shared_secret_len = n_bytes;
    return PSA_SUCCESS;
}

/* ------------------------------------------------------------------------
 * Self-test (mirrors nvt_psa_aes.c's nvt_aes_self_test() pattern). Covers
 * every HW-supported curve using freshly HW-generated keys - no hardcoded
 * vectors, so there is nothing to hand-transcribe incorrectly.
 * ------------------------------------------------------------------------ */

static int s_ecc_selftest_pass;
static int s_ecc_selftest_total;

static void ecc_selftest_report(const char *name, int ok)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    s_ecc_selftest_total++;
    s_ecc_selftest_pass += ok ? 1 : 0;
}

static void ecc_selftest_curve(E_ECC_CURVE curve, const char *name)
{
    static const uint8_t hash[32] = "nvt ECC self-test fixed digest!"; /* 32 bytes, not a real SHA-256 - fine, HW only derives e from it. */
    uint8_t priv[NVT_ECC_MAX_BYTES], qx[NVT_ECC_MAX_BYTES], qy[NVT_ECC_MAX_BYTES];
    uint8_t sig[2 * NVT_ECC_MAX_BYTES];
    uint8_t alice_priv[NVT_ECC_MAX_BYTES], alice_qx[NVT_ECC_MAX_BYTES], alice_qy[NVT_ECC_MAX_BYTES];
    uint8_t bob_priv[NVT_ECC_MAX_BYTES], bob_qx[NVT_ECC_MAX_BYTES], bob_qy[NVT_ECC_MAX_BYTES];
    uint8_t alice_secret[NVT_ECC_MAX_BYTES], bob_secret[NVT_ECC_MAX_BYTES];
    size_t priv_len, qx_len, qy_len, sig_len;
    size_t alice_priv_len, alice_qx_len, alice_qy_len;
    size_t bob_priv_len, bob_qx_len, bob_qy_len;
    size_t alice_secret_len, bob_secret_len;
    char label[64];

    snprintf(label, sizeof(label), "ECC %s keygen/sign/verify", name);

    if (nvt_ecc_generate_keypair(curve, priv, sizeof(priv), &priv_len,
                                 qx, sizeof(qx), &qx_len, qy, sizeof(qy), &qy_len) != PSA_SUCCESS ||
            nvt_ecc_sign_hash(curve, priv, priv_len, hash, sizeof(hash), sig, sizeof(sig), &sig_len) != PSA_SUCCESS)
    {
        ecc_selftest_report(label, 0);
    }
    else
    {
        ecc_selftest_report(label,
                            nvt_ecc_verify_hash(curve, qx, qx_len, qy, qy_len, hash, sizeof(hash), sig, sig_len) == PSA_SUCCESS);
    }

    snprintf(label, sizeof(label), "ECC %s ECDH Alice==Bob", name);

    if (nvt_ecc_generate_keypair(curve, alice_priv, sizeof(alice_priv), &alice_priv_len,
                                 alice_qx, sizeof(alice_qx), &alice_qx_len,
                                 alice_qy, sizeof(alice_qy), &alice_qy_len) != PSA_SUCCESS ||
            nvt_ecc_generate_keypair(curve, bob_priv, sizeof(bob_priv), &bob_priv_len,
                                     bob_qx, sizeof(bob_qx), &bob_qx_len,
                                     bob_qy, sizeof(bob_qy), &bob_qy_len) != PSA_SUCCESS)
    {
        ecc_selftest_report(label, 0);
        return;
    }

    if (nvt_ecdh_key_agreement(curve, alice_priv, alice_priv_len, bob_qx, bob_qx_len, bob_qy, bob_qy_len,
                               alice_secret, sizeof(alice_secret), &alice_secret_len) != PSA_SUCCESS ||
            nvt_ecdh_key_agreement(curve, bob_priv, bob_priv_len, alice_qx, alice_qx_len, alice_qy, alice_qy_len,
                                   bob_secret, sizeof(bob_secret), &bob_secret_len) != PSA_SUCCESS)
    {
        ecc_selftest_report(label, 0);
        return;
    }

    ecc_selftest_report(label,
                        alice_secret_len == bob_secret_len && memcmp(alice_secret, bob_secret, alice_secret_len) == 0);
}

int nvt_ecc_self_test(void)
{
    static const struct
    {
        E_ECC_CURVE curve;
        const char *name;
    } s_curves[] =
    {
        {CURVE_P_192,  "P-192"},  {CURVE_P_224,  "P-224"},  {CURVE_P_256,  "P-256"},
        {CURVE_P_384,  "P-384"},  {CURVE_P_521,  "P-521"},
        {CURVE_KO_192, "KO-192"}, {CURVE_KO_256, "KO-256"},
        {CURVE_B_163,  "B-163"},  {CURVE_B_233,  "B-233"},  {CURVE_B_283,  "B-283"},
        {CURVE_B_409,  "B-409"},  {CURVE_B_571,  "B-571"},
        {CURVE_K_163,  "K-163"},  {CURVE_K_233,  "K-233"},  {CURVE_K_283,  "K-283"},
        {CURVE_K_409,  "K-409"},  {CURVE_K_571,  "K-571"},
        {CURVE_BP_256, "BP-256"}, {CURVE_BP_384, "BP-384"}, {CURVE_BP_512, "BP-512"},
    };
    size_t i;

    s_ecc_selftest_pass = 0;
    s_ecc_selftest_total = 0;

    printf("=========================================\n");
    printf("  ECC HW Accelerator Verification Suite\n");
    printf("=========================================\n");

    for (i = 0; i < sizeof(s_curves) / sizeof(s_curves[0]); i++)
    {
        ecc_selftest_curve(s_curves[i].curve, s_curves[i].name);
    }

    printf("\n=========================================\n");
    printf("  Test Results: %d/%d PASSED\n", s_ecc_selftest_pass, s_ecc_selftest_total);
    printf("=========================================\n");

    return (s_ecc_selftest_pass == s_ecc_selftest_total) ? 0 : 1;
}
