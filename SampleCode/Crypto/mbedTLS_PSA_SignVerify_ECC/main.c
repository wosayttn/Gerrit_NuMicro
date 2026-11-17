/*
 * ECDSA HW/SW interoperability verification across every curve the M2354
 * ECC accelerator supports (except Curve25519 and SM2 - see nvt_psa_ecc.h).
 *
 * Keys are generated FRESH on every run via nvt_ecc_generate_keypair() (HW
 * TRNG-seeded) instead of hand-transcribed fixed vectors - this removes the
 * class of bug found earlier in this sample (a mistyped curve-point Y
 * coordinate in a hardcoded test vector). Correctness is instead verified
 * by round-tripping: HW-sign -> HW-verify always; additionally HW-sign ->
 * SW-verify and SW-sign -> HW-verify wherever mbedTLS 4.x/TF-PSA-Crypto
 * actually has software support for that curve (mbedtls_ecp_group_id only
 * defines SECP256R1/384R1/521R1, SECP256K1, BP256R1/384R1/512R1 - P-192/224,
 * KO-192/224, and every binary/GF(2^m) curve were dropped upstream and have
 * no software counterpart at all in this tree, so those curves only get the
 * HW-only round-trip check).
 *
 * A second section repeats the round-trip through the standard PSA API
 * (psa_sign_hash/psa_verify_hash, dispatched to the nvt_ecc driver) for the
 * curves enabled in crypto_config.h, timing it against the direct call.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"
#include "nvt_crpt.h"
#include "nvt_ecc_driver_entrypoints.h"
#include "nvt_psa_ecc.h"
#include "mbedtls/private/bignum.h"
#include "mbedtls/private/ecdsa.h"
#include "mbedtls/private/ecp.h"
#include "mbedtls/private/sha256.h"
#include "psa/crypto.h"

typedef struct
{
    const char *name;
    psa_ecc_family_t family;
    size_t bits;
    mbedtls_ecp_group_id mbedtls_id; /* MBEDTLS_ECP_DP_NONE => no SW support in this tree */
} curve_entry_t;

/* HW-supported curves; KO-224 is omitted because StdDriver's curve table has a truncated order for it. */
static const curve_entry_t s_curves[] =
{
    {"P-192",  PSA_ECC_FAMILY_SECP_R1,       192, MBEDTLS_ECP_DP_NONE},
    {"P-224",  PSA_ECC_FAMILY_SECP_R1,       224, MBEDTLS_ECP_DP_NONE},
    {"P-256",  PSA_ECC_FAMILY_SECP_R1,       256, MBEDTLS_ECP_DP_SECP256R1},
    {"P-384",  PSA_ECC_FAMILY_SECP_R1,       384, MBEDTLS_ECP_DP_SECP384R1},
    {"P-521",  PSA_ECC_FAMILY_SECP_R1,       521, MBEDTLS_ECP_DP_SECP521R1},
    {"KO-192", PSA_ECC_FAMILY_SECP_K1,       192, MBEDTLS_ECP_DP_NONE},
    {"KO-256", PSA_ECC_FAMILY_SECP_K1,       256, MBEDTLS_ECP_DP_SECP256K1},
    {"B-163",  PSA_ECC_FAMILY_SECT_R1,       163, MBEDTLS_ECP_DP_NONE},
    {"B-233",  PSA_ECC_FAMILY_SECT_R1,       233, MBEDTLS_ECP_DP_NONE},
    {"B-283",  PSA_ECC_FAMILY_SECT_R1,       283, MBEDTLS_ECP_DP_NONE},
    {"B-409",  PSA_ECC_FAMILY_SECT_R1,       409, MBEDTLS_ECP_DP_NONE},
    {"B-571",  PSA_ECC_FAMILY_SECT_R1,       571, MBEDTLS_ECP_DP_NONE},
    {"K-163",  PSA_ECC_FAMILY_SECT_K1,       163, MBEDTLS_ECP_DP_NONE},
    {"K-233",  PSA_ECC_FAMILY_SECT_K1,       233, MBEDTLS_ECP_DP_NONE},
    {"K-283",  PSA_ECC_FAMILY_SECT_K1,       283, MBEDTLS_ECP_DP_NONE},
    {"K-409",  PSA_ECC_FAMILY_SECT_K1,       409, MBEDTLS_ECP_DP_NONE},
    {"K-571",  PSA_ECC_FAMILY_SECT_K1,       571, MBEDTLS_ECP_DP_NONE},
    {"BP-256", PSA_ECC_FAMILY_BRAINPOOL_P_R1, 256, MBEDTLS_ECP_DP_BP256R1},
    {"BP-384", PSA_ECC_FAMILY_BRAINPOOL_P_R1, 384, MBEDTLS_ECP_DP_BP384R1},
    {"BP-512", PSA_ECC_FAMILY_BRAINPOOL_P_R1, 512, MBEDTLS_ECP_DP_BP512R1},
};

static uint8_t s_hash[32];
static int s_pass_count;
static int s_fail_count;
static volatile uint32_t s_systick_wraps;

psa_status_t mbedtls_psa_external_get_random(
    mbedtls_psa_external_random_context_t *context,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    (void) context;
    return nvt_crpt_get_random(output, output_size, output_length);
}

void CRPT_IRQHandler(void)
{
    nvt_crpt_irq_handler();
}

void SysTick_Handler(void)
{
    s_systick_wraps++;
}

/* Free-running cycle counter: SysTick wraps every 2^24 core cycles. */
static uint64_t cycles_now(void)
{
    uint32_t wraps, val;

    do
    {
        wraps = s_systick_wraps;
        val = SysTick->VAL;
    } while (wraps != s_systick_wraps);

    return (uint64_t) wraps * 0x01000000U + (0x00FFFFFFU - val);
}

static uint32_t elapsed_us(uint64_t start)
{
    return (uint32_t)((cycles_now() - start) / (SystemCoreClock / 1000000U));
}

/* Acceptance budget: the PSA path may cost at most 20% more than the direct call. */
static int within_dispatch_budget(uint32_t psa_us, uint32_t direct_us)
{
    return (uint64_t) psa_us * 10U <= (uint64_t) direct_us * 12U;
}

#define SIGN_TIMING_RUNS 5

static int myrand(void *context, unsigned char *output, size_t length)
{
    size_t output_length;

    (void) context;
    return nvt_crpt_get_random(output, length, &output_length) == PSA_SUCCESS ? 0 : -1;
}

static void report(const char *name, int passed)
{
    printf("  [%s] %s\n", passed ? "PASS" : "FAIL", name);

    if (passed)
    {
        s_pass_count++;
    }
    else
    {
        s_fail_count++;
    }
}

static void SYS_Init(void)
{
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_SetCoreClock(96000000);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(CRPT_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    SystemCoreClockUpdate();
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) |
                    UART0_RXD_PA6 | UART0_TXD_PA7;
}

static void UART0_Init(void)
{
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
}

/* Software ECDSA verify against an mbedTLS-supported curve; group must
 * already be loaded by the caller. */
static int software_verify(mbedtls_ecp_group *group,
                           const uint8_t *qx, size_t qx_len,
                           const uint8_t *qy, size_t qy_len,
                           const uint8_t *sig, size_t n_bytes)
{
    int result;
    mbedtls_ecp_point public_key;
    mbedtls_mpi r, s;

    mbedtls_ecp_point_init(&public_key);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    result = mbedtls_mpi_read_binary(&public_key.MBEDTLS_PRIVATE(X), qx, qx_len);

    if (result == 0) result = mbedtls_mpi_read_binary(&public_key.MBEDTLS_PRIVATE(Y), qy, qy_len);

    if (result == 0) result = mbedtls_mpi_lset(&public_key.MBEDTLS_PRIVATE(Z), 1);

    if (result == 0) result = mbedtls_mpi_read_binary(&r, sig, n_bytes);

    if (result == 0) result = mbedtls_mpi_read_binary(&s, sig + n_bytes, n_bytes);

    if (result == 0) result = mbedtls_ecdsa_verify(group, s_hash, sizeof(s_hash), &public_key, &r, &s);

    mbedtls_mpi_free(&s);
    mbedtls_mpi_free(&r);
    mbedtls_ecp_point_free(&public_key);
    return result;
}

/* Software ECDSA sign against an mbedTLS-supported curve; group must
 * already be loaded by the caller. */
static int software_sign(mbedtls_ecp_group *group,
                         const uint8_t *priv, size_t priv_len,
                         uint8_t *sig, size_t n_bytes)
{
    int result;
    mbedtls_mpi private_key, r, s;

    mbedtls_mpi_init(&private_key);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    result = mbedtls_mpi_read_binary(&private_key, priv, priv_len);

    if (result == 0) result = mbedtls_ecdsa_sign(group, &r, &s, &private_key, s_hash, sizeof(s_hash), myrand, NULL);

    if (result == 0) result = mbedtls_mpi_write_binary(&r, sig, n_bytes);

    if (result == 0) result = mbedtls_mpi_write_binary(&s, sig + n_bytes, n_bytes);

    mbedtls_mpi_free(&s);
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&private_key);
    return result;
}

static void test_curve(const curve_entry_t *curve)
{
    E_ECC_CURVE hw_curve = CURVE_UNDEF;
    size_t n_bytes;
    uint8_t priv[NVT_ECC_MAX_BYTES];
    uint8_t qx[NVT_ECC_MAX_BYTES], qy[NVT_ECC_MAX_BYTES];
    uint8_t pubkey[1 + 2 * NVT_ECC_MAX_BYTES];
    uint8_t sig[2 * NVT_ECC_MAX_BYTES];
    size_t priv_len, qx_len, qy_len, sig_len;
    char label[64];
    psa_key_attributes_t private_attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_attributes_t public_attributes = PSA_KEY_ATTRIBUTES_INIT;

    /* Map the PSA (family, bits) pair back to the StdDriver curve id the
     * same way nvt_ecc_driver_entrypoints.c's curve_from_attributes() does,
     * just so we can call the native keygen directly (not exposed at the
     * PSA-shaped layer yet). */
    psa_set_key_type(&private_attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(curve->family));
    psa_set_key_bits(&private_attributes, curve->bits);
    psa_set_key_type(&public_attributes, PSA_KEY_TYPE_ECC_PUBLIC_KEY(curve->family));
    psa_set_key_bits(&public_attributes, curve->bits);

    if (curve->family == PSA_ECC_FAMILY_SECP_R1)
    {
        switch (curve->bits)
        {
            case 192:
                hw_curve = CURVE_P_192;
                break;

            case 224:
                hw_curve = CURVE_P_224;
                break;

            case 256:
                hw_curve = CURVE_P_256;
                break;

            case 384:
                hw_curve = CURVE_P_384;
                break;

            case 521:
                hw_curve = CURVE_P_521;
                break;

            default:
                break;
        }
    }
    else if (curve->family == PSA_ECC_FAMILY_SECP_K1)
    {
        switch (curve->bits)
        {
            case 192:
                hw_curve = CURVE_KO_192;
                break;

            case 224:
                hw_curve = CURVE_KO_224;
                break;

            case 256:
                hw_curve = CURVE_KO_256;
                break;

            default:
                break;
        }
    }
    else if (curve->family == PSA_ECC_FAMILY_SECT_R1)
    {
        switch (curve->bits)
        {
            case 163:
                hw_curve = CURVE_B_163;
                break;

            case 233:
                hw_curve = CURVE_B_233;
                break;

            case 283:
                hw_curve = CURVE_B_283;
                break;

            case 409:
                hw_curve = CURVE_B_409;
                break;

            case 571:
                hw_curve = CURVE_B_571;
                break;

            default:
                break;
        }
    }
    else if (curve->family == PSA_ECC_FAMILY_SECT_K1)
    {
        switch (curve->bits)
        {
            case 163:
                hw_curve = CURVE_K_163;
                break;

            case 233:
                hw_curve = CURVE_K_233;
                break;

            case 283:
                hw_curve = CURVE_K_283;
                break;

            case 409:
                hw_curve = CURVE_K_409;
                break;

            case 571:
                hw_curve = CURVE_K_571;
                break;

            default:
                break;
        }
    }
    else if (curve->family == PSA_ECC_FAMILY_BRAINPOOL_P_R1)
    {
        switch (curve->bits)
        {
            case 256:
                hw_curve = CURVE_BP_256;
                break;

            case 384:
                hw_curve = CURVE_BP_384;
                break;

            case 512:
                hw_curve = CURVE_BP_512;
                break;

            default:
                break;
        }
    }

    n_bytes = nvt_ecc_curve_bytes(hw_curve);
    snprintf(label, sizeof(label), "ECDSA %s keygen+sign+verify (HW)", curve->name);

    if (n_bytes == 0 ||
            nvt_ecc_generate_keypair(hw_curve, priv, sizeof(priv), &priv_len,
                                     qx, sizeof(qx), &qx_len, qy, sizeof(qy), &qy_len) != PSA_SUCCESS)
    {
        report(label, 0);
        return;
    }

    pubkey[0] = 0x04;
    memcpy(pubkey + 1, qx, qx_len);
    memcpy(pubkey + 1 + qx_len, qy, qy_len);

    if (nvt_ecc_transparent_sign_hash(&private_attributes, priv, priv_len,
                                      PSA_ALG_ECDSA(PSA_ALG_SHA_256), s_hash, sizeof(s_hash),
                                      sig, sizeof(sig), &sig_len) != PSA_SUCCESS || sig_len != 2 * n_bytes)
    {
        report(label, 0);
        return;
    }

    report(label,
           nvt_ecc_transparent_verify_hash(&public_attributes, pubkey, 1 + 2 * n_bytes,
                                           PSA_ALG_ECDSA(PSA_ALG_SHA_256), s_hash, sizeof(s_hash),
                                           sig, sig_len) == PSA_SUCCESS);

    if (curve->mbedtls_id == MBEDTLS_ECP_DP_NONE)
    {
        return; /* No software implementation of this curve in mbedTLS 4.x - HW round-trip above is all we can check. */
    }

    {
        mbedtls_ecp_group group;
        uint8_t sw_sig[2 * NVT_ECC_MAX_BYTES];

        mbedtls_ecp_group_init(&group);

        if (mbedtls_ecp_group_load(&group, curve->mbedtls_id) != 0)
        {
            snprintf(label, sizeof(label), "ECDSA %s HW-sign -> SW-verify", curve->name);
            report(label, 0);
            mbedtls_ecp_group_free(&group);
            return;
        }

        snprintf(label, sizeof(label), "ECDSA %s HW-sign -> SW-verify", curve->name);
        report(label, software_verify(&group, qx, qx_len, qy, qy_len, sig, n_bytes) == 0);

        snprintf(label, sizeof(label), "ECDSA %s SW-sign -> HW-verify", curve->name);

        if (software_sign(&group, priv, priv_len, sw_sig, n_bytes) == 0)
        {
            report(label,
                   nvt_ecc_transparent_verify_hash(&public_attributes, pubkey, 1 + 2 * n_bytes,
                                                   PSA_ALG_ECDSA(PSA_ALG_SHA_256), s_hash, sizeof(s_hash),
                                                   sw_sig, 2 * n_bytes) == PSA_SUCCESS);
        }
        else
        {
            report(label, 0);
        }

        mbedtls_ecp_group_free(&group);
    }
}

static void test_rejects_ecdh(void)
{
    uint8_t priv[NVT_ECC_MAX_BYTES], qx[NVT_ECC_MAX_BYTES], qy[NVT_ECC_MAX_BYTES];
    uint8_t sig[2 * NVT_ECC_MAX_BYTES];
    size_t priv_len, qx_len, qy_len, sig_len;
    psa_key_attributes_t private_attributes = PSA_KEY_ATTRIBUTES_INIT;

    psa_set_key_type(&private_attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&private_attributes, 256);

    if (nvt_ecc_generate_keypair(CURVE_P_256, priv, sizeof(priv), &priv_len,
                                 qx, sizeof(qx), &qx_len, qy, sizeof(qy), &qy_len) != PSA_SUCCESS)
    {
        report("ECDSA rejects ECDH algorithm", 0);
        return;
    }

    report("ECDSA rejects ECDH algorithm",
           nvt_ecc_transparent_sign_hash(&private_attributes, priv, priv_len,
                                         PSA_ALG_ECDH, s_hash, sizeof(s_hash),
                                         sig, sizeof(sig), &sig_len) == PSA_ERROR_NOT_SUPPORTED);
}

/* Same round-trip through psa_* (dispatched to nvt_ecc), timed against the direct call. */
static void test_curve_psa(const curve_entry_t *curve)
{
    const psa_algorithm_t alg = PSA_ALG_ECDSA(PSA_ALG_SHA_256);
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_attributes_t public_attributes = PSA_KEY_ATTRIBUTES_INIT;
    mbedtls_svc_key_id_t key = MBEDTLS_SVC_KEY_ID_INIT;
    mbedtls_svc_key_id_t public_key = MBEDTLS_SVC_KEY_ID_INIT;
    uint8_t priv[NVT_ECC_MAX_BYTES];
    uint8_t pubkey[1 + 2 * NVT_ECC_MAX_BYTES];
    uint8_t sig[2 * NVT_ECC_MAX_BYTES], direct_sig[2 * NVT_ECC_MAX_BYTES];
    size_t priv_len = 0, pub_len = 0, sig_len = 0, direct_sig_len = 0, n_bytes;
    uint32_t psa_sign_us = UINT32_MAX, direct_sign_us = UINT32_MAX;
    uint32_t psa_verify_us, direct_verify_us, pair_verify_us, t_us;
    uint64_t start;
    mbedtls_ecp_group group;
    char label[80];
    int ok, run;

    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(curve->family));
    psa_set_key_bits(&attributes, curve->bits);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH | PSA_KEY_USAGE_VERIFY_HASH |
                            PSA_KEY_USAGE_EXPORT);
    psa_set_key_algorithm(&attributes, alg);

    psa_set_key_type(&public_attributes, PSA_KEY_TYPE_ECC_PUBLIC_KEY(curve->family));
    psa_set_key_bits(&public_attributes, curve->bits);
    psa_set_key_usage_flags(&public_attributes, PSA_KEY_USAGE_VERIFY_HASH);
    psa_set_key_algorithm(&public_attributes, alg);

    snprintf(label, sizeof(label), "PSA %s generate_key + export_public_key", curve->name);
    ok = psa_generate_key(&attributes, &key) == PSA_SUCCESS &&
         psa_export_key(key, priv, sizeof(priv), &priv_len) == PSA_SUCCESS &&
         psa_export_public_key(key, pubkey, sizeof(pubkey), &pub_len) == PSA_SUCCESS &&
         psa_import_key(&public_attributes, pubkey, pub_len, &public_key) == PSA_SUCCESS;
    report(label, ok);

    if (!ok)
    {
        goto cleanup;
    }

    n_bytes = priv_len;

    /* Best of several runs: k is rejection-sampled from the TRNG, so one sign may pay extra draws. */
    ok = 1;

    for (run = 0; run < SIGN_TIMING_RUNS && ok; run++)
    {
        start = cycles_now();
        ok = psa_sign_hash(key, alg, s_hash, sizeof(s_hash), sig, sizeof(sig), &sig_len) == PSA_SUCCESS;
        t_us = elapsed_us(start);
        psa_sign_us = (t_us < psa_sign_us) ? t_us : psa_sign_us;
    }

    mbedtls_ecp_group_init(&group);
    snprintf(label, sizeof(label), "PSA %s sign_hash -> SW verify", curve->name);
    report(label, ok && mbedtls_ecp_group_load(&group, curve->mbedtls_id) == 0 &&
           software_verify(&group, pubkey + 1, n_bytes, pubkey + 1 + n_bytes, n_bytes,
                           sig, n_bytes) == 0);
    mbedtls_ecp_group_free(&group);

    start = cycles_now();
    ok = psa_verify_hash(public_key, alg, s_hash, sizeof(s_hash), sig, sig_len) == PSA_SUCCESS;
    psa_verify_us = elapsed_us(start);
    snprintf(label, sizeof(label), "PSA %s verify_hash (public key)", curve->name);
    report(label, ok);

    start = cycles_now();
    ok = psa_verify_hash(key, alg, s_hash, sizeof(s_hash), sig, sig_len) == PSA_SUCCESS;
    pair_verify_us = elapsed_us(start);
    snprintf(label, sizeof(label), "PSA %s verify_hash (key pair)", curve->name);
    report(label, ok);

    sig[0] ^= 0x01;
    snprintf(label, sizeof(label), "PSA %s verify_hash rejects tampered signature", curve->name);
    report(label, psa_verify_hash(public_key, alg, s_hash, sizeof(s_hash), sig, sig_len) ==
           PSA_ERROR_INVALID_SIGNATURE);
    sig[0] ^= 0x01;

    for (run = 0; run < SIGN_TIMING_RUNS && ok; run++)
    {
        start = cycles_now();
        ok = nvt_ecc_transparent_sign_hash(&attributes, priv, priv_len, alg, s_hash, sizeof(s_hash),
                                           direct_sig, sizeof(direct_sig), &direct_sig_len) == PSA_SUCCESS;
        t_us = elapsed_us(start);
        direct_sign_us = (t_us < direct_sign_us) ? t_us : direct_sign_us;
    }

    start = cycles_now();
    ok = ok && nvt_ecc_transparent_verify_hash(&public_attributes, pubkey, pub_len, alg,
                                               s_hash, sizeof(s_hash), sig, sig_len) == PSA_SUCCESS;
    direct_verify_us = elapsed_us(start);
    snprintf(label, sizeof(label), "Direct %s sign_hash + verify_hash", curve->name);
    report(label, ok);

    printf("    sign:   PSA %lu us, direct %lu us (best of %d)\n",
           (unsigned long) psa_sign_us, (unsigned long) direct_sign_us, SIGN_TIMING_RUNS);
    printf("    verify: PSA %lu us, direct %lu us, PSA with key pair %lu us\n",
           (unsigned long) psa_verify_us, (unsigned long) direct_verify_us,
           (unsigned long) pair_verify_us);
    snprintf(label, sizeof(label), "PSA %s sign time <= 1.2 x direct", curve->name);
    report(label, within_dispatch_budget(psa_sign_us, direct_sign_us));
    snprintf(label, sizeof(label), "PSA %s verify time <= 1.2 x direct", curve->name);
    report(label, within_dispatch_budget(psa_verify_us, direct_verify_us));

cleanup:
    psa_destroy_key(public_key);
    psa_destroy_key(key);
}

/* HW only does randomized ECDSA, so two identical signatures prove the software fallback ran. */
static void test_deterministic_fallback(void)
{
    const psa_algorithm_t alg = PSA_ALG_DETERMINISTIC_ECDSA(PSA_ALG_SHA_256);
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    mbedtls_svc_key_id_t key = MBEDTLS_SVC_KEY_ID_INIT;
    uint8_t sig1[64], sig2[64];
    size_t sig1_len = 0, sig2_len = 0;
    uint32_t sign_us;
    uint64_t start;
    int ok;

    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attributes, 256);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH | PSA_KEY_USAGE_VERIFY_HASH);
    psa_set_key_algorithm(&attributes, alg);

    ok = psa_generate_key(&attributes, &key) == PSA_SUCCESS;
    start = cycles_now();
    ok = ok && psa_sign_hash(key, alg, s_hash, sizeof(s_hash), sig1, sizeof(sig1), &sig1_len) == PSA_SUCCESS;
    sign_us = elapsed_us(start);
    ok = ok && psa_sign_hash(key, alg, s_hash, sizeof(s_hash), sig2, sizeof(sig2), &sig2_len) == PSA_SUCCESS;
    ok = ok && sig1_len == sig2_len && memcmp(sig1, sig2, sig1_len) == 0;
    ok = ok && psa_verify_hash(key, alg, s_hash, sizeof(s_hash), sig1, sig1_len) == PSA_SUCCESS;

    printf("    deterministic sign (software): %lu us\n", (unsigned long) sign_us);
    report("PSA P-256 deterministic ECDSA (software fallback): repeatable + verifies", ok);
    psa_destroy_key(key);
}

int main(void)
{
    size_t i;
    static const char s_message[] = "nvt ECDSA HW/SW cross-check";

    SYS_UnlockReg();
    SYS_Init();
    UART0_Init();

    printf("+--------------------------------------------------+\n");
    printf("|  mbedTLS PSA SignVerify ECC (all curves, HW/SW)  |\n");
    printf("+--------------------------------------------------+\n");

    if (psa_crypto_init() != PSA_SUCCESS)
    {
        printf("psa_crypto_init failed\n");

        while (1)
        {
        }
    }

    SysTick_Config(0x01000000U);

    mbedtls_sha256((const unsigned char *) s_message, strlen(s_message), s_hash, 0);

    printf("\n--- Direct call (nvt_ecc_transparent_*) ---\n");

    for (i = 0; i < sizeof(s_curves) / sizeof(s_curves[0]); i++)
    {
        test_curve(&s_curves[i]);
    }

    test_rejects_ecdh();

    printf("\n--- PSA dispatch (psa_sign_hash / psa_verify_hash) ---\n");

    for (i = 0; i < sizeof(s_curves) / sizeof(s_curves[0]); i++)
    {
        if (s_curves[i].mbedtls_id != MBEDTLS_ECP_DP_NONE)
        {
            test_curve_psa(&s_curves[i]);
        }
    }

    test_deterministic_fallback();

    mbedtls_psa_crypto_free();

    printf("\n%d passed, %d failed\n", s_pass_count, s_fail_count);
    printf(s_fail_count == 0 ? "RESULT: ALL TESTS PASSED\n" : "RESULT: TESTS FAILED\n");

    while (1)
    {
    }
}
