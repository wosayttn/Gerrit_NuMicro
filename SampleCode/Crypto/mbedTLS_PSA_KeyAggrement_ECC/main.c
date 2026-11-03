/*
 * ECDH HW/SW interoperability verification across every curve the M2354
 * ECC accelerator supports (except Curve25519 and SM2 - see nvt_psa_ecc.h).
 *
 * Alice and Bob keypairs are generated FRESH on every run via
 * nvt_ecc_generate_keypair() (HW TRNG-seeded) instead of hand-transcribed
 * fixed vectors - this removes the class of bug found earlier in this
 * sample (a mistyped curve-point Y coordinate in a hardcoded test vector).
 * Correctness is verified by requiring HW_ECDH(dA, QB) == HW_ECDH(dB, QA)
 * (always checkable, any curve) and, wherever mbedTLS 4.x/TF-PSA-Crypto has
 * software support for that curve (mbedtls_ecp_group_id only defines
 * SECP256R1/384R1/521R1, SECP256K1, BP256R1/384R1/512R1), also cross-checking
 * against software mbedtls_ecp_mul().
 *
 * A second section repeats the agreement through psa_raw_key_agreement()
 * (dispatched to the nvt_ecc driver) for the curves enabled in crypto_config.h,
 * timing it against the direct call, and checks X25519 via software fallback.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"
#include "nvt_crpt.h"
#include "nvt_ecc_driver_entrypoints.h"
#include "nvt_psa_ecc.h"
#include "mbedtls/private/bignum.h"
#include "mbedtls/private/ecp.h"
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

static void print_hex(const char *name, const uint8_t *value, size_t length)
{
    size_t index;

    printf("    %s: ", name);

    for (index = 0; index < length; index++)
    {
        printf("%02X", value[index]);
    }

    printf("\n");
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

/* Software ECDH: shared.X = priv * peer, against an mbedTLS-supported
 * curve; group must already be loaded by the caller. */
static int software_ecdh(mbedtls_ecp_group *group,
                         const uint8_t *priv, size_t priv_len,
                         const uint8_t *peer_qx, size_t peer_qx_len,
                         const uint8_t *peer_qy, size_t peer_qy_len,
                         uint8_t *shared_secret, size_t n_bytes)
{
    int result;
    mbedtls_ecp_point peer, shared;
    mbedtls_mpi private_scalar;

    mbedtls_ecp_point_init(&peer);
    mbedtls_ecp_point_init(&shared);
    mbedtls_mpi_init(&private_scalar);

    result = mbedtls_mpi_read_binary(&private_scalar, priv, priv_len);

    if (result == 0) result = mbedtls_mpi_read_binary(&peer.MBEDTLS_PRIVATE(X), peer_qx, peer_qx_len);

    if (result == 0) result = mbedtls_mpi_read_binary(&peer.MBEDTLS_PRIVATE(Y), peer_qy, peer_qy_len);

    if (result == 0) result = mbedtls_mpi_lset(&peer.MBEDTLS_PRIVATE(Z), 1);

    if (result == 0) result = mbedtls_ecp_mul(group, &shared, &private_scalar, &peer, myrand, NULL);

    if (result == 0) result = mbedtls_mpi_write_binary(&shared.MBEDTLS_PRIVATE(X), shared_secret, n_bytes);

    mbedtls_mpi_free(&private_scalar);
    mbedtls_ecp_point_free(&shared);
    mbedtls_ecp_point_free(&peer);
    return result;
}

static E_ECC_CURVE hw_curve_for(const curve_entry_t *curve)
{
    if (curve->family == PSA_ECC_FAMILY_SECP_R1)
    {
        switch (curve->bits)
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
    else if (curve->family == PSA_ECC_FAMILY_SECP_K1)
    {
        switch (curve->bits)
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
    else if (curve->family == PSA_ECC_FAMILY_SECT_R1)
    {
        switch (curve->bits)
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
    else if (curve->family == PSA_ECC_FAMILY_SECT_K1)
    {
        switch (curve->bits)
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
    else if (curve->family == PSA_ECC_FAMILY_BRAINPOOL_P_R1)
    {
        switch (curve->bits)
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

static void test_curve(const curve_entry_t *curve)
{
    E_ECC_CURVE hw_curve = hw_curve_for(curve);
    size_t n_bytes;
    uint8_t alice_priv[NVT_ECC_MAX_BYTES], alice_qx[NVT_ECC_MAX_BYTES], alice_qy[NVT_ECC_MAX_BYTES];
    uint8_t bob_priv[NVT_ECC_MAX_BYTES], bob_qx[NVT_ECC_MAX_BYTES], bob_qy[NVT_ECC_MAX_BYTES];
    uint8_t hw_alice[NVT_ECC_MAX_BYTES] = {0}, hw_bob[NVT_ECC_MAX_BYTES] = {0};
    size_t alice_priv_len, alice_qx_len, alice_qy_len;
    size_t bob_priv_len, bob_qx_len, bob_qy_len;
    size_t hw_alice_len = 0, hw_bob_len = 0;
    psa_status_t hw_alice_status, hw_bob_status;
    int hw_match;
    char label[64];

    n_bytes = nvt_ecc_curve_bytes(hw_curve);
    snprintf(label, sizeof(label), "ECDH %s Alice/Bob HW keygen", curve->name);

    if (n_bytes == 0 ||
            nvt_ecc_generate_keypair(hw_curve, alice_priv, sizeof(alice_priv), &alice_priv_len,
                                     alice_qx, sizeof(alice_qx), &alice_qx_len,
                                     alice_qy, sizeof(alice_qy), &alice_qy_len) != PSA_SUCCESS)
    {
        report(label, 0);
        return;
    }

    if (nvt_ecc_generate_keypair(hw_curve, bob_priv, sizeof(bob_priv), &bob_priv_len,
                                 bob_qx, sizeof(bob_qx), &bob_qx_len,
                                 bob_qy, sizeof(bob_qy), &bob_qy_len) != PSA_SUCCESS)
    {
        report(label, 0);
        return;
    }

    report(label, 1);

    /* Native API: the transparent entry point declines HW-only curves it cannot validate. */
    hw_alice_status = nvt_ecdh_key_agreement(hw_curve, alice_priv, alice_priv_len,
                                             bob_qx, n_bytes, bob_qy, n_bytes,
                                             hw_alice, sizeof(hw_alice), &hw_alice_len);
    hw_bob_status = nvt_ecdh_key_agreement(hw_curve, bob_priv, bob_priv_len,
                                           alice_qx, n_bytes, alice_qy, n_bytes,
                                           hw_bob, sizeof(hw_bob), &hw_bob_len);

    hw_match = hw_alice_status == PSA_SUCCESS && hw_bob_status == PSA_SUCCESS &&
               hw_alice_len == n_bytes && hw_bob_len == n_bytes &&
               memcmp(hw_alice, hw_bob, n_bytes) == 0;

    snprintf(label, sizeof(label), "ECDH %s Alice-HW == Bob-HW", curve->name);
    report(label, hw_match);

    if (!hw_match)
    {
        printf("    Alice status=%ld, Bob status=%ld, Alice length=%u, Bob length=%u\n",
               (long) hw_alice_status, (long) hw_bob_status,
               (unsigned) hw_alice_len, (unsigned) hw_bob_len);
        print_hex("Alice secret", hw_alice, n_bytes);
        print_hex("Bob secret", hw_bob, n_bytes);
    }

    if (curve->mbedtls_id == MBEDTLS_ECP_DP_NONE)
    {
        return; /* No software implementation of this curve in mbedTLS 4.x - HW-only cross-check above is all we can check. */
    }

    {
        mbedtls_ecp_group group;
        uint8_t sw_alice[NVT_ECC_MAX_BYTES] = {0};

        mbedtls_ecp_group_init(&group);
        snprintf(label, sizeof(label), "ECDH %s Alice HW == SW", curve->name);

        if (mbedtls_ecp_group_load(&group, curve->mbedtls_id) != 0 ||
                software_ecdh(&group, alice_priv, alice_priv_len, bob_qx, n_bytes, bob_qy, n_bytes,
                              sw_alice, n_bytes) != 0)
        {
            report(label, 0);
        }
        else
        {
            report(label, hw_alice_status == PSA_SUCCESS && memcmp(hw_alice, sw_alice, n_bytes) == 0);
        }

        mbedtls_ecp_group_free(&group);
    }
}

static void test_rejects_ecdsa(void)
{
    uint8_t priv[NVT_ECC_MAX_BYTES], qx[NVT_ECC_MAX_BYTES], qy[NVT_ECC_MAX_BYTES];
    uint8_t peer_pub[1 + 2 * NVT_ECC_MAX_BYTES];
    uint8_t shared[NVT_ECC_MAX_BYTES];
    size_t priv_len, qx_len, qy_len, shared_len;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;

    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attributes, 256);

    if (nvt_ecc_generate_keypair(CURVE_P_256, priv, sizeof(priv), &priv_len,
                                 qx, sizeof(qx), &qx_len, qy, sizeof(qy), &qy_len) != PSA_SUCCESS)
    {
        report("ECDH rejects ECDSA algorithm", 0);
        return;
    }

    peer_pub[0] = 0x04;
    memcpy(peer_pub + 1, qx, qx_len);
    memcpy(peer_pub + 1 + qx_len, qy, qy_len);

    report("ECDH rejects ECDSA algorithm",
           nvt_ecc_transparent_key_agreement(&attributes, priv, priv_len,
                                             PSA_ALG_ECDSA(PSA_ALG_SHA_256),
                                             peer_pub, 1 + qx_len + qy_len,
                                             shared, sizeof(shared), &shared_len) == PSA_ERROR_NOT_SUPPORTED);
}

/* Same agreement through psa_raw_key_agreement (dispatched to nvt_ecc), timed against the direct call. */
static void test_curve_psa(const curve_entry_t *curve)
{
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    mbedtls_svc_key_id_t alice = MBEDTLS_SVC_KEY_ID_INIT, bob = MBEDTLS_SVC_KEY_ID_INIT;
    uint8_t alice_priv[NVT_ECC_MAX_BYTES];
    uint8_t alice_pub[1 + 2 * NVT_ECC_MAX_BYTES], bob_pub[1 + 2 * NVT_ECC_MAX_BYTES];
    uint8_t bad_pub[1 + 2 * NVT_ECC_MAX_BYTES];
    uint8_t z_alice[NVT_ECC_MAX_BYTES], z_bob[NVT_ECC_MAX_BYTES];
    uint8_t z_direct[NVT_ECC_MAX_BYTES], z_sw[NVT_ECC_MAX_BYTES] = {0};
    size_t alice_priv_len = 0, alice_pub_len = 0, bob_pub_len = 0;
    size_t z_alice_len = 0, z_bob_len = 0, z_direct_len = 0, n_bytes;
    uint32_t psa_us, direct_us;
    uint64_t start;
    mbedtls_ecp_group group;
    char label[80];
    int ok;

    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(curve->family));
    psa_set_key_bits(&attributes, curve->bits);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_EXPORT);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECDH);

    snprintf(label, sizeof(label), "PSA %s generate_key x2 + export_public_key", curve->name);
    ok = psa_generate_key(&attributes, &alice) == PSA_SUCCESS &&
         psa_generate_key(&attributes, &bob) == PSA_SUCCESS &&
         psa_export_key(alice, alice_priv, sizeof(alice_priv), &alice_priv_len) == PSA_SUCCESS &&
         psa_export_public_key(alice, alice_pub, sizeof(alice_pub), &alice_pub_len) == PSA_SUCCESS &&
         psa_export_public_key(bob, bob_pub, sizeof(bob_pub), &bob_pub_len) == PSA_SUCCESS;
    report(label, ok);

    if (!ok)
    {
        goto cleanup;
    }

    n_bytes = alice_priv_len;

    start = cycles_now();
    ok = psa_raw_key_agreement(PSA_ALG_ECDH, alice, bob_pub, bob_pub_len,
                               z_alice, sizeof(z_alice), &z_alice_len) == PSA_SUCCESS;
    psa_us = elapsed_us(start);
    ok = ok && psa_raw_key_agreement(PSA_ALG_ECDH, bob, alice_pub, alice_pub_len,
                                     z_bob, sizeof(z_bob), &z_bob_len) == PSA_SUCCESS;
    snprintf(label, sizeof(label), "PSA %s raw_key_agreement Alice == Bob", curve->name);
    report(label, ok && z_alice_len == n_bytes && z_bob_len == n_bytes &&
           memcmp(z_alice, z_bob, n_bytes) == 0);

    mbedtls_ecp_group_init(&group);
    snprintf(label, sizeof(label), "PSA %s Alice == SW", curve->name);
    report(label, mbedtls_ecp_group_load(&group, curve->mbedtls_id) == 0 &&
           software_ecdh(&group, alice_priv, alice_priv_len,
                         bob_pub + 1, n_bytes, bob_pub + 1 + n_bytes, n_bytes,
                         z_sw, n_bytes) == 0 &&
           memcmp(z_alice, z_sw, n_bytes) == 0);
    mbedtls_ecp_group_free(&group);

    memcpy(bad_pub, bob_pub, bob_pub_len);
    bad_pub[bob_pub_len - 1] ^= 0x01;
    snprintf(label, sizeof(label), "PSA %s rejects off-curve peer key", curve->name);
    report(label, psa_raw_key_agreement(PSA_ALG_ECDH, alice, bad_pub, bob_pub_len,
                                        z_direct, sizeof(z_direct), &z_direct_len) ==
           PSA_ERROR_INVALID_ARGUMENT);

    start = cycles_now();
    ok = nvt_ecc_transparent_key_agreement(&attributes, alice_priv, alice_priv_len, PSA_ALG_ECDH,
                                           bob_pub, bob_pub_len,
                                           z_direct, sizeof(z_direct), &z_direct_len) == PSA_SUCCESS;
    direct_us = elapsed_us(start);
    snprintf(label, sizeof(label), "Direct %s key_agreement == PSA", curve->name);
    report(label, ok && z_direct_len == n_bytes && memcmp(z_direct, z_alice, n_bytes) == 0);

    printf("    ECDH: PSA %lu us, direct %lu us\n", (unsigned long) psa_us, (unsigned long) direct_us);
    snprintf(label, sizeof(label), "PSA %s ECDH time <= 1.2 x direct", curve->name);
    report(label, within_dispatch_budget(psa_us, direct_us));

cleanup:
    psa_destroy_key(bob);
    psa_destroy_key(alice);
}

/* Curve25519 is not on the HW, so every step here runs in software. */
static void test_x25519_fallback(void)
{
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    mbedtls_svc_key_id_t alice = MBEDTLS_SVC_KEY_ID_INIT, bob = MBEDTLS_SVC_KEY_ID_INIT;
    uint8_t alice_pub[32], bob_pub[32], z_alice[32], z_bob[32];
    size_t alice_pub_len = 0, bob_pub_len = 0, z_alice_len = 0, z_bob_len = 0;
    uint32_t ecdh_us;
    uint64_t start;
    int ok;

    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
    psa_set_key_bits(&attributes, 255);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DERIVE);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECDH);

    ok = psa_generate_key(&attributes, &alice) == PSA_SUCCESS &&
         psa_generate_key(&attributes, &bob) == PSA_SUCCESS &&
         psa_export_public_key(alice, alice_pub, sizeof(alice_pub), &alice_pub_len) == PSA_SUCCESS &&
         psa_export_public_key(bob, bob_pub, sizeof(bob_pub), &bob_pub_len) == PSA_SUCCESS;
    start = cycles_now();
    ok = ok && psa_raw_key_agreement(PSA_ALG_ECDH, alice, bob_pub, bob_pub_len,
                                     z_alice, sizeof(z_alice), &z_alice_len) == PSA_SUCCESS;
    ecdh_us = elapsed_us(start);
    ok = ok && psa_raw_key_agreement(PSA_ALG_ECDH, bob, alice_pub, alice_pub_len,
                                     z_bob, sizeof(z_bob), &z_bob_len) == PSA_SUCCESS;
    ok = ok && z_alice_len == z_bob_len && memcmp(z_alice, z_bob, z_alice_len) == 0;

    printf("    X25519 ECDH (software): %lu us\n", (unsigned long) ecdh_us);
    report("PSA X25519 (software fallback): Alice == Bob", ok);
    psa_destroy_key(bob);
    psa_destroy_key(alice);
}

int main(void)
{
    size_t i;

    SYS_UnlockReg();
    SYS_Init();
    UART0_Init();

    printf("+--------------------------------------------------+\n");
    printf("| mbedTLS PSA KeyAggrement ECC (all curves, HW/SW) |\n");
    printf("+--------------------------------------------------+\n");

    if (psa_crypto_init() != PSA_SUCCESS)
    {
        printf("psa_crypto_init failed\n");

        while (1)
        {
        }
    }

    SysTick_Config(0x01000000U);

    printf("\n--- Direct call (nvt_ecdh_key_agreement) ---\n");

    for (i = 0; i < sizeof(s_curves) / sizeof(s_curves[0]); i++)
    {
        test_curve(&s_curves[i]);
    }

    test_rejects_ecdsa();

    printf("\n--- PSA dispatch (psa_raw_key_agreement) ---\n");

    for (i = 0; i < sizeof(s_curves) / sizeof(s_curves[0]); i++)
    {
        if (s_curves[i].mbedtls_id != MBEDTLS_ECP_DP_NONE)
        {
            test_curve_psa(&s_curves[i]);
        }
    }

    test_x25519_fallback();

    mbedtls_psa_crypto_free();

    printf("\n%d passed, %d failed\n", s_pass_count, s_fail_count);
    printf(s_fail_count == 0 ? "RESULT: ALL TESTS PASSED\n" : "RESULT: TESTS FAILED\n");

    while (1)
    {
    }
}
