/*
 * RSA OAEP encrypt/decrypt verification sample ("key agreement" = RSA used to
 * transport/wrap a symmetric key).
 *
 * Section 1 runs psa_asymmetric_encrypt/decrypt (dispatched to the nvt_rsa
 * driver: SHA-256 OAEP) for a 1024-bit key (SCAP) and a 2048-bit key
 * (CRT+SCAP), cross-checks every ciphertext against the software
 * mbedtls_rsa_rsaes_oaep_* implementation, and times the PSA path against the
 * direct nvt_rsa_* call. Section 2 checks that PKCS#1 v1.5 encryption (not
 * handled by the HW driver) still works through the software fallback.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"
#include "nvt_crpt.h"
#include "nvt_psa_rsa.h"
#include "mbedtls/private/bignum.h"
#include "mbedtls/private/rsa.h"
#include "psa/crypto.h"
#include "rsa_alt_helpers.h" /* mbedtls_rsa_deduce_crt() */
#include "rsa_internal.h"    /* mbedtls_rsa_write_key/pubkey() */

typedef struct
{
    const char *name;
    const char *n, *e, *d, *p, *q;
} test_key_t;

/* Fixed keys generated offline (.NET RSA.Create); every HW-dispatched mode is covered:
 * 1024-bit -> SCAP, 2048-bit -> CRT+SCAP. */
static const test_key_t s_keys[] =
{
    {
        "RSA-1024",
        "BDA3EA9B6E278B715B611A0922C2F1410620A50C84873F0EE9D92769A703B659"
        "461770C429B9EAA6C0D38BF9B95617DB6D365D5F9651189A869B047D3296D3C0"
        "D865D8A7377DDD9C426CAEA62DB73745942682801C0CB6E466B0C81136A2F0C4"
        "28C87347F0D6ECE85507D81B4DF6A9F558996BEED12BFCC1B88DF4F76556EF09",
        "010001",
        "55C3B70B7A42EA1E670C4021179680533EF92E8BBDB808BDAEE6A59B738C0B40"
        "CEA1353F91FB61F2EE4BFE06FF59BF4A5E56F2DD92F6B420D6756F0F4BFCC860"
        "ABB79481A27119356416D86AD33F69924206823207D3F4936A6655C2F29487CF"
        "61C4A36786F2BEA5BCE16D80F5E438AA1C50E6FE4BAA913E1E628B57FECFAB0D",
        "CA4FF6197CC7A088F7BB05C97836B2F176FC4CADA1210A4F3E05F04CF5AECDD1"
        "547A3E4FF4209A2AB594625813FCE5E53AFE52131CF891235D5561200E9E41C7",
        "EFF71521B1E0D4FBC0AD51BFDF3E6F4A810E91AEFB149C7F31CA5BDCD0AEAC17"
        "83F3EB8880BD382008055FEC9AAD4B9AD0258FF89DA439515DEF9EEF761848AF",
    },
    {
        "RSA-2048",
        "B4BB9E3DB360422426E35FFB861EA4F587CC91F7061940F4E98D57F58D87D38F"
        "087B43B12975FC64579FAE46811CB0906713C1E1CAB20D79B0062F3E0FC99B20"
        "9B34EE8194747CF7B8E38EBB041C3A15CD515BFB48405C38AF42D0D0C56BF64D"
        "A51EE3B815D7350F869E53D7C7E50D0E9D25E5703D9DC04D4F256A1B3E45D491"
        "F1CFB4CC1D422406C56155A10D8C34C8BF2309C5944DF2D1A544EEECD233B803"
        "C371B4B30348C4B1B0D4CA16A07D3984F234619D149025CB56C1CAC2A2A34E5B"
        "FEBC4B499F404D24BE2C2CB0A613A374EA3E8D0004A91E2570A930D009309A82"
        "148BCCF382E53F2C9BB5E0FDA30520F4AB488DDD74776245074D7A9604F638E5",
        "010001",
        "77FBA64DE7EB9B7D4CA3DE70A8D56745E96591F1A086DB6CBC302D98C2E0CD3E"
        "43AF317D47BD2055AD26411865569A91031D46B5934F294B03E4D7302862FB2D"
        "665C798A64F8008EA49BCD81B274DACAE68780BD0D764CC304F72291B6BEB9C1"
        "4A417B84165BCCCA05E0CE608E402DAEE9FCE298E50EF93F59AFC905550E1ED3"
        "B74CE694A5B55C0D09E61276F3C1FCC2D005DD9F66FB752E4CEE878C85318940"
        "5EA4FC9CDB5CD82727AF174F257644F527A36EA48C625375894E8C8A76746CCC"
        "F1110186617328E82A4881D34F510428B61DD9CE4063E4CD99BB382F95E5B9DA"
        "F36D3450BF2EA6246B79CA61E2CE6260AE800340EE4C4B41012CD649D90DD071",
        "D558176FFE2BE135C7703E360BBE6C75E1A51C1D57695EACD2364E71405F396C"
        "6FF650FA8F7B9859436EC79E096DFA626BBC42B444AA7905AB144DB38B055C26"
        "6657006BD5278897FC4DB0CE1D3F72BFD7830B9F4A17595C75549F319A549C8E"
        "5FF15021D6D85779086B9E352026D1F700161A9DA7AB43D244DCAA9E952B7B3B",
        "D8DE567BF0B59CBBA2282FD79CB62C46A53CB35FF49EEF5BD0BEE8700257D57F"
        "7C1ED9CA0EAE642021634910CACE82372FB05BADE2490B5D1EA064450ADDF7D9"
        "951CDB7375D443B1DACA25F4D6B50E2B8416FD69547B48EE2D56DD331EE06B0F"
        "7865D0D4AA019FA3D806837C3F75C14F6D6340DA71FD168856E4F110D5F69A5F",
    },
};

/* Software context, raw HW buffers and PSA DER encodings of the key under test. */
typedef struct
{
    mbedtls_rsa_context rsa;
    uint8_t n[NVT_RSA_MAX_BYTES];
    uint8_t e[8];
    uint8_t d[NVT_RSA_MAX_BYTES];
    uint8_t p[NVT_RSA_MAX_BYTES / 2];
    uint8_t q[NVT_RSA_MAX_BYTES / 2];
    size_t n_len, e_len, half;
    uint8_t der_pair_buf[1600];
    uint8_t der_pub_buf[600];
    const uint8_t *der_pair, *der_pub;
    size_t der_pair_len, der_pub_len;
} key_ctx_t;

static key_ctx_t s_key;

/* A symmetric-key-sized payload, as would be wrapped for key transport. */
static const uint8_t s_plaintext[] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};
static const uint8_t s_label[] = "nvt-oaep-label";

static int s_pass_count;
static int s_fail_count;
static volatile uint32_t s_systick_wraps;

#define TIMING_RUNS 3

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

/* Runs `expr` TIMING_RUNS times (stopping at the first failure), keeping the fastest time. */
#define TIME_BEST(best_us, ok, expr)                                \
    do                                                              \
    {                                                               \
        int run_;                                                   \
        (best_us) = UINT32_MAX;                                     \
        for (run_ = 0; run_ < TIMING_RUNS && (ok); run_++)          \
        {                                                           \
            uint64_t start_ = cycles_now();                         \
            uint32_t us_;                                           \
            (ok) = (expr);                                          \
            us_ = elapsed_us(start_);                               \
            if (us_ < (best_us))                                    \
            {                                                       \
                (best_us) = us_;                                    \
            }                                                       \
        }                                                           \
    } while (0)

static int myrand(void *ctx, unsigned char *out, size_t len)
{
    size_t output_length;

    (void) ctx;
    return nvt_crpt_get_random(out, len, &output_length) == PSA_SUCCESS ? 0 : -1;
}

static void report(const char *name, int ok)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);

    if (ok)
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

    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* RSA HW engine needs its SRAM banked in. */
    SYS_SetPSRAMPowerMode(SYS_SRAMPC1_RSA_Msk, SYS_SRAMPC1_SRAM_NORMAL);
}

static void UART0_Init(void)
{
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
}

/* Loads `key` into s_key: software context, raw HW buffers, and PSA DER key pair/public key. */
static int load_key(const test_key_t *key)
{
    mbedtls_rsa_context *rsa = &s_key.rsa;
    unsigned char *der;
    int len;

    mbedtls_rsa_init(rsa);

    if (mbedtls_mpi_read_string(&rsa->MBEDTLS_PRIVATE(N), 16, key->n) != 0 ||
            mbedtls_mpi_read_string(&rsa->MBEDTLS_PRIVATE(E), 16, key->e) != 0 ||
            mbedtls_mpi_read_string(&rsa->MBEDTLS_PRIVATE(D), 16, key->d) != 0 ||
            mbedtls_mpi_read_string(&rsa->MBEDTLS_PRIVATE(P), 16, key->p) != 0 ||
            mbedtls_mpi_read_string(&rsa->MBEDTLS_PRIVATE(Q), 16, key->q) != 0 ||
            mbedtls_rsa_deduce_crt(&rsa->MBEDTLS_PRIVATE(P), &rsa->MBEDTLS_PRIVATE(Q),
                                   &rsa->MBEDTLS_PRIVATE(D), &rsa->MBEDTLS_PRIVATE(DP),
                                   &rsa->MBEDTLS_PRIVATE(DQ), &rsa->MBEDTLS_PRIVATE(QP)) != 0)
    {
        return 0;
    }

    rsa->MBEDTLS_PRIVATE(len) = mbedtls_mpi_size(&rsa->MBEDTLS_PRIVATE(N));

    s_key.n_len = rsa->MBEDTLS_PRIVATE(len);
    s_key.half = s_key.n_len / 2;
    s_key.e_len = mbedtls_mpi_size(&rsa->MBEDTLS_PRIVATE(E));

    if (mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(N), s_key.n, s_key.n_len) != 0 ||
            mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(E), s_key.e, s_key.e_len) != 0 ||
            mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(D), s_key.d, s_key.n_len) != 0 ||
            mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(P), s_key.p, s_key.half) != 0 ||
            mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(Q), s_key.q, s_key.half) != 0)
    {
        return 0;
    }

    der = s_key.der_pair_buf + sizeof(s_key.der_pair_buf);
    len = mbedtls_rsa_write_key(rsa, s_key.der_pair_buf, &der);

    if (len <= 0)
    {
        return 0;
    }

    s_key.der_pair = der;
    s_key.der_pair_len = (size_t) len;

    der = s_key.der_pub_buf + sizeof(s_key.der_pub_buf);
    len = mbedtls_rsa_write_pubkey(rsa, s_key.der_pub_buf, &der);

    if (len <= 0)
    {
        return 0;
    }

    s_key.der_pub = der;
    s_key.der_pub_len = (size_t) len;
    return 1;
}

static psa_status_t import_key(int pair, psa_algorithm_t alg, psa_key_id_t *id)
{
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;

    psa_set_key_type(&attributes, pair ? PSA_KEY_TYPE_RSA_KEY_PAIR : PSA_KEY_TYPE_RSA_PUBLIC_KEY);
    psa_set_key_usage_flags(&attributes, pair ? (PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT)
                            : PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, alg);
    return psa_import_key(&attributes,
                          pair ? s_key.der_pair : s_key.der_pub,
                          pair ? s_key.der_pair_len : s_key.der_pub_len, id);
}

static int matches_plaintext(const uint8_t *pt, size_t pt_len)
{
    return pt_len == sizeof(s_plaintext) && memcmp(pt, s_plaintext, pt_len) == 0;
}

/* Software OAEP decrypt of `ct`, compared against s_plaintext. */
static int sw_decrypts(const uint8_t *ct, const uint8_t *label, size_t label_len)
{
    uint8_t pt[sizeof(s_plaintext)];
    size_t pt_len = 0;

    return mbedtls_rsa_rsaes_oaep_decrypt(&s_key.rsa, myrand, NULL, label, label_len, &pt_len,
                                          ct, pt, sizeof(pt)) == 0 &&
           matches_plaintext(pt, pt_len);
}

static void test_oaep(const char *key_name)
{
    const psa_algorithm_t alg = PSA_ALG_RSA_OAEP(PSA_ALG_SHA_256);
    static uint8_t ct[NVT_RSA_MAX_BYTES], ct_direct[NVT_RSA_MAX_BYTES], ct_sw[NVT_RSA_MAX_BYTES];
    uint8_t pt[NVT_RSA_MAX_BYTES];
    psa_key_id_t pair = 0, pub = 0;
    size_t ct_len = 0, direct_len = 0, pt_len = 0;
    uint32_t psa_enc_us, direct_enc_us, psa_dec_us, direct_dec_us;
    int ok, enc_ok, dec_ok;

    printf("\n  -- %s OAEP SHA-256 --\n", key_name);
    mbedtls_rsa_set_padding(&s_key.rsa, MBEDTLS_RSA_PKCS_V21, MBEDTLS_MD_SHA256);

    ok = import_key(1, alg, &pair) == PSA_SUCCESS && import_key(0, alg, &pub) == PSA_SUCCESS;
    report("import key pair + public key (DER)", ok);

    if (!ok)
    {
        goto exit;
    }

    enc_ok = 1;
    TIME_BEST(psa_enc_us, enc_ok,
              psa_asymmetric_encrypt(pub, alg, s_plaintext, sizeof(s_plaintext), NULL, 0,
                                     ct, sizeof(ct), &ct_len) == PSA_SUCCESS);
    report("psa_asymmetric_encrypt -> SW decrypt", enc_ok && ct_len == s_key.n_len && sw_decrypts(ct, NULL, 0));

    ok = 1;
    TIME_BEST(direct_enc_us, ok,
              nvt_rsa_oaep_encrypt_sha256(s_key.n, s_key.n_len, s_key.e, s_key.e_len, NULL, 0,
                                          s_plaintext, sizeof(s_plaintext),
                                          ct_direct, sizeof(ct_direct), &direct_len) == PSA_SUCCESS);
    report("direct encrypt -> SW decrypt", ok && sw_decrypts(ct_direct, NULL, 0));

    dec_ok = enc_ok;
    TIME_BEST(psa_dec_us, dec_ok,
              psa_asymmetric_decrypt(pair, alg, ct, ct_len, NULL, 0, pt, sizeof(pt), &pt_len) == PSA_SUCCESS);
    report("psa_asymmetric_decrypt", dec_ok && matches_plaintext(pt, pt_len));

    ok = enc_ok;
    TIME_BEST(direct_dec_us, ok,
              nvt_rsa_oaep_decrypt_sha256_protected(s_key.n, s_key.n_len, s_key.p, s_key.half,
                                                    s_key.q, s_key.half, s_key.d, s_key.n_len,
                                                    NULL, 0, ct, ct_len,
                                                    pt, sizeof(pt), &pt_len) == PSA_SUCCESS);
    report("direct decrypt", ok && matches_plaintext(pt, pt_len));

    report("SW encrypt -> psa_asymmetric_decrypt",
           mbedtls_rsa_rsaes_oaep_encrypt(&s_key.rsa, myrand, NULL, NULL, 0,
                                          sizeof(s_plaintext), s_plaintext, ct_sw) == 0 &&
           psa_asymmetric_decrypt(pair, alg, ct_sw, s_key.n_len, NULL, 0,
                                  pt, sizeof(pt), &pt_len) == PSA_SUCCESS &&
           matches_plaintext(pt, pt_len));

    report("encrypt with key pair -> SW decrypt",
           psa_asymmetric_encrypt(pair, alg, s_plaintext, sizeof(s_plaintext), NULL, 0,
                                  ct_sw, sizeof(ct_sw), &ct_len) == PSA_SUCCESS &&
           sw_decrypts(ct_sw, NULL, 0));

    report("label round trip (PSA encrypt -> SW + PSA decrypt)",
           psa_asymmetric_encrypt(pub, alg, s_plaintext, sizeof(s_plaintext), s_label, sizeof(s_label),
                                  ct_sw, sizeof(ct_sw), &ct_len) == PSA_SUCCESS &&
           sw_decrypts(ct_sw, s_label, sizeof(s_label)) &&
           psa_asymmetric_decrypt(pair, alg, ct_sw, ct_len, s_label, sizeof(s_label),
                                  pt, sizeof(pt), &pt_len) == PSA_SUCCESS &&
           matches_plaintext(pt, pt_len));

    ct[s_key.n_len / 2] ^= 0x01;
    report("tampered ciphertext rejected",
           enc_ok && psa_asymmetric_decrypt(pair, alg, ct, ct_len, NULL, 0, pt, sizeof(pt), &pt_len) ==
           PSA_ERROR_INVALID_PADDING);

    if (enc_ok && dec_ok)
    {
        printf("    encrypt: PSA %lu us, direct %lu us (best of %d)\n",
               (unsigned long) psa_enc_us, (unsigned long) direct_enc_us, TIMING_RUNS);
        printf("    decrypt: PSA %lu us, direct %lu us (best of %d)\n",
               (unsigned long) psa_dec_us, (unsigned long) direct_dec_us, TIMING_RUNS);
    }

    report("PSA encrypt within 1.2x direct", enc_ok && within_dispatch_budget(psa_enc_us, direct_enc_us));
    report("PSA decrypt within 1.2x direct", dec_ok && within_dispatch_budget(psa_dec_us, direct_dec_us));

exit:
    psa_destroy_key(pair);
    psa_destroy_key(pub);
}

/* PKCS#1 v1.5 encryption is not handled by the HW driver, so both calls must succeed via software. */
static void test_software_fallback(const char *key_name)
{
    const psa_algorithm_t alg = PSA_ALG_RSA_PKCS1V15_CRYPT;
    static uint8_t ct[NVT_RSA_MAX_BYTES];
    uint8_t pt[NVT_RSA_MAX_BYTES];
    psa_key_id_t pair = 0;
    size_t ct_len = 0, pt_len = 0;

    printf("\n  -- %s PKCS#1 v1.5 encryption (software fallback) --\n", key_name);
    report("PKCS#1 v1.5 encrypt + decrypt",
           import_key(1, alg, &pair) == PSA_SUCCESS &&
           psa_asymmetric_encrypt(pair, alg, s_plaintext, sizeof(s_plaintext), NULL, 0,
                                  ct, sizeof(ct), &ct_len) == PSA_SUCCESS &&
           psa_asymmetric_decrypt(pair, alg, ct, ct_len, NULL, 0, pt, sizeof(pt), &pt_len) == PSA_SUCCESS &&
           matches_plaintext(pt, pt_len));
    psa_destroy_key(pair);
}

int main(void)
{
    size_t i;

    SYS_UnlockReg();
    SYS_Init();
    UART0_Init();
    SysTick_Config(0x01000000U);

    printf("+-----------------------------------------------+\n");
    printf("|  mbedTLS PSA KeyAggrement RSA (PSA vs direct) |\n");
    printf("+-----------------------------------------------+\n");

    if (psa_crypto_init() != PSA_SUCCESS)
    {
        printf("psa_crypto_init failed\n");

        while (1)
        {
        }
    }

    for (i = 0; i < sizeof(s_keys) / sizeof(s_keys[0]); i++)
    {
        if (!load_key(&s_keys[i]))
        {
            report(s_keys[i].name, 0);
            mbedtls_rsa_free(&s_key.rsa);
            continue;
        }

        test_oaep(s_keys[i].name);

        if (i == 1)
        {
            test_software_fallback(s_keys[i].name);
        }

        mbedtls_rsa_free(&s_key.rsa);
    }

    printf("\n%d passed, %d failed\n", s_pass_count, s_fail_count);
    printf(s_fail_count == 0 ? "RESULT: ALL TESTS PASSED\n" : "RESULT: TESTS FAILED\n");

    while (1)
    {
    }
}
