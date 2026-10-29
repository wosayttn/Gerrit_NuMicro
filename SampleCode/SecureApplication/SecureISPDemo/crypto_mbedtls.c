/******************************************************************************
 * @file     crypto_mbedtls.c
 * @version  V1.00
 * @brief    Crypto mbedTLS wrapper functions
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "crypto_mbedtls.h"
#include "mbedtls/ecp.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"

static char s_achHexCharTbl[] = "0123456789abcdef";

static char  ch2hex(char ch)
{
    if (ch <= '9')
    {
        ch = ch - '0';
    }
    else if ((ch <= 'z') && (ch >= 'a'))
    {
        ch = ch - 'a' + 10U;
    }
    else
    {
        ch = ch - 'A' + 10U;
    }

    return ch;
}

static int  __strlen(char *str)
{
    char  *p = str;
    int   len = 0;

    while (*p != 0)
    {
        p++;
        len++;

        if (len > 1024)   /* max. 4096 bits */
            break;
    }

    // printf("< %d >\n", len);
    return len;
}

void Hex2Reg(char input[], uint32_t volatile reg[])
{
    char      hex;
    int       si, ri;
    uint32_t  i, val32;

    si = (int)__strlen(input) - 1;
    ri = 0;

    // printf("<%d>\n", si);

    while (si >= 0)
    {
        val32 = 0UL;

        for (i = 0UL; (i < 8UL) && (si >= 0); i++)
        {
            hex = ch2hex(input[si]);
            val32 |= (uint32_t)hex << (i * 4UL);
            si--;
        }

        reg[ri++] = val32;
    }
}

void Hex2RegEx(char input[], uint32_t volatile reg[], int shift)
{
    uint32_t  hex, carry;
    int       si, ri;
    uint32_t  i, val32;

    si = (int)__strlen(input) - 1;
    ri = 0L;
    carry = 0UL;

    while (si >= 0)
    {
        val32 = 0UL;

        for (i = 0UL; (i < 8UL) && (si >= 0L); i++)
        {
            hex = (uint32_t)ch2hex(input[si]);
            hex <<= shift;

            val32 |= (uint32_t)((hex & 0xFUL) | carry) << (i * 4UL);
            carry = (hex >> 4UL) & 0xFUL;
            si--;
        }

        reg[ri++] = val32;
    }

    if (carry != 0UL)
    {
        reg[ri] = carry;
    }
}

/**
  * @brief  Extract specified nibble from an unsigned word in character format.
  *         For example:
  *                Suppose val32 is 0x786543210, get_Nth_nibble_char(val32, 3) will return a '3'.
  * @param[in]  val32   The input unsigned word
  * @param[in]  idx     The Nth nibble to be extracted.
  * @return  The nibble in character format.
  */
static char get_Nth_nibble_char(uint32_t val32, uint32_t idx)
{
    return s_achHexCharTbl[(val32 >> (idx * 4U)) & 0xfU];
}

void Reg2Hex(int32_t count, uint32_t volatile reg[], char output[])
{
    int32_t    idx, ri;
    uint32_t   i;

    output[count] = 0U;
    idx = count - 1;

    for (ri = 0; idx >= 0; ri++)
    {
        for (i = 0UL; (i < 8UL) && (idx >= 0); i++)
        {
            output[idx] = get_Nth_nibble_char(reg[ri], i);
            idx--;
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Mbedtls wrapper functions                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Given a curve parameter, the other party's public key, and one's own private key to generate the secret Z.
  * @param[in]  psCRYPTO    Not used in mbedtls wrappers.
  * @param[in]  eECC_Curve  The pre-defined ECC curve.
  * @param[in]  private_k   One's own private key.
  * @param[in]  public_k1   The other party's publick key 1.
  * @param[in]  public_k2   The other party's publick key 2.
  * @param[out] secret_z    The ECC CDH secret Z.
  * @return  0    Success.
  * @return  -1   "eECC_Curve" value is invalid.
  */
int32_t  ECC_GenerateSecretZ(CRYPTO_T *psCRYPTO, E_ECC_CURVE eECC_Curve, char *private_k, char public_k1[], char public_k2[], char secret_z[])
{
    int32_t i32RetCode = 0;
    mbedtls_ecp_group grp;
    mbedtls_mpi d;
    mbedtls_ecp_point Qp;
    mbedtls_mpi z;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_entropy_context entropy;
    unsigned char shared_secret[MBEDTLS_ECP_MAX_BYTES];
    const char *pers = "ecc_gen_secret";
    int i;
    size_t keylen;

    NVT_UNUSED(psCRYPTO);

    // Initialize
    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Qp);
    mbedtls_mpi_init(&z);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_entropy_init(&entropy);

    // Seed the random number generator
    if ((i32RetCode = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                            (const unsigned char *)pers, strlen(pers))) != 0)
    {
        goto cleanup;
    }

    // Load the elliptic curve
    if ((i32RetCode = mbedtls_ecp_group_load(&grp, eECC_Curve)) != 0)
    {
        i32RetCode = -1;
        goto cleanup;
    }

    // Read private key from hex string
    if ((i32RetCode = mbedtls_mpi_read_string(&d, 16, private_k)) != 0)
    {
        goto cleanup;
    }

    // Read public key (x, y coordinates) from hex strings
    if ((i32RetCode = mbedtls_ecp_point_read_string(&Qp, 16, public_k1, public_k2)) != 0)
    {
        goto cleanup;
    }

    // Compute shared secret using low-level ECDH compute function
    if ((i32RetCode = mbedtls_ecdh_compute_shared(&grp, &z, &Qp, &d,
                                                  mbedtls_ctr_drbg_random, &ctr_drbg)) != 0)
    {
        goto cleanup;
    }

    // Convert the shared secret to binary
    keylen = (grp.nbits + 7) / 8;

    if ((i32RetCode = mbedtls_mpi_write_binary(&z, shared_secret, keylen)) != 0)
    {
        goto cleanup;
    }

    // Convert binary to hex string
    for (i = 0; i < (int)keylen; i++)
    {
        sprintf(&secret_z[i * 2], "%02x", shared_secret[i]);
    }

    secret_z[keylen * 2] = '\0';

cleanup:
#ifdef DBG
    DBG("i32RetCode: %d\n", i32RetCode);
#endif
    // Free resources
    mbedtls_ecp_group_free(&grp);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Qp);
    mbedtls_mpi_free(&z);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);

    return i32RetCode;
}

/**
  * @brief  Check if the private key is located in valid range of curve.
  * @param[in]  psCRYPTO    Not used in mbedtls wrappers.
  * @param[in]  eECC_Curve  The pre-defined ECC curve.
  * @param[in]  private_k   The input private key.
  * @return  1    Is valid.
  * @return  0    Is not valid.
  * @return  -1   Invalid curve.
  */
int32_t ECC_IsPrivateKeyValid(CRYPTO_T *psCRYPTO, E_ECC_CURVE eECC_Curve, char private_k[])
{
    int32_t i32RetCode = 0;
    mbedtls_ecp_group grp;
    mbedtls_mpi d;

    NVT_UNUSED(psCRYPTO);

    // Initialize
    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);

    // Load the elliptic curve
    if ((i32RetCode = mbedtls_ecp_group_load(&grp, eECC_Curve)) != 0)
    {
        i32RetCode = -1;
        goto cleanup;
    }

    // Read private key from hex string
    if ((i32RetCode = mbedtls_mpi_read_string(&d, 16, private_k)) != 0)
    {
        i32RetCode = 0; // Invalid key format, treat as invalid
        goto cleanup;
    }

    // Check if private key is valid for the curve
    if ((i32RetCode = mbedtls_ecp_check_privkey(&grp, &d)) == 0)
    {
        i32RetCode = 1; // Valid
    }
    else
    {
        i32RetCode = 0; // Invalid
    }

cleanup:
    // Free resources
    mbedtls_ecp_group_free(&grp);
    mbedtls_mpi_free(&d);

    return i32RetCode;
}

/**
  * @brief  Given a private key and curve to generate the public key pair.
  * @param[in]  psCRYPTO    Not used in mbedtls wrappers.
  * @param[in]  eECC_Curve  The pre-defined ECC curve.
  * @param[in]  private_k   The input private key.
  * @param[out] public_k1   The output public key 1.
  * @param[out] public_k2   The output public key 2.
  * @return  0    Success.
  * @return  -1   "eECC_Curve" value is invalid.
  */
int32_t ECC_GeneratePublicKey(CRYPTO_T *psCRYPTO, E_ECC_CURVE eECC_Curve, char *private_k, char public_k1[], char public_k2[])
{
    int32_t i32RetCode = 0;
    mbedtls_ecp_group grp;
    mbedtls_mpi d;
    mbedtls_ecp_point Q;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_entropy_context entropy;
    unsigned char pubkey_buf[MBEDTLS_ECP_MAX_PT_LEN];
    size_t olen;
    const char *pers = "ecc_gen_pubkey";
    int i;
    size_t keylen;

    NVT_UNUSED(psCRYPTO);

    // Initialize
    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_entropy_init(&entropy);

    // Seed the random number generator
    if ((i32RetCode = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                            (const unsigned char *)pers, strlen(pers))) != 0)
    {
        goto cleanup;
    }

    // Load the elliptic curve
    if ((i32RetCode = mbedtls_ecp_group_load(&grp, eECC_Curve)) != 0)
    {
        i32RetCode = -1;
        goto cleanup;
    }

    // Read private key from hex string
    if ((i32RetCode = mbedtls_mpi_read_string(&d, 16, private_k)) != 0)
    {
        goto cleanup;
    }

    // Generate public key: Q = d * G (generator point)
    if ((i32RetCode = mbedtls_ecp_mul(&grp, &Q, &d, &grp.G, mbedtls_ctr_drbg_random, &ctr_drbg)) != 0)
    {
        goto cleanup;
    }

    // Export public key to uncompressed format (04 + x + y)
    if ((i32RetCode = mbedtls_ecp_point_write_binary(&grp, &Q, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                                     &olen, pubkey_buf, sizeof(pubkey_buf))) != 0)
    {
        goto cleanup;
    }

    // Verify uncompressed format and extract coordinates
    if (pubkey_buf[0] != 0x04)
    {
        i32RetCode = -1;
        goto cleanup;
    }

    keylen = (grp.nbits + 7) / 8;

    // Convert x coordinate to hex string
    for (i = 0; i < (int)keylen; i++)
    {
        sprintf(&public_k1[i * 2], "%02x", pubkey_buf[1 + i]);
    }

    public_k1[keylen * 2] = '\0';

    // Convert y coordinate to hex string
    for (i = 0; i < (int)keylen; i++)
    {
        sprintf(&public_k2[i * 2], "%02x", pubkey_buf[1 + keylen + i]);
    }

    public_k2[keylen * 2] = '\0';

cleanup:
    // Free resources
    mbedtls_ecp_group_free(&grp);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);

    return i32RetCode;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
