/******************************************************************************
 * @file     crypto_mbedtls.c
 * @version  V1.00
 * @brief    Crypto mbedTLS wrapper functions header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __CRYPTO_MBEDTLS_H__
#define __CRYPTO_MBEDTLS_H__

#include "NuMicro.h"
#include "mbedtls/ecdh.h"

#define E_ECC_CURVE         mbedtls_ecp_group_id
#define CURVE_P_256         MBEDTLS_ECP_DP_SECP256R1

#ifdef __cplusplus
extern "C"
{
#endif

int32_t ECC_VerifySignature(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *message, char *public_k1, char *public_k2, char *R, char *S);
int32_t ECC_GenerateSecretZ(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *private_k, char public_k1[], char public_k2[], char secret_z[]);
int32_t ECC_IsPrivateKeyValid(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char private_k[]);
int32_t ECC_GeneratePublicKey(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *private_k, char public_k1[], char public_k2[]);
void Hex2Reg(char input[], uint32_t volatile reg[]);
void Reg2Hex(int32_t count, uint32_t volatile reg[], char output[]);

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_MBEDTLS_H__ */

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
