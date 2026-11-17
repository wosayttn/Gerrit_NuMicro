/*
 * Shared CRPT engine services for the Nuvoton PSA drivers: the CRPT IRQ
 * dispatcher and a single-caller guard.
 *
 * Copyright (c) 2026, Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 */

#ifndef NVT_CRPT_H
#define NVT_CRPT_H

#include <psa/crypto.h>

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
    extern volatile uint32_t nvt_psa_test_force_hardware_failure;
#endif

#if defined(MBEDTLS_THREADING_C) && \
    (defined(MBEDTLS_PSA_NVT_HASH_DRIVER_ENABLED) || \
     defined(MBEDTLS_PSA_NVT_CIPHER_DRIVER_ENABLED) || \
     defined(MBEDTLS_PSA_NVT_AEAD_DRIVER_ENABLED) || \
     defined(MBEDTLS_PSA_NVT_ECC_DRIVER_ENABLED) || \
     defined(MBEDTLS_PSA_NVT_RSA_DRIVER_ENABLED))
#error "Nuvoton CRPT drivers support one caller at a time only: disable MBEDTLS_THREADING_C or the MBEDTLS_PSA_NVT_*_DRIVER_ENABLED drivers"
#endif

/* Set by nvt_crpt_irq_handler() when the SHA engine completes. */
extern volatile int g_SHA_done;
extern volatile uint32_t g_SHA_error;
extern volatile uint32_t g_SHA_irq_status;
extern volatile uint32_t g_SHA_error_status;

/* The application's CRPT_IRQHandler() must call this and nothing else. */
void nvt_crpt_irq_handler(void);

/* Returns PSA_ERROR_BAD_STATE if another call is already using the CRPT engine. */
psa_status_t nvt_crpt_acquire(void);
void nvt_crpt_release(void);

psa_status_t nvt_crpt_get_random(uint8_t *output, size_t output_size,
                                 size_t *output_length);

#endif /* NVT_CRPT_H */
