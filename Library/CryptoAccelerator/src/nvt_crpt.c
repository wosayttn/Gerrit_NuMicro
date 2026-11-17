/*
 * Shared CRPT engine services for the Nuvoton PSA drivers.
 *
 * Copyright (c) 2026, Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 */

#include "nvt_crpt.h"

#include "NuMicro.h"
#include "mbedtls/platform_util.h"

volatile int g_SHA_done;
volatile uint32_t g_SHA_error;
volatile uint32_t g_SHA_irq_status;
volatile uint32_t g_SHA_error_status;

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
    volatile uint32_t nvt_psa_test_force_hardware_failure;
#endif

static volatile uint8_t s_crpt_busy;

void nvt_crpt_irq_handler(void)
{
    uint32_t sha_int_status = CRPT->INTSTS;

    if ((sha_int_status & (CRPT_INTSTS_HMACIF_Msk |
                           CRPT_INTSTS_HMACEIF_Msk)) != 0U)
    {
        g_SHA_irq_status = sha_int_status;

        if ((sha_int_status & CRPT_INTSTS_HMACEIF_Msk) != 0U)
        {
            g_SHA_error = 1U;
            g_SHA_error_status = CRPT->HMAC_STS;
        }

        if ((sha_int_status & CRPT_INTSTS_HMACIF_Msk) != 0U)
        {
            g_SHA_done = 1;
        }

        SHA_CLR_INT_FLAG(CRPT);
    }

    ECC_DriverISR(CRPT);

    /* AES/RSA are polled; only drain their flags if someone enabled their IRQ,
     * otherwise the NVIC would re-enter this handler forever. */
    if ((CRPT->INTEN & (CRPT_INTEN_AESIEN_Msk | CRPT_INTEN_AESEIEN_Msk)) != 0U &&
            AES_GET_INT_FLAG(CRPT))
    {
        AES_CLR_INT_FLAG(CRPT);
    }

    if ((CRPT->INTEN & (CRPT_INTEN_RSAIEN_Msk | CRPT_INTEN_RSAEIEN_Msk)) != 0U &&
            RSA_GET_INT_FLAG(CRPT))
    {
        RSA_CLR_INT_FLAG(CRPT);
    }
}

psa_status_t nvt_crpt_acquire(void)
{
    psa_status_t status = PSA_SUCCESS;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    if (s_crpt_busy != 0U)
    {
        status = PSA_ERROR_BAD_STATE;
    }
    else
    {
        s_crpt_busy = 1U;
    }

    __set_PRIMASK(primask);

    return status;
}

void nvt_crpt_release(void)
{
    s_crpt_busy = 0U;
}

psa_status_t nvt_crpt_get_random(uint8_t *output, size_t output_size,
                                 size_t *output_length)
{
    size_t index;
    uint32_t base_ctl;
    int32_t timeout;

    if (output_length == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    *output_length = 0U;

    if (output_size == 0U)
    {
        return PSA_SUCCESS;
    }

    if (output == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;
    CLK->APBCLK1 |= CLK_APBCLK1_TRNGCKEN_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;

    if ((TRNG->CTL & TRNG_CTL_READY_Msk) == 0U)
    {
        RTC->LXTCTL |= RTC_LXTCTL_C32KSEL_Msk | RTC_LXTCTL_LIRC32KEN_Msk;
        TRNG->ACT |= TRNG_ACT_ACT_Msk;
        timeout = 0x1000;

        while ((TRNG->CTL & TRNG_CTL_READY_Msk) == 0U)
        {
            if (timeout-- <= 0)
            {
                mbedtls_platform_zeroize(output, output_size);
                return PSA_ERROR_HARDWARE_FAILURE;
            }
        }

        TRNG->CTL = (0U << TRNG_CTL_CLKPSC_Pos);
    }

    base_ctl = TRNG->CTL;

    for (index = 0U; index < output_size; index++)
    {
        timeout = 0x100000;
        TRNG->CTL = TRNG_CTL_TRNGEN_Msk | base_ctl;

        while ((TRNG->CTL & TRNG_CTL_DVIF_Msk) == 0U)
        {
            if (timeout-- <= 0)
            {
                mbedtls_platform_zeroize(output, output_size);
                return PSA_ERROR_HARDWARE_FAILURE;
            }
        }

        output[index] = (uint8_t) TRNG->DATA;
    }

    *output_length = output_size;
    return PSA_SUCCESS;
}
