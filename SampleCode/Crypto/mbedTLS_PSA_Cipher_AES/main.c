/*
 * Entry point for the PSA symmetric cipher (AES) sample.
 *
 * The cipher/AEAD test logic lives in symmetric_ciphers_test_main.c (main_task);
 * this file provides board bring-up + PSA init and then runs it.
 */
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "psa/crypto.h"
#include "nvt_crpt.h"

extern int main_task(void);

static int test_psa_random(void)
{
    static const size_t sizes[] = {1U, 32U, 257U};
    uint8_t first[257];
    uint8_t second[257];
    size_t index;
    psa_status_t status;

    status = psa_generate_random(first, 0U);

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] PSA RNG zero-length request: %ld\n", (long) status);
        return -1;
    }

    for (index = 0U; index < sizeof(sizes) / sizeof(sizes[0]); index++)
    {
        status = psa_generate_random(first, sizes[index]);

        if (status == PSA_SUCCESS)
        {
            status = psa_generate_random(second, sizes[index]);
        }

        if (status != PSA_SUCCESS)
        {
            printf("[FAIL] PSA RNG %lu bytes: %ld\n",
                   (unsigned long) sizes[index], (long) status);
            return -1;
        }

        if (sizes[index] > 1U && memcmp(first, second, sizes[index]) == 0)
        {
            printf("[FAIL] PSA RNG repeated output: %lu bytes\n",
                   (unsigned long) sizes[index]);
            return -1;
        }

        printf("[PASS] PSA RNG %lu bytes\n", (unsigned long) sizes[index]);
    }

    return 0;
}

void CRPT_IRQHandler(void)
{
    nvt_crpt_irq_handler();
}

static void SYS_Init(void)
{
    /* Enable HIRC and set core clock to 96 MHz */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_SetCoreClock(96000000);

    /* Module clocks: UART0 (console) and CRYPTO (AES engine) */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(CRPT_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    SystemCoreClockUpdate();

    /* UART0 RXD=PA6, TXD=PA7 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;
}

static void UART0_Init(void)
{
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
}

int main(void)
{
    SYS_UnlockReg();
    SYS_Init();
    UART0_Init();

    printf("+-----------------------------------+\n");
    printf("|   MbedTLS PSA Cipher (AES) Demo   |\n");
    printf("+-----------------------------------+\n");

    if (psa_crypto_init() != PSA_SUCCESS)
    {
        printf("psa_crypto_init failed\n");
        return -1;
    }

    if (test_psa_random() != 0)
    {
        return -1;
    }

    if (main_task() != 0)
    {
        return -1;
    }

    while (1)
    {
    }
}
