#include <stdio.h>
#include "NuMicro.h"
#include "nvt_crpt.h"
#include "psa/crypto.h"

int run_aead_tests(void);

psa_status_t mbedtls_psa_external_get_random(
    mbedtls_psa_external_random_context_t *context,
    uint8_t *output,
    size_t output_size,
    size_t *output_length)
{
    (void) context;
    return nvt_crpt_get_random(output, output_size, output_length);
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
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC,
                       CLK_CLKDIV0_UART0(1));

    SystemCoreClockUpdate();

    /* UART0 RXD=PA6, TXD=PA7 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk)) |
                    UART0_RXD_PA6 | UART0_TXD_PA7;
}

static void UART0_Init(void)
{
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
}

int main(void)
{
    psa_status_t status;

    SYS_UnlockReg();
    SYS_Init();
    UART0_Init();

    printf("+-----------------------------------+\n");
    printf("|       MbedTLS PSA AEAD Demo       |\n");
    printf("+-----------------------------------+\n");

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("[FAIL] psa_crypto_init: %ld\n", (long) status);

        while (1)
        {
        }
    }

    if (run_aead_tests() != 0)
    {
        while (1)
        {
        }
    }

    printf("[PASS] PSA AEAD one-shot checks complete\n");

    while (1)
    {
    }
}