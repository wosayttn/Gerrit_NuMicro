/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show Crypto IP SHA function
 *
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

/*To call psa and nvt_psa(native) API.*/
#include "tf_psa_crypto_common.h"
#include "psa_get_started_example.h"
#include "mbedtls/platform.h"
#include "mbedtls/md.h"
#include "mbedtls/private/crypto_builtin_primitives.h"
#include "psa/crypto.h"
#include "nvt_psa_sha.h"
#include "nvt_crpt.h"

/* App-provided RNG required when MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG is enabled. */
psa_status_t mbedtls_psa_external_get_random(
    mbedtls_psa_external_random_context_t *context,
    uint8_t *output, size_t output_size, size_t *output_length)
{
    (void) context;
    return nvt_crpt_get_random(output, output_size, output_length);
}

uint32_t u32buf[640] =
{
    0xBC5FF810, 0xEB089048, 0xB8AB3020, 0xA7BD3B16, 0xC0E0CA3D, 0x6B97E464, 0x6C2CCAE0, 0xBBF19EF7,
    0xBA2B57C4, 0x46556EE2, 0xB72C78B9, 0x6BB7A850, 0x3DE40AFB, 0x54184E3B, 0x5463C21A, 0xF7485323,
    0xDF98F016, 0x0AE5D137, 0x512725F8, 0x9D563BC9, 0xA189D31D, 0x20B3B3C8, 0xFFAAF5E4, 0x86E79051,
    0xF6F3605C, 0xCA2569FD, 0xB46B3318, 0xD23842CE, 0x40D64386, 0x13F68B45, 0x5B0D3BCA, 0x0E050D4D,
    0x119988A2, 0xC4801B90, 0x84E0B048, 0xC9280922, 0x30902406, 0x49984065, 0x5A268ADA, 0x3290DA48,
    0x08228190, 0xC81461DC, 0x166A2147, 0x8E08B221, 0xE308681A, 0x024414C6, 0x65E19871, 0x90C6690C,
    0x1544C9A0, 0x11CC3471, 0x83404500, 0x124D9108, 0x00833684, 0x12854CCC, 0x00410990, 0x70189500,
    0xA2B28594, 0x26410B00, 0x410AB980, 0xC0C6100C, 0x3329A3A6, 0x281C2610, 0x1A374913, 0x358A0B29,
    0x2E82A270, 0x8B384994, 0x04809B26, 0x10A48030, 0x04370C48, 0xB2600417, 0x2E194964, 0xC8C4641A,
    0x96601083, 0x6D203822, 0x4906081B, 0xB7200148, 0x4DE11049, 0x0838444C, 0x168E04B2, 0x2C099150,
    0x8336065C, 0x028C8A38, 0x051AB581, 0x82C00951, 0x12722210, 0x0D04082A, 0xA4841058, 0x08520826,
    0x0A02B62D, 0x8A120E14, 0x22104844, 0x0A142391, 0x5040900A, 0x2792A4B0, 0x60088411, 0xC4402263,
    0x462ADAA6, 0x90C1B230, 0x501000A0, 0xB46C53B4, 0x5013058D, 0x02310E08, 0x201091C8, 0x4C22A750,
    0xC2A67092, 0x46416316, 0x8C840011, 0x0B819013, 0x83712185, 0x301B1846, 0xA3108414, 0x43408098,
    0x250C272C, 0x944241A2, 0x88450213, 0x0544B644, 0x522292D0, 0x80504BA6, 0x04843670, 0x09B24D19,
    0xA8842493, 0x4994A280, 0x49B44891, 0x2864A1C8, 0x654B8285, 0x93300663, 0x12661037, 0x014A4080,
    0x18189044, 0xC40D4B28, 0x81A20640, 0xD4302C1B, 0x964CE1B2, 0x60442841, 0xD8306524, 0x09046400,
    0x8963C224, 0xC0004992, 0x165223C1, 0x29422691, 0xD0383183, 0x28284C28, 0x611AB288, 0x80262D0C,
    0x19525B22, 0x608A9228, 0xA21890D9, 0x4252CB40, 0x8E9B1606, 0x4BC805E0, 0x066C49C2, 0x25D42269,
    0x1411691C, 0x34900C85, 0x8D1C8449, 0x63108508, 0x34895816, 0x66A03868, 0x0BA250E0, 0x8461C126,
    0x6EC9146A, 0xC0C03184, 0x182EDC16, 0x52141891, 0x0A392C83, 0xA68D1229, 0x6292A660, 0x08A510D9,
    0xC84D2448, 0x24839824, 0x1A444208, 0x0848C896, 0x2DA11020, 0x6024519C, 0x3011DBC8, 0x04112251,
    0x180468E2, 0x244E10C4, 0x0D183729, 0x11B78443, 0x848D5440, 0x70648070, 0x21B04D00, 0x26620045,
    0x251B8384, 0x84807019, 0xB0040AC9, 0x28CA8005, 0x1B214ED4, 0xB0498C96, 0x6CD09625, 0x48327254,
    0x12050998, 0x8C8A326D, 0xC0043163, 0xC861C8C2, 0x689BB48C, 0x5BC02DC3, 0xA88D1B16, 0x6CD89641,
    0x98340244, 0x142CD386, 0x440A188A, 0x1A46921A, 0xB7084C34, 0x2421910D, 0x9A962DCB, 0x24229286,
    0x0898A050, 0xDA446A19, 0xB1058B20, 0x6E24C481, 0xC9984022, 0x19321105, 0x69D39491, 0x08A771DB,
    0xC07002B8, 0x2883B449, 0xA0448A0B, 0xB4106102, 0x481CA611, 0xE3206640, 0xB47003B6, 0x04604821,
    0x5B218339, 0x88DB674A, 0x61E7C208, 0xDEBE8DAE, 0x4119AFB0, 0x2661A69A, 0xBC8BDD24, 0x5B5D0FB1,
    0xA26701C9, 0xB9C9A8F7, 0xD9FCD4C2, 0x87FF3D60, 0x8CF25828, 0x2A1EB29F, 0x9304E89C, 0x14F3E1CE,
    0x5612891C, 0x60293452, 0x6006C99B, 0x4AA2399B, 0xF494BCF8, 0xDF61D6DF, 0x4C69BC93, 0xE02D4995,
    0xE2F76E9F, 0xDA4EF67E, 0xB7256CA8, 0x9A3F38FE, 0xB2E9DF6A, 0x010DC1C1, 0x5002FBD4, 0x56FAE884,
    0x821A3416, 0x6B0658A2, 0x41259571, 0x8E149BBC, 0x6E220AEE, 0x268D4D82, 0x18C25F6A, 0x957DE5B2,
    0x6CEA7B14, 0xCB320D89, 0xE1699AD9, 0xF2B389C6, 0x7EF93386, 0xA2C65F2C, 0x32233367, 0xD76AE4AB,
    0x2ABBD422, 0xE98E493D, 0xCC3CC5DA, 0xF689B65C, 0xC4BC3FA5, 0x1C9C59EE, 0xAF075517, 0x0C249580,
    0x4D02A607, 0xC5BF887C, 0xD86A0389, 0xF28FC972, 0x5EF46003, 0xF13B0187, 0x684BEAB1, 0xF24A29F5,
    0x319601F3, 0x09C91D2A, 0x333D1B88, 0xDF205A51, 0x20C4CFDC, 0x2238124E, 0x4E2B47D0, 0xB5E66A65,
    0x4FE4CCCB, 0x078F07CB, 0xD455D15D, 0x3EEC7DA2, 0x74D24A2E, 0x571884DE, 0x41C3A9A4, 0xFDB3F609,
    0x8A172C30, 0x968039BD, 0x0E4EB3E2, 0xFB6D6EEE, 0xD39E0B63, 0x62D54E7B, 0x88959869, 0xDDD5D873,
    0xD9652401, 0xA29F27A2, 0x8EA66D32, 0xCCB0EF3B, 0xF4600F75, 0x57EE8D54, 0xBF1DAD18, 0xF45DDCD4,
    0xC9ED57B1, 0x3E445BF1, 0x22A43F53, 0x940389BF, 0x8714FFAC, 0x721E5931, 0x7E4B7050, 0x0AD0D1B9,
    0xA627054D, 0x3193208C, 0x774E0B20, 0xED041A8C, 0x055E75EE, 0xCD3738F0, 0x07158FAD, 0xDFCA5F43,
    0x562D636A, 0x5ACF3A39, 0x83D3CFEB, 0xCA10B813, 0xF9F65265, 0x19199A03, 0x13CD1DE1, 0x3F06AD53,
    0x86E1E120, 0x795FD267, 0xB7F42019, 0xD84DF6CD, 0x1BF91930, 0xFCA7AFD5, 0x2E80700F, 0x4CF5CDC3,
    0x8A5F7A57, 0x49791C2F, 0xDFFC4A10, 0x753C24DB, 0x19E8EB65, 0x1C5B3632, 0x00F0B5D1, 0x69947026,
    0xE9F74F01, 0x2DC7C5B3, 0x39DD49D2, 0x61CA1D37, 0xF8F28346, 0xE6197805, 0x4F45AEE4, 0x36DCCBE7,
    0xBFAFE07C, 0xE9A8B83C, 0x90A2686F, 0xA9540285, 0x0925C858, 0x2BC9B734, 0xE4ECA1F7, 0xB20B086F,
    0x129F277A, 0x5CBDAA96, 0x3C92717E, 0xF70EC19B, 0xF3DBC6DA, 0x203AD90F, 0x3B13BBC2, 0x2FBD980B,
    0xB1B9D3A3, 0x4452B335, 0x7021CE36, 0x13584E09, 0x36BF1D09, 0x42093781, 0x5E11CC5D, 0x5DDB4BF1,
    0xD830C4F8, 0x3F30E515, 0x921C784D, 0x87BB20C0, 0x9E3C64BD, 0xCE9AB1C6, 0x9FD307EF, 0xE359C7F9,
    0x38566C9F, 0x2517B063, 0x385167E2, 0x47F31081, 0x119BAC6B, 0x55A0BDD7, 0x1425510F, 0xFA2ABDFA,
    0x888376A3, 0x7F20C248, 0x0152BB36, 0x16345200, 0x07C5B34B, 0xF22817CB, 0x2E67AC1A, 0x82670B71,
    0xF196C89F, 0x23BA314B, 0x16A94849, 0x9304EF5C, 0x03DCF58E, 0x52BE3148, 0x63E723C5, 0x6D3AEB34,
    0x0BFF18AB, 0xFA20DC03, 0x44203023, 0x0533D912, 0x9B83ED22, 0xC351F232, 0x8172E363, 0x447444AE,
    0x5C6902B7, 0x92799F54, 0x44507871, 0x19612E9B, 0xB4130A33, 0xA2A5962A, 0xC09D577D, 0x6DDC881F,
    0xE6616126, 0xD8A0A7DF, 0x2B2253BC, 0x8EC4E353, 0x86EA5511, 0xF0F15887, 0x145B6C23, 0xAB3D4033,
    0x39E40407, 0x3ED9C6A8, 0x96A2F9EC, 0x70C44BD2, 0xAEC10FC4, 0x360E8763, 0x6BE155B6, 0xA67B7EDF,
    0x38CF7300, 0x4813C9E7, 0xD2C654C2, 0x530A71E5, 0xF8C10942, 0xFB6D8841, 0x535AB1DA, 0x43E8CB0B,
    0xB89E78EC, 0x91F8DE15, 0x31A03665, 0xCCD5A75B, 0xDA0ED0E5, 0x9864EEEF, 0x51A83FA5, 0x53AF662A,
    0xEE00D1F8, 0x367B4D5D, 0xDDC34554, 0x4C6BD514, 0xF888E603, 0x3C255DB6, 0x50DA734A, 0xD33A3CF8,
    0x4BD3F06F, 0xA1A7CA02, 0xE4B8E993, 0xAE7AE634, 0x20A46BA8, 0xA3813D1E, 0x9D2966BB, 0x8560D71C,
    0x62A044EA, 0x94179F4E, 0xB1B6ED60, 0x719D51E0, 0xEEF6CD07, 0x9152F6BE, 0x488EC919, 0x11C6D3F1,
    0xD1173C54, 0x1F9D25BF, 0x342FCAA3, 0xFF46C18F, 0x2A0441D8, 0x3BDE3546, 0xA9826C34, 0x96E06F2F,
    0x2B0EEB9D, 0x5BE8739F, 0x83A42D3B, 0x300E70EE, 0x84DFFFB2, 0x0764A060, 0x212F058C, 0x8A5FFA9A,
    0x34E928D6, 0xA7E07708, 0xFE5393E3, 0x017CE470, 0xEB9658A7, 0x4E4951E6, 0xFA4854C9, 0xE9C28988,
    0x812E4418, 0xA2E83258, 0x0B4A2703, 0x72BC6967, 0x6889D0CC, 0x43240EDA, 0xBC1D3114, 0xD8F35AB2,
    0xE9EA9530, 0x82E95362, 0x79ACB3BE, 0x16D3A205, 0xF46CB67B, 0x22149693, 0x5AC04292, 0xBBFB9A61,
    0xC0A03EF4, 0xC9B68204, 0x95F3D80E, 0x4A6FB7E1, 0xC69903FA, 0x226E023E, 0x95BA416D, 0xF2E5E454,
    0x1E15DCC0, 0x00B5E65C, 0x9720DAF6, 0x96012FA2, 0xA6CF758E, 0xD6D225A3, 0xE4FEE45A, 0xC5FB4870,
    0x7FAE133D, 0x592CFD2E, 0x8C43C212, 0x6F652BEE, 0x9BAB43A1, 0xA10BE241, 0x1A6794B2, 0x6CB55CC2,
    0x17EB7B0B, 0x146D23F7, 0x922D3222, 0xAE5EE8C6, 0xD38E8399, 0xBA51C681, 0xB83816FC, 0xF7443882,
    0x5920F9CE, 0x8A202A8F, 0x6D942DA8, 0x6238FB4C, 0x9F2198EA, 0x8DFF81C1, 0x7286E018, 0xDF4B7FE3,
    0x884D1759, 0xE4C59BB5, 0x2617AED4, 0xE78E4E7C, 0x4E9A36E4, 0xE996D323, 0x91A34A0D, 0xAAAB6B54,
    0x0815A34D, 0x20407AEF, 0x81949BE6, 0x7B906950, 0xD89BE9F0, 0x85E99EB5, 0x87269517, 0x3B3EFACA,
    0xE9455D2B, 0x2CD4F710, 0xB872CF66, 0x2B736216, 0xB1BBFB1F, 0x5F3D486C, 0x7B4B8756, 0x12333F8E,
    0x4BA933DC, 0x79F0EDFD, 0x7BAADE2C, 0x16F2146A, 0x496F79C4, 0x2A4D6B52, 0x39A30DD3, 0xC48BEB09,
    0x2CA07500, 0x10F69ED4, 0xB9232014, 0x7DBBE208, 0xF6E8EB1C, 0xF247D21A, 0x3A3B01DF, 0x58C0AA62,
    0x944DA0EF, 0x0450E8CE, 0x48AA137E, 0x7E1516C1, 0xD5C86EEA, 0x17FDFAC1, 0x690746E7, 0x267045A3,
    0xE90596BD, 0xB75D50B6, 0xDD5C34E5, 0xC8D89DC6, 0xF2F1D244, 0x40E57B47, 0x59B8625F, 0x72BC4A7B,
    0x10D519D3, 0x31F9C400, 0xAAE1E50D, 0x480CAAE5, 0xA1C0FA99, 0xD77924CF, 0x8DFE56CD, 0x7092E7B9
};


/**
 * Macros and define
 */
#define ptr_to_u32(x)     ((uint32_t)((uint64_t)(x)))
#define u32_to_ptr32(x)   ((uint32_t *)((uint64_t)(x)))
#define u32_to_ptr8(x)     ((uint8_t *)((uint64_t)(x)))

#define BUFF_SIZE      (32*1024)

#define DLOG(...)      do{}while(0);
//#define DLOG  printf
#define CHECK_OVERALL_TIME

/**
 * Variables declaration
 */
uint32_t MY_NUM = 1024;// (2024+1)
uint32_t au32OutputDigest[16];
unsigned char actual_hash[PSA_HASH_MAX_SIZE];
uint8_t    g_buff_pool[BUFF_SIZE] __attribute__((aligned(32)));
uint32_t   g_fdbk_buff[88] __attribute__((aligned(32)));
uint8_t    *g_sha_msg;
uint8_t    g_sha_digest[1024];

extern uint64_t t_start, t0;

/**
 * SysTick initialisation
 */
static uint64_t s_u64CPUCycleCount = 0;    /* 64-bit cpu cycle counter */
static int InitSysTick(void)
{
    const uint32_t u32Ticks10ms = SystemCoreClock / 100 + 1;
    int i32Err = 0;

    /* Reset CPU cycle count value. */
    s_u64CPUCycleCount = 0;
    /* Changing configuration for sys tick => guard from being
     * interrupted. */
    NVIC_DisableIRQ(SysTick_IRQn);
    /* SysTick init - this will enable interrupt too. */
    i32Err = SysTick_Config(u32Ticks10ms);
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    /* Enable interrupt again. */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Wait for SysTick to kick off */
    while (!i32Err && !SysTick->VAL)
    {
        __NOP();
    }

    return i32Err;
}

/**
 * SysTick IRQ handler
 */
void SysTick_Handler(void)
{
    /* Increment the cycle counter based on load value. */
    s_u64CPUCycleCount += SysTick->LOAD + 1;
    __DSB();
    __ISB();
}

/**
 * Gets the current SysTick derived counter value
 */
uint64_t GetSysTickCycleCount(void)
{
    uint64_t high1, high2;
    uint32_t val;

    do
    {
        high1 = s_u64CPUCycleCount;
        val = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
        high2 = s_u64CPUCycleCount;
    } while (high1 != high2);

    return high2 + (SysTick->LOAD - val);
}

static void print_elapsed_time_us(const char *label, uint64_t cycles)
{
    uint64_t seconds = cycles / SystemCoreClock;
    uint64_t remaining_cycles = cycles % SystemCoreClock;
    uint64_t fractional_nanoseconds =
        remaining_cycles * 1000000000ULL / SystemCoreClock;
    uint64_t elapsed_microseconds = seconds * 1000000ULL + fractional_nanoseconds / 1000ULL;

    printf("%s: %lu.%03lu us\n", label,
           (unsigned long) elapsed_microseconds,
           (unsigned long)(fractional_nanoseconds % 1000ULL));
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Enable GPIOB module clock */
    //CLK_EnableModuleClock(GPB_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

}

void DEBUG_PORT_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}

void CRPT_IRQHandler(void)
{
    nvt_crpt_irq_handler();
}

static int print_sha_execution_path(const char *label)
{
    switch (nvt_psa_sha_last_compute_path)
    {
        case NVT_PSA_SHA_COMPUTE_HARDWARE:
            printf("[PATH] %s: hardware\n", label);
            return 1;

        case NVT_PSA_SHA_COMPUTE_SOFTWARE_FALLBACK:
            printf("[PATH] %s: software fallback after hardware failure\n", label);
            return 0;

        default:
            printf("[FAIL] %s did not report the NVT hardware path\n", label);
            return 0;
    }
}

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
static int print_hmac_execution_path(const char *label,
                                     nvt_psa_hmac_path_t expected_path)
{
    const char *observed_path;

    switch (nvt_psa_hmac_last_compute_path)
    {
        case NVT_PSA_HMAC_PATH_HARDWARE:
            observed_path = "hardware";
            break;

        case NVT_PSA_HMAC_PATH_BUILTIN_FALLBACK:
            observed_path = "builtin fallback";
            break;

        case NVT_PSA_HMAC_PATH_SOFTWARE_FALLBACK:
            observed_path = "software fallback";
            break;

        default:
            observed_path = "failed";
            break;
    }

    if (nvt_psa_hmac_last_compute_path != expected_path)
    {
        printf("[FAIL] %s path: observed %s\n", label, observed_path);
        return 0;
    }

    printf("[PATH] %s: %s\n", label, observed_path);
    return 1;
}
#endif

int32_t RunSHA_BSP()
{
    uint32_t  u32TimeOutCnt;
    uint32_t  u32BytesPerLoop = 256;    // 2K Byte to save memory
    uint32_t  u32BytesTotal = MY_NUM;   // Size of u32buf, u32[640]
    uint32_t  u32FedIdx = 0;
    uint8_t   u8IsFirst = 1;
    uint32_t  hmac_ctl;

    while (u32BytesTotal >= u32BytesPerLoop)
    {

        hmac_ctl = CRPT_HMAC_CTL_DMACSCAD_Msk |
                   (SHA_MODE_SHA256 << CRPT_HMAC_CTL_OPMODE_Pos) |
                   CRPT_HMAC_CTL_INSWAP_Msk |
                   CRPT_HMAC_CTL_START_Msk | CRPT_HMAC_CTL_DMAEN_Msk;


        g_SHA_done = 0;

        /* Start SHA calculation */
        if (u8IsFirst)
        {
            hmac_ctl |= CRPT_HMAC_CTL_DMAFIRST_Msk;
            u8IsFirst = 0;
        }

        CRPT->HMAC_DMACNT = u32BytesPerLoop;
        CRPT->HMAC_SADDR = (uint32_t)(g_sha_msg + u32FedIdx);
        CRPT->HMAC_CTL = hmac_ctl;

        /* Waiting for SHA calcuation done */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (!g_SHA_done)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for SHA calcuation done time-out!\n");
                return (-1);
            }
        }

        u32BytesTotal -= (u32BytesPerLoop);
        u32FedIdx     += u32BytesPerLoop;
    }

    g_SHA_done = 0;
    /* Start SHA calculation */
    hmac_ctl = CRPT_HMAC_CTL_DMACSCAD_Msk |
               (SHA_MODE_SHA256 << CRPT_HMAC_CTL_OPMODE_Pos) |
               CRPT_HMAC_CTL_INSWAP_Msk |
               CRPT_HMAC_CTL_START_Msk | CRPT_HMAC_CTL_DMAEN_Msk;

    if (u8IsFirst)
    {
        hmac_ctl &= ~CRPT_HMAC_CTL_DMACSCAD_Msk;
        hmac_ctl |= CRPT_HMAC_CTL_DMALAST_Msk;
        u8IsFirst = 0;
    }
    else
    {
        hmac_ctl |= CRPT_HMAC_CTL_DMALAST_Msk;
    }

    CRPT->HMAC_DMACNT = u32BytesTotal;
    CRPT->HMAC_SADDR = (uint32_t)(g_sha_msg + u32FedIdx);
    CRPT->HMAC_CTL = hmac_ctl;

    /* Waiting for SHA calcuation done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    g_SHA_done = 0;

    while (!g_SHA_done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for SHA calcuation done time-out!\n");
            return (-1);
        }
    }

    /* Read SHA calculation result */
    SHA_Read(CRPT, au32OutputDigest);

    return 0;
}


static psa_status_t hash_a_message_nvt(uint8_t *digest, size_t *digest_length)
{
    psa_status_t status;
    psa_algorithm_t alg = PSA_ALG_SHA_256;
    static uint64_t ts_start, ts_end, ts0;

    ts_start = GetSysTickCycleCount();
    status = psa_hash_compute(alg,
                              g_sha_msg,
                              MY_NUM,
                              digest,
                              PSA_HASH_MAX_SIZE,
                              digest_length);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to compute hash operation: status=%ld sts=%08lx ctl=%08lx int=%08lx len=%lu\n",
               (long) status,
               (unsigned long) CRPT->HMAC_STS,
               (unsigned long) CRPT->HMAC_CTL,
               (unsigned long) CRPT->INTSTS,
               (unsigned long) MY_NUM);
        return status;
    }

    ts0 = GetSysTickCycleCount() - ts_start;
    DLOG("psa_hash_compute: %f us\n", (double)ts0 * 1000000.0 / SystemCoreClock);
    return PSA_SUCCESS;
}

static psa_status_t hash_a_message_nvt_native(uint8_t *digest,
                                              size_t *digest_length)
{
    psa_status_t status;
    psa_algorithm_t alg = PSA_ALG_SHA_256;
    static uint64_t ts_start, ts_end, ts0;

    ts_start = GetSysTickCycleCount();
    status = nvt_psa_sha_compute(alg,
                                 g_sha_msg,
                                 MY_NUM,
                                 digest,
                                 PSA_HASH_MAX_SIZE,
                                 digest_length);

    if (status != PSA_SUCCESS)
    {
        printf("Failed to compute hash operation: status=%ld sts=%08lx ctl=%08lx int=%08lx len=%lu\n",
               (long) status,
               (unsigned long) CRPT->HMAC_STS,
               (unsigned long) CRPT->HMAC_CTL,
               (unsigned long) CRPT->INTSTS,
               (unsigned long) MY_NUM);
        return status;
    }

    ts0 = GetSysTickCycleCount() - ts_start;
    DLOG("nvt_psa_sha_compute: %f us\n", (double)ts0 * 1000000.0 / SystemCoreClock);
    return PSA_SUCCESS;
}

void hash_a_message_nvt_oneshot(void)
{
    psa_status_t status;
    psa_algorithm_t alg = PSA_ALG_SHA_256;
    psa_hash_operation_t operation = PSA_HASH_OPERATION_INIT;
    size_t actual_hash_len;
    static uint64_t ts_start, ts_end, ts0;

    status = nvt_psa_sha_compute(alg,
                                 (uint8_t *)(u32buf),
                                 MY_NUM,
                                 actual_hash,
                                 sizeof(actual_hash),
                                 &actual_hash_len);

    printf("OS: actual_hash_len=%d \n", actual_hash_len);

    for (int i = 0; i < actual_hash_len; i++)
        printf("%02x", actual_hash[i]);

    printf("\n");
}

static int test_hmac_sha256(void)
{
    static const uint8_t key[20] =
    {
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b
    };
    static const uint8_t message[] = "Hi There";
    static const uint8_t expected[32] =
    {
        0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53,
        0x5c, 0xa8, 0xaf, 0xce, 0xaf, 0x0b, 0xf1, 0x2b,
        0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83, 0x3d, 0xa7,
        0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7
    };
    static const size_t test_lengths[] =
    {
        55u, 56u, 63u, 64u, 65u,
        255u, 256u, 257u,
        492u, 493u,
        1003u, 1004u, 1005u
    };
    uint8_t long_message[1005];
    uint8_t actual[32];
    uint8_t reference[32];
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    const mbedtls_md_info_t *md_info;
    size_t actual_length;
    size_t index;
    size_t byte_index;
    psa_status_t status;
    int result = 1;

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("[HMAC] PSA initialization failed: %ld\n", (long) status);
        return 1;
    }

    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE |
                            PSA_KEY_USAGE_VERIFY_MESSAGE);
    psa_set_key_algorithm(&attributes, PSA_ALG_HMAC(PSA_ALG_SHA_256));
    psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
    psa_set_key_bits(&attributes, sizeof(key) * 8u);
    status = psa_import_key(&attributes, key, sizeof(key), &key_id);
    psa_reset_key_attributes(&attributes);

    if (status != PSA_SUCCESS)
    {
        printf("[HMAC] Key import failed: %ld\n", (long) status);
        goto cleanup;
    }

    status = psa_mac_compute(key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256),
                             message, sizeof(message) - 1u,
                             actual, sizeof(actual), &actual_length);

    if (status != PSA_SUCCESS || actual_length != sizeof(expected) ||
            memcmp(actual, expected, sizeof(expected)) != 0)
    {
        printf("[HMAC] RFC 4231 test case 1 failed: status=%ld\n", (long) status);
        goto cleanup;
    }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

    if (!print_hmac_execution_path("HMAC-SHA-256 RFC case 1",
                                   NVT_PSA_HMAC_PATH_HARDWARE))
    {
        goto cleanup;
    }

#endif
    printf("[PASS] HMAC-SHA-256 RFC 4231 test case 1\n");

    for (index = 0u; index < sizeof(long_message); index++)
    {
        long_message[index] = (uint8_t)(index * 29u + 7u);
    }

    md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    if (md_info == NULL)
    {
        printf("[HMAC] Software reference unavailable\n");
        goto cleanup;
    }

    result = 0;

    for (index = 0u; index < sizeof(test_lengths) / sizeof(test_lengths[0]); index++)
    {
        size_t input_length = test_lengths[index];

        if (mbedtls_md_hmac(md_info, key, sizeof(key), long_message,
                            input_length, reference) != 0)
        {
            printf("[HMAC] Software reference failed for %lu bytes\n",
                   (unsigned long) input_length);
            goto cleanup;
        }

        actual_length = 0u;
        status = psa_mac_compute(key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256),
                                 long_message, input_length,
                                 actual, sizeof(actual), &actual_length);

        if (status != PSA_SUCCESS || actual_length != sizeof(reference) ||
                memcmp(actual, reference, sizeof(reference)) != 0)
        {
            printf("[HMAC-DEBUG] mismatch len=%lu status=%ld actual_len=%lu sts=%08lx ctl=%08lx int=%08lx\n",
                   (unsigned long) input_length, (long) status,
                   (unsigned long) actual_length,
                   (unsigned long) CRPT->HMAC_STS,
                   (unsigned long) CRPT->HMAC_CTL,
                   (unsigned long) CRPT->INTSTS);
            printf("[HMAC-DEBUG] actual=");

            for (byte_index = 0u; byte_index < sizeof(actual); byte_index++)
            {
                printf("%02x", actual[byte_index]);
            }

            printf("\n[HMAC-DEBUG] reference=");

            for (byte_index = 0u; byte_index < sizeof(reference); byte_index++)
            {
                printf("%02x", reference[byte_index]);
            }

            printf("\n");
            result = 1;
            continue;
        }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

        if (!print_hmac_execution_path("HMAC-SHA-256 input boundary",
                                       input_length >= 1004u ?
                                       NVT_PSA_HMAC_PATH_SOFTWARE_FALLBACK :
                                       NVT_PSA_HMAC_PATH_HARDWARE))
        {
            result = 1;
            continue;
        }

#endif

        printf("[PASS] HMAC-SHA-256 boundary len=%lu\n",
               (unsigned long) input_length);
    }

cleanup:

    if (key_id != PSA_KEY_ID_NULL)
    {
        psa_destroy_key(key_id);
    }

    mbedtls_psa_crypto_free();
    return result;
}

static int test_hmac_rfc_case1_all_sha(void)
{
    static const uint8_t key[20] =
    {
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b
    };
    static const uint8_t message[] = "Hi There";
    static const uint8_t expected_sha1[20] =
    {
        0xb6, 0x17, 0x31, 0x86, 0x55, 0x05, 0x72, 0x64, 0xe2, 0x8b,
        0xc0, 0xb6, 0xfb, 0x37, 0x8c, 0x8e, 0xf1, 0x46, 0xbe, 0x00
    };
    static const uint8_t expected_sha224[28] =
    {
        0x89, 0x6f, 0xb1, 0x12, 0x8a, 0xbb, 0xdf, 0x19, 0x68, 0x32,
        0x10, 0x7c, 0xd4, 0x9d, 0xf3, 0x3f, 0x47, 0xb4, 0xb1, 0x16,
        0x99, 0x12, 0xba, 0x4f, 0x53, 0x68, 0x4b, 0x22
    };
    static const uint8_t expected_sha256[32] =
    {
        0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53,
        0x5c, 0xa8, 0xaf, 0xce, 0xaf, 0x0b, 0xf1, 0x2b,
        0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83, 0x3d, 0xa7,
        0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7
    };
    static const uint8_t expected_sha384[48] =
    {
        0xaf, 0xd0, 0x39, 0x44, 0xd8, 0x48, 0x95, 0x62, 0x6b, 0x08, 0x25, 0xf4,
        0xab, 0x46, 0x90, 0x7f, 0x15, 0xf9, 0xda, 0xdb, 0xe4, 0x10, 0x1e, 0xc6,
        0x82, 0xaa, 0x03, 0x4c, 0x7c, 0xeb, 0xc5, 0x9c, 0xfa, 0xea, 0x9e, 0xa9,
        0x07, 0x6e, 0xde, 0x7f, 0x4a, 0xf1, 0x52, 0xe8, 0xb2, 0xfa, 0x9c, 0xb6
    };
    static const uint8_t expected_sha512[64] =
    {
        0x87, 0xaa, 0x7c, 0xde, 0xa5, 0xef, 0x61, 0x9d, 0x4f, 0xf0, 0xb4, 0x24,
        0x1a, 0x1d, 0x6c, 0xb0, 0x23, 0x79, 0xf4, 0xe2, 0xce, 0x4e, 0xc2, 0x78,
        0x7a, 0xd0, 0xb3, 0x05, 0x45, 0xe1, 0x7c, 0xde, 0xda, 0xa8, 0x33, 0xb7,
        0xd6, 0xb8, 0xa7, 0x02, 0x03, 0x8b, 0x27, 0x4e, 0xae, 0xa3, 0xf4, 0xe4,
        0xbe, 0x9d, 0x91, 0x4e, 0xeb, 0x61, 0xf1, 0x70, 0x2e, 0x69, 0x6c, 0x20,
        0x3a, 0x12, 0x68, 0x54
    };
    static const struct
    {
        psa_algorithm_t hash_alg;
        const char *name;
        const uint8_t *expected;
        size_t expected_length;
    } cases[] =
    {
        {PSA_ALG_SHA_1, "SHA-1", expected_sha1, sizeof(expected_sha1)},
        {PSA_ALG_SHA_224, "SHA-224", expected_sha224, sizeof(expected_sha224)},
        {PSA_ALG_SHA_256, "SHA-256", expected_sha256, sizeof(expected_sha256)},
        {PSA_ALG_SHA_384, "SHA-384", expected_sha384, sizeof(expected_sha384)},
        {PSA_ALG_SHA_512, "SHA-512", expected_sha512, sizeof(expected_sha512)}
    };
    uint8_t actual[PSA_HASH_MAX_SIZE];
    size_t actual_length;
    size_t index;
    psa_status_t status;

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("[HMAC] PSA initialization failed: %ld\n", (long) status);
        return 1;
    }

    for (index = 0u; index < sizeof(cases) / sizeof(cases[0]); index++)
    {
        psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
        psa_key_id_t key_id = PSA_KEY_ID_NULL;
        psa_algorithm_t alg = PSA_ALG_HMAC(cases[index].hash_alg);

        psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE |
                                PSA_KEY_USAGE_VERIFY_MESSAGE);
        psa_set_key_algorithm(&attributes,
                              PSA_ALG_AT_LEAST_THIS_LENGTH_MAC(alg, 16u));
        psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
        psa_set_key_bits(&attributes, sizeof(key) * 8u);
        status = psa_import_key(&attributes, key, sizeof(key), &key_id);
        psa_reset_key_attributes(&attributes);

        if (status == PSA_SUCCESS)
        {
            status = psa_mac_compute(key_id, alg,
                                     message, sizeof(message) - 1u,
                                     actual, sizeof(actual), &actual_length);
        }

        if (status != PSA_SUCCESS || actual_length != cases[index].expected_length ||
                memcmp(actual, cases[index].expected, cases[index].expected_length) != 0)
        {
            printf("[FAIL] HMAC-%s RFC case 1: status=%ld\n",
                   cases[index].name, (long) status);

            if (key_id != PSA_KEY_ID_NULL)
            {
                psa_destroy_key(key_id);
            }

            return 1;
        }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

        if (!print_hmac_execution_path(cases[index].name,
                                       NVT_PSA_HMAC_PATH_HARDWARE))
        {
            psa_destroy_key(key_id);
            return 1;
        }

#endif

        actual_length = 0u;
        status = psa_mac_compute(key_id,
                                 PSA_ALG_TRUNCATED_MAC(alg, 16u),
                                 message, sizeof(message) - 1u,
                                 actual, sizeof(actual), &actual_length);

        if (status != PSA_SUCCESS || actual_length != 16u ||
                memcmp(actual, cases[index].expected, 16u) != 0)
        {
            printf("[FAIL] HMAC-%s truncated RFC case 1: status=%ld\n",
                   cases[index].name, (long) status);
            psa_destroy_key(key_id);
            return 1;
        }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

        if (!print_hmac_execution_path(cases[index].name,
                                       NVT_PSA_HMAC_PATH_BUILTIN_FALLBACK))
        {
            psa_destroy_key(key_id);
            return 1;
        }

#endif
        printf("[PASS] HMAC-%s truncated RFC case 1\n", cases[index].name);

        actual_length = 0u;
        status = psa_mac_compute(key_id, alg,
                                 message, sizeof(message) - 1u,
                                 actual, cases[index].expected_length - 1u,
                                 &actual_length);

        if (status != PSA_ERROR_BUFFER_TOO_SMALL)
        {
            printf("[FAIL] HMAC-%s too-small output status=%ld\n",
                   cases[index].name, (long) status);
            psa_destroy_key(key_id);
            return 1;
        }

        printf("[PASS] HMAC-%s too-small output rejected\n", cases[index].name);

        {
            psa_mac_operation_t operation = PSA_MAC_OPERATION_INIT;

            status = psa_mac_sign_setup(&operation, key_id, alg);

            if (status == PSA_SUCCESS)
            {
                status = psa_mac_update(&operation, message, 2u);
            }

            if (status == PSA_SUCCESS)
            {
                status = psa_mac_update(&operation, message + 2u, 3u);
            }

            if (status == PSA_SUCCESS)
            {
                status = psa_mac_update(&operation, message + 5u,
                                        sizeof(message) - 1u - 5u);
            }

            if (status == PSA_SUCCESS)
            {
                actual_length = 0u;
                status = psa_mac_sign_finish(&operation, actual, sizeof(actual),
                                             &actual_length);
            }

            if (status != PSA_SUCCESS || actual_length != cases[index].expected_length ||
                    memcmp(actual, cases[index].expected,
                           cases[index].expected_length) != 0)
            {
                printf("[FAIL] HMAC-%s multipart RFC case 1: status=%ld\n",
                       cases[index].name, (long) status);
                psa_mac_abort(&operation);
                psa_destroy_key(key_id);
                return 1;
            }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
            printf("[PATH] HMAC-%s multipart: builtin (no NVT multipart entrypoint)\n",
                   cases[index].name);
#endif
            printf("[PASS] HMAC-%s multipart RFC case 1\n", cases[index].name);
        }

        printf("[PASS] HMAC-%s RFC case 1\n", cases[index].name);
        psa_destroy_key(key_id);
    }

    mbedtls_psa_crypto_free();
    return 0;
}

static int test_hmac_key_boundaries_all_sha(void)
{
    static const uint8_t long_message[] =
        "Test Using Larger Than Block-Size Key - Hash Key First";
    static const uint8_t expected_long_sha1[20] =
    {
        0xaa, 0x4a, 0xe5, 0xe1, 0x52, 0x72, 0xd0, 0x0e, 0x95, 0x70,
        0x56, 0x37, 0xce, 0x8a, 0x3b, 0x55, 0xed, 0x40, 0x21, 0x12
    };
    static const uint8_t expected_long_sha224[28] =
    {
        0x95, 0xe9, 0xa0, 0xdb, 0x96, 0x20, 0x95, 0xad, 0xae, 0xbe,
        0x9b, 0x2d, 0x6f, 0x0d, 0xbc, 0xe2, 0xd4, 0x99, 0xf1, 0x12,
        0xf2, 0xd2, 0xb7, 0x27, 0x3f, 0xa6, 0x87, 0x0e
    };
    static const uint8_t expected_long_sha256[32] =
    {
        0x60, 0xe4, 0x31, 0x59, 0x1e, 0xe0, 0xb6, 0x7f,
        0x0d, 0x8a, 0x26, 0xaa, 0xcb, 0xf5, 0xb7, 0x7f,
        0x8e, 0x0b, 0xc6, 0x21, 0x37, 0x28, 0xc5, 0x14,
        0x05, 0x46, 0x04, 0x0f, 0x0e, 0xe3, 0x7f, 0x54
    };
    static const uint8_t expected_long_sha384[48] =
    {
        0x4e, 0xce, 0x08, 0x44, 0x85, 0x81, 0x3e, 0x90, 0x88, 0xd2, 0xc6, 0x3a,
        0x04, 0x1b, 0xc5, 0xb4, 0x4f, 0x9e, 0xf1, 0x01, 0x2a, 0x2b, 0x58, 0x8f,
        0x3c, 0xd1, 0x1f, 0x05, 0x03, 0x3a, 0xc4, 0xc6, 0x0c, 0x2e, 0xf6, 0xab,
        0x40, 0x30, 0xfe, 0x82, 0x96, 0x24, 0x8d, 0xf1, 0x63, 0xf4, 0x49, 0x52
    };
    static const uint8_t expected_long_sha512[64] =
    {
        0x80, 0xb2, 0x42, 0x63, 0xc7, 0xc1, 0xa3, 0xeb, 0xb7, 0x14, 0x93, 0xc1,
        0xdd, 0x7b, 0xe8, 0xb4, 0x9b, 0x46, 0xd1, 0xf4, 0x1b, 0x4a, 0xee, 0xc1,
        0x12, 0x1b, 0x01, 0x37, 0x83, 0xf8, 0xf3, 0x52, 0x6b, 0x56, 0xd0, 0x37,
        0xe0, 0x5f, 0x25, 0x98, 0xbd, 0x0f, 0xd2, 0x21, 0x5d, 0x6a, 0x1e, 0x52,
        0x95, 0xe6, 0x4f, 0x73, 0xf6, 0x3f, 0x0a, 0xec, 0x8b, 0x91, 0x5a, 0x98,
        0x5d, 0x78, 0x65, 0x98
    };
    static const struct
    {
        psa_algorithm_t hash_alg;
        mbedtls_md_type_t md_type;
        const char *name;
        size_t block_size;
        size_t digest_size;
        size_t rfc_long_key_length;
        const uint8_t *rfc_long_expected;
    } cases[] =
    {
        {
            PSA_ALG_SHA_1, MBEDTLS_MD_SHA1, "SHA-1", 64u, 20u, 80u,
            expected_long_sha1
        },
        {
            PSA_ALG_SHA_224, MBEDTLS_MD_SHA224, "SHA-224", 64u, 28u, 131u,
            expected_long_sha224
        },
        {
            PSA_ALG_SHA_256, MBEDTLS_MD_SHA256, "SHA-256", 64u, 32u, 131u,
            expected_long_sha256
        },
        {
            PSA_ALG_SHA_384, MBEDTLS_MD_SHA384, "SHA-384", 128u, 48u, 131u,
            expected_long_sha384
        },
        {
            PSA_ALG_SHA_512, MBEDTLS_MD_SHA512, "SHA-512", 128u, 64u, 131u,
            expected_long_sha512
        }
    };
    uint8_t key[132];
    uint8_t actual[PSA_HASH_MAX_SIZE];
    uint8_t reference[PSA_HASH_MAX_SIZE];
    size_t case_index;
    psa_status_t status;

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("[HMAC] PSA initialization failed: %ld\n", (long) status);
        return 1;
    }

    for (case_index = 0u; case_index < sizeof(cases) / sizeof(cases[0]); case_index++)
    {
        const mbedtls_md_info_t *md_info =
            mbedtls_md_info_from_type(cases[case_index].md_type);
        size_t boundary_lengths[3] =
        {
            cases[case_index].block_size - 1u,
            cases[case_index].block_size,
            cases[case_index].block_size + 1u
        };
        size_t boundary_index;

        if (md_info == NULL)
        {
            printf("[HMAC] %s software reference unavailable\n",
                   cases[case_index].name);
            goto failure;
        }

        for (boundary_index = 0u; boundary_index < 3u; boundary_index++)
        {
            psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
            psa_key_id_t key_id = PSA_KEY_ID_NULL;
            size_t key_length = boundary_lengths[boundary_index];
            size_t actual_length = 0u;
            size_t byte_index;
            psa_algorithm_t alg = PSA_ALG_HMAC(cases[case_index].hash_alg);

            for (byte_index = 0u; byte_index < key_length; byte_index++)
            {
                key[byte_index] = (uint8_t)(byte_index * 13u + case_index + 1u);
            }

            psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE |
                                    PSA_KEY_USAGE_VERIFY_MESSAGE);
            psa_set_key_algorithm(&attributes, alg);
            psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
            psa_set_key_bits(&attributes, key_length * 8u);
            status = psa_import_key(&attributes, key, key_length, &key_id);
            psa_reset_key_attributes(&attributes);

            if (status != PSA_SUCCESS)
            {
                printf("[FAIL] HMAC-%s key import length=%lu status=%ld\n",
                       cases[case_index].name, (unsigned long) key_length,
                       (long) status);
                goto failure;
            }

            if (mbedtls_md_hmac(md_info, key, key_length,
                                long_message, sizeof(long_message) - 1u,
                                reference) != 0)
            {
                psa_destroy_key(key_id);
                printf("[HMAC] %s software reference failed\n",
                       cases[case_index].name);
                goto failure;
            }

            status = psa_mac_compute(key_id, alg,
                                     long_message, sizeof(long_message) - 1u,
                                     actual, sizeof(actual), &actual_length);
            psa_destroy_key(key_id);

            if (status != PSA_SUCCESS || actual_length != cases[case_index].digest_size ||
                    memcmp(actual, reference, cases[case_index].digest_size) != 0)
            {
                printf("[FAIL] HMAC-%s key boundary length=%lu status=%ld\n",
                       cases[case_index].name, (unsigned long) key_length,
                       (long) status);
                goto failure;
            }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

            if (!print_hmac_execution_path(cases[case_index].name,
                                           boundary_index == 2u ?
                                           NVT_PSA_HMAC_PATH_BUILTIN_FALLBACK :
                                           NVT_PSA_HMAC_PATH_HARDWARE))
            {
                goto failure;
            }

#endif

            printf("[PASS] HMAC-%s key boundary length=%lu\n",
                   cases[case_index].name, (unsigned long) key_length);
        }

        {
            psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
            psa_key_id_t key_id = PSA_KEY_ID_NULL;
            size_t actual_length = 0u;
            size_t byte_index;
            psa_algorithm_t alg = PSA_ALG_HMAC(cases[case_index].hash_alg);

            memset(key, 0xaa, cases[case_index].rfc_long_key_length);
            psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE |
                                    PSA_KEY_USAGE_VERIFY_MESSAGE);
            psa_set_key_algorithm(&attributes, alg);
            psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
            psa_set_key_bits(&attributes, cases[case_index].rfc_long_key_length * 8u);
            status = psa_import_key(&attributes, key,
                                    cases[case_index].rfc_long_key_length, &key_id);
            psa_reset_key_attributes(&attributes);

            if (status != PSA_SUCCESS)
            {
                printf("[FAIL] HMAC-%s RFC long-key import status=%ld\n",
                       cases[case_index].name, (long) status);
                goto failure;
            }

            status = psa_mac_compute(key_id, alg,
                                     long_message, sizeof(long_message) - 1u,
                                     actual, sizeof(actual), &actual_length);
            psa_destroy_key(key_id);

            if (status != PSA_SUCCESS || actual_length != cases[case_index].digest_size ||
                    memcmp(actual, cases[case_index].rfc_long_expected,
                           cases[case_index].digest_size) != 0)
            {
                printf("[FAIL] HMAC-%s RFC long-key vector status=%ld\n",
                       cases[case_index].name, (long) status);
                goto failure;
            }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

            if (!print_hmac_execution_path(cases[case_index].name,
                                           NVT_PSA_HMAC_PATH_BUILTIN_FALLBACK))
            {
                goto failure;
            }

#endif

            printf("[PASS] HMAC-%s RFC long-key vector\n",
                   cases[case_index].name);
        }
    }

    mbedtls_psa_crypto_free();
    return 0;

failure:
    mbedtls_psa_crypto_free();
    return 1;
}

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)
static int test_hmac_hardware_error_is_not_fallback(void)
{
    static const uint8_t key[20] =
    {
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b
    };
    static const uint8_t message[] = "Hi There";
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    uint8_t actual[PSA_HASH_MAX_SIZE];
    size_t actual_length = 0u;
    psa_status_t status;
    int result = 1;

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("[HMAC] PSA initialization failed: %ld\n", (long) status);
        return 1;
    }

    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attributes, PSA_ALG_HMAC(PSA_ALG_SHA_256));
    psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
    psa_set_key_bits(&attributes, sizeof(key) * 8u);
    status = psa_import_key(&attributes, key, sizeof(key), &key_id);
    psa_reset_key_attributes(&attributes);

    if (status != PSA_SUCCESS)
    {
        printf("[HMAC] Fault-test key import failed: %ld\n", (long) status);
        goto cleanup;
    }

    nvt_psa_test_force_hardware_failure = 1u;
    status = psa_mac_compute(key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256),
                             message, sizeof(message) - 1u,
                             actual, sizeof(actual), &actual_length);
    nvt_psa_test_force_hardware_failure = 0u;

    if (status != PSA_ERROR_HARDWARE_FAILURE ||
            nvt_psa_hmac_last_compute_path != NVT_PSA_HMAC_PATH_FAILED)
    {
        printf("[FAIL] HMAC hardware error was not propagated: status=%ld\n",
               (long) status);
        goto cleanup;
    }

    printf("[PASS] HMAC hardware error propagates without fallback\n");
    result = 0;

cleanup:
    nvt_psa_test_force_hardware_failure = 0u;

    if (key_id != PSA_KEY_ID_NULL)
    {
        psa_destroy_key(key_id);
    }

    mbedtls_psa_crypto_free();
    return result;
}
#endif

static int test_additional_sha_algorithms(void)
{
    static const uint8_t message[] = {'a', 'b', 'c'};
    static const uint8_t sha1_expected[20] =
    {
        0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81, 0x6a,
        0xba, 0x3e, 0x25, 0x71, 0x78, 0x50, 0xc2, 0x6c,
        0x9c, 0xd0, 0xd8, 0x9d
    };
    static const uint8_t sha224_expected[28] =
    {
        0x23, 0x09, 0x7d, 0x22, 0x34, 0x05, 0xd8, 0x22,
        0x86, 0x42, 0xa4, 0x77, 0xbd, 0xa2, 0x55, 0xb3,
        0x2a, 0xad, 0xbc, 0xe4, 0xbd, 0xa0, 0xb3, 0xf7,
        0xe3, 0x6c, 0x9d, 0xa7
    };
    static const uint8_t sha384_expected[48] =
    {
        0xcb, 0x00, 0x75, 0x3f, 0x45, 0xa3, 0x5e, 0x8b,
        0xb5, 0xa0, 0x3d, 0x69, 0x9a, 0xc6, 0x50, 0x07,
        0x27, 0x2c, 0x32, 0xab, 0x0e, 0xde, 0xd1, 0x63,
        0x1a, 0x8b, 0x60, 0x5a, 0x43, 0xff, 0x5b, 0xed,
        0x80, 0x86, 0x07, 0x2b, 0xa1, 0xe7, 0xcc, 0x23,
        0x58, 0xba, 0xec, 0xa1, 0x34, 0xc8, 0x25, 0xa7
    };
    static const uint8_t sha1_empty[20] =
    {
        0xda, 0x39, 0xa3, 0xee, 0x5e, 0x6b, 0x4b, 0x0d,
        0x32, 0x55, 0xbf, 0xef, 0x95, 0x60, 0x18, 0x90,
        0xaf, 0xd8, 0x07, 0x09
    };
    static const uint8_t sha224_empty[28] =
    {
        0xd1, 0x4a, 0x02, 0x8c, 0x2a, 0x3a, 0x2b, 0xc9,
        0x47, 0x61, 0x02, 0xbb, 0x28, 0x82, 0x34, 0xc4,
        0x15, 0xa2, 0xb0, 0x1f, 0x82, 0x8e, 0xa6, 0x2a,
        0xc5, 0xb3, 0xe4, 0x2f
    };
    static const uint8_t sha384_empty[48] =
    {
        0x38, 0xb0, 0x60, 0xa7, 0x51, 0xac, 0x96, 0x38,
        0x4c, 0xd9, 0x32, 0x7e, 0xb1, 0xb1, 0xe3, 0x6a,
        0x21, 0xfd, 0xb7, 0x11, 0x14, 0xbe, 0x07, 0x43,
        0x4c, 0x0c, 0xc7, 0xbf, 0x63, 0xf6, 0xe1, 0xda,
        0x27, 0x4e, 0xde, 0xbf, 0xe7, 0x6f, 0x65, 0xfb,
        0xd5, 0x1a, 0xd2, 0xf1, 0x48, 0x98, 0xb9, 0x5b
    };
    static const struct
    {
        psa_algorithm_t alg;
        mbedtls_md_type_t md_type;
        const char *name;
        const uint8_t *expected;
        size_t expected_length;
        const uint8_t *empty_expected;
    } cases[] =
    {
        {PSA_ALG_SHA_1, MBEDTLS_MD_SHA1, "SHA-1", sha1_expected, sizeof(sha1_expected), sha1_empty},
        {PSA_ALG_SHA_224, MBEDTLS_MD_SHA224, "SHA-224", sha224_expected, sizeof(sha224_expected), sha224_empty},
        {PSA_ALG_SHA_384, MBEDTLS_MD_SHA384, "SHA-384", sha384_expected, sizeof(sha384_expected), sha384_empty}
    };
    uint8_t actual[PSA_HASH_MAX_SIZE];
    uint8_t software[PSA_HASH_MAX_SIZE];
    const mbedtls_md_info_t *md_info;
    size_t actual_length;
    size_t index;
    psa_status_t status;

    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("[SHA] PSA initialization failed: %ld\n", (long) status);
        return 1;
    }

    for (index = 0u; index < sizeof(cases) / sizeof(cases[0]); index++)
    {
        psa_hash_operation_t operation = PSA_HASH_OPERATION_INIT;

        status = psa_hash_compute(cases[index].alg, NULL, 0u, actual,
                                  sizeof(actual), &actual_length);

        if (status != PSA_SUCCESS || actual_length != cases[index].expected_length ||
                memcmp(actual, cases[index].empty_expected, actual_length) != 0)
        {
            printf("[FAIL] %s PSA empty-message vector: %ld\n",
                   cases[index].name, (long) status);
            goto failure;
        }

        status = psa_hash_compute(cases[index].alg, message, sizeof(message),
                                  actual, sizeof(actual), &actual_length);

        if (status != PSA_SUCCESS || actual_length != cases[index].expected_length ||
                memcmp(actual, cases[index].expected, actual_length) != 0)
        {
            printf("[FAIL] %s PSA one-shot known vector: %ld\n",
                   cases[index].name, (long) status);
            goto failure;
        }

        status = psa_hash_setup(&operation, cases[index].alg);

        if (status == PSA_SUCCESS)
        {
            status = psa_hash_update(&operation, message, 1u);
        }

        if (status == PSA_SUCCESS)
        {
            status = psa_hash_update(&operation, message + 1u, 2u);
        }

        if (status == PSA_SUCCESS)
        {
            status = psa_hash_finish(&operation, actual, sizeof(actual),
                                     &actual_length);
        }

        if (status != PSA_SUCCESS || actual_length != cases[index].expected_length ||
                memcmp(actual, cases[index].expected, actual_length) != 0)
        {
            printf("[FAIL] %s PSA multipart known vector: %ld\n",
                   cases[index].name, (long) status);
            psa_hash_abort(&operation);
            goto failure;
        }

        psa_hash_abort(&operation);

        md_info = mbedtls_md_info_from_type(cases[index].md_type);

        if (md_info == NULL ||
                mbedtls_md(md_info, (const uint8_t *) u32buf, 257u, software) != 0)
        {
            printf("[FAIL] %s software reference unavailable\n", cases[index].name);
            goto failure;
        }

        operation = (psa_hash_operation_t) PSA_HASH_OPERATION_INIT;
        status = psa_hash_setup(&operation, cases[index].alg);

        if (status == PSA_SUCCESS)
        {
            status = psa_hash_update(&operation, (const uint8_t *) u32buf, 13u);
        }

        if (status == PSA_SUCCESS)
        {
            status = psa_hash_update(&operation, (const uint8_t *) u32buf + 13u, 243u);
        }

        if (status == PSA_SUCCESS)
        {
            status = psa_hash_update(&operation, (const uint8_t *) u32buf + 256u, 1u);
        }

        if (status == PSA_SUCCESS)
        {
            status = psa_hash_finish(&operation, actual, sizeof(actual),
                                     &actual_length);
        }

        if (status != PSA_SUCCESS || actual_length != cases[index].expected_length ||
                memcmp(actual, software, actual_length) != 0)
        {
            printf("[FAIL] %s PSA multipart 257-byte comparison: %ld\n",
                   cases[index].name, (long) status);
            psa_hash_abort(&operation);
            goto failure;
        }

        psa_hash_abort(&operation);
        printf("[PASS] %s PSA known-vector and multipart checks\n", cases[index].name);
    }

    mbedtls_psa_crypto_free();
    return 0;

failure:
    mbedtls_psa_crypto_free();
    return 1;
}

/*-----------------------------------------------------------------------------*/
int main(void)
{
    static const size_t hash_test_lengths[] = {255u, 256u, 2433u, 2497u};
    static uint64_t tb_start, tb_end, tb0;
    static uint64_t tp_start, tp_end, tp0;
    uint8_t psa_digest[PSA_HASH_MAX_SIZE];
    uint8_t native_digest[PSA_HASH_MAX_SIZE];
    size_t psa_digest_length;
    size_t native_digest_length;
    size_t sample_index;
    psa_status_t status;
    int psa_used_hardware;
    int native_used_hardware;

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    /* Init Systick for printf */
    InitSysTick();

    printf("+-----------------------------------+\n");
    printf("|      MbedTLS TF_PSA_HASH Demo     |\n");
    printf("+-----------------------------------+\n");

    if (test_additional_sha_algorithms() != 0)
    {
        while (1)
        {
        }
    }

    if (test_hmac_sha256() != 0)
    {
        while (1)
        {
        }
    }

    if (test_hmac_rfc_case1_all_sha() != 0)
    {
        while (1)
        {
        }
    }

    if (test_hmac_key_boundaries_all_sha() != 0)
    {
        while (1)
        {
        }
    }

#if defined(MBEDTLS_PSA_NVT_TEST_HOOKS)

    if (test_hmac_hardware_error_is_not_fallback() != 0)
    {
        while (1)
        {
        }
    }

#endif

    /* Initialize PSA once so the loop only times the hash itself, not setup. */
    status = psa_crypto_init();

    if (status != PSA_SUCCESS)
    {
        printf("Failed to initialize PSA Crypto: %ld\n", (long) status);

        while (1) {}
    }

    for (sample_index = 0u;
            sample_index < sizeof(hash_test_lengths) / sizeof(hash_test_lengths[0]);
            sample_index++)
    {
        size_t kk = hash_test_lengths[sample_index];
        int require_hardware = (kk <= 256u);

        MY_NUM = kk;
        g_sha_msg = g_buff_pool;

        for (size_t fill_idx = 0u; fill_idx < BUFF_SIZE; fill_idx++)
        {
            g_sha_msg[fill_idx] = (uint8_t)(fill_idx * 29u + 7u);
        }

        NVIC_EnableIRQ(CRPT_IRQn);
        SHA_ENABLE_INT(CRPT);

        /* PSA standard API: now dispatches to nvt HW via psa_crypto_driver_wrappers.h */
        printf("\n+-------PSA INFO-----------+\n");
        tp_start = GetSysTickCycleCount();
        status = hash_a_message_nvt(psa_digest, &psa_digest_length);
        tp0 = GetSysTickCycleCount() - tp_start;

        if (status != PSA_SUCCESS)
        {
            printf("PSA SHA-256 failed: %ld\n", (long) status);

            while (1) {}
        }

        psa_used_hardware = print_sha_execution_path("PSA SHA-256");
#ifdef CHECK_OVERALL_TIME
        print_elapsed_time_us("TEST done - PSA", tp0);
#endif

        /* Native nvt_psa_* API: direct call to the M2354 Crypto HW driver */
        printf("\n+-------NATIVE PSA---------+\n");
        tp_start = GetSysTickCycleCount();
        status = hash_a_message_nvt_native(native_digest, &native_digest_length);
        tp0 = GetSysTickCycleCount() - tp_start;

        if (status != PSA_SUCCESS)
        {
            printf("Native SHA-256 failed: %ld\n", (long) status);

            while (1) {}
        }

        native_used_hardware = print_sha_execution_path("Native SHA-256");
#ifdef CHECK_OVERALL_TIME
        print_elapsed_time_us("TEST done - PSA", tp0);
#endif

        /* BSP direct register access: M2354 Crypto HW */
        printf("\n+-------BSP INFO---------+\n");
        tb_start = GetSysTickCycleCount();
        RunSHA_BSP();
        tb0 = GetSysTickCycleCount() - tb_start;
#ifdef CHECK_OVERALL_TIME
        print_elapsed_time_us("TEST done - BSP", tb0);
#endif

        NVIC_DisableIRQ(CRPT_IRQn);
        SHA_DISABLE_INT(CRPT);

        if (psa_digest_length != 32u || native_digest_length != 32u)
        {
            printf("SHA digest length mismatch.\n");

            while (1) {}
        }

        /* Built-in pure-software SHA reference (no HW) */
        printf("\n+------BUILT-IN--------+\n");
        unsigned char sw_hash[32];
        tb_start = GetSysTickCycleCount();
        mbedtls_sha256((const unsigned char *)(g_sha_msg), MY_NUM, sw_hash, 0);
        tb0 = GetSysTickCycleCount() - tb_start;
#ifdef CHECK_OVERALL_TIME
        print_elapsed_time_us("TEST done - BUILT-IN SW SHA-256", tb0);
#endif

        /*
           Data format preparation for comparing: psa/psa-native and nuilt-in sw.
           To fit bsp SHA_Read u32 format.
        */

        if (memcmp(psa_digest, sw_hash, 32u) != 0)
        {
            printf("PSA SHA test fail.\n");

            while (1) {}
        }

        if (memcmp(native_digest, sw_hash, 32u) != 0)
        {
            printf("Native SHA test fail.\n");

            while (1) {}
        }

        if (require_hardware && (!psa_used_hardware || !native_used_hardware))
        {
            printf("[FAIL] PSA and Native must complete this short test in hardware.\n");

            while (1) {}
        }

        printf("SHA test Pass.len=%lu\n", (unsigned long) kk);

    }

    mbedtls_psa_crypto_free();

    while (1);
}
