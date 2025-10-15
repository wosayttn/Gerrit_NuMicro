/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use Helium technology to improve JPEG encoding performance.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/**
 * Define
 */
#define PATH_IMAGE_LENNA  "../lenna.rgb"
#define PATH_IMAGE_RESOLUTION_WIDTH    512
#define PATH_IMAGE_RESOLUTION_HEIGHT   512
#define DEF_ENCODE_TIME                1

#define STR2(x) #x  // Convert the macro argument 'x' into a string literal.
#define STR(x) STR2(x)  // Call STR2 to ensure that the argument is converted into a string.
#define INCBIN(name, file) \
    __asm__(".section .rodata\n" \
            ".global incbin_" STR(name) "_start\n" \
            ".balign 16\n" \
            "incbin_" STR(name) "_start:\n" \
            ".incbin \"" file "\"\n" \
            \
            ".global incbin_" STR(name) "_end\n" \
            ".balign 1\n" \
            "incbin_" STR(name) "_end:\n" \
            ".byte 0\n" \
           ); \
    extern const __attribute__((aligned(32))) void* incbin_ ## name ## _start; \
    extern const void* incbin_ ## name ## _end; \


/**
 * Global variables
 */
INCBIN(img_rgb888, PATH_IMAGE_LENNA);  // Include binary data for lena image from the specified path.
NVT_NOINIT static uint8_t g_au8RGB888ImageBuffer [PATH_IMAGE_RESOLUTION_WIDTH * PATH_IMAGE_RESOLUTION_HEIGHT * 3] __ALIGNED(DCACHE_LINE_SIZE);
NVT_NOINIT static uint8_t g_au8JpegBuffer [PATH_IMAGE_RESOLUTION_WIDTH * PATH_IMAGE_RESOLUTION_HEIGHT / 2] __ALIGNED(DCACHE_LINE_SIZE);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

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
static uint64_t GetSysTickCycleCount(void)
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

/**
 *  Main Function
 */
void JpegEncode(unsigned char *image, unsigned char *jBuf, unsigned long *jSize, int width, int height);
int32_t main(void)
{
    int i = 0;
    uint64_t start, elapsed;
    unsigned long u32JpegByteSize;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for print message */
    InitDebugUart();

    InitSysTick();

    printf("+---------------------------------------------+\n");
#ifdef WITH_JPEGACC
    printf("M55M1 JPEG Encode with JPEG accleration\n");
#else
    printf("M55M1 JPEG Encode without JPEG accleration\n");
#endif
    printf("+---------------------------------------------+\n");

    memcpy(g_au8RGB888ImageBuffer, (const uint8_t *)&incbin_img_rgb888_start, sizeof(g_au8RGB888ImageBuffer));

    start = GetSysTickCycleCount();

    for (i = 0; i < DEF_ENCODE_TIME; i++)
    {
        u32JpegByteSize = sizeof(g_au8RGB888ImageBuffer);
        JpegEncode((unsigned char *)g_au8RGB888ImageBuffer, g_au8JpegBuffer, &u32JpegByteSize, PATH_IMAGE_RESOLUTION_WIDTH, PATH_IMAGE_RESOLUTION_HEIGHT);
        SCB_CleanDCache_by_Addr(g_au8JpegBuffer, u32JpegByteSize);
    }

    elapsed = GetSysTickCycleCount() - start;

    printf("%d times JPEG Encoding: %llu cycles, Elapsed time: %.2f ms, Avg: %.2f ms(%u Hz)\n",
           DEF_ENCODE_TIME,
           elapsed,
           (double)elapsed * 1000.0 / SystemCoreClock,
           (double)elapsed * 1000.0 / SystemCoreClock / DEF_ENCODE_TIME,
           SystemCoreClock);


    /* Forces a write of all user-space buffered data for the given output */
    fflush(stdout);

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
