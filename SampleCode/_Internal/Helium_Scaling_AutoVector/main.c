/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Vectorize Image Scaling with Helium.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "scaling.h"


/****************************************************************************
 * User Define Macro
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/
volatile uint32_t g_u32Ticks = 0;
uint8_t image_src[SRC_IMG_WIDTH * SRC_IMG_HEIGHT * RGB_BYTES] = {0};
uint8_t image_dst[SCALED_IMG_WIDTH * SCALED_IMG_HEIGHT * RGB_BYTES] = {0};

/****************************************************************************
 * Function Proto Type
 ****************************************************************************/
void SYS_Init(void);


void SysTick_Handler(void)
{
    /* Increment the cycle counter based on load value. */
    g_u32Ticks += SysTick->LOAD + 1;
}


int Init_SysTick_Export(void)
{
    const uint32_t ticks_10ms = SystemCoreClock / 100 + 1;
    int err = 0;

    /* Reset CPU cycle count value. */
    g_u32Ticks = 0;

    /* Changing configuration for sys tick => guard from being
     * interrupted. */
    NVIC_DisableIRQ(SysTick_IRQn);

    /* SysTick init - this will enable interrupt too. */
    err = SysTick_Config(ticks_10ms);

    /* Enable interrupt again. */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Wait for SysTick to kick off */
    while (!err && !SysTick->VAL)
    {
        __NOP();
    }

    return err;
}

/**
 * Gets the current SysTick derived counter value
 */
static uint64_t Get_SysTick_Cycle_Count(void)
{
    uint32_t systick_val;

    NVIC_DisableIRQ(SysTick_IRQn);
    systick_val = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);

    return g_u32Ticks + (SysTick->LOAD - systick_val);
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeVal1, u32TimeVal2;
    uint32_t ii;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    printf("+---------------------------------------+\n");
    printf("|     Helium Bilinear Scaling           |\n");
    printf("|     [%d x %d] to [%d x %d]        |\n", SRC_IMG_WIDTH, SRC_IMG_HEIGHT, SCALED_IMG_WIDTH, SCALED_IMG_HEIGHT);
    printf("+---------------------------------------+\n");

    /* Use systick to measure inference time */

    for (ii = 0; ii < SRC_IMG_WIDTH * SRC_IMG_HEIGHT; ii++)
    {
        image_src[ii] = (uint8_t)(ii);
        image_src[SRC_IMG_WIDTH * SRC_IMG_HEIGHT * 1 + ii] = (uint8_t)(ii);
        image_src[SRC_IMG_WIDTH * SRC_IMG_HEIGHT * 2 + ii] = (uint8_t)(ii);
    }

    while (1)
    {
        printf("\n Execute Resize Function\n");
        Init_SysTick_Export();
        /*
           The bilinear scaling source code is originating from Edge Impulse inferencing library.
           https://edgeimpulse.com/
        */
        //Start of resize function++++
        int FRAC_BITS = 14;
        int FRAC_VAL = (1 << FRAC_BITS);
        int FRAC_MASK = (FRAC_VAL - 1);

        uint32_t src_x_accum, src_y_accum; // accumulators and fractions for scaling the image
        uint32_t x_frac, nx_frac, y_frac, ny_frac;
        uint32_t x, y;
        int ty;
        uint32_t srcWidth = SRC_IMG_WIDTH;
        uint32_t srcHeight = SRC_IMG_HEIGHT;
        uint32_t dstWidth = SCALED_IMG_WIDTH;
        uint32_t dstHeight = SCALED_IMG_HEIGHT;
        uint32_t pixel_size_B = RGB_BYTES;
        /*
           These parameters run a 320x240(QVGA) to 128x128 scaling down.
           https://edgeimpulse.com/
        */

        // start at 1/2 pixel in to account for integer downsampling which might miss pixels
        src_y_accum = FRAC_VAL / 2;
        uint32_t src_x_frac = (srcWidth * FRAC_VAL) / dstWidth;
        uint32_t src_y_frac = (srcHeight * FRAC_VAL) / dstHeight;

        //from here out, *3 b/c RGB
        srcWidth *= pixel_size_B;
        //srcHeight not used for indexing
        //dstWidth still needed as is
        //dstHeight shouldn't be scaled

        uint8_t *s;
        uint8_t *d;

        for (y = 0; y < dstHeight; y++)
        {
            // do indexing computations
            ty = src_y_accum >> FRAC_BITS; // src y
            y_frac = src_y_accum & FRAC_MASK;
            src_y_accum += src_y_frac;
            ny_frac = FRAC_VAL - y_frac; // y fraction and 1.0 - y fraction

            s = &image_src[ty * srcWidth];
            d = &image_dst[y * dstWidth * pixel_size_B]; //not scaled above
            // start at 1/2 pixel in to account for integer downsampling which might miss pixels
            src_x_accum = FRAC_VAL / 2;

            for (x = 0; x < dstWidth; x++)
            {
                uint32_t tx, p00, p01, p10, p11;
                // do indexing computations
                tx = (src_x_accum >> FRAC_BITS) * pixel_size_B;
                x_frac = src_x_accum & FRAC_MASK;
                nx_frac = FRAC_VAL - x_frac; // x fraction and 1.0 - x fraction
                src_x_accum += src_x_frac;

                //interpolate and write out
                for (uint32_t color = 0; color < pixel_size_B;
                        color++) // do pixel_size_B times for pixel_size_B colors
                {
                    p00 = s[tx];
                    p10 = s[tx + pixel_size_B];
                    p01 = s[tx + srcWidth];
                    p11 = s[tx + srcWidth + pixel_size_B];
                    p00 = ((p00 * nx_frac) + (p10 * x_frac) + FRAC_VAL / 2) >> FRAC_BITS; // top line
                    p01 = ((p01 * nx_frac) + (p11 * x_frac) + FRAC_VAL / 2) >> FRAC_BITS; // bottom line
                    p00 = ((p00 * ny_frac) + (p01 * y_frac) + FRAC_VAL / 2) >> FRAC_BITS; //top + bottom
                    *d++ = (uint8_t)p00; // store new pixel
                    //ready next loop
                    tx++;
                }
            } // for x
        } // for y

        //End of resize function----

        u32TimeVal1 = Get_SysTick_Cycle_Count();
        printf("\n With -o2  optimize, use %d Tick Counters.\n", u32TimeVal1);

        /* Run an identical code with different optimization level, not vectorized flow*/
        Init_SysTick_Export();
        scaling_nonvectorize();
        u32TimeVal2 = Get_SysTick_Cycle_Count();
        printf("\n With -o0  optimize, use %d Tick Counters. \n", u32TimeVal2);

        printf("\n Helium Scaling Speed up %.2f times.\n", (float)(u32TimeVal2) / (float)(u32TimeVal1));

        while (1);
    }
}
