/**************************************************************************//**
 * @file    scaling.c
 * @version V1.00
 * @brief   Image scaling test code(bilinear scale)
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.

 *****************************************************************************/
/*
       The bilinear scaling source code is originating from Edge Impulse inferencing library.
       https://edgeimpulse.com/
*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "scaling.h"

/*---------------------------------------------------------------------------------------------------------*/
/*  Scaling Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
int32_t scaling_nonvectorize(void)
{

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
    return 0;
}
