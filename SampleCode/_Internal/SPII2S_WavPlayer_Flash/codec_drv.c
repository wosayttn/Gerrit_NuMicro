/**************************************************************************//**
 * @file        codec_drv.c
 * @brief       Codec driver
 *
 * @note
 * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "codec_drv.h"

#define NAU8822_ADDR    0x1A

void Delay(uint32_t count)
{
    volatile uint32_t i;

    for (i = 0; i < count ; i++);
}

void I2C_WriteNAU8822(uint8_t u8addr, uint16_t u16data)
{

    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, NAU8822_ADDR << 1);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    I2C_STOP(I2C0);
}

void NAU8822_Setup(uint32_t u32SampleRate, uint32_t u32AudioFormat)
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

    I2C_WriteNAU8822(1,  0x03F);   /* Codec is Master; enable internal PLL , enable micbias */
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */

    if (u32AudioFormat == SPII2S_MONO)
        I2C_WriteNAU8822(4,  0x011);   /* 16-bit word length, I2S format, Mono */
    else
        I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format */

    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */

    switch (u32SampleRate)
    {
        case 48000:
            I2C_WriteNAU8822(6, 0x14D);     /* Divide by 2, 48K */
            I2C_WriteNAU8822(7, 0x000);     /* 48K for internal filter cofficients */
            break;

        case 32000:
            I2C_WriteNAU8822(6, 0x16D);     /* Divide by 3, 32K */
            I2C_WriteNAU8822(7, 0x002);     /* 32 for internal filter cofficients */
            break;

        case 24000:
            I2C_WriteNAU8822(6, 0x18D);     /* Divide by 4, 24K */
            I2C_WriteNAU8822(7, 0x004);     /* 24K for internal filter cofficients */
            break;

        case 16000:
            I2C_WriteNAU8822(6, 0x1AD);   /* Divide by 6, 16K */
            I2C_WriteNAU8822(7, 0x006);   /* 16K for internal filter coefficients */
            break;

        case 12000:
            I2C_WriteNAU8822(6, 0x1CD);     /* Divide by 8, 12K */
            I2C_WriteNAU8822(7, 0x008);     /* 12K for internal filter cofficients */
            break;

        case 8000:
        default:
            I2C_WriteNAU8822(6, 0x1ED);     /* Divide by 12, 8K */
            I2C_WriteNAU8822(7, 0x00A);     /* 8K for internal filter cofficients */
    }

    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1FF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1FF);   /* ADC right digital volume control */
    I2C_WriteNAU8822(44, 0x000);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */

    printf("[OK]\n");
}

