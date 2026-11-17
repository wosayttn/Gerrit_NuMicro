/***************************************************************************//**
 * @file    audio_codec.c
 * @version V1.00
 * @brief   Audio codec setting.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"

#if NAU8822
    void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data);
    void NAU8822_Setup(void);
#else
    uint8_t I2C_WriteMultiByteforNAU88L25(uint8_t u8ChipAddr, uint16_t u16SubAddr, const uint8_t *p, uint32_t u32Len);
    uint8_t I2C_WriteNAU88L25(uint16_t u16Addr, uint16_t u16Dat);
    void NAU88L25_Reset(void);
    void NAU88L25_Setup(void);
#endif

#if NAU8822

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C                                         */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief    Write a 9-bit data value to a 7-bit address register of the NAU8822 audio codec via I2C.
 *
 * @return   None
 *
 * @details  This function sends a 9-bit data value to a specified register
 *           of the NAU8822 audio codec using the I2C interface.
 */
void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data)
{
    I2C_START(I2C_PORT);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, 0x1A << 1);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, (uint8_t)((u8Addr << 1) | (u16Data >> 8)));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, (uint8_t)(u16Data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_STOP(I2C_PORT);
}

/* Config play sampling rate */
/**
 * @brief    Configure the sampling rate of the NAU8822 audio codec via I2C.
 *
 * @return   None
 *
 * @details  This function sets the internal PLL and clock dividers of the NAU8822
 *           audio codec to achieve the desired sampling rate.
 */
void NAU8822_ConfigSampleRate(uint32_t u32SampleRate)
{
    printf("[NAU8822] Configure Sampling Rate to %d\n", u32SampleRate);

    if ((u32SampleRate % 8) == 0)
    {
        I2C_WriteNAU8822(36, 0x008); //12.288Mhz
        I2C_WriteNAU8822(37, 0x00C);
        I2C_WriteNAU8822(38, 0x093);
        I2C_WriteNAU8822(39, 0x0E9);
    }
    else
    {
        I2C_WriteNAU8822(36, 0x007); //11.2896Mhz
        I2C_WriteNAU8822(37, 0x021);
        I2C_WriteNAU8822(38, 0x161);
        I2C_WriteNAU8822(39, 0x026);
    }

    switch (u32SampleRate)
    {
        case 16000:
            I2C_WriteNAU8822(6, 0x1AD); /* Divide by 6, 16K */
            I2C_WriteNAU8822(7, 0x006); /* 16K for internal filter coefficients */
            break;

        case 44100:
            I2C_WriteNAU8822(6, 0x14D); /* Divide by 2, 48K */
            I2C_WriteNAU8822(7, 0x000); /* 48K for internal filter coefficients */
            break;

        case 48000:
            I2C_WriteNAU8822(6, 0x14D); /* Divide by 2, 48K */
            I2C_WriteNAU8822(7, 0x000); /* 48K for internal filter coefficients */
            break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief    Configure the NAU8822 audio codec via I2C.
 *
 * @return   None
 *
 * @details  This function configures the NAU8822 audio codec with a series of I2C commands.
 *           It sets up the codec for audio playback and recording, including enabling the necessary
 *           features and setting the appropriate audio formats.
 */
void NAU8822_Setup(void)
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000); /* Reset all registers */
    CLK_SysTickDelay(10000);

    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3); /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F); /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010); /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000); /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x14D); /* Divide by 2, 48K */
    I2C_WriteNAU8822(7,  0x000); /* 48K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008); /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108); /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF); /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF); /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000); /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x050); /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x050); /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001); /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001); /* Right DAC connected to RMIX */

    printf("[OK]\n");
}

#else   // NAU88L25

/**
 * @brief      Write multiple bytes to the NAU88L25 audio codec via I2C.
 *
 * @param[in]  u8ChipAddr The I2C address of the NAU88L25 audio codec.
 * @param[in]  u16SubAddr The register address to write to.
 * @param[in]  p          Pointer to the data buffer containing the bytes to write.
 * @param[in]  u32Len     The number of bytes to write.
 *
 * @return     0 if successful, otherwise an error code.
 *
 * @details    This function sends multiple bytes of data to a specified register
 *             of the NAU88L25 audio codec using the I2C interface.
 */
uint8_t I2C_WriteMultiByteforNAU88L25(uint8_t u8ChipAddr, uint16_t u16SubAddr, const uint8_t *p, uint32_t u32Len)
{
    (void)u32Len;

    /* Send START */
    I2C_START(I2C_PORT);
    I2C_WAIT_READY(I2C_PORT);

    /* Send device address */
    I2C_SET_DATA(I2C_PORT, u8ChipAddr);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C_PORT, (uint8_t)(u16SubAddr >> 8));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C_PORT, (uint8_t)(u16SubAddr & 0x00FF));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send data */
    I2C_SET_DATA(I2C_PORT, p[0]);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send data */
    I2C_SET_DATA(I2C_PORT, p[1]);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send STOP */
    I2C_STOP(I2C_PORT);

    return  0;
}

/**
 * @brief      Write a single 16-bit value to a register of the NAU88L25 audio codec via I2C.
 *
 * @param[in]  u16Addr The 16-bit register address of the NAU88L25.
 * @param[in]  u16Dat  The 16-bit data to be written to the register.
 *
 * @return     0 if successful, otherwise an error code.
 */
uint8_t I2C_WriteNAU88L25(uint16_t u16Addr, uint16_t u16Dat)
{
    uint8_t u8TxData0[2];

    u8TxData0[0] = (uint8_t)(u16Dat >> 8);
    u8TxData0[1] = (uint8_t)(u16Dat & 0x00FF);

    return (I2C_WriteMultiByteforNAU88L25(0x1A << 1, u16Addr, &u8TxData0[0], 2));
}

/* Config play sampling rate */
/**
 * @brief      Configure the sampling rate of the NAU88L25 audio codec via I2C.
 *
 * @param[in]  u32SampleRate The desired sampling rate in Hz.
 *
 * @return     None
 *
 * @details    This function sets the internal PLL and clock dividers of the NAU88L25
 */
void NAU88L25_ConfigSampleRate(uint32_t u32SampleRate)
{
    printf("[NAU88L25] Configure Sampling Rate to %d\n", u32SampleRate);

    if ((u32SampleRate % 8) == 0)
    {
        I2C_WriteNAU88L25(0x0005, 0x3126); //12.288Mhz
        I2C_WriteNAU88L25(0x0006, 0x0008);
    }
    else
    {
        I2C_WriteNAU88L25(0x0005, 0x86C2); //11.2896Mhz
        I2C_WriteNAU88L25(0x0006, 0x0007);
    }

    switch (u32SampleRate)
    {
        case 16000:
            I2C_WriteNAU88L25(0x0003,  0x801B); /* MCLK = SYSCLK_SRC/12 */
            I2C_WriteNAU88L25(0x0004,  0x0001);
            I2C_WriteNAU88L25(0x0005,  0x3126); /* MCLK = 4.096MHz */
            I2C_WriteNAU88L25(0x0006,  0x0008);
            I2C_WriteNAU88L25(0x001D,  0x301A); /* 301A:Master, BCLK_DIV=MCLK/8=512K, LRC_DIV=512K/32=16K */
            I2C_WriteNAU88L25(0x002B,  0x0002);
            I2C_WriteNAU88L25(0x002C,  0x0082);
            break;

        case 44100:
            I2C_WriteNAU88L25(0x001D,  0x301A); /* 301A:Master, BCLK_DIV=11.2896M/8=1.4112M, LRC_DIV=1.4112M/32=44.1K */
            I2C_WriteNAU88L25(0x002B,  0x0012);
            I2C_WriteNAU88L25(0x002C,  0x0082);
            break;

        case 48000:
            I2C_WriteNAU88L25(0x001D,  0x301A); /* 301A:Master, BCLK_DIV=12.288M/8=1.536M, LRC_DIV=1.536M/32=48K */
            I2C_WriteNAU88L25(0x002B,  0x0012);
            I2C_WriteNAU88L25(0x002C,  0x0082);
            break;
    }
}

/**
 * @brief    Reset the NAU88L25 audio codec via I2C.
 *
 * @return   None
 *
 * @details  This function sends a reset command to the NAU88L25 audio codec,
 *           which resets all registers to their default values.
 */
void NAU88L25_Reset(void)
{
    I2C_WriteNAU88L25(0,  0x1);
    I2C_WriteNAU88L25(0,  0); /* Reset all registers */
    CLK_SysTickDelay(10000);

    printf("NAU88L25 Software Reset.\n");
}

/**
 * @brief    Configure the NAU88L25 audio codec via I2C.
 *
 * @return   None
 *
 * @details  This function configures the NAU88L25 audio codec with a series
 *           of I2C commands to set up its internal registers.
 */
void NAU88L25_Setup(void)
{
    I2C_WriteNAU88L25(0x0003,  0x8053);
    I2C_WriteNAU88L25(0x0004,  0x0001);
    I2C_WriteNAU88L25(0x0005,  0x3126);
    I2C_WriteNAU88L25(0x0006,  0x0008);
    I2C_WriteNAU88L25(0x0007,  0x0010);
    I2C_WriteNAU88L25(0x0008,  0xC000);
    I2C_WriteNAU88L25(0x0009,  0x6000);
    I2C_WriteNAU88L25(0x000A,  0xF13C);
    I2C_WriteNAU88L25(0x000C,  0x0048);
    I2C_WriteNAU88L25(0x000D,  0x0000);
    I2C_WriteNAU88L25(0x000F,  0x0000);
    I2C_WriteNAU88L25(0x0010,  0x0000);
    I2C_WriteNAU88L25(0x0011,  0x0000);
    I2C_WriteNAU88L25(0x0012,  0xFFFF);
    I2C_WriteNAU88L25(0x0013,  0x0015);
    I2C_WriteNAU88L25(0x0014,  0x0110);
    I2C_WriteNAU88L25(0x0015,  0x0000);
    I2C_WriteNAU88L25(0x0016,  0x0000);
    I2C_WriteNAU88L25(0x0017,  0x0000);
    I2C_WriteNAU88L25(0x0018,  0x0000);
    I2C_WriteNAU88L25(0x0019,  0x0000);
    I2C_WriteNAU88L25(0x001A,  0x0000);
    I2C_WriteNAU88L25(0x001B,  0x0000);
    I2C_WriteNAU88L25(0x001C,  0x0002);
    I2C_WriteNAU88L25(0x001D,  0x301A); /* 301A:Master, BCLK_DIV=12.288M/8=1.536M, LRC_DIV=1.536M/32=48K */
    I2C_WriteNAU88L25(0x001E,  0x0000);
    I2C_WriteNAU88L25(0x001F,  0x0000);
    I2C_WriteNAU88L25(0x0020,  0x0000);
    I2C_WriteNAU88L25(0x0021,  0x0000);
    I2C_WriteNAU88L25(0x0022,  0x0000);
    I2C_WriteNAU88L25(0x0023,  0x0000);
    I2C_WriteNAU88L25(0x0024,  0x0000);
    I2C_WriteNAU88L25(0x0025,  0x0000);
    I2C_WriteNAU88L25(0x0026,  0x0000);
    I2C_WriteNAU88L25(0x0027,  0x0000);
    I2C_WriteNAU88L25(0x0028,  0x0000);
    I2C_WriteNAU88L25(0x0029,  0x0000);
    I2C_WriteNAU88L25(0x002A,  0x0000);
    I2C_WriteNAU88L25(0x002B,  0x0012);
    I2C_WriteNAU88L25(0x002C,  0x0082);
    I2C_WriteNAU88L25(0x002D,  0x0000);
    I2C_WriteNAU88L25(0x0030,  0x00CF);
    I2C_WriteNAU88L25(0x0031,  0x0000);
    I2C_WriteNAU88L25(0x0032,  0x0000);
    I2C_WriteNAU88L25(0x0033,  0x009E);
    I2C_WriteNAU88L25(0x0034,  0x029E);
    I2C_WriteNAU88L25(0x0038,  0x1486);
    I2C_WriteNAU88L25(0x0039,  0x0F12);
    I2C_WriteNAU88L25(0x003A,  0x25FF);
    I2C_WriteNAU88L25(0x003B,  0x3457);
    I2C_WriteNAU88L25(0x0045,  0x1486);
    I2C_WriteNAU88L25(0x0046,  0x0F12);
    I2C_WriteNAU88L25(0x0047,  0x25F9);
    I2C_WriteNAU88L25(0x0048,  0x3457);
    I2C_WriteNAU88L25(0x004C,  0x0000);
    I2C_WriteNAU88L25(0x004D,  0x0000);
    I2C_WriteNAU88L25(0x004E,  0x0000);
    I2C_WriteNAU88L25(0x0050,  0x2007);
    I2C_WriteNAU88L25(0x0051,  0x0000);
    I2C_WriteNAU88L25(0x0053,  0xC201);
    I2C_WriteNAU88L25(0x0054,  0x0C95);
    I2C_WriteNAU88L25(0x0055,  0x0000);
    I2C_WriteNAU88L25(0x0058,  0x1A14);
    I2C_WriteNAU88L25(0x0059,  0x00FF);
    I2C_WriteNAU88L25(0x0066,  0x0060);
    I2C_WriteNAU88L25(0x0068,  0xC300);
    I2C_WriteNAU88L25(0x0069,  0x0000);
    I2C_WriteNAU88L25(0x006A,  0x0083);
    I2C_WriteNAU88L25(0x0071,  0x0011);
    I2C_WriteNAU88L25(0x0072,  0x0260);
    I2C_WriteNAU88L25(0x0073,  0x332C);
    I2C_WriteNAU88L25(0x0074,  0x4502);
    I2C_WriteNAU88L25(0x0076,  0x3140);
    I2C_WriteNAU88L25(0x0077,  0x0000);
    I2C_WriteNAU88L25(0x007F,  0x553F);
    I2C_WriteNAU88L25(0x0080,  0x0420);
    I2C_WriteNAU88L25(0x0001,  0x07D4);

    printf("NAU88L25 Configured done.\n");
}

#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
