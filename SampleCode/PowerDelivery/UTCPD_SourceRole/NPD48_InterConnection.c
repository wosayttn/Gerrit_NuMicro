#include <stdio.h>
#include <ctype.h>
#include "NuMicro.h"
#include "utcpdlib.h"
#include "usb_pd.h"

#define NPD48_SLAVE_ADDR	(0xC0>>1)	// I2C_ADH=0, I2C_ADL=0
#define NPD48_REG_WIDTH   (1)
#define NPD48_UFCSCR     	(0x121)
#define NPD48_PWRCR	      (0x100)


#define BC_UFCSCR_TXDPEN_Pos            (3)                                               /*!< ADC_T::UFCSCR: TXDPEN Position         */
#define BC_UFCSCR_TXDPEN_Msk            (0x1ul << BC_UFCSCR_TXDPEN_Pos)                  /*!< ADC_T::UFCSCR: TXDPEN Mask             */

#define BC_UFCSCR_TXDNEN_Pos            (4)                                               /*!< ADC_T::UFCSCR: TXDNEN Position         */
#define BC_UFCSCR_TXDNEN_Msk            (0x1ul << BC_UFCSCR_TXDNEN_Pos)                  /*!< ADC_T::UFCSCR: TXDNEN Mask             */

#define BC_UFCSCR_UFCSEN_Pos            (0)
#define BC_UFCSCR_UFCSEN_Msk            (0x1ul << BC_UFCSCR_UFCSEN_Pos)

#define BC_UFCSCR_DPRXEN_Pos            (1)                                               /*!< ADC_T::UFCSCR: DPRXEN Position         */
#define BC_UFCSCR_DPRXEN_Msk            (0x1ul << BC_UFCSCR_DPRXEN_Pos)                  /*!< ADC_T::UFCSCR: DPRXEN Mask             */

#define BC_UFCSCR_DNRXEN_Pos            (2)                                               /*!< ADC_T::UFCSCR: DNRXEN Position         */
#define BC_UFCSCR_DNRXEN_Msk            (0x1ul << BC_UFCSCR_DNRXEN_Pos)                  /*!< ADC_T::UFCSCR: DNRXEN Mask             */

#define CHIP_PWRCR_LDO33EN_Pos          (2)
#define CHIP_PWRCR_LDO33EN_Msk          (0x1ul << CHIP_PWRCR_LDO33EN_Pos)

/* enable BC power */
CODE_LDROM void bc_pwr_en(void)
{
    uint8_t u8i2cBuf[2];

    /* read Power Control register */
    I2C_ReadMultiBytesTwoRegs(I2C0, NPD48_SLAVE_ADDR, NPD48_PWRCR/*(uint32_t)&NPD48_CHIP->PWRCR*/, u8i2cBuf, NPD48_REG_WIDTH);

    /* enable BC power (LDO33) */
    u8i2cBuf[0] |= CHIP_PWRCR_LDO33EN_Msk;

    /* write Power Control register */
    I2C_WriteMultiBytesTwoRegs(I2C0, NPD48_SLAVE_ADDR, NPD48_PWRCR/*(uint32_t)&NPD48_CHIP->PWRCR*/, u8i2cBuf, NPD48_REG_WIDTH);
}

CODE_LDROM void ufcs_en(void)
{
    uint8_t u8i2cBuf[2];

    /* read NPD48_BC->UFCSCR */
    I2C_ReadMultiBytesTwoRegs(I2C0, NPD48_SLAVE_ADDR, NPD48_UFCSCR/*(uint32_t)&NPD48_BC->UFCSCR*/, u8i2cBuf, NPD48_REG_WIDTH);

    /* enable UFCSCR_UFCSEN_Msk */
    u8i2cBuf[0] |= BC_UFCSCR_UFCSEN_Msk;

    /* write NPD48_BC->UFCSCR */
    I2C_WriteMultiBytesTwoRegs(I2C0, NPD48_SLAVE_ADDR, NPD48_UFCSCR/*(uint32_t)&NPD48_BC->UFCSCR*/, u8i2cBuf, NPD48_REG_WIDTH);
}

CODE_LDROM void dp_tx_en(void)
{
    uint8_t u8i2cBuf[2];

    /* read NPD48_BC->UFCSCR */
    I2C_ReadMultiBytesTwoRegs(I2C0, NPD48_SLAVE_ADDR, NPD48_UFCSCR/*(uint32_t)&NPD48_BC->UFCSCR*/, u8i2cBuf, NPD48_REG_WIDTH);

    /* enable rx to D+ */
    u8i2cBuf[0] |= BC_UFCSCR_DPRXEN_Msk;

    /* disable rx to D- */
    u8i2cBuf[0] &= ~BC_UFCSCR_DNRXEN_Msk;

    /* write NPD48_BC->UFCSCR */
    I2C_WriteMultiBytesTwoRegs(I2C0, NPD48_SLAVE_ADDR, NPD48_UFCSCR/*(uint32_t)&NPD48_BC->UFCSCR*/, u8i2cBuf, NPD48_REG_WIDTH);
}

/* enable NPD48 TX to D- */
CODE_LDROM void dm_rx_en(void)
{
    uint8_t u8i2cBuf[2];

    /* read NPD48_BC->UFCSCR */
    I2C_ReadMultiBytesTwoRegs(I2C0, NPD48_SLAVE_ADDR, NPD48_UFCSCR/*(uint32_t)&NPD48_BC->UFCSCR*/, u8i2cBuf, NPD48_REG_WIDTH);

    /* enable D- to tx */
    u8i2cBuf[0] |= BC_UFCSCR_TXDNEN_Msk;

    /* disable D+ to tx */
    u8i2cBuf[0] &= ~BC_UFCSCR_TXDPEN_Msk;

    /* write NPD48_BC->UFCSCR */
    I2C_WriteMultiBytesTwoRegs(I2C0, NPD48_SLAVE_ADDR, NPD48_UFCSCR/*(uint32_t)&NPD48_BC->UFCSCR*/, u8i2cBuf, NPD48_REG_WIDTH);
}

void UFCS_Init(void)
{
    bc_pwr_en();
    ufcs_en();
    dp_tx_en();
    dm_rx_en();
}