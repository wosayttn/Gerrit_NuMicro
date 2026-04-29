/**************************************************************************//**
 * @file     i2c.c
 * @version  V1.00
 * @brief    M251 series I2C driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"
/*---------------------------------------------------------------------------------------------------------*/
/* I2C Define Error Code                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define I2C_TIMEOUT     SystemCoreClock  /*!< I2C time-out counter (1 second time-out)                                    \hideinitializer */
#define I2C_OK          ( 0L)            /*!< I2C operation OK                                                            \hideinitializer */
#define I2C_ERR_TIMEOUT (-1L)            /*!< I2C operation abort due to timeout error                                    \hideinitializer */
#define I2C_TIMEOUT_ERR (I2C_ERR_TIMEOUT)/*!< I2C operation abort due to timeout error (backward compatibility)           \hideinitializer */
#define I2C_ERR_FAIL    (-2L)            /*!< I2C operation failed                                                        \hideinitializer */

//#define  CODE_LDROM  __attribute__((section("LDROM")))
#define CODE_LDROM

#if !defined(NPD48_ALERT)
#define NPD48_ALERT     EINT0_IRQn
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup I2C_Driver I2C Driver
  @{
*/

/** @addtogroup I2C_EXPORTED_FUNCTIONS I2C Exported Functions
  @{
*/

/**
  * @brief      Specify two bytes register address and Write a byte to Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u16DataAddr     Specify a address (2 byte) of data write to
  * @param[in]  data            Write a byte data to Slave
  *
  * @retval     0               Write data success
  * @retval     1               Write data fail, or bus occurs error events
  *
  * @details    The function is used for I2C Master specify two bytes address that data write to in Slave.
  *
  * @note       This function sets g_I2C_i32ErrCode to I2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */

CODE_LDROM uint8_t I2C_WriteByteTwoRegsProtect(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data)
{
    uint8_t bIsEnInt = 0u, u8Err = 0u;
    uint32_t u32TimeOutCount;

    if(NVIC_GetEnableIRQ(NPD48_ALERT)>0)
    {
        NVIC_DisableIRQ(NPD48_ALERT);
        bIsEnInt = 1;
    }

    u8Err = I2C_WriteByteTwoRegs(i2c, u8SlaveAddr, u16DataAddr, data);

    u32TimeOutCount = I2C_TIMEOUT;
    while ((i2c)->CTL0 & I2C_CTL0_STO_Msk)
    {
        if (--u32TimeOutCount == 0)
        {
            g_I2C_i32ErrCode = I2C_ERR_TIMEOUT;
            break;
        }
    }

    if(bIsEnInt)
        NVIC_EnableIRQ(NPD48_ALERT);
    return (u8Err);                                             /* return (Success)/(Fail) status */
}


/**
  * @brief      Specify two bytes register address and write multi bytes to Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u16DataAddr     Specify a address (2 bytes) of data write to
  * @param[in]  data[]          A data array for write data to Slave
  * @param[in]  u32wLen         How many bytes need to write to Slave
  *
  * @return     A length of how many bytes have been transmitted.
  *
  * @details    The function is used for I2C Master specify a byte address that multi data write to in Slave.
  *
  * @note       This function sets g_I2C_i32ErrCode to I2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */

CODE_LDROM uint32_t I2C_WriteMultiBytesTwoRegsProtect(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen)
{
    uint8_t bIsEnInt = 0u;
    uint32_t u32txLen = 0u;
    uint32_t u32TimeOutCount;

    if(NVIC_GetEnableIRQ(NPD48_ALERT)>0)
    {
        NVIC_DisableIRQ(NPD48_ALERT);
        bIsEnInt = 1;
    }

    u32txLen = I2C_WriteMultiBytesTwoRegs(i2c, u8SlaveAddr, u16DataAddr, data, u32wLen);

    u32TimeOutCount = I2C_TIMEOUT;
    while ((i2c)->CTL0 & I2C_CTL0_STO_Msk)
    {
        if (--u32TimeOutCount == 0)
        {
            g_I2C_i32ErrCode = I2C_ERR_TIMEOUT;
            break;
        }
    }

    if(bIsEnInt)
        NVIC_EnableIRQ(NPD48_ALERT);
    return u32txLen;                                                        /* Return bytes length that have been transmitted */
}

/**
  * @brief      Specify two bytes register address and read a byte from Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u16DataAddr     Specify an address(2 bytes) of data read from
  *
  * @return     Read a byte data from Slave
  *
  * @details    The function is used for I2C Master specify two bytes address that a data byte read from Slave.
  *
  * @note       This function sets g_I2C_i32ErrCode to I2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */
CODE_LDROM uint8_t I2C_ReadByteTwoRegsProtect(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr)
{
    uint8_t bIsEnInt = 0u, rdata = 0u;
    uint32_t u32TimeOutCount;

    if(NVIC_GetEnableIRQ(NPD48_ALERT)>0)
    {
        NVIC_DisableIRQ(NPD48_ALERT);
        bIsEnInt = 1;
    }

    rdata = I2C_ReadByteTwoRegs(i2c, u8SlaveAddr, u16DataAddr);

    u32TimeOutCount = I2C_TIMEOUT;
    while ((i2c)->CTL0 & I2C_CTL0_STO_Msk)
    {
        if (--u32TimeOutCount == 0)
        {
            g_I2C_i32ErrCode = I2C_ERR_TIMEOUT;
            break;
        }
    }

    if(bIsEnInt)
        NVIC_EnableIRQ(NPD48_ALERT);
    return rdata;                                                           /* Return read data */
}

/**
  * @brief      Specify two bytes register address and read multi bytes from Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u16DataAddr     Specify a address (2 bytes) of data read from
  * @param[out] rdata[]         A data array to store data from Slave
  * @param[in]  u32rLen         How many bytes need to read from Slave
  *
  * @return     A length of how many bytes have been received
  *
  * @details    The function is used for I2C Master specify two bytes address that multi data bytes read from Slave.
  *
  * @note       This function sets g_I2C_i32ErrCode to I2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */
CODE_LDROM uint32_t _I2C_ReadMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Addr = 1u, u8Ctrl = 0u;
    uint32_t u32rxLen = 0u;
    uint32_t u32TimeOutCount;

    g_I2C_i32ErrCode = 0;

    I2C_START(i2c);                                                         /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = I2C_TIMEOUT;
        I2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_I2C_i32ErrCode = I2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (I2C_GET_STATUS(i2c))
        {
        case 0x08u:
            I2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));               /* Write SLA+W to Register I2CDAT */
            u8Ctrl = I2C_CTL_SI;                                      /* Clear SI */
            break;

        case 0x18u:                                                      /* Slave Address ACK */
            I2C_SET_DATA(i2c, (uint8_t)((u16DataAddr & 0xFF00u) >> 8u));    /* Write Hi byte address of register */
            break;

        case 0x20u:                                                      /* Slave Address NACK */
        case 0x30u:                                                      /* Master transmit data NACK */
            u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
            u8Err = 1u;
            break;

        case 0x28u:
            if (u8Addr)
            {
                I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFFu));       /* Write Lo byte address of register */
                u8Addr = 0u;
            }
            else
            {
                u8Ctrl = I2C_CTL_STA_SI;                              /* Clear SI and send repeat START */
            }

            break;

        case 0x10u:
            I2C_SET_DATA(i2c, (uint8_t)((u8SlaveAddr << 1u) | 0x01u));             /* Write SLA+R to Register I2CDAT */
            u8Ctrl = I2C_CTL_SI;                                      /* Clear SI */
            break;

        case 0x40u:                                                      /* Slave Address ACK */
            if (u32rLen == 1)
            {
                u8Ctrl = I2C_CTL_SI;
            }
            else
            {
                u8Ctrl = I2C_CTL_SI_AA;                                  /* Clear SI and set ACK */
            }
            break;

        case 0x48u:                                                      /* Slave Address NACK */
            u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
            u8Err = 1u;
            break;

        case 0x50u:
            rdata[u32rxLen++] = (uint8_t) I2C_GET_DATA(i2c);      /* Receive Data */

            if (u32rxLen < (u32rLen - 1u))
            {
                u8Ctrl = I2C_CTL_SI_AA;                               /* Clear SI and set ACK */
            }
            else
            {
                u8Ctrl = I2C_CTL_SI;                                  /* Clear SI */
            }

            break;

        case 0x58u:
            rdata[u32rxLen++] = (uint8_t) I2C_GET_DATA(i2c);      /* Receive Data */
            u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
            u8Xfering = 0u;
            break;

        case 0x38u:                                                      /* Arbitration Lost */
        default:                                                        /* Unknow status */
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);                 /* Clear SI and send STOP */
            u8Ctrl = I2C_CTL_SI;
            u8Err = 1u;
            break;
        }

        I2C_SET_CONTROL_REG(i2c, u8Ctrl);                                   /* Write controlbit to I2C_CTL register */
    }

    return u32rxLen;                                                        /* Return bytes length that have been received */
}

/**
  * @brief      Specify two bytes register address and read multi bytes from Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u16DataAddr     Specify a address (2 bytes) of data read from
  * @param[out] rdata[]         A data array to store data from Slave
  * @param[in]  u32rLen         How many bytes need to read from Slave
  *
  * @return     A length of how many bytes have been received
  *
  * @details    The function is used for I2C Master specify two bytes address that multi data bytes read from Slave.
  *
  * @note       This function sets g_I2C_i32ErrCode to I2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */
CODE_LDROM uint32_t I2C_ReadMultiBytesTwoRegsProtect(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen)
{
    uint8_t bIsEnInt = 0, u32rxLen = 0u;
    uint32_t u32TimeOutCount;

    if(NVIC_GetEnableIRQ(NPD48_ALERT)>0)
    {
        NVIC_DisableIRQ(NPD48_ALERT);
        bIsEnInt = 1;
    }

    u32rxLen = _I2C_ReadMultiBytesTwoRegs(i2c, u8SlaveAddr, u16DataAddr, rdata, u32rLen);

    u32TimeOutCount = I2C_TIMEOUT;
    while ((i2c)->CTL0 & I2C_CTL0_STO_Msk)
    {
        if (--u32TimeOutCount == 0)
        {
            g_I2C_i32ErrCode = I2C_ERR_TIMEOUT;
            break;
        }
    }

    if(bIsEnInt)
        NVIC_EnableIRQ(NPD48_ALERT);
    return u32rxLen;                                                        /* Return bytes length that have been received */
}


/** @} end of group I2C_EXPORTED_FUNCTIONS */

/** @} end of group I2C_Driver */

/** @} end of group Standard_Driver */

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
