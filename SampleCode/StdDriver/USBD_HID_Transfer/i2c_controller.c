/* Copyright 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* I2C cross-platform code for Chrome EC */
#include "utcpdlib.h"

#define USBC_PORT_C0  0

struct tcpc_config_t tcpc_config[] =
{
    [USBC_PORT_C0] = {
        .bus_type = EC_BUS_TYPE_I2C,
        .i2c_info = {
            .port = 0, //I2C_PORT_TCPC0,
            .addr_flags = 0, //NCT38XX_I2C_ADDR1_1_FLAGS,
        },
        .drv = NULL,
        .flags = TCPC_FLAGS_TCPCI_REV2_0,
    },
};

void i2c_check_ready(int port)
{
    uint32_t BaseAddr;
    if(port == 0)
        BaseAddr = UTCPD0_BASE;
    else
        BaseAddr = UTCPD1_BASE;
    do
    {
    }
    while ( (inp32(BaseAddr+UTCPD_CLKINFO) & UTCPD_I2C_READY) == 0 );
}

int i2c_read32(const int port, const uint16_t addr_flags, int offset, int *data)
{
    uint32_t BaseAddr;
    if(port == 0)
        BaseAddr = UTCPD0_BASE;
    else
        BaseAddr = UTCPD1_BASE;

    *data = inp32(BaseAddr+offset);
    return EC_SUCCESS;
}

int i2c_write32(const int port, const uint16_t addr_flags, int offset, int data)
{
    uint32_t BaseAddr;
    if(port == 0)
        BaseAddr = UTCPD0_BASE;
    else
        BaseAddr = UTCPD1_BASE;

    i2c_check_ready(port);

    outp32(BaseAddr+offset, data);

    return EC_SUCCESS;
}

int i2c_read16(const int port, const uint16_t addr_flags, int offset, int *data)
{
    uint32_t BaseAddr;
    if(port == 0)
        BaseAddr = UTCPD0_BASE;
    else
        BaseAddr = UTCPD1_BASE;

    *data = inp16((BaseAddr+offset));
    return EC_SUCCESS;
}

int i2c_write16(const int port, const uint16_t addr_flags, int offset, int data)
{
    uint32_t BaseAddr;
    if(port == 0)
        BaseAddr = UTCPD0_BASE;
    else
        BaseAddr = UTCPD1_BASE;

    i2c_check_ready(port);

    outp16(BaseAddr+offset, data&0xFFFF);

    return EC_SUCCESS;
}

int i2c_read8(const int port, const uint16_t addr_flags, int offset, int *data)
{
    uint32_t BaseAddr;
    if(port == 0)
        BaseAddr = UTCPD0_BASE;
    else
        BaseAddr = UTCPD1_BASE;

    *data = inp8(BaseAddr+offset);
    return EC_SUCCESS;
}

int i2c_write8(const int port, const uint16_t addr_flags, int offset, int data)
{
    uint32_t BaseAddr;
    if(port == 0)
        BaseAddr = UTCPD0_BASE;
    else
        BaseAddr = UTCPD1_BASE;

    i2c_check_ready(port);

    if(offset == 0xE0)
    {
        printf("data = 0x%x\n", data);
    }

    outp8((BaseAddr+offset), data&0xFF);
    return EC_SUCCESS;
}

int i2c_update8(const int port,
                const uint16_t addr_flags,
                const int offset,
                const uint8_t mask,
                const enum mask_update_action action)
{
    int rv;
    int read_val;
    int write_val;

    rv = i2c_read8(port, addr_flags, offset, &read_val);
    if (rv)
        return rv;

    write_val = (action == MASK_SET) ? (read_val | mask)
                : (read_val & ~mask);

    return i2c_write8(port, addr_flags, offset, write_val);
}

int i2c_update16(const int port,
                 const uint16_t addr_flags,
                 const int offset,
                 const uint16_t mask,
                 const enum mask_update_action action)
{
    int rv;
    int read_val;
    int write_val;

    rv = i2c_read16(port, addr_flags, offset, &read_val);
    if (rv)
        return rv;

    write_val = (action == MASK_SET) ? (read_val | mask)
                : (read_val & ~mask);

    return i2c_write16(port, addr_flags, offset, write_val);
}

int i2c_read_block(const int port, const uint16_t addr_flags, int offset,
                   uint8_t *data, int len)
{
    uint32_t i;
    uint32_t BaseAddr;
    if(port == 0)
        BaseAddr = UTCPD0_BASE;
    else
        BaseAddr = UTCPD1_BASE;
    for(i = 0; i < len; i = i+1)
        *(data+i) = inp8( (BaseAddr+offset)+i );
    return EC_SUCCESS;
}

int i2c_write_block(const int port,
                    const uint16_t addr_flags,
                    int offset, const uint8_t *data, int len)
{
    uint32_t i;
    uint32_t BaseAddr;
    uint32_t u32data;
    if(port == 0)
        BaseAddr = UTCPD0_BASE;
    else
        BaseAddr = UTCPD1_BASE;
    for(i = 0; i < len; i = i+4)
    {
        u32data = *data | (*(data+1)<<8) | (*(data+2)<<16) | (*(data+3)<<24);
        i2c_check_ready(port);
        outp32( (BaseAddr+offset)+i, u32data);
        data = data+4;
    }
    return EC_SUCCESS;
}
