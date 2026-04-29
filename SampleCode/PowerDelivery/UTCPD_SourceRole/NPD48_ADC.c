#include <stdio.h>
#include <ctype.h>
#include "NuMicro.h"
#include "utcpdlib.h"
#include "usb_pd.h"

#undef DBG_PRINTF
#define DBG_PRINTF(...)
/*----------------------------------------------------------*/
/* Prototype and Global variables                           */
/*----------------------------------------------------------*/
uint8_t g_u8DeviceAddr;

static void NPD48_ADC_START_CONV(void)
{
    uint8_t readBuff[4], writeBuff[4];

    I2C_ReadMultiBytesTwoRegsProtect(I2C0, g_u8DeviceAddr, NPD48_ADCR, readBuff, 1);
    writeBuff[0] = (readBuff[0] | NPD48_ADCR_ADCEN_Msk);
    I2C_WriteMultiBytesTwoRegsProtect(I2C0, g_u8DeviceAddr, NPD48_ADCR, writeBuff, 1);
}

static void NPD48_ADC_STOP_CONV(void)
{
    uint8_t readBuff[4], writeBuff[4];

    I2C_ReadMultiBytesTwoRegsProtect(I2C0, g_u8DeviceAddr, NPD48_ADCR, readBuff, 1);
    writeBuff[0] = (readBuff[0] & ~NPD48_ADCR_ADCEN_Msk);
    I2C_WriteMultiBytesTwoRegsProtect(I2C0, g_u8DeviceAddr, NPD48_ADCR, writeBuff, 1);
}

static void NPD48_ADC_SCAN_MODE(uint32_t mode)
{
    uint8_t readBuff[4], writeBuff[4];

    I2C_ReadMultiBytesTwoRegsProtect(I2C0, g_u8DeviceAddr, NPD48_ADCR, readBuff, 1);
    if (mode == 1)
        writeBuff[0] = (readBuff[0] | NPD48_ADCR_SCANMD_Msk);
    else
        writeBuff[0] = (readBuff[0] & ~NPD48_ADCR_SCANMD_Msk);
    I2C_WriteMultiBytesTwoRegsProtect(I2C0, g_u8DeviceAddr, NPD48_ADCR, writeBuff, 1);
}

static void NPD48_ADC_SET_INPUT_CHANNEL(uint8_t chMask)
{
    uint8_t writeBuff[4];

    writeBuff[0] = chMask;
    I2C_WriteMultiBytesTwoRegsProtect(I2C0, g_u8DeviceAddr, NPD48_ADCHEN, writeBuff, 1);
}

CODE_LDROM uint32_t NPD48_ADC_GET_CONVERSION_DATA(uint8_t ch)
{
    int data;
    i2c_read16(0, NULL, NPD48_ADCCH0L+(ch*2), &data);
    return (int)data;
}

CODE_LDROM uint32_t NPD48_ADC_SET_CONVERSION_DATA(int port, uint8_t ch, uint32_t val)
{
    int data;
    val &= 0xFFF;
    i2c_write16(port, NULL, NPD48_ADCCH0L+(ch*2), val);
}

CODE_LDROM void NPD48_ADC_Init(int port)
{
    uint32_t i, data;

    g_u8DeviceAddr = (0xC0 >> 1);

    DBG_PRINTF("\n======= NPD48 ADC Test =======\n");

    DBG_PRINTF("Enable all ADC channels.\n");
    i2c_write8(port, NULL, NPD48_ADEXTEND, 0x20);
    NPD48_ADC_SET_INPUT_CHANNEL(0xFF);

    DBG_PRINTF("Start ADC conversion.\n");

    NPD48_ADC_START_CONV();

}

