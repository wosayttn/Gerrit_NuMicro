/**
  * INA219 Key Registers
  * 0x00  Configuration Register
  * 0x01  Shunt Voltage
  * 0x02  Bus Voltage
  * 0x03  Power
  * 0x04  Current
  * 0x05  Calibration
	*
	* Base on condition:
	* Bus Voltage Range: 32V, VCurrent Range: 5A, Shunt Resistor: 10m Ohm, ADC Resolution: 12-bit, Mode: Continuous shunt + bus voltage
	* ==>
	* Current_LSB = 5 A / 32767 = 0.000152uA. Calibration = 0.04096 / (0.000152 × 0.01) ~= 26947
	*
  **/

#include "NuMicro.h"
#include "utcpdlib.h"


#define INA219_ADDRESS        0x40  // Default I2C address

#define REG_CONFIG            0x00
#define REG_SHUNT_VOLTAGE     0x01
#define REG_BUS_VOLTAGE       0x02
#define REG_POWER             0x03
#define REG_CURRENT           0x04
#define REG_CALIBRATION       0x05


static int _i2c_read16(int slaveaddr, int reg, int *val)
{
    uint8_t data[2];
    int ret = 0;

    ret = I2C_ReadMultiBytesOneReg(I2C0, slaveaddr, reg, data, 2);
    if(ret == 2)
    {
        *val = (data[1] << 8) | data[0];
        return 0;
    }
    printf("I2C Read err\n");
    return ret;
}


uint32_t ina219_read16(int slaveaddr, int reg, int *val)
{
    uint32_t rv;
    uint16_t u16rdata;
    uint8_t u8rdata[2];
    I2C_WriteByte(I2C0, slaveaddr, (uint8_t)reg);
    printf("....");
    rv = I2C_ReadMultiBytes(I2C0, slaveaddr, u8rdata, 2);
    *val = ((uint16_t)u8rdata[0] << 8) | u8rdata[1];		//Big endian

    return rv;
}

static int _i2c_write16(int slaveaddr, int reg, int val)
{
    uint8_t data[2];
    uint32_t ret = 0;

    data[0] = val;
    data[1] = val >> 8;

    ret = I2C_WriteMultiBytesOneReg(I2C0, (uint8_t)slaveaddr, (uint8_t)reg, data, 2);
    if(ret == 2)
        return 0;
    printf("I2C write err\n");
    return ret;
}

static uint16_t htobe16(uint16_t val)
{
    uint16_t swapval;
    swapval = (val << 8) | (val >> 8);
    return swapval;
}


enum ec_error_list ina219_write16(int chgnum, int reg, uint16_t val)
{
    int reg_val = htobe16(val);
    return _i2c_write16(INA219_ADDRESS, reg, reg_val);
}

// Calibration value, based on shunt and current range
#define INA219_CALIBRATION_VALUE    20480  // This must be computed for your shunt
#define INA219_CONFIG_VALUE        0x399F  // 320mV, Gain /8, 12-bit ADCs, Shunt and bus, continuous
#define CURRENT_LSB                0.0002

void ina219_Init()
{
    /**
    		* Configuration: 32V, 2A, 12-bit ADCs
      * [15:13] BRNG = 0 (16V) or 1 (32V)
      * [12:11] PG = 00 to 11 (gain: /1 to /8)
      * [10:7] BADC = bus ADC resolution
      * [6:3] SADC = shunt ADC resolution
      * [2:0] MODE = 111 = continuous shunt and bus
      *
      * MAX VBUS Current = 	5.5A
    	* Shunt 10m OHM
    	* Current_LSB = 5.5A/10m Ohm = 167.8uA   ===>  Current_LSB	 = 200uA
    	* Calibration= 0.04096/(Current_LSB*Rshunt) = 20480
    	* Calibration Register	20480 (0x5000)
    	* Vertify Max Measurable Current = 200uA*32767 = 6.5A > 5.5A.
      **/

    int value;
    ina219_write16(INA219_ADDRESS, REG_CALIBRATION, INA219_CALIBRATION_VALUE);
    ina219_read16(INA219_ADDRESS, REG_CALIBRATION, &value);
    if (value != INA219_CALIBRATION_VALUE)
        printf("INA219 set REG_CALIBRATION reg wrong 0x%x\n", value);
    else
        printf("Write Calibration OK \n");

    ina219_write16(INA219_ADDRESS, REG_CONFIG, INA219_CONFIG_VALUE);
    ina219_read16(INA219_ADDRESS, REG_CONFIG, &value);
    if (value != INA219_CONFIG_VALUE)
        printf("INA219 set REG_CONFIG reg wrong 0x%x\n", value);
    else
        printf("Write REG_CONFIG OK \n");
}


uint16_t ina219_ReadBusVoltage_mV()
{
    int value;
    ina219_read16(INA219_ADDRESS, REG_BUS_VOLTAGE, &value);

    // Shift to remove CNVR and OVF bits
    value = (value >> 3) * 4;  // Each bit = 4mV
    return (uint16_t)value;
}

float ina219_ReadBusVoltage_V(void)
{
    int raw;
    ina219_read16(INA219_ADDRESS, REG_BUS_VOLTAGE, &raw);

    raw = (raw >> 3) & 0x1FFF;         // Bits [15:13] = CNVR/OVF, discard
    return raw * 0.004f;               // 1 LSB = 4 mV
}

float ina219_ReadShuntVoltage_mV(void) //OK
{
    int value;
    ina219_read16(INA219_ADDRESS, REG_SHUNT_VOLTAGE, &value);
    printf("Shunt %d\n", value);

    return value * 0.01f;                // 1 LSB = 10uV = 0.01 mV
}

float ina219_ReadCurrent_mA(void)
{
    int  raw;
    ina219_read16(INA219_ADDRESS, REG_CURRENT, &raw);
    return raw * CURRENT_LSB * 1000.0f;  // convert
}

