/* Copyright 2022 The ChromiumOS Authors
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Richtek 5A 1-4 cell buck-boost switching battery charger driver.
 */
#if 0
#include "battery.h"
#include "battery_smart.h"
#include "builtin/assert.h"
#include "builtin/endian.h"
#include "charge_manager.h"
#include "charger.h"
#include "common.h"
#include "config.h"
#include "console.h"
#include "hooks.h"
#include "i2c.h"
#include "rt9490.h"
#include "task.h"
#include "temp_sensor/temp_sensor.h"
#include "temp_sensor/thermistor.h"
#include "usb_charge.h"
#include "usb_pd.h"
#include "util.h"

#ifndef CONFIG_CHARGER_NARROW_VDC
#error "RT9490 is a NVDC charger, please enable CONFIG_CHARGER_NARROW_VDC."
#endif
#else
#include "NuMicro.h"
#include "utcpdlib.h"
#include "rt9490.h"
#define be16toh(x) bswap16(x)
//#include <endian.h>
//#include <endian.h>
//#include <byteswap.h>
#define CLAMP(v, min, max) MIN(max, MAX(v, min))
//#define IN_RANGE(x, min, max) ((x) >= (min) && (x) <= (max))
#define CPRINTS(...)

#define CONFIG_CHARGER_OTG
#define CONFIG_CMD_CHARGER_DUMP
extern const struct battery_info *battery_get_info(void);
#endif




#if 0
/* Console output macros */
#define CPRINTF(format, args...) cprintf(CC_CHARGER, format, ##args)
#define CPRINTS(format, args...) \
	cprints(CC_CHARGER, "%s " format, "RT9490", ##args)
#endif

/* Charger parameters */
#define CHARGER_NAME "rt9490"
#define CHARGE_V_MAX 18800
#define CHARGE_V_MIN 3000
#define CHARGE_V_STEP 10
#define CHARGE_I_MAX 5000

/* b/238980988
 * RT9490 can't measure the 50mA charge current precisely due to insufficient
 * ADC resolution, and faulty leads it into battery supply mode.
 * the final number would be expected between 100mA ~ 200mA.
 * Vendor has done the FT correlation and will revise the datasheet's
 * CHARGE_I_MIN value from 50mA to 150mA as the final solution.
 */
#define CHARGE_I_MIN 150
#define CHARGE_I_STEP 10
#define INPUT_I_MAX 3300
#define INPUT_I_MIN 100
#define INPUT_I_STEP 10

/* Charger parameters */
static const struct charger_info rt9490_charger_info = {
    .name = CHARGER_NAME,
    .voltage_max = CHARGE_V_MAX,
    .voltage_min = CHARGE_V_MIN,
    .voltage_step = CHARGE_V_STEP,
    .current_max = CHARGE_I_MAX,
    .current_min = CHARGE_I_MIN,
    .current_step = CHARGE_I_STEP,
    .input_current_max = INPUT_I_MAX,
    .input_current_min = INPUT_I_MIN,
    .input_current_step = INPUT_I_STEP,
};

#ifndef CONFIG_ZEPHYR
const struct rt9490_init_setting rt9490_setting = {
    /* b/230442545#comment28
     * With EOC-Force-CCM disabled, the real IEOC would be
     * 30~50mA lower than expected, so move eoc_current one step up
     */
    .eoc_current = 240,
    .mivr = 4000,
#if SW
    .boost_voltage = 5050,
    .boost_current = 1500,
#else
    .boost_voltage = 5250,
    .boost_current = 3320,
#endif
};
#endif


static int _i2c_read8(int slaveaddr, int reg, int *val)
{
    uint8_t data[1];
    int ret = 0;

    ret = I2C_ReadMultiBytesOneReg(I2C0, slaveaddr, reg, data, 1);

    if(reg == 0)
    {
        printf("\n=====================\n");
        printf("data = 0x%x\n", data[0]);
        printf("ret = 0x%x\n", ret);
        printf("\n=====================\n");
    }
    if(ret == 1)
    {
        *val = data[0];
        return 0;
    }
    return ret;
}

static int _i2c_write8(int slaveaddr, int reg, int val)
{
    uint8_t data[1];
    int ret = 0;

    data[0] = val;
    ret = I2C_WriteMultiBytesOneReg(I2C0, slaveaddr, reg, data, 1);
    if(ret == 1)
        return 0;//Successful
    return ret;
}
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
    return ret;
}

static int _i2c_write16(int slaveaddr, int reg, int val)
{
    uint8_t data[2];
    int ret = 0;

    data[0] = val;
    data[1] = val >> 8;

    ret = I2C_WriteMultiBytesOneReg(I2C0, slaveaddr, reg, data, 2);
    if(ret == 2)
        return 0;
    return ret;
}

static enum ec_error_list rt9490_read8(int chgnum, int reg, int *val)
{
    int reg_val;

    return _i2c_read8(chg_chips[chgnum].i2c_addr_flags, reg, val);
}

static enum ec_error_list rt9490_write8(int chgnum, int reg, int val)
{
    return _i2c_write8(chg_chips[chgnum].i2c_addr_flags, reg, val);
}

uint16_t be16toh(uint16_t val)
{
    uint16_t swapval;
    swapval = (val << 8) | (val >> 8);
    return 	swapval;
}

static enum ec_error_list rt9490_read16(int chgnum, int reg, int* val)
{
    int reg_val;

    RETURN_ERROR(_i2c_read16(chg_chips[chgnum].i2c_addr_flags, reg, &reg_val));
    *val = be16toh(reg_val);

    return EC_SUCCESS;
}

uint16_t htobe16(uint16_t val)
{
    uint16_t swapval;
    swapval = (val << 8) | (val >> 8);
    return swapval;
}

static enum ec_error_list rt9490_write16(int chgnum, int reg, int val)
{
#if 1
    int reg_val = htobe16(val);
#else
    int reg_val = val;
#endif

    return _i2c_write16(chg_chips[chgnum].i2c_addr_flags, reg, reg_val);
}

int _i2c_field_update8(uint16_t slaveaddr,
                       int offset,
                       uint8_t field_mask,
                       uint8_t set_value);

static int rt9490_field_update8(int chgnum, int reg, uint8_t mask, uint8_t val)
{
    return _i2c_field_update8(chg_chips[chgnum].i2c_addr_flags, reg, mask, val);
}

int _i2c_field_update8(uint16_t slaveaddr, int offset, uint8_t field_mask, uint8_t set_value)
{
    int rv;
    int read_val;
    int write_val;

    rv = _i2c_read8(slaveaddr, offset, &read_val);
    if (rv)
        return rv;

    write_val = (read_val & ~field_mask) | set_value;

    return _i2c_write8(slaveaddr, offset, write_val);
}

int _i2c_update8(int slaveaddr, int offset, int mask, enum mask_update_action action)
{
    int rv;
    int read_val;
    int write_val;

    rv = _i2c_read8(slaveaddr, offset, &read_val);
    if (rv)
        return rv;

    write_val = (action == MASK_SET) ? (read_val | mask)
                : (read_val & ~mask);

    return _i2c_write8(slaveaddr, offset, write_val);
}

//static inline int rt9490_update8(int chgnum, int reg, int mask,
int rt9490_update8(int chgnum, int reg, int mask,
                   enum mask_update_action action)
{
    return _i2c_update8(chg_chips[chgnum].i2c_addr_flags, reg, mask, action);
}

static inline int rt9490_set_bit(int chgnum, int reg, int mask)
{
    return rt9490_update8(chgnum, reg, mask, MASK_SET);
}

static inline int rt9490_clr_bit(int chgnum, int reg, int mask)
{
    return rt9490_update8(chgnum, reg, mask, MASK_CLR);
}

/* Enable ADC: set bit0 of CTRL3 = 1 */
static void rt9492_enable_adc(int chgnum)
{
    uint8_t val;
    rt9490_set_bit(chgnum, RT9490_REG_ADC_CTRL, RT9490_ADC_EN);
}

static inline int rt9490_enable_hz(int chgnum, bool en)
{
    return rt9490_update8(chgnum, RT9490_REG_CHG_CTRL0, RT9490_EN_HZ,
                          en ? MASK_SET : MASK_CLR);
}

static const struct charger_info *rt9490_get_info(int chgnum)
{
    return &rt9490_charger_info;
}

static enum ec_error_list rt9490_get_current(int chgnum, int *current)
{
    int val = 0;
    const struct charger_info *const info = rt9490_get_info(chgnum);

    RETURN_ERROR(rt9490_read16(chgnum, RT9490_REG_ICHG_CTRL, &val));

    val = (val & RT9490_ICHG_MASK) >> RT9490_ICHG_SHIFT;
    val *= info->current_step;
    *current = CLAMP(val, info->current_min, info->current_max);

    return EC_SUCCESS;
}

/* Clone from MB-Brian { */
void rt9492_dp_ctrl_dm_ctrl_hz(int chgnum)
{
    int i32data;
    /* Set DP_CTRL & DM_CTRL as HZ mode */
    rt9490_read8(chgnum, RT9490_REG_DPDM_MANU_CTRL, &i32data);
    i32data &= 0x00;				//Set DP_CTRL(b7:b5)=000, DM_CTRL(b4:b2)=000
    rt9490_write8(chgnum, RT9490_REG_DPDM_MANU_CTRL, i32data);
}

void rt9492_rst_all(int chgnum)
{
    int i32data;
    /* Reset All of RT9492*/
    rt9490_read8(chgnum, RT9490_REG_EOC_CTRL, &i32data);
    i32data |= 0x80 ;				// Set RST_ALL(Bit7)=1 in Register_09
    rt9490_write8(chgnum, RT9490_REG_EOC_CTRL, i32data);

    /* Check RESET finished? */
    do {
        rt9490_read8(chgnum, RT9490_REG_EOC_CTRL, &i32data);
    } while((i32data &= 0xC0) != 0);

}

void rt9492_ts_adc_dis(int chgnum)
{
    int i32data;
    /* Disable TS_ADC_DIS function */
    rt9490_read8(chgnum, RT9490_REG_ADC_CHANNEL0, &i32data);
    i32data |= (0x04);				//Set bit2=1 to disable TS_ADC
    rt9490_write8(chgnum , RT9490_REG_ADC_CHANNEL0, i32data);
}
#define RT9492_REG_NUM  78

void rt9492_dump_register(void)//dump all register
{
    unsigned int i = 0;
    int i32data;
    printf("[RT9492]\n");

    for (i = 0; i < RT9492_REG_NUM; i++)
    {
        switch (i)
        {
        case 0x00:
        case 0x05:
        case 0x08:
        case 0x09:
        case 0x0A:
        case 0x0D:
        case 0x0E:
        case 0x0F:
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13:
        case 0x14:
        case 0x16:
        case 0x17:
        case 0x18:
        case 0x1B:
        case 0x1C:
        case 0x1D:
        case 0x1E:  //BIT[7:6] = 0  -> Single Input
        case 0x1F:
        case 0x20:
        case 0x21:
        case 0x22:
        case 0x23:
        case 0x24:
        case 0x25:
        case 0x26:
        case 0x27:
        case 0x28:
        case 0x29:
        case 0x2A:
        case 0x2B:
        case 0x2C:
        case 0x2D:
        case 0x2E:
        case 0x2F:
        case 0x30:
        case 0x47:
        case 0x48:
        case 0x49:
        case 0x4A:
        case 0x4B:
        case 0x4C:
        case 0x4D:
        case 0x4E:
            rt9490_read8(0, i, &i32data);
            printf("rt9492_reg[0x%x]=0x%x\n", i, i32data);
            break;

        case 0x01:
        case 0x03:
        case 0x06:
        case 0x0B:
        case 0x19:
        case 0x31:
        case 0x33:
        case 0x35:
        case 0x37:
        case 0x39:
        case 0x3B:
        case 0x3D:
        case 0x3F:
        case 0x41:
        case 0x43:
        case 0x45:
            rt9490_read16(0, i, &i32data);
            printf("rt9492_reg[0x%x]=0x%x, 0x%x\n", i, i32data & 0xFF, i32data >> 8);
            break;
        }

    }

    printf("\n");
}


/* Clone from MB-Brian }*/

static enum ec_error_list rt9490_set_current(int chgnum, int current)
{
    uint16_t reg_ichg;
    const struct charger_info *const info = rt9490_get_info(chgnum);

    if (current == 0) {
        current = info->current_min;
        rt9490_clr_bit(chgnum, RT9490_REG_CHG_CTRL0, RT9490_EN_CHG);
    } else
        rt9490_set_bit(chgnum, RT9490_REG_CHG_CTRL0, RT9490_EN_CHG);

    if (!IN_RANGE(current, info->current_min, info->current_max))
        return EC_ERROR_PARAM2;
    reg_ichg = current / info->current_step;

    return rt9490_write16(chgnum, RT9490_REG_ICHG_CTRL, reg_ichg);
}

static enum ec_error_list rt9490_get_voltage(int chgnum, int *voltage)
{
    int val = 0;
    const struct charger_info *const info = rt9490_get_info(chgnum);

    RETURN_ERROR(rt9490_read16(chgnum, RT9490_REG_VCHG_CTRL, &val));

    val = val & RT9490_CV_MASK;
    val *= info->voltage_step;
    *voltage = CLAMP(val, info->voltage_min, info->voltage_max);

    return EC_SUCCESS;
}

static enum ec_error_list rt9490_set_voltage(int chgnum, int voltage)
{
    int reg_cv;
    const struct charger_info *const info = rt9490_get_info(chgnum);

    if (voltage == 0)
        voltage = info->voltage_min;

    if (!IN_RANGE(voltage, info->voltage_min, info->voltage_max))
        return EC_ERROR_PARAM2;
    reg_cv = voltage / info->voltage_step;

    return rt9490_write16(chgnum, RT9490_REG_VCHG_CTRL, reg_cv);
}

#ifdef CONFIG_CHARGER_OTG
static enum ec_error_list rt9490_enable_otg_power(int chgnum, int enabled)
{
    int32_t ret;

    /* Disable Charger Mode */
//	rt9492_read_byte(RT9492_REG0F, &rt9492_reg[RT9492_REG0F*2]);
//	if((rt9492_reg[RT9492_REG0F*2] & 0x20) == 0x20)
//	{
//		 rt9492_reg[RT9492_REG0F*2] &= (~0x20);			//Set EN_CHG = 0
//		 rt9492_write_byte(RT9492_REG0F, rt9492_reg[RT9492_REG0F*2]);
//	}
    rt9490_update8(chgnum, RT9490_REG_CHG_CTRL0, 0x20, MASK_CLR);		//Set EN_CHG = 0


    /* Set PWM_FREQ = 1.0MHz */
//	rt9492_read_byte(RT9492_REG4B, &rt9492_reg[RT9492_REG4B*2]);
//	rt9492_reg[RT9492_REG4B*2] |= 0x10;
//	rt9492_write_byte(RT9492_REG4B, rt9492_reg[RT9492_REG4B*2]);
    rt9490_update8(chgnum, RT9490_REG_ADD_CTRL1, 0x10, MASK_SET);

    /* Disable HZ Mode*/
//	rt9492_read_byte(RT9492_REG0F, &rt9492_reg[RT9492_REG0F*2]);
//	rt9492_reg[RT9492_REG0F*2] &= ~0x04 ;				// Set EN_HZ= = 0 to disenable High Z mode
//	rt9492_write_byte(RT9492_REG0F, rt9492_reg[RT9492_REG0F*2]);
    rt9490_update8(chgnum, RT9490_REG_CHG_CTRL0, 0x04, MASK_CLR);

    /* Enable/Disable OTG */
    ret = rt9490_update8(chgnum, RT9490_REG_CHG_CTRL3, RT9490_EN_OTG,
                         enabled ? MASK_SET : MASK_CLR);

    return ret;
}

enum ec_error_list rt9490_set_otg_current_voltage(int chgnum,
        int output_current,
        int output_voltage)
{
    int ret = 0;
    uint16_t reg_cur, reg_vol;

    if (!IN_RANGE(output_current, RT9490_IOTG_MIN, RT9490_IOTG_MAX))
    {
        printf("Set Curr Error\n");
        return EC_ERROR_PARAM2;
    }
    if (!IN_RANGE(output_voltage, RT9490_VOTG_MIN, RT9490_VOTG_MAX))
    {
        printf("Set Volt Error\n");
        return EC_ERROR_PARAM3;
    }

    reg_cur = (output_current - RT9490_IOTG_MIN) / RT9490_IOTG_STEP + 3;
    reg_vol = (output_voltage - RT9490_VOTG_MIN) / RT9490_VOTG_STEP;

    RETURN_ERROR(rt9490_write8(chgnum, RT9490_REG_IOTG_REGU, reg_cur));
    printf("Set Reg OK Curr = %d. Volt = %d\n", reg_cur, reg_vol);
    ret = rt9490_write16(chgnum, RT9490_REG_VOTG_REGU, reg_vol);
    if( ret != EC_SUCCESS )
        return ret;

#if 0
    /* DIS_OTG_OOA */
    ret = rt9490_field_update8(chgnum, RT9490_REG_CHG_CTRL3, RT9490_DIS_VOTG_UVP_HICCUP_MASK, RT9490_DIS_OTG_OOA);
    if( ret != EC_SUCCESS )
    {
        printf("Set CHG_CTRL3 Error\n");
        return ret;
    }

    /* DIS_VOTG_UVP_HICCUP */
    ret = rt9490_field_update8(chgnum, RT9490_REG_CHG_CTRL4, RT9490_DIS_VOTG_UVP_HICCUP_MASK, RT9490_DIS_VOTG_UVP_HICCUP);
    if( ret != EC_SUCCESS )
    {
        printf("Set CHG_CTRL4 Error\n");
        return ret;
    }


    /* IBAT_REG */
    ret = rt9490_field_update8(chgnum, RT9490_REG_CHG_CTRL5, RT9490_IBAT_REG_MASK, RT9490_IBAT_REG_5A);
    if( ret != EC_SUCCESS )
    {
        printf("Set CHG_CTRL5 Error\n");
        return ret;
    }
#endif

    /* Disable JEDIA */
    //rt9490_enable_jeita()
    rt9490_field_update8(chgnum, RT9490_REG_JEITA_CTRL1, RT9490_JEITA_DIS, 1);


//    {
//        int cur = 0, vol = 0;
//        rt9490_read8(chgnum, RT9490_REG_IOTG_REGU, &cur);
//        rt9490_read16(chgnum, RT9490_REG_VOTG_REGU, &vol);
//        printf("read Reg Curr = %d. Volt = %d\n", cur, vol);
//    }
    return ret;
}

static int rt9490_is_sourcing_otg_power(int chgnum, int port)
{
    int val;

    if (rt9490_read8(chgnum, RT9490_REG_CHG_CTRL3, &val))
        return 0;
    return !!(val & RT9490_EN_OTG);
}






#endif

/* Reset all registers' value to default */
static int rt9490_reset_chip(int chgnum)
{
    /* disable hz before reset chip */
    RETURN_ERROR(rt9490_enable_hz(chgnum, false));

    return rt9490_set_bit(chgnum, RT9490_REG_EOC_CTRL, RT9490_RST_ALL_MASK);
}

static inline int rt9490_enable_chgdet_flow(int chgnum, bool en)
{
    return rt9490_update8(chgnum, RT9490_REG_CHG_CTRL2, RT9490_BC12_EN,
                          en ? MASK_SET : MASK_CLR);
}

//static inline int rt9490_enable_wdt(int chgnum, bool en)
int rt9490_enable_wdt(int chgnum, bool en)
{
    int val = en ? RT9490_WATCHDOG_40_SEC : RT9490_WATCHDOG_DISABLE;

    return rt9490_field_update8(chgnum, RT9490_REG_CHG_CTRL1,
                                RT9490_WATCHDOG_MASK, val);
}

static inline int rt9490_set_mivr(int chgnum, unsigned int mivr)
{
    uint8_t reg_mivr = mivr / RT9490_MIVR_STEP;

    return rt9490_write8(chgnum, RT9490_REG_MIVR_CTRL, reg_mivr);
}

static inline int rt9490_set_ieoc(int chgnum, unsigned int ieoc)
{
    uint8_t reg_ieoc = ieoc / RT9490_IEOC_STEP;

    return rt9490_field_update8(chgnum, RT9490_REG_EOC_CTRL,
                                RT9490_IEOC_MASK, reg_ieoc);
}

static inline int rt9490_enable_jeita(int chgnum, bool en)
{
    return rt9490_update8(chgnum, RT9490_REG_JEITA_CTRL1, RT9490_JEITA_DIS,
                          en ? MASK_CLR : MASK_SET);
}

int rt9490_enable_adc(int chgnum, bool en)
{
    return rt9490_update8(chgnum, RT9490_REG_ADC_CTRL, RT9490_ADC_EN,
                          en ? MASK_SET : MASK_CLR);
}

static int rt9490_set_iprec(int chgnum, unsigned int iprec)
{
    uint8_t reg_iprec = iprec / RT9490_IPRE_CHG_STEP;

    return rt9490_field_update8(chgnum, RT9490_REG_PRE_CHG,
                                RT9490_IPRE_CHG_MASK,
                                reg_iprec << RT9490_IPREC_SHIFT);
}
static void dump_range(int chgnum, int from, int to);


/**
	* Disable H/W or MCU to Control ACDRV1 and ACDRV2
  * The EN_ACDRV2 and EN_ACDRV1 will be set to 0.
  */
void rt9492_disable_hw_ctrl_gatedrive(int chgnum)
{
    rt9490_set_bit(chgnum, RT9490_REG_CHG_CTRL3, RT9492_DIS_ACDRV_EN);
}

/**
	* Enable H/W or MCU to Control ACDRV1 and ACDRV2
  * The EN_ACDRV2 and EN_ACDRV1 will be control by MCU or (VBUS voltage < VAC_UVLO_RISE (fixed) )
  */
void rt9492_enable_hw_ctrl_gatedrive(int chgnum)
{
    rt9490_clr_bit(chgnum, RT9490_REG_CHG_CTRL3, RT9492_DIS_ACDRV_EN);
}


void rt9492_turnon_gatedrive(int chgnum)
{
    rt9490_set_bit(chgnum, RT9490_REG_CHG_CTRL4, RT9490_EN_ACDRV1);
}

void rt9492_turnoff_gatedrive(int chgnum)
{
    rt9490_clr_bit(chgnum, RT9490_REG_CHG_CTRL4, RT9490_EN_ACDRV1);
}



static int rt9490_init_setting(int chgnum)
{
#if 0
    const struct battery_info *batt_info = battery_get_info();

#ifdef CONFIG_CHARGER_OTG
    /*  Disable boost-mode output voltage */
    RETURN_ERROR(rt9490_enable_otg_power(chgnum, 0));	//Ori
    //RETURN_ERROR(rt9490_enable_otg_power(chgnum, 1));	//SW
    RETURN_ERROR(rt9490_set_otg_current_voltage(
                     chgnum, rt9490_setting.boost_current,
                     rt9490_setting.boost_voltage));
#endif
    /* Disable ILIM_HZ pin current limit */
    RETURN_ERROR(rt9490_clr_bit(chgnum, RT9490_REG_CHG_CTRL5,
                                RT9490_ILIM_HZ_EN));
    /* Disable BC 1.2 detection by default. It will be enabled on demand */
    RETURN_ERROR(rt9490_enable_chgdet_flow(chgnum, false));
    /* Disable WDT */
    RETURN_ERROR(rt9490_enable_wdt(chgnum, false));
    /* Disable battery thermal protection */
    RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_ADD_CTRL0,
                                RT9490_JEITA_COLD_HOT));
    /* Disable AUTO_AICR / AUTO_MIVR */
    RETURN_ERROR(rt9490_clr_bit(chgnum, RT9490_REG_ADD_CTRL0,
                                RT9490_AUTO_AICR | RT9490_AUTO_MIVR));
    RETURN_ERROR(rt9490_set_mivr(chgnum, rt9490_setting.mivr));
    RETURN_ERROR(rt9490_set_ieoc(chgnum, rt9490_setting.eoc_current));
    RETURN_ERROR(rt9490_set_iprec(chgnum, batt_info->precharge_current));
    RETURN_ERROR(rt9490_enable_adc(chgnum, true));
    RETURN_ERROR(rt9490_enable_jeita(chgnum, false));
    RETURN_ERROR(rt9490_field_update8(
                     chgnum, RT9490_REG_CHG_CTRL1, RT9490_VAC_OVP_MASK,
                     RT9490_VAC_OVP_26V << RT9490_VAC_OVP_SHIFT));

    /* Mask all interrupts except BC12 done */
    RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_CHG_IRQ_MASK0,
                                RT9490_CHG_IRQ_MASK0_ALL));
    RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_CHG_IRQ_MASK1,
                                RT9490_CHG_IRQ_MASK1_ALL &
                                ~RT9490_BC12_DONE_MASK));
    RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_CHG_IRQ_MASK2,
                                RT9490_CHG_IRQ_MASK2_ALL));
    RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_CHG_IRQ_MASK3,
                                RT9490_CHG_IRQ_MASK3_ALL));
    RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_CHG_IRQ_MASK4,
                                RT9490_CHG_IRQ_MASK4_ALL));
    RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_CHG_IRQ_MASK5,
                                RT9490_CHG_IRQ_MASK5_ALL));
    /* Reduce SW freq from 1.5MHz to 1MHz
     * for 10% higher current rating b/215294785
     */
    RETURN_ERROR(rt9490_enable_pwm_1mhz(CHARGER_SOLO, true));

    /* b/230442545#comment28
     * Disable EOC-Force-CCM which would potentially
     * cause Vsys drop problem for all silicon version(ES1~ES4)
     */
    RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_CHG_CTRL2,
                                RT9490_DIS_EOC_FCCM));

    /* b/253568743#comment14 vsys workaround */
    RETURN_ERROR(rt9490_enable_hidden_mode(chgnum, true));
    rt9490_clr_bit(chgnum, RT9490_REG_HD_ADD_CTRL2,
                   RT9490_EN_FON_PP_BAT_TRACK);
    RETURN_ERROR(rt9490_enable_hidden_mode(chgnum, false));

    /* Disable non-standard TA detection */
    RETURN_ERROR(rt9490_clr_bit(chgnum, RT9490_REG_ADD_CTRL2,
                                RT9490_SPEC_TA_EN));
#else

    rt9492_dp_ctrl_dm_ctrl_hz(chgnum);
    rt9492_ts_adc_dis(chgnum);

    rt9492_disable_hw_ctrl_gatedrive(chgnum);

    rt9492_enable_adc(chgnum);

#endif
    /* Disable non-standard TA detection */
    //dump_range(chgnum, 0, 0x20);

    return EC_SUCCESS;
}

int rt9490_enable_hidden_mode(int chgnum, bool en)
{
    if (en) {
        RETURN_ERROR(
            rt9490_write8(chgnum, RT9490_REG_TM_PAS_CODE1, 0x69));
        RETURN_ERROR(
            rt9490_write8(chgnum, RT9490_REG_TM_PAS_CODE2, 0x96));
    } else {
        RETURN_ERROR(rt9490_write8(chgnum, RT9490_REG_TM_PAS_CODE1, 0));
        RETURN_ERROR(rt9490_write8(chgnum, RT9490_REG_TM_PAS_CODE2, 0));
    }

    return EC_SUCCESS;
}



int rt9490_enable_pwm_1mhz(int chgnum, bool en)
{
    return rt9490_update8(chgnum, RT9490_REG_ADD_CTRL1, RT9490_PWM_1MHZ_EN,
                          en ? MASK_SET : MASK_CLR);
}

int rt9490_set_snk_vbus_currentlimit(int chgnum, int curr)
{
    return rt9490_write16(chgnum, RT9490_REG_AICC_CTRL, curr);
}

static void rt9490_init(int chgnum)
{
    int ret = rt9490_init_setting(chgnum);

    CPRINTS("init%d %s(%d)", chgnum, ret ? "fail" : "good", ret);
}

static enum ec_error_list rt9490_get_status(int chgnum, int *status)
{
    int val = 0;

    *status = 0;

    RETURN_ERROR(rt9490_read8(chgnum, RT9490_REG_CHG_CTRL0, &val));
    if (!(val & RT9490_EN_CHG))
        *status |= CHARGER_CHARGE_INHIBITED;

    RETURN_ERROR(rt9490_read8(chgnum, RT9490_REG_FAULT_STATUS0, &val));
    if (val & RT9490_VBAT_OVP_STAT)
        *status |= CHARGER_VOLTAGE_OR;

    RETURN_ERROR(rt9490_read8(chgnum, RT9490_REG_CHG_STATUS4, &val));
    if (val & RT9490_JEITA_COLD_MASK) {
        *status |= CHARGER_RES_COLD;
        *status |= CHARGER_RES_UR;
    }
    if (val & RT9490_JEITA_COOL_MASK) {
        *status |= CHARGER_RES_COLD;
    }
    if (val & RT9490_JEITA_WARM_MASK) {
        *status |= CHARGER_RES_HOT;
    }
    if (val & RT9490_JEITA_HOT_MASK) {
        *status |= CHARGER_RES_HOT;
        *status |= CHARGER_RES_OR;
    }
    return EC_SUCCESS;
}

static int rt9490_reset_to_zero(int chgnum)
{
    RETURN_ERROR(rt9490_set_current(chgnum, 0));
    RETURN_ERROR(rt9490_set_voltage(chgnum, 0));
    RETURN_ERROR(rt9490_enable_hz(chgnum, true));

    return EC_SUCCESS;
}

static enum ec_error_list rt9490_set_mode(int chgnum, int mode)
{
    if (mode & CHARGE_FLAG_POR_RESET)
        RETURN_ERROR(rt9490_reset_chip(chgnum));
    if (mode & CHARGE_FLAG_RESET_TO_ZERO)
        RETURN_ERROR(rt9490_reset_to_zero(chgnum));
    return EC_SUCCESS;
}

static enum ec_error_list rt9490_get_actual_current(int chgnum, int *current)
{
    int reg_val;

    RETURN_ERROR(rt9490_read16(chgnum, RT9490_REG_IBAT_ADC, &reg_val));
    *current = (int)reg_val;
    return EC_SUCCESS;
}

static enum ec_error_list rt9490_get_actual_voltage(int chgnum, int *voltage)
{
    int reg_val;

    RETURN_ERROR(rt9490_read16(chgnum, RT9490_REG_VBAT_ADC, &reg_val));
    *voltage = (int)reg_val;
    return EC_SUCCESS;
}

static enum ec_error_list rt9490_discharge_on_ac(int chgnum, int enable)
{
    return rt9490_enable_hz(chgnum, enable);
}

static enum ec_error_list rt9490_get_vbus_voltage(int chgnum, int port,
        int *voltage)
{
    int reg_val;

    RETURN_ERROR(rt9490_read16(chgnum, RT9490_REG_VBUS_ADC, &reg_val));
    *voltage = (int)reg_val;
    return EC_SUCCESS;
}

static enum ec_error_list rt9490_set_input_current_limit(int chgnum,
        int input_current)
{
    int reg_val;

    input_current = CLAMP(input_current, RT9490_AICR_MIN, RT9490_AICR_MAX);
    reg_val = input_current / RT9490_AICR_STEP;
    return rt9490_write16(chgnum, RT9490_REG_AICR_CTRL, reg_val);
}

static enum ec_error_list rt9490_get_input_current_limit(int chgnum,
        int *input_current)
{
    int val = 0;

    RETURN_ERROR(rt9490_read16(chgnum, RT9490_REG_AICR_CTRL, &val));
    val = (val & RT9490_AICR_MASK) >> RT9490_AICR_SHIFT;
    val *= RT9490_AICR_STEP;
    *input_current = CLAMP(val, RT9490_AICR_MIN, RT9490_AICR_MAX);
    return EC_SUCCESS;
}

static enum ec_error_list rt9490_get_input_current(int chgnum,
        int *input_current)
{
    int reg_val;

    RETURN_ERROR(rt9490_read16(chgnum, RT9490_REG_IBUS_ADC,
                               &reg_val));
    *input_current = reg_val;
    return EC_SUCCESS;
}

static enum ec_error_list rt9490_device_id(int chgnum, int *id)
{
    RETURN_ERROR(rt9490_read8(chgnum, RT9490_REG_DEVICE_INFO, id));
    *id &= RT9490_DEVICE_INFO_MASK;
    return EC_SUCCESS;
}

#ifdef CONFIG_CHARGE_RAMP_HW
static enum ec_error_list rt9490_set_hw_ramp(int chgnum, int enable)
{
    if (enable) {
        RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_CHG_CTRL0,
                                    RT9490_EN_AICC));
        RETURN_ERROR(rt9490_set_bit(chgnum, RT9490_REG_CHG_CTRL0,
                                    RT9490_FORCE_AICC));
    } else {
        RETURN_ERROR(rt9490_clr_bit(chgnum, RT9490_REG_CHG_CTRL0,
                                    RT9490_EN_AICC));
    }
    return EC_SUCCESS;
}

static int rt9490_ramp_is_stable(int chgnum)
{
    int rv;
    int val = 0;

    rv = rt9490_read8(chgnum, RT9490_REG_CHG_CTRL0, &val);
    return !rv && !(val & RT9490_FORCE_AICC);
}

static int rt9490_ramp_is_detected(int chgnum)
{
    return true;
}

static int rt9490_ramp_get_current_limit(int chgnum)
{
    int rv;
    int input_current = 0;

    rv = rt9490_get_input_current_limit(chgnum, &input_current);
    return rv ? -1 : input_current;
}
#endif

static enum ec_error_list rt9490_get_option(int chgnum, int *option)
{
    /* Ignored: does not exist */
    *option = 0;
    return EC_SUCCESS;
}

static enum ec_error_list rt9490_set_option(int chgnum, int option)
{
    /* Ignored: does not exist */
    return EC_SUCCESS;
}

#ifdef CONFIG_CMD_CHARGER_DUMP
static void dump_range(int chgnum, int from, int to)
{
    for (int reg = from; reg <= to; ++reg) {
        int val = 0;

        if (!rt9490_read8(chgnum, reg, &val))
            printf("    0x%02x: 0x%02x", reg, val);
        else
            printf("    0x%02x: (error)", reg);
    }
}



#if 0
static void rt9490_dump_registers(int chgnum)
{
    uint16_t ts, tdie;

    printf("\nCHG_STATUS:\n\n");
    dump_range(chgnum, RT9490_REG_CHG_STATUS0, RT9490_REG_CHG_STATUS4);
    printf("\nFAULT_STATUS:\n\n");
    dump_range(chgnum, RT9490_REG_FAULT_STATUS0, RT9490_REG_FAULT_STATUS1);
    printf("\nIRQ_FLAG:\n\n");
    dump_range(chgnum, RT9490_REG_CHG_IRQ_FLAG0, RT9490_REG_CHG_IRQ_FLAG5);

    printf("\nCTRL:\n\n");
    dump_range(chgnum, RT9490_REG_CHG_CTRL0, RT9490_REG_CHG_CTRL5);

    rt9490_read16(chgnum, RT9490_REG_TS_ADC, &ts);
    printf("TS_ADC: %d.%d%%\n\n", ts / 10, ts % 10);
    rt9490_read16(chgnum, RT9490_REG_TDIE_ADC, &tdie);
    printf("TDIE_ADC: %d deg C\n\n", tdie);
}
#else
static void rt9490_dump_registers(int chgnum, int start_addr, int end_addr)
{
    uint32_t  u32Addr, u32Data;
    uint32_t  u32Line = 0;
    int val;
    printf("\n[CPU DUMP: 0x%x ~ 0x%x]\n", start_addr, end_addr);
    for (u32Addr = start_addr; u32Addr < end_addr; u32Addr += 1)
    {
        //u32Data = inp32(u32Addr);
        u32Data = rt9490_read8(chgnum, u32Addr, &val);
        if(u32Data != 0)
        {
            printf("Err\n");
        }
        if (u32Addr % 16 == 0)
        {
            printf("\n");
            printf("0x%08x --     ", u32Line * 16);
            u32Line = u32Line + 1;
        }
        printf("0x%02x ", val);
    }
    printf("\n\n");
}
#endif
#endif


/* Read VBAT (Volt) */
int rt9492_read_vbat(int chgnum)
{
    int adc_code;

    /* 1. Enable ADC first */
    rt9492_enable_adc(chgnum);

    /* 2. Read DC_BAT register */
    rt9490_read16(chgnum, RT9490_REG_VBAT_ADC, &adc_code);
    printf("VBAT = %d\n", adc_code);

    /* 4. Convert to Voltage (LSB: 1mV) */
    return adc_code;
}



const struct charger_drv rt9490_drv = {
    .init = &rt9490_init,
    .get_info = &rt9490_get_info,
    .get_status = &rt9490_get_status,
    .set_mode = &rt9490_set_mode,
#ifdef CONFIG_CHARGER_OTG
    .enable_otg_power = &rt9490_enable_otg_power,
    .set_otg_current_voltage = &rt9490_set_otg_current_voltage,
    .is_sourcing_otg_power = &rt9490_is_sourcing_otg_power,
#endif
    .get_current = &rt9490_get_current,		//charge current
    .set_current = &rt9490_set_current,
    .get_voltage = &rt9490_get_voltage,
    .set_voltage = &rt9490_set_voltage,
    .get_actual_current = &rt9490_get_actual_current,
    .get_actual_voltage = &rt9490_get_actual_voltage,
    .discharge_on_ac = &rt9490_discharge_on_ac,
    .get_vbus_voltage = &rt9490_get_vbus_voltage,
    .set_input_current_limit = &rt9490_set_input_current_limit,
    .get_input_current_limit = &rt9490_get_input_current_limit,
    .get_input_current = &rt9490_get_input_current,
    .get_option = &rt9490_get_option,
    .set_option = &rt9490_set_option,
    .device_id = &rt9490_device_id,
#ifdef CONFIG_CHARGE_RAMP_HW
    .set_hw_ramp = &rt9490_set_hw_ramp,
    .ramp_is_stable = &rt9490_ramp_is_stable,
    .ramp_is_detected = &rt9490_ramp_is_detected,
    .ramp_get_current_limit = &rt9490_ramp_get_current_limit,
#endif
#ifdef CONFIG_CMD_CHARGER_DUMP
    //.dump_registers = &rt9490_dump_registers,
    .dump_registers = &rt9492_dump_register,
#endif
    .set_snk_vbus_currentlimit = rt9490_set_snk_vbus_currentlimit
};
/**
  *
	000: Not charging (default)
	001: Trickle charge
	010: Pre-charge
	011: Fast charge (CC mode)
	100: Fast charge (CV mode)
	101: IEOC (EOC and TE = 0)
	110: Back-Ground charging
	111: Charge done
  **/
int Charger_get_charge_state(int chgnum, int* reg)
{
    RETURN_ERROR( rt9490_read8(chgnum, RT9490_REG_CHG_STATUS1, reg) );
    *reg = (*reg & 0xE0) >> 5;
    return EC_SUCCESS;
}

#ifdef CONFIG_USB_CHARGER
/* BC1.2 */
static int rt9490_get_bc12_ilim(enum charge_supplier supplier)
{
    switch (supplier) {
    case CHARGE_SUPPLIER_BC12_DCP:
    case CHARGE_SUPPLIER_BC12_CDP:
        return USB_CHARGER_MAX_CURR_MA;
    case CHARGE_SUPPLIER_BC12_SDP:
    default:
        return USB_CHARGER_MIN_CURR_MA;
    }
}

static enum charge_supplier rt9490_get_bc12_device_type(int chgnum)
{
    int reg, vbus_stat;

    if (rt9490_read8(chgnum, RT9490_REG_CHG_STATUS1, &reg))
        return CHARGE_SUPPLIER_NONE;

    vbus_stat = (reg & RT9490_VBUS_STAT_MASK) >> RT9490_VBUS_STAT_SHIFT;

    switch (vbus_stat) {
    case RT9490_SDP:
        CPRINTS("BC12 SDP");
        return CHARGE_SUPPLIER_BC12_SDP;
    case RT9490_CDP:
        CPRINTS("BC12 CDP");
        return CHARGE_SUPPLIER_BC12_CDP;
    case RT9490_DCP:
        CPRINTS("BC12 DCP");
        return CHARGE_SUPPLIER_BC12_DCP;
    default:
        CPRINTS("BC12 UNKNOWN 0x%02X", vbus_stat);
        return CHARGE_SUPPLIER_NONE;
    }
}

static void rt9490_update_charge_manager(int port,
        enum charge_supplier new_bc12_type)
{
    static enum charge_supplier current_bc12_type = CHARGE_SUPPLIER_NONE;

    if (new_bc12_type != current_bc12_type) {
        if (current_bc12_type >= 0)
            charge_manager_update_charge(current_bc12_type, port,
                                         NULL);

        if (new_bc12_type != CHARGE_SUPPLIER_NONE) {
            struct charge_port_info chg = {
                .current = rt9490_get_bc12_ilim(new_bc12_type),
                .voltage = USB_CHARGER_VOLTAGE_MV,
            };

            charge_manager_update_charge(new_bc12_type, port, &chg);
        }

        current_bc12_type = new_bc12_type;
    }
}

/* TODO: chgnum is not passed into the task, assuming only one charger */
#ifndef CONFIG_CHARGER_SINGLE_CHIP
#error rt9490 bc1.2 driver only works in single charger mode.
#endif

static void rt9490_usb_charger_task_init(const int port)
{
    rt9490_enable_chgdet_flow(CHARGER_SOLO, false);
}

static void rt9490_usb_charger_task_event(const int port, uint32_t evt)
{
    /*
     * b/193753475#comment33: don't trigger bc1.2 detection after
     * PRSwap/FRSwap.
     *
     * Note that the only scenario we want to catch is power role swap. For
     * other cases, `is_non_pd_sink` may have false positive (e.g.
     * pd_capable() is false during initial PD negotiation). But it's okay
     * to always trigger bc1.2 detection for other cases.
     */
    bool is_non_pd_sink = !pd_capable(port) &&
                          !usb_charger_port_is_sourcing_vbus(port) &&
                          pd_check_vbus_level(port, VBUS_PRESENT);

    /* vbus change, start bc12 detection */
    if (evt & USB_CHG_EVENT_VBUS) {
        if (is_non_pd_sink)
            rt9490_enable_chgdet_flow(CHARGER_SOLO, true);
        else
            rt9490_update_charge_manager(port,
                                         CHARGE_SUPPLIER_NONE);
    }

    /* detection done, update charge_manager and stop detection */
    if (evt & USB_CHG_EVENT_BC12) {
        enum charge_supplier supplier;

        if (is_non_pd_sink)
            supplier = rt9490_get_bc12_device_type(CHARGER_SOLO);
        else
            supplier = CHARGE_SUPPLIER_NONE;

        rt9490_update_charge_manager(port, supplier);
        rt9490_enable_chgdet_flow(CHARGER_SOLO, false);
    }
}

static atomic_t pending_events;

void rt9490_deferred_interrupt(void)
{
    atomic_t current = atomic_clear(&pending_events);

    for (int port = 0; port < CONFIG_USB_PD_PORT_MAX_COUNT; ++port) {
        int ret, irq_flag;

        if (!(current & BIT(port)))
            continue;

        if (bc12_ports[port].drv != &rt9490_bc12_drv)
            continue;

        /* IRQ flag is read clear, no need to write back */
        ret = rt9490_read8(CHARGER_SOLO, RT9490_REG_CHG_IRQ_FLAG1,
                           &irq_flag);
        if (ret)
            return;

        if (irq_flag & RT9490_BC12_DONE_FLAG)
            usb_charger_task_set_event(port, USB_CHG_EVENT_BC12);
    }
}
DECLARE_DEFERRED(rt9490_deferred_interrupt);

void rt9490_interrupt(int port)
{
    atomic_or(&pending_events, BIT(port));
    hook_call_deferred(&rt9490_deferred_interrupt_data, 0);
}

const struct bc12_drv rt9490_bc12_drv = {
    .usb_charger_task_init = rt9490_usb_charger_task_init,
    .usb_charger_task_event = rt9490_usb_charger_task_event,
};

#ifdef CONFIG_BC12_SINGLE_DRIVER
/* provide a default bc12_ports[] for backward compatibility */
struct bc12_config bc12_ports[CHARGE_PORT_COUNT] = {
    [0 ... (CHARGE_PORT_COUNT - 1)] = {
        .drv = &rt9490_bc12_drv,
    },
};
#endif /* CONFIG_BC12_SINGLE_DRIVER */
#endif /* CONFIG_USB_CHARGER */

#if SW
int rt9490_get_thermistor_val(const struct temp_sensor_t *sensor, int *temp_ptr)
{
    uint16_t mv;
    int idx = sensor->idx;
#if IS_ENABLED(CONFIG_ZEPHYR) && IS_ENABLED(CONFIG_TEMP_SENSOR)
    const struct thermistor_info *info = sensor->zephyr_info->thermistor;
#else
    const struct thermistor_info *info = &rt9490_thermistor_info;
#endif

    if (idx != 0)
        return EC_ERROR_PARAM1;
    RETURN_ERROR(rt9490_read16(idx, RT9490_REG_TS_ADC, &mv));
    *temp_ptr = thermistor_linear_interpolate(mv, info);
    *temp_ptr = C_TO_K(*temp_ptr);
    return EC_SUCCESS;
}
#endif