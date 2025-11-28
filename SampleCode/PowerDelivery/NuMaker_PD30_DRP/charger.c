#include "utcpdlib.h"
#include "rt9490.h"

/* \include\charger.h */
//#define NUMAKER_DRP_V1_0			(1)
#define NUMAKER_DRP_V1_1			(1)
#if (NUMAKER_DRP_V1_0 == 1)
#define PIN_CE 													PA0
#define PIN_DISCHARGE_POWER_PATH   			PA3	//BUG: Didn't support VBUS discharge. Use sink enable Q1 to replace the VBUS discharge. It need to add a low resistor from V_LOAD to GND.  
#endif
#if (NUMAKER_DRP_V1_1 == 1)
#define PIN_CE 													PA6
#define PIN_DISCHARGE_POWER_PATH   			PC4	//(NMOS_Q4) control the power path. Set 0 to turn OFF the power path	
#endif


/**************************************************************************//**
 * @file     charger.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/18 3:19p $
 * @brief
 *           Interface between application and RT9492 drover
 * @note
 *
 * Copyright (c) 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

/**************************************************************************//**
 * PA0: /CE			Charger Enable, Low Active
 *
 *****************************************************************************/
#if 1
typedef struct {
    int i2c_port;
    uint16_t i2c_addr_flags;
    const struct charger_drv *drv;
} charger_config_t;
#else
struct charger_config_t {
    int i2c_port;
    uint16_t i2c_addr_flags;
    const struct charger_drv *drv;
} charger_config_t;
#endif

/* baseboard.c */
extern const struct charger_drv rt9490_drv;
const struct charger_config_t chg_chips[] = {
    {
        .i2c_port = 0, 					//I2C_PORT_CHARGER,		/* SW change to useless */
        .i2c_addr_flags = 0x53, //ISL923X_ADDR_FLAGS,
        .drv = &rt9490_drv,
    },
};


void Charger_init(int chgnum)
{
    /* /CE and VBUS Discharge pin */
#if (NUMAKER_DRP_V1_0 == 1)
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA3MFP_Msk));
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
#endif
#if (NUMAKER_DRP_V1_1 == 1)
    SYS->GPA_MFP1 = SYS->GPA_MFP1 & ~SYS_GPA_MFP1_PA6MFP_Msk;
    GPIO_SetMode(PA, BIT6, GPIO_MODE_OUTPUT);

    /* PA9 = 1 to enable big capacitor for RT9492 source current */
    /* PA9 = 0 default for inrusgh current */
    SYS->GPA_MFP2 = SYS->GPA_MFP2 & ~SYS_GPA_MFP2_PA9MFP_Msk;
    GPIO_SetMode(PA, BIT9, GPIO_MODE_OUTPUT);
    PA9 = 1;

    SYS->GPC_MFP1 = SYS->GPC_MFP1 & ~SYS_GPC_MFP1_PC4MFP_Msk;
    GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);
#endif

    chg_chips[chgnum].drv->init(chgnum);
}

void Charger_get_current(int chgnum, int* curr)
{
    chg_chips[chgnum].drv->get_current(chgnum, curr);
}
void Charger_get_voltage(int chgnum, int* volt)
{
    chg_chips[chgnum].drv->get_voltage(chgnum, volt);
}

void Charger_enable_charge_mode(int chgnum)
{
    /* /CE pin low : enable charger*/
    PIN_CE = 0;

    /* Set current !=0 will enable chargrer mode */
    chg_chips[chgnum].drv->set_current(chgnum, 3000);
}
void Charger_disable_charge_mode(int chgnum)
{
    /* /CE pin high : disable charger */
    PIN_CE = 1;

    /* Set current !=0 will disable chargrer mode */
    chg_chips[chgnum].drv->set_current(chgnum, 0);
}
void Charger_enable_otg_mode(int chgnum)
{
    /* Enaable OTG Power */
    chg_chips[chgnum].drv->enable_otg_power(chgnum, 1);
}
void Charger_disable_otg_mode(int chgnum)
{
    /* Disable OTG Power */
    chg_chips[chgnum].drv->enable_otg_power(chgnum, 0);
}
void Charger_set_otg_current_voltage(int chgnum, int output_current, int output_voltage)
{
    chg_chips[chgnum].drv->set_otg_current_voltage(chgnum, output_current, output_voltage);
}
//void Charger_dump_register(int chgnum, int begin, int end)
//{
//  chg_chips[chgnum].drv->dump_registers(chgnum, begin, end);
//}
void Charger_dump_register(int chgnum)
{
    chg_chips[chgnum].drv->dump_registers();
}

void Charger_get_input_current(int chgnum, int* ibus)
{
    chg_chips[chgnum].drv->get_input_current(chgnum, ibus);
}

void Charger_get_actual_current(int chgnum, int* ibat)
{
    chg_chips[chgnum].drv->get_actual_current(chgnum, ibat);
}

void Charger_get_actual_voltage(int chgnum, int* vbat)
{
    chg_chips[chgnum].drv->get_actual_voltage(chgnum, vbat);
}

/* Enable High-z Mode or not as sink role */
//void Charge_discharge_on_ac(int chgnum, bool en)

//int _i2c_update8(int slaveaddr, int offset, int mask, enum mask_update_action action);
int rt9490_update8(int chgnum, int reg, int mask,
                   enum mask_update_action action);
void Charger_vbus_highz_mode(int chgnum, bool en)
{
#if 0
    if(en == 1)
    {
#if 0
        //printf("Enable HZ Mode\n");
        rt9490_update8(chgnum, RT9490_REG_ADC_CTRL, RT9490_ADC_EN, MASK_CLR);	//disable adc
        rt9490_update8(chgnum, RT9490_REG_CHG_CTRL2, RT9490_BC12_EN, MASK_CLR);	//disable bc12
#endif
#if 0
        /* PA11 Connects to ILIM_HZ to make sure ILIM_HZ < 0.75V */
        GPIO_SetMode(PA, BIT11, GPIO_MODE_OUTPUT);
        PA11 = 0;
#else
        GPIO_SetMode(PB, BIT7, GPIO_MODE_OUTPUT);
        PB7 = 0;
        //printf("Enable HZ\n");
#endif
    }
    else
    {   /* Disable HZ mode */
#if 0
        PA11 = 1;
        GPIO_SetMode(PA, BIT11, GPIO_MODE_INPUT);
#else
        PB7 = 1;
        GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);
        //printf("Disable HZ\n");
#endif
#if 0
        rt9490_update8(chgnum, RT9490_REG_ADC_CTRL, RT9490_ADC_EN, MASK_SET);	//enable adc
        rt9490_update8(chgnum, RT9490_REG_CHG_CTRL2, RT9490_BC12_EN, MASK_SET);	//enable bc12
#endif
        //Charger_init(0);
    }

    chg_chips[chgnum].drv->discharge_on_ac(chgnum, en);
#endif
}

void Charger_set_snk_vbus_currentlimit(int chgnum, int curr)
{
    chg_chips[chgnum].drv->set_snk_vbus_currentlimit(chgnum, curr);
}

void charger_discharge(int enable)
{
    if(enable)
        PIN_DISCHARGE_POWER_PATH = 1;
    else
        PIN_DISCHARGE_POWER_PATH = 0;
}

