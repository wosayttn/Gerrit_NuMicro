/**************************************************************************//**
 * Copyright (c) 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "utcpdlib.h"
#include "charger.h"

int rt9490_enable_wdt(int chgnum, bool en);

#define DBG_PRINTF(...)
/*******************************************************************************
 * VBUS_SRC_EN: PA2 active high                                                *
 * VBUS can be measure on JP27.1                                               *
 *******************************************************************************/
static void VBUS_CMD_Enable_Source_VBus(int port)
{
    DBG_PRINTF("E SRC VBUS\n");
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_ENABLE_SRC_VBUS);
}
static void VBUS_CMD_Disable_Source_VBus(int port)
{
    DBG_PRINTF("D SRC VBUS\n");
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_DISABLE_SRC_VBUS);
}
/*******************************************************************************
 * VBUS_SINK_EN: PA3 active high                                               *
 * VBUS can be measure on JP27.2                                               *
 *******************************************************************************/
static void VBUS_CMD_Enable_Sink_VBus(int port)
{
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_ENABLE_SNK_VBUS);
}
static void VBUS_CMD_Disable_Sink_VBus(int port)
{
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_DISABLE_SNK_VBUS);
}

void VBUS_Sink_Enable(int32_t port, bool bIsEnable)
{
    if(bIsEnable)
    {
        VBUS_CMD_Enable_Sink_VBus(port);
        Charger_disable_otg_mode(0);
        Charger_enable_charge_mode(0);
        void rt9492_enable_hw_ctrl_gatedrive(int chgnum);
        rt9492_enable_hw_ctrl_gatedrive(0);
        void rt9492_turnon_gatedrive(int chgnum);
        rt9492_turnon_gatedrive(0);
    }
    else
    {
        VBUS_CMD_Disable_Sink_VBus(port);
        Charger_disable_charge_mode(0);
    }
}


void vbus_auto_discharge(uint32_t u32IsEnable)
{
    if(u32IsEnable == 1)
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) | TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT));
    else
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) & ~TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT));
}

void vbus_bleed_discharge(uint32_t u32IsEnable)
{
    if(u32IsEnable == 1)
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) | TCPC_REG_POWER_CTRL_BLEED_DISCHARGE));
    else
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) & ~TCPC_REG_POWER_CTRL_BLEED_DISCHARGE));
}

void vbus_force_discharge(uint32_t u32IsEnable)
{
    if(u32IsEnable == 1)
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) | TCPC_REG_POWER_CTRL_FORCE_DISCHARGE));
    else
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) & ~TCPC_REG_POWER_CTRL_FORCE_DISCHARGE));
}

static char i8RecLevel = 0xFF;
/* Define Periphal */


#define NUMAKER_DRP_V1_1			(1)

#if (NUMAKER_DRP_V1_1 == 1)
#define PIN_RT9492_CE   						PA6			//PA0 is connected to PIN_RT9492_CE pin.
#define PIN_LEDG1										PB6			//PIN_LEDG1 (Green), Set 0 to light on, set 1 to turn off.
#define PIN_LEDR1										PB7			//PIN_LEDR1 (Red), Set 0 to light on, set 1 to turn off.
#define PIN_DISCHARGE_POWER_PATH		PC4			//NMOS_Q4 control the power path. Set 0 to turn OFF the power path	
#define PIN_LARGECAP_Q3							PA9			//PIN_LARGECAP_Q3(NMOS_Q3) control the Large Cap on Vbus. Set 0 to turn OFF
#define RT9492_STAT_OTG							PA2			//With external pull-up, Input: detect state, output: control OTG
#define	RT9492_ILIM_HZ	PA11								//PA11 is connected to RT9492_ILIM_HZ ==> Deprecated 
#endif


extern int32_t pd_get_deadbattry_threshold(int chgnum);

void VBUS_Source_Level(int port, char i8Level)
{
    /* Judgement if dead battery mode */
    int chgnum = port; 															/* Assume, one port for one charger manager */
    if(rt9492_read_vbat(chgnum) < pd_get_deadbattry_threshold(chgnum))
    {
        /* Set RT9492 /CE to disable/Enable Charge Mode */
        GPIO_SetMode(PA, BIT6, GPIO_MODE_QUASI);		//PA6 is wired to RT9492./CE pin ( /Charge Enable) with extrnal 10K pull-R
        Charger_enable_charge_mode(chgnum);
        PIN_RT9492_CE = 0; 													//Set 0 to enable Charge function
        return;
    }

    VBUS_Sink_Enable(port, 0);      								/* Disable VBSNNKEN pin */
    if(i8RecLevel == i8Level)
        return;

    /* Set RT9492 /CE to disable/Enable Charge Mode */
    GPIO_SetMode(PA, BIT6, GPIO_MODE_QUASI);			//PA0 is wired to RT9492./CE pin ( /Charge Enable) with extrnal 10K pull-R
    Charger_disable_charge_mode(0);
    PIN_RT9492_CE = 1; 		//Set 1 to disable Charge function


    //--Set LEG_G & LEG_R ---//
    GPIO_SetMode(PB, BIT6 | BIT7, GPIO_MODE_QUASI); //PIN_LEDG1(PB6) & PIN_LEDR1(PB7)
    PIN_LEDG1 = 0; 																	//Set 0 to light on PIN_LEDG1 (Green)
    PIN_LEDR1 = 1;																	//Set 1 to turn off PIN_LEDR1 (Red)


    /* Set RT9492./CE to disable/Enable Charge Mode */
    GPIO_SetMode(PA, BIT6, GPIO_MODE_QUASI);				//PA0 is wired to RT9492./CE pin ( /Charge Enable) with extrnal 10K pull-R
    PIN_RT9492_CE = 1; 															//Set 1 to disable Charge function

    //--Setting for Q1 NMOS (Power path) ---//
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);				//Q1 NMOS
    PIN_DISCHARGE_POWER_PATH = 0;										//Turn OFF the NMOS(Q1) to turn off the Power PMOS

    //--Setting for Q3 NMOS (Large Cap on Vbus) ---//
    GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);				//Q3 NMOS
    PIN_LARGECAP_Q3 = 0;														//Turn OFF the NMOS_Q3

    /* Set RT9492.STAT_OTG as Quasi-High */
    GPIO_SetMode(PA, BIT2, GPIO_MODE_QUASI);				//PA2 wired to RT9492.STAT_OTG pin with extrnal 10K pull-R
    RT9492_STAT_OTG = 1; 														//Set 1 to keep Quasi-high state. enablt OTG mode

    if(i8Level  == 0)
    {
        //0V
        Charger_enable_otg_mode(0);

        Charger_set_otg_current_voltage(0, 3200, 5000);
        printf("OTG Enable\n");
        rt9490_enable_wdt(0, 0);
        printf("Power lvl 0\n");

        void rt9492_disable_hw_ctrl_gatedrive(int chgnum);
        rt9492_disable_hw_ctrl_gatedrive(0);

        void rt9492_turnoff_gatedrive(int chgnum);
        rt9492_turnoff_gatedrive(0);

        VBUS_CMD_Disable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Disable\n");
        i8RecLevel = 0;
        //Charger_dump_register(0);

    }
    else if(i8Level  == 1)
    {
        //5V
        /* Set OTG mode needs prior to enable hw control gatedriver */
        Charger_enable_otg_mode(0);
        Charger_set_otg_current_voltage(0, 3200, 5000);
        printf("OTG Enable\n");
        rt9490_enable_wdt(0, 0);
        printf("Power lvl 1\n");

        void rt9492_enable_hw_ctrl_gatedrive(int chgnum);
        rt9492_enable_hw_ctrl_gatedrive(0);
        void rt9492_turnon_gatedrive(int chgnum);
        rt9492_turnon_gatedrive(0);

        DBG_PRINTF("Buck output 5V\n");
        VBUS_CMD_Enable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Enable\n");
        i8RecLevel = 1;
    }
    else if(i8Level  == 2)
    {
        //9V
        Charger_set_otg_current_voltage(0, 1600, 9000);
        Charger_enable_otg_mode(0);
        printf("OTG Enable\n");
        rt9490_enable_wdt(0, 0);
        printf("Power lvl 2\n");

        VBUS_CMD_Enable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Enable\n");
        i8RecLevel = 2;
    }
    else if(i8Level  == 3)
    {
        //15V
        Charger_set_otg_current_voltage(0, 1200, 15000);
        Charger_enable_otg_mode(0);
        printf("OTG Enable\n");
        rt9490_enable_wdt(0, 0);
        printf("Power lvl 3\n");

        DBG_PRINTF("Buck output 20V\n");
        VBUS_CMD_Enable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Enable\n");
        i8RecLevel = 3;
    }
    else if(i8Level  == 4)
    {
        //PPS 3.3V ~ 15V
        Charger_set_otg_current_voltage(0, 1200, 15000);
        Charger_enable_otg_mode(0);
        printf("OTG Enable\n");
        rt9490_enable_wdt(0, 0);
        printf("Power lvl 3\n");

        DBG_PRINTF("Buck output 20V\n");
        VBUS_CMD_Enable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Enable\n");
        i8RecLevel = 3;
    }


    /* Set VBUS OCP Threshold */
    void vbus_set_overcurrent_threshold(int port, uint32_t u32PdoIdx);
    vbus_set_overcurrent_threshold(port, i8RecLevel);
}
