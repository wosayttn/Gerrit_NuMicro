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

#define MAD025_UTCPD  0
#define MAD026_UTCPD  1

#if (MAD025_UTCPD == 1)
#define SOURCEDC_PORT		PE
#define SOURCEDC_PIN		BIT11
#define SOURCEDC            PE11

#define GPIO0_PORT		    PE
#define GPIO0_PIN           BIT13
#define GPIO0               PE13

#define GPIO1_PORT		    PE
#define GPIO1_PIN           BIT12
#define GPIO1               PE12
#endif

#if (MAD026_UTCPD == 1)
#define SOURCEDC_PORT		PB
#define SOURCEDC_PIN		BIT6
#define SOURCEDC            PB6

#define GPIO0_PORT		    PF
#define GPIO0_PIN           BIT6
#define GPIO0               PF6

#define GPIO1_PORT		    PD
#define GPIO1_PIN           BIT15
#define GPIO1               PD15
#endif
/*******************************************************************************
 * Base on TC8260_UTCPD_BOARD_V1                                                *
 * Design by MS50 THWang                                                       *
 *                                                                             *
 *  VBUS_Source_Level --> Specified the VBUS level                             *
 *******************************************************************************/
/*******************************************************************************
 * VBUS Enable Output Active High
 * Control signal: SOURCE_DC/DC_EN (PE11)
 * The signal should be replaced by VBSRCEN
 *******************************************************************************/
static void VBUS_Enable_Output(int port)
{
    if(port == 0)
    {
        SOURCEDC = 1;
        //GPIO_SetMode(PE, BIT11, GPIO_MODE_OUTPUT);
        GPIO_SetMode(SOURCEDC_PORT, SOURCEDC_PIN, GPIO_MODE_OUTPUT);
    }
}
static void VBUS_Disable_Output(int port)
{
    if(port == 0)
    {
        SOURCEDC = 0;
        // GPIO_SetMode(PE, BIT11, GPIO_MODE_OUTPUT);
        GPIO_SetMode(SOURCEDC_PORT, SOURCEDC_PIN, GPIO_MODE_OUTPUT);
    }
}

/*******************************************************************************
 * VBUS_SRC_EN: PA2 active high                                                *
 * VBUS can be measure on JP27.1                                               *
 *******************************************************************************/
static void VBUS_CMD_Enable_Source_VBus(int port)
{
    DBG_PRINTF("E SRC VBUS\n");
#if 1
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_ENABLE_SRC_VBUS);
#else
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 1;
#endif
}
static void VBUS_CMD_Disable_Source_VBus(int port)
{
    DBG_PRINTF("D SRC VBUS\n");
#if 1
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_DISABLE_SRC_VBUS);
#else
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 0;
#endif
}
/*******************************************************************************
 * VBUS_SINK_EN: PA3 active high                                               *
 * VBUS can be measure on JP27.2                                               *
 *******************************************************************************/
static void VBUS_CMD_Enable_Sink_VBus(int port)
{
#if 1
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_ENABLE_SNK_VBUS);
#else
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 1;
#endif
}
static void VBUS_CMD_Disable_Sink_VBus(int port)
{
#if 1
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_DISABLE_SNK_VBUS);
#else
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 0;
#endif
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
#if (NUMAKER_DRP_V1_0 == 1)
#define PIN_RT9492_CE   						PA0			//PA0 is connected to PIN_RT9492_CE pin.
#define PIN_LEDG1										PB6			//PIN_LEDG1 (Green), Set 0 to light on, set 1 to turn off.
#define PIN_LEDR1										PB7			//PIN_LEDR1 (Red), Set 0 to light on, set 1 to turn off.
#define PIN_DISCHARGE_POWER_PATH		PA3			//BUG: Didn't support VBUS discharge. Use sink enable Q1 to replace the VBUS discharge. It need to add a low resistor from V_LOAD to GND.  
#define LargeCap_Q3									PB0			//LargeCap_Q3(NMOS_Q3) control the Large Cap on Vbus. Set 0 to turn OFF
#define RT9492_STAT_OTG							PA2			//With external pull-up, Input: detect state, output: control OTG
#define	RT9492_ILIM_HZ	PA11								//PA11 is connected to RT9492_ILIM_HZ ==> Deprecated 

#endif
#if (NUMAKER_DRP_V1_1 == 1)
#define PIN_RT9492_CE   						PA6			//PA0 is connected to PIN_RT9492_CE pin.
#define PIN_LEDG1										PB6			//PIN_LEDG1 (Green), Set 0 to light on, set 1 to turn off.
#define PIN_LEDR1										PB7			//PIN_LEDR1 (Red), Set 0 to light on, set 1 to turn off.
#define PIN_DISCHARGE_POWER_PATH		PC4			////(NMOS_Q4) control the power path. Set 0 to turn OFF the power path	
#define LargeCap_Q3									PA9			//LargeCap_Q3(NMOS_Q3) control the Large Cap on Vbus. Set 0 to turn OFF
#define RT9492_STAT_OTG							PA2			//With external pull-up, Input: detect state, output: control OTG
#define	RT9492_ILIM_HZ	PA11								//PA11 is connected to RT9492_ILIM_HZ ==> Deprecated 
#endif

void VBUS_Source_Level(int port, char i8Level)
{
    //printf("%s %d\n", __FUNCTION__, i8Level);
    VBUS_Sink_Enable(port, 0);      /* Disable VBSNNKEN pin */
    if(i8RecLevel == i8Level)
        return;

    /* Set RT9492./CE to disable/Enable Charge Mode */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_QUASI);			//PA0 is wired to RT9492./CE pin ( /Charge Enable) with extrnal 10K pull-R
    Charger_disable_charge_mode(0);
    PIN_RT9492_CE = 1; 		//Set 1 to disable Charge function


    //--Set LEG_G & LEG_R ---//
    GPIO_SetMode(PB, BIT6 | BIT7, GPIO_MODE_QUASI); //PIN_LEDG1(PB6) & PIN_LEDR1(PB7)
    PIN_LEDG1 = 0; 		//Set 0 to light on PIN_LEDG1 (Green)
    PIN_LEDR1 = 1;		//Set 1 to turn off PIN_LEDR1 (Red)


    /* Set RT9492./CE to disable/Enable Charge Mode */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_QUASI);			//PA0 is wired to RT9492./CE pin ( /Charge Enable) with extrnal 10K pull-R
    PIN_RT9492_CE = 1; 		//Set 1 to disable Charge function

    //--Setting for Q1 NMOS (Power path) ---//
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);			//Q1 NMOS
    PIN_DISCHARGE_POWER_PATH = 0;		//Turn OFF the NMOS(Q1) to turn off the Power PMOS

    //--Setting for Q3 NMOS (Large Cap on Vbus) ---//
    GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);			//Q3 NMOS
    LargeCap_Q3 = 0;		//Turn OFF the NMOS_Q3

    /* Set RT9492.STAT_OTG as Quasi-High */
    GPIO_SetMode(PA, BIT2, GPIO_MODE_QUASI);			//PA0 i2 wired to RT9492.STAT_OTG pin with extrnal 10K pull-R
    RT9492_STAT_OTG = 1; 		//Set 1 to keep Quasi-high state. enablt OTG mode
    
    if(i8Level  == 0)
    {
        //0V
#if 0
        VBUS_Disable_Output(port);
        DBG_PRINTF("Buck output disable\n");
#else
        Charger_enable_otg_mode(0);
			
				Charger_set_otg_current_voltage(0, 3200, 5000);
        printf("OTG Enable\n");
        rt9490_enable_wdt(0, 0);
        printf("Power lvl 0\n");
			
				void rt9492_disable_hw_ctrl_gatedrive(int chgnum);
        rt9492_disable_hw_ctrl_gatedrive(0);
			
				void rt9492_turnoff_gatedrive(int chgnum);
				rt9492_turnoff_gatedrive(0);
//				{
//					uint8_t u8data; 
//					rt9490_read8(0, RT9490_REG_CHG_CTRL3, &u8data);
//					u8data &= (~0x40);				//Clear bit6 to stop OTG
//					rt9490_write8(0, RT9490_REG_CHG_CTRL3, u8data);	
//				}
//			
				//PIN_DISCHARGE_POWER_PATH = 1;		//Turn ON the power path to discharge the large capacitors.
#endif
			
			
			
        VBUS_CMD_Disable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Disable\n");
        i8RecLevel = 0;
		//Charger_dump_register(0);
			
    }
    else if(i8Level  == 1)
    {
        //5V
			  
#if 0
        //VBUS_Disable_Output(port);
        GPIO_SetPullCtl(GPIO0_PORT, GPIO0_PIN, GPIO_PUSEL_DISABLE);
        GPIO_SetPullCtl(GPIO1_PORT, GPIO1_PIN, GPIO_PUSEL_DISABLE);
        GPIO_SetMode(GPIO0_PORT, GPIO0_PIN, GPIO_MODE_INPUT);
        GPIO_SetMode(GPIO1_PORT, GPIO1_PIN, GPIO_MODE_INPUT);
        delay(3);
        VBUS_Enable_Output(port);
#else
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
			
#endif
        DBG_PRINTF("Buck output 5V\n");
        VBUS_CMD_Enable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Enable\n");
        i8RecLevel = 1;
    }
    else if(i8Level  == 2)
    {
        //9V
#if 0
        //VBUS_Disable_Output(port);
        GPIO_SetPullCtl(GPIO0_PORT, GPIO0_PIN, GPIO_PUSEL_DISABLE);	//GPIO0
        GPIO_SetMode(GPIO0_PORT, GPIO0_PIN, GPIO_MODE_INPUT);

        GPIO_SetPullCtl(GPIO1_PORT, GPIO1_PIN, GPIO_PUSEL_PULL_UP);	//GPIO1
        GPIO_SetMode(GPIO1_PORT, GPIO1_PIN, GPIO_MODE_OUTPUT);
        GPIO1 = 0;

        VBUS_Enable_Output(port);
        DBG_PRINTF("Buck output 9V\n");
#else
        Charger_set_otg_current_voltage(0, 1600, 9000);
        Charger_enable_otg_mode(0);
        printf("OTG Enable\n");
        rt9490_enable_wdt(0, 0);
        printf("Power lvl 2\n");
#endif
        VBUS_CMD_Enable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Enable\n");
        i8RecLevel = 2;
    }
    else if(i8Level  == 3)
    {
        //20V
        //VBUS_Disable_Output(port);
#if 0
        /* Two steps adjust VBUS to 20V OK */
        GPIO_SetPullCtl(GPIO0_PORT, GPIO0_PIN, GPIO_PUSEL_DISABLE);	//GPIO0
        GPIO_SetMode(GPIO0_PORT, GPIO0_PIN, GPIO_MODE_INPUT);

        GPIO_SetPullCtl(GPIO1_PORT, GPIO1_PIN, GPIO_PUSEL_PULL_UP);	//GPIO1
        GPIO_SetMode(GPIO1_PORT, GPIO1_PIN, GPIO_MODE_OUTPUT);
        GPIO1 = 0;

        delay(3);   //if remove the delay, the protocol will generate error due to sink not to reply GoodCRC.

        GPIO_SetMode(GPIO0_PORT, GPIO0_PIN, GPIO_MODE_OUTPUT);	//GPIO0
        GPIO0 = 0;

        GPIO_SetPullCtl(GPIO1_PORT, GPIO1_PIN, GPIO_PUSEL_DISABLE);	//GPIO1
        GPIO_SetMode(GPIO1_PORT, GPIO1_PIN, GPIO_MODE_INPUT);
        VBUS_Enable_Output(port);
#else
        Charger_set_otg_current_voltage(0, 1200, 15000);
        Charger_enable_otg_mode(0);
        printf("OTG Enable\n");
        rt9490_enable_wdt(0, 0);
        printf("Power lvl 3\n");
#endif
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
char GetChar(void);
void VBUS_Source_Level_Item(int port)
{
    uint8_t ch;
#if 0
    VBUS_Disable_Output(port);  /* Disable VBUS output default */
#else
#if 0
    Charger_disable_charge_mode(0);
#endif
#endif
    while(1)
    {
        do
        {
            printf("[0] Disable VBUS Output                            \n");
            printf("[1] VBUS Output 5V                                 \n");
            printf("[2] VBUS Output 9V                                 \n");
            printf("[3] VBUS Output 12V                                \n");
            printf("[4] Dump RT9490 Registers                          \n");
            ch = getchar();
            printf("Input = %c\n", ch);

        } while( ((ch < '0') || (ch > '4')) && ((ch != 'q') && (ch != 'Q')));
#if 1
        switch(ch)
        {
        case '0':
            VBUS_Source_Level(port, 0);
            break;
        case '1':
            VBUS_Source_Level(port, 1);
            break;
        case '2':
            VBUS_Source_Level(port, 2);
            break;
        case '3':
            VBUS_Source_Level(port, 3);
            break;
        case '4':
            //Charger_dump_register(0, 0, 0x100);
						Charger_dump_register(0);
            break;
        case 'Q':
        case 'q':
            return;
        }
#endif
    }
}
#if 0
/*******************************************************************************
 * TC8260_UTCPD BOARD V1                                                        *
 * If set VBUS_SOURCE = 20V, Even disable VB_SRC_EN                            *
 * The VBUS_S+ Still up to 15.88V                                              *
 * It seems not protect the VBUS will be = 0, if SNK device plug out           *
 *                                                                             *
 * Use VBSRC_EN to replace SOURC_DC/DC_EN signal should be better              *
 *******************************************************************************/

void VBUS_SRC_Control(int port)
{
    uint8_t ch;

    printf("Set SRC VBUS Level 20V\n");
    VBUS_Source_Level(port, 3);

    while(1)
    {
        do
        {
            DBG_PRINTF("[0] Disable Source VBUS                            \n");
            DBG_PRINTF("[1] Enable  Source VBUS                            \n");
            ch = getchar();

        }
        while( ((ch < '0') || (ch > '1')) && ((ch != 'q') && (ch != 'Q')));

        switch(ch)
        {
        case '0':
            printf("VBUS on JP27.1 will be about 0V\n");
            VBUS_Disable_Output(port);
            VBUS_CMD_Disable_Source_VBus(port);

            /* The force discharge function will work if a Valid "Source-to-Sink connection" */
            SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(0xFFUL << (1 * 8))) | (17UL << (1 * 8));
            vbus_discharge_polarity_active_high();
            vbus_force_discharge(0);    //Force discharge bit won't be cleaned to 0 by hardware auto. Clear it first
            vbus_force_discharge(1);    //Enable force discharge

            break;
        case '1':
            printf("VBUS on JP27.1 will be about 20V\n");
            VBUS_Enable_Output(port);
            VBUS_CMD_Enable_Source_VBus(port);
            break;
        case 'Q':
        case 'q':
            return;
        }
    }
}

void VBUS_SNK_Control(void)
{
    uint8_t ch;
    int port  = 0;

    while(1)
    {
        do
        {
            DBG_PRINTF("[0] Disable Sink VBUS                            \n");
            DBG_PRINTF("[1] Enable  Sink VBUS                            \n");
            ch = getchar();

        }
        while( ((ch < '0') || (ch > '1')) && ((ch != 'q') && (ch != 'Q')));

        switch(ch)
        {
        case '0':
            VBUS_CMD_Disable_Sink_VBus(port);
            break;
        case '1':
            VBUS_CMD_Enable_Sink_VBus(port);
            break;
        case 'Q':
        case 'q':
            return;
        }
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////
// VCONN
//////////////////////////////////////////////////////////////////////////////////////////////

/* Enable/Disable Discharge Signal for UUTCPD0_DISCHG for VCONN */
void  vconn_disable_discharge(void)
{
    /* Disable VCONN discharge */
    outp32(UTCPD0_BASE + UTCPD_VCDGCTL, inp32(UTCPD0_BASE + UTCPD_VCDGCTL) & ~VCDGEN);
}
void  vconn_enable_discharge(void)
{
    /* Enable VCONN discharge */
    outp32(UTCPD0_BASE + UTCPD_VCDGCTL, inp32(UTCPD0_BASE + UTCPD_VCDGCTL) | VCDGEN);
}
void  vconn_discharge_polarity_active_low(void)
{
    /* VCONN discharge active low */
    outp32(UTCPD0_BASE + TCPC_REG_PINPL, inp32(UTCPD0_BASE + TCPC_REG_PINPL) & ~TCPC_REG_PINPL_VCDCHG);
}
void  vconn_discharge_polarity_active_high(void)
{
    /* VCONN discharge active low */
    outp32(UTCPD0_BASE + TCPC_REG_PINPL, inp32(UTCPD0_BASE + TCPC_REG_PINPL) | TCPC_REG_PINPL_VCDCHG);
}