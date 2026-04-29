/******************************************************************************
 * @file     NPD48_ChipInit.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/18 3:19p $
 * @brief    Initialize NPD48 Registers
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "utcpdlib.h"
#include "usb_pd.h"

void pd_protection_enable_int(int port, int data);

void NPD48_ChipReset(void)
{
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk));
    GPIO_SetPullCtl(PB, BIT5, GPIO_PUSEL_PULL_UP);
    /* Configure PB.5 as GPB Int and enable interrupt by Low Level trigger */
    GPIO_SetMode(PB, BIT5, GPIO_MODE_OUTPUT);
    PB5 = 0;
    DBG_PRINTF("***********************************************************************************\n");
    PB5 = 1;
}

void NPD48_ChipInit(int port)
{
    DBG_PRINTF("%s\n", __FUNCTION__);
    /* Chip Refer Voltage 2.56V and LDO 3.3V Enable. It needs to delay 30us at least */
    i2c_write8(port, NULL, NPD48_PWRCR, (NPD48_PWRCR_VRF256EN_Msk) );

    /* Chip Refer Voltage 2.56V and LDO 3.3V Enable. It needs to delay 30us at least */
    i2c_write8(port, NULL, NPD48_PWRCR, (NPD48_PWRCR_LDO33EN_Msk | NPD48_PWRCR_VRF256EN_Msk) );

    /**
      * Enable OPA0, OPA1 and PGA0
      * OPA0 for voltage feedback for request voltage from primary side
      * OPA1 for current feedback for request current from primary side
      * PGA0 for measuring the voltage on the shunt resistor.
      **/
    i2c_write8(port, NULL,  NPD48_DACOPCR, (NPD48_DACOPCR_OPA0EN_Msk | \
                                            NPD48_DACOPCR_OPA1EN_Msk | \
                                            NPD48_DACOPCR_PGA0EN_Msk));

    /* Chip clock is stop, Only useful if chip is unattached state and won't send I2C command */
    //i2c_write8(port, NULL,  NPD48_CLKCTL, 0x04);

    /* Gate Driver Clock Selection Bits = 2MHz */
    i2c_update8(port, NULL,  NPD48_CLKCTL, 0x40, MASK_SET);//2MHz (2.91)
//  i2c_update8(port, NULL,  NPD48_CLKCTL, 0xC0, MASK_CLR);//1MHz (3.16)
//	i2c_update8(port, NULL,  NPD48_CLKCTL, 0x80, MASK_SET);//500KHz (3.35)
//	i2c_update8(port, NULL,  NPD48_CLKCTL, 0xC0, MASK_SET);//250KHz  (3.41)

    /**
    	* Set H/W Current Limit
    	* However, Now BSP use software polling to monitor VBUS OCP
    	**/
    i2c_write8(port, NULL,  NPD48_WRPROT, 0x5A);		//Non-Protect NPD48_DAC1DBL register
#if (CONFIG_NPD48_CURRENT_LIMIT == 1)
    i2c_write16(port, NULL,  NPD48_DAC1DBL, 960);	/* 6.0A current limit */
    i2c_write16(port, NULL,  NPD48_DAC1DBL, 480);	/* 3A current limit */

    i2c_write16(port, NULL,  NPD48_DAC1DBL, 176);	/* 1.1A current limit */
    i2c_write16(port, NULL,  NPD48_DAC1DBL, 192);	/* 1.2A current limit ==> CC Error */

    i2c_write16(port, NULL,  NPD48_DAC1DBL, 336);	/* 2.1A current limit */
    i2c_write16(port, NULL,  NPD48_DAC1DBL, 352);	/* 2.2A current limit  */

    i2c_write16(port, NULL,  NPD48_DAC1DBL, 496);	/* 3.1A current limit */
    i2c_write16(port, NULL,  NPD48_DAC1DBL, 512);	/* 3.2A current limit */

    i2c_write16(port, NULL,  NPD48_DAC1DBL, 800);	/* 5A current limit */
    i2c_write16(port, NULL,  NPD48_DAC1DBL, 880);	/* 5.5A current limit */

    i2c_write16(port, NULL,  NPD48_DAC1DBL, 960);	/* 6.0A current limit */

#else
    i2c_write16(port, NULL,  NPD48_DAC1DBL, 960);	/* 6.0A current limit */
#endif
    //	i2c_write8(port, NULL,  NPD48_WRPROT, 0x00);		//Protect NPD48_DAC1DBL register




#if (CONFIG_NPD48_VBUS_OCP == 1)
#if 0
    i2c_write8(port, NULL,  NPD48_DACOPCR, NPD48_DACOPCR_PGA0EN_Msk | NPD48_DACOPCR_OPA1EN_Msk | \
               NPD48_DACOPCR_OPA0EN_Msk | NPD48_DACOPCR_DAC0EN_Msk);
#else
    /* Disable Current feedback (ACBEL)*/
    i2c_write8(port, NULL,  NPD48_DACOPCR,  NPD48_DACOPCR_PGA0EN_Msk | \
               NPD48_DACOPCR_OPA0EN_Msk | NPD48_DACOPCR_DAC0EN_Msk);
#endif
//    i2c_write8(port, NULL,  NPD48_PEIE,  NPD48_PEIE_OCPIE_Msk);	  /* OCP Protectopn Interrupt Enable */
//    i2c_write8(port, NULL,  NPD48_PECTL,  NPD48_PECTL_OCPEN_Msk);	/* OCP Protectopn Function Enable */

#if 0//Due to DRP board will hard fault 
    pd_protection_enable_int(port, BIT2);
#endif

#else
#if (CONFIG_NPD48_DISABLE_OPA1 == 1)
    /* Disable Current feedback */
    i2c_write8(port, NULL,  NPD48_DACOPCR, NPD48_DACOPCR_PGA0EN_Msk | \
               NPD48_DACOPCR_OPA0EN_Msk | NPD48_DACOPCR_DAC0EN_Msk);
#else
    i2c_write8(port, NULL,  NPD48_DACOPCR, NPD48_DACOPCR_PGA0EN_Msk | NPD48_DACOPCR_OPA1EN_Msk | \
               NPD48_DACOPCR_OPA0EN_Msk | NPD48_DACOPCR_DAC0EN_Msk);
#endif
    i2c_write8(port, NULL,  NPD48_PEIE,  0x0);	    /* Disable Protectopn Interrupt Enable */
    i2c_write8(port, NULL,  NPD48_PECTL,  0x0);	    /* Disable Protectopn Function Enable */
#endif


    /* ADC clock enable */
    i2c_update8(port, NULL, 0x107,  BIT0, MASK_SET);


    DBG_PRINTF("%s", "NPD48_ADC_Init\n");
    /* Init NPD48 ADC */
    NPD48_ADC_Init(port);

    DBG_PRINTF("%s", "UTCPD_OPEN\n");
    /* Init NPD48 UTCPC */
    UTCPD_Open(0);	/* Enable port 0 PHY */

    /* Disable Fault Interrupt */
    i2c_update16(0, NULL, NPD48_ALERTL, ((NPD48_ALERTH_FUTIS_Msk << 8) | NPD48_ALERTL_CCSCHIS_Msk), MASK_CLR);

    /* Disable All Fault Control */
    {
        int32_t i32faultctl;
        DBG_PRINTF("Init FaultCTL\n");
        tcpc_update8(0, NPD48_FAULTCTL, 0xFF, MASK_SET);	/* 1: Disable */
        tcpc_addr_read(0, NULL, NPD48_FAULTCTL, &i32faultctl);
        DBG_PRINTF("FAULTCTL = 0x%x\n", i32faultctl);
    }

    UTCPD_DisableAlertMask(0, UTCPD_ALERTM_FUTIE);

    DBG_PRINTF("Please enable EINT Interrupt\n");

    UTCPD_SetVbusPresentThreshold(0x0, 30);	//2.5/1024 = 0.00244V. 3.5V/20/0.00244 = 72.9/0.00244 = 71.72

#if 0
    tcpc_update8(port, NPD48_FSAMASK, NPD48_FSAMASK_FSDISRXEN_Msk, MASK_SET);
#endif

    /**
      * 2.56V/1024 = 0.0025V  (NPD48 ADC Reference Voltage = 2.56V)
      * VBUS pre-divider = 20 (External Voltage Divider)
      *	3.5/20 = 0.175V
      * 0.175V/0.0025 ~= 70.
      **/
    UTCPD_SetVbusPresentThreshold(0, 70);

    UTCPD_ForceDischargeActivePolarity(port, UTCPD_PCACTL_FORCEDLVL_LOW);

    DBG_PRINTF("%s", "UTCPD_ForceDischargeActivePolarity\n");

    //VBUS_Source_Level(port, 0);	/* Default VIN 5V, and disable SRC_Gate */

}

void NPD48_CertificationFineTune(int port)
{
    int data;

    /* Eye-Diagram */
    uint8_t u8ccifr  = (2 << 3 | 4); //F = ave 338ns, R = ave 146ns.       fail test 1
    //uint8_t u8ccifr  = (2<<3 | 2);  	//F = ave 337ns, R = ave 159ns.        fail test 2
    //uint8_t u8ccifr  = (2<<3 | 6);  	//F = ave 339ns, R = ave 287ns.      fail test 3
    //uint8_t u8ccifr  = (0<<3 | 0);   //F = ave 344ns, R = ave 682ns.       fail test 4
    //uint8_t u8ccifr  = (7<<3 | 7);   //fail test 5
    DBG_PRINTF("%s\n", __FUNCTION__);

    i2c_write8(port, NULL, TCPC_REG_CCTRIM, u8ccifr );    /* Fastest BMC Falling and Rising Time */
    i2c_read8(port, NULL, TCPC_REG_CCTRIM, &data);



    i2c_write8(port, NULL, 0x9D, 0x48);    /* 6us each step */
    i2c_read8(port, NULL, 0x9D, &data);


    /* Bus Idle Select */
    i2c_read8(port, NULL, 0x8B, &data);
    i2c_write8(port, NULL, 0x8B, (data & ~0xC0) | (0 << 6) );

    //i2c_write8(port, NULL, TCPC_REG_PREDET1, 0x27);    /* 5 bits number and check 7 rounds */
    i2c_write8(port, NULL, TCPC_REG_PREDET1, 0x55);    /* 8 bits number and check 7 rounds */
    i2c_read8(port, NULL, TCPC_REG_PREDET1, &data);
    if(data != 0x27 )
        DBG_PRINTF("Error TCPC_REG_PREDET1\n");

    i2c_write8(port, NULL, TCPC_REG_PREDET2, 0xF0);	/* Start detect after 7 bits */
    i2c_read8(port, NULL, TCPC_REG_PREDET2, &data);
    if(data != 0xF0 )
        DBG_PRINTF("Error TCPC_REG_PREDET2\n");

    /* Enable CRC error */
    i2c_read8(port, NULL, NPD48_FSAMASK, &data);
    i2c_write8(port, NULL, NPD48_FSAMASK, data | 0x80);

    i2c_write8(port, NULL, NPD48_EARLYTM, 0x97);	//4.3A 1 min.
//	i2c_write8(0, NULL, NPD48_EARLYTM, 0xA8);	//4.2A 1 min.
//	i2c_write8(0, NULL, NPD48_DIVIDER, 0xA4);

    //TRIM VREF again.
    i2c_write8(port, NULL, 0xD2, 0x81);
		
		/* Disable FRS RX */
    i2c_update8(port, NULL, NPD48_FSTXCTL, (BIT3), MASK_CLR);
		
    DBG_PRINTF("%s\n", "done\n");
}