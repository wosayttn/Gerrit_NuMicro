/* Copyright 2019 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */
#include "NuMicro.h"
#include "utcpdlib.h"
#include "usb_pd.h"

static int volatile i32currvolt;
static int volatile i32tarvolt;
static int8_t volatile i8tarpdo;
uint32_t u32pdo_flag = 0;
#define PDO_TRANSITION_FALLING  (1<<0)
#define PDO_TRANSITION_UPDATED  (1<<1)

volatile uint32_t u32PdoChkFlags[CONFIG_USB_PD_PORT_MAX_COUNT] = {0};
void pdo_set_flags(int port, uint32_t mask)
{
    u32PdoChkFlags[port] |= mask;
}

void pdo_clr_flags(int port, uint32_t mask)
{
    u32PdoChkFlags[port] &= ~mask;
}

bool pdo_chk_flags(int port, uint32_t mask)
{
    return (u32PdoChkFlags[port] & mask) ? 1 : 0;
}



CODE_LDROM void vbus_auto_discharge(int port, uint32_t u32IsEnable)
{
    //TBD

}

void vbus_bleed_discharge(int port, uint32_t u32IsEnable)
{
    //TBD

}

void vbus_force_discharge(int port, uint32_t u32IsEnable)
{
    //TBD
    if(u32IsEnable)
        i2c_update8(port, NULL, NPD48_PWRCTL, NPD48_PWRCTL_FDGEN_Msk, MASK_SET);
    else
        i2c_update8(port, NULL, NPD48_PWRCTL, NPD48_PWRCTL_FDGEN_Msk, MASK_CLR);
}

void pd_vbus_discharge_start(int port)
{
    i2c_write8(port, NULL, NPD48_DISCCTL, (BIT4 | BIT5));
    vbus_force_discharge(port, 0);	//Force discharge bit won't be cleaned to 0 by hardware auto. Clear it first
    vbus_force_discharge(port, 1);	//Enable force discharge
}
#if 0
#define DBG_RAMPDOWN		printf
#else
#define DBG_RAMPDOWN		DBG_PRINTF
#endif
void pd_vbus_discharge_stop(int port)
{
    i2c_write8(port, NULL, NPD48_DISCCTL, 0);
    vbus_force_discharge(port, 0);	//Force discharge bit won't be cleaned to 0 by hardware auto. Clear it first
}

/**
 * @brief       VBUS Falling Transition to Set Target Voltage and Current Voltage
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The API will be called as change
 */
void pdo_transition_set_target_level_and_start_discharge(int port, int i32currv, int i32targetv, int8_t i8pdo)
{
    i32currvolt = i32currv;
    i32tarvolt = i32targetv;
    i8tarpdo = i8pdo;
    DBG_RAMPDOWN("Falling Transition Start %d --> %d\n", i32currvolt, i32tarvolt);
    pdo_set_flags(port, PDO_TRANSITION_FALLING);
    pd_vbus_discharge_start(port);
}
/**
 * @brief       VBUS Falling Transition.
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The API will be called periodically every 10ms to ramp down VBUS.
 *              The API is for fixing the under shoot issue.
 * 							Ramp down method depends on the promary side. Users can modify it.
 */
void pdo_transition_falling_periodically(int port, uint32_t ticks)
{
    static uint32_t sticks = 0x0;
    if( pdo_chk_flags(port, PDO_TRANSITION_FALLING) == 0 )
    {
        //DBG_RAMPDOWN("==0\n");
        return;
    }
    if(sticks == ticks)
    {
        return;
    }
    sticks = ticks;

#if 0
    if( (i32currvolt != i32tarvolt) && (i32currvolt > i32tarvolt) )
    {
        //i32currvolt = i32currvolt - i32currvolt*1/2;
        i32currvolt = i32currvolt - 1000;
        DBG_RAMPDOWN("1. V = %d\n", i32currvolt);
    }
#endif
#if 0 //28-->9V and 36V --> 9V Undershoot
    if( (i32currvolt - i32tarvolt) > 2000)
    {
        //i32currvolt = i32currvolt - i32currvolt*1/2;
        i32currvolt = i32currvolt - 1500;
        DBG_RAMPDOWN("1. V = %d\n", i32currvolt);
    }
    else if( ((i32currvolt - i32tarvolt) <= 2000) && 	((i32currvolt - i32tarvolt) >= 250) )
    {
        i32currvolt = i32currvolt - 250;
        DBG_RAMPDOWN("V = %d\n", i32currvolt);
    }
#endif
#if 0
    if( (i32currvolt - i32tarvolt) > 4000)
    {
        //i32currvolt = i32currvolt - i32currvolt*1/2;
        i32currvolt = i32currvolt - 1000;
        DBG_RAMPDOWN("1. V = %d\n", i32currvolt);
    }
    else if( ((i32currvolt - i32tarvolt) <= (4000)) && 	((i32currvolt - i32tarvolt) > 1000) )
    {
        i32currvolt = i32currvolt - 1000;
        DBG_RAMPDOWN("V = %d\n", i32currvolt);
    }
    else if( ((i32currvolt - i32tarvolt) <= 1000) && ((i32currvolt - i32tarvolt) > 500) )
    {
        i32currvolt = i32currvolt - 500;
        DBG_RAMPDOWN("V = %d\n", i32currvolt);
    }
#endif
#if 0
    if( (i32currvolt - i32tarvolt) > 3000)
    {
        //i32currvolt = i32currvolt - i32currvolt*1/2;
        i32currvolt = i32currvolt - 1000;
        DBG_RAMPDOWN("1. V = %d\n", i32currvolt);
    }
    else if( ((i32currvolt - i32tarvolt) <= (3000)) && 	((i32currvolt - i32tarvolt) > 750) )
    {
        i32currvolt = i32currvolt - 750;
        DBG_RAMPDOWN("V = %d\n", i32currvolt);
    }
#endif
#if 0
    if( (i32currvolt - i32tarvolt) > 3000)
    {
        //i32currvolt = i32currvolt - i32currvolt*1/2;
        i32currvolt = i32currvolt - 1000;
        DBG_RAMPDOWN("1. V = %d\n", i32currvolt);
    }
    else if( ((i32currvolt - i32tarvolt) <= (3000)) && 	((i32currvolt - i32tarvolt) > 2500) )
    {
        i32currvolt = i32currvolt - 500;
    }
    else if( ((i32currvolt - i32tarvolt) <= (2500)) && 	((i32currvolt - i32tarvolt) > 750) )
    {
        i32currvolt = i32currvolt - 1000;
        DBG_RAMPDOWN("V = %d\n", i32currvolt);
    }
#endif
#if 1
    if( (i32currvolt - i32tarvolt) > 3000)
    {
        //i32currvolt = i32currvolt - i32currvolt*1/2;
        i32currvolt = i32currvolt - 1000;
        DBG_RAMPDOWN("1. V = %d\n", i32currvolt);
    }
#endif
    /*
    else if ( (i32currvolt - i32tarvolt) > 2000)
    {
        i32currvolt = i32currvolt - 1000;
        DBG_RAMPDOWN("V = %d\n", i32currvolt);
    }
    else if( ((i32currvolt - i32tarvolt) <= 2000) && 	((i32currvolt - i32tarvolt) >= 250) )
    {
        i32currvolt = i32currvolt - 250;
        DBG_RAMPDOWN("V = %d\n", i32currvolt);
    }
    */
    else
    {
        //printf(" Tracking %d --> %d\n", i32currvolt, i32tarvolt);
        i32currvolt = i32tarvolt;
        pdo_clr_flags(port, PDO_TRANSITION_FALLING);
        DBG_RAMPDOWN("Falling Transition done\n");
        VBUS_Source_Level(port, i8tarpdo);

        return;
    }
    setVinVoltage(port, (uint32_t)i32currvolt);
    pdo_set_flags(port, PDO_TRANSITION_UPDATED);

    if(i32currvolt == i32tarvolt)
        pd_vbus_discharge_stop(port);
}