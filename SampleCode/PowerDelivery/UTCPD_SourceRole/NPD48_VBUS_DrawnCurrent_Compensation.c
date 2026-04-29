/* Copyright 2012 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Monitor VBUS module for vary loading */
#include "utcpdlib.h"
#include "usb_pd.h"


extern volatile uint32_t gu32TimeBase;
extern volatile uint32_t u32RecVolt;
uint32_t gu32CompTimeBase = 0;
uint32_t gu32CompDac0 = 0;


#define E_MOVING_AVE_SAMPLE  4

#if 1
uint32_t gu32CurrentReadTimeBase = 0;
uint32_t gi32NewestIfbCurr = 0;
uint32_t gi32AverageIfbCurr = 0;
uint32_t gu32Array[E_MOVING_AVE_SAMPLE] = {0};
uint32_t gidx=0;
uint16_t gu16OcpThreshold;

CODE_LDROM void vbus_current_clear(int port)
{
		int i;
		for(i=0; i<E_MOVING_AVE_SAMPLE; i = i+1)
			gu32Array[i] = 0;
	
		gidx = 0;
		gi32AverageIfbCurr = 0; 
}

CODE_LDROM void vbus_set_overcurrent_threshold(int port, uint32_t u32PdoIdx, uint16_t u16PdoCurrent)
{
#if 1 //by polling
		gu16OcpThreshold = u16PdoCurrent*110/100;
		if(gu16OcpThreshold < 300)	
				gu16OcpThreshold += 300; 
		DBG_PRINTF("Req Current, OCP Current = %d / %d\n", u16PdoCurrent, gu16OcpThreshold); 
		
		vbus_current_clear(port);
#else
    //By interrupt
//	DBG_PRINTF("OCP TH [%d][%d]= %d\n", u32PdoIdx, u8CurrIdx, gu16OcpThreshold);
//	gu16OcpThreshold = gu16OcpThreshold + 500;	/* +500mA guard band */
//
//	i2c_update8(port, NULL, NPD48_PEIE, BIT2,  MASK_CLR);	// Interrupt Disable
//	i2c_update8(port, NULL, NPD48_PECTL, BIT2, MASK_CLR);   // SYS_OCP function disable
//	i2c_update8(port, NULL, NPD48_PEIF, BIT2,  MASK_SET);	// Clear Flag First
//
//	u32Volt = gu16OcpThreshold*5*80/1000 ;
//	u32CmpValue = u32Volt*4096/2560;//Volt --> ADC value
//	i2c_write16(port, NULL, NPD48_OCPCMPL, u32CmpValue);
//
//	i2c_update8(port, NULL, NPD48_PEIE, BIT2,  MASK_SET);	// Interrupt Enable
//	i2c_update8(port, NULL, NPD48_PECTL, BIT2, MASK_SET);   // SYS_OCP function enable
#endif
}


//uint32_t gu32CompDac0 = 0;
CODE_LDROM uint32_t vbus_current_read(int port)
{
    /* Read the voltage on IFB pin */
    uint32_t u16OcpAdcData;
    uint32_t u32IfbV;
    int32_t i32CurrmA;
    int i, j;
    if(gu32TimeBase > (gu32CurrentReadTimeBase+10))
    {
        //Every 10ms
        gu32CurrentReadTimeBase = gu32TimeBase;
        u16OcpAdcData = NPD48_ADC_GET_CONVERSION_DATA(1);
        u32IfbV = u16OcpAdcData*2560/4096;
        i32CurrmA = u32IfbV*1000/80/5;
        //DBG_PRINTF("Current = %d\n", i32CurrmA);
        gu32Array[gidx] = i32CurrmA;
        gidx = gidx+1;
        if(gidx == E_MOVING_AVE_SAMPLE)
        {
            uint32_t ave=0;
            for(i=0; i<E_MOVING_AVE_SAMPLE; i=i+1)
            {
                ave = ave+gu32Array[i];
            }
            ave = ave/E_MOVING_AVE_SAMPLE;
            gi32AverageIfbCurr = (gi32AverageIfbCurr + ave)/2;

            //DBG_PRINTF("Ave Current = %d\n", gi32AverageIfbCurr);

            gidx = 0;
        }
    }
    return gi32AverageIfbCurr;
}
#endif

