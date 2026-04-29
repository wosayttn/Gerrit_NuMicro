#include <stdio.h>
#include "NuMicro.h"
#include <stdbool.h>
#include "utcpdlib.h"
#include "usb_pd.h"
//#include "tcpci.h"
//#include "EMU.h"

#if 0
#define T_ENABLE_PROTECT_EVENT	  (10)	/* To enable protect events again after 10mss    */
#define T_DURATION_CNT            (30)  /* 30ms duration to check OVP/UVP count over 2 times
																						 The value will be more than T_ENABLE_PROTECT_EVENT*2 
																				   */
#else
#define T_ENABLE_PROTECT_EVENT		(10)		/* To enable protect events again after 10mss    */
#define T_DURATION_CNT            (60)    /* 60ms duration to check OVP/UVP count over 2 times
																						 The value will be more than T_ENABLE_PROTECT_EVENT*2 
																				   */
#endif


#define DBG_PRINTF(...)

volatile uint8_t protect_flag = 0;
#define PD_PROTECT_VIN_OVP_Flag 		(1<<0)
#define PD_PROTECT_VIN_UVP_Flag 		(1<<1)
#define PD_PROTECT_SYS_OCP_Flag 		(1<<2)
#define PD_PROTECT_NTCP_Flag 	    	(1<<3)
#define PD_PROTECT_CC_OVP_Flag 	    (1<<4)
#define PD_PROTECT_VCONN_OCP_Flag 	(1<<5)
#define PD_PROTECT_CHIP_OVP_Flag 		(1<<6)
#define PD_PROTECT_ITP_Flag         (1<<7)

extern uint32_t gi32AverageIfbCurr;
extern uint16_t gu16OcpThreshold;


/** NAME          Target          ADC Channel      PEIE      CMP
  * VIN_OVP	      VIN             0                0          OVPCMPL & OVPCMPH         // VBUS OVP
  * VIN_UVP       VIN             0                1          UVPCMPL & UVPCMPH         // VBUS UVP
  * SYS_OCP       IS+/IS-(IFB)    1                2          OCPCMPL & OCPCMPH         // SYS OCP
  * NTCP          NTC pin         2                3          NTCPCMPL & NTCPCMPH       // External Temperature Sensor
  * ITP           INTP            3                7          ITPCMPL & ITPCMPH.  		// Internal Temperature Sensor
  * COVP          VIN             6                6          COVPCMPL & COVPCMPH       // Chip OVP
  * CCOVP         CC pin          built-in         4          built-in                  // CC OVP
  * VCONN_OCP     VCONN           built-in         5          built-in                  // VCONN OCP
  * 	          VBUS            7
*/

CODE_LDROM void pd_protection_disable_int(int port, int data)
{
    i2c_update8(port, NULL, NPD48_PECTL, data,  MASK_CLR);
    i2c_update8(port, NULL, NPD48_PEIE, data,  MASK_CLR);
    i2c_write8(port, NULL, NPD48_PEIF, data);
    if(data == PD_PROTECT_NTCP_Flag)
    {
        int rdata;
        DBG_PRINTF("after disable NTCP\n");
        i2c_read8(port, NULL, NPD48_PECTL, &rdata);
        DBG_PRINTF("pectl = 0x%x\n", rdata);
        i2c_read8(port, NULL, NPD48_PEIE, &rdata);
        DBG_PRINTF("peie = 0x%x\n", rdata);
    }
}

CODE_LDROM void pd_protection_enable_int(int port, int data)
{
    i2c_write8(port, NULL, NPD48_PEIF, data);
    i2c_update8(port, NULL, NPD48_PECTL, data,  MASK_SET);
    i2c_update8(port, NULL, NPD48_PEIE, data,  MASK_SET);
}

/**
  * NPD48_PECTL
  *	7	    6	    5	   4	   3	    2	   1	    0
  *	ITPEN	COVPEN	VCOCPEN	CCOVPEN	NTCPEN	OCPEN	UVPEN	OVPEN
  *
  * NPD48_PEIE
  * 7	    6	    5	    4	    3	    2	    1	    0
  * ITPIE	COVPIE	VCOCPIE	CCOVPIE	NTCPIE	OCPIE	UVPIE	OVPIE
  *
  * UPD48_PEIF
  * 7	    6	    5	    4	    3	    2	    1	    0
  * ITPIF	COVPIF	VCOCPIF	CCOVPIF	NTCPIF	OCPIF	UVPIF	OVPIF

  **/

CODE_LDROM void pd_protection_handler(int port, uint32_t u32Alert)
{
    if(u32Alert & NPD48_PEIF_OVPIF_Msk)
    {
        //bit0
        DBG_PRINTF("VIN OVP Det\n");

#if (CONFIG_NPD48_VIN_OVP == 1)
        protect_flag |= PD_PROTECT_VIN_OVP_Flag;
#endif
        pd_protection_disable_int(port, PD_PROTECT_VIN_OVP_Flag);
        return;
    }
    if(u32Alert & NPD48_PEIF_UVPIF_Msk)
    {
        //bit1
        DBG_PRINTF("VIN UVP Det\n");
#if (CONFIG_NPD48_VIN_OVP == 1)
        protect_flag |= PD_PROTECT_VIN_UVP_Flag;
#endif
        pd_protection_disable_int(port, PD_PROTECT_VIN_UVP_Flag); 	/* If didn't disable UVP, it will appear again and again */
        return;
    }

    if(u32Alert & NPD48_PEIF_OCPIF_Msk)
    {
        /* Just disable OCP interrupt */
        i2c_update8(port, NULL, NPD48_PECTL, BIT2, MASK_CLR);	/* Disable VBUS OCP Interrupt */
        i2c_update8(port, NULL, NPD48_PEIE, BIT2, MASK_CLR);	/* Disable VBUS OCP Interrupt */
        i2c_update8(port, NULL, NPD48_PEIF, BIT2, MASK_SET); /* Clear VBUS OCP Flag only */
    }
#if (CONFIG_NPD48_NTCP == 1)
    if(u32Alert & NPD48_PEIF_NTCPIF_Msk)
    {
        //bit3
        DBG_PRINTF("NTCP Det\n");
        protect_flag |= PD_PROTECT_NTCP_Flag;
        pd_protection_disable_int(port, PD_PROTECT_NTCP_Flag); 	/* If didn't disable NTCP, it will appear again and again */
    }
#endif
#if (CONFIG_NPD48_CC_OVP == 1)
    if(u32Alert & NPD48_PEIF_CCOVPIF_Msk)
    {
        //bit4
        DBG_PRINTF("CC OVP Det\n");
        VBUS_CMD_Disable_Source_VBus(port);
        setVinVoltage(port, 5000);
//				pd_protection_disable_int(port, PD_PROTECT_CC_OVP_Flag); 	/* If didn't disable NTCP, it will appear again and again */
//        pd_protection_clear_all_int(port, PD_PROTECT_CC_OVP_Flag);
        SYS_ResetChip();
    }
#endif
#if (CONFIG_NPD48_VCONN_OCP == 1)
    if(u32Alert & NPD48_PEIF_VCOCPIF_Msk)
    {
        //bit5
        DBG_PRINTF("VCONN OCP Det\n");
        protect_flag |= PD_PROTECT_VCONN_OCP_Flag;
        pd_protection_disable_int(port, PD_PROTECT_VCONN_OCP_Flag); 	/* If didn't disable NTCP, it will appear again and again */
    }
#endif
#if (CONFIG_NPD48_CHIP_OVP == 1)
    if(u32Alert & NPD48_PEIF_COVPIF_Msk)
    {
        //bit6
        DBG_PRINTF("CHIP OVP Det\n");
        VBUS_CMD_Disable_Source_VBus(port);
        setVinVoltage(port, 5000);
        SYS_ResetChip();
    }
#endif
#if (CONFIG_NPD48_ITP == 1)
    if(u32Alert & NPD48_PEIF_ITPIF_Msk)
    {
        //bit7
        DBG_PRINTF("ITP Det\n");
        protect_flag |= PD_PROTECT_ITP_Flag;
        pd_protection_disable_int(port, PD_PROTECT_ITP_Flag); 	/* If didn't disable NTCP, it will appear again and again */
    }
#endif
}

uint32_t u32PeDisableFlag = 0;
extern volatile uint32_t gu32TimeBase;

#if 0
uint32_t volatile u32VIN_OVP_TS = 0x7FFFFFFF, u32VIN_UVP_TS = 0x7FFFFFFF;
uint8_t volatile u8UVPCnt = 0, u8OVPCnt = 0;
uint32_t volatile u32SYS_NTCP_TS = 0x7FFFFFFF, u32SYS_ITP_TS = 0x7FFFFFFF;
uint8_t volatile u8NTCPCnt = 0, u8ITPCnt = 0;
#else
uint32_t volatile u32Protect_TS[8] = {0x7FFFFFFF};
uint8_t volatile u8Protect_Cnt[8] = {0};
#endif
CODE_LDROM void protect_hard_reset_procedure(int port, int evt)
{
#if 0
    if(u32Protect_TS[evt] == 0x7FFFFFFF)
        u32Protect_TS[evt] = gu32TimeBase;
    u8Protect_Cnt[evt] = u8Protect_Cnt[evt] + 1;
    if((gu32TimeBase - u32Protect_TS[evt]) < T_DURATION_CNT)
    {
        //Under duation, check CNT continuous....
        if(u8Protect_Cnt[evt] > 1)
        {
            u8Protect_Cnt[evt] = 0;
            u32Protect_TS[evt] = 0x7FFFFFFF;
            DBG_PRINTF("Protect hard reset\n");
            pd_dpm_request(port, DPM_REQUEST_HARD_RESET_SEND);
        }
    }
    else
    {
        //The duration between current OVP time stamp and last OVP time stamp over the T_DURATION_CNT. Reset the cnt to 1
        u8Protect_Cnt[evt] = 1;
        u32Protect_TS[evt] = gu32TimeBase;
    }
#else
    if(u32Protect_TS[evt] == 0x7FFFFFFF)		/* The first time */
        u32Protect_TS[evt] = gu32TimeBase;
    u8Protect_Cnt[evt] = u8Protect_Cnt[evt] + 1;
    if((gu32TimeBase - u32Protect_TS[evt]) < T_DURATION_CNT)
    {
        //Under duation, check CNT continuous....
        if(u8Protect_Cnt[evt] > 4)
        {
            u8Protect_Cnt[evt] = 0;
            u32Protect_TS[evt] = 0x7FFFFFFF;
            DBG_PRINTF("Protect hard reset\n");
            pd_dpm_request(port, DPM_REQUEST_HARD_RESET_SEND);
        }
    }
    else
    {
        /*The gap between this time and the last time exceeds T_DURATION_CNT ==> reset */
        u8Protect_Cnt[evt] = 1;
        u32Protect_TS[evt] = gu32TimeBase;
    }
#endif
}

uint16_t volatile gu16OcpAdcValue = 0;
static char i8OcpCnt = 0;
/**
  *	Protect time stamp total 0xFFFFFFFF = 4294967295 ms.
  * 4294967295/1000=4284967 sec
  * 4284967/60/60/24 ~= 49 days.
  * It should be enough for USBC charger
  * or it should be to call SYS_ResetChip() if the time stamp over 0x7FFFFFFF.
  */
CODE_LDROM void pd_protection_run(int port)
{
    if( (protect_flag & PD_PROTECT_VIN_OVP_Flag) != 0 )
    {
        //By interrupt
        DBG_PRINTF("OVP Evt\n");
        protect_flag &= ~ PD_PROTECT_VIN_OVP_Flag;
        u32PeDisableFlag |= PD_PROTECT_VIN_OVP_Flag;
#if (CONFIG_NPD48_VIN_OVP == 1)
        protect_hard_reset_procedure(port, 0);
#endif
        return;
    }
    if( (protect_flag & PD_PROTECT_VIN_UVP_Flag) != 0 )
    {
        //By Interupt
        DBG_PRINTF("UVP Evt\n");
        protect_flag &= ~ PD_PROTECT_VIN_UVP_Flag;
        u32PeDisableFlag |= PD_PROTECT_VIN_UVP_Flag;
#if (CONFIG_NPD48_VIN_OVP == 1)
        protect_hard_reset_procedure(port, 1);
#endif
        return;
    }
#if (CONFIG_NPD48_VBUS_OCP == 1)
    if(gi32AverageIfbCurr > gu16OcpThreshold)
    {
        //By Polling
        DBG_PRINTF("OCP hard reset\n");
        DBG_PRINTF("OCP Curr = %d\n", gi32AverageIfbCurr);
        DBG_PRINTF("OCP Thre = %d\n", gu16OcpThreshold);
        gi32AverageIfbCurr = 0;
        pd_dpm_request(port, DPM_REQUEST_HARD_RESET_SEND);
        return;
    }
#endif

    if( (protect_flag & PD_PROTECT_NTCP_Flag) != 0 )
    {
        //By Interupt
        DBG_PRINTF("NTCP Evt\n");
        protect_flag &= ~PD_PROTECT_NTCP_Flag;
        u32PeDisableFlag |= PD_PROTECT_NTCP_Flag;
#if (CONFIG_NPD48_NTCP == 1)
        protect_hard_reset_procedure(port, 3);
#endif
        return;
    }

    if( (protect_flag & PD_PROTECT_ITP_Flag) != 0 )
    {
        //By Interupt
        int vbus;
        DBG_PRINTF("ITP Evt\n");
        protect_flag &= ~ PD_PROTECT_ITP_Flag;
        u32PeDisableFlag |= PD_PROTECT_ITP_Flag;

        vbus = NPD48_ADC_GET_CONVERSION_DATA(3);
        pd_protect_temperature(port, 1, vbus);

#if (CONFIG_NPD48_ITP == 1)
        protect_hard_reset_procedure(port, 7);
#endif
        return;
    }

#if 1		//PS_READY callback will enable all interrupt except VCONN_OCP(not support) and OCP(by polling) */	

#if (CONFIG_NPD48_VIN_OVP == 1)
    if ( (u32PeDisableFlag & PD_PROTECT_VIN_OVP_Flag) && ((u32Protect_TS[0] + 10) > gu32TimeBase))
    {
        //VBUS OVP
        u32PeDisableFlag &= ~PD_PROTECT_VIN_OVP_Flag;
        pd_protection_enable_int(port, PD_PROTECT_VIN_OVP_Flag);
    }
    if ( (u32PeDisableFlag &  PD_PROTECT_VIN_UVP_Flag) && ((u32Protect_TS[1] + 10) > gu32TimeBase))
    {
        //VBUS UVP
        u32PeDisableFlag &= ~PD_PROTECT_VIN_UVP_Flag;
        pd_protection_enable_int(port, PD_PROTECT_VIN_UVP_Flag);
    }
#endif
#if (CONFIG_NPD48_NTCP == 1)
    if ( (u32PeDisableFlag & PD_PROTECT_NTCP_Flag) && ((u32Protect_TS[3] + 1000) > gu32TimeBase))
    {
        //NTCP
        DBG_PRINTF("Enable NTCP Int Again\n");
        u32PeDisableFlag &= ~PD_PROTECT_NTCP_Flag;
        pd_protection_enable_int(port, PD_PROTECT_NTCP_Flag);
    }
#endif
#if (CONFIG_NPD48_ITP == 1)
    if ( (u32PeDisableFlag & PD_PROTECT_ITP_Flag) && ((u32Protect_TS[7] + 1000) > gu32TimeBase))
    {
        //ITP
        u32PeDisableFlag &= ~PD_PROTECT_ITP_Flag;
        pd_protection_enable_int(port, PD_PROTECT_ITP_Flag);
    }
#endif
#endif
}



/** NAME          Target          ADC Channel      PEIE      CMP
  * VIN_OVP	      VIN             0                0          OVPCMPL & OVPCMPH
  * VIN_UVP       VIN             0                1          UVPCMPL & UVPCMPH
  * SYS_OCP       IS+/IS-         1                2          OCPCMPL & OCPCMPH
  * NTCP          NTC pin         2                3          NTCPCMPL & NTCPCMPH
  * CCOVP         CC pin          built-in         4          built-in 5.75V comparator
  * VCONN_OCP     VCONN           built-in         5          built-in 160mA comparator
  * COVP          VIN             6                6          COVPCMPL & COVPCMPH
  * ITP           INTP            3                7          ITPCMPL & ITPCMPH.
  */
CODE_LDROM void pd_protection_event_threshold(int port, int target , int cmplvl_mill_unit)
{
#if 1
    int cmpvalue;

    DBG_PRINTF("cmpvalue = %d\n", cmpvalue);
    DBG_PRINTF("target = %d\n", target);
    cmpvalue = cmplvl_mill_unit * 4096 / 2560;
    switch(target)
    {
    case 0://VIN OVP    PDO's voltage: Unit mV.
        cmpvalue = cmpvalue / 20;
        i2c_write16(port, NULL, NPD48_OVPCMPL,  cmpvalue);
        DBG_PRINTF("write OVP CMP\n");
        /* PE function and Interrupt will be control according to Power Sorce Ready or not */
        break;
    case 1://VIN UVP    PDO's voltage: Unit mV.
        cmpvalue = cmpvalue / 20;
        i2c_write16(port, NULL, NPD48_UVPCMPL,  cmpvalue);
        /* PE function and Interrupt will be control according to Power Sorce Ready or not */
        break;
    case 2:
#if 1
        /* Move to callback function by request PDO to decide OCP */
#else
        //SYS_OCP //ADC value to sample IS+/IS-.  mV/2560 = x/4096
        //cmpvalue = cmplvl_mill_unit*1.6L;   // 2.56/4096 //39384/2856/96
#if (CONFIG_NPD48_VBUS_OCP	== 1)
        i2c_update8(port, NULL, NPD48_PEIE, BIT2,  MASK_CLR);	// Interrupt Disable
        i2c_update8(port, NULL, NPD48_PECTL, BIT2, MASK_CLR);   // SYS_OCP function disable
        i2c_update8(port, NULL, NPD48_PEIF, BIT2,  MASK_SET);	// Clear Flag First
#endif
        cmpvalue = cmplvl_mill_unit * 4096 / 2560;         //Code=39368 RO-data=2856 RW-data=96 ZI-data=3936
        i2c_write16(port, NULL, NPD48_OCPCMPL,  cmpvalue);
#if (CONFIG_NPD48_VBUS_OCP	== 1)
        i2c_update8(port, NULL, NPD48_PEIE, BIT2,  MASK_SET);	// Interrupt Enable
        i2c_update8(port, NULL, NPD48_PECTL, BIT2, MASK_SET);   // SYS_OCP function enable
#endif
#endif
        break;
    case 3://
#if (CONFIG_NPD48_NTCP == 1)
        i2c_write16(port, NULL, NPD48_NTCPCMPL,  cmpvalue);
        i2c_update8(port, NULL, NPD48_PEIE, BIT3, MASK_SET);
        i2c_update8(port, NULL, NPD48_PECTL, BIT3, MASK_SET);
#endif
        break;
    case 4://CC OVP
#if (CONFIG_NPD48_CC_OVP == 1)
        /* CC OVP Comparator Built-In */
        i2c_update8(port, NULL, NPD48_PEIE, BIT4, MASK_SET);
        i2c_update8(port, NULL, NPD48_PECTL, BIT4, MASK_SET);// CC_OVP
#endif
        break;
    case 5://VCONN OCP
        /* VCONN OCP Built-In */
        i2c_update8(port, NULL, NPD48_PEIE, BIT5, MASK_SET);
        i2c_update8(port, NULL, NPD48_PECTL, BIT5, MASK_SET);// VCONN OCP
        break;
    case 6://CHIP_OVP
        cmpvalue = (cmpvalue*10)/266;
        i2c_write16(port, NULL, NPD48_COVPCMPL, cmpvalue);
        i2c_update8(port, NULL, NPD48_PEIE, BIT6, MASK_SET);
        i2c_update8(port, NULL, NPD48_PECTL, BIT6, MASK_SET);// CHIP_OVP
        break;

    case 7://ITP
#if (CONFIG_NPD48_ITP == 1)
    {
        int data;
        i2c_write16(port, NULL, NPD48_ITPCMPL, cmpvalue);
        i2c_read16(port, NULL, NPD48_ITPCMPL, &data);
        DBG_PRINTF("ITP Threshold = %d\n", data);
        i2c_update8(port, NULL, NPD48_PEIE, BIT7, MASK_SET);
        i2c_update8(port, NULL, NPD48_PECTL, BIT7, MASK_SET);// ITP
    }
#endif
    break;
    }
    DBG_PRINTF("done\n");
#endif
}

/**
	* Built-In ITP
	* 0 degree Celsius --> 708.28mV
	* Temperatur coefficient is 1.77mV/degree. TT is 1.84mV/degree
	* 100 degree Celsius, 708 - 1.77*100 = 531 mV
	**/
/**
	* NTCP on board WF104
	* 25 degree Celsius --> 100K Ohm, 20u*100K = 2000mV
	* 100 degree Celsius --> 5K Ohm,  20u*5K = 100mV
	* Temperatur coefficient is 1.9V/75 = 0.0253V/degree = 25.3mV/degree = 25300uV/degree
	**/
CODE_LDROM int pd_protect_temperature(int port, int type, int value)
{
    int uv;
    int degree;
    if(type == 0)
    {
#if (CONFIG_NPD48_NTCP == 1)			
        //NTCP
        uv = value*2560/4096*1000;
        //DBG_PRINTF("uv = %d\n", uv);
        if(uv<=2000000)
            degree = 25+(2000000-uv)/25300;
        else
            degree =  25 - (uv-2000000)/25300;
        DBG_PRINTF("NTC ADC = %d / %d C\n", value, degree);
#endif			
    }
    else if(type == 1)
    {
#if (CONFIG_NPD48_ITP == 1)			
        //ITP
        uv = value*2560/4096*1000;
        //DBG_PRINTF("uv = %d\n", uv);
        if(uv<708280)
            degree = (708280-uv)/1840;
        else
            degree =  0 - (uv-708280)/1840;
        DBG_PRINTF("ITP ADC = %d / %d C\n", value, degree);
#endif				
    }
    return degree;
}
