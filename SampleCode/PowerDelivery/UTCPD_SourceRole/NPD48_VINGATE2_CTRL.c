#include <stdio.h>
#include "NuMicro.h"
#include <stdbool.h>
#include "utcpdlib.h"
#include "usb_pd.h"


//#include "tcpci.h"
//#include "EMU.h"
/*
	KH depends on circuit:
	J11 SRC_Gate PIN 36
	J56 VIN_Gate PIN 37
*/
#define GATE_SRC       1   //J56. still fail for non-FIB chip.
#define GATE_VINGATE2  0   //JP11. OK for non-FIB chip. 
int32_t VinDacFormula(int port, uint16_t mv)
{
    uint16_t y0, y1;
    int32_t dy;
    int32_t dx;
    int32_t dac;

    UTCPD_GetTrimDACData(port, &y0, &y1);

    dy = (int32_t)y1 - (int32_t)y0;
    dx = (int32_t)mv - 5000;

    dac = (int32_t)y0 + (dy * dx) / 43000;

    return (int)dac;
}



#undef DBG_PRINTF
#define DBG_PRINTF(...) // printf
/*******************************************************************************
		M2L31:PHY       FPGA
		FRS_TX_CC1      ---> P12.12
		FRS_TX_CC2      ---> P12.14
		VCONN_DISCHARGE ---> P13.25
		SOURCE EN	      ---> P11.27 (PE11)
		GPIO1           ---> P11.29 (PE12)
		GPIO0           ---> P11.31 (PE13)
================================================================================
		NPD48:PHY        FPGA
		FRS_TX_CC1      ---> P12.12 (?)
		FRS_TX_CC2      ---> P12.14 (?)
 ** VCONN_DISCHARGE ---> P13.25 should be fixed in "LOW" (NPD48 NOT SUPPORT)
										 M471
		SOURCE EN	      ---> PE11
		GPIO1           ---> PE12
		GPIO0           ---> PE13
 *******************************************************************************/
/*******************************************************************************
 * Base on TC8260_TCPD_BOARD_V1                                                *
 * Design by MS50 THWang                                                       *
 *                                                                             *
 *	VBUS_Source_Level --> Specified the VBUS level                             *
 *******************************************************************************/
/*******************************************************************************
 * VBUS Enable Output Active High
 * Control signal: SOURCE_DC/DC_EN (PE11)
 * The signal should be replaced by VBSRCEN
 *******************************************************************************/
static void VBUS_Enable_Output(void)
{
    //Dummy

}
static void VBUS_Disable_Output(void)
{
    //Dummy

}

/*******************************************************************************
 * VBUS_SRC_EN: PA2 active high                                                *
 * VBUS can be measure on UTCPD board JP27.1                                   *
 *******************************************************************************/
void VBUS_CMD_Enable_Source_VBus(int port)
{
    uint8_t i;
    DBG_PRINTF("E SRC VBUS\n");
#if 1
    //i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGAEN_Msk, MASK_SET);
    i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_SET);
    UTCPD_IssueCmd(port, UTCPD_CMD_SRC_VBUS_DEFAULT);
//	//for(i=0; i< 3; i=i+1)
//	{
//		#if (GATE_SRC == 1)
//		i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGAEN_Msk, MASK_SET);
//		#endif
//        #if (GATE_VINGATE2 == 1)
//		i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGA2EN_Msk, MASK_SET);
//		#endif
//		i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_SET);
//
//	}
#else
    //for(i=0; i< 3; i=i+1)
    {
        i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_SET);
#if (GATE_SRC == 1)
        i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGAEN_Msk | NPD48_GDDBGCR_GDDBGEN_Msk, MASK_SET);
#endif
#if (GATE_VINGATE2 == 1)
        i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGA2EN_Msk, MASK_SET);
#endif
    }
#endif
}


void VBUS_CMD_Disable_Source_VBus(int port)
{
    uint8_t i;
    DBG_PRINTF("D SRC VBUS\n");

#if 1
    //i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGAEN_Msk, MASK_SET);
    UTCPD_IssueCmd(port, UTCPD_CMD_DISABLE_SRC_VBUS);
    i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_SET);
//	#if (GATE_SRC == 1)			//PIN36 J11
//	i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGAEN_Msk, MASK_CLR);//FIB always fixed in enable
//	i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_CLR);		//turn off by charge pump enable bit
//	#endif
//	#if (GATE_VINGATE2 == 1)	//PIN37 J56
//	i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGA2EN_Msk, MASK_CLR);
//    i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_SET);
//	#endif
#else
#if (GATE_SRC == 1)			//PIN36 J11
#if 0//FIB by src gate
    i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGAEN_Msk, MASK_CLR);//FIB always fixed in enable
    i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_SET);		//turn off by charge pump enable bit
#else//FIB by charge pump
    //i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGAEN_Msk | NPD48_GDDBGCR_GDDBGEN_Msk, MASK_SET);//FIB always fixed in enable
    i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_GDDBGEN_Msk, MASK_CLR);//FIB always fixed in enable
    i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_CLR);		//turn off by charge pump enable bit
#endif
#endif
#if (GATE_VINGATE2 == 1)	//PIN37 J56
    i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_GDDBGCR_SRCGA2EN_Msk, MASK_CLR);
    i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_SET);
#endif
#endif
}
/*******************************************************************************
 * VBUS_SINK_EN: PA3 active high                                               *
 * VBUS can be measure on JP27.2                                               *
 *******************************************************************************/
CODE_LDROM static void VBUS_CMD_Enable_Sink_VBus(int port)
{
    //TBD by cmd
    DBG_PRINTF("E SRC VBUS\n");
    UTCPD_SnkActivePolarity(port, UTCPD_PCACTL_SNKENLVL_HIGH);
    UTCPD_IssueCmd(port, UTCPD_CMD_SINK_VBUS);
}

CODE_LDROM static void VBUS_CMD_Disable_Sink_VBus(int port)
{
    //TBD by cmd.
    DBG_PRINTF("D SRC VBUS\n");
    UTCPD_SnkActivePolarity(port, UTCPD_PCACTL_SNKENLVL_HIGH);
    UTCPD_IssueCmd(port, UTCPD_CMD_DISABLE_SINK_VBUS);
}

void VBUS_Sink_Enable(int32_t port, bool bIsEnable)
{
    //TBD
    if(bIsEnable)
        VBUS_CMD_Enable_Sink_VBus(port);
    else
        VBUS_CMD_Disable_Sink_VBus(port);
}





static char i8RecLevel = 0xFF;
volatile uint32_t u32RecVolt = 0;
volatile uint32_t u32RecDac0;
char bIsChangePDO = 0;
char bIsVinDischarge = 0;
extern uint16_t u16Dac48VTrim;
extern uint16_t u16Dac5VTrim;

CODE_LDROM void setVinVoltage(int port, uint32_t vbus_volt)
{
    uint32_t dac;
#if 0
    if(vbus_volt == 48000)
        dac = u16Dac48VTrim; //0x2C8; //u16Dac48VTrim;
    else if(vbus_volt == 5000)
        dac = u16Dac5VTrim; //0x2C8; //u16Dac48VTrim;
    else
#endif
        dac = VinDacFormula(port, vbus_volt);
    //dac = dac - 15;	//enlarge about  200/4V = 50/V ==>  0.5V ~= 25
    //printf("DACV %d\n", dac);
    i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
}

void VBUS_Source_Level(int port, char i8Level)
{
    uint32_t dac;
    uint32_t new_volt;
    uint32_t pdopos;
    uint32_t curr, volt;
    uint32_t threshold;
    DBG_PRINTF("LVL = %d\n", i8Level);
    VBUS_Sink_Enable(port, 0);		/* Disable VBSNNKEN pin */

    if( i8RecLevel != i8Level)
    {
        //Disable Protection Event then Enable Protection Event
        bIsChangePDO = 1;
    }
    switch(i8Level)
    {
    case 0:
        new_volt= 5000;
        break;
    case 1:
        new_volt= 5000;
        break;
    case 2:
        new_volt= 9000;
        break;
    case 3:
        new_volt= 15000;
        break;
    case 4:
        new_volt= 20000;
        break;

    case 8:
        new_volt= 28000;
        break;
    case 9:
        new_volt= 36000;
        break;
#if (CONFIG_WELTREND_SOLUTION == 1)
    case 10:
        new_volt= 48000;
        break;
#endif
#if (CONFIG_OB_SOLUTION == 1)
    case 10://AVS
#endif
#if (CONFIG_WELTREND_SOLUTION == 1)
    case 11://AVS
#endif
        pd_get_adjoutput_voltage_current(port, &pdopos, &volt, &curr);
        DBG_PRINTF("Vot = %d, Curr = %d\n", volt, curr);
        new_volt= volt;
        break;
    }

    if( i8Level == 5)
    {//If PPS PDO
        threshold = 21000*105/100+250;
        DBG_PRINTF("Threshold H = %d\n", threshold);
        pd_protection_event_threshold(port, 0, threshold);

        threshold = 5000*95/100-750;
        DBG_PRINTF("Threshold L = %d\n", threshold);
        pd_protection_event_threshold(port, 1, threshold);
    }
#if (CONFIG_WELTREND_SOLUTION == 1)
    else if(i8Level == 11)
    {
        //AVS
        threshold = 48000*105/100+250;
        DBG_PRINTF("Threshold H = %d\n", threshold);
        pd_protection_event_threshold(port, 0, threshold);

        threshold = 15000*95/100-750;
        DBG_PRINTF("Threshold L = %d\n", threshold);
        pd_protection_event_threshold(port, 1, threshold);
    }
#endif
#if (CONFIG_OB_SOLUTION == 1)
    else if(i8Level == 10)
    {
        //AVS
        threshold = 36000*105/100+250;
        DBG_PRINTF("Threshold H = %d\n", threshold);
        pd_protection_event_threshold(port, 0, threshold);

        threshold = 15000*95/100-750;
        DBG_PRINTF("Threshold L = %d\n", threshold);
        pd_protection_event_threshold(port, 1, threshold);
    }
#endif
    else
    {
        //Fixed PDO
        threshold = new_volt*105/100+250;
        DBG_PRINTF(" H = %d\n", threshold);
        pd_protection_event_threshold(port, 0, threshold);
        threshold = new_volt*95/100-750;
        DBG_PRINTF(" L = %d\n", threshold);
        pd_protection_event_threshold(port, 1, threshold);
    }
    i2c_update8(port, NULL, NPD48_PEIF, 0xFF,  MASK_SET);


    setVinVoltage(port, new_volt);
		DBG_PRINTF("Set %d\n", new_volt);
    if(i8Level == 0)
        VBUS_CMD_Disable_Source_VBus(port);
		else
        VBUS_CMD_Enable_Source_VBus(port);
    u32RecVolt = new_volt;
    u32RecDac0 = dac;
}


void vbus_monitor(int port)
{

}

void VBUS_Source_Level_Item(int port)
{
    uint8_t ch;

    VBUS_Disable_Output();	/* Disable VBUS output default */

    while(1)
    {
        do
        {
            DBG_PRINTF("[0] Disable VBUS Output                            \n");
            DBG_PRINTF("[1] VBUS Output 5V                                 \n");
            DBG_PRINTF("[2] VBUS Output 9V                                 \n");
            DBG_PRINTF("[3] VBUS Output 20V                                \n");
            ch = getchar();

        }
        while( ((ch<'0') || (ch>'4')) && ((ch != 'q') && (ch != 'Q')));

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
        case 'Q':
        case 'q':
            return;
        }
    }
}





#if 0 //Emulation UTCPD Board 

/*******************************************************************************
 * TC8260_TCPD BOARD V1                                                        *
 * If set VBUS_SOURCE = 20V, Even disable VB_SRC_EN                            *
 * The VBUS_S+ Still up to 15.88V                                              *
 * It seems not protect the VBUS will be = 0, if SNK device plug out           *
 *                                                                             *
 * Use VBSRC_EN to replace SOURC_DC/DC_EN signal should be better              *
 *******************************************************************************/

void VBUS_SRC_Control(int port)
{
    char ch;
    int i;
    uint32_t dac, rdac;
    uint32_t u32Level = 0xFF;
    uint8_t u8Array[2];
    DBG_PRINTF("Set SRC VBUS Level 20V\n");
    VBUS_Source_Level(port, 1);

    while(1)
    {

        DBG_PRINTF("[0] Disable VBUS Output                \n");
        DBG_PRINTF("[1] 5V VBUS                            \n");
        DBG_PRINTF("[2] 9V VBUS                            \n");
        DBG_PRINTF("[3] 12V VBUS                           \n");
        DBG_PRINTF("[4] 15V VBUS                           \n");
        DBG_PRINTF("[5] 20V VBUS                           \n");
        DBG_PRINTF("[6] 28V VBUS                           \n");
        i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
        DBG_PRINTF("NPD48_DAC0 Reg = 0x%x\n", rdac);
        //cpu_dump(0xF0, 0x100);
        ch = getchar();
        DBG_PRINTF("%c\n", ch);
        if(u32Level == 6)
        {
            /* Drop to 21V first if current great than 21V*/
            dac = VinDacFormula(port, 21000);
            DBG_PRINTF("20V DAC = %d\n", dac);
            i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
            i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
            if(dac!=rdac)
                DBG_PRINTF("I2C Write DAC Err\n");
            delay(500); /* for Safe */
        }
        switch(ch)
        {
        case '0':
            /* To turn off VBUS Gatem, the VIN should keep 5V output */
            dac = VinDacFormula(port, 5000);
            DBG_PRINTF("5V DAC = %d\n", dac);
            //			for(i= 0; i<3; i=i+1)
            {
                i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
                i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
                if(dac!=rdac)
                    DBG_PRINTF("I2C Write DAC Err\n");
            }
            DBG_PRINTF("dac read back = %d\n", rdac);
            VBUS_CMD_Disable_Source_VBus(port);
            u32Level = 0;
            break;
        case '1':
            dac = VinDacFormula(port, 5500);
            DBG_PRINTF("5V DAC = %d\n", dac);
            //            for(i= 0; i<3; i=i+1)
            {
                i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
                i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
                if(dac!=rdac)
                    DBG_PRINTF("I2C Write DAC Err\n");
            }
            delay(1000);
            DBG_PRINTF("dac read back = %d\n", rdac);
            VBUS_CMD_Enable_Source_VBus(port);
            u32Level = 1;
            break;
        case '2':
            dac = VinDacFormula(port, 9000);
            DBG_PRINTF("9V DAC = %d\n", dac);
            //		for(i= 0; i<3; i=i+1)
            {
                i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
                i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
                if(dac!=rdac)
                    DBG_PRINTF("I2C Write DAC Err\n");
            }
            DBG_PRINTF("dac read back = %d\n", rdac);
            VBUS_CMD_Enable_Source_VBus(port);
            u32Level = 2;
            break;
        case '3':
            dac = VinDacFormula(port, 12000);
            DBG_PRINTF("12V DAC = %d\n", dac);
//                    for(i= 0; i<3; i=i+1)
            {
                i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
                i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
                if(dac!=rdac)
                    DBG_PRINTF("I2C Write DAC Err\n");
            }
            DBG_PRINTF("dac read back = %d\n", rdac);
            VBUS_CMD_Enable_Source_VBus(port);
            u32Level = 3;
            break;
        case '4':
            dac = VinDacFormula(port, 15000);
            DBG_PRINTF("15V DAC = %d\n", dac);
//                    for(i= 0; i<3; i=i+1)
            {
                i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
                i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
                if(dac!=rdac)
                    DBG_PRINTF("I2C Write DAC Err\n");
            }
            DBG_PRINTF("dac read back = %d\n", rdac);
            VBUS_CMD_Enable_Source_VBus(port);
            u32Level = 4;
            break;
        case '5':
            dac = VinDacFormula(port, 20000);
            DBG_PRINTF("20V DAC = %d\n", dac);
//                    for(i= 0; i<3; i=i+1)
            {
                i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
                i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
                if(dac!=rdac)
                    DBG_PRINTF("I2C Write DAC Err\n");
            }
            DBG_PRINTF("dac read back = %d\n", rdac);
            VBUS_CMD_Enable_Source_VBus(port);
            u32Level = 5;
            break;
        case '6':
            dac = VinDacFormula(port, 28000);
            DBG_PRINTF("20V DAC = %d\n", dac);
//                    for(i= 0; i<3; i=i+1)
            {
                i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
                i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
                if(dac!=rdac)
                    DBG_PRINTF("I2C Write DAC Err\n");
            }
            DBG_PRINTF("dac read back = %d\n", rdac);
            VBUS_CMD_Enable_Source_VBus(port);
            u32Level = 6;
            break;
        case 'Q':
        case 'q':
            return;
        }
//			cpu_dump(0, 0x100);
//			cpu_dump(0x100, 0x130);
    }
}


void VBUS_ProgramPPS(int port)
{
    char ch;
    int i, Vin;
    uint32_t dac, rdac;
    uint32_t u32Level = 0xFF;
    uint8_t u8Array[2];
    DBG_PRINTF("Set SRC VBUS Level 20V\n");
    VBUS_Source_Level(port, 1);

    dac = 2939;
    i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
    i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
    if(dac!=rdac)
        DBG_PRINTF("I2C Write DAC Err\n");
    delay(20);

    while(1)
    {
        for(Vin = 3300; Vin< 5000; Vin = Vin+20)
        {
            dac = VinDacFormula(port, Vin);
            i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
            i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
            if(dac!=rdac)
                DBG_PRINTF("I2C Write DAC Err\n");
            delay(20);
        }

        for(Vin = 3800; Vin> 3300; Vin = Vin-20)
        {
            dac = VinDacFormula(port, Vin);
            i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
            i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
            if(dac!=rdac)
                DBG_PRINTF("I2C Write DAC Err\n");
            delay(20);
        }
    }
}

void VBUS_ProgramAVS(int port)
{
    char ch;
    int i, Vin;
    uint32_t dac, rdac;
    uint32_t u32Level = 0xFF;
    uint8_t u8Array[2];
    DBG_PRINTF("Set SRC VBUS Level 20V\n");
    VBUS_Source_Level(port, 1);
    while(1)
    {
        for(Vin = 15000; Vin< 48000; Vin = Vin+20)
        {
            dac = VinDacFormula(port, Vin);
            i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
            i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
            if(dac!=rdac)
                DBG_PRINTF("I2C Write DAC Err\n");
            delay(20);
        }

        for(Vin = 48000; Vin> 15000; Vin = Vin-20)
        {
            dac = VinDacFormula(port, Vin);
            i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
            i2c_read16(port, NULL, NPD48_DAC0DBL, &rdac);
            if(dac!=rdac)
                DBG_PRINTF("I2C Write DAC Err\n");
            delay(20);
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
        while( ((ch<'0') || (ch>'1')) && ((ch != 'q') && (ch != 'Q')));

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

/* Enable/Disable Discharge Signal for UTCPD0_DISCHG for VCONN */
void  vconn_disable_discharge(void)
{
#if 0
    /* Disable VCONN discharge */
    outp32(TCPD0_BASE+TCPD_VCDGCTL, inp32(TCPD0_BASE+TCPD_VCDGCTL) & ~VCDGEN);
#else
    DBG_PRINTF("NPD48 didn't support VCONN dischg\n");
#endif
}
void  vconn_enable_discharge(void)
{
#if 0
    /* Enable VCONN discharge */
    outp32(TCPD0_BASE+TCPD_VCDGCTL, inp32(TCPD0_BASE+TCPD_VCDGCTL) | VCDGEN);
#else
    DBG_PRINTF("NPD48 didn't support VCONN dischg\n");
#endif
}
void  vconn_discharge_polarity_active_low(void)
{
#if 0
    /* VCONN discharge active low */
    outp32(TCPD0_BASE+TCPC_REG_PINPL, inp32(TCPD0_BASE+TCPC_REG_PINPL) & ~TCPC_REG_PINPL_VCDCHG);
#else
    DBG_PRINTF("NPD48 didn't support VCONN dischg\n");
#endif
}
void  vconn_discharge_polarity_active_high(void)
{
#if 0
    /* VCONN discharge active low */
    outp32(TCPD0_BASE+TCPC_REG_PINPL, inp32(TCPD0_BASE+TCPC_REG_PINPL) | TCPC_REG_PINPL_VCDCHG);
#else
    DBG_PRINTF("NPD48 didn't support VCONN dischg\n");
#endif
}

volatile uint32_t gu32VBusDeUndershotTime;
volatile uint32_t gu32VBusDeUndershotTimeDly;
volatile uint32_t gu32VBusDeUndershotVolt[1];

