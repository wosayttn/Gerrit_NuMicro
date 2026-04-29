/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/18 3:19p $
 * @brief    Show UTCPD Protocol Stack
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "utcpdlib.h"
#include "usb_pd.h"


#define DBG_PRINTF(...)

uint32_t u32Alert;

void frs_disable_rx(int port)
{
    /* Disable FRS RX */
    i2c_update8(port, NULL, NPD48_FSTXCTL, (BIT3), MASK_CLR);
}

void Alert_VenderDef(int port)
{
    int32_t rv;
    int32_t i32Val;
    uint32_t u32ClrMask = 0;
    rv = tcpc_addr_read(0, NULL, NPD48_FSASTS, &i32Val);
    if(rv != 0)
    {
        DBG_PRINTF("Error\n");
        while(1);
    }

    if(i32Val & NPD48_FSASTS_FSDISRXSTS_Msk)
    {
        u32ClrMask |= NPD48_FSASTS_FSDISRXSTS_Msk;
        DBG_PRINTF("R_FRS\n");
        pd_got_frs_signal(port);
        /* Disable FRS RX detection to avoid the second time detection if low up to 60us */
        frs_disable_rx(port);
    }
    if(i32Val & NPD48_FSASTS_FSDISTXSTS_Msk)
    {
        i2c_update8(0, NULL, 0xA0, (BIT1), MASK_CLR);	/* Clear FRS enable bit first */
        u32ClrMask |= NPD48_FSASTS_FSDISTXSTS_Msk;
        DBG_PRINTF("FRS_TX STS\n");
    }

    if(i32Val & NPD48_FSASTS_CRCESTS_Msk)
    {
        u32ClrMask |=  NPD48_FSASTS_CRCESTS_Msk;
        DBG_PRINTF("CRC_ERR\n");
    }
    if(i32Val & VCDCHGIS)
    {
        u32ClrMask |= VCDCHGIS;
        DBG_PRINTF("VC_DCHG\n");
    }
    tcpc_addr_write(0, NULL, NPD48_FSASTS, u32ClrMask);
}

volatile uint32_t gu32TimeBase = 0;

volatile uint8_t gu8FineTuneVinTime;
volatile uint8_t gu8FineTuneVinFlag;
void TCPCi_Handler(void);

void TMR0_IRQHandler(void)
{
    uint32_t port = 0;

    gu32TimeBase = gu32TimeBase + 1;

    /* Notify portx to increase the time stamp */
    UTCPD_TimerBaseInc(port);

    /* clear timer interrupt flag */
    TIMER_ClearIntFlag(TIMER0);
}

volatile bool bIsTCPCInterrupt = 0;
void GPB_IRQHandler(void)
{
#if (OPT_M251 == 1)
#if (OPT_FPGA == 1)
    /* To check if PB.1 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT5))
#else
    /* To check if PB.1 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT1))
#endif
    {
        __set_PRIMASK(1);
        bIsTCPCInterrupt = 1;
#if (OPT_FPGA == 1)
        GPIO_CLR_INT_FLAG(PB, BIT5);
#else
        GPIO_CLR_INT_FLAG(PB, BIT1);
#endif
    }
#endif
#if (OPT_M2003 == 1)
    /* To check if PB.11 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT11))
    {
        __set_PRIMASK(1);		/* Mask all of interrupts */
        GPIO_CLR_INT_FLAG(PB, BIT11);
        bIsTCPCInterrupt = 1;
    }
#endif
}
void TCPCi_Handler(void)
{
    uint32_t port = 0;
    int32_t rv;
    uint32_t u32AlertMsk = 0;
    uint32_t u32AlertEnable, u32Ie;

    rv = tcpc_addr_read16(0, NULL, TCPC_REG_ALERT_MASK, &u32AlertEnable);
    if(rv != 0)
    {
        DBG_PRINTF("Error\n");
        while(1);
    }
    rv = tcpc_addr_read16(0, NULL, TCPC_REG_ALERT, &u32Alert);
    if(rv != 0)
    {
        DBG_PRINTF("Error\n");
        while(1);
    }

    u32Alert = u32Alert & u32AlertEnable;
    if(u32Alert & 0x200)
    {
        int32_t i32faultctl;
        tcpc_addr_read(0, NULL, NPD48_FAULTCTL, &i32faultctl);
        DBG_PRINTF("INT FAUCTL = 0x%x\n", i32faultctl);
        tcpc_update8(0, NPD48_FAULTCTL, 0xFF, MASK_SET);	/* 1: Disable FAULT Function */
    }

    if(u32Alert == 1)
    {
        /* Disable CC STS change Interrupt */
        tcpc_update16(0, NPD48_ALERTL, NPD48_ALERTL_CCSCHIS_Msk, MASK_CLR);
        tcpc_update16(0, NPD48_ALERTML, NPD48_ALERTML_CCSCHIE_Msk, MASK_CLR);
    }

    /* Driving TCPC Interrupt */
    tcpci_tcpc_alert(port);

    /* Handle NPD48 Vendor Defined Interrupts */
    if(u32Alert & TCPC_REG_ALERT_VENDOR_DEF)
    {
        Alert_VenderDef(port);
    }

    /* Handle NPD48 Protection Events */
    rv = i2c_read8(0, NULL, NPD48_PEIE, &u32Ie);
    if(u32Ie != 0)
    {
        rv = i2c_read8(0, NULL, NPD48_PEIF, &u32Alert);
        u32Alert = u32Ie & u32Alert;
        if( (u32Alert) != 0)
        {
            DBG_PRINTF("PEIF = 0x%x\n", u32Alert);
        }
#if (CONFIG_NPD48_VIN_OVP == 1)	|| (CONFIG_NPD48_VBUS_OCP == 1) || (CONFIG_NPD48_NTCP == 1) ||\
			(CONFIG_NPD48_CHIP_OVP ==1) || (CONFIG_NPD48_ITP == 1)
        pd_protection_handler(port, u32Alert);
#endif
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void BOD_IRQHandler(void)
{
    /* Clear BOD Interrupt Flag */
    SYS_CLEAR_BOD_INT_FLAG();

    DBG_PRINTF("Brown Out is Detected\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    //CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV4);
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Enable Internal RC 32KHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Enable External HXT clock */
    //CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

#if (OPT_M251 == 1)
    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
#endif
#if (OPT_M2003 == 1)
    /* Enable GPIO clock */
//    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
//    CLK_EnableModuleClock(GPD_MODULE);
#endif


    /* === Enable IP clock === */
#if (OPT_M251 == 1)
#if (OPT_FPGA == 1)
    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
#else
    /* Enable UART1 clock */
    CLK_EnableModuleClock(UART1_MODULE);
    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
#endif
#endif
#if (OPT_M2003 == 1)
    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
#endif
    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable TIMER 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable TIMER 1 module clock */
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

#if (OPT_M251 == 1)
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
#endif
#if (OPT_M2003 == 1)
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
#endif

    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
#if (OPT_M251 == 1)
#if (OPT_FPGA == 1)
    /* Set PA multi-function pins for UART0 RXD=PA.2 and TXD=PA.3 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_UART0_RXD;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB13MFP_Msk) | SYS_GPB_MFPH_PB13MFP_UART0_TXD;
#else
    /* Set PA multi-function pins for UART0 RXD=PA.2 and TXD=PA.3 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA2MFP_Msk) | SYS_GPA_MFPL_PA2MFP_UART1_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA3MFP_Msk) | SYS_GPA_MFPL_PA3MFP_UART1_TXD;
#endif
#endif
#if (OPT_M2003 == 1)
    /* Set PB multi-function pins for UART0 RXD=PB.3 and TXD=PB.2 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB2MFP_Msk) | SYS_GPB_MFPL_PB2MFP_UART0_TXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB3MFP_Msk) | SYS_GPB_MFPL_PB3MFP_UART0_RXD;
#endif

#if (OPT_M251 == 1)
#if (OPT_FPGA == 1)
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk)) |  // MB20 I2C Pin
                    (SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL);
    /* I2C pin enable schmitt trigger */
    PC->SMTEN |= (GPIO_SMTEN_SMTEN0_Msk | GPIO_SMTEN_SMTEN1_Msk);
#else
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk)) |  // MB20 I2C Pin
                    (SYS_GPA_MFPL_PA4MFP_I2C0_SDA | SYS_GPA_MFPL_PA5MFP_I2C0_SCL);
    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk);
#endif
#endif
#if (OPT_M2003 == 1)
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB8MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk)) |  // MB20 I2C Pin
                    (SYS_GPB_MFPH_PB8MFP_I2C0_SDA | SYS_GPB_MFPH_PB9MFP_I2C0_SCL);
    /* I2C pin enable schmitt trigger */
    PB->SMTEN |= (GPIO_SMTEN_SMTEN8_Msk | GPIO_SMTEN_SMTEN9_Msk);
#endif
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    DBG_PRINTF("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    I2C_SetBusClockFreq(I2C0, 800000); 	//800KHz
    DBG_PRINTF("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);
}

void TIMER0_Init(void)
{
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    /* Enable timer interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    /* Start Timer 0 */
    TIMER_Start(TIMER0);
}

void TIMER1_Init(void)
{
    /* Reset TMR1 */
    SYS_ResetModule(TMR1_RST);

    /* Set timer frequency to 100HZ */
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 100);

    /* Enable timer interrupt */
//    TIMER_EnableInt(TIMER0);
//    NVIC_EnableIRQ(TMR0_IRQn);
#if (OPT_M251 == 1)
    /* Timer1 trigger target is EADC */
    TIMER1->TRGCTL |= TIMER_TRGCTL_TRGEADC_Msk;
#endif
#if (OPT_M2003 == 1)
    /* Timer1 trigger target is EADC */
    TIMER1->TRGCTL |= TIMER_TRGCTL_TRGADC_Msk;
#endif
    /* Start Timer 1 */
    TIMER_Start(TIMER1);
}



/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_M2351.s.
 */
void TMR1_IRQHandler(void)
{
    static uint32_t u32Sec = 1;

    DBG_PRINTF("%d sec\n", u32Sec++);

    /* clear timer interrupt flag */
    TIMER_ClearIntFlag(TIMER1);
}

/**
 * @brief       UUTCPD Callback Function
 *
 * @param       event: UTCPD_PD_ATTACHED = 0,            : Port partner attached (op=1) or disattached (op=0)
 *                     UTCPD_PD_CONTRACT = 1,            : PD contract established
 *                     UTCPD_PD_SNK_VOLTAGE = 2,         : Contract voltage
 *	                   UTCPD_PD_CABLE_MAX_POWER = 3,     : Cable Max Voltage and Max Current : ((max_vol<<16) | max_curr)
 *                     UTCPD_PD_VCONN_DISCHARGE = 4,     : To do VCONN Discharge
 *                     UTCPD_PD_PS_TRANSITION = 5,       : To Disable VIN OVP/UVP, RDO IDX
 *	                   UTCPD_PD_PS_READY = 6,            : To Enable VIN OVP/UVP, 0
 *	                   UTCPD_PD_VIN_DISCHARGE_DONE = 7,  : VIN Discharge Done, 0
 *              op: 0 = event flag set
 *                  1 = event flag clear
 * @return      None
 *
 * @details     None
 *
 */
extern char bIsChangePDO;
extern char bIsVinDischarge;
extern volatile uint8_t protect_flag;
volatile uint8_t gu8Rdo = 1;
extern volatile uint32_t u32RecVolt;
volatile uint8_t bIsPowerReady = 0;

void pd_get_request_pdo_info(int port, uint32_t pdo_idx, uint32_t* u32volt, uint32_t* u32curr)
{
    uint32_t pdopos; 	/* It will be equal to pdo_idx */
    if(pdo_idx <= pd_src_pdo_cnt)
    {
        //SPR
        pdo_idx = pdo_idx - 1;
        if( (pd_src_pdo[pdo_idx] & PDO_TYPE_MASK) == PDO_TYPE_FIXED)
        {
            //FIXED
            *u32volt = PDO_FIXED_GET_VOLT(pd_src_pdo[pdo_idx]);
            *u32curr = PDO_FIXED_GET_CURR(pd_src_pdo[pdo_idx]);
        }
        else if( (pd_src_pdo[pdo_idx] & PDO_TYPE_MASK) == PDO_TYPE_AUGMENTED)
        {
            //PPS
            pd_get_adjoutput_voltage_current(port, &pdopos, u32volt, u32curr);

        }
    }
    else if(pdo_idx >= 8 )
    {
        //EPR
        pdo_idx = pdo_idx - 8;
        if( (pd_src_epr_pdo[pdo_idx] & PDO_TYPE_MASK) == PDO_TYPE_FIXED)
        {
            //Fixed
            *u32volt = PDO_FIXED_GET_VOLT(pd_src_epr_pdo[pdo_idx]);
            *u32curr = PDO_FIXED_GET_CURR(pd_src_epr_pdo[pdo_idx]);
        }
        else if( (pd_src_epr_pdo[pdo_idx] & PDO_TYPE_MASK) == PDO_TYPE_AUGMENTED)
        {
            //AVS
            pd_get_adjoutput_voltage_current(port, &pdopos, u32volt, u32curr);

        }
    }
}
extern uint32_t gu32DischargeTime;
extern volatile uint32_t u32RecVolt;

void UTCPD_Callback(int port, E_UTCPD_PD_EVENT event, uint32_t op)
{
    DBG_PRINTF("evt = %d\n", event);
    DBG_PRINTF("op = %x\n", op);
    if(event == UTCPD_PD_CABLE_MAX_POWER)
    {
        /* Modify SRC PDO According to Cable Capabilities */
#if (OPT_CAPTIVE_CABLE == 0 )
        DBG_PRINTF("Cable Max VBUS = %d\n", op >> 16);
        DBG_PRINTF("Cable Max Curr = %d\n", op & 0xFFFF);
        pd_backup_src_pdo(port);
        pd_modify_src_pdo(port, op >> 16, op & 0xFFFF);
#endif
    }
    else if( (event == UTCPD_PD_ATTACHED) && (op == 0))
    {
        /* Recovery SRC PDO as Deattached */
        /* Disabke VIN OVP and UVP */
        int32_t i32data;

        bIsPowerReady = 0;
        protect_flag = 0;
        i2c_read8(port, NULL, NPD48_PEIF, &i32data);
        DBG_PRINTF("NPD48_PEIF = 0x%x\n", i32data);
#if (OPT_CAPTIVE_CABLE == 0 )
        pd_recovery_src_pdo(port);
#endif
    }
    else if(event == UTCPD_PD_VCONN_DISCHARGE)
    {
        /* Inform VCONN Discharge */
#if (OPT_M251 == 1)
        PA1 = 1; 	/* Start VCONN Discharge */
        delay(120);
        PA1 = 0;    /* Stop VCONN Discharge */
#endif
#if (OPT_M2003 == 1)

#endif
    }
    else if(event == UTCPD_PD_PS_TRANSITION)
    {
        /**
          * purpose: To Disable VIN OVP/UVP function
          *	event = UTCPD_PD_PS_TRANSITION
        	* op = request pdo position.
          **/
        uint32_t new_volt;
        uint32_t curr, volt;
        uint32_t threshold;
        bIsPowerReady = 0;

        /* clean temperatory drawn current */
        vbus_current_clear(port);

        //DBG_PRINTF("PS TRANSITION op = %d\n", op);
        if( bIsChangePDO == 1)
        {
            DBG_PRINTF("Disable OVP/UVP/OCP\n");
#if (CONFIG_NPD48_VIN_OVP == 1)
            /* VIN OVP and UVP */
            pd_protection_disable_int(port, BIT7 | BIT0 | BIT1);
#else
            /* VBUS OVP */
            i2c_update8(port, NULL, NPD48_PEIE, BIT7 | BIT1,  MASK_CLR);
#endif
        }

        {
            //Wrte a function according pdo get Volt and current
            uint32_t u32volt, u32curr;
            //DBG_PRINTF("OV = %d\n", gu32ReqVolt);
            pd_get_request_pdo_info(port, op, &u32volt, &u32curr);
            DBG_PRINTF("OV = %d\n", u32RecVolt);
            DBG_PRINTF("NV = %d\n", u32volt);
        }
    }
    else if(event == UTCPD_PD_PS_READY)
    {
        /* To Enable VIN OVP/UVP */
        bIsPowerReady = 1;
        if( bIsChangePDO == 1)
        {
            int data = 0;
            DBG_PRINTF("Enable OVP/UVP/OCP\n");
            bIsChangePDO = 0;
#if (CONFIG_NPD48_VIN_OVP == 1) || (CONFIG_NPD48_VIN_UVP == 1)
            data |= 0x3;
#endif
#if (CONFIG_NPD48_VBUS_OCP == 1)
            //data |= 0x04;  //OCP by polling mode
#endif
#if (CONFIG_NPD48_NTCP == 1)
            data |= 0x08;
#endif
#if (CONFIG_NPD48_CC_OVP == 1)
            data |= 0x10;
#endif
#if (CONFIG_NPD48_VCONN_OCP == 1)
            data |= 0x20;
#endif
#if (CONFIG_NPD48_CHIP_OVP == 1)
            data |= 0x40;
#endif
#if (CONFIG_NPD48_VIN_OVP == 1)
            data |= 0x80;
#endif
            pd_protection_enable_int(port, data); //Except VCONN_OCP and VBUS OCP
        }
    }
    else if(event == UTCPD_PD_VIN_DISCHARGE_DONE)
    {
        /* Inform VIN Dicharge done */
        gu8FineTuneVinFlag = 1; 	/* Start fine tune VBUS to avoid the dampling phenomenon */
        gu8FineTuneVinTime = 0;
    }
    else if(event == UTCPD_PD_ACCEPT_REQUEST_PDO)
    {
        /* Inform Upper layer to Provide the Power of Requested PDO */
        int32_t i32volt, i32curr;
        uint32_t pdo_idx = op;
        pd_get_request_pdo_info(port, op, &i32volt, &i32curr);
				DBG_PRINTF("Req V = %d\n", i32volt);
        /* To fix VBUS generate under shoot issue as VBUS High Voltage to Low Voltage */
        if( ( (int32_t)u32RecVolt - i32volt) > 3000 )
        {
            /* Prepare to ramp down VBUS */
            void pdo_transition_set_target_level_and_start_discharge(int port, int i16targetvolt, int i16currvolt, int8_t pdo_idx);
            pdo_transition_set_target_level_and_start_discharge(port, (int)u32RecVolt, i32volt, (int8_t)pdo_idx);
        }
        else
            VBUS_Source_Level(port, pdo_idx);
        vbus_set_overcurrent_threshold(port, pdo_idx, i32curr);
    }
    else if(event == UTCPD_PD_VBUS_DISCHARGE_START)
    {
        void pd_vbus_discharge_start(int port);
        pd_vbus_discharge_start(port);

    }
    else if(event == UTCPD_PD_VBUS_DISCHARGE_STOP)
    {
        void pd_vbus_discharge_stop(int port);
        pd_vbus_discharge_stop(port);
    }
    DBG_PRINTF("CB done\n");
}

#if (CONFIG_TEST_PDO28V_OVP ==1) || (CONFIG_TEST_PDO28V_UVP ==1) || (CONFIG_TEST_PDO36V_COVP ==1)
extern volatile uint32_t u32RecVolt;
volatile uint32_t btime = 0, etime = 0;
volatile uint32_t dac;
#endif

extern bool bisTestFlag;

void pd_task(void)
{
    int port = TASK_ID_TO_PD_PORT(task_get_current());

    /*
     * If port does not exist, return
     */
    if (port >= board_get_usb_pd_port_count())
        return;

    /* Install UTCPD Callback Function */
    UTCPD_InstallCallback(port, (utcpd_pvFunPtr*)UTCPD_Callback);

    while (1)
    {
//      pd_timer_init(port);
//      pd_task_init(port);
        pd_task_reinit(port);
        /* As long as pd_task_loop returns true, keep running the loop.
         * pd_task_loop returns false when the code needs to re-init
         * the task, so once the code breaks out of the inner while
         * loop, the re-init code at the top of the outer while loop
         * will run.
         */

        while (pd_task_loop(port))
        {
            if (bIsTCPCInterrupt == 1)
            {
                TCPCi_Handler();
                bIsTCPCInterrupt = 0;
                __set_PRIMASK(0); 	/* Unmask all of interrupts */
            }

            void pdo_transition_falling_periodically(int port, uint32_t ticks);
            bool pdo_chk_flags(int port, uint32_t mask);
#define PDO_TRANSITION_FALLING BIT0
            {
                uint32_t ticks = pd_get_tick();
                if( pdo_chk_flags(port, PDO_TRANSITION_FALLING) && ((ticks % 10) == 0) )
                    pdo_transition_falling_periodically(port, ticks);
            }


            {
                /** User Tasks:
                  * Please separate User Tasks code piece by piece.
                  * Suggestion not to over 100us for every piece code.
                  * pd_task_loop() needs to be called every 1ms
                  **/

                /* Check Protection event every 5 sec*/
                if( (pd_get_tick() % 5) == 0)
                    pd_protection_run(port);

#if 1
                /**	Read VBUS Current to compensate VBUS
                  *
                  *
                  **/
                if (bIsPowerReady == 1)
                {
#if (CONFIG_NPD48_VBUS_OCP == 1)
                    uint32_t u32drawncurr;
                    if( (pd_get_tick() % 5) == 0)
                        u32drawncurr = vbus_current_read(port);

                    /* Print drawn current every 5 sec */
                    if( (pd_get_tick() % 5000) == 0)
                    {
                        DBG_PRINTF("Curr = %d\n", u32drawncurr);
                    }
#endif
                    /* Check Protect event every 20ms */
                    if( (pd_get_tick() % 20) == 0)
                    {
                        uint32_t vbus;
                        int data;

#if (CONFIG_TEST_NTCP == 1)
                        /**
                        	* NTCP on board WF104
                        	* 25 degree Celsius --> 100K, 20u*100K = 2000mV
                        	* 100 degree Celsius --> 5K,  20u*5K = 100mV
                        	* Temperatur coefficient is 1.9V/75 = 0.0253/degree
                        	**/
                        vbus = NPD48_ADC_GET_CONVERSION_DATA(2);
                        pd_protect_temperature(port, 0, vbus);	/* 0 means NTCP */
#endif
#if (CONFIG_NPD48_ITP == 1)
                        /**
                          * Built-In ITP
                        	* 0 degree Celsius --> 708.28mV
                        	* Temperatur coefficient is 1.84mV/degree
                        	* 100 degree Celsius, 708 - 1.84*100 ~= 524 mV
                        	**/
                        vbus = NPD48_ADC_GET_CONVERSION_DATA(3);
                        pd_protect_temperature(port, 1, vbus);	/* 1 means ITP */
#endif
#if (CONFIG_NPD48_CHIP_OVP == 1)
                        {
                            int cmpvalue;
                            vbus = NPD48_ADC_GET_CONVERSION_DATA(6);
                            i2c_read16(port, NULL, NPD48_COVPCMPL,  &cmpvalue);
                            DBG_PRINTF("COVP = %d, %d\n", vbus, cmpvalue);

                            i2c_read8(port, NULL, NPD48_PECTL,  &cmpvalue);
                            DBG_PRINTF("PECTL = 0x%x\n", cmpvalue);

                            i2c_read8(port, NULL, NPD48_PEIE,  &cmpvalue);
                            DBG_PRINTF("PEIE = 0x%x\n", cmpvalue);
                            if(cmpvalue == 0)
                                i2c_update8(port, NULL, NPD48_PEIE,  BIT6, MASK_SET);

                        }
#endif
                    }
                }
#endif


#if (CONFIG_TEST_PDO28V_OVP == 1)
                if(u32RecVolt == 28000)
                {
                    if(btime == 0)
                    {
                        btime = gu32TimeBase;
                        etime = gu32TimeBase;
                    }
                    etime = gu32TimeBase;
                    if( ( etime - btime ) > 10000)
                    {
                        dac = VinDacFormula(0, 31000);
                        i2c_write16(0, NULL, NPD48_DAC0DBL, dac);
                        //i2c_read16(0, NULL, NPD48_DAC0DBL, &dac);
                        btime = 0;
                        etime = 0;
                    }
                }
                else
                {
                    btime = 0;
                    etime = 0;
                }
#endif
#if (CONFIG_TEST_PDO28V_UVP == 1)
                if(u32RecVolt == 28000)
                {
                    if(btime == 0)
                    {
                        btime = gu32TimeBase;
                        etime = gu32TimeBase;
                    }
                    etime = gu32TimeBase;
                    if( ( etime - btime ) > 10000)
                    {
                        dac = VinDacFormula(port, 25000);
                        i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
                        i2c_read16(port, NULL, NPD48_DAC0DBL, &dac);
                        btime = 0;
                        etime = 0;
                    }
                }
                else
                {
                    btime = 0;
                    etime = 0;
                }
#endif
#if (CONFIG_TEST_PDO36V_COVP == 1)
                if(u32RecVolt == 36000)
                {
                    if(btime == 0)
                    {
                        btime = gu32TimeBase;
                        etime = gu32TimeBase;
                    }
                    etime = gu32TimeBase;
                    if( ( etime - btime ) > 10000)
                    {
                        dac = VinDacFormula(port, 44000);
                        i2c_write16(port, NULL, NPD48_DAC0DBL, dac);
                        i2c_read16(port, NULL, NPD48_DAC0DBL, &dac);
                        btime = 0;
                        etime = 0;
                    }
                }
                else
                {
                    btime = 0;
                    etime = 0;
                }
#endif

#if (CONFIG_COMMAND_SHELL == 1)
                UART_Commandshell(port);
#endif
                continue;
            }//if (bIsTCPCInterrupt == 0)
        }//while (pd_task_loop(port))
    }//while(1);
}

extern struct usb_state pe_states[];



CODE_LDROM int main()
{
    int data = 0;
    uint32_t    i, u32Data, count = 0;
    int port = 0, i32Val;
    uint32_t 		role;
    char ch;
    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if (OPT_M2003 == 1)	/* MCP */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF0MFP_Msk | SYS_GPF_MFPL_PF1MFP_Msk));
    GPIO_SetMode(PF, BIT0, GPIO_MODE_INPUT);
    GPIO_SetMode(PF, BIT1, GPIO_MODE_INPUT);
#endif

    I2C0_Init();

#if (OPT_M251 == 1)
    /* Make Host UART port bypass to D+/D- */
    //UFCS_Init();
    /* Init UART1 to 115200-8n1 for print message */
#if (OPT_FPGA == 1)
    UART_Open(UART0, 460800);
#else
    UART_Open(UART1, 460800);
#endif
    DBG_PRINTF("UART_Init\n")
#endif
#if (OPT_M2003 == 1)
    /* Make Host UART port bypass to D+/D- */
    UFCS_Init();

    /**
      Due to MCP, D+ bound with M_nReset together. And M2003 PE15 wire with M_nReset
      To avoid UART output to PE15(M2003 nReset). It needs to set PE15 to GPIO Input mode
      **/
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE15MFP_Msk));
    GPIO_SetMode(PE, BIT15, GPIO_MODE_INPUT);

    /* Init UART0 to 115200-8n1 for print message */
    //UART_Open(UART0, 115200);
    UART_Open(UART0, 460800);
    DBG_PRINTF("UART Init\n");
#endif

    //VinDacFormula_Test(0);    	//by formula to set DAC value
    //VinDacInterlace_Test(0);		//by trimmed 5V/48V data to set DAC value

#if (CONFIG_COMMAND_SHELL == 1)
#if (OPT_M251 == 1)
#if (OPT_FPGA == 1)
    NVIC_EnableIRQ(UART0_IRQn);
    //UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk));
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
#else
    NVIC_EnableIRQ(UART1_IRQn);
    //UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk));
    UART_EnableInt(UART1, UART_INTEN_RDAIEN_Msk);
#endif
#endif
#if (OPT_M2003 == 1)
    NVIC_EnableIRQ(UART0_IRQn);
    //UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk));
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
#endif
#endif

    /* Set timer frequency to 1000HZ for system time base */
    TIMER0_Init();

    /* NPD48_ChipInit */
    DBG_PRINTF("NPD48 Init\n");
    NPD48_ChipInit(port);
    printf("NPD48 Init Done\n");
#if (CONFIG_NPD48_VIN_OVP == 1)
    /** VIN OVP/UVP
      * Enable/Disable as UTCPD_Callback
      * UTCPD_PD_PS_TRANSITION disable OVP/UVP
      * UTCPD_PD_PS_READY enable OVP/UVP
      **/
#endif

#if (CONFIG_NPD48_VBUS_OCP == 1)
    /** VBUS OCP
      * By polling ADC value to filter out noise
      **/
#endif

#if (CONFIG_NPD48_NTCP == 1)
    /**
      * NTCP on board WF104
    	* 25 degree Celsius --> 100K, 20u*100K = 2000mV
    	* 100 degree Celsius --> 5K,  20u*5K = 100mV
    	* Temperatur coefficient is 1.9V/75 => 25.3mV/degree
      **/
#if(CONFIG_TEST_NTCP == 1)
    pd_protection_event_threshold(port, 3, 609);		/**
																											* degree = 25+(2000-mV)/25.3;
                                                      * mV = 2000-(degree-25)*25.3
																											* 80 degree Celsius ==> 609.
																											**/
#else
    pd_protection_event_threshold(port, 3, 100);		/**
                                                      * Parameter: 3 meaning External Temperature Sensor Protection
                                                      * Parameter: 100 means 100mV on the thermosensor@ 100 degree Celsius
																											**/
#endif
#endif

#if (CONFIG_NPD48_CC_OVP == 1)
    /**
      * CC OVP
      **/
    pd_protection_event_threshold(port, 4, 0);              //Builtin CC OVP comparator
#endif

#if (CONFIG_NPD48_VCONN_OCP == 1)
    /**
      * VCONN OCP
      **/
    pd_protection_event_threshold(port, 5, 0);              //Builtin VCONN OCP comparator
#endif

#if (CONFIG_NPD48_CHIP_OVP == 1)
    /**
      * Chip OVP
      **/
#if (CONFIG_TEST_PDO36V_COVP == 1)
    pd_protection_event_threshold(port, 6, 40000);          //Builtin Chip OCP comparator
#else
    pd_protection_event_threshold(port, 6, 56000);          //Builtin Chip OCP comparator
#endif
#endif

#if (CONFIG_NPD48_ITP == 1)
    /**
      * Built-In ITP
      * 0 degree Celsius --> 708.28mV
    	* Temperatur coefficient is 1.84 mV/degree
      * 100 degree Celsius, 708 - 1.84*100 ~= 524 mV
    	*		degree = (708280-uv)/1840;  --> uV = 708280-degree*1840 . uV = 708280-147200 = 561080 uV.
    	* 105 degree Celsius, 708 - 1.84*100 ~= 515 mV
    	*		degree = (708280-uv)/1840;  --> uV = 708280-degree*1840 . uV = 708280-147200 = 561080 uV.
    	*
      **/
    /* Enable ITP sensor power */
    tcpc_update8(port, NPD48_PWRCR, BIT6, MASK_SET);
#if (CONFIG_TEST_ITP == 1)
    pd_protection_event_threshold(port, 7, 561);		/* 80 degree Celsius on thermosensor is about 531mVm
																											It will be transform into ADC in the API */
#else
    pd_protection_event_threshold(port, 7, 515);		/* 105 degree Celsius on thermosensor is about 515mV
																											It will be transform into ADC in the API */
#endif
#endif


#if(OPT_M251 == 1) //M252 FAE nALERT (GPB1)
#if (OPT_FPGA == 1)
    /* M253 PB5 acts as Alert */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk));
    GPIO_SetPullCtl(PB, BIT5, GPIO_PUSEL_PULL_UP);
    /* Configure PB.1 as GPB Int and enable interrupt by Low Level trigger */
    GPIO_SetMode(PB, BIT5, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 5, GPIO_INT_LOW);
    NVIC_EnableIRQ(GPB_IRQn);
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, BIT5);
#else
    /* M253 PB1 acts as Alert */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB1MFP_Msk));
    GPIO_SetPullCtl(PB, BIT1, GPIO_PUSEL_PULL_UP);
    /* Configure PB.1 as GPB Int and enable interrupt by Low Level trigger */
    GPIO_SetMode(PB, BIT1, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 1, GPIO_INT_LOW);
    NVIC_EnableIRQ(GPB_IRQn);
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, BIT1);
#endif
#endif

#if(OPT_M2003 == 1) //M2003 FAE nALERT (GPB11)
    /* M2003 PB11 acts as Alert */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB11MFP_Msk));
    GPIO_SetPullCtl(PB, BIT11, GPIO_PUSEL_PULL_UP);
    /* Configure PB.11 as GPB Int and enable interrupt by Low Level trigger */
    GPIO_SetMode(PB, BIT11, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 11, GPIO_INT_LOW);
    NVIC_EnableIRQ(GPB_IRQn);
#endif

    /* NPD48_CertificationFineTune */
    NPD48_CertificationFineTune(port);

#if (CONFIG_DEBUG_PIN == 1)
    GPIO_SetPullCtl(PB, BIT6, GPIO_PUSEL_PULL_UP);
    GPIO_SetMode(PB, BIT6, GPIO_MODE_OUTPUT);
    PB6 = 1;
    PB6 = 0;
    PB6 = 1;
#endif

    i2c_write8(port, NULL, NPD48_PEGCTL, 0);
    i2c_read8(port, NULL, NPD48_PEGCTL, &data);
    if(data != 0)
    {
        delay(80);
    }

    pd_task();
}

#if 1
void HardFault_Handler(void)
{
    DBG_PRINTF("Hard Fault\n");
    while(1);
}
#else
void hard_fault_handler_c(unsigned int * hardfault_args, unsigned lr_value)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;
    DBG_PRINTF("hard fault\n");
    stacked_r0 = ((unsigned int) hardfault_args[0]);
    stacked_r1 = ((unsigned int) hardfault_args[1]);
    stacked_r2 = ((unsigned int) hardfault_args[2]);
    stacked_r3 = ((unsigned int) hardfault_args[3]);
    stacked_r12 = ((unsigned int)hardfault_args[4]);
    stacked_lr = ((unsigned int) hardfault_args[5]);
    stacked_pc = ((unsigned int) hardfault_args[6]);
    stacked_psr = ((unsigned int) hardfault_args[7]);

    {
        printf("[Hard fault handler]\r\n");
        DBG_PRINTF("R0 = %x\r\n", stacked_r0);
        DBG_PRINTF("R1 = %x\r\n", stacked_r1);
        DBG_PRINTF("R2 = %x\r\n", stacked_r2);
        DBG_PRINTF("R3 = %x\r\n", stacked_r3);
        DBG_PRINTF("R12 = %x\r\n", stacked_r12);
        DBG_PRINTF("Stacked LR = %x\r\n", stacked_lr);
        DBG_PRINTF("Stacked PC = %x\r\n", stacked_pc);
        DBG_PRINTF("Stacked PSR = %x\r\n", stacked_psr);
        DBG_PRINTF("SCB_SHCSR=%x\r\n", SCB->SHCSR);
        DBG_PRINTF("Current LR = %x\r\n", lr_value);
    }
    while(1);
}
#endif
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
