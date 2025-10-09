
/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/18 3:19p $
 * @brief
 *           Demonstrates Source Role Power Device
 * @note
 *
 * Copyright (c) 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "utcpdlib.h"

#define ADC_INIT

void UTCPD_IRQHandler(void)
{
    uint32_t port = 0;
    tcpci_tcpc_alert(port);
}
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV4);

    /* Enable Internal RC 32KHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable GPIO Clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UTCPD clock */
    CLK_EnableModuleClock(UTCPD0_MODULE);

    /* === Enable IP clock === */

    /* Enable TIMER 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    //CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_LIRC, 0);

    /* Enable TIMER 1 module clock */
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    /* Enable EADC peripheral clock */
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_HIRC, CLK_CLKDIV0_EADC0(8));
    CLK_EnableModuleClock(EADC0_MODULE);

    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC, 72000000);
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(SYS_GPB_MFP3_PB12MFP_Msk | SYS_GPB_MFP3_PB13MFP_Msk)) |
                    (SYS_GPB_MFP3_PB12MFP_UART0_RXD | SYS_GPB_MFP3_PB13MFP_UART0_TXD);

#ifdef ADC_INIT
    /* Set PB.2 - PB.3 to input mode For EADC pin to measure VBUS and VCONN */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);    //For VBUS Voltage
    GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);    //For VCONN Voltage

    /* Configure the PB.2 - PB.3 ADC analog input pins. */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~( SYS_GPB_MFP0_PB2MFP_Msk | SYS_GPB_MFP0_PB3MFP_Msk)) |
                    (SYS_GPB_MFP0_PB2MFP_EADC0_CH2 | SYS_GPB_MFP0_PB3MFP_EADC0_CH3 );
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2 | BIT3);

#if 0  /* ADC comparator */
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);    //For VBUS Over Curerent (ADC compaarator)
    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);    //For VCONN Over Curerent  (ADC compaarator)
    /* VBUS Over Current from EADC comparator 0 */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB4MFP_Msk) |   SYS_GPB_MFP1_PB4MFP_EADC0_CH4);

    /* VCONN Over Current from EADC comparator 1*/
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB0MFP_Msk) |   SYS_GPB_MFP0_PB0MFP_EADC0_CH0);

    /* Disable the PB.2 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2 | BIT3 | BIT4 | BIT0);
#endif
#endif

    /* Configure UUTCPD CC1/CC2 */
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(SYS_GPC_MFP0_PC0MFP_Msk | SYS_GPC_MFP0_PC1MFP_Msk)) | (SYS_GPC_MFP0_PC0MFP_UTCPD0_CC1 | SYS_GPC_MFP0_PC1MFP_UTCPD0_CC2);

    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB5MFP_Msk | SYS_GPB_MFP1_PB4MFP_Msk)) |
                    (SYS_GPB_MFP1_PB5MFP_INT0 | SYS_GPB_MFP1_PB4MFP_INT1);

    /* UTCPD VBSRCEN Multiple Function Pin */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~SYS_GPA_MFP0_PA2MFP_Msk) | SYS_GPA_MFP0_PA2MFP_UTCPD0_VBSRCEN;

    /* UTCPD VBSNKEN Multiple Function Pin */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~SYS_GPA_MFP0_PA3MFP_Msk) | SYS_GPA_MFP0_PA3MFP_UTCPD0_VBSNKEN;

    /* UTCPD FRSCC1 and FRS_CC2  Multiple Function Pin */
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(SYS_GPC_MFP1_PC4MFP_Msk | SYS_GPC_MFP1_PC5MFP_Msk)) |
                    (SYS_GPC_MFP1_PC4MFP_UTCPD0_FRSTX1 | SYS_GPC_MFP1_PC5MFP_UTCPD0_FRSTX2);

    /* UTCPD VCONN Enable: VCEN0:PA0, VCEN1:PB0, Multiple Function Pin */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~SYS_GPA_MFP0_PA0MFP_Msk) | SYS_GPA_MFP0_PA0MFP_UTCPD0_VCNEN1;
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~SYS_GPB_MFP0_PB0MFP_Msk) | SYS_GPB_MFP0_PB0MFP_UTCPD0_VCNEN2;


    /* UTCPD VCONN Discharge: Don't force VCONN Discharge First */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk));
    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);

    /* Lock protected registers */
//    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* Base timer initializtion function                                                                       */
/* The timer is used to maintain the PD protocol stack.                                                    */
/* The tick unit is 1 millisecond.                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void TIMER0_Init(void)
{
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    /* Enable timer interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    /* Start Timer 0 */
    TIMER_Start(TIMER0);
}

/*---------------------------------------------------------------------------------------------------------*/
/* VBUS/VCONN voltage conversion timer initializtion function                                              */
/* The timer is used to trigger EADC to conversion VBUS or VCONN.                                          */
/* The tick unit is 10 millisecond (depends on the system requirement)                                     */
/*---------------------------------------------------------------------------------------------------------*/
void TIMER1_Init(void)
{
    /* Reset TMR1 */
    SYS_ResetModule(TMR1_RST);

    /* Set timer frequency to 100HZ */
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 100);

    /* Enable timer interrupt */
//    TIMER_EnableInt(TIMER0);
//    NVIC_EnableIRQ(TMR0_IRQn);

    /* Timer0 trigger target is EADC */
    TIMER1->TRGCTL |= TIMER_TRGCTL_TRGEADC_Msk;
    /* Start Timer 0 */
    TIMER_Start(TIMER1);
}

/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ.
 *              Software Timer base for PD check time our mechanism
 */
static volatile uint32_t pd_vbus_transition_tick = 0;
extern void vbus_discharge(int enable);
void TMR0_IRQHandler(void)
{
    UTCPD_TimerBaseInc();

    if(pd_vbus_transition_tick != 0)
    {   /* VBUS Discharge if VBUS from High Level to Low Level */
        pd_vbus_transition_tick -= 1;
        if(pd_vbus_transition_tick == 0)
            vbus_discharge(0);		/* Stop VBUS Discharge */
    }

    /* clear timer interrupt flag */
    TIMER_ClearIntFlag(TIMER0);
}


/**
 * @brief       Timer1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer1 default IRQ.
 *              Utilizaton for driving ADC trigger conversion VBUS and VCONN to periodicly
 */
void TMR1_IRQHandler(void)
{
    //printf("%d sec\n", u32Sec++);

    /* clear timer interrupt flag */
    TIMER_ClearIntFlag(TIMER1);
}


/**
  *	To get requested PDO's information then to set the power circuit.
  * If the requested PDO is Fixed PDO, user will parsing the pd_src_pdo[] array bases on request PDO index.
  * If the requested PDO is PPS PDO, user should call API pd_get_adjoutput_voltage_current() to get the requested information.
  **/
void pd_get_request_pdo_info(int port, uint32_t pdo_idx, uint32_t* u32volt, uint32_t* u32curr)
{
    uint32_t pdopos; 	/* It will be equal to pdo_idx */
    if(pdo_idx <= pd_src_pdo_cnt)
    {   //SPR
        pdo_idx = pdo_idx - 1;
        if( (pd_src_pdo[pdo_idx] & PDO_TYPE_MASK) == PDO_TYPE_FIXED)
        {   //FIXED
            *u32volt = PDO_FIXED_GET_VOLT(pd_src_pdo[pdo_idx]);
            *u32curr = PDO_FIXED_GET_CURR(pd_src_pdo[pdo_idx]);
        }
        else if( (pd_src_pdo[pdo_idx] & PDO_TYPE_MASK) == PDO_TYPE_AUGMENTED)
        {   //PPS
            pd_get_adjoutput_voltage_current(port, &pdopos, u32volt, u32curr);
        }
    }
#if 0 /* M2L31 didn't support EPR */
    else if(pdo_idx >= 8 )
    {   //EPR
        pdo_idx = pdo_idx - 8;
        if( (pd_src_epr_pdo[pdo_idx] & PDO_TYPE_MASK) == PDO_TYPE_FIXED)
        {   //Fix
            *u32volt = PDO_FIXED_GET_VOLT(pd_src_epr_pdo[pdo_idx]);
            *u32curr = PDO_FIXED_GET_CURR(pd_src_epr_pdo[pdo_idx]);
        }
        else if( (pd_src_epr_pdo[pdo_idx] & PDO_TYPE_MASK) == PDO_TYPE_AUGMENTED)
        {   //AVS
            pd_get_adjoutput_voltage_current(port, &pdopos, u32volt, u32curr);
        }
    }
#endif
}

/**
 * @brief       UUTCPD Callback Function
 *
 * @param       event: UUTCPD_PD_ATTACHED = 0,                 : Port partner attached or disattached
 *                     UUTCPD_PD_CONTRACT = 1,                 : PD contract established
 *                     UTCPD_PD_SNK_VOLTAGE = 2,               : SNK Role Contract voltage
 *                     UTCPD_PD_CABLE_MAX_POWER = 3,           : M2L31 Didn't Support. NPD48: Cable Max Voltage and Max Current : ((max_vol<<16) | max_curr)
 *                     UTCPD_PD_VCONN_DISCHARGE = 4,           : M2L31/NPD48 Didn't Support. To do VCONN Discharge
 *                     UTCPD_PD_PS_TRANSITION = 5,             : M2L31 Didn't Support. NPD48: To Disable VIN OVP/UVP, RDO IDX
 *                     UTCPD_PD_PS_READY = 6,                  : M2L31 Didn't Support. NPD48: To Enable VIN OVP/UVP
 *                     UTCPD_PD_VIN_DISCHARGE_DONE = 7,        : M2L31 Didn't Support. NPD48: Inform Upper Layer VIN Discharge Done,
 *                     UTCPD_PD_ACCEPT_REQUEST_PDO = 8,        : M2L31/NPD48 Inform Upper Layer to Provide the Requested PDO
 *                     UTCPD_PD_VIN_DISCHAGE = 9, 	           : M2L31/NPD48 Didn't Support.
 *                     UTCPD_PD_RECEIVE_HR = 10, 	             : M2L31 Didn't Support. NPD48: PD Receive Hard Reset
 *
 *                     UTCPD_PD_VBUS_DISCHARGE_START = 0x20,   : VBUS Discharge Start
 *                     UTCPD_PD_VBUS_DISCHARGE_STOP = 0x21,    : VBUS Discharge End
 *                     UTCPD_PD_TC_ASSERT_RD = 0x22,           : Assert Rd
 *                     UTCPD_PD_TC_PD_DISCONNECTION = 0x23,    : Deattached
 *                     UTCPD_PD_SRC_TC_PD_CONNECTION = 0x24,   : Acts As SRC Role
 *                     UTCPD_PD_SNK_TC_PD_CONNECTION = 0x25,   : Acts As SNK Role
 *                     UTCPD_PD_GET_BATTERY_CAP = 0x26         : GET_BAT_CAP Message From Port Partner
 *                     UTCPD_PD_GET_BATTERY_STATUS = 0x27,     : GET_BAT_STATUS Message From Port Partner
 *                     UTCPD_PD_SNK_REC_SOURCE_CAP = 0x28,     : SNK Role Receive SOURCE_CAP Message From Port Partner
 *                     UTCPD_PD_SNK_REC_ACCEPT = 0x29,         : SNK Role Receive ACCEPT Message From Port Partner
 *                     UTCPD_PD_SRC_SEND_ACCEPT = 0x2A,        : SRC Role Send ACCEPT Message to Accept Power Negotiation
 *              op:    event flag set/clear or the 2nd parameter
 *
 * @return      None
 *
 * @details     None
 *
 */
extern void vbus_discharge(int enable);
static bool bIsConnection = FALSE;
static uint32_t u32RecVolt = 0;
extern void VBUS_Source_Level(int port, char i8Level);
void UTCPD_Callback(int port, E_UTCPD_PD_EVENT event, uint32_t op)
{
    printf("Callback event = %d\n", event);
    printf("op = %d\n", op);
    if(event == UTCPD_PD_VBUS_DISCHARGE_START)
    {
        printf("VBUS DISG B\n");
        vbus_discharge(1);		//Turn ON the power path to start discharge
    }
    else if(event == UTCPD_PD_VBUS_DISCHARGE_STOP)
    {
        printf("VBUS DISG E\n");
        vbus_discharge(0);	//Turn Off the power path to stop discharge
    }
    else if(event == UTCPD_PD_TC_PD_DISCONNECTION)
    {
        u32RecVolt = 0; 						/* Disconnection --> VBUS to zero */
        bIsConnection = FALSE;
    }
    else if(event == UTCPD_PD_ACCEPT_REQUEST_PDO)
    {   /* Inform Upper layer to Provide the Power of Requested PDO */
        uint32_t u32volt, u32curr;
        uint32_t pdo_idx = op;
        pd_get_request_pdo_info(port, op, &u32volt, &u32curr);
        if (u32RecVolt > u32volt)
        {   /* Start up VBUS discharge */
            printf("discharge\n");
            vbus_discharge(1);		//Turn ON the power path to start discharge
            pd_vbus_transition_tick = 100;  /* Start up VBUS Discharge 100ms */
        }
        printf("old/new = %d / %d\n", u32RecVolt, u32volt);
        u32RecVolt = 	u32volt;
        VBUS_Source_Level(port, pdo_idx);
    }
}


void pd_task(void)
{
    int port = TASK_ID_TO_PD_PORT(task_get_current());

    /*
     * If port does not exist, return
     */
    if (port >= board_get_usb_pd_port_count())
        return;

    /* Install UUTCPD Callback Function */
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
            /* User Tasks:
             * Please separate User Tasks code piece by piece.
             * Suggestion not to over 100us for every piece code.
             * pd_task_loop() needs to be called every 1ms
             */
#if (CONFIG_COMMAND_SHELL == 1)
            UART_Commandshell(port);
#endif
            continue;
        }
    }
}

void UTCPD_Init(int port)
{

    UTCPD_Open(port);

    /* Didn't Force VCONN Discharge PA1 */
    PA1 = 0;

    /* VBSRCEN Polarity */
    UTCPD_vbus_srcen_polarity_active_high(port);

    /* VBSNKEN Polarity */
    UTCPD_vbus_snken_polarity_active_high(port);

    /* FRSTXCC1 and FRSTXCC2 Polarity */
    UTCPD_frs_tx_polarity_active_high(port);

    UTCPD_vconn_polarity_active_low(port);

    /** Due to board connect Vref pin to AVDD33
      * Set reference voltage to external pin
      * Sink disconnection threshold base on 1/10 voltage
      * will be 350/(3300/1024) ~= 108.6
      **/
    SYS_SetVRef(SYS_VREFCTL_VREF_PIN);
    i2c_write16(port, (uint16_t)0, TCPC_REG_VBUS_SINK_DISCONNECT_THRESH, 108);

    /* Set External Voltage Divider 1/10 */
    UTCPD_SetExternalDivider(port, 10);
}
int main()
{
    int32_t port = 0;

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    RMC->CYCCTL = 0x108;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);
    printf("UART Initial\n");

    /* Init UTCPD */
    UTCPD_Init(port);

#if (CONFIG_COMMAND_SHELL == 1)
    /* Enable UART RDA interrupt for command */
    NVIC_EnableIRQ(UART0_IRQn);
    //UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk));
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
#endif


    printf("DRP: UART Init\n");

    /* Set timer frequency to 1000HZ for system time base */
    TIMER0_Init();

#ifdef ADC_INIT
    /* Set timer frequency to 100HZ for measuring VBUS/VCONN*/
    TIMER1_Init();
    EADC_ConfigPins();
    EADC_Init();
    //EADC_Compare_Init();
#endif

#ifdef ACMP_INIT
    ACMP_Init();
#endif

    /* Non-VBUS PB1 Discharge */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(0xFFUL << (1 * 8))) | (0UL << (1 * 8));
    GPIO_ENABLE_DIGITAL_PATH(PB, BIT1);
    GPIO_SetMode(PB, BIT1, GPIO_MODE_OUTPUT);

    /* Google EC need to enable interrupt */
    NVIC_EnableIRQ(UTCPD_IRQn);

    pd_task();

}



#if 0
void HardFault_Handler(void)
{
    printf("Hard Fault\n");
    while(1);
}
#else
void hard_fault_handler_c(unsigned int * hardfault_args, unsigned lr_value)
{
#if 0
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;
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
        printf("R0 = %x\r\n", stacked_r0);
        printf("R1 = %x\r\n", stacked_r1);
        printf("R2 = %x\r\n", stacked_r2);
        printf("R3 = %x\r\n", stacked_r3);
        printf("R12 = %x\r\n", stacked_r12);
        printf("Stacked LR = %x\r\n", stacked_lr);
        printf("Stacked PC = %x\r\n", stacked_pc);
        printf("Stacked PSR = %x\r\n", stacked_psr);
        printf("SCB_SHCSR=%x\r\n", SCB->SHCSR);
        printf("Current LR = %x\r\n", lr_value);
    }
#endif
    while(1);
}
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
#endif /* #if 0 */
