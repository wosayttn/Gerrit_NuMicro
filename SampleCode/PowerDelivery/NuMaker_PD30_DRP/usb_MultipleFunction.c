/**************************************************************************//**
 * @file     usb_MultipleFunction.c
 * @version  V0.10
 * @brief    UUTCPD Multiple Function Pin
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "utcpdlib.h"

void vconn_disable_src_cc(void)
{
    /* Disable VCONN Source CC*/
    outp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL, inp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL) & ~TCPC_REG_POWER_CTRL_ENABLE_VCONN);
}
/*
    The CC ststus interupt won't be issued if Enable_VCONN_SRC_CCC with wrong VCONN source CC pin
*/

void vconn_enable_src_cc(void)
{
    /* Enable VCONN Source CC*/
    outp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL, inp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL) | TCPC_REG_POWER_CTRL_ENABLE_VCONN);
}
void vconn_from_cc2(void)
{
    /* Enable VCONN Source CC2 */
    /* Communication through CC1 */
    outp32(UTCPD0_BASE+TCPC_REG_TCPC_CTRL, inp32(UTCPD0_BASE+TCPC_REG_TCPC_CTRL) & ~TCPC_REG_TCPC_CTRL_PLUG_ORIENTATION);
}
void vconn_from_cc1(void)
{
    /* Enable VCONN Source CC1*/
    /* Communication through CC2 */
    outp32(UTCPD0_BASE+TCPC_REG_TCPC_CTRL, inp32(UTCPD0_BASE+TCPC_REG_TCPC_CTRL) | TCPC_REG_TCPC_CTRL_PLUG_ORIENTATION);
}
void vconn_polarity_active_low()
{
    /* Set VCONN Polarity Active Low due to CC1VCENS and CC2VCENS default low  */
    outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) & ~TCPC_REG_PINPL_VCEN);
}
void vconn_polarity_active_high()
{
    /* Set VCONN Polarity Active Low due to CC1VCENS and CC2VCENS default high */
    outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) | TCPC_REG_PINPL_VCEN);
}
/* VCONN ocp fault */
void vconn_disable_oc_fault(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL, inp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL) | TCPC_REG_FAULT_CTRL_VCONN_OCP_FAULT_DIS);
}
void vconn_enable_oc_fault(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL, inp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL) & ~TCPC_REG_FAULT_CTRL_VCONN_OCP_FAULT_DIS);
}

void vconn_mux_selection(uint32_t cc1vcensel, uint32_t cc2vcensel)
{
    outp32(UTCPD0_BASE+UTCPD_MUXSEL, (inp32(UTCPD0_BASE+UTCPD_MUXSEL) &~(CC2VCENS|CC1VCENS)) |
           ((cc1vcensel<<24) | (cc2vcensel<<28)));
}

void vconn_configure_oc_detection_soruce(uint32_t u32Src)
{
    /* Configure Force Off VCONN source detection: EINTx/ADC/ACMP0/ACMP1/ACMP2 */
    outp32(UTCPD0_BASE+UTCPD_MUXSEL, (inp32(UTCPD0_BASE+UTCPD_MUXSEL) &~VCOCS) | (u32Src<<4));
}

/* ============  VBUS ocp fault ==========*/
void vbus_srcen_polarity_active_low(int port)
{
    /* Set VBUS SRCEN Polarity active Low */
    outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) & ~TCPC_REG_PINPL_SRCEN);
}
void vbus_srcen_polarity_active_high(int port)
{
    /* Set VBUS SRCEN Polarity Active high */
    outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) | TCPC_REG_PINPL_SRCEN);
}

void vbus_srnk_polarity_active_low(int port)
{	
	/* Set VBUS SRCEN Polarity active Low */
	outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) & ~TCPC_REG_PINPL_SNKEN); 
}
void vbus_snken_polarity_active_high(int port)
{	
	/* Set VBUS SRCEN Polarity Active high */
	outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) | TCPC_REG_PINPL_SNKEN); 
}


void vbus_disable_oc_fault(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL, inp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL) | TCPC_REG_FAULT_CTRL_VBUS_OCP_FAULT_DIS);
}
void vbus_enable_oc_fault(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL, inp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL) & ~TCPC_REG_FAULT_CTRL_VBUS_OCP_FAULT_DIS);
}

void vbus_discharge_polarity_active_low()
{
    /* Set VBUS discharge Polarity Active low */
    outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) & ~TCPC_REG_PINPL_VBDCHG);
}
void vbus_discharge_polarity_active_high()
{
    /* Set VBUS discharge Polarity Active high */
    outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) | TCPC_REG_PINPL_VBDCHG);
}


void vbus_configure_oc_soruce(uint32_t u32Src)
{
    /* Configure VBUS Over Current source detection: EINTx/ADC/ACMP0/ACMP1/ACMP2 */
    outp32(UTCPD0_BASE+UTCPD_MUXSEL, (inp32(UTCPD0_BASE+UTCPD_MUXSEL) &~VBOCS) | (u32Src<<0));
}

/* VBUS ovp fault */
void vbus_disable_ov_fault(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL, inp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL) | TCPC_REG_FAULT_CTRL_VBUS_OVP_FAULT_DIS);
}
void vbus_enable_ov_fault(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL, inp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL) & ~TCPC_REG_FAULT_CTRL_VBUS_OVP_FAULT_DIS);
}

/* VBUS force off fault */
void vbus_disable_forceoff_fault(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL, inp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL) | TCPC_REG_FAULT_CTRL_VBUS_FORCE_OFF_DIS);
}
void vbus_enable_forceoff_fault(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL, inp32(UTCPD0_BASE+TCPC_REG_FAULT_CTRL) & ~TCPC_REG_FAULT_CTRL_VBUS_FORCE_OFF_DIS);
}


uint32_t vbus_is_source(void)
{
    return ((inp32(UTCPD0_BASE+TCPC_REG_POWER_STATUS) & TCPC_REG_POWER_STATUS_SOURCING_VBUS) == TCPC_REG_POWER_STATUS_SOURCING_VBUS) ? 1:0;
}

uint32_t vbus_is_sink(void)
{
    return ((inp32(UTCPD0_BASE+TCPC_REG_POWER_STATUS) & TCPC_REG_POWER_STATUS_SINKING_VBUS) == TCPC_REG_POWER_STATUS_SINKING_VBUS) ? 1:0;
}

uint32_t vbus_is_source_hv(void)
{
    return ((inp32(UTCPD0_BASE+TCPC_REG_POWER_STATUS) & TCPC_REG_POWER_STATUS_SOURCING_HIGH_VBUS) == TCPC_REG_POWER_STATUS_SOURCING_HIGH_VBUS) ? 1:0;
}

//=================================== for VBUS and VCONN
void power_enable_monitor(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL) & ~TCPC_REG_POWER_CTRL_VBUS_VOL_MONITOR_DIS));
}

void power_disable_monitor(void)
{
    outp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL) | TCPC_REG_POWER_CTRL_VBUS_VOL_MONITOR_DIS));
}

/* VBUS enable auto discharge */
void power_disable_auto_discharge(void)
{
    /* Disable Auto Discharge = 0 */
    outp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL, inp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL) & ~TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT);
}
void power_enable_auto_discharge(void)
{
    /* Enable Auto Discharge = 0 */
    outp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL, inp32(UTCPD0_BASE+TCPC_REG_POWER_CTRL) | TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT);
}

void frs_tx_polarity_active_low()
{
    /* Set VBUS discharge Polarity Active low */
    outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) & ~TCPC_REG_PINPL_FRSTX);
}
void frs_tx_polarity_active_high()
{
    /* Set VBUS discharge Polarity Active high */
    outp32(UTCPD0_BASE+TCPC_REG_PINPL, inp32(UTCPD0_BASE+TCPC_REG_PINPL) | TCPC_REG_PINPL_FRSTX);
}

void frs_mux_selection(uint32_t cc1frssel, uint32_t cc2frssel)
{
    outp32(UTCPD0_BASE+UTCPD_MUXSEL, (inp32(UTCPD0_BASE+UTCPD_MUXSEL) &~(CC2FRSS|CC1FRSS)) |
           ((cc1frssel<<25) | (cc2frssel<<29)));
}

void GPIO_Toggle(void)
{

	GPIO_SetMode(PD, BIT15, GPIO_MODE_OUTPUT);
    PD15 = 1;
    PD15 = 0;
    delay(1);
    PD15 = 1;
}

void GPIO_ForcePulse(void)
{
#if 0
    uint64_t u64Btime, u64Etime;
    PB2 = 0;
    u64Btime = __hw_clock_source_read64();
    PB2 = 1;
    do
    {
        u64Etime = __hw_clock_source_read64();
    }
    while((u64Etime-u64Btime) < 30);
    PB2 = 0;
#else
    PB2 = 1;
    usdelay(1);
    PB2 = 0;
#endif
}

#if 0
----------------------------------------
UUTCPD0
UUTCPD0_CC1      PC.0    17
UUTCPD0_CC2      PC.1    17
UUTCPD0_CCDB1    PC.2    17
UUTCPD0_CCDB2    PC.3    17
UUTCPD0_DISCHG   PA.1    18
PC.4    18
PC.5    18
PD.15   18
PF.0    18
PF.1    18
UUTCPD0_FRSTX1   PA.1    17
PC.4    17
PF.1    17
UUTCPD0_FRSTX2   PC.5    17
PD.15   17
PF.0    17
UUTCPD0_VBDCHG   PB.1    17
UUTCPD0_VBSNKEN  PA.3    17
PA.5    17
PA.7    17
PB.15   17
PF.5    17
UUTCPD0_VBSRCEN  PA.2    17
PA.4    17
PA.6    17
PB.14   17
PF.4    17
UUTCPD0_VCNEN1   PA.0    17
UUTCPD0_VCNEN2   PB.0    17
----------------------------------------
#endif


void UUTCPD_MultipleFunction()
{
    //UUTCPD0_CC1      PC.0    17
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(0xFFUL << (0*8))) | (17UL << (0*8));
    //UUTCPD0_CC2      PC.1    17
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(0xFFUL << (1*8))) | (17UL << (1*8));

    //UUTCPD0_CCDB1    PC.2    17
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(0xFFUL << (2*8))) | (17UL << (2*8));
    //UUTCPD0_CCDB2    PC.3    17
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(0xFFUL << (3*8))) | (17UL << (3*8));


    //UUTCPD0_DISCHG   PA.1    18
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (1*8))) | (18UL << (1*8));
    //UUTCPD0_DISCHG   PC.4    18
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(0xFFUL << (0*8))) | (18UL << (0*8));
    //UUTCPD0_DISCHG   PC.5    18
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(0xFFUL << (1*8))) | (18UL << (1*8));
    //UUTCPD0_DISCHG   PD.15   18
    SYS->GPD_MFP3 = (SYS->GPD_MFP3 & ~(0xFFUL << (3*8))) | (18UL << (3*8));
    //UUTCPD0_DISCHG   PF.0    18
    SYS->GPF_MFP0 = (SYS->GPF_MFP0 & ~(0xFFUL << (0*8))) | (18UL << (0*8));
    //UUTCPD0_DISCHG     PF.1    18
    SYS->GPF_MFP0 = (SYS->GPF_MFP0 & ~(0xFFUL << (1*8))) | (18UL << (1*8));

    //UUTCPD0_FRSTX1   PA.1    17
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (1*8))) | (17UL << (1*8));
    //UUTCPD0_FRSTX1   PC.4    17
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(0xFFUL << (0*8))) | (17UL << (0*8));
    //UUTCPD0_FRSTX1   PF.1    17
    SYS->GPF_MFP0 = (SYS->GPF_MFP0 & ~(0xFFUL << (1*8))) | (17UL << (1*8));

    //UUTCPD0_FRSTX2   PC.5    17
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(0xFFUL << (1*8))) | (17UL << (1*8));
    //UUTCPD0_FRSTX2   PD.15   17
    SYS->GPD_MFP3 = (SYS->GPD_MFP3 & ~(0xFFUL << (3*8))) | (17UL << (3*8));
    //UUTCPD0_FRSTX2   PF.0    17
    SYS->GPF_MFP0 = (SYS->GPF_MFP0 & ~(0xFFUL << (0*8))) | (17UL << (0*8));

    //UUTCPD0_VBDCHG   PB.1    17
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(0xFFUL << (1*8))) | (17UL << (1*8));

    //UUTCPD0_VBSNKEN  PA.3    17
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (3*8))) | (17UL << (3*8));
    //UTCPD0_VBSNKEN  PA.5    17
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(0xFFUL << (1*8))) | (17UL << (1*8));
    //UUTCPD0_VBSNKEN  PA.7    17
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(0xFFUL << (3*8))) | (17UL << (3*8));
    //UUTCPD0_VBSNKEN  PB.15   17
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(0xFFUL << (3*8))) | (17UL << (3*8));
    //UUTCPD0_VBSNKEN  PF.5    17
    SYS->GPF_MFP1 = (SYS->GPF_MFP1 & ~(0xFFUL << (1*8))) | (17UL << (1*8));
    //UUTCPD0_VBSNKEN  PA.3    17
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (3*8))) | (17UL << (3*8));

    //UUTCPD0_VBSRCEN  PA.2    17
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (2*8))) | (17UL << (2*8));
    //UUTCPD0_VBSRCEN  PA.4    17
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(0xFFUL << (0*8))) | (17UL << (0*8));
    //UUTCPD0_VBSRCEN  PA.6    17
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(0xFFUL << (2*8))) | (17UL << (2*8));
    //UUTCPD0_VBSRCEN  PB.14   17
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(0xFFUL << (2*8))) | (17UL << (2*8));
    //UUTCPD0_VBSRCEN  PF.4    17
    SYS->GPF_MFP1 = (SYS->GPF_MFP1 & ~(0xFFUL << (0*8))) | (17UL << (0*8));

    //UUTCPD0_VCNEN1   PA.0    17
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (0*8))) | (18UL << (0*8));
    //UUTCPD0_VCNEN2   PB.0    17
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(0xFFUL << (0*8))) | (18UL << (0*8));

}

