/**************************************************************************//**
 * @file     utcpdlib.h
 * @version  V3.00
 * @brief    Power Delivery library header file
 * @note
 *
 * Copyright (c) 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __UTCPDLIB_H__
#define __UTCPDLIB_H__

#include "NuMicro.h"
#include "utcpd_config.h"

#include "Inc/atomic.h"
#include "Inc/usb_pd.h"
#include "Inc/tcpci.h"
#include "Inc/usb_common.h"
#include "Inc/ec_timer.h"
#include "Inc/usb_sm.h"
#include "Inc/usb_pd_flags.h"
#include "Inc/nct38xx.h"
#include "Inc/usb_pe_sm.h"
#include "Inc/usb_emsg.h"
#include "Inc/usb_pd_timer.h"
#include "Inc/usb_tc_sm.h"
#include "Inc/usbc_ppc.h"
#include "Inc/usb_pd_tcpm.h"
#include "Inc/usb_prl_sm.h"
#include "Inc/usb_pd_dpm.h"
//#include "Inc/usb_pd_tcpm.h"



#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup LIBRARY Library
  @{
*/

/** @addtogroup UTCPDLIB Power Delivery Library
  @{
*/

/** @addtogroup UTCPDLIB_EXPORTED_CONSTANTS Power Delivery Library Exported Constants
  @{
*/

#define CONFIG_USB_PID                           0x8260

typedef enum
{
    UTCPD_PD_ATTACHED = 0,                     /* Port partner attached or disattached */
    UTCPD_PD_CONTRACT = 1,                     /* PD contract established? */
    UTCPD_PD_SNK_VOLTAGE = 2,                  /* Contract voltage */
    UTCPD_PD_CABLE_MAX_POWER = 3,              /* Cable Max Voltage and Max Current : ((max_vol<<16) | max_curr) */
    UTCPD_PD_VCONN_DISCHARGE = 4,              /* To do VCONN Discharge */
    UTCPD_PD_PS_TRANSITION = 5,                /* To Disable VIN OVP/UVP, RDO IDX  */
    UTCPD_PD_PS_READY = 6,                     /* To Enable VIN OVP/UVP, 0 */
    UTCPD_PD_VIN_DISCHARGE_DONE = 7,           /* Inform Upper Layer VIN Discharge Done, 0 */
    UTCPD_PD_ACCEPT_REQUEST_PDO = 8,           /* Inform Upper Layer to Provide the Requested PDO */
    UTCPD_PD_VIN_DISCHAGE = 9, 	               /* To do VIN Discharge */
    UTCPD_PD_RECEIVE_HR = 10, 	               /* PD Receive Hard Reset */


    UTCPD_PD_VBUS_DISCHARGE_START = 0x20,
    UTCPD_PD_VBUS_DISCHARGE_STOP = 0x21,
} E_UTCPD_PD_EVENT;

/*@}*/ /* end of group UTCPDLIB_EXPORTED_CONSTANTS */



/** @addtogroup UTCPDLIB_EXPORTED_STRUCTS Power Delivery Library Exported Structs
  @{
*/
typedef void (*utcpd_pvFunPtr)(int port, E_UTCPD_PD_EVENT event, uint32_t op);   /* function pointer declaration */

/*@}*/ /* end of group UTCPDLIB_EXPORTED_STRUCTS */



/** @addtogroup SCLIB_EXPORTED_FUNCTIONS Smartcard Library Exported Functions
  @{
*/
void UTCPD_InstallCallback(int port, utcpd_pvFunPtr* pfn);

enum pd_cc_states UTCPD_TC_get_cc_state(int port);
enum pd_cc_states UTCPD_TC_get_polarity(int port);
void UTCPD_PE_get_src_caps(int port, int32_t* pu32SrcArray, int32_t* pi32SrcCnt);
void UTCPD_PE_get_snk_caps(int port, int32_t* pu32SnkArray, int32_t* pi32SnkCnt);

void pd_get_adjoutput_voltage_current(int port, uint32_t* pdopos, uint32_t* supply_voltage,	/* Unit: mV */
                                      uint32_t* supply_curr);			/* Unit: mA */

extern void UTCPD_TimerBaseInc(int port);        /* Notify portx to increase the time stamp */
extern void EADC_ConfigPins(void);
extern void EADC_Init(void);
extern void pd_task_reinit(int port);
extern bool pd_task_loop(int port);
extern void vconn_polarity_active_low();
extern void VBUS_Sink_Enable(int32_t port, bool bIsEnable);
extern void UART_Commandshell(int port);
extern void cpu_dump(uint32_t start_addr, uint32_t end_addr);
extern void VBUS_Source_Level(int port, char i8Level);
extern void VBUS_Sink_Enable(int32_t port, bool bIsEnable);

/*@}*/ /* end of group UTCPDLIB_EXPORTED_FUNCTIONS */



/*@}*/ /* end of group UTCPDLIB */

/*@}*/ /* end of group LIBRARY */

#ifdef __cplusplus
}
#endif

#endif /* __UTCPDLIB_H__ */