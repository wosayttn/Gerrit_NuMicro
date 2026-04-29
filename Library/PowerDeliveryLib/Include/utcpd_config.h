/**************************************************************************//**
 * @file     utcpd_config.h
 * @version  V1.00
 * @brief    Power Delivery config header file
 * @note
 *
 * Copyright (c) 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __UTCPD_CONFIG_H__
#define __UTCPD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define CONFIG_KEIL_V40													1

#if (CONFIG_KEIL_V40 == 1)
#include <stdio.h>
#include <string.h>
#endif

//#define printf1 ..)
/* VBUS OCP        */
/* SW Add {*/


/* Enable only one test configuration at most below*/
#define CONFIG_TEST_PDO28V_OVP									 0 /* For test 28V OVP, Normal is 0  */
#define CONFIG_TEST_PDO28V_UVP									 0 /* For test 28V UVP, Normal is 0  */
#define CONFIG_TEST_PDO36V_COVP									 0 /* For test Chip OVP, Normal is 0 */
#define CONFIG_TEST_NTCP 			       						 0 /* For test NTCP, Normal is 0     */
#define CONFIG_TEST_ITP       									 0 /* For test ITCP, Normal is 0     */

#if (OPT_FPGA == 1)
#define CONFIG_NPD48_VIN_OVP										 0 /* VIN OVP and UVP */
#define CONFIG_NPD48_VBUS_OCP										 0 /* VBUS OCP        */
#define CONFIG_NPD48_NTCP  									   	 0 /* NTCP            */
#define CONFIG_NPD48_CC_OVP									   	 0 /* CC OVP          */
#define CONFIG_NPD48_VCONN_OCP	    				   	 0 /* VCONN OCP       */
#define CONFIG_NPD48_CHIP_OVP                    0 /* Chip OVP        */
#define CONFIG_NPD48_ITP  									   	 0 /* ITP             */
#else
#define CONFIG_NPD48_VIN_OVP										 1 /* VIN OVP and UVP */
#define CONFIG_NPD48_VBUS_OCP										 1 /* VBUS OCP        */
#define CONFIG_NPD48_NTCP  									   	 0 /* NTCP            */
#define CONFIG_NPD48_CC_OVP									   	 1 /* CC OVP          */
#define CONFIG_NPD48_VCONN_OCP	    				   	 0 /* VCONN OCP       */
#define CONFIG_NPD48_CHIP_OVP                    1 /* Chip OVP        */
#define CONFIG_NPD48_ITP  									   	 1 /* ITP             */

//#define CONFIG_NPD48_VIN_OVP										 0 /* VIN OVP and UVP */
//#define CONFIG_NPD48_VBUS_OCP										 0 /* VBUS OCP        */
//#define CONFIG_NPD48_NTCP  									   	 0 /* NTCP            */
//#define CONFIG_NPD48_CC_OVP									   	 0 /* CC OVP          */
//#define CONFIG_NPD48_VCONN_OCP	    				   	 0 /* VCONN OCP       */
//#define CONFIG_NPD48_CHIP_OVP                    0 /* Chip OVP        */
//#define CONFIG_NPD48_ITP  									   	 0 /* ITP             */
#endif


#define CONFIG_NPD48_CURRENT_LIMIT               0 /* It just be used set Current Limit, Not relation to enable OPA1 and PGA or Not */
#define CONFIG_VBUS_COMPENSATION                 0

#define CONFIG_NPD48_DISABLE_OPA1								 1		/** Just for tring current feedback. 
																											  * Set 1 to disable OPA1, Set 0 to enable OPA1 
																												*/

#define CONFIG_HOSTCMD_ALIGNED
#define CHROMIUM_EC                              1
#define CONFIG_USB_PD_TRY_SRC                    1
#define CONFIG_USB_PE_SM                         1
#define CONFIG_USB_PRL_SM                        1
#define CONFIG_VBOOT_EFS2                        0
#define CONFIG_USB_PD_ALT_MODE_DFP               0
#define CONFIG_USB_PD_ALT_MODE_UFP_DP            0
#define CONFIG_USBC_OCP                          0
#define CONFIG_USBC_SS_MUX                       0
#define USB_PD_DEBUG_LABELS                      0
#define CONFIG_CHARGE_MANAGER                    0
#define CONFIG_USBC_PPC                          0
#define CONFIG_LOW_POWER_IDLE                    0
#define CONFIG_USB_PD_TCPC_ON_CHIP               0
#define CONFIG_USBC_VCONN                        1
#define CONFIG_USBC_VCONN_SWAP                   0
#define CONFIG_USBC_PPC_VCONN                    0
#define CONFIG_POWER_COMMON                      1
#define CONFIG_CHARGE_MANAGER                    0
#define CONFIG_BC12_DETECT_DATA_ROLE_TRIGGER     0
#define CONFIG_USB_PD_REV30                      1

#define CONFIG_BOARD_RESET_AFTER_POWER_ON        0
#define CONFIG_VBOOT_EFS2                        0
#define CONFIG_BATTERY                           0
#define TEST_BUILD                               0

#define CONFIG_USBC_RETIMER_FW_UPDATE            0
#define CONFIG_USB_PD_REQUIRE_AP_MODE_ENTRY      1
#define CONFIG_USB_PD_FRS                        1


#define CONFIG_USB_PD_TCPM_TCPCI                 1
#define CONFIG_USB_PD_LOGGING                    0

#define CONFIG_USB_PD_DISCHARGE_TCPC             1
#define CONFIG_USB_PD_DUAL_ROLE                  1


/* Following option will always not to be modified { */
#define CONFIG_USB_PD_VBUS_DETECT_TCPC           1
#define CONFIG_USB_PD_DECODE_SOP                 1
#define CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE      1
#define CONFIG_USB_PD_PPC                        1   /* Power path control */
#define CONFIG_USB_PD_TCPC_LOW_POWER             0
#define CONFIG_USB_PD_FRS_TCPC                   0
#define CONFIG_USB_PD_TCPC_VCONN                 0   /* For VCONN Power. tcpm.h - tcpm_set_vconn() */
/* } */

/* usb_pd_alt_mode_dfp.c */
#define CONFIG_CMD_MFALLOW                       0
#define CONFIG_USB_PD_TCPMV1                     0
#define CONFIG_USB_PD_TBT_COMPAT_MODE            0
#define CONFIG_USBC_PPC_SBU                      0
#define CONFIG_USB_PD_USB4                       0
#define CONFIG_USB_PD_USB4_DRD                   0
#define CONFIG_USB_PD_USB32_DRD                  0
#define CONFIG_USB_PD_PCIE_TUNNELING             0
#define CONFIG_USB_PD_ALT_MODE_DFP               0
#define CONFIG_MKBP_EVENT                        0
#define CONFIG_USB_PD_CUSTOM_PDO                 1
#define CONFIG_IO_EXPANDER_NCT38XX               0
#define CONFIG_USB_CTVPD                         0
#define CONFIG_USB_VPD                           0
#define CONFIG_USB_PD_EXTENDED_MESSAGES          1    /* Furture it will be enabled */
#define CONFIG_USB_PD_HOST_CMD                   0    /* We didn't support "PD Host commands" */
#define CONFIG_USB_PD_DPS                        0    /* We didn't support "dynamic PDO selection" */

#define CONFIG_USB_PD_PORT_MAX_COUNT             (1)  /* may 1 ~ 2 ~ 3 */
#define CONFIG_USB_PD_3A_PORTS                   0    /* To turn off the TCPMv2 3.0 A current allocation from the DPM, set
                                                       * CONFIG_USB_PD_3A_PORTS to 0. */
#define CONFIG_USB_PD_PULLUP                     TYPEC_RP_1A5   /* Default pull-up value on the USB-C ports when they are used as source. */

#define CONFIG_USB_DRP_ACC_TRYSRC                1
#define CONFIG_USB_TYPEC_SM                      1
#define CONFIG_USB_PRL_SM                        1
#define CONFIG_USB_PE_SM                         1
#define CONFIG_USB_PD_TCPC                       0


#define CONFIG_TEST_USB_PE_SM                    0
#define CONFIG_DUMP_REGISTER                     0

/* tcpci.c */
#define DEBUG_I2C_FAULT_LAST_WRITE_OP            0
#define DEBUG_FORCED_DISCHARGE                   0
#define DEBUG_AUTO_DISCHARGE_DISCONNECT          0
#define DEBUG_GET_CC                             0    /* Turn off it as release */
#define DEBUG_ROLE_CTRL_UPDATES                  0    /* Turn off it as release */
#define CONFIG_CMD_TCPC_DUMP                     0

#define CONFIG_SUPPORT_SRC_CAP_EXT			     		 1
#define CONFIG_SUPPORT_SNK_CAP_EXT               0
#if 0
#define CONFIG_SUPPORT_MANUFACTURER_EXT          0	/* Not to enable it, due to some fields of message not to be implemented */
#endif

#define CONFIG_MKBP_EVENT                        0

/* usb_dual_role.c */

#define CONFIG_USB_PD_PREFER_MV                  0
#define CONFIG_USB_PD_ONLY_FIXED_PDOS            1
#define PD_MAX_CURRENT_MA                        3000     /* defined in board level */
#define PD_MAX_POWER_MW                          65000    /* defined in board level */
#define PD_PREFER_LOW_VOLTAGE                    0        /* Can't found on config.h */
#define PD_PREFER_HIGH_VOLTAGE                   0        /* Can't found on config.h */
#define CONFIG_USB_PD_CHECK_MAX_REQUEST_ALLOWED  0

#define CONFIG_USB_PD_TCPC_RUNTIME_CONFIG

#define CONFIG_USB_PD_EPR                        1
#define CONFIG_NPD_TCPC                          1        /* define 0 if M2L31 Series */
/* define 1 if NPD48 Series */

#define CONFIG_DEBUG_PIN                         0        /* define 0 if didn't use it */

#if (OPT_SNK_ONLY == 1)
#undef CONFIG_USBC_VCONN
#define CONFIG_USBC_VCONN                        0
#undef CONFIG_USB_PD_TRY_SRC
#define CONFIG_USB_PD_TRY_SRC                    0
#undef CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE
#define CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE      0

#define CONFIG_COMMAND_SHELL                     1
#define CONFIG_SUPPORT_SNK                       1
#define CONFIG_SUPPORT_SRC                       0
#define CONFIG_SUPPORT_PR_SWAP                   0
#define CONFIG_SUPPORT_DR_SWAP                   0
#define CONFIG_VDM_IDENTITY_REQUEST              0
#define CONFIG_VDM_SVIDS_REQUEST                 0
#define CONFIG_VDM_MODES_REQUEST                 0
#define CONFIG_VDM_REQUEST_DPM                   0
#endif

#if (OPT_SRC_ONLY==1)
#undef CONFIG_USBC_VCONN
#define CONFIG_USBC_VCONN                        0

#undef CONFIG_USB_PD_TRY_SRC
#define CONFIG_USB_PD_TRY_SRC                    0
#undef CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE
#define CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE      0
#define CONFIG_COMMAND_SHELL                     0

#define CONFIG_SUPPORT_SNK                       0
#define CONFIG_SUPPORT_SRC                       1
#define CONFIG_SUPPORT_PR_SWAP                   0
#define CONFIG_SUPPORT_DR_SWAP                   0

#define CONFIG_CAPTIVE_CABLE                     1  //(define it to 1 if captive cable, define it to 0 if non-captive cable)

#if (CONFIG_CAPTIVE_CABLE ==1)	//ACBEL Captive Cable
#define CONFIG_VDM_IDENTITY_REQUEST              0
#define CONFIG_VDM_SVIDS_REQUEST                 0
#define CONFIG_VDM_MODES_REQUEST                 0
#define CONFIG_VDM_REQUEST_DPM                   0
#define CONFIG_CAPTIVE_CABLE_MAX_VOL             (50000)
#define CONFIG_CAPTIVE_CABLE_MAX_CURR            (5000)
#else   //Normal USBC Cable
#define CONFIG_VDM_IDENTITY_REQUEST              1
#define CONFIG_VDM_SVIDS_REQUEST                 1
#define CONFIG_VDM_MODES_REQUEST           		 1
#define CONFIG_VDM_REQUEST_DPM                   1
#endif


#endif


#if (OPT_DRP == 1)
#undef CONFIG_USB_PD_TRY_SRC
#define CONFIG_USB_PD_TRY_SRC                    1
#undef CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE
#define CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE      1
#define CONFIG_COMMAND_SHELL                     1

#define CONFIG_SUPPORT_SNK                       1
#define CONFIG_SUPPORT_SRC                       1
#define CONFIG_SUPPORT_PR_SWAP                   1
#define CONFIG_SUPPORT_DR_SWAP                   1
#define CONFIG_VDM_IDENTITY_REQUEST              1
#define CONFIG_VDM_SVIDS_REQUEST                 1
#define CONFIG_VDM_MODES_REQUEST                 1
#define CONFIG_VDM_REQUEST_DPM                   1
#endif



#ifdef __cplusplus
}
#endif

#endif /* __UTCPD_CONFIG_H__ */