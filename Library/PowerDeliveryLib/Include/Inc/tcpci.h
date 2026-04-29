/* Copyright 2015 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* USB Power delivery port management */

#ifndef __CROS_EC_USB_PD_TCPM_TCPCI_H
#define __CROS_EC_USB_PD_TCPM_TCPCI_H

#ifdef SW
#include "config.h"
#endif

#include "ec_commands.h"

#ifdef SW
#include "tcpm/tcpm.h"
#else
#include "tcpm.h"
#endif

#include "usb_mux.h"
#include "usb_pd_tcpm.h"

#include "common.h" /* SW ADD */

#define TCPC_REG_VENDOR_ID         0x0
#define TCPC_REG_PRODUCT_ID        0x2
#define TCPC_REG_BCD_DEV           0x4
#define TCPC_REG_TC_REV            0x6
#define TCPC_REG_PD_REV            0x8
#define TCPC_REG_PD_INT_REV        0xa

#define TCPC_REG_PD_INT_REV_REV_MASK  0xff00
#define TCPC_REG_PD_INT_REV_REV_1_0   0x10
#define TCPC_REG_PD_INT_REV_REV_2_0   0x20
#define TCPC_REG_PD_INT_REV_VER_MASK  0x00ff
#define TCPC_REG_PD_INT_REV_VER_1_0   0x10
#define TCPC_REG_PD_INT_REV_VER_1_1   0x11
#define TCPC_REG_PD_INT_REV_REV(reg) \
		((reg & TCOC_REG_PD_INT_REV_REV_MASK) >> 8)
#define TCPC_REG_PD_INT_REV_VER(reg) \
		(reg & TCOC_REG_PD_INT_REV_VER_MASK)

#define TCPC_REG_ALERT              0x10
#define TCPC_REG_ALERT_NONE         0x0000
#define TCPC_REG_ALERT_MASK_ALL     0xffff
#define TCPC_REG_ALERT_VENDOR_DEF   BIT(15)
#define TCPC_REG_ALERT_ALERT_EXT    BIT(14)
#define TCPC_REG_ALERT_EXT_STATUS   BIT(13)
#define TCPC_REG_ALERT_RX_BEGINNING BIT(12)
#define TCPC_REG_ALERT_VBUS_DISCNCT BIT(11)
#define TCPC_REG_ALERT_RX_BUF_OVF   BIT(10)
#define TCPC_REG_ALERT_FAULT        BIT(9)
#define TCPC_REG_ALERT_V_ALARM_LO   BIT(8)
#define TCPC_REG_ALERT_V_ALARM_HI   BIT(7)
#define TCPC_REG_ALERT_TX_SUCCESS   BIT(6)
#define TCPC_REG_ALERT_TX_DISCARDED BIT(5)
#define TCPC_REG_ALERT_TX_FAILED    BIT(4)
#define TCPC_REG_ALERT_RX_HARD_RST  BIT(3)
#define TCPC_REG_ALERT_RX_STATUS    BIT(2)
#define TCPC_REG_ALERT_POWER_STATUS BIT(1)
#define TCPC_REG_ALERT_CC_STATUS    BIT(0)
#define TCPC_REG_ALERT_TX_COMPLETE  (TCPC_REG_ALERT_TX_SUCCESS | \
				      TCPC_REG_ALERT_TX_DISCARDED | \
				      TCPC_REG_ALERT_TX_FAILED)

#define TCPC_REG_ALERT_MASK         0x12
#define TCPC_REG_ALERT_MASK_VENDOR_DEF   BIT(15)
    #define TCPC_REG_ALERT_MASK_SINK_DISCONNECT_DET BIT(11)
		#define TCPC_REG_ALERT_MASK_RXBUF_OV BIT(10)
		#define TCPC_REG_ALERT_MASK_FAULT    BIT(9)
#define TCPC_REG_ALERT_MASK_V_ALARM_LO   BIT(8)
#define TCPC_REG_ALERT_MASK_V_ALARM_HI   BIT(7)
#define TCPC_REG_ALERT_MASK_TX_SUCCESS   BIT(6)
#define TCPC_REG_ALERT_MASK_TX_DISCARDED BIT(5)
#define TCPC_REG_ALERT_MASK_TX_FAILED    BIT(4)
#define TCPC_REG_ALERT_MASK_RX_HARD_RST  BIT(3)
#define TCPC_REG_ALERT_MASK_RX           BIT(2)
#define TCPC_REG_ALERT_MASK_POWER        BIT(1)
#define TCPC_REG_ALERT_MASK_CC           BIT(0)


#define TCPC_REG_POWER_STATUS_MASK  0x14
		#define TCPC_REG_POWER_STATUS_MASK_SRC_NONDEFAULT_VBUS	BIT(5)	
		#define TCPC_REG_POWER_STATUS_MASK_SRC_VBUS		BIT(4)
		#define TCPC_REG_POWER_STATUS_MASK_VBUS_DET		BIT(3)
		#define TCPC_REG_POWER_STATUS_MASK_VBUS_PS				BIT(2)
		#define TCPC_REG_POWER_STATUS_MASK_VCONN_PS				BIT(1)
		#define TCPC_REG_POWER_STATUS_MASK_SINK_VBUS_STS	BIT(0) 
		
#define TCPC_REG_FAULT_STATUS_MASK  0x15
		//#define TCPC_REG_FAULT_STATUS_MASK_ALL_REGS_RESET            BIT(7)
		#define TCPC_REG_FAULT_STATUS_MASK_FORCE_OFF_VBUS            BIT(6)
		#define TCPC_REG_FAULT_STATUS_MASK_AUTO_DISCHARGE_FAIL       BIT(5)
		#define TCPC_REG_FAULT_STATUS_MASK_FORCE_DISCHARGE_FAIL      BIT(4)
		#define TCPC_REG_FAULT_STATUS_MASK_VBUS_OVER_CURRENT         BIT(3)
		#define TCPC_REG_FAULT_STATUS_MASK_VBUS_OVER_VOLTAGE         BIT(2)
		#define TCPC_REG_FAULT_STATUS_MASK_VCONN_OVER_CURRENT        BIT(1)
		//#define TCPC_REG_FAULT_STATUS_MASK_I2C_INTERFACE_ERR         BIT(0)
			
#define TCPC_REG_EXT_STATUS_MASK    0x16
#define TCPC_REG_ALERT_EXTENDED_MASK 0x17

#define TCPC_REG_CONFIG_STD_OUTPUT   0x18
#define TCPC_REG_CONFIG_STD_OUTPUT_DBG_ACC_CONN_N    BIT(6)
#define TCPC_REG_CONFIG_STD_OUTPUT_AUDIO_CONN_N      BIT(5)
#define TCPC_REG_CONFIG_STD_OUTPUT_MUX_MASK          (3 << 2)
#define TCPC_REG_CONFIG_STD_OUTPUT_MUX_NONE          (0 << 2)
#define TCPC_REG_CONFIG_STD_OUTPUT_MUX_USB           BIT(2)
#define TCPC_REG_CONFIG_STD_OUTPUT_MUX_DP            (2 << 2)
#define TCPC_REG_CONFIG_STD_OUTPUT_CONNECTOR_FLIPPED BIT(0)

#define TCPC_REG_TCPC_CTRL         0x19
		#define TCPC_REG_TCPC_CTRL_PLUG_ORIENTATION		BIT(0)
		#define TCPC_REG_TCPC_CTRL_BIST_TEST_MODE		BIT(1)	
#define TCPC_REG_TCPC_CTRL_SET(polarity) (polarity)
#define TCPC_REG_TCPC_CTRL_POLARITY(reg) ((reg) & 0x1)
/*
 * In TCPCI Rev 2.0, this bit must be set this to generate CC status alerts when
 * a connection is found.
 */
#define TCPC_REG_TCPC_CTRL_EN_LOOK4CONNECTION_ALERT  BIT(6)
#define TCPC_REG_TCPC_CTRL_DEBUG_ACC_CONTROL         BIT(4)
#define TCPC_REG_TCPC_CTRL_BIST_TEST_MODE            BIT(1)

#if 0
#define TCPC_REG_PINPL	         0x28  //TCPD added
#if 0
	#define TCPC_REG_PINPL_SRCEN				BIT(0)
	#define TCPC_REG_PINPL_SNKEN				BIT(1)
	#define TCPC_REG_PINPL_VBDCHG				BIT(2)
	#define TCPC_REG_PINPL_VCEN					BIT(8)	
	#define TCPC_REG_PINPL_VCDCHG				BIT(9)

  #define TCPC_REG_PINPL_TXFRSPL      BIT(3)    //M2L31
	#define TCPC_REG_PINPL_VCOCPL	      BIT(4)    //M2L31	
  #define TCPC_REG_PINPL_VBOCPL	      BIT(5)    //M2L31	 
  #define TCPC_REG_PINPL_FOFFVBPL     BIT(10)   //M2L31	
#else
  #define TCPC_REG_PINPL_SRCEN				BIT(0)
	#define TCPC_REG_PINPL_SNKEN				BIT(1)
	#define TCPC_REG_PINPL_VBDCHG				BIT(2)
	#define TCPC_REG_PINPL_VCEN					BIT(8)	
	#define TCPC_REG_PINPL_VCDCHG				BIT(9)

  #define TCPC_REG_PINPL_TXFRSPL      BIT(3)    //M2L31
	#define TCPC_REG_PINPL_VCOCPL	      BIT(10)    //M2L31	
  #define TCPC_REG_PINPL_VBOCPL	      BIT(5)    //M2L31	 
  #define TCPC_REG_PINPL_FOFFVBPL     BIT(4)   //M2L31	
#endif
#endif	

#define TCPC_REG_ROLE_CTRL         0x1a
#define TCPC_REG_ROLE_CTRL_DRP_MASK                    BIT(6)
#define TCPC_REG_ROLE_CTRL_RP_MASK                     (BIT(5)|BIT(4))
#define TCPC_REG_ROLE_CTRL_CC2_MASK                    (BIT(3)|BIT(2))
#define TCPC_REG_ROLE_CTRL_CC1_MASK                    (BIT(1)|BIT(0))
#define TCPC_REG_ROLE_CTRL_SET(drp, rp, cc1, cc2) \
		((((drp) << 6) & TCPC_REG_ROLE_CTRL_DRP_MASK) | \
		(((rp) << 4) & TCPC_REG_ROLE_CTRL_RP_MASK) | \
		(((cc2) << 2) & TCPC_REG_ROLE_CTRL_CC2_MASK) | \
		((cc1) & TCPC_REG_ROLE_CTRL_CC1_MASK))
#define TCPC_REG_ROLE_CTRL_DRP(reg) \
		(((reg) & TCPC_REG_ROLE_CTRL_DRP_MASK) >> 6)
#define TCPC_REG_ROLE_CTRL_RP(reg) \
		(((reg) & TCPC_REG_ROLE_CTRL_RP_MASK) >> 4)
#define TCPC_REG_ROLE_CTRL_CC2(reg) \
		(((reg) & TCPC_REG_ROLE_CTRL_CC2_MASK) >> 2)
#define TCPC_REG_ROLE_CTRL_CC1(reg) \
		((reg) & TCPC_REG_ROLE_CTRL_CC1_MASK)

#define TCPC_REG_FAULT_CTRL        0x1b
			#define TCPC_REG_FAULT_CTRL_VBUS_FORCE_OFF_DIS         BIT(4)
			#define TCPC_REG_FAULT_CTRL_VBUS_DISCHARGE_FAULT_DET_TIMER_DIS         BIT(3)
			#define TCPC_REG_FAULT_CTRL_VBUS_OCP_FAULT_DIS         BIT(2)
#define TCPC_REG_FAULT_CTRL_VBUS_OVP_FAULT_DIS         BIT(1)
#define TCPC_REG_FAULT_CTRL_VCONN_OCP_FAULT_DIS         BIT(0)

#define TCPC_REG_POWER_CTRL        0x1c
#define TCPC_REG_POWER_CTRL_FRS_ENABLE                 BIT(7)
#define TCPC_REG_POWER_CTRL_VBUS_VOL_MONITOR_DIS       BIT(6)
#define TCPC_REG_POWER_CTRL_VOLT_ALARM_DIS             BIT(5)
#define TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT  BIT(4)
#define TCPC_REG_POWER_CTRL_BLEED_DISCHARGE  					 BIT(3)
#define TCPC_REG_POWER_CTRL_FORCE_DISCHARGE            BIT(2)
			#define TCPC_REG_POWER_CTRL_ENABLE_VCONN         				BIT(0)//SW ADD
#define TCPC_REG_POWER_CTRL_SET(vconn) (vconn)
#define TCPC_REG_POWER_CTRL_VCONN(reg)    ((reg) & 0x1)

#define TCPC_REG_CC_STATUS         0x1d
#define TCPC_REG_CC_STATUS_LOOK4CONNECTION_MASK        BIT(5)
#define TCPC_REG_CC_STATUS_CONNECT_RESULT_MASK         BIT(4)
#define TCPC_REG_CC_STATUS_CC2_STATE_MASK              (BIT(3)|BIT(2))
#define TCPC_REG_CC_STATUS_CC1_STATE_MASK              (BIT(1)|BIT(0))
#define TCPC_REG_CC_STATUS_SET(term, cc1, cc2) \
		((term) << 4 | ((cc2) & 0x3) << 2 | ((cc1) & 0x3))
#define TCPC_REG_CC_STATUS_LOOK4CONNECTION(reg) \
		((reg & TCPC_REG_CC_STATUS_LOOK4CONNECTION_MASK) >> 5)
#define TCPC_REG_CC_STATUS_TERM(reg) \
		(((reg) & TCPC_REG_CC_STATUS_CONNECT_RESULT_MASK) >> 4)
#define TCPC_REG_CC_STATUS_CC2(reg) \
		(((reg) & TCPC_REG_CC_STATUS_CC2_STATE_MASK) >> 2)
#define TCPC_REG_CC_STATUS_CC1(reg) \
		((reg) & TCPC_REG_CC_STATUS_CC1_STATE_MASK)

#define TCPC_REG_POWER_STATUS      0x1e
#define TCPC_REG_POWER_STATUS_MASK_ALL  0xff
#define TCPC_REG_POWER_STATUS_DEBUG_ACC_CON BIT(7)
#define TCPC_REG_POWER_STATUS_UNINIT    BIT(6)
		 #define TCPC_REG_POWER_STATUS_SOURCING_HIGH_VBUS BIT(5)//SW Add
#define TCPC_REG_POWER_STATUS_SOURCING_VBUS BIT(4)
#define TCPC_REG_POWER_STATUS_VBUS_DET  BIT(3)
#define TCPC_REG_POWER_STATUS_VBUS_PRES BIT(2)
		 #define TCPC_REG_POWER_STATUS_VCONN_PRES BIT(1)				//SW Add
#define TCPC_REG_POWER_STATUS_SINKING_VBUS BIT(0)

#define TCPC_REG_FAULT_STATUS      0x1f
		#define TCPC_REG_FAULT_STATUS_ALL_REGS_RESET            BIT(7)
		#define TCPC_REG_FAULT_STATUS_DET_INVALID_RESET         BIT(7)
		#define TCPC_REG_FAULT_STATUS_FORCE_OFF_VBUS            BIT(6)
		#define TCPC_REG_FAULT_STATUS_AUTO_DISCHARGE_FAIL       BIT(5)
		#define TCPC_REG_FAULT_STATUS_FORCE_DISCHARGE_FAIL      BIT(4)
		#define TCPC_REG_FAULT_STATUS_VBUS_OVER_CURRENT         BIT(3)
		#define TCPC_REG_FAULT_STATUS_VBUS_OVER_VOLTAGE         BIT(2)
		#define TCPC_REG_FAULT_STATUS_VCONN_OVER_CURRENT        BIT(1)
		#define TCPC_REG_FAULT_STATUS_I2C_INTERFACE_ERR         BIT(0)

#define TCPC_REG_EXT_STATUS        0x20 //TCPD has not the register
#define TCPC_REG_EXT_STATUS_SAFE0V   BIT(0)

#define TCPC_REG_ALERT_EXT         0x20 //0x21 //TCPD has not the register
#define TCPC_REG_ALERT_EXT_TIMER_EXPIRED        BIT(2)
#define TCPC_REG_ALERT_EXT_SRC_FRS              BIT(1)
#define TCPC_REG_ALERT_EXT_SNK_FRS              BIT(0)

#define TCPC_REG_COMMAND           0x23
#define TCPC_REG_COMMAND_ENABLE_VBUS_DETECT      0x33

#define TCPC_REG_COMMAND_SNK_CTRL_LOW            0x44
				#define TCPC_REG_COMMAND_DISABLE_SNK_VBUS    TCPC_REG_COMMAND_SNK_CTRL_LOW 
#define TCPC_REG_COMMAND_SNK_CTRL_HIGH           0x55
				#define TCPC_REG_COMMAND_ENABLE_SNK_VBUS    TCPC_REG_COMMAND_SNK_CTRL_HIGH 
#define TCPC_REG_COMMAND_SRC_CTRL_LOW            0x66
				#define TCPC_REG_COMMAND_DISABLE_SRC_VBUS    TCPC_REG_COMMAND_SRC_CTRL_LOW 
#define TCPC_REG_COMMAND_SRC_CTRL_HIGH           0x77
				#define TCPC_REG_COMMAND_ENABLE_SRC_VBUS    TCPC_REG_COMMAND_SRC_CTRL_HIGH 
				
#define TCPC_REG_COMMAND_LOOK4CONNECTION         0x99
				#define TCPC_REG_COMMAND_SOURCE_VBUS_NONDEFAULT	0x88
				#define TCPC_REG_COMMAND_SOURCE_VBUS_DEFAULT		0x77
				#define TCPC_REG_COMMAND_ENABLE_VBUS_DETECT			0x33   //Control Phy's signal - "VBUS_DET" connect to VBUS present. 
				#define TCPC_REG_COMMAND_DISABLE_VBUS_DETECT		0x22	 //Control Phy's signal - "VBUS_DET" disconnect to VBUS present. 
																															 /* It is different POWER_CONTROL[VBUS Voltage Monitor] ==> 
																																	POWER_CONTROL[VBUS Voltage Monitor] is used to report ADC measure to VBVOL and VCVOL  registers */
				
#define TCPC_REG_COMMAND_RESET_TRANSMIT_BUF      0xDD
#define TCPC_REG_COMMAND_RESET_RECEIVE_BUF       0xEE
#define TCPC_REG_COMMAND_I2CIDLE                 0xFF

#define TCPC_REG_DEV_CAP_1         0x24
#define TCPC_REG_DEV_CAP_1_VBUS_NONDEFAULT_TARGET	BIT(15)
#define TCPC_REG_DEV_CAP_1_VBUS_OCP_REPORTING		BIT(14)
#define TCPC_REG_DEV_CAP_1_VBUS_OVP_REPORTING		BIT(13)
#define TCPC_REG_DEV_CAP_1_BLEED_DISCHARGE		BIT(12)
#define TCPC_REG_DEV_CAP_1_FORCE_DISCHARGE		BIT(11)
#define TCPC_REG_DEV_CAP_1_VBUS_MEASURE_ALARM_CAPABLE	BIT(10)
#define TCPC_REG_DEV_CAP_1_SRC_RESISTOR_MASK		(BIT(8)|BIT(9))
#define TCPC_REG_DEV_CAP_1_SRC_RESISTOR_RP_DEF		(0 << 8)
#define TCPC_REG_DEV_CAP_1_SRC_RESISTOR_RP_1P5_DEF	(1 << 8)
#define TCPC_REG_DEV_CAP_1_SRC_RESISTOR_RP_3P0_1P5_DEF	(2 << 8)
#define TCPC_REG_DEV_CAP_1_PWRROLE_MASK			(BIT(5)|BIT(6)|BIT(7))
#define TCPC_REG_DEV_CAP_1_PWRROLE_SRC_OR_SNK		(0 << 5)
#define TCPC_REG_DEV_CAP_1_PWRROLE_SRC			(1 << 5)
#define TCPC_REG_DEV_CAP_1_PWRROLE_SNK			(2 << 5)
#define TCPC_REG_DEV_CAP_1_PWRROLE_SNK_ACC		(3 << 5)
#define TCPC_REG_DEV_CAP_1_PWRROLE_DRP			(4 << 5)
#define TCPC_REG_DEV_CAP_1_PWRROLE_SRC_SNK_DRP_ADPT_CBL	(5 << 5)
#define TCPC_REG_DEV_CAP_1_PWRROLE_SRC_SNK_DRP		(6 << 5)
#define TCPC_REG_DEV_CAP_1_ALL_SOP_STAR_MSGS_SUPPORTED	BIT(4)
#define TCPC_REG_DEV_CAP_1_SOURCE_VCONN			BIT(3)
#define TCPC_REG_DEV_CAP_1_SINK_VBUS			BIT(2)
#define TCPC_REG_DEV_CAP_1_SOURCE_NONDEFAULT_VBUS	BIT(1)
#define TCPC_REG_DEV_CAP_1_SOURCE_VBUS			BIT(0)

#define TCPC_REG_DEV_CAP_2         0x26
#define TCPC_REG_DEV_CAP_2_LONG_MSG		BIT(12)
#define TCPC_REG_DEV_CAP_2_SNK_FR_SWAP		BIT(9)

//#define TCPC_REG_STD_INPUT_CAP     0x0 //TCPD has not the register
#define TCPC_REG_STD_INPUT_CAP_SRC_FR_SWAP	(BIT(4)|BIT(3))
#define TCPC_REG_STD_INPUT_CAP_EXT_OVR_V_F	BIT(2)
#define TCPC_REG_STD_INPUT_CAP_EXT_OVR_C_F	BIT(1)
#define TCPC_REG_STD_INPUT_CAP_FORCE_OFF_VBUS	BIT(0)

//#define TCPC_REG_STD_OUTPUT_CAP    0x0 //TCPD has not the register  //0x29
#define TCPC_REG_STD_OUTPUT_CAP_SNK_DISC_DET		BIT(7)
#define TCPC_REG_STD_OUTPUT_CAP_DBG_ACCESSORY		BIT(6)
#define TCPC_REG_STD_OUTPUT_CAP_VBUS_PRESENT_MON	BIT(5)
#define TCPC_REG_STD_OUTPUT_CAP_AUDIO_ACCESSORY		BIT(4)
#define TCPC_REG_STD_OUTPUT_CAP_ACTIVE_CABLE		BIT(3)
#define TCPC_REG_STD_OUTPUT_CAP_MUX_CONF_CTRL		BIT(2)
#define TCPC_REG_STD_OUTPUT_CAP_CONN_PRESENT		BIT(1)
#define TCPC_REG_STD_OUTPUT_CAP_CONN_ORIENTATION	BIT(0)

//#define TCPC_REG_CONFIG_EXT_1      0x0 //TCPD has not the register  //0x2A
#define TCPC_REG_CONFIG_EXT_1_FR_SWAP_SNK_DIR	BIT(1)

#define TCPC_REG_GENERIC_TIMER     0x2C //TCPD has not the register  

#define TCPC_REG_MSG_HDR_INFO      0x2e

/* Return GoodCRC PD revision is always 2.0 */
/* It is different from Message revision    */
#define TCPC_REG_MSG_HDR_INFO_SET(drole, prole) \
		((drole) << 3 | (PD_REV20 << 1) | (prole))

#define TCPC_REG_MSG_HDR_INFO_DROLE(reg) (((reg) & 0x8) >> 3)
#define TCPC_REG_MSG_HDR_INFO_PROLE(reg) ((reg) & 0x1)

#define TCPC_REG_RX_DETECT         0x2f
#define TCPC_REG_RX_DETECT_SOP_HRST_MASK 0x21
#define TCPC_REG_RX_DETECT_SOP_SOPP_SOPPP_HRST_MASK 0x27
#define TCPD_REG_RX_DETECT_ALL_MASK 0x7F	/* SW add */

/* TCPCI Rev 1.0 receive registers */
#define TCPC_REG_RX_BYTE_CNT       0x30
#define TCPC_REG_RX_BUF_FRAME_TYPE 0x31
#define TCPC_REG_RX_HDR            0x32
#define TCPC_REG_RX_DATA           0x34 /* through 0x4f */

/*
 * In TCPCI Rev 2.0, the RECEIVE_BUFFER is comprised of three sets of registers:
 * READABLE_BYTE_COUNT, RX_BUF_FRAME_TYPE and RX_BUF_BYTE_x. These registers can
 * only be accessed by reading at a common register address 30h.
 */
#define TCPC_REG_RX_BUFFER         0x30 //TCPD didn't support receive data by TCPCi 2.0. It is invalid.


#define TCPC_REG_TRANSMIT          0x50
#define TCPC_REG_TRANSMIT_SET_WITH_RETRY(retries, type) \
		((retries) << 4 | (type))
#define TCPC_REG_TRANSMIT_SET_WITHOUT_RETRY(type) (type)
#define TCPC_REG_TRANSMIT_RETRY(reg) (((reg) & 0x30) >> 4)
#define TCPC_REG_TRANSMIT_TYPE(reg)  ((reg) & 0x7)

/* TCPCI Rev 1.0 transmit registers */
#define TCPC_REG_TX_BYTE_CNT       0x51
#define TCPC_REG_TX_HDR            0x52
#define TCPC_REG_TX_DATA           0x54 /* through 0x6f */

/*
 * In TCPCI Rev 2.0, the TRANSMIT_BUFFER holds the I2C_WRITE_BYTE_COUNT and the
 * portion of the SOP* USB PD message payload (including the header and/or the
 * data bytes) most recently written by the TCPM in TX_BUF_BYTE_x. TX_BUF_BYTE_x
 * is ?œhidden??and can only be accessed by writing to register address 51h
 */
#define TCPC_REG_TX_BUFFER                   0x51		/* TCPCi Rev 2.0 */

#define TCPC_REG_VBUS_VOLTAGE                0x70

#define TCPC_REG_VBUS_SINK_DISCONNECT_THRESH 0x72
#define TCPC_REG_VBUS_SINK_DISCONNECT_THRESH_DEFAULT 0x008C /* 3.5 V */

#define TCPC_REG_VBUS_STOP_DISCHARGE_THRESH  0x74
#define TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG   0x76
#define TCPC_REG_VBUS_VOLTAGE_ALARM_LO_CFG   0x78

#define TCPC_REG_VBUS_NONDEFAULT_TARGET      0x7a

#define TCPC_REG_CCTRIM				0x8B /* BMC EyeDiagram */
#define TCPC_REG_PREDET1            0xE5 /* Noise rejection */
#define TCPC_REG_PREDET2            0xE6 /* Noise rejection */
#define TCPC_REG_NIRR1              0xE9 /* Noise rejection */
#define TCPC_REG_NIRR2              0xEA /* Noise rejection */
#define TCPC_REG_SLICECR            0xF9 /* Noise rejection */
#define TCPC_REG_SLICECR2           0xFA /* Noise rejection */

#define TCPD_VNDIS					0xD4
		#define RXFRSIS					BIT(0)
		#define TXFRSIS				  BIT(1)
		#define CRCERRIS			  BIT(3)
		#define VCDCHGIS		    BIT(5)
#define TCPD_VNDIE					0xD8
		#define RXFRSIE					BIT(0)
		#define TXFRSIE					BIT(1)
		#define CRCERRIE				BIT(3)
		#define VCDGIE		     	BIT(5)

#define TCPD_MUXSEL				0xDC
		#define CC2FRSS         BIT(29) 
		#define CC2VCENS				BIT(28)
		#define CC1FRSS         BIT(25) 
		#define CC1VCENS				BIT(24)
		#define ADCSELVC        (BIT(21)|BIT(20)|BIT(19)|BIT(18)|BIT(17))
		#define ADCSELVB        (BIT(16)|BIT(15)|BIT(14)|BIT(13)|BIT(12))
		#define FBVBS						(BIT(10)|BIT(9)|BIT(8))
		#define VCOCS						(BIT(6)|BIT(5)|BIT(4))
		#define VBOCS						(BIT(2)|BIT(1)|BIT(0))	
		
#define TCPD_VCDGCTL				0xE0
		#define VCDGEN					BIT(1)
		#define VCDGDTEN				BIT(0)
		
#define TCPD_PHTX_SLW				0xE4
#define TCPD_ADG_TM					0xE8
#define TCPD_VSAFE0V				0xEC
#define TCPD_VSAFE5V				0xF0
#define TCPD_RATIO					0xF4
#define TCPD_IN_F_TM				0xF8
#define TCPD_VBOVTH					0xFC
#define TCPD_VND_INIT				0x100
#define TCPD_BMC_TXBP				0x104
#define TCPD_BMC_TXDT				0x108
#define TCPD_VCPSVOL				0x10C
#define TCPD_VCUV					  0x110
#define TCPD_FRS_V					0x114
#define TCPD_BMC_SL_C				0x118
#define TCPD_PHY_PWR				0x11C
//#define TCPD_FRS_RX_C				0x120
#define TCPD_FRSRXCTL				0x120
		#define FRSTX						BIT(0)
		#define FRSDVVB					BIT(2)
		#define FRSRXEN					BIT(3)
#define TCPD_VCVOL          0x124


#define TCPD_SLI_CTL				0x200
#define TCPD_CC_DB_TM				0x204
#define TCPD_FILTM					0x208
#define TCPD_RSTVNT					0x20C
#define TCPD_NOGCRC				  0x210
#define TCPD_DS_STS					0x214
#define TCPD_PHY_IDL				0x218
#define TCPD_CC_STATE				0x21C
#define TCPD_CC_STS					0x220
#define TCPD_BM_14_TM				0x224
#define TCPD_BM_34_TM				0x228
#define TCPD_MS_DUT_TM			0x22C
#define TCPD_CC_UP_DN				0x230
#define TCPD_BM_STATE				0x234
#define TCPD_CC_PHY_CMP			0x238
#define TCPD_SK_DIS_CD			0x23C
#define TCPD_ADG_STATE			0x240
#define TCPD_VB_RD_TO				0x244
#define TCPD_ITEST					0x248
#define TCPD_TM				 			0x24C

#define TCPD_STATE_MN				0x250
		#define  TCPD_STATE_MN_TC_ACTIVE 			0x1
		#define  TCPD_STATE_MN_TX_PKG_Pos		 	(1)
		#define  TCPD_STATE_MN_TX_PKG_Msk		 	(0x7 << TCPD_STATE_MN_TX_PKG_Pos)
		#define  TCPD_STATE_MN_RX_PKG_Pos			(4)
		#define  TCPD_STATE_MN_RX_PKG_Msk		 	(0x7 << TCPD_STATE_MN_RX_PKG_Pos)
		#define  TCPD_STATE_MN_RX_IN_BUF_Pos	(7)
		#define  TCPD_STATE_MN_RX_IN_BUF_Msk  (1<<TCPD_STATE_MN_RX_IN_BUF_Pos)

		#define  TCPD_STATE_MN_RX_PKG_HARD_RESET	(2<<TCPD_STATE_MN_RX_PKG_Pos)
		#define  TCPD_STATE_MN_RX_PKG_CABLE_RESET	(3<<TCPD_STATE_MN_RX_PKG_Pos)

#define TCPD_ADC_STRP				0x254
#define TCPD_VB_ADC_AEN			0x258
#define TCPD_ADC_AEN_TM			0x25C
#define TCPD_SRST						0x260
#define TCPD_VC_OC_V				0x264

#define TCPD_CLKDIV					0x300
		#define  TCPD_CLKDIV_WKEN			BIT(4)
		#define  TCPD_CLKDIV_DIV  		(BIT(2)|BIT(1)|BIT(0))	
    #define TCPD_READY      BIT(0)
		
#define TCPD_TEST1					0x304
#define TCPD_TEST2					0x308
#define TCPD_TEST3					0x30C
#define TCPD_TEST4					0x310
extern const struct tcpm_drv tcpci_tcpm_drv;
extern const struct usb_mux_driver tcpci_tcpm_usb_mux_driver;

void tcpci_set_cached_rp(int port, int rp);
int tcpci_get_cached_rp(int port);
void tcpci_set_cached_pull(int port, enum tcpc_cc_pull pull);
enum tcpc_cc_pull tcpci_get_cached_pull(int port);

void tcpci_tcpc_alert(int port);
int tcpci_tcpm_init(int port);
int tcpci_tcpm_get_cc(int port, enum tcpc_cc_voltage_status *cc1,
	enum tcpc_cc_voltage_status *cc2);
bool tcpci_tcpm_check_vbus_level(int port, enum vbus_level level);
int tcpci_tcpm_select_rp_value(int port, int rp);
int tcpci_tcpm_set_cc(int port, int pull);
int tcpci_tcpm_set_polarity(int port, enum tcpc_cc_polarity polarity);
int tcpci_tcpm_sop_prime_enable(int port, bool enable);
int tcpci_tcpm_set_vconn(int port, int enable);
int tcpci_tcpm_set_msg_header(int port, int power_role, int data_role);
int tcpci_tcpm_set_rx_enable(int port, int enable);
int tcpci_tcpm_get_message_raw(int port, uint32_t *payload, int *head);
int tcpci_tcpm_transmit(int port, enum tcpci_msg_type type,
			uint16_t header, const uint32_t *data);
int tcpci_tcpm_release(int port);
//#ifdef CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE
#if (CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE == 1)
int tcpci_set_role_ctrl(int port, enum tcpc_drp drp, enum tcpc_rp_value rp,
	enum tcpc_cc_pull pull);
int tcpci_tcpc_drp_toggle(int port);
#endif
#ifdef CONFIG_USB_PD_TCPC_LOW_POWER
int tcpci_enter_low_power_mode(int port);
#endif
enum ec_error_list tcpci_set_bist_test_mode(const int port,
		const bool enable);
#ifdef CONFIG_USB_PD_DISCHARGE_TCPC
void tcpci_tcpc_discharge_vbus(int port, int enable);
#endif
void tcpci_tcpc_enable_auto_discharge_disconnect(int port, int enable);
int tcpci_tcpc_debug_accessory(int port, bool enable);

int tcpci_tcpm_mux_init(const struct usb_mux *me);
int tcpci_tcpm_mux_set(const struct usb_mux *me, mux_state_t mux_state,
		       bool *ack_required);
int tcpci_tcpm_mux_get(const struct usb_mux *me, mux_state_t *mux_state);
int tcpci_tcpm_mux_enter_low_power(const struct usb_mux *me);
//int tcpci_get_chip_info(int port, int live, struct ec_response_pd_chip_info_v1* chip_info);
int tcpci_get_chip_info(int port, int live, struct ec_response_pd_chip_info_v1* chip_info);


#ifdef CONFIG_USBC_PPC
bool tcpci_tcpm_get_snk_ctrl(int port);
int tcpci_tcpm_set_snk_ctrl(int port, int enable);
bool tcpci_tcpm_get_src_ctrl(int port);
int tcpci_tcpm_set_src_ctrl(int port, int enable);
#endif

int tcpci_tcpc_fast_role_swap_enable(int port, int enable);

#endif /* __CROS_EC_USB_PD_TCPM_TCPCI_H */
