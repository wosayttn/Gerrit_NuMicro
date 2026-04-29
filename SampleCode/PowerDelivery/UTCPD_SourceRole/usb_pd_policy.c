
/**************************************************************************//**
 * @file     usb_pd_policy.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/18 3:19p $
 * @brief
 *           Device policy layer
 * @note
 *
 * Copyright (c) 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <string.h>
#include "utcpdlib.h"
#include "usb_pd.h"

#define CPRINTF(format, args...) cprintf(CC_USBPD, format, ## args)
#define CPRINTS(format, args...) cprints(CC_USBPD, format, ## args)


#define PEAK_CURRENT_100_PERCENTAGE 	(0<<20)
#define PEAK_CURRENT_125_PERCENTAGE 	(1<<20)
#define PEAK_CURRENT_150_PERCENTAGE 	(2<<20)
#define PEAK_CURRENT_200_PERCENTAGE 	(3<<20)
#define SRC_PDO_FIXED_FLAGS (PDO_FIXED_UNCONSTRAINED | PDO_FIXED_EPR_MODE_CAPABLE)

//#define SYSTEM_SUPPORT_MAX_CURRENT     3000
#if (CONFIG_OB_SOLUTION == 1)
#define TEST_CURR (5000)
#define TEST_WALT	(180)
#define PEAK_CURRENT_SELECT  	PEAK_CURRENT_125_PERCENTAGE
#endif
#if (CONFIG_WELTREND_SOLUTION == 1)
#define TEST_CURR (5000)
#define TEST_WALT	(240)
#define PEAK_CURRENT_SELECT  	PEAK_CURRENT_100_PERCENTAGE
#endif

uint32_t pd_src_pdo[] =
{
    PDO_FIXED(5000,  3000, SRC_PDO_FIXED_FLAGS | PEAK_CURRENT_SELECT),
    PDO_FIXED(9000,  3000, PEAK_CURRENT_SELECT),
    PDO_FIXED(15000, 3000, PEAK_CURRENT_SELECT),
    PDO_FIXED(20000, TEST_CURR, PEAK_CURRENT_SELECT),
};
const int pd_src_pdo_cnt = ARRAY_SIZE(pd_src_pdo);

uint32_t pd_src_epr_pdo[] =
{
    /* The flag field needs to keep 0 */

    PDO_FIXED(28000, TEST_CURR, PEAK_CURRENT_SELECT),			/* Please modify  VBUS_Source_Level() i8Level 8 to provide 28V/5A */
    PDO_FIXED(36000, TEST_CURR, PEAK_CURRENT_SELECT),			/* Please modify  VBUS_Source_Level() i8Level 9 to provide 36V/5A */
#if (CONFIG_OB_SOLUTION == 1)
    PDO_AVS(15000, 36000, TEST_WALT, PEAK_CURRENT_SELECT),      /* Please modify  VBUS_Source_Level() i8Level 10 to provide AVS 15V ~ 48V */
#endif
#if (CONFIG_WELTREND_SOLUTION == 1)
    PDO_FIXED(48000, TEST_CURR, PEAK_CURRENT_SELECT),			/* Please modify  VBUS_Source_Level() i8Level 10 to provide 48V/5A */
    PDO_AVS(15000, 48000, TEST_WALT, PEAK_CURRENT_SELECT),      /* Please modify  VBUS_Source_Level() i8Level 11 to provide AVS 15V ~ 48V */
#endif
};
const int pd_src_epr_pdo_cnt = ARRAY_SIZE(pd_src_epr_pdo);

uint32_t pd_src_epr_pdo_backup[6];


#define SNK_EPR_PDO_FIXED_FLAGS 	(RDO_FIXED_EPR_MODE_CAPABLE)	//EPR
#define SNK_SPR_PDO_FIXED_FLAGS 	(0)	//SPR

#if (OPT_SNK_ONLY == 1)
const uint32_t pd_snk_pdo[] =
{
    PDO_FIXED(5000, 3000, SNK_EPR_PDO_FIXED_FLAGS),
    PDO_FIXED(9000, 3000, 0),
    PDO_FIXED(12000, 3000, 0),
    PDO_FIXED(15000, 3000, 0),
    PDO_FIXED(20000, 3000, 0),
//    PDO_BATT(4750, 21000, 15000),
//    PDO_VAR(4750, 21000, 3000),
};
int pd_snk_pdo_cnt = ARRAY_SIZE(pd_snk_pdo);


const uint32_t pd_snk_epr_pdo[] =
{
    PDO_FIXED(28000, 5000, SNK_EPR_PDO_FIXED_FLAGS),
    PDO_AUG(3300, 21000, 3000, 0),
};
int pd_snk_epr_pdo_cnt = ARRAY_SIZE(pd_snk_epr_pdo);
#else
const uint32_t pd_snk_pdo[] =
{
    0
};
int pd_snk_pdo_cnt = ARRAY_SIZE(pd_snk_pdo);

const uint32_t pd_snk_epr_pdo[] =
{
    0
};
int pd_snk_epr_pdo_cnt = ARRAY_SIZE(pd_snk_epr_pdo);
#endif



#if (OPT_CAPTIVE_CABLE == 0)
uint32_t pd_src_pdo_backup[7];
void pd_backup_src_pdo(int port)
{
    uint32_t i;
    for( i = 0; i < pd_src_pdo_cnt; i = i + 1)
    {
        pd_src_pdo_backup[i] = pd_src_pdo[i];
    }
    for( i = 0; i < pd_src_pdo_cnt; i = i + 1)
    {
        pd_src_epr_pdo_backup[i] = pd_src_epr_pdo[i];
    }
}
void pd_recovery_src_pdo(int port)
{
    uint32_t i;
    if(pd_src_pdo_backup[0] != 0x0) /* Default Deattached */
    {
        for( i = 0; i < pd_src_pdo_cnt; i = i + 1)
        {
            pd_src_pdo[i] = pd_src_pdo_backup[i];
        }
        for( i = 0; i < pd_src_pdo_cnt; i = i + 1)
        {
            pd_src_epr_pdo[i] = pd_src_epr_pdo_backup[i];
        }
    }
    pd_src_pdo_backup[0] = 0x0;
}

/*
 *  The SRC PDOs should be limitied on following conditions
 *       1. Cable Capable
 *       2. System Capable
 *	If the declaratoon PDO exceed the cable capable.
 *	The PDOs need to be modified to meet cable capable
 *  These original PDOs will be restored after de-attached
 */
void pd_modify_src_pdo(int port, uint32_t u32CblMaxVbus, uint32_t u32CblMaxCurr)
{
    uint32_t cnt, i;

    for( i = 0; i < pd_src_pdo_cnt; i = i + 1)
    {
        switch((pd_src_pdo[i] & PDO_TYPE_MASK))
        {
        case PDO_TYPE_FIXED:
        {
            uint32_t flags, vol, curr;
            vol = PDO_FIXED_GET_VOLT(pd_src_pdo[i]);
            curr = PDO_FIXED_GET_CURR(pd_src_pdo[i]);
            flags = pd_src_pdo[i] & PDO_FIXED_FLAGS_MASK;
            if( vol > u32CblMaxVbus)
                vol = u32CblMaxVbus;
            if( curr > u32CblMaxCurr)
                curr = u32CblMaxCurr;
            pd_src_pdo[i] = PDO_FIXED(vol, curr, flags);
        }
        break;
        case PDO_TYPE_AUGMENTED:	//SPR means AUG PDO
        {
            uint32_t min_mv, max_mv, max_ma;
            min_mv = PDO_AUG_GET_MIN_VOLT(pd_src_pdo[i]);
            max_mv = PDO_AUG_GET_MAX_VOLT(pd_src_pdo[i]);
            max_ma = PDO_AUG_GET_CURR(pd_src_pdo[i]);
            if( min_mv > u32CblMaxVbus)
                min_mv = u32CblMaxVbus;
            if( max_mv > u32CblMaxVbus)
                max_mv = u32CblMaxVbus;
            if( max_ma > u32CblMaxCurr)
                max_ma = u32CblMaxCurr;
            pd_src_pdo[i] = PDO_AUG(min_mv, max_mv, max_ma, 0);
        }
        break;
        }
    }

    /* EPR PDO */
    for( i = 0; i < pd_src_epr_pdo_cnt; i = i + 1)
    {
        switch((pd_src_epr_pdo[i] & PDO_TYPE_MASK))
        {
        case PDO_TYPE_FIXED:
        {
            uint32_t flags, vol, curr;
            vol = PDO_FIXED_GET_VOLT(pd_src_epr_pdo[i]);
            curr = PDO_FIXED_GET_CURR(pd_src_epr_pdo[i]);
            //flags = pd_src_epr_pdo[i] & PDO_FIXED_FLAGS_MASK;
            if( vol > u32CblMaxVbus)
                vol = u32CblMaxVbus;
            if( curr > u32CblMaxCurr)
                curr = u32CblMaxCurr;
            pd_src_epr_pdo[i] = PDO_FIXED(vol, curr, 0);
        }
        break;
//			case PDO_TYPE_BATTERY:
//			{
//				uint32_t min_mv, max_mv, walt;
//				min_mv = PDO_BATT_GET_MIN_VOLT(pd_src_epr_pdo[i]);
//				max_mv = PDO_BATT_GET_MAX_VOLT(pd_src_epr_pdo[i]);
//				walt = PDO_BATT_GET_WALT(pd_src_epr_pdo[i]);
//				if( min_mv > u32MaxVbus)
//					min_mv = u32MaxVbus;
//				if( max_mv > u32MaxVbus)
//					max_mv = u32MaxVbus;
//				if( walt > (u32MaxVbus*u32MaxCurr))
//					walt = (u32MaxVbus*u32MaxCurr);
//				pd_src_epr_pdo[i] = PDO_BATT(min_mv, max_mv, walt);
//			}
//			break;
//			case PDO_TYPE_VARIABLE:
//			{
//				uint32_t min_mv, max_mv, curr;
//				min_mv = PDO_VAR_GET_MIN_VOLT(pd_src_epr_pdo[i]);
//				max_mv = PDO_VAR_GET_MAX_VOLT(pd_src_epr_pdo[i]);
//				curr = PDO_VAR_GET_CURR(pd_src_epr_pdo[i]);
//				if( min_mv > u32MaxVbus)
//					min_mv = u32MaxVbus;
//				if( max_mv > u32MaxVbus)
//					max_mv = u32MaxVbus;
//				if(curr > u32CblMaxCurr)
//					curr = u32CblMaxCurr;
//				pd_src_epr_pdo[i] = PDO_VAR(min_mv, max_mv, curr);
//			}
//			break;
        case PDO_TYPE_AUGMENTED:	//EPR means AVS PDO
        {
            uint32_t min_mv, max_mv, walt;
            min_mv = PDO_AVS_GET_MIN_VOLT(pd_src_epr_pdo[i]);
            max_mv = PDO_AVS_GET_MAX_VOLT(pd_src_epr_pdo[i]);
            walt = PDO_AVS_GET_WALT(pd_src_epr_pdo[i]);
            if( min_mv > u32CblMaxVbus)
                min_mv = u32CblMaxVbus;
            if( max_mv > u32CblMaxVbus)
                max_mv = u32CblMaxVbus;
            if( walt > (u32CblMaxVbus * u32CblMaxCurr))
                walt = (u32CblMaxVbus * u32CblMaxCurr);
            pd_src_epr_pdo[i] = PDO_AVS(min_mv, max_mv, walt, 0);
        }
        break;
        }
    }
}
#endif

int pd_set_power_supply_ready(int port)
{
    VBUS_Source_Level(port, 1);         /* Enable Buck output 5V and issue SRC EN command */
    return EC_SUCCESS; /* we are ready */
}
 
void pd_power_supply_reset(int port)
{
    VBUS_Source_Level(port, 0);         /* Disable Buck output 5V and issue SRC EN command */
}

int pd_snk_is_vbus_provided(int port)
{
    return 1;
}

#if 0
/** Please remove it due to usb_common.c has implement it
  * And the implement seems wrong. It will cause reject message is sent
  *	if (!pd_check_power_swap(port)) {
  *  	// PE_PRS_SRC_SNK_Reject_PR_Swap state embedded here
  *		send_ctrl_msg(port, TCPCI_MSG_SOP, PD_CTRL_REJECT);
  *	}
  */
__override int pd_check_power_swap(int port)
{
#if ((OPT_SNK_ONLY == 1) && (OPT_SRC_ONLY == 0)	)
    return 0;
#endif
#if ((OPT_SNK_ONLY == 0) && (OPT_SRC_ONLY == 1)	)
    return 0;
#endif
#if ((OPT_SNK_ONLY == 1) && (OPT_SRC_ONLY == 1)	)
    /* Always allow power swap */
    return 1;
#endif
}
#endif

__override int pd_check_data_swap(int port,
        enum pd_data_role data_role)
{
    /* Always allow data swap */
    return 1;
}

__override void pd_check_pr_role(int port,
        enum pd_power_role pr_role,
        int flags)
{

}

 __override void pd_check_dr_role(int port,
        enum pd_data_role dr_role,
        int flags)
{
}

__override int pd_custom_vdm(int port, int cnt, uint32_t *payload,
        uint32_t **rpayload)
{
    return 0;
}

int board_vbus_sink_enable(int port, int enable)
{
    VBUS_Sink_Enable(port, enable);
    return 0;
}

/* Copy from board/dingdong/usb_pd_policy.c */
void pd_set_input_current_limit(int port, uint32_t max_ma,
        uint32_t supply_voltage)
{
    /* No battery, nothing to do */
    if( supply_voltage == 0 )
        board_vbus_sink_enable(port, 0);        //disable sink vbus
    else
        board_vbus_sink_enable(port, 1);
    return;
}

/* Copy from board/ambassador/usb_pd_policy.c */
int pd_check_vconn_swap(int port)
{
#if (CONFIG_USBC_VCONN_SWAP == 1)
    return 1;
#else
    return 0;
#endif
}

/*
 * Default Port Discovery DR Swap Policy.
 *
 * 1) If dr_swap_to_dfp_flag == true and port data role is UFP,
 *    transition to pe_drs_send_swap
 */
__overridable bool port_discovery_dr_swap_policy(int port,
        enum pd_data_role dr, bool dr_swap_flag)
{
    if (dr_swap_flag && dr == PD_ROLE_UFP)
        return true;

    /* Do not perform a DR swap */
    return false;
}

/*
 * Default Port Discovery VCONN Swap Policy.
 *
 * 1) If vconn_swap_to_on_flag == true, and vconn is currently off,
 * 2) Sourcing VCONN is possible
 *    then transition to pe_vcs_send_swap
 */
CODE_LDROM __overridable bool port_discovery_vconn_swap_policy(int port,
        bool vconn_swap_flag)
{
#ifdef SW
    if (IS_ENABLED(CONFIG_USBC_VCONN) && vconn_swap_flag &&
            !tc_is_vconn_src(port) && tc_check_vconn_swap(port))
        return true;

    /* Do not perform a VCONN swap */
    return false;
#else
#if 0
    if (CONFIG_USBC_VCONN == 1)
    {
        if( (vconn_swap_flag && !tc_is_vconn_src(port)) && tc_check_vconn_swap(port))
            return true;
        else
            return false;
    }
    else
    {
        return false;
    }
#else
    return false;
#endif
#endif
}


//=============================================================================================//
#define CONFIG_USB_BCD_DEV 0x0100 /* V1.0 */
#define CONFIG_USB_PD_IDENTITY_HW_VERS  1
#define CONFIG_USB_PD_IDENTITY_SW_VERS  1

/* Holds valid object position (opos) for entered mode */
static int alt_mode[PD_AMODE_COUNT];

/* ----------------- Vendor Defined Messages ------------------ */
#if 0
const uint32_t vdo_idh = VDO_IDH(0, /* data caps as USB host */
#if 0
                                 1, /* data caps as USB device */
                                 IDH_PTYPE_AMA, /* Alternate mode */
                                 1, /* supports alt modes */
#else
                                 0, /* data caps as USB device */
                                 IDH_PTYPE_PSD, /* Alternate mode */
                                 0, /* not supports alt modes */
#endif

                                 USB_VID_GOOGLE);
#else
const uint32_t vdo_idh = VDO_IDH_REV30(
                             0, /* data caps as USB host */
                             0, /* data caps as USB device */
                             IDH_PTYPE_PSD, /* DFP product type */
                             0, /* not supports alt modes */
                             IDH_PTYPE_PSD,  /* UFP product type */
                             3,              /* Connector type */
                             USB_VID_GOOGLE);
#endif

#if 1
const uint32_t vdo_product = VDO_PRODUCT(CONFIG_USB_PID, CONFIG_USB_BCD_DEV);

const uint32_t vdo_ama = VDO_AMA(CONFIG_USB_PD_IDENTITY_HW_VERS,
                                 CONFIG_USB_PD_IDENTITY_SW_VERS,
                                 0, 0, 0, 0, /* SS[TR][12] */
                                 0, /* Vconn power */
                                 0, /* Vconn power required */
                                 1, /* Vbus power required */
                                 AMA_USBSS_BBONLY /* USB SS support */);
#endif

CODE_LDROM static int svdm_response_identity(int port, uint32_t *payload)
{
#if 0
    payload[VDO_I(IDH)] = vdo_idh;
#else
    payload[VDO_I(IDH)] = vdo_idh;
#endif	/* TODO(tbroch): Do we plan to obtain TID (test ID) for hoho */
    payload[VDO_I(CSTAT)] = VDO_CSTAT(0);
    payload[VDO_I(PRODUCT)] = vdo_product;
    payload[VDO_I(AMA)] = vdo_ama;
    return VDO_I(AMA) + 1;
}

CODE_LDROM static int svdm_response_svids(int port, uint32_t *payload)
{
    payload[1] = VDO_SVID(USB_SID_DISPLAYPORT, USB_VID_GOOGLE);
    payload[2] = 0;
    return 3;
}

#define OPOS_DP 1
#define OPOS_GFU 1

const uint32_t vdo_dp_modes[1] =
{
    VDO_MODE_DP(0,		   /* UFP pin cfg supported : none */
    MODE_DP_PIN_C, /* DFP pin cfg supported */
    1,		   /* no usb2.0	signalling in AMode */
    CABLE_PLUG,	   /* its a plug */
    MODE_DP_V13,   /* DPv1.3 Support, no Gen2 */
    MODE_DP_SNK)   /* Its a sink only */
};

const uint32_t vdo_goog_modes[1] =
{
    VDO_MODE_GOOGLE(MODE_GOOGLE_FU)
};

CODE_LDROM static int svdm_response_modes(int port, uint32_t *payload)
{
    if (PD_VDO_VID(payload[0]) == USB_SID_DISPLAYPORT)
    {
        memcpy(payload + 1, vdo_dp_modes, sizeof(vdo_dp_modes));
        return ARRAY_SIZE(vdo_dp_modes) + 1;
    }
    else if (PD_VDO_VID(payload[0]) == USB_VID_GOOGLE)
    {
        memcpy(payload + 1, vdo_goog_modes, sizeof(vdo_goog_modes));
        return ARRAY_SIZE(vdo_goog_modes) + 1;
    }
    else
    {
        return 0; /* nak */
    }
}
#if 1
CODE_LDROM static int fdp_status(int port, uint32_t *payload)
{
#if 0
    int opos = PD_VDO_OPOS(payload[0]);
    int hpd = gpio_get_level(GPIO_DP_HPD);
    if (opos != OPOS_DP)
        return 0; /* nak */

    payload[1] = VDO_DP_STATUS(0,                /* IRQ_HPD */
                               (hpd == 1),       /* HPD_HI|LOW */
                               0,		     /* request exit DP */
                               0,		     /* request exit USB */
                               0,		     /* MF pref */
                               gpio_get_level(GPIO_PD_SBU_ENABLE),
                               0,		     /* power low */
                               0x2);
    return 2;
#else
    return 2;
#endif
}

CODE_LDROM static int fdp_config(int port, uint32_t *payload)
{
#if 0
    if (PD_DP_CFG_DPON(payload[1]))
        gpio_set_level(GPIO_PD_SBU_ENABLE, 1);
#endif
    return 1;
}
#endif
CODE_LDROM static int svdm_enter_mode(int port, uint32_t *payload)
{
    int rv = 0; /* will generate a NAK */

    /* SID & mode request is valid */
    if ((PD_VDO_VID(payload[0]) == USB_SID_DISPLAYPORT) &&
            (PD_VDO_OPOS(payload[0]) == OPOS_DP))
    {
        alt_mode[PD_AMODE_DISPLAYPORT] = OPOS_DP;
        rv = 1;
        //pd_log_event(PD_EVENT_VIDEO_DP_MODE, 0, 1, NULL);
    }
    else if ((PD_VDO_VID(payload[0]) == USB_VID_GOOGLE) &&
             (PD_VDO_OPOS(payload[0]) == OPOS_GFU))
    {
        alt_mode[PD_AMODE_GOOGLE] = OPOS_GFU;
        rv = 1;
    }
#if 0
    if (rv)
        /*
         * If we failed initial mode entry we'll have enumerated the USB
         * Billboard class.  If so we should disconnect.
         */
        usb_disconnect();
#endif
    return rv;
}
#if 0
//int pd_alt_mode(int port, enum tcpm_transmit_type type, uint16_t svid)
int pd_alt_mode(int port, enum tcpci_msg_type type, uint16_t svid)
{

    if (type != TCPC_TX_SOP)
        return 0;

    if (svid == USB_SID_DISPLAYPORT)
        return alt_mode[PD_AMODE_DISPLAYPORT];
    else if (svid == USB_VID_GOOGLE)
        return alt_mode[PD_AMODE_GOOGLE];
    return 0;

}
#endif

CODE_LDROM static int svdm_exit_mode(int port, uint32_t *payload)
{
    return 1; /* Must return ACK */
}

static struct amode_fx dp_fx =
{
    .status = &fdp_status,
    .config = &fdp_config,
};

const struct svdm_response svdm_rsp =
{
#if 0
    .identity = &svdm_response_identity,
    .svids = &svdm_response_svids,
    .modes = &svdm_response_modes,
    .enter_mode = &svdm_enter_mode,
    .amode = &dp_fx,
    .exit_mode = &svdm_exit_mode,
#else
    .identity = NULL,
    .svids = NULL,
    .modes = NULL,
    .enter_mode = NULL,
    .amode = NULL,
    .exit_mode = NULL,
#endif
};

/**
   Both ext_src_cap[] and ext_snk_cap[] arrays should be specified according to the device features.
   They just for pass following test items.
					Test.PD.PROT.SRC3.3 Get_Source_Cap_Extended
					Test.PD.PROT.SNK3.1 Get_Sink_Cap_Extended
          Test.PD.PROT.PORT3.1 Get Battery Status Response
					Test.PD.PROT.PORT3.2 Invalid Battery Status
   */
#if (CONFIG_SUPPORT_SRC_CAP_EXT == 1)
const uint8_t ext_src_cap[] =
{
    0x16, 0x04, 								//VID
    0x02, 0x90, 								//PID
    0x00, 0x00, 0x00, 0x00, 		//XID
    0x01, 											//FW Version
    0x01,												//HW Version
    0x01, 											//Voltage Regulation
    0x01, 											//Holdup Time
    0x00, 											//Compliance
    0x00, 											//Touch Current
    0x00, 0x02,									//Peak Current1
    0x00, 0x00,									//Peak Current2
    0x00, 0x00,									//Peak Current3
    0x00,												//Touch Temp
    0x00, 											//Source Inputs
    0x00, 											//Number of Batteries/Battery Slots

    0x64, 											//SPR Source PDP, up to 100W. It needs to meet with pd_src_pdo[]
#if (CONFIG_OB_SOLUTION == 1)
    0xB4,      									//EPR Source PDP, up to 180W. It needs to meet with pd_src_epr_pdo[]
#endif
#if (CONFIG_WELTREND_SOLUTION == 1)
    0xF0,												//EPR Source PDP, up to 240W. It needs to meet with pd_src_epr_pdo[]
#endif
};
#endif

#if (CONFIG_SUPPORT_SNK_CAP_EXT == 1)
const uint8_t ext_snk_cap[] = {0x16, 0x04, 0x02, 0x90, 0x00, 0x00, 0x00, 0x00,
                               0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x0A, 0x05, 0x05, 0x64, 0x00, 0x00, 0x00,
                              };
#endif

#if (CONFIG_SUPPORT_MANUFACTURER_EXT == 1)	/* Don't enable it current stage */
const uint8_t ext_manufacturer_info[] = {0x16, 0x04, 0x02, 0x90,
                                         0x00, 0x00, 0x00, 0x00, 	//Battery Not Present
                                         0x00, 0x00, 0x00, 0x00, 	//If Battery Not Present
                                         0x01, 										//Bit 0: Invalid Battery reference when set
                                        };
#endif

#if 0
int pd_custom_vdm(int port, int cnt, uint32_t *payload,
                  uint32_t **rpayload)
{
    int rsize;

    if (PD_VDO_VID(payload[0]) != USB_VID_GOOGLE ||
            !alt_mode[PD_AMODE_GOOGLE])
        return 0;

    *rpayload = payload;

    rsize = pd_custom_flash_vdm(port, cnt, payload);
    if (!rsize)
    {
        int cmd = PD_VDO_CMD(payload[0]);
        switch (cmd)
        {
        case VDO_CMD_GET_LOG:
            rsize = pd_vdm_get_log_entry(payload);
            break;
        default:
            /* Unknown : do not answer */
            return 0;
        }
    }

    /* respond (positively) to the request */
    payload[0] |= VDO_SRC_RESPONDER;

    return rsize;
}
#endif