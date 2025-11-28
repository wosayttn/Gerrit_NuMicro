
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

#define CPRINTF(format, args...) cprintf(CC_USBPD, format, ## args)
#define CPRINTS(format, args...) cprints(CC_USBPD, format, ## args)


/**
  * PDO_FIXED_UNCONSTRAINED: The power source is not limited by power constraints
  * PDO_FIXED_DUAL_ROLE: The device can operate as both a power source and a power sink
  * PDO_FIXED_DATA_SWAP: The device supports swapping data roles
  **/
#define SRC_PDO_FIXED_FLAGS (PDO_FIXED_UNCONSTRAINED | PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP)

/**
  * USB PD Source Power Data Objects (PDOs) configuration
  * Each PDO entry defines a voltage/current capability that this device can provide
  * Recommendation: Maximum 7 PDOs (per USB PD specification), and each voltage/current should not exceed hardware capability
  *
  * Since the source PDO may be changed based on cable capability going forward, we declare it as a array on SRAM area.
  **/
uint32_t pd_src_pdo[] =
{
    PDO_FIXED(5000, 3000, SRC_PDO_FIXED_FLAGS),
    PDO_FIXED(9000, 3000, 0),
};
const int pd_src_pdo_cnt = ARRAY_SIZE(pd_src_pdo);

#if defined(__clang__) && (defined(__ARMCOMPILER) || defined(__arm__))
/* Optional: Compile-time check for PDO count (C11) */
_Static_assert( pd_src_pdo_cnt <= 7, "SPR PDO count exceeds USB PD specification limit (7)");
#endif

#define SNK_PDO_FIXED_FLAGS (PDO_FIXED_UNCONSTRAINED | PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP )

const uint32_t pd_snk_pdo[] =
{
    PDO_FIXED(5000, 3000, SNK_PDO_FIXED_FLAGS),
//      PDO_FIXED(9000, 3000, SNK_PDO_FIXED_FLAGS),
//      PDO_FIXED(12000, 3000, SNK_PDO_FIXED_FLAGS),
//      PDO_FIXED(15000, 3000, SNK_PDO_FIXED_FLAGS),
    PDO_FIXED(20000, 3000, SNK_PDO_FIXED_FLAGS),

//      PDO_BATT(4750, 21000, 15000),
//      PDO_VAR(4750, 21000, 3000),
};
#if 1
int pd_snk_pdo_cnt = ARRAY_SIZE(pd_snk_pdo);
#else
int pd_snk_pdo_cnt =  1;
#endif
#ifdef SW
void pd_set_input_current_limit(int port, uint32_t max_ma,
                                uint32_t supply_voltage)
{
    int red = supply_voltage == 20000;
    int green = supply_voltage == 5000;
    int blue = supply_voltage && !(red || green);
    gpio_set_level(GPIO_LED_R_L, !red);
    gpio_set_level(GPIO_LED_G_L, !green);
    gpio_set_level(GPIO_LED_B_L, !blue);
}
#endif

extern void VBUS_Source_Level(int port, char i8Level);
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
#ifdef SW
    /* assume the alert was programmed to detect bus voltage above 4.5V */
    return (gpio_get_level(GPIO_VBUS_ALERT_L) == 0);
#else
    return 1;
#endif
}

__override int pd_check_power_swap(int port)
{
    /* Always allow power swap */
    return 1;
}

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

//extern struct tcpm_drv nct38xx_tcpm_drv;
#define USBC_PORT_C0  0

struct tcpc_config_t tcpc_config[] =
{
    [USBC_PORT_C0] = {
        .bus_type = EC_BUS_TYPE_I2C,
        .i2c_info = {
            .port = 0, //I2C_PORT_TCPC0,
            .addr_flags = 0, //NCT38XX_I2C_ADDR1_1_FLAGS,
        },
        .drv = &nct38xx_tcpm_drv,
        .flags = TCPC_FLAGS_TCPCI_REV2_0,
    },

//  [USBC_PORT_C1] = {
//      .bus_type = EC_BUS_TYPE_I2C,
//      .i2c_info = {
//          .port = I2C_PORT_TCPC1,
//          .addr_flags = NCT38XX_I2C_ADDR1_1_FLAGS,
//      },
//      .drv = &nct38xx_tcpm_drv,
//      .flags = TCPC_FLAGS_TCPCI_REV2_0,
//  },
};

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
    return 0;
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

/**
  * Default Port Discovery VCONN Swap Policy.
  *
  * 1) If vconn_swap_to_on_flag == true, and vconn is currently off,
  * 2) Sourcing VCONN is possible
  *    then transition to pe_vcs_send_swap
  **/
__overridable bool port_discovery_vconn_swap_policy(int port,
        bool vconn_swap_flag)
{
    return false;
}


//=============================================================================================//
#define CONFIG_USB_BCD_DEV 0x0100 /* V1.0 */
#define CONFIG_USB_PD_IDENTITY_HW_VERS  1
#define CONFIG_USB_PD_IDENTITY_SW_VERS  1

/* Holds valid object position (opos) for entered mode */
static int volatile alt_mode[PD_AMODE_COUNT];

/* ----------------- Vendor Defined Messages ------------------ */
const uint32_t vdo_idh = VDO_IDH(0, /* data caps as USB host */
                                 1, /* data caps as USB device */
                                 IDH_PTYPE_AMA, /* Alternate mode */
                                 1, /* supports alt modes */
                                 USB_VID_GOOGLE);

const uint32_t vdo_product = VDO_PRODUCT(CONFIG_USB_PID, CONFIG_USB_BCD_DEV);

const uint32_t vdo_ama = VDO_AMA(CONFIG_USB_PD_IDENTITY_HW_VERS,
                                 CONFIG_USB_PD_IDENTITY_SW_VERS,
                                 0, 0, 0, 0, /* SS[TR][12] */
                                 0, /* Vconn power */
                                 0, /* Vconn power required */
                                 1, /* Vbus power required */
                                 AMA_USBSS_BBONLY /* USB SS support */);

static int svdm_response_identity(int port, uint32_t *payload)
{
    payload[VDO_I(IDH)] = vdo_idh;
    /* TODO(tbroch): Do we plan to obtain TID (test ID) for hoho */
    payload[VDO_I(CSTAT)] = VDO_CSTAT(0);
    payload[VDO_I(PRODUCT)] = vdo_product;
    payload[VDO_I(AMA)] = vdo_ama;
    return VDO_I(AMA) + 1;
}

static int svdm_response_svids(int port, uint32_t *payload)
{
    payload[1] = VDO_SVID(USB_SID_DISPLAYPORT, USB_VID_GOOGLE);
    payload[2] = 0;
    return 3;
}

#define OPOS_DP 1
#define OPOS_GFU 1

const uint32_t vdo_dp_modes[1] =
{
    VDO_MODE_DP(0,         /* UFP pin cfg supported : none */
    MODE_DP_PIN_C,         /* DFP pin cfg supported */
    1,                     /* no usb2.0 signalling in AMode */
    CABLE_PLUG,            /* its a plug */
    MODE_DP_V13,           /* DPv1.3 Support, no Gen2 */
    MODE_DP_SNK)           /* Its a sink only */
};

const uint32_t vdo_goog_modes[1] =
{
    VDO_MODE_GOOGLE(MODE_GOOGLE_FU)
};

static int svdm_response_modes(int port, uint32_t *payload)
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

static int fdp_status(int port, uint32_t *payload)
{
    return 2;
}

static int fdp_config(int port, uint32_t *payload)
{
    return 1;
}

static int svdm_enter_mode(int port, uint32_t *payload)
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

static int svdm_exit_mode(int port, uint32_t *payload)
{
#if 0
    if (PD_VDO_VID(payload[0]) == USB_SID_DISPLAYPORT)
    {
        gpio_set_level(GPIO_PD_SBU_ENABLE, 0);
        alt_mode[PD_AMODE_DISPLAYPORT] = 0;
        pd_log_event(PD_EVENT_VIDEO_DP_MODE, 0, 0, NULL);
    }
    else if (PD_VDO_VID(payload[0]) == USB_VID_GOOGLE)
    {
        alt_mode[PD_AMODE_GOOGLE] = 0;
    }
    else
    {
        CPRINTF("Unknown exit mode req:0x%08x\n", payload[0]);
    }
#endif
    return 1; /* Must return ACK */
}

static struct amode_fx dp_fx =
{
    .status = &fdp_status,
    .config = &fdp_config,
};

const struct svdm_response svdm_rsp =
{
    .identity = &svdm_response_identity,
    .svids = &svdm_response_svids,
    .modes = &svdm_response_modes,
    .enter_mode = &svdm_enter_mode,
    .amode = &dp_fx,
    .exit_mode = &svdm_exit_mode,
};

/**
  * Both ext_src_cap[] and ext_snk_cap[] arrays should be specified according to the device features.
  * They just for pass following test items.
  *				Test.PD.PROT.SRC3.3 Get_Source_Cap_Extended
  *				Test.PD.PROT.SNK3.1 Get_Sink_Cap_Extended
  *       Test.PD.PROT.PORT3.1 Get Battery Status Response
  *				Test.PD.PROT.PORT3.2 Invalid Battery Status
  **/
/* Source Capabilities Extended Data Block */
const uint8_t ext_src_cap[] = { //====> Programmer needs to modify the content 
    0x16, 0x04, 								//VID
    0x60, 0x82, 								//PID
    0x00, 0x00, 0x00, 0x00, 		//XID
    0x01, 											//FW Version
    0x01,												//HW Version
    0x01, 											//Voltage Regulation
    0x01, 											//Holdup Time
    0x00, 											//Compliance
    0x00, 											//Touch Current
    0x00, 0x00,									//Peak Current1
    0x00, 0x00,									//Peak Current2
    0x00, 0x00,									//Peak Current3
    0x00,												//Touch Temp
    0x00, 											//Source Inputs
    0x10, 											//Number of Hot Swapable Batteries(High)/Fixed Battery(Low) Slots
#if 1
    0x0F,   //SPR Source PDP Rating  15W, It needs to meet VIF file and pd_src_pdo[] 
    0x00	//Didn't support EPR
#else
    0x64, //SPR Source PDP, up to 100W. It needs to meet with pd_src_pdo[]
    0xF0	//EPR Source PDP, up to 240W. It needs to meet with pd_src_epr_pdo[]
#endif
};

/* Sink Capabilities Extended Data Block */
const uint8_t ext_snk_cap[] = {//====> Programmer needs to modify the content 
    0x16, 0x04, 0x60, 0x82, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0A, 0x05, 0x05, 0x64, 0x00, 0x00, 0x00,
};

const uint8_t battery_capabilities_rom[] = {
  0x16, 0x04,       // VID = 0x0416                                                   ====> Programmer needs to modify the content 
  0x60, 0x82,       // PID = 0x8260                                                   ====> Programmer needs to modify the content 
  0xED, 0x00,       // 0x77, 0x00,       // Design Capacity = 11.9 Wh (0.1Wh units)   ====> Programmer needs to modify the content 
  0xFF, 0xFF,       // 0xFF, 0xFF,       // Last Full Charge is unknown (0.1Wh units) 
  0x01              // Battery Type: valid                                ====> Program need to modify 
};

uint8_t battery_capabilities[9] = {0x0};

/**
  * Battery Present Capacity[31:16] 0.1 WH increments (Program need to modify)
  * Battery Info[15:8]
  *      bit 8 : Invalid Battery reference
  *      bit 9 : Battery is present when set
  *      bit [11:10] : Battery Status                 (Program need to modify)
  *             00b : Battery is Charging.
  *             01b : Battery is Discharging.
  *             10b : Battery is Idle.
  *             11b : Reserved, Shall Not be used
  *      bit [15:12]  : Reserved
  * Reserved [7:0]  : Reserved
  **/
uint32_t battery_status[] = {//Battery non-present
    0x00000900,
};