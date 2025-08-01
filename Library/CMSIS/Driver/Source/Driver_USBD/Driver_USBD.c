/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**************************************************************************//**
* @file     Driver_USBD.c
* @version  V1.00
* @brief    USBD driver for Nuvoton M5531
*
* @copyright SPDX-License-Identifier: Apache-2.0
* @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
******************************************************************************/

/*! \page Dirver_USBD USBD

# Revision History

#include "Driver_USBD.h"


  - Initial release

# Requirements

This driver requires the M5531 BSP.
The driver instance is mapped to hardware as shown in the table below:

  CMSIS Driver Instance | M5531 Hardware Resource
  :---------------------|:-----------------------
  Driver_USBD0          | USBD0

*/


/* Project can define PRJ_RTE_DEVICE_HEADER macro to include private or global RTE_Device.h. */
#ifdef   PRJ_RTE_DEVICE_HEADER
    #include PRJ_RTE_DEVICE_HEADER
#else
    #include "RTE_Device/RTE_Device.h"
#endif

#include "Driver_USBD.h"
#include <string.h>
#include "NuMicro.h"

#define ARM_USBD_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION usbd_driver_version =
{
    ARM_USBD_API_VERSION,
    ARM_USBD_DRV_VERSION
};
// Compile-time configuration **************************************************

// Configuration depending on RTE_USBD.h
// Check if at least one peripheral instance is configured in RTE_USBD.h
#if    (!(RTE_USBD0))
    #warning  USB Device driver requires at least one USB (Device) peripheral configured in RTE_USBD.h
#else
    #define DRIVER_CONFIG_VALID             1
#endif

// Configuration depending on the local macros

// Compile-time configuration (that can be externally overridden if necessary)
// Maximum number of endpoints
#ifndef USBD_MAX_ENDPOINT_NUM
    #define USBD_MAX_ENDPOINT_NUM           (12) //25 mono-directional to 12 bidirectional
#endif

// Maximum packet size for Endpoint 0
#ifndef USBD_EP0_MAX_PACKET_SIZE
    #define USBD_EP0_MAX_PACKET_SIZE        (64)
#endif

// *****************************************************************************

#ifdef  DRIVER_CONFIG_VALID     // Driver code is available only if configuration is valid

// Macros
// Macro for section for RW info
#ifdef  USBD_SECTION_NAME
    #define USBDn_SECTION_(name,n)  __attribute__((section(name #n)))
    #define USBDn_SECTION(n)        USBDn_SECTION_(USBD_SECTION_NAME,n)
#else
    #define USBDn_SECTION(n)
#endif

// Macro to create usbd_ro_info and usbd_rw_info (for instances)
#define INFO_DEFINE(n)                                                                                                     \
    static        RW_Info_t         usbd##n##_rw_info USBDn_SECTION(n);                                                    \
    static  const USBD_Info_t       usbd##n##_info = { USBD,                                                               \
                                                       &usbd##n##_rw_info                                                                                                 \
                                                     };

// Macro for declaring functions (for instances)
#define FUNCS_DECLARE(n)                                                                                                   \
    static  ARM_USBD_CAPABILITIES   USBD##n##_GetCapabilities           (void);                                            \
    static  int32_t                 USBD##n##_Initialize                (ARM_USBD_SignalDeviceEvent_t   cb_device_event,   \
                                                                         ARM_USBD_SignalEndpointEvent_t cb_endpoint_event);\
    static  int32_t                 USBD##n##_Uninitialize              (void);                                            \
    static  int32_t                 USBD##n##_PowerControl              (ARM_POWER_STATE state);                           \
    static  int32_t                 USBD##n##_DeviceConnect             (void);                                            \
    static  int32_t                 USBD##n##_DeviceDisconnect          (void);                                            \
    static  ARM_USBD_STATE          USBD##n##_DeviceGetState            (void);                                            \
    static  int32_t                 USBD##n##_DeviceRemoteWakeup        (void);                                            \
    static  int32_t                 USBD##n##_DeviceSetAddress          (uint8_t  dev_addr);                               \
    static  int32_t                 USBD##n##_ReadSetupPacket           (uint8_t *setup);                                  \
    static  int32_t                 USBD##n##_EndpointConfigure         (uint8_t  ep_addr,                                 \
                                                                         uint8_t  ep_type,                                 \
                                                                         uint16_t ep_max_packet_size);                     \
    static  int32_t                 USBD##n##_EndpointUnconfigure       (uint8_t  ep_addr);                                \
    static  int32_t                 USBD##n##_EndpointStall             (uint8_t  ep_addr, bool stall);                    \
    static  int32_t                 USBD##n##_EndpointTransfer          (uint8_t  ep_addr,                                 \
                                                                         uint8_t *data,                                    \
                                                                         uint32_t num);                                    \
    static  uint32_t                USBD##n##_EndpointTransferGetResult (uint8_t  ep_addr);                                \
    static  int32_t                 USBD##n##_EndpointTransferAbort     (uint8_t  ep_addr);                                \
    static  uint16_t                USBD##n##_GetFrameNumber            (void);

// Macro for defining functions (for instances)
#define FUNCS_DEFINE(n)                                                                                                                                                                                                        \
    static  ARM_USBD_CAPABILITIES   USBD##n##_GetCapabilities           (void)                                             { return USBDn_GetCapabilities           (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_Initialize                (ARM_USBD_SignalDeviceEvent_t   cb_device_event,                                                                                                       \
                                                                         ARM_USBD_SignalEndpointEvent_t cb_endpoint_event) { return USBDn_Initialize                (&usbd##n##_info, cb_device_event, cb_endpoint_event); }   \
    static  int32_t                 USBD##n##_Uninitialize              (void)                                             { return USBDn_Uninitialize              (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_PowerControl              (ARM_POWER_STATE state)                            { return USBDn_PowerControl              (&usbd##n##_info, state); }                                \
    static  int32_t                 USBD##n##_DeviceConnect             (void)                                             { return USBDn_DeviceConnect             (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_DeviceDisconnect          (void)                                             { return USBDn_DeviceDisconnect          (&usbd##n##_info); }                                       \
    static  ARM_USBD_STATE          USBD##n##_DeviceGetState            (void)                                             { return USBDn_DeviceGetState            (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_DeviceRemoteWakeup        (void)                                             { return USBDn_DeviceRemoteWakeup        (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_DeviceSetAddress          (uint8_t  dev_addr)                                { return USBDn_DeviceSetAddress          (&usbd##n##_info, dev_addr); }                             \
    static  int32_t                 USBD##n##_ReadSetupPacket           (uint8_t *setup)                                   { return USBDn_ReadSetupPacket           (&usbd##n##_info, setup); }                                \
    static  int32_t                 USBD##n##_EndpointConfigure         (uint8_t  ep_addr,                                                                                                                                     \
                                                                         uint8_t  ep_type,                                                                                                                                     \
                                                                         uint16_t ep_max_packet_size)                      { return USBDn_EndpointConfigure         (&usbd##n##_info, ep_addr, ep_type, ep_max_packet_size); } \
    static  int32_t                 USBD##n##_EndpointUnconfigure       (uint8_t  ep_addr)                                 { return USBDn_EndpointUnconfigure       (&usbd##n##_info, ep_addr); }                              \
    static  int32_t                 USBD##n##_EndpointStall             (uint8_t  ep_addr, bool stall)                     { return USBDn_EndpointStall             (&usbd##n##_info, ep_addr, stall); }                       \
    static  int32_t                 USBD##n##_EndpointTransfer          (uint8_t  ep_addr,                                                                                                                                     \
                                                                         uint8_t *data,                                                                                                                                        \
                                                                         uint32_t num)                                     { return USBDn_EndpointTransfer          (&usbd##n##_info, ep_addr, data, num); }                   \
    static  uint32_t                USBD##n##_EndpointTransferGetResult (uint8_t  ep_addr)                                 { return USBDn_EndpointTransferGetResult (&usbd##n##_info, ep_addr); }                              \
    static  int32_t                 USBD##n##_EndpointTransferAbort     (uint8_t  ep_addr)                                 { return USBDn_EndpointTransferAbort     (&usbd##n##_info, ep_addr); }                              \
    static  uint16_t                USBD##n##_GetFrameNumber            (void)                                             { return USBDn_GetFrameNumber            (&usbd##n##_info); }
// Macro for defining driver structures (for instances)
#define USBD_DRIVER(n)                  \
    ARM_DRIVER_USBD Driver_USBD##n = {      \
                                            USBD_GetVersion,                      \
                                            USBD##n##_GetCapabilities,            \
                                            USBD##n##_Initialize,                 \
                                            USBD##n##_Uninitialize,               \
                                            USBD##n##_PowerControl,               \
                                            USBD##n##_DeviceConnect,              \
                                            USBD##n##_DeviceDisconnect,           \
                                            USBD##n##_DeviceGetState,             \
                                            USBD##n##_DeviceRemoteWakeup,         \
                                            USBD##n##_DeviceSetAddress,           \
                                            USBD##n##_ReadSetupPacket,            \
                                            USBD##n##_EndpointConfigure,          \
                                            USBD##n##_EndpointUnconfigure,        \
                                            USBD##n##_EndpointStall,              \
                                            USBD##n##_EndpointTransfer,           \
                                            USBD##n##_EndpointTransferGetResult,  \
                                            USBD##n##_EndpointTransferAbort,      \
                                            USBD##n##_GetFrameNumber              \
                                     };

// Endpoint related macros
#define EP_OUT_INDEX            (0U)
#define EP_IN_INDEX             (1U)
#define EP_DIR(ep_addr)         (((ep_addr) >> 7) & 1U)
#define EP_NUM(ep_addr)         (ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK)
#define EP_IDX(ep_addr)         ((EP_NUM(ep_addr) * 2) + EP_DIR(ep_addr))

// Driver status
typedef struct
{
    uint8_t                       initialized  : 1;       // Initialized status: 0 - not initialized, 1 - initialized
    uint8_t                       powered      : 1;       // Power status:       0 - not powered,     1 - powered
    uint8_t                       reserved     : 6;       // Reserved (for padding)
} DriverStatus_t;

// USB Device state
typedef struct
{
    volatile uint8_t              vbus;                   // USB Device VBUS state
    volatile uint8_t              speed;                  // USB Device speed (ARM_USB_SPEED_xxx) state
    volatile uint8_t              active;                 // USB Device active state
} USBD_State_t;

// Endpoint information
typedef struct
{
    uint8_t *volatile             data;                   // Pointer to data
    volatile uint32_t             num;                    // Number of bytes to transfer
    volatile uint16_t             configured;             // Endpoint configuration status
    uint16_t                      max_packet_size;        // Maximum packet size (in bytes)
    volatile uint32_t             num_transferred_total;  // Number of totally transferred bytes
    volatile uint32_t             num_transferring;       // Number of transferred bytes in last transfer
} EP_Info_t;

// Instance run-time information (RW)
typedef struct
{
    ARM_USBD_SignalDeviceEvent_t  cb_device_event;        // Device event callback
    ARM_USBD_SignalEndpointEvent_t cb_endpoint_event;     // Endpoint event callback
    DriverStatus_t                drv_status;             // Driver status
    USBD_State_t                  usbd_state;             // USB Device state
    volatile uint32_t             setup_received;         // Setup Packet received flag (0 - not received or read already, 1 - received and unread yet)
    volatile uint8_t              setup_packet[8];        // Setup Packet data
    EP_Info_t                     ep_info[USBD_MAX_ENDPOINT_NUM][2];         // Endpoint information, EP0 store
} RW_Info_t;

// Instance compile-time information (RO)
// also contains pointer to run-time information
#define USBD_HandleTypeDef  USBD_T
typedef struct
{
    USBD_HandleTypeDef           *ptr_USBD;               // Pointer to USBD handle
    RW_Info_t                    *ptr_rw_info;            // Pointer to run-time information (RW)
} USBD_Info_t;

// Information definitions (for instances)
#if (RTE_USBD0)
    INFO_DEFINE(0)
#endif

// List of available USBD instance infos
static const USBD_Info_t *const usbd_info_list[] =
{
#if (RTE_USBD0)
    &usbd0_info,
#endif
    NULL
};

// Local functions prototypes
static const USBD_Info_t        *USBD_GetInfo(const USBD_HandleTypeDef * husbd);
static int32_t                  USBDn_EndpointConfigureBuffer(const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_max_packet_size);
static ARM_DRIVER_VERSION       USBD_GetVersion(void);
static ARM_USBD_CAPABILITIES    USBDn_GetCapabilities(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_Initialize(const USBD_Info_t *const ptr_usbd_info, ARM_USBD_SignalDeviceEvent_t cb_device_event, ARM_USBD_SignalEndpointEvent_t cb_endpoint_event);
static int32_t                  USBDn_Uninitialize(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_PowerControl(const USBD_Info_t *const ptr_usbd_info, ARM_POWER_STATE state);
static int32_t                  USBDn_DeviceConnect(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_DeviceDisconnect(const USBD_Info_t *const ptr_usbd_info);
static ARM_USBD_STATE           USBDn_DeviceGetState(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_DeviceRemoteWakeup(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_DeviceSetAddress(const USBD_Info_t *const ptr_usbd_info, uint8_t  dev_addr);
static int32_t                  USBDn_ReadSetupPacket(const USBD_Info_t *const ptr_usbd_info, uint8_t *setup);
static int32_t                  USBDn_EndpointConfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr, uint8_t  ep_type, uint16_t ep_max_packet_size);
static int32_t                  USBDn_EndpointUnconfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr);
static int32_t                  USBDn_EndpointStall(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr, bool stall);
static int32_t                  USBDn_EndpointTransfer(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr, uint8_t *data, uint32_t num);
static uint32_t                 USBDn_EndpointTransferGetResult(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr);
static int32_t                  USBDn_EndpointTransferAbort(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr);
static uint16_t                 USBDn_GetFrameNumber(const USBD_Info_t *const ptr_usbd_info);

// Local driver functions declarations (for instances)
#if (RTE_USBD0)
    FUNCS_DECLARE(0)
#endif

// Auxiliary functions

/**
  \fn          USBD_Info_t *USBD_GetInfo (const USBD_HandleTypeDef *husbd)
  \brief       Get pointer to USBD_GetInfo structure corresponding to specified huart.
  \param[in]   huart    Pointer to USBD handle structure (USBD_HandleTypeDef)
  \return      pointer to USBD info structure (USBD_Info_t)
*/
static const USBD_Info_t *USBD_GetInfo(const USBD_HandleTypeDef *husbd)
{
    const USBD_Info_t *ptr_usbd_info;
    uint8_t i;

    ptr_usbd_info = NULL;
    i = 0U;

    // Find USBD which uses same huart handle as parameter ptr_USBD
    for (i = 0U; i < (sizeof(usbd_info_list) / sizeof(USBD_HandleTypeDef *)); i++)
    {
        if (usbd_info_list[i] != NULL)
        {
            if (usbd_info_list[i]->ptr_USBD == husbd)
            {
                ptr_usbd_info = usbd_info_list[i];
                break;
            }
        }
    }

    return ptr_usbd_info;
}

/**
  \fn          int32_t USBDn_EndpointConfigureBuffer (const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_max_packet_size)
  \brief       Configure buffer for USB Endpoint (separate IN and OUT).
  \detail      The function will update the ep_max_packet of the endpoint and then update the Endpoint Buffer Segmentation for each endpoint one by one.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type  Endpoint Type (ARM_USB_ENDPOINT_xxx)
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      \ref execution_status
*/
static int32_t USBDn_EndpointConfigureBuffer(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_max_packet_size)
{

    uint32_t addr;

    uint8_t  ep_num = EP_NUM(ep_addr);
    uint8_t ep_dir = EP_DIR(ep_addr);
    (void)ep_type;
    // Update buf for FIFO
    addr = 0;
    // Buffer for setup packet -> [0 ~ 0x7]
    ptr_usbd_info->ptr_USBD->STBUFSEG = addr;
    addr += 8;

    if (ep_num == 0U)
    {
        // Control endpoint: Select EP0 for OUT, EP1 for IN,share same buf
        ptr_usbd_info->ptr_USBD->EP[EP0].BUFSEG  = addr;
        ptr_usbd_info->ptr_USBD->EP[EP1].BUFSEG  = addr;
    }

    addr += ptr_usbd_info->ptr_rw_info->ep_info[ep_num][ep_dir].max_packet_size;

    for (ep_num = 1U; ep_num < USBD_MAX_ENDPOINT_NUM; ep_num++)
    {
        for (ep_dir = 0; ep_dir < 2; ep_dir++)
        {
            ptr_usbd_info->ptr_USBD->EP[ep_num * 2 + ep_dir].BUFSEG  = addr;
            addr += ptr_usbd_info->ptr_rw_info->ep_info[ep_num][ep_dir].max_packet_size;
        }
    }

    return ARM_DRIVER_OK;
}

typedef struct
{
    USBD_HandleTypeDef *husbd;
    int32_t irq_n;
} S_IRQ_SEL_t;


static S_IRQ_SEL_t usbd_IRQ_table[] =
{
#if (RTE_USBD0)
    {USBD, USBD_IRQn},
#endif
};
static S_IRQ_SEL_t *IRQSelector(USBD_HandleTypeDef *husbd)
{
    int cnt = sizeof(usbd_IRQ_table) / sizeof(S_IRQ_SEL_t);
    int i;

    for (i = 0; i < cnt; i++)
    {
        if (usbd_IRQ_table[i].husbd == husbd)
            return &usbd_IRQ_table[i];
    }

    return NULL;
}

/**
  \fn          void USBDn_Set_NVIC (const USBD_Info_t *ptr_usbd_info)
  \brief       Set USBDn NVIC
  \param[in]   ptr_usbd_info   Pointer to USBD info structure (USBD_Info_t)
*/
static void USBDn_Set_NVIC(const USBD_Info_t *ptr_usbd_info)
{
    USBD_HandleTypeDef *husbd = ptr_usbd_info->ptr_USBD;
    /* Unlock protected registers */
    SYS_UnlockReg();

    S_IRQ_SEL_t *irq_sel;

    irq_sel = IRQSelector(husbd);

    if (irq_sel == NULL)
    {
        printf("Error! unable select USBD IRQ table \n");
        return;
    }

    NVIC_EnableIRQ(irq_sel->irq_n);
    /* Lock protected registers */
    SYS_LockReg();
}

/**
  \fn          void USBDn_Clear_NVIC (const USBD_Info_t *ptr_usbd_info)
  \brief       Clear USBDn NVIC
  \param[in]   ptr_usbd_info   Pointer to USBD info structure (USBD_Info_t)
*/
static void USBDn_Clear_NVIC(const USBD_Info_t *ptr_usbd_info)
{
    USBD_HandleTypeDef *husbd = ptr_usbd_info->ptr_USBD;

    /* Unlock protected registers */
    SYS_UnlockReg();

    S_IRQ_SEL_t *irq_sel;

    irq_sel = IRQSelector(husbd);

    if (irq_sel == NULL)
    {
        printf("Error! unable select USBD IRQ table \n");
        return;
    }

    /* Lock protected registers */
    SYS_LockReg();
}

static void USBD_EP_Transmit(USBD_HandleTypeDef *husbd, uint8_t epnum, uint8_t *data, uint32_t num)
{
    uint32_t addr;
    addr = USBD_BUF_BASE + husbd->EP[epnum].BUFSEG;
    USBD_MemCopy((uint8_t *)addr, (uint8_t *)data, num);
    /* Prepare the data for next IN transfer */
    husbd->EP[epnum].MXPLD = num;
}

static void USBD_EP_Receive(USBD_HandleTypeDef *husbd, uint8_t epnum, uint8_t *data, uint32_t num)
{
    uint32_t addr;
    addr = USBD_BUF_BASE + husbd->EP[epnum].BUFSEG;
    USBD_MemCopy((uint8_t *)data, (uint8_t *)addr, husbd->EP[epnum].MXPLD);
    /* Ready to get next OUT transfer */
    husbd->EP[epnum].MXPLD = num;

}

// Driver functions ************************************************************

/**
  \fn          ARM_DRIVER_VERSION USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USBD_GetVersion(void)
{
    return usbd_driver_version;
}

/**
  \fn          ARM_USBD_CAPABILITIES USBDn_GetCapabilities (const USBD_Info_t * const ptr_usbd_info)
  \brief       Get driver capabilities.
  \param[in]   ptr_usbd_info   Pointer to USBD info structure (USBD_Info_t)
  \return      \ref ARM_USBD_CAPABILITIES
*/
static ARM_USBD_CAPABILITIES USBDn_GetCapabilities(const USBD_Info_t *const ptr_usbd_info)
{
    ARM_USBD_CAPABILITIES driver_capabilities;

    // Clear capabilities structure
    memset(&driver_capabilities, 0, sizeof(ARM_USBD_CAPABILITIES));

    // If VBUS detection is available
    driver_capabilities.vbus_detection = 1U;
    driver_capabilities.event_vbus_on  = 1U;
    driver_capabilities.event_vbus_off = 1U;
    return driver_capabilities;
}

/**
  \fn          int32_t USBDn_Initialize (const USBD_Info_t * const        ptr_usbd_info,
                                         ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                         ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   ptr_usbd_info   Pointer to USBD info structure (USBD_Info_t)
  \param[in]   cb_device_event    Pointer to \ref ARM_USBD_SignalDeviceEvent
  \param[in]   cb_endpoint_event  Pointer to \ref ARM_USBD_SignalEndpointEvent
  \return      \ref execution_status
*/
static int32_t USBDn_Initialize(const USBD_Info_t *const ptr_usbd_info, ARM_USBD_SignalDeviceEvent_t cb_device_event,
                                ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
{
    // Clear run-time info
    memset((void *)ptr_usbd_info->ptr_rw_info, 0, sizeof(RW_Info_t));

    // Register callback functions
    ptr_usbd_info->ptr_rw_info->cb_device_event   = cb_device_event;
    ptr_usbd_info->ptr_rw_info->cb_endpoint_event = cb_endpoint_event;
    // Set driver status to initialized
    ptr_usbd_info->ptr_rw_info->drv_status.initialized = 1U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_Uninitialize (const USBD_Info_t * const ptr_usbd_info)
  \brief       De-initialize USB Device Interface.
  \param[in]   ptr_usbd_info   Pointer to USBD info structure (USBD_Info_t)
  \return      \ref execution_status
*/
static int32_t USBDn_Uninitialize(const USBD_Info_t *const ptr_usbd_info)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered != 0U)
    {
        // If peripheral is powered, power off the peripheral
        (void)USBDn_PowerControl(ptr_usbd_info, ARM_POWER_OFF);
    }

    // Clear run-time info
    memset((void *)ptr_usbd_info->ptr_rw_info, 0, sizeof(RW_Info_t));

    return ARM_DRIVER_OK;
}

static int32_t USBDn_PowerControl(const USBD_Info_t *const ptr_usbd_info, ARM_POWER_STATE state)
{
    ARM_USBD_SignalDeviceEvent_t   cb_device_event;
    ARM_USBD_SignalEndpointEvent_t cb_endpoint_event;
    DriverStatus_t                 drv_status;
    uint8_t                        ep_num;
    uint8_t                        ep_dir;

    switch (state)
    {
        case ARM_POWER_FULL:
            if (ptr_usbd_info->ptr_rw_info->drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            // Store variables we need to preserve
            cb_device_event   = ptr_usbd_info->ptr_rw_info->cb_device_event;
            cb_endpoint_event = ptr_usbd_info->ptr_rw_info->cb_endpoint_event;
            drv_status        = ptr_usbd_info->ptr_rw_info->drv_status;

            // Clear run-time info
            memset((void *)ptr_usbd_info->ptr_rw_info, 0, sizeof(RW_Info_t));

            // Restore variables we wanted to preserve
            ptr_usbd_info->ptr_rw_info->cb_device_event   = cb_device_event;
            ptr_usbd_info->ptr_rw_info->cb_endpoint_event = cb_endpoint_event;
            ptr_usbd_info->ptr_rw_info->drv_status        = drv_status;
            // Initialize pins, clocks, interrupts and peripheral
            USBDn_Set_NVIC(ptr_usbd_info);

            // Set driver status to powered
            ptr_usbd_info->ptr_rw_info->drv_status.powered = 1U;

            // Initial USB engine
            ptr_usbd_info->ptr_USBD->ATTR = 0x7D0ul;

            USBDn_DeviceDisconnect(ptr_usbd_info);

            // Clear USB-related interrupts before enable interrupt
            ptr_usbd_info->ptr_USBD->INTSTS = (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);

            // Enable USB-related interrupts.
            ptr_usbd_info->ptr_USBD->INTEN |= (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);

            // Clear SOF
            ptr_usbd_info->ptr_USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

            break;

        case ARM_POWER_OFF:
            USBD_DISABLE_PHY();
            USBD_SwReset();
            USBDn_Clear_NVIC(ptr_usbd_info);

            // Set driver status to not powered
            ptr_usbd_info->ptr_rw_info->drv_status.powered = 0U;

            // Store variables we need to preserve
            cb_device_event   = ptr_usbd_info->ptr_rw_info->cb_device_event;
            cb_endpoint_event = ptr_usbd_info->ptr_rw_info->cb_endpoint_event;
            drv_status        = ptr_usbd_info->ptr_rw_info->drv_status;

            // Clear run-time info
            memset((void *)ptr_usbd_info->ptr_rw_info, 0, sizeof(RW_Info_t));

            // Restore variables we wanted to preserve
            ptr_usbd_info->ptr_rw_info->cb_device_event   = cb_device_event;
            ptr_usbd_info->ptr_rw_info->cb_endpoint_event = cb_endpoint_event;
            ptr_usbd_info->ptr_rw_info->drv_status        = drv_status;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_DeviceConnect (const USBD_Info_t * const ptr_usbd_info)
  \brief       Connect USB Device.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \return      \ref execution_status
*/
static int32_t USBDn_DeviceConnect(const USBD_Info_t *const ptr_usbd_info)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Disable software-disconnect function */
    ptr_usbd_info->ptr_USBD->SE0 &= ~USBD_DRVSE0;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_DeviceDisconnect (const USBD_Info_t * const ptr_usbd_info)
  \brief       Disconnect USB Device.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \return      \ref execution_status
*/
static int32_t USBDn_DeviceDisconnect(const USBD_Info_t *const ptr_usbd_info)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    /* enable software-disconnect function. */
    ptr_usbd_info->ptr_USBD->SE0 |= USBD_DRVSE0;

    return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USBD_STATE USBDn_DeviceGetState (const USBD_Info_t * const ptr_usbd_info)
  \brief       Get current USB Device State.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \return      Device State \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE USBDn_DeviceGetState(const USBD_Info_t *const ptr_usbd_info)
{
    ARM_USBD_STATE state;
    // Clear state structure
    memset(&state, 0, sizeof(ARM_USBD_STATE));

    // Process additionally handled communication information
    if (ptr_usbd_info->ptr_rw_info->usbd_state.vbus != 0U)
    {
        state.vbus = 1U;
    }

    switch (ptr_usbd_info->ptr_rw_info->usbd_state.speed)
    {
        case ARM_USB_SPEED_FULL:
            state.speed = ARM_USB_SPEED_FULL;
            break;

        case ARM_USB_SPEED_HIGH:
            break;

        default:
            break;
    }

    if (ptr_usbd_info->ptr_rw_info->usbd_state.active != 0U)
    {
        state.active = 1U;
    }

    return state;
}

/**
  \fn          int32_t USBDn_DeviceRemoteWakeup (const USBD_Info_t * const ptr_usbd_info)
  \brief       Trigger USB Remote Wakeup.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \return      \ref execution_status
*/
static int32_t USBDn_DeviceRemoteWakeup(const USBD_Info_t *const ptr_usbd_info)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Enable PHY before sending Resume('K') state */
    ptr_usbd_info->ptr_USBD->ATTR |= USBD_PHY_EN;
    ptr_usbd_info->ptr_USBD->ATTR |= USBD_RWAKEUP;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_DeviceSetAddress (const USBD_Info_t * const ptr_usbd_info, uint8_t dev_addr)
  \brief       Set USB Device Address.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \param[in]   dev_addr  Device Address
  \return      \ref execution_status
*/
static int32_t USBDn_DeviceSetAddress(const USBD_Info_t *const ptr_usbd_info, uint8_t dev_addr)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    uint32_t addr;
    addr = ptr_usbd_info->ptr_USBD->FADDR;

    if ((addr != dev_addr) && (addr == 0ul))
        ptr_usbd_info->ptr_USBD->FADDR = dev_addr;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_ReadSetupPacket (const USBD_Info_t * const ptr_usbd_info, uint8_t *setup)
  \brief       Read setup packet received over Control Endpoint.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \param[out]  setup  Pointer to buffer for setup packet
  \return      \ref execution_status
*/
static int32_t USBDn_ReadSetupPacket(const USBD_Info_t *const ptr_usbd_info, uint8_t *setup)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (ptr_usbd_info->ptr_rw_info->setup_received == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    do
    {
        ptr_usbd_info->ptr_rw_info->setup_received = 0U;
        USBD_MemCopy((uint8_t *)setup, (uint8_t *)ptr_usbd_info->ptr_rw_info->setup_packet, 8U);
    } while (ptr_usbd_info->ptr_rw_info->setup_received != 0U);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_EndpointConfigure (const USBD_Info_t * const ptr_usbd_info,
                                                      uint8_t           ep_addr,
                                                      uint8_t           ep_type,
                                                      uint16_t          ep_max_packet_size)
  \brief       Configure USB Endpoint.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type  Endpoint Type (ARM_USB_ENDPOINT_xxx)
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      \ref execution_status
*/
static int32_t USBDn_EndpointConfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr,
                                       uint8_t  ep_type,
                                       uint16_t ep_max_packet_size)
{
    EP_Info_t *ptr_ep;
    uint32_t   ep_dir_mask, ep_type_mask;
    uint8_t    ep_num = EP_NUM(ep_addr);
    uint8_t    ep_dir = EP_DIR(ep_addr);

    ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep_num][ep_dir];
    // Clear Endpoint information
    memset((void *)ptr_ep, 0, sizeof(EP_Info_t));


    ptr_ep->data = NULL;
    ptr_ep->num = 0U;

    // Store max packet size information (for Endpoint buffer configuration)
    ptr_ep->max_packet_size = ep_max_packet_size;
    // Reconfigure Endpoint buffer
    USBDn_EndpointConfigureBuffer(ptr_usbd_info, ep_addr, ep_type, ep_max_packet_size);

    if (ep_dir)
        ep_dir_mask = USBD_CFG_EPMODE_IN;
    else
        ep_dir_mask = USBD_CFG_EPMODE_OUT;

    if (ep_type == ARM_USB_ENDPOINT_ISOCHRONOUS)
        ep_type_mask = USBD_CFG_TYPE_ISO;
    else
        ep_type_mask = 0;

    // configured endpoint
    ptr_usbd_info->ptr_USBD->EP[EP_IDX(ep_addr)].CFG  = (ep_dir_mask | ep_type_mask | ep_num);
    // Update endpoint configured status
    ptr_ep->configured = 1U;
    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_EndpointUnconfigure (const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr)
  \brief       Unconfigure USB Endpoint.
  \param[in]   ptr_ro_info     Pointer to USBD info structure (ptr_usbd_info)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBDn_EndpointUnconfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    EP_Info_t *ptr_ep;
    uint8_t    ep_num;
    uint8_t    ep_dir;
    ep_num = EP_NUM(ep_addr);
    ep_dir = EP_DIR(ep_addr);

    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep_num][ep_dir];

    // Clear Endpoint information
    memset((void *)ptr_ep, 0, sizeof(EP_Info_t));
    ptr_ep->data = NULL;
    ptr_ep->num = 0U;
    // Clear Endpoint CFG
    ptr_usbd_info->ptr_USBD->EP[EP_IDX(ep_addr)].CFG = USBD_CFG_CSTALL;
    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_EndpointStall (const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr, bool stall)
  \brief       Set/Clear Stall for USB Endpoint.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   stall  Operation
                - \b false Clear
                - \b true Set
  \return      \ref execution_status
*/
static int32_t USBDn_EndpointStall(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, bool stall)
{

    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (stall != 0U)
    {
        // Set STALL
        ptr_usbd_info->ptr_USBD->EP[EP_IDX(ep_addr)].CFGP  |= USBD_CFGP_SSTALL_Msk;
    }
    else
    {
        // Clear STALL and reset Packet Identifier(PID)
        ptr_usbd_info->ptr_USBD->EP[EP_IDX(ep_addr)].CFGP  &= ~USBD_CFGP_SSTALL_Msk;
        ptr_usbd_info->ptr_USBD->EP[EP_IDX(ep_addr)].CFG &= ~USBD_CFG_DSQSYNC_Msk;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_EndpointTransfer (const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr, uint8_t *data, uint32_t num)
  \brief       Read data from or Write data to USB Endpoint.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  data Pointer to buffer for data to read or with data to write
  \param[in]   num  Number of data bytes to transfer
  \return      \ref execution_status
*/
static int32_t USBDn_EndpointTransfer(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, uint8_t *data, uint32_t num)
{
    EP_Info_t *ptr_ep;
    uint8_t    ep_num = EP_NUM(ep_addr);
    uint8_t    ep_dir = EP_DIR(ep_addr);

    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep_num][ep_dir];
    // Clear number of transferred bytes
    ptr_ep->num_transferred_total = 0U;

    // Register pointer to data and number of bytes to transfer for Endpoint
    ptr_ep->data = data;
    ptr_ep->num = num;

    if ((ep_num == 0U) && ep_dir)
    {
        //Set EP0 IN token PID to DATA1
        ptr_usbd_info->ptr_USBD->EP[EP_IDX(ep_addr)].CFG |= USBD_CFG_DSQSYNC_Msk;
    }

    ptr_ep->num_transferring = num;

    if (ptr_ep->max_packet_size < num)
    {
        // For Endpoint total transfer is done in transfers of maximum packet size at a time
        ptr_ep->num_transferring = ptr_ep->max_packet_size;
    }

    if (ep_dir)
        USBD_EP_Transmit(ptr_usbd_info->ptr_USBD, EP_IDX(ep_addr), data, ptr_ep->num_transferring);
    else
        USBD_EP_Receive(ptr_usbd_info->ptr_USBD, EP_IDX(ep_addr), data, ptr_ep->num_transferring);

    return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t USBDn_EndpointTransferGetResult (const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr)
  \brief       Get result of USB Endpoint transfer.
  \param[in]   ptr_usbd_info     Pointer to USBD info structure (ptr_usbd_info)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      number of successfully transferred data bytes
*/
static uint32_t USBDn_EndpointTransferGetResult(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    uint8_t ep_num = EP_NUM(ep_addr);
    uint8_t ep_dir = EP_DIR(ep_addr);

    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return 0U;
    }

    if (ep_num >= USBD_MAX_ENDPOINT_NUM)
    {
        return 0U;
    }

    return ptr_usbd_info->ptr_rw_info->ep_info[ep_num][ep_dir].num_transferred_total;
}

static int32_t USBDn_EndpointTransferAbort(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    // STOP_TRANSACTION
    ptr_usbd_info->ptr_USBD->EP[EP_IDX(ep_addr)].CFGP |= USBD_CFGP_CLRRDY_Msk;
    return ARM_DRIVER_OK;
}

/**
  \fn          uint16_t USBDn_GetFrameNumber (const USBD_Info_t * const ptr_usbd_info)
  \brief       Get current USB Frame Number.
  \param[in]   ptr_usbd_info   Pointer to USBD info structure (USBD_Info_t)
  \return      Frame Number
*/
static uint16_t USBDn_GetFrameNumber(const USBD_Info_t *const ptr_usbd_info)
{
    return ptr_usbd_info->ptr_USBD->FN;
}

// event Callback functions *********************************************************
/**
  \fn          void DataOutStageCallback (USBD_HandleTypeDef *husbd, uint8_t epnum)
  \brief       Data OUT stage callback.
  \param[in]   hpcd     USBD handle
  \param[in]   epnum    endpoint number
  */
void USBD_DataOutStageCallback(USBD_HandleTypeDef *husbd, uint8_t epnum)
{
    const USBD_Info_t *ptr_usbd_info;
    EP_Info_t *ptr_ep;
    uint32_t   num_transferred, addr;
    uint32_t   num_to_transfer;
    uint8_t   *data_to_transfer;
    uint32_t   event;

    ptr_usbd_info = USBD_GetInfo(husbd);

    if (ptr_usbd_info == NULL)
    {
        return;
    }

    if (ptr_usbd_info->ptr_rw_info == NULL)
    {
        return;
    }

    ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[epnum][EP_OUT_INDEX];

    event  = 0U;

    if (epnum != 0U)                      // Endpoint other than 0
    {
        ptr_ep->num_transferred_total = ptr_usbd_info->ptr_USBD->EP[epnum * 2].MXPLD;
        USBD_MemCopy((uint8_t *)ptr_ep->data, (uint8_t *)(USBD_BUF_BASE + husbd->EP[epnum * 2].BUFSEG), ptr_ep->num_transferred_total);
        event = ARM_USBD_EVENT_OUT;
    }
    else                                  // Endpoint 0
    {
        num_transferred = ptr_usbd_info->ptr_USBD->EP[epnum * 2].MXPLD;
        ptr_ep->num_transferred_total += num_transferred;

        if ((num_transferred < ptr_ep->max_packet_size) || (ptr_ep->num_transferred_total == ptr_ep->num))
        {
            // If all data was transferred
            event = ARM_USBD_EVENT_OUT;
        }
        else
        {
            // If there is more data to transfer
            data_to_transfer = ptr_ep->data + ptr_ep->num_transferred_total;
            num_to_transfer  = ptr_ep->num - ptr_ep->num_transferred_total;

            if (num_to_transfer > ptr_ep->max_packet_size)
            {
                num_to_transfer = ptr_ep->max_packet_size;
            }

            ptr_ep->num_transferring = num_to_transfer;
            USBD_EP_Receive(ptr_usbd_info->ptr_USBD, epnum * 2, data_to_transfer, ptr_ep->num_transferring);
        }
    }

    if ((ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL) && (event != 0U))
    {
        ptr_usbd_info->ptr_rw_info->cb_endpoint_event(epnum, event);
    }
}

/**
  \fn          void USBD_DataInStageCallback(USBD_HandleTypeDef *husbd, uint8_t epnum)
  \brief       Data IN stage callback.
  \param[in]   husbd     USBD handle
  \param[in]   epnum    endpoint number
  */
void USBD_DataInStageCallback(USBD_HandleTypeDef *husbd, uint8_t epnum)
{
    const USBD_Info_t *ptr_usbd_info;
    EP_Info_t *ptr_ep;
    uint32_t   num_to_transfer;
    uint8_t   *data_to_transfer;
    uint32_t   event;

    ptr_usbd_info = USBD_GetInfo(husbd);

    if (ptr_usbd_info == NULL)
    {
        return;
    }

    if (ptr_usbd_info->ptr_rw_info == NULL)
    {
        return;
    }

    ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[epnum][EP_IN_INDEX];

    event  = 0U;

    // Update transferred number
    ptr_ep->num_transferred_total += ptr_ep->num_transferring;

    if (ptr_ep->num_transferred_total == ptr_ep->num)
    {
        // If all data was transferred
        event = ARM_USBD_EVENT_IN;
    }
    else
    {
        // If there is more data to transfer
        data_to_transfer = ptr_ep->data + ptr_ep->num_transferred_total;
        num_to_transfer  = ptr_ep->num - ptr_ep->num_transferred_total;

        if (num_to_transfer > ptr_ep->max_packet_size)
        {
            num_to_transfer = ptr_ep->max_packet_size;
        }

        ptr_ep->num_transferring = num_to_transfer;

        USBD_EP_Transmit(ptr_usbd_info->ptr_USBD, epnum * 2 + 1, data_to_transfer, ptr_ep->num_transferring);
    }

    if ((ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL) && (event != 0U))
    {
        ptr_usbd_info->ptr_rw_info->cb_endpoint_event(epnum | ARM_USB_ENDPOINT_DIRECTION_MASK, event);
    }
}

/**
  \fn          USBD_SetupStageCallback (const USBD_HandleTypeDef *husbd)
  \brief       Setup stage callback.
  \param[in]   husbd     husbd handle
  */
void USBD_SetupStageCallback(const USBD_HandleTypeDef *husbd)
{
    const USBD_Info_t *ptr_usbd_info;

    ptr_usbd_info = USBD_GetInfo(husbd);

    if (ptr_usbd_info == NULL)
    {
        return;
    }

    if (ptr_usbd_info->ptr_rw_info == NULL)
    {
        return;
    }

    USBD_MemCopy((uint8_t *)ptr_usbd_info->ptr_rw_info->setup_packet, (uint8_t *)USBD_BUF_BASE, 8U);
    ptr_usbd_info->ptr_rw_info->setup_received = 1U;

    if (ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL)
    {
        ptr_usbd_info->ptr_rw_info->cb_endpoint_event(0U, ARM_USBD_EVENT_SETUP);
    }

}

/**
  \fn          void USBD_ResetCallback(const USBD_HandleTypeDef *husbd)
  \brief       USB Reset callback.
  \param[in]   husbd USBD handle
  */
void USBD_ResetCallback(const USBD_HandleTypeDef *husbd)
{
    const USBD_Info_t *ptr_usbd_info;

    ptr_usbd_info = USBD_GetInfo(husbd);

    if (ptr_usbd_info == NULL)
    {
        return;
    }

    if (ptr_usbd_info->ptr_rw_info == NULL)
    {
        return;
    }

    // Clear Endpoints information
    memset((void *)ptr_usbd_info->ptr_rw_info->ep_info, 0U, 2 * USBD_MAX_ENDPOINT_NUM * sizeof(EP_Info_t));

    // Reset USBD state information
    ptr_usbd_info->ptr_rw_info->usbd_state.speed  = ARM_USB_SPEED_FULL;
    ptr_usbd_info->ptr_rw_info->usbd_state.active = 0U;

    if (ptr_usbd_info->ptr_rw_info->cb_device_event != NULL)
    {
        ptr_usbd_info->ptr_rw_info->cb_device_event(ARM_USBD_EVENT_RESET);
    }

    // Configure Endpoint 0 OUT
    (void)USBDn_EndpointConfigure(ptr_usbd_info, 0x00U, ARM_USB_ENDPOINT_CONTROL, USBD_EP0_MAX_PACKET_SIZE);

    // Configure Endpoint 0 IN
    (void)USBDn_EndpointConfigure(ptr_usbd_info, 0x80U, ARM_USB_ENDPOINT_CONTROL, USBD_EP0_MAX_PACKET_SIZE);

    // After reset we consider USB as active
    ptr_usbd_info->ptr_rw_info->usbd_state.active = 1U;
}


// USBD IRQ handler
NVT_ITCM void USBDn_IRQHandler(const USBD_Info_t *ptr_usbd_info)
{
    uint32_t volatile u32IntSts = USBD_GET_INT_FLAG();
    uint32_t volatile u32EpIntSts = USBD_GET_EP_INT_FLAG();
    uint32_t volatile u32State = USBD_GET_BUS_STATE();
    uint32_t event = 0;

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            event |= ARM_USBD_EVENT_VBUS_ON;
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            event |= ARM_USBD_EVENT_VBUS_OFF;
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_WAKEUP)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            USBD_ResetCallback(ptr_usbd_info->ptr_USBD);
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            event |= ARM_USBD_EVENT_SUSPEND;
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME)
        {
            event |= ARM_USBD_EVENT_RESUME;
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }
    }

    if ((ptr_usbd_info->ptr_rw_info->cb_device_event != NULL) && (event != 0U))
    {
        ptr_usbd_info->ptr_rw_info->cb_device_event(event);
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);
            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);
            ptr_usbd_info->ptr_USBD->EP[EP0].CFGP &= ~USBD_CFGP_SSTALL_Msk;
            ptr_usbd_info->ptr_USBD->EP[EP1].CFGP &= ~USBD_CFGP_SSTALL_Msk;
            USBD_SetupStageCallback(ptr_usbd_info->ptr_USBD);
        }

        // EP events
        if (u32EpIntSts & USBD_EPINTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP0);
            // control OUT
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 0U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP1);
            // control IN
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 0U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP2);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 1U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP3);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 1U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP4);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 2U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP5);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 2U);

        }

        if (u32EpIntSts & USBD_EPINTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP6);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 3U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP7);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 3U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP8)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP8);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 4U);

        }

        if (u32EpIntSts & USBD_EPINTSTS_EP9)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP9);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 4U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP10)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP10);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 5U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP11)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP11);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 5U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP12)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP12);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 6U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP13)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP13);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 6U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP14)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP14);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 7U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP15)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP15);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 7U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP16)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP16);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 8U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP17)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP17);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 8U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP18)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP18);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 9U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP19)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP19);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 9U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP20)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP20);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 10U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP21)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP21);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 10U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP22)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP22);
            USBD_DataOutStageCallback(ptr_usbd_info->ptr_USBD, 11U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP23)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP23);
            USBD_DataInStageCallback(ptr_usbd_info->ptr_USBD, 11U);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP24)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP24);
        }
    }
}

// End USBD Interface
#if (RTE_USBD0)
NVT_ITCM void USBD_IRQHandler(void)
{
    const USBD_Info_t *ptr_usbd_info = NULL;
    ptr_usbd_info = USBD_GetInfo(USBD);

    if (ptr_usbd_info)
    {
        USBDn_IRQHandler(ptr_usbd_info);
    }
}

#endif


// Local driver functions definitions (for instances)
#if (RTE_USBD0)
    FUNCS_DEFINE(0)
#endif

// Global driver structures ****************************************************
#if (RTE_USBD0)
    USBD_DRIVER(0)
#endif
#endif  // DRIVER_CONFIG_VALID

/*! \endcond */
