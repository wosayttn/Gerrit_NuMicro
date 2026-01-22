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
#include "NuMicro.h"
#include "Driver_I2C.h"
#include <string.h>

/*
    1. The implementation does not include pin/clock settings, which need to be configured within the application.
    2. Does not support thread-safety or DMA.
    3. If the master's data transfer exceeds the specified size, UI2C_SlaveReceive will discard any extra received data.
    4. If the master's data request exceeds the specified size, UI2C_SlaveTransmit will send dummy data (all zeros).
*/

#ifdef _RTE_
    #include "RTE_Components.h"
#endif
/* Project can define PRJ_RTE_DEVICE_HEADER macro to include private or global RTE_Device.h. */
#ifdef   PRJ_RTE_DEVICE_HEADER
    #include PRJ_RTE_DEVICE_HEADER
#else
    #include "RTE_Device/RTE_Device.h"
#endif

#ifndef I2C_MAX_PORTS_NUM
    #define I2C_MAX_PORTS_NUM      (2U)
#endif

#if (RTE_I2C3 == 1) || (RTE_I2C4 == 1)

#if defined (RTE_Driver_USART) || defined (RTE_Driver_SPI)
#if (RTE_I2C3 == 1) && (RTE_USART10 || RTE_SPI3)
    #error "USCI0 is used by multiple CMSIS Drivers! Please check RTE device configuration to fix it."
#endif

#if (RTE_I2C4 == 1) && (RTE_USART11 || RTE_SPI4)
    #error "USCI1 is used by multiple CMSIS Drivers! Please check RTE device configuration to fix it."
#endif
#endif

#define SLV_10BIT_ADDR (0x1E<<2)             //1111+0xx+r/w

typedef void (*I2C_FUNC)(uint32_t port);

// Driver status
typedef struct
{
    uint8_t                       initialized  : 1;       // Initialized status: 0 - not initialized, 1 - initialized
    uint8_t                       powered      : 1;       // Power status:       0 - not powered,     1 - powered
    uint8_t                       reserved     : 6;       // Reserved (for padding)
} DriverStatus_t;

// Instance run-time information (RW)
typedef struct
{
    I2C_FUNC                      i2c_handler_fn;
    ARM_I2C_STATUS                status;
    ARM_I2C_SignalEvent_t         cb_event;               // Event callback
    DriverStatus_t                drv_status;             // Driver status
    uint8_t                       xfer_no_stop;           // Transfer not generating STOP: 0 - not generating STOP, not 0 - generating STOP
    volatile uint8_t              xfer_abort;             // Transfer abort staus: 1 - abort not done might be in progress, 0 - abort done
    uint32_t                      xfer_size;              // Requested transfer size (in bytes)
    uint32_t                      xfer_size_tx;
    uint32_t                      xfer_size_rx;
    uint32_t                      xfer_remain;
    uint32_t                      xfer_remain_tx;
    uint32_t                      xfer_remain_rx;
    const    uint8_t *volatile    slave_xfer_tx_data;     // Pointer to transmit data (for Slave only)
    uint8_t *volatile             slave_xfer_rx_data;     // Pointer to receive  data (for Slave only)
    uint16_t                      slave_xfer_tx_num;      // Requested number of bytes to transmit
    uint16_t                      slave_xfer_rx_num;      // Requested number of bytes to receive
    const    uint8_t *volatile    master_xfer_tx_data;    // Pointer to transmit data (for Master only)
    uint8_t *volatile             master_xfer_rx_data;    // Pointer to receive  data (for Master only)
    uint16_t                      master_xfer_tx_num;     // Requested number of bytes to transmit
    uint16_t                      master_xfer_rx_num;     // Requested number of bytes to receive
    volatile uint8_t              master_xfer_addrH;
    volatile uint8_t              master_xfer_addrL;
    uint32_t                      master_prev_status;
    volatile enum UI2C_MASTER_EVENT master_event;
    volatile enum UI2C_SLAVE_EVENT  slave_event;
} RW_Info_t;

// Instance compile-time information (RO)
// also contains pointer to run-time information
typedef struct
{
    UI2C_T     *i2c;
    IRQn_Type  IRQn;
} RO_Info_t;

// Instance information
typedef struct
{
    const RO_Info_t  *ptr_ro_info;      // Pointer to compile-time information (RO)
    RW_Info_t  *ptr_rw_info;            // Pointer to run-time information (RW)
} I2C_Info_t;

// Macro for declaring functions (for instances)
#define FUNCS_DECLARE(n)                                                                                       \
    static  int32_t                 I2C##n##_Initialize      (ARM_I2C_SignalEvent_t cb_event);                     \
    static  int32_t                 I2C##n##_Uninitialize    (void);                                               \
    static  int32_t                 I2C##n##_PowerControl    (ARM_POWER_STATE state);                              \
    static  int32_t                 I2C##n##_MasterTransmit  (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending); \
    static  int32_t                 I2C##n##_MasterReceive   (uint32_t addr,       uint8_t *data, uint32_t num, bool xfer_pending); \
    static  int32_t                 I2C##n##_SlaveTransmit   (               const uint8_t *data, uint32_t num);   \
    static  int32_t                 I2C##n##_SlaveReceive    (                     uint8_t *data, uint32_t num);   \
    static  int32_t                 I2C##n##_GetDataCount    (void);                                               \
    static  int32_t                 I2C##n##_Control         (uint32_t control, uint32_t arg);                     \
    static  ARM_I2C_STATUS          I2C##n##_GetStatus       (void);

// Macro for defining functions (for instances)
#define FUNCS_DEFINE(n)                                                                                       \
    static  int32_t                 I2C##n##_Initialize      (ARM_I2C_SignalEvent_t cb_event)                     { return I2Cn_Initialize    (n, cb_event); }                         \
    static  int32_t                 I2C##n##_Uninitialize    (void)                                               { return I2Cn_Uninitialize    (n); }                         \
    static  int32_t                 I2C##n##_PowerControl    (ARM_POWER_STATE state)                                                { return I2Cn_PowerControl    (n, state); }                         \
    static  int32_t                 I2C##n##_MasterTransmit  (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)  { return I2Cn_MasterTransmit  (n, addr, data, num, xfer_pending); } \
    static  int32_t                 I2C##n##_MasterReceive   (uint32_t addr,       uint8_t *data, uint32_t num, bool xfer_pending)  { return I2Cn_MasterReceive   (n, addr, data, num, xfer_pending); } \
    static  int32_t                 I2C##n##_SlaveTransmit   (               const uint8_t *data, uint32_t num)                     { return I2Cn_SlaveTransmit   (n, data, num); }                     \
    static  int32_t                 I2C##n##_SlaveReceive    (                     uint8_t *data, uint32_t num)                     { return I2Cn_SlaveReceive    (n, data, num); }                     \
    static  int32_t                 I2C##n##_GetDataCount    (void)                                                                 { return I2Cn_GetDataCount    (n); }                                \
    static  int32_t                 I2C##n##_Control         (uint32_t control, uint32_t arg)                                       { return I2Cn_Control         (n, control, arg); }                  \
    static  ARM_I2C_STATUS          I2C##n##_GetStatus       (void)                                                                 { return I2Cn_GetStatus       (n); }

// Macro for defining driver structures (for instances)
#define I2C_DRIVER(n, m)                   \
    ARM_DRIVER_I2C Driver_I2C##m = {        \
                                            I2C_GetVersion,                       \
                                            I2C_GetCapabilities,                  \
                                            I2C##n##_Initialize,                  \
                                            I2C##n##_Uninitialize,                \
                                            I2C##n##_PowerControl,                \
                                            I2C##n##_MasterTransmit,              \
                                            I2C##n##_MasterReceive,               \
                                            I2C##n##_SlaveTransmit,               \
                                            I2C##n##_SlaveReceive,                \
                                            I2C##n##_GetDataCount,                \
                                            I2C##n##_Control,                     \
                                            I2C##n##_GetStatus                    \
                                   };


#define ARM_I2C_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_I2C_API_VERSION,
    ARM_I2C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities =
{
    1U,                           // 10-bit addressing supported
    0U                            // Reserved (must be zero)
};

// Local functions prototypes
static ARM_DRIVER_VERSION       I2C_GetVersion(void);
static ARM_I2C_CAPABILITIES     I2C_GetCapabilities(void);
static int32_t                  I2Cn_Initialize(uint32_t port, ARM_I2C_SignalEvent_t cb_event);
static int32_t                  I2Cn_Uninitialize(uint32_t port);
static int32_t                  I2Cn_PowerControl(uint32_t port, ARM_POWER_STATE state);
static int32_t                  I2Cn_MasterTransmit(uint32_t port, uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  I2Cn_MasterReceive(uint32_t port, uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  I2Cn_SlaveTransmit(uint32_t port, const uint8_t *data, uint32_t num);
static int32_t                  I2Cn_SlaveReceive(uint32_t port, uint8_t *data, uint32_t num);
static int32_t                  I2Cn_GetDataCount(uint32_t port);
static int32_t                  I2Cn_Control(uint32_t port, uint32_t control, uint32_t arg);
static ARM_I2C_STATUS           I2Cn_GetStatus(uint32_t port);
static void                     I2C_MasterTx(uint32_t port);
static void                     I2C_MasterRx(uint32_t port);
static void                     I2C_SlaveTRx(uint32_t port);

#if (RTE_I2C3 == 1)
static const RO_Info_t i2c0_ro_info =
{
    (UI2C_T *)USCI0_BASE, USCI0_IRQn,
};
static RW_Info_t i2c0_rw_info;
FUNCS_DECLARE(0)
FUNCS_DEFINE(0)
I2C_DRIVER(0, 3)
#endif

#if (RTE_I2C4 == 1)
static const RO_Info_t i2c1_ro_info =
{
    (UI2C_T *)USCI1_BASE, USCI1_IRQn,
};
static RW_Info_t i2c1_rw_info;
FUNCS_DECLARE(1)
FUNCS_DEFINE(1)
I2C_DRIVER(1, 4)
#endif

static const I2C_Info_t i2c_info[I2C_MAX_PORTS_NUM] =
{
#if (RTE_I2C3 == 1)
    {&i2c0_ro_info, &i2c0_rw_info},
#else
    {NULL, NULL},
#endif

#if (RTE_I2C4 == 1)
    {&i2c1_ro_info, &i2c1_rw_info},
#else
    {NULL, NULL},
#endif

};


//
//  Functions
//

static ARM_DRIVER_VERSION I2C_GetVersion(void)
{
    return DriverVersion;
}

static ARM_I2C_CAPABILITIES I2C_GetCapabilities(void)
{
    return DriverCapabilities;
}

static int32_t I2Cn_Initialize(uint32_t port, ARM_I2C_SignalEvent_t cb_event)
{
    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Clear run-time info
    memset((void *)i2c_info[port].ptr_rw_info, 0, sizeof(RW_Info_t));
    i2c_info[port].ptr_rw_info->cb_event = cb_event;
    i2c_info[port].ptr_rw_info->drv_status.initialized = 1;
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_Uninitialize(uint32_t port)
{
    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Clear run-time info
    memset((void *)i2c_info[port].ptr_rw_info, 0, sizeof(RW_Info_t));
    i2c_info[port].ptr_rw_info->cb_event = NULL;
    i2c_info[port].ptr_rw_info->drv_status.initialized = 0;
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_PowerControl(uint32_t port, ARM_POWER_STATE state)
{
    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
            CLK->APBCLK1 &= ~(1U << CLK_APBCLK1_USCI0CKEN_Pos);
            NVIC_DisableIRQ(i2c_info[port].ptr_ro_info->IRQn);
            i2c_info[port].ptr_rw_info->drv_status.powered = 0U;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            if (i2c_info[port].ptr_rw_info->drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            i2c_info[port].ptr_rw_info->drv_status.powered = 1U;
            CLK->APBCLK1 |= (1U << CLK_APBCLK1_USCI0CKEN_Pos);
            NVIC_EnableIRQ(i2c_info[port].ptr_ro_info->IRQn);
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}


static int32_t I2Cn_MasterTransmit(uint32_t port, uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    ARM_I2C_STATUS status;
    UI2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((data == NULL) || (num == 0U) || (num > (uint32_t)UINT16_MAX) ||
            ((addr & ~((uint32_t)ARM_I2C_ADDRESS_10BIT | (uint32_t)ARM_I2C_ADDRESS_GC)) > 0x3FFU))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (addr & ARM_I2C_ADDRESS_10BIT)
    {
        i2c_info[port].ptr_rw_info->master_xfer_addrL = addr & 0xFF;
        i2c_info[port].ptr_rw_info->master_xfer_addrH = (((addr >> 8) & 0x03) | SLV_10BIT_ADDR);
    }
    else
    {
        i2c_info[port].ptr_rw_info->master_xfer_addrL = addr & 0x7F;
        i2c_info[port].ptr_rw_info->master_xfer_addrH = 0;
    }

    i2c_info[port].ptr_rw_info->master_xfer_tx_data = data;
    i2c_info[port].ptr_rw_info->master_xfer_tx_num = num;
    i2c_info[port].ptr_rw_info->xfer_no_stop = xfer_pending;
    i2c_info[port].ptr_rw_info->i2c_handler_fn = I2C_MasterTx;
    // set xfer_size to 0 for I2Cn_GetDataCount
    i2c_info[port].ptr_rw_info->xfer_size = 0;
    //
    status.busy = 1;
    status.mode = 1; // 1=Master
    status.direction = 0; // 0=Transmitter, 1=Receiver
    i2c_info[port].ptr_rw_info->status = status;
    i2c_info[port].ptr_rw_info->master_event = MASTER_SEND_START;
    UI2C_ENABLE_PROT_INT(i2c, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk
                               | UI2C_PROTIEN_ARBLOIEN_Msk | UI2C_PROTIEN_ERRIEN_Msk));
    i2c->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_STA);
    UI2C_START(i2c);
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_MasterReceive(uint32_t port, uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    ARM_I2C_STATUS status;
    UI2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((data == NULL) || (num == 0U) || (num > (uint32_t)UINT16_MAX) ||
            ((addr & ~((uint32_t)ARM_I2C_ADDRESS_10BIT | (uint32_t)ARM_I2C_ADDRESS_GC)) > 0x3FFU))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (addr & ARM_I2C_ADDRESS_10BIT)
    {
        i2c_info[port].ptr_rw_info->master_xfer_addrL = addr & 0xFF;
        i2c_info[port].ptr_rw_info->master_xfer_addrH = (((addr >> 8) & 0x03) | SLV_10BIT_ADDR);
    }
    else
    {
        i2c_info[port].ptr_rw_info->master_xfer_addrL = addr & 0x7F;
        i2c_info[port].ptr_rw_info->master_xfer_addrH = 0;
    }

    i2c_info[port].ptr_rw_info->master_xfer_rx_data = data;
    i2c_info[port].ptr_rw_info->master_xfer_rx_num = num;
    i2c_info[port].ptr_rw_info->xfer_no_stop = xfer_pending;
    i2c_info[port].ptr_rw_info->i2c_handler_fn = I2C_MasterRx;
    // set xfer_size to 0 for I2Cn_GetDataCount
    i2c_info[port].ptr_rw_info->xfer_size = 0;
    //
    status.busy = 1;
    status.mode = 1; // 1=Master
    status.direction = (addr & ARM_I2C_ADDRESS_10BIT) ? 0 : 1; // 0=Transmitter, 1=Receiver
    i2c_info[port].ptr_rw_info->status = status;
    i2c_info[port].ptr_rw_info->master_event = MASTER_SEND_START;
    UI2C_ENABLE_PROT_INT(i2c, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk
                               | UI2C_PROTIEN_ARBLOIEN_Msk | UI2C_PROTIEN_ERRIEN_Msk));
    i2c->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_STA);
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_SlaveTransmit(uint32_t port, const uint8_t *data, uint32_t num)
{
    ARM_I2C_STATUS status;
    UI2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if ((data == NULL) || (num == 0U) || (port >= I2C_MAX_PORTS_NUM))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    i2c_info[port].ptr_rw_info->slave_xfer_tx_data = data;
    i2c_info[port].ptr_rw_info->slave_xfer_tx_num = num;
    i2c_info[port].ptr_rw_info->i2c_handler_fn = I2C_SlaveTRx;
    //
    status.busy = 1;
    status.mode = 0; // 0=Slave
    status.direction = 0; // 0=Transmitter, 1=Receiver
    i2c_info[port].ptr_rw_info->status = status;
    i2c_info[port].ptr_rw_info->slave_event = SLAVE_ADDRESS_ACK;
    UI2C_ENABLE_PROT_INT(i2c, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk
                               | UI2C_PROTIEN_ARBLOIEN_Msk | UI2C_PROTIEN_ERRIEN_Msk));
    i2c->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_SlaveReceive(uint32_t port, uint8_t *data, uint32_t num)
{
    ARM_I2C_STATUS status;
    UI2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if ((data == NULL) || (num == 0U) || (port >= I2C_MAX_PORTS_NUM))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    i2c_info[port].ptr_rw_info->slave_xfer_rx_data = data;
    i2c_info[port].ptr_rw_info->slave_xfer_rx_num = num;
    i2c_info[port].ptr_rw_info->i2c_handler_fn = I2C_SlaveTRx;
    //
    status.busy = 1;
    status.mode = 0; // 0=Slave
    status.direction = 1; // 0=Transmitter, 1=Receiver
    i2c_info[port].ptr_rw_info->status = status;
    i2c_info[port].ptr_rw_info->slave_event = SLAVE_ADDRESS_ACK;
    UI2C_ENABLE_PROT_INT(i2c, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk
                               | UI2C_PROTIEN_ARBLOIEN_Msk | UI2C_PROTIEN_ERRIEN_Msk));
    i2c->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_GetDataCount(uint32_t port)
{
    return i2c_info[port].ptr_rw_info->xfer_size;
}

static int32_t I2Cn_Control(uint32_t port, uint32_t control, uint32_t arg)
{
    UI2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (control)
    {
        case ARM_I2C_OWN_ADDRESS:
            if ((arg & ARM_I2C_ADDRESS_10BIT) != 0U)
            {
                // Own address is a 10-bit address
                UI2C_ENABLE_10BIT_ADDR_MODE(i2c);
            }
            else
            {
                // Own address is a 7-bit address
                UI2C_DISABLE_10BIT_ADDR_MODE(i2c);
            }

            if ((arg & ARM_I2C_ADDRESS_GC) != 0U)
            {
                UI2C_SetSlaveAddr(i2c, 0, (arg & 0x3FF), UI2C_GCMODE_ENABLE);
            }
            else
            {
                // Disable general call
                UI2C_SetSlaveAddr(i2c, 0, (arg & 0x3FF), UI2C_GCMODE_DISABLE);
            }

            break;

        case ARM_I2C_BUS_SPEED:
            switch (arg)
            {
                case ARM_I2C_BUS_SPEED_STANDARD:
                    UI2C_Open(i2c, 100000);
                    break;

                case ARM_I2C_BUS_SPEED_FAST:
                    UI2C_Open(i2c, 400000);
                    break;

                case ARM_I2C_BUS_SPEED_FAST_PLUS:
                    UI2C_Open(i2c, 1000000);
                    break;

                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            break;

        case ARM_I2C_BUS_CLEAR:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_I2C_ABORT_TRANSFER:
            i2c_info[port].ptr_rw_info->xfer_abort = 1;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

static ARM_I2C_STATUS I2Cn_GetStatus(uint32_t port)
{
    return i2c_info[port].ptr_rw_info->status;
}

// End I2C Interface


#if (RTE_I2C3 == 1)

void USCI0_IRQHandler(void)
{
    I2C_FUNC func = i2c_info[0].ptr_rw_info->i2c_handler_fn;

    if (func != NULL)
    {
        func(0);
    }
}
#endif

#if (RTE_I2C4 == 1)

void USCI1_IRQHandler(void)
{
    I2C_FUNC func = i2c_info[1].ptr_rw_info->i2c_handler_fn;

    if (func != NULL)
    {
        func(1);
    }
}
#endif


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void I2C_MasterRx(uint32_t port)
{
    UI2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;
    RW_Info_t *ptr_rw_info = i2c_info[port].ptr_rw_info;
    uint32_t u32Status = UI2C_GET_PROT_STATUS(i2c);
    ARM_I2C_SignalEvent_t cb_event = ptr_rw_info->cb_event;
    uint32_t event = 0;

    if (ptr_rw_info->xfer_abort == 1)
    {
        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        ptr_rw_info->xfer_abort = 0;
        //
        ptr_rw_info->status.busy = 0;

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
        }

        return;
    }

    if (UI2C_GET_TIMEOUT_FLAG(i2c))
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_ClearTimeoutFlag(i2c);
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        uint8_t u8DeviceHAddr = ptr_rw_info->master_xfer_addrH;
        uint8_t u8DeviceLAddr = ptr_rw_info->master_xfer_addrL;
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_STARIF_Msk); /* Clear START INT Flag */
        ptr_rw_info->xfer_remain = ptr_rw_info->master_xfer_rx_num;

        if (ptr_rw_info->master_event == MASTER_SEND_START)
        {
            if (u8DeviceHAddr)
            {
                UI2C_SET_DATA(i2c, (u8DeviceHAddr << 1)); /* Write SLA+W to Register I2CDAT */
                ptr_rw_info->master_event = MASTER_SEND_H_WR_ADDRESS;
            }
            else
            {
                UI2C_SET_DATA(i2c, (u8DeviceLAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
                ptr_rw_info->master_event = MASTER_SEND_ADDRESS;
            }
        }
        else if (ptr_rw_info->master_event == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(i2c, (u8DeviceHAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
            ptr_rw_info->master_event = MASTER_SEND_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */
        uint8_t u8DeviceLAddr = ptr_rw_info->master_xfer_addrL;

        if (ptr_rw_info->master_event == MASTER_SEND_H_WR_ADDRESS)
        {
            UI2C_SET_DATA(i2c, u8DeviceLAddr);
            ptr_rw_info->master_event = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
        }
        else if (ptr_rw_info->master_event == MASTER_SEND_L_ADDRESS)
        {
            ptr_rw_info->master_event = MASTER_SEND_REPEAT_START;
            UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STA));
        }
        else if (ptr_rw_info->master_event == MASTER_SEND_ADDRESS)
        {
            if (ptr_rw_info->master_xfer_rx_num == 1)
            {
                UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
            }
            else
            {
                UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG | UI2C_CTL_AA);
            }

            ptr_rw_info->master_event = MASTER_READ_DATA;
        }
        else if (ptr_rw_info->master_event == MASTER_READ_DATA)
        {
            uint8_t u8Data = UI2C_GET_DATA(i2c);

            if (ptr_rw_info->xfer_remain > 0)
            {
                *(ptr_rw_info->master_xfer_rx_data + ptr_rw_info->xfer_size) = u8Data;
                ptr_rw_info->xfer_size += 1;
                ptr_rw_info->xfer_remain -= 1;
            }

            if (ptr_rw_info->xfer_remain > 1)
            {
                UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG | UI2C_CTL_AA);
            }
            else if (ptr_rw_info->xfer_remain == 1)
            {
                UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
            }
            else
            {
                ptr_rw_info->status.busy = 0;
                event = ARM_I2C_EVENT_TRANSFER_DONE;

                if (ptr_rw_info->xfer_no_stop != 0)
                {
                    i2c->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
                    UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
                }
                else
                {
                    UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
                }
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */

        if (ptr_rw_info->master_event != MASTER_READ_DATA)
        {
            event = ARM_I2C_EVENT_ADDRESS_NACK;
            UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        }
        else
        {
            uint8_t u8Data = UI2C_GET_DATA(i2c);

            if (ptr_rw_info->xfer_remain > 0)
            {
                *(ptr_rw_info->master_xfer_rx_data + ptr_rw_info->xfer_size) = u8Data;
                ptr_rw_info->xfer_size += 1;
                ptr_rw_info->xfer_remain -= 1;
            }

            if (ptr_rw_info->xfer_remain == 0)
            {
                event = ARM_I2C_EVENT_TRANSFER_DONE;

                if (ptr_rw_info->xfer_no_stop != 0)
                {
                    i2c->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
                    UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
                }
                else
                {
                    UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
                }
            }
            else
            {
                event = ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
                UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_STORIF_Msk);  /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ARBLOIF_Msk) == UI2C_PROTSTS_ARBLOIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_ARBLOIF_Msk);  /* Clear Arbitration Lost INT Flag */
        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        event = ARM_I2C_EVENT_ARBITRATION_LOST;
    }
    else if ((u32Status & UI2C_PROTSTS_ERRIF_Msk) == UI2C_PROTSTS_ERRIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_ERRIF_Msk);  /* Clear Error INT Flag */
        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        event = ARM_I2C_EVENT_BUS_ERROR;
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        ptr_rw_info->status.busy = 0;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void I2C_MasterTx(uint32_t port)
{
    UI2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;
    RW_Info_t *ptr_rw_info = i2c_info[port].ptr_rw_info;
    uint32_t u32Status = UI2C_GET_PROT_STATUS(i2c);
    ARM_I2C_SignalEvent_t cb_event = ptr_rw_info->cb_event;
    uint32_t event = 0;
    uint8_t u8Data = 0;

    if (ptr_rw_info->xfer_abort == 1)
    {
        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        ptr_rw_info->xfer_abort = 0;
        //
        ptr_rw_info->status.busy = 0;

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
        }

        return;
    }

    if (UI2C_GET_TIMEOUT_FLAG(i2c))
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_ClearTimeoutFlag(i2c);
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_STARIF_Msk);    /* Clear START INT Flag */
        uint8_t u8DeviceHAddr = ptr_rw_info->master_xfer_addrH;
        uint8_t u8DeviceLAddr = ptr_rw_info->master_xfer_addrL;
        ptr_rw_info->xfer_size = 0;

        if (u8DeviceHAddr)
        {
            UI2C_SET_DATA(i2c, (u8DeviceHAddr << 1)); /* Write SLA+W to Register I2CDAT */
            ptr_rw_info->master_event = MASTER_SEND_H_WR_ADDRESS;
        }
        else
        {
            UI2C_SET_DATA(i2c, (u8DeviceLAddr << 1)); /* Write SLA+W to Register I2CDAT */
            ptr_rw_info->master_event = MASTER_SEND_L_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_ACKIF_Msk);   /* Clear ACK INT Flag */

        /* Event process */
        if (ptr_rw_info->master_event == MASTER_SEND_H_WR_ADDRESS)
        {
            uint8_t u8DeviceLAddr = ptr_rw_info->master_xfer_addrL;
            UI2C_SET_DATA(i2c, u8DeviceLAddr);
            ptr_rw_info->master_event = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
        }
        else
        {
            ptr_rw_info->master_event = MASTER_SEND_DATA;

            if (ptr_rw_info->xfer_size != ptr_rw_info->master_xfer_tx_num)
            {
                u8Data = *(ptr_rw_info->master_xfer_tx_data + ptr_rw_info->xfer_size);
                UI2C_SET_DATA(i2c, u8Data);
                ptr_rw_info->xfer_size += 1;
                UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
            }
            else
            {
                ptr_rw_info->status.busy = 0;
                event = ARM_I2C_EVENT_TRANSFER_DONE;

                if (ptr_rw_info->xfer_no_stop != 0)
                {
                    i2c->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
                    UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
                }
                else
                {
                    UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
                }
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */

        if (ptr_rw_info->master_event != MASTER_SEND_DATA)
        {
            /* SLA+W has been transmitted and NACK has been received */
            event = ARM_I2C_EVENT_ADDRESS_NACK;
        }
        else
        {
            event = ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
        }

        if (cb_event != NULL)
        {
            cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        }

        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_STORIF_Msk);  /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ARBLOIF_Msk) == UI2C_PROTSTS_ARBLOIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_ARBLOIF_Msk);  /* Clear Arbitration Lost INT Flag */
        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        event = ARM_I2C_EVENT_ARBITRATION_LOST;
    }
    else if ((u32Status & UI2C_PROTSTS_ERRIF_Msk) == UI2C_PROTSTS_ERRIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_ERRIF_Msk);  /* Clear Error INT Flag */
        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        event = ARM_I2C_EVENT_BUS_ERROR;
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        ptr_rw_info->status.busy = 0;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
static void I2C_SlaveTRx(uint32_t port)
{
    UI2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;
    RW_Info_t *ptr_rw_info = i2c_info[port].ptr_rw_info;
    uint32_t u32Status = UI2C_GET_PROT_STATUS(i2c);
    uint32_t direction = ptr_rw_info->status.direction;
    ARM_I2C_SignalEvent_t cb_event = ptr_rw_info->cb_event;
    uint32_t event = 0;
    uint8_t u8Data;

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_STARIF_Msk);
        /* Event process */
        ptr_rw_info->slave_event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (ptr_rw_info->slave_event == SLAVE_ADDRESS_ACK)
        {
            if ((i2c->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                /* Own SLA+R has been receive; ACK has been return */
                ptr_rw_info->xfer_size_tx = 0;
                ptr_rw_info->xfer_remain_tx = ptr_rw_info->slave_xfer_tx_num;
                ptr_rw_info->slave_event = SLAVE_SEND_DATA;

                if ((direction == 0) && (ptr_rw_info->xfer_remain_tx > 0))   // Direction: 0=Transmitter, 1=Receiver
                {
                    u8Data = *(ptr_rw_info->slave_xfer_tx_data + ptr_rw_info->xfer_size_tx);
                    UI2C_SET_DATA(i2c, u8Data);
                    ptr_rw_info->xfer_remain_tx -= 1;
                    ptr_rw_info->xfer_size_tx += 1;
                }
                else
                {
                    UI2C_SET_DATA(i2c, 0x00); // dummy
                }
            }
            else
            {
                /* Own SLA+W has been receive; ACK has been return */
                ptr_rw_info->xfer_size_rx = 0;
                ptr_rw_info->xfer_remain_rx = ptr_rw_info->slave_xfer_rx_num;

                if (i2c->PROTCTL & UI2C_PROTCTL_ADDR10EN_Msk)
                {
                    ptr_rw_info->slave_event = SLAVE_GET_DATA;
                    // Address Byte (High)
                    u8Data = (uint8_t)UI2C_GET_DATA(i2c);
                }
                else
                {
                    ptr_rw_info->slave_event = SLAVE_GET_DATA;
                }
            }

            // Address Byte (Low)
            u8Data = (uint8_t)UI2C_GET_DATA(i2c);

            if ((u8Data == 0) && ptr_rw_info->slave_event == SLAVE_GET_DATA)
            {
                event = ARM_I2C_EVENT_GENERAL_CALL;
            }
        }
        else if (ptr_rw_info->slave_event == SLAVE_L_WR_ADDRESS_ACK)
        {
            // Address Byte (Low)
            // u8Data = (uint8_t)UI2C_GET_DATA(i2c);
            ptr_rw_info->slave_event = SLAVE_GET_DATA;
        }
        else if (ptr_rw_info->slave_event == SLAVE_GET_DATA)
        {
            u8Data = UI2C_GET_DATA(i2c);
            ptr_rw_info->xfer_size += 1;

            if (direction == 1)   // Direction: 0=Transmitter, 1=Receiver
            {
                if (ptr_rw_info->xfer_remain_rx > 0)
                {
                    *(ptr_rw_info->slave_xfer_rx_data + ptr_rw_info->xfer_size_rx) = u8Data;
                    ptr_rw_info->xfer_remain_rx -= 1;
                    ptr_rw_info->xfer_size_rx += 1;

                    if (ptr_rw_info->xfer_remain_rx == 0)
                    {
                        event = ARM_I2C_EVENT_TRANSFER_DONE;
                    }
                }
            }
        }
        else if (ptr_rw_info->slave_event == SLAVE_SEND_DATA)
        {
            if ((direction == 0) && (ptr_rw_info->xfer_remain_tx > 0))   // Direction: 0=Transmitter, 1=Receiver
            {
                u8Data = *(ptr_rw_info->slave_xfer_tx_data + ptr_rw_info->xfer_size_tx);
                UI2C_SET_DATA(i2c, u8Data);
                ptr_rw_info->xfer_remain_tx -= 1;
                ptr_rw_info->xfer_size_tx += 1;

                if (ptr_rw_info->xfer_remain_tx == 0)
                {
                    event = ARM_I2C_EVENT_TRANSFER_DONE;
                }
            }
            else
            {
                UI2C_SET_DATA(i2c, 0x00); // dummy
            }
        }

        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_NACKIF_Msk);

        if (ptr_rw_info->slave_event == SLAVE_GET_DATA)
        {
            u8Data = UI2C_GET_DATA(i2c);
            ptr_rw_info->xfer_size += 1;

            if (direction == 1)   // Direction: 0=Transmitter, 1=Receiver
            {
                if (ptr_rw_info->xfer_remain_rx > 0)
                {
                    *(ptr_rw_info->slave_xfer_rx_data + ptr_rw_info->xfer_size_rx) = u8Data;
                    ptr_rw_info->xfer_remain_rx -= 1;
                    ptr_rw_info->xfer_size_rx += 1;

                    if (ptr_rw_info->xfer_remain_rx == 0)
                    {
                        event = ARM_I2C_EVENT_TRANSFER_DONE;
                    }
                }
            }
        }
        else if (ptr_rw_info->slave_event == SLAVE_SEND_DATA)
        {
            // Transfer Done. Received NACK
        }

        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_STORIF_Msk);
        UI2C_SET_CONTROL_REG(i2c, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ARBLOIF_Msk) == UI2C_PROTSTS_ARBLOIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_ARBLOIF_Msk);  /* Clear Arbitration Lost INT Flag */
        event = ARM_I2C_EVENT_ARBITRATION_LOST;
    }
    else if ((u32Status & UI2C_PROTSTS_ERRIF_Msk) == UI2C_PROTSTS_ERRIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(i2c, UI2C_PROTSTS_ERRIF_Msk);  /* Clear Error INT Flag */
        event = ARM_I2C_EVENT_BUS_ERROR;
    }

    if ((event != 0) && (cb_event != NULL))
    {
        if (event & ARM_I2C_EVENT_GENERAL_CALL)
        {
            cb_event(event);
        }
        else
        {
            cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
            ptr_rw_info->status.busy = 0;
        }
    }
}

#endif // #if (RTE_I2C3 == 1) || (RTE_I2C4 == 1)
