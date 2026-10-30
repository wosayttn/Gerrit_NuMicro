/**************************************************************************//**
 * @file     rndis_priv.h
 * @brief    USB Host RNDIS MVP private definitions.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef _USBH_RNDIS_PRIV_H_
#define _USBH_RNDIS_PRIV_H_

#include <string.h>


#include "usbh_rndis.h"

/// @cond HIDDEN_SYMBOLS
/*
 * Debug message
 */
#define RNDIS_ERRMSG     (void)usbh_printf
#ifdef RNDIS_DEBUG
    #define RNDIS_DBGMSG      (void)usbh_printf
#else
    #define RNDIS_DBGMSG(...)
#endif

#define RNDIS_SLOT_SIZE                    2048U
#define RNDIS_RX_SLOT_COUNT                2U
#define RNDIS_TX_SLOT_COUNT                1U
#define RNDIS_CTRL_RESPONSE_SIZE            2048U
#define RNDIS_CTRL_COMMAND_SIZE             64U
#define RNDIS_HOST_MAX_TRANSFER_SIZE        RNDIS_SLOT_SIZE
#define RNDIS_DMA_HEADROOM_UNITS            16U
#define RNDIS_RESOURCE_UNITS                14U

#define RNDIS_CTRL_MAX_POLLS                10U
#define RNDIS_CTRL_INITIAL_BACKOFF           1U
#define RNDIS_CTRL_MAX_BACKOFF               8U
#define RNDIS_CTRL_DEADLINE_TICKS          100U
#define RNDIS_CTRL_ONE_XFER_TIMEOUT         10U
#define RNDIS_CTRL_TIMEOUT_GUARD             1U
#define RNDIS_CTRL_MAX_RECOVERY              3U
#define RNDIS_NOTIFICATION_SIZE               8U
#define RNDIS_CTRL_NOTIFICATION_WAIT_TICKS    4U

#define RNDIS_MSG_PACKET             0x00000001UL
#define RNDIS_MSG_INITIALIZE         0x00000002UL
#define RNDIS_MSG_QUERY              0x00000004UL
#define RNDIS_MSG_SET                0x00000005UL
#define RNDIS_MSG_INITIALIZE_CMPLT   0x80000002UL
#define RNDIS_MSG_QUERY_CMPLT        0x80000004UL
#define RNDIS_MSG_SET_CMPLT          0x80000005UL
#define RNDIS_STATUS_SUCCESS         0x00000000UL
#define RNDIS_MEDIUM_802_3           0x00000000UL
#define RNDIS_MEDIA_CONNECTED         0x00000000UL
#define RNDIS_PACKET_FILTER           0x0000000BUL

#define OID_GEN_SUPPORTED_LIST        0x00010101UL
#define OID_GEN_HARDWARE_STATUS       0x00010102UL
#define OID_GEN_MEDIA_IN_USE          0x00010104UL
#define OID_GEN_MAXIMUM_FRAME_SIZE    0x00010106UL
#define OID_GEN_CURRENT_PACKET_FILTER 0x0001010EUL
#define OID_GEN_MAXIMUM_TOTAL_SIZE    0x00010111UL
#define OID_GEN_MEDIA_CONNECT_STATUS  0x00010114UL
#define OID_802_3_CURRENT_ADDRESS     0x01010102UL

/**
 * @brief Descriptor-derived interface and endpoint information for one RNDIS pair.
 */
typedef struct
{
    uint8_t comm_ifnum;       /**< Communication/control interface number. */
    uint8_t data_ifnum;       /**< Data interface number associated by Union descriptor. */
    uint8_t bulk_in_addr;     /**< Bulk-IN endpoint address. */
    uint8_t bulk_out_addr;    /**< Bulk-OUT endpoint address. */
    uint8_t int_in_addr;      /**< Optional interrupt-IN notification endpoint address; zero when absent. */
    uint16_t bulk_in_mps;     /**< Bulk-IN endpoint `wMaxPacketSize` value. */
    uint16_t bulk_out_mps;    /**< Bulk-OUT endpoint `wMaxPacketSize` value. */
    uint16_t int_in_mps;      /**< Interrupt-IN endpoint `wMaxPacketSize` value. */
    uint8_t int_in_interval;  /**< Interrupt-IN polling interval from the endpoint descriptor. */
} RNDIS_PAIR_INFO_T;

typedef enum
{
    RNDIS_SLOT_FREE = 0,
    RNDIS_SLOT_FILLING,
    RNDIS_SLOT_READY,
    RNDIS_SLOT_DMA_OWNED,
    RNDIS_SLOT_COMPLETED,
    RNDIS_SLOT_STOPPING
} RNDIS_SLOT_STATE_E;

typedef enum
{
    RNDIS_CTRL_IDLE = 0,
    RNDIS_CTRL_WAIT_INITIALIZE,
    RNDIS_CTRL_WAIT_SUPPORTED,
    RNDIS_CTRL_WAIT_QUERY,
    RNDIS_CTRL_WAIT_SET
} RNDIS_CTRL_STATE_E;

typedef enum
{
    RNDIS_CTRL_WAIT_NONE = 0,
    RNDIS_CTRL_WAIT_NOTIFICATION,
    RNDIS_CTRL_WAIT_RESPONSE
} RNDIS_CTRL_WAIT_E;

/**
 * @brief Fixed-size DMA storage owned by one RNDIS transfer slot.
 */
typedef struct
{
    uint8_t data[RNDIS_SLOT_SIZE];
} RNDIS_SLOT_BUFFER_T;

/**
 * @brief One fixed-size RNDIS RX or TX DMA transfer slot.
 */
typedef struct
{
    RNDIS_SLOT_BUFFER_T *buffer;             /**< DMA-capable slot buffer owned by the session. */
    UTR_T *utr;                              /**< Host-controller transfer request owned by the slot. */
    volatile RNDIS_SLOT_STATE_E state;       /**< Slot ownership and processing state. */
    uint32_t io_generation;                  /**< Private I/O epoch captured at submission. */
    uint32_t actual_length;                  /**< Actual USB transfer length reported on completion. */
    uint32_t expected_wire_length;           /**< RNDIS wire message length submitted for TX. */
    uint32_t submit_id;                      /**< Non-zero TX submission identity, or zero when unused. */
    int status;                              /**< USB transfer completion status. */
} RNDIS_SLOT_T;

/**
 * @brief Private state and resources for one live RNDIS USB function.
 * @details The session is owned by the RNDIS driver. Public callers may access
 *          it only through the opaque RNDIS_SESSION_T API from usbh_rndis.h.
 */
struct rndis_session_t
{
    UDEV_T *udev;                            /**< USB device owning this RNDIS function. */
    IFACE_T *iface_comm;                     /**< Claimed RNDIS communication interface. */
    IFACE_T *iface_data;                     /**< Claimed RNDIS data interface. */
    EP_INFO_T *ep_bulk_in;                   /**< Validated bulk-IN endpoint object. */
    EP_INFO_T *ep_bulk_out;                  /**< Validated bulk-OUT endpoint object. */
    EP_INFO_T *ep_notification;              /**< Optional interrupt-IN notification endpoint object. */
    struct rndis_session_t *next;            /**< Next session in the driver-owned live-session list. */
    RNDIS_RX_CB_FUNC *rx_callback;           /**< Worker-context callback for accepted Ethernet frames. */
    RNDIS_STATE_E state;                     /**< Public RNDIS session state. */
    RNDIS_CTRL_STATE_E ctrl_state;           /**< Encapsulated-control transaction state. */
    RNDIS_CTRL_WAIT_E ctrl_wait;             /**< Current control response wait mode. */
    RNDIS_FAILURE_E failure_reason;          /**< Last retained control-plane failure reason. */
    uint32_t generation;                     /**< Immutable public session identity. */
    volatile uint32_t io_generation;         /**< Private non-zero UTR/callback epoch. */
    uint32_t request_id;                     /**< Current non-zero RNDIS control request identifier. */
    uint32_t pending_oid;                    /**< OID associated with the current QUERY or SET transaction. */
    uint32_t query_index;                    /**< Index of the next required OID query. */
    uint32_t deadline;                       /**< Absolute tick deadline for the current control transaction. */
    uint32_t next_get_tick;                  /**< Absolute tick at which the next GET request may run. */
    uint32_t recovery_count;                 /**< Number of bounded control recovery attempts. */
    uint32_t poll_count;                     /**< GET response requests submitted in this transaction. */
    uint32_t payload_mtu;                    /**< Negotiated maximum Ethernet payload size in bytes. */
    uint32_t max_frame;                      /**< Maximum Ethernet frame size, including the 14-byte header. */
    uint32_t max_total;                      /**< Device-reported maximum Ethernet total size in bytes. */
    uint32_t device_max_transfer_size;       /**< Device-reported maximum RNDIS transfer size in bytes. */
    uint32_t tx_max_transfer_size;           /**< Effective host TX transfer limit in bytes. */
    uint8_t comm_ifnum;                      /**< Communication interface number copied from the pair. */
    uint8_t data_ifnum;                      /**< Data interface number copied from the pair. */
    uint8_t bulk_in_addr;                    /**< Bulk-IN endpoint address copied from the pair. */
    uint8_t bulk_out_addr;                   /**< Bulk-OUT endpoint address copied from the pair. */
    uint8_t int_in_addr;                     /**< Optional interrupt-IN endpoint address copied from the pair. */
    uint16_t bulk_in_mps;                    /**< Bulk-IN endpoint maximum packet size. */
    uint16_t bulk_out_mps;                   /**< Bulk-OUT endpoint maximum packet size. */
    uint16_t int_in_mps;                     /**< Interrupt-IN endpoint maximum packet size. */
    uint8_t int_in_interval;                 /**< Interrupt-IN endpoint polling interval. */
    uint8_t stopping;                        /**< Non-zero while I/O is being stopped or resources released. */
    uint8_t resources_ready;                /**< Non-zero when control, RX, and TX resources are allocated. */
    uint8_t media_connected;                 /**< Non-zero after successful connected-media OID validation. */
    uint8_t *notification_buffer;            /**< DMA buffer for the interrupt notification transfer. */
    UTR_T *notification_utr;                 /**< UTR used by the interrupt notification endpoint. */
    uint32_t notification_io_generation;     /**< I/O epoch captured when notification UTR was armed. */
    volatile uint32_t notification_request_id; /**< Control request ID associated with the armed notification. */
    volatile uint32_t notification_event_request_id; /**< Request ID captured when notification completion was published. */
    volatile uint8_t notification_active;    /**< Non-zero while the notification UTR is owned by USB DMA. */
    volatile uint8_t notification_event;     /**< Non-zero when an IRQ callback published a notification event. */
    volatile int notification_status;        /**< USB status captured by the notification callback. */
    volatile uint32_t notification_length;   /**< Actual notification transfer length captured by the callback. */
    volatile uint8_t notification_header[RNDIS_NOTIFICATION_SIZE]; /**< Bounded notification header snapshot. */
    uint8_t mac[6];                          /**< Negotiated six-byte Ethernet MAC address. */
    uint8_t *ctrl_command;                   /**< DMA-capable encapsulated-control command buffer. */
    uint8_t *ctrl_response;                  /**< DMA-capable encapsulated-control response buffer. */
    RNDIS_SLOT_T rx[RNDIS_RX_SLOT_COUNT];   /**< Fixed RX slot array owned by this session. */
    RNDIS_SLOT_T tx[RNDIS_TX_SLOT_COUNT];   /**< Fixed TX slot array owned by this session. */
    RNDIS_STATS_T stats;                     /**< Accumulated RX, TX, notification, and transfer counters. */
    RNDIS_RX_DIAG_T rx_diagnostic;           /**< Last RX diagnostic snapshot; written only by the deferred RX worker. */
    uint32_t tx_completion_sequence;         /**< Non-zero sequence for the latest published TX outcome. */
    uint32_t tx_submit_sequence;             /**< Monotonic allocator sequence for TX submission IDs. */
    uint32_t tx_last_submit_id;              /**< Submission ID associated with the latest TX outcome. */
    int32_t tx_last_status;                  /**< USB status associated with the latest TX outcome. */
    uint32_t tx_last_actual_length;          /**< Actual USB length associated with the latest TX outcome. */
    uint32_t tx_last_expected_wire_length;   /**< Expected RNDIS wire length associated with the latest TX outcome. */
    uint8_t tx_abort_pending;                /**< Reserved compatibility flag; normal RNDIS TX reports zero. */
};

uint32_t rndis_get_le32(uint8_t const *data);
void rndis_put_le32(uint8_t *data, uint32_t value);
int rndis_find_pair(UDEV_T const *udev, int comm_ifnum, int data_ifnum, RNDIS_PAIR_INFO_T *pair);
void rndis_record_stale_completion(RNDIS_SESSION_T *session);

int rndis_ctrl_start(RNDIS_SESSION_T *session);
void rndis_ctrl_poll(RNDIS_SESSION_T *session);
void rndis_ctrl_stop(RNDIS_SESSION_T *session);
int rndis_ctrl_allocate_notification(RNDIS_SESSION_T *session);
void rndis_ctrl_free_notification(RNDIS_SESSION_T *session);

int rndis_bulk_allocate(RNDIS_SESSION_T *session);
void rndis_bulk_free(RNDIS_SESSION_T *session);
void rndis_bulk_poll(RNDIS_SESSION_T *session);
int32_t rndis_bulk_send(RNDIS_SESSION_T *session, uint8_t const *frame, uint16_t frame_len,
                        uint32_t *submit_id);
int32_t rndis_bulk_get_tx_outcome(RNDIS_SESSION_T const *session,
                                  RNDIS_TX_OUTCOME_T *outcome);

uint32_t rndis_enter_critical(void);
void rndis_exit_critical(uint32_t primask);

/// @endcond HIDDEN_SYMBOLS

#endif  /* _USBH_RNDIS_PRIV_H_ */
