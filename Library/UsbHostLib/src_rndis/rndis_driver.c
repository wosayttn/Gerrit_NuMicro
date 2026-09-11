/**************************************************************************//**
 * @file     rndis_driver.c
 * @brief    USB Host RNDIS interface ownership and session lifecycle.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "rndis_priv.h"

/** @addtogroup LIBRARY Library
  @{
*/

/** @addtogroup USBH_Library USBH Library
  @{
*/

/** @addtogroup USBH_EXPORTED_FUNCTIONS USBH Exported Functions
  @{
*/

/// @cond HIDDEN_SYMBOLS

static int rndis_probe(IFACE_T *iface);
static void rndis_disconnect(IFACE_T *iface);

static RNDIS_SESSION_T *g_rndis_list = USBNULL;
static UDEV_DRV_T g_rndis_driver =
{
    rndis_probe,
    rndis_disconnect,
    USBNULL,
    USBNULL
};

/**
 * @brief Allocate the next non-zero public session identity.
 * @return New driver-lifetime session identity.
 * @note The value is monotonic until the 32-bit sequence wraps; zero is
 *       skipped and the identity is never changed for an existing session.
 */
static uint32_t rndis_allocate_session_generation(void)
{
    static uint32_t g_rndis_next_session_generation;

    g_rndis_next_session_generation++;

    if (g_rndis_next_session_generation == 0U)
    {
        g_rndis_next_session_generation++;
    }

    return g_rndis_next_session_generation;
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief Atomically record a completion or notification event that is stale.
 * @param[in,out] session Session whose stale-event counter is incremented.
 * @note This helper is safe for both USB callback and worker contexts. The
 *       exclusive-access retry prevents an IRQ and worker update from losing
 *       either increment.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 17.3<br>
 * <b>Justification:</b> CMSIS exclusive-access intrinsics implement the ARM
 *                       atomic retry required for concurrent stale counters.
 */
void rndis_record_stale_completion(RNDIS_SESSION_T *session)
{
    volatile uint32_t *counter;
    uint32_t value;

    if (session == USBNULL)
    {
        return;
    }

    counter = &session->stats.stale_completions;

    do
    {
        /* cppcheck-suppress misra-c2012-17.3 */
        value = __LDREXW(counter);
        /* cppcheck-suppress misra-c2012-17.3 */
    } while (__STREXW(value + 1U, counter) != 0U);

    __DMB();
}

/// @cond HIDDEN_SYMBOLS

/**
 * @brief Find a session bound to a USB device and data interface number.
 * @param[in] udev USB device to match.
 * @param[in] data_ifnum Data interface number to match.
 * @return Matching live session, or USBNULL when none exists.
 * @note The returned pointer is owned by the driver and is valid only while
 *       the session remains attached.
 */
static RNDIS_SESSION_T *rndis_find_session(UDEV_T const *udev, uint8_t data_ifnum)
{
    RNDIS_SESSION_T *session = g_rndis_list;

    while (session != USBNULL)
    {
        if ((session->udev == udev) && (session->data_ifnum == data_ifnum))
        {
            return session;
        }

        session = session->next;
    }

    return USBNULL;
}

/**
 * @brief Find a session bound to a USB device and communication interface.
 * @param[in] udev USB device to match.
 * @param[in] comm_ifnum Communication interface number to match.
 * @return Matching live session, or USBNULL when none exists.
 * @note The returned pointer is owned by the driver and is valid only while
 *       the session remains attached.
 */
static RNDIS_SESSION_T *rndis_find_comm_session(UDEV_T const *udev, uint8_t comm_ifnum)
{
    RNDIS_SESSION_T *session = g_rndis_list;

    while (session != USBNULL)
    {
        if ((session->udev == udev) && (session->comm_ifnum == comm_ifnum))
        {
            return session;
        }

        session = session->next;
    }

    return USBNULL;
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief Allocate and register a new RNDIS session for a descriptor pair.
 * @param[in] udev USB device containing the RNDIS function.
 * @param[in] pair Validated communication/data interface and endpoint pair.
 * @return Newly registered session, or USBNULL when memory allocation fails.
 * @note The session starts in RNDIS_STATE_ENUMERATED with no transfer
 *       resources allocated.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 11.5<br>
 * <b>Justification:</b> usbh_alloc_mem returns void * for the common DMA pool;
 *                       the returned storage is restored as its RNDIS session
 *                       object type.
 */
static RNDIS_SESSION_T *rndis_create_session(UDEV_T *udev, RNDIS_PAIR_INFO_T const *pair)
{
    RNDIS_SESSION_T *session;

    /* cppcheck-suppress misra-c2012-11.5 */
    session = (RNDIS_SESSION_T *)usbh_alloc_mem((int)sizeof(*session));

    if (session == USBNULL)
    {
        return USBNULL;
    }

    (void)memset(session, 0, sizeof(*session));
    session->udev = udev;
    session->comm_ifnum = pair->comm_ifnum;
    session->data_ifnum = pair->data_ifnum;
    session->bulk_in_addr = pair->bulk_in_addr;
    session->bulk_out_addr = pair->bulk_out_addr;
    session->bulk_in_mps = pair->bulk_in_mps;
    session->bulk_out_mps = pair->bulk_out_mps;
    session->int_in_addr = pair->int_in_addr;
    session->int_in_mps = pair->int_in_mps;
    session->int_in_interval = pair->int_in_interval;
    session->generation = rndis_allocate_session_generation();
    /* This private epoch is invalidated by cleanup; public generation never is. */
    session->io_generation = 1U;
    session->state = RNDIS_STATE_ENUMERATED;
    session->next = g_rndis_list;
    g_rndis_list = session;
    return session;
}

/// @cond HIDDEN_SYMBOLS

/**
 * @brief Remove a session from the global live-session list.
 * @param[in] session Session to unlink.
 * @note This function does not free the session or release its resources;
 *       destruction is completed by rndis_destroy_session().
 */
static void rndis_remove_session(RNDIS_SESSION_T *session)
{
    RNDIS_SESSION_T **link = &g_rndis_list;

    while (*link != USBNULL)
    {
        if (*link == session)
        {
            *link = session->next;
            return;
        }

        link = &(*link)->next;
    }
}

/**
 * @brief Stop, unbind, unlink, and free one RNDIS session.
 * @param[in] session Session being detached.
 * @note All session-owned USB resources are released before the session
 *       allocation is freed. Callers must not retain the pointer afterwards.
 */
static void rndis_destroy_session(RNDIS_SESSION_T *session)
{
    IFACE_T *iface_comm;
    IFACE_T *iface_data;

    session->state = RNDIS_STATE_DETACHED;
    rndis_ctrl_stop(session);
    rndis_bulk_free(session);

    iface_comm = session->iface_comm;
    iface_data = session->iface_data;

    if ((iface_comm != USBNULL) && (iface_comm->context == session))
    {
        iface_comm->context = USBNULL;
    }

    if ((iface_data != USBNULL) && (iface_data->context == session))
    {
        iface_data->context = USBNULL;
    }

    rndis_remove_session(session);
    (void)usbh_memory_used();
    (void)usbh_free_mem(session, (int)sizeof(*session));
}

/**
 * @brief Bind one communication or data interface to an RNDIS session.
 * @param[in,out] iface Interface being claimed by the RNDIS driver.
 * @param[in] pair Descriptor-validated interface and endpoint pair.
 * @return USBH_OK when the interface is bound; otherwise a not-matched or
 *         memory allocation error code.
 * @note The interface context is set to the session on success. Both
 *       interfaces must be bound before the session enters PAIRED state.
 */
static int rndis_bind_interface(IFACE_T *iface, RNDIS_PAIR_INFO_T const *pair)
{
    RNDIS_SESSION_T *session;
    RNDIS_SESSION_T *comm_session;

    if ((iface == USBNULL) || (pair == USBNULL) ||
            ((iface->if_num != pair->comm_ifnum) && (iface->if_num != pair->data_ifnum)) ||
            (iface->context != USBNULL))
    {
        return USBH_ERR_NOT_MATCHED;
    }

    session = rndis_find_session(iface->udev, pair->data_ifnum);
    comm_session = rndis_find_comm_session(iface->udev, pair->comm_ifnum);

    if ((session != USBNULL) && (comm_session != USBNULL) && (session != comm_session))
    {
        return USBH_ERR_NOT_MATCHED;
    }

    if (session == USBNULL)
    {
        session = comm_session;
    }

    if (session == USBNULL)
    {
        session = rndis_create_session(iface->udev, pair);

        if (session == USBNULL)
        {
            return USBH_ERR_MEMORY_OUT;
        }
    }
    else if ((session->comm_ifnum != pair->comm_ifnum) || (session->data_ifnum != pair->data_ifnum) ||
             (session->bulk_in_addr != pair->bulk_in_addr) || (session->bulk_out_addr != pair->bulk_out_addr) ||
             (session->bulk_in_mps != pair->bulk_in_mps) || (session->bulk_out_mps != pair->bulk_out_mps) ||
             (session->int_in_addr != pair->int_in_addr) || (session->int_in_mps != pair->int_in_mps) ||
             (session->int_in_interval != pair->int_in_interval))
    {
        return USBH_ERR_NOT_MATCHED;
    }
    else
    {
        /* Session already exists and matches the pair; nothing to do. */
    }

    if (iface->if_num == pair->comm_ifnum)
    {
        if ((session->iface_comm != USBNULL) && (session->iface_comm != iface))
        {
            return USBH_ERR_NOT_MATCHED;
        }

        session->iface_comm = iface;
    }
    else if (iface->if_num == pair->data_ifnum)
    {
        if ((session->iface_data != USBNULL) && (session->iface_data != iface))
        {
            return USBH_ERR_NOT_MATCHED;
        }

        session->iface_data = iface;
    }
    else
    {
        return USBH_ERR_NOT_MATCHED;
    }

    iface->context = session;

    if ((session->iface_comm != USBNULL) && (session->iface_data != USBNULL))
    {
        session->state = RNDIS_STATE_PAIRED;
        RNDIS_DBGMSG("RNDIS pair bound: VID=0x%04x PID=0x%04x, comm=%u data=%u, int-in=0x%02x MPS=%u interval=%u.\n",
                     session->udev->descriptor.idVendor, session->udev->descriptor.idProduct,
                     session->comm_ifnum, session->data_ifnum, session->int_in_addr,
                     session->int_in_mps, session->int_in_interval);
        (void)usbh_memory_used();
    }

    return USBH_OK;
}

/**
 * @brief Probe one USB interface for a supported RNDIS topology.
 * @param[in] iface Interface presented by the USB host core.
 * @return USBH_OK when the interface is claimed or associated with a session;
 *         otherwise USBH_ERR_NOT_MATCHED or a binding error.
 * @note Called by the USB host driver matching path. Descriptor parsing is
 *       bounded and no bulk resources are allocated until deferred polling.
 */
static int rndis_probe(IFACE_T *iface)
{
    DESC_IF_T const *ifd;
    RNDIS_PAIR_INFO_T pair;
    int ret;

    if ((iface == USBNULL) || (iface->aif == USBNULL) || (iface->aif->ifd == USBNULL))
    {
        return USBH_ERR_NOT_MATCHED;
    }

    ifd = iface->aif->ifd;

    if ((ifd->bInterfaceClass == USB_CLASS_DATA) && (ifd->bInterfaceSubClass == 0U) &&
            (ifd->bInterfaceProtocol == 0U))
    {
        ret = rndis_find_pair(iface->udev, -1, (int32_t)iface->if_num, &pair);
    }
    else if (((ifd->bInterfaceClass == 0xE0U) && (ifd->bInterfaceSubClass == 0x01U) &&
              (ifd->bInterfaceProtocol == 0x03U)) ||
             ((ifd->bInterfaceClass == USB_CLASS_COMM) && (ifd->bInterfaceSubClass == 0x02U) &&
              (ifd->bInterfaceProtocol == 0xFFU)))
    {
        ret = rndis_find_pair(iface->udev, (int32_t)iface->if_num, -1, &pair);
    }
    else
    {
        return USBH_ERR_NOT_MATCHED;
    }

    if (ret != USBH_OK)
    {
        RNDIS_DBGMSG("RNDIS rejected interface %u (descriptor result %d).\n", iface->if_num, ret);
        return USBH_ERR_NOT_MATCHED;
    }

    return rndis_bind_interface(iface, &pair);
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief Disconnect and destroy the session associated with an interface.
 * @param[in] iface Interface being disconnected by the USB host core.
 * @note Called from the USB host driver disconnect path. It is safe when the
 *       interface is NULL or has no RNDIS session context.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 11.5<br>
 * <b>Justification:</b> Interface context is stored as void * by the USB host
 *                       core and is restored as the RNDIS session set at bind.
 */
static void rndis_disconnect(IFACE_T *iface)
{
    RNDIS_SESSION_T *session;

    if (iface == USBNULL)
    {
        return;
    }

    /* cppcheck-suppress misra-c2012-11.5 */
    session = (RNDIS_SESSION_T *)iface->context;

    if (session != USBNULL)
    {
        rndis_destroy_session(session);
    }
}

/**
 * @brief Register the RNDIS class driver with the USB host library.
 * @note Call before usbh_core_init(). Since core initialization resets the
 *       driver table, call usbh_rndis_core_ready() immediately afterwards.
 */
void usbh_rndis_init(void)
{
    g_rndis_list = USBNULL;
    (void)usbh_register_driver(&g_rndis_driver);
}

/**
 * @brief Re-register the RNDIS class driver after USB core initialization.
 * @return Result returned by usbh_register_driver().
 * @note Call after usbh_core_init() and before the first hub-polling call.
 */
int32_t usbh_rndis_core_ready(void)
{
    /* usbh_core_init() resets its driver table; re-register before hub polling. */
    return (int32_t)usbh_register_driver(&g_rndis_driver);
}

/**
 * @brief Run deferred RNDIS control and bulk processing for all live sessions.
 * @note Bare-metal worker-context API. Call periodically from the main loop;
 *       it does not replace the USB host hub polling function and does not
 *       block waiting for control or bulk completion.
 */
void usbh_rndis_poll(void)
{
    RNDIS_SESSION_T *session = g_rndis_list;

    while (session != USBNULL)
    {
        RNDIS_SESSION_T *next = session->next;

        if ((session->stopping == 0U) && (session->iface_comm != USBNULL) &&
                (session->iface_data != USBNULL))
        {
            if ((session->resources_ready == 0U) && (session->state == RNDIS_STATE_PAIRED))
            {
                if (rndis_bulk_allocate(session) != USBH_OK)
                {
                    session->state = RNDIS_STATE_FAILED;
                }
            }

            if (session->resources_ready != 0U)
            {
                if (session->ctrl_state == RNDIS_CTRL_IDLE)
                {
                    (void)rndis_ctrl_start(session);
                }

                rndis_ctrl_poll(session);
                rndis_bulk_poll(session);
            }
        }

        session = next;
    }
}

/**
 * @brief Return the current live RNDIS session list head.
 * @return Current live session, or USBNULL when no RNDIS function is bound.
 * @note The pointer is valid only for the current session lifecycle. Do not
 *       retain or dereference it after detach; pair it with
 *       usbh_rndis_get_session_generation() when detecting replacement.
 */
RNDIS_SESSION_T *usbh_rndis_get_device_list(void)
{
    return g_rndis_list;
}

/**
 * @brief Configure the worker-context Ethernet RX callback for a session.
 * @param[in] session Live RNDIS session from the current device-list snapshot.
 * @param[in] callback Callback to invoke for an accepted Ethernet frame, or
 *                     USBNULL to disable notification.
 * @return RNDIS_OK on success, otherwise RNDIS_ERR_NOT_READY for a NULL session.
 * @note The callback runs from usbh_rndis_poll(), not USB interrupt context.
 *       Its frame pointer refers to a driver-owned RX slot and must not be
 *       retained or freed.
 */
int32_t usbh_rndis_set_rx_callback(RNDIS_SESSION_T *session, RNDIS_RX_CB_FUNC *callback)
{
    if (session == USBNULL)
    {
        return RNDIS_ERR_NOT_READY;
    }

    session->rx_callback = callback;
    return RNDIS_OK;
}

/**
 * @brief Return the current public RNDIS session state.
 * @param[in] session Live session to inspect.
 * @return Current RNDIS_STATE_E value, or RNDIS_STATE_DETACHED for NULL.
 */
RNDIS_STATE_E usbh_rndis_get_state(RNDIS_SESSION_T const *session)
{
    if (session == USBNULL)
    {
        return RNDIS_STATE_DETACHED;
    }

    return session->state;
}

/**
 * @brief Copy the immutable identity of the current live session.
 * @param[in] session Session pointer from the current device-list snapshot.
 * @param[out] generation Destination for the non-zero session identity.
 * @return RNDIS_OK on success, otherwise RNDIS_ERR_NOT_READY.
 * @details A newly created session is assigned a monotonic driver-lifetime identity;
 *          it is never changed by cleanup, recovery, or detach. Zero is skipped when
 *          the 32-bit allocator wraps. The pointer is valid only during its current
 *          lifecycle, so callers must not use this API with a stale detached pointer
 *          and must compare pointer plus identity. Private I/O epochs are not exposed.
 */
int32_t usbh_rndis_get_session_generation(RNDIS_SESSION_T const *session, uint32_t *generation)
{
    if ((session == USBNULL) || (generation == USBNULL))
    {
        return RNDIS_ERR_NOT_READY;
    }

    *generation = session->generation;
    return RNDIS_OK;
}

/**
 * @brief Copy session statistics to caller-owned storage.
 * @param[in] session Live session to inspect.
 * @param[out] stats Destination for the statistics snapshot.
 * @return RNDIS_OK on success, otherwise RNDIS_ERR_NOT_READY for a NULL
 *         session or destination.
 * @note The snapshot is intended for worker-context use and contains counters
 *       accumulated by the driver; it does not reset them.
 */
int32_t usbh_rndis_get_stats(RNDIS_SESSION_T const *session, RNDIS_STATS_T *stats)
{
    uint32_t primask;

    if ((session == USBNULL) || (stats == USBNULL))
    {
        return RNDIS_ERR_NOT_READY;
    }

    primask = rndis_enter_critical();
    *stats = session->stats;
    __DMB();

    rndis_exit_critical(primask);

    return RNDIS_OK;
}

/**
 * @brief Copy and queue one Ethernet frame with a tracked submission ID.
 * @param[in,out] session Running RNDIS session.
 * @param[in] frame Ethernet frame to copy into the driver TX slot.
 * @param[in] frame_len Ethernet frame length in bytes.
 * @param[out] submit_id Destination for the non-zero accepted submission ID.
 * @return RNDIS_OK when accepted; otherwise RNDIS_ERR_NOT_READY,
 *         RNDIS_ERR_FRAME_SIZE, RNDIS_ERR_BUSY, or RNDIS_ERR_PROTOCOL.
 * @note Worker-context API. The frame is copied before return. The ID can be
 *       matched with usbh_rndis_get_tx_outcome() after completion, which proves
 *       only local Host-side USB completion.
 */
int32_t usbh_rndis_send_frame_tracked(RNDIS_SESSION_T *session, uint8_t const *frame,
                                      uint16_t frame_len, uint32_t *submit_id)
{
    int32_t status;

    if (submit_id == USBNULL)
    {
        return RNDIS_ERR_NOT_READY;
    }

    *submit_id = 0U;
    status = rndis_bulk_send(session, frame, frame_len, submit_id);

    if ((status == RNDIS_OK) && (*submit_id == 0U))
    {
        return RNDIS_ERR_PROTOCOL;
    }

    return status;
}

/**
 * @brief Copy the latest normal TX completion outcome.
 * @param[in] session Live session to inspect.
 * @param[out] outcome Destination for the scalar completion snapshot.
 * @return RNDIS_OK on success; otherwise RNDIS_ERR_NOT_READY for invalid
 *         arguments.
 * @note Compare completion_sequence with a snapshot taken before the tracked
 *       send. A successful exact-length completion does not prove peer receipt.
 */
int32_t usbh_rndis_get_tx_outcome(RNDIS_SESSION_T const *session,
                                  RNDIS_TX_OUTCOME_T *outcome)
{
    return rndis_bulk_get_tx_outcome(session, outcome);
}

/**
 * @brief Copy the latest deferred RX validation diagnostic.
 * @param[in] session Live session to inspect.
 * @param[out] diagnostic Destination for the diagnostic snapshot.
 * @return RNDIS_OK on success, otherwise RNDIS_ERR_NOT_READY for invalid
 *         arguments.
 * @note The snapshot is published by the bulk worker and is not cleared by
 *       this function.
 */
int32_t usbh_rndis_get_rx_diagnostic(RNDIS_SESSION_T const *session, RNDIS_RX_DIAG_T *diagnostic)
{
    if ((session == USBNULL) || (diagnostic == USBNULL))
    {
        return RNDIS_ERR_NOT_READY;
    }

    *diagnostic = session->rx_diagnostic;
    return RNDIS_OK;
}

/**
 * @brief Return the last retained control-plane failure reason.
 * @param[in] session Live session to inspect.
 * @return RNDIS_FAILURE_E value, or RNDIS_FAILURE_NONE for NULL.
 */
RNDIS_FAILURE_E usbh_rndis_get_failure(RNDIS_SESSION_T const *session)
{
    if (session == USBNULL)
    {
        return RNDIS_FAILURE_NONE;
    }

    return session->failure_reason;
}

/**
 * @brief Convert a RNDIS failure reason to a constant diagnostic string.
 * @param[in] failure Failure enumeration value.
 * @return Constant ASCII string for the known reason, or "unknown" when the
 *         enumeration value is outside the supported range.
 * @note The returned string is statically stored and must not be modified or
 *       freed.
 */
char const *usbh_rndis_failure_string(RNDIS_FAILURE_E failure)
{
    static char const *const reasons[] =
    {
        "none",
        "allocation",
        "notification-arm",
        "notification-timeout-fallback",
        "notification-transport",
        "notification-protocol",
        "send-transport",
        "send-length",
        "get-transport",
        "get-limit",
        "control-deadline",
        "response-protocol"
    };

    if ((uint32_t)failure >= (uint32_t)(sizeof(reasons) / sizeof(reasons[0])))
    {
        return "unknown";
    }

    return reasons[(uint32_t)failure];
}

/**
 * @brief Copy the negotiated Ethernet MAC address.
 * @param[in] session Running RNDIS session with completed initialization.
 * @param[out] mac Destination buffer of at least six bytes.
 * @return RNDIS_OK on success; otherwise RNDIS_ERR_NOT_READY when the session,
 *         destination, or running state is invalid.
 * @note The address is copied from the session and the caller owns the output
 *       buffer.
 */
int32_t usbh_rndis_get_mac_address(RNDIS_SESSION_T const *session, uint8_t mac[6])
{
    if ((session == USBNULL) || (mac == USBNULL) || (session->state != RNDIS_STATE_RUNNING))
    {
        return RNDIS_ERR_NOT_READY;
    }

    (void)memcpy(mac, session->mac, 6U);
    return RNDIS_OK;
}

/** @} end of group USBH_EXPORTED_FUNCTIONS */

/** @} end of group USBH_Library */

/** @} end of group LIBRARY */