/**************************************************************************//**
 * @file     rndis_ctrl.c
 * @brief    RNDIS encapsulated-control worker state machine.
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

static void rndis_recover(RNDIS_SESSION_T *session);

/**
 * @brief Test whether a tick counter has reached a target tick.
 * @param[in] now Current tick value.
 * @param[in] target Deadline or scheduled target tick.
 * @return 1 when @p now is at or after @p target using signed wrap-safe
 *         subtraction; otherwise 0.
 */
static int rndis_time_due(uint32_t now, uint32_t target)
{
    uint32_t elapsed;

    elapsed = now - target;
    return ((elapsed & 0x80000000U) == 0U) ? 1 : 0;
}

/**
 * @brief Allocate the next non-zero RNDIS control request identifier.
 * @param[in,out] session Session whose request sequence is advanced.
 * @return The newly allocated request identifier.
 * @note The identifier wraps from `UINT32_MAX` to 1; zero is never returned.
 */
static uint32_t rndis_next_request_id(RNDIS_SESSION_T *session)
{
    session->request_id++;

    if (session->request_id == 0U)
    {
        session->request_id = 1U;
    }

    return session->request_id;
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief Clear a latched interrupt notification event atomically.
 * @param[in,out] session Session whose notification state is reset.
 * @note This helper disables interrupts only for the short shared-state update
 *       and is intended for worker context.
 */
static void rndis_notification_clear_event(RNDIS_SESSION_T *session)
{
    uint32_t primask;
    uint32_t index;

    primask = rndis_enter_critical();
    session->notification_event = 0U;
    session->notification_event_request_id = 0U;
    session->notification_status = USBH_OK;
    session->notification_length = 0U;

    for (index = 0U; index < RNDIS_NOTIFICATION_SIZE; index++)
    {
        session->notification_header[index] = 0U;
    }

    __DMB();

    rndis_exit_critical(primask);
}

/**
 * @brief Capture completion of the interrupt notification transfer.
 * @param[in] utr Completed notification transfer request.
 * @note Called from USB interrupt/callback context. The callback copies only
 *       the bounded notification header and defers protocol processing to the
 *       control worker.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 11.5<br>
 * <b>Justification:</b> UTR context is stored as void * by the generic USB
 *                       host layer and is restored as its RNDIS session type.
 */
static void rndis_notification_irq(UTR_T *utr)
{
    /* cppcheck-suppress misra-c2012-11.5 */
    RNDIS_SESSION_T *session = (RNDIS_SESSION_T *)utr->context;
    uint32_t index;
    uint32_t length;
    uint32_t request_id;

    if (session == USBNULL)
    {
        return;
    }

    if (session->notification_utr != utr)
    {
        return;
    }

    if (session->stopping != 0U)
    {
        rndis_record_stale_completion(session);
        return;
    }

    if (session->notification_io_generation != session->io_generation)
    {
        rndis_record_stale_completion(session);
        return;
    }

    if (session->notification_active == 0U)
    {
        rndis_record_stale_completion(session);
        return;
    }

    length = utr->xfer_len;
    request_id = session->notification_request_id;

    if (length > RNDIS_NOTIFICATION_SIZE)
    {
        length = RNDIS_NOTIFICATION_SIZE;
    }

    session->notification_status = utr->status;
    session->notification_length = utr->xfer_len;

    for (index = 0U; index < length; index++)
    {
        session->notification_header[index] = session->notification_buffer[index];
    }

    session->notification_active = 0U;
    session->notification_event_request_id = request_id;
    __DMB();
    session->notification_event = 1U;
}

/// @cond HIDDEN_SYMBOLS

/**
 * @brief Arm the interrupt endpoint used for RNDIS response notifications.
 * @param[in,out] session Session with an allocated notification UTR and buffer.
 * @return USBH_OK when no arm is needed or the transfer was submitted;
 *         otherwise the USB transfer error returned by the host stack.
 * @note This function is called from worker context. A device without an
 *       interrupt notification endpoint is supported and returns USBH_OK.
 */
static int rndis_notification_arm(RNDIS_SESSION_T *session)
{
    int ret;

    if (session->notification_utr == USBNULL)
    {
        return USBH_OK;
    }

    if (session->notification_active != 0U)
    {
        return USBH_OK;
    }

    session->notification_utr->udev = session->udev;
    session->notification_utr->ep = session->ep_notification;
    session->notification_utr->buff = session->notification_buffer;
    session->notification_utr->data_len = RNDIS_NOTIFICATION_SIZE;
    session->notification_utr->status = USBH_OK;
    session->notification_utr->xfer_len = 0U;
    session->notification_utr->bIsTransferDone = 0U;
    session->notification_utr->context = session;
    session->notification_utr->func = rndis_notification_irq;
    session->notification_io_generation = session->io_generation;
    session->notification_request_id = session->request_id;
    __DMB();
    session->notification_active = 1U;
    ret = usbh_int_xfer(session->notification_utr);

    if (ret != USBH_OK)
    {
        session->notification_active = 0U;
        session->notification_status = ret;
        session->stats.notification_errors++;
        session->failure_reason = RNDIS_FAILURE_NOTIFICATION_ARM;
        RNDIS_ERRMSG("[RNDIS CTRL] notification arm ep=0x%02x ret=%d.\n",
                     session->int_in_addr, ret);
    }

    return ret;
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief Validate the notification endpoint and allocate its DMA resources.
 * @param[in,out] session Session containing the communication interface and
 *                        descriptor-derived notification endpoint information.
 * @return USBH_OK on success or when notifications are not present;
 *         USBH_ERR_INVALID_PARAM, USBH_ERR_EP_NOT_FOUND, or USBH_ERR_MEMORY_OUT
 *         on failure.
 * @note Allocated resources remain owned by the session until
 *       rndis_ctrl_free_notification() or rndis_bulk_free() releases them.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 11.5<br>
 * <b>Justification:</b> usbh_alloc_mem returns void * for the common DMA pool;
 *                       the returned storage is restored as the requested
 *                       RNDIS notification byte-buffer type.
 */
int rndis_ctrl_allocate_notification(RNDIS_SESSION_T *session)
{
    if (session == USBNULL)
    {
        return USBH_ERR_INVALID_PARAM;
    }

    if (session->int_in_addr == 0U)
    {
        return USBH_OK;
    }

    session->ep_notification = usbh_iface_find_ep(session->iface_comm, session->int_in_addr, 0U);

    if ((session->ep_notification == USBNULL) ||
            ((session->ep_notification->bmAttributes & EP_ATTR_TT_MASK) != EP_ATTR_TT_INT) ||
            ((session->ep_notification->bEndpointAddress & EP_ADDR_DIR_MASK) != EP_ADDR_DIR_IN) ||
            (session->ep_notification->wMaxPacketSize != session->int_in_mps) ||
            ((session->int_in_mps & 0x07FFU) < RNDIS_NOTIFICATION_SIZE))
    {
        return USBH_ERR_EP_NOT_FOUND;
    }

    /* cppcheck-suppress misra-c2012-11.5 */
    session->notification_buffer = (uint8_t *)usbh_alloc_mem((int32_t)RNDIS_NOTIFICATION_SIZE);
    session->notification_utr = alloc_utr(session->udev);

    if ((session->notification_buffer == USBNULL) || (session->notification_utr == USBNULL))
    {
        rndis_ctrl_free_notification(session);
        return USBH_ERR_MEMORY_OUT;
    }

    session->notification_status = USBH_OK;
    session->notification_length = 0U;
    return USBH_OK;
}

/// @cond HIDDEN_SYMBOLS

/**
 * @brief Stop notification I/O and release its buffer and UTR.
 * @param[in,out] session Session whose notification resources are released.
 * @note Safe for a NULL session. The caller must not use the released
 *       notification endpoint, buffer, or UTR afterwards.
 */
void rndis_ctrl_free_notification(RNDIS_SESSION_T *session)
{
    if (session == USBNULL)
    {
        return;
    }

    if (session->notification_utr != USBNULL)
    {
        (void)usbh_quit_utr(session->notification_utr);
        session->notification_utr->func = USBNULL;
        session->notification_utr->context = USBNULL;
        free_utr(session->notification_utr);
        session->notification_utr = USBNULL;
    }

    if (session->notification_buffer != USBNULL)
    {
        (void)usbh_free_mem(session->notification_buffer, (int32_t)RNDIS_NOTIFICATION_SIZE);
        session->notification_buffer = USBNULL;
    }

    session->ep_notification = USBNULL;
    session->notification_active = 0U;
    session->notification_event = 0U;
    session->notification_request_id = 0U;
    session->notification_event_request_id = 0U;
    session->notification_length = 0U;
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief Consume and validate one latched interrupt notification event.
 * @param[in,out] session Session whose notification event is consumed.
 * @return 1 for a valid current notification, 0 when no event is pending,
 *         2 for a stale event, or -1 for a transport/protocol error.
 * @note Called from worker context. The event is cleared atomically before
 *       validation so an interrupt can safely publish the next event.
 */
static int rndis_notification_take(RNDIS_SESSION_T *session)
{
    uint8_t header[RNDIS_NOTIFICATION_SIZE];
    uint32_t index;
    uint32_t length;
    uint32_t request_id;
    uint32_t primask;
    int status;

    if (session->notification_event == 0U)
    {
        return 0;
    }

    primask = rndis_enter_critical();

    if (session->notification_event == 0U)
    {
        rndis_exit_critical(primask);
        return 0;
    }

    status = session->notification_status;
    length = session->notification_length;
    request_id = session->notification_event_request_id;

    for (index = 0U; index < RNDIS_NOTIFICATION_SIZE; index++)
    {
        header[index] = session->notification_header[index];
    }

    session->notification_event = 0U;
    session->notification_event_request_id = 0U;
    __DMB();

    rndis_exit_critical(primask);

    RNDIS_DBGMSG("[RNDIS CTRL] notification id=%lu active-id=%lu status=%d len=%lu hdr=%02x %02x %02x %02x %02x %02x %02x %02x.\n",
                 (unsigned long)request_id, (unsigned long)session->request_id, status, (unsigned long)length,
                 header[0], header[1], header[2], header[3], header[4], header[5], header[6], header[7]);

    if (request_id != session->request_id)
    {
        rndis_record_stale_completion(session);
        RNDIS_DBGMSG("[RNDIS CTRL] stale notification id=%lu active-id=%lu ignored.\n",
                     (unsigned long)request_id, (unsigned long)session->request_id);
        return 2;
    }

    if (status != USBH_OK)
    {
        session->stats.notification_errors++;
        session->failure_reason = RNDIS_FAILURE_NOTIFICATION_TRANSPORT;
        return -1;
    }

    if ((length != RNDIS_NOTIFICATION_SIZE) || (header[0] != 0xA1U) || (header[1] != 0x01U) ||
            (header[2] != 0U) || (header[3] != 0U) || (header[4] != session->comm_ifnum) ||
            (header[5] != 0U) || (header[6] != 0U) || (header[7] != 0U))
    {
        session->stats.notification_invalid++;
        session->failure_reason = RNDIS_FAILURE_NOTIFICATION_PROTOCOL;
        return -1;
    }

    session->stats.notification_events++;
    return 1;
}

/// @cond HIDDEN_SYMBOLS

/**
 * @brief Clear the current notification event and re-arm notification I/O.
 * @param[in,out] session Session whose notification state is prepared.
 * @return USBH_OK when preparation succeeds, otherwise the notification arm
 *         error returned by the host stack.
 */
static int rndis_notification_prepare(RNDIS_SESSION_T *session)
{
    rndis_notification_clear_event(session);

    if (session->notification_utr != USBNULL)
    {
        if (session->notification_active == 0U)
        {
            return rndis_notification_arm(session);
        }
    }

    return USBH_OK;
}

/**
 * @brief Send the current encapsulated RNDIS control command.
 * @param[in,out] session Session containing the prepared command buffer.
 * @param[in] length Number of command bytes to send.
 * @return USBH_OK when the complete command was transmitted; otherwise a USB
 *         transfer error or RNDIS_ERR_PROTOCOL for a short transfer.
 * @note Starts the bounded response deadline and selects notification or GET
 *       polling for the response transaction.
 */
static int rndis_send_command(RNDIS_SESSION_T *session, uint16_t length)
{
    uint32_t xfer_len;
    uint32_t now;
    int ret;

    ret = rndis_notification_prepare(session);

    if (ret != USBH_OK)
    {
        return ret;
    }

    xfer_len = 0U;

    ret = usbh_ctrl_xfer(session->udev,
                         REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE,
                         0x00U, 0U, session->comm_ifnum, length,
                         session->ctrl_command, &xfer_len, RNDIS_CTRL_ONE_XFER_TIMEOUT);

    RNDIS_DBGMSG("[RNDIS CTRL] SEND state=%u id=%lu cmd=0x%08lx len=%u ret=%d xfer=%lu.\n",
                 (uint32_t)session->state, (unsigned long)session->request_id,
                 (unsigned long)rndis_get_le32(&session->ctrl_command[0]), length, ret, (unsigned long)xfer_len);

    if (ret != USBH_OK)
    {
        session->stats.control_errors++;
        session->failure_reason = RNDIS_FAILURE_SEND_TRANSPORT;
        return ret;
    }

    if (xfer_len != length)
    {
        session->stats.control_errors++;
        session->failure_reason = RNDIS_FAILURE_SEND_LENGTH;
        return RNDIS_ERR_PROTOCOL;
    }

    now = get_ticks();
    session->deadline = now + RNDIS_CTRL_DEADLINE_TICKS;
    session->next_get_tick = now + ((session->notification_utr != USBNULL) ?
                                    RNDIS_CTRL_NOTIFICATION_WAIT_TICKS : RNDIS_CTRL_INITIAL_BACKOFF);
    session->poll_count = 0U;
    session->ctrl_wait = (session->notification_utr != USBNULL) ?
                         RNDIS_CTRL_WAIT_NOTIFICATION : RNDIS_CTRL_WAIT_RESPONSE;
    return USBH_OK;
}

/**
 * @brief Build and submit an RNDIS INITIALIZE command.
 * @param[in,out] session Paired session whose control state is advanced.
 * @note On submission failure, transitions through the bounded recovery path.
 */
static void rndis_begin_initialize(RNDIS_SESSION_T *session)
{
    uint32_t request_id = rndis_next_request_id(session);

    (void)memset(session->ctrl_command, 0, RNDIS_CTRL_COMMAND_SIZE);
    rndis_put_le32(&session->ctrl_command[0], RNDIS_MSG_INITIALIZE);
    rndis_put_le32(&session->ctrl_command[4], 24U);
    rndis_put_le32(&session->ctrl_command[8], request_id);
    rndis_put_le32(&session->ctrl_command[12], 1U);
    rndis_put_le32(&session->ctrl_command[16], 0U);
    rndis_put_le32(&session->ctrl_command[20], RNDIS_HOST_MAX_TRANSFER_SIZE);

    session->ctrl_state = RNDIS_CTRL_WAIT_INITIALIZE;
    session->state = RNDIS_STATE_INITIALIZING;

    if (rndis_send_command(session, 24U) != USBH_OK)
    {
        rndis_recover(session);
    }
}

/**
 * @brief Build and submit an RNDIS QUERY command for one OID.
 * @param[in,out] session Session whose control state is advanced.
 * @param[in] oid Object identifier to query.
 * @param[in] state Expected response state for the query.
 * @note On submission failure, transitions through the bounded recovery path.
 */
static void rndis_begin_query(RNDIS_SESSION_T *session, uint32_t oid, RNDIS_CTRL_STATE_E state)
{
    uint32_t request_id = rndis_next_request_id(session);

    (void)memset(session->ctrl_command, 0, RNDIS_CTRL_COMMAND_SIZE);
    rndis_put_le32(&session->ctrl_command[0], RNDIS_MSG_QUERY);
    rndis_put_le32(&session->ctrl_command[4], 28U);
    rndis_put_le32(&session->ctrl_command[8], request_id);
    rndis_put_le32(&session->ctrl_command[12], oid);
    session->pending_oid = oid;

    session->ctrl_state = state;
    session->state = RNDIS_STATE_QUERYING;

    if (rndis_send_command(session, 28U) != USBH_OK)
    {
        rndis_recover(session);
    }
}

/**
 * @brief Build and submit the packet-filter SET command.
 * @param[in,out] session Session whose media receive filter is configured.
 * @note On submission failure, transitions through the bounded recovery path.
 */
static void rndis_begin_set_filter(RNDIS_SESSION_T *session)
{
    uint32_t request_id = rndis_next_request_id(session);

    (void)memset(session->ctrl_command, 0, RNDIS_CTRL_COMMAND_SIZE);
    rndis_put_le32(&session->ctrl_command[0], RNDIS_MSG_SET);
    rndis_put_le32(&session->ctrl_command[4], 32U);
    rndis_put_le32(&session->ctrl_command[8], request_id);
    rndis_put_le32(&session->ctrl_command[12], OID_GEN_CURRENT_PACKET_FILTER);
    rndis_put_le32(&session->ctrl_command[16], 4U);
    rndis_put_le32(&session->ctrl_command[20], 20U);
    rndis_put_le32(&session->ctrl_command[28], RNDIS_PACKET_FILTER);
    session->pending_oid = OID_GEN_CURRENT_PACKET_FILTER;

    session->ctrl_state = RNDIS_CTRL_WAIT_SET;
    session->state = RNDIS_STATE_SETTING_FILTER;

    if (rndis_send_command(session, 32U) != USBH_OK)
    {
        rndis_recover(session);
    }
}

/**
 * @brief Tear down current control/bulk resources and schedule bounded recovery.
 * @param[in,out] session Session entering recovery or failure state.
 * @note After the maximum recovery count, the session becomes
 *       RNDIS_STATE_FAILED; otherwise it returns to RNDIS_STATE_PAIRED.
 */
static void rndis_recover(RNDIS_SESSION_T *session)
{
    session->stats.control_errors++;
    RNDIS_ERRMSG("[RNDIS CTRL] recover state=%u id=%lu reason=%s(%u) attempt=%lu.\n",
                 (uint32_t)session->state, (unsigned long)session->request_id,
                 usbh_rndis_failure_string(session->failure_reason), (uint32_t)session->failure_reason,
                 (unsigned long)(session->recovery_count + 1U));
    session->ctrl_state = RNDIS_CTRL_IDLE;
    session->ctrl_wait = RNDIS_CTRL_WAIT_NONE;
    session->media_connected = 0U;
    rndis_bulk_free(session);
    session->stopping = 0U;
    session->recovery_count++;

    if (session->recovery_count >= RNDIS_CTRL_MAX_RECOVERY)
    {
        session->state = RNDIS_STATE_FAILED;
        return;
    }

    session->state = RNDIS_STATE_PAIRED;
}

/**
 * @brief Validate the common header of an encapsulated RNDIS response.
 * @param[in] session Session containing the received response and request ID.
 * @param[in] actual Number of bytes returned by the control transfer.
 * @param[in] expected_type Expected RNDIS completion message type.
 * @param[in] fixed_length Minimum fixed response length for the message type.
 * @param[out] message_length Receives the message length from the response.
 * @return USBH_OK when the header is valid; otherwise RNDIS_ERR_PROTOCOL.
 */
static int rndis_response_header(RNDIS_SESSION_T *session, uint32_t actual, uint32_t expected_type,
                                 uint32_t fixed_length, uint32_t *message_length)
{
    uint32_t type;
    uint32_t length;

    if ((actual < 8U) || (actual > RNDIS_CTRL_RESPONSE_SIZE))
    {
        return RNDIS_ERR_PROTOCOL;
    }

    type = rndis_get_le32(&session->ctrl_response[0]);
    length = rndis_get_le32(&session->ctrl_response[4]);

    if ((length == 0U) || (length > actual) || (type != expected_type) ||
            (length < fixed_length) || (actual < fixed_length))
    {
        return RNDIS_ERR_PROTOCOL;
    }

    if (rndis_get_le32(&session->ctrl_response[8]) != session->request_id)
    {
        return RNDIS_ERR_PROTOCOL;
    }

    if (rndis_get_le32(&session->ctrl_response[12]) != RNDIS_STATUS_SUCCESS)
    {
        return RNDIS_ERR_PROTOCOL;
    }

    *message_length = length;
    return USBH_OK;
}

/**
 * @brief Validate and locate the information payload in a QUERY completion.
 * @param[in] session Session containing the received response buffer.
 * @param[in] actual Number of bytes returned by the control transfer.
 * @param[in] message_length Validated RNDIS message length.
 * @param[out] info Receives a pointer into the session response buffer.
 * @param[out] info_length Receives the information payload length.
 * @return USBH_OK when the payload is within the response; otherwise
 *         RNDIS_ERR_PROTOCOL.
 * @note The returned pointer is owned by the session and is valid only until
 *       the response buffer is reused.
 */
static int rndis_get_query_info(RNDIS_SESSION_T *session, uint32_t actual, uint32_t message_length,
                                uint8_t const **info, uint32_t *info_length)
{
    uint32_t offset = rndis_get_le32(&session->ctrl_response[20]);
    uint32_t length = rndis_get_le32(&session->ctrl_response[16]);
    uint32_t start;
    uint32_t end;

    if (offset > (UINT32_MAX - 8U))
    {
        return RNDIS_ERR_PROTOCOL;
    }

    start = 8U + offset;

    if ((start < 24U) || (length > (UINT32_MAX - start)))
    {
        return RNDIS_ERR_PROTOCOL;
    }

    end = start + length;

    if ((end > message_length) || (message_length > actual) || (actual > RNDIS_CTRL_RESPONSE_SIZE))
    {
        return RNDIS_ERR_PROTOCOL;
    }

    *info = &session->ctrl_response[start];
    *info_length = length;
    return USBH_OK;
}

/**
 * @brief Validate a supported-OID list returned by the device.
 * @param[in] info OID list encoded as little-endian 32-bit values.
 * @param[in] info_length Number of bytes in @p info.
 * @return USBH_OK when the list is aligned, non-empty, duplicate-free, and
 *         contains all OIDs required by this driver; otherwise RNDIS_ERR_PROTOCOL.
 */
static int rndis_supported_list_valid(uint8_t const *info, uint32_t info_length)
{
    uint32_t i;
    uint32_t j;

    static uint32_t const g_required_oids[] =
    {
        OID_GEN_HARDWARE_STATUS,
        OID_GEN_MEDIA_IN_USE,
        OID_GEN_MAXIMUM_FRAME_SIZE,
        OID_GEN_MAXIMUM_TOTAL_SIZE,
        OID_802_3_CURRENT_ADDRESS,
        OID_GEN_MEDIA_CONNECT_STATUS,
        OID_GEN_CURRENT_PACKET_FILTER
    };

    if ((info_length == 0U) || ((info_length & 3U) != 0U))
    {
        return RNDIS_ERR_PROTOCOL;
    }

    for (i = 0U; i < (info_length / 4U); i++)
    {
        for (j = i + 1U; j < (info_length / 4U); j++)
        {
            if (rndis_get_le32(&info[i * 4U]) == rndis_get_le32(&info[j * 4U]))
            {
                return RNDIS_ERR_PROTOCOL;
            }
        }
    }

    for (i = 0U; i < (uint32_t)(sizeof(g_required_oids) / sizeof(g_required_oids[0])); i++)
    {
        uint8_t found = 0U;

        for (j = 0U; j < (info_length / 4U); j++)
        {
            if (rndis_get_le32(&info[j * 4U]) == g_required_oids[i])
            {
                found = 1U;
                break;
            }
        }

        if (found == 0U)
        {
            return RNDIS_ERR_PROTOCOL;
        }
    }

    return USBH_OK;
}

/**
 * @brief Validate a unicast Ethernet MAC address.
 * @param[in] mac Pointer to six MAC address bytes.
 * @return 1 for a non-zero, non-broadcast, unicast address; otherwise 0.
 */
static int rndis_valid_mac(uint8_t const *mac)
{
    uint8_t index;
    uint8_t all_zero = 1U;
    uint8_t all_ff = 1U;

    if ((mac[0] & 0x01U) != 0U)
    {
        return 0;
    }

    for (index = 0U; index < 6U; index++)
    {
        if (mac[index] != 0U)
        {
            all_zero = 0U;
        }

        if (mac[index] != 0xFFU)
        {
            all_ff = 0U;
        }
    }

    return ((all_zero == 0U) && (all_ff == 0U)) ? 1 : 0;
}

/**
 * @brief Apply one validated QUERY result to the session.
 * @param[in,out] session Session containing the pending OID and negotiated values.
 * @param[in] info Query information payload.
 * @param[in] info_length Number of bytes in @p info.
 * @return USBH_OK when the payload matches the pending OID; otherwise
 *         RNDIS_ERR_PROTOCOL.
 * @note Updates MTU, transfer limits, MAC address, and media state as
 *       appropriate for the pending OID.
 */
static int rndis_process_query(RNDIS_SESSION_T *session, uint8_t const *info, uint32_t info_length)
{
    uint32_t value;

    switch (session->pending_oid)
    {
        case OID_GEN_HARDWARE_STATUS:
            if (info_length != 4U)
            {
                return RNDIS_ERR_PROTOCOL;
            }

            if (rndis_get_le32(info) != 0U)
            {
                return RNDIS_ERR_PROTOCOL;
            }

            break;

        case OID_GEN_MEDIA_IN_USE:
            if (info_length != 4U)
            {
                return RNDIS_ERR_PROTOCOL;
            }

            if (rndis_get_le32(info) != RNDIS_MEDIUM_802_3)
            {
                return RNDIS_ERR_PROTOCOL;
            }

            break;

        case OID_GEN_MAXIMUM_FRAME_SIZE:
            if (info_length != 4U)
            {
                return RNDIS_ERR_PROTOCOL;
            }

            value = rndis_get_le32(info);

            if ((value > (UINT32_MAX - 14U)) || (value > (RNDIS_SLOT_SIZE - 44U - 14U)))
            {
                return RNDIS_ERR_PROTOCOL;
            }

            session->payload_mtu = value;
            session->max_frame = 14U + value;
            break;

        case OID_GEN_MAXIMUM_TOTAL_SIZE:
            if (info_length != 4U)
            {
                return RNDIS_ERR_PROTOCOL;
            }

            value = rndis_get_le32(info);

            if (value < 14U)
            {
                return RNDIS_ERR_PROTOCOL;
            }

            session->max_total = value;
            break;

        case OID_802_3_CURRENT_ADDRESS:
            if ((info_length != 6U) || (rndis_valid_mac(info) == 0))
            {
                return RNDIS_ERR_PROTOCOL;
            }

            (void)memcpy(session->mac, info, 6U);
            break;

        case OID_GEN_MEDIA_CONNECT_STATUS:
            if (info_length != 4U)
            {
                return RNDIS_ERR_PROTOCOL;
            }

            if (rndis_get_le32(info) != RNDIS_MEDIA_CONNECTED)
            {
                return RNDIS_ERR_PROTOCOL;
            }

            session->media_connected = 1U;
            break;

        default:
            return RNDIS_ERR_PROTOCOL;
    }

    return USBH_OK;
}

/**
 * @brief Validate and consume one RNDIS control response.
 * @param[in,out] session Session whose pending control transaction is processed.
 * @param[in] actual Number of bytes present in the response buffer.
 * @return USBH_OK when the response is valid and the next control operation is
 *         scheduled; otherwise RNDIS_ERR_PROTOCOL.
 * @note This worker-context function advances initialization, OID query, and
 *       packet-filter state, and may transition the session to RUNNING.
 */
static int rndis_process_response(RNDIS_SESSION_T *session, uint32_t actual)
{
    uint32_t expected_type;
    uint32_t fixed_length;
    uint32_t message_length;
    uint8_t const *info;
    uint32_t info_length;
    int ret;

    static uint32_t const g_query_oids[] =
    {
        OID_GEN_HARDWARE_STATUS,
        OID_GEN_MEDIA_IN_USE,
        OID_GEN_MAXIMUM_FRAME_SIZE,
        OID_GEN_MAXIMUM_TOTAL_SIZE,
        OID_802_3_CURRENT_ADDRESS,
        OID_GEN_MEDIA_CONNECT_STATUS
    };

    switch (session->ctrl_state)
    {
        case RNDIS_CTRL_WAIT_INITIALIZE:
            expected_type = RNDIS_MSG_INITIALIZE_CMPLT;
            fixed_length = 52U;
            break;

        case RNDIS_CTRL_WAIT_SUPPORTED:
        case RNDIS_CTRL_WAIT_QUERY:
            expected_type = RNDIS_MSG_QUERY_CMPLT;
            fixed_length = 24U;
            break;

        case RNDIS_CTRL_WAIT_SET:
            expected_type = RNDIS_MSG_SET_CMPLT;
            fixed_length = 16U;
            break;

        default:
            return RNDIS_ERR_PROTOCOL;
    }

    ret = rndis_response_header(session, actual, expected_type, fixed_length, &message_length);

    if (ret != USBH_OK)
    {
        return ret;
    }

    if (session->ctrl_state == RNDIS_CTRL_WAIT_INITIALIZE)
    {
        uint32_t major_version = rndis_get_le32(&session->ctrl_response[16]);
        uint32_t minor_version = rndis_get_le32(&session->ctrl_response[20]);
        uint32_t medium = rndis_get_le32(&session->ctrl_response[28]);
        uint32_t device_max_transfer_size = rndis_get_le32(&session->ctrl_response[36]);
        uint32_t tx_max_transfer_size = device_max_transfer_size;

#ifdef RNDIS_DEBUG
        uint32_t type = rndis_get_le32(&session->ctrl_response[0]);
        uint32_t length = rndis_get_le32(&session->ctrl_response[4]);
        uint32_t request_id = rndis_get_le32(&session->ctrl_response[8]);
        uint32_t status = rndis_get_le32(&session->ctrl_response[12]);
        uint32_t device_flags = rndis_get_le32(&session->ctrl_response[24]);
        uint32_t max_packets_per_transfer = rndis_get_le32(&session->ctrl_response[32]);
        uint32_t packet_alignment_factor = rndis_get_le32(&session->ctrl_response[40]);
        uint32_t af_list_offset = rndis_get_le32(&session->ctrl_response[44]);
        uint32_t af_list_size = rndis_get_le32(&session->ctrl_response[48]);
#endif

        if (tx_max_transfer_size > RNDIS_HOST_MAX_TRANSFER_SIZE)
        {
            tx_max_transfer_size = RNDIS_HOST_MAX_TRANSFER_SIZE;
        }

#ifdef RNDIS_DEBUG
        RNDIS_DBGMSG("[RNDIS CTRL] INIT-CMPLT type=0x%08lx len=%lu id=%lu status=0x%08lx ver=%lu.%lu flags=0x%08lx medium=0x%08lx max-pkts=%lu device-max=%lu align=%lu af-off=%lu af-size=%lu host-cap=%lu tx-cap=%lu.\n",
                     (unsigned long)type, (unsigned long)length, (unsigned long)request_id,
                     (unsigned long)status, (unsigned long)major_version, (unsigned long)minor_version,
                     (unsigned long)device_flags, (unsigned long)medium,
                     (unsigned long)max_packets_per_transfer, (unsigned long)device_max_transfer_size,
                     (unsigned long)packet_alignment_factor, (unsigned long)af_list_offset,
                     (unsigned long)af_list_size, (unsigned long)RNDIS_HOST_MAX_TRANSFER_SIZE,
                     (unsigned long)tx_max_transfer_size);
#endif

        if ((major_version != 1U) || (minor_version != 0U) || (medium != RNDIS_MEDIUM_802_3) ||
                (device_max_transfer_size < 58U))
        {
            RNDIS_ERRMSG("[RNDIS CTRL] INIT-CMPLT rejected ver=%lu.%lu medium=0x%08lx device-max=%lu.\n",
                         (unsigned long)major_version, (unsigned long)minor_version,
                         (unsigned long)medium, (unsigned long)device_max_transfer_size);
            return RNDIS_ERR_PROTOCOL;
        }

        session->device_max_transfer_size = device_max_transfer_size;
        session->tx_max_transfer_size = tx_max_transfer_size;
        session->query_index = 0U;
        rndis_begin_query(session, OID_GEN_SUPPORTED_LIST, RNDIS_CTRL_WAIT_SUPPORTED);
        return USBH_OK;
    }

    if (session->ctrl_state == RNDIS_CTRL_WAIT_SET)
    {
        uint32_t tx_message_length;

        if ((session->media_connected == 0U) || (session->max_frame == 0U) || (session->max_total == 0U))
        {
            return RNDIS_ERR_PROTOCOL;
        }

        /* OID_GEN_MAXIMUM_TOTAL_SIZE limits an Ethernet frame, not a RNDIS wire message. */
        if ((session->max_total < session->max_frame) ||
                (session->max_frame > (UINT32_MAX - 44U)))
        {
            return RNDIS_ERR_PROTOCOL;
        }

        tx_message_length = 44U + session->max_frame;

        /* RNDIS header overhead applies only to transfer and local-slot capacities. */
        if ((tx_message_length > session->tx_max_transfer_size) ||
                (tx_message_length > RNDIS_SLOT_SIZE))
        {
            return RNDIS_ERR_PROTOCOL;
        }

        session->ctrl_state = RNDIS_CTRL_IDLE;
        session->ctrl_wait = RNDIS_CTRL_WAIT_NONE;
        session->state = RNDIS_STATE_RUNNING;
        session->failure_reason = RNDIS_FAILURE_NONE;
        session->recovery_count = 0U;
        return USBH_OK;
    }

    ret = rndis_get_query_info(session, actual, message_length, &info, &info_length);

    if (ret != USBH_OK)
    {
        return ret;
    }

    if (session->ctrl_state == RNDIS_CTRL_WAIT_SUPPORTED)
    {
        ret = rndis_supported_list_valid(info, info_length);

        if (ret != USBH_OK)
        {
            return ret;
        }

        session->query_index = 0U;
        rndis_begin_query(session, g_query_oids[0], RNDIS_CTRL_WAIT_QUERY);
        return USBH_OK;
    }

    ret = rndis_process_query(session, info, info_length);

    if (ret != USBH_OK)
    {
        return ret;
    }

    session->query_index++;

    if (session->query_index < (uint32_t)(sizeof(g_query_oids) / sizeof(g_query_oids[0])))
    {
        rndis_begin_query(session, g_query_oids[session->query_index], RNDIS_CTRL_WAIT_QUERY);
    }
    else
    {
        rndis_begin_set_filter(session);
    }

    return USBH_OK;
}

/**
 * @brief Schedule the next bounded GET response poll.
 * @param[in,out] session Session containing the active control transaction.
 * @param[in] now Current tick value used as the backoff origin.
 * @return USBH_OK when another poll is permitted; otherwise RNDIS_ERR_PROTOCOL
 *         when the transaction has exhausted or lacks a valid poll count.
 */
static int rndis_schedule_next_poll(RNDIS_SESSION_T *session, uint32_t now)
{
    uint32_t backoff;
    uint32_t shift_count;

    /* poll_count is the number of submitted GET requests for this transaction. */
    if ((session->poll_count == 0U) || (session->poll_count >= RNDIS_CTRL_MAX_POLLS))
    {
        return RNDIS_ERR_PROTOCOL;
    }

    shift_count = session->poll_count - 1U;
    backoff = RNDIS_CTRL_INITIAL_BACKOFF;

    while ((shift_count > 0U) && (backoff < RNDIS_CTRL_MAX_BACKOFF))
    {
        backoff *= 2U;
        shift_count--;
    }

    session->next_get_tick = now + backoff;
    session->ctrl_wait = RNDIS_CTRL_WAIT_RESPONSE;
    return USBH_OK;
}

/**
 * @brief Check whether another encapsulated-response GET may be submitted.
 * @param[in] session Session containing the current poll count.
 * @return 1 when the bounded poll limit has not been reached; otherwise 0.
 */
static int rndis_can_submit_get(RNDIS_SESSION_T const *session)
{
    /* Invariant: usbh_ctrl_xfer(GET) is reached only when fewer than 10 GETs were submitted. */
    return (session->poll_count < RNDIS_CTRL_MAX_POLLS) ? 1 : 0;
}

/**
 * @brief Start the RNDIS control initialization transaction.
 * @param[in,out] session Paired session with allocated resources.
 * @return USBH_OK when initialization is started; otherwise RNDIS_ERR_NOT_READY
 *         if the session is not paired, ready, or is stopping.
 * @note The actual control transfers are completed later by rndis_ctrl_poll().
 */
int rndis_ctrl_start(RNDIS_SESSION_T *session)
{
    if ((session == USBNULL) || (session->resources_ready == 0U) ||
            (session->state != RNDIS_STATE_PAIRED) || (session->stopping != 0U))
    {
        return RNDIS_ERR_NOT_READY;
    }

    rndis_begin_initialize(session);
    return USBH_OK;
}

/**
 * @brief Advance the deferred RNDIS control state machine.
 * @param[in,out] session Session whose control transaction is serviced.
 * @note Worker-context API. It handles notification events, bounded GET polls,
 *       response validation, deadlines, and recovery; it does not block waiting
 *       for a response.
 */
void rndis_ctrl_poll(RNDIS_SESSION_T *session)
{
    uint32_t now;
    uint32_t remaining;
    uint32_t timeout;
    uint32_t actual;
    int ret;

    if ((session == USBNULL) || (session->stopping != 0U) ||
            (session->ctrl_state == RNDIS_CTRL_IDLE))
    {
        return;
    }

    now = get_ticks();

    if (rndis_time_due(now, session->deadline) != 0)
    {
        session->failure_reason = RNDIS_FAILURE_CONTROL_DEADLINE;
        rndis_recover(session);
        return;
    }

    if (session->ctrl_wait == RNDIS_CTRL_WAIT_NOTIFICATION)
    {
        ret = rndis_notification_take(session);

        if (ret != 0)
        {
            if (rndis_notification_arm(session) != USBH_OK)
            {
                rndis_recover(session);
                return;
            }

            if (ret < 0)
            {
                return;
            }

            if (ret > 1)
            {
                return;
            }

            session->ctrl_wait = RNDIS_CTRL_WAIT_RESPONSE;
            session->next_get_tick = now;
        }
        else if (rndis_time_due(now, session->next_get_tick) != 0)
        {
            session->failure_reason = RNDIS_FAILURE_NOTIFICATION_TIMEOUT;
            RNDIS_ERRMSG("[RNDIS CTRL] notification wait timeout id=%lu; bounded GET fallback.\n",
                         (unsigned long)session->request_id);
            session->ctrl_wait = RNDIS_CTRL_WAIT_RESPONSE;
            session->next_get_tick = now;
        }
        else
        {
            return;
        }
    }

    if ((session->ctrl_wait != RNDIS_CTRL_WAIT_RESPONSE) ||
            (rndis_time_due(now, session->next_get_tick) == 0))
    {
        return;
    }

    remaining = session->deadline - now;

    if (remaining <= RNDIS_CTRL_TIMEOUT_GUARD)
    {
        session->failure_reason = RNDIS_FAILURE_CONTROL_DEADLINE;
        rndis_recover(session);
        return;
    }

    timeout = RNDIS_CTRL_ONE_XFER_TIMEOUT;

    if (timeout > (remaining - RNDIS_CTRL_TIMEOUT_GUARD))
    {
        timeout = remaining - RNDIS_CTRL_TIMEOUT_GUARD;
    }

    if (rndis_can_submit_get(session) == 0)
    {
        session->failure_reason = RNDIS_FAILURE_GET_LIMIT;
        rndis_recover(session);
        return;
    }

    actual = 0U;
    (void)memset(session->ctrl_response, 0, RNDIS_CTRL_RESPONSE_SIZE);
    session->poll_count++;
    session->stats.get_polls++;
    RNDIS_DBGMSG("[RNDIS CTRL] GET state=%u id=%lu poll=%lu deadline=%lu timeout=%lu.\n",
                 (uint32_t)session->state, (unsigned long)session->request_id,
                 (unsigned long)session->poll_count, (unsigned long)session->deadline, (unsigned long)timeout);
    ret = usbh_ctrl_xfer(session->udev,
                         REQ_TYPE_IN | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE,
                         0x01U, 0U, session->comm_ifnum, RNDIS_CTRL_RESPONSE_SIZE,
                         session->ctrl_response, &actual, timeout);

    RNDIS_DBGMSG("[RNDIS CTRL] GET result id=%lu ret=%d actual=%lu.\n",
                 (unsigned long)session->request_id, ret, (unsigned long)actual);

    now = get_ticks();

    if (ret == USBH_OK)
    {
        if (actual == 0U)
        {
            /* RNDIS response is not ready yet; retain this transaction and retry. */
            if (rndis_schedule_next_poll(session, now) != USBH_OK)
            {
                session->failure_reason = RNDIS_FAILURE_GET_LIMIT;
                rndis_recover(session);
            }
            else if (rndis_time_due(now, session->deadline) != 0)
            {
                session->failure_reason = RNDIS_FAILURE_CONTROL_DEADLINE;
                rndis_recover(session);
            }
            else
            {
                RNDIS_DBGMSG("[RNDIS CTRL] GET id=%lu empty; retry poll=%lu.\n",
                             (unsigned long)session->request_id, (unsigned long)session->poll_count);
            }

            return;
        }

        if (actual >= 8U)
        {
            RNDIS_DBGMSG("[RNDIS CTRL] RESP id=%lu type=0x%08lx len=%lu parsed-id=%lu status=0x%08lx.\n",
                         (unsigned long)session->request_id,
                         (unsigned long)rndis_get_le32(&session->ctrl_response[0]),
                         (unsigned long)rndis_get_le32(&session->ctrl_response[4]),
                         (unsigned long)((actual >= 12U) ? rndis_get_le32(&session->ctrl_response[8]) : 0U),
                         (unsigned long)((actual >= 16U) ? rndis_get_le32(&session->ctrl_response[12]) : 0U));
        }
        else
        {
            RNDIS_ERRMSG("[RNDIS CTRL] RESP id=%lu short actual=%lu.\n",
                         (unsigned long)session->request_id, (unsigned long)actual);
        }

        ret = rndis_process_response(session, actual);

        if (ret != USBH_OK)
        {
            session->failure_reason = RNDIS_FAILURE_RESPONSE_PROTOCOL;
            rndis_recover(session);
        }

        return;
    }

    session->stats.control_errors++;
    session->stats.get_transport_errors++;
    session->failure_reason = RNDIS_FAILURE_GET_TRANSPORT;

    if (rndis_schedule_next_poll(session, now) != USBH_OK)
    {
        session->failure_reason = RNDIS_FAILURE_GET_LIMIT;
        rndis_recover(session);
    }
    else if (rndis_time_due(now, session->deadline) != 0)
    {
        session->failure_reason = RNDIS_FAILURE_CONTROL_DEADLINE;
        rndis_recover(session);
    }
    else
    {
        RNDIS_ERRMSG("[RNDIS CTRL] GET id=%lu ret=%d; retry poll=%lu.\n",
                     (unsigned long)session->request_id, ret, (unsigned long)session->poll_count);
    }
}

/**
 * @brief Stop the active RNDIS control transaction.
 * @param[in,out] session Session whose control state is reset.
 * @note Safe for a NULL session. It does not release control buffers; resource
 *       release is performed by rndis_bulk_free().
 */
void rndis_ctrl_stop(RNDIS_SESSION_T *session)
{
    if (session != USBNULL)
    {
        session->ctrl_state = RNDIS_CTRL_IDLE;
        session->ctrl_wait = RNDIS_CTRL_WAIT_NONE;
        session->pending_oid = 0U;
    }
}

/// @endcond HIDDEN_SYMBOLS

/** @} end of group USBH_EXPORTED_FUNCTIONS */

/** @} end of group USBH_Library */

/** @} end of group LIBRARY */
