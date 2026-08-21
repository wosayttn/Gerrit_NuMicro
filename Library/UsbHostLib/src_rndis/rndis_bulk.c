/**************************************************************************//**
 * @file     rndis_bulk.c
 * @brief    RNDIS packet codec and deferred bulk transfer processing.
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

/**
 * @brief Enter the short critical section used by shared slot state updates.
 * @return Previous PRIMASK value, to be passed to rndis_exit_critical().
 * @note Interrupts are disabled before returning. The caller must keep the
 *       protected section short and must always restore the returned mask.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 17.3<br>
 * <b>Justification:</b> __disable_irq is a CMSIS compiler intrinsic used to
 *                       protect the shared slot state update.
 */
uint32_t rndis_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    /* cppcheck-suppress misra-c2012-17.3 */
    __disable_irq();
    return primask;
}

/**
 * @brief Restore the interrupt mask saved by rndis_enter_critical().
 * @param[in] primask Previous PRIMASK value returned by rndis_enter_critical().
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 17.3<br>
 * <b>Justification:</b> __enable_irq is a CMSIS compiler intrinsic used only
 *                       to restore the caller's saved interrupt mask state.
 */
void rndis_exit_critical(uint32_t primask)
{
    if (primask == 0U)
    {
        /* cppcheck-suppress misra-c2012-17.3 */
        __enable_irq();
    }
}

/// @cond HIDDEN_SYMBOLS

/**
 * @brief Mark an in-flight slot for asynchronous shutdown.
 * @param[in,out] slot DMA slot whose ownership state is updated.
 * @note This function does not abort the UTR. The caller must restore IRQ
 *       delivery and later call rndis_release_slot().
 */
static void rndis_stop_slot(RNDIS_SLOT_T *slot)
{
    if (slot->state == RNDIS_SLOT_DMA_OWNED)
    {
        slot->state = RNDIS_SLOT_STOPPING;
    }
}

/**
 * @brief Allocate the next non-zero private I/O generation value.
 * @param[in] io_generation Current private I/O epoch.
 * @return Next non-zero epoch; zero is skipped on wraparound.
 * @note This value is used only to reject stale UTR completions and is not the
 *       public session identity.
 */
static uint32_t rndis_next_io_generation(uint32_t io_generation)
{
    if (io_generation == UINT32_MAX)
    {
        return 1U;
    }

    return io_generation + 1U;
}

/**
 * @brief Allocate the next non-zero public TX submission identifier.
 * @param[in,out] session Session whose TX submission sequence is advanced.
 * @return Identifier assigned to the accepted TX slot.
 */
static uint32_t rndis_next_submit_id(RNDIS_SESSION_T *session)
{
    session->tx_submit_sequence++;

    if (session->tx_submit_sequence == 0U)
    {
        session->tx_submit_sequence = 1U;
    }

    return session->tx_submit_sequence;
}

/**
 * @brief Publish one completed normal TX result atomically.
 * @param[in,out] session Session receiving the scalar completion snapshot.
 * @param[in] submit_id Identifier of the completed accepted transmission.
 * @param[in] status USB completion status.
 * @param[in] actual_length Actual number of bytes reported by the host controller.
 * @param[in] expected_wire_length RNDIS message length submitted to USB.
 * @note The snapshot is read by worker-context API code and is protected from
 *       concurrent interrupt access using a short critical section.
 */
static void rndis_publish_tx_outcome(RNDIS_SESSION_T *session, uint32_t submit_id,
                                     int32_t status, uint32_t actual_length,
                                     uint32_t expected_wire_length)
{
    uint32_t primask = rndis_enter_critical();

    session->tx_last_submit_id = submit_id;
    session->tx_last_status = status;
    session->tx_last_actual_length = actual_length;
    session->tx_last_expected_wire_length = expected_wire_length;
    session->tx_completion_sequence++;

    if (session->tx_completion_sequence == 0U)
    {
        session->tx_completion_sequence = 1U;
    }

    session->tx_abort_pending = 0U;
    rndis_exit_critical(primask);
}

/**
 * @brief Abort and release one bulk slot's UTR and DMA buffer.
 * @param[in,out] slot Slot whose resources are released.
 * @param[in] buffer_size Allocated slot buffer size in bytes.
 * @note The slot must no longer be accepted by the worker or a callback. The
 *       function returns the slot to RNDIS_SLOT_FREE.
 */
static void rndis_release_slot(RNDIS_SLOT_T *slot, uint32_t buffer_size)
{
    UTR_T *utr = slot->utr;

    if (utr != USBNULL)
    {
        /*
         * The host-controller quit path may wait for an interrupt-driven
         * unlink. It must run after the short ownership claim has restored
         * IRQ delivery. On return, the controller has released this UTR.
         */
        (void)usbh_quit_utr(utr);
        utr->func = USBNULL;
        utr->context = USBNULL;
        free_utr(utr);
        slot->utr = USBNULL;
    }

    if (slot->buffer != USBNULL)
    {
        (void)usbh_free_mem(slot->buffer, (int)buffer_size);
        slot->buffer = USBNULL;
    }

    slot->actual_length = 0U;
    slot->expected_wire_length = 0U;
    slot->status = USBH_OK;
    slot->state = RNDIS_SLOT_FREE;
}

/**
 * @brief Publish the latest deferred RX validation result.
 * @param[in,out] session Session receiving the diagnostic snapshot.
 * @param[in] event Validation result to publish.
 * @param[in] slot Completed RX slot supplying status and length fields.
 * @param[in] header_valid Non-zero when the fixed RNDIS header was validated.
 * @param[in] message_type Parsed RNDIS message type.
 * @param[in] message_length Parsed RNDIS message length.
 * @param[in] data_offset Parsed RNDIS data offset.
 * @param[in] data_length Parsed Ethernet payload length.
 * @note Intended for deferred worker context; the sequence counter lets a
 *       reader detect a newly published diagnostic event.
 */
static void rndis_publish_rx_diagnostic(RNDIS_SESSION_T *session, RNDIS_RX_DIAG_EVENT_E event,
                                        RNDIS_SLOT_T const *slot, uint8_t header_valid,
                                        uint32_t message_type, uint32_t message_length,
                                        uint32_t data_offset, uint32_t data_length)
{
    RNDIS_RX_DIAG_T *diagnostic = &session->rx_diagnostic;

    diagnostic->sequence++;

    if (diagnostic->sequence == 0U)
    {
        diagnostic->sequence = 1U;
    }

    diagnostic->event = event;
    diagnostic->usb_status = (int32_t)slot->status;
    diagnostic->actual_length = slot->actual_length;
    diagnostic->message_type = (header_valid != 0U) ? message_type : 0U;
    diagnostic->message_length = (header_valid != 0U) ? message_length : 0U;
    diagnostic->data_offset = (header_valid != 0U) ? data_offset : 0U;
    diagnostic->data_length = (header_valid != 0U) ? data_length : 0U;
}

/**
 * @brief Return the negotiated upper bound for a TX RNDIS packet.
 * @param[in] session Session containing the device transfer limit.
 * @return Maximum TX RNDIS message length in bytes.
 */
static uint32_t rndis_tx_packet_total_limit(RNDIS_SESSION_T const *session)
{
    return session->tx_max_transfer_size;
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief Capture completion of one RX bulk transfer.
 * @param[in] utr Completed RX transfer request.
 * @note Called from USB interrupt/callback context. It only changes slot
 *       completion fields; packet parsing is deferred to rndis_bulk_poll().
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 11.5<br>
 * <b>Justification:</b> UTR context is stored as void * by the generic USB
 *                       host layer and is restored as its RNDIS session type.
 */
static void rndis_rx_irq(UTR_T *utr)
{
    /* cppcheck-suppress misra-c2012-11.5 */
    RNDIS_SESSION_T *session = (RNDIS_SESSION_T *)utr->context;
    uint32_t index;

    if (session == USBNULL)
    {
        return;
    }

    for (index = 0U; index < RNDIS_RX_SLOT_COUNT; index++)
    {
        RNDIS_SLOT_T *slot = &session->rx[index];

        if (slot->utr == utr)
        {
            if ((session->stopping != 0U) || (slot->io_generation != session->io_generation) ||
                    (slot->state != RNDIS_SLOT_DMA_OWNED))
            {
                rndis_record_stale_completion(session);
                return;
            }

            slot->status = utr->status;
            slot->actual_length = utr->xfer_len;
            slot->state = RNDIS_SLOT_COMPLETED;
            return;
        }
    }

    rndis_record_stale_completion(session);
}

/**
 * @brief Capture completion of the single TX bulk transfer slot.
 * @param[in] utr Completed TX transfer request.
 * @note Called from USB interrupt/callback context. Completion accounting and
 *       publication are deferred to rndis_bulk_poll().
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 11.5<br>
 * <b>Justification:</b> UTR context is stored as void * by the generic USB
 *                       host layer and is restored as its RNDIS session type.
 */
static void rndis_tx_irq(UTR_T *utr)
{
    /* cppcheck-suppress misra-c2012-11.5 */
    RNDIS_SESSION_T *session = (RNDIS_SESSION_T *)utr->context;
    RNDIS_SLOT_T *slot;

    if ((session == USBNULL) || (session->tx[0].utr != utr))
    {
        return;
    }

    slot = &session->tx[0];

    if ((session->stopping != 0U) || (slot->io_generation != session->io_generation) ||
            (slot->state != RNDIS_SLOT_DMA_OWNED))
    {
        rndis_record_stale_completion(session);
        return;
    }

    slot->status = utr->status;
    slot->actual_length = utr->xfer_len;
    slot->state = RNDIS_SLOT_COMPLETED;
}

/**
 * @brief Submit one free RX slot to the bulk-IN endpoint.
 * @param[in,out] session Running session owning the endpoint and slot.
 * @param[in,out] slot Free RX slot with allocated buffer and UTR.
 * @return USBH_OK when submitted; otherwise RNDIS_ERR_NOT_READY or the host
 *         transfer error returned by usbh_bulk_xfer().
 * @note Worker-context function. On immediate submission failure the slot is
 *       returned to FREE and the transfer error counter is incremented.
 */
/// @cond HIDDEN_SYMBOLS

static int rndis_submit_rx(RNDIS_SESSION_T *session, RNDIS_SLOT_T *slot)
{
    int ret;

    if ((session->state != RNDIS_STATE_RUNNING) || (session->stopping != 0U) ||
            (slot->state != RNDIS_SLOT_FREE))
    {
        return RNDIS_ERR_NOT_READY;
    }

    slot->utr->udev = session->udev;
    slot->utr->ep = session->ep_bulk_in;
    slot->utr->buff = slot->buffer;
    slot->utr->data_len = RNDIS_SLOT_SIZE;
    slot->utr->status = USBH_OK;
    slot->utr->xfer_len = 0U;
    slot->utr->bIsTransferDone = 0U;
    slot->utr->context = session;
    slot->utr->func = rndis_rx_irq;
    slot->status = USBH_OK;
    slot->actual_length = 0U;
    slot->io_generation = session->io_generation;
    slot->state = RNDIS_SLOT_DMA_OWNED;
    ret = usbh_bulk_xfer(slot->utr);

    if (ret != USBH_OK)
    {
        slot->state = RNDIS_SLOT_FREE;
        slot->status = ret;
        session->stats.transfer_errors++;
    }

    return ret;
}

/**
 * @brief Determine whether any RX slot is currently owned by DMA.
 * @param[in] session Session whose RX slots are inspected.
 * @return 1 when a slot is in RNDIS_SLOT_DMA_OWNED; otherwise 0.
 */
static int rndis_rx_is_active(RNDIS_SESSION_T const *session)
{
    uint32_t index;

    for (index = 0U; index < RNDIS_RX_SLOT_COUNT; index++)
    {
        if (session->rx[index].state == RNDIS_SLOT_DMA_OWNED)
        {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief Submit the ready TX slot to the bulk-OUT endpoint.
 * @param[in,out] session Running session owning the endpoint and slot.
 * @param[in,out] slot Ready TX slot containing one complete RNDIS packet.
 * @return USBH_OK when submitted; otherwise RNDIS_ERR_NOT_READY or the host
 *         transfer error returned by usbh_bulk_xfer().
 * @note Worker-context function. A synchronous submission failure is retained
 *       as a completed slot for normal TX outcome publication.
 */
static int rndis_submit_tx(RNDIS_SESSION_T *session, RNDIS_SLOT_T *slot)
{
    int ret;

    if ((session->state != RNDIS_STATE_RUNNING) || (session->stopping != 0U) ||
            (slot->state != RNDIS_SLOT_READY))
    {
        return RNDIS_ERR_NOT_READY;
    }

    slot->utr->udev = session->udev;
    slot->utr->ep = session->ep_bulk_out;
    slot->utr->buff = slot->buffer;
    slot->utr->data_len = slot->expected_wire_length;
    slot->utr->status = USBH_OK;
    slot->utr->xfer_len = 0U;
    slot->utr->bIsTransferDone = 0U;
    slot->utr->context = session;
    slot->utr->func = rndis_tx_irq;
    slot->status = USBH_OK;
    slot->actual_length = 0U;
    slot->io_generation = session->io_generation;
    __DMB();
    slot->state = RNDIS_SLOT_DMA_OWNED;
    ret = usbh_bulk_xfer(slot->utr);

    if (ret != USBH_OK)
    {
        slot->status = ret;
        slot->actual_length = 0U;
        slot->state = RNDIS_SLOT_COMPLETED;
    }

    return ret;
}

/**
 * @brief Account for and publish one completed TX slot in worker context.
 * @param[in,out] session Session receiving TX counters and outcome snapshot.
 * @param[in,out] slot Completed TX slot to return to FREE.
 * @note A packet counts as successfully transmitted locally only when the USB
 *       status is successful and the actual length exactly matches the wire
 *       length. This does not prove peer receipt.
 */
static void rndis_complete_tx_worker(RNDIS_SESSION_T *session, RNDIS_SLOT_T *slot)
{
    int32_t status = slot->status;
    uint32_t actual_length = slot->actual_length;
    uint32_t expected_wire_length = slot->expected_wire_length;
    uint32_t submit_id = slot->submit_id;

    if ((slot->status == USBH_OK) && (slot->expected_wire_length >= 44U) &&
            (slot->actual_length == slot->expected_wire_length))
    {
        session->stats.tx_packets++;
        session->stats.tx_bytes += (slot->expected_wire_length - 44U);
    }
    else
    {
        session->stats.transfer_errors++;
        session->stats.tx_drops++;
    }

    rndis_publish_tx_outcome(session, submit_id, status, actual_length,
                             expected_wire_length);
    slot->actual_length = 0U;
    slot->expected_wire_length = 0U;
    slot->submit_id = 0U;
    slot->status = USBH_OK;
    slot->state = RNDIS_SLOT_FREE;
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief Allocate bulk endpoints, control buffers, DMA buffers, and UTRs.
 * @param[in,out] session Paired session receiving the allocated resources.
 * @return USBH_OK on success; otherwise a USB endpoint or memory allocation
 *         error code.
 * @note Resources are owned by the session after success and must be released
 *       with rndis_bulk_free(). The function enforces the configured DMA
 *       headroom gate before allocating buffers.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 11.5<br>
 * <b>Justification:</b> usbh_alloc_mem returns void * for the common DMA pool;
 *                       the returned storage is restored as the requested
 *                       RNDIS byte-buffer type.
 */
int rndis_bulk_allocate(RNDIS_SESSION_T *session)
{
    uint32_t used;
    uint32_t index;

    if ((session == USBNULL) || (session->resources_ready != 0U))
    {
        return USBH_ERR_INVALID_PARAM;
    }

    used = usbh_memory_used();

    if ((used + RNDIS_RESOURCE_UNITS + RNDIS_DMA_HEADROOM_UNITS) > (uint32_t)DMA_MEM_UNIT_NUM)
    {
        USB_error("RNDIS: DMA headroom gate rejected allocation (%u + %u + %u > %u).\n",
                  used, RNDIS_RESOURCE_UNITS, RNDIS_DMA_HEADROOM_UNITS, DMA_MEM_UNIT_NUM);
        return USBH_ERR_MEMORY_OUT;
    }

    session->ep_bulk_in = usbh_iface_find_ep(session->iface_data, session->bulk_in_addr, 0U);
    session->ep_bulk_out = usbh_iface_find_ep(session->iface_data, session->bulk_out_addr, 0U);

    if ((session->ep_bulk_in == USBNULL) || (session->ep_bulk_out == USBNULL) ||
            ((session->ep_bulk_in->bmAttributes & EP_ATTR_TT_MASK) != EP_ATTR_TT_BULK) ||
            ((session->ep_bulk_out->bmAttributes & EP_ATTR_TT_MASK) != EP_ATTR_TT_BULK) ||
            ((session->ep_bulk_in->bEndpointAddress & EP_ADDR_DIR_MASK) != EP_ADDR_DIR_IN) ||
            ((session->ep_bulk_out->bEndpointAddress & EP_ADDR_DIR_MASK) != EP_ADDR_DIR_OUT) ||
            (session->ep_bulk_in->wMaxPacketSize != session->bulk_in_mps) ||
            (session->ep_bulk_out->wMaxPacketSize != session->bulk_out_mps))
    {
        return USBH_ERR_EP_NOT_FOUND;
    }

    if (rndis_ctrl_allocate_notification(session) != USBH_OK)
    {
        session->failure_reason = RNDIS_FAILURE_ALLOCATION;
        rndis_bulk_free(session);
        return USBH_ERR_MEMORY_OUT;
    }

    /* cppcheck-suppress misra-c2012-11.5 */
    session->ctrl_command = (uint8_t *)usbh_alloc_mem(RNDIS_CTRL_COMMAND_SIZE);
    /* cppcheck-suppress misra-c2012-11.5 */
    session->ctrl_response = (uint8_t *)usbh_alloc_mem(RNDIS_CTRL_RESPONSE_SIZE);

    if ((session->ctrl_command == USBNULL) || (session->ctrl_response == USBNULL))
    {
        session->failure_reason = RNDIS_FAILURE_ALLOCATION;
        rndis_bulk_free(session);
        return USBH_ERR_MEMORY_OUT;
    }

    for (index = 0U; index < RNDIS_RX_SLOT_COUNT; index++)
    {
        /* cppcheck-suppress misra-c2012-11.5 */
        session->rx[index].buffer = (uint8_t *)usbh_alloc_mem(RNDIS_SLOT_SIZE);
        session->rx[index].utr = alloc_utr(session->udev);

        if ((session->rx[index].buffer == USBNULL) || (session->rx[index].utr == USBNULL))
        {
            session->failure_reason = RNDIS_FAILURE_ALLOCATION;
            rndis_bulk_free(session);
            return USBH_ERR_MEMORY_OUT;
        }

        session->rx[index].state = RNDIS_SLOT_FREE;
    }

    /* cppcheck-suppress misra-c2012-11.5 */
    session->tx[0].buffer = (uint8_t *)usbh_alloc_mem(RNDIS_SLOT_SIZE);
    session->tx[0].utr = alloc_utr(session->udev);

    if ((session->tx[0].buffer == USBNULL) || (session->tx[0].utr == USBNULL))
    {
        session->failure_reason = RNDIS_FAILURE_ALLOCATION;
        rndis_bulk_free(session);
        return USBH_ERR_MEMORY_OUT;
    }

    session->tx[0].state = RNDIS_SLOT_FREE;
    session->resources_ready = 1U;
    RNDIS_DBGMSG("RNDIS: DMA buffers allocated through usbh_alloc_mem(); verify NVT_NONCACHEABLE placement in map.\n");
    (void)usbh_memory_used();
    return USBH_OK;
}

/// @cond HIDDEN_SYMBOLS

/**
 * @brief Stop bulk I/O and release all session-owned transfer resources.
 * @param[in,out] session Session whose bulk and control resources are released.
 * @note Safe for a NULL session. The function first invalidates the private I/O
 *       generation, then releases notification, RX, TX, and control resources;
 *       it must run outside a disabled-interrupt section while aborting UTRs.
 */
void rndis_bulk_free(RNDIS_SESSION_T *session)
{
    uint32_t index;
    uint32_t primask;

    if (session == USBNULL)
    {
        return;
    }

    /*
     * Phase 1: block new submissions and make in-flight UTR callbacks stale.
     * The public session identity is immutable after creation; only this private
     * non-zero I/O epoch is invalidated during cleanup or recovery.
     */
    primask = rndis_enter_critical();
    session->stopping = 1U;
    session->io_generation = rndis_next_io_generation(session->io_generation);

    for (index = 0U; index < RNDIS_RX_SLOT_COUNT; index++)
    {
        rndis_stop_slot(&session->rx[index]);
    }

    rndis_stop_slot(&session->tx[0]);
    rndis_exit_critical(primask);

    /* Phase 2: abort only with interrupts enabled, then release returned UTRs. */
    rndis_ctrl_free_notification(session);

    for (index = 0U; index < RNDIS_RX_SLOT_COUNT; index++)
    {
        rndis_release_slot(&session->rx[index], RNDIS_SLOT_SIZE);
    }

    rndis_release_slot(&session->tx[0], RNDIS_SLOT_SIZE);

    if (session->ctrl_response != USBNULL)
    {
        (void)usbh_free_mem(session->ctrl_response, RNDIS_CTRL_RESPONSE_SIZE);
        session->ctrl_response = USBNULL;
    }

    if (session->ctrl_command != USBNULL)
    {
        (void)usbh_free_mem(session->ctrl_command, RNDIS_CTRL_COMMAND_SIZE);
        session->ctrl_command = USBNULL;
    }

    session->resources_ready = 0U;
}

/**
 * @brief Validate and dispatch one completed RNDIS RX packet.
 * @param[in,out] session Session receiving RX counters and callback dispatch.
 * @param[in] slot Completed RX slot containing the received DMA buffer.
 * @note Worker-context function. The callback, when configured, receives a
 *       pointer into the slot buffer and must consume it before the slot is
 *       reused; the callback must not retain or free that pointer.
 */
static void rndis_process_rx(RNDIS_SESSION_T *session, RNDIS_SLOT_T *slot)
{
    uint32_t actual = slot->actual_length;
    uint32_t message_length;
    uint32_t data_offset;
    uint32_t data_length;
    uint32_t data_start;
    uint32_t data_end;

    if (slot->status != USBH_OK)
    {
        session->stats.rx_drops++;
        session->stats.transfer_errors++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_USB_ERROR, slot, 0U, 0U, 0U, 0U, 0U);
        return;
    }

    if (actual < 44U)
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_SHORT_ACTUAL, slot, 0U, 0U, 0U, 0U, 0U);
        return;
    }

    if (actual > RNDIS_SLOT_SIZE)
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_ACTUAL_EXCEEDS_SLOT, slot, 0U, 0U, 0U, 0U, 0U);
        return;
    }

    message_length = rndis_get_le32(&slot->buffer[4]);
    data_offset = rndis_get_le32(&slot->buffer[8]);
    data_length = rndis_get_le32(&slot->buffer[12]);

    if (rndis_get_le32(&slot->buffer[0]) != RNDIS_MSG_PACKET)
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_MESSAGE_TYPE, slot, 1U,
                                    rndis_get_le32(&slot->buffer[0]), message_length, data_offset, data_length);
        return;
    }

    if ((message_length != actual) || (message_length > RNDIS_SLOT_SIZE))
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_MESSAGE_LENGTH_MISMATCH, slot, 1U,
                                    RNDIS_MSG_PACKET, message_length, data_offset, data_length);
        return;
    }

    if (data_offset > (UINT32_MAX - 8U))
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_OFFSET_ADDITION_OVERFLOW, slot, 1U,
                                    RNDIS_MSG_PACKET, message_length, data_offset, data_length);
        return;
    }

    data_start = 8U + data_offset;

    if ((data_start > (UINT32_MAX - data_length)))
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_DATA_START_LENGTH_OVERFLOW, slot, 1U,
                                    RNDIS_MSG_PACKET, message_length, data_offset, data_length);
        return;
    }

    data_end = data_start + data_length;

    if (data_start < 44U)
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_DATA_START_IN_FIXED_HEADER, slot, 1U,
                                    RNDIS_MSG_PACKET, message_length, data_offset, data_length);
        return;
    }

    if (data_end > message_length)
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_DATA_END_EXCEEDS_MESSAGE, slot, 1U,
                                    RNDIS_MSG_PACKET, message_length, data_offset, data_length);
        return;
    }

    if (data_length < 14U)
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_ETHERNET_TOO_SHORT, slot, 1U,
                                    RNDIS_MSG_PACKET, message_length, data_offset, data_length);
        return;
    }

    if ((data_length > session->max_frame) ||
            ((session->max_total != 0U) && (data_length > session->max_total)))
    {
        session->stats.rx_drops++;
        rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_DATA_EXCEEDS_LIMIT, slot, 1U,
                                    RNDIS_MSG_PACKET, message_length, data_offset, data_length);
        return;
    }

    session->stats.rx_packets++;
    session->stats.rx_bytes += data_length;
    rndis_publish_rx_diagnostic(session, RNDIS_RX_DIAG_ACCEPTED, slot, 1U,
                                RNDIS_MSG_PACKET, message_length, data_offset, data_length);

    if (session->rx_callback != USBNULL)
    {
        session->rx_callback(session, &slot->buffer[data_start], (uint16_t)data_length);
    }
}

/**
 * @brief Process completed bulk transfers and schedule the next transfers.
 * @param[in,out] session Running session whose RX/TX slots are serviced.
 * @note Worker-context, non-blocking API. At most one RX slot is submitted at
 *       a time because the supported EHCI schedule uses one bulk QH per
 *       endpoint; the single TX slot provides bounded back-pressure.
 */
void rndis_bulk_poll(RNDIS_SESSION_T *session)
{

    if ((session == USBNULL) || (session->resources_ready == 0U) || (session->stopping != 0U))
    {
        return;
    }

    if (session->state == RNDIS_STATE_RUNNING)
    {
        uint32_t index;

        for (index = 0U; index < RNDIS_RX_SLOT_COUNT; index++)
        {
            RNDIS_SLOT_T *slot = &session->rx[index];

            if (slot->state == RNDIS_SLOT_COMPLETED)
            {
                rndis_process_rx(session, slot);
                slot->state = RNDIS_SLOT_FREE;
            }

        }

        /* EHCI has one QH per bulk endpoint; only one RX UTR may own it at a time. */
        if (rndis_rx_is_active(session) == 0)
        {
            for (index = 0U; index < RNDIS_RX_SLOT_COUNT; index++)
            {
                RNDIS_SLOT_T *slot = &session->rx[index];

                if (slot->state == RNDIS_SLOT_FREE)
                {
                    (void)rndis_submit_rx(session, slot);
                    break;
                }
            }
        }

        if (session->tx[0].state == RNDIS_SLOT_READY)
        {
            (void)rndis_submit_tx(session, &session->tx[0]);
        }
        else if (session->tx[0].state == RNDIS_SLOT_COMPLETED)
        {
            RNDIS_SLOT_T *slot = &session->tx[0];

            rndis_complete_tx_worker(session, slot);
        }
        else
        {
            /* No TX slot is ready or completed; nothing to do. */
        }
    }
}

/**
 * @brief Atomically copy the latest normal TX completion snapshot.
 * @param[in] session Live session containing the TX outcome fields.
 * @param[out] outcome Destination for the scalar completion snapshot.
 * @return RNDIS_OK on success; otherwise RNDIS_ERR_NOT_READY for a NULL
 *         session or destination.
 * @note Worker-context API. The result identifies local Host-side completion
 *       only and does not indicate peer receipt or forwarding.
 */
int32_t rndis_bulk_get_tx_outcome(RNDIS_SESSION_T const *session,
                                  RNDIS_TX_OUTCOME_T *outcome)
{
    uint32_t primask;

    if ((session == USBNULL) || (outcome == USBNULL))
    {
        return RNDIS_ERR_NOT_READY;
    }

    primask = rndis_enter_critical();
    outcome->submit_id = session->tx_last_submit_id;
    outcome->completion_status = session->tx_last_status;
    outcome->actual_length = session->tx_last_actual_length;
    outcome->expected_wire_length = session->tx_last_expected_wire_length;
    outcome->completion_sequence = session->tx_completion_sequence;
    outcome->abort_pending = session->tx_abort_pending;
    rndis_exit_critical(primask);
    return RNDIS_OK;
}

/**
 * @brief Copy and queue one Ethernet frame in the single bounded TX slot.
 * @param[in,out] session Running RNDIS session.
 * @param[in] frame Ethernet frame to copy into the session-owned DMA buffer.
 * @param[in] frame_len Ethernet frame length in bytes.
 * @param[out] submit_id Optional destination for the non-zero accepted TX ID.
 * @return RNDIS_OK when the frame is copied and queued; otherwise
 *         RNDIS_ERR_NOT_READY, RNDIS_ERR_FRAME_SIZE, or RNDIS_ERR_BUSY.
 * @note Worker-context API. The frame is copied before return, while the slot
 *       remains owned by the driver until asynchronous completion processing.
 */
int32_t rndis_bulk_send(RNDIS_SESSION_T *session, uint8_t const *frame, uint16_t frame_len,
                        uint32_t *submit_id)
{
    RNDIS_SLOT_T *slot;
    uint32_t primask;
    uint32_t message_length;
    uint32_t packet_limit;

    if ((session == USBNULL) || (frame == USBNULL) || (session->state != RNDIS_STATE_RUNNING) ||
            (session->stopping != 0U))
    {
        return RNDIS_ERR_NOT_READY;
    }

    message_length = (uint32_t)frame_len;

    if (message_length > (UINT32_MAX - 44U))
    {
        session->stats.tx_drops++;
        return RNDIS_ERR_FRAME_SIZE;
    }

    message_length += 44U;
    packet_limit = rndis_tx_packet_total_limit(session);

    if ((frame_len < 14U) || (frame_len > session->max_frame) ||
            (message_length > packet_limit) || (message_length > RNDIS_SLOT_SIZE))
    {
        session->stats.tx_drops++;
        return RNDIS_ERR_FRAME_SIZE;
    }

    primask = rndis_enter_critical();
    slot = &session->tx[0];

    if (slot->state != RNDIS_SLOT_FREE)
    {
        rndis_exit_critical(primask);
        session->stats.tx_drops++;
        return RNDIS_ERR_BUSY;
    }

    slot->state = RNDIS_SLOT_FILLING;
    slot->submit_id = rndis_next_submit_id(session);

    if (submit_id != USBNULL)
    {
        *submit_id = slot->submit_id;
    }

    rndis_exit_critical(primask);

    (void)memset(slot->buffer, 0, 44U);
    rndis_put_le32(&slot->buffer[0], RNDIS_MSG_PACKET);
    rndis_put_le32(&slot->buffer[4], message_length);
    rndis_put_le32(&slot->buffer[8], 36U);
    rndis_put_le32(&slot->buffer[12], (uint32_t)frame_len);
    (void)memcpy(&slot->buffer[44], frame, frame_len);
    slot->actual_length = message_length;
    slot->expected_wire_length = message_length;
    slot->io_generation = session->io_generation;
    __DMB();
    slot->state = RNDIS_SLOT_READY;
    return RNDIS_OK;
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief Copy and queue one Ethernet frame through the normal RNDIS TX path.
 * @param[in,out] session Running RNDIS session.
 * @param[in] frame Ethernet frame to copy.
 * @param[in] frame_len Ethernet frame length in bytes.
 * @return RNDIS_OK when accepted; otherwise a RNDIS readiness, size, or busy
 *         error code.
 * @note This public wrapper does not expose the asynchronous submission ID.
 *       It must be called from worker context, never from an ISR.
 */
int32_t usbh_rndis_send_frame(RNDIS_SESSION_T *session, uint8_t const *frame, uint16_t frame_len)
{
    return rndis_bulk_send(session, frame, frame_len, USBNULL);
}

/*! @}*/ /* end of group USBH_EXPORTED_FUNCTIONS */

/*! @}*/ /* end of group USBH_Library */

/*! @}*/ /* end of group LIBRARY */
