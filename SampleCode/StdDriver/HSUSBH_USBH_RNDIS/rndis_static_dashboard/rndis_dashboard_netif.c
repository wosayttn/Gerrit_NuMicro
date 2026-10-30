/**************************************************************************//**
 * @file     rndis_dashboard_netif.c
 * @brief    DHCP lifecycle for the fixed HTTP raw TCP service.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "usbh_rndis.h"
#include "rndis_dashboard_netif.h"
#include "rndis_dashboard_http.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#include "lwip/def.h"
#include "lwip/err.h"
#include "lwip/etharp.h"
#include "lwip/init.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"

#if (NO_SYS != 1) || (LWIP_CALLBACK_API != 1) || (LWIP_DHCP != 1) || \
    (LWIP_UDP != 1) || (LWIP_TCP != 1) || (LWIP_RAW != 0) || \
    (LWIP_NETCONN != 0) || (LWIP_SOCKET != 0) || (LWIP_ALTCP != 0) || \
    (LWIP_DNS != 0) || (LWIP_AUTOIP != 0) || (LWIP_IPV6 != 0) || \
    (LWIP_ICMP != 0) || (LWIP_IGMP != 0) || (IP_FORWARD != 0) || \
    (IP_REASSEMBLY != 0) || (IP_FRAG != 0)
    #error RNDIS Dashboard requires the fixed HTTP NO_SYS raw TCP lwIP configuration
#endif

#define RNDIS_DASHBOARD_FRAME_MAX 1514U
#define RNDIS_DASHBOARD_TIMER_GAP_LIMIT_MS 250U
#define RNDIS_DASHBOARD_LEASE_DEADLINE_MAX_SECONDS (0x7FFFFFFFUL / 1000UL)

typedef enum
{
    RNDIS_DASHBOARD_IDLE = 0,
    RNDIS_DASHBOARD_RUNNING,
    RNDIS_DASHBOARD_BOUND,
    RNDIS_DASHBOARD_TERMINAL
} RNDIS_DASHBOARD_PHASE_E;

/**
 * @brief Runtime state for the fixed NO_SYS RNDIS Dashboard netif.
 * @details Owns the lwIP netif/DHCP state, bounded RX handoff storage, and
 *          per-generation lifecycle counters for the dashboard worker.
 */
typedef struct
{
    struct netif netif;                         /**< lwIP network interface owned by the dashboard. */
    struct dhcp dhcp;                           /**< DHCP client state associated with @ref netif. */
    RNDIS_SESSION_T *session;                    /**< Current RNDIS session bound to the dashboard. */
    uint8_t tx_staging[RNDIS_DASHBOARD_FRAME_MAX]; /**< Bounded staging buffer for lwIP TX frames. */
    uint8_t rx_handoff[RNDIS_DASHBOARD_FRAME_MAX]; /**< Bounded worker-to-lwIP RX handoff buffer. */
    uint32_t generation;                        /**< Current public RNDIS session generation. */
    uint32_t iteration_generation;              /**< Generation accepted for the current worker iteration. */
    uint32_t rx_handoff_generation;              /**< Generation that produced the pending RX handoff. */
    uint32_t wait_state_generation;              /**< Generation associated with the wait-state log mask. */
    uint32_t timer_last_ms;                     /**< Tick at which lwIP timers were last serviced. */
    uint32_t timer_max_gap_ms;                  /**< Maximum observed timer service gap in milliseconds. */
    uint32_t deadline_ms;                       /**< DHCP acquisition deadline in milliseconds. */
    uint32_t lease_deadline_ms;                 /**< Captured DHCP lease expiry deadline in milliseconds. */
    uint32_t rx_frames;                         /**< Number of Ethernet frames accepted into lwIP. */
    uint32_t rx_drops;                          /**< Total RX frames dropped by validation or processing. */
    uint32_t rx_callbacks;                      /**< Number of invocations of the RNDIS RX callback. */
    uint32_t rx_gate_drops;                     /**< RX drops caused by lifecycle or binding gates. */
    uint32_t rx_handoff_drops;                  /**< RX drops caused by an already occupied handoff buffer. */
    uint32_t rx_handoff_stale;                  /**< RX handoffs rejected because their generation was stale. */
    uint32_t rx_alloc_failures;                 /**< RX pbuf allocation failures. */
    uint32_t rx_copy_failures;                  /**< RX pbuf copy failures. */
    uint32_t rx_input_failures;                 /**< lwIP ethernet_input failures. */
    uint32_t tx_frames;                         /**< Ethernet frames accepted for RNDIS TX. */
    uint32_t tx_failures;                       /**< TX staging, binding, or RNDIS submission failures. */
    uint16_t rx_handoff_length;                 /**< Length of the pending RX handoff in bytes. */
    uint8_t wait_state_mask;                    /**< Bit mask suppressing duplicate wait-state logs. */
    uint8_t netif_added;                        /**< Non-zero after netif_add() succeeds. */
    uint8_t lwip_initialized;                   /**< Non-zero after lwip_init() has run. */
    uint8_t link_enabled;                       /**< Non-zero while the dashboard netif link is enabled. */
    uint8_t rx_enabled;                         /**< Non-zero while RNDIS RX callback ingress is enabled. */
    uint8_t rx_handoff_pending;                 /**< Non-zero when the RX handoff awaits lwIP processing. */
    uint8_t tx_enabled;                         /**< Non-zero while dashboard TX binding is permitted. */
    uint8_t dhcp_associated;                    /**< Non-zero while the netif owns an associated DHCP state. */
    uint8_t dhcp_start_called;                  /**< Non-zero after DHCP start was called for this iteration. */
    uint8_t lease_deadline_enabled;             /**< Non-zero when an explicit lease deadline is enforced. */
    uint8_t tcp_ready;                          /**< Non-zero after DHCP bound and HTTP service is ready. */
    uint8_t detached_latched;                  /**< Prevents repeated detach handling for one absence event. */
    RNDIS_DASHBOARD_PHASE_E phase;              /**< Dashboard lifecycle phase. */
} RNDIS_DASHBOARD_CONTEXT_T;

static RNDIS_DASHBOARD_CONTEXT_T s_rndis_dashboard;

/**
 * @brief Clear the deferred RX handoff state.
 * @note Worker-context helper. The handoff buffer contents are not erased;
 *       only its ownership and length metadata are invalidated.
 */
static void rndis_dashboard_handoff_clear(void)
{
    s_rndis_dashboard.rx_handoff_pending = 0U;
    s_rndis_dashboard.rx_handoff_generation = 0U;
    s_rndis_dashboard.rx_handoff_length = 0U;
}

/**
 * @brief Test whether a wrap-safe millisecond deadline has been reached.
 * @param[in] now Current millisecond tick.
 * @param[in] deadline Absolute deadline tick.
 * @return 1 when the deadline is reached or passed; otherwise 0.
 */
static uint8_t rndis_dashboard_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

/**
 * @brief Read the non-zero public generation of a live RNDIS session.
 * @param[in] session Session pointer obtained from the current RNDIS list.
 * @param[out] generation Destination for the session generation.
 * @return 1 when the generation was read successfully; otherwise 0.
 * @note The session pointer is valid only for its current attach lifecycle.
 */
static uint8_t rndis_dashboard_get_generation(RNDIS_SESSION_T *session, uint32_t *generation)
{
    return ((session != NULL) && (generation != NULL) &&
            (usbh_rndis_get_session_generation(session, generation) == RNDIS_OK) &&
            (*generation != 0U)) ? 1U : 0U;
}

/**
 * @brief Reset the once-per-generation RNDIS wait-state reporting mask.
 */
static void rndis_dashboard_wait_state_reset(void)
{
    s_rndis_dashboard.wait_state_generation = 0U;
    s_rndis_dashboard.wait_state_mask = 0U;
}

/**
 * @brief Report that the dashboard is waiting for a specific RNDIS state.
 * @param[in] generation Session generation being waited on.
 * @param[in] state RNDIS state that has not yet reached RUNNING.
 * @note Each generation/state pair is printed at most once.
 */
static void rndis_dashboard_wait_rndis(uint32_t generation, RNDIS_STATE_E state)
{
    uint8_t state_bit = (uint8_t)(1UL << (uint32_t)state);

    if (s_rndis_dashboard.wait_state_generation != generation)
    {
        s_rndis_dashboard.wait_state_generation = generation;
        s_rndis_dashboard.wait_state_mask = 0U;
    }

    if ((s_rndis_dashboard.wait_state_mask & state_bit) == 0U)
    {
        s_rndis_dashboard.wait_state_mask |= state_bit;
        printf("RNDIS_DASHBOARD_WAIT_RNDIS gen=%lu state=%lu\n",
               (unsigned long)generation, (unsigned long)state);
    }
}

/**
 * @brief Validate a unicast Ethernet MAC address.
 * @param[in] mac Six-byte Ethernet address to validate.
 * @return 1 for a non-zero, non-broadcast unicast address; otherwise 0.
 */
static uint8_t rndis_dashboard_mac_valid(uint8_t const mac[ETH_HWADDR_LEN])
{
    uint8_t index;
    uint8_t all_zero = 1U;
    uint8_t all_ff = 1U;

    if ((mac[0] & 1U) != 0U)
    {
        return 0U;
    }

    for (index = 0U; index < ETH_HWADDR_LEN; index++)
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

    return ((all_zero == 0U) && (all_ff == 0U)) ? 1U : 0U;
}

/**
 * @brief Verify that a session is running and expose its generation and MAC.
 * @param[in] session Candidate RNDIS session.
 * @param[out] generation Destination for the session generation.
 * @param[out] mac Destination for the negotiated MAC address.
 * @return 1 when the session is running and has a valid MAC; otherwise 0.
 */
static uint8_t rndis_dashboard_running(RNDIS_SESSION_T *session, uint32_t *generation,
                                       uint8_t mac[ETH_HWADDR_LEN])
{
    return ((rndis_dashboard_get_generation(session, generation) != 0U) &&
            (usbh_rndis_get_state(session) == RNDIS_STATE_RUNNING) &&
            (usbh_rndis_get_mac_address(session, mac) == RNDIS_OK) &&
            (rndis_dashboard_mac_valid(mac) != 0U)) ? 1U : 0U;
}

/**
 * @brief Validate that the dashboard may use the current RNDIS/netif binding.
 * @param[in] session Candidate RNDIS session.
 * @return 1 when generation, link, netif, and TX ownership are all valid;
 *         otherwise 0.
 * @note This gate rejects stale session pointers and detached or terminal
 *       dashboard states.
 */
static uint8_t rndis_dashboard_binding(RNDIS_SESSION_T *session)
{
    uint32_t generation;

    if ((session == NULL) || (s_rndis_dashboard.phase == RNDIS_DASHBOARD_TERMINAL) ||
            (s_rndis_dashboard.session != session) || (s_rndis_dashboard.iteration_generation == 0U) ||
            (s_rndis_dashboard.generation != s_rndis_dashboard.iteration_generation) ||
            (s_rndis_dashboard.link_enabled == 0U) || (s_rndis_dashboard.tx_enabled == 0U) ||
            (netif_is_up(&s_rndis_dashboard.netif) == 0) ||
            (netif_is_link_up(&s_rndis_dashboard.netif) == 0))
    {
        return 0U;
    }

    return ((usbh_rndis_get_session_generation(session, &generation) == RNDIS_OK) &&
            (generation == s_rndis_dashboard.iteration_generation) &&
            (usbh_rndis_get_state(session) == RNDIS_STATE_RUNNING)) ? 1U : 0U;
}

/**
 * @brief Convert one lwIP pbuf chain into a RNDIS Ethernet TX submission.
 * @param[in] netif lwIP network interface invoking the callback.
 * @param[in] pbuf Packet buffer chain containing the Ethernet frame.
 * @return ERR_OK when queued to RNDIS; ERR_MEM for temporary TX resource
 *         exhaustion; otherwise ERR_IF.
 * @note Called by lwIP in worker context. The frame is copied into a bounded
 *       staging buffer before the RNDIS driver is called.
 */
static err_t rndis_dashboard_linkoutput(struct netif *netif, struct pbuf *pbuf)
{
    uint16_t copied;
    int32_t status;

    (void)netif;

    if ((pbuf == NULL) || (pbuf->tot_len > RNDIS_DASHBOARD_FRAME_MAX) ||
            (rndis_dashboard_binding(s_rndis_dashboard.session) == 0U))
    {
        s_rndis_dashboard.tx_failures++;
        return ERR_IF;
    }

    copied = pbuf_copy_partial(pbuf, s_rndis_dashboard.tx_staging, pbuf->tot_len, 0U);

    if (copied != pbuf->tot_len)
    {
        s_rndis_dashboard.tx_failures++;
        return ERR_IF;
    }

    status = usbh_rndis_send_frame(s_rndis_dashboard.session, s_rndis_dashboard.tx_staging, pbuf->tot_len);

    if (status == RNDIS_OK)
    {
        s_rndis_dashboard.tx_frames++;
        return ERR_OK;
    }

    s_rndis_dashboard.tx_failures++;
    return ((status == RNDIS_ERR_BUSY) || (status == RNDIS_ERR_NO_RESOURCE)) ? ERR_MEM : ERR_IF;
}

/**
 * @brief Initialize the lwIP netif callbacks and fixed Ethernet properties.
 * @param[in,out] netif Network interface being initialized by lwIP.
 * @return ERR_OK after the callbacks and MTU are configured.
 */
static err_t rndis_dashboard_netif_init(struct netif *netif)
{
    netif->name[0] = 'r';
    netif->name[1] = '1';
    netif->state = NULL;
    netif->output = etharp_output;
    netif->linkoutput = rndis_dashboard_linkoutput;
    netif->mtu = 1500U;
    netif->hwaddr_len = ETH_HWADDR_LEN;
    netif->flags = (uint8_t)(NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHERNET);
    return ERR_OK;
}

/**
 * @brief Stop DHCP and clear the dashboard-owned DHCP state.
 * @note Safe to call repeatedly. The netif remains allocated, but no DHCP
 *       client association remains active afterwards.
 */
static void rndis_dashboard_stop_dhcp(void)
{
    if (s_rndis_dashboard.dhcp_associated != 0U)
    {
        dhcp_stop(&s_rndis_dashboard.netif);
        dhcp_remove_struct(&s_rndis_dashboard.netif);
        s_rndis_dashboard.dhcp_associated = 0U;
    }

    (void)memset(&s_rndis_dashboard.dhcp, 0, sizeof(s_rndis_dashboard.dhcp));
}

/**
 * @brief Disable dashboard data paths and close the HTTP service state.
 * @param[in] reason Constant diagnostic reason for the fail-closed transition.
 * @note This function does not destroy the lwIP netif object; it only brings
 *       the link and protocol ownership down.
 */
static void rndis_dashboard_fail_closed(char const *reason)
{
    s_rndis_dashboard.tcp_ready = 0U;
    RndisDashboardHttpSetReady(s_rndis_dashboard.generation, 0U);
    s_rndis_dashboard.tx_enabled = 0U;
    s_rndis_dashboard.rx_enabled = 0U;
    rndis_dashboard_handoff_clear();
    RndisDashboardHttpQuiesce(reason, s_rndis_dashboard.generation);
    s_rndis_dashboard.session = NULL;
    s_rndis_dashboard.iteration_generation = 0U;

    if (s_rndis_dashboard.netif_added != 0U)
    {
        rndis_dashboard_stop_dhcp();
        netif_set_link_down(&s_rndis_dashboard.netif);
        netif_set_down(&s_rndis_dashboard.netif);
        etharp_cleanup_netif(&s_rndis_dashboard.netif);
        netif_set_addr(&s_rndis_dashboard.netif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4);
    }
    else
    {
        rndis_dashboard_stop_dhcp();
    }

    s_rndis_dashboard.link_enabled = 0U;
    s_rndis_dashboard.lease_deadline_enabled = 0U;
    s_rndis_dashboard.lease_deadline_ms = 0U;
}

/**
 * @brief Latch a terminal dashboard failure and fail closed.
 * @param[in] reason Constant diagnostic reason for the terminal state.
 * @note Once terminal, the current iteration remains disabled until release or
 *       a new session generation is explicitly initialized.
 */
static void rndis_dashboard_terminal(char const *reason)
{
    if (s_rndis_dashboard.phase != RNDIS_DASHBOARD_TERMINAL)
    {
        printf("RNDIS_DASHBOARD_TERMINAL gen=%lu reason=%s\n",
               (unsigned long)s_rndis_dashboard.generation, reason);
        s_rndis_dashboard.phase = RNDIS_DASHBOARD_TERMINAL;
    }

    rndis_dashboard_fail_closed(reason);
}

/**
 * @brief Release the current dashboard iteration and reset lifecycle state.
 * @note The lwIP core initialization flag is preserved so the NO_SYS lwIP
 *       stack is initialized only once for the application lifetime.
 */
static void rndis_dashboard_release(void)
{
    rndis_dashboard_fail_closed("RELEASE");
    rndis_dashboard_handoff_clear();
    rndis_dashboard_wait_state_reset();
    s_rndis_dashboard.generation = 0U;
    s_rndis_dashboard.deadline_ms = 0U;
    s_rndis_dashboard.timer_last_ms = 0U;
    s_rndis_dashboard.timer_max_gap_ms = 0U;
    s_rndis_dashboard.dhcp_start_called = 0U;
    s_rndis_dashboard.phase = RNDIS_DASHBOARD_IDLE;
}

/**
 * @brief Validate an IPv4 netmask as a non-zero contiguous mask.
 * @param[in] mask IPv4 netmask in lwIP address representation.
 * @return 1 for a valid non-zero, non-host mask; otherwise 0.
 */
static uint8_t rndis_dashboard_mask_valid(ip4_addr_t const *mask)
{
    uint32_t host_mask = lwip_ntohl(mask->addr);

    if ((host_mask == 0U) || (host_mask == 0xFFFFFFFFUL))
    {
        return 0U;
    }

    return ((host_mask | (host_mask - 1U)) == 0xFFFFFFFFUL) ? 1U : 0U;
}

/**
 * @brief Validate that an IPv4 address is a usable unicast host address.
 * @param[in] address IPv4 address to inspect.
 * @param[in] mask Netmask defining the host portion.
 * @return 1 for an address in the usable unicast range and subnet;
 *         otherwise 0.
 */
static uint8_t rndis_dashboard_ip_unicast(ip4_addr_t const *address, ip4_addr_t const *mask)
{
    uint32_t value = lwip_ntohl(address->addr);
    uint32_t host_mask = ~lwip_ntohl(mask->addr);
    uint32_t host_part = value & host_mask;
    uint8_t first = (uint8_t)(value >> 24U);

    return ((first != 0U) && (first < 224U) && (host_part != 0U) &&
            (host_part != host_mask)) ? 1U : 0U;
}

/**
 * @brief Validate the currently assigned IPv4 address, mask, and gateway.
 * @return 1 when the lease contains usable same-subnet unicast values;
 *         otherwise 0.
 */
static uint8_t rndis_dashboard_lease_valid(void)
{
    ip4_addr_t const *ipaddr = netif_ip4_addr(&s_rndis_dashboard.netif);
    ip4_addr_t const *mask = netif_ip4_netmask(&s_rndis_dashboard.netif);
    ip4_addr_t const *gateway = netif_ip4_gw(&s_rndis_dashboard.netif);

    return ((rndis_dashboard_mask_valid(mask) != 0U) && (rndis_dashboard_ip_unicast(ipaddr, mask) != 0U) &&
            (rndis_dashboard_ip_unicast(gateway, mask) != 0U) &&
            (ip4_addr_netcmp(ipaddr, gateway, mask) != 0U)) ? 1U : 0U;
}

/**
 * @brief Test whether the lwIP DHCP state represents an active lease.
 * @return 1 for BOUND, RENEWING, or REBINDING; otherwise 0.
 */
static uint8_t rndis_dashboard_lease_state(void)
{
    return ((s_rndis_dashboard.dhcp.state == DHCP_STATE_BOUND) ||
            (s_rndis_dashboard.dhcp.state == DHCP_STATE_RENEWING) ||
            (s_rndis_dashboard.dhcp.state == DHCP_STATE_REBINDING)) ? 1U : 0U;
}

/**
 * @brief Capture the finite DHCP lease deadline for dashboard enforcement.
 * @param[in] now Current millisecond tick.
 * @return 1 when the deadline is disabled or captured successfully; otherwise 0
 *         when the lease duration cannot be represented safely.
 */
static uint8_t rndis_dashboard_capture_lease_deadline(uint32_t now)
{
    if (s_rndis_dashboard.dhcp.t0_timeout == 0U)
    {
        return 1U;
    }

    if (s_rndis_dashboard.dhcp.offered_t0_lease > RNDIS_DASHBOARD_LEASE_DEADLINE_MAX_SECONDS)
    {
        return 0U;
    }

    s_rndis_dashboard.lease_deadline_ms = now + (s_rndis_dashboard.dhcp.offered_t0_lease * 1000UL);
    s_rndis_dashboard.lease_deadline_enabled = 1U;
    s_rndis_dashboard.dhcp.t0_timeout = 0U;
    return 1U;
}

/**
 * @brief Validate an already-bound dashboard lease and enforce its deadline.
 * @param[in] now Current millisecond tick.
 * @return 1 when ingress may continue; otherwise 0 after entering terminal state.
 */
static uint8_t rndis_dashboard_bound_ingress_valid(uint32_t now)
{
    if (s_rndis_dashboard.phase != RNDIS_DASHBOARD_BOUND)
    {
        return 1U;
    }

    if ((s_rndis_dashboard.lease_deadline_enabled != 0U) &&
            (rndis_dashboard_reached(now, s_rndis_dashboard.lease_deadline_ms) != 0U))
    {
        rndis_dashboard_terminal("DHCP_BOUND_LEASE_EXPIRED");
        return 0U;
    }

    if ((rndis_dashboard_lease_state() == 0U) || (rndis_dashboard_lease_valid() == 0U) ||
            (rndis_dashboard_capture_lease_deadline(now) == 0U))
    {
        rndis_dashboard_terminal("DHCP_BOUND_LEASE_LOST");
        return 0U;
    }

    return 1U;
}

/**
 * @brief Receive one accepted Ethernet frame from the RNDIS worker.
 * @param[in] session RNDIS session that produced the frame.
 * @param[in] frame Driver-owned Ethernet frame buffer.
 * @param[in] frame_length Ethernet frame length in bytes.
 * @note Worker-context callback. The frame is copied into the single handoff
 *       buffer and must not be retained by this function.
 */
static void rndis_dashboard_rx(RNDIS_SESSION_T *session, uint8_t const *frame, uint16_t frame_length)
{
    s_rndis_dashboard.rx_callbacks++;

    if ((session == NULL) || (frame == NULL) || (frame_length == 0U) ||
            (frame_length > RNDIS_DASHBOARD_FRAME_MAX) || (s_rndis_dashboard.rx_enabled == 0U) ||
            (s_rndis_dashboard.session != session) || (s_rndis_dashboard.phase == RNDIS_DASHBOARD_TERMINAL) ||
            (s_rndis_dashboard.iteration_generation == 0U) ||
            (s_rndis_dashboard.generation != s_rndis_dashboard.iteration_generation))
    {
        s_rndis_dashboard.rx_gate_drops++;
        s_rndis_dashboard.rx_drops++;
        return;
    }

    if (s_rndis_dashboard.rx_handoff_pending != 0U)
    {
        s_rndis_dashboard.rx_handoff_drops++;
        s_rndis_dashboard.rx_drops++;
        return;
    }

    (void)memcpy(s_rndis_dashboard.rx_handoff, frame, frame_length);
    s_rndis_dashboard.rx_handoff_length = frame_length;
    s_rndis_dashboard.rx_handoff_generation = s_rndis_dashboard.iteration_generation;
    s_rndis_dashboard.rx_handoff_pending = 1U;
}

/**
 * @brief Move one deferred RNDIS frame into lwIP as a pbuf.
 * @param[in] session Current RNDIS session used for binding validation.
 * @note Worker-context function. It consumes the pending handoff and enters
 *       terminal state if allocation, copy, or ethernet_input fails.
 */
static void rndis_dashboard_rx_drain(RNDIS_SESSION_T *session)
{
    struct pbuf *pbuf;
    uint16_t frame_length;
    uint32_t generation;
    err_t status;

    if (s_rndis_dashboard.rx_handoff_pending == 0U)
    {
        return;
    }

    frame_length = s_rndis_dashboard.rx_handoff_length;
    generation = s_rndis_dashboard.rx_handoff_generation;
    rndis_dashboard_handoff_clear();

    if ((frame_length == 0U) || (frame_length > RNDIS_DASHBOARD_FRAME_MAX) ||
            (generation == 0U) || (generation != s_rndis_dashboard.iteration_generation) ||
            (generation != s_rndis_dashboard.generation) || (rndis_dashboard_binding(session) == 0U))
    {
        s_rndis_dashboard.rx_handoff_stale++;
        s_rndis_dashboard.rx_drops++;
        return;
    }

    pbuf = pbuf_alloc(PBUF_RAW, frame_length, PBUF_POOL);

    if (pbuf == NULL)
    {
        s_rndis_dashboard.rx_alloc_failures++;
        s_rndis_dashboard.rx_drops++;
        rndis_dashboard_terminal("RX_PBUF_ALLOC_FAILED");
        return;
    }

    if (pbuf_take(pbuf, s_rndis_dashboard.rx_handoff, frame_length) != ERR_OK)
    {
        (void)pbuf_free(pbuf);
        s_rndis_dashboard.rx_copy_failures++;
        s_rndis_dashboard.rx_drops++;
        rndis_dashboard_terminal("RX_PBUF_COPY_FAILED");
        return;
    }

    status = ethernet_input(pbuf, &s_rndis_dashboard.netif);

    if (status != ERR_OK)
    {
        s_rndis_dashboard.rx_input_failures++;
        s_rndis_dashboard.rx_drops++;
        rndis_dashboard_terminal("RX_INPUT_FAILED");
        return;
    }

    s_rndis_dashboard.rx_frames++;
}

/**
 * @brief Associate DHCP with the netif and start the dashboard DHCP phase.
 * @param[in] session Running RNDIS session owning the netif binding.
 * @note The function is idempotent for an already-started DHCP iteration.
 */
static void rndis_dashboard_start(RNDIS_SESSION_T *session)
{
    err_t status;
    uint32_t now;

    if ((s_rndis_dashboard.dhcp_start_called != 0U) || (rndis_dashboard_binding(session) == 0U))
    {
        return;
    }

    dhcp_set_struct(&s_rndis_dashboard.netif, &s_rndis_dashboard.dhcp);
    s_rndis_dashboard.dhcp_associated = 1U;
    s_rndis_dashboard.dhcp_start_called = 1U;
    now = sys_now();
    s_rndis_dashboard.phase = RNDIS_DASHBOARD_RUNNING;
    s_rndis_dashboard.timer_last_ms = now;
    s_rndis_dashboard.timer_max_gap_ms = 0U;
    s_rndis_dashboard.deadline_ms = now + RNDIS_DASHBOARD_DHCP_DEADLINE_MS;
    status = dhcp_start(&s_rndis_dashboard.netif);

    if (status != ERR_OK)
    {
        rndis_dashboard_terminal("DHCP_START_FAILED");
        return;
    }

    printf("RNDIS_DASHBOARD_WAIT_IPV4 gen=%lu deadline_ms=%lu\n",
           (unsigned long)s_rndis_dashboard.generation, (unsigned long)s_rndis_dashboard.deadline_ms);
}

/**
 * @brief Service lwIP timers, DHCP transitions, lease checks, and HTTP work.
 * @param[in] session Current RNDIS session used for binding validation.
 * @note Worker-context, non-blocking service function. A timer gap beyond the
 *       configured limit or a lost lease causes a fail-closed transition.
 */
static void rndis_dashboard_service(RNDIS_SESSION_T *session)
{
    uint32_t now;
    uint32_t gap;

    if ((rndis_dashboard_binding(session) == 0U) ||
            ((s_rndis_dashboard.phase != RNDIS_DASHBOARD_RUNNING) && (s_rndis_dashboard.phase != RNDIS_DASHBOARD_BOUND)))
    {
        return;
    }

    now = sys_now();
    gap = now - s_rndis_dashboard.timer_last_ms;

    if (gap > s_rndis_dashboard.timer_max_gap_ms)
    {
        s_rndis_dashboard.timer_max_gap_ms = gap;
    }

    if (gap > RNDIS_DASHBOARD_TIMER_GAP_LIMIT_MS)
    {
        rndis_dashboard_terminal("DHCP_TIMER_SERVICE_GAP");
        return;
    }

    if ((s_rndis_dashboard.phase == RNDIS_DASHBOARD_RUNNING) && rndis_dashboard_reached(now, s_rndis_dashboard.deadline_ms))
    {
        rndis_dashboard_terminal("DHCP_TIMEOUT");
        return;
    }

    sys_check_timeouts();
    s_rndis_dashboard.timer_last_ms = now;

    if (s_rndis_dashboard.phase == RNDIS_DASHBOARD_RUNNING)
    {
        if (s_rndis_dashboard.dhcp.state != DHCP_STATE_BOUND)
        {
            return;
        }

        s_rndis_dashboard.phase = RNDIS_DASHBOARD_BOUND;

        if (rndis_dashboard_bound_ingress_valid(now) == 0U)
        {
            return;
        }

        printf("RNDIS_DASHBOARD_DHCP_BOUND gen=%lu ip=%lu.%lu.%lu.%lu max_gap_ms=%lu\n",
               (unsigned long)s_rndis_dashboard.generation,
               (unsigned long)((lwip_ntohl(netif_ip4_addr(&s_rndis_dashboard.netif)->addr) >> 24U) & 0xFFUL),
               (unsigned long)((lwip_ntohl(netif_ip4_addr(&s_rndis_dashboard.netif)->addr) >> 16U) & 0xFFUL),
               (unsigned long)((lwip_ntohl(netif_ip4_addr(&s_rndis_dashboard.netif)->addr) >> 8U) & 0xFFUL),
               (unsigned long)(lwip_ntohl(netif_ip4_addr(&s_rndis_dashboard.netif)->addr) & 0xFFUL),
               (unsigned long)s_rndis_dashboard.timer_max_gap_ms);
        s_rndis_dashboard.tcp_ready = 1U;
        RndisDashboardHttpSetIpv4(netif_ip4_addr(&s_rndis_dashboard.netif));
        RndisDashboardHttpOnBound(s_rndis_dashboard.generation);
        RndisDashboardHttpSetReady(s_rndis_dashboard.generation, s_rndis_dashboard.tcp_ready);
    }

    if (rndis_dashboard_bound_ingress_valid(now) == 0U)
    {
        return;
    }

    RndisDashboardHttpService(s_rndis_dashboard.generation, s_rndis_dashboard.tcp_ready);
}

/**
 * @brief Start or advance one RNDIS Dashboard session generation.
 * @param[in] session Current RNDIS session, or NULL when detached.
 * @note Call from the application worker loop after RNDIS polling. This function
 *       initializes lwIP once, validates session replacement, configures the
 *       netif, and starts DHCP when the RNDIS state reaches RUNNING.
 */
void RndisDashboardNetifBeginIteration(RNDIS_SESSION_T *session)
{
    uint32_t generation = 0U;
    uint8_t mac[ETH_HWADDR_LEN];
    RNDIS_STATE_E state;

    if (s_rndis_dashboard.lwip_initialized == 0U)
    {
        lwip_init();
        s_rndis_dashboard.lwip_initialized = 1U;
        printf("RNDIS_DASHBOARD_INIT no_sys=1\n");
    }

    if (session == NULL)
    {
        if (s_rndis_dashboard.detached_latched == 0U)
        {
            rndis_dashboard_terminal("RNDIS_DETACHED");
            rndis_dashboard_release();
            s_rndis_dashboard.detached_latched = 1U;
        }

        return;
    }

    if (rndis_dashboard_get_generation(session, &generation) == 0U)
    {
        rndis_dashboard_terminal("RNDIS_GENERATION_UNAVAILABLE");
        return;
    }

    s_rndis_dashboard.detached_latched = 0U;

    if ((s_rndis_dashboard.generation != 0U) && (s_rndis_dashboard.generation != generation))
    {
        rndis_dashboard_terminal("RNDIS_NEW_GENERATION");
        rndis_dashboard_release();
    }

    if (s_rndis_dashboard.generation == 0U)
    {
        rndis_dashboard_wait_state_reset();
        s_rndis_dashboard.generation = generation;
    }

    state = usbh_rndis_get_state(session);

    if (s_rndis_dashboard.phase == RNDIS_DASHBOARD_TERMINAL)
    {
        rndis_dashboard_terminal("RNDIS_DASHBOARD_TERMINAL_LATCHED");
        return;
    }

    if (rndis_dashboard_running(session, &generation, mac) == 0U)
    {
        if ((state < RNDIS_STATE_RUNNING) && (s_rndis_dashboard.phase == RNDIS_DASHBOARD_IDLE) &&
                (s_rndis_dashboard.link_enabled == 0U) && (s_rndis_dashboard.session == NULL) &&
                (s_rndis_dashboard.iteration_generation == 0U))
        {
            rndis_dashboard_wait_rndis(generation, state);
            return;
        }

        rndis_dashboard_terminal("RNDIS_SESSION_NOT_RUNNING");
        return;
    }

    if (s_rndis_dashboard.link_enabled == 0U)
    {
        if (s_rndis_dashboard.netif_added == 0U)
        {
            if (netif_add(&s_rndis_dashboard.netif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4,
                          NULL, rndis_dashboard_netif_init, ethernet_input) == NULL)
            {
                rndis_dashboard_terminal("NETIF_ADD_FAILED");
                return;
            }

            s_rndis_dashboard.netif_added = 1U;
            netif_set_default(&s_rndis_dashboard.netif);
        }
        else
        {
            netif_set_addr(&s_rndis_dashboard.netif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4);
        }

        (void)memcpy(s_rndis_dashboard.netif.hwaddr, mac, ETH_HWADDR_LEN);

        if (usbh_rndis_set_rx_callback(session, rndis_dashboard_rx) != RNDIS_OK)
        {
            rndis_dashboard_terminal("RX_CALLBACK_FAILED");
            return;
        }

        s_rndis_dashboard.link_enabled = 1U;
        s_rndis_dashboard.rx_enabled = 1U;
        netif_set_up(&s_rndis_dashboard.netif);
        netif_set_link_up(&s_rndis_dashboard.netif);
        netif_set_flags(&s_rndis_dashboard.netif, NETIF_FLAG_ETHARP);
    }

    s_rndis_dashboard.session = session;
    s_rndis_dashboard.iteration_generation = generation;
    s_rndis_dashboard.tx_enabled = 1U;
    rndis_dashboard_start(session);
}

/**
 * @brief Complete one dashboard worker iteration.
 * @param[in] session Current RNDIS session, or NULL when detached.
 * @note Drains the deferred RX handoff, services lwIP timers and HTTP, and
 *       disables TX ownership at the end of the iteration.
 */
void RndisDashboardNetifEndIteration(RNDIS_SESSION_T *session)
{
    uint32_t generation;
    uint32_t now;

    if (session == NULL)
    {
        if (s_rndis_dashboard.detached_latched == 0U)
        {
            rndis_dashboard_terminal("RNDIS_DETACHED");
            rndis_dashboard_release();
            s_rndis_dashboard.detached_latched = 1U;
        }
    }
    else if (s_rndis_dashboard.session != NULL)
    {
        if (rndis_dashboard_get_generation(session, &generation) == 0U)
        {
            rndis_dashboard_terminal("RNDIS_GENERATION_UNAVAILABLE");
        }
        else if (generation != s_rndis_dashboard.generation)
        {
            rndis_dashboard_terminal("RNDIS_NEW_GENERATION");
            rndis_dashboard_release();
        }
        else if (rndis_dashboard_binding(session) != 0U)
        {
            now = sys_now();

            if ((s_rndis_dashboard.phase == RNDIS_DASHBOARD_BOUND) && (rndis_dashboard_bound_ingress_valid(now) == 0U))
            {
                rndis_dashboard_handoff_clear();
            }
            else
            {
                RndisDashboardHttpSetReady(s_rndis_dashboard.generation, s_rndis_dashboard.tcp_ready);
                rndis_dashboard_rx_drain(session);

                if (rndis_dashboard_binding(session) != 0U)
                {
                    rndis_dashboard_service(session);
                }
            }
        }
        else
        {
            rndis_dashboard_terminal("RNDIS_SESSION_NOT_RUNNING");
        }
    }

    s_rndis_dashboard.session = NULL;
    s_rndis_dashboard.iteration_generation = 0U;
    s_rndis_dashboard.tx_enabled = 0U;
}