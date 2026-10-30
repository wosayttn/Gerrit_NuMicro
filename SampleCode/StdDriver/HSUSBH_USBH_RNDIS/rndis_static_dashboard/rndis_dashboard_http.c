/**************************************************************************//**
 * @file     rndis_dashboard_http.c
 * @brief    single-client static dashboard raw TCP service.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "rndis_dashboard_http.h"

#define RNDIS_DASHBOARD_HTTP_PORT 80U
#define RNDIS_DASHBOARD_HTTP_ACTIVE_TIMEOUT_MS 2000UL
#define RNDIS_DASHBOARD_HTTP_HEADER_MAX 1024U
#define RNDIS_DASHBOARD_BODY_MAX 160U
#define RNDIS_DASHBOARD_RESPONSE_MAX 256U

typedef enum
{
    RNDIS_DASHBOARD_HTTP_RESPONSE_DASHBOARD = 0,
    RNDIS_DASHBOARD_HTTP_RESPONSE_NOT_FOUND
} RNDIS_DASHBOARD_HTTP_RESPONSE_E;

typedef enum
{
    RNDIS_DASHBOARD_HTTP_COLLECTING = 0,
    RNDIS_DASHBOARD_HTTP_WAIT_ACK
} RNDIS_DASHBOARD_HTTP_STATE_E;

/**
 * @brief Runtime state for the single-client raw TCP dashboard service.
 * @details Contains the listener/active PCB ownership, callback epoch, bounded
 *          request buffer, response state, and diagnostic counters.
 */
typedef struct
{
    struct tcp_pcb *listener;                   /**< Listening TCP PCB owned by the dashboard. */
    struct tcp_pcb *active;                     /**< Single active client PCB, or NULL when idle. */
    uint32_t generation;                        /**< RNDIS session generation owning the service. */
    uint32_t epoch;                             /**< Callback validity epoch; zero means quiesced. */
    uint32_t accepts;                           /**< Number of accepted client connections. */
    uint32_t rejects;                           /**< Number of rejected client or protocol operations. */
    uint32_t closes;                            /**< Number of active/listener close or abort operations. */
    uint32_t tcp_errors;                        /**< Number of current-epoch TCP error callbacks. */
    uint32_t stale_callbacks;                   /**< Number of callbacks rejected by epoch/PCB validation. */
    uint32_t requests;                          /**< Number of syntactically accepted HTTP requests. */
    uint32_t queued_responses;                  /**< Number of responses passed to tcp_write(). */
    uint32_t acknowledged_responses;            /**< Number of responses fully acknowledged by lwIP. */
    uint32_t acknowledged_response_aborts;      /**< Number of PCBs aborted after full response acknowledgement. */
    uint32_t active_deadline_ms;                /**< Active-client request deadline in milliseconds. */
    uint16_t header_length;                     /**< Number of request-header bytes currently buffered. */
    uint16_t acknowledged_bytes;                /**< Number of response bytes acknowledged for the active PCB. */
    uint16_t response_length;                   /**< Selected response length in bytes. */
    uint8_t ready;                              /**< Non-zero when the current generation may accept clients. */
    RNDIS_DASHBOARD_HTTP_STATE_E state;         /**< Active request/response state. */
    char const *response;                       /**< Pointer to the selected response buffer. */
    uint8_t header[RNDIS_DASHBOARD_HTTP_HEADER_MAX]; /**< Fixed-size HTTP request header buffer. */
} RNDIS_DASHBOARD_HTTP_CONTEXT_T;

static RNDIS_DASHBOARD_HTTP_CONTEXT_T s_rndis_dashboard_http;

static char s_rndis_dashboard_response[RNDIS_DASHBOARD_RESPONSE_MAX];
static char s_rndis_dashboard_body[RNDIS_DASHBOARD_BODY_MAX];
static uint16_t s_rndis_dashboard_response_length;

static char const s_rndis_dashboard_not_found_response[] =
    "HTTP/1.1 404 Not Found\r\n"
    "Content-Type: text/plain; charset=us-ascii\r\n"
    "Content-Length: 10\r\n"
    "Connection: close\r\n"
    "\r\n"
    "Not Found\n";

_Static_assert(sizeof(s_rndis_dashboard_response) <= TCP_MSS,
               "RNDIS Dashboard response must fit the configured TCP send buffer");
_Static_assert((sizeof(s_rndis_dashboard_not_found_response) - 1U) <= TCP_MSS,
               "RNDIS Dashboard not-found response must fit the configured TCP send buffer");

/**
 * @brief Decode a HTTP callback epoch stored in a lwIP callback argument.
 * @param[in] arg Callback argument previously assigned with tcp_arg().
 * @return Decoded non-zero epoch value, or zero when the argument is NULL.
 */
static uint32_t rndis_dashboard_http_epoch_from_arg(void const *arg)
{
    return (uint32_t)(uintptr_t)arg;
}

/**
 * @brief Test whether a wrap-safe millisecond deadline has been reached.
 * @param[in] now Current millisecond tick.
 * @param[in] deadline Active-connection deadline tick.
 * @return 1 when the deadline is reached or passed; otherwise 0.
 */
static uint8_t rndis_dashboard_http_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

/**
 * @brief Print the accumulated HTTP service counters.
 * @note Diagnostic-only helper; it does not modify connection state.
 */
static void rndis_dashboard_http_log_counters(void)
{
    printf("RNDIS_DASHBOARD_HTTP_COUNTERS gen=%lu epoch=%lu accept=%lu request=%lu queue=%lu ack=%lu ack_abort=%lu reject=%lu close=%lu active=%u stale_cb=%lu tcp_err=%lu\n",
           (unsigned long)s_rndis_dashboard_http.generation, (unsigned long)s_rndis_dashboard_http.epoch,
           (unsigned long)s_rndis_dashboard_http.accepts, (unsigned long)s_rndis_dashboard_http.requests,
           (unsigned long)s_rndis_dashboard_http.queued_responses,
           (unsigned long)s_rndis_dashboard_http.acknowledged_responses,
           (unsigned long)s_rndis_dashboard_http.acknowledged_response_aborts,
           (unsigned long)s_rndis_dashboard_http.rejects, (unsigned long)s_rndis_dashboard_http.closes,
           (unsigned int)((s_rndis_dashboard_http.active != NULL) ? 1U : 0U),
           (unsigned long)s_rndis_dashboard_http.stale_callbacks,
           (unsigned long)s_rndis_dashboard_http.tcp_errors);
}

/**
 * @brief Clear the active HTTP connection metadata.
 * @note This helper does not close or abort a TCP PCB. The caller must perform
 *       the required lwIP PCB cleanup separately when needed.
 */
static void rndis_dashboard_http_clear_active(void)
{
    s_rndis_dashboard_http.active = NULL;
    s_rndis_dashboard_http.active_deadline_ms = 0U;
    s_rndis_dashboard_http.header_length = 0U;
    s_rndis_dashboard_http.acknowledged_bytes = 0U;
    s_rndis_dashboard_http.response_length = 0U;
    s_rndis_dashboard_http.response = NULL;
    s_rndis_dashboard_http.state = RNDIS_DASHBOARD_HTTP_COLLECTING;
}

/**
 * @brief Abort a TCP PCB that is not the current active dashboard connection.
 * @param[in] pcb PCB to detach and abort, or NULL.
 * @note Used for stale callback protection and rejected connections.
 */
static void rndis_dashboard_http_abort_foreign(struct tcp_pcb *pcb)
{
    if (pcb != NULL)
    {
        tcp_recv(pcb, NULL);
        tcp_sent(pcb, NULL);
        tcp_err(pcb, NULL);
        tcp_arg(pcb, NULL);
        tcp_abort(pcb);
    }
}

/**
 * @brief Abort and clear the current active dashboard connection.
 * @param[in] reason Constant diagnostic reason printed for the close.
 * @note Callback registrations are removed before the PCB is aborted.
 */
static void rndis_dashboard_http_abort_active(char const *reason)
{
    struct tcp_pcb *active = s_rndis_dashboard_http.active;

    if (active == NULL)
    {
        return;
    }

    rndis_dashboard_http_clear_active();
    tcp_recv(active, NULL);
    tcp_sent(active, NULL);
    tcp_err(active, NULL);
    tcp_arg(active, NULL);
    tcp_abort(active);
    s_rndis_dashboard_http.closes++;
    printf("RNDIS_DASHBOARD_HTTP_CLOSE gen=%lu epoch=%lu reason=%s\n",
           (unsigned long)s_rndis_dashboard_http.generation,
           (unsigned long)s_rndis_dashboard_http.epoch, reason);
    rndis_dashboard_http_log_counters();
}

/**
 * @brief Reject the current HTTP operation and abort the active connection.
 * @param[in] reason Constant diagnostic reason for the rejection.
 * @return ERR_ABRT for direct return from a lwIP callback.
 */
static err_t rndis_dashboard_http_reject(char const *reason)
{
    s_rndis_dashboard_http.rejects++;
    printf("RNDIS_DASHBOARD_HTTP_REJECT gen=%lu epoch=%lu reason=%s\n",
           (unsigned long)s_rndis_dashboard_http.generation,
           (unsigned long)s_rndis_dashboard_http.epoch, reason);
    rndis_dashboard_http_abort_active(reason);
    return ERR_ABRT;
}

/**
 * @brief Compare an input token with an ASCII token case-insensitively.
 * @param[in] text Input token bytes.
 * @param[in] text_length Number of bytes in @p text.
 * @param[in] token NUL-terminated expected token.
 * @return 1 when the complete token matches; otherwise 0.
 */
static uint8_t rndis_dashboard_http_token_equal(uint8_t const *text, uint16_t text_length,
                                                char const *token)
{
    uint16_t index;

    for (index = 0U; index < text_length; index++)
    {
        char expected = token[index];
        uint8_t actual = text[index];

        if (expected == '\0')
        {
            return 0U;
        }

        if ((actual >= (uint8_t)'A') && (actual <= (uint8_t)'Z'))
        {
            actual = (uint8_t)(actual + ((uint8_t)'a' - (uint8_t)'A'));
        }

        if ((uint8_t)expected != actual)
        {
            return 0U;
        }
    }

    return (token[text_length] == '\0') ? 1U : 0U;
}

/**
 * @brief Validate one HTTP header field line.
 * @param[in] line Header field bytes excluding CRLF.
 * @param[in] length Number of bytes in @p line.
 * @return 1 when the field syntax and supported-header restrictions are valid;
 *         otherwise 0.
 * @note Content-Length, transfer framing, upgrade, expectation, and trailer
 *       fields are rejected because the dashboard accepts only a bounded GET.
 */
static uint8_t rndis_dashboard_http_field_valid(uint8_t const *line, uint16_t length)
{
    uint16_t index;
    uint16_t colon = length;

    for (index = 0U; index < length; index++)
    {
        if (line[index] == (uint8_t)':')
        {
            colon = index;
            break;
        }
    }

    if ((colon == 0U) || (colon == length))
    {
        return 0U;
    }

    for (index = 0U; index < colon; index++)
    {
        uint8_t character = line[index];

        if (!(((character >= (uint8_t)'0') && (character <= (uint8_t)'9')) ||
                ((character >= (uint8_t)'A') && (character <= (uint8_t)'Z')) ||
                ((character >= (uint8_t)'a') && (character <= (uint8_t)'z')) ||
                (character == (uint8_t)'!') || (character == (uint8_t)'#') ||
                (character == (uint8_t)'$') || (character == (uint8_t)'%') ||
                (character == (uint8_t)'&') || (character == (uint8_t)'\'') ||
                (character == (uint8_t)'*') || (character == (uint8_t)'+') ||
                (character == (uint8_t)'-') || (character == (uint8_t)'.') ||
                (character == (uint8_t)'^') || (character == (uint8_t)'_') ||
                (character == (uint8_t)'`') || (character == (uint8_t)'|') ||
                (character == (uint8_t)'~')))
        {
            return 0U;
        }
    }

    if ((rndis_dashboard_http_token_equal(line, colon, "content-length") != 0U) ||
            (rndis_dashboard_http_token_equal(line, colon, "transfer-encoding") != 0U) ||
            (rndis_dashboard_http_token_equal(line, colon, "upgrade") != 0U) ||
            (rndis_dashboard_http_token_equal(line, colon, "expect") != 0U) ||
            (rndis_dashboard_http_token_equal(line, colon, "trailer") != 0U))
    {
        return 0U;
    }

    for (index = (uint16_t)(colon + 1U); index < length; index++)
    {
        uint8_t character = line[index];

        if ((character != (uint8_t)'\t') &&
                ((character < 0x20U) || (character > 0x7EU)))
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief Validate the HTTP request line and select the response type.
 * @param[in] line_length Length of the request line excluding CRLF.
 * @param[out] response Receives dashboard or not-found response selection.
 * @return 1 when the request line is a supported GET; otherwise 0.
 * @note The request target accepts `/` and `/index.html`; other targets map to
 *       the bounded 404 response.
 */
static uint8_t rndis_dashboard_http_request_line_valid(uint16_t line_length,
                                                       RNDIS_DASHBOARD_HTTP_RESPONSE_E *response)
{
    uint16_t first_space = line_length;
    uint16_t second_space = line_length;
    uint16_t index;

    for (index = 0U; index < line_length; index++)
    {
        if (s_rndis_dashboard_http.header[index] == (uint8_t)' ')
        {
            if (first_space == line_length)
            {
                first_space = index;
            }
            else if (second_space == line_length)
            {
                second_space = index;
            }
            else
            {
                return 0U;
            }
        }
    }

    if ((first_space != 3U) || (second_space <= (uint16_t)(first_space + 1U)) ||
            ((uint16_t)(second_space + 9U) != line_length) ||
            (memcmp(s_rndis_dashboard_http.header, "GET", 3U) != 0) ||
            ((memcmp(&s_rndis_dashboard_http.header[second_space + 1U], "HTTP/1.0", 8U) != 0) &&
             (memcmp(&s_rndis_dashboard_http.header[second_space + 1U], "HTTP/1.1", 8U) != 0)))
    {
        return 0U;
    }

    for (index = (uint16_t)(first_space + 1U); index < second_space; index++)
    {
        if ((s_rndis_dashboard_http.header[index] < 0x21U) || (s_rndis_dashboard_http.header[index] > 0x7EU))
        {
            return 0U;
        }
    }

    if ((s_rndis_dashboard_http.header[first_space + 1U] != (uint8_t)'/'))
    {
        return 0U;
    }

    *response = (((uint16_t)(second_space - first_space) == 2U) ||
                 (((uint16_t)(second_space - first_space) == 12U) &&
                  (memcmp(&s_rndis_dashboard_http.header[first_space + 1U], "/index.html", 11U) == 0))) ?
                RNDIS_DASHBOARD_HTTP_RESPONSE_DASHBOARD : RNDIS_DASHBOARD_HTTP_RESPONSE_NOT_FOUND;
    return 1U;
}

/**
 * @brief Validate the complete bounded HTTP request header.
 * @param[out] response Receives the response selected by the request target.
 * @return 1 when the header contains a complete valid request; otherwise 0.
 * @note The header is read from the session-owned fixed-size buffer.
 */
static uint8_t rndis_dashboard_http_header_valid(RNDIS_DASHBOARD_HTTP_RESPONSE_E *response)
{
    uint16_t line_start;
    uint16_t line_end;

    if ((response == NULL) || (s_rndis_dashboard_http.header_length < 18U))
    {
        return 0U;
    }

    line_end = 0U;

    while ((line_end < (uint16_t)(s_rndis_dashboard_http.header_length - 1U)) &&
            !((s_rndis_dashboard_http.header[line_end] == (uint8_t)'\r') &&
              (s_rndis_dashboard_http.header[line_end + 1U] == (uint8_t)'\n')))
    {
        line_end++;
    }

    if ((line_end == 0U) || (line_end >= (uint16_t)(s_rndis_dashboard_http.header_length - 1U)) ||
            (rndis_dashboard_http_request_line_valid(line_end, response) == 0U))
    {
        return 0U;
    }

    line_start = (uint16_t)(line_end + 2U);

    while (line_start < s_rndis_dashboard_http.header_length)
    {
        if ((line_start <= (uint16_t)(s_rndis_dashboard_http.header_length - 2U)) &&
                (s_rndis_dashboard_http.header[line_start] == (uint8_t)'\r') &&
                (s_rndis_dashboard_http.header[line_start + 1U] == (uint8_t)'\n'))
        {
            return ((uint16_t)(line_start + 2U) == s_rndis_dashboard_http.header_length) ? 1U : 0U;
        }

        line_end = line_start;

        while ((line_end < (uint16_t)(s_rndis_dashboard_http.header_length - 1U)) &&
                !((s_rndis_dashboard_http.header[line_end] == (uint8_t)'\r') &&
                  (s_rndis_dashboard_http.header[line_end + 1U] == (uint8_t)'\n')))
        {
            line_end++;
        }

        if ((line_end >= (uint16_t)(s_rndis_dashboard_http.header_length - 1U)) ||
                (rndis_dashboard_http_field_valid(&s_rndis_dashboard_http.header[line_start],
                                                  (uint16_t)(line_end - line_start)) == 0U))
        {
            return 0U;
        }

        line_start = (uint16_t)(line_end + 2U);
    }

    return 0U;
}

/**
 * @brief Build the dashboard response with the current M55M1 IPv4 address.
 * @param[in] address IPv4 address assigned to the dashboard netif.
 * @note Uses fixed buffers only. The response length is updated only when both
 *       the body and HTTP response fit their configured bounds.
 */
void RndisDashboardHttpSetIpv4(ip4_addr_t const *address)
{
    char address_text[IP4ADDR_STRLEN_MAX];
    char const *address_string;
    int body_length;
    int response_length;

    if (address == NULL)
    {
        address_string = "0.0.0.0";
    }
    else
    {
        address_string = ip4addr_ntoa_r(address, address_text, sizeof(address_text));

        if (address_string == NULL)
        {
            address_string = "0.0.0.0";
        }
    }

    body_length = snprintf(s_rndis_dashboard_body, sizeof(s_rndis_dashboard_body),
                           "<!doctype html><html><body>I'm M55xx NuMark Board<br>IPv4: %s</body></html>\n",
                           address_string);

    if ((body_length < 0) || ((size_t)body_length >= sizeof(s_rndis_dashboard_body)))
    {
        s_rndis_dashboard_response_length = 0U;
        return;
    }

    response_length = snprintf(s_rndis_dashboard_response, sizeof(s_rndis_dashboard_response),
                               "HTTP/1.1 200 OK\r\n"
                               "Content-Type: text/html; charset=us-ascii\r\n"
                               "Content-Length: %d\r\n"
                               "Connection: close\r\n"
                               "\r\n"
                               "%s",
                               body_length, s_rndis_dashboard_body);

    if ((response_length < 0) || ((size_t)response_length >= sizeof(s_rndis_dashboard_response)))
    {
        s_rndis_dashboard_response_length = 0U;
        return;
    }

    s_rndis_dashboard_response_length = (uint16_t)response_length;
}

/**
 * @brief Queue the selected HTTP response for TCP transmission.
 * @param[in] pcb Active TCP PCB that must match the session PCB.
 * @return ERR_OK when queued and flushed; otherwise ERR_ABRT after rejection.
 * @note Uses TCP_WRITE_FLAG_COPY so the response buffer remains independent of
 *       lwIP send-buffer ownership.
 */
static err_t rndis_dashboard_http_queue_response(struct tcp_pcb *pcb)
{
    err_t status;

    if ((pcb != s_rndis_dashboard_http.active) ||
            (s_rndis_dashboard_http.state != RNDIS_DASHBOARD_HTTP_COLLECTING))
    {
        return rndis_dashboard_http_reject("QUEUE_STATE");
    }

    if ((s_rndis_dashboard_http.response == NULL) || (s_rndis_dashboard_http.response_length == 0U))
    {
        return rndis_dashboard_http_reject("RESPONSE_STATE");
    }

    status = tcp_write(pcb, s_rndis_dashboard_http.response, s_rndis_dashboard_http.response_length,
                       TCP_WRITE_FLAG_COPY);

    if (status != ERR_OK)
    {
        return rndis_dashboard_http_reject("QUEUE_FAILED");
    }

    s_rndis_dashboard_http.state = RNDIS_DASHBOARD_HTTP_WAIT_ACK;
    s_rndis_dashboard_http.queued_responses++;
    printf("RNDIS_DASHBOARD_HTTP_QUEUE bytes=%u\n", (unsigned int)s_rndis_dashboard_http.response_length);
    status = tcp_output(pcb);

    if (status != ERR_OK)
    {
        return rndis_dashboard_http_reject("OUTPUT_FAILED");
    }

    return ERR_OK;
}

/**
 * @brief Receive and parse one or more chunks of an HTTP request.
 * @param[in] arg Callback epoch encoded by tcp_arg().
 * @param[in] pcb TCP PCB delivering the data.
 * @param[in] p Received pbuf chain, or NULL for peer FIN.
 * @param[in] err lwIP receive status.
 * @return lwIP callback status; ERR_ABRT for stale, invalid, or closed
 *         connections, otherwise ERR_OK while collecting data.
 * @note Callback context. The pbuf is consumed and freed before return.
 */
static err_t rndis_dashboard_http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    uint32_t epoch = rndis_dashboard_http_epoch_from_arg(arg);
    uint16_t length;
    uint16_t copied;
    uint16_t index;
    uint8_t header_complete = 0U;
    RNDIS_DASHBOARD_HTTP_RESPONSE_E response;

    if ((epoch == 0U) || (epoch != s_rndis_dashboard_http.epoch) || (pcb != s_rndis_dashboard_http.active))
    {
        s_rndis_dashboard_http.stale_callbacks++;

        if (p != NULL)
        {
            (void)pbuf_free(p);
        }

        rndis_dashboard_http_abort_foreign(pcb);
        return ERR_ABRT;
    }

    if (p == NULL)
    {
        rndis_dashboard_http_abort_active("PEER_FIN");
        return ERR_ABRT;
    }

    if ((err != ERR_OK) || (rndis_dashboard_http_reached(sys_now(), s_rndis_dashboard_http.active_deadline_ms) != 0U))
    {
        (void)pbuf_free(p);
        rndis_dashboard_http_abort_active((err == ERR_OK) ? "ACTIVE_TIMEOUT" : "RX_ERR");
        return ERR_ABRT;
    }

    if (s_rndis_dashboard_http.state == RNDIS_DASHBOARD_HTTP_WAIT_ACK)
    {
        (void)pbuf_free(p);
        return rndis_dashboard_http_reject("POST_QUEUE_DATA");
    }

    length = p->tot_len;

    if ((length == 0U) || (length > (uint16_t)(RNDIS_DASHBOARD_HTTP_HEADER_MAX - s_rndis_dashboard_http.header_length)))
    {
        (void)pbuf_free(p);
        return rndis_dashboard_http_reject("HEADER_OVERFLOW");
    }

    copied = pbuf_copy_partial(p, &s_rndis_dashboard_http.header[s_rndis_dashboard_http.header_length], length, 0U);

    if (copied != length)
    {
        (void)pbuf_free(p);
        return rndis_dashboard_http_reject("HEADER_COPY");
    }

    tcp_recved(pcb, length);
    (void)pbuf_free(p);
    s_rndis_dashboard_http.header_length = (uint16_t)(s_rndis_dashboard_http.header_length + length);

    if (s_rndis_dashboard_http.header_length >= 4U)
    {
        for (index = 0U; index <= (uint16_t)(s_rndis_dashboard_http.header_length - 4U); index++)
        {
            if ((s_rndis_dashboard_http.header[index] == (uint8_t)'\r') &&
                    (s_rndis_dashboard_http.header[index + 1U] == (uint8_t)'\n') &&
                    (s_rndis_dashboard_http.header[index + 2U] == (uint8_t)'\r') &&
                    (s_rndis_dashboard_http.header[index + 3U] == (uint8_t)'\n'))
            {
                header_complete = 1U;
                break;
            }
        }
    }

    if (header_complete == 0U)
    {
        return (s_rndis_dashboard_http.header_length == RNDIS_DASHBOARD_HTTP_HEADER_MAX) ?
               rndis_dashboard_http_reject("HEADER_OVERFLOW") : ERR_OK;
    }

    if (rndis_dashboard_http_header_valid(&response) == 0U)
    {
        return rndis_dashboard_http_reject("REQUEST_FORMAT");
    }

    if (response == RNDIS_DASHBOARD_HTTP_RESPONSE_DASHBOARD)
    {
        s_rndis_dashboard_http.response = s_rndis_dashboard_response;
        s_rndis_dashboard_http.response_length = s_rndis_dashboard_response_length;
    }
    else
    {
        s_rndis_dashboard_http.response = s_rndis_dashboard_not_found_response;
        s_rndis_dashboard_http.response_length = (uint16_t)(sizeof(s_rndis_dashboard_not_found_response) - 1U);
    }

    s_rndis_dashboard_http.requests++;
    printf("RNDIS_DASHBOARD_HTTP_REQUEST bytes=%u\n", (unsigned int)s_rndis_dashboard_http.header_length);
    return rndis_dashboard_http_queue_response(pcb);
}

/**
 * @brief Handle an asynchronous lwIP TCP error callback.
 * @param[in] arg Callback epoch encoded by tcp_arg().
 * @param[in] err lwIP TCP error code.
 * @note The PCB has already been invalidated by lwIP when this callback runs;
 *       only dashboard metadata is cleared here.
 */
static void rndis_dashboard_http_err(void *arg, err_t err)
{
    if ((rndis_dashboard_http_epoch_from_arg(arg) == 0U) ||
            (rndis_dashboard_http_epoch_from_arg(arg) != s_rndis_dashboard_http.epoch))
    {
        s_rndis_dashboard_http.stale_callbacks++;
        return;
    }

    s_rndis_dashboard_http.tcp_errors++;
    rndis_dashboard_http_clear_active();
    s_rndis_dashboard_http.closes++;
    printf("RNDIS_DASHBOARD_HTTP_CLOSE gen=%lu epoch=%lu reason=TCP_ERR err=%d\n",
           (unsigned long)s_rndis_dashboard_http.generation,
           (unsigned long)s_rndis_dashboard_http.epoch, (int)err);
    rndis_dashboard_http_log_counters();
}

/**
 * @brief Consume an acknowledged portion of the queued HTTP response.
 * @param[in] arg Callback epoch encoded by tcp_arg().
 * @param[in] pcb Active TCP PCB acknowledging data.
 * @param[in] length Number of bytes acknowledged by lwIP.
 * @return ERR_OK while waiting for more acknowledgements; ERR_ABRT after the
 *         complete response or an invalid/stale acknowledgement.
 * @note Callback context. The connection is intentionally aborted after the
 *       complete response is acknowledged because the service is single-client
 *       and uses `Connection: close`.
 */
static err_t rndis_dashboard_http_sent(void *arg, struct tcp_pcb *pcb, u16_t length)
{
    uint32_t epoch = rndis_dashboard_http_epoch_from_arg(arg);

    if ((epoch == 0U) || (epoch != s_rndis_dashboard_http.epoch) ||
            (pcb != s_rndis_dashboard_http.active) || (s_rndis_dashboard_http.state != RNDIS_DASHBOARD_HTTP_WAIT_ACK) ||
            (length == 0U) || (s_rndis_dashboard_http.response_length == 0U) ||
            ((uint32_t)s_rndis_dashboard_http.acknowledged_bytes + length >
             s_rndis_dashboard_http.response_length))
    {
        if ((epoch != s_rndis_dashboard_http.epoch) || (pcb != s_rndis_dashboard_http.active))
        {
            s_rndis_dashboard_http.stale_callbacks++;
            rndis_dashboard_http_abort_foreign(pcb);
        }
        else
        {
            (void)rndis_dashboard_http_reject("ACK_INVALID");
        }

        return ERR_ABRT;
    }

    s_rndis_dashboard_http.acknowledged_bytes = (uint16_t)(s_rndis_dashboard_http.acknowledged_bytes + length);

    if (s_rndis_dashboard_http.acknowledged_bytes != s_rndis_dashboard_http.response_length)
    {
        return ERR_OK;
    }

    s_rndis_dashboard_http.acknowledged_responses++;
    printf("RNDIS_DASHBOARD_HTTP_ACK bytes=%u\n", (unsigned int)s_rndis_dashboard_http.response_length);
    rndis_dashboard_http_clear_active();
    tcp_recv(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_arg(pcb, NULL);
    s_rndis_dashboard_http.closes++;
    s_rndis_dashboard_http.acknowledged_response_aborts++;
    tcp_abort(pcb);
    printf("RNDIS_DASHBOARD_HTTP_CLOSE gen=%lu epoch=%lu reason=ACK_RESPONSE_ABORT\n",
           (unsigned long)s_rndis_dashboard_http.generation,
           (unsigned long)s_rndis_dashboard_http.epoch);
    rndis_dashboard_http_log_counters();
    return ERR_ABRT;
}

/**
 * @brief Accept one incoming TCP client when the HTTP service is ready.
 * @param[in] arg Listener callback epoch encoded by tcp_arg().
 * @param[in] new_pcb Newly accepted client PCB, or NULL on failure.
 * @param[in] err lwIP accept status.
 * @return ERR_OK when the client is registered; ERR_ABRT when rejected with a
 *         valid PCB, otherwise ERR_OK for a failed accept without a PCB.
 * @note The service allows only one active client at a time.
 */
static err_t rndis_dashboard_http_accept(void *arg, struct tcp_pcb *new_pcb, err_t err)
{
    uint32_t epoch = rndis_dashboard_http_epoch_from_arg(arg);

    if ((epoch == 0U) || (epoch != s_rndis_dashboard_http.epoch) || (err != ERR_OK) ||
            (new_pcb == NULL) || (s_rndis_dashboard_http.listener == NULL) ||
            (s_rndis_dashboard_http.ready == 0U) || (s_rndis_dashboard_http.active != NULL))
    {
        s_rndis_dashboard_http.rejects++;
        printf("RNDIS_DASHBOARD_HTTP_REJECT gen=%lu epoch=%lu reason=ACCEPT\n",
               (unsigned long)s_rndis_dashboard_http.generation, (unsigned long)epoch);

        if (new_pcb != NULL)
        {
            rndis_dashboard_http_abort_foreign(new_pcb);
        }

        rndis_dashboard_http_log_counters();
        return (new_pcb != NULL) ? ERR_ABRT : ERR_OK;
    }

    s_rndis_dashboard_http.active = new_pcb;
    s_rndis_dashboard_http.accepts++;
    s_rndis_dashboard_http.header_length = 0U;
    s_rndis_dashboard_http.acknowledged_bytes = 0U;
    s_rndis_dashboard_http.state = RNDIS_DASHBOARD_HTTP_COLLECTING;
    s_rndis_dashboard_http.active_deadline_ms = sys_now() + RNDIS_DASHBOARD_HTTP_ACTIVE_TIMEOUT_MS;
    tcp_arg(new_pcb, (void *)(uintptr_t)epoch);
    tcp_recv(new_pcb, rndis_dashboard_http_recv);
    tcp_sent(new_pcb, rndis_dashboard_http_sent);
    tcp_err(new_pcb, rndis_dashboard_http_err);
    printf("RNDIS_DASHBOARD_HTTP_ACCEPT gen=%lu epoch=%lu\n",
           (unsigned long)s_rndis_dashboard_http.generation, (unsigned long)epoch);
    rndis_dashboard_http_log_counters();
    return ERR_OK;
}

/**
 * @brief Close or abort the listening TCP PCB.
 * @note Listener callbacks and arguments are cleared before attempting a
 *       graceful close; abort is used when graceful close fails.
 */
static void rndis_dashboard_http_close_listener(void)
{
    struct tcp_pcb *listener = s_rndis_dashboard_http.listener;

    s_rndis_dashboard_http.listener = NULL;

    if (listener != NULL)
    {
        tcp_accept(listener, NULL);
        tcp_arg(listener, NULL);

        if (tcp_close(listener) != ERR_OK)
        {
            tcp_abort(listener);
        }
    }
}

/**
 * @brief Create and start the dashboard HTTP listener for a session generation.
 * @param[in] generation Non-zero RNDIS session generation owning the listener.
 * @note Call after DHCP reaches BOUND. The operation is ignored when an HTTP
 *       epoch or listener/active connection already exists.
 */
void RndisDashboardHttpOnBound(uint32_t generation)
{
    struct tcp_pcb *pcb;
    struct tcp_pcb *listener;

    if ((generation == 0U) || (s_rndis_dashboard_http.epoch != 0U) ||
            (s_rndis_dashboard_http.listener != NULL) || (s_rndis_dashboard_http.active != NULL))
    {
        return;
    }

    (void)memset(&s_rndis_dashboard_http, 0, sizeof(s_rndis_dashboard_http));
    s_rndis_dashboard_http.generation = generation;
    s_rndis_dashboard_http.epoch = generation;
    pcb = tcp_new();

    if (pcb == NULL)
    {
        printf("RNDIS_DASHBOARD_HTTP_LISTEN gen=%lu status=NO_PCB\n", (unsigned long)generation);
        return;
    }

    if (tcp_bind(pcb, IP_ADDR_ANY, RNDIS_DASHBOARD_HTTP_PORT) != ERR_OK)
    {
        tcp_abort(pcb);
        s_rndis_dashboard_http.epoch = 0U;
        s_rndis_dashboard_http.generation = 0U;
        printf("RNDIS_DASHBOARD_HTTP_LISTEN gen=%lu status=BIND_FAILED\n", (unsigned long)generation);
        return;
    }

    listener = tcp_listen(pcb);

    if (listener == NULL)
    {
        tcp_abort(pcb);
        s_rndis_dashboard_http.epoch = 0U;
        s_rndis_dashboard_http.generation = 0U;
        printf("RNDIS_DASHBOARD_HTTP_LISTEN gen=%lu status=LISTEN_FAILED\n", (unsigned long)generation);
        return;
    }

    s_rndis_dashboard_http.listener = listener;
    tcp_arg(listener, (void *)(uintptr_t)generation);
    tcp_accept(listener, rndis_dashboard_http_accept);
    printf("RNDIS_DASHBOARD_HTTP_LISTEN gen=%lu epoch=%lu port=80\n",
           (unsigned long)generation, (unsigned long)s_rndis_dashboard_http.epoch);
    rndis_dashboard_http_log_counters();
}

/**
 * @brief Enable or disable acceptance of new dashboard HTTP clients.
 * @param[in] generation RNDIS session generation expected by the caller.
 * @param[in] ready Non-zero to enable service, zero to disable it.
 * @note Readiness is accepted only when @p generation matches the current HTTP
 *       epoch; stale generations cannot re-enable the service.
 */
void RndisDashboardHttpSetReady(uint32_t generation, uint8_t ready)
{
    s_rndis_dashboard_http.ready = ((ready != 0U) && (generation != 0U) &&
                                    (generation == s_rndis_dashboard_http.epoch)) ? 1U : 0U;
}

/**
 * @brief Service HTTP connection timeout and listener readiness.
 * @param[in] generation RNDIS session generation expected by the caller.
 * @param[in] ready Non-zero when the owning netif/DHCP path is ready.
 * @note Worker-context, non-blocking API. Invalid or stale readiness quiesces
 *       the listener and active connection.
 */
void RndisDashboardHttpService(uint32_t generation, uint8_t ready)
{
    if ((ready == 0U) || (s_rndis_dashboard_http.ready == 0U) || (generation == 0U) ||
            (generation != s_rndis_dashboard_http.epoch))
    {
        RndisDashboardHttpQuiesce("SERVICE_NOT_READY", generation);
        return;
    }

    if (s_rndis_dashboard_http.listener == NULL)
    {
        RndisDashboardHttpQuiesce("LISTENER_LOST", generation);
        return;
    }

    if ((s_rndis_dashboard_http.active != NULL) &&
            (rndis_dashboard_http_reached(sys_now(), s_rndis_dashboard_http.active_deadline_ms) != 0U))
    {
        rndis_dashboard_http_abort_active("ACTIVE_TIMEOUT");
    }
}

/**
 * @brief Quiesce the HTTP listener and invalidate its callback epoch.
 * @param[in] reason Constant diagnostic reason for the quiesce operation.
 * @param[in] generation Caller generation associated with the quiesce request.
 * @note Safe to call repeatedly. Invalidating the epoch makes pending lwIP
 *       callbacks stale before listener and active-PCB cleanup completes.
 */
void RndisDashboardHttpQuiesce(char const *reason, uint32_t generation)
{
    uint32_t previous_epoch = s_rndis_dashboard_http.epoch;

    if ((previous_epoch == 0U) && (s_rndis_dashboard_http.listener == NULL) &&
            (s_rndis_dashboard_http.active == NULL))
    {
        return;
    }

    printf("RNDIS_DASHBOARD_HTTP_QUIESCE reason=%s gen=%lu epoch=%lu\n",
           reason, (unsigned long)generation, (unsigned long)previous_epoch);
    s_rndis_dashboard_http.ready = 0U;
    rndis_dashboard_http_abort_active("QUIESCE");
    rndis_dashboard_http_close_listener();
    s_rndis_dashboard_http.epoch = 0U;
    s_rndis_dashboard_http.generation = 0U;
    printf("RNDIS_DASHBOARD_HTTP_EPOCH_INVALIDATE old=%lu new=0\n", (unsigned long)previous_epoch);
    rndis_dashboard_http_log_counters();
}