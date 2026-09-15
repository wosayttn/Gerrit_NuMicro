/**************************************************************************//**
 * @file     rndis_dashboard_http.h
 * @brief    RNDIS Dashboard raw TCP lifecycle.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef RNDIS_DASHBOARD_HTTP_H
#define RNDIS_DASHBOARD_HTTP_H

#include <stdint.h>

#include "lwip/ip4_addr.h"

#ifdef __cplusplus
extern "C"
{
#endif

void RndisDashboardHttpOnBound(uint32_t generation);
void RndisDashboardHttpSetIpv4(ip4_addr_t const *address);
void RndisDashboardHttpSetReady(uint32_t generation, uint8_t ready);
void RndisDashboardHttpService(uint32_t generation, uint8_t ready);
void RndisDashboardHttpQuiesce(char const *reason, uint32_t generation);

#ifdef __cplusplus
}
#endif

#endif /* RNDIS_DASHBOARD_HTTP_H */