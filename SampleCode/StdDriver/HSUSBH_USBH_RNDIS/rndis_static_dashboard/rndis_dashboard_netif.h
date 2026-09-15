/**************************************************************************//**
 * @file     rndis_dashboard_netif.h
 * @brief    RNDIS Dashboard DHCP and raw TCP netif lifecycle.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef RNDIS_DASHBOARD_NETIF_H
#define RNDIS_DASHBOARD_NETIF_H

#include "usbh_rndis.h"

#define RNDIS_DASHBOARD_DHCP_DEADLINE_MS 60000UL

#ifdef __cplusplus
extern "C"
{
#endif

void RndisDashboardNetifBeginIteration(RNDIS_SESSION_T *session);
void RndisDashboardNetifEndIteration(RNDIS_SESSION_T *session);

#ifdef __cplusplus
}
#endif

#endif /* RNDIS_DASHBOARD_NETIF_H */