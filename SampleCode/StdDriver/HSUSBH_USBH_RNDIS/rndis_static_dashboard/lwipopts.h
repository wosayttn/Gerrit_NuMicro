/**************************************************************************//**
 * @file     lwipopts.h
 * @brief    RNDIS Dashboard NO_SYS DHCP/raw TCP configuration.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef RNDIS_DASHBOARD_LWIPOPTS_H
#define RNDIS_DASHBOARD_LWIPOPTS_H

#define NO_SYS                          1
#define NO_SYS_NO_TIMERS                0
#define LWIP_TIMERS                     1
#define LWIP_TIMERS_CUSTOM              0
#define SYS_LIGHTWEIGHT_PROT            0
#define LWIP_CALLBACK_API               1

#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0
#define LWIP_NETIF_API                  0
#define LWIP_RAW                        0
#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define LWIP_DNS                        0
#define LWIP_DHCP                       1
#define LWIP_AUTOIP                     0
#define DHCP_DOES_ARP_CHECK             1
#define LWIP_DHCP_AUTOIP_COOP           0
#define LWIP_IPV6                       0
#define PPP_SUPPORT                     0

#define LWIP_IPV4                       1
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       0
#define LWIP_IGMP                       0
#define IP_FORWARD                      0
#define IP_REASSEMBLY                   0
#define IP_FRAG                         0
#define LWIP_ALTCP                      0

#define LWIP_SINGLE_NETIF               1
#define LWIP_NETIF_STATUS_CALLBACK      0
#define LWIP_NETIF_LINK_CALLBACK        0
#define LWIP_NETIF_HOSTNAME             0
#define LWIP_NETIF_TX_SINGLE_PBUF       0
#define LWIP_STATS                      1
#define LWIP_STATS_DISPLAY              0
#define MEM_STATS                       1
#define MEMP_STATS                      1
#define MIB2_STATS                      0
#define LWIP_DEBUG                      0

#define MEM_ALIGNMENT                   4
#define MEM_LIBC_MALLOC                 0
#define MEM_USE_POOLS                   1
#define MEMP_USE_CUSTOM_POOLS           1
#define MEMP_MEM_MALLOC                 0
#define MEMP_NUM_PBUF                   12
#define PBUF_POOL_SIZE                  6
#define MEMP_NUM_SYS_TIMEOUT            6
#define MEMP_NUM_UDP_PCB                1
#define MEMP_NUM_TCP_PCB                2
#define MEMP_NUM_TCP_PCB_LISTEN         1
#define MEMP_NUM_TCP_SEG                4

#define TCP_MSS                         536
#define TCP_SND_BUF                     536
#define TCP_WND                         (2 * TCP_MSS)
#define TCP_SND_QUEUELEN                4
#define TCP_QUEUE_OOSEQ                 0
#define TCP_LISTEN_BACKLOG              0
#define LWIP_DISABLE_TCP_SANITY_CHECKS  1

#define ARP_TABLE_SIZE                  1
#define ARP_QUEUEING                    0

#endif /* RNDIS_DASHBOARD_LWIPOPTS_H */