/*
 * Copyright (c) 2024 Arm Limited. All rights reserved.
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
 *
 * -----------------------------------------------------------------------------
 *
 * $Date:       28. May 2024
 * $Revision:   V1.0
 *
 * Project:     USB Host EHCI Controller Driver Registers header
 *              for customized EHCI.
 *              (with full/low speed support)
 *
 * -----------------------------------------------------------------------------
 */

#ifndef DRIVER_USBH_EHCI_REGS_H_
#define DRIVER_USBH_EHCI_REGS_H_

#include <stdint.h>

#define USBH_EHCI_PORT                  1 //Only one port
// Structure Definitions
typedef struct                          // qH/iTD/siTD Common Link Pointer
{
    struct                                // Double Word 0 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Typ                :  2;   // qH, iTD, siTD or FSTN Item Type
        uint32_t Rsvd0              :  2;   // Reserved - unused bits
        uint32_t LinkPtr            : 27;   // Link Pointer
    } DW0;
} USBH_EHCI_COMMON;

typedef struct                          // Isochronous Transfer Descriptor (iTD)
{
    struct                                // iTD Double Word 0 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Typ                :  2;   // qH, iTD, siTD or FSTN Item Type
        uint32_t Rsvd0              :  2;   // Reserved - unused bits
        uint32_t NextLinkPtr        : 27;   // Next Link Pointer
    } DW0;
    struct                                // iTD Double Word 1 (32 bits)
    {
        uint32_t Offset             : 12;   // Transaction 0 Offset
        uint32_t PAGE               :  3;   // Page Select
        uint32_t IOC                :  1;   // Interrupt On Complete
        volatile uint32_t Length    : 12;   // Transaction 0 Length
        volatile uint32_t Status    :  4;   // Status
    } DW1;
    struct                                // iTD Double Word 2 (32 bits)
    {
        uint32_t Offset             : 12;   // Transaction 1 Offset
        uint32_t PAGE               :  3;   // Page Select
        uint32_t IOC                :  1;   // Interrupt On Complete
        volatile uint32_t Length    : 12;   // Transaction 1 Length
        volatile uint32_t Status    :  4;   // Status
    } DW2;
    struct                                // iTD Double Word 3 (32 bits)
    {
        uint32_t Offset             : 12;   // Transaction 2 Offset
        uint32_t PAGE               :  3;   // Page Select
        uint32_t IOC                :  1;   // Interrupt On Complete
        volatile uint32_t Length    : 12;   // Transaction 2 Length
        volatile uint32_t Status    :  4;   // Status
    } DW3;
    struct                                // iTD Double Word 4 (32 bits)
    {
        uint32_t Offset             : 12;   // Transaction 3 Offset
        uint32_t PAGE               :  3;   // Page Select
        uint32_t IOC                :  1;   // Interrupt On Complete
        volatile uint32_t Length    : 12;   // Transaction 3 Length
        volatile uint32_t Status    :  4;   // Status
    } DW4;
    struct                                // iTD Double Word 5 (32 bits)
    {
        uint32_t Offset             : 12;   // Transaction 4 Offset
        uint32_t PAGE               :  3;   // Page Select
        uint32_t IOC                :  1;   // Interrupt On Complete
        volatile uint32_t Length    : 12;   // Transaction 4 Length
        volatile uint32_t Status    :  4;   // Status
    } DW5;
    struct                                // iTD Double Word 6 (32 bits)
    {
        uint32_t Offset             : 12;   // Transaction 5 Offset
        uint32_t PAGE               :  3;   // Page Select
        uint32_t IOC                :  1;   // Interrupt On Complete
        volatile uint32_t Length    : 12;   // Transaction 5 Length
        volatile uint32_t Status    :  4;   // Status
    } DW6;
    struct                                // iTD Double Word 7 (32 bits)
    {
        uint32_t Offset             : 12;   // Transaction 6 Offset
        uint32_t PG                 :  3;   // Page Select
        uint32_t IOC                :  1;   // Interrupt On Complete
        volatile uint32_t Length    : 12;   // Transaction 6 Length
        volatile uint32_t Status    :  4;   // Status
    } DW7;
    struct                                // iTD Double Word 8 (32 bits)
    {
        uint32_t Offset             : 12;   // Transaction 7 Offset
        uint32_t PAGE               :  3;   // Page Select
        uint32_t IOC                :  1;   // Interrupt On Complete
        volatile uint32_t Length    : 12;   // Transaction 7 Length
        volatile uint32_t Status    :  4;   // Status
    } DW8;
    struct                                // iTD Double Word 9 (32 bits)
    {
        uint32_t DevAddr            :  7;   // Device Address
        uint32_t Rsvd0              :  1;   // Reserved
        uint32_t EndPt              :  4;   // Endpoint Number
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 0)
    } DW9;
    struct                                // iTD Double Word 10 (32 bits)
    {
        uint32_t MaxPcktSz          : 11;   // Maximum Packet Size
        uint32_t IO                 :  1;   // Direction (0 = OUT, 1 = IN)
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 1)
    } DW10;
    struct                                // iTD Double Word 11 (32 bits)
    {
        uint32_t Mult               :  2;   // Num of transactions per microframe
        uint32_t Rsvd0              : 10;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 2)
    } DW11;
    struct                                // iTD Double Word 12 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 3)
    } DW12;
    struct                                // iTD Double Word 13 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 4)
    } DW13;
    struct                                // iTD Double Word 14 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 5)
    } DW14;
    struct                                // iTD Double Word 15 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 6)
    } DW15;
} USBH_EHCI_iTD;

typedef struct                          // Split Transaction Isochronous Transfer Descriptor (siTD)
{
    struct                                // siTD Double Word 0 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Typ                :  2;   // qH, iTD, siTD or FSTN Item Type
        uint32_t Rsvd0              :  2;   // Reserved - unused bits
        uint32_t NextLinkPtr        : 27;   // Next Link Pointer
    } DW0;
    struct                                // siTD Double Word 1 (32 bits)
    {
        uint32_t DevAddr            :  7;   // Device Address
        uint32_t Rsvd0              :  1;   // Reserved
        uint32_t EndPt              :  4;   // Endpoint Number
        uint32_t Rsvd1              :  4;   // Reserved
        uint32_t HubAddr            :  7;   // HUB Address
        uint32_t Rsvd2              :  1;   // Reserved
        uint32_t PortNum            :  7;   // Port Number
        uint32_t IO                 :  1;   // Direction (0 = OUT, 1 = IN)
    } DW1;
    struct                                // siTD Double Word 2 (32 bits)
    {
        uint32_t SSM                :  8;   // Split Start Mask (uFrame S-mask)
        uint32_t SCM                :  8;   // Split Completion Mask (uFrame C-Mask)
        uint32_t Rsvd0              : 16;   // Reserved
    } DW2;
    struct                                // siTD Double Word 3 (32 bits)
    {
        volatile uint32_t Status    :  8;   // Status
        volatile uint32_t CPM       :  8;   // uFrame Complete-split Progress Mask (C-prog-Mask)
        volatile uint32_t TBT       : 10;   // Total Bytes to Transfer
        volatile uint32_t Rsvd0     :  4;   // Reserved
        volatile uint32_t P         :  1;   // Page Select
        uint32_t IOC                :  1;   // Interrupt On Complete
    } DW3;
    struct                                // siTD Double Word 4 (32 bits)
    {
        volatile uint32_t CurOfs    : 12;   // Current Offset
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 0)
    } DW4;
    struct                                // siTD Double Word 5 (32 bits)
    {
        volatile uint32_t TC        :  3;   // Transaction Count
        volatile uint32_t TP        :  2;   // Transaction Position
        uint32_t Rsvd0              :  7;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 1)
    } DW5;
    struct                                // siTD Double Word 6 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Rsvd0              :  4;   // Reserved
        uint32_t BackPtr            : 27;   // siTD Back Pointer
    } DW6;
    uint32_t RsvdDW[1];                   // Reserved to align qH to 32 bytes
} USBH_EHCI_siTD;

typedef struct                          // Split Transaction Isochronous Transfer Descriptor (siTD)
{
    uint32_t DW[8];                       // structure allowing direct Double Word access
} USBH_EHCI_siTD_DW;

typedef struct                          // Queue Element Transfer Desc (qTD)
{
    struct                                // qTD Double Word 0 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Rsvd0              :  4;   // Reserved - unused bits
        uint32_t NextPtr            : 27;   // Next Transfer Element Pointer
    } DW0;
    struct                                // qTD Double Word 1 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Rsvd0              :  4;   // Reserved - unused bits
        uint32_t AltNextPtr         : 27;   // Alternate Next Transfer Element Ptr
    } DW1;
    struct                                // qTD Double Word 2 (32 bits)
    {
        volatile uint32_t Status    :  8;   // Status
        uint32_t PID                :  2;   // PID Code
        volatile uint32_t CERR      :  2;   // Error Counter
        volatile uint32_t C_Page    :  3;   // Current Page
        uint32_t IOC                :  1;   // Interrupt On Complete
        volatile uint32_t TBT       : 15;   // Total Bytes to Transfer
        volatile uint32_t DT        :  1;   // Data Toggle
    } DW2;
    struct                                // qTD Double Word 3 (32 bits)
    {
        volatile uint32_t CurOfs    : 12;   // Current Offset
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 0)
    } DW3;
    struct                                // qTD Double Word 4 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 1)
    } DW4;
    struct                                // qTD Double Word 5 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 2)
    } DW5;
    struct                                // qTD Double Word 6 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 3)
    } DW6;
    struct                                // qTD Double Word 7 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 4)
    } DW7;
} USBH_EHCI_qTD;

typedef struct                          // Queue Head (qH)
{
    struct                                // qH Double Word 0 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Typ                :  2;   // QH, iTD, siTD or FSTN Item Type
        uint32_t Rsvd0              :  2;   // Reserved - unused bits
        uint32_t HorizLinkPtr       : 27;   // Queue Head Horizontal Link Pointer
    } DW0;
    struct                                // qH Double Word 1 (32 bits)
    {
        uint32_t DevAddr            :  7;   // Device Address
        uint32_t I                  :  1;   // Inactive on Next Transaction
        uint32_t EndPt              :  4;   // Endpoint Number
        uint32_t EPS                :  2;   // Endpoint Speed
        uint32_t DTC                :  1;   // Data Toggle Control
        uint32_t H                  :  1;   // Head of Reclamation List Flag
        uint32_t MaxPcktLen         : 11;   // Maximum Packet Length
        uint32_t C                  :  1;   // Control Endpoint Flag
        uint32_t RL                 :  4;   // NAK Count Reload
    } DW1;
    struct                                // qH Double Word 2 (32 bits)
    {
        uint32_t ISM                :  8;   // Interrupt Schedule Mask (uFrame S-mask)
        uint32_t SCM                :  8;   // Split Completion Mask (uFrame C-Mask)
        uint32_t HubAddr            :  7;   // HUB address
        uint32_t PortNum            :  7;   // Port Number
        uint32_t Mult               :  2;   // High-Bandwidth Pipe Multiplier
    } DW2;
    volatile struct                       // qH Double Word 3 (32 bits)
    {
        uint32_t Rsvd0              :  5;   // Reserved
        uint32_t CurrPtr            : 27;   // Current Element Transaction Descriptor Link Pointer
    } DW3;
    volatile struct                       // qH Double Word 4 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Rsvd0              :  4;   // Reserved
        uint32_t NextPtr            : 27;   // Next Element Transaction Descriptor Link Pointer
    } DW4;
    volatile struct                       // qH Double Word 5 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t NakCnt             :  4;   // NAK Counter
        uint32_t AltNextPtr         : 27;   // Alternate Next Element Transaction Descriptor Link Pointer
    } DW5;
    volatile struct                       // qH Double Word 6 (32 bits)
    {
        uint32_t Status             :  8;   // Status
        uint32_t PID                :  2;   // PID Code
        uint32_t CERR               :  2;   // Error Counter
        uint32_t C_Page             :  3;   // Current Page
        uint32_t IOC                :  1;   // Interrupt On Complete
        uint32_t TBT                : 15;   // Total Bytes to Transfer
        uint32_t DT                 :  1;   // Data Toggle
    } DW6;
    volatile struct                       // qH Double Word 7 (32 bits)
    {
        uint32_t CurOfs             : 12;   // Current Offset
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 0)
    } DW7;
    volatile struct                       // qH Double Word 8 (32 bits)
    {
        uint32_t CPM                :  8;   // Split-transaction Complete-split Progress (C-prog-mask)
        uint32_t Rsvd0              :  4;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 1)
    } DW8;
    volatile struct                       // qH Double Word 9 (32 bits)
    {
        uint32_t FrameTag           :  5;   // Split-transaction Frame Tag (Frame Tag)
        uint32_t SBytes             :  7;   // S-bytes
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 2)
    } DW9;
    volatile struct                       // qH Double Word 10 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 3)
    } DW10;
    volatile struct                       // qH Double Word 11 (32 bits)
    {
        uint32_t Rsvd0              : 12;   // Reserved
        uint32_t BufPtr             : 20;   // Buffer Pointer (Page 4)
    } DW11;
    uint32_t RsvdDW[4];                   // Reserved to align qH to 64 bytes
} USBH_EHCI_qH;

typedef struct                          // Periodic Frame Span Traversal Node (FSTN)
{
    struct                                // FSTN Double Word 0 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Typ                :  2;   // qH, iTD, siTD or FSTN Item Type
        uint32_t NPLP               : 29;   // Normal Path Link Pointer (NPLP)
    } DW0;
    struct                                // FSTN Double Word 1 (32 bits)
    {
        uint32_t T                  :  1;   // Terminate
        uint32_t Typ                :  2;   // Must be qH Type
        uint32_t BackPathLinkPtr    : 29;   // Back Path Link Pointer
    } DW1;
} USBH_EHCI_FSTN;


// Constant Definitions

/*----------------------------------------------------------------------------------------*/
/*  Interrupt Threshold Control (1, 2, 4, 6, .. 64)                                       */
/*----------------------------------------------------------------------------------------*/
#define UCMDR_INT_THR_CTRL     (0x1<<HSUSBH_UCMDR_ITC_Pos)     /* 1 micro-frames          */


#define USBH_EHCI_ASYNC_SCHED           (        0U)
#define USBH_EHCI_PERIODIC_SCHED        (        1U)

#define USBH_EHCI_ELEMENT_TYPE_iTD      (0U)
#define USBH_EHCI_ELEMENT_TYPE_qH       (1U)
#define USBH_EHCI_ELEMENT_TYPE_siTD     (2U)
#define USBH_EHCI_ELEMENT_TYPE_FSTN     (3U)


#define USBH_EHCI_ISO_DELAY             (2U)

#endif /* DRIVER_USBH_EHCI_REGS_H_ */
