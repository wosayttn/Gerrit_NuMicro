/******************************************************************************
 * @file     descriptors.c
 * @version  V2.00
 * @brief    USB composite device(CCID smart card reader and VCOM) sample descriptors file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"
#include "ccid_vcom.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t gu8DeviceDescriptor[] =
{
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x10, 0x01,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    (USBD_VID & 0xFF00) >> 8,
    /* idProduct */
    USBD_PID & 0x00FF,
    (USBD_PID & 0xFF00) >> 8,
    0x00, 0x00,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber - no serial */
    0x01            /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
const uint8_t gu8ConfigDescriptor[] =
{
    LEN_CONFIG,     /* bLength              */
    DESC_CONFIG,    /* bDescriptorType      */
    /* wTotalLength */
    (LEN_CONFIG + LEN_INTERFACE * 3 + LEN_CCID + LEN_ENDPOINT * 6 + LEN_VCOM + LEN_IAD) & 0x00FF,
    ((LEN_CONFIG + LEN_INTERFACE * 3 + LEN_CCID + LEN_ENDPOINT * 6 + LEN_VCOM + LEN_IAD) & 0xFF00) >> 8,
    0x03,           /* bNumInterfaces      */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration      */
    0x80,           /* bmAttributes        */
    0x32,           /* MaxPower            */

    /* Interface descriptor (Interface 0 = Smart Card Reader) */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x03,           /* bNumEndpoints */
    0x0B,           /* bInterfaceClass */
    0x00,           /* bInterfaceSubClass */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* CCID class descriptor */
    0x36,           /* bLength: CCID Descriptor size */
    0x21,           /* bDescriptorType: HID To be updated with CCID specific number */
    0x00,           /* bcdHID(LSB): CCID Class Spec release number (1.10) */
    0x01,           /* bcdHID(MSB) */
    0x01,           /* bMaxSlotIndex */
    0x07,           /* bVoltageSupport: 5v, 3v and 1.8v */
    0x03, 0x00, 0x00, 0x00,         /* dwProtocols: supports T=0 and T=1 */
    0xA0, 0x0F, 0x00, 0x00,         /* dwDefaultClock: 4 Mhz (0x00000FA0) */
    0xA0, 0x0F, 0x00, 0x00,         /* dwMaximumClock: 4 Mhz (0x00000FA0) */
    0x00,                           /* bNumClockSupported => no manual setting */
    0xDA, 0x26, 0x00, 0x00,         /* dwDataRate: 10080 bps  //10752 bps (0x00002A00) */
    0x48, 0xDB, 0x04, 0x00,         /* dwMaxDataRate: 312500 bps  // 129032 bps (0x0001F808) */
    0x00,                           /* bNumDataRatesSupported => no manual setting */
    0xFE, 0x00, 0x00, 0x00,         /* dwMaxIFSD: 0 (T=0 only)   */
    0x07, 0x00, 0x00, 0x00,         /* dwSynchProtocols  */
    0x00, 0x00, 0x00, 0x00,         /* dwMechanical: no special characteristics */
    0xBA, 0x04, 0x02, 0x00,         /* dwFeatures: clk, baud rate, voltage : automatic, clock stop mode */  // short APDU
    0x0F, 0x01, 0x00, 0x00,         /* dwMaxCCIDMessageLength : Maximum block size + header*/
    0xFF,                   /* bClassGetResponse*/
    0xFF,                   /* bClassEnvelope */
    0x00, 0x00,             /* wLcdLayout */
    0x00,                   /* bPINSupport : no PIN verif and modif  */
    0x01,                   /* bMaxCCIDBusySlots */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM),     /* bEndpointAddress */
    EP_INT,                         /* bmAttributes     */
    EP4_MAX_PKT_SIZE, 0x00,             /* wMaxPacketSize   */
    0x01,                           /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM),    /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP2_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM),  /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP3_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    //VCOM-1
    // IAD
    LEN_IAD,               // bLength: Interface Descriptor size
    0x0B,               // bDescriptorType: IAD
    0x01,               // bFirstInterface
    0x02,               // bInterfaceCount
    0x02,               // bFunctionClass: CDC
    0x02,               // bFunctionSubClass
    0x01,               // bFunctionProtocol
    0x02,               // iFunction

    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x01,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x01,           /* bNumEndpoints        */
    0x02,           /* bInterfaceClass      */
    0x02,           /* bInterfaceSubClass   */
    0x01,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x00,           /* Header functional descriptor subtype */
    0x10, 0x01,     /* Communication device compliant to the communication spec. ver. 1.10 */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x01,           /* Call management functional descriptor */
    0x00,           /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a Data Class Interface 0 */
    0x01,           /* Interface number of data class interface optionally used for call management */

    /* Communication Class Specified INTERFACE descriptor */
    0x04,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x02,           /* Abstract control management functional descriptor subtype */
    0x00,           /* bmCapabilities       */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* bLength              */
    0x24,           /* bDescriptorType: CS_INTERFACE descriptor type */
    0x06,           /* bDescriptorSubType   */
    0x00,           /* bMasterInterface     */
    0x01,           /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | VCOM_INT_IN_EP_NUM),     /* bEndpointAddress */
    EP_INT,                         /* bmAttributes     */
    EP7_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x01,                           /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x02,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x02,           /* bNumEndpoints        */
    0x0A,           /* bInterfaceClass      */
    0x00,           /* bInterfaceSubClass   */
    0x00,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | VCOM_BULK_IN_EP_NUM),    /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP5_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_OUTPUT | VCOM_BULK_OUT_EP_NUM),  /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP6_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00                            /* bInterval        */
};

/*!<USB Language String Descriptor */
const uint8_t gu8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
const uint8_t gu8VendorStringDesc[] =
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
const uint8_t gu8ProductStringDesc[] =
{
    52,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'N', 0, 'u', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, ' ', 0,
    'S', 0, 'm', 0, 'a', 0, 'r', 0, 't', 0, ' ', 0,
    'C', 0, 'a', 0, 'r', 0, 'd', 0, ' ', 0,
    'R', 0, 'e', 0, 'a', 0, 'd', 0, 'e', 0, 'r', 0
};



const uint8_t gu8StringSerial[26] =
{
    26,             // bLength
    DESC_STRING,    // bDescriptorType
    'A', 0, '0', 0, '2', 0, '0', 0, '1', 0, '4', 0, '0', 0, '9', 0, '0', 0, '5', 0, '0', 0, '1', 0
};

const uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const S_USBD_INFO_T gsInfo =
{
    (uint8_t *) gu8DeviceDescriptor,
    (uint8_t *) gu8ConfigDescriptor,
    (uint8_t **) gpu8UsbString,
    NULL,
    NULL,
    NULL,
    NULL
};

