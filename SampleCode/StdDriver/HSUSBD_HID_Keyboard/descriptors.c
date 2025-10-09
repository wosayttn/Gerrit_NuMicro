/**************************************************************************//**
 * @file     descriptors.c
 * @version  V3.00
 * @brief    HSUSBD descriptor.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "hid_keyboard.h"

/*!<USB HID Report Descriptor */
uint8_t HID_KeyboardReportDescriptor[] =
{
    0x05, 0x01,     /* Usage Page(Generic Desktop Controls) */
    0x09, 0x06,     /* Usage(Keyboard) */
    0xA1, 0x01,     /* Collection(Application) */
    0x05, 0x07,         /* Usage Page(Keyboard/Keypad) */
    0x19, 0xE0,         /* Usage Minimum(0xE0) */
    0x29, 0xE7,         /* Usage Maximum(0xE7) */
    0x15, 0x00,         /* Logical Minimum(0x0) */
    0x25, 0x01,         /* Logical Maximum(0x1) */
    0x75, 0x01,         /* Report Size(0x1) */
    0x95, 0x08,         /* Report Count(0x8) */
    0x81, 0x02,         /* Input (Data) => Modifier byte */
    0x95, 0x01,         /* Report Count(0x1) */
    0x75, 0x08,         /* Report Size(0x8) */
    0x81, 0x01,         /* Input (Constant) => Reserved byte */
    0x95, 0x05,         /* Report Count(0x5) */
    0x75, 0x01,         /* Report Size(0x1) */
    0x05, 0x08,         /* Usage Page(LEDs) */
    0x19, 0x01,         /* Usage Minimum(0x1) */
    0x29, 0x05,         /* Usage Maximum(0x5) */
    0x91, 0x02,         /* Output (Data) => LED report */
    0x95, 0x01,         /* Report Count(0x1) */
    0x75, 0x03,         /* Report Size(0x3) */
    0x91, 0x01,         /* Output (Constant) => LED report padding */
    0x95, 0x06,         /* Report Count(0x6) */
    0x75, 0x08,         /* Report Size(0x8) */
    0x15, 0x00,         /* Logical Minimum(0x0) */
    0x25, 0x65,         /* Logical Maximum(0x65) */
    0x05, 0x07,         /* Usage Page(Keyboard/Keypad) */
    0x19, 0x00,         /* Usage Minimum(0x0) */
    0x29, 0x65,         /* Usage Maximum(0x65) */
    0x81, 0x00,         /* Input (Data) */
    0xC0            /* End Collection */
};

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
uint8_t gu8DeviceDescriptor[] =
{
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
#ifdef SUPPORT_LPM
    0x01, 0x02,     /* bcdUSB >= 0x0201 to support LPM */
#else
    0x00, 0x02,     /* bcdUSB */
#endif
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    CEP_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    ((USBD_VID & 0xFF00) >> 8),
    /* idProduct */
    USBD_PID & 0x00FF,
    ((USBD_PID & 0xFF00) >> 8),
    0x00, 0x00,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x00,           /* iSerialNumber - no serial */
    0x01            /* bNumConfigurations */
};

/*!<USB Qualifier Descriptor */
uint8_t gu8QualifierDescriptor[] =
{
    LEN_QUALIFIER,  /* bLength */
    DESC_QUALIFIER, /* bDescriptorType */
#ifdef SUPPORT_LPM
    0x01, 0x02,     /* bcdUSB >= 0x0201 to support LPM */
#else
    0x00, 0x02,     /* bcdUSB */
#endif
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    CEP_OTHER_MAX_PKT_SIZE, /* bMaxPacketSize0 */
    0x01,           /* bNumConfigurations */
    0x00
};

/*!<USB Configure Descriptor */
uint8_t gu8ConfigDescriptor[] =
{
    LEN_CONFIG,     /* bLength */
    DESC_CONFIG,    /* bDescriptorType */
    /* wTotalLength */
    LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
    ((LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8),
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5),/* bmAttributes */
    USBD_MAX_POWER,         /* MaxPower */

    /* I/F descr: HID */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x01,           /* bInterfaceSubClass */
    HID_KEYBOARD,   /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* HID Descriptor */
    LEN_HID,        /* Size of this descriptor in UINT8s. */
    DESC_HID,       /* HID descriptor type. */
    0x10, 0x01,     /* HID Class Spec. release number. */
    0x00,           /* H/W target country. */
    0x01,           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_KeyboardReportDescriptor) & 0x00FF,
    ((sizeof(HID_KeyboardReportDescriptor) & 0xFF00) >> 8),

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EPA_MAX_PKT_SIZE & 0x00FF,
    ((EPA_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL     /* bInterval */
};

uint8_t gu8OtherConfigDescriptorHS[] =
{
    LEN_CONFIG,     /* bLength */
    DESC_OTHERSPEED,    /* bDescriptorType */
    /* wTotalLength */
    LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
    ((LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8),
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5),/* bmAttributes */
    USBD_MAX_POWER,         /* MaxPower */

    /* I/F descr: HID */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x01,           /* bInterfaceSubClass */
    HID_MOUSE,      /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* HID Descriptor */
    LEN_HID,        /* Size of this descriptor in UINT8s. */
    DESC_HID,       /* HID descriptor type. */
    0x10, 0x01,     /* HID Class Spec. release number. */
    0x00,           /* H/W target country. */
    0x01,           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_KeyboardReportDescriptor) & 0x00FF,
    ((sizeof(HID_KeyboardReportDescriptor) & 0xFF00) >> 8),

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EPA_OTHER_MAX_PKT_SIZE & 0x00FF,
    ((EPA_OTHER_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL     /* bInterval */
};

/*!<USB Other Speed Configure Descriptor */
uint8_t gu8ConfigDescriptorFS[] =
{
    LEN_CONFIG,     /* bLength */
    DESC_CONFIG,    /* bDescriptorType */
    /* wTotalLength */
    LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
    ((LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8),
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5),/* bmAttributes */
    USBD_MAX_POWER,         /* MaxPower */

    /* I/F descr: HID */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x01,           /* bInterfaceSubClass */
    HID_MOUSE,      /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* HID Descriptor */
    LEN_HID,        /* Size of this descriptor in UINT8s. */
    DESC_HID,       /* HID descriptor type. */
    0x10, 0x01,     /* HID Class Spec. release number. */
    0x00,           /* H/W target country. */
    0x01,           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_KeyboardReportDescriptor) & 0x00FF,
    ((sizeof(HID_KeyboardReportDescriptor) & 0xFF00) >> 8),

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EPA_OTHER_MAX_PKT_SIZE & 0x00FF,
    ((EPA_OTHER_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL     /* bInterval */
};

uint8_t gu8OtherConfigDescriptorFS[] =
{
    LEN_CONFIG,     /* bLength */
    DESC_OTHERSPEED,    /* bDescriptorType */
    /* wTotalLength */
    LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
    ((LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8),
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5),/* bmAttributes */
    USBD_MAX_POWER,         /* MaxPower */

    /* I/F descr: HID */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x01,           /* bInterfaceSubClass */
    HID_MOUSE,      /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* HID Descriptor */
    LEN_HID,        /* Size of this descriptor in UINT8s. */
    DESC_HID,       /* HID descriptor type. */
    0x10, 0x01,     /* HID Class Spec. release number. */
    0x00,           /* H/W target country. */
    0x01,           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_KeyboardReportDescriptor) & 0x00FF,
    ((sizeof(HID_KeyboardReportDescriptor) & 0xFF00) >> 8),

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EPA_MAX_PKT_SIZE & 0x00FF,
    ((EPA_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL     /* bInterval */
};

/*!<USB Language String Descriptor */
uint8_t gu8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
uint8_t gu8VendorStringDesc[] =
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
uint8_t gu8ProductStringDesc[] =
{
    20,
    DESC_STRING,
    'H', 0, 'I', 0, 'D', 0, ' ', 0, 'M', 0, 'o', 0, 'u', 0, 's', 0, 'e', 0
};

uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    NULL
};

#ifdef SUPPORT_LPM
/*!<USB BOS Descriptor */
uint8_t gu8BOSDescriptor[] =
{
    LEN_BOS,        /* bLength */
    DESC_BOS,       /* bDescriptorType */
    /* wTotalLength */
    0x0C & 0x00FF,
    (0x0C & 0xFF00) >> 8,
    0x01,           /* bNumDeviceCaps */

    /* Device Capability */
    LEN_BOSCAP,     /* bLength */
    DESC_CAPABILITY,/* bDescriptorType */
    CAP_USB20_EXT,  /* bDevCapabilityType, 0x02 is USB 2.0 Extension */
    0x0E, 0x04, 0x00, 0x00  /* bmAttributes, 32 bits */
    /* bit 0 : Reserved. Must 0. */
    /* bit 1 : 1 to support LPM. */
    /* bit 2 : 1 to support BSL & Alternat HIRD. */
    /* bit 3 : 1 to recommend Baseline BESL. */
    /* bit 4 : 1 to recommand Deep BESL. */
    /* bit 11:8 : Recommend Baseline BESL value. Ignore by bit3 is zero. */
    /* bit 15:12 : Recommend Deep BESL value. Ignore by bit4 is zero. */
    /* bit 31:16 : Reserved. Must 0. */
};
#endif

uint8_t *gu8UsbHidReport[3] =
{
    HID_KeyboardReportDescriptor,
    NULL,
    NULL
};

uint32_t gu32UsbHidReportLen[3] =
{
    sizeof(HID_KeyboardReportDescriptor),
    0,
    0
};

uint32_t gu32ConfigHidDescIdx[3] =
{
    (LEN_CONFIG + LEN_INTERFACE),
    (sizeof(gu8ConfigDescriptor) - LEN_HID - LEN_ENDPOINT),
    0
};

S_HSUSBD_INFO_T gsHSInfo =
{
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    gu8QualifierDescriptor,
    gu8ConfigDescriptorFS,
    gu8OtherConfigDescriptorHS,
    gu8OtherConfigDescriptorFS,
#ifdef SUPPORT_LPM
    gu8BOSDescriptor,
#else
    NULL,
#endif
    gu8UsbHidReport,
    gu32UsbHidReportLen,
    gu32ConfigHidDescIdx
};
