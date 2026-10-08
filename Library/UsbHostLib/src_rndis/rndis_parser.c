/**************************************************************************//**
 * @file     rndis_parser.c
 * @brief    Bounded USB configuration descriptor matcher for RNDIS.
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

#define RNDIS_COMM_CLASS_E0       0xE0U
#define RNDIS_COMM_SUBCLASS_E0    0x01U
#define RNDIS_COMM_PROTOCOL_E0    0x03U
#define RNDIS_COMM_CLASS_CDC      0x02U
#define RNDIS_COMM_SUBCLASS_CDC   0x02U
#define RNDIS_COMM_PROTOCOL_CDC   0xFFU
#define RNDIS_CS_INTERFACE        0x24U
#define RNDIS_UNION_SUBTYPE       0x06U
#define RNDIS_IAD_DESCRIPTOR      0x0BU

/**
 * @brief Read one little-endian 32-bit value from a byte buffer.
 * @param[in] data Pointer to at least four bytes containing the value.
 * @return The decoded unsigned 32-bit value.
 * @note The input buffer is not modified and must be valid for the complete
 *       four-byte read.
 */
uint32_t rndis_get_le32(uint8_t const *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/**
 * @brief Write one unsigned 32-bit value in little-endian byte order.
 * @param[out] data Pointer to at least four writable bytes.
 * @param[in] value Value to encode.
 * @note The caller owns the destination buffer and must ensure that it is
 *       valid for the complete four-byte write.
 */
void rndis_put_le32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

/**
 * @brief Test whether an interface descriptor uses a supported RNDIS profile.
 * @param[in] class_code USB interface class code.
 * @param[in] subclass USB interface subclass code.
 * @param[in] protocol USB interface protocol code.
 * @return 1 when the class/subclass/protocol triplet is supported; otherwise 0.
 */
static int rndis_is_comm_profile(uint8_t class_code, uint8_t subclass, uint8_t protocol)
{
    if ((class_code == RNDIS_COMM_CLASS_E0) &&
            (subclass == RNDIS_COMM_SUBCLASS_E0) &&
            (protocol == RNDIS_COMM_PROTOCOL_E0))
    {
        return 1;
    }

    if ((class_code == RNDIS_COMM_CLASS_CDC) &&
            (subclass == RNDIS_COMM_SUBCLASS_CDC) &&
            (protocol == RNDIS_COMM_PROTOCOL_CDC))
    {
        return 1;
    }

    return 0;
}

/**
 * @brief Validate and return the active configuration descriptor.
 * @param[in] udev USB device whose configuration descriptor is inspected.
 * @param[out] buffer Receives the descriptor buffer address.
 * @param[out] total Receives the validated descriptor length.
 * @return USBH_OK on success, otherwise a USB descriptor error code.
 * @note The returned buffer belongs to @p udev and must not be freed or
 *       retained beyond the device lifecycle.
 */
static int rndis_get_config(UDEV_T const *udev, uint8_t const **buffer, uint16_t *total)
{
    uint16_t length;

    if ((udev == USBNULL) || (udev->cfd_buff == USBNULL))
    {
        return USBH_ERR_DESCRIPTOR;
    }

    if ((udev->cfd_buff[0] < 9U) || (udev->cfd_buff[1] != USB_DT_CONFIGURATION))
    {
        return USBH_ERR_DESCRIPTOR;
    }

    length = (uint16_t)udev->cfd_buff[2] | ((uint16_t)udev->cfd_buff[3] << 8);

    if ((length < (uint16_t)udev->cfd_buff[0]) || (length > (uint16_t)MAX_DESC_BUFF_SIZE))
    {
        return USBH_ERR_DESCRIPTOR;
    }

    *buffer = udev->cfd_buff;
    *total = length;
    return USBH_OK;
}

/**
 * @brief Validate a bulk endpoint address and maximum packet size.
 * @param[in] speed USB bus speed used by the device.
 * @param[in] ep_addr Endpoint address, including direction.
 * @param[in] mps Raw `wMaxPacketSize` value from the endpoint descriptor.
 * @return 1 when the endpoint is a valid bulk endpoint for @p speed; otherwise 0.
 */
static int rndis_is_valid_bulk_mps(SPEED_E speed, uint8_t ep_addr, uint16_t mps)
{
    uint16_t payload = mps & 0x07FFU;

    /* Bulk endpoints have no high-bandwidth transaction-count encoding. */
    if (((ep_addr & 0x0FU) == 0U) || ((mps & 0xF800U) != 0U) || (payload == 0U))
    {
        return 0;
    }

    if (speed == SPEED_FULL)
    {
        return (payload <= 64U) ? 1 : 0;
    }

    if (speed == SPEED_HIGH)
    {
        return (payload <= 512U) ? 1 : 0;
    }

    return 0;
}

/**
 * @brief Validate the interrupt notification endpoint packet size.
 * @param[in] mps Raw `wMaxPacketSize` value from the endpoint descriptor.
 * @return 1 when the value can carry an RNDIS notification; otherwise 0.
 */
static int rndis_is_valid_notification_mps(uint16_t mps)
{
    uint16_t payload = mps & 0x07FFU;

    return (((mps & 0x0800U) == 0U) && (payload >= RNDIS_NOTIFICATION_SIZE)) ? 1 : 0;
}

/**
 * @brief Find the unique communication/data interface pair described by Union descriptors.
 * @param[in] buffer Complete USB configuration descriptor buffer.
 * @param[in] total Number of valid bytes in @p buffer.
 * @param[in] comm_hint Communication interface number, or a negative value for any.
 * @param[in] data_hint Data interface number, or a negative value for any.
 * @param[out] comm_ifnum Selected communication interface number.
 * @param[out] data_ifnum Selected data interface number.
 * @return USBH_OK when exactly one valid pair is found; otherwise a descriptor
 *         or not-matched error code.
 * @note This MVP accepts exactly one communication interface and one data slave
 *       in a five-byte Union functional descriptor.
 */
static int rndis_find_union(uint8_t const *buffer, uint16_t total, int comm_hint, int data_hint,
                            uint8_t *comm_ifnum, uint8_t *data_ifnum)
{
    uint16_t offset = buffer[0];
    uint8_t current_ifnum = 0U;
    uint8_t current_comm = 0U;
    uint8_t current_active = 0U;
    uint8_t current_union_seen = 0U;
    uint8_t candidate_count = 0U;

    while (offset < total)
    {
        uint8_t const *desc = &buffer[offset];
        uint8_t length = desc[0];

        if ((length < 2U) || ((uint16_t)((uint32_t)offset + (uint32_t)length) > total))
        {
            return USBH_ERR_DESCRIPTOR;
        }

        if (desc[1] == USB_DT_INTERFACE)
        {
            if (length < 9U)
            {
                return USBH_ERR_DESCRIPTOR;
            }

            current_ifnum = desc[2];
            current_active = (desc[3] == 0U) ? 1U : 0U;
            current_comm = ((current_active != 0U) &&
                            (rndis_is_comm_profile(desc[5], desc[6], desc[7]) != 0)) ? 1U : 0U;
            current_union_seen = 0U;
        }
        else if ((desc[1] == RNDIS_CS_INTERFACE) && (current_active != 0U) &&
                 (current_comm != 0U) && (length >= 3U) && (desc[2] == RNDIS_UNION_SUBTYPE))
        {
            if (length < 5U)
            {
                return USBH_ERR_DESCRIPTOR;
            }

            /* MVP owns one communication interface and exactly one data slave. */
            if ((length != 5U) || (desc[3] != current_ifnum) || (current_union_seen != 0U))
            {
                return USBH_ERR_NOT_MATCHED;
            }

            current_union_seen = 1U;

            if (((comm_hint < 0) || (current_ifnum == (uint8_t)comm_hint)) &&
                    ((data_hint < 0) || (desc[4] == (uint8_t)data_hint)))
            {
                candidate_count++;

                if (candidate_count > 1U)
                {
                    return USBH_ERR_NOT_MATCHED;
                }

                *comm_ifnum = current_ifnum;
                *data_ifnum = desc[4];
            }
        }
        else
        {
            /* Ignore other descriptors. */
        }

        offset = (uint16_t)(offset + length);
    }

    if (candidate_count != 1U)
    {
        return USBH_ERR_NOT_MATCHED;
    }

    /* A selected data interface may have only this Union communication owner. */
    offset = buffer[0];
    current_active = 0U;

    while (offset < total)
    {
        uint8_t const *desc = &buffer[offset];
        uint8_t length = desc[0];

        if ((length < 2U) || ((uint16_t)((uint32_t)offset + (uint32_t)length) > total))
        {
            return USBH_ERR_DESCRIPTOR;
        }

        if (desc[1] == USB_DT_INTERFACE)
        {
            if (length < 9U)
            {
                return USBH_ERR_DESCRIPTOR;
            }

            current_ifnum = desc[2];
            current_active = (desc[3] == 0U) ? 1U : 0U;
        }
        else if ((desc[1] == RNDIS_CS_INTERFACE) && (current_active != 0U) &&
                 (length >= 3U) && (desc[2] == RNDIS_UNION_SUBTYPE))
        {
            if (length < 5U)
            {
                if (current_ifnum == *comm_ifnum)
                {
                    return USBH_ERR_DESCRIPTOR;
                }
            }
            else
            {
                uint8_t references_selected_data = 0U;
                uint8_t slave_index;

                for (slave_index = 4U; slave_index < length; slave_index++)
                {
                    if (desc[slave_index] == *data_ifnum)
                    {
                        references_selected_data = 1U;
                        break;
                    }
                }

                if ((current_ifnum == *comm_ifnum) || (references_selected_data != 0U))
                {
                    if ((length != 5U) || (desc[3] != current_ifnum) ||
                            (desc[3] != *comm_ifnum) || (desc[4] != *data_ifnum))
                    {
                        return USBH_ERR_NOT_MATCHED;
                    }
                }
            }
        }
        else
        {
            /* Ignore other descriptors. */
        }

        offset = (uint16_t)(offset + length);
    }

    return USBH_OK;
}

/**
 * @brief Validate the endpoint topology of a selected RNDIS interface pair.
 * @param[in] udev USB device providing the bus speed for endpoint validation.
 * @param[in] buffer Complete USB configuration descriptor buffer.
 * @param[in] total Number of valid bytes in @p buffer.
 * @param[in,out] pair Selected interface pair; endpoint addresses and MPS values
 *                    are filled on success.
 * @return USBH_OK when the pair contains the required interfaces and endpoints;
 *         otherwise a descriptor or not-matched error code.
 * @note The accepted topology requires one bulk IN, one bulk OUT, and a valid
 *       communication interface. The interrupt notification endpoint is optional.
 */
static int rndis_validate_topology(UDEV_T const *udev, uint8_t const *buffer, uint16_t total,
                                   RNDIS_PAIR_INFO_T *pair)
{
    uint16_t offset = buffer[0];
    uint8_t current_ifnum = 0U;
    uint8_t current_active = 0U;
    uint8_t comm_found = 0U;
    uint8_t data_found = 0U;
    uint8_t bulk_in = 0U;
    uint8_t bulk_out = 0U;
    uint8_t int_in = 0U;
    uint8_t iad_for_pair = 0U;
    uint8_t iad_conflict = 0U;

    while (offset < total)
    {
        uint8_t const *desc = &buffer[offset];
        uint8_t length = desc[0];

        if ((length < 2U) || ((uint16_t)((uint32_t)offset + (uint32_t)length) > total))
        {
            return USBH_ERR_DESCRIPTOR;
        }

        if (desc[1] == RNDIS_IAD_DESCRIPTOR)
        {
            uint16_t iad_start;
            uint16_t iad_end;
            uint8_t contains_comm;
            uint8_t contains_data;

            if (length < 8U)
            {
                return USBH_ERR_DESCRIPTOR;
            }

            iad_start = (uint16_t)desc[2];
            iad_end = iad_start + (uint16_t)desc[3];
            contains_comm = (((uint16_t)pair->comm_ifnum >= iad_start) &&
                             ((uint16_t)pair->comm_ifnum < iad_end)) ? 1U : 0U;
            contains_data = (((uint16_t)pair->data_ifnum >= iad_start) &&
                             ((uint16_t)pair->data_ifnum < iad_end)) ? 1U : 0U;

            if ((contains_comm != 0U) || (contains_data != 0U))
            {
                if ((contains_comm == 0U) || (contains_data == 0U))
                {
                    iad_conflict = 1U;
                }
                else
                {
                    iad_for_pair = 1U;
                }
            }
        }
        else if (desc[1] == USB_DT_INTERFACE)
        {
            if (length < 9U)
            {
                return USBH_ERR_DESCRIPTOR;
            }

            current_ifnum = desc[2];
            current_active = (desc[3] == 0U) ? 1U : 0U;

            if ((current_active != 0U) && (current_ifnum == pair->comm_ifnum) &&
                    (rndis_is_comm_profile(desc[5], desc[6], desc[7]) != 0))
            {
                comm_found = 1U;
            }

            if ((current_active != 0U) && (current_ifnum == pair->data_ifnum) &&
                    (desc[5] == (uint8_t)USB_CLASS_DATA) && (desc[6] == 0U) && (desc[7] == 0U))
            {
                data_found = 1U;
            }
        }
        else if ((desc[1] == USB_DT_ENDPOINT) && (current_active != 0U) &&
                 (current_ifnum == pair->data_ifnum))
        {
            uint16_t mps;

            if (length < 7U)
            {
                return USBH_ERR_DESCRIPTOR;
            }

            mps = (uint16_t)desc[4] | ((uint16_t)desc[5] << 8);

            if ((desc[3] & EP_ATTR_TT_MASK) == EP_ATTR_TT_BULK)
            {
                if (rndis_is_valid_bulk_mps(udev->speed, desc[2], mps) == 0)
                {
                    RNDIS_ERRMSG("RNDIS: invalid bulk endpoint 0x%02x MPS 0x%04x at speed %u.\n",
                                 desc[2], mps, (uint32_t)udev->speed);
                    return USBH_ERR_NOT_MATCHED;
                }

                if (((desc[2] & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_IN) && (bulk_in == 0U))
                {
                    bulk_in = 1U;
                    pair->bulk_in_addr = desc[2];
                    pair->bulk_in_mps = mps;
                }
                else if (((desc[2] & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_OUT) && (bulk_out == 0U))
                {
                    bulk_out = 1U;
                    pair->bulk_out_addr = desc[2];
                    pair->bulk_out_mps = mps;
                }
                else
                {
                    /* Multiple same-direction Bulk endpoints are ambiguous for this MVP. */
                    return USBH_ERR_NOT_MATCHED;
                }
            }
        }
        else if ((desc[1] == USB_DT_ENDPOINT) && (current_active != 0U) &&
                 (current_ifnum == pair->comm_ifnum))
        {
            uint16_t mps;

            if (length < 7U)
            {
                return USBH_ERR_DESCRIPTOR;
            }

            mps = (uint16_t)desc[4] | ((uint16_t)desc[5] << 8);

            if (((desc[3] & EP_ATTR_TT_MASK) == EP_ATTR_TT_INT) &&
                    ((desc[2] & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_IN))
            {
                if ((int_in != 0U) || (rndis_is_valid_notification_mps(mps) == 0))
                {
                    return USBH_ERR_NOT_MATCHED;
                }

                int_in = 1U;
                pair->int_in_addr = desc[2];
                pair->int_in_mps = mps;
                pair->int_in_interval = desc[6];
            }
        }
        else
        {
            /* Ignore other descriptors. */
        }

        offset = (uint16_t)(offset + length);
    }

    if ((comm_found == 0U) || (data_found == 0U) || (bulk_in == 0U) || (bulk_out == 0U) ||
            (iad_conflict != 0U))
    {
        if (iad_conflict != 0U)
        {
            RNDIS_ERRMSG("RNDIS: IAD range conflicts with Union pair interfaces %u/%u.\n",
                         pair->comm_ifnum, pair->data_ifnum);
        }

        return USBH_ERR_NOT_MATCHED;
    }

    (void)iad_for_pair; /* IAD is optional and is only a Union-pair topology/range check. */
    return USBH_OK;
}

/**
 * @brief Locate and validate an RNDIS communication/data interface pair.
 * @param[in] udev USB device whose configuration descriptor is parsed.
 * @param[in] comm_ifnum Communication interface hint, or -1 for any interface.
 * @param[in] data_ifnum Data interface hint, or -1 for any interface.
 * @param[out] pair Receives the validated interface and endpoint information.
 * @return USBH_OK on success; otherwise USBH_ERR_INVALID_PARAM,
 *         USBH_ERR_DESCRIPTOR, USBH_ERR_NOT_MATCHED, or another parser error.
 * @note This function only parses descriptors. It does not claim interfaces,
 *       allocate DMA resources, or change USB device state.
 */
int rndis_find_pair(UDEV_T const *udev, int comm_ifnum, int data_ifnum, RNDIS_PAIR_INFO_T *pair)
{
    uint8_t const *buffer;
    uint16_t total;
    int ret;

    if (pair == USBNULL)
    {
        return USBH_ERR_INVALID_PARAM;
    }

    (void)memset(pair, 0, sizeof(*pair));
    ret = rndis_get_config(udev, &buffer, &total);

    if (ret != USBH_OK)
    {
        return ret;
    }

    ret = rndis_find_union(buffer, total, comm_ifnum, data_ifnum, &pair->comm_ifnum, &pair->data_ifnum);

    if (ret != USBH_OK)
    {
        return ret;
    }

    return rndis_validate_topology(udev, buffer, total, pair);
}

/// @endcond HIDDEN_SYMBOLS

/*! @}*/ /* end of group USBH_EXPORTED_FUNCTIONS */

/*! @}*/ /* end of group USBH_Library */

/*! @}*/ /* end of group LIBRARY */
