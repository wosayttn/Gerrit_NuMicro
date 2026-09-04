/**************************************************************************//**
 * @file     lcdlib.c
 * @version  V3.00
 * @brief    RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD library source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

#include "lcdlib.h"

/** @addtogroup Library Library
  @{
*/

/** @addtogroup LCDLIB LCD Library
  @{
*/

/** @addtogroup LCDLIB_EXPORTED_FUNCTIONS LCD Library Exported Functions
  @{
*/

/**
 *  @brief Display text on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  InputStr    Text string to show on display
 */
void LCDLIB_Printf(uint32_t u32Zone, const char *InputStr)
{
    uint32_t i;
    uint32_t index;
    uint32_t len;
    int32_t ch;

    len = (uint32_t)strlen(InputStr);

    /* Fill out all characters on display */
    for (index = 0; index < g_LCDZoneInfo[u32Zone].u8LCDDispTableNum; index++)
    {
        if (index < len)
        {
            char c;

            c = InputStr[index];
            ch = (int32_t)c;
        }
        else
        {
            /* Padding with SPACE */
            ch = 0x20;
        }

        /* For Main Zone */
        uint16_t    DispData;

        if (g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum == (uint32_t)DIGITAL_SEG_NUM_14)
        {
            /* The Main Digit Table is an ASCII table beginning with "SPACE" (hex is 0x20) */
            ch -= 0x20L;

            /* ASCII "0" to "z" */
            if ((ch >= 0) && (ch < 91))
            {
                DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[(uint32_t)ch];
            }
            else
            {
                /* Out of definition. Will show "SPACE" */
                DispData = 0U;
            }
        }
        /* For Other Zones (Support '0' ~ '9' only) */
        else if ((ch >= 48L) && (ch <= 57L) && (g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum == (uint32_t)DIGITAL_SEG_NUM_7))
        {
            ch -= 48L;
            DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[(uint32_t)ch];
        }
        /* Out of definition. Will show "SPACE" */
        else
        {
            DispData = 0U;
        }

        for (i = 0UL; i < (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            uint32_t segMapOffset;
            uint32_t com;
            uint32_t seg;

            segMapOffset = (index * (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2U) + (i * 2U);
            com = g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[segMapOffset];
            seg = g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[segMapOffset + 1U];

            if ((DispData & ((uint16_t)1U << i)) != 0U)
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1U);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0U);
            }
        }
    }
}

/**
 *  @brief Display number on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  InputNum    number to show on display
 */
void LCDLIB_PrintNumber(uint32_t u32Zone, uint32_t InputNum)
{
    uint32_t i;
    uint32_t div;

    /* Extract useful digits */
    div = 1U;

    /* Fill out all characters on display */
    uint32_t index;

    index = g_LCDZoneInfo[u32Zone].u8LCDDispTableNum;

    while (index != 0UL)
    {
        index--;

        uint32_t val;

        val = (InputNum / div) % 10U;

        if (g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum == (uint32_t)DIGITAL_SEG_NUM_14)
        {
            val += 16U; /* The Main Digit Table is an ASCII table beginning with "SPACE" */
        }

        uint16_t    DispData;

        DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[val];

        for (i = 0UL; i < (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            uint32_t segMapOffset;
            uint32_t com;
            uint32_t seg;

            segMapOffset = (index * (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2U) + (i * 2U);
            com = g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[segMapOffset];
            seg = g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[segMapOffset + 1U];

            if ((DispData & ((uint16_t)1U << i)) != 0U)
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1U);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0U);
            }
        }

        div = div * 10U;
    }
}

/**
 *  @brief Display character on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  u32Index    the requested display position in zone
 *  @param[in]  u8Ch        Character to show on display
 */
void LCDLIB_PutChar(uint32_t u32Zone, uint32_t u32Index, uint8_t u8Ch)
{
    if (u32Index <= g_LCDZoneInfo[u32Zone].u8LCDDispTableNum)
    {
        uint32_t    ch;
        uint16_t    DispData;
        uint32_t    i;

        /* For Main Zone */
        if (g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum == (uint32_t)DIGITAL_SEG_NUM_14)
        {
            /* Defined letters currently starts at "SPACE" - 0x20; */
            ch       = (uint32_t)u8Ch - 0x20U;
            DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[ch];
        }
        /* For Other Zones (Support '0' ~ '9' only) */
        else if ((u8Ch >= 0x30U) && (u8Ch <= 0x39U) && (g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum == (uint32_t)DIGITAL_SEG_NUM_7))
        {
            ch = (uint32_t)u8Ch - 0x30U;
            DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[ch];
        }
        /* Out of definition. Will show "SPACE" */
        else
        {
            DispData = 0U;
        }

        for (i = 0UL; i < (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            uint32_t segMapOffset;
            uint32_t com;
            uint32_t seg;

            segMapOffset = (u32Index * (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2U) + (i * 2U);
            com = g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[segMapOffset];
            seg = g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[segMapOffset + 1U];

            if ((DispData & ((uint16_t)1U << i)) != 0U)
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1U);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0U);
            }
        }
    }
}

/**
 *  @brief Display symbol on LCD
 *
 *  @param[in]  u32Symbol   the combination of com, seg position
 *  @param[in]  u32OnOff    1: display symbol
 *                          0: not display symbol
 */
void LCDLIB_SetSymbol(uint32_t u32Symbol, uint32_t u32OnOff)
{
    uint32_t com;
    uint32_t seg;

    com = (u32Symbol & 0xFU);
    seg = ((u32Symbol & 0xFF0U) >> 4U);

    if (u32OnOff != 0UL)
    {
        LCD_SetPixel(com, seg, 1U); /* Turn on display */
    }
    else
    {
        LCD_SetPixel(com, seg, 0U); /* Turn off display */
    }
}

/** @} end of group LCDLIB_EXPORTED_FUNCTIONS */
/** @} end of group LCDLIB */
/** @} end of group Library */
