/**************************************************************************//**
 * @file     lcdzone.h
 * @version  V1.00
 * @brief    RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD zone header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __LCDZONE_H
#define __LCDZONE_H

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Library Library
  @{
*/

/** @addtogroup LCDLIB LCD Library
  @{
*/

/** @addtogroup LCDLIB_EXPORTED_CONSTANTS LCD Zone Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Digit Zone Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define ZONE_MAIN_DIGIT                                     0
#define ZONE_TIME_DIGIT                                     1
#define ZONE_NUMICRO_DIGIT                                  2
#define ZONE_PPM_DIGIT                                      3
#define ZONE_TEMP_DIGIT                                     4
#define ZONE_VER_DIGIT                                      5


/*---------------------------------------------------------------------------------------------------------*/
/*  COM and SEG Position of Symbol Constant Definitions                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define SYMBOL_PLUS_1                                       ((3)<<4 | (1)<<0)
#define SYMBOL_MINUS_2                                      ((3)<<4 | (3)<<0)
#define SYMBOL_S01_10                                       ((11)<<4 | (0)<<0)
#define SYMBOL_S02_10                                       ((12)<<4 | (0)<<0)
#define SYMBOL_S03_10                                       ((13)<<4 | (0)<<0)
#define SYMBOL_S04_10                                       ((14)<<4 | (0)<<0)
#define SYMBOL_S05_10                                       ((15)<<4 | (0)<<0)
#define SYMBOL_S06_10                                       ((11)<<4 | (1)<<0)
#define SYMBOL_S07_10                                       ((12)<<4 | (1)<<0)
#define SYMBOL_S08_10                                       ((13)<<4 | (1)<<0)
#define SYMBOL_S09_10                                       ((14)<<4 | (1)<<0)
#define SYMBOL_S10_10                                       ((15)<<4 | (1)<<0)
#define SYMBOL_S11_10                                       ((11)<<4 | (2)<<0)
#define SYMBOL_S12_10                                       ((12)<<4 | (2)<<0)
#define SYMBOL_S13_10                                       ((13)<<4 | (2)<<0)
#define SYMBOL_S14_10                                       ((14)<<4 | (2)<<0)
#define SYMBOL_S15_10                                       ((15)<<4 | (2)<<0)
#define SYMBOL_S16_10                                       ((11)<<4 | (3)<<0)
#define SYMBOL_S17_10                                       ((12)<<4 | (3)<<0)
#define SYMBOL_S18_10                                       ((13)<<4 | (3)<<0)
#define SYMBOL_S19_10                                       ((14)<<4 | (3)<<0)
#define SYMBOL_S20_10                                       ((15)<<4 | (3)<<0)
#define SYMBOL_S21_10                                       ((11)<<4 | (4)<<0)
#define SYMBOL_S22_10                                       ((12)<<4 | (4)<<0)
#define SYMBOL_S23_10                                       ((13)<<4 | (4)<<0)
#define SYMBOL_S24_10                                       ((14)<<4 | (4)<<0)
#define SYMBOL_S25_10                                       ((15)<<4 | (4)<<0)
#define SYMBOL_S26_10                                       ((11)<<4 | (5)<<0)
#define SYMBOL_S27_10                                       ((12)<<4 | (5)<<0)
#define SYMBOL_S28_10                                       ((13)<<4 | (5)<<0)
#define SYMBOL_S29_10                                       ((14)<<4 | (5)<<0)
#define SYMBOL_S30_10                                       ((15)<<4 | (5)<<0)
#define SYMBOL_S31_10                                       ((11)<<4 | (6)<<0)
#define SYMBOL_S32_10                                       ((12)<<4 | (6)<<0)
#define SYMBOL_S33_10                                       ((13)<<4 | (6)<<0)
#define SYMBOL_S34_10                                       ((14)<<4 | (6)<<0)
#define SYMBOL_S35_10                                       ((15)<<4 | (6)<<0)
#define SYMBOL_S36_10                                       ((11)<<4 | (7)<<0)
#define SYMBOL_S37_10                                       ((12)<<4 | (7)<<0)
#define SYMBOL_S38_10                                       ((13)<<4 | (7)<<0)
#define SYMBOL_S39_10                                       ((14)<<4 | (7)<<0)
#define SYMBOL_S40_10                                       ((15)<<4 | (7)<<0)
#define SYMBOL_V_11                                         ((39)<<4 | (4)<<0)
#define SYMBOL_A_12                                         ((37)<<4 | (4)<<0)
#define SYMBOL_W_13                                         ((35)<<4 | (4)<<0)
#define SYMBOL_NVT_14                                       ((10)<<4 | (4)<<0)
#define SYMBOL_NUMICRO_15                                   ((9)<<4 | (3)<<0)
#define SYMBOL_WIFI_16                                      ((10)<<4 | (5)<<0)
#define SYMBOL_SOUND_17                                     ((10)<<4 | (6)<<0)
#define SYMBOL_BAT_FRAME_18                                 ((10)<<4 | (0)<<0)
#define SYMBOL_BAT_1_18                                     ((10)<<4 | (2)<<0)
#define SYMBOL_BAT_2_18                                     ((10)<<4 | (3)<<0)
#define SYMBOL_BAT_3_18                                     ((10)<<4 | (1)<<0)
#define SYMBOL_ARROW_UP_26                                  ((1)<<4 | (4)<<0)
#define SYMBOL_ARROW_LEFT_26                                ((1)<<4 | (6)<<0)
#define SYMBOL_ARROW_DOWN_26                                ((0)<<4 | (7)<<0)
#define SYMBOL_ARROW_RIGHT_26                               ((0)<<4 | (4)<<0)
#define SYMBOL_CIRCLE_UP_26                                 ((1)<<4 | (5)<<0)
#define SYMBOL_CIRCLE_LEFT_26                               ((0)<<4 | (6)<<0)
#define SYMBOL_CIRCLE_RIGHT_26                              ((0)<<4 | (5)<<0)
#define SYMBOL_PERCENTAGE_30                                ((23)<<4 | (4)<<0)
#define SYMBOL_PPM_31                                       ((21)<<4 | (4)<<0)
#define SYMBOL_TEMP_C_35                                    ((25)<<4 | (4)<<0)
#define SYMBOL_TEMP_F_36                                    ((27)<<4 | (4)<<0)
#define SYMBOL_VERSION_37                                   ((31)<<4 | (4)<<0)
#define SYMBOL_P_44                                         ((3)<<4 | (7)<<0)
#define SYMBOL_P_45                                         ((5)<<4 | (3)<<0)
#define SYMBOL_P_46                                         ((7)<<4 | (3)<<0)
#define SYMBOL_COL_47                                       ((5)<<4 | (7)<<0)
#define SYMBOL_P_49                                         ((16)<<4 | (3)<<0)
#define SYMBOL_P_50                                         ((20)<<4 | (3)<<0)
#define SYMBOL_P_51                                         ((24)<<4 | (3)<<0)
#define SYMBOL_P_52                                         ((28)<<4 | (3)<<0)
#define SYMBOL_P_53                                         ((32)<<4 | (3)<<0)
#define SYMBOL_P_54                                         ((36)<<4 | (3)<<0)
#define SYMBOL_COL_55                                       ((16)<<4 | (1)<<0)
#define SYMBOL_COL_57                                       ((20)<<4 | (1)<<0)
#define SYMBOL_COL_59                                       ((24)<<4 | (1)<<0)
#define SYMBOL_COL_61                                       ((28)<<4 | (1)<<0)
#define SYMBOL_COL_63                                       ((32)<<4 | (1)<<0)
#define SYMBOL_COL_65                                       ((36)<<4 | (1)<<0)
#define SYMBOL_P_67                                         ((29)<<4 | (4)<<0)
#define SYMBOL_P_68                                         ((33)<<4 | (4)<<0)


/** @} end of group LCDLIB_EXPORTED_CONSTANTS */



/** @addtogroup LCDLIB_EXPORTED_STRUCTS LCD Zone Exported Structs
  @{
 */
typedef struct
{
    unsigned char   u8LCDDispTableNum;          /*!< LCD Display Table Number */
    unsigned char   u8GetLCDComSegNum;          /*!< LCD Com Seg Table Number */
    const unsigned short  *pu16LCDDispTable;    /*!< LCD Display Table Pointer */
    const unsigned char   *pu8GetLCDComSeg;     /*!< LCD Com Seg Table Pointer */

} LCD_ZONE_INFO_T;

/** @} end of group LCDLIB_EXPORTED_STRUCTS */


/** @addtogroup LCDLIB_EXPORTED_CONSTANTS LCD Zone Exported Constants
  @{
*/

#define ZONE_MAIN_DIG_CNT                                       7
#define ZONE_MAIN_SEG_NUM                                       14

#define ZONE_TIME_DIG_CNT                                       4
#define ZONE_TIME_SEG_NUM                                       7

#define ZONE_NUMICRO_DIG_CNT                                    3
#define ZONE_NUMICRO_SEG_NUM                                    7

#define ZONE_PPM_DIG_CNT                                        3
#define ZONE_PPM_SEG_NUM                                        7

#define ZONE_TEMP_DIG_CNT                                       3
#define ZONE_TEMP_SEG_NUM                                       7

#define ZONE_VER_DIG_CNT                                        6
#define ZONE_VER_SEG_NUM                                        7



/** @} end of group LCDLIB_EXPORTED_CONSTANTS */


/** @addtogroup LCDLIB_EXPORTED_STRUCTS LCD Zone Exported Structs
  @{
 */

/**************************************************************************//**
*
* Defines each text's segment (alphabet+numeric) in terms of COM and SEG numbers,
* Using this way that text segment can be consisted of each bit in the
* following bit pattern:
*
*              A
*         -----------
*         |\   |   /|
*         F G  H  I B
*         |  \ | /  |
*         --J-- --K--
*         |   /| \  |
*         E  L M  N C
*         | /  |   \|
*         -----------
*              D
*
*              0
*         -----------
*         |\   |   /|
*        5| 6  7  8 |1
*         |  \ | /  |
*         --9-- -10--
*         |   /| \  |
*        4| 11 12 13|2
*         | /  |   \|
*         -----------
*              3
*
*****************************************************************************/

static const unsigned char acMAINDigitRawData[ZONE_MAIN_DIG_CNT][ZONE_MAIN_SEG_NUM][2] =
{
    {
        {0U, 1U},       {0U, 0U},       {3U, 0U},       {3U, 1U},       {2U, 3U},       {0U, 3U},       {0U, 2U},       {1U, 1U},       {1U, 0U},       {1U, 2U},       {2U, 0U},       {3U, 2U},       {2U, 2U},       {2U, 1U},
    },
    {
        {0U, 18U},      {0U, 19U},      {3U, 19U},      {3U, 18U},      {2U, 16U},      {0U, 16U},      {0U, 17U},      {1U, 18U},      {1U, 19U},      {1U, 17U},      {2U, 19U},      {3U, 17U},      {2U, 17U},      {2U, 18U},
    },
    {
        {0U, 22U},      {0U, 23U},      {3U, 23U},      {3U, 22U},      {2U, 20U},      {0U, 20U},      {0U, 21U},      {1U, 22U},      {1U, 23U},      {1U, 21U},      {2U, 23U},      {3U, 21U},      {2U, 21U},      {2U, 22U},
    },
    {
        {0U, 26U},      {0U, 27U},      {3U, 27U},      {3U, 26U},      {2U, 24U},      {0U, 24U},      {0U, 25U},      {1U, 26U},      {1U, 27U},      {1U, 25U},      {2U, 27U},      {3U, 25U},      {2U, 25U},      {2U, 26U},
    },
    {
        {0U, 30U},      {0U, 31U},      {3U, 31U},      {3U, 30U},      {2U, 28U},      {0U, 28U},      {0U, 29U},      {1U, 30U},      {1U, 31U},      {1U, 29U},      {2U, 31U},      {3U, 29U},      {2U, 29U},      {2U, 30U},
    },
    {
        {0U, 34U},      {0U, 35U},      {3U, 35U},      {3U, 34U},      {2U, 32U},      {0U, 32U},      {0U, 33U},      {1U, 34U},      {1U, 35U},      {1U, 33U},      {2U, 35U},      {3U, 33U},      {2U, 33U},      {2U, 34U},
    },
    {
        {0U, 38U},      {0U, 39U},      {3U, 39U},      {3U, 38U},      {2U, 36U},      {0U, 36U},      {0U, 37U},      {1U, 38U},      {1U, 39U},      {1U, 37U},      {2U, 39U},      {3U, 37U},      {2U, 37U},      {2U, 38U},
    },
};

/**************************************************************************//**
*
* Defines each text's segment (numeric) in terms of COM and BIT numbers,
* Using this way that text segment can be consisted of each bit in the
* following bit pattern:
*
*         ---A---
*         |     |
*         F     B
*         |     |
*         ---G---
*         |     |
*         E     C
*         |     |
*         ---D---
*
*         ---0---
*         |     |
*         5     1
*         |     |
*         ---6---
*         |     |
*         4     2
*         |     |
*         ---3---
*
*****************************************************************************/

static const unsigned char acTIMEDigitRawData[ZONE_TIME_DIG_CNT][ZONE_TIME_SEG_NUM][2] =
{
    {
        {7U, 2U},       {6U, 3U},       {4U, 3U},       {4U, 2U},       {5U, 2U},       {6U, 2U},       {5U, 3U},
    },
    {
        {7U, 4U},       {6U, 5U},       {4U, 5U},       {4U, 4U},       {5U, 4U},       {6U, 4U},       {5U, 5U},
    },
    {
        {7U, 6U},       {6U, 7U},       {4U, 7U},       {4U, 6U},       {5U, 6U},       {6U, 6U},       {5U, 7U},
    },
    {
        {7U, 8U},       {6U, 9U},       {4U, 9U},       {4U, 8U},       {5U, 8U},       {6U, 8U},       {5U, 9U},
    },
};
static const unsigned char acNUMICRODigitRawData[ZONE_NUMICRO_DIG_CNT][ZONE_NUMICRO_SEG_NUM][2] =
{
    {
        {3U, 4U},       {2U, 5U},       {0U, 5U},       {0U, 4U},       {1U, 4U},       {2U, 4U},       {1U, 5U},
    },
    {
        {3U, 6U},       {2U, 7U},       {0U, 7U},       {0U, 6U},       {1U, 6U},       {2U, 6U},       {1U, 7U},
    },
    {
        {3U, 8U},       {2U, 9U},       {0U, 9U},       {0U, 8U},       {1U, 8U},       {2U, 8U},       {1U, 9U},
    },
};
static const unsigned char acPPMDigitRawData[ZONE_PPM_DIG_CNT][ZONE_PPM_SEG_NUM][2] =
{
    {
        {4U, 16U},      {5U, 17U},      {7U, 17U},      {7U, 16U},      {6U, 16U},      {5U, 16U},      {6U, 17U},
    },
    {
        {4U, 18U},      {5U, 19U},      {7U, 19U},      {7U, 18U},      {6U, 18U},      {5U, 18U},      {6U, 19U},
    },
    {
        {4U, 20U},      {5U, 21U},      {7U, 21U},      {7U, 20U},      {6U, 20U},      {5U, 20U},      {6U, 21U},
    },
};
static const unsigned char acTEMPDigitRawData[ZONE_TEMP_DIG_CNT][ZONE_TEMP_SEG_NUM][2] =
{
    {
        {4U, 22U},      {5U, 23U},      {7U, 23U},      {7U, 22U},      {6U, 22U},      {5U, 22U},      {6U, 23U},
    },
    {
        {4U, 24U},      {5U, 25U},      {7U, 25U},      {7U, 24U},      {6U, 24U},      {5U, 24U},      {6U, 25U},
    },
    {
        {4U, 26U},      {5U, 27U},      {7U, 27U},      {7U, 26U},      {6U, 26U},      {5U, 26U},      {6U, 27U},
    },
};
static const unsigned char acVERDigitRawData[ZONE_VER_DIG_CNT][ZONE_VER_SEG_NUM][2] =
{
    {
        {4U, 28U},      {5U, 29U},      {7U, 29U},      {7U, 28U},      {6U, 28U},      {5U, 28U},      {6U, 29U},
    },
    {
        {4U, 30U},      {5U, 31U},      {7U, 31U},      {7U, 30U},      {6U, 30U},      {5U, 30U},      {6U, 31U},
    },
    {
        {4U, 32U},      {5U, 33U},      {7U, 33U},      {7U, 32U},      {6U, 32U},      {5U, 32U},      {6U, 33U},
    },
    {
        {4U, 34U},      {5U, 35U},      {7U, 35U},      {7U, 34U},      {6U, 34U},      {5U, 34U},      {6U, 35U},
    },
    {
        {4U, 36U},      {5U, 37U},      {7U, 37U},      {7U, 36U},      {6U, 36U},      {5U, 36U},      {6U, 37U},
    },
    {
        {4U, 38U},      {5U, 39U},      {7U, 39U},      {7U, 38U},      {6U, 38U},      {5U, 38U},      {6U, 39U},
    },
};

/**************************************************************************//**
*
* Defines segments for the alphabet - ASCII table 0x20 to 0x7A
* Bit pattern below defined for alphabet (text segments)
*
*****************************************************************************/

static const unsigned short auMAINDigitMap[] =
{
    0x0000, /* space */
    0x1100, /* ! */
    0x0280, /* " */
    0x0000, /* # */
    0x0000, /* $ */
    0x0000, /* % */
    0x0000, /* & */
    0x0000, /* ? */
    0x0039, /* ( */
    0x000f, /* ) */
    0x3fc0, /* * */
    0x1540, /* + */
    0x0000, /* , */
    0x0440, /* - */
    0x8000, /* . */
    0x2200, /* / */

    0x003F, /* 0 */
    0x0006, /* 1 */
    0x061B, /* 2 */
    0x060F, /* 3 */
    0x0626, /* 4 */
    0x062D, /* 5 */
    0x063D, /* 6 */
    0x0007, /* 7 */
    0x063F, /* 8 */
    0x062F, /* 9 */

    0x0000, /* : */
    0x0000, /* ; */
    0x2100, /* < */
    0x0000, /* = */
    0x0840, /* > */
    0x1403, /* ? */
    0x3FFF, /* @ */

    0x0637, /* A */
    0x2339, /* B */
    0x0039, /* C */
    0x2139, /* D */
    0x0639, /* E */
    0x0631, /* F */
    0x043D, /* G */
    0x0636, /* H */
    0x1080, /* I */
    0x000E, /* J */
    0x2330, /* K */
    0x0038, /* L */
    0x0176, /* M */
    0x2076, /* N */
    0x003F, /* O */
    0x0633, /* P */
    0x203F, /* Q */
    0x2331, /* R */
    0x062D, /* S */
    0x1081, /* T */
    0x003E, /* U */
    0x0930, /* V */
    0x2836, /* W */
    0x2940, /* X */
    0x1140, /* Y */
    0x0909, /* Z */

    0x0039, /* [ */
    0x0900, /* backslash */
    0x000F, /* ] */
    0x2800, /* ^ */
    0x0008, /* _ */
    0x0040, /* ` */

    0x1218, /* a */
    0x063C, /* b */
    0x0618, /* c */
    0x061E, /* d */
    0x0A18, /* e */
    0x0231, /* f */
    0x048F, /* g */
    0x1230, /* h */
    0x1000, /* i */
    0x000E, /* j */
    0x2330, /* k */
    0x0038, /* l */
    0x1614, /* m */
    0x1404, /* n */
    0x061C, /* o */
    0x0331, /* p */
    0x0447, /* q */
    0x1400, /* r */
    0x2408, /* s */
    0x0238, /* t */
    0x1018, /* u */
    0x0810, /* v */
    0x2814, /* w */
    0x2940, /* x */
    0x0446, /* y */
    0x0A08, /* z */

    0x0000,
};

/**************************************************************************//**
* Defines segments for the numeric display
*****************************************************************************/

static const unsigned short auTIMEDigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};
static const unsigned short auNUMICRODigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};
static const unsigned short auPPMDigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};
static const unsigned short auTEMPDigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};
static const unsigned short auVERDigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};

static const LCD_ZONE_INFO_T g_LCDZoneInfo[] =
{
    {ZONE_MAIN_DIG_CNT,         ZONE_MAIN_SEG_NUM,      auMAINDigitMap, (const unsigned char *)acMAINDigitRawData},
    {ZONE_TIME_DIG_CNT,         ZONE_TIME_SEG_NUM,      auTIMEDigitMap, (const unsigned char *)acTIMEDigitRawData},
    {ZONE_NUMICRO_DIG_CNT,      ZONE_NUMICRO_SEG_NUM,   auNUMICRODigitMap, (const unsigned char *)acNUMICRODigitRawData},
    {ZONE_PPM_DIG_CNT,          ZONE_PPM_SEG_NUM,       auPPMDigitMap, (const unsigned char *)acPPMDigitRawData},
    {ZONE_TEMP_DIG_CNT,         ZONE_TEMP_SEG_NUM,      auTEMPDigitMap, (const unsigned char *)acTEMPDigitRawData},
    {ZONE_VER_DIG_CNT,          ZONE_VER_SEG_NUM,       auVERDigitMap, (const unsigned char *)acVERDigitRawData},

};

/** @} end of group LCDLIB_EXPORTED_STRUCTS */
/** @} end of group LCDLIB */
/** @} end of group Library */

#ifdef __cplusplus
}

#endif

#endif  /* __LCDZONE_H */
