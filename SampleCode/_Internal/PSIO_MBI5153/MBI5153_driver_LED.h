/**************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * $Revision:   1 $
 * $Date:       17/07/19 9:34a $
 * @brief       MBI5153 LED Driver
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __MBI5153_DRIVER_LED_H__
#define __MBI5153_DRIVER_LED_H__

#define LED_PANEL_WIDTH     64
#define LED_PANEL_HEIGHT    32

#define CONFIG_REG1_VAL             0x9F2B/*0xDF2B*/
#define CONFIG_REG2_VAL_FOR_R       0x032B/*0x4601*/
#define CONFIG_REG2_VAL_FOR_GB      0x6600

#define ROW_0_15_PCS    1
#define ROW_16_31_PCS   1
#define LINE_PCS        2
#define RED_PCS         4
#define GREEN_PCS       4
#define BLUE_PCS        4
#define CHANNEL_NUM     16

#define LINE_NUM    1/*(32)*/
#define PCS_TOTAL   (ROW_0_15_PCS+ROW_16_31_PCS+RED_PCS+GREEN_PCS+BLUE_PCS)
#define PIN         (1+1)       /* Le + Data */

#define TIMER_PORT  TIMER0
#define GCLK_ON()   TIMER_Start(TIMER_PORT)
#define GCLK_OFF()  TIMER_Stop(TIMER_PORT)

#define LE_PIN      PSIO_PIN0
#define DATA_PIN    PSIO_PIN1
#define DCLK_PIN    PSIO_PIN2

#define SC_CTRL     PSIO_SC0

#define PDMA_CHANNEL 0
/**********************************/
#define PATTERN_ON_FLASH    0

#define PSIO_DMA_PER_LINE_DATA_CNT  (MBI5153_TOTAL_PCS * MBI5153_CHANNEL_NUM * 2)

#define PER_CHANNEL_DATA_CNT        (MBI5153_TOTAL_PCS * 2)
#define PER_LINE_COLOR_DATA_CNT     (MBI5153_CHANNEL_NUM * 4)

#define MBI5153_RED_PCS         4
#define MBI5153_GREEN_PCS       4
#define MBI5153_BLUE_PCS        4
#define MBI5153_LINE_PCS        2
#define MBI5153_TOTAL_PCS       (MBI5153_RED_PCS + MBI5153_GREEN_PCS    \
                                 + MBI5153_BLUE_PCS + MBI5153_LINE_PCS)

#define MBI5153_CHANNEL_NUM     16
#define MBI5152_SCAN_LINE_NUM   32

#define FIRST_LINE_BUF_INDEX    2
#define SECEND_LINE_BUF_INDEX   3
#define FIRRST_RED_BUF_INDEX    6
#define FIRRST_GREEN_BUF_INDEX  14
#define FIRST_BLUE_BUF_INDEX    22
/**********************************/
typedef struct
{
    uint16_t configReg1;
    uint16_t configReg2;
} MBI5153_REG_t;

typedef struct
{
    MBI5153_REG_t red[RED_PCS];
    MBI5153_REG_t green[GREEN_PCS];
    MBI5153_REG_t blue[BLUE_PCS];
    MBI5153_REG_t line[LINE_PCS];
} MBI5153_t;

typedef enum
{
    TRANSFER_BUSY,
    TRANSFER_DONE,
} TRANSFER_STATUS;

typedef enum
{
    TRANSFER_PREACTIVE,
    TRANSFER_SWRESET,
    TRANSFER_DATA,
    TRANSFER_VERTICALSYNC,
    TRANSFER_CONFIG,
} TRANSFER_TYPE;

enum
{
    RED = 0,
    GREEN,
    BLUE,
};

typedef struct
{
#if (PATTERN_ON_FLASH==1)
    uint32_t *pu32Pattern;
#else
    uint16_t red[LED_PANEL_WIDTH * LED_PANEL_HEIGHT];
    uint16_t green[LED_PANEL_WIDTH * LED_PANEL_HEIGHT];
    uint16_t blue[LED_PANEL_WIDTH * LED_PANEL_HEIGHT];
#endif
} FRAME_BUF_t;

#if (PATTERN_ON_FLASH==1)
    extern const uint32_t GREEN_COLOR[(PSIO_DMA_PER_LINE_DATA_CNT / 2)*MBI5152_SCAN_LINE_NUM];
    extern const uint32_t RED_COLOR[(PSIO_DMA_PER_LINE_DATA_CNT / 2)*MBI5152_SCAN_LINE_NUM];
    extern const uint32_t BLUE_COLOR[(PSIO_DMA_PER_LINE_DATA_CNT / 2)*MBI5152_SCAN_LINE_NUM];
    extern const uint32_t BLANK_COLOR[(PSIO_DMA_PER_LINE_DATA_CNT / 2)*MBI5152_SCAN_LINE_NUM];
#endif

void MBI5153_Open(void);
void MBI5153_Close(void);

#if (PATTERN_ON_FLASH==1)
    void MBI5153_LoadPattern(FRAME_BUF_t sLED_Pattern)
#else
    void MBI5153_LoadPattern(FRAME_BUF_t *sLED_Pattern, uint8_t u8Line);
#endif
void MBI5153_SetGrayScale(void);
TRANSFER_STATUS MBI5153_CheckStatus(void);
void MBI5153_verticalSync(void);

#endif  //__MBI5153_DRIVER_LED_H__
/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
