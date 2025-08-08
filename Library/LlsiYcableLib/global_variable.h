/******************************************************************************
 * @file     global_variable.h
 * @version  V3.00
 * @brief    Global Variable header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __GLOBAL_VARIABLE_H__
#define __GLOBAL_VARIABLE_H__

#include <stdint.h>
#include "NuMicro.h"

#define HCLK_CLK 180000000

enum eLEDType
{
    Type_GRB = 0,
    Type_RGB = 1,
};

enum eDirection
{
    Dir_Forward = 0,
    Dir_Backward = 1,
};

enum eMusicAction
{
    Music_POP = 0x01,
    Music_JAZZ = 0x02,
    Music_Mixed = 0x03,
};

enum eColorTable
{
    eColorRed = 0,
    eColorOrange,
    eColorYellow,
    eColorGreen,
    eColorCyan,
    eColorBlue,
    eColorIndigo,
    eColorPurple
};

extern uint32_t TimeCounterFrameUpdate;
extern uint8_t LED_Frame_Update_flag;
#define TIMER_FRAME_UPDATE 25

struct LED_Setting_Tag;
typedef void (*LED_FUNC)(volatile struct LED_Setting_Tag* LED_Setting);
#define LED_MODE_COUNT 11
extern void * const Mode_Function[LED_MODE_COUNT];

typedef struct LED_Setting_Tag
{
    uint32_t  TimeCounter;    // 0
    uint8_t   AP_Sync;        // 0
    uint8_t   LEDNum;         // 100
    uint8_t   LightingMode;   // 1
    uint8_t   Color_R;        // 255
    uint8_t   Color_G;        // 0
    uint8_t   Color_B;        // 0
    uint8_t   Brightness;     // 0xFF
    uint8_t   Speed;          // 0
    uint8_t   Direction;      // Dir_Forward
    uint8_t   LED_Type;       // Type_GRB
    uint8_t   fPDMA_Done;     // 1
    uint8_t   RandomFlag;     // 1
    uint32_t  Random;         // 0
    LED_FUNC  Mode_FUNC;      // FUNC_Static
    uint8_t*  LED_Data;
    uint8_t   Main_Volume;    // 0
    uint8_t   Left_Volume;    // 0
    uint8_t   Right_Volume;   // 0
    uint8_t   Music_Action;   // Music_POP
    uint32_t  Array_Size;
    uint8_t   LLSI_Num;
    uint8_t   ColorIndex;
    uint16_t  LED_Offset;
} LED_Setting_T;

#define TOTAL_LED_AREA 10
/* LED Gen2 amount setting */
#define LED_GEN2_MAX_SUPPORT_PORT   TOTAL_LED_AREA
#define LED_GEN2_MAX_STRIP_COUNT    5
#define LED_GEN2_DEFAULT_LED_NUMBER 30
#define LED_GEN2_MAX_LED_NUMBER     LED_GEN2_MAX_STRIP_COUNT*LED_GEN2_DEFAULT_LED_NUMBER
/* Static Y cable mode data */
#define PIX_Y_CABLE_RESET    25    // 250 us
#define PIX_Y_CABLE_H        4     // 40  us
#define PIX_Y_CABLE_L        1     // 10  us
#define PIX_Y_CABLE          (PIX_Y_CABLE_RESET + PIX_Y_CABLE_H + PIX_Y_CABLE_L)
#define PIX_COMMAND_ID       1     //
#define PIX_CONTROL_CMD      (PIX_Y_CABLE + PIX_COMMAND_ID)
#define BYTE_Y_CABLE         (PIX_Y_CABLE * 3)
#define BYTE_COMMAND_ID      (PIX_COMMAND_ID * 3)
#define BYTE_CONTROL_CMD     (PIX_CONTROL_CMD * 3)

#define cStripA_LED 144
#if((cStripA_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_STRIPA_LEN (cStripA_LED*3)
#else
    #define LED_STRIPA_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t StripALEDData[LED_STRIPA_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T StripA_LEDSetting;

#define cStripB_LED 144
#if((cStripB_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_STRIPB_LEN (cStripB_LED*3)
#else
    #define LED_STRIPB_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t StripBLEDData[LED_STRIPB_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T StripB_LEDSetting;

#define cFAN1_LED   54
#if((cFAN1_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_FAN1LED_LEN (cFAN1_LED*3)
#else
    #define LED_FAN1LED_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t FAN1LEDData[LED_FAN1LED_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T FAN1_LEDSetting;

#define cFAN2_LED   54
#if((cFAN2_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_FAN2LED_LEN (cFAN2_LED*3)
#else
    #define LED_FAN2LED_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t FAN2LEDData[LED_FAN2LED_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T FAN2_LEDSetting;

#define cFAN3_LED   54
#if((cFAN3_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_FAN3LED_LEN (cFAN3_LED*3)
#else
    #define LED_FAN3LED_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t FAN3LEDData[LED_FAN3LED_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T FAN3_LEDSetting;

#define cFAN4_LED   54
#if((cFAN4_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_FAN4LED_LEN (cFAN4_LED*3)
#else
    #define LED_FAN4LED_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t FAN4LEDData[LED_FAN4LED_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T FAN4_LEDSetting;

#define cFAN5_LED   54
#if((cFAN5_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_FAN5LED_LEN (cFAN5_LED*3)
#else
    #define LED_FAN5LED_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t FAN5LEDData[LED_FAN5LED_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T FAN5_LEDSetting;

#define cFAN6_LED   54
#if((cFAN6_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_FAN6LED_LEN (cFAN6_LED*3)
#else
    #define LED_FAN6LED_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t FAN6LEDData[LED_FAN6LED_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T FAN6_LEDSetting;

#define cFAN7_LED   54
#if((cFAN7_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_FAN7LED_LEN (cFAN7_LED*3)
#else
    #define LED_FAN7LED_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t FAN7LEDData[LED_FAN7LED_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T FAN7_LEDSetting;

#define cFAN8_LED   54
#if((cFAN8_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_FAN8LED_LEN (cFAN8_LED*3)
#else
    #define LED_FAN8LED_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
extern __attribute__((aligned (4))) uint8_t FAN8LEDData[LED_FAN8LED_LEN];
extern __attribute__((aligned (4))) volatile LED_Setting_T FAN8_LEDSetting;

extern uint8_t *LED_Data_Array_Mapping[TOTAL_LED_AREA];
extern const uint32_t LED_Data_Array_Size[TOTAL_LED_AREA];
extern volatile LLSI_T *LLSI_Port_Mapping[TOTAL_LED_AREA];

/* LLSI port GPIO setting table */
typedef struct
{
    GPIO_T * IO_Port;
    uint32_t IO_Number;
    uint32_t IO_Bit;
    uint32_t IO_IRQn;
    uint32_t IO_MFP_OFFSET;
    uint32_t IO_PDIO_OFFSET;
    uint32_t IO_GPIO_MSK;
    uint32_t IO_LLSI_MSK;
} LED_LLSI_IO_SETTING;

extern const LED_LLSI_IO_SETTING LED_LLSI_IO_Setting[TOTAL_LED_AREA];
extern volatile LED_Setting_T *LED_Mapping[TOTAL_LED_AREA];
extern volatile LED_Setting_T *PDMA_Mapping[TOTAL_LED_AREA];

extern void PDMA_SetREQSEL(PDMA_T * pdma, uint32_t u32Ch);

/* The all Gen2 strip setting of all port */
typedef struct
{
    uint32_t Fixed_ID;
    uint32_t LED_Number;
    LED_Setting_T LEDSetting;
    uint8_t Sequence;
} LED_Gen2_Strip_T;

typedef struct
{
    uint8_t Control_Flag;
    uint8_t Control_Status;
    uint8_t Control_ID;
    uint8_t Strip_Count;
    uint8_t Use_Gen2;
    uint8_t ClearData_Flag;
    uint8_t LLSI_INT_Count;
    LED_Gen2_Strip_T LED_Gen2_Setting[LED_GEN2_MAX_STRIP_COUNT+1];
} LED_Gen2_Setting_T;

extern __attribute__((aligned (4))) volatile LED_Gen2_Setting_T LED_Gen2_Port_Setting[LED_GEN2_MAX_SUPPORT_PORT];

/* Interrupt priority level */
#define INT_PRIORITY_HIGHEST    0
#define INT_PRIORITY_HIGH       1
#define INT_PRIORITY_NORMAL     2
#define INT_PRIORITY_LOW        3

#endif  /* __GLOBAL_VARIABLE_H__ */
