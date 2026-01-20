/******************************************************************************
 * @file     led_gen2_control.h
 * @version  V3.00
 * @brief    Control AP6112Y by using LLSI interface header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __LED_GEN2_CONTROL_H__
#define __LED_GEN2_CONTROL_H__

#include <stdint.h>
#include "global_variable.h"

/* LED Gen2 fixed ID '1' return length */
#define LED_GEN2_FIXED_ID_CODE_LENGTH    30

/* LED Gen2 capture source */
#define LED_GEN2_CAPTURE_TIMER      0
    #define CAPTURE_TIMER           TIMER0
    #define CAPTURE_TIMER_MODULE    TMR0_MODULE
    #define CAPTURE_TIMER_CLK_SRC   CLK_CLKSEL1_TMR0SEL_HIRC
    #define CAPTURE_TIMER_PRESCALER (__HIRC / 1000000)
    #define CAPTURE_TIMER_IRQN      TMR0_IRQn
    #define CAPTURE_TIMER_HANDLER   TMR0_IRQHandler
#define LED_GEN2_CAPTURE_BPWM       1
    #define CAPTURE_BPWM            BPWM0
    #define CAPTURE_BPWM_MODULE     BPWM0_MODULE
    #define CAPTURE_BPWM_PRESCALER  (CLK_GetPCLK0Freq() / 1000000)
    #define CAPTURE_BPWM_IRQN       BPWM0_IRQn
    #define CAPTURE_BPWM_HANDLER    BPWM0_IRQHandler
#define LED_GEN2_CAPTURE_SOURCE     LED_GEN2_CAPTURE_BPWM

/* Port supports Gen2 ARGB LED */
enum eGen2TargetPort
{
    Gen2_STRIPA  = 0,
    Gen2_STRIPB  = 1,
    Gen2_FAN1LED = 2,
    Gen2_FAN2LED = 3,
    Gen2_FAN3LED = 4,
    Gen2_FAN4LED = 5,
    Gen2_FAN5LED = 6,
    Gen2_FAN6LED = 7,
    Gen2_FAN7LED = 8,
    Gen2_FAN8LED = 9,
};

/* The setting of current Gen2 control port */
typedef struct
{
    volatile uint8_t* Control_Data;
    volatile uint8_t* fPDMA_Done;
    volatile uint8_t  Current_Port;
    volatile uint8_t  PDMA_Number;
} LED_Gen2_Ctrl_T;
extern __attribute__((aligned (4))) volatile LED_Gen2_Ctrl_T Gen2_Ctrl;

extern void * const Gen2Mode_Function[LED_MODE_COUNT];

/* The store structure of LED Gen2 setting */
typedef struct
{
    uint16_t LEDNum;
    uint8_t  LED_Type;
    uint8_t  Direction;
} LED_Gen2_FLASH1_T;
typedef struct
{
    uint8_t  AP_Sync;
    uint8_t  LightingMode;
    uint8_t  Color_R;
    uint8_t  Color_G;
} LED_Gen2_FLASH2_T;
typedef struct
{
    uint8_t  Color_B;
    uint8_t  Brightness;
    uint8_t  Speed;
    uint8_t  Reserved;
} LED_Gen2_FLASH3_T;
typedef struct
{
    uint32_t Fixed_ID;
} LED_Gen2_FLASH4_T;

#define LED_GEN2_LEN_SETTING    16    // FLASH1_T, FLASH2_T, FLASH3_T, Fixed_ID
#define LED_GEN2_OFFSET_SETTING (LED_GEN2_LEN_SETTING * LED_GEN2_MAX_STRIP_COUNT)
#define LED_GEN2_OFFSET_ENABLE  (LED_GEN2_OFFSET_SETTING * LED_GEN2_MAX_SUPPORT_PORT)

#define Packet_Length_Gen2      (LED_GEN2_LEN_SETTING * 15 + 2)

/* LED Gen2 detect error code */
#define DETECT_ERR_CODE_NOERR         0x00
#define DETECT_ERR_CODE_NUMOVERMAX    0x01
#define DETECT_ERR_CODE_STRIPOVERMAX  0x02

void Init_Gen2_LED_Capture(void);
void LED_Gen2_Set_Ctrl_Setting(uint8_t target_port);
void LED_Gen2_Transfer_Data(uint8_t target_port);
void LED_Gen2_Check_ACK(uint8_t one_ACK, uint32_t timeout_us);
uint8_t LED_Gen2_Wait_ACK_Done(void);
void LED_Gen2_Get_LED_Number(uint8_t id);
void LED_Gen2_Set_SEL_CUR(uint8_t id, uint8_t sel_cur);
void LED_Gen2_Get_Feedback(uint8_t id);
uint32_t LED_Gen2_Merge_Fixed_ID(uint8_t id);
void LED_Gen2_Control_Port(void);
void LED_Gen2_LLSI_PDMA_Init(uint8_t target_port);
void LED_Gen2_Initial_Setting(uint8_t target_port);
void LED_Gen2_Default_Setting(uint8_t target_port, uint8_t id);
void LED_Effect_Setting(uint8_t LightingMode, uint8_t Color_R, uint8_t Color_G, uint8_t Color_B, uint8_t Brightness);
void LED_Gen2_ReadStoredSetting(uint8_t target_port, uint8_t id);
void LED_Gen2_Set_Setting(uint8_t target_port, uint8_t *setting);
void LED_Gen2_Control(uint8_t target_port);
void LED_Gen2_Disable_Mode(uint8_t target_port);
void LED_Gen2_Enable_Control(uint8_t target_port, uint8_t enable);
void LED_Gen2_ACK(void);

#define LED_GEN2_PDMA_DESC_NUM    8    // Total dexcriptior table for single strip conctrol
extern volatile DSCT_T* LED_Gen2_PDMA_DESC[LED_GEN2_MAX_SUPPORT_PORT];

#endif  /* __LED_GEN2_CONTROL_H__ */
