/******************************************************************************//**
 * @file     led_gen2_control.c
 * @version  V3.00
 * @brief    Control AP6112Y by using LLSI interface sample file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdlib.h>
#include "NuMicro.h"
#include "led_control.h"
#include "led_gen2_control.h"

volatile uint8_t u8SettingFlag = 0;

void * const Gen2Mode_Function[LED_MODE_COUNT] = {FUNC_Off, FUNC_Static, FUNC_Breathing, FUNC_Strobe, FUNC_Cycling,
                                                  FUNC_Random, FUNC_Music, FUNC_Wave, FUNC_Spring, FUNC_Water,
                                                  FUNC_Rainbow};

/* Initial User LED Setting */
#define cUser_LED 144
#if((cUser_LED*3) > (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3))
    #define LED_USER_LEN (cUser_LED*3)
#else
    #define LED_USER_LEN (LED_GEN2_MAX_LED_NUMBER*3 + LED_GEN2_MAX_STRIP_COUNT*3)
#endif
__attribute__((aligned (4))) uint8_t UserLEDData[LED_USER_LEN];
__attribute__((aligned (4))) volatile LED_Setting_T User_LEDSetting = {0, 0, 100, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                       1, 1, 0, FUNC_Static, UserLEDData, 0, 0, 0, Music_POP, cUser_LED*3, 0, eColorRed, 0};

/* The all Gen2 strip setting of all port */
__attribute__((aligned (4))) volatile LED_Gen2_Setting_T LED_Gen2_Port_Setting[LED_GEN2_MAX_SUPPORT_PORT];
/* The setting of current Gen2 control port */
__attribute__((aligned (4))) volatile LED_Gen2_Ctrl_T Gen2_Ctrl;
/* Data */
/* Descriptor table */
#define LED_Gen2_PDMA_DESC_NUM    11    // Total dexcriptior table for single strip conctrol
/* 0: Set Setup_LLSI_PERIOD_DUTY to LLSI_PERIOD and LLSI_DUTY
   1: Set LLSI_PCNT
   2: Transfer Y_Cable data (Burst 3 words)
   3: Transfer Y_Cable data (Single 20 words)
   4: Set LLSI_PCNT
   5: Transfer command + ID data (Burst 3 words)
   6: Transfer command + ID data (Single 1 word)
   7: Set Data_LLSI_PERIOD_DUTY to LLSI_PERIOD and LLSI_DUTY
   8: Set LLSI_PCNT
   9: Transfer LED data (Burst 3 words)
   10: Transfer LED data (Single mode)
*/
volatile DSCT_T StripA_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T StripB_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T Fan1LED_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T Fan2LED_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T Fan3LED_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T Fan4LED_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T Fan5LED_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T Fan6LED_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T Fan7LED_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T Fan8LED_PDMA_DESC[LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM];
volatile DSCT_T* LED_Gen2_PDMA_DESC[LED_GEN2_MAX_SUPPORT_PORT] = 
{StripA_PDMA_DESC, StripB_PDMA_DESC,
 Fan1LED_PDMA_DESC, Fan2LED_PDMA_DESC,
 Fan3LED_PDMA_DESC, Fan4LED_PDMA_DESC,
 Fan5LED_PDMA_DESC, Fan6LED_PDMA_DESC,
 Fan7LED_PDMA_DESC, Fan8LED_PDMA_DESC,
};

/* LED strip detect */
volatile uint8_t *LED_Gen2_ACK_Width;
volatile uint32_t LED_Gen2_ACK_Count = 0;
volatile uint8_t LED_Gen2_ACK_Done_Flag = 0;
volatile uint8_t LED_Gen2_One_ACK_Flag = 0;
volatile uint32_t Total_LED_Gen2_Num;

/* LED setting */
#define LED_Gen2_Setting_Default 0xC00000
#define BLUE_DIMMING_Pos   0
#define RED_DIMMING_Pos    8
#define SEL_CUR_Pos        13
#define SEL_LED_CUR        0x0
#define SEL_LED_ID         0x1
#define REFRESH_Pos        15
#define GREEN_DIMMING_Pos  16
#define PWM_FREQ_Pos       22
#define PWM_FREQ_1250      0x0
#define PWM_FREQ_2500      0x1
#define PWM_FREQ_5000      0x2
#define PWM_FREQ_10000     0x3

/* Static Y cable mode data */
#define PIX_Y_CABLE_RESET    25    // 250 us
#define PIX_Y_CABLE_H        4     // 40  us
#define PIX_Y_CABLE_L        1     // 10  us
#define PIX_Y_CABLE          (PIX_Y_CABLE_RESET + PIX_Y_CABLE_H + PIX_Y_CABLE_L)
#define PIX_COMMAND_ID       1     //
#define PIX_CONTROL_CMD      (PIX_Y_CABLE + PIX_COMMAND_ID)
const uint32_t Pix_Y_Cable      = PIX_Y_CABLE;
const uint32_t Pix_Command_ID   = PIX_COMMAND_ID;
const uint32_t Pix_Control_Cmd  = PIX_CONTROL_CMD;
#define BYTE_Y_CABLE         (PIX_Y_CABLE * 3)
#define BYTE_COMMAND_ID      (PIX_COMMAND_ID * 3)
#define BYTE_CONTROL_CMD     (PIX_CONTROL_CMD * 3)
const uint32_t Byte_Y_Cable     = BYTE_Y_CABLE;
const uint32_t Byte_Command_ID  = BYTE_COMMAND_ID;
const uint32_t Byte_Control_Cmd = BYTE_CONTROL_CMD;
#if(BYTE_Y_CABLE%4)
    #define WORD_Y_CABLE     (BYTE_Y_CABLE/4+1)
#else
    #define WORD_Y_CABLE     BYTE_Y_CABLE/4
#endif
#if(BYTE_COMMAND_ID%4)
    #define WORD_COMMAND_ID  (BYTE_COMMAND_ID/4+1)
#else
    #define WORD_COMMAND_ID  BYTE_COMMAND_ID/4
#endif
#if(BYTE_CONTROL_CMD%4)
    #define WORD_CONTROL_CMD (BYTE_CONTROL_CMD/4+1)
#else
    #define WORD_CONTROL_CMD BYTE_CONTROL_CMD/4
#endif
const uint32_t Word_Y_Cable     = WORD_Y_CABLE;
const uint32_t Word_Command_ID  = WORD_COMMAND_ID;
const uint32_t Word_Control_Cmd = WORD_CONTROL_CMD;
const __attribute__((aligned (4))) uint8_t Data_Y_Cable[WORD_Y_CABLE*4] =
{
    /* 250 us (L) Reset */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    /* 40 us (H) */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF,
    /* 10 us (L) */
    0x00, 0x00, 0x00,
    /* Dummy to align word */
    0x00, 0x00,
};

/* Setting for Gen2 control command */
#define Setup_Period  ((HCLK_CLK / 2) / 1000000 * 10 / 24)          // PCLK * (10 us / 24 bits)
#define Setup_T1H     (((HCLK_CLK / 2) / 1000000 * 10 / 24) << 16)  // PCLK * (10 us / 24 bits)
const uint32_t Setup_LLSI_PERIOD_DUTY[2] = {Setup_Period, Setup_T1H};

#define Data_Period   ((HCLK_CLK / 2) / 1000000 * 12 / 10)          // PCLK * 1.2 us
#define Data_T1H      (((HCLK_CLK / 2) / 1000000 * 9 / 10) << 16)   // PCLK * 0.9 us
#define Data_T0H      (((HCLK_CLK / 2) / 1000000 * 3 / 10) << 0)    // PCLK * 0.3 us
const uint32_t Data_LLSI_PERIOD_DUTY[2] = {Data_Period, (Data_T1H | Data_T0H)};

const uint8_t g_u8LLSI_Output_Low  = 0x00;
const uint8_t g_u8LLSI_Output_High = 0xFF;
enum Y_CABLE_CMD
{
    CMD_SET_ID = 0x01,
    CMD_CLEAN_ID,
    CMD_CHECK_ID,
    CMD_SPECIFY_ID,
};
#define REMAP_COMMAND_ID_BYTER(command, id) (((id & 0xF00) >> 8) | ((command &0x00F) << 4))
#define REMAP_COMMAND_ID_BYTEG(command, id) ((command & 0xFF0) >> 4)
#define REMAP_COMMAND_ID_BYTEB(command, id) (id & 0x0FF)
__attribute__((aligned (4))) uint8_t Data_CMD_ID[WORD_COMMAND_ID*4];
uint16_t LLSI_Command_ID[16] =
{
    0x924,    //4'b0000 = 100100100100
    0x926,    //4'b0001 = 100100100110
    0x934,    //4'b0010 = 100100110100
    0x936,    //4'b0011 = 100100110110
    0x9A4,    //4'b0100 = 100110100100
    0x9A6,    //4'b0101 = 100110100110
    0x9B4,    //4'b0110 = 100110110100
    0x9B6,    //4'b0111 = 100110110110
    0xD24,    //4'b1000 = 110100100100
    0xD26,    //4'b1001 = 110100100110
    0xD34,    //4'b1010 = 110100110100
    0xD36,    //4'b1011 = 110100110110
    0xDA4,    //4'b1100 = 110110100100
    0xDA6,    //4'b1101 = 110110100110
    0xDB4,    //4'b1110 = 110110110100
    0xDB6,    //4'b1111 = 110110110110
};
#define PIX_COMMAND_ID_DUMMY  (PIX_COMMAND_ID + 4)
const uint32_t Pix_Command_ID_Dummy  = PIX_COMMAND_ID_DUMMY;
#define BYTE_COMMAND_ID_DUMMY (PIX_COMMAND_ID_DUMMY * 3)
const uint32_t Byte_Command_ID_Dummy = BYTE_COMMAND_ID_DUMMY;
#if(BYTE_COMMAND_ID_DUMMY%4)
    #define WORD_COMMAND_ID_DUMMY  (BYTE_COMMAND_ID_DUMMY/4+1)
#else
    #define WORD_COMMAND_ID_DUMMY  BYTE_COMMAND_ID_DUMMY/4
#endif
const uint32_t Word_Command_ID_Dummy  = WORD_COMMAND_ID_DUMMY;
const __attribute__((aligned (4))) uint8_t Data_Specify_ID_Dummy[16][WORD_COMMAND_ID_DUMMY*4] =
{
    {0x49, 0x9A, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 0  & Dummy 0x00
    {0x49, 0x9A, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 1  & Dummy 0x00
    {0x49, 0x9A, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 2  & Dummy 0x00
    {0x49, 0x9A, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 3  & Dummy 0x00
    {0x49, 0x9A, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 4  & Dummy 0x00
    {0x49, 0x9A, 0xA6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 5  & Dummy 0x00
    {0x49, 0x9A, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 6  & Dummy 0x00
    {0x49, 0x9A, 0xB6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 7  & Dummy 0x00
    {0x4D, 0x9A, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 8  & Dummy 0x00
    {0x4D, 0x9A, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 9  & Dummy 0x00
    {0x4D, 0x9A, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 10 & Dummy 0x00
    {0x4D, 0x9A, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 11 & Dummy 0x00
    {0x4D, 0x9A, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 12 & Dummy 0x00
    {0x4D, 0x9A, 0xA6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 13 & Dummy 0x00
    {0x4D, 0x9A, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 14 & Dummy 0x00
    {0x4D, 0x9A, 0xB6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // Specify + 15 & Dummy 0x00
};

/* Static Feedback mode data */
#define PIX_FEEDBACK_RESET  25    // 250 us
#define PIX_FEEDBACK_H1     2     // 20  us
#define PIX_FEEDBACK_L      1     // 10  us
#define PIX_FEEDBACK_H2     5     // 50  us
#define PIX_ENTER_FEEDBACK  (PIX_FEEDBACK_RESET + PIX_FEEDBACK_H1 + PIX_FEEDBACK_L + PIX_FEEDBACK_H2)
const uint32_t Pix_Enter_Feedback  = PIX_ENTER_FEEDBACK;
#define BYTE_ENTER_FEEDBACK (PIX_ENTER_FEEDBACK * 3)
const uint32_t Byte_Enter_Feedback = BYTE_ENTER_FEEDBACK;
#if(BYTE_ENTER_FEEDBACK%4)
    #define WORD_ENTER_FEEDBACK (BYTE_ENTER_FEEDBACK/4+1)
#else
    #define WORD_ENTER_FEEDBACK BYTE_ENTER_FEEDBACK/4
#endif
const uint32_t Word_Enter_Feedback = WORD_ENTER_FEEDBACK;
const __attribute__((aligned (4))) uint8_t Data_Enter_Feedback[WORD_ENTER_FEEDBACK*4] =
{
    /* 250 us (L) Reset */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    /* 20 us (H1) */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /* 10 us (L) */
    0x00, 0x00, 0x00,
    /* 50 us (H2) */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /* Dummy to align word */
    0xFF
};

/* Static TH20 mode data */
#define PIX_TH20_RESET1     25    // 250 us
#define PIX_TH20_H          2     // 20  us
#define PIX_TH20_RESET2     25    // 250 us
#define PIX_ENTER_TH20      (PIX_TH20_RESET1 + PIX_TH20_H + PIX_TH20_RESET2)
const uint32_t Pix_Enter_TH20  = PIX_ENTER_TH20;
#define BYTE_ENTER_TH20     (PIX_ENTER_TH20 * 3)
const uint32_t Byte_Enter_TH20 = BYTE_ENTER_TH20;
#if(BYTE_ENTER_TH20%4)
    #define WORD_ENTER_TH20 (BYTE_ENTER_TH20/4+1)
#else
    #define WORD_ENTER_TH20 BYTE_ENTER_TH20/4
#endif
const uint32_t Word_Enter_TH20 = WORD_ENTER_TH20;
const __attribute__((aligned (4))) uint8_t Data_Enter_TH20[WORD_ENTER_TH20*4] =
{
    /* 250 us (L) Reset1 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    /* 20 us (H) */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /* 250 us (L) Reset1 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

void Init_Gen2_LED_Capture()
{
    /* Set unused Timer/BPWM to count Gen2 LED feedback waveform length */
#if (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_TIMER)
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable module clock */
    CLK_EnableModuleClock(CAPTURE_TIMER_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(CAPTURE_TIMER_MODULE, CAPTURE_TIMER_CLK_SRC, (uint32_t)NULL);

    /* Lock protected registers */
    SYS_LockReg();

    /* Unit Time Counter = 1 us */
    CAPTURE_TIMER->CTL = TIMER_ONESHOT_MODE | (CAPTURE_TIMER_PRESCALER - 1);
    TIMER_EnableInt(CAPTURE_TIMER);
    NVIC_EnableIRQ(CAPTURE_TIMER_IRQN);

    /* Change interrupt priority to normal */
    NVIC_SetPriority(CAPTURE_TIMER_IRQN, INT_PRIORITY_NORMAL);

#elif (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_BPWM)
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable module clock */
    CLK_EnableModuleClock(CAPTURE_BPWM_MODULE);

    /* Lock protected registers */
    SYS_LockReg();

    /* Unit BPWM Counter = 1 us */
    BPWM_SET_PRESCALER(CAPTURE_BPWM, NULL, CAPTURE_BPWM_PRESCALER - 1);
    BPWM_EnablePeriodInt(CAPTURE_BPWM, (uint32_t)NULL, (uint32_t)NULL);
    NVIC_EnableIRQ(CAPTURE_BPWM_IRQN);

    /* Change interrupt priority to normal */
    NVIC_SetPriority(CAPTURE_BPWM_IRQN, INT_PRIORITY_NORMAL);
#endif
}

#if (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_TIMER)
void CAPTURE_TIMER_HANDLER()
{
    TIMER_ClearIntFlag(CAPTURE_TIMER);

    LED_Gen2_ACK_Done_Flag = 1;
}
#elif (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_BPWM)
void CAPTURE_BPWM_HANDLER()
{
    BPWM_ClearPeriodIntFlag(CAPTURE_BPWM, (uint32_t)NULL);
    BPWM_ForceStop(CAPTURE_BPWM, (uint32_t)NULL);
    CAPTURE_BPWM->CNTCLR = BPWM_CNTCLR_CNTCLR0_Msk;

    LED_Gen2_ACK_Done_Flag = 1;
}
#endif

void LED_Gen2_Set_Ctrl_Setting(uint8_t target_port)
{
    /* Set configuration for current control target port */
    Gen2_Ctrl.Current_Port = target_port;
    Gen2_Ctrl.Control_Data = PDMA_Mapping[target_port]->LED_Data;
    Gen2_Ctrl.fPDMA_Done = &PDMA_Mapping[target_port]->fPDMA_Done;
    Gen2_Ctrl.PDMA_Number = target_port;
}

void LED_Gen2_Transfer_Data(uint8_t target_port)
{
    uint32_t i, j;
    uint32_t byte_size;
    uint32_t word_size;

    /* Clear PDMA flag */
    PDMA_Mapping[target_port]->fPDMA_Done = 0;

    for(i = 1, j= 0; i < (LED_Gen2_Port_Setting[target_port].Strip_Count + 1); i++, j++)
    {
        /* Set Setup_LLSI_PERIOD_DUTY to LLSI_PERIOD and LLSI_DUTY */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 0].CTL =
            ((2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_INC |
            PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 0].SA = (uint32_t)&Setup_LLSI_PERIOD_DUTY;
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 0].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->PERIOD;

        /* Set LLSI_PCNT */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 1].CTL =
            ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
            PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 1].SA = (uint32_t)&Pix_Y_Cable;
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 1].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->PCNT;

        /* Transfer Y_Cable data (Burst 3 words) */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 2].CTL =
            ((3 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
            PDMA_REQ_BURST | PDMA_BURST_4 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 2].SA = (uint32_t)&Data_Y_Cable;
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 2].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->DATA;

        /* Transfer Y_Cable data */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 3].CTL =
            ((WORD_Y_CABLE - 3 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
            PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 3].SA = (uint32_t)&Data_Y_Cable + 12;
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 3].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->DATA;

        /* Set LLSI_PCNT */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 4].CTL =
            ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
            PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 4].SA = (uint32_t)&Pix_Command_ID_Dummy;
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 4].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->PCNT;

        /* Transfer command + ID data (Burst 2 words) */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 5].CTL =
            ((2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
            PDMA_REQ_BURST | PDMA_BURST_2 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 5].SA = (uint32_t)&Data_Specify_ID_Dummy[i][0];
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 5].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->DATA;

        /* Transfer command + ID data (Single 2 words) */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 6].CTL =
            ((WORD_COMMAND_ID_DUMMY - 2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
            PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 6].SA = (uint32_t)&Data_Specify_ID_Dummy[i][0] + 8;
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 6].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->DATA;

        /* Set Data_LLSI_PERIOD_DUTY to LLSI_PERIOD and LLSI_DUTY */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 7].CTL =
            ((2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_INC |
            PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 7].SA = (uint32_t)&Data_LLSI_PERIOD_DUTY;
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 7].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->PERIOD;

        /* Set LLSI_PCNT */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 8].CTL =
            ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
            PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 8].SA = (uint32_t)&LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LED_Number;
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 8].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->PCNT;

        byte_size = LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LED_Number * 3;
        if(byte_size % 4)
            word_size = byte_size/4 + 1;
        else
            word_size = byte_size/4;

        /* Transfer LED data (Burst 3 words) */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 9].CTL =
            ((3 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
            PDMA_REQ_BURST | PDMA_BURST_4 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 9].SA = (uint32_t)(LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LED_Data + \
                                                                                      LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LED_Offset);
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM + 9].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->DATA;

        /* Transfer LED data (Single mode) */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM +10].CTL =
            ((word_size - 3 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
            PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
            PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
            PDMA_OP_SCATTER;
        /* Configure source address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM +10].SA = (uint32_t)(LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LED_Data + 12 + \
                                                                                      LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LED_Offset);
        /* Configure destination address */
        LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM +10].DA = (uint32_t)&LLSI_Port_Mapping[target_port]->DATA;
    }
    /* Change last descriptor to basic mode */
    LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM - 1].CTL 
        = (LED_Gen2_PDMA_DESC[target_port][j*LED_Gen2_PDMA_DESC_NUM - 1].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;

    /* Set LLSI interrupt count */
    LED_Gen2_Port_Setting[target_port].LLSI_INT_Count = 3 * i;

    /* Start LLSI PDMA transfer */
    PDMA_SetTransferMode(PDMA0, target_port, 99 + target_port, TRUE, (uint32_t)&LED_Gen2_PDMA_DESC[target_port][0]);    // PDMA_LLSI0 = 99
}

void LED_Gen2_Check_ACK(uint8_t one_ACK, uint32_t timeout_us)
{
    /* Change MFP to GPIO */
    *(uint32_t *)(SYS_BASE + LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_MFP_OFFSET) =
        (*(uint32_t *)(SYS_BASE + LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_MFP_OFFSET) & \
         (~LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_GPIO_MSK));

    /* Enable GPIO interrupt to get ACK pulse */
    GPIO_EnableInt(LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_Port,
                   LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_Number,
                   GPIO_INT_BOTH_EDGE);

    /* Change interrupt priority to highest */
    NVIC_SetPriority(LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_IRQn, INT_PRIORITY_HIGHEST);

    /* Set ACK Width Recoard Array */
    LED_Gen2_ACK_Width = Gen2_Ctrl.Control_Data;
    LED_Gen2_ACK_Count = 0;

#if (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_TIMER)
    /* Wait for ACK pulse width */
    CAPTURE_TIMER->CMP = timeout_us;

    /* Start Timer */
    LED_Gen2_ACK_Done_Flag = 0;
    LED_Gen2_One_ACK_Flag = one_ACK;
    TIMER_Start(CAPTURE_TIMER);
#elif (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_BPWM)
    /* Wait for ACK pulse width */
    CAPTURE_BPWM->PERIOD = timeout_us;

    /* Start Timer */
    LED_Gen2_ACK_Done_Flag = 0;
    LED_Gen2_One_ACK_Flag = one_ACK;
    BPWM_Start(CAPTURE_BPWM, (uint32_t)NULL);
#endif
}

uint8_t LED_Gen2_Wait_ACK_Done(void)
{
    /* Check ACK done */
    if(LED_Gen2_ACK_Done_Flag)
    {
        LED_Gen2_ACK_Done_Flag = 0;

        /* Change interrupt priority to normal */
        NVIC_SetPriority(LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_IRQn, INT_PRIORITY_NORMAL);

        /* Change MFP to LLSI */
        *(uint32_t *)(SYS_BASE + LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_MFP_OFFSET) =
            (*(uint32_t *)(SYS_BASE + LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_MFP_OFFSET) & \
             (~LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_GPIO_MSK)) | \
            LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_LLSI_MSK;

        /* Disable GPIO Interrupt */
        GPIO_DisableInt(LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_Port,
                        LED_LLSI_IO_Setting[Gen2_Ctrl.Current_Port].IO_Number);

        return 1;
    }

    /* Not done yet */
    return 0;
}

void LED_Gen2_Control_CMD_ID(uint8_t cmd, uint8_t id)
{
    /* Clear PDMA flag */
    *Gen2_Ctrl.fPDMA_Done = 0;

    /* Set Setup_LLSI_PERIOD_DUTY to LLSI_PERIOD and LLSI_DUTY */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].CTL =
        ((2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_INC |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].SA = (uint32_t)&Setup_LLSI_PERIOD_DUTY;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PERIOD;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].SA = (uint32_t)&Pix_Y_Cable;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Transfer Y_Cable data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].CTL =
        ((WORD_Y_CABLE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].SA = (uint32_t)&Data_Y_Cable;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].SA = (uint32_t)&Pix_Command_ID;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Command + ID */
    Data_CMD_ID[0] = REMAP_COMMAND_ID_BYTER(LLSI_Command_ID[cmd], LLSI_Command_ID[id]);
    Data_CMD_ID[1] = REMAP_COMMAND_ID_BYTEG(LLSI_Command_ID[cmd], LLSI_Command_ID[id]);
    Data_CMD_ID[2] = REMAP_COMMAND_ID_BYTEB(LLSI_Command_ID[cmd], LLSI_Command_ID[id]);

    /* Transfer command + ID data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].CTL =
        ((WORD_COMMAND_ID - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_BASIC;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].SA = (uint32_t)Data_CMD_ID;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI interrupt count */
    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LLSI_INT_Count = 2;

    /* Start LLSI PDMA transfer */
    PDMA_SetTransferMode(PDMA0, Gen2_Ctrl.Current_Port, 99 + Gen2_Ctrl.Current_Port, TRUE, (uint32_t)&LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0]);    // PDMA_LLSI0 = 99
    LLSI_SET_PDMA_MODE(LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]);

    switch(cmd)
    {
        case CMD_SET_ID:
        {
            /* Wait transfer done */
            while(*Gen2_Ctrl.fPDMA_Done == 0);

            /* Get one ACK, max width is 85 us */
            LED_Gen2_Check_ACK(1, 85);

            break;
        }
        case CMD_CHECK_ID:
        {
            /* Wait transfer done */
            while(*Gen2_Ctrl.fPDMA_Done == 0);

            /* Get all ACK, max width is 85 us */
            LED_Gen2_Check_ACK(0, 85);

            break;
        }
        case CMD_CLEAN_ID:
        case CMD_SPECIFY_ID:
        {
            break;
        }
    }
}

void LED_Gen2_Get_LED_Number(uint8_t id)
{
    /* Clear PDMA flag */
    *Gen2_Ctrl.fPDMA_Done = 0;

    /* Set Setup_LLSI_PERIOD_DUTY to LLSI_PERIOD and LLSI_DUTY */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].CTL =
        ((2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_INC |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].SA = (uint32_t)&Setup_LLSI_PERIOD_DUTY;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PERIOD;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].SA = (uint32_t)&Pix_Y_Cable;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Transfer Y_Cable data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].CTL =
        ((WORD_Y_CABLE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].SA = (uint32_t)&Data_Y_Cable;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].SA = (uint32_t)&Pix_Command_ID;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Command + ID */
    Data_CMD_ID[0] = REMAP_COMMAND_ID_BYTER(LLSI_Command_ID[CMD_SPECIFY_ID], LLSI_Command_ID[id]);
    Data_CMD_ID[1] = REMAP_COMMAND_ID_BYTEG(LLSI_Command_ID[CMD_SPECIFY_ID], LLSI_Command_ID[id]);
    Data_CMD_ID[2] = REMAP_COMMAND_ID_BYTEB(LLSI_Command_ID[CMD_SPECIFY_ID], LLSI_Command_ID[id]);

    /* Transfer command + ID data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].CTL =
        ((WORD_COMMAND_ID - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].SA = (uint32_t)Data_CMD_ID;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][5].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][5].SA = (uint32_t)&Pix_Enter_Feedback;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][5].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Transfer Feedback_mode data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][6].CTL =
        ((WORD_ENTER_FEEDBACK - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_BASIC;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][6].SA = (uint32_t)&Data_Enter_Feedback;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][6].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI interrupt count */
    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LLSI_INT_Count = 2;

    /* Start LLSI PDMA transfer */
    PDMA_SetTransferMode(PDMA0, Gen2_Ctrl.Current_Port, 99 + Gen2_Ctrl.Current_Port, TRUE, (uint32_t)&LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0]);    // PDMA_LLSI0 = 99
    LLSI_SET_PDMA_MODE(LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]);

    /* Wait transfer done */
    while(*Gen2_Ctrl.fPDMA_Done == 0);

    /* Get all ACK, max width is 160 us */
    LED_Gen2_Check_ACK(0, 160);
}

void LED_Gen2_Set_SEL_CUR(uint8_t id, uint8_t sel_cur)
{
    uint32_t i;
    uint32_t setting;
    uint8_t setting_byte1, setting_byte2, setting_byte3;

    /* Clear PDMA flag */
    *Gen2_Ctrl.fPDMA_Done = 0;

    /* Set Setup_LLSI_PERIOD_DUTY to LLSI_PERIOD and LLSI_DUTY */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].CTL =
        ((2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_INC |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].SA = (uint32_t)&Setup_LLSI_PERIOD_DUTY;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PERIOD;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].SA = (uint32_t)&Pix_Y_Cable;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Transfer Y_Cable data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].CTL =
        ((WORD_Y_CABLE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].SA = (uint32_t)&Data_Y_Cable;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].SA = (uint32_t)&Pix_Command_ID;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Command + ID */
    Data_CMD_ID[0] = REMAP_COMMAND_ID_BYTER(LLSI_Command_ID[CMD_SPECIFY_ID], LLSI_Command_ID[id]);
    Data_CMD_ID[1] = REMAP_COMMAND_ID_BYTEG(LLSI_Command_ID[CMD_SPECIFY_ID], LLSI_Command_ID[id]);
    Data_CMD_ID[2] = REMAP_COMMAND_ID_BYTEB(LLSI_Command_ID[CMD_SPECIFY_ID], LLSI_Command_ID[id]);

    /* Transfer command + ID data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].CTL =
        ((WORD_COMMAND_ID - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].SA = (uint32_t)Data_CMD_ID;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][5].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][5].SA = (uint32_t)&Pix_Enter_TH20;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][5].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Transfer TH20_mode data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][6].CTL =
        ((WORD_ENTER_TH20 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][6].SA = (uint32_t)&Data_Enter_TH20;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][6].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set Data_LLSI_PERIOD_FUTY to LLSI_PERIOD and LLSI_DUTY */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][7].CTL =
        ((2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_INC |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][7].SA = (uint32_t)&Data_LLSI_PERIOD_DUTY;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][7].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PERIOD;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][8].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][8].SA = (uint32_t)&LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LED_Gen2_Setting[id].LED_Number;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][8].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Set SEL_CUR */
    setting = LED_Gen2_Setting_Default | \
              (0x1F << BLUE_DIMMING_Pos) |   // Blue Dimming
              (0x1F << RED_DIMMING_Pos) |    // Red Dimming
              (sel_cur << SEL_CUR_Pos) |     // SEL_CUR
              (0x1F << GREEN_DIMMING_Pos);   // Green Dimming
    setting_byte1 = (setting & 0x00FF00) >>  8;
    setting_byte2 = (setting & 0xFF0000) >> 16;
    setting_byte3 = (setting & 0x0000FF)      ;
    for(i = 0; i < LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LED_Gen2_Setting[id].LED_Number; i++)
    {
        Gen2_Ctrl.Control_Data[i*3    ] = setting_byte1;
        Gen2_Ctrl.Control_Data[i*3 + 1] = setting_byte2;
        Gen2_Ctrl.Control_Data[i*3 + 2] = setting_byte3;
    }
    if(i*3%4)
        i = i*3/4 + 1;
    else
        i = i*3/4;

    /* Transfer SEL_CUR setting */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][9].CTL =
        ((i - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_BASIC;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][9].SA = (uint32_t)Gen2_Ctrl.Control_Data;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][9].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI interrupt count */
    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LLSI_INT_Count = 3;

    /* Start LLSI PDMA transfer */
    PDMA_SetTransferMode(PDMA0, Gen2_Ctrl.Current_Port, 99 + Gen2_Ctrl.Current_Port, TRUE, (uint32_t)&LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0]);    // PDMA_LLSI0 = 99
    LLSI_SET_PDMA_MODE(LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]);
}

void LED_Gen2_Get_Feedback(uint8_t id)
{
    /* Clear PDMA flag */
    *Gen2_Ctrl.fPDMA_Done = 0;

    /* Set Setup_LLSI_PERIOD_DUTY to LLSI_PERIOD and LLSI_DUTY */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].CTL =
        ((2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_INC |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].SA = (uint32_t)&Setup_LLSI_PERIOD_DUTY;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PERIOD;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].SA = (uint32_t)&Pix_Y_Cable;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][1].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Transfer Y_Cable data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].CTL =
        ((WORD_Y_CABLE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].SA = (uint32_t)&Data_Y_Cable;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][2].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].SA = (uint32_t)&Pix_Command_ID;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][3].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Command + ID */
    Data_CMD_ID[0] = REMAP_COMMAND_ID_BYTER(LLSI_Command_ID[CMD_SPECIFY_ID], LLSI_Command_ID[id]);
    Data_CMD_ID[1] = REMAP_COMMAND_ID_BYTEG(LLSI_Command_ID[CMD_SPECIFY_ID], LLSI_Command_ID[id]);
    Data_CMD_ID[2] = REMAP_COMMAND_ID_BYTEB(LLSI_Command_ID[CMD_SPECIFY_ID], LLSI_Command_ID[id]);

    /* Transfer command + ID data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].CTL =
        ((WORD_COMMAND_ID - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].SA = (uint32_t)Data_CMD_ID;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][4].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI_PCNT */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][5].CTL =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX |
        PDMA_REQ_BURST | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_SCATTER;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][5].SA = (uint32_t)&Pix_Enter_Feedback;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][5].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->PCNT;

    /* Transfer Feedback_mode data */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][6].CTL =
        ((WORD_ENTER_FEEDBACK - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
        PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX |
        PDMA_REQ_SINGLE | PDMA_BURST_128 | PDMA_TBINTDIS_DISABLE |
        PDMA_OP_BASIC;
    /* Configure source address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][6].SA = (uint32_t)&Data_Enter_Feedback;
    /* Configure destination address */
    LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][6].DA = (uint32_t)&LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]->DATA;

    /* Set LLSI interrupt count */
    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LLSI_INT_Count = 2;

    /* Start LLSI PDMA transfer */
    PDMA_SetTransferMode(PDMA0, Gen2_Ctrl.Current_Port, 99 + Gen2_Ctrl.Current_Port, TRUE, (uint32_t)&LED_Gen2_PDMA_DESC[Gen2_Ctrl.Current_Port][0]);    // PDMA_LLSI0 = 99
    LLSI_SET_PDMA_MODE(LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]);

    /* Wait transfer done */
    while(*Gen2_Ctrl.fPDMA_Done == 0);

    /* Get all ACK, max width is 160 us */
    LED_Gen2_Check_ACK(0, 160);
}

uint32_t LED_Gen2_Merge_Fixed_ID(uint8_t id)
{
    uint32_t i;
    uint32_t strip_id = 0;

    /* Merge fixed ID of current strip */
    for(i = 0; (i < LED_Gen2_ACK_Count) && (i < 32); i++)
    {
        if(Gen2_Ctrl.Control_Data[i] > LED_GEN2_FIXED_ID_CODE_LENGTH)
        {
            /* Bit 1 */
            strip_id |= 0x1 << (31 - i);
        }
    }

    /* Return ID */
    return strip_id;
}

void LED_Gen2_Control_Port(void)
{
    switch(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status)
    {
        /* No action */
        case 0x00:
        {
            break;
        }
        /* Clean all dynamic ID */
        case 0x01:
        {
            /* Clean target ID */
            LED_Gen2_Control_CMD_ID(CMD_CLEAN_ID, 0x00);

            /* Change status to wait transfer done */
            LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x02;
        }
        /* Clear all LED data */
        case 0x02:
        {
            if(*Gen2_Ctrl.fPDMA_Done == 1)
            {
                /* Change LLSI and PDMA setting to transfer LED data */
                LLSI_Initial(Gen2_Ctrl.Current_Port);
                PDMA_Initial(Gen2_Ctrl.Current_Port);

                Clear_LED_Data(LED_Mapping[Gen2_Ctrl.Current_Port]);
                LLSI_SET_PDMA_MODE(LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]);

                /* Change status to wait transfer done */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x03;
            }

            break;
        }
        /* Wait transfer done */
        case 0x03:
        {
            if(*Gen2_Ctrl.fPDMA_Done == 1)
            {
                /* Change LLSI and PDMA setting for LED Gen2 control */
                LED_Gen2_LLSI_PDMA_Init(Gen2_Ctrl.Current_Port);

                /* Initial to start set dynamic ID */
                /* Start from ID = 0x01 */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID = 1;

                /* Clear strip count */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count = 0;

                /* Change status to start set ID loop */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x04;
            }

            break;
        }
        /* Set dynamic ID */
        case 0x04:
        {
            /* Check strip count is not larger than 15 (max strip number in spec) */
            if(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count < 15)
            {
                /* Set target ID */
                LED_Gen2_Control_CMD_ID(CMD_SET_ID, LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID);

                /* Change status to wait ACK done */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x05;
            }
            else
            {
                /* Change status to check strip number */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x08;
            }

            break;
        }
        /* Wait Set ID ACK done */
        case 0x05:
        {
            if(LED_Gen2_Wait_ACK_Done() == 1)
            {
                /* Check ACK */
                if(LED_Gen2_ACK_Count == 0)
                {
                    /* No ACK, clear this dynamic ID */
                    LED_Gen2_Control_CMD_ID(CMD_CLEAN_ID, LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID);

                    /* Change status to wait transfer done */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x08;
                }
                else
                {
                    /* Check ID */
                    LED_Gen2_Control_CMD_ID(CMD_CHECK_ID, LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID);

                    /* Change status to wait ACK done */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x06;
                }
            }

            break;
        }
        /* Wait Check ID ACK done */
        case 0x06:
        {
            if(LED_Gen2_Wait_ACK_Done() == 1)
            {
                /* Check ACK */
                if(LED_Gen2_ACK_Count == 1)
                {
                    /* Add target ID */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID++;

                    /* Add strip count */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count++;

                    /* Change status to set next ID */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x04;
                }
                else
                {
                    /* Invalid, clean this dynamic ID */
                    LED_Gen2_Control_CMD_ID(CMD_CLEAN_ID, LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID);

                    /* Change status to wait transfer done */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x07;
                }
            }

            break;
        }
        /* Wait clean target ID transfer done to set target ID again*/
        case 0x07:
        {
            if(*Gen2_Ctrl.fPDMA_Done == 1)
            {
                /* Change status to set target ID again */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x04;
            }

            break;
        }
        /* Wait clean target ID transfer done to get LED number */
        case 0x08:
        {
            if(*Gen2_Ctrl.fPDMA_Done == 1)
            {
                if(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count > LED_GEN2_MAX_STRIP_COUNT)
                {
                    /* Disable LED Gen2 control */
                    LED_Gen2_Enable_Control(Gen2_Ctrl.Current_Port, FALSE);

                    /* Clear detected strip count */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count = 0;

                    /* Change status to finish detection */
                    /* Clear flag */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Flag = 0;

                    /* Change status to finish */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x00;
                }
                else
                {
                    /* Start from ID = 0x01 */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID = 1;

                    /* Change status to get LED number */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x09;

                    /* Clear total LED number count */
                    Total_LED_Gen2_Num = 0;
                }
            }

            break;
        }
        /* Get LED number of each strip */
        case 0x09:
        {
            /* Check target ID is not larger than strip count */
            if(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID  < (LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count + 1))
            {
                /* Get LED number of target ID strip */
                LED_Gen2_Get_LED_Number(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID);

                /* Change status to wait ACK done */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x0A;
            }
            else
            {
                /* Change status to get fixed ID of each strip */
                /* Start from ID = 0x01 */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID = 1;

                /* Change status to get fixed ID of each strip */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x0B;
            }

            break;
        }
        /* Wait Get_LED_Number ACK done */
        case 0x0A:
        {
            if(LED_Gen2_Wait_ACK_Done() == 1)
            {
                /* Check LED_Gen2_ACK_Count, the total max value is LED_GEN2_MAX_LED_NUMBER */
                if((Total_LED_Gen2_Num + LED_Gen2_ACK_Count) > LED_GEN2_MAX_LED_NUMBER)
                {
                    /* Disable LED Gen2 control */
                    LED_Gen2_Enable_Control(Gen2_Ctrl.Current_Port, FALSE);
                    /* Clear detected strip count */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count = 0;

                    /* Change status to finish detection */
                    /* Clear flag */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Flag = 0;

                    /* Change status to finish */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x00;
                }
                else
                {
                    /* Get LED number */
                    LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LED_Gen2_Setting[LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID].LED_Number = LED_Gen2_ACK_Count;

                    /* Add total LED numbre */
                    Total_LED_Gen2_Num += LED_Gen2_ACK_Count;
                }

                /* Add target ID */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID++;

                /* Change status to get LED number of next ID strip */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x09;
            }

            break;
        }
        /* Get fixed ID of each strip */
        case 0x0B:
        {
            /* Check target ID is not larger than strip count */
            if(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID < (LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count + 1))
            {
                /* Set SEL_CUR to SEL_LED_ID for feedback fixed ID */
                LED_Gen2_Set_SEL_CUR(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID, SEL_LED_ID);

                /* Change status to wait transfer done */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x0C;
            }
            else
            {
                /* Change status to finish detection */
                /* Clear flag */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Flag = 0;

                /* Change status to finish */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x00;

                /* Check detect result and get stored setting from default setting */
                if(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count != 0)
                {
                    uint8_t i;

                    /* Read out the setting from user setting */
                    for(i = 1; i < (LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count + 1); i++)
                    {
                        /* Find setting */
                        if(u8SettingFlag == 0xFF)
                            LED_Gen2_ReadStoredSetting(Gen2_Ctrl.Current_Port, i);
                        else
                            /* No exist setting, use default setting */
                            LED_Gen2_Default_Setting(Gen2_Ctrl.Current_Port, i);
                    }

                    /* Use_Gen2 */
                    /* Clear all ID if Use_Gen2 = 0 */
                    if(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Use_Gen2 == 0)
                    {
                        LED_Gen2_Control_CMD_ID(CMD_CLEAN_ID, 0x00);
                        while(*Gen2_Ctrl.fPDMA_Done == 0);

                        /* Clear LLSI interrupt count */
                        LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LLSI_INT_Count = 0;

                        /* Initial target port */
                        LLSI_Initial(Gen2_Ctrl.Current_Port);
                        PDMA_Initial(Gen2_Ctrl.Current_Port);
                    }

                    /* Sort sequence number by fixed ID */
                    uint32_t left = 0;
                    uint32_t right = LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count - 1;
                    uint32_t shift = 0;
                    uint32_t Data_Pair[TOTAL_LED_AREA][2];
                    /* Initial sorting data */
                    uint32_t index;
                    for(index = 0; index < LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count; index++)
                    {
                        Data_Pair[index][0] = index+1;
                        Data_Pair[index][1] = LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LED_Gen2_Setting[index+1].Fixed_ID;
                    }
                    uint32_t temp0, temp1;
                    while(left < right) {
                        for(index = left; index < right; index++)
                        {
                            if(Data_Pair[index][1] > Data_Pair[index+1][1])    // Swap larger ID to right
                            {
                                temp0 = Data_Pair[index][0];
                                temp1 = Data_Pair[index][1];
                                Data_Pair[index][0] = Data_Pair[index+1][0];
                                Data_Pair[index][1] = Data_Pair[index+1][1];
                                Data_Pair[index+1][0] = temp0;
                                Data_Pair[index+1][1] = temp1;
                                shift = index;
                            }
                        }
                        right = shift;
                        for(index = right; index > left; index--)
                        {
                            if(Data_Pair[index][1] < Data_Pair[index-1][1])    // Swap larger ID to right
                            {
                                temp0 = Data_Pair[index][0];
                                temp1 = Data_Pair[index][1];
                                Data_Pair[index][0] = Data_Pair[index-1][0];
                                Data_Pair[index][1] = Data_Pair[index-1][1];
                                Data_Pair[index-1][0] = temp0;
                                Data_Pair[index-1][1] = temp1;
                                shift = index;
                            }
                        }
                        left = shift;
                    }
                    for(index = 0; index < LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Strip_Count; index++)
                    {
                        LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LED_Gen2_Setting[Data_Pair[index][0]].Sequence = index;
                    }
                }
            }

            break;
        }
        /* Wait Set_SEL_CUR transfer done */
        case 0x0C:
        {
            if(*Gen2_Ctrl.fPDMA_Done == 1)
            {
                /* Get feedback */
                LED_Gen2_Get_Feedback(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID);

                /* Change status to wait ACK done */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x0D;
            }

            break;
        }
        /* Wait feedback mode ACK done */
        case 0x0D:
        {
            if(LED_Gen2_Wait_ACK_Done() == 1)
            {
                /* Get fixed ID from target strip  */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].LED_Gen2_Setting[LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID].Fixed_ID = LED_Gen2_Merge_Fixed_ID(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID);

                /* Set SEL_CUR to SEL_LED_CUR for feedback fixed ID */
                LED_Gen2_Set_SEL_CUR(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID, SEL_LED_CUR);

                /* Change status to wait transfer done */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x0E;
            }

            break;
        }
        /* Wait Set_SEL_CUR transfer done */
        case 0x0E:
        {
            if(*Gen2_Ctrl.fPDMA_Done == 1)
            {
                /* Add target ID */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_ID++;

                /* Change status to get fixed ID of next ID strip */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x0B;
            }

            break;
        }
        /* Clear LED data */
        case 0x20:
        {
            /* Clear LED data */
            if(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].ClearData_Flag == 1)
            {
                if(LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Use_Gen2 == TRUE)
                {
                    /* Clear LED Gen1 data */
                    LLSI_Initial(Gen2_Ctrl.Current_Port);
                    PDMA_Initial(Gen2_Ctrl.Current_Port);
                    Clear_LED_Data(LED_Mapping[Gen2_Ctrl.Current_Port]);
                    LLSI_SET_PDMA_MODE(LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]);
                }
                else
                {
                    /* Clear LED Gen2 data */
                    LED_Gen2_Disable_Mode(Gen2_Ctrl.Current_Port);
                    LLSI_SET_PDMA_MODE(LLSI_Port_Mapping[Gen2_Ctrl.Current_Port]);
                }

                /* Clear flag */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].ClearData_Flag = 0;
            }

            /* Change status to enable/disable target port control LED Gen2 */
            LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x21;
        }

        /* Change target port enable/disable Gen2 */
        case 0x21:
        {
            if(*Gen2_Ctrl.fPDMA_Done == 1)
            {
                /* Enable/Disable Gen2 */
                LED_Gen2_Enable_Control(Gen2_Ctrl.Current_Port, LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Use_Gen2);

                /* Change status to finish detection */
                /* Clear flag */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Flag = 0;

                /* Change status to finish */
                LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x00;
            }

            break;
        }
    }
}

void LED_Gen2_LLSI_PDMA_Init(uint8_t target_port)
{
    uint8_t i;

    /* Disable reset command function */
    LLSI_DISABLE_RESET_COMMAND(LLSI_Port_Mapping[target_port]);

    /* Change LLSI_PERIOD and LLSI_DUTY setting */
    LLSI_Port_Mapping[target_port]->PERIOD = Setup_LLSI_PERIOD_DUTY[0];
    LLSI_Port_Mapping[target_port]->DUTY = Setup_LLSI_PERIOD_DUTY[1];

    /* Disable reset command done interrupt */
    LLSI_DisableInt((LLSI_T *)LLSI_Port_Mapping[target_port], LLSI_FEND_INT_MASK);

    /* Enable frame done interrupt */
    LLSI_EnableInt((LLSI_T *)LLSI_Port_Mapping[target_port], LLSI_FEND_INT_MASK);

    /* Point to first PDMA descriptor */
    PDMA0->DSCT[target_port].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][0] - (PDMA0->SCATBA);

    /* Even descriptor table configuration */
    for(i = 0; i < (LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM); i+=2)
        /* Configure next descriptor table address */
        LED_Gen2_PDMA_DESC[target_port][i].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][i+1] - (PDMA0->SCATBA); /* next descriptor table is table i+1 */

    /* Odd descriptor table configuration */
    for(i = 1; i < (LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM - 1); i+=2)
        /* Configure next descriptor table address */
        LED_Gen2_PDMA_DESC[target_port][i].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][i+1] - (PDMA0->SCATBA); /* next descriptor table is table i+1 */

    /* Last descriptor table configuration */
    /* Configure next descriptor table address */
    LED_Gen2_PDMA_DESC[target_port][LED_GEN2_MAX_STRIP_COUNT*LED_Gen2_PDMA_DESC_NUM-1].NEXT = 0; /* No next operation table. No effect in basic mode */

    /* Change PDMA setting */
    PDMA_SetTransferMode(PDMA0, target_port, 99 + target_port, TRUE, 0);    // PDMA_LLSI0 = 99
    PDMA0->DSCT[target_port].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][0] - (PDMA0->SCATBA);

    /* Set GPIO to input mode */
    LED_LLSI_IO_Setting[target_port].IO_Port->SMTEN |= LED_LLSI_IO_Setting[target_port].IO_Bit;
    GPIO_SetMode(LED_LLSI_IO_Setting[target_port].IO_Port,
                 LED_LLSI_IO_Setting[target_port].IO_Bit,
                 GPIO_MODE_INPUT);

    /* Enable GPIO NVIC interrupt to get ACK pulse */
    NVIC_EnableIRQ(LED_LLSI_IO_Setting[target_port].IO_IRQn);

    /* Change interrupt priority to normal */
    NVIC_SetPriority(LED_LLSI_IO_Setting[target_port].IO_IRQn, INT_PRIORITY_NORMAL);
}

void LED_Gen2_Initial_Setting(uint8_t target_port)
{
    uint8_t i;

    /* Initial default setting */
    /* LED_Gen2_Setting_T */
    LED_Gen2_Port_Setting[target_port].Control_Flag = 0;
    LED_Gen2_Port_Setting[target_port].Control_Status = 0;
    LED_Gen2_Port_Setting[target_port].Control_ID = 0;
    LED_Gen2_Port_Setting[target_port].Strip_Count = 0;
    LED_Gen2_Port_Setting[target_port].Use_Gen2 = 0;
    LED_Gen2_Port_Setting[target_port].LLSI_INT_Count = 0;

    /* Initial offset start address */
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[0].LEDSetting.LED_Offset = 0;

    /* LED_Gen2_Strip_T */
    for(i = 1; i < (LED_GEN2_MAX_STRIP_COUNT + 1); i++)
    {
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].Fixed_ID = 0xFFFFFFFF;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LED_Number = 0;

        /* LED_Setting_T */
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.TimeCounter = 0;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.AP_Sync = 0;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LEDNum = LED_GEN2_DEFAULT_LED_NUMBER;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LightingMode = 0x0A;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Color_R = 255;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Color_G = 0;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Color_B = 0;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Brightness = 0xFF;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Speed = 0xD7;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Direction = Dir_Forward;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LED_Type = Type_GRB;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.fPDMA_Done = 1;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.RandomFlag = 1;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Random = 0;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Mode_FUNC = FUNC_Rainbow;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LED_Data = LED_Data_Array_Mapping[target_port];
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Main_Volume = 0;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Left_Volume = 0;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Right_Volume = 0;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Music_Action = Music_POP;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Array_Size = 
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LLSI_Num = target_port;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.ColorIndex = eColorRed;

        /* Offset */
        uint32_t byte_size = LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i-1].LEDSetting.LEDNum * 3;
        if(byte_size % 4)
            byte_size = (byte_size/4 + 1) * 4;
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LED_Offset = (LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i-1].LEDSetting.LED_Offset)
                                                                                       + byte_size;
    }
}

void LED_Gen2_Default_Setting(uint8_t target_port, uint8_t id)
{
    /* AP_Sync, LED_Nums, LightingMode */
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.AP_Sync = 0;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.LEDNum = LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LED_Number;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.LightingMode = 0x0A;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Mode_FUNC = FUNC_Rainbow;

    /* Color_R, Color_G, Color_B, Brightness */
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Color_R = 255;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Color_G = 0;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Color_B = 0;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Brightness = 0xFF;

    /* Speed, Direction, LED_Type */
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Speed = 0xD7;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Direction = Dir_Forward;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.LED_Type = Type_GRB;

    /* Offset */
    uint32_t byte_size = LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id-1].LEDSetting.LEDNum*3;
    if(byte_size%4)
        byte_size = (byte_size/4 + 1) * 4;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.LED_Offset = (LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id-1].LEDSetting.LED_Offset)
                                                                                    + byte_size;
}

void LED_Effect_Setting(uint8_t LightingMode, uint8_t Color_R, uint8_t Color_G, uint8_t Color_B, uint8_t Brightness)
{
    u8SettingFlag = 0xFF;

    /* LightingMode */
    User_LEDSetting.LightingMode = LightingMode;
    User_LEDSetting.Mode_FUNC = Gen2Mode_Function[User_LEDSetting.LightingMode];

    /* Color_R, Color_G, Color_B, Brightness */
    User_LEDSetting.Color_R = Color_R;
    User_LEDSetting.Color_G = Color_G;
    User_LEDSetting.Color_B = Color_B;
    User_LEDSetting.Brightness = Brightness;
}

void LED_Gen2_ReadStoredSetting(uint8_t target_port, uint8_t id)
{
    /* AP_Sync, LED_Nums, LightingMode */
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.AP_Sync = 0;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.LEDNum = LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LED_Number;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.LightingMode = User_LEDSetting.LightingMode;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Mode_FUNC = User_LEDSetting.Mode_FUNC;

    /* Color_R, Color_G, Color_B, Brightness */
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Color_R = User_LEDSetting.Color_R;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Color_G = User_LEDSetting.Color_G;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Color_B = User_LEDSetting.Color_B;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Brightness = User_LEDSetting.Brightness;

    /* Speed, Direction, LED_Type */
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Speed = 0xD7;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.Direction = Dir_Forward;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.LED_Type = Type_GRB;

    /* Offset */
    uint32_t byte_size = LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id-1].LEDSetting.LEDNum*3;
    if(byte_size%4)
        byte_size = (byte_size/4 + 1) * 4;
    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id].LEDSetting.LED_Offset = (LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[id-1].LEDSetting.LED_Offset)
                                                                                    + byte_size;
}

void LED_Gen2_Set_Setting(uint8_t target_port, uint8_t *setting)
{
    uint8_t i;
    uint8_t counter = 0;
    uint32_t id;

    do
    {
        /* Check fixed ID is valid nor not */
        id = (uint32_t)((setting[12 + 16*counter]      )|
                        (setting[13 + 16*counter] <<  8)|
                        (setting[14 + 16*counter] << 16)|
                        (setting[15 + 16*counter] << 24));
        if(id == 0xFFFFFFFF)
        {
            break;
        }
        else
        {
            /* Check fixed ID to current strip */
            for(i = 1; i < (LED_GEN2_MAX_STRIP_COUNT + 1); i++)
            {
                if(LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].Fixed_ID == id)
                {
                    /* LED_Nums, LED_Type, Direction, AP_Sync,  */
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LEDNum = (uint16_t)(setting[    16*counter] | setting[1 + 16*counter] << 8);
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LED_Type = setting[2 + 16*counter]; // Fixed GRB type in default
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Direction = setting[3 + 16*counter];
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.AP_Sync = setting[4 + 16*counter];

                    /* LightingMode, Color_R, Color_G, Color_B, Brightness, Speed */
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LightingMode = setting[5 + 16*counter];
                    if(LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LightingMode >= LED_MODE_COUNT)
                        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LightingMode = 0;
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Mode_FUNC = (LED_FUNC)Mode_Function[LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LightingMode];
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Color_R = setting[6 + 16*counter];
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Color_G = setting[7 + 16*counter];
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Color_B = setting[8 + 16*counter];
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Brightness = setting[9 + 16*counter];
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Speed = setting[10 + 16*counter];

                    /* Offset */
                    uint32_t byte_size = LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i-1].LEDSetting.LEDNum * 3;
                    if(byte_size % 4)
                        byte_size = (byte_size/4 + 1) * 4;
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.LED_Offset = (LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i-1].LEDSetting.LED_Offset)
                                                                                                   + byte_size;

                    /* Reset Timer */
                    LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.TimeCounter = 0;
                }
            }
        }
        counter++;
    } while(1);
}

void LED_Gen2_Control(uint8_t target_port)
{
    uint32_t i;

    /* Prepare data */
    for(i = 1; i < (LED_Gen2_Port_Setting[target_port].Strip_Count + 1); i++)
    {
        /* Calculate LED data */
        LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting.Mode_FUNC(&LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting);
    }

    /* Send out command and LED data */
    LED_Gen2_Transfer_Data(target_port);
}

void LED_Gen2_Disable_Mode(uint8_t target_port)
{
    uint32_t i;
    uint8_t* Control_Data;

    Control_Data = (uint8_t *)LED_Data_Array_Mapping[target_port];

    /* Prepare data */
    for(i = 1; i < (LED_Gen2_Port_Setting[target_port].Strip_Count + 1); i++)
        /* Calculate LED data */
        FUNC_Off(&LED_Gen2_Port_Setting[target_port].LED_Gen2_Setting[i].LEDSetting);

    /* Send out command and LED data */
    LED_Gen2_Transfer_Data(target_port);
}

void LED_Gen2_Enable_Control(uint8_t target_port, uint8_t enable)
{
    /* Enable LED Gen2 control */
    if(enable == TRUE)
    {
        if(LED_Gen2_Port_Setting[target_port].Strip_Count != 0)
        {
            /* Use LED Gen2 */
            LED_Gen2_Port_Setting[target_port].Use_Gen2 = TRUE;

            /* Initial LED Gen2 Setting */
            LED_Gen2_LLSI_PDMA_Init(target_port);
        }
        else
        {
            /* Disable LED Gen2 */
            LED_Gen2_Port_Setting[target_port].Use_Gen2 = FALSE;
        }
    }
    else
    {
        /* Use LED Gen1 */
        LED_Gen2_Port_Setting[target_port].Use_Gen2 = FALSE;

        /* Clean all ID */
        LED_Gen2_Set_Ctrl_Setting(target_port);
        LED_Gen2_Control_CMD_ID(CMD_CLEAN_ID, 0x00);
        while(*Gen2_Ctrl.fPDMA_Done == 0);

        /* Clear LLSI interrupt count */
        LED_Gen2_Port_Setting[target_port].LLSI_INT_Count = 0;

        /* Initial target port */
        LLSI_Initial(target_port);
        PDMA_Initial(target_port);
    }
}

void LED_Gen2_Capture_Rising(void)
{
#if (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_TIMER)
    /* Get Pulse Response, Reset Timer to Count Pulse Width */
    CAPTURE_TIMER->CNT = 0;
#elif (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_BPWM)
    /* Get Pulse Response, Reset BPWM to Count Pulse Width */
    CAPTURE_BPWM->CNTCLR = BPWM_CNTCLR_CNTCLR0_Msk;
#endif
}

void LED_Gen2_Capture_Falling(void)
{
#if (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_TIMER)
    /* Get Timer Count */
    LED_Gen2_ACK_Width[LED_Gen2_ACK_Count++] = TIMER0->CNT;

    if(LED_Gen2_One_ACK_Flag)
    {
        /* Stop Timer */
        TIMER_Stop(CAPTURE_TIMER);
        LED_Gen2_ACK_Done_Flag = 1;
    }
    else
    {
        /* Reset Timer */
        CAPTURE_TIMER->CNT = 0;
    }
#elif (LED_GEN2_CAPTURE_SOURCE == LED_GEN2_CAPTURE_BPWM)
    /* Get BPWM Count */
    LED_Gen2_ACK_Width[LED_Gen2_ACK_Count++] = CAPTURE_BPWM->CNT & BPWM_CNT_CNT_Msk;

    if(LED_Gen2_One_ACK_Flag)
    {
        /* Stop BPWM */
        BPWM_ForceStop(CAPTURE_BPWM, (uint32_t)NULL);
        CAPTURE_BPWM->CNTCLR = BPWM_CNTCLR_CNTCLR0_Msk;
        LED_Gen2_ACK_Done_Flag = 1;
    }
    else
    {
        /* Reset BPWM */
        CAPTURE_BPWM->CNTCLR = BPWM_CNTCLR_CNTCLR0_Msk;
    }
#endif
}

void GPB_IRQHandler(void)
{
    /* LLSI0 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[0].IO_Port, LED_LLSI_IO_Setting[0].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[0].IO_Port, LED_LLSI_IO_Setting[0].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[0].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }

    /* LLSI1 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[1].IO_Port, LED_LLSI_IO_Setting[1].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[1].IO_Port, LED_LLSI_IO_Setting[1].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[1].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }

    /* LLSI4 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[4].IO_Port, LED_LLSI_IO_Setting[4].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[4].IO_Port, LED_LLSI_IO_Setting[4].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[4].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }

    /* LLSI5 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[5].IO_Port, LED_LLSI_IO_Setting[5].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[5].IO_Port, LED_LLSI_IO_Setting[5].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[5].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }

    /* LLSI6 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[6].IO_Port, LED_LLSI_IO_Setting[6].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[6].IO_Port, LED_LLSI_IO_Setting[6].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[6].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }

    /* LLSI7 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[7].IO_Port, LED_LLSI_IO_Setting[7].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[7].IO_Port, LED_LLSI_IO_Setting[7].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[7].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }
}

void GPC_IRQHandler(void)
{
    /* LLSI2 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[2].IO_Port, LED_LLSI_IO_Setting[2].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[2].IO_Port, LED_LLSI_IO_Setting[2].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[2].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }

    /* LLSI3 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[3].IO_Port, LED_LLSI_IO_Setting[3].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[3].IO_Port, LED_LLSI_IO_Setting[3].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[3].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }

    /* LLSI8 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[8].IO_Port, LED_LLSI_IO_Setting[8].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[8].IO_Port, LED_LLSI_IO_Setting[8].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[8].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }

    /* LLSI9 */
    if(GPIO_GET_INT_FLAG(LED_LLSI_IO_Setting[9].IO_Port, LED_LLSI_IO_Setting[9].IO_Bit))
    {
        GPIO_CLR_INT_FLAG(LED_LLSI_IO_Setting[9].IO_Port, LED_LLSI_IO_Setting[9].IO_Bit);
    
        /* Rising Edge Trigger */
        if(*((volatile uint32_t *)(GPIO_PIN_DATA_BASE + LED_LLSI_IO_Setting[9].IO_PDIO_OFFSET)))
            LED_Gen2_Capture_Rising();
        /* Falling Edge Trigger */
        else
            LED_Gen2_Capture_Falling();
    }
}
