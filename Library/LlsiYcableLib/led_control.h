/******************************************************************************
 * @file     led_control.h
 * @version  V3.00
 * @brief    Control LED lighting effects header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __LED_CONTROL_H__
#define __LED_CONTROL_H__

#include <stdint.h>

struct LED_Setting_Tag;

void LLSI_Initial(uint8_t port);
void PDMA_Initial(uint8_t port);
void Set_Single(uint8_t *LED_DATA, uint32_t TotalLED, uint32_t Offset, uint8_t Data_R, uint8_t Data_G, uint8_t Data_B);
void Set_Array(uint8_t *LED_DATA, uint32_t TotalLED, uint32_t Offset, uint8_t *DisplayData, uint8_t MaxBright);
void Set_InverseArray(uint8_t *LED_DATA, uint32_t TotalLED, uint32_t Offset, uint8_t *DisplayData, uint8_t MaxBright);
void Set_LED_Data(volatile struct LED_Setting_Tag *LED_Setting);
void Clear_LED_Data(volatile struct LED_Setting_Tag *LED_Setting);
void LLSI_Underflow_Handler(uint8_t target_port);
void Polling_PDMA_Abort(void);

/*------Lighting Mode-----------------------------------------*/
void FUNC_Off(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Static(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Breathing(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Cycling(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Random(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Strobe(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Music(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Wave(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Spring(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Water(volatile struct LED_Setting_Tag *LED_Setting);
void FUNC_Rainbow(volatile struct LED_Setting_Tag *LED_Setting);

#endif  /* __LED_CONTROL_H__ */
