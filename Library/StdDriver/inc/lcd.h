/**************************************************************************//**
 * @file     lcd.h
 * @version  V3.00
 * @brief    Liquid-Crystal Display(LCD) driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __LCD_H__
#define __LCD_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LCD_Driver LCD Driver
  @{
*/

/** @addtogroup LCD_EXPORTED_CONSTANTS LCD Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  LCD Bias Voltage Level Selection Constant Definitions                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_BIAS_LV_1_2                 (1ul << LCD_PSET_BIAS_Pos) /*!< LCD bias voltage level selection - 1/2 Bias \hideinitializer */
#define LCD_BIAS_LV_1_3                 (2ul << LCD_PSET_BIAS_Pos) /*!< LCD bias voltage level selection - 1/3 Bias \hideinitializer */
#define LCD_BIAS_LV_1_4                 (3ul << LCD_PSET_BIAS_Pos) /*!< LCD bias voltage level selection - 1/4 Bias \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LCD COM Duty Ratio Selection Constant Definitions                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_COM_DUTY_1_1                (0ul << LCD_PSET_DUTY_Pos) /*!< LCD com duty ratio selection - 1/1 Duty \hideinitializer */
#define LCD_COM_DUTY_1_2                (1ul << LCD_PSET_DUTY_Pos) /*!< LCD com duty ratio selection - 1/2 Duty \hideinitializer */
#define LCD_COM_DUTY_1_3                (2ul << LCD_PSET_DUTY_Pos) /*!< LCD com duty ratio selection - 1/3 Duty \hideinitializer */
#define LCD_COM_DUTY_1_4                (3ul << LCD_PSET_DUTY_Pos) /*!< LCD com duty ratio selection - 1/4 Duty \hideinitializer */
#define LCD_COM_DUTY_1_5                (4ul << LCD_PSET_DUTY_Pos) /*!< LCD com duty ratio selection - 1/5 Duty \hideinitializer */
#define LCD_COM_DUTY_1_6                (5ul << LCD_PSET_DUTY_Pos) /*!< LCD com duty ratio selection - 1/6 Duty \hideinitializer */
#define LCD_COM_DUTY_1_7                (6ul << LCD_PSET_DUTY_Pos) /*!< LCD com duty ratio selection - 1/7 Duty \hideinitializer */
#define LCD_COM_DUTY_1_8                (7ul << LCD_PSET_DUTY_Pos) /*!< LCD com duty ratio selection - 1/8 Duty \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LCD Waveform Attribute Selection Constant Definitions                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_WAVEFORM_TYPE_A_NORMAL      (0ul << LCD_PSET_TYPE_Pos) /*!< LCD waveform Type-A, no inverse \hideinitializer */
#define LCD_WAVEFORM_TYPE_B_NORMAL      (1ul << LCD_PSET_TYPE_Pos) /*!< LCD waveform Type-B, no inverse \hideinitializer */
#define LCD_WAVEFORM_TYPE_A_INVERSE     (2ul << LCD_PSET_TYPE_Pos) /*!< LCD waveform Type-A and inverse \hideinitializer */
#define LCD_WAVEFORM_TYPE_B_INVERSE     (3ul << LCD_PSET_TYPE_Pos) /*!< LCD waveform Type-B and inverse \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LCD Charge Pump Voltage Selection Constant Definitions                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_CP_VOLTAGE_VL1_100          (0ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.00 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_105          (1ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.05 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_110          (2ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.10 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_115          (3ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.15 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_120          (4ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.20 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_125          (5ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.25 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_130          (6ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.30 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_135          (7ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.35 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_140          (8ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.40 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_145          (9ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.45 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_150          (10ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.50 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_155          (11ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.55 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_160          (12ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.60 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_165          (13ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.65 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_170          (14ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.70 V \hideinitializer */
#define LCD_CP_VOLTAGE_VL1_175          (15ul << LCD_ASET1_VL1SEL_Pos) /*!< Select LCD charge pump voltage VL1 1.75 V \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LCD Interrupt Source Constant Definitions                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_DISABLE_ALL_INT             (0ul << LCD_INTEN_FIECEND_Pos) /*!< Disable all LCD interrupt sources \hideinitializer */
#define LCD_FRAME_COUNTING_END_INT      (1ul << LCD_INTEN_FIECEND_Pos) /*!< Indicate frame count end interrupt \hideinitializer */
#define LCD_FRAME_END_INT               (1ul << LCD_INTEN_FIEEND_Pos)  /*!< Indicate frame end interrupt \hideinitializer */
#define LCD_ENABLE_ALL_INT              (3ul << LCD_INTEN_FIECEND_Pos) /*!< Enable all LCD interrupt sources \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LCD Operation Voltage Source Constant Definitions                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_VOLTAGE_SOURCE_CP           (0ul << LCD_ASET1_PSTRUC_Pos) /*!< LCD voltage source from built-in charge pump \hideinitializer */
#define LCD_VOLTAGE_SOURCE_VLCD_R_MODE  (1ul << LCD_ASET1_PSTRUC_Pos) /*!< LCD voltage source from external VLCD power with external resistor divider \hideinitializer */
#define LCD_VOLTAGE_SOURCE_VLCD_C_MODE  (2ul << LCD_ASET1_PSTRUC_Pos) /*!< LCD voltage source from external VLCD with capacitor split \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LCD Output Control Constant Definitions                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_OUTPUT_SEL9_COM4_TO_SEG43   (1ul << LCD_OSET_SEL9_Pos)  /*!< The output SEL9  is SEG43 \hideinitializer */
#define LCD_OUTPUT_SEL10_COM5_TO_SEG42  (1ul << LCD_OSET_SEL10_Pos) /*!< The output SEL10 is SEG42 \hideinitializer */
#define LCD_OUTPUT_SEL11_SEG20_TO_COM0  (1ul << LCD_OSET_SEL11_Pos) /*!< The output SEL11 is COM0 \hideinitializer */
#define LCD_OUTPUT_SEL12_SEG19_TO_COM1  (1ul << LCD_OSET_SEL12_Pos) /*!< The output SEL12 is COM1 \hideinitializer */
#define LCD_OUTPUT_SEL13_SEG18_TO_COM2  (1ul << LCD_OSET_SEL13_Pos) /*!< The output SEL13 is COM2 \hideinitializer */
#define LCD_OUTPUT_SEL14_SEG17_TO_COM3  (1ul << LCD_OSET_SEL14_Pos) /*!< The output SEL14 is COM3 \hideinitializer */
#define LCD_OUTPUT_SEL15_COM6_TO_SEG41  (1ul << LCD_OSET_SEL15_Pos) /*!< The output SEL15 is SEG41 \hideinitializer */
#define LCD_OUTPUT_SEL16_COM7_TO_SEG40  (1ul << LCD_OSET_SEL16_Pos) /*!< The output SEL16 is COM40 \hideinitializer */
#define LCD_OUTPUT_SEL24_SEG31_TO_COM4  (1ul << LCD_OSET_SEL24_Pos) /*!< The output SEL24 is COM4 \hideinitializer */
#define LCD_OUTPUT_SEL25_SEG30_TO_COM5  (1ul << LCD_OSET_SEL25_Pos) /*!< The output SEL25 is COM5 \hideinitializer */
#define LCD_OUTPUT_SEL26_SEG29_TO_COM6  (1ul << LCD_OSET_SEL26_Pos) /*!< The output SEL26 is COM6 \hideinitializer */
#define LCD_OUTPUT_SEL27_SEG28_TO_COM7  (1ul << LCD_OSET_SEL27_Pos) /*!< The output SEL27 is COM7 \hideinitializer */
#define LCD_OUTPUT_SEL28_SEG27_TO_COM2  (1ul << LCD_OSET_SEL28_Pos) /*!< The output SEL28 is COM2 \hideinitializer */
#define LCD_OUTPUT_SEL29_SEG26_TO_COM3  (1ul << LCD_OSET_SEL29_Pos) /*!< The output SEL29 is COM3 \hideinitializer */
#define LCD_OUTPUT_SEL37_SEG18_TO_COM6  (1ul << LCD_OSET_SEL37_Pos) /*!< The output SEL37 is COM6 \hideinitializer */
#define LCD_OUTPUT_SEL37_SEG18_TO_SEG45 (2ul << LCD_OSET_SEL37_Pos) /*!< The output SEL37 is SEG45 \hideinitializer */
#define LCD_OUTPUT_SEL38_SEG17_TO_COM7  (1ul << LCD_OSET_SEL38_Pos) /*!< The output SEL38 is COM7 \hideinitializer */
#define LCD_OUTPUT_SEL38_SEG17_TO_SEG44 (2ul << LCD_OSET_SEL38_Pos) /*!< The output SEL38 is SEG44 \hideinitializer */
#define LCD_OUTPUT_SEL39_SEG14_TO_COM0  (1ul << LCD_OSET_SEL39_Pos) /*!< The output SEL39 is COM0 \hideinitializer */
#define LCD_OUTPUT_SEL40_SEG13_TO_COM1  (1ul << LCD_OSET_SEL40_Pos) /*!< The output SEL42 is COM1 \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LCD Pause Type Constant Definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_FRAME_PAUSE                 (0ul << LCD_FSET_PTYPE_Pos)  /*!< Select frame pause type \hideinitializer */
#define LCD_DUTY_PAUSE                  (1ul << LCD_FSET_PTYPE_Pos)  /*!< Select duty pause type \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LCD Power Saving Level Constant Definitions                                                            */
/*---------------------------------------------------------------------------------------------------------*/
enum
{
    LCD_PWR_SAVING_LEVEL0 = 0,
    LCD_PWR_SAVING_LEVEL1,
    LCD_PWR_SAVING_LEVEL2,
    LCD_PWR_SAVING_LEVEL3
};

/*@}*/ /* end of group LCD_EXPORTED_CONSTANTS */


/** @addtogroup LCD_EXPORTED_STRUCTS LCD Exported Structs
  @{
*/
/**
  * @details    LCD Configuration Data Struct
  */
typedef struct
{
    uint32_t u32SrcFreq;        /*!< LCD clock source frequency */
    uint32_t u32ComDuty;        /*!< COM duty */
    uint32_t u32BiasLevel;      /*!< Bias level */
    uint32_t u32Framerate;      /*!< Operation frame rate */
    uint32_t u32WaveformType;   /*!< Waveform type */
    uint32_t u32IntSrc;         /*!< Interrupt source */
    uint32_t u32VL1Select;      /*!< VL1 voltage select */
    uint32_t u32VSrc;           /*!< Voltage source */
} S_LCD_CFG_T;

/*@}*/ /* end of group LCD_EXPORTED_STRUCTS */

/** @addtogroup LCD_EXPORTED_MACROS LCD Exported Macros
  @{
*/

/**
  * @brief      Enable LCD Display
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro is used to enable LCD display.
  */
#define LCD_ENABLE_DISPLAY()        do{ LCD->CTL |= LCD_CTL_LCDEN_Msk; while(LCD->CTL & LCD_CTL_LCDENIND_Msk) {} }while(0)

/**
  * @brief      Disable LCD Display
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro is used to disable LCD display.
  */
#define LCD_DISABLE_DISPLAY()       do{ LCD->CTL &= ~LCD_CTL_LCDEN_Msk; while(LCD->CTL & LCD_CTL_LCDENIND_Msk) {} }while(0)

/**
  * @brief      Set LCD Waveform Type
  *
  * @param[in]  type       The LCD waveform type. It could be one of the following type
  *                             - \ref LCD_WAVEFORM_TYPE_A_NORMAL
  *                             - \ref LCD_WAVEFORM_TYPE_B_NORMAL
  *                             - \ref LCD_WAVEFORM_TYPE_A_INVERSE
  *                             - \ref LCD_WAVEFORM_TYPE_B_INVERSE
  *
  * @return     None
  *
  * @details    This macro is used to set the attribute of LCD output waveform.
  */
#define LCD_WAVEFORM_TYPE(type)     (LCD->PSET = (LCD->PSET & ~(LCD_PSET_TYPE_Msk|LCD_PSET_INV_Msk)) | (type))

/**
  * @brief      Set LCD Source Clock Divider
  *
  * @param[in]  div         The frequency divider, valid value is between 1 to 1024.
  *
  * @return     None
  *
  * @details    This macro is used to set the LCD operarion frequency is (LCD source frequency / div).
  */
#define LCD_SET_FREQDIV(div)        (LCD->PSET = (LCD->PSET & ~LCD_PSET_LCDDIV_Msk) | (((div)-1) << LCD_PSET_LCDDIV_Pos))

/**
  * @brief      Set Charge Pump Voltage
  *
  * @param[in]  voltage     The target charge pump voltage. It could be one of the following voltage level
  *                             - \ref LCD_CP_VOLTAGE_VL1_100, VL1 = 1.00 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_105, VL1 = 1.05 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_110, VL1 = 1.10 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_115, VL1 = 1.15 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_120, VL1 = 1.20 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_125, VL1 = 1.25 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_130, VL1 = 1.30 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_135, VL1 = 1.35 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_140, VL1 = 1.40 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_145, VL1 = 1.45 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_150, VL1 = 1.50 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_155, VL1 = 1.55 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_160, VL1 = 1.60 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_165, VL1 = 1.65 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_170, VL1 = 1.70 V
  *                             - \ref LCD_CP_VOLTAGE_VL1_175, VL1 = 1.75 V
  *
  * @return     None
  *
  * @details    This macro is used to set charge pump voltage for VLCD.
  */
#define LCD_SET_CP_VOLTAGE(voltage) (LCD->ASET1 = (LCD->ASET1 & ~LCD_ASET1_VL1SEL_Msk) | (voltage))

/**
  * @brief      Decrease Charge Pump Voltage
  *
  * @param[in]  unit        The tuning units, valid value is between 1 to 7.
  *                         One unit of voltage is about 0.04V, and the charge pump voltage is decreased (unit * 0.04)V.
  *
  * @return     None
  *
  * @details    This macro is used to decrease charge pump voltage by specific units.
  */
#define LCD_CP_VOLTAGE_DECREASE(unit)   (LCD->PSET = (LCD->PSET & ~LCD_PSET_VTUNE_Msk) | ((unit) << LCD_PSET_VTUNE_Pos))

/**
  * @brief      Increase Charge Pump Voltage
  *
  * @param[in]  unit        The tuning units, valid value is between 1 to 8.
  *                         One unit of voltage is about 0.04V, and the charge pump voltage is increased (unit * 0.04)V.
  *
  * @return     None
  *
  * @details    This macro is used to increase charge pump voltage by specific units.
  */
#define LCD_CP_VOLTAGE_INCREASE(unit)   (LCD->PSET = (LCD->PSET & ~LCD_PSET_VTUNE_Msk) | (16-(unit)) << LCD_PSET_VTUNE_Pos)

/**
  * @brief      Set LCD Blinking ON
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro is used to enable LCD blinking.
  */
#define LCD_BLINKING_ON()       (LCD->FSET |= LCD_FSET_BLINK_Msk)

/**
  * @brief      Set LCD Blinking OFF
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro is used to disable LCD blinking.
  */
#define LCD_BLINKING_OFF()      (LCD->FSET &= ~LCD_FSET_BLINK_Msk)

/**
  * @brief      Set LCD Frame Counting Value
  *
  * @param[in]  value       Frame counting value. Valid value is between 1 to 1024.
  *
  * @return     None
  *
  * @details    This macro is used to set the LCD frame counting value to configure the blink interval.
  * @note       For type-B waveform, the frame counter increases at the end of odd frames, not even frames.
  */
#define LCD_SET_FRAME_COUNTING_VALUE(value)     (LCD->FSET = (LCD->FSET & ~LCD_FSET_FCV_Msk) | (((value)-1) << LCD_FSET_FCV_Pos))

/**
  * @brief      Set Pause Type
  *
  * @param[in]  type        The LCD pause type. It could be one of the following type.
  *                             - \ref LCD_FRAME_PAUSE
  *                             - \ref LCD_DUTY_PAUSE
  *
  * @return     None
  *
  * @details    This macro is used to select LCD pause type.
  */
#define LCD_SET_PAUSE_TYPE(type)       (LCD->FSET = (LCD->FSET & ~LCD_FSET_PTYPE_Msk) | (type))

/**
  * @brief      Set Pause Time
  *
  * @param[in]  unit       The pause time, valid setting is between 0 to 15.
  *
  * @return     None
  *
  * @details    This macro is used to specify the number of pause time units to insert per frame or duty.
  */
#define LCD_SET_PAUSE_TIME(unit)     (LCD->FSET = (LCD->FSET & ~LCD_FSET_PTIME_Msk) | ((unit) << LCD_FSET_PTIME_Pos))

/**
  * @brief      Select LCD Voltage Source
  *
  * @param[in]  source      The LCD operation voltage source. It could be one of the following source
  *                             - \ref LCD_VOLTAGE_SOURCE_CP
  *                             - \ref LCD_VOLTAGE_SOURCE_VLCD_R_MODE
  *                             - \ref LCD_VOLTAGE_SOURCE_VLCD_C_MODE
  *
  * @return     None
  *
  * @details    This macro is used to select LCD operation voltage source.
  */
#define LCD_VOLTAGE_SOURCE(source)          (LCD->ASET1 = (LCD->ASET1 & ~LCD_ASET1_PSTRUC_Msk) | (source) | LCD_ASET1_LCDPEN_Msk)

/**
  * @brief      Enable LCD Frame Counting End Interrupt
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro is used to enable frame count end interrupt function.
  */
#define LCD_ENABLE_FRAME_COUNTING_END_INT()     (LCD->INTENN |= LCD_INTEN_FCEND_Msk)

/**
  * @brief      Disable LCD Frame Counting End Interrupt
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro is used to disable frame count end interrupt function.
  */
#define LCD_DISABLE_FRAME_COUNTING_END_INT()    (LCD->INTENN &= ~LCD_INTEN_FCEND_Msk)

/**
  * @brief      Enable LCD Frame End Interrupt
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro is used to enable frame end interrupt function.
  */
#define LCD_ENABLE_FRAME_END_INT()          (LCD->INTENN |= LCD_INTEN_FEND_Msk)

/**
  * @brief      Disable LCD Frame End Interrupt
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro is used to disable frame end interrupt function.
  */
#define LCD_DISABLE_FRAME_END_INT()         (LCD->INTENN &= ~LCD_INTEN_FEND_Msk)

/**
  * @brief      Get LCD Frame Counting End Flag
  *
  * @param      None
  *
  * @retval     0   Frame count end flag did not occur
  * @retval     1   Frame count end flag occurred
  *
  * @details    This macro gets frame count end flag.
  */
#define LCD_GET_FRAME_COUNTING_END_FLAG()       ((LCD->STS & LCD_STS_FSCEND_Msk)? 1UL : 0UL)

/**
  * @brief      Clear LCD Frame Counting End Flag
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro clears frame count end flag.
  */
#define LCD_CLEAR_FRAME_COUNTING_END_FLAG()     (LCD->STS = LCD_STS_FSCEND_Msk)

/**
  * @brief      Get LCD Frame End Flag
  *
  * @param      None
  *
  * @retval     0   Frame end flag did not occur
  * @retval     1   Frame end flag occurred
  *
  * @details    This macro gets frame end flag.
  */
#define LCD_GET_FRAME_END_FLAG()                ((LCD->STS & LCD_STS_FSCEND_Msk)? 1UL : 0UL)

/**
  * @brief      Clear LCD Frame End Flag
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This macro clears frame end flag.
  */
#define LCD_CLEAR_FRAME_END_FLAG()              (LCD->STS = LCD_STS_FSCEND_Msk)

/**
  * @brief      Set Output select
  *
  * @param      sel       The output function select value. Reference LCD Output Control Constant Definitions.
  *
  * @return     None
  *
  * @details    This macro is used to set lcd output pin function.
  */
#define LCD_OUTPUT_SET(sel)              (LCD->OSET |= (sel))

/**
  * @brief      Reset Output select
  *
  * @param      sel:      The output function select mask.
  *
  * @return     None
  *
  * @details    This macro is used to reset lcd output pin function to default.
  */
#define LCD_OUTPUT_RST(sel)              (LCD->OSET &= (~(sel)))

/*@}*/ /* end of group LCD_EXPORTED_MACROS */

/** @addtogroup LCD_EXPORTED_FUNCTIONS LCD Exported Functions
  @{
*/
uint32_t LCD_Open(S_LCD_CFG_T *pLCDSET);
void LCD_Close(void);
void LCD_SetPixel(uint32_t u32Com, uint32_t u32Seg, uint32_t u32OnFlag);
void LCD_SetAllPixels(uint32_t u32OnOff);
uint32_t LCD_EnableBlink(uint32_t u32ms);
void LCD_DisableBlink(void);
void LCD_EnableInt(uint32_t u32IntSrc);
void LCD_DisableInt(uint32_t u32IntSrc);

/*@}*/ /* end of group LCD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group LCD_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __LCD_H__ */
