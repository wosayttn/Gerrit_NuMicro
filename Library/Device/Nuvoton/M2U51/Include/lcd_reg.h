/**************************************************************************//**
 * @file     lcd_reg.h
 * @version  V1.00
 * @brief    LCD register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __LCD_REG_H__
#define __LCD_REG_H__

/**
    @addtogroup REGISTER Control Register
    @{
*/

/*---------------------- Liquid-Crystal Display -------------------------*/
/**
    @addtogroup LCD Liquid-Crystal Display(LCD)
    Memory Mapped Structure for LCD Controller
@{ */

typedef struct
{

    /**
     * @var LCD_T::CTL
     * Offset: 0x00  LCD Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LCDEN     |LCD Operation Enable Bit
     * |        |          |0 = LCD operation Disabled.
     * |        |          |1 = LCD operation Enabled.
     * |[31]    |LCDENIND  |LCD Operation Enable Indicator (Read Only)
     * |        |          |This bit is LCD operation enable and disable indicator.
     * |        |          |0 = When LCDEN changed, LCD operation Enable/Disable is completed.
     * |        |          |1 = When LCDEN changed, LCD operation Enable/Disable is processing.
     * |        |          |Note: This bit is read only.
     * @var LCD_T::PSET
     * Offset: 0x04  LCD Panel Setting Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |BIAS      |LCD Bias Level Selection
     * |        |          |This field is used to select the LCD bias level.
     * |        |          |00 = Reserved.
     * |        |          |01 = 1/2 Bias.
     * |        |          |10 = 1/3 Bias.
     * |        |          |11 = 1/4 Bias.
     * |        |          |Note: The register can't be changed after LCD enable.
     * |[4:2]   |DUTY      |LCD Duty Ratio Selection
     * |        |          |This field is used to select the LCD duty ratio.
     * |        |          |000 = 1/1 Duty.
     * |        |          |001 = 1/2 Duty.
     * |        |          |010 = 1/3 Duty.
     * |        |          |011 = 1/4 Duty.
     * |        |          |100 = 1/5 Duty.
     * |        |          |101 = 1/6 Duty.
     * |        |          |110 = 1/7 Duty.
     * |        |          |111 = 1/8 Duty.
     * |        |          |Note: The register can't be changed after LCD enable.
     * |[5]     |TYPE      |LCD Waveform Type Selection
     * |        |          |This bit is used to select the LCD waveform type.
     * |        |          |0 = LCD waveform type A.
     * |        |          |1 = LCD waveform type B.
     * |        |          |Note: The register can't be changed after LCD enable.
     * |[6]     |INV       |LCD Waveform Inverse Control
     * |        |          |This bit is used to inverse LCD waveform.
     * |        |          |0 = LCD waveform inverse function Disabled.
     * |        |          |1 = LCD waveform inverse function Enabled.
     * |        |          |Note: The register can¡¦t be changed after LCD enabled.
     * |[17:8]  |LCDDIV    |LCD Operating Clock Divide Number from LCDCLK Clock Source
     * |        |          |LCDOPCLK = LCDCLK / (LCDDIV + 1).
     * |        |          |Note: The register can't be changed after LCD enable.
     * @var LCD_T::FSET
     * Offset: 0x08  LCD Frame Setting Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BLINK     |LCD Blinking Function Enable Bit
     * |        |          |0 = LCD blinking function Disabled.
     * |        |          |1 = LCD blinking function Enabled.
     * |[17:8]  |FCV       |Frame Counter Comparison Value
     * |        |          |This field indicates the frame counter comparison value.
     * @var LCD_T::OSET
     * Offset: 0x10  LCD Output Setting Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SEL9      |LCD_PIN9 Output Selection
     * |        |          |0 = LCD_COM4 is shown on LCD_PIN9.
     * |        |          |1 = LCD_SEG43 is shown on LCD_PIN9.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[1]     |SEL10     |LCD_PIN10 Output Selection
     * |        |          |0 = LCD_COM5 is shown on LCD_PIN10.
     * |        |          |1 = LCD_SEG42 is shown on LCD_PIN10.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[2]     |SEL11     |LCD_PIN11 Output Selection
     * |        |          |0 = LCD_SEG20 is shown on LCD_PIN11.
     * |        |          |1 = LCD_COM0 is shown on LCD_PIN11.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[3]     |SEL12     |LCD_PIN12 Output Selection
     * |        |          |0 = LCD_SEG19 is shown on LCD_PIN12.
     * |        |          |1 = LCD_COM1 is shown on LCD_PIN12.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[4]     |SEL13     |LCD_PIN13 Output Selection
     * |        |          |0 = LCD_SEG18 is shown on LCD_PIN13.
     * |        |          |1 = LCD_COM2 is shown on LCD_PIN13.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[5]     |SEL14     |LCD_PIN14 Output Selection
     * |        |          |0 = LCD_SEG17 is shown on LCD_PIN14.
     * |        |          |1 = LCD_COM3 is shown on LCD_PIN14.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[6]     |SEL15     |LCD_PIN15 Output Selection
     * |        |          |0 = LCD_COM6 is shown on LCD_PIN15.
     * |        |          |1 = LCD_SEG41 is shown on LCD_PIN15.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[7]     |SEL16     |LCD_PIN16 Output Selection
     * |        |          |0 = LCD_COM7 is shown on LCD_PIN16.
     * |        |          |1 = LCD_SEG40 is shown on LCD_PIN16.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[8]     |SEL24     |LCD_PIN24 Output Selection
     * |        |          |0 = LCD_SEG31 is shown on LCD_PIN24.
     * |        |          |1 = LCD_COM4 is shown on LCD_PIN24.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[9]     |SEL25     |LCD_PIN25 Output Selection
     * |        |          |0 = LCD_SEG30 is shown on LCD_PIN25.
     * |        |          |1 = LCD_COM5 is shown on LCD_PIN25.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[10]    |SEL26     |LCD_PIN26 Output Selection
     * |        |          |0 = LCD_SEG29 is shown on LCD_PIN26.
     * |        |          |1 = LCD_COM6 is shown on LCD_PIN26.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[11]    |SEL27     |LCD_PIN27 Output Selection
     * |        |          |0 = LCD_SEG28 is shown on LCD_PIN27.
     * |        |          |1 = LCD_COM7 is shown on LCD_PIN27.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[12]    |SEL28     |LCD_PIN28 Output Selection
     * |        |          |0 = LCD_SEG27 is shown on LCD_PIN28.
     * |        |          |1 = LCD_COM2 is shown on LCD_PIN28.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[13]    |SEL29     |LCD_PIN29 Output Selection
     * |        |          |0 = LCD_SEG26 is shown on LCD_PIN29.
     * |        |          |1 = LCD_COM3 is shown on LCD_PIN29.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[15:14] |SEL37     |LCD_PIN37 Output Selection
     * |        |          |00 = LCD_SEG18 is shown on LCD_PIN37.
     * |        |          |01 = LCD_COM6 is shown on LCD_PIN37.
     * |        |          |10 = LCD_SEG45 is shown on LCD_PIN37.
     * |        |          |11 = Reserved.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[17:16] |SEL38     |LCD_PIN38 Output Selection
     * |        |          |00 = LCD_SEG17 is shown on LCD_PIN38.
     * |        |          |01 = LCD_COM7 is shown on LCD_PIN38.
     * |        |          |10 = LCD_SEG44 is shown on LCD_PIN38.
     * |        |          |11 = Reserved.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[18]    |SEL39     |LCD_PIN39 Output Selection
     * |        |          |0 = LCD_SEG14 is shown on LCD_PIN39 .
     * |        |          |1 = LCD_COM0 is shown on LCD_PIN39.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * |[19]    |SEL40     |LCD_PIN40 Output Selection
     * |        |          |0 = LCD_SEG13 is shown on LCD_PIN40.
     * |        |          |1 = LCD_COM1 is shown on LCD_PIN40.
     * |        |          |Note 1: The register can't be changed after LCD enable.
     * |        |          |Note 2: Please refer to Error! Reference source not found..
     * @var LCD_T::STS
     * Offset: 0x14  LCD Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FSCEND    |Frame Counter Interrupt Flag
     * |        |          |0 = No frame counter interrupt event.
     * |        |          |1 = Frame counter interrupt event occurred.
     * |        |          |Note: This bit can be written 1 to clear.
     * |[1]     |FSEND     |Frame End Interrupt Flag
     * |        |          |0 = No frame end interrupt event.
     * |        |          |1 = Frame end interrupt event occurred.
     * |        |          |Note: This bit can be written 1 to clear.
     * @var LCD_T::INTEN
     * Offset: 0x18  LCD Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FIECEND   |Frame Counter Interrupt Enable Bit
     * |        |          |This bit is used to enable the LCD interrupt when frame counter interrupt event occurred.
     * |        |          |0 = Frame counter interrupt Disabled.
     * |        |          |1 = Frame counter interrupt Enabled.
     * |[1]     |FIEEND    |Frame End Interrupt Enable Bit
     * |        |          |This bit is used to enable the LCD interrupt when frame end interrupt event occurred.
     * |        |          |0 = Frame end interrupt Disabled.
     * |        |          |1 = Frame end interrupt Enabled.

     * @var LCD_T::SEGDAT
     * Offset: 0x20  LCD Segment Display Data Register 0 (SEG0 ~ SEG3)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG0      |SEG0 Display Data
     * |        |          |This bit field indicates the display data of SEG0 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG0[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG1      |SEG1 Display Data
     * |        |          |This bit field indicates the display data of SEG1 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG1[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG2      |SEG2 Display Data
     * |        |          |This bit field indicates the display data of SEG2 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG2[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG3      |SEG3 Display Data
     * |        |          |This bit field indicates the display data of SEG3 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG3[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x24  LCD Segment Display Data Register 1 (SEG4 ~ SEG7)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG4      |SEG4 Display Data
     * |        |          |This bit field indicates the display data of SEG4 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG4[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG5      |SEG5 Display Data
     * |        |          |This bit field indicates the display data of SEG5 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG5[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG6      |SEG6 Display Data
     * |        |          |This bit field indicates the display data of SEG6 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG6[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG7      |SEG7 Display Data
     * |        |          |This bit field indicates the display data of SEG7 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG7[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x28  LCD Segment Display Data Register 2 (SEG8 ~ SEG11)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG8      |SEG8 Display Data
     * |        |          |This bit field indicates the display data of SEG8 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG8[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG9      |SEG9 Display Data
     * |        |          |This bit field indicates the display data of SEG9 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG9[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG10     |SEG10 Display Data
     * |        |          |This bit field indicates the display data of SEG10 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG10[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG11     |SEG11 Display Data
     * |        |          |This bit field indicates the display data of SEG11 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG11[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x2C  LCD Segment Display Data Register 3 (LCD SEG12 ~ SEG15)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG12     |SEG12 Display Data
     * |        |          |This bit field indicates the display data of SEG12 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG12[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG13     |SEG13 Display Data
     * |        |          |This bit field indicates the display data of SEG13 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG13[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG14     |SEG14 Display Data
     * |        |          |This bit field indicates the display data of SEG14 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG14[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG15     |SEG15 Display Data
     * |        |          |This bit field indicates the display data of SEG15 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG15[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x30  LCD Segment Display Data Register 4 (LCD SEG16 ~ SEG19)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG16     |SEG16 Display Data
     * |        |          |This bit field indicates the display data of SEG16 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG16[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG17     |SEG17 Display Data
     * |        |          |This bit field indicates the display data of SEG17 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG17[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG18     |SEG18 Display Data
     * |        |          |This bit field indicates the display data of SEG18 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG18[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG19     |SEG19 Display Data
     * |        |          |This bit field indicates the display data of SEG19 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG19[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x34  LCD Segment Display Data Register 5 (SEG20 ~ SEG23)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG20     |SEG20 Display Data
     * |        |          |This bit field indicates the display data of SEG20 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG20[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG21     |SEG21 Display Data
     * |        |          |This bit field indicates the display data of SEG21 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG21[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG22     |SEG22 Display Data
     * |        |          |This bit field indicates the display data of SEG22 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG22[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG23     |SEG23 Display Data
     * |        |          |This bit field indicates the display data of SEG23 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG23[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x38  LCD Segment Display Data Register 6 (SEG24 ~ SEG27)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG24     |SEG24 Display Data
     * |        |          |This bit field indicates the display data of SEG24 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG24[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG25     |SEG25 Display Data
     * |        |          |This bit field indicates the display data of SEG25 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG25[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG26     |SEG26 Display Data
     * |        |          |This bit field indicates the display data of SEG26 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG26[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG27     |SEG27 Display Data
     * |        |          |This bit field indicates the display data of SEG27 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG27[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x3C  LCD Segment Display Data Register 7 (SEG28 ~ SEG31)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG28     |SEG28 Display Data
     * |        |          |This bit field indicates the display data of SEG28 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG28[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG29     |SEG29 Display Data
     * |        |          |This bit field indicates the display data of SEG29 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG29[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG30     |SEG30 Display Data
     * |        |          |This bit field indicates the display data of SEG30 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG30[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG31     |SEG31 Display Data
     * |        |          |This bit field indicates the display data of SEG31 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG31[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x40  LCD Segment Display Data Register 8 (SEG32 ~ SEG35)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG32     |SEG32 Display Data
     * |        |          |This bit field indicates the display data of SEG32 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG32[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG33     |SEG33 Display Data
     * |        |          |This bit field indicates the display data of SEG33 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG33[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG34     |SEG34 Display Data
     * |        |          |This bit field indicates the display data of SEG34 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG34[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG35     |SEG35 Display Data
     * |        |          |This bit field indicates the display data of SEG35 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG35[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x44  LCD Segment Display Data Register 9 (SEG36 ~ SEG39)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG36     |SEG36 Display Data
     * |        |          |This bit field indicates the display data of SEG36 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG36[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG37     |SEG37 Display Data
     * |        |          |This bit field indicates the display data of SEG37 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG37[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG38     |SEG38 Display Data
     * |        |          |This bit field indicates the display data of SEG38 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG38[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG39     |SEG39 Display Data
     * |        |          |This bit field indicates the display data of SEG39 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG39[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x48  LCD Segment Display Data Register 10 (SEG40 ~ SEG43)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG40     |SEG40 Display Data
     * |        |          |This bit field indicates the display data of SEG40 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG40[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG41     |SEG41 Display Data
     * |        |          |This bit field indicates the display data of SEG41 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG41[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[23:16] |SEG42     |SEG42 Display Data
     * |        |          |This bit field indicates the display data of SEG42 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG42[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[31:24] |SEG43     |SEG43 Display Data
     * |        |          |This bit field indicates the display data of SEG43 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG43[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * Offset: 0x4C  LCD Segment Display Data Register 11 (SEG44 ~ SEG45)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SEG44     |SEG44 Display Data
     * |        |          |This bit field indicates the display data of SEG44 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG44[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * |[15:8]  |SEG45     |SEG45 Display Data
     * |        |          |This bit field indicates the display data of SEG45 corresponding to COM0 ~ COM7.
     * |        |          |0 = This dot is inactive.
     * |        |          |1 = This dot is active.
     * |        |          |Note 1: Each bit of SEG45[n] is corresponding to COMn, n = 0 ~ 7.
     * |        |          |Note 2: Refer to Error! Reference source not found..
     * @var LCD_T::ASET1
     * Offset: 0x60  LCD Analog Setting Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LCDPEN    |LCD Power Enable Bit
     * |        |          |This bit is used to turn on the LCD power.
     * |        |          |0 = LCD power Disabled.
     * |        |          |1 = LCD power Enabled.
     * |        |          |Note: The register can't be changed after LCD enable.
     * |[2:1]   |PSTRUC    |LCD Power Structure Selection
     * |        |          |This bit field is used to select the LCD power structure.
     * |        |          |00 = Internal charge pump.
     * |        |          |01 = External resistance divider.
     * |        |          |10 = Capacitor split.
     * |        |          |11= Reserved.
     * |        |          |Note: The register can't be changed after LCD enable.
     * |[6:3]   |VL1SEL    |LCD Charge Pump Unit Voltage VL1 Selection
     * |        |          |This setting is available only when power structure is internal charge pump mode by setting PSTRUC (LCD_ASET1[2:1]) to "00b".
     * |        |          |0000 = 1.00 V.
     * |        |          |0001 = 1.05 V.
     * |        |          |0010 = 1.10 V.
     * |        |          |0011 = 1.15 V.
     * |        |          |0100 = 1.20 V.
     * |        |          |0101 = 1.25 V.
     * |        |          |0110 = 1.30 V.
     * |        |          |0111 = 1.35 V.
     * |        |          |1000 = 1.40 V.
     * |        |          |1001 = 1.45 V.
     * |        |          |1010 = 1.50 V.
     * |        |          |1011 = 1.55 V.
     * |        |          |1100 = 1.60 V.
     * |        |          |1101 = 1.65 V.
     * |        |          |1110 = 1.70 V.
     * |        |          |1111 = 1.75 V.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] LCD Control Register                                             */
    __IO uint32_t PSET;                  /*!< [0x0004] LCD Panel Setting Register                                       */
    __IO uint32_t FSET;                  /*!< [0x0008] LCD Frame Setting Register                                       */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t OSET;                  /*!< [0x0010] LCD Output Setting Register                                      */
    __IO uint32_t STS;                   /*!< [0x0014] LCD Status Register                                              */
    __IO uint32_t INTEN;                 /*!< [0x0018] LCD Interrupt Enable Register                                    */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t SEGDAT[12];            /*!< [0x0020- 0x004C] LCD Segment Display Data Register 0-11                   */
    __I  uint32_t RESERVE2[4];
    __IO uint32_t ASET1;                 /*!< [0x0060] LCD Analog Setting Register 1                                    */

} LCD_T;

/**
    @addtogroup LCD_CONST LCD Bit Field Definition
    Constant Definitions for LCD Controller
@{ */

#define LCD_CTL_LCDEN_Pos                (0)                                               /*!< LCD_T::CTL: LCDEN Position             */
#define LCD_CTL_LCDEN_Msk                (0x1ul << LCD_CTL_LCDEN_Pos)                      /*!< LCD_T::CTL: LCDEN Mask                 */

#define LCD_CTL_LCDENIND_Pos             (31)                                              /*!< LCD_T::CTL: LCDENIND Position          */
#define LCD_CTL_LCDENIND_Msk             (0x1ul << LCD_CTL_LCDENIND_Pos)                   /*!< LCD_T::CTL: LCDENIND Mask              */

#define LCD_PSET_BIAS_Pos                (0)                                               /*!< LCD_T::PSET: BIAS Position             */
#define LCD_PSET_BIAS_Msk                (0x3ul << LCD_PSET_BIAS_Pos)                      /*!< LCD_T::PSET: BIAS Mask                 */

#define LCD_PSET_DUTY_Pos                (2)                                               /*!< LCD_T::PSET: DUTY Position             */
#define LCD_PSET_DUTY_Msk                (0x7ul << LCD_PSET_DUTY_Pos)                      /*!< LCD_T::PSET: DUTY Mask                 */

#define LCD_PSET_TYPE_Pos                (5)                                               /*!< LCD_T::PSET: TYPE Position             */
#define LCD_PSET_TYPE_Msk                (0x1ul << LCD_PSET_TYPE_Pos)                      /*!< LCD_T::PSET: TYPE Mask                 */

#define LCD_PSET_INV_Pos                 (6)                                               /*!< LCD_T::PSET: INV Position             */
#define LCD_PSET_INV_Msk                 (0x1ul << LCD_PSET_INV_Pos)                       /*!< LCD_T::PSET: INV Mask                 */

#define LCD_PSET_LCDDIV_Pos              (8)                                               /*!< LCD_T::PSET: LCDDIV Position           */
#define LCD_PSET_LCDDIV_Msk              (0x3fful << LCD_PSET_LCDDIV_Pos)                  /*!< LCD_T::PSET: LCDDIV Mask               */

#define LCD_FSET_BLINK_Pos               (0)                                               /*!< LCD_T::FSET: BLINK Position            */
#define LCD_FSET_BLINK_Msk               (0x1ul << LCD_FSET_BLINK_Pos)                     /*!< LCD_T::FSET: BLINK Mask                */

#define LCD_FSET_FCV_Pos                 (8)                                               /*!< LCD_T::FSET: FCV Position              */
#define LCD_FSET_FCV_Msk                 (0x3fful << LCD_FSET_FCV_Pos)                     /*!< LCD_T::FSET: FCV Mask                  */

#define LCD_FSET_PTYPE_Pos               (19)                                              /*!< LCD_T::FSET: PTYPE Position            */
#define LCD_FSET_PTYPE_Msk               (0x1ul << LCD_FSET_PTYPE_Pos)                     /*!< LCD_T::FSET: PTYPE Mask                */

#define LCD_FSET_PTIME_Pos               (20)                                              /*!< LCD_T::FSET: PTIME Position            */
#define LCD_FSET_PTIME_Msk               (0xful << LCD_FSET_PTIME_Pos)                     /*!< LCD_T::FSET: PTIME Mask                */

#define LCD_OSET_SEL9_Pos                (0)                                               /*!< LCD_T::OSET: SEL9 Position             */
#define LCD_OSET_SEL9_Msk                (0x1ul << LCD_OSET_SEL9_Pos)                      /*!< LCD_T::OSET: SEL9 Mask                 */

#define LCD_OSET_SEL10_Pos               (1)                                               /*!< LCD_T::OSET: SEL10 Position            */
#define LCD_OSET_SEL10_Msk               (0x1ul << LCD_OSET_SEL10_Pos)                     /*!< LCD_T::OSET: SEL10 Mask                */

#define LCD_OSET_SEL11_Pos               (2)                                               /*!< LCD_T::OSET: SEL11 Position            */
#define LCD_OSET_SEL11_Msk               (0x1ul << LCD_OSET_SEL11_Pos)                     /*!< LCD_T::OSET: SEL11 Mask                */

#define LCD_OSET_SEL12_Pos               (3)                                               /*!< LCD_T::OSET: SEL12 Position            */
#define LCD_OSET_SEL12_Msk               (0x1ul << LCD_OSET_SEL12_Pos)                     /*!< LCD_T::OSET: SEL12 Mask                */

#define LCD_OSET_SEL13_Pos               (4)                                               /*!< LCD_T::OSET: SEL13 Position            */
#define LCD_OSET_SEL13_Msk               (0x1ul << LCD_OSET_SEL13_Pos)                     /*!< LCD_T::OSET: SEL13 Mask                */

#define LCD_OSET_SEL14_Pos               (5)                                               /*!< LCD_T::OSET: SEL14 Position            */
#define LCD_OSET_SEL14_Msk               (0x1ul << LCD_OSET_SEL14_Pos)                     /*!< LCD_T::OSET: SEL14 Mask                */

#define LCD_OSET_SEL15_Pos               (6)                                               /*!< LCD_T::OSET: SEL15 Position            */
#define LCD_OSET_SEL15_Msk               (0x1ul << LCD_OSET_SEL15_Pos)                     /*!< LCD_T::OSET: SEL15 Mask                */

#define LCD_OSET_SEL16_Pos               (7)                                               /*!< LCD_T::OSET: SEL16 Position            */
#define LCD_OSET_SEL16_Msk               (0x1ul << LCD_OSET_SEL16_Pos)                     /*!< LCD_T::OSET: SEL16 Mask                */

#define LCD_OSET_SEL24_Pos               (8)                                               /*!< LCD_T::OSET: SEL24 Position            */
#define LCD_OSET_SEL24_Msk               (0x1ul << LCD_OSET_SEL24_Pos)                     /*!< LCD_T::OSET: SEL24 Mask                */

#define LCD_OSET_SEL25_Pos               (9)                                               /*!< LCD_T::OSET: SEL25 Position            */
#define LCD_OSET_SEL25_Msk               (0x1ul << LCD_OSET_SEL25_Pos)                     /*!< LCD_T::OSET: SEL25 Mask                */

#define LCD_OSET_SEL26_Pos               (10)                                              /*!< LCD_T::OSET: SEL26 Position            */
#define LCD_OSET_SEL26_Msk               (0x1ul << LCD_OSET_SEL26_Pos)                     /*!< LCD_T::OSET: SEL26 Mask                */

#define LCD_OSET_SEL27_Pos               (11)                                              /*!< LCD_T::OSET: SEL27 Position            */
#define LCD_OSET_SEL27_Msk               (0x1ul << LCD_OSET_SEL27_Pos)                     /*!< LCD_T::OSET: SEL27 Mask                */

#define LCD_OSET_SEL28_Pos               (12)                                              /*!< LCD_T::OSET: SEL28 Position            */
#define LCD_OSET_SEL28_Msk               (0x1ul << LCD_OSET_SEL28_Pos)                     /*!< LCD_T::OSET: SEL28 Mask                */

#define LCD_OSET_SEL29_Pos               (13)                                              /*!< LCD_T::OSET: SEL29 Position            */
#define LCD_OSET_SEL29_Msk               (0x1ul << LCD_OSET_SEL29_Pos)                     /*!< LCD_T::OSET: SEL29 Mask                */

#define LCD_OSET_SEL37_Pos               (14)                                              /*!< LCD_T::OSET: SEL37 Position            */
#define LCD_OSET_SEL37_Msk               (0x3ul << LCD_OSET_SEL37_Pos)                     /*!< LCD_T::OSET: SEL37 Mask                */

#define LCD_OSET_SEL38_Pos               (16)                                              /*!< LCD_T::OSET: SEL38 Position            */
#define LCD_OSET_SEL38_Msk               (0x3ul << LCD_OSET_SEL38_Pos)                     /*!< LCD_T::OSET: SEL38 Mask                */

#define LCD_OSET_SEL39_Pos               (18)                                              /*!< LCD_T::OSET: SEL39 Position            */
#define LCD_OSET_SEL39_Msk               (0x1ul << LCD_OSET_SEL39_Pos)                     /*!< LCD_T::OSET: SEL39 Mask                */

#define LCD_OSET_SEL40_Pos               (19)                                              /*!< LCD_T::OSET: SEL40 Position            */
#define LCD_OSET_SEL40_Msk               (0x1ul << LCD_OSET_SEL40_Pos)                     /*!< LCD_T::OSET: SEL40 Mask                */

#define LCD_STS_FSCEND_Pos               (0)                                               /*!< LCD_T::STS: FSCEND Position            */
#define LCD_STS_FSCEND_Msk               (0x1ul << LCD_STS_FSCEND_Pos)                     /*!< LCD_T::STS: FSCEND Mask                */

#define LCD_STS_FSEND_Pos                (1)                                               /*!< LCD_T::STS: FSEND Position             */
#define LCD_STS_FSEND_Msk                (0x1ul << LCD_STS_FSEND_Pos)                      /*!< LCD_T::STS: FSEND Mask                 */

#define LCD_INTEN_FIECEND_Pos            (0)                                               /*!< LCD_T::INTEN: FIECEND Position         */
#define LCD_INTEN_FIECEND_Msk            (0x1ul << LCD_INTEN_FIECEND_Pos)                  /*!< LCD_T::INTEN: FIECEND Mask             */

#define LCD_INTEN_FIEEND_Pos             (1)                                               /*!< LCD_T::INTEN: FIEEND Position          */
#define LCD_INTEN_FIEEND_Msk             (0x1ul << LCD_INTEN_FIEEND_Pos)                   /*!< LCD_T::INTEN: FIEEND Mask              */

#define LCD_SEGDAT00_SEG0_Pos            (0)                                               /*!< LCD_T::SEGDAT00: SEG0 Position         */
#define LCD_SEGDAT00_SEG0_Msk            (0xfful << LCD_SEGDAT00_SEG0_Pos)                 /*!< LCD_T::SEGDAT00: SEG0 Mask             */

#define LCD_SEGDAT00_SEG1_Pos            (8)                                               /*!< LCD_T::SEGDAT00: SEG1 Position         */
#define LCD_SEGDAT00_SEG1_Msk            (0xfful << LCD_SEGDAT00_SEG1_Pos)                 /*!< LCD_T::SEGDAT00: SEG1 Mask             */

#define LCD_SEGDAT00_SEG2_Pos            (16)                                              /*!< LCD_T::SEGDAT00: SEG2 Position         */
#define LCD_SEGDAT00_SEG2_Msk            (0xfful << LCD_SEGDAT00_SEG2_Pos)                 /*!< LCD_T::SEGDAT00: SEG2 Mask             */

#define LCD_SEGDAT00_SEG3_Pos            (24)                                              /*!< LCD_T::SEGDAT00: SEG3 Position         */
#define LCD_SEGDAT00_SEG3_Msk            (0xfful << LCD_SEGDAT00_SEG3_Pos)                 /*!< LCD_T::SEGDAT00: SEG3 Mask             */

#define LCD_SEGDAT01_SEG4_Pos            (0)                                               /*!< LCD_T::SEGDAT04: SEG4 Position         */
#define LCD_SEGDAT01_SEG4_Msk            (0xfful << LCD_SEGDAT01_SEG4_Pos)                 /*!< LCD_T::SEGDAT04: SEG4 Mask             */

#define LCD_SEGDAT01_SEG5_Pos            (8)                                               /*!< LCD_T::SEGDAT04: SEG5 Position         */
#define LCD_SEGDAT01_SEG5_Msk            (0xfful << LCD_SEGDAT01_SEG5_Pos)                 /*!< LCD_T::SEGDAT04: SEG5 Mask             */

#define LCD_SEGDAT01_SEG6_Pos            (16)                                              /*!< LCD_T::SEGDAT04: SEG6 Position         */
#define LCD_SEGDAT01_SEG6_Msk            (0xfful << LCD_SEGDAT01_SEG6_Pos)                 /*!< LCD_T::SEGDAT04: SEG6 Mask             */

#define LCD_SEGDAT01_SEG7_Pos            (24)                                              /*!< LCD_T::SEGDAT04: SEG7 Position         */
#define LCD_SEGDAT01_SEG7_Msk            (0xfful << LCD_SEGDAT01_SEG7_Pos)                 /*!< LCD_T::SEGDAT04: SEG7 Mask             */

#define LCD_SEGDAT02_SEG8_Pos            (0)                                               /*!< LCD_T::SEGDAT08: SEG8 Position         */
#define LCD_SEGDAT02_SEG8_Msk            (0xfful << LCD_SEGDAT02_SEG8_Pos)                 /*!< LCD_T::SEGDAT08: SEG8 Mask             */

#define LCD_SEGDAT02_SEG9_Pos            (8)                                               /*!< LCD_T::SEGDAT08: SEG9 Position         */
#define LCD_SEGDAT02_SEG9_Msk            (0xfful << LCD_SEGDAT02_SEG9_Pos)                 /*!< LCD_T::SEGDAT08: SEG9 Mask             */

#define LCD_SEGDAT02_SEG10_Pos           (16)                                              /*!< LCD_T::SEGDAT08: SEG10 Position        */
#define LCD_SEGDAT02_SEG10_Msk           (0xfful << LCD_SEGDAT02_SEG10_Pos)                /*!< LCD_T::SEGDAT08: SEG10 Mask            */

#define LCD_SEGDAT02_SEG11_Pos           (24)                                              /*!< LCD_T::SEGDAT08: SEG11 Position        */
#define LCD_SEGDAT02_SEG11_Msk           (0xfful << LCD_SEGDAT02_SEG11_Pos)                /*!< LCD_T::SEGDAT08: SEG11 Mask            */

#define LCD_SEGDAT03_SEG12_Pos           (0)                                               /*!< LCD_T::SEGDAT12: SEG12 Position        */
#define LCD_SEGDAT03_SEG12_Msk           (0xfful << LCD_SEGDAT03_SEG12_Pos)                /*!< LCD_T::SEGDAT12: SEG12 Mask            */

#define LCD_SEGDAT03_SEG13_Pos           (8)                                               /*!< LCD_T::SEGDAT12: SEG13 Position        */
#define LCD_SEGDAT03_SEG13_Msk           (0xfful << LCD_SEGDAT03_SEG13_Pos)                /*!< LCD_T::SEGDAT12: SEG13 Mask            */

#define LCD_SEGDAT03_SEG14_Pos           (16)                                              /*!< LCD_T::SEGDAT12: SEG14 Position        */
#define LCD_SEGDAT03_SEG14_Msk           (0xfful << LCD_SEGDAT03_SEG14_Pos)                /*!< LCD_T::SEGDAT12: SEG14 Mask            */

#define LCD_SEGDAT03_SEG15_Pos           (24)                                              /*!< LCD_T::SEGDAT12: SEG15 Position        */
#define LCD_SEGDAT03_SEG15_Msk           (0xfful << LCD_SEGDAT03_SEG15_Pos)                /*!< LCD_T::SEGDAT12: SEG15 Mask            */

#define LCD_SEGDAT04_SEG16_Pos           (0)                                               /*!< LCD_T::SEGDAT16: SEG16 Position        */
#define LCD_SEGDAT04_SEG16_Msk           (0xfful << LCD_SEGDAT04_SEG16_Pos)                /*!< LCD_T::SEGDAT16: SEG16 Mask            */

#define LCD_SEGDAT04_SEG17_Pos           (8)                                               /*!< LCD_T::SEGDAT16: SEG17 Position        */
#define LCD_SEGDAT04_SEG17_Msk           (0xfful << LCD_SEGDAT04_SEG17_Pos)                /*!< LCD_T::SEGDAT16: SEG17 Mask            */

#define LCD_SEGDAT04_SEG18_Pos           (16)                                              /*!< LCD_T::SEGDAT16: SEG18 Position        */
#define LCD_SEGDAT04_SEG18_Msk           (0xfful << LCD_SEGDAT04_SEG18_Pos)                /*!< LCD_T::SEGDAT16: SEG18 Mask            */

#define LCD_SEGDAT04_SEG19_Pos           (24)                                              /*!< LCD_T::SEGDAT16: SEG19 Position        */
#define LCD_SEGDAT04_SEG19_Msk           (0xfful << LCD_SEGDAT04_SEG19_Pos)                /*!< LCD_T::SEGDAT16: SEG19 Mask            */

#define LCD_SEGDAT05_SEG20_Pos           (0)                                               /*!< LCD_T::SEGDAT20: SEG20 Position        */
#define LCD_SEGDAT05_SEG20_Msk           (0xfful << LCD_SEGDAT05_SEG20_Pos)                /*!< LCD_T::SEGDAT20: SEG20 Mask            */

#define LCD_SEGDAT05_SEG21_Pos           (8)                                               /*!< LCD_T::SEGDAT20: SEG21 Position        */
#define LCD_SEGDAT05_SEG21_Msk           (0xfful << LCD_SEGDAT05_SEG21_Pos)                /*!< LCD_T::SEGDAT20: SEG21 Mask            */

#define LCD_SEGDAT05_SEG22_Pos           (16)                                              /*!< LCD_T::SEGDAT20: SEG22 Position        */
#define LCD_SEGDAT05_SEG22_Msk           (0xfful << LCD_SEGDAT05_SEG22_Pos)                /*!< LCD_T::SEGDAT20: SEG22 Mask            */

#define LCD_SEGDAT05_SEG23_Pos           (24)                                              /*!< LCD_T::SEGDAT20: SEG23 Position        */
#define LCD_SEGDAT05_SEG23_Msk           (0xfful << LCD_SEGDAT05_SEG23_Pos)                /*!< LCD_T::SEGDAT20: SEG23 Mask            */

#define LCD_SEGDAT06_SEG24_Pos           (0)                                               /*!< LCD_T::SEGDAT24: SEG24 Position        */
#define LCD_SEGDAT06_SEG24_Msk           (0xfful << LCD_SEGDAT06_SEG24_Pos)                /*!< LCD_T::SEGDAT24: SEG24 Mask            */

#define LCD_SEGDAT06_SEG25_Pos           (8)                                               /*!< LCD_T::SEGDAT24: SEG25 Position        */
#define LCD_SEGDAT06_SEG25_Msk           (0xfful << LCD_SEGDAT06_SEG25_Pos)                /*!< LCD_T::SEGDAT24: SEG25 Mask            */

#define LCD_SEGDAT06_SEG26_Pos           (16)                                              /*!< LCD_T::SEGDAT24: SEG26 Position        */
#define LCD_SEGDAT06_SEG26_Msk           (0xfful << LCD_SEGDAT06_SEG26_Pos)                /*!< LCD_T::SEGDAT24: SEG26 Mask            */

#define LCD_SEGDAT06_SEG27_Pos           (24)                                              /*!< LCD_T::SEGDAT24: SEG27 Position        */
#define LCD_SEGDAT06_SEG27_Msk           (0xfful << LCD_SEGDAT06_SEG27_Pos)                /*!< LCD_T::SEGDAT24: SEG27 Mask            */

#define LCD_SEGDAT07_SEG28_Pos           (0)                                               /*!< LCD_T::SEGDAT28: SEG28 Position        */
#define LCD_SEGDAT07_SEG28_Msk           (0xfful << LCD_SEGDAT07_SEG28_Pos)                /*!< LCD_T::SEGDAT28: SEG28 Mask            */

#define LCD_SEGDAT07_SEG29_Pos           (8)                                               /*!< LCD_T::SEGDAT28: SEG29 Position        */
#define LCD_SEGDAT07_SEG29_Msk           (0xfful << LCD_SEGDAT07_SEG29_Pos)                /*!< LCD_T::SEGDAT28: SEG29 Mask            */

#define LCD_SEGDAT07_SEG30_Pos           (16)                                              /*!< LCD_T::SEGDAT28: SEG30 Position        */
#define LCD_SEGDAT07_SEG30_Msk           (0xfful << LCD_SEGDAT07_SEG30_Pos)                /*!< LCD_T::SEGDAT28: SEG30 Mask            */

#define LCD_SEGDAT07_SEG31_Pos           (24)                                              /*!< LCD_T::SEGDAT28: SEG31 Position        */
#define LCD_SEGDAT07_SEG31_Msk           (0xfful << LCD_SEGDAT07_SEG31_Pos)                /*!< LCD_T::SEGDAT28: SEG31 Mask            */

#define LCD_SEGDAT08_SEG32_Pos           (0)                                               /*!< LCD_T::SEGDAT32: SEG32 Position        */
#define LCD_SEGDAT08_SEG32_Msk           (0xfful << LCD_SEGDAT08_SEG32_Pos)                /*!< LCD_T::SEGDAT32: SEG32 Mask            */

#define LCD_SEGDAT08_SEG33_Pos           (8)                                               /*!< LCD_T::SEGDAT32: SEG33 Position        */
#define LCD_SEGDAT08_SEG33_Msk           (0xfful << LCD_SEGDAT08_SEG33_Pos)                /*!< LCD_T::SEGDAT32: SEG33 Mask            */

#define LCD_SEGDAT08_SEG34_Pos           (16)                                              /*!< LCD_T::SEGDAT32: SEG34 Position        */
#define LCD_SEGDAT08_SEG34_Msk           (0xfful << LCD_SEGDAT08_SEG34_Pos)                /*!< LCD_T::SEGDAT32: SEG34 Mask            */

#define LCD_SEGDAT08_SEG35_Pos           (24)                                              /*!< LCD_T::SEGDAT32: SEG35 Position        */
#define LCD_SEGDAT08_SEG35_Msk           (0xfful << LCD_SEGDAT08_SEG35_Pos)                /*!< LCD_T::SEGDAT32: SEG35 Mask            */

#define LCD_SEGDAT09_SEG36_Pos           (0)                                               /*!< LCD_T::SEGDAT36: SEG36 Position        */
#define LCD_SEGDAT09_SEG36_Msk           (0xfful << LCD_SEGDAT09_SEG36_Pos)                /*!< LCD_T::SEGDAT36: SEG36 Mask            */

#define LCD_SEGDAT09_SEG37_Pos           (8)                                               /*!< LCD_T::SEGDAT36: SEG37 Position        */
#define LCD_SEGDAT09_SEG37_Msk           (0xfful << LCD_SEGDAT09_SEG37_Pos)                /*!< LCD_T::SEGDAT36: SEG37 Mask            */

#define LCD_SEGDAT09_SEG38_Pos           (16)                                              /*!< LCD_T::SEGDAT36: SEG38 Position        */
#define LCD_SEGDAT09_SEG38_Msk           (0xfful << LCD_SEGDAT09_SEG38_Pos)                /*!< LCD_T::SEGDAT36: SEG38 Mask            */

#define LCD_SEGDAT09_SEG39_Pos           (24)                                              /*!< LCD_T::SEGDAT36: SEG39 Position        */
#define LCD_SEGDAT09_SEG39_Msk           (0xfful << LCD_SEGDAT09_SEG39_Pos)                /*!< LCD_T::SEGDAT36: SEG39 Mask            */

#define LCD_SEGDAT10_SEG40_Pos           (0)                                               /*!< LCD_T::SEGDAT40: SEG40 Position        */
#define LCD_SEGDAT10_SEG40_Msk           (0xfful << LCD_SEGDAT10_SEG40_Pos)                /*!< LCD_T::SEGDAT40: SEG40 Mask            */

#define LCD_SEGDAT10_SEG41_Pos           (8)                                               /*!< LCD_T::SEGDAT40: SEG41 Position        */
#define LCD_SEGDAT10_SEG41_Msk           (0xfful << LCD_SEGDAT10_SEG41_Pos)                /*!< LCD_T::SEGDAT40: SEG41 Mask            */

#define LCD_SEGDAT10_SEG42_Pos           (16)                                              /*!< LCD_T::SEGDAT40: SEG42 Position        */
#define LCD_SEGDAT10_SEG42_Msk           (0xfful << LCD_SEGDAT10_SEG42_Pos)                /*!< LCD_T::SEGDAT40: SEG42 Mask            */

#define LCD_SEGDAT10_SEG43_Pos           (24)                                              /*!< LCD_T::SEGDAT40: SEG43 Position        */
#define LCD_SEGDAT10_SEG43_Msk           (0xfful << LCD_SEGDAT10_SEG43_Pos)                /*!< LCD_T::SEGDAT40: SEG43 Mask            */

#define LCD_SEGDAT11_SEG44_Pos           (0)                                               /*!< LCD_T::SEGDAT44: SEG44 Position        */
#define LCD_SEGDAT11_SEG44_Msk           (0xfful << LCD_SEGDAT11_SEG44_Pos)                /*!< LCD_T::SEGDAT44: SEG44 Mask            */

#define LCD_SEGDAT11_SEG45_Pos           (8)                                               /*!< LCD_T::SEGDAT44: SEG45 Position        */
#define LCD_SEGDAT11_SEG45_Msk           (0xfful << LCD_SEGDAT11_SEG45_Pos)                /*!< LCD_T::SEGDAT44: SEG45 Mask            */

#define LCD_ASET1_LCDPEN_Pos             (0)                                               /*!< LCD_T::ASET1: LCDPEN Position          */
#define LCD_ASET1_LCDPEN_Msk             (0x1ul << LCD_ASET1_LCDPEN_Pos)                   /*!< LCD_T::ASET1: LCDPEN Mask              */

#define LCD_ASET1_PSTRUC_Pos             (1)                                               /*!< LCD_T::ASET1: PSTRUC Position          */
#define LCD_ASET1_PSTRUC_Msk             (0x3ul << LCD_ASET1_PSTRUC_Pos)                   /*!< LCD_T::ASET1: PSTRUC Mask              */

#define LCD_ASET1_VL1SEL_Pos             (3)                                               /*!< LCD_T::ASET1: VL1SEL Position          */
#define LCD_ASET1_VL1SEL_Msk             (0xful << LCD_ASET1_VL1SEL_Pos)                   /*!< LCD_T::ASET1: VL1SEL Mask              */

/** @} LCD_CONST */
/** @} end of LCD register group */
/** @} end of REGISTER group */

#endif /* __LCD_REG_H__ */
