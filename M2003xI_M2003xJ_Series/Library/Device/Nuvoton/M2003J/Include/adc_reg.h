/**************************************************************************//**
 * @file     adc_reg.h
 * @version  V1.00
 * @brief    ADC register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __ADC_REG_H__
#define __ADC_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
    @addtogroup ADC Analog to Digital Converter (ADC)
    Memory Mapped Structure for ADC Controller
@{ */

typedef struct
{
    /**
     * @var ADC_T::ADDR0
     * Offset: 0x00  ADC Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR1
     * Offset: 0x04  ADC Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR2
     * Offset: 0x08  ADC Data Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR3
     * Offset: 0x0C  ADC Data Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR4
     * Offset: 0x10  ADC Data Register 4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR5
     * Offset: 0x14  ADC Data Register 5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR6
     * Offset: 0x18  ADC Data Register 6
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR7
     * Offset: 0x1C  ADC Data Register 7
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR8
     * Offset: 0x20  ADC Data Register 8
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR9
     * Offset: 0x24  ADC Data Register 9
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR10
     * Offset: 0x28  ADC Data Register 10
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR11
     * Offset: 0x2C  ADC Data Register 11
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR12
     * Offset: 0x30  ADC Data Register 12
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR13
     * Offset: 0x34  ADC Data Register 13
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR14
     * Offset: 0x38  ADC Data Register 14
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR15
     * Offset: 0x3C  ADC Data Register 15
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR16
     * Offset: 0x40  ADC Data Register 16
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR17
     * Offset: 0x44  ADC Data Register 17
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR18
     * Offset: 0x48  ADC Data Register 18
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR19
     * Offset: 0x4C  ADC Data Register 19
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR20
     * Offset: 0x50  ADC Data Register 20
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR21
     * Offset: 0x54  ADC Data Register 21
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR22
     * Offset: 0x58  ADC Data Register 22
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR23
     * Offset: 0x5C  ADC Data Register 23
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADDR29
     * Offset: 0x74  ADC Data Register 29
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADCR
     * Offset: 0x80  ADC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADEN      |A/D Converter Enable Bit
     * |        |          |0 = A/D converter Disabled.
     * |        |          |1 = A/D converter Enabled.
     * |        |          |Note: Before starting A/D conversion function, this bit should be set to 1
     * |        |          |Clear it to 0 to disable A/D converter analog circuit to save power consumption.
     * |[1]     |ADIE      |A/D Interrupt Enable Bit
     * |        |          |A/D conversion end interrupt request is generated if ADIE bit is set to 1.
     * |        |          |0 = A/D conversion end interrupt function Disabled.
     * |        |          |1 = A/D conversion end interrupt function Enabled.
     * |[3:2]   |ADMD      |A/D Converter Operation Mode Control
     * |        |          |00 = Single conversion.
     * |        |          |01 = Burst conversion.
     * |        |          |10 = Single-cycle Scan.
     * |        |          |11 = Continuous Scan.
     * |        |          |Note 1: When changing the operation mode, software should clear ADST bit first.
     * |        |          |Note 2: In Burst mode, the A/D conversion result data is always at ADC Data Register 0.
     * |[7:6]   |TRGCOND   |External Trigger Condition
     * |        |          |These two bits decide external pin ADC0_ST trigger event is level or edge
     * |        |          |The signal must be kept at stable state at least 8 PCLKs for level trigger and at least 4 PCLKs for edge trigger.
     * |        |          |00 = Low level.
     * |        |          |01 = High level.
     * |        |          |10 = Falling edge.
     * |        |          |11 = Rising edge.
     * |[8]     |TRGEN     |External Trigger Enable Bit
     * |        |          |Enable or disable triggering of A/D conversion by external ADC0_ST pin, BPWM trigger and Timer trigger
     * |        |          |If external trigger is enabled, the ADST bit can be set to 1 by the selected hardware trigger source.
     * |        |          |0 = External trigger Disabled.
     * |        |          |1 = External trigger Enabled.
     * |[9]     |PTEN      |PDMA Transfer Enable Bit
     * |        |          |When A/D conversion is completed, the converted data is loaded into ADDR0~18
     * |        |          |Software can enable this bit to generate a PDMA data transfer request.
     * |        |          |0 = PDMA data transfer Disabled.
     * |        |          |1 = PDMA data transfer in ADDR0~18 Enabled.
     * |        |          |Note: When PTEN=1, software must set ADIE=0 to disable interrupt.
     * |[11]    |ADST      |A/D Conversion Start
     * |        |          |ADST bit can be set to 1 from five sources: software, external pin ADC0_ST, BPWM trigger and Timer trigger
     * |        |          |ADST bit will be cleared to 0 by hardware automatically at the ends of Single mode, Single-cycle Scan mode
     * |        |          |In Continuous Scan mode and Burst mode, A/D conversion is continuously performed until software writes 0 to this bit or chip is reset.
     * |        |          |0 = Conversion stops and A/D converter enters idle state.
     * |        |          |1 = Conversion starts.
     * |        |          |Note 1: When ADST becomes from 1 to 0, ADC macro will reset to initial state
     * |        |          |After macro reset to initial state, user should wait at most 2 ADC clock and set this bit to start next conversion.
     * |[14]    |ADDRRST   |ADDRx Reset
     * |        |          |If user writes this bit, VALID and OVERRUN of all ADDRx will be reset.
     * |        |          |Note: This bit is cleared by hardware.
     * |[19:16] |TRGS      |Hardware Trigger Source
     * |        |          |0000 = A/D conversion is started by external ADC0_ST pin.
     * |        |          |0001 = Timer0 ~ Timer8 overflow pulse trigger.
     * |        |          |0010 = A/D conversion is started by BPWM trigger.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: Software should clear TRGEN bit and ADST bit to 0 before changing TRGS bits.
     * |        |          |Note 2: These trigger sources are only abaliable when chip is in run and idle mode.
     * |[25:24] |STBSEL    |ADC Stable Count Select Bits
     * |        |          |The ADC stable count is as follows.
     * |        |          |00 = ADC stable count is 32 ADC_CLK (When ADC_CLK is 16 MHz).
     * |        |          |01 = ADC stable count is 24 ADC_CLK (When ADC_CLK is 12 MHz).
     * |        |          |10 = ADC stable count is 20 ADC_CLK (When ADC_CLK is 10 MHz).
     * |        |          |11 = ADC stable count is 16 ADC_CLK (When ADC_CLK is 8 MHz).
     * @var ADC_T::ADCHER
     * Offset: 0x84  ADC Channel Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CHEN      |Analog   Input Channel Enable Control
     * |        |          |Set ADCHER[23:0] bits to enable the corresponding analog external input channel 23 ~ 0.
     * |        |          |If DIFFEN bit is set to 1, only the even number channel needs to be enabled.
     * |        |          |Besides, setting the ADCHER[29] bit will enable internal channel for band-gap voltage. Other bits are reserved.
     * |        |          |0 =   Channel Disabled.
     * |        |          |1 =   Channel Enabled.
     * |        |          |Note: If the internal channel band-gap voltage (CHEN[29]), the maximum sampling rate will be 300k SPS.
     * @var ADC_T::ADCMPR0
     * Offset: 0x88  ADC Compare Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CMPEN     |Compare Enable Bit
     * |        |          |Set this bit to 1 to enable ADC to compare CMPD (ADCMPRx[27:16]) with specified channel conversion result when converted data is loaded into ADDR register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |CMPIE     |Compare Interrupt Enable Bit
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, CMPFx bit will be asserted, in the meanwhile, if CMPIE bit is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 12-bit A/D conversion result is less than the 12-bit CMPD bits, the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 12-bit A/D conversion result is greater than or equal to the 12-bit CMPD bits, the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches to (CMPMCNT +1), the CMPFx bit will be set.
     * |[7:3]   |CMPCH     |Compare Channel Selection
     * |        |          |00000 = Channel 0 conversion result is selected to be compared.
     * |        |          |00001 = Channel 1 conversion result is selected to be compared.
     * |        |          |00010 = Channel 2 conversion result is selected to be compared.
     * |        |          |00011 = Channel 3 conversion result is selected to be compared.
     * |        |          |00100 = Channel 4 conversion result is selected to be compared.
     * |        |          |00101 = Channel 5 conversion result is selected to be compared.
     * |        |          |00110 = Channel 6 conversion result is selected to be compared.
     * |        |          |00111 = Channel 7 conversion result is selected to be compared.
     * |        |          |01000 = Channel 8 conversion result is selected to be compared.
     * |        |          |01001 = Channel 9 conversion result is selected to be compared.
     * |        |          |01010 = Channel 10 conversion result is selected to be compared.
     * |        |          |01011 = Channel 11 conversion result is selected to be compared.
     * |        |          |01100 = Channel 12 conversion result is selected to be compared.
     * |        |          |01101 = Channel 13 conversion result is selected to be compared.
     * |        |          |01110 = Channel 14 conversion result is selected to be compared.
     * |        |          |01111 = Channel 15 conversion result is selected to be compared.
     * |        |          |10000 = Channel 16 conversion result is selected to be compared.
     * |        |          |10001 = Channel 17 conversion result is selected to be compared.
     * |        |          |10010 = Channel 18 conversion result is selected to be compared.
     * |        |          |10011 = Channel 19 conversion result is selected to be compared.
     * |        |          |10100 = Channel 20 conversion result is selected to be compared.
     * |        |          |10101 = Channel 21 conversion result is selected to be compared.
     * |        |          |10110 = Channel 22 conversion result is selected to be compared.
     * |        |          |10111 = Channel 23 conversion result is selected to be compared.
     * |        |          |10000 = Band-gap voltage conversion result is selected to be compared.
     * |        |          |Others = Reserved.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND bit, the internal match counter will increase 1
     * |        |          |When the internal counter reaches the value to (CMPMCNT +1), the CMPFx bit will be set.
     * |[15]    |CMPWEN    |Compare Window Mode Enable Bit
     * |        |          |0 = Compare Window Mode Disabled.
     * |        |          |1 = Compare Window Mode Enabled.
     * |        |          |Note: This bit is only presented in ADCMPR0 register.
     * |[27:16] |CMPD      |Comparison Data
     * |        |          |The 12-bit data is used to compare with conversion result of specified channel.
     * |        |          |Note: CMPD bits should be filled in unsigned format (straight binary format).
     * @var ADC_T::ADCMPR1
     * Offset: 0x8C  ADC Compare Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CMPEN     |Compare Enable Bit
     * |        |          |Set this bit to 1 to enable ADC to compare CMPD (ADCMPRx[27:16]) with specified channel conversion result when converted data is loaded into ADDR register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |CMPIE     |Compare Interrupt Enable Bit
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, CMPFx bit will be asserted, in the meanwhile, if CMPIE bit is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 12-bit A/D conversion result is less than the 12-bit CMPD bits, the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 12-bit A/D conversion result is greater than or equal to the 12-bit CMPD bits, the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches to (CMPMCNT +1), the CMPFx bit will be set.
     * |[7:3]   |CMPCH     |Compare Channel Selection
     * |        |          |00000 = Channel 0 conversion result is selected to be compared.
     * |        |          |00001 = Channel 1 conversion result is selected to be compared.
     * |        |          |00010 = Channel 2 conversion result is selected to be compared.
     * |        |          |00011 = Channel 3 conversion result is selected to be compared.
     * |        |          |00100 = Channel 4 conversion result is selected to be compared.
     * |        |          |00101 = Channel 5 conversion result is selected to be compared.
     * |        |          |00110 = Channel 6 conversion result is selected to be compared.
     * |        |          |00111 = Channel 7 conversion result is selected to be compared.
     * |        |          |01000 = Channel 8 conversion result is selected to be compared.
     * |        |          |01001 = Channel 9 conversion result is selected to be compared.
     * |        |          |01010 = Channel 10 conversion result is selected to be compared.
     * |        |          |01011 = Channel 11 conversion result is selected to be compared.
     * |        |          |01100 = Channel 12 conversion result is selected to be compared.
     * |        |          |01101 = Channel 13 conversion result is selected to be compared.
     * |        |          |01110 = Channel 14 conversion result is selected to be compared.
     * |        |          |01111 = Channel 15 conversion result is selected to be compared.
     * |        |          |10000 = Channel 16 conversion result is selected to be compared.
     * |        |          |10001 = Channel 17 conversion result is selected to be compared.
     * |        |          |10010 = Channel 18 conversion result is selected to be compared.
     * |        |          |10011 = Channel 19 conversion result is selected to be compared.
     * |        |          |10100 = Channel 20 conversion result is selected to be compared.
     * |        |          |10101 = Channel 21 conversion result is selected to be compared.
     * |        |          |10110 = Channel 22 conversion result is selected to be compared.
     * |        |          |10111 = Channel 23 conversion result is selected to be compared.
     * |        |          |10000 = Band-gap voltage conversion result is selected to be compared.
     * |        |          |Others = Reserved.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND bit, the internal match counter will increase 1
     * |        |          |When the internal counter reaches the value to (CMPMCNT +1), the CMPFx bit will be set.
     * |[15]    |CMPWEN    |Compare Window Mode Enable Bit
     * |        |          |0 = Compare Window Mode Disabled.
     * |        |          |1 = Compare Window Mode Enabled.
     * |        |          |Note: This bit is only presented in ADCMPR0 register.
     * |[27:16] |CMPD      |Comparison Data
     * |        |          |The 12-bit data is used to compare with conversion result of specified channel.
     * |        |          |Note: CMPD bits should be filled in unsigned format (straight binary format).
     * @var ADC_T::ADSR0
     * Offset: 0x90  ADC Status Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADF       |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion
     * |        |          |This bit can be cleared by software writing 1 to it.
     * |        |          |The ADF bit is set to 1 at the following three conditions:
     * |        |          |1. When A/D conversion ends in Single mode.
     * |        |          |2
     * |        |          |When A/D conversion ends on all specified channels in Single-cycle Scan mode and Continuous Scan mode.
     * |        |          |3. When number of samples in the FIFO is greater than or equal to half of its size in Burst mode.
     * |[1]     |CMPF0     |Compare Flag 0
     * |        |          |When the A/D conversion result of the selected channel meets setting condition in ADCMPR0 register then this bit is set to 1
     * |        |          |This bit is cleared by writing 1 to it.
     * |        |          |0 = Conversion result in ADDR does not meet ADCMPR0 setting.
     * |        |          |1 = Conversion result in ADDR meets ADCMPR0 setting.
     * |[2]     |CMPF1     |Compare Flag 1
     * |        |          |When the A/D conversion result of the selected channel meets setting condition in ADCMPR1 register, this bit is set to 1; it is cleared by writing 1 to it
     * |        |          |0 = Conversion result in ADDR does not meet ADCMPR1 setting.
     * |        |          |1 = Conversion result in ADDR meets ADCMPR1 setting.
     * |[7]     |BUSY      |BUSY/IDLE (Read Only)
     * |        |          |This bit is a mirror of ADST bit in ADCR register.
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |[8]     |VALIDF    |Data Valid Flag (Read Only)
     * |        |          |If any one of VALID (ADDRx[17]) is set, this flag will be set to 1.
     * |        |          |Note: When ADC is in burst mode and any conversion result is valid, this flag will be set to 1.
     * |[16]    |OVERRUNF  |Overrun Flag (Read Only)
     * |        |          |If any one of OVERRUN (ADDRx[16]) is set, this flag will be set to 1.
     * |        |          |Note: When ADC is in burst mode and the FIFO is overrun, this flag will be set to 1.
     * |[24]    |ADCRDY    |ADC Power On Ready (Read Only)
     * |        |          |When ADEN (ADC_ADCR[0]) set to 1 or auto-operation Trigger to enable ADC, the ADC need a stable time
     * |        |          |The ADCRDY will keep low until ADC stable
     * |        |          |The ADC power ready time depends on STBSEL (ADC_ ADCR [25:24]).
     * |        |          |Note: ADC can start converting after ADCRDY is 1.
     * |[31:27] |CHANNEL   |Current Conversion Channel (Read Only)
     * |        |          |When BUSY=1, this filed reflects current conversion channel
     * |        |          |When BUSY=0, it shows the number of the next converted channel.
     * @var ADC_T::ADSR1
     * Offset: 0x94  ADC Status Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VALID     |Data Valid Flag (Read Only)
     * |        |          |VALID[29, 23:0] are the mirror of the VALID bits in ADDR29[17], ADDR23[17]~ ADDR0[17]
     * |        |          |The other bits are reserved.
     * |        |          |Note: When ADC is in burst mode and any conversion result is valid, VALID[29, 23:0] will be set to 1.
     * @var ADC_T::ADSR2
     * Offset: 0x98  ADC Status Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |OVERRUN[29, 23:0] are the mirror of the OVERRUN bit in ADDR29[16], ADDR23[16] ~ ADDR0[16]
     * |        |          |The other bits are reserved.
     * |        |          |Note: When ADC is in burst mode and the FIFO is overrun, OVERRUN[29, 23:0] will be set to 1.
     * @var ADC_T::ESMPCTL
     * Offset: 0xA0  ADC Extend Sample Time Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[13:0]  |EXTSMPT   |ADC Sampling Time Extend
     * |        |          |When A/D conversion at high conversion rate, the sampling time of analog input voltage may not enough if input channel loading is heavy, user can extend ADC sampling time after trigger source is coming to get enough sampling time.
     * |        |          |The range of start delay time is from 0~16367 ADC clock.
     * |        |          |Note: EXTSMPT will be set to 16367 if write a value more than 16367 in the register.
     * @var ADC_T::ADPDMA
     * Offset: 0x100  ADC PDMA Current Transfer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[17:0]  |CURDAT    |ADC PDMA Current Transfer Data Register (Read Only)
     * |        |          |When PDMA transfers data, reading the register can monitor the current PDMA transfer data.
     * |        |          |Current PDMA transfer data could be the content of ADDR0 ~ ADDR23, ADDR29 registers.
     */
    __I  uint32_t ADDR[30];              /*!< [0x0000-0x0074] ADC Data Register 0 ~ 29                                  */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t ADCR;                  /*!< [0x0080] ADC Control Register                                             */
    __IO uint32_t ADCHER;                /*!< [0x0084] ADC Channel Enable Register                                      */
    __IO uint32_t ADCMPR[2];             /*!< [0x0088-0x008c] ADC Compare Register 0/1                                  */
    __IO uint32_t ADSR0;                 /*!< [0x0090] ADC Status Register0                                             */
    __I  uint32_t ADSR1;                 /*!< [0x0094] ADC Status Register1                                             */
    __I  uint32_t ADSR2;                 /*!< [0x0098] ADC Status Register2                                             */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t ESMPCTL;               /*!< [0x00a0] ADC Extend Sample Time Control Register                          */
    __I  uint32_t RESERVE3[23];
    __I  uint32_t ADPDMA;                /*!< [0x0100] ADC PDMA Current Transfer Data Register                          */
} ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */

#define ADC_ADDR_RSLT_Pos                (0)                                               /*!< ADC_T::ADDR: RSLT Position            */
#define ADC_ADDR_RSLT_Msk                (0xfffful << ADC_ADDR_RSLT_Pos)                   /*!< ADC_T::ADDR: RSLT Mask                */

#define ADC_ADDR_OVERRUN_Pos             (16)                                              /*!< ADC_T::ADDR: OVERRUN Position         */
#define ADC_ADDR_OVERRUN_Msk             (0x1ul << ADC_ADDR_OVERRUN_Pos)                   /*!< ADC_T::ADDR: OVERRUN Mask             */

#define ADC_ADDR_VALID_Pos               (17)                                              /*!< ADC_T::ADDR: VALID Position           */
#define ADC_ADDR_VALID_Msk               (0x1ul << ADC_ADDR_VALID_Pos)                     /*!< ADC_T::ADDR: VALID Mask               */

#define ADC_ADDR0_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR0: RSLT Position            */
#define ADC_ADDR0_RSLT_Msk               (0xfffful << ADC_ADDR0_RSLT_Pos)                  /*!< ADC_T::ADDR0: RSLT Mask                */

#define ADC_ADDR0_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR0: OVERRUN Position         */
#define ADC_ADDR0_OVERRUN_Msk            (0x1ul << ADC_ADDR0_OVERRUN_Pos)                  /*!< ADC_T::ADDR0: OVERRUN Mask             */

#define ADC_ADDR0_VALID_Pos              (17)                                              /*!< ADC_T::ADDR0: VALID Position           */
#define ADC_ADDR0_VALID_Msk              (0x1ul << ADC_ADDR0_VALID_Pos)                    /*!< ADC_T::ADDR0: VALID Mask               */

#define ADC_ADDR1_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR1: RSLT Position            */
#define ADC_ADDR1_RSLT_Msk               (0xfffful << ADC_ADDR1_RSLT_Pos)                  /*!< ADC_T::ADDR1: RSLT Mask                */

#define ADC_ADDR1_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR1: OVERRUN Position         */
#define ADC_ADDR1_OVERRUN_Msk            (0x1ul << ADC_ADDR1_OVERRUN_Pos)                  /*!< ADC_T::ADDR1: OVERRUN Mask             */

#define ADC_ADDR1_VALID_Pos              (17)                                              /*!< ADC_T::ADDR1: VALID Position           */
#define ADC_ADDR1_VALID_Msk              (0x1ul << ADC_ADDR1_VALID_Pos)                    /*!< ADC_T::ADDR1: VALID Mask               */

#define ADC_ADDR2_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR2: RSLT Position            */
#define ADC_ADDR2_RSLT_Msk               (0xfffful << ADC_ADDR2_RSLT_Pos)                  /*!< ADC_T::ADDR2: RSLT Mask                */

#define ADC_ADDR2_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR2: OVERRUN Position         */
#define ADC_ADDR2_OVERRUN_Msk            (0x1ul << ADC_ADDR2_OVERRUN_Pos)                  /*!< ADC_T::ADDR2: OVERRUN Mask             */

#define ADC_ADDR2_VALID_Pos              (17)                                              /*!< ADC_T::ADDR2: VALID Position           */
#define ADC_ADDR2_VALID_Msk              (0x1ul << ADC_ADDR2_VALID_Pos)                    /*!< ADC_T::ADDR2: VALID Mask               */

#define ADC_ADDR3_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR3: RSLT Position            */
#define ADC_ADDR3_RSLT_Msk               (0xfffful << ADC_ADDR3_RSLT_Pos)                  /*!< ADC_T::ADDR3: RSLT Mask                */

#define ADC_ADDR3_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR3: OVERRUN Position         */
#define ADC_ADDR3_OVERRUN_Msk            (0x1ul << ADC_ADDR3_OVERRUN_Pos)                  /*!< ADC_T::ADDR3: OVERRUN Mask             */

#define ADC_ADDR3_VALID_Pos              (17)                                              /*!< ADC_T::ADDR3: VALID Position           */
#define ADC_ADDR3_VALID_Msk              (0x1ul << ADC_ADDR3_VALID_Pos)                    /*!< ADC_T::ADDR3: VALID Mask               */

#define ADC_ADDR4_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR4: RSLT Position            */
#define ADC_ADDR4_RSLT_Msk               (0xfffful << ADC_ADDR4_RSLT_Pos)                  /*!< ADC_T::ADDR4: RSLT Mask                */

#define ADC_ADDR4_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR4: OVERRUN Position         */
#define ADC_ADDR4_OVERRUN_Msk            (0x1ul << ADC_ADDR4_OVERRUN_Pos)                  /*!< ADC_T::ADDR4: OVERRUN Mask             */

#define ADC_ADDR4_VALID_Pos              (17)                                              /*!< ADC_T::ADDR4: VALID Position           */
#define ADC_ADDR4_VALID_Msk              (0x1ul << ADC_ADDR4_VALID_Pos)                    /*!< ADC_T::ADDR4: VALID Mask               */

#define ADC_ADDR5_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR5: RSLT Position            */
#define ADC_ADDR5_RSLT_Msk               (0xfffful << ADC_ADDR5_RSLT_Pos)                  /*!< ADC_T::ADDR5: RSLT Mask                */

#define ADC_ADDR5_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR5: OVERRUN Position         */
#define ADC_ADDR5_OVERRUN_Msk            (0x1ul << ADC_ADDR5_OVERRUN_Pos)                  /*!< ADC_T::ADDR5: OVERRUN Mask             */

#define ADC_ADDR5_VALID_Pos              (17)                                              /*!< ADC_T::ADDR5: VALID Position           */
#define ADC_ADDR5_VALID_Msk              (0x1ul << ADC_ADDR5_VALID_Pos)                    /*!< ADC_T::ADDR5: VALID Mask               */

#define ADC_ADDR6_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR6: RSLT Position            */
#define ADC_ADDR6_RSLT_Msk               (0xfffful << ADC_ADDR6_RSLT_Pos)                  /*!< ADC_T::ADDR6: RSLT Mask                */

#define ADC_ADDR6_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR6: OVERRUN Position         */
#define ADC_ADDR6_OVERRUN_Msk            (0x1ul << ADC_ADDR6_OVERRUN_Pos)                  /*!< ADC_T::ADDR6: OVERRUN Mask             */

#define ADC_ADDR6_VALID_Pos              (17)                                              /*!< ADC_T::ADDR6: VALID Position           */
#define ADC_ADDR6_VALID_Msk              (0x1ul << ADC_ADDR6_VALID_Pos)                    /*!< ADC_T::ADDR6: VALID Mask               */

#define ADC_ADDR7_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR7: RSLT Position            */
#define ADC_ADDR7_RSLT_Msk               (0xfffful << ADC_ADDR7_RSLT_Pos)                  /*!< ADC_T::ADDR7: RSLT Mask                */

#define ADC_ADDR7_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR7: OVERRUN Position         */
#define ADC_ADDR7_OVERRUN_Msk            (0x1ul << ADC_ADDR7_OVERRUN_Pos)                  /*!< ADC_T::ADDR7: OVERRUN Mask             */

#define ADC_ADDR7_VALID_Pos              (17)                                              /*!< ADC_T::ADDR7: VALID Position           */
#define ADC_ADDR7_VALID_Msk              (0x1ul << ADC_ADDR7_VALID_Pos)                    /*!< ADC_T::ADDR7: VALID Mask               */

#define ADC_ADDR8_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR8: RSLT Position            */
#define ADC_ADDR8_RSLT_Msk               (0xfffful << ADC_ADDR8_RSLT_Pos)                  /*!< ADC_T::ADDR8: RSLT Mask                */

#define ADC_ADDR8_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR8: OVERRUN Position         */
#define ADC_ADDR8_OVERRUN_Msk            (0x1ul << ADC_ADDR8_OVERRUN_Pos)                  /*!< ADC_T::ADDR8: OVERRUN Mask             */

#define ADC_ADDR8_VALID_Pos              (17)                                              /*!< ADC_T::ADDR8: VALID Position           */
#define ADC_ADDR8_VALID_Msk              (0x1ul << ADC_ADDR8_VALID_Pos)                    /*!< ADC_T::ADDR8: VALID Mask               */

#define ADC_ADDR9_RSLT_Pos               (0)                                               /*!< ADC_T::ADDR9: RSLT Position            */
#define ADC_ADDR9_RSLT_Msk               (0xfffful << ADC_ADDR9_RSLT_Pos)                  /*!< ADC_T::ADDR9: RSLT Mask                */

#define ADC_ADDR9_OVERRUN_Pos            (16)                                              /*!< ADC_T::ADDR9: OVERRUN Position         */
#define ADC_ADDR9_OVERRUN_Msk            (0x1ul << ADC_ADDR9_OVERRUN_Pos)                  /*!< ADC_T::ADDR9: OVERRUN Mask             */

#define ADC_ADDR9_VALID_Pos              (17)                                              /*!< ADC_T::ADDR9: VALID Position           */
#define ADC_ADDR9_VALID_Msk              (0x1ul << ADC_ADDR9_VALID_Pos)                    /*!< ADC_T::ADDR9: VALID Mask               */

#define ADC_ADDR10_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR10: RSLT Position           */
#define ADC_ADDR10_RSLT_Msk              (0xfffful << ADC_ADDR10_RSLT_Pos)                 /*!< ADC_T::ADDR10: RSLT Mask               */

#define ADC_ADDR10_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR10: OVERRUN Position        */
#define ADC_ADDR10_OVERRUN_Msk           (0x1ul << ADC_ADDR10_OVERRUN_Pos)                 /*!< ADC_T::ADDR10: OVERRUN Mask            */

#define ADC_ADDR10_VALID_Pos             (17)                                              /*!< ADC_T::ADDR10: VALID Position          */
#define ADC_ADDR10_VALID_Msk             (0x1ul << ADC_ADDR10_VALID_Pos)                   /*!< ADC_T::ADDR10: VALID Mask              */

#define ADC_ADDR11_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR11: RSLT Position           */
#define ADC_ADDR11_RSLT_Msk              (0xfffful << ADC_ADDR11_RSLT_Pos)                 /*!< ADC_T::ADDR11: RSLT Mask               */

#define ADC_ADDR11_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR11: OVERRUN Position        */
#define ADC_ADDR11_OVERRUN_Msk           (0x1ul << ADC_ADDR11_OVERRUN_Pos)                 /*!< ADC_T::ADDR11: OVERRUN Mask            */

#define ADC_ADDR11_VALID_Pos             (17)                                              /*!< ADC_T::ADDR11: VALID Position          */
#define ADC_ADDR11_VALID_Msk             (0x1ul << ADC_ADDR11_VALID_Pos)                   /*!< ADC_T::ADDR11: VALID Mask              */

#define ADC_ADDR12_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR12: RSLT Position           */
#define ADC_ADDR12_RSLT_Msk              (0xfffful << ADC_ADDR12_RSLT_Pos)                 /*!< ADC_T::ADDR12: RSLT Mask               */

#define ADC_ADDR12_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR12: OVERRUN Position        */
#define ADC_ADDR12_OVERRUN_Msk           (0x1ul << ADC_ADDR12_OVERRUN_Pos)                 /*!< ADC_T::ADDR12: OVERRUN Mask            */

#define ADC_ADDR12_VALID_Pos             (17)                                              /*!< ADC_T::ADDR12: VALID Position          */
#define ADC_ADDR12_VALID_Msk             (0x1ul << ADC_ADDR12_VALID_Pos)                   /*!< ADC_T::ADDR12: VALID Mask              */

#define ADC_ADDR13_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR13: RSLT Position           */
#define ADC_ADDR13_RSLT_Msk              (0xfffful << ADC_ADDR13_RSLT_Pos)                 /*!< ADC_T::ADDR13: RSLT Mask               */

#define ADC_ADDR13_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR13: OVERRUN Position        */
#define ADC_ADDR13_OVERRUN_Msk           (0x1ul << ADC_ADDR13_OVERRUN_Pos)                 /*!< ADC_T::ADDR13: OVERRUN Mask            */

#define ADC_ADDR13_VALID_Pos             (17)                                              /*!< ADC_T::ADDR13: VALID Position          */
#define ADC_ADDR13_VALID_Msk             (0x1ul << ADC_ADDR13_VALID_Pos)                   /*!< ADC_T::ADDR13: VALID Mask              */

#define ADC_ADDR14_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR14: RSLT Position           */
#define ADC_ADDR14_RSLT_Msk              (0xfffful << ADC_ADDR14_RSLT_Pos)                 /*!< ADC_T::ADDR14: RSLT Mask               */

#define ADC_ADDR14_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR14: OVERRUN Position        */
#define ADC_ADDR14_OVERRUN_Msk           (0x1ul << ADC_ADDR14_OVERRUN_Pos)                 /*!< ADC_T::ADDR14: OVERRUN Mask            */

#define ADC_ADDR14_VALID_Pos             (17)                                              /*!< ADC_T::ADDR14: VALID Position          */
#define ADC_ADDR14_VALID_Msk             (0x1ul << ADC_ADDR14_VALID_Pos)                   /*!< ADC_T::ADDR14: VALID Mask              */

#define ADC_ADDR15_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR15: RSLT Position           */
#define ADC_ADDR15_RSLT_Msk              (0xfffful << ADC_ADDR15_RSLT_Pos)                 /*!< ADC_T::ADDR15: RSLT Mask               */

#define ADC_ADDR15_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR15: OVERRUN Position        */
#define ADC_ADDR15_OVERRUN_Msk           (0x1ul << ADC_ADDR15_OVERRUN_Pos)                 /*!< ADC_T::ADDR15: OVERRUN Mask            */

#define ADC_ADDR15_VALID_Pos             (17)                                              /*!< ADC_T::ADDR15: VALID Position          */
#define ADC_ADDR15_VALID_Msk             (0x1ul << ADC_ADDR15_VALID_Pos)                   /*!< ADC_T::ADDR15: VALID Mask              */

#define ADC_ADDR16_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR16: RSLT Position           */
#define ADC_ADDR16_RSLT_Msk              (0xfffful << ADC_ADDR16_RSLT_Pos)                 /*!< ADC_T::ADDR16: RSLT Mask               */

#define ADC_ADDR16_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR16: OVERRUN Position        */
#define ADC_ADDR16_OVERRUN_Msk           (0x1ul << ADC_ADDR16_OVERRUN_Pos)                 /*!< ADC_T::ADDR16: OVERRUN Mask            */

#define ADC_ADDR16_VALID_Pos             (17)                                              /*!< ADC_T::ADDR16: VALID Position          */
#define ADC_ADDR16_VALID_Msk             (0x1ul << ADC_ADDR16_VALID_Pos)                   /*!< ADC_T::ADDR16: VALID Mask              */

#define ADC_ADDR17_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR17: RSLT Position           */
#define ADC_ADDR17_RSLT_Msk              (0xfffful << ADC_ADDR17_RSLT_Pos)                 /*!< ADC_T::ADDR17: RSLT Mask               */

#define ADC_ADDR17_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR17: OVERRUN Position        */
#define ADC_ADDR17_OVERRUN_Msk           (0x1ul << ADC_ADDR17_OVERRUN_Pos)                 /*!< ADC_T::ADDR17: OVERRUN Mask            */

#define ADC_ADDR17_VALID_Pos             (17)                                              /*!< ADC_T::ADDR17: VALID Position          */
#define ADC_ADDR17_VALID_Msk             (0x1ul << ADC_ADDR17_VALID_Pos)                   /*!< ADC_T::ADDR17: VALID Mask              */

#define ADC_ADDR18_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR18: RSLT Position           */
#define ADC_ADDR18_RSLT_Msk              (0xfffful << ADC_ADDR18_RSLT_Pos)                 /*!< ADC_T::ADDR18: RSLT Mask               */

#define ADC_ADDR18_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR18: OVERRUN Position        */
#define ADC_ADDR18_OVERRUN_Msk           (0x1ul << ADC_ADDR18_OVERRUN_Pos)                 /*!< ADC_T::ADDR18: OVERRUN Mask            */

#define ADC_ADDR18_VALID_Pos             (17)                                              /*!< ADC_T::ADDR18: VALID Position          */
#define ADC_ADDR18_VALID_Msk             (0x1ul << ADC_ADDR18_VALID_Pos)                   /*!< ADC_T::ADDR18: VALID Mask              */

#define ADC_ADDR19_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR19: RSLT Position           */
#define ADC_ADDR19_RSLT_Msk              (0xfffful << ADC_ADDR19_RSLT_Pos)                 /*!< ADC_T::ADDR19: RSLT Mask               */

#define ADC_ADDR19_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR19: OVERRUN Position        */
#define ADC_ADDR19_OVERRUN_Msk           (0x1ul << ADC_ADDR19_OVERRUN_Pos)                 /*!< ADC_T::ADDR19: OVERRUN Mask            */

#define ADC_ADDR19_VALID_Pos             (17)                                              /*!< ADC_T::ADDR19: VALID Position          */
#define ADC_ADDR19_VALID_Msk             (0x1ul << ADC_ADDR19_VALID_Pos)                   /*!< ADC_T::ADDR19: VALID Mask              */

#define ADC_ADDR20_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR20: RSLT Position           */
#define ADC_ADDR20_RSLT_Msk              (0xfffful << ADC_ADDR20_RSLT_Pos)                 /*!< ADC_T::ADDR20: RSLT Mask               */

#define ADC_ADDR20_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR20: OVERRUN Position        */
#define ADC_ADDR20_OVERRUN_Msk           (0x1ul << ADC_ADDR20_OVERRUN_Pos)                 /*!< ADC_T::ADDR20: OVERRUN Mask            */

#define ADC_ADDR20_VALID_Pos             (17)                                              /*!< ADC_T::ADDR20: VALID Position          */
#define ADC_ADDR20_VALID_Msk             (0x1ul << ADC_ADDR20_VALID_Pos)                   /*!< ADC_T::ADDR20: VALID Mask              */

#define ADC_ADDR21_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR21: RSLT Position           */
#define ADC_ADDR21_RSLT_Msk              (0xfffful << ADC_ADDR21_RSLT_Pos)                 /*!< ADC_T::ADDR21: RSLT Mask               */

#define ADC_ADDR21_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR21: OVERRUN Position        */
#define ADC_ADDR21_OVERRUN_Msk           (0x1ul << ADC_ADDR21_OVERRUN_Pos)                 /*!< ADC_T::ADDR21: OVERRUN Mask            */

#define ADC_ADDR21_VALID_Pos             (17)                                              /*!< ADC_T::ADDR21: VALID Position          */
#define ADC_ADDR21_VALID_Msk             (0x1ul << ADC_ADDR21_VALID_Pos)                   /*!< ADC_T::ADDR21: VALID Mask              */

#define ADC_ADDR22_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR22: RSLT Position           */
#define ADC_ADDR22_RSLT_Msk              (0xfffful << ADC_ADDR22_RSLT_Pos)                 /*!< ADC_T::ADDR22: RSLT Mask               */

#define ADC_ADDR22_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR22: OVERRUN Position        */
#define ADC_ADDR22_OVERRUN_Msk           (0x1ul << ADC_ADDR22_OVERRUN_Pos)                 /*!< ADC_T::ADDR22: OVERRUN Mask            */

#define ADC_ADDR22_VALID_Pos             (17)                                              /*!< ADC_T::ADDR22: VALID Position          */
#define ADC_ADDR22_VALID_Msk             (0x1ul << ADC_ADDR22_VALID_Pos)                   /*!< ADC_T::ADDR22: VALID Mask              */

#define ADC_ADDR23_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR23: RSLT Position           */
#define ADC_ADDR23_RSLT_Msk              (0xfffful << ADC_ADDR23_RSLT_Pos)                 /*!< ADC_T::ADDR23: RSLT Mask               */

#define ADC_ADDR23_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR23: OVERRUN Position        */
#define ADC_ADDR23_OVERRUN_Msk           (0x1ul << ADC_ADDR23_OVERRUN_Pos)                 /*!< ADC_T::ADDR23: OVERRUN Mask            */

#define ADC_ADDR23_VALID_Pos             (17)                                              /*!< ADC_T::ADDR23: VALID Position          */
#define ADC_ADDR23_VALID_Msk             (0x1ul << ADC_ADDR23_VALID_Pos)                   /*!< ADC_T::ADDR23: VALID Mask              */

#define ADC_ADDR29_RSLT_Pos              (0)                                               /*!< ADC_T::ADDR29: RSLT Position           */
#define ADC_ADDR29_RSLT_Msk              (0xfffful << ADC_ADDR29_RSLT_Pos)                 /*!< ADC_T::ADDR29: RSLT Mask               */

#define ADC_ADDR29_OVERRUN_Pos           (16)                                              /*!< ADC_T::ADDR29: OVERRUN Position        */
#define ADC_ADDR29_OVERRUN_Msk           (0x1ul << ADC_ADDR29_OVERRUN_Pos)                 /*!< ADC_T::ADDR29: OVERRUN Mask            */

#define ADC_ADDR29_VALID_Pos             (17)                                              /*!< ADC_T::ADDR29: VALID Position          */
#define ADC_ADDR29_VALID_Msk             (0x1ul << ADC_ADDR29_VALID_Pos)                   /*!< ADC_T::ADDR29: VALID Mask              */

#define ADC_ADCR_ADEN_Pos                (0)                                               /*!< ADC_T::ADCR: ADEN Position             */
#define ADC_ADCR_ADEN_Msk                (0x1ul << ADC_ADCR_ADEN_Pos)                      /*!< ADC_T::ADCR: ADEN Mask                 */

#define ADC_ADCR_ADIE_Pos                (1)                                               /*!< ADC_T::ADCR: ADIE Position             */
#define ADC_ADCR_ADIE_Msk                (0x1ul << ADC_ADCR_ADIE_Pos)                      /*!< ADC_T::ADCR: ADIE Mask                 */

#define ADC_ADCR_ADMD_Pos                (2)                                               /*!< ADC_T::ADCR: ADMD Position             */
#define ADC_ADCR_ADMD_Msk                (0x3ul << ADC_ADCR_ADMD_Pos)                      /*!< ADC_T::ADCR: ADMD Mask                 */

#define ADC_ADCR_TRGCOND_Pos             (6)                                               /*!< ADC_T::ADCR: TRGCOND Position          */
#define ADC_ADCR_TRGCOND_Msk             (0x3ul << ADC_ADCR_TRGCOND_Pos)                   /*!< ADC_T::ADCR: TRGCOND Mask              */

#define ADC_ADCR_TRGEN_Pos               (8)                                               /*!< ADC_T::ADCR: TRGEN Position            */
#define ADC_ADCR_TRGEN_Msk               (0x1ul << ADC_ADCR_TRGEN_Pos)                     /*!< ADC_T::ADCR: TRGEN Mask                */

#define ADC_ADCR_PTEN_Pos                (9)                                               /*!< ADC_T::ADCR: PTEN Position             */
#define ADC_ADCR_PTEN_Msk                (0x1ul << ADC_ADCR_PTEN_Pos)                      /*!< ADC_T::ADCR: PTEN Mask                 */

#define ADC_ADCR_ADST_Pos                (11)                                              /*!< ADC_T::ADCR: ADST Position             */
#define ADC_ADCR_ADST_Msk                (0x1ul << ADC_ADCR_ADST_Pos)                      /*!< ADC_T::ADCR: ADST Mask                 */

#define ADC_ADCR_ADDRRST_Pos             (14)                                              /*!< ADC_T::ADCR: ADDRRST Position          */
#define ADC_ADCR_ADDRRST_Msk             (0x1ul << ADC_ADCR_ADDRRST_Pos)                   /*!< ADC_T::ADCR: ADDRRST Mask              */

#define ADC_ADCR_TRGS_Pos                (16)                                              /*!< ADC_T::ADCR: TRGS Position             */
#define ADC_ADCR_TRGS_Msk                (0xful << ADC_ADCR_TRGS_Pos)                      /*!< ADC_T::ADCR: TRGS Mask                 */

#define ADC_ADCR_STBSEL_Pos              (24)                                              /*!< ADC_T::ADCR: STBSEL Position           */
#define ADC_ADCR_STBSEL_Msk              (0x3ul << ADC_ADCR_STBSEL_Pos)                    /*!< ADC_T::ADCR: STBSEL Mask               */

#define ADC_ADCHER_CHEN_Pos              (0)                                               /*!< ADC_T::ADCHER: CHEN Position           */
#define ADC_ADCHER_CHEN_Msk              (0xfffffffful << ADC_ADCHER_CHEN_Pos)             /*!< ADC_T::ADCHER: CHEN Mask               */

#define ADC_ADCMPR_CMPEN_Pos             (0)                                               /*!< ADC_T::ADCMPR: CMPEN Position         */
#define ADC_ADCMPR_CMPEN_Msk             (0x1ul << ADC_ADCMPR_CMPEN_Pos)                   /*!< ADC_T::ADCMPR: CMPEN Mask             */

#define ADC_ADCMPR_CMPIE_Pos             (1)                                               /*!< ADC_T::ADCMPR: CMPIE Position         */
#define ADC_ADCMPR_CMPIE_Msk             (0x1ul << ADC_ADCMPR_CMPIE_Pos)                   /*!< ADC_T::ADCMPR: CMPIE Mask             */

#define ADC_ADCMPR_CMPCOND_Pos           (2)                                               /*!< ADC_T::ADCMPR: CMPCOND Position       */
#define ADC_ADCMPR_CMPCOND_Msk           (0x1ul << ADC_ADCMPR_CMPCOND_Pos)                 /*!< ADC_T::ADCMPR: CMPCOND Mask           */

#define ADC_ADCMPR_CMPCH_Pos             (3)                                               /*!< ADC_T::ADCMPR: CMPCH Position         */
#define ADC_ADCMPR_CMPCH_Msk             (0x1ful << ADC_ADCMPR_CMPCH_Pos)                  /*!< ADC_T::ADCMPR: CMPCH Mask             */

#define ADC_ADCMPR_CMPMCNT_Pos           (8)                                               /*!< ADC_T::ADCMPR: CMPMCNT Position       */
#define ADC_ADCMPR_CMPMCNT_Msk           (0xful << ADC_ADCMPR_CMPMCNT_Pos)                 /*!< ADC_T::ADCMPR: CMPMCNT Mask           */

#define ADC_ADCMPR_CMPWEN_Pos            (15)                                              /*!< ADC_T::ADCMPR: CMPWEN Position        */
#define ADC_ADCMPR_CMPWEN_Msk            (0x1ul << ADC_ADCMPR_CMPWEN_Pos)                  /*!< ADC_T::ADCMPR: CMPWEN Mask            */

#define ADC_ADCMPR_CMPD_Pos              (16)                                              /*!< ADC_T::ADCMPR: CMPD Position          */
#define ADC_ADCMPR_CMPD_Msk              (0xffful << ADC_ADCMPR_CMPD_Pos)                  /*!< ADC_T::ADCMPR: CMPD Mask              */

#define ADC_ADCMPR0_CMPEN_Pos            (0)                                               /*!< ADC_T::ADCMPR0: CMPEN Position         */
#define ADC_ADCMPR0_CMPEN_Msk            (0x1ul << ADC_ADCMPR0_CMPEN_Pos)                  /*!< ADC_T::ADCMPR0: CMPEN Mask             */

#define ADC_ADCMPR0_CMPIE_Pos            (1)                                               /*!< ADC_T::ADCMPR0: CMPIE Position         */
#define ADC_ADCMPR0_CMPIE_Msk            (0x1ul << ADC_ADCMPR0_CMPIE_Pos)                  /*!< ADC_T::ADCMPR0: CMPIE Mask             */

#define ADC_ADCMPR0_CMPCOND_Pos          (2)                                               /*!< ADC_T::ADCMPR0: CMPCOND Position       */
#define ADC_ADCMPR0_CMPCOND_Msk          (0x1ul << ADC_ADCMPR0_CMPCOND_Pos)                /*!< ADC_T::ADCMPR0: CMPCOND Mask           */

#define ADC_ADCMPR0_CMPCH_Pos            (3)                                               /*!< ADC_T::ADCMPR0: CMPCH Position         */
#define ADC_ADCMPR0_CMPCH_Msk            (0x1ful << ADC_ADCMPR0_CMPCH_Pos)                 /*!< ADC_T::ADCMPR0: CMPCH Mask             */

#define ADC_ADCMPR0_CMPMCNT_Pos          (8)                                               /*!< ADC_T::ADCMPR0: CMPMCNT Position       */
#define ADC_ADCMPR0_CMPMCNT_Msk          (0xful << ADC_ADCMPR0_CMPMCNT_Pos)                /*!< ADC_T::ADCMPR0: CMPMCNT Mask           */

#define ADC_ADCMPR0_CMPWEN_Pos           (15)                                              /*!< ADC_T::ADCMPR0: CMPWEN Position        */
#define ADC_ADCMPR0_CMPWEN_Msk           (0x1ul << ADC_ADCMPR0_CMPWEN_Pos)                 /*!< ADC_T::ADCMPR0: CMPWEN Mask            */

#define ADC_ADCMPR0_CMPD_Pos             (16)                                              /*!< ADC_T::ADCMPR0: CMPD Position          */
#define ADC_ADCMPR0_CMPD_Msk             (0xffful << ADC_ADCMPR0_CMPD_Pos)                 /*!< ADC_T::ADCMPR0: CMPD Mask              */

#define ADC_ADCMPR1_CMPEN_Pos            (0)                                               /*!< ADC_T::ADCMPR1: CMPEN Position         */
#define ADC_ADCMPR1_CMPEN_Msk            (0x1ul << ADC_ADCMPR1_CMPEN_Pos)                  /*!< ADC_T::ADCMPR1: CMPEN Mask             */

#define ADC_ADCMPR1_CMPIE_Pos            (1)                                               /*!< ADC_T::ADCMPR1: CMPIE Position         */
#define ADC_ADCMPR1_CMPIE_Msk            (0x1ul << ADC_ADCMPR1_CMPIE_Pos)                  /*!< ADC_T::ADCMPR1: CMPIE Mask             */

#define ADC_ADCMPR1_CMPCOND_Pos          (2)                                               /*!< ADC_T::ADCMPR1: CMPCOND Position       */
#define ADC_ADCMPR1_CMPCOND_Msk          (0x1ul << ADC_ADCMPR1_CMPCOND_Pos)                /*!< ADC_T::ADCMPR1: CMPCOND Mask           */

#define ADC_ADCMPR1_CMPCH_Pos            (3)                                               /*!< ADC_T::ADCMPR1: CMPCH Position         */
#define ADC_ADCMPR1_CMPCH_Msk            (0x1ful << ADC_ADCMPR1_CMPCH_Pos)                 /*!< ADC_T::ADCMPR1: CMPCH Mask             */

#define ADC_ADCMPR1_CMPMCNT_Pos          (8)                                               /*!< ADC_T::ADCMPR1: CMPMCNT Position       */
#define ADC_ADCMPR1_CMPMCNT_Msk          (0xful << ADC_ADCMPR1_CMPMCNT_Pos)                /*!< ADC_T::ADCMPR1: CMPMCNT Mask           */

#define ADC_ADCMPR1_CMPWEN_Pos           (15)                                              /*!< ADC_T::ADCMPR1: CMPWEN Position        */
#define ADC_ADCMPR1_CMPWEN_Msk           (0x1ul << ADC_ADCMPR1_CMPWEN_Pos)                 /*!< ADC_T::ADCMPR1: CMPWEN Mask            */

#define ADC_ADCMPR1_CMPD_Pos             (16)                                              /*!< ADC_T::ADCMPR1: CMPD Position          */
#define ADC_ADCMPR1_CMPD_Msk             (0xffful << ADC_ADCMPR1_CMPD_Pos)                 /*!< ADC_T::ADCMPR1: CMPD Mask              */

#define ADC_ADSR0_ADF_Pos                (0)                                               /*!< ADC_T::ADSR0: ADF Position             */
#define ADC_ADSR0_ADF_Msk                (0x1ul << ADC_ADSR0_ADF_Pos)                      /*!< ADC_T::ADSR0: ADF Mask                 */

#define ADC_ADSR0_CMPF0_Pos              (1)                                               /*!< ADC_T::ADSR0: CMPF0 Position           */
#define ADC_ADSR0_CMPF0_Msk              (0x1ul << ADC_ADSR0_CMPF0_Pos)                    /*!< ADC_T::ADSR0: CMPF0 Mask               */

#define ADC_ADSR0_CMPF1_Pos              (2)                                               /*!< ADC_T::ADSR0: CMPF1 Position           */
#define ADC_ADSR0_CMPF1_Msk              (0x1ul << ADC_ADSR0_CMPF1_Pos)                    /*!< ADC_T::ADSR0: CMPF1 Mask               */

#define ADC_ADSR0_BUSY_Pos               (7)                                               /*!< ADC_T::ADSR0: BUSY Position            */
#define ADC_ADSR0_BUSY_Msk               (0x1ul << ADC_ADSR0_BUSY_Pos)                     /*!< ADC_T::ADSR0: BUSY Mask                */

#define ADC_ADSR0_VALIDF_Pos             (8)                                               /*!< ADC_T::ADSR0: VALIDF Position          */
#define ADC_ADSR0_VALIDF_Msk             (0x1ul << ADC_ADSR0_VALIDF_Pos)                   /*!< ADC_T::ADSR0: VALIDF Mask              */

#define ADC_ADSR0_OVERRUNF_Pos           (16)                                              /*!< ADC_T::ADSR0: OVERRUNF Position        */
#define ADC_ADSR0_OVERRUNF_Msk           (0x1ul << ADC_ADSR0_OVERRUNF_Pos)                 /*!< ADC_T::ADSR0: OVERRUNF Mask            */

#define ADC_ADSR0_ADCRDY_Pos             (24)                                              /*!< ADC_T::ADSR0: ADCRDY Position          */
#define ADC_ADSR0_ADCRDY_Msk             (0x1ul << ADC_ADSR0_ADCRDY_Pos)                   /*!< ADC_T::ADSR0: ADCRDY Mask              */

#define ADC_ADSR0_CHANNEL_Pos            (27)                                              /*!< ADC_T::ADSR0: CHANNEL Position         */
#define ADC_ADSR0_CHANNEL_Msk            (0x1ful << ADC_ADSR0_CHANNEL_Pos)                 /*!< ADC_T::ADSR0: CHANNEL Mask             */

#define ADC_ADSR1_VALID_Pos              (0)                                               /*!< ADC_T::ADSR1: VALID Position           */
#define ADC_ADSR1_VALID_Msk              (0xfffffffful << ADC_ADSR1_VALID_Pos)             /*!< ADC_T::ADSR1: VALID Mask               */

#define ADC_ADSR2_OVERRUN_Pos            (0)                                               /*!< ADC_T::ADSR2: OVERRUN Position         */
#define ADC_ADSR2_OVERRUN_Msk            (0xfffffffful << ADC_ADSR2_OVERRUN_Pos)           /*!< ADC_T::ADSR2: OVERRUN Mask             */

#define ADC_ESMPCTL_EXTSMPT_Pos          (0)                                               /*!< ADC_T::ESMPCTL: EXTSMPT Position       */
#define ADC_ESMPCTL_EXTSMPT_Msk          (0x3ffful << ADC_ESMPCTL_EXTSMPT_Pos)             /*!< ADC_T::ESMPCTL: EXTSMPT Mask           */

#define ADC_ADPDMA_CURDAT_Pos            (0)                                               /*!< ADC_T::ADPDMA: CURDAT Position         */
#define ADC_ADPDMA_CURDAT_Msk            (0x3fffful << ADC_ADPDMA_CURDAT_Pos)              /*!< ADC_T::ADPDMA: CURDAT Mask             */

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __ADC_REG_H__ */
