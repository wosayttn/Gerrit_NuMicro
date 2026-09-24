/**************************************************************************//**
 * @file     adc_reg.h
 * @version  V1.00
 * @brief    ADC register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
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
     * @var ADC_T::ADDR
     * Offset: 0x00-0x48  ADC Data Register 0-18
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1.
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed. This bit will be cleared to 0 by hardware after ADDR register is read.
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
     * |        |          |Note: Before starting A/D conversion function, this bit should be set to 1. Clear it to 0 to disable A/D converter analog circuit to save power consumption.
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
     * |        |          |These two bits decide external pin ADC0_ST trigger event is level or edge. The signal must be kept at stable state at least 8 PCLKs for level trigger and at least 4 PCLKs for edge trigger.
     * |        |          |00 = Low level.
     * |        |          |01 = High level.
     * |        |          |10 = Falling edge.
     * |        |          |11 = Rising edge.
     * |[8]     |TRGEN     |External Trigger Enable Bit
     * |        |          |Enable or disable triggering of A/D conversion by external ADC0_ST pin, BPWM trigger, PWM trigger, ACMP interrupt event and Timer trigger.
     * |        |          |If external trigger is enabled, the ADST bit can be set to 1 by the selected hardware trigger source.
     * |        |          |0 = External trigger Disabled.
     * |        |          |1 = External trigger Enabled.
     * |[9]     |PTEN      |PDMA Transfer Enable Bit
     * |        |          |When A/D conversion is completed, the converted data is loaded into ADDR0~18.
     * |        |          |Software can enable this bit to generate a PDMA data transfer request.
     * |        |          |0 = PDMA data transfer Disabled.
     * |        |          |1 = PDMA data transfer in ADDR0~18 Enabled.
     * |        |          |Note: When PTEN=1, software must set ADIE=0 to disable interrupt.
     * |[11]    |ADST      |A/D Conversion Start
     * |        |          |ADST bit can be set to 1 from five sources: software, external pin ADC0_ST, PWM trigger, BPWM trigger, ACMP trigger and Timer trigger.
     * |        |          |ADST bit will be cleared to 0 by hardware automatically at the ends of Single mode, Single-cycle Scan mode.
     * |        |          |In Continuous Scan mode and Burst mode, A/D conversion is continuously performed until software writes 0 to this bit or chip is reset.
     * |        |          |0 = Conversion stops and A/D converter enters idle state.
     * |        |          |1 = Conversion starts.
     * |        |          |Note 1: When ADST becomes from 1 to 0, ADC macro will reset to initial state.
     * |        |          |After macro reset to initial state, user should wait at most 2 ADC clock and set this bit to start next conversion.
     * |[14]    |ADDRRST   |ADDRx Reset
     * |        |          |If user writes this bit, VALID and OVERRUN of all ADDRx will be reset.
     * |        |          |Note: This bit is cleared by hardware.
     * |[19:16] |TRGS      |Hardware Trigger Source
     * |        |          |0000 = A/D conversion is started by external ADC0_ST pin.
     * |        |          |0001 = Timer0 ~ Timer3 overflow pulse trigger.
     * |        |          |0010 = A/D conversion is started by BPWM trigger.
     * |        |          |0011 = A/D conversion is started by PWM trigger.
     * |        |          |1000 = ACMP0_O edge event.
     * |        |          |1001 = ACMP1_O edge event.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: Software should clear TRGEN bit and ADST bit to 0 before changing TRGS bits.
     * |        |          |Note 2: These trigger sources are only abaliable when chip is in run and idle mode.
     * |[21:20] |ACMPTES   |ACMP Trigger Event Selection
     * |        |          |00 = ACMP0/1 both edge event as trigger source.
     * |        |          |01 = ACMP0/1 rising edge event as trigger source.
     * |        |          |10 = ACMP0/1 falling edge event as trigger source.
     * |        |          |11 = Reserved.
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
     * |[18:0]  |CHEN      |Analog   Input Channel Enable Control
     * |        |          |Set ADCHER[18:0] bits to enable the corresponding analog external input channel 18 ~ 0.
     * |        |          |Besides, setting the ADCHER[16] bit will enable internal channel for AVDD divided by four, setting the ADCHER[17] bit will enable internal channel for band-gap voltage, setting the ADCHER[18] bit will enable internal channel for temperature sensor output voltage.
     * |        |          |Other bits are reserved.
     * |        |          |0 = Channel Disabled.
     * |        |          |1 = Channel Enabled.
     * |        |          |Note: If the internal channel for band-gap voltage (CHEN[16]), temperature sensor output voltage (CHEN[17]), AVDD divide by four (CHEN[18]), the maximum sampling rate will be 300k SPS.
     * @var ADC_T::ADCMPR
     * Offset: 0x88/0x8C  ADC Compare Register 0/1
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
     * |        |          |10000 = Band-gap voltage conversion result is selected to be compared.
     * |        |          |10001 = Temperature sensor voltage conversion result is selected to be compared.
     * |        |          |10010 = AVDD/4 channel conversion result is selected to be compared.
     * |        |          |Others = Reserved.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND bit, the internal match counter will increase 1. When the internal counter reaches the value to (CMPMCNT +1), the CMPFx bit will be set.
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
     * |        |          |A status flag that indicates the end of A/D conversion. Software can write 1 to clear this bit.
     * |        |          |The ADF bit is set to 1 at the following three conditions:
     * |        |          |1. When A/D conversion ends in Single mode.
     * |        |          |2. When A/D conversion ends on all specified channels in Single-cycle Scan mode and Continuous Scan mode.
     * |        |          |3. When number of samples in the FIFO is greater than or equal to half of its size in Burst mode.
     * |[1]     |CMPF0     |Compare Flag 0
     * |        |          |When the A/D conversion result of the selected channel meets setting condition in ADCMPR0 register then this bit is set to 1.
     * |        |          |This bit is cleared by writing 1 to it.
     * |        |          |0 = Conversion result in ADDR does not meet ADCMPR0 setting.
     * |        |          |1 = Conversion result in ADDR meets ADCMPR0 setting.
     * |[2]     |CMPF1     |Compare Flag 1
     * |        |          |When the A/D conversion result of the selected channel meets setting condition in ADCMPR1 register, this bit is set to 1; it is cleared by writing 1 to it.
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
     * |[24]    |ADPRDY    |ADC Power On Ready (Read Only)
     * |        |          |When ADEN (ADC_ADCR[0]) set to 1 or auto-operation Trigger to enable ADC, the ADC need a stable time.
     * |        |          |The ADPRDY will keep low until ADC stable.
     * |        |          |The ADC power ready time depends on STBSEL (ADC_ ADCR [25:24]).
     * |        |          |Note: ADC can start converting after ADPRDY is 1.
     * |[31:27] |CHANNEL   |Current Conversion Channel (Read Only)
     * |        |          |When BUSY=1, this filed reflects current conversion channel.
     * |        |          |When BUSY=0, it shows the number of the next converted channel.
     * @var ADC_T::ADSR1
     * Offset: 0x94  ADC Status Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[18:0]  |VALID     |Data Valid Flag (Read Only)
     * |        |          |VALID[18:0] are the mirror of the VALID bits in ADDR18[17]~ ADDR0[17]. The other bits are reserved.
     * |        |          |Note: When ADC is in burst mode and any conversion result is valid, VALID[18:0] will be set to 1.
     * @var ADC_T::ADSR2
     * Offset: 0x98  ADC Status Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[18:0]  |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |OVERRUN[18:0] are the mirror of the OVERRUN bit in ADDR18[16] ~ ADDR0[16]. The other bits are reserved.
     * |        |          |Note: When ADC is in burst mode and the FIFO is overrun, OVERRUN[18:0] will be set to 1.
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
     * |        |          |Current PDMA transfer data could be the content of ADDR0 ~ ADDR18 registers.
     * @var ADC_T::AUTOCTL
     * Offset: 0x800  ADC Auto Operation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TRIGSEL   |Automatic Operation Trigger Source Select
     * |        |          |0000 = Auto-operation Trigger Source from TMR0.
     * |        |          |0001 = Auto-operation Trigger Source from TMR1.
     * |        |          |0010 = Auto-operation Trigger Source from TMR2.
     * |        |          |0011 = Auto-operation Trigger Source from TMR3.
     * |        |          |0100 = Auto-operation Trigger Source from WKIOA0.
     * |        |          |0101 = Auto-operation Trigger Source from WKIOB0.
     * |        |          |0110 = Auto-operation Trigger Source from WKIOC0.
     * |        |          |0111 = Auto-operation Trigger Source from WKIOD0.
     * |        |          |1000 = Auto-operation Trigger Source from ACMP0 analog output edge event.
     * |        |          |1001 = Auto-operation Trigger Source from ACMP1 analog output edge event.
     * |        |          |Others = Reserved.
     * |        |          |Note: TRIGSEL cannot be changed when TRIGEN is 1.
     * |[4]     |TRIGEN    |Automatic Operation Trigger Enable Bit
     * |        |          |When automatic operation mode is enabled, the ADC will start working if ADC is triggered by a event sent from the trigger source selected by TRIGSEL[3:0] after this bit is set to 1.
     * |        |          |0 = ADC Automatic Operation Trigger Disabled.
     * |        |          |1 = ADC Automatic Operation Trigger Enabled.
     * |[8]     |ADWKEN    |Automatic Operation Mode Conversion End Wake-up Enable Bit
     * |        |          |0 = ADC automatic operation conversion end wake-up Disabled.
     * |        |          |1 = ADC automatic operation conversion end wake-up Enabled.
     * |[9]     |CMP0WKEN  |Automatic Operation Mode Comparator 0 Wake-up Enable Bit
     * |        |          |0 = ADC automatic operation comparator 0 wake-up Disabled.
     * |        |          |1 = ADC automatic operation comparator 0 wake-up Enabled.
     * |[10]    |CMP1WKEN  |Automatic Operation Mode Comparator 1 Wake-up Enable Bit
     * |        |          |0 = ADC automatic operation comparator 1 wake-up Disabled.
     * |        |          |1 = ADC automatic operation comparator 1 wake-up Enabled.
     * |[31]    |AUTOEN    |Automatic Operation Mode Enable Bit
     * |        |          |0 = ADC automatic operation Disabled.
     * |        |          |1 = ADC automatic operation Enabled.
     * @var ADC_T::AUTOSTRG
     * Offset: 0x804  ADC Auto Operation Software Trigger Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SWTRIG    |Automatic Operation Software Trigger Bit
     * |        |          |Write 1 to this bit will trigger ADC start automatic operation.
     * |        |          |Note: This bit will be auto cleared by hardware.
     * @var ADC_T::AUTOSTS
     * Offset: 0x810  ADC Auto Operation Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADWKF     |Automatic Operation Conversion End Wake-up Flag Bit
     * |        |          |When automatic operation mode is enabled, and the conversion is finished, this flag will be set
     * |        |          |If WKEN is also set to 1, the chip will be woken up.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |CMP0WKF   |Automatic Operation Compare 0 Wake-up Flag Bit
     * |        |          |When automatic operation mode is enabled, and the conversion result monitor comparator 1 matches setting criteria, this flag will be set
     * |        |          |If CMP1WKEN is also set to 1, the chip will be woken up.
     * |        |          |Note: Software can write1 to clear this bit.
     * |[2]     |CMP1WKF   |Automatic Operation Compare 1 Wake-up Flag Bit
     * |        |          |When automatic operation mode is enabled, and the conversion result monitor comparator 0 is match setting criteria, this flag will be set
     * |        |          |If CMP0WKEN is also set to 1, the chip will be woken up.
     * |        |          |Note: Software can write 1 to clear this bit.
     */
    __I  uint32_t ADDR[19];         /*!< [0x0000-0x0048] ADC Data Register 0 ~ 18                       */
    __I  uint32_t RESERVE0[13];
    __IO uint32_t ADCR;             /*!< [0x0080] ADC Control Register                                  */
    __IO uint32_t ADCHER;           /*!< [0x0084] ADC Channel Enable Register                           */
    __IO uint32_t ADCMPR[2];        /*!< [0x0088-0x008c] ADC Compare Register 0 ~ 1                     */
    __IO uint32_t ADSR0;            /*!< [0x0090] ADC Status Register0                                  */
    __I  uint32_t ADSR1;            /*!< [0x0094] ADC Status Register1                                  */
    __I  uint32_t ADSR2;            /*!< [0x0098] ADC Status Register2                                  */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t ESMPCTL;          /*!< [0x00a0] ADC Extend Sample Time Control Register               */
    __I  uint32_t RESERVE2[23];
    __I  uint32_t ADPDMA;           /*!< [0x0100] ADC PDMA Current Transfer Data Register               */
    __I  uint32_t RESERVE3[447];
    __IO uint32_t AUTOCTL;          /*!< [0x0800] ADC Auto Operation Control Register                   */
    __O  uint32_t AUTOSTRG;         /*!< [0x0804] ADC Auto Operation Software Trigger Register          */
    __I  uint32_t RESERVE4[2];
    __IO uint32_t AUTOSTS;          /*!< [0x0810] ADC Auto Operation Status Register                    */
} ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */

#define ADC_ADDR_RSLT_Pos               (0)                                         /*!< ADC_T::ADDR: RSLT Position                 */
#define ADC_ADDR_RSLT_Msk               (0xfffful << ADC_ADDR_RSLT_Pos)             /*!< ADC_T::ADDR: RSLT Mask                     */

#define ADC_ADDR_OVERRUN_Pos            (16)                                        /*!< ADC_T::ADDR: OVERRUN Position              */
#define ADC_ADDR_OVERRUN_Msk            (0x1ul << ADC_ADDR_OVERRUN_Pos)             /*!< ADC_T::ADDR: OVERRUN Mask                  */

#define ADC_ADDR_VALID_Pos              (17)                                        /*!< ADC_T::ADDR: VALID Position                */
#define ADC_ADDR_VALID_Msk              (0x1ul << ADC_ADDR_VALID_Pos)               /*!< ADC_T::ADDR: VALID Mask                    */

#define ADC_ADCR_ADEN_Pos               (0)                                         /*!< ADC_T::ADCR: ADEN Position                 */
#define ADC_ADCR_ADEN_Msk               (0x1ul << ADC_ADCR_ADEN_Pos)                /*!< ADC_T::ADCR: ADEN Mask                     */

#define ADC_ADCR_ADIE_Pos               (1)                                         /*!< ADC_T::ADCR: ADIE Position                 */
#define ADC_ADCR_ADIE_Msk               (0x1ul << ADC_ADCR_ADIE_Pos)                /*!< ADC_T::ADCR: ADIE Mask                     */

#define ADC_ADCR_ADMD_Pos               (2)                                         /*!< ADC_T::ADCR: ADMD Position                 */
#define ADC_ADCR_ADMD_Msk               (0x3ul << ADC_ADCR_ADMD_Pos)                /*!< ADC_T::ADCR: ADMD Mask                     */

#define ADC_ADCR_TRGCOND_Pos            (6)                                         /*!< ADC_T::ADCR: TRGCOND Position              */
#define ADC_ADCR_TRGCOND_Msk            (0x3ul << ADC_ADCR_TRGCOND_Pos)             /*!< ADC_T::ADCR: TRGCOND Mask                  */

#define ADC_ADCR_TRGEN_Pos              (8)                                         /*!< ADC_T::ADCR: TRGEN Position                */
#define ADC_ADCR_TRGEN_Msk              (0x1ul << ADC_ADCR_TRGEN_Pos)               /*!< ADC_T::ADCR: TRGEN Mask                    */

#define ADC_ADCR_PTEN_Pos               (9)                                         /*!< ADC_T::ADCR: PTEN Position                 */
#define ADC_ADCR_PTEN_Msk               (0x1ul << ADC_ADCR_PTEN_Pos)                /*!< ADC_T::ADCR: PTEN Mask                     */

#define ADC_ADCR_ADST_Pos               (11)                                        /*!< ADC_T::ADCR: ADST Position                 */
#define ADC_ADCR_ADST_Msk               (0x1ul << ADC_ADCR_ADST_Pos)                /*!< ADC_T::ADCR: ADST Mask                     */

#define ADC_ADCR_ADDRRST_Pos            (14)                                        /*!< ADC_T::ADCR: ADDRRST Position              */
#define ADC_ADCR_ADDRRST_Msk            (0x1ul << ADC_ADCR_ADDRRST_Pos)             /*!< ADC_T::ADCR: ADDRRST Mask                  */

#define ADC_ADCR_TRGS_Pos               (16)                                        /*!< ADC_T::ADCR: TRGS Position                 */
#define ADC_ADCR_TRGS_Msk               (0xful << ADC_ADCR_TRGS_Pos)                /*!< ADC_T::ADCR: TRGS Mask                     */

#define ADC_ADCR_ACMPTES_Pos            (20)                                        /*!< ADC_T::ADCR: ACMPTES Position              */
#define ADC_ADCR_ACMPTES_Msk            (0x3ul << ADC_ADCR_ACMPTES_Pos)             /*!< ADC_T::ADCR: ACMPTES Mask                  */

#define ADC_ADCR_STBSEL_Pos             (24)                                        /*!< ADC_T::ADCR: STBSEL Position               */
#define ADC_ADCR_STBSEL_Msk             (0x3ul << ADC_ADCR_STBSEL_Pos)              /*!< ADC_T::ADCR: STBSEL Mask                   */

#define ADC_ADCHER_CHEN_Pos             (0)                                         /*!< ADC_T::ADCHER: CHEN Position               */
#define ADC_ADCHER_CHEN_Msk             (0x7fffful << ADC_ADCHER_CHEN_Pos)          /*!< ADC_T::ADCHER: CHEN Mask                   */

#define ADC_ADCMPR_CMPEN_Pos            (0)                                         /*!< ADC_T::ADCMPR: CMPEN Position              */
#define ADC_ADCMPR_CMPEN_Msk            (0x1ul << ADC_ADCMPR_CMPEN_Pos)             /*!< ADC_T::ADCMPR: CMPEN Mask                  */

#define ADC_ADCMPR_CMPIE_Pos            (1)                                         /*!< ADC_T::ADCMPR: CMPIE Position              */
#define ADC_ADCMPR_CMPIE_Msk            (0x1ul << ADC_ADCMPR_CMPIE_Pos)             /*!< ADC_T::ADCMPR: CMPIE Mask                  */

#define ADC_ADCMPR_CMPCOND_Pos          (2)                                         /*!< ADC_T::ADCMPR: CMPCOND Position            */
#define ADC_ADCMPR_CMPCOND_Msk          (0x1ul << ADC_ADCMPR_CMPCOND_Pos)           /*!< ADC_T::ADCMPR: CMPCOND Mask                */

#define ADC_ADCMPR_CMPCH_Pos            (3)                                         /*!< ADC_T::ADCMPR: CMPCH Position              */
#define ADC_ADCMPR_CMPCH_Msk            (0x1ful << ADC_ADCMPR_CMPCH_Pos)            /*!< ADC_T::ADCMPR: CMPCH Mask                  */

#define ADC_ADCMPR_CMPMCNT_Pos          (8)                                         /*!< ADC_T::ADCMPR: CMPMCNT Position            */
#define ADC_ADCMPR_CMPMCNT_Msk          (0xful << ADC_ADCMPR_CMPMCNT_Pos)           /*!< ADC_T::ADCMPR: CMPMCNT Mask                */

#define ADC_ADCMPR_CMPWEN_Pos           (15)                                        /*!< ADC_T::ADCMPR: CMPWEN Position             */
#define ADC_ADCMPR_CMPWEN_Msk           (0x1ul << ADC_ADCMPR_CMPWEN_Pos)            /*!< ADC_T::ADCMPR: CMPWEN Mask                 */

#define ADC_ADCMPR_CMPD_Pos             (16)                                        /*!< ADC_T::ADCMPR: CMPD Position               */
#define ADC_ADCMPR_CMPD_Msk             (0xffful << ADC_ADCMPR_CMPD_Pos)            /*!< ADC_T::ADCMPR: CMPD Mask                   */

#define ADC_ADSR0_ADF_Pos               (0)                                         /*!< ADC_T::ADSR0: ADF Position                 */
#define ADC_ADSR0_ADF_Msk               (0x1ul << ADC_ADSR0_ADF_Pos)                /*!< ADC_T::ADSR0: ADF Mask                     */

#define ADC_ADSR0_CMPF0_Pos             (1)                                         /*!< ADC_T::ADSR0: CMPF0 Position               */
#define ADC_ADSR0_CMPF0_Msk             (0x1ul << ADC_ADSR0_CMPF0_Pos)              /*!< ADC_T::ADSR0: CMPF0 Mask                   */

#define ADC_ADSR0_CMPF1_Pos             (2)                                         /*!< ADC_T::ADSR0: CMPF1 Position               */
#define ADC_ADSR0_CMPF1_Msk             (0x1ul << ADC_ADSR0_CMPF1_Pos)              /*!< ADC_T::ADSR0: CMPF1 Mask                   */

#define ADC_ADSR0_BUSY_Pos              (7)                                         /*!< ADC_T::ADSR0: BUSY Position                */
#define ADC_ADSR0_BUSY_Msk              (0x1ul << ADC_ADSR0_BUSY_Pos)               /*!< ADC_T::ADSR0: BUSY Mask                    */

#define ADC_ADSR0_VALIDF_Pos            (8)                                         /*!< ADC_T::ADSR0: VALIDF Position              */
#define ADC_ADSR0_VALIDF_Msk            (0x1ul << ADC_ADSR0_VALIDF_Pos)             /*!< ADC_T::ADSR0: VALIDF Mask                  */

#define ADC_ADSR0_OVERRUNF_Pos          (16)                                        /*!< ADC_T::ADSR0: OVERRUNF Position            */
#define ADC_ADSR0_OVERRUNF_Msk          (0x1ul << ADC_ADSR0_OVERRUNF_Pos)           /*!< ADC_T::ADSR0: OVERRUNF Mask                */

#define ADC_ADSR0_ADPRDY_Pos            (24)                                        /*!< ADC_T::ADSR0: ADPRDY Position              */
#define ADC_ADSR0_ADPRDY_Msk            (0x1ul << ADC_ADSR0_ADPRDY_Pos)             /*!< ADC_T::ADSR0: ADPRDY Mask                  */

#define ADC_ADSR0_CHANNEL_Pos           (27)                                        /*!< ADC_T::ADSR0: CHANNEL Position             */
#define ADC_ADSR0_CHANNEL_Msk           (0x1ful << ADC_ADSR0_CHANNEL_Pos)           /*!< ADC_T::ADSR0: CHANNEL Mask                 */

#define ADC_ADSR1_VALID_Pos             (0)                                         /*!< ADC_T::ADSR1: VALID Position               */
#define ADC_ADSR1_VALID_Msk             (0x7fffful << ADC_ADSR1_VALID_Pos)          /*!< ADC_T::ADSR1: VALID Mask                   */

#define ADC_ADSR2_OVERRUN_Pos           (0)                                         /*!< ADC_T::ADSR2: OVERRUN Position             */
#define ADC_ADSR2_OVERRUN_Msk           (0x7fffful << ADC_ADSR2_OVERRUN_Pos)        /*!< ADC_T::ADSR2: OVERRUN Mask                 */

#define ADC_ESMPCTL_EXTSMPT_Pos         (0)                                         /*!< ADC_T::ESMPCTL: EXTSMPT Position           */
#define ADC_ESMPCTL_EXTSMPT_Msk         (0x3ffful << ADC_ESMPCTL_EXTSMPT_Pos)       /*!< ADC_T::ESMPCTL: EXTSMPT Mask               */

#define ADC_ADPDMA_CURDAT_Pos           (0)                                         /*!< ADC_T::ADPDMA: CURDAT Position             */
#define ADC_ADPDMA_CURDAT_Msk           (0x3fffful << ADC_ADPDMA_CURDAT_Pos)        /*!< ADC_T::ADPDMA: CURDAT Mask                 */

#define ADC_AUTOCTL_TRIGSEL_Pos         (0)                                         /*!< ADC_T::AUTOCTL: TRIGSEL Position           */
#define ADC_AUTOCTL_TRIGSEL_Msk         (0xful << ADC_AUTOCTL_TRIGSEL_Pos)          /*!< ADC_T::AUTOCTL: TRIGSEL Mask               */

#define ADC_AUTOCTL_TRIGEN_Pos          (4)                                         /*!< ADC_T::AUTOCTL: TRIGEN Position            */
#define ADC_AUTOCTL_TRIGEN_Msk          (0x1ul << ADC_AUTOCTL_TRIGEN_Pos)           /*!< ADC_T::AUTOCTL: TRIGEN Mask                */

#define ADC_AUTOCTL_ADWKEN_Pos          (8)                                         /*!< ADC_T::AUTOCTL: ADWKEN Position            */
#define ADC_AUTOCTL_ADWKEN_Msk          (0x1ul << ADC_AUTOCTL_ADWKEN_Pos)           /*!< ADC_T::AUTOCTL: ADWKEN Mask                */

#define ADC_AUTOCTL_CMP0WKEN_Pos        (9)                                         /*!< ADC_T::AUTOCTL: CMP0WKEN Position          */
#define ADC_AUTOCTL_CMP0WKEN_Msk        (0x1ul << ADC_AUTOCTL_CMP0WKEN_Pos)         /*!< ADC_T::AUTOCTL: CMP0WKEN Mask              */

#define ADC_AUTOCTL_CMP1WKEN_Pos        (10)                                        /*!< ADC_T::AUTOCTL: CMP1WKEN Position          */
#define ADC_AUTOCTL_CMP1WKEN_Msk        (0x1ul << ADC_AUTOCTL_CMP1WKEN_Pos)         /*!< ADC_T::AUTOCTL: CMP1WKEN Mask              */

#define ADC_AUTOCTL_AUTOEN_Pos          (31)                                        /*!< ADC_T::AUTOCTL: AUTOEN Position            */
#define ADC_AUTOCTL_AUTOEN_Msk          (0x1ul << ADC_AUTOCTL_AUTOEN_Pos)           /*!< ADC_T::AUTOCTL: AUTOEN Mask                */

#define ADC_AUTOSTRG_SWTRIG_Pos         (0)                                         /*!< ADC_T::AUTOSTRG: SWTRIG Position           */
#define ADC_AUTOSTRG_SWTRIG_Msk         (0x1ul << ADC_AUTOSTRG_SWTRIG_Pos)          /*!< ADC_T::AUTOSTRG: SWTRIG Mask               */

#define ADC_AUTOSTS_ADWKF_Pos           (0)                                         /*!< ADC_T::AUTOSTS: ADWKF Position             */
#define ADC_AUTOSTS_ADWKF_Msk           (0x1ul << ADC_AUTOSTS_ADWKF_Pos)            /*!< ADC_T::AUTOSTS: ADWKF Mask                 */

#define ADC_AUTOSTS_CMP0WKF_Pos         (1)                                         /*!< ADC_T::AUTOSTS: CMP0WKF Position           */
#define ADC_AUTOSTS_CMP0WKF_Msk         (0x1ul << ADC_AUTOSTS_CMP0WKF_Pos)          /*!< ADC_T::AUTOSTS: CMP0WKF Mask               */

#define ADC_AUTOSTS_CMP1WKF_Pos         (2)                                         /*!< ADC_T::AUTOSTS: CMP1WKF Position           */
#define ADC_AUTOSTS_CMP1WKF_Msk         (0x1ul << ADC_AUTOSTS_CMP1WKF_Pos)          /*!< ADC_T::AUTOSTS: CMP1WKF Mask               */

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif


#endif /* __ADC_REG_H__ */
