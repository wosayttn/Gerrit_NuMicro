/**************************************************************************//**
 * @file     acmp_reg.h
 * @version  V1.00
 * @brief    ACMP register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __ACMP_REG_H__
#define __ACMP_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

  @{

*/


/*---------------------- Analog Comparator Controller -------------------------*/
/**
    @addtogroup ACMP Analog Comparator Controller(ACMP)
    Memory Mapped Structure for ACMP Controller
@{ */
typedef struct
{


    /**
     * @var ACMP_T::CTL0
     * Offset: 0x00  Analog Comparator Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |ACMP Enable Bit
     * |        |          |0 = ACMP 0 Disabled.
     * |        |          |1 = ACMP 0 Enabled.
     * |[1]     |ACMPIE    |ACMP Interrupt Enable Bit
     * |        |          |0 = ACMP0 interrupt Disabled.
     * |        |          |1 = ACMP0 interrupt Enabled.
     * |        |          |Note:If WKEN (ACMP_CTL0[16]) is set to 1, the wake-up interrupt function will be enabled as well.
     * |[3]     |ACMPOINV  |ACMP Output Inverse
     * |        |          |0 = ACMP0 output inverse Disabled.
     * |        |          |1 = ACMP0 output inverse Enabled.
     * |[6:4]   |NEGSEL    |ACMP Negative Input Selection
     * |        |          |000 = ACMP0_N.
     * |        |          |001 = Internal comparator reference voltage (CRV0).
     * |        |          |010 = Band-gap voltage.
     * |        |          |Others = Reserved.
     * |        |          |Note: NEGSEL must select 0x1 in calibration mode.
     * |[10:8]  |POSSEL    |ACMP Positive Input Selection
     * |        |          |000 = ACMP0_P0 pin.
     * |        |          |001 = ACMP0_P1 pin.
     * |        |          |010 = ACMP0_P2 pin.
     * |        |          |011 = ACMP0_P3 pin.
     * |        |          |Others = Reserved.
     * |[12]    |OUTSEL    |ACMP Output Select
     * |        |          |0 = ACMP0 output to ACMP0_O pin is unfiltered ACMP output.
     * |        |          |1 = ACMP0 output to ACMP0_O pin is from filter output.
     * |[15:13] |FILTSEL   |ACMP Output Filter Count Selection
     * |        |          |000 = Filter function is Disabled.
     * |        |          |001 = ACMP0 output is sampled 1 consecutive PCLK.
     * |        |          |010 = ACMP0 output is sampled 2 consecutive PCLKs.
     * |        |          |011 = ACMP0 output is sampled 4 consecutive PCLKs.
     * |        |          |100 = ACMP0 output is sampled 8 consecutive PCLKs.
     * |        |          |101 = ACMP0 output is sampled 16 consecutive PCLKs.
     * |        |          |110 = ACMP0 output is sampled 32 consecutive PCLKs.
     * |        |          |111 = ACMP0 output is sampled 64 consecutive PCLKs.
     * |[16]    |WKEN      |Power-down Wake-up Enable Bit
     * |        |          |0 = Wake-up function Disabled.
     * |        |          |1 = Wake-up function Enabled.
     * |[17]    |WLATEN    |Window Latch Mode Enable Bit
     * |        |          |0 = Window Latch Mode Disabled.
     * |        |          |1 = Window Latch Mode Enabled.
     * |[18]    |WCMPSEL   |Window Compare Mode Selection
     * |        |          |0 = Window Compare Mode Disabled.
     * |        |          |1 = Window Compare Mode is Selected.
     * |[21:20] |FCLKDIV   |ACMP Output Filter Clock Divider
     * |        |          |00 = ACMP0 output filter clock = PCLK.
     * |        |          |01 = ACMP0 output filter clock = PCLK/2.
     * |        |          |10 = ACMP0 output filter clock = PCLK/4.
     * |        |          |11 = Reserved.
     * |        |          |Note: Use FCLKDIV under the condition of FILTSEL = 3u2019h7, then set FCLKDIV to get the effect of filtering 128,256 consecutive PCLKs.
     * |[26:24] |HYSSEL    |Hysteresis Mode Selection
     * |        |          |000 = Hysteresis is 0mV.
     * |        |          |001 = Hysteresis is 10mV.
     * |        |          |010 = Hysteresis is 20mV.
     * |        |          |011 = Hysteresis is 30mV.
     * |        |          |100 = Hysteresis is 40mV.
     * |        |          |Others = Reserved.
     * |[29:28] |MODESEL   |ACMP Operating Mode Selection
     * |        |          |The register affects analog characteristics as operating current, input offset voltage, setup time, propagation delay, etc.
     * |        |          |00 = ACMP0 operating mode0.
     * |        |          |01 = ACMP0 operating mode1.
     * |        |          |10 = ACMP0 operating mode2.
     * |        |          |11 = ACMP0 operating mode3.
     * |        |          |Note: Please refer to Analog comparator (ACMP) of Analog characteristics in the Datasheet.
     * |[31:30] |INTPOL    |Interrupt Condition Polarity Selection
     * |        |          |ACMPIF0 will be set to 1 when ACMP output edge condition is detected.
     * |        |          |00 = Rising edge or falling edge.
     * |        |          |01 = Rising edge.
     * |        |          |10 = Falling edge.
     * |        |          |11 = Reserved.
     * @var ACMP_T::CTL1
     * Offset: 0x04  Analog Comparator Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |ACMP Enable Bit
     * |        |          |0 = ACMP1 Disabled.
     * |        |          |1 = ACMP1 Enabled.
     * |[1]     |ACMPIE    |ACMP Interrupt Enable Bit
     * |        |          |0 = ACMP1 interrupt Disabled.
     * |        |          |1 = ACMP1 interrupt Enabled.
     * |        |          |Note:If WKEN (ACMP_CTL1[16]) is set to 1, the wake-up interrupt function will be enabled as well.
     * |[3]     |ACMPOINV  |ACMP Output Inverse
     * |        |          |0 = ACMP1 output inverse Disabled.
     * |        |          |1 = ACMP1 output inverse Enabled.
     * |[6:4]   |NEGSEL    |ACMP Negative Input Selection
     * |        |          |000 = ACMP1_N.
     * |        |          |001 = Internal comparator reference voltage (CRV1).
     * |        |          |010 = Band-gap voltage.
     * |        |          |Others = Reserved.
     * |        |          |Note: NEGSEL must select 0x1 in calibration mode.
     * |[10:8]  |POSSEL    |ACMP Positive Input Selection
     * |        |          |000 = ACMP1_P0 pin.
     * |        |          |001 = ACMP1_P1 pin.
     * |        |          |010 = ACMP1_P2 pin.
     * |        |          |011 = ACMP1_P3 pin.
     * |        |          |Others = Reserved.
     * |[12]    |OUTSEL    |ACMP Output Selection
     * |        |          |0 = ACMP1 output to ACMP1_O pin is unfiltered ACMP output.
     * |        |          |1 = ACMP1 output to ACMP1_O pin is from filter output.
     * |[15:13] |FILTSEL   |ACMP Output Filter Count Selection
     * |        |          |000 = Filter function is Disabled.
     * |        |          |001 = ACMP1 output is sampled 1 consecutive PCLK.
     * |        |          |010 = ACMP1 output is sampled 2 consecutive PCLKs.
     * |        |          |011 = ACMP1 output is sampled 4 consecutive PCLKs.
     * |        |          |100 = ACMP1 output is sampled 8 consecutive PCLKs.
     * |        |          |101 = ACMP1 output is sampled 16 consecutive PCLKs.
     * |        |          |110 = ACMP1 output is sampled 32 consecutive PCLKs.
     * |        |          |111 = ACMP1 output is sampled 64 consecutive PCLKs.
     * |[16]    |WKEN      |Power-down Wake-up Enable Bit
     * |        |          |0 = Wake-up function Disabled.
     * |        |          |1 = Wake-up function Enabled.
     * |[17]    |WLATEN    |Window Latch Mode Enable Bit
     * |        |          |0 = Window Latch Mode Disabled.
     * |        |          |1 = Window Latch Mode Enabled.
     * |[18]    |WCMPSEL   |Window Compare Mode Selection
     * |        |          |0 = Window Compare Mode Disabled.
     * |        |          |1 = Window Compare Mode is Selected.
     * |[21:20] |FCLKDIV   |ACMP Output Filter Clock Divider
     * |        |          |00 = ACMP1 output filter clock = PCLK.
     * |        |          |01 = ACMP1 output filter clock = PCLK/2.
     * |        |          |10 = ACMP1 output filter clock = PCLK/4.
     * |        |          |11 = Reserved.
     * |        |          |Note: Use FCLKDIV under the condition of FILTSEL = 3u2019h7, then set FCLKDIV to get the effect of filtering 128,256 consecutive PCLKs.[MW1]
     * |        |          |[MW1]u61C9u8A72u5F37u8ABFu53EAu6709u5728FILSEL=7u7684u60C5u6CC1u4E0B, FCLKDIVu624Du6709u4F5Cu7528uFF0Cu6B64u8655u7684note u5BB9u6613u8AA4u6703u53EAu662Fu8209u4F8B
     * |[26:24] |HYSSEL    |Hysteresis Mode Selection
     * |        |          |000 = Hysteresis is 0mV.
     * |        |          |001 = Hysteresis is 10mV.
     * |        |          |010 = Hysteresis is 20mV.
     * |        |          |011 = Hysteresis is 30mV.
     * |        |          |100 = Hysteresis is 40mV.
     * |        |          |Others = Reserved.
     * |[29:28] |MODESEL   |ACMP Operating Mode Selection
     * |        |          |The register affects analog characteristics as operating current, input offset voltage, setup time, propagation delay, etc.
     * |        |          |00 = ACMP1 operating mode0.
     * |        |          |01 = ACMP1 operating mode1.
     * |        |          |10 = ACMP1 operating mode2.
     * |        |          |11 = ACMP1 operating mode3.
     * |        |          |Note: Please refer to Analog comparator (ACMP) of Analog characteristics in the Datasheet.
     * |[31:30] |INTPOL    |Interrupt Condition Polarity Selection
     * |        |          |ACMPIF1 will be set to 1 when ACMP output edge condition is detected.
     * |        |          |00 = Rising edge or falling edge.
     * |        |          |01 = Rising edge.
     * |        |          |10 = Falling edge.
     * |        |          |11 = Reserved.
     * @var ACMP_T::STATUS
     * Offset: 0x08  Analog Comparator Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPIF0   |ACMP Interrupt Flag 0
     * |        |          |This bit is set by hardware when the edge condition defined by INTPOL (ACMP_CTL0[31:30]) is detected on ACMP0 output
     * |        |          |This will generate an interrupt if ACMPIE (ACMP_CTL0[1]) is set to 1.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[1]     |ACMPIF1   |ACMP Interrupt Flag 1
     * |        |          |This bit is set by hardware when the edge condition defined by INTPOL (ACMP_CTL1[31:30]) is detected on ACMP1 output
     * |        |          |This will cause an interrupt if ACMPIE (ACMP_CTL1[1]) is set to 1.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |ACMPO0    |ACMP Output 0
     * |        |          |Synchronized to the PCLK to allow reading by software
     * |        |          |Cleared when the ACMP0 is disabled, i.e
     * |        |          |ACMPEN (ACMP_CTL0[0]) is cleared to 0.
     * |[5]     |ACMPO1    |ACMP Output 1
     * |        |          |Synchronized to the PCLK to allow reading by software
     * |        |          |Cleared when the ACMP1 is disabled, i.e
     * |        |          |ACMPEN (ACMP_CTL1[0]) is cleared to 0.
     * |[8]     |WKIF0     |ACMP Power-down Wake-up Interrupt Flag 0
     * |        |          |This bit will be set to 1 when ACMP0 wake-up interrupt event occurs.
     * |        |          |0 = No power-down wake-up occurred.
     * |        |          |1 = Power-down wake-up occurred.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[9]     |WKIF1     |ACMP Power-down Wake-up Interrupt Flag 1
     * |        |          |This bit will be set to 1 when ACMP1 wake-up interrupt event occurs.
     * |        |          |0 = No power-down wake-up occurred.
     * |        |          |1 = Power-down wake-up occurred.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[12]    |ACMPS0    |ACMP Status 0
     * |        |          |Synchronized to the PCLK to allow reading by software
     * |        |          |Cleared when the ACMP 0 is disabled, i.e
     * |        |          |ACMPEN (ACMP_CTL0[0]) is cleared to 0.
     * |[13]    |ACMPS1    |ACMP Status 1
     * |        |          |Synchronized to the PCLK to allow reading by software
     * |        |          |Cleared when the ACMP1 is disabled, i.e
     * |        |          |ACMPEN (ACMP_CTL1[0]) is cleared to 0.
     * |[16]    |ACMPWO    |ACMP Window Output
     * |        |          |This bit shows the output status of window compare mode
     * |        |          |0 = The positive input voltage is outside the window.
     * |        |          |1 = The positive input voltage is in the window.
     * @var ACMP_T::VREF
     * Offset: 0x0C  Analog Comparator Reference Voltage Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CRV0SEL   |ACMP Reference Voltage Setting 0
     * |        |          |Set CRV output voltage for ACMP0 negtive input.
     * |        |          |CRV0 = CRV0 source voltage * (CRV0SEL)/ 63.
     * |[6]     |CRV0SSEL  |CRV0 Source Voltage Selection
     * |        |          |0 = AVDD is selected as CRV0 source voltage.
     * |        |          |1 = The reference voltage defined by VREFCTL (SYS_VREFCTL[4:0]) register is selected as CRV0 source voltage.
     * |[8]     |CRV0EN    |CRV0 Enable Bit
     * |        |          |0 = CRV0 Disabled.
     * |        |          |1 = CRV0 Enabled.
     * |[15]    |CLAMPEN0  |ACMP Speed Up Function Enable Bit
     * |        |          |0 = ACMP0 Speed Up Function Disabled.
     * |        |          |1 = ACMP0 Speed Up Function Enabled.
     * |        |          |Note: ACMP speed up function only support MODESEL(ACMP_CTL0 [29:28])=2bu201911 & 2bu201910.
     * |[21:16] |CRV1SEL   |ACMP Reference Voltage Setting 1
     * |        |          |Set CRV output voltage for ACMP1 negtive input.
     * |        |          |CRV1 = CRV1 source voltage * (CRV1SEL)/ 63.
     * |[22]    |CRV1SSEL  |CRV1 Source Voltage Selection
     * |        |          |0 = AVDD is selected as CRV1 source voltage.
     * |        |          |1 = The reference voltage defined by VREFCTL (SYS_VREFCTL[4:0]) register is selected as CRV1 source voltage.
     * |[24]    |CRV1EN    |CRV1 Enable Bit
     * |        |          |0 = CRV1 Disabled.
     * |        |          |1 = CRV1 Enabled.
     * |[31]    |CLAMPEN1  |ACMP Speed Up Function Enable Bit
     * |        |          |0 = ACMP1 Speed Up Function Disabled.
     * |        |          |1 = ACMP1 Speed Up Function Enabled.
     * |        |          |Note: ACMP speed up function only support MODESEL(ACMP_CTL1 [29:28])=2bu201911 & 2bu201910.
     * @var ACMP_T::CALCTL
     * Offset: 0x10  Analog Comparator Calibration Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CALTRG0   |ACMP Calibration Trigger 0
     * |        |          |Set ACMP0 calibration trigger.
     * |        |          |0 = Calibration is stopped.
     * |        |          |1 = Calibration is triggered.
     * |        |          |Note 1: Before this bit is enabled, ACMPEN(ACMP_CTL0[0]) should be set and the internal high speed RC oscillator (HIRC) should be enabled in advance.
     * |        |          |Note 2: Hardware will automatically clear this bit when the next calibration is triggered by software.
     * |        |          |Note 3: If user must trigger calibration twice or more times, the second trigger has to wait at least 300us after the previous calibration is done.
     * |[1]     |CALTRG1   |ACMP Calibration Trigger 1
     * |        |          |Set ACMP1 calibration trigger.
     * |        |          |0 = Calibration is stopped.
     * |        |          |1 = Calibration is triggered.
     * |        |          |Note 1: Before this bit is enabled, ACMPEN(ACMP_CTL1[0]) should be set and the internal high speed RC oscillator (HIRC) should be enabled in advance.
     * |        |          |Note 2: Hardware will automatically clear this bit when the next calibration is triggered by software.
     * |        |          |Note 3: If user must trigger calibration twice or more times, the second trigger has to wait at least 300us after the previous calibration is done.
     * |[5:4]   |CALCLK0   |ACMP Calibration Clock Rate Selection 0
     * |        |          |Set ACMP0 calibration clock rate.
     * |        |          |00 = 1.5 kHz.
     * |        |          |01 = 6 kHz.
     * |        |          |10 = 24 kHz.
     * |        |          |11 = 95 kHz.
     * |[7:6]   |CALCLK1   |ACMP Calibration Clock Rate Selection 1
     * |        |          |Set ACMP1 calibration clock rate.
     * |        |          |00 = 1.5 kHz.
     * |        |          |01 = 6 kHz.
     * |        |          |10 = 24 kHz.
     * |        |          |11 = 95 kHz.
     * |[8]     |OFFSETSEL |ACMP Trim Code Selection
     * |        |          |0 = calibration trim code will not minus 1 when calibrated done.
     * |        |          |1 = calibration trim code will not minus 1 when calibrated done.
     * |[17:16] |CALRVS    |Calibration Reference Voltage Selection
     * |        |          |00 = option0 (N-pair calibration: 5V - 80mV, P-pair calibration: 80mV).
     * |        |          |01 = option1.
     * |        |          |10 = option2 (N-pair calibration: 5V - 160mV, P-pair calibration: 160mV).
     * |        |          |11 = Reserved.
     * |        |          |Note: CRV0 and CRV1 must be the same setting in calibration
     * |        |          |Note: The details refer to Analog ACMP SPEC
     * @var ACMP_T::CALSTS
     * Offset: 0x14  Analog Comparator Calibration Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DONE0     |ACMP Calibration Done Status 0
     * |        |          |This bit will be set when ACMP0 calibration is done.
     * |        |          |0 = Calibrating.
     * |        |          |1 = Calibration done.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |DONE1     |ACMP Calibration Done Status 1
     * |        |          |This bit will be set when ACMP1 calibration is done.
     * |        |          |0 = Calibrating.
     * |        |          |1 = Calibration done.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * @var ACMP_T::COFF
     * Offset: 0xFF0  Analog Comparator Calibration Offset Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |NCODE0    |ACMP Offset 0 of NMOS
     * |        |          |ACMP0 offset canceling trim code of NMOS
     * |        |          |Note: 1. Once ACMP0 is enabled, reading these bits will gets initial value from ROMMAP46/50[19:16]
     * |        |          |2. write MODESEL ACMP_CTL0[29:28] will decide NCODE0 load from which ROMMAP
     * |        |          | MODESEL = 2u2019b00, NCODE0 load from ROM......... MAP46[3:0]
     * |        |          | MODESEL = 2u2019b01, NCODE0 load from ROM......... MAP46[19:16]
     * |        |          | MODESEL = 2u2019b10, NCODE0 load from ROM......... MAP47[3:0]
     * |        |          | MODESEL = 2u2019b11, NCODE0 load from ROM......... MAP47[19:16]
     * |[7]     |NSEL0     |ACMP Offset 0 of NMOS
     * |        |          |0 = trim NMOS negative offset.
     * |        |          |1 = trim NMOS positive offset.
     * |        |          |Note: 1. Once ACMP0 is enabled, reading this bit default will get initial value from ROMMAP46/50[20]
     * |        |          | 2. write MODESEL ACMP_CTL0[29:28] will decide NSEL0 load from which ROMMAP
     * |        |          | MODESEL = 2u2019b00, NSEL0 load from ROM......... MAP46[7]
     * |        |          | MODESEL = 2u2019b01, NSEL0 load from ROM......... MAP46[23]
     * |        |          | MODESEL = 2u2019b10, NSEL0 load from ROM......... MAP47[7]
     * |        |          | MODESEL = 2u2019b11, NSEL0 load from ROM......... MAP47[23]
     * |        |          |2
     * |        |          |If ACMP0 is enabled and CALTRG0 (ACMP_CALCTL[0]]) is set, after calibration done DONE0(ACMP_CALSTS[0]) will get NSEL0 value
     * |[11:8]  |PCODE0    |ACMP Offset 0 of PMOS
     * |        |          |ACMP0 offset canceling trim code of PMOS
     * |        |          |Note: 1
     * |        |          |Once ACMP0 is enabled, reading these bits default will get initial value from ROMMAP46/50[27:24]
     * |        |          |2. write MODESEL ACMP_CTL0[29:28] will decide PCODE0 load from which ROMMAP
     * |        |          | MODESEL = 2u2019b00, PCODE0 load from ROM......... MAP46[11:8]
     * |        |          | MODESEL = 2u2019b01, PCODE0 load from ROM......... MAP46[27:24]
     * |        |          | MODESEL = 2u2019b10, PCODE0 load from ROM......... MAP47[11:8]
     * |        |          | MODESEL = 2u2019b11, PCODE0 load from ROM......... MAP47[27:24]
     * |[15]    |PSEL0     |ACMP Offset 0 of PMOS
     * |        |          |0 = trim PMOS negative offset.
     * |        |          |1 = trim PMOS positive offset.
     * |        |          |Note: 1. Once ACMP0 is enabled, reading this bit default will get initial value from ROMMAP46/50[28].
     * |        |          |2. write MODESEL ACMP_CTL0[29:28] will decide PSEL0 load from which ROMMAP
     * |        |          | MODESEL = 2u2019b00, PSEL0 load from ROM......... MAP46[15]
     * |        |          | MODESEL = 2u2019b01, PSEL0 load from ROM......... MAP46[31]
     * |        |          | MODESEL = 2u2019b10, PSEL0 load from ROM......... MAP47[15]
     * |        |          | MODESEL = 2u2019b11, PSEL0 load from ROM......... MAP47[31]
     * |        |          |3
     * |        |          |If ACMP0 is enabled and CALTRG0 (ACMP_CALCTL[0]]) is set, after calibration done DONE0(ACMP_CALSTS[0]) will get PSEL0 value
     * |[19:16] |NCODE1    |ACMP Offset 1 of NMOS
     * |        |          |ACMP1 offset canceling trim code of PMOS
     * |        |          |Note: 1
     * |        |          |Once ACMP1 is enabled, reading these bits default will get initial value from ROMMAP48/52[19:16]
     * |        |          |2. write MODESEL ACMP_CTL1[29:28] will decide NCODE1 load from which ROMMAP
     * |        |          | MODESEL = 2u2019b00, NCODE1 load from ROM......... MAP48[3:0]
     * |        |          | MODESEL = 2u2019b01, NCODE1 load from ROM......... MAP48[19:16]
     * |        |          | MODESEL = 2u2019b10, NCODE1 load from ROM......... MAP49[3:0]
     * |        |          | MODESEL = 2u2019b11, NCODE1 load from ROM......... MAP49[19:16]
     * |[23]    |NSEL1     |ACMP Offset 1 of NMOS
     * |        |          |0 = trim NMOS negative offset.
     * |        |          |1 = trim NMOS positive offset.
     * |        |          |Note: 1. Once ACMP1 is enabled, reading this bit default will get initial value from ROMMAP48/52[20]
     * |        |          | 2. write MODESEL ACMP_CTL1[29:28] will decide NSEL1 load from which ROMMAP
     * |        |          | MODESEL = 2u2019b00, NSEL1 load from ROM......... MAP48[7]
     * |        |          | MODESEL = 2u2019b01, NSEL1 load from ROM......... MAP48[23]
     * |        |          | MODESEL = 2u2019b10, NSEL1 load from ROM......... MAP49[7]
     * |        |          | MODESEL = 2u2019b11, NSEL1 load from ROM......... MAP49[23]
     * |        |          |3
     * |        |          |If ACMP1 is enabled and CALTRG1 (ACMP_CALCTL[1]]) is set, after calibration done DONE1(ACMP_CALSTS[4]) will get NSEL1 value
     * |[27:24] |PCODE1    |ACMP Offset 1 of PMOS
     * |        |          |ACMP1 offset canceling trim code of PMOS
     * |        |          |Note: 1
     * |        |          |Once ACMP1 is enabled, reading these bits default will get initial value from ROMMAP48/52[27:24]
     * |        |          |2. write MODESEL ACMP_CTL1[29:28] will decide PCODE1 load from which ROMMAP
     * |        |          | MODESEL = 2u2019b00, PCODE1 load from ROM......... MAP48[11:8]
     * |        |          | MODESEL = 2u2019b01, PCODE1 load from ROM......... MAP48[27:24]
     * |        |          | MODESEL = 2u2019b10, PCODE1 load from ROM......... MAP49[11:8]
     * |        |          | MODESEL = 2u2019b11, PCODE1 load from ROM......... MAP49[27:24]
     * |[31]    |PSEL1     |ACMP Offset 1 of PMOS
     * |        |          |0 = trim PMOS negative offset.
     * |        |          |1 = trim PMOS positive offset.
     * |        |          |Note: 1. Once ACMP1 is enabled, reading this bit default will get initial value from ROMMAP48/52[28]
     * |        |          | 2. write MODESEL ACMP_CTL1[29:28] will decide PSEL1 load from which ROMMAP
     * |        |          | MODESEL = 2u2019b00, PSEL1 load from ROM......... MAP48[15]
     * |        |          | MODESEL = 2u2019b01, PSEL1 load from ROM......... MAP48[31]
     * |        |          | MODESEL = 2u2019b10, PSEL1 load from ROM......... MAP49[15]
     * |        |          | MODESEL = 2u2019b11, PSEL1 load from ROM......... MAP49[31]
     * |        |          |3
     * |        |          |If ACMP1 is enabled and CALTRG1 (ACMP_CALCTL[1]]) is set, after calibration done DONE1(ACMP_CALSTS[4]) will get PSEL1 value
     * @var ACMP_T::HYSOFF
     * Offset: 0xFF4  Analog Comparator Hysteresis Offset
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HYSN0     |The N-differential pair of ACMP interface control 0
     * |        |          |HYSN0 can monitor whether interface control of hysteresis to meet expectation for ACMP0.
     * |        |          |Note: 1. HYSSEL (ACMP_CTL0[26:24]) = 3u2019b000. HYSN0 is not from ROMMAP and keep 4u2019b0000.
     * |        |          | 2. Update HYSN0:
     * |        |          | a.ACMP enable or MODESEL (ACMP_CTL0[29:28]) change or HYSSEL (ACMP_CTL0[26:24]) change.
     * |        |          |1
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL0[26:24])=3u2019b001 from ROMu2026u2026.MAP50[3:0].
     * |        |          |2
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL0[26:24])=3u2019b001 from ROMu2026u2026.MAP51[3:0].
     * |        |          |3
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL0[26:24])=3u2019b010 from ROMu2026u2026.MAP50[11:8].
     * |        |          |4
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL0[26:24])=3u2019b010 from ROMu2026u2026.MAP51[11:8].
     * |        |          |5
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL0[26:24])=3u2019b011 from ROMu2026u2026.MAP50[19:16].
     * |        |          |6
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL0[26:24])=3u2019b011 from ROMu2026u2026.MAP51[19:16].
     * |        |          |7
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL0[26:24])=3u2019b100 from ROMu2026u2026.MAP50[27:24].
     * |        |          |8
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL0[26:24])=3u2019b100 from ROMu2026u2026.MAP51[27:24].
     * |        |          | b. 4u2019b0000 reload by reset.
     * |        |          | c. Software write register.
     * |[7:4]   |HYSP0     |The P-differential pair of ACMP interface control 0
     * |        |          |HYSP0 can monitor whether interface control of hysteresis to meet expectation for ACMP0.
     * |        |          |Note: 1. HYSSEL (ACMP_CTL0[26:24]) = 3u2019b000. HYSP0 is not from ROMMAP and keep 4u2019b0000.
     * |        |          | 2. Update HYSP0:
     * |        |          | a.ACMP enable or MODESEL (ACMP_CTL0[29:28]) change or HYSSEL (ACMP_CTL0[26:24]) change.
     * |        |          |1
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL0[26:24])=3u2019b001 from ROMu2026u2026.MAP50[7:4].
     * |        |          |2
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL0[26:24])=3u2019b001 from ROMu2026u2026.MAP51[7:4].
     * |        |          |3
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL0[26:24])=3u2019b010 from ROMu2026u2026.MAP50[15:12].
     * |        |          |4
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL0[26:24])=3u2019b010 from ROMu2026u2026.MAP51[15:12].
     * |        |          |5
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL0[26:24])=3u2019b011 from ROMu2026u2026.MAP50[23:20].
     * |        |          |6
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL0[26:24])=3u2019b011 from ROMu2026u2026.MAP51[23:20].
     * |        |          |7
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL0[26:24])=3u2019b100 from ROMu2026u2026.MAP50[3128].
     * |        |          |8
     * |        |          |MODESEL (ACMP_CTL0[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL0[26:24])=3u2019b100 from ROMu2026u2026.MAP51[31:28].
     * |        |          | b. 4u2019b0000 reload by reset.
     * |        |          | c. Software write register.
     * |[11:8]  |HYSN1     |The N-differential pair of ACMP interface control 1
     * |        |          |HYSN0 can monitor whether interface control of hysteresis to meet expectation for ACMP1.
     * |        |          |Note: 1. HYSSEL (ACMP_CTL1[26:24]) = 3u2019b000. HYSN0 is not from ROMMAP and keep 4u2019b0000.
     * |        |          | 2. Update HYSN1:
     * |        |          | a.ACMP enable or MODESEL (ACMP_CTL0[29:28]) change or HYSSEL (ACMP_CTL0[26:24]) change.
     * |        |          |1
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL1[26:24])=3u2019b001 from ROMu2026u2026.MAP52[3:0].
     * |        |          |2
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL1[26:24])=3u2019b001 from ROMu2026u2026.MAP53[3:0].
     * |        |          |3
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL1[26:24])=3u2019b010 from ROMu2026u2026.MAP52[11:8].
     * |        |          |4
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL1[26:24])=3u2019b010 from ROMu2026u2026.MAP53[11:8].
     * |        |          |5
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL1[26:24])=3u2019b011 from ROMu2026u2026.MAP52[19:16].
     * |        |          |6
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL1[26:24])=3u2019b011 from ROMu2026u2026.MAP53[19:16].
     * |        |          |7
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL1[26:24])=3u2019b100 from ROMu2026u2026.MAP52[27:24].
     * |        |          |8
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL1[26:24])=3u2019b100 from ROMu2026u2026.MAP53[27:24].
     * |        |          | b. 4u2019b0000 reload by reset.
     * |        |          | c. Software write register.
     * |[15:12] |HYSP1     |The P-differential pair of ACMP interface control 1
     * |        |          |HYSP0 can monitor whether interface control of hysteresis to meet expectation for ACMP1.
     * |        |          |Note: 1. HYSSEL (ACMP_CTL1[26:24]) = 3u2019b000. HYSP0 is not from ROMMAP and keep 4u2019b0000.
     * |        |          | 2. Update HYSP1:
     * |        |          | a.ACMP enable or MODESEL (ACMP_CTL1[29:28]) change or HYSSEL (ACMP_CTL0[26:24]) change.
     * |        |          |1
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL1[26:24])=3u2019b001 from ROMu2026u2026.MAP52[7:4].
     * |        |          |2
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL1[26:24])=3u2019b001 from ROMu2026u2026.MAP53[7:4].
     * |        |          |3
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL1[26:24])=3u2019b010 from ROMu2026u2026.MAP52[15:12].
     * |        |          |4
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL1[26:24])=3u2019b010 from ROMu2026u2026.MAP53[15:12].
     * |        |          |5
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL1[26:24])=3u2019b011 from ROMu2026u2026.MAP52[23:20].
     * |        |          |6
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL1[26:24])=3u2019b011 from ROMu2026u2026.MAP53[23:20].
     * |        |          |7
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b00 or 2u2019b01 and HYSSEL (ACMP_CTL1[26:24])=3u2019b100 from ROMu2026u2026.MAP52[3128].
     * |        |          |8
     * |        |          |MODESEL (ACMP_CTL1[29:28])=2u2019b10 or 2u2019b11 and HYSSEL (ACMP_CTL1[26:24])=3u2019b100 from ROMu2026u2026.MAP53[31:28].
     * |        |          | b. 4u2019b0000 reload by reset.
     * |        |          | c. Software write register.
     * @var ACMP_T::TEST
     * Offset: 0xFF8  Analog Comparator Test Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRV0TEST  |CRV0 Test Mode Enable Bit (Write Protect)
     * |        |          |0 = No effect.
     * |        |          |1 = CRV voltage output to ACMP0_N pin for voltage measure.
     * |        |          |This bit is designed for Nuvoton Lab use only.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note: 1.NEGSEL (ACMP_CTL0[6:4]) or NEGSEL (ACMP_CTL1[6:4]) must select to 3u2019b001 in CRV test mode
     * |        |          | 2.NEGSEL (ACMP_CTL0[6:4]) or NEGSEL (ACMP_CTL1[6:4]) must select to 3u2019b010 in VBG test mode.
     * |        |          | 3.ACMPEN (ACMP_CTL0[0]) or ACMPEN (ACMP_CTL1[0]) must select to 1u2019b1 in VBG test mode.
     * |[1]     |CRV1TEST  |CRV1 Test Mode Enable Bit (Write Protect)
     * |        |          |0 = No effect.
     * |        |          |1 = CRV voltage output to ACMP1_N pin for voltage measure.
     * |        |          |This bit is designed for Nuvoton Lab use only.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note: 1.NEGSEL (ACMP_CTL0[6:4]) or NEGSEL (ACMP_CTL1[6:4]) must select to 3u2019b001 in CRV test mode
     * |        |          | 2.NEGSEL (ACMP_CTL0[6:4]) or NEGSEL (ACMP_CTL1[6:4]) must select to 3u2019b010 in VBG test mode.
     * |        |          | 3.ACMPEN (ACMP_CTL0[0]) or ACMPEN (ACMP_CTL1[0]) must select to 1u2019b1 in VBG test mode.
     * |[2]     |PDNPAIR0  |ACMP Power Down N-Intput Pair 0
     * |        |          |0 = ACMP0 normal.
     * |        |          |1 = ACMP0 power down N-input pair.
     * |[3]     |PDNPAIR1  |ACMP Power Down N-Intput Pair 01
     * |        |          |0 = ACMP1 normal.
     * |        |          |1 = ACMP1 power down N-input pair.
     * |[4]     |OUTSEL0   |ACMP CRV Output Source Selection 0
     * |        |          |Set CRV output source selection for ACMP0.
     * |        |          |0 = CRV0 output from resistor string.
     * |        |          |1 = CRV0 output from band-gap voltage.
     * |[5]     |OUTSEL1   |ACMP CRV Output Source Selection 1
     * |        |          |Set CRV output source selection for ACMP1.
     * |        |          |0 = CRV1 output from resistor string.
     * |        |          |1 = CRV1 output from band-gap voltage.
     * |[6]     |OTEN0     |ACMP OFFSET Test Mode Enable Bits 0
     * |        |          |Set offset test for ACMP0.
     * |        |          |0 = ACMP0 offset test mode Disable.
     * |        |          |1 = ACMP0 offset test mode Enable.
     * |        |          |Note: If offset test mode enable, the ACMPx_N0 should add 1uF cap and ACMP will become an unity gain buffer
     * |        |          |User can measure P0 channel and N0 channel voltage to get offset value.
     * |[7]     |OTEN1     |ACMP OFFSET Test Mode Enable Bits 1
     * |        |          |Set offset test for ACMP1.
     * |        |          |0 = ACMP1 offset test mode Disable.
     * |        |          |1 = ACMP1 offset test mode Enable.
     * |        |          |Note: If offset test mode enable, the ACMPx_N0 should add 1uF cap and ACMP will become an unity gain buffer
     * |        |          |User can measure P0 channel and N0 channel voltage to get offset value.
     * |[8]     |HYSBYP0   |ACMP Hysteresis Adjust Function Selectionm0
     * |        |          |Set hysteresis adjust function for ACMP0.
     * |        |          |0 = adjust function is enabled.
     * |        |          |1 = adjust function is bypass.
     * |[9]     |HYSBYP1   |ACMP Hysteresis Adjust Function Selection 1
     * |        |          |Set hysteresis adjust function for ACMP1.
     * |        |          |0 = adjust function is enabled.
     * |        |          |1 = adjust function is bypass.
     * |[10]    |PDPPAIR0  |ACMP Power Down P-Intput Pair 0
     * |        |          |0 = ACMP0 normal.
     * |        |          |1 = ACMP0 power down P-input pair.
     * |[11]    |PDPPAIR1  |ACMP Power Down P-Intput Pair 1
     * |        |          |0 = ACMP1 normal.
     * |        |          |1 = ACMP1 power down P-input pair.
     * |[14]    |TESTHYSSEL0|ACMP Feedback Test Mode Hysteresis Select Control Pin 0
     * |        |          |0 = ACMP0 generate 0~+hysteresis/2.
     * |        |          |1 = ACMP0 generate 0~-hysteresis/2.
     * |[15]    |TESTHYSSEL1|ACMP Feedback Test Mode Hysteresis Select Control Pin 1
     * |        |          |0 = ACMP1 generate 0~+hysteresis/2.
     * |        |          |1 = ACMP1 generate 0~-hysteresis/2.
     * @var ACMP_T::VERSION
     * Offset: 0xFFC  Analog Comparator RTL Design Version Number
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MINOR     |Comp RTL Design MINOR Version Number
     * |        |          |Minor version number is dependent on module ECO version control.
     * |[23:16] |SUB       |Comp RTL Design SUB Version Number
     * |        |          |Major version number is correlated to Product Line.
     * |[31:24] |MAJOR     |Comp RTL Design MAJOR Version Number
     * |        |          |Major version number is correlated to Product Line.
     */
    __IO uint32_t CTL[2];                /*!< [0x0000-0x0004] Analog Comparator Control Register                        */
    __IO uint32_t STATUS;                /*!< [0x0008] Analog Comparator Status Register                                */
    __IO uint32_t VREF;                  /*!< [0x000c] Analog Comparator Reference Voltage Control Register             */
    __IO uint32_t CALCTL;                /*!< [0x0010] Analog Comparator Calibration Control Register                   */
    __IO uint32_t CALSTS;                /*!< [0x0014] Analog Comparator Calibration Status Register                    */

} ACMP_T;

/**
    @addtogroup ACMP_CONST ACMP Bit Field Definition
    Constant Definitions for ACMP Controller
@{ */

#define ACMP_CTL_ACMPEN_Pos             (0)                                                /*!< ACMP_T::CTL0: ACMPEN Position          */
#define ACMP_CTL_ACMPEN_Msk             (0x1ul << ACMP_CTL_ACMPEN_Pos)                     /*!< ACMP_T::CTL0: ACMPEN Mask              */

#define ACMP_CTL_ACMPIE_Pos             (1)                                                /*!< ACMP_T::CTL0: ACMPIE Position          */
#define ACMP_CTL_ACMPIE_Msk             (0x1ul << ACMP_CTL_ACMPIE_Pos)                     /*!< ACMP_T::CTL0: ACMPIE Mask              */

#define ACMP_CTL_ACMPOINV_Pos           (3)                                                /*!< ACMP_T::CTL0: ACMPOINV Position        */
#define ACMP_CTL_ACMPOINV_Msk           (0x1ul << ACMP_CTL_ACMPOINV_Pos)                   /*!< ACMP_T::CTL0: ACMPOINV Mask            */

#define ACMP_CTL_NEGSEL_Pos             (4)                                                /*!< ACMP_T::CTL0: NEGSEL Position          */
#define ACMP_CTL_NEGSEL_Msk             (0x7ul << ACMP_CTL_NEGSEL_Pos)                     /*!< ACMP_T::CTL0: NEGSEL Mask              */

#define ACMP_CTL_POSSEL_Pos             (8)                                                /*!< ACMP_T::CTL0: POSSEL Position          */
#define ACMP_CTL_POSSEL_Msk             (0x7ul << ACMP_CTL_POSSEL_Pos)                     /*!< ACMP_T::CTL0: POSSEL Mask              */

#define ACMP_CTL_OUTSEL_Pos             (12)                                               /*!< ACMP_T::CTL0: OUTSEL Position          */
#define ACMP_CTL_OUTSEL_Msk             (0x1ul << ACMP_CTL_OUTSEL_Pos)                     /*!< ACMP_T::CTL0: OUTSEL Mask              */

#define ACMP_CTL_FILTSEL_Pos            (13)                                               /*!< ACMP_T::CTL0: FILTSEL Position         */
#define ACMP_CTL_FILTSEL_Msk            (0x7ul << ACMP_CTL_FILTSEL_Pos)                    /*!< ACMP_T::CTL0: FILTSEL Mask             */

#define ACMP_CTL_WKEN_Pos               (16)                                               /*!< ACMP_T::CTL0: WKEN Position            */
#define ACMP_CTL_WKEN_Msk               (0x1ul << ACMP_CTL_WKEN_Pos)                       /*!< ACMP_T::CTL0: WKEN Mask                */

#define ACMP_CTL_WLATEN_Pos             (17)                                               /*!< ACMP_T::CTL0: WLATEN Position          */
#define ACMP_CTL_WLATEN_Msk             (0x1ul << ACMP_CTL_WLATEN_Pos)                     /*!< ACMP_T::CTL0: WLATEN Mask              */

#define ACMP_CTL_WCMPSEL_Pos            (18)                                               /*!< ACMP_T::CTL0: WCMPSEL Position         */
#define ACMP_CTL_WCMPSEL_Msk            (0x1ul << ACMP_CTL_WCMPSEL_Pos)                    /*!< ACMP_T::CTL0: WCMPSEL Mask             */

#define ACMP_CTL_FCLKDIV_Pos            (20)                                               /*!< ACMP_T::CTL0: FCLKDIV Position         */
#define ACMP_CTL_FCLKDIV_Msk            (0x3ul << ACMP_CTL_FCLKDIV_Pos)                    /*!< ACMP_T::CTL0: FCLKDIV Mask             */

#define ACMP_CTL_HYSSEL_Pos             (24)                                               /*!< ACMP_T::CTL0: HYSSEL Position          */
#define ACMP_CTL_HYSSEL_Msk             (0x7ul << ACMP_CTL_HYSSEL_Pos)                     /*!< ACMP_T::CTL0: HYSSEL Mask              */

#define ACMP_CTL_MODESEL_Pos            (28)                                               /*!< ACMP_T::CTL0: MODESEL Position         */
#define ACMP_CTL_MODESEL_Msk            (0x3ul << ACMP_CTL_MODESEL_Pos)                    /*!< ACMP_T::CTL0: MODESEL Mask             */

#define ACMP_CTL_INTPOL_Pos             (30)                                               /*!< ACMP_T::CTL0: INTPOL Position          */
#define ACMP_CTL_INTPOL_Msk             (0x3ul << ACMP_CTL_INTPOL_Pos)                     /*!< ACMP_T::CTL0: INTPOL Mask              */

#define ACMP_STATUS_ACMPIF0_Pos          (0)                                               /*!< ACMP_T::STATUS: ACMPIF0 Position       */
#define ACMP_STATUS_ACMPIF0_Msk          (0x1ul << ACMP_STATUS_ACMPIF0_Pos)                /*!< ACMP_T::STATUS: ACMPIF0 Mask           */

#define ACMP_STATUS_ACMPIF1_Pos          (1)                                               /*!< ACMP_T::STATUS: ACMPIF1 Position       */
#define ACMP_STATUS_ACMPIF1_Msk          (0x1ul << ACMP_STATUS_ACMPIF1_Pos)                /*!< ACMP_T::STATUS: ACMPIF1 Mask           */

#define ACMP_STATUS_ACMPO0_Pos           (4)                                               /*!< ACMP_T::STATUS: ACMPO0 Position        */
#define ACMP_STATUS_ACMPO0_Msk           (0x1ul << ACMP_STATUS_ACMPO0_Pos)                 /*!< ACMP_T::STATUS: ACMPO0 Mask            */

#define ACMP_STATUS_ACMPO1_Pos           (5)                                               /*!< ACMP_T::STATUS: ACMPO1 Position        */
#define ACMP_STATUS_ACMPO1_Msk           (0x1ul << ACMP_STATUS_ACMPO1_Pos)                 /*!< ACMP_T::STATUS: ACMPO1 Mask            */

#define ACMP_STATUS_WKIF0_Pos            (8)                                               /*!< ACMP_T::STATUS: WKIF0 Position         */
#define ACMP_STATUS_WKIF0_Msk            (0x1ul << ACMP_STATUS_WKIF0_Pos)                  /*!< ACMP_T::STATUS: WKIF0 Mask             */

#define ACMP_STATUS_WKIF1_Pos            (9)                                               /*!< ACMP_T::STATUS: WKIF1 Position         */
#define ACMP_STATUS_WKIF1_Msk            (0x1ul << ACMP_STATUS_WKIF1_Pos)                  /*!< ACMP_T::STATUS: WKIF1 Mask             */

#define ACMP_STATUS_ACMPS0_Pos           (12)                                              /*!< ACMP_T::STATUS: ACMPS0 Position        */
#define ACMP_STATUS_ACMPS0_Msk           (0x1ul << ACMP_STATUS_ACMPS0_Pos)                 /*!< ACMP_T::STATUS: ACMPS0 Mask            */

#define ACMP_STATUS_ACMPS1_Pos           (13)                                              /*!< ACMP_T::STATUS: ACMPS1 Position        */
#define ACMP_STATUS_ACMPS1_Msk           (0x1ul << ACMP_STATUS_ACMPS1_Pos)                 /*!< ACMP_T::STATUS: ACMPS1 Mask            */

#define ACMP_STATUS_ACMPWO_Pos           (16)                                              /*!< ACMP_T::STATUS: ACMPWO Position        */
#define ACMP_STATUS_ACMPWO_Msk           (0x1ul << ACMP_STATUS_ACMPWO_Pos)                 /*!< ACMP_T::STATUS: ACMPWO Mask            */

#define ACMP_VREF_CRV0SEL_Pos            (0)                                               /*!< ACMP_T::VREF: CRV0SEL Position         */
#define ACMP_VREF_CRV0SEL_Msk            (0x3ful << ACMP_VREF_CRV0SEL_Pos)                 /*!< ACMP_T::VREF: CRV0SEL Mask             */

#define ACMP_VREF_CRV0SSEL_Pos           (6)                                               /*!< ACMP_T::VREF: CRV0SSEL Position        */
#define ACMP_VREF_CRV0SSEL_Msk           (0x1ul << ACMP_VREF_CRV0SSEL_Pos)                 /*!< ACMP_T::VREF: CRV0SSEL Mask            */

#define ACMP_VREF_CRV0EN_Pos             (8)                                               /*!< ACMP_T::VREF: CRV0EN Position          */
#define ACMP_VREF_CRV0EN_Msk             (0x1ul << ACMP_VREF_CRV0EN_Pos)                   /*!< ACMP_T::VREF: CRV0EN Mask              */

#define ACMP_VREF_CLAMPEN0_Pos           (15)                                              /*!< ACMP_T::VREF: CLAMPEN0 Position        */
#define ACMP_VREF_CLAMPEN0_Msk           (0x1ul << ACMP_VREF_CLAMPEN0_Pos)                 /*!< ACMP_T::VREF: CLAMPEN0 Mask            */

#define ACMP_VREF_CRV1SEL_Pos            (16)                                              /*!< ACMP_T::VREF: CRV1SEL Position         */
#define ACMP_VREF_CRV1SEL_Msk            (0x3ful << ACMP_VREF_CRV1SEL_Pos)                 /*!< ACMP_T::VREF: CRV1SEL Mask             */

#define ACMP_VREF_CRV1SSEL_Pos           (22)                                              /*!< ACMP_T::VREF: CRV1SSEL Position        */
#define ACMP_VREF_CRV1SSEL_Msk           (0x1ul << ACMP_VREF_CRV1SSEL_Pos)                 /*!< ACMP_T::VREF: CRV1SSEL Mask            */

#define ACMP_VREF_CRV1EN_Pos             (24)                                              /*!< ACMP_T::VREF: CRV1EN Position          */
#define ACMP_VREF_CRV1EN_Msk             (0x1ul << ACMP_VREF_CRV1EN_Pos)                   /*!< ACMP_T::VREF: CRV1EN Mask              */

#define ACMP_VREF_CLAMPEN1_Pos           (31)                                              /*!< ACMP_T::VREF: CLAMPEN1 Position        */
#define ACMP_VREF_CLAMPEN1_Msk           (0x1ul << ACMP_VREF_CLAMPEN1_Pos)                 /*!< ACMP_T::VREF: CLAMPEN1 Mask            */

#define ACMP_CALCTL_CALTRG0_Pos          (0)                                               /*!< ACMP_T::CALCTL: CALTRG0 Position       */
#define ACMP_CALCTL_CALTRG0_Msk          (0x1ul << ACMP_CALCTL_CALTRG0_Pos)                /*!< ACMP_T::CALCTL: CALTRG0 Mask           */

#define ACMP_CALCTL_CALTRG1_Pos          (1)                                               /*!< ACMP_T::CALCTL: CALTRG1 Position       */
#define ACMP_CALCTL_CALTRG1_Msk          (0x1ul << ACMP_CALCTL_CALTRG1_Pos)                /*!< ACMP_T::CALCTL: CALTRG1 Mask           */

#define ACMP_CALCTL_CALCLK0_Pos          (4)                                               /*!< ACMP_T::CALCTL: CALCLK0 Position       */
#define ACMP_CALCTL_CALCLK0_Msk          (0x3ul << ACMP_CALCTL_CALCLK0_Pos)                /*!< ACMP_T::CALCTL: CALCLK0 Mask           */

#define ACMP_CALCTL_CALCLK1_Pos          (6)                                               /*!< ACMP_T::CALCTL: CALCLK1 Position       */
#define ACMP_CALCTL_CALCLK1_Msk          (0x3ul << ACMP_CALCTL_CALCLK1_Pos)                /*!< ACMP_T::CALCTL: CALCLK1 Mask           */

#define ACMP_CALCTL_OFFSETSEL_Pos        (8)                                               /*!< ACMP_T::CALCTL: OFFSETSEL Position     */
#define ACMP_CALCTL_OFFSETSEL_Msk        (0x1ul << ACMP_CALCTL_OFFSETSEL_Pos)              /*!< ACMP_T::CALCTL: OFFSETSEL Mask         */

#define ACMP_CALCTL_CALRVS_Pos           (16)                                              /*!< ACMP_T::CALCTL: CALRVS Position        */
#define ACMP_CALCTL_CALRVS_Msk           (0x3ul << ACMP_CALCTL_CALRVS_Pos)                 /*!< ACMP_T::CALCTL: CALRVS Mask            */

#define ACMP_CALSTS_DONE0_Pos            (0)                                               /*!< ACMP_T::CALSTS: DONE0 Position         */
#define ACMP_CALSTS_DONE0_Msk            (0x1ul << ACMP_CALSTS_DONE0_Pos)                  /*!< ACMP_T::CALSTS: DONE0 Mask             */

#define ACMP_CALSTS_DONE1_Pos            (4)                                               /*!< ACMP_T::CALSTS: DONE1 Position         */
#define ACMP_CALSTS_DONE1_Msk            (0x1ul << ACMP_CALSTS_DONE1_Pos)                  /*!< ACMP_T::CALSTS: DONE1 Mask             */
/**@}*/ /* ACMP_CONST */
/**@}*/ /* end of ACMP register group */


/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __ACMP_REG_H__ */
