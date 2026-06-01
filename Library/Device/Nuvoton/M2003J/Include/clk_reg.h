/**************************************************************************//**
 * @file     clk_reg.h
 * @version  V1.00
 * @brief    CLK register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CLK_REG_H__
#define __CLK_REG_H__

/** @addtogroup REGISTER Control Register
  @{
*/

/*---------------------- System Clock Controller -------------------------*/
/**
    @addtogroup CLK Clock Controller (CLK)
    Memory Mapped Structure for CLK Controller
  @{
*/

typedef struct
{
    /**
     * @var CLK_T::SRCCTL
     * Offset: 0x00  Clock Source Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LIRCEN    |LIRC Enable Bit (Write Protect)
     * |        |          |0 = 32 kHz internal low speed RC oscillator (LIRC) Disabled.
     * |        |          |1 = 32 kHz internal low speed RC oscillator (LIRC) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |LXTEN     |LXT Enable Bit (Write Protect) (Write Only)
     * |        |          |0 = 32.768 kHz external low speed crystal (LXT) Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal (LXT) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is clear if LXTFIF(CLK_CLKDSTS[13]) is 1.
     * |[2]     |HXTEN     |HXT Enable Bit (Write Protect)
     * |        |          |0 = 4~32 MHz external high speed crystal (HXT) Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal (HXT) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is write ignore when HCLKSEL(CLK_HCLKSEL[2:0]) is set to HXT.
     * |        |          |Note 3: This bit is clear if HXTFIF(CLK_CLKDSTS[5]) is 1.
     * |[3]     |HIRCEN    |HIRC Enable Bit (Write Protect)
     * |        |          |0 = 40 MHz internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = 40 MHz internal high speed RC oscillator (HIRC) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is write ignore when HCLKSEL(CLK_HCLKSEL[2:0]) is set to HIRC.
     * |[4]     |PLLEN     |PLL1 Enable Bit (Write Protect)
     * |        |          |0 = PLL Disabled.
     * |        |          |1 = PLL Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::STATUS
     * Offset: 0x04  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 32 kHz internal low speed RC oscillator (LIRC) clock is not stable or disabled.
     * |        |          |1 = 32 kHz internal low speed RC oscillator (LIRC) clock is stable and enabled.
     * |[1]     |LXTSTB    |LXT Clock Source Stable Flag (Read Only)
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock is not stable or disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock is stabled and enabled.
     * |[2]     |HXTSTB    |HXT Clock Source Stable Flag (Read Only)
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock is not stable or disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock is stable and enabled.
     * |[3]     |HIRCSTB   |HIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 40 MHz internal high speed RC oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = 40 MHz internal high speed RC oscillator (HIRC) clock is stable and enabled.
     * |[4]     |PLLSTB    |Internal PLL Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal PLL clock is not stable or disabled.
     * |        |          |1 = Internal PLL clock is stable and enabled.
     * |[8]     |HCLKSWF   |HCLK Switching Finish Flag (Read Only)
     * |        |          |This bit is updated when software switches system clock source
     * |        |          |If switch target clock is stable, this bit will be set to 1
     * |        |          |If switch target clock is not stable, this bit will be set to 0.
     * |        |          |0 = Clock switching not finish.
     * |        |          |1 = Clock switching finish.
     * |        |          |Note 1: This bit is read only
     * |        |          |After selected clock source is stable, hardware will switch system clock to selected clock automatically, and HCLKSWF will be setted automatically by hardware.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var CLK_T::LXTCTL
     * Offset: 0x08  LXT Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |LXTGAIN   |LXT Gain Control Bit (Write Protect)
     * |        |          |Gain control is used to enlarge the gain of crystal to make sure crystal work normally.
     * |        |          |00 = gm is 3.9uA/V.
     * |        |          |01 = gm is 9.6uA/V.
     * |        |          |10 = gm is 14.2uA/V..
     * |        |          |11 = gm is 20.8uA/V.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::HIRCCTL
     * Offset: 0x0C  HIRC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |HIRCSTBS  |HIRC Stable Count Select (Write Protect)
     * |        |          |00 = HIRC stable count is 61 clocks and clock output deviation about 2%.
     * |        |          |01 = HIRC stable count is 51 clocks and clock output deviation about 3%.
     * |        |          |10 = HIRC stable count is 46 clocks and clock output deviation about 4%.
     * |        |          |11 = HIRC stable count is 42 clocks and clock output deviation about 5%.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |HIRCFDIS  |HIRC Clock Filter Disable Bit (Write Protect)
     * |        |          |0 = HIRC Filter enabled.
     * |        |          |1 = HIRC Filter disabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::HXTCTL
     * Offset: 0x10  HXT Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HXTGAIN   |HXT Gain Control Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register XT1XSG (CONFIG0 [18:16]).
     * |        |          |Gain control is used to enlarge the gain of crystal to make sure crystal work normally.
     * |        |          |000 = HXT frequency is from 4 MHz.
     * |        |          |001 = HXT frequency is from 4 MHz to 8 MHz.
     * |        |          |010 = HXT frequency is from 8 MHz to 12 MHz.
     * |        |          |011 = HXT frequency is from 12 MHz to 16 MHz.
     * |        |          |100 = HXT frequency is from 16 MHz to 24 MHz.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: The default value is set by UCFG0[18:16].
     * |[4]     |HXTSELTYP |HXT Crystal Type Select Bit (Write Protect)
     * |        |          |0 = Select INV type.
     * |        |          |1 = Select GM type.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |HXTMD     |HXT Mode Selection (Write Protect)
     * |        |          |0 = HXT work as crystal mode. PF.2 and PF.3 are configured as external high speed crystal (HXT) pins.
     * |        |          |1 = HXT works as external clock mode. PF.3 is configured as external clock input pin.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGCTL register.
     * |        |          |Note 2: When external clock mode enable, HXTSELTYP(CLK_HXTCTL[4]) must be set as GM type.
     * |[6]     |HXTFDIS   |HXT Clock Filter Disable Bit (Write Protect)
     * |        |          |0 = HXT Filter enabled.
     * |        |          |1 = HXT Filter disabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |HXTFSEL   |HXT Filter Select (Write Protect)
     * |        |          |0 = HXT frequency is <= 24 MHz.
     * |        |          |1 = HXT frequency is <= 4 MHz.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::PLLCTL
     * Offset: 0x18  PLL Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FBDIV     |PLL Feedback Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are write ignore when PLLEN(CLK_SRCCTL[4]) is set to enable.
     * |[11:8]  |INDIV     |PLL Input Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are write ignore when PLLEN(CLK_SRCCTL[4]) is set to enable.
     * |[29:28] |STBSEL    |PLL Stable Counter Selection (Write Protect)
     * |        |          |00 = PLL stable time is 400 PLL source clock (source clock is equal to 4M).
     * |        |          |01 = PLL stable time is 1200 PLL source clock (4 MHz < source clock <=12 MHz).
     * |        |          |10 = PLL stable time is 2400 PLL source clock (12 MHz < source clock <=24 MHz).
     * |        |          |11 = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are write ignore when PLLEN(CLK_SRCCTL[4]) is set to enable.
     * |[30]    |BP        |PLL Bypass Control (Write Protect)
     * |        |          |0 = PLL is in normal mode (default).
     * |        |          |1 = PLL clock output is same as PLL input clock FIN.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is write ignore when PLLEN(CLK_SRCCTL[4]) is set to enable.
     * @var CLK_T::CLKOCTL
     * Offset: 0x28  Clock Output Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FREQSEL   |Clock Output Frequency Selection
     * |        |          |The formula of output frequency is
     * |        |          |Fout = Fin/2(N+1).
     * |        |          |Fin is the input clock frequency.
     * |        |          |Fout is the frequency of divider output clock.
     * |        |          |N is the 4-bit value of FREQSEL[3:0].
     * |[4]     |CLKOEN    |Clock Output Enable Bit
     * |        |          |0 = Clock Output function Disabled.
     * |        |          |1 = Clock Output function Enabled.
     * |[5]     |DIV1EN    |Clock Output Divide One Enable Bit
     * |        |          |0 = Clock Output will output clock with source frequency divided by FREQSEL.
     * |        |          |1 = Clock Output will output clock with source frequency.
     * |[6]     |CLK1HZEN  |Clock Output 1Hz Enable Bit
     * |        |          |0 = 1 Hz clock output for 32.768 kHz frequency compensation Disabled.
     * |        |          |1 = 1 Hz clock output for 32.768 kHz frequency compensation Enabled.
     * @var CLK_T::CLKDCTL
     * Offset: 0x30  Clock Fail Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |HXTFDEN   |HXT Clock Fail Detector Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector Enabled.
     * |[5]     |HXTFIEN   |HXT Clock Fail Interrupt Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail interrupt Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail interrupt Enabled.
     * |[6]     |HXTFDSEL  |HXT Clock Fail Detector Selection
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector after HXT stable.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector bypass HXT stable.
     * |        |          |Note 1: When HXT Clock Fail Detector Selection is set, detector will keep detect whether HXT is stable or not, prevent HXT fail before stable.
     * |[12]    |LXTFDEN   |LXT Clock Fail Detector Enable Bit
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector Enabled.
     * |[13]    |LXTFIEN   |LXT Clock Fail Interrupt Enable Bit
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail interrupt Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail interrupt Enabled.
     * |[16]    |HXTFQDEN  |HXT Clock Frequency Range Detector Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector Enabled.
     * |[17]    |HXTFQIEN  |HXT Clock Frequency Range Detector Interrupt Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail interrupt Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail interrupt Enabled.
     * |[18]    |HXTFQASW  |HXT Clock Frequency Range Detector Event Auto Switch Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail event happened and HCLK will not switch to HIRC automatically.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail event happened and HCLK will switch to HIRC automatically.
     * |        |          |Note 1: This bit should be set before HXTFQDEN(CLK_CLKDCTL[16]).
     * @var CLK_T::CLKDSTS
     * Offset: 0x34  Clock Fail Detector Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |HXTFDST   |HXT Clock Fail Detector Status Bit (Read Only)
     * |        |          |0 = Indicate 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector already disabled.
     * |        |          |1 = Indicate 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector already enabled.
     * |[5]     |HXTFIF    |HXT Clock Fail Interrupt Flag (Write Protect)
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock is normal.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock stops.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[12]    |LXTFDST   |LXT Clock Fail Detector Status Bit (Read Only)
     * |        |          |0 = Indicate 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector already disabled.
     * |        |          |1 = Indicate 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector already enabled.
     * |[13]    |LXTFIF    |LXT Clock Fail Interrupt Flag (Write Protect)
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock is normal.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) stops.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[17]    |HXTFQIF   |HXT Clock Frequency Range Detector Interrupt Flag (Write Protect)
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency is normal.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency is abnormal.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::CDUPB
     * Offset: 0x38  Clock Frequency Range Detector Upper Boundary Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |UPERBD    |HXT Clock Frequency Range Detector Upper Boundary Value
     * |        |          |The bits define the high value of frequency monitor window.
     * |        |          |When HXT frequency monitor value higher than this register, the HXT frequency detect fail interrupt flag will set to 1.
     * |        |          |Frequency out of range will be asserted when HXT frequency > ((UPERBD+1)/512)* HIRC frequency.
     * @var CLK_T::CDLOWB
     * Offset: 0x3C  Clock Frequency Range Detector Lower Boundary Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |LOWERBD   |HXT Clock Frequency Range Detector Lower Boundary Value
     * |        |          |The bits define the low value of frequency monitor window.
     * |        |          |When HXT frequency monitor value lower than this register, the HXT frequency detect fail interrupt flag will set to 1.
     * |        |          |Frequency out of range will be asserted when HXT frequency < ((LOWERBD+1)/512)* HIRC frequency.
     * @var CLK_T::ADCCTL
     * Offset: 0x80  ADC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADC0CKEN  |ADCx Clock Enable Bit
     * |        |          |0 = ADCx clock Disabled.
     * |        |          |1 = ADCx clock Enabled.
     * @var CLK_T::BPWMCTL
     * Offset: 0x84  BPWM Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BPWM0CKEN |BPWMx Clock Enable Bit
     * |        |          |0 = BPWMx clock Disabled.
     * |        |          |1 = BPWMx clock Enabled.
     * |[1]     |BPWM1CKEN |BPWMx Clock Enable Bit
     * |        |          |0 = BPWMx clock Disabled.
     * |        |          |1 = BPWMx clock Enabled.
     * @var CLK_T::CRCCTL
     * Offset: 0x88  CRC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRC0CKEN  |CRCx Clock Enable Bit
     * |        |          |0 = CRCx clock Disabled.
     * |        |          |1 = CRCx clock Enabled.
     * @var CLK_T::FMCCTL
     * Offset: 0x8C  FMC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FMC0CKEN  |FMCx Clock Enable Bit
     * |        |          |0 = FMCx clock Disabled when chip is under IDLE mode.
     * |        |          |1 = FMCx clock Enabled when chip is under IDLE mode.
     * |[8]     |DFMC0CKEN |Data FMCx Clock Enable Bit
     * |        |          |0 = DFMCx clock Disabled.
     * |        |          |1 = DFMCx clock Enabled.
     * |[16]    |ISP0CKEN  |ISPx Clock Enable Bit
     * |        |          |0 = ISPx clock Disabled.
     * |        |          |1 = ISPx clock Enabled.
     * @var CLK_T::GPIOCTL
     * Offset: 0x90  GPIO Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPIOACKEN |GPIOA Clock Enable Bit
     * |        |          |0 = GPIOA clock Disabled.
     * |        |          |1 = GPIOA clock Enabled.
     * |[1]     |GPIOBCKEN |GPIOB Clock Enable Bit
     * |        |          |0 = GPIOB clock Disabled.
     * |        |          |1 = GPIOB clock Enabled.
     * |[2]     |GPIOCCKEN |GPIOC Clock Enable Bit
     * |        |          |0 = GPIOC clock Disabled.
     * |        |          |1 = GPIOC clock Enabled.
     * |[3]     |GPIODCKEN |GPIOD Clock Enable Bit
     * |        |          |0 = GPIOD clock Disabled.
     * |        |          |1 = GPIOD clock Enabled.
     * |[4]     |GPIOECKEN |GPIOE Clock Enable Bit
     * |        |          |0 = GPIOE clock Disabled.
     * |        |          |1 = GPIOE clock Enabled.
     * |[5]     |GPIOFCKEN |GPIOF Clock Enable Bit
     * |        |          |0 = GPIOF clock Disabled.
     * |        |          |1 = GPIOF clock Enabled.
     * |[6]     |GPIOGCKEN |GPIOG Clock Enable Bit
     * |        |          |0 = GPIOG clock Disabled.
     * |        |          |1 = GPIOG clock Enabled.
     * @var CLK_T::I2CCTL
     * Offset: 0x94  I2C Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2C0CKEN  |I2Cx Clock Enable Bit
     * |        |          |0 = I2Cx clock Disabled.
     * |        |          |1 = I2Cx clock Enabled.
     * |[1]     |I2C1CKEN  |I2Cx Clock Enable Bit
     * |        |          |0 = I2Cx clock Disabled.
     * |        |          |1 = I2Cx clock Enabled.
     * |[2]     |I2C2CKEN  |I2Cx Clock Enable Bit
     * |        |          |0 = I2Cx clock Disabled.
     * |        |          |1 = I2Cx clock Enabled.
     * @var CLK_T::PDMACTL
     * Offset: 0x98  PDMA Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDMA0CKEN |PDMAx Clock Enable Bit
     * |        |          |0 = PDMAx clock Disabled.
     * |        |          |1 = PDMAx clock Enabled.
     * @var CLK_T::RTCCTL
     * Offset: 0x9C  RTC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RTC0CKEN  |RTCx Clock Enable Bit
     * |        |          |0 = RTCx clock Disabled.
     * |        |          |1 = RTCx clock Enabled.
     * @var CLK_T::SRAMCTL
     * Offset: 0xA0  System SRAM Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SRAM0CKEN |SRAM0 Clock Enable Bit
     * |        |          |0 = SRAM0 clock Disabled.
     * |        |          |1 = SRAM0 clock Enabled.
     * |[1]     |SRAM1CKEN |SRAM1 Clock Enable Bit
     * |        |          |0 = SRAM1 clock Disabled.
     * |        |          |1 = SRAM1 clock Enabled.
     * @var CLK_T::STCTL
     * Offset: 0xA4  System Tick Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ST0CKEN   |SYSTICKx Clock Enable Bit
     * |        |          |0 = SYSTICKx clock Disabled.
     * |        |          |1 = SYSTICKx clock Enabled.
     * @var CLK_T::TMRCTL
     * Offset: 0xA8  Timer Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TMR0CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |[1]     |TMR1CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |[2]     |TMR2CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |[3]     |TMR3CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |[4]     |TMR4CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |[5]     |TMR5CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |[6]     |TMR6CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |[7]     |TMR7CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |[8]     |TMR8CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * @var CLK_T::UARTCTL
     * Offset: 0xAC  UART Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |UART0CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |[1]     |UART1CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |[2]     |UART2CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |[3]     |UART3CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |[4]     |UART4CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * @var CLK_T::USCICTL
     * Offset: 0xB0  USCI Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USCI0CKEN |USCIx Clock Enable Bit
     * |        |          |0 = USCIx clock Disabled.
     * |        |          |1 = USCIx clock Enabled.
     * |[1]     |USCI1CKEN |USCIx Clock Enable Bit
     * |        |          |0 = USCIx clock Disabled.
     * |        |          |1 = USCIx clock Enabled.
     * |[2]     |USCI2CKEN |USCIx Clock Enable Bit
     * |        |          |0 = USCIx clock Disabled.
     * |        |          |1 = USCIx clock Enabled.
     * |[3]     |USCI3CKEN |USCIx Clock Enable Bit
     * |        |          |0 = USCIx clock Disabled.
     * |        |          |1 = USCIx clock Enabled.
     * |[4]     |USCI4CKEN |USCIx Clock Enable Bit
     * |        |          |0 = USCIx clock Disabled.
     * |        |          |1 = USCIx clock Enabled.
     * @var CLK_T::WDTCTL
     * Offset: 0xB4  WDT Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDT0CKEN  |WDTx Clock Enable Bit
     * |        |          |0 = WDTx clock Disabled.
     * |        |          |1 = WDTx clock Enabled.
     * @var CLK_T::WWDTCTL
     * Offset: 0xB8  WWDT Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT0CKEN |WWDTx Clock Enable Bit
     * |        |          |0 = WWDTx clock Disabled.
     * |        |          |1 = WWDTx clock Enabled.
     * @var CLK_T::HCLKSEL
     * Offset: 0x100  AHB Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLKSEL   |System Clock Source Selection (Write Protect)
     * |        |          |The system clock source is defined by SCLKSEL.
     * |        |          |000 = Clock source from HIRC.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from PLL.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::BPWMSEL
     * Offset: 0x104  BPWM Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BPWM0SEL  |BPWM0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of BPWM0 is defined by BPWM0SEL.
     * |        |          |0 = Clock source from PCLK0.
     * |        |          |1 = Clock source from HCLK.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |BPWM1SEL  |BPWM1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of BPWM1 is defined by BPWM1SEL.
     * |        |          |0 = Clock source from PCLK1.
     * |        |          |1 = Clock source from HCLK.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::CLKOSEL
     * Offset: 0x108  Clock Output Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CLKOSEL   |Clock Output Clock Source Selection (Write Protect)
     * |        |          |000 = Clock source from HCLK.
     * |        |          |001 = Clock source from PCLK0.
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |100 = Clock source from LXT.
     * |        |          |101 = Clock source from HXT.
     * |        |          |110 = Clock source from HIRC.
     * |        |          |111 = Clock source from PLL.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::TMRSEL0
     * Offset: 0x10C  Timer Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |TMR0SEL   |Timer 0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR0 is defined by TMR0SEL.
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM0 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[6:4]   |TMR1SEL   |Timer 1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR1 is defined by TMR1SEL.
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM1 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[10:8]  |TMR2SEL   |Timer 2 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR2 is defined by TMR2SEL.
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM2 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[14:12] |TMR3SEL   |Timer 3 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR3 is defined by TMR3SEL.
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM3 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[18:16] |TMR4SEL   |Timer 4 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR4 is defined by TMR4SEL.
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM4 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[22:20] |TMR5SEL   |Timer 5 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR5 is defined by TMR5SEL.
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM5 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[26:24] |TMR6SEL   |Timer 6 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR6 is defined by TMR6SEL.
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM6 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[30:28] |TMR7SEL   |Timer 7 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR7 is defined by TMR7SEL.
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM7 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::TMRSEL1
     * Offset: 0x110  Timer Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |TMR8SEL   |Timer 8 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR8 is defined by TMR8SEL.
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM8 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::UARTSEL
     * Offset: 0x114  UART Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |UART0SEL  |UART 0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART0 is defined by UART0SEL.
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[6:4]   |UART1SEL  |UART 1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART1 is defined by UART1SEL.
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[10:8]  |UART2SEL  |UART 2 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART2 is defined by UART2SEL.
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[14:12] |UART3SEL  |UART 3 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART3 is defined by UART3SEL.
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[18:16] |UART4SEL  |UART 4 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART4 is defined by UART4SEL.
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from LIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::WDTSEL
     * Offset: 0x118  WDT Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDT0SEL   |WDT0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of WDT0 is defined by WDT0SEL.
     * |        |          |00 = Clock source from HCLK/2048.
     * |        |          |01 = Clock source from LXT.
     * |        |          |10 = Clock source from LIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::WWDTSEL
     * Offset: 0x11C  WWDT Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT0SEL  |WWDT0 Clock Source Selection (Write Protect)
     * |        |          |0 = Clock source from HCLK/2048.
     * |        |          |1 = Clock source from LIRC.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::HCLKDIV
     * Offset: 0x180  HCLK Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLKDIV   |HCLK Clock Divide Number from HCLK0 Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1).
     * @var CLK_T::PCLKDIV
     * Offset: 0x184  PCLK Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |PCLK0DIV  |PCLK0 Clock Divide Number from PCLK0 Clock Source
     * |        |          |000 = PCLK0 is divided by 1.
     * |        |          |001 = PCLK0 is divided by 2.
     * |        |          |010 = PCLK0 is divided by 4.
     * |        |          |011 = PCLK0 is divided by 8.
     * |        |          |100 = PCLK0 is divided by 16.
     * |        |          |Others = Reserved.
     * |[6:4]   |PCLK1DIV  |PCLK1 Clock Divide Number from PCLK1 Clock Source
     * |        |          |000 = PCLK1 is divided by 1.
     * |        |          |001 = PCLK1 is divided by 2.
     * |        |          |010 = PCLK1 is divided by 4.
     * |        |          |011 = PCLK1 is divided by 8.
     * |        |          |100 = PCLK1 is divided by 16.
     * |        |          |Others = Reserved.
     * @var CLK_T::ADCDIV
     * Offset: 0x190  ADC Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ADC0DIV   |ADC0 Clock Divide Number from ADC0 Clock Source
     * |        |          |ADC0 clock frequency = (ADC0 clock source frequency) / (ADC0DIV + 1).
     * @var CLK_T::UARTDIV
     * Offset: 0x194  UART Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |UART0DIV  |UART0 Clock Divide Number from UART0 Clock Source
     * |        |          |UART0 clock frequency = (UART0 clock source frequency) / (UART0DIV + 1).
     * |[7:4]   |UART1DIV  |UART1 Clock Divide Number from UART1 Clock Source
     * |        |          |UART1 clock frequency = (UART1 clock source frequency) / (UART1DIV + 1).
     * |[11:8]  |UART2DIV  |UART2 Clock Divide Number from UART2 Clock Source
     * |        |          |UART2 clock frequency = (UART2 clock source frequency) / (UART2DIV + 1).
     * |[15:12] |UART3DIV  |UART3 Clock Divide Number from UART3 Clock Source
     * |        |          |UART3 clock frequency = (UART3 clock source frequency) / (UART3DIV + 1).
     * |[19:16] |UART4DIV  |UART4 Clock Divide Number from UART4 Clock Source
     * |        |          |UART4 clock frequency = (UART4 clock source frequency) / (UART4DIV + 1).
     */
    __IO uint32_t SRCCTL;                 /*!< [0x0000] Clock Source Control Register                                    */
    __I  uint32_t STATUS;                 /*!< [0x0004] Clock Status Monitor Register                                    */
    __IO uint32_t LXTCTL;                 /*!< [0x0008] LXT Control Register                                             */
    __IO uint32_t HIRCCTL;                /*!< [0x000c] HIRC Control Register                                            */
    __IO uint32_t HXTCTL;                 /*!< [0x0010] HXT Control Register                                             */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t PLLCTL;                 /*!< [0x0018] PLL Control Register                                             */
    __I  uint32_t RESERVE1[3];
    __IO uint32_t CLKOCTL;                /*!< [0x0028] Clock Output Control Register                                    */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t CLKDCTL;                /*!< [0x0030] Clock Fail Detector Control Register                             */
    __IO uint32_t CLKDSTS;                /*!< [0x0034] Clock Fail Detector Status Register                              */
    __IO uint32_t CDUPB;                  /*!< [0x0038] Clock Frequency Range Detector Upper Boundary Register           */
    __IO uint32_t CDLOWB;                 /*!< [0x003c] Clock Frequency Range Detector Lower Boundary Register           */
    __I  uint32_t RESERVE3[16];
    __IO uint32_t ADCCTL;                 /*!< [0x0080] ADC Clock Enable Control Register                                */
    __IO uint32_t BPWMCTL;                /*!< [0x0084] BPWM Clock Enable Control Register                               */
    __IO uint32_t CRCCTL;                 /*!< [0x0088] CRC Clock Enable Control Register                                */
    __IO uint32_t FMCCTL;                 /*!< [0x008c] FMC Clock Enable Control Register                                */
    __IO uint32_t GPIOCTL;                /*!< [0x0090] GPIO Clock Enable Control Register                               */
    __IO uint32_t I2CCTL;                 /*!< [0x0094] I2C Clock Enable Control Register                                */
    __IO uint32_t PDMACTL;                /*!< [0x0098] PDMA Clock Enable Control Register                               */
    __IO uint32_t RTCCTL;                 /*!< [0x009c] RTC Clock Enable Control Register                                */
    __IO uint32_t SRAMCTL;                /*!< [0x00a0] System SRAM Clock Enable Control Register                        */
    __IO uint32_t STCTL;                  /*!< [0x00a4] System Tick Clock Enable Control Register                        */
    __IO uint32_t TMRCTL;                 /*!< [0x00a8] Timer Clock Enable Control Register                              */
    __IO uint32_t UARTCTL;                /*!< [0x00ac] UART Clock Enable Control Register                               */
    __IO uint32_t USCICTL;                /*!< [0x00b0] USCI Clock Enable Control Register                               */
    __IO uint32_t WDTCTL;                 /*!< [0x00b4] WDT Clock Enable Control Register                                */
    __IO uint32_t WWDTCTL;                /*!< [0x00b8] WWDT Clock Enable Control Register                               */
    __I  uint32_t RESERVE4[17];
    __IO uint32_t HCLKSEL;                /*!< [0x0100] AHB Clock Source Select Control Register                         */
    __IO uint32_t BPWMSEL;                /*!< [0x0104] BPWM Clock Source Select Control Register                        */
    __IO uint32_t CLKOSEL;                /*!< [0x0108] Clock Output Clock Source Select Control Register                */
    __IO uint32_t TMRSEL0;                /*!< [0x010c] Timer Clock Source Select Control Register 0                     */
    __IO uint32_t TMRSEL1;                /*!< [0x0110] Timer Clock Source Select Control Register 1                     */
    __IO uint32_t UARTSEL;                /*!< [0x0114] UART Clock Source Select Control Register                        */
    __IO uint32_t WDTSEL;                 /*!< [0x0118] WDT Clock Source Select Control Register                         */
    __IO uint32_t WWDTSEL;                /*!< [0x011c] WWDT Clock Source Select Control Register                        */
    __I  uint32_t RESERVE5[24];
    __IO uint32_t HCLKDIV;                /*!< [0x0180] HCLK Clock Divider Control Register                              */
    __IO uint32_t PCLKDIV;                /*!< [0x0184] PCLK Clock Divider Control Register                              */
    __I  uint32_t RESERVE6[2];
    __IO uint32_t ADCDIV;                 /*!< [0x0190] ADC Clock Divider Control Register                               */
    __IO uint32_t UARTDIV;                /*!< [0x0194] UART Clock Divider Control Register                              */
} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
  @{
*/

#define CLK_SRCCTL_LIRCEN_Pos            (0)                                               /*!< CLK_T::SRCCTL: LIRCEN Position         */
#define CLK_SRCCTL_LIRCEN_Msk            (0x1ul << CLK_SRCCTL_LIRCEN_Pos)                  /*!< CLK_T::SRCCTL: LIRCEN Mask             */

#define CLK_SRCCTL_LXTEN_Pos             (1)                                               /*!< CLK_T::SRCCTL: LXTEN Position          */
#define CLK_SRCCTL_LXTEN_Msk             (0x1ul << CLK_SRCCTL_LXTEN_Pos)                   /*!< CLK_T::SRCCTL: LXTEN Mask              */

#define CLK_SRCCTL_HXTEN_Pos             (2)                                               /*!< CLK_T::SRCCTL: HXTEN Position          */
#define CLK_SRCCTL_HXTEN_Msk             (0x1ul << CLK_SRCCTL_HXTEN_Pos)                   /*!< CLK_T::SRCCTL: HXTEN Mask              */

#define CLK_SRCCTL_HIRCEN_Pos            (3)                                               /*!< CLK_T::SRCCTL: HIRCEN Position         */
#define CLK_SRCCTL_HIRCEN_Msk            (0x1ul << CLK_SRCCTL_HIRCEN_Pos)                  /*!< CLK_T::SRCCTL: HIRCEN Mask             */

#define CLK_SRCCTL_PLLEN_Pos             (4)                                               /*!< CLK_T::SRCCTL: PLLEN Position          */
#define CLK_SRCCTL_PLLEN_Msk             (0x1ul << CLK_SRCCTL_PLLEN_Pos)                   /*!< CLK_T::SRCCTL: PLLEN Mask              */

#define CLK_STATUS_LIRCSTB_Pos           (0)                                               /*!< CLK_T::STATUS: LIRCSTB Position        */
#define CLK_STATUS_LIRCSTB_Msk           (0x1ul << CLK_STATUS_LIRCSTB_Pos)                 /*!< CLK_T::STATUS: LIRCSTB Mask            */

#define CLK_STATUS_LXTSTB_Pos            (1)                                               /*!< CLK_T::STATUS: LXTSTB Position         */
#define CLK_STATUS_LXTSTB_Msk            (0x1ul << CLK_STATUS_LXTSTB_Pos)                  /*!< CLK_T::STATUS: LXTSTB Mask             */

#define CLK_STATUS_HXTSTB_Pos            (2)                                               /*!< CLK_T::STATUS: HXTSTB Position         */
#define CLK_STATUS_HXTSTB_Msk            (0x1ul << CLK_STATUS_HXTSTB_Pos)                  /*!< CLK_T::STATUS: HXTSTB Mask             */

#define CLK_STATUS_HIRCSTB_Pos           (3)                                               /*!< CLK_T::STATUS: HIRCSTB Position        */
#define CLK_STATUS_HIRCSTB_Msk           (0x1ul << CLK_STATUS_HIRCSTB_Pos)                 /*!< CLK_T::STATUS: HIRCSTB Mask            */

#define CLK_STATUS_PLLSTB_Pos            (4)                                               /*!< CLK_T::STATUS: PLLSTB Position         */
#define CLK_STATUS_PLLSTB_Msk            (0x1ul << CLK_STATUS_PLLSTB_Pos)                  /*!< CLK_T::STATUS: PLLSTB Mask             */

#define CLK_STATUS_HCLKSWF_Pos           (8)                                               /*!< CLK_T::STATUS: HCLKSWF Position        */
#define CLK_STATUS_HCLKSWF_Msk           (0x1ul << CLK_STATUS_HCLKSWF_Pos)                 /*!< CLK_T::STATUS: HCLKSWF Mask            */

#define CLK_LXTCTL_LXTGAIN_Pos           (0)                                               /*!< CLK_T::LXTCTL: LXTGAIN Position        */
#define CLK_LXTCTL_LXTGAIN_Msk           (0x3ul << CLK_LXTCTL_LXTGAIN_Pos)                 /*!< CLK_T::LXTCTL: LXTGAIN Mask            */

#define CLK_HIRCCTL_HIRCSTBS_Pos         (0)                                               /*!< CLK_T::HIRCCTL: HIRCSTBS Position      */
#define CLK_HIRCCTL_HIRCSTBS_Msk         (0x3ul << CLK_HIRCCTL_HIRCSTBS_Pos)               /*!< CLK_T::HIRCCTL: HIRCSTBS Mask          */

#define CLK_HIRCCTL_HIRCFDIS_Pos         (4)                                               /*!< CLK_T::HIRCCTL: HIRCFDIS Position      */
#define CLK_HIRCCTL_HIRCFDIS_Msk         (0x1ul << CLK_HIRCCTL_HIRCFDIS_Pos)               /*!< CLK_T::HIRCCTL: HIRCFDIS Mask          */

#define CLK_HXTCTL_HXTGAIN_Pos           (0)                                               /*!< CLK_T::HXTCTL: HXTGAIN Position        */
#define CLK_HXTCTL_HXTGAIN_Msk           (0x7ul << CLK_HXTCTL_HXTGAIN_Pos)                 /*!< CLK_T::HXTCTL: HXTGAIN Mask            */

#define CLK_HXTCTL_HXTSELTYP_Pos         (4)                                               /*!< CLK_T::HXTCTL: HXTSELTYP Position      */
#define CLK_HXTCTL_HXTSELTYP_Msk         (0x1ul << CLK_HXTCTL_HXTSELTYP_Pos)               /*!< CLK_T::HXTCTL: HXTSELTYP Mask          */

#define CLK_HXTCTL_HXTMD_Pos             (5)                                               /*!< CLK_T::HXTCTL: HXTMD Position          */
#define CLK_HXTCTL_HXTMD_Msk             (0x1ul << CLK_HXTCTL_HXTMD_Pos)                   /*!< CLK_T::HXTCTL: HXTMD Mask              */

#define CLK_HXTCTL_HXTFDIS_Pos           (6)                                               /*!< CLK_T::HXTCTL: HXTFDIS Position        */
#define CLK_HXTCTL_HXTFDIS_Msk           (0x1ul << CLK_HXTCTL_HXTFDIS_Pos)                 /*!< CLK_T::HXTCTL: HXTFDIS Mask            */

#define CLK_HXTCTL_HXTFSEL_Pos           (7)                                               /*!< CLK_T::HXTCTL: HXTFSEL Position        */
#define CLK_HXTCTL_HXTFSEL_Msk           (0x1ul << CLK_HXTCTL_HXTFSEL_Pos)                 /*!< CLK_T::HXTCTL: HXTFSEL Mask            */

#define CLK_PLLCTL_FBDIV_Pos             (0)                                               /*!< CLK_T::PLLCTL: FBDIV Position          */
#define CLK_PLLCTL_FBDIV_Msk             (0xful << CLK_PLLCTL_FBDIV_Pos)                   /*!< CLK_T::PLLCTL: FBDIV Mask              */

#define CLK_PLLCTL_INDIV_Pos             (8)                                               /*!< CLK_T::PLLCTL: INDIV Position          */
#define CLK_PLLCTL_INDIV_Msk             (0xful << CLK_PLLCTL_INDIV_Pos)                   /*!< CLK_T::PLLCTL: INDIV Mask              */

#define CLK_PLLCTL_STBSEL_Pos            (28)                                              /*!< CLK_T::PLLCTL: STBSEL Position         */
#define CLK_PLLCTL_STBSEL_Msk            (0x3ul << CLK_PLLCTL_STBSEL_Pos)                  /*!< CLK_T::PLLCTL: STBSEL Mask             */

#define CLK_PLLCTL_BP_Pos                (30)                                              /*!< CLK_T::PLLCTL: BP Position             */
#define CLK_PLLCTL_BP_Msk                (0x1ul << CLK_PLLCTL_BP_Pos)                      /*!< CLK_T::PLLCTL: BP Mask                 */

#define CLK_CLKOCTL_FREQSEL_Pos          (0)                                               /*!< CLK_T::CLKOCTL: FREQSEL Position       */
#define CLK_CLKOCTL_FREQSEL_Msk          (0xful << CLK_CLKOCTL_FREQSEL_Pos)                /*!< CLK_T::CLKOCTL: FREQSEL Mask           */

#define CLK_CLKOCTL_CLKOEN_Pos           (4)                                               /*!< CLK_T::CLKOCTL: CLKOEN Position        */
#define CLK_CLKOCTL_CLKOEN_Msk           (0x1ul << CLK_CLKOCTL_CLKOEN_Pos)                 /*!< CLK_T::CLKOCTL: CLKOEN Mask            */

#define CLK_CLKOCTL_DIV1EN_Pos           (5)                                               /*!< CLK_T::CLKOCTL: DIV1EN Position        */
#define CLK_CLKOCTL_DIV1EN_Msk           (0x1ul << CLK_CLKOCTL_DIV1EN_Pos)                 /*!< CLK_T::CLKOCTL: DIV1EN Mask            */

#define CLK_CLKOCTL_CLK1HZEN_Pos         (6)                                               /*!< CLK_T::CLKOCTL: CLK1HZEN Position      */
#define CLK_CLKOCTL_CLK1HZEN_Msk         (0x1ul << CLK_CLKOCTL_CLK1HZEN_Pos)               /*!< CLK_T::CLKOCTL: CLK1HZEN Mask          */

#define CLK_CLKDCTL_HXTFDEN_Pos          (4)                                               /*!< CLK_T::CLKDCTL: HXTFDEN Position       */
#define CLK_CLKDCTL_HXTFDEN_Msk          (0x1ul << CLK_CLKDCTL_HXTFDEN_Pos)                /*!< CLK_T::CLKDCTL: HXTFDEN Mask           */

#define CLK_CLKDCTL_HXTFIEN_Pos          (5)                                               /*!< CLK_T::CLKDCTL: HXTFIEN Position       */
#define CLK_CLKDCTL_HXTFIEN_Msk          (0x1ul << CLK_CLKDCTL_HXTFIEN_Pos)                /*!< CLK_T::CLKDCTL: HXTFIEN Mask           */

#define CLK_CLKDCTL_HXTFDSEL_Pos         (6)                                               /*!< CLK_T::CLKDCTL: HXTFDSEL Position      */
#define CLK_CLKDCTL_HXTFDSEL_Msk         (0x1ul << CLK_CLKDCTL_HXTFDSEL_Pos)               /*!< CLK_T::CLKDCTL: HXTFDSEL Mask          */

#define CLK_CLKDCTL_LXTFDEN_Pos          (12)                                              /*!< CLK_T::CLKDCTL: LXTFDEN Position       */
#define CLK_CLKDCTL_LXTFDEN_Msk          (0x1ul << CLK_CLKDCTL_LXTFDEN_Pos)                /*!< CLK_T::CLKDCTL: LXTFDEN Mask           */

#define CLK_CLKDCTL_LXTFIEN_Pos          (13)                                              /*!< CLK_T::CLKDCTL: LXTFIEN Position       */
#define CLK_CLKDCTL_LXTFIEN_Msk          (0x1ul << CLK_CLKDCTL_LXTFIEN_Pos)                /*!< CLK_T::CLKDCTL: LXTFIEN Mask           */

#define CLK_CLKDCTL_HXTFQDEN_Pos         (16)                                              /*!< CLK_T::CLKDCTL: HXTFQDEN Position      */
#define CLK_CLKDCTL_HXTFQDEN_Msk         (0x1ul << CLK_CLKDCTL_HXTFQDEN_Pos)               /*!< CLK_T::CLKDCTL: HXTFQDEN Mask          */

#define CLK_CLKDCTL_HXTFQIEN_Pos         (17)                                              /*!< CLK_T::CLKDCTL: HXTFQIEN Position      */
#define CLK_CLKDCTL_HXTFQIEN_Msk         (0x1ul << CLK_CLKDCTL_HXTFQIEN_Pos)               /*!< CLK_T::CLKDCTL: HXTFQIEN Mask          */

#define CLK_CLKDCTL_HXTFQASW_Pos         (18)                                              /*!< CLK_T::CLKDCTL: HXTFQASW Position      */
#define CLK_CLKDCTL_HXTFQASW_Msk         (0x1ul << CLK_CLKDCTL_HXTFQASW_Pos)               /*!< CLK_T::CLKDCTL: HXTFQASW Mask          */

#define CLK_CLKDSTS_HXTFDST_Pos          (4)                                               /*!< CLK_T::CLKDSTS: HXTFDST Position       */
#define CLK_CLKDSTS_HXTFDST_Msk          (0x1ul << CLK_CLKDSTS_HXTFDST_Pos)                /*!< CLK_T::CLKDSTS: HXTFDST Mask           */

#define CLK_CLKDSTS_HXTFIF_Pos           (5)                                               /*!< CLK_T::CLKDSTS: HXTFIF Position        */
#define CLK_CLKDSTS_HXTFIF_Msk           (0x1ul << CLK_CLKDSTS_HXTFIF_Pos)                 /*!< CLK_T::CLKDSTS: HXTFIF Mask            */

#define CLK_CLKDSTS_LXTFDST_Pos          (12)                                              /*!< CLK_T::CLKDSTS: LXTFDST Position       */
#define CLK_CLKDSTS_LXTFDST_Msk          (0x1ul << CLK_CLKDSTS_LXTFDST_Pos)                /*!< CLK_T::CLKDSTS: LXTFDST Mask           */

#define CLK_CLKDSTS_LXTFIF_Pos           (13)                                              /*!< CLK_T::CLKDSTS: LXTFIF Position        */
#define CLK_CLKDSTS_LXTFIF_Msk           (0x1ul << CLK_CLKDSTS_LXTFIF_Pos)                 /*!< CLK_T::CLKDSTS: LXTFIF Mask            */

#define CLK_CLKDSTS_HXTFQIF_Pos          (17)                                              /*!< CLK_T::CLKDSTS: HXTFQIF Position       */
#define CLK_CLKDSTS_HXTFQIF_Msk          (0x1ul << CLK_CLKDSTS_HXTFQIF_Pos)                /*!< CLK_T::CLKDSTS: HXTFQIF Mask           */

#define CLK_CDUPB_UPERBD_Pos             (0)                                               /*!< CLK_T::CDUPB: UPERBD Position          */
#define CLK_CDUPB_UPERBD_Msk             (0x3fful << CLK_CDUPB_UPERBD_Pos)                 /*!< CLK_T::CDUPB: UPERBD Mask              */

#define CLK_CDLOWB_LOWERBD_Pos           (0)                                               /*!< CLK_T::CDLOWB: LOWERBD Position        */
#define CLK_CDLOWB_LOWERBD_Msk           (0x3fful << CLK_CDLOWB_LOWERBD_Pos)               /*!< CLK_T::CDLOWB: LOWERBD Mask            */

#define CLK_ADCCTL_ADC0CKEN_Pos          (0)                                               /*!< CLK_T::ADCCTL: ADC0CKEN Position       */
#define CLK_ADCCTL_ADC0CKEN_Msk          (0x1ul << CLK_ADCCTL_ADC0CKEN_Pos)                /*!< CLK_T::ADCCTL: ADC0CKEN Mask           */

#define CLK_BPWMCTL_BPWM0CKEN_Pos        (0)                                               /*!< CLK_T::BPWMCTL: BPWM0CKEN Position     */
#define CLK_BPWMCTL_BPWM0CKEN_Msk        (0x1ul << CLK_BPWMCTL_BPWM0CKEN_Pos)              /*!< CLK_T::BPWMCTL: BPWM0CKEN Mask         */

#define CLK_BPWMCTL_BPWM1CKEN_Pos        (1)                                               /*!< CLK_T::BPWMCTL: BPWM1CKEN Position     */
#define CLK_BPWMCTL_BPWM1CKEN_Msk        (0x1ul << CLK_BPWMCTL_BPWM1CKEN_Pos)              /*!< CLK_T::BPWMCTL: BPWM1CKEN Mask         */

#define CLK_CRCCTL_CRC0CKEN_Pos          (0)                                               /*!< CLK_T::CRCCTL: CRC0CKEN Position       */
#define CLK_CRCCTL_CRC0CKEN_Msk          (0x1ul << CLK_CRCCTL_CRC0CKEN_Pos)                /*!< CLK_T::CRCCTL: CRC0CKEN Mask           */

#define CLK_FMCCTL_FMC0CKEN_Pos          (0)                                               /*!< CLK_T::FMCCTL: FMC0CKEN Position       */
#define CLK_FMCCTL_FMC0CKEN_Msk          (0x1ul << CLK_FMCCTL_FMC0CKEN_Pos)                /*!< CLK_T::FMCCTL: FMC0CKEN Mask           */

#define CLK_FMCCTL_DFMC0CKEN_Pos         (8)                                               /*!< CLK_T::FMCCTL: DFMC0CKEN Position      */
#define CLK_FMCCTL_DFMC0CKEN_Msk         (0x1ul << CLK_FMCCTL_DFMC0CKEN_Pos)               /*!< CLK_T::FMCCTL: DFMC0CKEN Mask          */

#define CLK_FMCCTL_ISP0CKEN_Pos          (16)                                              /*!< CLK_T::FMCCTL: ISP0CKEN Position       */
#define CLK_FMCCTL_ISP0CKEN_Msk          (0x1ul << CLK_FMCCTL_ISP0CKEN_Pos)                /*!< CLK_T::FMCCTL: ISP0CKEN Mask           */

#define CLK_GPIOCTL_GPIOACKEN_Pos        (0)                                               /*!< CLK_T::GPIOCTL: GPIOACKEN Position     */
#define CLK_GPIOCTL_GPIOACKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOACKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOACKEN Mask         */

#define CLK_GPIOCTL_GPIOBCKEN_Pos        (1)                                               /*!< CLK_T::GPIOCTL: GPIOBCKEN Position     */
#define CLK_GPIOCTL_GPIOBCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOBCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOBCKEN Mask         */

#define CLK_GPIOCTL_GPIOCCKEN_Pos        (2)                                               /*!< CLK_T::GPIOCTL: GPIOCCKEN Position     */
#define CLK_GPIOCTL_GPIOCCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOCCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOCCKEN Mask         */

#define CLK_GPIOCTL_GPIODCKEN_Pos        (3)                                               /*!< CLK_T::GPIOCTL: GPIODCKEN Position     */
#define CLK_GPIOCTL_GPIODCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIODCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIODCKEN Mask         */

#define CLK_GPIOCTL_GPIOECKEN_Pos        (4)                                               /*!< CLK_T::GPIOCTL: GPIOECKEN Position     */
#define CLK_GPIOCTL_GPIOECKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOECKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOECKEN Mask         */

#define CLK_GPIOCTL_GPIOFCKEN_Pos        (5)                                               /*!< CLK_T::GPIOCTL: GPIOFCKEN Position     */
#define CLK_GPIOCTL_GPIOFCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOFCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOFCKEN Mask         */

#define CLK_GPIOCTL_GPIOGCKEN_Pos        (6)                                               /*!< CLK_T::GPIOCTL: GPIOGCKEN Position     */
#define CLK_GPIOCTL_GPIOGCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOGCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOGCKEN Mask         */

#define CLK_I2CCTL_I2C0CKEN_Pos          (0)                                               /*!< CLK_T::I2CCTL: I2C0CKEN Position       */
#define CLK_I2CCTL_I2C0CKEN_Msk          (0x1ul << CLK_I2CCTL_I2C0CKEN_Pos)                /*!< CLK_T::I2CCTL: I2C0CKEN Mask           */

#define CLK_I2CCTL_I2C1CKEN_Pos          (1)                                               /*!< CLK_T::I2CCTL: I2C1CKEN Position       */
#define CLK_I2CCTL_I2C1CKEN_Msk          (0x1ul << CLK_I2CCTL_I2C1CKEN_Pos)                /*!< CLK_T::I2CCTL: I2C1CKEN Mask           */

#define CLK_I2CCTL_I2C2CKEN_Pos          (2)                                               /*!< CLK_T::I2CCTL: I2C2CKEN Position       */
#define CLK_I2CCTL_I2C2CKEN_Msk          (0x1ul << CLK_I2CCTL_I2C2CKEN_Pos)                /*!< CLK_T::I2CCTL: I2C2CKEN Mask           */

#define CLK_PDMACTL_PDMA0CKEN_Pos        (0)                                               /*!< CLK_T::PDMACTL: PDMA0CKEN Position     */
#define CLK_PDMACTL_PDMA0CKEN_Msk        (0x1ul << CLK_PDMACTL_PDMA0CKEN_Pos)              /*!< CLK_T::PDMACTL: PDMA0CKEN Mask         */

#define CLK_RTCCTL_RTC0CKEN_Pos          (0)                                               /*!< CLK_T::RTCCTL: RTC0CKEN Position       */
#define CLK_RTCCTL_RTC0CKEN_Msk          (0x1ul << CLK_RTCCTL_RTC0CKEN_Pos)                /*!< CLK_T::RTCCTL: RTC0CKEN Mask           */

#define CLK_SRAMCTL_SRAM0CKEN_Pos        (0)                                               /*!< CLK_T::SRAMCTL: SRAM0CKEN Position     */
#define CLK_SRAMCTL_SRAM0CKEN_Msk        (0x1ul << CLK_SRAMCTL_SRAM0CKEN_Pos)              /*!< CLK_T::SRAMCTL: SRAM0CKEN Mask         */

#define CLK_SRAMCTL_SRAM1CKEN_Pos        (1)                                               /*!< CLK_T::SRAMCTL: SRAM1CKEN Position     */
#define CLK_SRAMCTL_SRAM1CKEN_Msk        (0x1ul << CLK_SRAMCTL_SRAM1CKEN_Pos)              /*!< CLK_T::SRAMCTL: SRAM1CKEN Mask         */

#define CLK_STCTL_ST0CKEN_Pos            (0)                                               /*!< CLK_T::STCTL: ST0CKEN Position         */
#define CLK_STCTL_ST0CKEN_Msk            (0x1ul << CLK_STCTL_ST0CKEN_Pos)                  /*!< CLK_T::STCTL: ST0CKEN Mask             */

#define CLK_TMRCTL_TMR0CKEN_Pos          (0)                                               /*!< CLK_T::TMRCTL: TMR0CKEN Position       */
#define CLK_TMRCTL_TMR0CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR0CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR0CKEN Mask           */

#define CLK_TMRCTL_TMR1CKEN_Pos          (1)                                               /*!< CLK_T::TMRCTL: TMR1CKEN Position       */
#define CLK_TMRCTL_TMR1CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR1CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR1CKEN Mask           */

#define CLK_TMRCTL_TMR2CKEN_Pos          (2)                                               /*!< CLK_T::TMRCTL: TMR2CKEN Position       */
#define CLK_TMRCTL_TMR2CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR2CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR2CKEN Mask           */

#define CLK_TMRCTL_TMR3CKEN_Pos          (3)                                               /*!< CLK_T::TMRCTL: TMR3CKEN Position       */
#define CLK_TMRCTL_TMR3CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR3CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR3CKEN Mask           */

#define CLK_TMRCTL_TMR4CKEN_Pos          (4)                                               /*!< CLK_T::TMRCTL: TMR4CKEN Position       */
#define CLK_TMRCTL_TMR4CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR4CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR4CKEN Mask           */

#define CLK_TMRCTL_TMR5CKEN_Pos          (5)                                               /*!< CLK_T::TMRCTL: TMR5CKEN Position       */
#define CLK_TMRCTL_TMR5CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR5CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR5CKEN Mask           */

#define CLK_TMRCTL_TMR6CKEN_Pos          (6)                                               /*!< CLK_T::TMRCTL: TMR6CKEN Position       */
#define CLK_TMRCTL_TMR6CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR6CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR6CKEN Mask           */

#define CLK_TMRCTL_TMR7CKEN_Pos          (7)                                               /*!< CLK_T::TMRCTL: TMR7CKEN Position       */
#define CLK_TMRCTL_TMR7CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR7CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR7CKEN Mask           */

#define CLK_TMRCTL_TMR8CKEN_Pos          (8)                                               /*!< CLK_T::TMRCTL: TMR8CKEN Position       */
#define CLK_TMRCTL_TMR8CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR8CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR8CKEN Mask           */

#define CLK_UARTCTL_UART0CKEN_Pos        (0)                                               /*!< CLK_T::UARTCTL: UART0CKEN Position     */
#define CLK_UARTCTL_UART0CKEN_Msk        (0x1ul << CLK_UARTCTL_UART0CKEN_Pos)              /*!< CLK_T::UARTCTL: UART0CKEN Mask         */

#define CLK_UARTCTL_UART1CKEN_Pos        (1)                                               /*!< CLK_T::UARTCTL: UART1CKEN Position     */
#define CLK_UARTCTL_UART1CKEN_Msk        (0x1ul << CLK_UARTCTL_UART1CKEN_Pos)              /*!< CLK_T::UARTCTL: UART1CKEN Mask         */

#define CLK_UARTCTL_UART2CKEN_Pos        (2)                                               /*!< CLK_T::UARTCTL: UART2CKEN Position     */
#define CLK_UARTCTL_UART2CKEN_Msk        (0x1ul << CLK_UARTCTL_UART2CKEN_Pos)              /*!< CLK_T::UARTCTL: UART2CKEN Mask         */

#define CLK_UARTCTL_UART3CKEN_Pos        (3)                                               /*!< CLK_T::UARTCTL: UART3CKEN Position     */
#define CLK_UARTCTL_UART3CKEN_Msk        (0x1ul << CLK_UARTCTL_UART3CKEN_Pos)              /*!< CLK_T::UARTCTL: UART3CKEN Mask         */

#define CLK_UARTCTL_UART4CKEN_Pos        (4)                                               /*!< CLK_T::UARTCTL: UART4CKEN Position     */
#define CLK_UARTCTL_UART4CKEN_Msk        (0x1ul << CLK_UARTCTL_UART4CKEN_Pos)              /*!< CLK_T::UARTCTL: UART4CKEN Mask         */

#define CLK_USCICTL_USCI0CKEN_Pos        (0)                                               /*!< CLK_T::USCICTL: USCI0CKEN Position     */
#define CLK_USCICTL_USCI0CKEN_Msk        (0x1ul << CLK_USCICTL_USCI0CKEN_Pos)              /*!< CLK_T::USCICTL: USCI0CKEN Mask         */

#define CLK_USCICTL_USCI1CKEN_Pos        (1)                                               /*!< CLK_T::USCICTL: USCI1CKEN Position     */
#define CLK_USCICTL_USCI1CKEN_Msk        (0x1ul << CLK_USCICTL_USCI1CKEN_Pos)              /*!< CLK_T::USCICTL: USCI1CKEN Mask         */

#define CLK_USCICTL_USCI2CKEN_Pos        (2)                                               /*!< CLK_T::USCICTL: USCI2CKEN Position     */
#define CLK_USCICTL_USCI2CKEN_Msk        (0x1ul << CLK_USCICTL_USCI2CKEN_Pos)              /*!< CLK_T::USCICTL: USCI2CKEN Mask         */

#define CLK_USCICTL_USCI3CKEN_Pos        (3)                                               /*!< CLK_T::USCICTL: USCI3CKEN Position     */
#define CLK_USCICTL_USCI3CKEN_Msk        (0x1ul << CLK_USCICTL_USCI3CKEN_Pos)              /*!< CLK_T::USCICTL: USCI3CKEN Mask         */

#define CLK_USCICTL_USCI4CKEN_Pos        (4)                                               /*!< CLK_T::USCICTL: USCI4CKEN Position     */
#define CLK_USCICTL_USCI4CKEN_Msk        (0x1ul << CLK_USCICTL_USCI4CKEN_Pos)              /*!< CLK_T::USCICTL: USCI4CKEN Mask         */

#define CLK_WDTCTL_WDT0CKEN_Pos          (0)                                               /*!< CLK_T::WDTCTL: WDT0CKEN Position       */
#define CLK_WDTCTL_WDT0CKEN_Msk          (0x1ul << CLK_WDTCTL_WDT0CKEN_Pos)                /*!< CLK_T::WDTCTL: WDT0CKEN Mask           */

#define CLK_WWDTCTL_WWDT0CKEN_Pos        (0)                                               /*!< CLK_T::WWDTCTL: WWDT0CKEN Position     */
#define CLK_WWDTCTL_WWDT0CKEN_Msk        (0x1ul << CLK_WWDTCTL_WWDT0CKEN_Pos)              /*!< CLK_T::WWDTCTL: WWDT0CKEN Mask         */

#define CLK_HCLKSEL_HCLKSEL_Pos          (0)                                               /*!< CLK_T::HCLKSEL: HCLKSEL Position       */
#define CLK_HCLKSEL_HCLKSEL_Msk          (0x7ul << CLK_HCLKSEL_HCLKSEL_Pos)                /*!< CLK_T::HCLKSEL: HCLKSEL Mask           */

#define CLK_BPWMSEL_BPWM0SEL_Pos         (0)                                               /*!< CLK_T::BPWMSEL: BPWM0SEL Position      */
#define CLK_BPWMSEL_BPWM0SEL_Msk         (0x1ul << CLK_BPWMSEL_BPWM0SEL_Pos)               /*!< CLK_T::BPWMSEL: BPWM0SEL Mask          */

#define CLK_BPWMSEL_BPWM1SEL_Pos         (4)                                               /*!< CLK_T::BPWMSEL: BPWM1SEL Position      */
#define CLK_BPWMSEL_BPWM1SEL_Msk         (0x1ul << CLK_BPWMSEL_BPWM1SEL_Pos)               /*!< CLK_T::BPWMSEL: BPWM1SEL Mask          */

#define CLK_CLKOSEL_CLKOSEL_Pos          (0)                                               /*!< CLK_T::CLKOSEL: CLKOSEL Position       */
#define CLK_CLKOSEL_CLKOSEL_Msk          (0x7ul << CLK_CLKOSEL_CLKOSEL_Pos)                /*!< CLK_T::CLKOSEL: CLKOSEL Mask           */

#define CLK_TMRSEL0_TMR0SEL_Pos          (0)                                               /*!< CLK_T::TMRSEL0: TMR0SEL Position       */
#define CLK_TMRSEL0_TMR0SEL_Msk          (0x7ul << CLK_TMRSEL0_TMR0SEL_Pos)                /*!< CLK_T::TMRSEL0: TMR0SEL Mask           */

#define CLK_TMRSEL0_TMR1SEL_Pos          (4)                                               /*!< CLK_T::TMRSEL0: TMR1SEL Position       */
#define CLK_TMRSEL0_TMR1SEL_Msk          (0x7ul << CLK_TMRSEL0_TMR1SEL_Pos)                /*!< CLK_T::TMRSEL0: TMR1SEL Mask           */

#define CLK_TMRSEL0_TMR2SEL_Pos          (8)                                               /*!< CLK_T::TMRSEL0: TMR2SEL Position       */
#define CLK_TMRSEL0_TMR2SEL_Msk          (0x7ul << CLK_TMRSEL0_TMR2SEL_Pos)                /*!< CLK_T::TMRSEL0: TMR2SEL Mask           */

#define CLK_TMRSEL0_TMR3SEL_Pos          (12)                                              /*!< CLK_T::TMRSEL0: TMR3SEL Position       */
#define CLK_TMRSEL0_TMR3SEL_Msk          (0x7ul << CLK_TMRSEL0_TMR3SEL_Pos)                /*!< CLK_T::TMRSEL0: TMR3SEL Mask           */

#define CLK_TMRSEL0_TMR4SEL_Pos          (16)                                              /*!< CLK_T::TMRSEL0: TMR4SEL Position       */
#define CLK_TMRSEL0_TMR4SEL_Msk          (0x7ul << CLK_TMRSEL0_TMR4SEL_Pos)                /*!< CLK_T::TMRSEL0: TMR4SEL Mask           */

#define CLK_TMRSEL0_TMR5SEL_Pos          (20)                                              /*!< CLK_T::TMRSEL0: TMR5SEL Position       */
#define CLK_TMRSEL0_TMR5SEL_Msk          (0x7ul << CLK_TMRSEL0_TMR5SEL_Pos)                /*!< CLK_T::TMRSEL0: TMR5SEL Mask           */

#define CLK_TMRSEL0_TMR6SEL_Pos          (24)                                              /*!< CLK_T::TMRSEL0: TMR6SEL Position       */
#define CLK_TMRSEL0_TMR6SEL_Msk          (0x7ul << CLK_TMRSEL0_TMR6SEL_Pos)                /*!< CLK_T::TMRSEL0: TMR6SEL Mask           */

#define CLK_TMRSEL0_TMR7SEL_Pos          (28)                                              /*!< CLK_T::TMRSEL0: TMR7SEL Position       */
#define CLK_TMRSEL0_TMR7SEL_Msk          (0x7ul << CLK_TMRSEL0_TMR7SEL_Pos)                /*!< CLK_T::TMRSEL0: TMR7SEL Mask           */

#define CLK_TMRSEL1_TMR8SEL_Pos          (0)                                               /*!< CLK_T::TMRSEL1: TMR8SEL Position       */
#define CLK_TMRSEL1_TMR8SEL_Msk          (0x7ul << CLK_TMRSEL1_TMR8SEL_Pos)                /*!< CLK_T::TMRSEL1: TMR8SEL Mask           */

#define CLK_UARTSEL_UART0SEL_Pos         (0)                                               /*!< CLK_T::UARTSEL: UART0SEL Position      */
#define CLK_UARTSEL_UART0SEL_Msk         (0x7ul << CLK_UARTSEL_UART0SEL_Pos)               /*!< CLK_T::UARTSEL: UART0SEL Mask          */

#define CLK_UARTSEL_UART1SEL_Pos         (4)                                               /*!< CLK_T::UARTSEL: UART1SEL Position      */
#define CLK_UARTSEL_UART1SEL_Msk         (0x7ul << CLK_UARTSEL_UART1SEL_Pos)               /*!< CLK_T::UARTSEL: UART1SEL Mask          */

#define CLK_UARTSEL_UART2SEL_Pos         (8)                                               /*!< CLK_T::UARTSEL: UART2SEL Position      */
#define CLK_UARTSEL_UART2SEL_Msk         (0x7ul << CLK_UARTSEL_UART2SEL_Pos)               /*!< CLK_T::UARTSEL: UART2SEL Mask          */

#define CLK_UARTSEL_UART3SEL_Pos         (12)                                              /*!< CLK_T::UARTSEL: UART3SEL Position      */
#define CLK_UARTSEL_UART3SEL_Msk         (0x7ul << CLK_UARTSEL_UART3SEL_Pos)               /*!< CLK_T::UARTSEL: UART3SEL Mask          */

#define CLK_UARTSEL_UART4SEL_Pos         (16)                                              /*!< CLK_T::UARTSEL: UART4SEL Position      */
#define CLK_UARTSEL_UART4SEL_Msk         (0x7ul << CLK_UARTSEL_UART4SEL_Pos)               /*!< CLK_T::UARTSEL: UART4SEL Mask          */

#define CLK_WDTSEL_WDT0SEL_Pos           (0)                                               /*!< CLK_T::WDTSEL: WDT0SEL Position        */
#define CLK_WDTSEL_WDT0SEL_Msk           (0x3ul << CLK_WDTSEL_WDT0SEL_Pos)                 /*!< CLK_T::WDTSEL: WDT0SEL Mask            */

#define CLK_WWDTSEL_WWDT0SEL_Pos         (0)                                               /*!< CLK_T::WWDTSEL: WWDT0SEL Position      */
#define CLK_WWDTSEL_WWDT0SEL_Msk         (0x1ul << CLK_WWDTSEL_WWDT0SEL_Pos)               /*!< CLK_T::WWDTSEL: WWDT0SEL Mask          */

#define CLK_HCLKDIV_HCLKDIV_Pos          (0)                                               /*!< CLK_T::HCLKDIV: HCLKDIV Position       */
#define CLK_HCLKDIV_HCLKDIV_Msk          (0xful << CLK_HCLKDIV_HCLKDIV_Pos)                /*!< CLK_T::HCLKDIV: HCLKDIV Mask           */

#define CLK_PCLKDIV_PCLK0DIV_Pos         (0)                                               /*!< CLK_T::PCLKDIV: PCLK0DIV Position      */
#define CLK_PCLKDIV_PCLK0DIV_Msk         (0x7ul << CLK_PCLKDIV_PCLK0DIV_Pos)               /*!< CLK_T::PCLKDIV: PCLK0DIV Mask          */

#define CLK_PCLKDIV_PCLK1DIV_Pos         (4)                                               /*!< CLK_T::PCLKDIV: PCLK1DIV Position      */
#define CLK_PCLKDIV_PCLK1DIV_Msk         (0x7ul << CLK_PCLKDIV_PCLK1DIV_Pos)               /*!< CLK_T::PCLKDIV: PCLK1DIV Mask          */

#define CLK_ADCDIV_ADC0DIV_Pos           (0)                                               /*!< CLK_T::ADCDIV: ADC0DIV Position        */
#define CLK_ADCDIV_ADC0DIV_Msk           (0xfful << CLK_ADCDIV_ADC0DIV_Pos)                /*!< CLK_T::ADCDIV: ADC0DIV Mask            */

#define CLK_UARTDIV_UART0DIV_Pos         (0)                                               /*!< CLK_T::UARTDIV: UART0DIV Position      */
#define CLK_UARTDIV_UART0DIV_Msk         (0xful << CLK_UARTDIV_UART0DIV_Pos)               /*!< CLK_T::UARTDIV: UART0DIV Mask          */

#define CLK_UARTDIV_UART1DIV_Pos         (4)                                               /*!< CLK_T::UARTDIV: UART1DIV Position      */
#define CLK_UARTDIV_UART1DIV_Msk         (0xful << CLK_UARTDIV_UART1DIV_Pos)               /*!< CLK_T::UARTDIV: UART1DIV Mask          */

#define CLK_UARTDIV_UART2DIV_Pos         (8)                                               /*!< CLK_T::UARTDIV: UART2DIV Position      */
#define CLK_UARTDIV_UART2DIV_Msk         (0xful << CLK_UARTDIV_UART2DIV_Pos)               /*!< CLK_T::UARTDIV: UART2DIV Mask          */

#define CLK_UARTDIV_UART3DIV_Pos         (12)                                              /*!< CLK_T::UARTDIV: UART3DIV Position      */
#define CLK_UARTDIV_UART3DIV_Msk         (0xful << CLK_UARTDIV_UART3DIV_Pos)               /*!< CLK_T::UARTDIV: UART3DIV Mask          */

#define CLK_UARTDIV_UART4DIV_Pos         (16)                                              /*!< CLK_T::UARTDIV: UART4DIV Position      */
#define CLK_UARTDIV_UART4DIV_Msk         (0xful << CLK_UARTDIV_UART4DIV_Pos)               /*!< CLK_T::UARTDIV: UART4DIV Mask          */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */
/**@}*/ /* end of REGISTER group */

#endif /* __CLK_REG_H__ */
