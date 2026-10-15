/**************************************************************************//**
 * @file     clk_reg.h
 * @version  V1.00
 * @brief    CLK register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CLK_REG_H__
#define __CLK_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/*---------------------- System Clock Controller -------------------------*/
/**
    @addtogroup CLK System Clock Controller (CLK)
    Memory Mapped Structure for CLK Controller
@{ */

typedef struct
{
    /**
     * @var CLK_T::PWRCTL
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |LXTEN     |LXT Enable Bit (Write Protect)
     * |        |          |0 = 32.768 kHz external low speed crystal (LXT) Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal (LXT) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: When LXT is enabled, PF.4(X32_OUT) and PF.5(X32_IN) must be set as input mode.
     * |[2]     |HIRCEN    |HIRC Enable Bit (Write Protect)
     * |        |          |0 = 16 MHz internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = 16 MHz internal high speed RC oscillator (HIRC) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |LIRCEN    |LIRC Enable Bit (Write Protect)
     * |        |          |0 = 38.4 kHz internal low speed RC oscillator (LIRC) Disabled.
     * |        |          |1 = 38.4 kHz internal low speed RC oscillator (LIRC) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |PDWKIEN   |Power-down Mode Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Power-down mode wake-up interrupt Disabled.
     * |        |          |1 = Power-down mode wake-up interrupt Enabled.
     * |        |          |Note1: The interrupt will occur when both PDWKIF and PDWKIEN are high.
     * |        |          |Note2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |PDWKIF    |Power-down Mode Wake-up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", it indicates that resume from Power-down mode.
     * |        |          |The flag is set if any wake-up source is occurred. Refer Power Modes and Wake-up Sources chapter.
     * |        |          |Note1: Write 1 to clear the bit to 0.
     * |        |          |Note2: This bit works only if PDWKIEN (CLK_PWRCTL[5]) set to 1.
     * |[7]     |PDEN      |System Power-down Enable (Write Protect)
     * |        |          |When this bit is set to 1, Power-down mode is enabled and chip keeps active till the CPU sleep mode is also active and then the chip enters Power-down mode.
     * |        |          |When chip wakes up from Power-down mode, this bit is auto cleared
     * |        |          |Users need to set this bit again for next Power-down.
     * |        |          |0 = Chip will not enter Power-down mode after CPU sleep command WFI.
     * |        |          |1 = Chip enters Power-down mode after CPU sleep command WFI.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[13]    |LIRCMSEL  |LIRC High Accuracy Mode Selection (Write Protect)
     * |        |          |0 = LIRC is in normal mode.
     * |        |          |1 = LIRC is in high accuracy mode.
     * |        |          |Note 1: User must disable LIRC before change LIRC mode.
     * |        |          |Note 2: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[17:16] |HIRCSTBS  |HIRC Stable Count Select (Write Protect )
     * |        |          |00 = HIRC stable count is 96 clocks.
     * |        |          |01 = HIRC stable count is 64 clocks.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[24]    |MIRCSTBS  |MIRC Stable Count Select
     * |        |          |0 = MIRC 2% stable count.
     * |        |          |1 = MIRC 5% stable count.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[26]    |MIRCEN    |MIRC Enable Bit (Write Protect)
     * |        |          |0 = MIRC is Disabled.
     * |        |          |1 = MIRC is Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[31:28] |MIRCFSEL  |MIRC Frequency Select Bits (Write Protect)
     * |        |          |0000 = 1 MHz.
     * |        |          |0001 = 2 MHz.
     * |        |          |0010 = 4 MHz.
     * |        |          |0011 = 8 MHz.
     * |        |          |0100 = 12 MHz.
     * |        |          |0101 = 16 MHz.
     * |        |          |0110 = 24 MHz.
     * |        |          |0111 = 32 MHz.
     * |        |          |1000 = 40 MHz.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Users must wait for MIRCSTB(CLK_STATUS[5]) =1 to change MIRCFSEL.
     * |        |          |Note 3: MIRCFSEL(CLK_PWRCTL[31:28]) update value from PDWKMIRCS (CLK_PMUCTL[31:30]) after chip wakeup from NPD1/NPD2 with Flash powerless setting(CLK_PMUCTL[28]) or SPD mode
     * @var CLK_T::AHBCLK0
     * Offset: 0x04  AHB Devices Clock Enable Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |PDMA0CKEN |PDMA0 Controller Clock Enable Bit
     * |        |          |0 = PDMA0 peripheral clock Disabled.
     * |        |          |1 = PDMA0 peripheral clock Enabled.
     * |[3]     |ISPCKEN   |Flash ISP Controller Clock Enable Bit
     * |        |          |0 = Flash ISP peripheral clock Disabled.
     * |        |          |1 = Flash ISP peripheral clock Enabled.
     * |[7]     |CRCCKEN   |CRC Generator Controller Clock Enable Bit
     * |        |          |0 = CRC peripheral clock Disabled.
     * |        |          |1 = CRC peripheral clock Enabled.
     * |[12]    |CRPTCKEN  |Cryptographic Accelerator Clock Enable Bit
     * |        |          |0 = Cryptographic Accelerator clock Disabled.
     * |        |          |1 = Cryptographic Accelerator clock Enabled.
     * |[15]    |FMCIDLE   |Flash Memory Controller Clock Enable Bit in IDLE Mode
     * |        |          |0 = FMC clock Disabled when chip is under IDLE mode.
     * |        |          |1 = FMC clock Enabled when chip is under IDLE mode.
     * |[23]    |FMCFDIS   |FMC Clock Force Disable Bit
     * |        |          |0 = FMC clock Enabled.
     * |        |          |1 = FMC clock force Disable to save power.
     * |        |          |Note: User should make sure program no FLASH access during this bit is 1
     * |[24]    |GPACKEN   |GPIOA Clock Enable Bit
     * |        |          |0 = GPIOA clock Disabled.
     * |        |          |1 = GPIOA clock Enabled.
     * |[25]    |GPBCKEN   |GPIOB Clock Enable Bit
     * |        |          |0 = GPIOB clock Disabled.
     * |        |          |1 = GPIOB clock Enabled.
     * |[26]    |GPCCKEN   |GPIOC Clock Enable Bit
     * |        |          |0 = GPIOC clock Disabled.
     * |        |          |1 = GPIOC clock Enabled.
     * |[27]    |GPDCKEN   |GPIOD Clock Enable Bit
     * |        |          |0 = GPIOD clock Disabled.
     * |        |          |1 = GPIOD clock Enabled.
     * |[28]    |GPECKEN   |GPIOE Clock Enable Bit
     * |        |          |0 = GPIOE clock Disabled.
     * |        |          |1 = GPIOE clock Enabled.
     * |[29]    |GPFCKEN   |GPIOF Clock Enable Bit
     * |        |          |0 = GPIOF clock Disabled.
     * |        |          |1 = GPIOF clock Enabled.
     * |[30]    |GPGCKEN   |GPIOG Clock Enable Bit
     * |        |          |0 = GPIOG clock Disabled.
     * |        |          |1 = GPIOG clock Enabled.
     * |[31]    |GPHCKEN   |GPIOH Clock Enable Bit
     * |        |          |0 = GPIOH clock Disabled.
     * |        |          |1 = GPIOH clock Enabled.
     * @var CLK_T::APBCLK0
     * Offset: 0x08  APB Devices Clock Enable Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDTCKEN   |Watchdog Timer Clock Enable Bit (Write Protect)
     * |        |          |0 = Watchdog timer clock Disabled.
     * |        |          |1 = Watchdog timer clock Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |RTCCKEN   |Real-time-clock APB Interface Clock Enable Bit
     * |        |          |This bit is used to control the RTC APB clock only.
     * |        |          |The RTC peripheral clock source is selected from RTCCKSEL (RTC_LXTCTL[7]).
     * |        |          |It can be selected to external low speed crystal oscillator (LXT) or internal low speed RC oscillator (LIRC).
     * |        |          |0 = RTC clock Disabled.
     * |        |          |1 = RTC clock Enabled.
     * |[2]     |TMR0CKEN  |Timer0 Clock Enable Bit
     * |        |          |0 = Timer0 clock Disabled.
     * |        |          |1 = Timer0 clock Enabled.
     * |[3]     |TMR1CKEN  |Timer1 Clock Enable Bit
     * |        |          |0 = Timer1 clock Disabled.
     * |        |          |1 = Timer1 clock Enabled.
     * |[4]     |TMR2CKEN  |Timer2 Clock Enable Bit
     * |        |          |0 = Timer2 clock Disabled.
     * |        |          |1 = Timer2 clock Enabled.
     * |[5]     |TMR3CKEN  |Timer3 Clock Enable Bit
     * |        |          |0 = Timer3 clock Disabled.
     * |        |          |1 = Timer3 clock Enabled.
     * |[6]     |CLKOCKEN  |CLKO Clock Enable Bit
     * |        |          |0 = CLKO clock Disabled.
     * |        |          |1 = CLKO clock Enabled.
     * |[7]     |ACMP01CKEN|ACMP01 Clock Enable Bit
     * |        |          |0 = ACMP01 clock Disabled.
     * |        |          |1 = ACMP01 clock Enabled.
     * |[8]     |I2C0CKEN  |I2C0 Clock Enable Bit
     * |        |          |0 = I2C0 clock Disabled.
     * |        |          |1 = I2C0 clock Enabled.
     * |[9]     |I2C1CKEN  |I2C1 Clock Enable Bit
     * |        |          |0 = I2C1 clock Disabled.
     * |        |          |1 = I2C1 clock Enabled.
     * |[10]    |I2C2CKEN  |I2C2 Clock Enable Bit
     * |        |          |0 = I2C2 clock Disabled.
     * |        |          |1 = I2C2 clock Enabled.
     * |[12]    |SPI0CKEN  |SPI0 Clock Enable Bit
     * |        |          |0 = SPI0 clock Disabled.
     * |        |          |1 = SPI0 clock Enabled.
     * |[13]    |SPI1CKEN  |SPI1 Clock Enable Bit
     * |        |          |0 = SPI1 clock Disabled.
     * |        |          |1 = SPI1 clock Enabled.
     * |[14]    |SPI2CKEN  |SPI2 Clock Enable Bit
     * |        |          |0 = SPI2 clock Disabled.
     * |        |          |1 = SPI2 clock Enabled.
     * |[16]    |UART0CKEN |UART0 Clock Enable Bit
     * |        |          |0 = UART0 clock Disabled.
     * |        |          |1 = UART0 clock Enabled.
     * |[17]    |UART1CKEN |UART1 Clock Enable Bit
     * |        |          |0 = UART1 clock Disabled.
     * |        |          |1 = UART1 clock Enabled.
     * |[18]    |UART2CKEN |UART2 Clock Enable Bit
     * |        |          |0 = UART2 clock Disabled.
     * |        |          |1 = UART2 clock Enabled.
     * |[20]    |USCI0CKEN |USCI0 Clock Enable Bit
     * |        |          |0 = USCI0 clock Disabled.
     * |        |          |1 = USCI0 clock Enabled.
     * |[21]    |WWDTCKEN  |Window Watchdog Timer Clock Enable Bit (Write Protect)
     * |        |          |0 = Window Watchdog timer clock Disabled.
     * |        |          |1 = Window Watchdog timer clock Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[22]    |PWM0CKEN  |PWM0 Clock Enable Bit
     * |        |          |0 = PWM0 clock Disabled.
     * |        |          |1 = PWM0 clock Enabled.
     * |[23]    |BPWM0CKEN |BPWM0 Clock Enable Bit
     * |        |          |0 = BPWM0 clock Disabled.
     * |        |          |1 = BPWM0 clock Enabled.
     * |[24]    |LCDCKEN   |LCD Clock Enable Bit
     * |        |          |0 = LCD clock Disabled.
     * |        |          |1 = LCD clock Enabled.
     * |[25]    |LCDCCKEN  |LCD Charge Pump Clock Enable Bit
     * |        |          |0 = LCD charge pump clock Disabled.
     * |        |          |1 = LCD charge pump clock Enabled.
     * |[28]    |ADC0CKEN  |ADC0 Clock Enable Bit
     * |        |          |0 = ADC0 clock Disabled.
     * |        |          |1 = ADC0 clock Enabled.
     * @var CLK_T::CLKSEL0
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |HCLKSEL   |HCLK Clock Source Selection (Write Protect)
     * |        |          |Before clock switching, the related clock sources (both pre-select and new-select) must be turned on.
     * |        |          |00 = Clock source from MIRC.
     * |        |          |01 = Clock source from HIRC.
     * |        |          |10 = Clock source from LIRC.
     * |        |          |11 = Clock source from LXT.
     * |        |          |Other = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::CLKSEL1
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDTSEL    |Watchdog Timer Clock Source Selection (Write Protect)
     * |        |          |0 = Clock source from LIRC.
     * |        |          |1 = Clock source from LXT.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[2:1]   |WWDTSEL   |Window Watchdog Timer Clock Source Selection
     * |        |          |00 = Clock source from LIRC.
     * |        |          |01 = Clock source from LXT.
     * |        |          |10 = Clock source from HCLK/2048.
     * |        |          |others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[6:4]   |CLKOSEL   |Clock Divider Clock Source Selection
     * |        |          |000 = Clock source from HCLK.
     * |        |          |001 = Clock source from MIRC.
     * |        |          |010 = Clock source from HIRC .
     * |        |          |011 = Clock source from LIRC.
     * |        |          |100 = Clock source from LXT.
     * |        |          |others = Reserved.
     * |        |          |Note: CLKOSEL can only be changed under CLKOCKEN = 0.
     * |[10:8]  |TMR0SEL   |TIMER0 Clock Source Selection
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from MIRC.
     * |        |          |010 = Clock source from HIRC.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |100 = Clock source from LXT.
     * |        |          |101 = Clock source from external clock TM0 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[14:12] |TMR1SEL   |TIMER1 Clock Source Selection
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from MIRC.
     * |        |          |010 = Clock source from HIRC.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |100 = Clock source from LXT.
     * |        |          |101 = Clock source from external clock TM1 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[18:16] |TMR2SEL   |TIMER2 Clock Source Selection
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from MIRC.
     * |        |          |010 = Clock source from HIRC.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |100 = Clock source from LXT.
     * |        |          |101 = Clock source from external clock TM2 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[22:20] |TMR3SEL   |TIMER3 Clock Source Selection
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from MIRC.
     * |        |          |010 = Clock source from HIRC.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |100 = Clock source from LXT.
     * |        |          |101 = Clock source from external clock TM3 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[25:24] |ADC0SEL   |ADC0 Clock Source Selection (Write Protect)
     * |        |          |00 = Clock source from PCLK1.
     * |        |          |01 = Clock source from MIRC.
     * |        |          |10 = Clock source from HIRC.
     * |        |          |11 = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[28]    |LCDSEL    |LCD Clock Source Selection
     * |        |          |0 = Clock source from LIRC.
     * |        |          |1 = Clock source from LXT.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[30:29] |LCDCPSEL  |LCD Charge Pump Clock Source Selection
     * |        |          |00 = LCD Clock source /32.
     * |        |          |01 = LCD Clock source /16.
     * |        |          |10 = LCD Clock source /8.
     * |        |          |11 = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: When changing the clock source selection during peripheral operation, user must perform a peripheral reset (SYS_IPRST1) after clock source selection is changed.
     * @var CLK_T::CLKSEL2
     * Offset: 0x18  Clock Source Select Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |SPI0SEL   |SPI0 Clock Source Selection
     * |        |          |00 = Clock source from PCLK1.
     * |        |          |01 = Clock source from MIRC.
     * |        |          |10 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |[5:4]   |SPI1SEL   |SPI1 Clock Source Selection
     * |        |          |00 = Clock source from PCLK0.
     * |        |          |01 = Clock source from MIRC.
     * |        |          |10 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |[9:8]   |SPI2SEL   |SPI2 Clock Source Selection
     * |        |          |00 = Clock source from PCLK1.
     * |        |          |01 = Clock source from MIRC.
     * |        |          |10 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |[18:16] |UART0SEL  |UART0 Clock Source Selection
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from MIRC.
     * |        |          |010 = Clock source from HIRC.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |100 = Clock source from LXT.
     * |        |          |Others = Reserved.
     * |[22:20] |UART1SEL  |UART1 Clock Source Selection
     * |        |          |000 = Clock source from PCLK1.
     * |        |          |001 = Clock source from MIRC.
     * |        |          |010 = Clock source from HIRC.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |100 = Clock source from LXT.
     * |        |          |Others = Reserved.
     * |[26:24] |UART2SEL  |UART2 Clock Source Selection
     * |        |          |000 = Clock source from PCLK0.
     * |        |          |001 = Clock source from MIRC.
     * |        |          |010 = Clock source from HIRC.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |100 = Clock source from LXT.
     * |        |          |Others = Reserved.
     * @var CLK_T::HCLKDIV
     * Offset: 0x20  AHB Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLKDIV   |HCLK Clock Divide Number from HCLK Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1).
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::PCLKDIV
     * Offset: 0x24  APB Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |APB0DIV   |APB0 Clock Divider
     * |        |          |APB0 clock can be divided from HCLK.
     * |        |          |000 = PCLK0 frequency is HCLK.
     * |        |          |001= PCLK0 frequency is 1/2 HCLK.
     * |        |          |010 = PCLK0 frequency is 1/4 HCLK.
     * |        |          |011 = PCLK0 frequency is 1/8 HCLK.
     * |        |          |100 = PCLK0 frequency is 1/16 HCLK.
     * |        |          |Others = Reserved.
     * |[6:4]   |APB1DIV   |APB1 Clock Divider
     * |        |          |APB1 clock can be divided from HCLK.
     * |        |          |000 = PCLK1 frequency is HCLK.
     * |        |          |001 = PCLK1 frequency is 1/2 HCLK.
     * |        |          |010 = PCLK1 frequency is 1/4 HCLK.
     * |        |          |011 = PCLK1 frequency is 1/8 HCLK.
     * |        |          |100 = PCLK1 frequency is 1/16 HCLK.
     * |        |          |Others = Reserved.
     * @var CLK_T::CLKDIV
     * Offset: 0x28  Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |UART0DIV  |UART0 Clock Divide Number from UART0 Clock Source
     * |        |          |UART0 clock frequency = (UART0 clock source frequency) / (UART0DIV + 1).
     * |[7:4]   |UART1DIV  |UART1 Clock Divide Number from UART1 Clock Source
     * |        |          |UART1 clock frequency = (UART1 clock source frequency) / (UART1DIV + 1).
     * |[11:8]  |UART2DIV  |UART2 Clock Divide Number from UART2 Clock Source
     * |        |          |UART2 clock frequency = (UART2 clock source frequency) / (UART2DIV + 1).
     * |[23:16] |ADC0DIV   |ADC0 Clock Divide Number from ADC0 Clock Source
     * |        |          |ADC0 clock frequency = (ADC0 clock source frequency) / (ADC0DIV + 1).
     * @var CLK_T::KEEP
     * Offset: 0x30  Clock Keep Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |TMR0KEEP  |Timer0 Clock Keep Bit
     * |        |          |0 = Timer0 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = Timer0 clock Enabled when entering NPD0~1 mode.
     * |[3]     |TMR1KEEP  |Timer1 Clock Keep Bit
     * |        |          |0 = Timer1 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = Timer1 clock Enabled when entering NPD0~1 mode.
     * |[4]     |TMR2KEEP  |Timer2 Clock Keep Bit
     * |        |          |0 = Timer2 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = Timer2 clock Enabled when entering NPD0~1 mode.
     * |[5]     |TMR3KEEP  |Timer3 Clock Keep Bit
     * |        |          |0 = Timer3 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = Timer3 clock Enabled when entering NPD0~1 mode.
     * |[8]     |I2C0KEEP  |I2C0 Clock Keep Bit
     * |        |          |0 = I2C0 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = I2C0 clock Enabled when entering NPD0~1 mode.
     * |[9]     |I2C1KEEP  |I2C1 Clock Keep Bit
     * |        |          |0 = I2C1 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = I2C1 clock Enabled when entering NPD0~1 mode.
     * |[10]    |I2C2KEEP  |I2C2 Clock Keep Bit
     * |        |          |0 = I2C2 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = I2C2 clock Enabled when entering NPD0~1 mode.
     * |[12]    |SPI0KEEP  |SPI0 Clock Keep Bit
     * |        |          |0 = SPI0 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = SPI0 clock Enabled when entering NPD0~1 mode.
     * |[13]    |SPI1KEEP  |SPI1 Clock Keep Bit
     * |        |          |0 = SPI1 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = SPI1 clock Enabled when entering NPD0~1 mode.
     * |[14]    |SPI2KEEP  |SPI2 Clock Keep Bit
     * |        |          |0 = SPI2 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = SPI2 clock Enabled when entering NPD0~1 mode.
     * |[16]    |UART0KEEP |UART0 Clock Keep Bit
     * |        |          |0 = UART0 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = UART0 clock Enabled when entering NPD0~1 mode.
     * |[17]    |UART1KEEP |UART1 Clock Keep Bit
     * |        |          |0 = UART1 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = UART1 clock Enabled when entering NPD0~1 mode.
     * |[18]    |UART2KEEP |UART2 Clock Keep Bit
     * |        |          |0 = UART2 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = UART2 clock Enabled when entering NPD0~1 mode.
     * |[28]    |ADC0KEEP  |ADC0 Clock Keep Bit
     * |        |          |0 = ADC0 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = ADC0 clock Enabled when entering NPD0~1 mode.
     * |[29]    |PDMA0KEEP |PDMA0 Clock Keep Bit
     * |        |          |0 = PDMA0 clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = PDMA0 clock Enabled when entering NPD0~1 mode.
     * |[30]    |GPIOKEEP  |GPIO Clock Keep Bit
     * |        |          |0 = GPIO clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = GPIO clock Enabled when entering NPD0~1 mode.
     * |[31]    |SRAMKEEP  |SRAM Clock Keep Bit
     * |        |          |0 = SRAM clock Disabled when entering NPD0~1 mode.
     * |        |          |1 = SRAM clock Enabled when entering NPD0~1 mode..
     * @var CLK_T::MCTL
     * Offset: 0x34  Clock Monitor Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |CLKM0SEL  |Clock Monitor 0 Source Select
     * |        |          |00 = Monitor source is HCLK.
     * |        |          |01 = Monitor source is HIRC.
     * |        |          |10 = Monitor source is MIRC.
     * |        |          |Others = Reserved.
     * |[9:8]   |CLKM1SEL  |Clock Monitor 1 Source Select
     * |        |          |00 = Monitor source is HCLK.
     * |        |          |01 = Monitor source is HIRC.
     * |        |          |10 = Monitor source is MIRC.
     * |        |          |Others = Reserved.
     * |[24]    |AOCMEN1   |Auto-operation Clock Monitor Enable 1
     * |        |          |0 = Auto-operation clock monitor function is Disabled.
     * |        |          |1 = Auto-operation clock monitor function is Enabled.
     * |        |          |If clock monitor source (selected by CLKM0SEL) is active, PF.0 will output high.
     * |        |          |If clock monitor source (selected by CLKM1SEL) is active, PF.1 will output high.
     * |[25]    |AOCMEN2   |Auto-operation Clock Monitor Enable 2
     * |        |          |0 = Auto-operation clock monitor function is Disabled.
     * |        |          |1 = Auto-operation clock monitor function is Enabled.
     * |        |          |If clock monitor source (selected by CLKM0SEL) is active, PF.2 will output high.
     * |        |          |If clock monitor source (selected by CLKM1SEL) is active, PF.3 will output high.
     * @var CLK_T::STATUS
     * Offset: 0x50  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |LXTSTB    |LXT Clock Source Stable Flag (Read Only)
     * |        |          |0 = LXT is not stable or disabled.
     * |        |          |1 = LXT is stabled and enabled.
     * |[3]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = LIRC is not stable or disabled.
     * |        |          |1 = LIRC is stable and enabled.
     * |[4]     |HIRCSTB   |HIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = HIRC is not stable or disabled.
     * |        |          |1 = HIRC is stable and enabled.
     * |[5]     |MIRCSTB   |MIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = MIRC is not stable or disabled.
     * |        |          |1 = MIRC is stable and enabled.
     * |[7]     |CLKSFAIL  |Clock Switching Fail Flag (Read Only)
     * |        |          |This bit is updated when software switches system clock source
     * |        |          |If switch target clock is stable, this bit will be set to 0
     * |        |          |If switch target clock is not stable, this bit will be set to 1.
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failure.
     * |        |          |Note: This bit is read only
     * |        |          |After selected clock source is stable, hardware will switch system clock to selected clock automatically, and CLKSFAIL will be cleared automatically by hardware.
     * @var CLK_T::CLKOCTL
     * Offset: 0x60  Clock Output Control Register
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
     * Offset: 0x70  Clock Fail Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LXTFDEN   |LXT Clock Fail Detector Enable Bit
     * |        |          |0 = LXT fail detector Disabled.
     * |        |          |1 = LXT fail detector Enabled.
     * |[1]     |LXTFIEN   |LXT Clock Fail Interrupt Enable Bit
     * |        |          |0 = LXT fail detector interrupt Disabled.
     * |        |          |1 = LXT fail detector interrupt Enabled.
     * @var CLK_T::CLKDSTS
     * Offset: 0x74  Clock Fail Detector Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LXTFIF    |LXT Clock Fail Interrupt Flag (Write Protect)
     * |        |          |0 = LXT clock operates normally.
     * |        |          |1 = LXT clock stops.
     * |        |          |Note 1: Write 1 to clear the bit to 0.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::PMUCTL
     * Offset: 0x90  Power Manager Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |PDMSEL    |Power-down Mode Selection (Write Protect)
     * |        |          |These bits control chip power-down mode grade selection when CPU execute WFI/WFE instruction.
     * |        |          |000 = Normal Power-down mode 0 is selected (NPD0).
     * |        |          |001 = Normal Power-down mode 1 is selected (NPD1).
     * |        |          |010 = Normal Power-down mode 2 is selected (NPD2).
     * |        |          |100 = Standby Power-down mode 0 is selected (SPD0).
     * |        |          |110 = Deep Power-down mode 0 is selected (DPD0).
     * |        |          |Other = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[8]     |SRETSEL   |SRAM Retention Range Select Bit (Write Protect)
     * |        |          |Select SRAM retention range when chip enters SPD mode.
     * |        |          |0 = No SRAM retention.
     * |        |          |1 = SRAM retention when chip enters SPD mode.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: SRAM retention range for M2U51xG is 0x2000_0000 - 0x2000_3FFF, for M2U51xC is 0x2000_0000 - 0x2000_17FF
     * |        |          |Note 3: Users should make sure program stack is within SRAM retention range before chip enters SPD mode
     * |[16]    |NRLDCSEL  |Normal run LDO DCDC Power Mode Selection (Write Protect)
     * |        |          |Select LDO/DCDC power mode when chip normal run.
     * |        |          |0 = LDO/DCDC in Active mode during chip normal run. (Default)
     * |        |          |1 = LDO/DCDC in LP mode during chip normal run.
     * |        |          |Note 1 : This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2 : Set this bits 0 before change MVRS(SYS_PLCTL[4]) between LDO and DCDC.
     * |        |          |Note 3: This bit is default value after SPD wakeup.
     * |[18]    |NRBGLPEL  |Normal run Band-gap Power Mode Selection (Write Protect)
     * |        |          |Select Band-gap power mode when chip normal run.
     * |        |          |0 = Band-gap in Active during chip normal run. (Default)
     * |        |          |1 = Band-gap in LP mode during chip normal run.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is default value after SPD wakeup.
     * |[25:24] |PDLDCSEL  |Power Down LDO DCDC Power Mode Selection (Write Protect)
     * |        |          |Select LDO/DCDC power mode when chip enters NPDx/SPD0 mode.
     * |        |          |00 = LDO in Active mode during chip in Power-down mode.
     * |        |          |01 = LDO/DCDC in LP mode during chip in Power-down mode. (Default)
     * |        |          |10 = LDO/DCDC in ULP mode during chip in Power-down mode.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit change to default value after change PDMSEL(CLK_PMUCTL[2:0]), setting this bit after select PDMSEL(CLK_PMUCTL[2:0]).
     * |        |          |Note 3: Not support DCDC at active mode in Power-down mode.
     * |[27]    |PDLVRSEL  |Power Down LVR Power Mode Selection (Write Protect)
     * |        |          |Select LVR power mode when chip enters NPDx/SPD0 mode.
     * |        |          |0 = LVR in LP mode during chip in Power-down mode.
     * |        |          |1 = LVR in ULP mode during chip in Power-down mode. (Default)
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: After changing PDMSEL(CLK_PMUCTL[2:0]), this bit will returne to the default value
     * |        |          |User must set this bit again after changing PDMSEL(CLK_PMUCTL[2:0]).
     * |[28]    |PDFSHSEL  |Power Down FLASH Power Mode Selection (Write Protect)
     * |        |          |Select Flash power mode when chip enters NPD1/NPD2 mode.
     * |        |          |0 = Flash in DeepStandby mode during chip in Power-down mode.(.Default)
     * |        |          |1 = Flash in Power-Off mode during chip in Power-down mode.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: After changing PDMSEL(CLK_PMUCTL[2:0]), this bit will returne to the default value
     * |        |          |User must set this bit again after changing PDMSEL(CLK_PMUCTL[2:0]).
     * |[29]    |WKFSHSEL  |Wakeup FLASH Macro Mode Selection (Write Protect)
     * |        |          |Select Wakeup Flash macro mode select.
     * |        |          |0 = Flash Macro wakeup with slow speed.(.Default)
     * |        |          |1 = Flash Macro wakeup with fast speed.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: FLASH macro wakeup with fast speed, the peak current consumption is large than slow speed.
     * |[31:30] |PDWKMIRCS |Option for wakeup MIRC clock frequency selection
     * |        |          |For chip wakeup from NPD1/NPD2 with Flash powerless setting(CLK_PMUCTL[28]) or SPD, this options will change MIRC clock frequency selection to speed up wakeup time.
     * |        |          |00 = MIRC clock frequency is 1 MHz after wakeup. (Default)
     * |        |          |01 = MIRC clock frequency is 2 MHz after wakeup.
     * |        |          |10 = MIRC clock frequency is 4 MHz after wakeup.
     * |        |          |11 = MIRC clock frequency is 8 MHz after wakeup.
     * |        |          |Note: For the auto operation which using MIRC, the clock frequency may change after chip wakeup from NPD1 with Flash powerless.
     * @var CLK_T::PMUSTS
     * Offset: 0x94  Power Manager Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKPIN0    |Pin0 Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wake-up from DPD mode by WKPIN PC.0.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[1]     |TMRWK     |Timer Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wake-up from NPD0~2/SPD0 mode by wakeup timer time-out.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[2]     |RTCWK     |RTC Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wakeup of from NPD2/SPD0 mode by RTC alarm or tick time happened.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[3]     |WKPIN1    |Pin1 Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from DPD mode by WKPIN PB.0.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[4]     |WKPIN2    |Pin2 Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from DPD mode by WKPIN PB.2.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[5]     |WKPIN3    |Pin3 Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from DPD mode by WKPIN PB.12.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[6]     |WKPIN4    |Pin4 Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from DPD mode by WKPIN PF.6.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[8]     |GPAWK0    |GPA Wake-up 0 Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD0~2/SPD mode by a selected PA.x WKIO pin.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[9]     |GPBWK0    |GPB Wake-up 0 Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD0~2/SPD mode by a selected PB.x WKIO pin.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[10]    |GPCWK0    |GPC Wake-up 0 Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD0~2/SPD mode by a selected PC.x WKIO pin.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[11]    |GPDWK0    |GPD Wake-up 0 Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD0~2/SPD mode by a selected PD.x WKIO pin.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[12]    |LVRWK     |LVR Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD0~2/SPD mode by LVR.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[13]    |BODWK     |BOD Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from SPD mode by BOD.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[15]    |RSTWK     |RST pin Wake-up Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD2/SPD/DPD mode by nRESET pin.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[24]    |GPAWK1    |GPA Wake-up 1 Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD0~2/SPD mode by a selected PA.x WKIO pin.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[25]    |GPBWK1    |GPB Wake-up 1 Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD0~2/SPD mode by a selected PB.x WKIO pin.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[26]    |GPCWK1    |GPC Wake-up 1 Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD0~2/SPD mode by a selected PC.x WKIO pin.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[27]    |GPDWK1    |GPD Wake-up 1 Flag (Read Only)
     * |        |          |This flag indicates that the chip wake up from NPD0~2/SPD mode by a selected PD.x WKIO pin.
     * |        |          |Note: If DISAUTOC(CLK_ PMUWKCTL[31])=1, this flag must be cleared before chip entering Power-down mode by setting CLRWK=1.
     * |[31]    |CLRWK     |Clear Wake-up Flag
     * |        |          |0 = Not cleared.
     * |        |          |1 = Clear all wake-up flags.
     * |        |          |Note 1: This bit is auto cleared by hardware.
     * |        |          |Note 2: If DISAUTOC (CLK_ PMUWKCTL[31])=0, all wake-up flags in CLK_PMUSTS are auto cleared when chip enters Power-down mode.
     * @var CLK_T::PMUWKCTL
     * Offset: 0x98  Power Manager Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKTMREN   |Wake-up Timer Enable Bit (Write Protect)
     * |        |          |0 = Wake-up timer Disabled.
     * |        |          |1 = Wake-up timer Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |WKTMRMOD  |Wake-up Timer Mode (Write Protect)
     * |        |          |0 = Wake-up timer started when entering any of Power-down mode ( except CPU idle mode).
     * |        |          |1 = Wake-up timer started immedially when WKTMREN (CLK_PMUWKCTL[0]) =1.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |RTCWKEN   |RTC Wake-up Enable Bit (Write Protect)
     * |        |          |0 = RTC wake-up disable at Deep Power-down mode or Standby Power-down mode.
     * |        |          |1 = RTC wake-up enabled at Deep Power-down mode or Standby Power-down mode.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[11:8]  |WKTMRIS   |Wake-up Timer Time-out Interval Select (Write Protect)
     * |        |          |These bits control wake-up timer time-out interval when chip at SPD mode.
     * |        |          |0000 = Time-out interval is 512 LIRC clocks (16ms).
     * |        |          |0001 = Time-out interval is 1024 LIRC clocks (32ms).
     * |        |          |0010 = Time-out interval is 2048 LIRC clocks (64ms).
     * |        |          |0011 = Time-out interval is 4096 LIRC clocks (128ms).
     * |        |          |0100 = Time-out interval is 8192 LIRC clocks (256ms).
     * |        |          |0101 = Time-out interval is 16384 LIRC clocks (512ms).
     * |        |          |0110 = Time-out interval is 32768 LIRC clocks (1024ms).
     * |        |          |0111 = Time-out interval is 65536 LIRC clocks (2048ms).
     * |        |          |1000 = Time-out interval is 131072 LIRC clocks (4096ms).
     * |        |          |1001 = Time-out interval is 262144 LIRC clocks (8192ms).
     * |        |          |1010 = Time-out interval is 524288 LIRC clocks (16384ms).
     * |        |          |1011 = Time-out interval is 1048576 LIRC clocks (32768ms).
     * |        |          |1100 = Time-out interval is 2097152 LIRC clocks (65536ms).
     * |        |          |1101 = Time-out interval is 4194304 LIRC clocks (131072ms).
     * |        |          |Others = Time-out interval is 512 LIRC clocks (16ms).
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[17:16] |WKPINEN0  |Wake-up Pin0 Enable Bit (Write Protect)
     * |        |          |This is control register for GPC.0 to wake-up pin.
     * |        |          |00 = Wake-up pin disable at Deep Power-down mode.
     * |        |          |01 = Wake-up pin rising edge enabled at Deep Power-down mode.
     * |        |          |10 = Wake-up pin falling edge enabled at Deep Power-down mode.
     * |        |          |11 = Wake-up pin both edge enabled at Deep Power-down mode.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[19:18] |WKPINEN1  |Wake-up Pin1 Enable Bit (Write Protect)
     * |        |          |This is control register for GPB.0 to wake-up pin.
     * |        |          |00 = Wake-up pin disable at Deep Power-down mode.
     * |        |          |01 = Wake-up pin rising edge enabled at Deep Power-down mode.
     * |        |          |10 = Wake-up pin falling edge enabled at Deep Power-down mode.
     * |        |          |11 = Wake-up pin both edge enabled at Deep Power-down mode.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[21:20] |WKPINEN2  |Wake-up Pin2 Enable Bit (Write Protect)
     * |        |          |This is control register for GPB.2 to wake-up pin.
     * |        |          |00 = Wake-up pin disable at Deep Power-down mode.
     * |        |          |01 = Wake-up pin rising edge enabled at Deep Power-down mode.
     * |        |          |10 = Wake-up pin falling edge enabled at Deep Power-down mode.
     * |        |          |11 = Wake-up pin both edge enabled at Deep Power-down mode.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[23:22] |WKPINEN3  |Wake-up Pin3 Enable Bit (Write Protect)
     * |        |          |This is control register for GPB.12 to wake-up pin.
     * |        |          |00 = Wake-up pin disable at Deep Power-down mode.
     * |        |          |01 = Wake-up pin rising edge enabled at Deep Power-down mode.
     * |        |          |10 = Wake-up pin falling edge enabled at Deep Power-down mode.
     * |        |          |11 = Wake-up pin both edge enabled at Deep Power-down mode.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[25:24] |WKPINEN4  |Wake-up Pin4 Enable Bit (Write Protect)
     * |        |          |This is control register for GPF.6 to wake-up pin.
     * |        |          |00 = Wake-up pin disable at Deep Power-down mode.
     * |        |          |01 = Wake-up pin rising edge enabled at Deep Power-down mode.
     * |        |          |10 = Wake-up pin falling edge enabled at Deep Power-down mode.
     * |        |          |11 = Wake-up pin both edge enabled at Deep Power-down mode.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note: Setting IOCTLSEL(RTC_LXTCTL[8]) to avoid GPF6 unexpected falling edge.
     * |[31]    |DISAUTOC  |Auto Clear Wakeup flag Disable Bit (Write Protect)
     * |        |          |0 = Auto clear function is enabled
     * |        |          |All of wake-up flags in CLK_PMUSTS are auto cleared when chip enters Power-down mode.
     * |        |          |1 = Auto clear function is disabled.
     * |        |          |Note 1:This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2:When chip wakeup from SPD/DPD mode, this bits will be cleared to 0 automatically.
     * |        |          |Note 3: It takes one HCLK cycle to perform auto clear function
     * |        |          |In this one HCLK interval, any wake-up event will not work properly.
     * @var CLK_T::PWDBCTL
     * Offset: 0x9C  GPIO Pin WKIO De-bounce Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |SWKDBCLKSEL|WKIO De-bounce Sampling Cycle Selection
     * |        |          |0000 = Sample wake-up input once per 1 clocks.
     * |        |          |0001 = Sample wake-up input once per 2 clocks.
     * |        |          |0010 = Sample wake-up input once per 4 clocks.
     * |        |          |0011 = Sample wake-up input once per 8 clocks.
     * |        |          |0100 = Sample wake-up input once per 16 clocks.
     * |        |          |0101 = Sample wake-up input once per 32 clocks.
     * |        |          |0110 = Sample wake-up input once per 64 clocks.
     * |        |          |0111 = Sample wake-up input once per 128 clocks.
     * |        |          |1000 = Sample wake-up input once per 256 clocks.
     * |        |          |1001 = Sample wake-up input once per 2*256 clocks.
     * |        |          |1010 = Sample wake-up input once per 4*256 clocks.
     * |        |          |1011 = Sample wake-up input once per 8*256 clocks.
     * |        |          |1100 = Sample wake-up input once per 16*256 clocks.
     * |        |          |1101 = Sample wake-up input once per 32*256 clocks.
     * |        |          |1110 = Sample wake-up input once per 64*256 clocks.
     * |        |          |1111 = Sample wake-up input once per 128*256 clocks..
     * |        |          |Note: De-bounce counter clock source is LIRC.
     * @var CLK_T::PAPWCTL
     * Offset: 0xA0  GPA Pin WKIO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN0     |GPA Pin 0 Wake-up-I/O Enable Bit
     * |        |          |0 = GPA group pin 0 Wake-up-I/O function Disabled.
     * |        |          |1 = GPA group pin 0 Wakeup-I/O function Enabled.
     * |[1]     |PRWKEN0   |GPA Pin 0 Rising Edge Detect Enable Bit
     * |        |          |0 = GPA group pin 0 rising edge detect function Disabled.
     * |        |          |1 = GPA group pin 0 rising edge detect function Enabled.
     * |[2]     |PFWKEN0   |GPA Pin 0 Falling Edge Detect Enable Bit
     * |        |          |0 = GPA group pin 0 falling edge detect function Disabled.
     * |        |          |1 = GPA group pin 0 falling edge detect function Enabled.
     * |[7:4]   |WKPSEL0   |GPA Pin 0 Wakeup-I/O Pin Select
     * |        |          |0000 = GPA.0 as Wake-up-I/O function select.
     * |        |          |0001 = GPA.1 as Wakeup-I/O function select.
     * |        |          |0010 = GPA.2 as Wakeup-I/O function select.
     * |        |          |0011 = GPA.3 as Wakeup-I/O function select.
     * |        |          |0100 = GPA.4 as Wakeup-I/O function select.
     * |        |          |0101 = GPA.5 as Wakeup-I/O function select.
     * |        |          |0110 = GPA.6 as Wakeup-I/O function select.
     * |        |          |0111 = GPA.7 as Wakeup-I/O function select.
     * |        |          |1000 = GPA.8 as Wakeup-I/O function select.
     * |        |          |1001 = GPA.9 as Wakeup-I/O function select.
     * |        |          |1010 = GPA.10 as Wakeup-I/O function select.
     * |        |          |1011 = GPA.11 as Wakeup-I/O function select.
     * |        |          |1100 = GPA.12 as Wakeup-I/O function select.
     * |        |          |1101 = GPA.13 as Wakeup-I/O function select.
     * |        |          |1110 = GPA.14 as Wakeup-I/O function select.
     * |        |          |1111 = GPA.15 as Wakeup-I/O function select.
     * |[8]     |DBEN0     |GPA Pin 0 Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source LIRC.
     * |        |          |0 = GPA group pin 0 Wake-up-I/O pin De-bounce function Disabled.
     * |        |          |1 = GPA group pin 0 Wakeup-I/O pin De-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edgetriggered.
     * |        |          |Note: De-bounce function stable time is 4T of CLK8 kHz, enabled DBENx and wait function stable before enter power down in NMRx=0 condition.
     * |[10]    |TRIGM0    |GPA Pin 0 Wake-up Pin Trigger Mode Select
     * |        |          |0 = GPA group pin 0 wake-up chip Enabled, trigger ip Enabled.
     * |        |          |1 = GPA group pin 0 wake-up chip Disabled, trigger ip Enabled.
     * |[11]    |NMR0      |GPA Pin 0 Function Enable at Normal Run Mode Select
     * |        |          |0 = GPA group pin 0 wake-up function enable when chip enters power down.
     * |        |          |1 = GPA group pin 0 wake-up function enable when chip is normal run.
     * |        |          |Note: Enable this bit and the I/O wake-up function is enabled immediately.
     * |[16]    |WKEN1     |GPA Pin 1 Wake-up Enable Bit
     * |        |          |0 = GPA group pin 1 wake-up function Disabled.
     * |        |          |1 = GPA group pin 1 wake-up function Enabled.
     * |[17]    |PRWKEN1   |GPA Pin 1 Rising Edge Detect Enable Bit
     * |        |          |0 = GPA group pin 1 rising edge detect function Disabled.
     * |        |          |1 = GPA group pin 1 rising edge detect function Enabled.
     * |[18]    |PFWKEN1   |GPA Pin 1 Falling Edge Detect Enable Bit
     * |        |          |0 = GPA group pin 1 falling edge detect function Disabled.
     * |        |          |1 = GPA group pin 1 falling edge detect function Enabled.
     * |[23:20] |WKPSEL1   |GPA Pin 1 Wakeup-I/O Pin Select
     * |        |          |0000 = GPA.0 as Wakeup-I/O function select.
     * |        |          |0001 = GPA.1 as Wakeup-I/O function select.
     * |        |          |0010 = GPA.2 as Wakeup-I/O function select.
     * |        |          |0011 = GPA.3 as Wakeup-I/O function select.
     * |        |          |0100 = GPA.4 as Wakeup-I/O function select.
     * |        |          |0101 = GPA.5 as Wakeup-I/O function select.
     * |        |          |0110 = GPA.6 as Wakeup-I/O function select.
     * |        |          |0111 = GPA.7 as Wakeup-I/O function select.
     * |        |          |1000 = GPA.8 as Wakeup-I/O function select.
     * |        |          |1001 = GPA.9 as Wakeup-I/O function select.
     * |        |          |1010 = GPA.10 as Wakeup-I/O function select.
     * |        |          |1011 = GPA.11 as Wakeup-I/O function select.
     * |        |          |1100 = GPA.12 as Wakeup-I/O function select.
     * |        |          |1101 = GPA.13 as Wakeup-I/O function select.
     * |        |          |1110 = GPA.14 as Wakeup-I/O function select.
     * |        |          |1111 = GPA.15 as Wakeup-I/O function select.
     * |[24]    |DBEN1     |GPA Pin 1 Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is LIRC.
     * |        |          |0 = GPA group pin 1 Wakeup-I/O pin De-bounce function Disabled.
     * |        |          |1 = GPA group pin 1 Wakeup-I/O pin De-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edgetriggered.
     * |        |          |Note: De-bounce function stable time is 4T of CLK8 kHz, enabled DBENx and wait function stable before enter power down in NMRx=0 condition.
     * |[26]    |TRIGM1    |GPA Pin 1 Wake-up Pin trigger Mode Select
     * |        |          |0 = GPA group pin 1 wake-up chip Enabled, trigger ip Enabled.
     * |        |          |1 = GPA group pin 1 wake-up chip Disabled, trigger ip Enabled.
     * |[27]    |NMR1      |GPA Pin 1 Function Enable at Normal Run Mode Select
     * |        |          |0 = GPA group pin 1 wake-up function enable when chip enters power down.
     * |        |          |1 = GPA group pin 1 wake-up function enable when chip is normal run.
     * |        |          |Note: Enable this bit and the WKIO function is enabled immediately.
     * @var CLK_T::PBPWCTL
     * Offset: 0xA4  GPB Pin WKIO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN0     |GPB Pin 0 Wakeup-I/O Enable Bit
     * |        |          |0 = GPB group pin 0 Wakeup-I/O function Disabled.
     * |        |          |1 = GPB group pin 0 Wakeup-I/O function Enabled.
     * |[1]     |PRWKEN0   |GPB Pin 0 Rising Edge Detect Enable Bit
     * |        |          |0 = GPB group pin 0 rising edge detect function Disabled.
     * |        |          |1 = GPB group pin 0 rising edge detect function Enabled.
     * |[2]     |PFWKEN0   |GPB Pin 0 Falling Edge Detect Enable Bit
     * |        |          |0 = GPB group pin 0 falling edge detect function Disabled.
     * |        |          |1 = GPB group pin 0 falling edge detect function Enabled.
     * |[7:4]   |WKPSEL0   |GPB Pin 0 Wakeup-I/O Pin Select
     * |        |          |0000 = GPB.0 as Wakeup-I/O function select.
     * |        |          |0001 = GPB.1 as Wakeup-I/O function select.
     * |        |          |0010 = GPB.2 as Wakeup-I/O function select.
     * |        |          |0011 = GPB.3 as Wakeup-I/O function select.
     * |        |          |0100 = GPB.4 as Wakeup-I/O function select.
     * |        |          |0101 = GPB.5 as Wakeup-I/O function select.
     * |        |          |0110 = GPB.6 as Wakeup-I/O function select.
     * |        |          |0111 = GPB.7 as Wakeup-I/O function select.
     * |        |          |1000 = GPB.8 as Wakeup-I/O function select.
     * |        |          |1001 = GPB.9 as Wakeup-I/O function select.
     * |        |          |1010 = GPB.10 as Wakeup-I/O function select.
     * |        |          |1011 = GPB.11 as Wakeup-I/O function select.
     * |        |          |1100 = GPB.12 as Wakeup-I/O function select.
     * |        |          |1101 = GPB.13 as Wakeup-I/O function select.
     * |        |          |1110 = GPB.14 as Wakeup-I/O function select.
     * |        |          |1111 = GPB.15 as Wakeup-I/O function select.
     * |[8]     |DBEN0     |GPB Pin 0 Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is LIRC.
     * |        |          |0 = GPB group pin 0 Wakeup-I/O pin De-bounce function Disabled.
     * |        |          |1 = GPB group pin 0 Wakeup-I/O pin De-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edgetriggered.
     * |        |          |Note: De-bounce function stable time is 4T of CLK8 kHz, enabled DBENx and wait function stable before enter power down in NMRx=0 condition.
     * |[10]    |TRIGM0    |GPB Pin 0 Wake-up Pin Trigger Mode Select
     * |        |          |0 = GPB group pin 0 wake-up chip Enabled, trigger ip Enabled.
     * |        |          |1 = GPB group pin 0 wake-up chip Disabled, trigger ip Enabled.
     * |[11]    |NMR0      |GPB Pin 0 Function Enable at Normal Run Mode Select
     * |        |          |0 = GPB group pin 0 wake-up function enable when chip enters power down.
     * |        |          |1 = GPB group pin 0 wake-up function enable when chip is normal run.
     * |        |          |Note: Enable this bit and the I/O wake-up function is enabled immediately.
     * |[16]    |WKEN1     |GPB Pin 1 Wake-up Enable Bit
     * |        |          |0 = GPB group pin 1 wake-up function Disabled.
     * |        |          |1 = GPB group pin 1 wake-up function Enabled.
     * |[17]    |PRWKEN1   |GPB Pin 1 Rising Edge Detect Enable Bit
     * |        |          |0 = GPB group pin 1 rising edge detect function Disabled.
     * |        |          |1 = GPB group pin 1 rising edge detect function Enabled.
     * |[18]    |PFWKEN1   |GPB Pin 1 Falling Edge Detect Enable Bit
     * |        |          |0 = GPB group pin 1 falling edge detect function Disabled.
     * |        |          |1 = GPB group pin 1 falling edge detect function Enabled.
     * |[23:20] |WKPSEL1   |GPB Pin 1 Wakeup-I/O Pin Select
     * |        |          |0000 = GPB.0 as Wakeup-I/O function select.
     * |        |          |0001 = GPB.1 as Wakeup-I/O function select.
     * |        |          |0010 = GPB.2 as Wakeup-I/O function select.
     * |        |          |0011 = GPB.3 as Wakeup-I/O function select.
     * |        |          |0100 = GPB.4 as Wakeup-I/O function select.
     * |        |          |0101 = GPB.5 as Wakeup-I/O function select.
     * |        |          |0110 = GPB.6 as Wakeup-I/O function select.
     * |        |          |0111 = GPB.7 as Wakeup-I/O function select.
     * |        |          |1000 = GPB.8 as Wakeup-I/O function select.
     * |        |          |1001 = GPB.9 as Wakeup-I/O function select.
     * |        |          |1010 = GPB.10 as Wakeup-I/O function select.
     * |        |          |1011 = GPB.11 as Wakeup-I/O function select.
     * |        |          |1100 = GPB.12 as Wakeup-I/O function select.
     * |        |          |1101 = GPB.13 as Wakeup-I/O function select.
     * |        |          |1110 = GPB.14 as Wakeup-I/O function select.
     * |        |          |1111 = GPB.15 as Wakeup-I/O function select.
     * |[24]    |DBEN1     |GPB Pin 1 Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is LIRC.
     * |        |          |0 = GPB group pin 1 Wakeup-I/O pin De-bounce function Disabled.
     * |        |          |1 = GPB group pin 1 Wakeup-I/O pin De-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edgetriggered.
     * |        |          |Note: De-bounce function stable time is 4T of CLK8 kHz, enabled DBENx and wait function stable before enter power down in NMRx=0 condition.
     * |[26]    |TRIGM1    |GPB Pin 1 Wake-up Pin Trigger Mode Select
     * |        |          |0 = GPB group pin 1 wake-up chip Enabled, trigger ip Enabled.
     * |        |          |1 = GPB group pin 1 wake-up chip Disabled, trigger ip Enabled.
     * |[27]    |NMR1      |GPB Pin 1 Function Enable at Normal Run Mode Select
     * |        |          |0 = GPB group pin 1 wake-up function enable when chip enters power down.
     * |        |          |1 = GPB group pin 1 wake-up function enable when chip is normal run.
     * |        |          |Note: Enable this bit and the WKIO function is enabled immediately.
     * @var CLK_T::PCPWCTL
     * Offset: 0xA8  GPC Pin WKIO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN0     |GPC Pin 0 Wakeup-I/O Enable Bit
     * |        |          |0 = GPC group pin 0 Wakeup-I/O function Disabled.
     * |        |          |1 = GPC group pin 0 Wakeup-I/O function Enabled.
     * |[1]     |PRWKEN0   |GPC Pin 0 Rising Edge Detect Enable Bit
     * |        |          |0 = GPC group pin 0 rising edge detect function Disabled.
     * |        |          |1 = GPC group pin 0 rising edge detect function Enabled.
     * |[2]     |PFWKEN0   |GPC Pin 0 Falling Edge Detect Enable Bit
     * |        |          |0 = GPC group pin 0 falling edge detect function Disabled.
     * |        |          |1 = GPC group pin 0 falling edge detect function Enabled.
     * |[7:4]   |WKPSEL0   |GPC Pin 0 Wakeup-I/O Pin Select
     * |        |          |0000 = GPC.0 as Wakeup-I/O function select.
     * |        |          |0001 = GPC.1 as Wakeup-I/O function select.
     * |        |          |0010 = GPC.2 as Wakeup-I/O function select.
     * |        |          |0011 = GPC.3 as Wakeup-I/O function select.
     * |        |          |0100 = GPC.4 as Wakeup-I/O function select.
     * |        |          |0101 = GPC.5 as Wakeup-I/O function select.
     * |        |          |0110 = GPC.6 as Wakeup-I/O function select.
     * |        |          |0111 = GPC.7 as Wakeup-I/O function select.
     * |        |          |1000 = GPC.8 as Wakeup-I/O function select.
     * |        |          |1001 = GPC.9 as Wakeup-I/O function select.
     * |        |          |1010 = GPC.10 as Wakeup-I/O function select.
     * |        |          |1011 = GPC.11 as Wakeup-I/O function select.
     * |        |          |1100 = GPC.12 as Wakeup-I/O function select.
     * |        |          |1101 = GPC.13 as Wakeup-I/O function select.
     * |        |          |1110 = GPC.14 as Wakeup-I/O function select.
     * |        |          |1111 = GPC.15 as Wakeup-I/O function select.
     * |[8]     |DBEN0     |GPC Pin 0 Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is LIRC.
     * |        |          |0 = GPC group pin 0 Wakeup-I/O pin De-bounce function Disabled.
     * |        |          |1 = GPC group pin 0 Wakeup-I/O pin De-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edgetriggered.
     * |        |          |Note: De-bounce function stable time is 4T of CLK8 kHz, enabled DBENx and wait function stable before enter power down in NMRx=0 condition.
     * |[10]    |TRIGM0    |GPC Pin 0 Wake-up Pin trigger Mode Select
     * |        |          |0 = GPC group pin 0 wake-up chip Enabled, trigger ip Enabled.
     * |        |          |1 = GPC group pin 0 wake-up chip Disabled, trigger ip Enabled.
     * |[11]    |NMR0      |GPC Pin 0 Function Enable at Normal Run Mode Select
     * |        |          |0 = GPC group pin 0 wake-up function enabled when chip enters power down.
     * |        |          |1 = GPC group pin 0 wake-up function enabled when chip is normal run.
     * |        |          |Note: Enable this bit and the I/O wake-up function is enabled immediately.
     * |[16]    |WKEN1     |GPC Pin 1 Wake-up Enable Bit
     * |        |          |0 = GPC group pin 1 wake-up function Disabled.
     * |        |          |1 = GPC group pin 1 wake-up function Enabled.
     * |[17]    |PRWKEN1   |GPC Pin 1 Rising Edge Detect Enable Bit
     * |        |          |0 = GPC group pin 1 rising edge detect function Disabled.
     * |        |          |1 = GPC group pin 1 rising edge detect function Enabled.
     * |[18]    |PFWKEN1   |GPC Pin 1 Falling Edge Detect Enable Bit
     * |        |          |0 = GPC group pin 1 falling edge detect function Disabled.
     * |        |          |1 = GPC group pin 1 falling edge detect function Enabled.
     * |[23:20] |WKPSEL1   |GPC Pin 1 Wakeup-I/O Pin Select
     * |        |          |0000 = GPC.0 as Wakeup-I/O function select.
     * |        |          |0001 = GPC.1 as Wakeup-I/O function select.
     * |        |          |0010 = GPC.2 as Wakeup-I/O function select.
     * |        |          |0011 = GPC.3 as Wakeup-I/O function select.
     * |        |          |0100 = GPC.4 as Wakeup-I/O function select.
     * |        |          |0101 = GPC.5 as Wakeup-I/O function select.
     * |        |          |0110 = GPC.6 as Wakeup-I/O function select.
     * |        |          |0111 = GPC.7 as Wakeup-I/O function select.
     * |        |          |1000 = GPC.8 as Wakeup-I/O function select.
     * |        |          |1001 = GPC.9 as Wakeup-I/O function select.
     * |        |          |1010 = GPC.10 as Wakeup-I/O function select.
     * |        |          |1011 = GPC.11 as Wakeup-I/O function select.
     * |        |          |1100 = GPC.12 as Wakeup-I/O function select.
     * |        |          |1101 = GPC.13 as Wakeup-I/O function select.
     * |        |          |1110 = GPC.14 as Wakeup-I/O function select.
     * |        |          |1111 = GPC.15 as Wakeup-I/O function select.
     * |[24]    |DBEN1     |GPC Pin 1 Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is LIRC.
     * |        |          |0 = GPC group pin 1 Wakeup-I/O pin De-bounce function Disabled.
     * |        |          |1 = GPC group pin 1 Wakeup-I/O pin De-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edgetriggered.
     * |        |          |Note: De-bounce function stable time is 4T of CLK8 kHz, enabled DBENx and wait function stable before enter power down in NMRx=0 condition.
     * |[26]    |TRIGM1    |GPC Pin 1 Wake-up Pin Trigger Mode Select
     * |        |          |0 = GPC group pin 1 wake-up chip Enabled, trigger ip Enabled.
     * |        |          |1 = GPC group pin 1 wake-up chip Disabled, trigger ip Enabled.
     * |[27]    |NMR1      |GPC Pin 1 Function Enable at Normal Run Mode Select
     * |        |          |0 = GPC group pin 1 wake-up function enabled when chip enters power down.
     * |        |          |1 = GPC group pin 1 wake-up function enabled when chip is normal run.
     * |        |          |Note: Enable this bit and the WKIO function is enabled immediately.
     * @var CLK_T::PDPWCTL
     * Offset: 0xAC  GPD Pin WKIO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN0     |GPD Pin 0 Wakeup-I/O Enable Bit
     * |        |          |0 = GPD group pin 0 Wakeup-I/O function Disabled.
     * |        |          |1 = GPD group pin 0 Wakeup-I/O function Enabled.
     * |[1]     |PRWKEN0   |GPD Pin 0 Rising Edge Detect Enable Bit
     * |        |          |0 = GPD group pin 0 rising edge detect function Disabled.
     * |        |          |1 = GPD group pin 0 rising edge detect function Enabled.
     * |[2]     |PFWKEN0   |GPD Pin 0 Falling Edge Detect Enable Bit
     * |        |          |0 = GPD group pin 0 falling edge detect function Disabled.
     * |        |          |1 = GPD group pin 0 falling edge detect function Enabled.
     * |[7:4]   |WKPSEL0   |GPD Pin 0 Wakeup-I/O Pin Select
     * |        |          |0000 = GPD.0 as Wakeup-I/O function select.
     * |        |          |0001 = GPD.1 as Wakeup-I/O function select.
     * |        |          |0010 = GPD.2 as Wakeup-I/O function select.
     * |        |          |0011 = GPD.3 as Wakeup-I/O function select.
     * |        |          |0100 = GPD.4 as Wakeup-I/O function select.
     * |        |          |0101 = GPD.5 as Wakeup-I/O function select.
     * |        |          |0110 = GPD.6 as Wakeup-I/O function select.
     * |        |          |0111 = GPD.7 as Wakeup-I/O function select.
     * |        |          |1000 = GPD.8 as Wakeup-I/O function select.
     * |        |          |1001 = GPD.9 as Wakeup-I/O function select.
     * |        |          |1010 = GPD.10 as Wakeup-I/O function select.
     * |        |          |1011 = GPD.11 as Wakeup-I/O function select.
     * |        |          |1100 = GPD.12 as Wakeup-I/O function select.
     * |        |          |1101 = GPD.13 as Wakeup-I/O function select.
     * |        |          |1110 = GPD.14 as Wakeup-I/O function select.
     * |        |          |1111 = GPD.15 as Wakeup-I/O function select.
     * |[8]     |DBEN0     |GPD Pin 0 Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is LIRC.
     * |        |          |0 = GPD group pin 0 Wakeup-I/O pin De-bounce function Disabled.
     * |        |          |1 = GPD group pin 0 Wakeup-I/O pin De-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edgetriggered.
     * |        |          |Note: De-bounce function stable time is 4T of CLK8 kHz, enabled DBENx and wait function stable before enter power down in NMRx=0 condition.
     * |[10]    |TRIGM0    |GPD Pin 0 Wake-up Pin Trigger Mode Select
     * |        |          |0 = GPD group pin 0 wake-up chip Enabled, trigger ip Enabled.
     * |        |          |1 = GPD group pin 0 wake-up chip Disabled, trigger ip Enabled.
     * |[11]    |NMR0      |GPD Pin 0 Function Enable at Normal Run Mode Select
     * |        |          |0 = GPD group pin 0 wake-up function enable when chip enters power down.
     * |        |          |1 = GPD group pin 0 wake-up function enable when chip is normal run.
     * |        |          |Note: Enable this bit and the I/O wake-up function is enabled immediately.
     * |[16]    |WKEN1     |GPD Pin 1 Wake-up Enable Bit
     * |        |          |0 = GPD group pin 1 wake-up function Disabled.
     * |        |          |1 = GPD group pin 1 wake-up function Enabled.
     * |[17]    |PRWKEN1   |GPD Pin 1 Rising Edge Detect Enable Bit
     * |        |          |0 = GPD group pin 1 rising edge detect function Disabled.
     * |        |          |1 = GPD group pin 1 rising edge detect function Enabled.
     * |[18]    |PFWKEN1   |GPD Pin 1 Falling Edge Detect Enable Bit
     * |        |          |0 = GPD group pin 1 falling edge detect function Disabled.
     * |        |          |1 = GPD group pin 1 falling edge detect function Enabled.
     * |[23:20] |WKPSEL1   |GPD Pin 1 Wakeup-I/O Pin Select
     * |        |          |0000 = GPD.0 as Wakeup-I/O function select.
     * |        |          |0001 = GPD.1 as Wakeup-I/O function select.
     * |        |          |0010 = GPD.2 as Wakeup-I/O function select.
     * |        |          |0011 = GPD.3 as Wakeup-I/O function select.
     * |        |          |0100 = GPD.4 as Wakeup-I/O function select.
     * |        |          |0101 = GPD.5 as Wakeup-I/O function select.
     * |        |          |0110 = GPD.6 as Wakeup-I/O function select.
     * |        |          |0111 = GPD.7 as Wakeup-I/O function select.
     * |        |          |1000 = GPD.8 as Wakeup-I/O function select.
     * |        |          |1001 = GPD.9 as Wakeup-I/O function select.
     * |        |          |1010 = GPD.10 as Wakeup-I/O function select.
     * |        |          |1011 = GPD.11 as Wakeup-I/O function select.
     * |        |          |1100 = GPD.12 as Wakeup-I/O function select.
     * |        |          |1101 = GPD.13 as Wakeup-I/O function select.
     * |        |          |1110 = GPD.14 as Wakeup-I/O function select.
     * |        |          |1111 = GPD.15 as Wakeup-I/O function select.
     * |[24]    |DBEN1     |GPD Pin 1 Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is LIRC.
     * |        |          |0 = GPD group pin 1 Wakeup-I/O pin De-bounce function Disabled.
     * |        |          |1 = GPD group pin 1 Wakeup-I/O pin De-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edgetriggered.
     * |        |          |Note: De-bounce function stable time is 4T of CLK8 kHz, enabled DBENx and wait function stable before enter power down in NMRx=0 condition.
     * |[26]    |TRIGM1    |GPD Pin 1 Wake-up Pin Trigger Mode Select
     * |        |          |0 = GPD group pin 1 wake-up chip Enabled, trigger ip Enabled.
     * |        |          |1 = GPD group pin 1 wake-up chip Disabled, trigger ip Enabled.
     * |[27]    |NMR1      |GPD Pin 1 Function Enable at Normal Run Mode Select
     * |        |          |0 = GPD group pin 1 wake-up function enable when chip enters power down.
     * |        |          |1 = GPD group pin 1 wake-up function enable when chip is normal run.
     * |        |          |Note: Enable this bit and the WKIO function is enabled immediately.
     * @var CLK_T::IOPDCTL
     * Offset: 0xB0  GPIO Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IOHR      |GPIO Hold Release
     * |        |          |When GPIO enters deep power-down mode or standby power-down mode, all I/O status are hold to keep normal operating status
     * |        |          |After chip was waked up from deep power-down mode or standby power-down mode, the I/O are still keep hold status until user set this bit to release I/O hold status.
     * |        |          |Note: This bit is auto cleared by hardware.
     * |[8]     |DPDHOLDEN |Deep-Power-Down Mode GPIO Hold Enable Bit
     * |        |          |0= When GPIO enters deep power-down mode, all I/O status are tri-state.
     * |        |          |1= When GPIO enters deep power-down mode, all I/O status are hold to keep normal operating status
     * |        |          |After chip was waked up from deep power-down mode, the I/O are still keep hold status until user set CLK_IOPDCTL[0] to release I/O hold status.
     * @var CLK_T::PMUINTC
     * Offset: 0xC0  Power Manager Interrupt Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKTMRIE   |Wakeup-Timer Interrupt Enable Bit
     * |        |          |0 = Wakeup-Timer interrupt function Disabled.
     * |        |          |1 = Wakeup-Timer interrupt function Enabled.
     * |[8]     |WKIOA0IE  |Wakeup-I/O GPA group Pin 0 Interrupt Enable Bit
     * |        |          |0 = Wakeup-I/O interrupt function Disabled.
     * |        |          |1 = Wakeup-I/O interrupt function Enabled.
     * |[9]     |WKIOB0IE  |Wakeup-I/O GPB group Pin 0 Interrupt Enable Bit
     * |        |          |0 = Wakeup-I/O interrupt function Disabled.
     * |        |          |1 = Wakeup-I/O interrupt function Enabled.
     * |[10]    |WKIOC0IE  |Wakeup-I/O GPC group Pin 0 Interrupt Enable Bit
     * |        |          |0 = Wakeup-I/O interrupt function Disabled.
     * |        |          |1 = Wakeup-I/O interrupt function Enabled.
     * |[11]    |WKIOD0IE  |Wakeup-I/O GPD group Pin 0 Interrupt Enable Bit
     * |        |          |0 = Wakeup-I/O interrupt function Disabled.
     * |        |          |1 = Wakeup-I/O interrupt function Enabled.
     * |[12]    |WKIOA1IE  |Wakeup-I/O GPA group Pin 1 Interrupt Enable Bit
     * |        |          |0 = Wakeup-I/O interrupt function Disabled.
     * |        |          |1 = Wakeup-I/O interrupt function Enabled.
     * |[13]    |WKIOB1IE  |Wakeup-I/O GPB group Pin 1 Interrupt Enable Bit
     * |        |          |0 = Wakeup-I/O interrupt function Disabled.
     * |        |          |1 = Wakeup-I/O interrupt function Enabled.
     * |[14]    |WKIOC1IE  |Wakeup-I/O GPC group Pin 1 Interrupt Enable Bit
     * |        |          |0 = Wakeup-I/O interrupt function Disabled.
     * |        |          |1 = Wakeup-I/O interrupt function Enabled.
     * |[15]    |WKIOD1IE  |Wakeup-I/O GPD group Pin 1 Interrupt Enable Bit
     * |        |          |0 = Wakeup-I/O interrupt function Disabled.
     * |        |          |1 = Wakeup-I/O interrupt function Enabled.
     * @var CLK_T::PMUINTS
     * Offset: 0xC4  Power Manager Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKTMRIF   |Wakeup-Timer Interrupt Flag
     * |        |          |This flag indicates that Wakeup-Timer interrupt happened.
     * |        |          |Flag is set by hardware while Wakeup-Timer event happen when WKTMRIE(CLK_PMUINTC[0])=1.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     * |[8]     |WKIOA0IF  |Wakeup-I/O GPA group Pin 0 Interrupt Flag
     * |        |          |This flag indicates that Wakeup-I/O interrupt happened.
     * |        |          |Flag is set by hardware while Wakeup-I/O event happen when WKIOPA0IE(CLK_PMUINTC[8])=1.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     * |[9]     |WKIOB0IF  |Wakeup-I/O GPB group Pin 0 Interrupt Flag
     * |        |          |This flag indicates that Wakeup-I/O interrupt happened.
     * |        |          |Flag is set by hardware while Wakeup-I/O event happen when WKIOPB0IE(CLK_PMUINTC[9])=1.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     * |[10]    |WKIOC0IF  |Wakeup-I/O GPC group Pin 0 Interrupt Flag
     * |        |          |This flag indicates that Wakeup-I/O interrupt happened.
     * |        |          |Flag is set by hardware while Wakeup-I/O event happen when WKIOPC0IE(CLK_PMUINTC[10])=1.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     * |[11]    |WKIOD0IF  |Wakeup-I/O GPD group Pin 0 Interrupt Flag
     * |        |          |This flag indicates that Wakeup-I/O interrupt happened.
     * |        |          |Flag is set by hardware while Wakeup-I/O event happen when WKIOPD0IE (CLK_PMUINTC[11])=1.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     * |[12]    |WKIOA1IF  |Wakeup-I/O GPA group Pin 1 Interrupt Flag
     * |        |          |This flag indicates that Wakeup-I/O interrupt happened.
     * |        |          |Flag is set by hardware while Wakeup-I/O event happen when WKIOPA1IE(CLK_PMUINTC[12])=1.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     * |[13]    |WKIOB1IF  |Wakeup-I/O GPB group Pin 1 Interrupt Flag
     * |        |          |This flag indicates that Wakeup-I/O interrupt happened.
     * |        |          |Flag is set by hardware while Wakeup-I/O event happen when WKIOPB1IE(CLK_PMUINTC[13])=1.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     * |[14]    |WKIOC1IF  |Wakeup-I/O GPC group Pin 1 Interrupt Flag
     * |        |          |This flag indicates that Wakeup-I/O interrupt happened.
     * |        |          |Flag is set by hardware while Wakeup-I/O event happen when WKIOPC1IE(CLK_PMUINTC[14])=1.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     * |[15]    |WKIOD1IF  |Wakeup-I/O GPD group Pin 1 Interrupt Flag
     * |        |          |This flag indicates that Wakeup-I/O interrupt happened.
     * |        |          |Flag is set by hardware while Wakeup-I/O event happen when WKIOPD1IE(CLK_PMUINTC[15])=1.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     */
    __IO uint32_t PWRCTL;                /*!< [0x0000] System Power-down Control Register                               */
    __IO uint32_t AHBCLK0;               /*!< [0x0004] AHB Devices Clock Enable Control Register 0                      */
    __IO uint32_t APBCLK0;               /*!< [0x0008] APB Devices Clock Enable Control Register 0                      */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t CLKSEL0;               /*!< [0x0010] Clock Source Select Control Register 0                           */
    __IO uint32_t CLKSEL1;               /*!< [0x0014] Clock Source Select Control Register 1                           */
    __IO uint32_t CLKSEL2;               /*!< [0x0018] Clock Source Select Control Register 2                           */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t HCLKDIV;               /*!< [0x0020] AHB Clock Divider Register                                       */
    __IO uint32_t PCLKDIV;               /*!< [0x0024] APB Clock Divider Register                                       */
    __IO uint32_t CLKDIV;                /*!< [0x0028] Clock Divider Control Register                                   */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t KEEP;                  /*!< [0x0030] Clock Keep Control Register                                      */
    __IO uint32_t MCTL;                  /*!< [0x0034] Clock Monitor Control Register                                   */
    __I  uint32_t RESERVE3[6];
    __I  uint32_t STATUS;                /*!< [0x0050] Clock Status Monitor Register                                    */
    __I  uint32_t RESERVE4[3];
    __IO uint32_t CLKOCTL;               /*!< [0x0060] Clock Output Control Register                                    */
    __I  uint32_t RESERVE5[3];
    __IO uint32_t CLKDCTL;               /*!< [0x0070] Clock Fail Detector Control Register                             */
    __IO uint32_t CLKDSTS;               /*!< [0x0074] Clock Fail Detector Status Register                              */
    __I  uint32_t RESERVE6[6];
    __IO uint32_t PMUCTL;                /*!< [0x0090] Power Manager Control Register                                   */
    __IO uint32_t PMUSTS;                /*!< [0x0094] Power Manager Status Register                                    */
    __IO uint32_t PMUWKCTL;              /*!< [0x0098] Power Manager Wake-up Control Register                           */
    __IO uint32_t PWDBCTL;               /*!< [0x009c] GPIO Pin WKIO De-bounce Control Register                         */
    __IO uint32_t PAPWCTL;               /*!< [0x00a0] GPA Pin WKIO Control Register                                    */
    __IO uint32_t PBPWCTL;               /*!< [0x00a4] GPB Pin WKIO Control Register                                    */
    __IO uint32_t PCPWCTL;               /*!< [0x00a8] GPC Pin WKIO Control Register                                    */
    __IO uint32_t PDPWCTL;               /*!< [0x00ac] GPD Pin WKIO Control Register                                    */
    __IO uint32_t IOPDCTL;               /*!< [0x00b0] GPIO Power-down Control Register                                 */
    __I  uint32_t RESERVE7[3];
    __IO uint32_t PMUINTC;               /*!< [0x00c0] Power Manager Interrupt Control Register                         */
    __IO uint32_t PMUINTS;               /*!< [0x00c4] Power Manager Interrupt Status Register                          */
} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_LXTEN_Pos            (1)                                         /*!< CLK_T::PWRCTL: LXTEN Position              */
#define CLK_PWRCTL_LXTEN_Msk            (0x1ul << CLK_PWRCTL_LXTEN_Pos)             /*!< CLK_T::PWRCTL: LXTEN Mask                  */

#define CLK_PWRCTL_HIRCEN_Pos           (2)                                         /*!< CLK_T::PWRCTL: HIRCEN Position             */
#define CLK_PWRCTL_HIRCEN_Msk           (0x1ul << CLK_PWRCTL_HIRCEN_Pos)            /*!< CLK_T::PWRCTL: HIRCEN Mask                 */

#define CLK_PWRCTL_LIRCEN_Pos           (3)                                         /*!< CLK_T::PWRCTL: LIRCEN Position             */
#define CLK_PWRCTL_LIRCEN_Msk           (0x1ul << CLK_PWRCTL_LIRCEN_Pos)            /*!< CLK_T::PWRCTL: LIRCEN Mask                 */

#define CLK_PWRCTL_PDWKIEN_Pos          (5)                                         /*!< CLK_T::PWRCTL: PDWKIEN Position            */
#define CLK_PWRCTL_PDWKIEN_Msk          (0x1ul << CLK_PWRCTL_PDWKIEN_Pos)           /*!< CLK_T::PWRCTL: PDWKIEN Mask                */

#define CLK_PWRCTL_PDWKIF_Pos           (6)                                         /*!< CLK_T::PWRCTL: PDWKIF Position             */
#define CLK_PWRCTL_PDWKIF_Msk           (0x1ul << CLK_PWRCTL_PDWKIF_Pos)            /*!< CLK_T::PWRCTL: PDWKIF Mask                 */

#define CLK_PWRCTL_PDEN_Pos             (7)                                         /*!< CLK_T::PWRCTL: PDEN Position               */
#define CLK_PWRCTL_PDEN_Msk             (0x1ul << CLK_PWRCTL_PDEN_Pos)              /*!< CLK_T::PWRCTL: PDEN Mask                   */

#define CLK_PWRCTL_LIRCMSEL_Pos         (13)                                        /*!< CLK_T::PWRCTL: LIRCMSEL Position           */
#define CLK_PWRCTL_LIRCMSEL_Msk         (0x1ul << CLK_PWRCTL_LIRCMSEL_Pos)          /*!< CLK_T::PWRCTL: LIRCMSEL Mask               */

#define CLK_PWRCTL_HIRCSTBS_Pos         (16)                                        /*!< CLK_T::PWRCTL: HIRCSTBS Position           */
#define CLK_PWRCTL_HIRCSTBS_Msk         (0x3ul << CLK_PWRCTL_HIRCSTBS_Pos)          /*!< CLK_T::PWRCTL: HIRCSTBS Mask               */

#define CLK_PWRCTL_MIRCSTBS_Pos         (24)                                        /*!< CLK_T::PWRCTL: MIRCSTBS Position           */
#define CLK_PWRCTL_MIRCSTBS_Msk         (0x1ul << CLK_PWRCTL_MIRCSTBS_Pos)          /*!< CLK_T::PWRCTL: MIRCSTBS Mask               */

#define CLK_PWRCTL_MIRCEN_Pos           (26)                                        /*!< CLK_T::PWRCTL: MIRCEN Position             */
#define CLK_PWRCTL_MIRCEN_Msk           (0x1ul << CLK_PWRCTL_MIRCEN_Pos)            /*!< CLK_T::PWRCTL: MIRCEN Mask                 */

#define CLK_PWRCTL_MIRCFSEL_Pos         (28)                                        /*!< CLK_T::PWRCTL: MIRCFSEL Position           */
#define CLK_PWRCTL_MIRCFSEL_Msk         (0xful << CLK_PWRCTL_MIRCFSEL_Pos)          /*!< CLK_T::PWRCTL: MIRCFSEL Mask               */

#define CLK_AHBCLK0_PDMA0CKEN_Pos       (2)                                         /*!< CLK_T::AHBCLK0: PDMA0CKEN Position         */
#define CLK_AHBCLK0_PDMA0CKEN_Msk       (0x1ul << CLK_AHBCLK0_PDMA0CKEN_Pos)        /*!< CLK_T::AHBCLK0: PDMA0CKEN Mask             */

#define CLK_AHBCLK0_ISPCKEN_Pos         (3)                                         /*!< CLK_T::AHBCLK0: ISPCKEN Position           */
#define CLK_AHBCLK0_ISPCKEN_Msk         (0x1ul << CLK_AHBCLK0_ISPCKEN_Pos)          /*!< CLK_T::AHBCLK0: ISPCKEN Mask               */

#define CLK_AHBCLK0_CRCCKEN_Pos         (7)                                         /*!< CLK_T::AHBCLK0: CRCCKEN Position           */
#define CLK_AHBCLK0_CRCCKEN_Msk         (0x1ul << CLK_AHBCLK0_CRCCKEN_Pos)          /*!< CLK_T::AHBCLK0: CRCCKEN Mask               */

#define CLK_AHBCLK0_CRPTCKEN_Pos        (12)                                        /*!< CLK_T::AHBCLK0: CRPTCKEN Position          */
#define CLK_AHBCLK0_CRPTCKEN_Msk        (0x1ul << CLK_AHBCLK0_CRPTCKEN_Pos)         /*!< CLK_T::AHBCLK0: CRPTCKEN Mask              */

#define CLK_AHBCLK0_FMCIDLE_Pos         (15)                                        /*!< CLK_T::AHBCLK0: FMCIDLE Position           */
#define CLK_AHBCLK0_FMCIDLE_Msk         (0x1ul << CLK_AHBCLK0_FMCIDLE_Pos)          /*!< CLK_T::AHBCLK0: FMCIDLE Mask               */

#define CLK_AHBCLK0_FMCFDIS_Pos         (23)                                        /*!< CLK_T::AHBCLK0: FMCFDIS Position           */
#define CLK_AHBCLK0_FMCFDIS_Msk         (0x1ul << CLK_AHBCLK0_FMCFDIS_Pos)          /*!< CLK_T::AHBCLK0: FMCFDIS Mask               */

#define CLK_AHBCLK0_GPACKEN_Pos         (24)                                        /*!< CLK_T::AHBCLK0: GPACKEN Position           */
#define CLK_AHBCLK0_GPACKEN_Msk         (0x1ul << CLK_AHBCLK0_GPACKEN_Pos)          /*!< CLK_T::AHBCLK0: GPACKEN Mask               */

#define CLK_AHBCLK0_GPBCKEN_Pos         (25)                                        /*!< CLK_T::AHBCLK0: GPBCKEN Position           */
#define CLK_AHBCLK0_GPBCKEN_Msk         (0x1ul << CLK_AHBCLK0_GPBCKEN_Pos)          /*!< CLK_T::AHBCLK0: GPBCKEN Mask               */

#define CLK_AHBCLK0_GPCCKEN_Pos         (26)                                        /*!< CLK_T::AHBCLK0: GPCCKEN Position           */
#define CLK_AHBCLK0_GPCCKEN_Msk         (0x1ul << CLK_AHBCLK0_GPCCKEN_Pos)          /*!< CLK_T::AHBCLK0: GPCCKEN Mask               */

#define CLK_AHBCLK0_GPDCKEN_Pos         (27)                                        /*!< CLK_T::AHBCLK0: GPDCKEN Position           */
#define CLK_AHBCLK0_GPDCKEN_Msk         (0x1ul << CLK_AHBCLK0_GPDCKEN_Pos)          /*!< CLK_T::AHBCLK0: GPDCKEN Mask               */

#define CLK_AHBCLK0_GPECKEN_Pos         (28)                                        /*!< CLK_T::AHBCLK0: GPECKEN Position           */
#define CLK_AHBCLK0_GPECKEN_Msk         (0x1ul << CLK_AHBCLK0_GPECKEN_Pos)          /*!< CLK_T::AHBCLK0: GPECKEN Mask               */

#define CLK_AHBCLK0_GPFCKEN_Pos         (29)                                        /*!< CLK_T::AHBCLK0: GPFCKEN Position           */
#define CLK_AHBCLK0_GPFCKEN_Msk         (0x1ul << CLK_AHBCLK0_GPFCKEN_Pos)          /*!< CLK_T::AHBCLK0: GPFCKEN Mask               */

#define CLK_AHBCLK0_GPGCKEN_Pos         (30)                                        /*!< CLK_T::AHBCLK0: GPGCKEN Position           */
#define CLK_AHBCLK0_GPGCKEN_Msk         (0x1ul << CLK_AHBCLK0_GPGCKEN_Pos)          /*!< CLK_T::AHBCLK0: GPGCKEN Mask               */

#define CLK_AHBCLK0_GPHCKEN_Pos         (31)                                        /*!< CLK_T::AHBCLK0: GPHCKEN Position           */
#define CLK_AHBCLK0_GPHCKEN_Msk         (0x1ul << CLK_AHBCLK0_GPHCKEN_Pos)          /*!< CLK_T::AHBCLK0: GPHCKEN Mask               */

#define CLK_APBCLK0_WDTCKEN_Pos         (0)                                         /*!< CLK_T::APBCLK0: WDTCKEN Position           */
#define CLK_APBCLK0_WDTCKEN_Msk         (0x1ul << CLK_APBCLK0_WDTCKEN_Pos)          /*!< CLK_T::APBCLK0: WDTCKEN Mask               */

#define CLK_APBCLK0_RTCCKEN_Pos         (1)                                         /*!< CLK_T::APBCLK0: RTCCKEN Position           */
#define CLK_APBCLK0_RTCCKEN_Msk         (0x1ul << CLK_APBCLK0_RTCCKEN_Pos)          /*!< CLK_T::APBCLK0: RTCCKEN Mask               */

#define CLK_APBCLK0_TMR0CKEN_Pos        (2)                                         /*!< CLK_T::APBCLK0: TMR0CKEN Position          */
#define CLK_APBCLK0_TMR0CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR0CKEN_Pos)         /*!< CLK_T::APBCLK0: TMR0CKEN Mask              */

#define CLK_APBCLK0_TMR1CKEN_Pos        (3)                                         /*!< CLK_T::APBCLK0: TMR1CKEN Position          */
#define CLK_APBCLK0_TMR1CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR1CKEN_Pos)         /*!< CLK_T::APBCLK0: TMR1CKEN Mask              */

#define CLK_APBCLK0_TMR2CKEN_Pos        (4)                                         /*!< CLK_T::APBCLK0: TMR2CKEN Position          */
#define CLK_APBCLK0_TMR2CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR2CKEN_Pos)         /*!< CLK_T::APBCLK0: TMR2CKEN Mask              */

#define CLK_APBCLK0_TMR3CKEN_Pos        (5)                                         /*!< CLK_T::APBCLK0: TMR3CKEN Position          */
#define CLK_APBCLK0_TMR3CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR3CKEN_Pos)         /*!< CLK_T::APBCLK0: TMR3CKEN Mask              */

#define CLK_APBCLK0_CLKOCKEN_Pos        (6)                                         /*!< CLK_T::APBCLK0: CLKOCKEN Position          */
#define CLK_APBCLK0_CLKOCKEN_Msk        (0x1ul << CLK_APBCLK0_CLKOCKEN_Pos)         /*!< CLK_T::APBCLK0: CLKOCKEN Mask              */

#define CLK_APBCLK0_ACMP01CKEN_Pos      (7)                                         /*!< CLK_T::APBCLK0: ACMP01CKEN Position        */
#define CLK_APBCLK0_ACMP01CKEN_Msk      (0x1ul << CLK_APBCLK0_ACMP01CKEN_Pos)       /*!< CLK_T::APBCLK0: ACMP01CKEN Mask            */

#define CLK_APBCLK0_I2C0CKEN_Pos        (8)                                         /*!< CLK_T::APBCLK0: I2C0CKEN Position          */
#define CLK_APBCLK0_I2C0CKEN_Msk        (0x1ul << CLK_APBCLK0_I2C0CKEN_Pos)         /*!< CLK_T::APBCLK0: I2C0CKEN Mask              */

#define CLK_APBCLK0_I2C1CKEN_Pos        (9)                                         /*!< CLK_T::APBCLK0: I2C1CKEN Position          */
#define CLK_APBCLK0_I2C1CKEN_Msk        (0x1ul << CLK_APBCLK0_I2C1CKEN_Pos)         /*!< CLK_T::APBCLK0: I2C1CKEN Mask              */

#define CLK_APBCLK0_I2C2CKEN_Pos        (10)                                        /*!< CLK_T::APBCLK0: I2C2CKEN Position          */
#define CLK_APBCLK0_I2C2CKEN_Msk        (0x1ul << CLK_APBCLK0_I2C2CKEN_Pos)         /*!< CLK_T::APBCLK0: I2C2CKEN Mask              */

#define CLK_APBCLK0_SPI0CKEN_Pos        (12)                                        /*!< CLK_T::APBCLK0: SPI0CKEN Position          */
#define CLK_APBCLK0_SPI0CKEN_Msk        (0x1ul << CLK_APBCLK0_SPI0CKEN_Pos)         /*!< CLK_T::APBCLK0: SPI0CKEN Mask              */

#define CLK_APBCLK0_SPI1CKEN_Pos        (13)                                        /*!< CLK_T::APBCLK0: SPI1CKEN Position          */
#define CLK_APBCLK0_SPI1CKEN_Msk        (0x1ul << CLK_APBCLK0_SPI1CKEN_Pos)         /*!< CLK_T::APBCLK0: SPI1CKEN Mask              */

#define CLK_APBCLK0_SPI2CKEN_Pos        (14)                                        /*!< CLK_T::APBCLK0: SPI2CKEN Position          */
#define CLK_APBCLK0_SPI2CKEN_Msk        (0x1ul << CLK_APBCLK0_SPI2CKEN_Pos)         /*!< CLK_T::APBCLK0: SPI2CKEN Mask              */

#define CLK_APBCLK0_UART0CKEN_Pos       (16)                                        /*!< CLK_T::APBCLK0: UART0CKEN Position         */
#define CLK_APBCLK0_UART0CKEN_Msk       (0x1ul << CLK_APBCLK0_UART0CKEN_Pos)        /*!< CLK_T::APBCLK0: UART0CKEN Mask             */

#define CLK_APBCLK0_UART1CKEN_Pos       (17)                                        /*!< CLK_T::APBCLK0: UART1CKEN Position         */
#define CLK_APBCLK0_UART1CKEN_Msk       (0x1ul << CLK_APBCLK0_UART1CKEN_Pos)        /*!< CLK_T::APBCLK0: UART1CKEN Mask             */

#define CLK_APBCLK0_UART2CKEN_Pos       (18)                                        /*!< CLK_T::APBCLK0: UART2CKEN Position         */
#define CLK_APBCLK0_UART2CKEN_Msk       (0x1ul << CLK_APBCLK0_UART2CKEN_Pos)        /*!< CLK_T::APBCLK0: UART2CKEN Mask             */

#define CLK_APBCLK0_USCI0CKEN_Pos       (20)                                        /*!< CLK_T::APBCLK0: USCI0CKEN Position         */
#define CLK_APBCLK0_USCI0CKEN_Msk       (0x1ul << CLK_APBCLK0_USCI0CKEN_Pos)        /*!< CLK_T::APBCLK0: USCI0CKEN Mask             */

#define CLK_APBCLK0_WWDTCKEN_Pos        (21)                                        /*!< CLK_T::APBCLK0: WWDTCKEN Position          */
#define CLK_APBCLK0_WWDTCKEN_Msk        (0x1ul << CLK_APBCLK0_WWDTCKEN_Pos)         /*!< CLK_T::APBCLK0: WWDTCKEN Mask              */

#define CLK_APBCLK0_PWM0CKEN_Pos        (22)                                        /*!< CLK_T::APBCLK0: PWM0CKEN Position          */
#define CLK_APBCLK0_PWM0CKEN_Msk        (0x1ul << CLK_APBCLK0_PWM0CKEN_Pos)         /*!< CLK_T::APBCLK0: PWM0CKEN Mask              */

#define CLK_APBCLK0_BPWM0CKEN_Pos       (23)                                        /*!< CLK_T::APBCLK0: BPWM0CKEN Position         */
#define CLK_APBCLK0_BPWM0CKEN_Msk       (0x1ul << CLK_APBCLK0_BPWM0CKEN_Pos)        /*!< CLK_T::APBCLK0: BPWM0CKEN Mask             */

#define CLK_APBCLK0_LCDCKEN_Pos         (24)                                        /*!< CLK_T::APBCLK0: LCDCKEN Position           */
#define CLK_APBCLK0_LCDCKEN_Msk         (0x1ul << CLK_APBCLK0_LCDCKEN_Pos)          /*!< CLK_T::APBCLK0: LCDCKEN Mask               */

#define CLK_APBCLK0_LCDCCKEN_Pos        (25)                                        /*!< CLK_T::APBCLK0: LCDCCKEN Position          */
#define CLK_APBCLK0_LCDCCKEN_Msk        (0x1ul << CLK_APBCLK0_LCDCCKEN_Pos)         /*!< CLK_T::APBCLK0: LCDCCKEN Mask              */

#define CLK_APBCLK0_ADC0CKEN_Pos        (28)                                        /*!< CLK_T::APBCLK0: ADC0CKEN Position          */
#define CLK_APBCLK0_ADC0CKEN_Msk        (0x1ul << CLK_APBCLK0_ADC0CKEN_Pos)         /*!< CLK_T::APBCLK0: ADC0CKEN Mask              */

#define CLK_CLKSEL0_HCLKSEL_Pos         (0)                                         /*!< CLK_T::CLKSEL0: HCLKSEL Position           */
#define CLK_CLKSEL0_HCLKSEL_Msk         (0x3ul << CLK_CLKSEL0_HCLKSEL_Pos)          /*!< CLK_T::CLKSEL0: HCLKSEL Mask               */

#define CLK_CLKSEL1_WDTSEL_Pos          (0)                                         /*!< CLK_T::CLKSEL1: WDTSEL Position            */
#define CLK_CLKSEL1_WDTSEL_Msk          (0x1ul << CLK_CLKSEL1_WDTSEL_Pos)           /*!< CLK_T::CLKSEL1: WDTSEL Mask                */

#define CLK_CLKSEL1_WWDTSEL_Pos         (1)                                         /*!< CLK_T::CLKSEL1: WWDTSEL Position           */
#define CLK_CLKSEL1_WWDTSEL_Msk         (0x3ul << CLK_CLKSEL1_WWDTSEL_Pos)          /*!< CLK_T::CLKSEL1: WWDTSEL Mask               */

#define CLK_CLKSEL1_CLKOSEL_Pos         (4)                                         /*!< CLK_T::CLKSEL1: CLKOSEL Position           */
#define CLK_CLKSEL1_CLKOSEL_Msk         (0x7ul << CLK_CLKSEL1_CLKOSEL_Pos)          /*!< CLK_T::CLKSEL1: CLKOSEL Mask               */

#define CLK_CLKSEL1_TMR0SEL_Pos         (8)                                         /*!< CLK_T::CLKSEL1: TMR0SEL Position           */
#define CLK_CLKSEL1_TMR0SEL_Msk         (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)          /*!< CLK_T::CLKSEL1: TMR0SEL Mask               */

#define CLK_CLKSEL1_TMR1SEL_Pos         (12)                                        /*!< CLK_T::CLKSEL1: TMR1SEL Position           */
#define CLK_CLKSEL1_TMR1SEL_Msk         (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)          /*!< CLK_T::CLKSEL1: TMR1SEL Mask               */

#define CLK_CLKSEL1_TMR2SEL_Pos         (16)                                        /*!< CLK_T::CLKSEL1: TMR2SEL Position           */
#define CLK_CLKSEL1_TMR2SEL_Msk         (0x7ul << CLK_CLKSEL1_TMR2SEL_Pos)          /*!< CLK_T::CLKSEL1: TMR2SEL Mask               */

#define CLK_CLKSEL1_TMR3SEL_Pos         (20)                                        /*!< CLK_T::CLKSEL1: TMR3SEL Position           */
#define CLK_CLKSEL1_TMR3SEL_Msk         (0x7ul << CLK_CLKSEL1_TMR3SEL_Pos)          /*!< CLK_T::CLKSEL1: TMR3SEL Mask               */

#define CLK_CLKSEL1_ADC0SEL_Pos         (24)                                        /*!< CLK_T::CLKSEL1: ADC0SEL Position           */
#define CLK_CLKSEL1_ADC0SEL_Msk         (0x3ul << CLK_CLKSEL1_ADC0SEL_Pos)          /*!< CLK_T::CLKSEL1: ADC0SEL Mask               */

#define CLK_CLKSEL1_LCDSEL_Pos          (28)                                        /*!< CLK_T::CLKSEL1: LCDSEL Position            */
#define CLK_CLKSEL1_LCDSEL_Msk          (0x1ul << CLK_CLKSEL1_LCDSEL_Pos)           /*!< CLK_T::CLKSEL1: LCDSEL Mask                */

#define CLK_CLKSEL1_LCDCPSEL_Pos        (29)                                        /*!< CLK_T::CLKSEL1: LCDCPSEL Position          */
#define CLK_CLKSEL1_LCDCPSEL_Msk        (0x3ul << CLK_CLKSEL1_LCDCPSEL_Pos)         /*!< CLK_T::CLKSEL1: LCDCPSEL Mask              */

#define CLK_CLKSEL2_SPI0SEL_Pos         (0)                                         /*!< CLK_T::CLKSEL2: SPI0SEL Position           */
#define CLK_CLKSEL2_SPI0SEL_Msk         (0x3ul << CLK_CLKSEL2_SPI0SEL_Pos)          /*!< CLK_T::CLKSEL2: SPI0SEL Mask               */

#define CLK_CLKSEL2_SPI1SEL_Pos         (4)                                         /*!< CLK_T::CLKSEL2: SPI1SEL Position           */
#define CLK_CLKSEL2_SPI1SEL_Msk         (0x3ul << CLK_CLKSEL2_SPI1SEL_Pos)          /*!< CLK_T::CLKSEL2: SPI1SEL Mask               */

#define CLK_CLKSEL2_SPI2SEL_Pos         (8)                                         /*!< CLK_T::CLKSEL2: SPI2SEL Position           */
#define CLK_CLKSEL2_SPI2SEL_Msk         (0x3ul << CLK_CLKSEL2_SPI2SEL_Pos)          /*!< CLK_T::CLKSEL2: SPI2SEL Mask               */

#define CLK_CLKSEL2_UART0SEL_Pos        (16)                                        /*!< CLK_T::CLKSEL2: UART0SEL Position          */
#define CLK_CLKSEL2_UART0SEL_Msk        (0x7ul << CLK_CLKSEL2_UART0SEL_Pos)         /*!< CLK_T::CLKSEL2: UART0SEL Mask              */

#define CLK_CLKSEL2_UART1SEL_Pos        (20)                                        /*!< CLK_T::CLKSEL2: UART1SEL Position          */
#define CLK_CLKSEL2_UART1SEL_Msk        (0x7ul << CLK_CLKSEL2_UART1SEL_Pos)         /*!< CLK_T::CLKSEL2: UART1SEL Mask              */

#define CLK_CLKSEL2_UART2SEL_Pos        (24)                                        /*!< CLK_T::CLKSEL2: UART2SEL Position          */
#define CLK_CLKSEL2_UART2SEL_Msk        (0x7ul << CLK_CLKSEL2_UART2SEL_Pos)         /*!< CLK_T::CLKSEL2: UART2SEL Mask              */

#define CLK_HCLKDIV_HCLKDIV_Pos         (0)                                         /*!< CLK_T::HCLKDIV: HCLKDIV Position           */
#define CLK_HCLKDIV_HCLKDIV_Msk         (0xful << CLK_HCLKDIV_HCLKDIV_Pos)          /*!< CLK_T::HCLKDIV: HCLKDIV Mask               */

#define CLK_PCLKDIV_APB0DIV_Pos         (0)                                         /*!< CLK_T::PCLKDIV: APB0DIV Position           */
#define CLK_PCLKDIV_APB0DIV_Msk         (0x7ul << CLK_PCLKDIV_APB0DIV_Pos)          /*!< CLK_T::PCLKDIV: APB0DIV Mask               */

#define CLK_PCLKDIV_APB1DIV_Pos         (4)                                         /*!< CLK_T::PCLKDIV: APB1DIV Position           */
#define CLK_PCLKDIV_APB1DIV_Msk         (0x7ul << CLK_PCLKDIV_APB1DIV_Pos)          /*!< CLK_T::PCLKDIV: APB1DIV Mask               */

#define CLK_CLKDIV_UART0DIV_Pos         (0)                                         /*!< CLK_T::CLKDIV: UART0DIV Position           */
#define CLK_CLKDIV_UART0DIV_Msk         (0xful << CLK_CLKDIV_UART0DIV_Pos)          /*!< CLK_T::CLKDIV: UART0DIV Mask               */

#define CLK_CLKDIV_UART1DIV_Pos         (4)                                         /*!< CLK_T::CLKDIV: UART1DIV Position           */
#define CLK_CLKDIV_UART1DIV_Msk         (0xful << CLK_CLKDIV_UART1DIV_Pos)          /*!< CLK_T::CLKDIV: UART1DIV Mask               */

#define CLK_CLKDIV_UART2DIV_Pos         (8)                                         /*!< CLK_T::CLKDIV: UART2DIV Position           */
#define CLK_CLKDIV_UART2DIV_Msk         (0xful << CLK_CLKDIV_UART2DIV_Pos)          /*!< CLK_T::CLKDIV: UART2DIV Mask               */

#define CLK_CLKDIV_ADC0DIV_Pos          (16)                                        /*!< CLK_T::CLKDIV: ADC0DIV Position            */
#define CLK_CLKDIV_ADC0DIV_Msk          (0xfful << CLK_CLKDIV_ADC0DIV_Pos)          /*!< CLK_T::CLKDIV: ADC0DIV Mask                */

#define CLK_KEEP_TMR0KEEP_Pos           (2)                                         /*!< CLK_T::KEEP: TMR0KEEP Position             */
#define CLK_KEEP_TMR0KEEP_Msk           (0x1ul << CLK_KEEP_TMR0KEEP_Pos)            /*!< CLK_T::KEEP: TMR0KEEP Mask                 */

#define CLK_KEEP_TMR1KEEP_Pos           (3)                                         /*!< CLK_T::KEEP: TMR1KEEP Position             */
#define CLK_KEEP_TMR1KEEP_Msk           (0x1ul << CLK_KEEP_TMR1KEEP_Pos)            /*!< CLK_T::KEEP: TMR1KEEP Mask                 */

#define CLK_KEEP_TMR2KEEP_Pos           (4)                                         /*!< CLK_T::KEEP: TMR2KEEP Position             */
#define CLK_KEEP_TMR2KEEP_Msk           (0x1ul << CLK_KEEP_TMR2KEEP_Pos)            /*!< CLK_T::KEEP: TMR2KEEP Mask                 */

#define CLK_KEEP_TMR3KEEP_Pos           (5)                                         /*!< CLK_T::KEEP: TMR3KEEP Position             */
#define CLK_KEEP_TMR3KEEP_Msk           (0x1ul << CLK_KEEP_TMR3KEEP_Pos)            /*!< CLK_T::KEEP: TMR3KEEP Mask                 */

#define CLK_KEEP_I2C0KEEP_Pos           (8)                                         /*!< CLK_T::KEEP: I2C0KEEP Position             */
#define CLK_KEEP_I2C0KEEP_Msk           (0x1ul << CLK_KEEP_I2C0KEEP_Pos)            /*!< CLK_T::KEEP: I2C0KEEP Mask                 */

#define CLK_KEEP_I2C1KEEP_Pos           (9)                                         /*!< CLK_T::KEEP: I2C1KEEP Position             */
#define CLK_KEEP_I2C1KEEP_Msk           (0x1ul << CLK_KEEP_I2C1KEEP_Pos)            /*!< CLK_T::KEEP: I2C1KEEP Mask                 */

#define CLK_KEEP_I2C2KEEP_Pos           (10)                                        /*!< CLK_T::KEEP: I2C2KEEP Position             */
#define CLK_KEEP_I2C2KEEP_Msk           (0x1ul << CLK_KEEP_I2C2KEEP_Pos)            /*!< CLK_T::KEEP: I2C2KEEP Mask                 */

#define CLK_KEEP_SPI0KEEP_Pos           (12)                                        /*!< CLK_T::KEEP: SPI0KEEP Position             */
#define CLK_KEEP_SPI0KEEP_Msk           (0x1ul << CLK_KEEP_SPI0KEEP_Pos)            /*!< CLK_T::KEEP: SPI0KEEP Mask                 */

#define CLK_KEEP_SPI1KEEP_Pos           (13)                                        /*!< CLK_T::KEEP: SPI1KEEP Position             */
#define CLK_KEEP_SPI1KEEP_Msk           (0x1ul << CLK_KEEP_SPI1KEEP_Pos)            /*!< CLK_T::KEEP: SPI1KEEP Mask                 */

#define CLK_KEEP_SPI2KEEP_Pos           (14)                                        /*!< CLK_T::KEEP: SPI2KEEP Position             */
#define CLK_KEEP_SPI2KEEP_Msk           (0x1ul << CLK_KEEP_SPI2KEEP_Pos)            /*!< CLK_T::KEEP: SPI2KEEP Mask                 */

#define CLK_KEEP_UART0KEEP_Pos          (16)                                        /*!< CLK_T::KEEP: UART0KEEP Position            */
#define CLK_KEEP_UART0KEEP_Msk          (0x1ul << CLK_KEEP_UART0KEEP_Pos)           /*!< CLK_T::KEEP: UART0KEEP Mask                */

#define CLK_KEEP_UART1KEEP_Pos          (17)                                        /*!< CLK_T::KEEP: UART1KEEP Position            */
#define CLK_KEEP_UART1KEEP_Msk          (0x1ul << CLK_KEEP_UART1KEEP_Pos)           /*!< CLK_T::KEEP: UART1KEEP Mask                */

#define CLK_KEEP_UART2KEEP_Pos          (18)                                        /*!< CLK_T::KEEP: UART2KEEP Position            */
#define CLK_KEEP_UART2KEEP_Msk          (0x1ul << CLK_KEEP_UART2KEEP_Pos)           /*!< CLK_T::KEEP: UART2KEEP Mask                */

#define CLK_KEEP_ADC0KEEP_Pos           (28)                                        /*!< CLK_T::KEEP: ADC0KEEP Position             */
#define CLK_KEEP_ADC0KEEP_Msk           (0x1ul << CLK_KEEP_ADC0KEEP_Pos)            /*!< CLK_T::KEEP: ADC0KEEP Mask                 */

#define CLK_KEEP_PDMA0KEEP_Pos          (29)                                        /*!< CLK_T::KEEP: PDMA0KEEP Position            */
#define CLK_KEEP_PDMA0KEEP_Msk          (0x1ul << CLK_KEEP_PDMA0KEEP_Pos)           /*!< CLK_T::KEEP: PDMA0KEEP Mask                */

#define CLK_KEEP_GPIOKEEP_Pos           (30)                                        /*!< CLK_T::KEEP: GPIOKEEP Position             */
#define CLK_KEEP_GPIOKEEP_Msk           (0x1ul << CLK_KEEP_GPIOKEEP_Pos)            /*!< CLK_T::KEEP: GPIOKEEP Mask                 */

#define CLK_KEEP_SRAMKEEP_Pos           (31)                                        /*!< CLK_T::KEEP: SRAMKEEP Position             */
#define CLK_KEEP_SRAMKEEP_Msk           (0x1ul << CLK_KEEP_SRAMKEEP_Pos)            /*!< CLK_T::KEEP: SRAMKEEP Mask                 */

#define CLK_MCTL_CLKM0SEL_Pos           (0)                                         /*!< CLK_T::MCTL: CLKM0SEL Position             */
#define CLK_MCTL_CLKM0SEL_Msk           (0x3ul << CLK_MCTL_CLKM0SEL_Pos)            /*!< CLK_T::MCTL: CLKM0SEL Mask                 */

#define CLK_MCTL_CLKM1SEL_Pos           (8)                                         /*!< CLK_T::MCTL: CLKM1SEL Position             */
#define CLK_MCTL_CLKM1SEL_Msk           (0x3ul << CLK_MCTL_CLKM1SEL_Pos)            /*!< CLK_T::MCTL: CLKM1SEL Mask                 */

#define CLK_MCTL_AOCMEN1_Pos            (24)                                        /*!< CLK_T::MCTL: AOCMEN1 Position              */
#define CLK_MCTL_AOCMEN1_Msk            (0x1ul << CLK_MCTL_AOCMEN1_Pos)             /*!< CLK_T::MCTL: AOCMEN1 Mask                  */

#define CLK_MCTL_AOCMEN2_Pos            (25)                                        /*!< CLK_T::MCTL: AOCMEN2 Position              */
#define CLK_MCTL_AOCMEN2_Msk            (0x1ul << CLK_MCTL_AOCMEN2_Pos)             /*!< CLK_T::MCTL: AOCMEN2 Mask                  */

#define CLK_STATUS_LXTSTB_Pos           (1)                                         /*!< CLK_T::STATUS: LXTSTB Position             */
#define CLK_STATUS_LXTSTB_Msk           (0x1ul << CLK_STATUS_LXTSTB_Pos)            /*!< CLK_T::STATUS: LXTSTB Mask                 */

#define CLK_STATUS_LIRCSTB_Pos          (3)                                         /*!< CLK_T::STATUS: LIRCSTB Position            */
#define CLK_STATUS_LIRCSTB_Msk          (0x1ul << CLK_STATUS_LIRCSTB_Pos)           /*!< CLK_T::STATUS: LIRCSTB Mask                */

#define CLK_STATUS_HIRCSTB_Pos          (4)                                         /*!< CLK_T::STATUS: HIRCSTB Position            */
#define CLK_STATUS_HIRCSTB_Msk          (0x1ul << CLK_STATUS_HIRCSTB_Pos)           /*!< CLK_T::STATUS: HIRCSTB Mask                */

#define CLK_STATUS_MIRCSTB_Pos          (5)                                         /*!< CLK_T::STATUS: MIRCSTB Position            */
#define CLK_STATUS_MIRCSTB_Msk          (0x1ul << CLK_STATUS_MIRCSTB_Pos)           /*!< CLK_T::STATUS: MIRCSTB Mask                */

#define CLK_STATUS_CLKSFAIL_Pos         (7)                                         /*!< CLK_T::STATUS: CLKSFAIL Position           */
#define CLK_STATUS_CLKSFAIL_Msk         (0x1ul << CLK_STATUS_CLKSFAIL_Pos)          /*!< CLK_T::STATUS: CLKSFAIL Mask               */

#define CLK_CLKOCTL_FREQSEL_Pos         (0)                                         /*!< CLK_T::CLKOCTL: FREQSEL Position           */
#define CLK_CLKOCTL_FREQSEL_Msk         (0xful << CLK_CLKOCTL_FREQSEL_Pos)          /*!< CLK_T::CLKOCTL: FREQSEL Mask               */

#define CLK_CLKOCTL_CLKOEN_Pos          (4)                                         /*!< CLK_T::CLKOCTL: CLKOEN Position            */
#define CLK_CLKOCTL_CLKOEN_Msk          (0x1ul << CLK_CLKOCTL_CLKOEN_Pos)           /*!< CLK_T::CLKOCTL: CLKOEN Mask                */

#define CLK_CLKOCTL_DIV1EN_Pos          (5)                                         /*!< CLK_T::CLKOCTL: DIV1EN Position            */
#define CLK_CLKOCTL_DIV1EN_Msk          (0x1ul << CLK_CLKOCTL_DIV1EN_Pos)           /*!< CLK_T::CLKOCTL: DIV1EN Mask                */

#define CLK_CLKOCTL_CLK1HZEN_Pos        (6)                                         /*!< CLK_T::CLKOCTL: CLK1HZEN Position          */
#define CLK_CLKOCTL_CLK1HZEN_Msk        (0x1ul << CLK_CLKOCTL_CLK1HZEN_Pos)         /*!< CLK_T::CLKOCTL: CLK1HZEN Mask              */

#define CLK_CLKDCTL_LXTFDEN_Pos         (0)                                         /*!< CLK_T::CLKDCTL: LXTFDEN Position           */
#define CLK_CLKDCTL_LXTFDEN_Msk         (0x1ul << CLK_CLKDCTL_LXTFDEN_Pos)          /*!< CLK_T::CLKDCTL: LXTFDEN Mask               */

#define CLK_CLKDCTL_LXTFIEN_Pos         (1)                                         /*!< CLK_T::CLKDCTL: LXTFIEN Position           */
#define CLK_CLKDCTL_LXTFIEN_Msk         (0x1ul << CLK_CLKDCTL_LXTFIEN_Pos)          /*!< CLK_T::CLKDCTL: LXTFIEN Mask               */

#define CLK_CLKDSTS_LXTFIF_Pos          (0)                                         /*!< CLK_T::CLKDSTS: LXTFIF Position            */
#define CLK_CLKDSTS_LXTFIF_Msk          (0x1ul << CLK_CLKDSTS_LXTFIF_Pos)           /*!< CLK_T::CLKDSTS: LXTFIF Mask                */

#define CLK_PMUCTL_PDMSEL_Pos           (0)                                         /*!< CLK_T::PMUCTL: PDMSEL Position             */
#define CLK_PMUCTL_PDMSEL_Msk           (0x7ul << CLK_PMUCTL_PDMSEL_Pos)            /*!< CLK_T::PMUCTL: PDMSEL Mask                 */

#define CLK_PMUCTL_SRETSEL_Pos          (8)                                         /*!< CLK_T::PMUCTL: SRETSEL Position            */
#define CLK_PMUCTL_SRETSEL_Msk          (0x1ul << CLK_PMUCTL_SRETSEL_Pos)           /*!< CLK_T::PMUCTL: SRETSEL Mask                */

#define CLK_PMUCTL_NRLDCSEL_Pos         (16)                                        /*!< CLK_T::PMUCTL: NRLDCSEL Position           */
#define CLK_PMUCTL_NRLDCSEL_Msk         (0x1ul << CLK_PMUCTL_NRLDCSEL_Pos)          /*!< CLK_T::PMUCTL: NRLDCSEL Mask               */

#define CLK_PMUCTL_NRBGLPEL_Pos         (18)                                        /*!< CLK_T::PMUCTL: NRBGLPEL Position           */
#define CLK_PMUCTL_NRBGLPEL_Msk         (0x1ul << CLK_PMUCTL_NRBGLPEL_Pos)          /*!< CLK_T::PMUCTL: NRBGLPEL Mask               */

#define CLK_PMUCTL_PDLDCSEL_Pos         (24)                                        /*!< CLK_T::PMUCTL: PDLDCSEL Position           */
#define CLK_PMUCTL_PDLDCSEL_Msk         (0x3ul << CLK_PMUCTL_PDLDCSEL_Pos)          /*!< CLK_T::PMUCTL: PDLDCSEL Mask               */

#define CLK_PMUCTL_PDBGLPEL_Pos         (26)                                        /*!< CLK_T::PMUCTL: PDBGLPEL Position           */
#define CLK_PMUCTL_PDBGLPEL_Msk         (0x1ul << CLK_PMUCTL_PDBGLPEL_Pos)          /*!< CLK_T::PMUCTL: PDBGLPEL Mask               */

#define CLK_PMUCTL_PDLVRSEL_Pos         (27)                                        /*!< CLK_T::PMUCTL: PDLVRSEL Position           */
#define CLK_PMUCTL_PDLVRSEL_Msk         (0x1ul << CLK_PMUCTL_PDLVRSEL_Pos)          /*!< CLK_T::PMUCTL: PDLVRSEL Mask               */

#define CLK_PMUCTL_PDFSHSEL_Pos         (28)                                        /*!< CLK_T::PMUCTL: PDFSHSEL Position           */
#define CLK_PMUCTL_PDFSHSEL_Msk         (0x1ul << CLK_PMUCTL_PDFSHSEL_Pos)          /*!< CLK_T::PMUCTL: PDFSHSEL Mask               */

#define CLK_PMUCTL_WKFSHSEL_Pos         (29)                                        /*!< CLK_T::PMUCTL: WKFSHSEL Position           */
#define CLK_PMUCTL_WKFSHSEL_Msk         (0x1ul << CLK_PMUCTL_WKFSHSEL_Pos)          /*!< CLK_T::PMUCTL: WKFSHSEL Mask               */

#define CLK_PMUCTL_PDWKMIRCS_Pos        (30)                                        /*!< CLK_T::PMUCTL: PDWKMIRCS Position          */
#define CLK_PMUCTL_PDWKMIRCS_Msk        (0x3ul << CLK_PMUCTL_PDWKMIRCS_Pos)         /*!< CLK_T::PMUCTL: PDWKMIRCS Mask              */

#define CLK_PMUSTS_WKPIN0_Pos           (0)                                         /*!< CLK_T::PMUSTS: WKPIN0 Position             */
#define CLK_PMUSTS_WKPIN0_Msk           (0x1ul << CLK_PMUSTS_WKPIN0_Pos)            /*!< CLK_T::PMUSTS: WKPIN0 Mask                 */

#define CLK_PMUSTS_TMRWK_Pos            (1)                                         /*!< CLK_T::PMUSTS: TMRWK Position              */
#define CLK_PMUSTS_TMRWK_Msk            (0x1ul << CLK_PMUSTS_TMRWK_Pos)             /*!< CLK_T::PMUSTS: TMRWK Mask                  */

#define CLK_PMUSTS_RTCWK_Pos            (2)                                         /*!< CLK_T::PMUSTS: RTCWK Position              */
#define CLK_PMUSTS_RTCWK_Msk            (0x1ul << CLK_PMUSTS_RTCWK_Pos)             /*!< CLK_T::PMUSTS: RTCWK Mask                  */

#define CLK_PMUSTS_WKPIN1_Pos           (3)                                         /*!< CLK_T::PMUSTS: WKPIN1 Position             */
#define CLK_PMUSTS_WKPIN1_Msk           (0x1ul << CLK_PMUSTS_WKPIN1_Pos)            /*!< CLK_T::PMUSTS: WKPIN1 Mask                 */

#define CLK_PMUSTS_WKPIN2_Pos           (4)                                         /*!< CLK_T::PMUSTS: WKPIN2 Position             */
#define CLK_PMUSTS_WKPIN2_Msk           (0x1ul << CLK_PMUSTS_WKPIN2_Pos)            /*!< CLK_T::PMUSTS: WKPIN2 Mask                 */

#define CLK_PMUSTS_WKPIN3_Pos           (5)                                         /*!< CLK_T::PMUSTS: WKPIN3 Position             */
#define CLK_PMUSTS_WKPIN3_Msk           (0x1ul << CLK_PMUSTS_WKPIN3_Pos)            /*!< CLK_T::PMUSTS: WKPIN3 Mask                 */

#define CLK_PMUSTS_WKPIN4_Pos           (6)                                         /*!< CLK_T::PMUSTS: WKPIN4 Position             */
#define CLK_PMUSTS_WKPIN4_Msk           (0x1ul << CLK_PMUSTS_WKPIN4_Pos)            /*!< CLK_T::PMUSTS: WKPIN4 Mask                 */

#define CLK_PMUSTS_GPAWK0_Pos           (8)                                         /*!< CLK_T::PMUSTS: GPAWK0 Position             */
#define CLK_PMUSTS_GPAWK0_Msk           (0x1ul << CLK_PMUSTS_GPAWK0_Pos)            /*!< CLK_T::PMUSTS: GPAWK0 Mask                 */

#define CLK_PMUSTS_GPBWK0_Pos           (9)                                         /*!< CLK_T::PMUSTS: GPBWK0 Position             */
#define CLK_PMUSTS_GPBWK0_Msk           (0x1ul << CLK_PMUSTS_GPBWK0_Pos)            /*!< CLK_T::PMUSTS: GPBWK0 Mask                 */

#define CLK_PMUSTS_GPCWK0_Pos           (10)                                        /*!< CLK_T::PMUSTS: GPCWK0 Position             */
#define CLK_PMUSTS_GPCWK0_Msk           (0x1ul << CLK_PMUSTS_GPCWK0_Pos)            /*!< CLK_T::PMUSTS: GPCWK0 Mask                 */

#define CLK_PMUSTS_GPDWK0_Pos           (11)                                        /*!< CLK_T::PMUSTS: GPDWK0 Position             */
#define CLK_PMUSTS_GPDWK0_Msk           (0x1ul << CLK_PMUSTS_GPDWK0_Pos)            /*!< CLK_T::PMUSTS: GPDWK0 Mask                 */

#define CLK_PMUSTS_LVRWK_Pos            (12)                                        /*!< CLK_T::PMUSTS: LVRWK Position              */
#define CLK_PMUSTS_LVRWK_Msk            (0x1ul << CLK_PMUSTS_LVRWK_Pos)             /*!< CLK_T::PMUSTS: LVRWK Mask                  */

#define CLK_PMUSTS_BODWK_Pos            (13)                                        /*!< CLK_T::PMUSTS: BODWK Position              */
#define CLK_PMUSTS_BODWK_Msk            (0x1ul << CLK_PMUSTS_BODWK_Pos)             /*!< CLK_T::PMUSTS: BODWK Mask                  */

#define CLK_PMUSTS_RSTWK_Pos            (15)                                        /*!< CLK_T::PMUSTS: RSTWK Position              */
#define CLK_PMUSTS_RSTWK_Msk            (0x1ul << CLK_PMUSTS_RSTWK_Pos)             /*!< CLK_T::PMUSTS: RSTWK Mask                  */

#define CLK_PMUSTS_GPAWK1_Pos           (24)                                        /*!< CLK_T::PMUSTS: GPAWK1 Position             */
#define CLK_PMUSTS_GPAWK1_Msk           (0x1ul << CLK_PMUSTS_GPAWK1_Pos)            /*!< CLK_T::PMUSTS: GPAWK1 Mask                 */

#define CLK_PMUSTS_GPBWK1_Pos           (25)                                        /*!< CLK_T::PMUSTS: GPBWK1 Position             */
#define CLK_PMUSTS_GPBWK1_Msk           (0x1ul << CLK_PMUSTS_GPBWK1_Pos)            /*!< CLK_T::PMUSTS: GPBWK1 Mask                 */

#define CLK_PMUSTS_GPCWK1_Pos           (26)                                        /*!< CLK_T::PMUSTS: GPCWK1 Position             */
#define CLK_PMUSTS_GPCWK1_Msk           (0x1ul << CLK_PMUSTS_GPCWK1_Pos)            /*!< CLK_T::PMUSTS: GPCWK1 Mask                 */

#define CLK_PMUSTS_GPDWK1_Pos           (27)                                        /*!< CLK_T::PMUSTS: GPDWK1 Position             */
#define CLK_PMUSTS_GPDWK1_Msk           (0x1ul << CLK_PMUSTS_GPDWK1_Pos)            /*!< CLK_T::PMUSTS: GPDWK1 Mask                 */

#define CLK_PMUSTS_CLRWK_Pos            (31)                                        /*!< CLK_T::PMUSTS: CLRWK Position              */
#define CLK_PMUSTS_CLRWK_Msk            (0x1ul << CLK_PMUSTS_CLRWK_Pos)             /*!< CLK_T::PMUSTS: CLRWK Mask                  */

#define CLK_PMUWKCTL_WKTMREN_Pos        (0)                                         /*!< CLK_T::PMUWKCTL: WKTMREN Position          */
#define CLK_PMUWKCTL_WKTMREN_Msk        (0x1ul << CLK_PMUWKCTL_WKTMREN_Pos)         /*!< CLK_T::PMUWKCTL: WKTMREN Mask              */

#define CLK_PMUWKCTL_WKTMRMOD_Pos       (1)                                         /*!< CLK_T::PMUWKCTL: WKTMRMOD Position         */
#define CLK_PMUWKCTL_WKTMRMOD_Msk       (0x1ul << CLK_PMUWKCTL_WKTMRMOD_Pos)        /*!< CLK_T::PMUWKCTL: WKTMRMOD Mask             */

#define CLK_PMUWKCTL_RTCWKEN_Pos        (7)                                         /*!< CLK_T::PMUWKCTL: RTCWKEN Position          */
#define CLK_PMUWKCTL_RTCWKEN_Msk        (0x1ul << CLK_PMUWKCTL_RTCWKEN_Pos)         /*!< CLK_T::PMUWKCTL: RTCWKEN Mask              */

#define CLK_PMUWKCTL_WKTMRIS_Pos        (8)                                         /*!< CLK_T::PMUWKCTL: WKTMRIS Position          */
#define CLK_PMUWKCTL_WKTMRIS_Msk        (0xful << CLK_PMUWKCTL_WKTMRIS_Pos)         /*!< CLK_T::PMUWKCTL: WKTMRIS Mask              */

#define CLK_PMUWKCTL_WKPINEN0_Pos       (16)                                        /*!< CLK_T::PMUWKCTL: WKPINEN0 Position         */
#define CLK_PMUWKCTL_WKPINEN0_Msk       (0x3ul << CLK_PMUWKCTL_WKPINEN0_Pos)        /*!< CLK_T::PMUWKCTL: WKPINEN0 Mask             */

#define CLK_PMUWKCTL_WKPINEN1_Pos       (18)                                        /*!< CLK_T::PMUWKCTL: WKPINEN1 Position         */
#define CLK_PMUWKCTL_WKPINEN1_Msk       (0x3ul << CLK_PMUWKCTL_WKPINEN1_Pos)        /*!< CLK_T::PMUWKCTL: WKPINEN1 Mask             */

#define CLK_PMUWKCTL_WKPINEN2_Pos       (20)                                        /*!< CLK_T::PMUWKCTL: WKPINEN2 Position         */
#define CLK_PMUWKCTL_WKPINEN2_Msk       (0x3ul << CLK_PMUWKCTL_WKPINEN2_Pos)        /*!< CLK_T::PMUWKCTL: WKPINEN2 Mask             */

#define CLK_PMUWKCTL_WKPINEN3_Pos       (22)                                        /*!< CLK_T::PMUWKCTL: WKPINEN3 Position         */
#define CLK_PMUWKCTL_WKPINEN3_Msk       (0x3ul << CLK_PMUWKCTL_WKPINEN3_Pos)        /*!< CLK_T::PMUWKCTL: WKPINEN3 Mask             */

#define CLK_PMUWKCTL_WKPINEN4_Pos       (24)                                        /*!< CLK_T::PMUWKCTL: WKPINEN4 Position         */
#define CLK_PMUWKCTL_WKPINEN4_Msk       (0x3ul << CLK_PMUWKCTL_WKPINEN4_Pos)        /*!< CLK_T::PMUWKCTL: WKPINEN4 Mask             */

#define CLK_PMUWKCTL_DISAUTOC_Pos       (31)                                        /*!< CLK_T::PMUWKCTL: DISAUTOC Position         */
#define CLK_PMUWKCTL_DISAUTOC_Msk       (0x1ul << CLK_PMUWKCTL_DISAUTOC_Pos)        /*!< CLK_T::PMUWKCTL: DISAUTOC Mask             */

#define CLK_PWDBCTL_SWKDBCLKSEL_Pos     (0)                                         /*!< CLK_T::PWDBCTL: SWKDBCLKSEL Position       */
#define CLK_PWDBCTL_SWKDBCLKSEL_Msk     (0xful << CLK_PWDBCTL_SWKDBCLKSEL_Pos)      /*!< CLK_T::PWDBCTL: SWKDBCLKSEL Mask           */

#define CLK_PAPWCTL_WKEN0_Pos           (0)                                         /*!< CLK_T::PAPWCTL: WKEN0 Position             */
#define CLK_PAPWCTL_WKEN0_Msk           (0x1ul << CLK_PAPWCTL_WKEN0_Pos)            /*!< CLK_T::PAPWCTL: WKEN0 Mask                 */

#define CLK_PAPWCTL_PRWKEN0_Pos         (1)                                         /*!< CLK_T::PAPWCTL: PRWKEN0 Position           */
#define CLK_PAPWCTL_PRWKEN0_Msk         (0x1ul << CLK_PAPWCTL_PRWKEN0_Pos)          /*!< CLK_T::PAPWCTL: PRWKEN0 Mask               */

#define CLK_PAPWCTL_PFWKEN0_Pos         (2)                                         /*!< CLK_T::PAPWCTL: PFWKEN0 Position           */
#define CLK_PAPWCTL_PFWKEN0_Msk         (0x1ul << CLK_PAPWCTL_PFWKEN0_Pos)          /*!< CLK_T::PAPWCTL: PFWKEN0 Mask               */

#define CLK_PAPWCTL_WKPSEL0_Pos         (4)                                         /*!< CLK_T::PAPWCTL: WKPSEL0 Position           */
#define CLK_PAPWCTL_WKPSEL0_Msk         (0xful << CLK_PAPWCTL_WKPSEL0_Pos)          /*!< CLK_T::PAPWCTL: WKPSEL0 Mask               */

#define CLK_PAPWCTL_DBEN0_Pos           (8)                                         /*!< CLK_T::PAPWCTL: DBEN0 Position             */
#define CLK_PAPWCTL_DBEN0_Msk           (0x1ul << CLK_PAPWCTL_DBEN0_Pos)            /*!< CLK_T::PAPWCTL: DBEN0 Mask                 */

#define CLK_PAPWCTL_TRIGM0_Pos          (10)                                        /*!< CLK_T::PAPWCTL: TRIGM0 Position            */
#define CLK_PAPWCTL_TRIGM0_Msk          (0x1ul << CLK_PAPWCTL_TRIGM0_Pos)           /*!< CLK_T::PAPWCTL: TRIGM0 Mask                */

#define CLK_PAPWCTL_NMR0_Pos            (11)                                        /*!< CLK_T::PAPWCTL: NMR0 Position              */
#define CLK_PAPWCTL_NMR0_Msk            (0x1ul << CLK_PAPWCTL_NMR0_Pos)             /*!< CLK_T::PAPWCTL: NMR0 Mask                  */

#define CLK_PAPWCTL_WKEN1_Pos           (16)                                        /*!< CLK_T::PAPWCTL: WKEN1 Position             */
#define CLK_PAPWCTL_WKEN1_Msk           (0x1ul << CLK_PAPWCTL_WKEN1_Pos)            /*!< CLK_T::PAPWCTL: WKEN1 Mask                 */

#define CLK_PAPWCTL_PRWKEN1_Pos         (17)                                        /*!< CLK_T::PAPWCTL: PRWKEN1 Position           */
#define CLK_PAPWCTL_PRWKEN1_Msk         (0x1ul << CLK_PAPWCTL_PRWKEN1_Pos)          /*!< CLK_T::PAPWCTL: PRWKEN1 Mask               */

#define CLK_PAPWCTL_PFWKEN1_Pos         (18)                                        /*!< CLK_T::PAPWCTL: PFWKEN1 Position           */
#define CLK_PAPWCTL_PFWKEN1_Msk         (0x1ul << CLK_PAPWCTL_PFWKEN1_Pos)          /*!< CLK_T::PAPWCTL: PFWKEN1 Mask               */

#define CLK_PAPWCTL_WKPSEL1_Pos         (20)                                        /*!< CLK_T::PAPWCTL: WKPSEL1 Position           */
#define CLK_PAPWCTL_WKPSEL1_Msk         (0xful << CLK_PAPWCTL_WKPSEL1_Pos)          /*!< CLK_T::PAPWCTL: WKPSEL1 Mask               */

#define CLK_PAPWCTL_DBEN1_Pos           (24)                                        /*!< CLK_T::PAPWCTL: DBEN1 Position             */
#define CLK_PAPWCTL_DBEN1_Msk           (0x1ul << CLK_PAPWCTL_DBEN1_Pos)            /*!< CLK_T::PAPWCTL: DBEN1 Mask                 */

#define CLK_PAPWCTL_TRIGM1_Pos          (26)                                        /*!< CLK_T::PAPWCTL: TRIGM1 Position            */
#define CLK_PAPWCTL_TRIGM1_Msk          (0x1ul << CLK_PAPWCTL_TRIGM1_Pos)           /*!< CLK_T::PAPWCTL: TRIGM1 Mask                */

#define CLK_PAPWCTL_NMR1_Pos            (27)                                        /*!< CLK_T::PAPWCTL: NMR1 Position              */
#define CLK_PAPWCTL_NMR1_Msk            (0x1ul << CLK_PAPWCTL_NMR1_Pos)             /*!< CLK_T::PAPWCTL: NMR1 Mask                  */

#define CLK_PBPWCTL_WKEN0_Pos           (0)                                         /*!< CLK_T::PBPWCTL: WKEN0 Position             */
#define CLK_PBPWCTL_WKEN0_Msk           (0x1ul << CLK_PBPWCTL_WKEN0_Pos)            /*!< CLK_T::PBPWCTL: WKEN0 Mask                 */

#define CLK_PBPWCTL_PRWKEN0_Pos         (1)                                         /*!< CLK_T::PBPWCTL: PRWKEN0 Position           */
#define CLK_PBPWCTL_PRWKEN0_Msk         (0x1ul << CLK_PBPWCTL_PRWKEN0_Pos)          /*!< CLK_T::PBPWCTL: PRWKEN0 Mask               */

#define CLK_PBPWCTL_PFWKEN0_Pos         (2)                                         /*!< CLK_T::PBPWCTL: PFWKEN0 Position           */
#define CLK_PBPWCTL_PFWKEN0_Msk         (0x1ul << CLK_PBPWCTL_PFWKEN0_Pos)          /*!< CLK_T::PBPWCTL: PFWKEN0 Mask               */

#define CLK_PBPWCTL_WKPSEL0_Pos         (4)                                         /*!< CLK_T::PBPWCTL: WKPSEL0 Position           */
#define CLK_PBPWCTL_WKPSEL0_Msk         (0xful << CLK_PBPWCTL_WKPSEL0_Pos)          /*!< CLK_T::PBPWCTL: WKPSEL0 Mask               */

#define CLK_PBPWCTL_DBEN0_Pos           (8)                                         /*!< CLK_T::PBPWCTL: DBEN0 Position             */
#define CLK_PBPWCTL_DBEN0_Msk           (0x1ul << CLK_PBPWCTL_DBEN0_Pos)            /*!< CLK_T::PBPWCTL: DBEN0 Mask                 */

#define CLK_PBPWCTL_TRIGM0_Pos          (10)                                        /*!< CLK_T::PBPWCTL: TRIGM0 Position            */
#define CLK_PBPWCTL_TRIGM0_Msk          (0x1ul << CLK_PBPWCTL_TRIGM0_Pos)           /*!< CLK_T::PBPWCTL: TRIGM0 Mask                */

#define CLK_PBPWCTL_NMR0_Pos            (11)                                        /*!< CLK_T::PBPWCTL: NMR0 Position              */
#define CLK_PBPWCTL_NMR0_Msk            (0x1ul << CLK_PBPWCTL_NMR0_Pos)             /*!< CLK_T::PBPWCTL: NMR0 Mask                  */

#define CLK_PBPWCTL_WKEN1_Pos           (16)                                        /*!< CLK_T::PBPWCTL: WKEN1 Position             */
#define CLK_PBPWCTL_WKEN1_Msk           (0x1ul << CLK_PBPWCTL_WKEN1_Pos)            /*!< CLK_T::PBPWCTL: WKEN1 Mask                 */

#define CLK_PBPWCTL_PRWKEN1_Pos         (17)                                        /*!< CLK_T::PBPWCTL: PRWKEN1 Position           */
#define CLK_PBPWCTL_PRWKEN1_Msk         (0x1ul << CLK_PBPWCTL_PRWKEN1_Pos)          /*!< CLK_T::PBPWCTL: PRWKEN1 Mask               */

#define CLK_PBPWCTL_PFWKEN1_Pos         (18)                                        /*!< CLK_T::PBPWCTL: PFWKEN1 Position           */
#define CLK_PBPWCTL_PFWKEN1_Msk         (0x1ul << CLK_PBPWCTL_PFWKEN1_Pos)          /*!< CLK_T::PBPWCTL: PFWKEN1 Mask               */

#define CLK_PBPWCTL_WKPSEL1_Pos         (20)                                        /*!< CLK_T::PBPWCTL: WKPSEL1 Position           */
#define CLK_PBPWCTL_WKPSEL1_Msk         (0xful << CLK_PBPWCTL_WKPSEL1_Pos)          /*!< CLK_T::PBPWCTL: WKPSEL1 Mask               */

#define CLK_PBPWCTL_DBEN1_Pos           (24)                                        /*!< CLK_T::PBPWCTL: DBEN1 Position             */
#define CLK_PBPWCTL_DBEN1_Msk           (0x1ul << CLK_PBPWCTL_DBEN1_Pos)            /*!< CLK_T::PBPWCTL: DBEN1 Mask                 */

#define CLK_PBPWCTL_TRIGM1_Pos          (26)                                        /*!< CLK_T::PBPWCTL: TRIGM1 Position            */
#define CLK_PBPWCTL_TRIGM1_Msk          (0x1ul << CLK_PBPWCTL_TRIGM1_Pos)           /*!< CLK_T::PBPWCTL: TRIGM1 Mask                */

#define CLK_PBPWCTL_NMR1_Pos            (27)                                        /*!< CLK_T::PBPWCTL: NMR1 Position              */
#define CLK_PBPWCTL_NMR1_Msk            (0x1ul << CLK_PBPWCTL_NMR1_Pos)             /*!< CLK_T::PBPWCTL: NMR1 Mask                  */

#define CLK_PCPWCTL_WKEN0_Pos           (0)                                         /*!< CLK_T::PCPWCTL: WKEN0 Position             */
#define CLK_PCPWCTL_WKEN0_Msk           (0x1ul << CLK_PCPWCTL_WKEN0_Pos)            /*!< CLK_T::PCPWCTL: WKEN0 Mask                 */

#define CLK_PCPWCTL_PRWKEN0_Pos         (1)                                         /*!< CLK_T::PCPWCTL: PRWKEN0 Position           */
#define CLK_PCPWCTL_PRWKEN0_Msk         (0x1ul << CLK_PCPWCTL_PRWKEN0_Pos)          /*!< CLK_T::PCPWCTL: PRWKEN0 Mask               */

#define CLK_PCPWCTL_PFWKEN0_Pos         (2)                                         /*!< CLK_T::PCPWCTL: PFWKEN0 Position           */
#define CLK_PCPWCTL_PFWKEN0_Msk         (0x1ul << CLK_PCPWCTL_PFWKEN0_Pos)          /*!< CLK_T::PCPWCTL: PFWKEN0 Mask               */

#define CLK_PCPWCTL_WKPSEL0_Pos         (4)                                         /*!< CLK_T::PCPWCTL: WKPSEL0 Position           */
#define CLK_PCPWCTL_WKPSEL0_Msk         (0xful << CLK_PCPWCTL_WKPSEL0_Pos)          /*!< CLK_T::PCPWCTL: WKPSEL0 Mask               */

#define CLK_PCPWCTL_DBEN0_Pos           (8)                                         /*!< CLK_T::PCPWCTL: DBEN0 Position             */
#define CLK_PCPWCTL_DBEN0_Msk           (0x1ul << CLK_PCPWCTL_DBEN0_Pos)            /*!< CLK_T::PCPWCTL: DBEN0 Mask                 */

#define CLK_PCPWCTL_TRIGM0_Pos          (10)                                        /*!< CLK_T::PCPWCTL: TRIGM0 Position            */
#define CLK_PCPWCTL_TRIGM0_Msk          (0x1ul << CLK_PCPWCTL_TRIGM0_Pos)           /*!< CLK_T::PCPWCTL: TRIGM0 Mask                */

#define CLK_PCPWCTL_NMR0_Pos            (11)                                        /*!< CLK_T::PCPWCTL: NMR0 Position              */
#define CLK_PCPWCTL_NMR0_Msk            (0x1ul << CLK_PCPWCTL_NMR0_Pos)             /*!< CLK_T::PCPWCTL: NMR0 Mask                  */

#define CLK_PCPWCTL_WKEN1_Pos           (16)                                        /*!< CLK_T::PCPWCTL: WKEN1 Position             */
#define CLK_PCPWCTL_WKEN1_Msk           (0x1ul << CLK_PCPWCTL_WKEN1_Pos)            /*!< CLK_T::PCPWCTL: WKEN1 Mask                 */

#define CLK_PCPWCTL_PRWKEN1_Pos         (17)                                        /*!< CLK_T::PCPWCTL: PRWKEN1 Position           */
#define CLK_PCPWCTL_PRWKEN1_Msk         (0x1ul << CLK_PCPWCTL_PRWKEN1_Pos)          /*!< CLK_T::PCPWCTL: PRWKEN1 Mask               */

#define CLK_PCPWCTL_PFWKEN1_Pos         (18)                                        /*!< CLK_T::PCPWCTL: PFWKEN1 Position           */
#define CLK_PCPWCTL_PFWKEN1_Msk         (0x1ul << CLK_PCPWCTL_PFWKEN1_Pos)          /*!< CLK_T::PCPWCTL: PFWKEN1 Mask               */

#define CLK_PCPWCTL_WKPSEL1_Pos         (20)                                        /*!< CLK_T::PCPWCTL: WKPSEL1 Position           */
#define CLK_PCPWCTL_WKPSEL1_Msk         (0xful << CLK_PCPWCTL_WKPSEL1_Pos)          /*!< CLK_T::PCPWCTL: WKPSEL1 Mask               */

#define CLK_PCPWCTL_DBEN1_Pos           (24)                                        /*!< CLK_T::PCPWCTL: DBEN1 Position             */
#define CLK_PCPWCTL_DBEN1_Msk           (0x1ul << CLK_PCPWCTL_DBEN1_Pos)            /*!< CLK_T::PCPWCTL: DBEN1 Mask                 */

#define CLK_PCPWCTL_TRIGM1_Pos          (26)                                        /*!< CLK_T::PCPWCTL: TRIGM1 Position            */
#define CLK_PCPWCTL_TRIGM1_Msk          (0x1ul << CLK_PCPWCTL_TRIGM1_Pos)           /*!< CLK_T::PCPWCTL: TRIGM1 Mask                */

#define CLK_PCPWCTL_NMR1_Pos            (27)                                        /*!< CLK_T::PCPWCTL: NMR1 Position              */
#define CLK_PCPWCTL_NMR1_Msk            (0x1ul << CLK_PCPWCTL_NMR1_Pos)             /*!< CLK_T::PCPWCTL: NMR1 Mask                  */

#define CLK_PDPWCTL_WKEN0_Pos           (0)                                         /*!< CLK_T::PDPWCTL: WKEN0 Position             */
#define CLK_PDPWCTL_WKEN0_Msk           (0x1ul << CLK_PDPWCTL_WKEN0_Pos)            /*!< CLK_T::PDPWCTL: WKEN0 Mask                 */

#define CLK_PDPWCTL_PRWKEN0_Pos         (1)                                         /*!< CLK_T::PDPWCTL: PRWKEN0 Position           */
#define CLK_PDPWCTL_PRWKEN0_Msk         (0x1ul << CLK_PDPWCTL_PRWKEN0_Pos)          /*!< CLK_T::PDPWCTL: PRWKEN0 Mask               */

#define CLK_PDPWCTL_PFWKEN0_Pos         (2)                                         /*!< CLK_T::PDPWCTL: PFWKEN0 Position           */
#define CLK_PDPWCTL_PFWKEN0_Msk         (0x1ul << CLK_PDPWCTL_PFWKEN0_Pos)          /*!< CLK_T::PDPWCTL: PFWKEN0 Mask               */

#define CLK_PDPWCTL_WKPSEL0_Pos         (4)                                         /*!< CLK_T::PDPWCTL: WKPSEL0 Position           */
#define CLK_PDPWCTL_WKPSEL0_Msk         (0xful << CLK_PDPWCTL_WKPSEL0_Pos)          /*!< CLK_T::PDPWCTL: WKPSEL0 Mask               */

#define CLK_PDPWCTL_DBEN0_Pos           (8)                                         /*!< CLK_T::PDPWCTL: DBEN0 Position             */
#define CLK_PDPWCTL_DBEN0_Msk           (0x1ul << CLK_PDPWCTL_DBEN0_Pos)            /*!< CLK_T::PDPWCTL: DBEN0 Mask                 */

#define CLK_PDPWCTL_TRIGM0_Pos          (10)                                        /*!< CLK_T::PDPWCTL: TRIGM0 Position            */
#define CLK_PDPWCTL_TRIGM0_Msk          (0x1ul << CLK_PDPWCTL_TRIGM0_Pos)           /*!< CLK_T::PDPWCTL: TRIGM0 Mask                */

#define CLK_PDPWCTL_NMR0_Pos            (11)                                        /*!< CLK_T::PDPWCTL: NMR0 Position              */
#define CLK_PDPWCTL_NMR0_Msk            (0x1ul << CLK_PDPWCTL_NMR0_Pos)             /*!< CLK_T::PDPWCTL: NMR0 Mask                  */

#define CLK_PDPWCTL_WKEN1_Pos           (16)                                        /*!< CLK_T::PDPWCTL: WKEN1 Position             */
#define CLK_PDPWCTL_WKEN1_Msk           (0x1ul << CLK_PDPWCTL_WKEN1_Pos)            /*!< CLK_T::PDPWCTL: WKEN1 Mask                 */

#define CLK_PDPWCTL_PRWKEN1_Pos         (17)                                        /*!< CLK_T::PDPWCTL: PRWKEN1 Position           */
#define CLK_PDPWCTL_PRWKEN1_Msk         (0x1ul << CLK_PDPWCTL_PRWKEN1_Pos)          /*!< CLK_T::PDPWCTL: PRWKEN1 Mask               */

#define CLK_PDPWCTL_PFWKEN1_Pos         (18)                                        /*!< CLK_T::PDPWCTL: PFWKEN1 Position           */
#define CLK_PDPWCTL_PFWKEN1_Msk         (0x1ul << CLK_PDPWCTL_PFWKEN1_Pos)          /*!< CLK_T::PDPWCTL: PFWKEN1 Mask               */

#define CLK_PDPWCTL_WKPSEL1_Pos         (20)                                        /*!< CLK_T::PDPWCTL: WKPSEL1 Position           */
#define CLK_PDPWCTL_WKPSEL1_Msk         (0xful << CLK_PDPWCTL_WKPSEL1_Pos)          /*!< CLK_T::PDPWCTL: WKPSEL1 Mask               */

#define CLK_PDPWCTL_DBEN1_Pos           (24)                                        /*!< CLK_T::PDPWCTL: DBEN1 Position             */
#define CLK_PDPWCTL_DBEN1_Msk           (0x1ul << CLK_PDPWCTL_DBEN1_Pos)            /*!< CLK_T::PDPWCTL: DBEN1 Mask                 */

#define CLK_PDPWCTL_TRIGM1_Pos          (26)                                        /*!< CLK_T::PDPWCTL: TRIGM1 Position            */
#define CLK_PDPWCTL_TRIGM1_Msk          (0x1ul << CLK_PDPWCTL_TRIGM1_Pos)           /*!< CLK_T::PDPWCTL: TRIGM1 Mask                */

#define CLK_PDPWCTL_NMR1_Pos            (27)                                        /*!< CLK_T::PDPWCTL: NMR1 Position              */
#define CLK_PDPWCTL_NMR1_Msk            (0x1ul << CLK_PDPWCTL_NMR1_Pos)             /*!< CLK_T::PDPWCTL: NMR1 Mask                  */

#define CLK_IOPDCTL_IOHR_Pos            (0)                                         /*!< CLK_T::IOPDCTL: IOHR Position              */
#define CLK_IOPDCTL_IOHR_Msk            (0x1ul << CLK_IOPDCTL_IOHR_Pos)             /*!< CLK_T::IOPDCTL: IOHR Mask                  */

#define CLK_IOPDCTL_DPDHOLDEN_Pos       (8)                                         /*!< CLK_T::IOPDCTL: DPDHOLDEN Position         */
#define CLK_IOPDCTL_DPDHOLDEN_Msk       (0x1ul << CLK_IOPDCTL_DPDHOLDEN_Pos)        /*!< CLK_T::IOPDCTL: DPDHOLDEN Mask             */

#define CLK_PMUINTC_WKTMRIE_Pos         (0)                                         /*!< CLK_T::PMUINTC: WKTMRIE Position           */
#define CLK_PMUINTC_WKTMRIE_Msk         (0x1ul << CLK_PMUINTC_WKTMRIE_Pos)          /*!< CLK_T::PMUINTC: WKTMRIE Mask               */

#define CLK_PMUINTC_WKIOA0IE_Pos        (8)                                         /*!< CLK_T::PMUINTC: WKIOA0IE Position          */
#define CLK_PMUINTC_WKIOA0IE_Msk        (0x1ul << CLK_PMUINTC_WKIOA0IE_Pos)         /*!< CLK_T::PMUINTC: WKIOA0IE Mask              */

#define CLK_PMUINTC_WKIOB0IE_Pos        (9)                                         /*!< CLK_T::PMUINTC: WKIOB0IE Position          */
#define CLK_PMUINTC_WKIOB0IE_Msk        (0x1ul << CLK_PMUINTC_WKIOB0IE_Pos)         /*!< CLK_T::PMUINTC: WKIOB0IE Mask              */

#define CLK_PMUINTC_WKIOC0IE_Pos        (10)                                        /*!< CLK_T::PMUINTC: WKIOC0IE Position          */
#define CLK_PMUINTC_WKIOC0IE_Msk        (0x1ul << CLK_PMUINTC_WKIOC0IE_Pos)         /*!< CLK_T::PMUINTC: WKIOC0IE Mask              */

#define CLK_PMUINTC_WKIOD0IE_Pos        (11)                                        /*!< CLK_T::PMUINTC: WKIOD0IE Position          */
#define CLK_PMUINTC_WKIOD0IE_Msk        (0x1ul << CLK_PMUINTC_WKIOD0IE_Pos)         /*!< CLK_T::PMUINTC: WKIOD0IE Mask              */

#define CLK_PMUINTC_WKIOA1IE_Pos        (12)                                        /*!< CLK_T::PMUINTC: WKIOA1IE Position          */
#define CLK_PMUINTC_WKIOA1IE_Msk        (0x1ul << CLK_PMUINTC_WKIOA1IE_Pos)         /*!< CLK_T::PMUINTC: WKIOA1IE Mask              */

#define CLK_PMUINTC_WKIOB1IE_Pos        (13)                                        /*!< CLK_T::PMUINTC: WKIOB1IE Position          */
#define CLK_PMUINTC_WKIOB1IE_Msk        (0x1ul << CLK_PMUINTC_WKIOB1IE_Pos)         /*!< CLK_T::PMUINTC: WKIOB1IE Mask              */

#define CLK_PMUINTC_WKIOC1IE_Pos        (14)                                        /*!< CLK_T::PMUINTC: WKIOC1IE Position          */
#define CLK_PMUINTC_WKIOC1IE_Msk        (0x1ul << CLK_PMUINTC_WKIOC1IE_Pos)         /*!< CLK_T::PMUINTC: WKIOC1IE Mask              */

#define CLK_PMUINTC_WKIOD1IE_Pos        (15)                                        /*!< CLK_T::PMUINTC: WKIOD1IE Position          */
#define CLK_PMUINTC_WKIOD1IE_Msk        (0x1ul << CLK_PMUINTC_WKIOD1IE_Pos)         /*!< CLK_T::PMUINTC: WKIOD1IE Mask              */

#define CLK_PMUINTS_WKTMRIF_Pos         (0)                                         /*!< CLK_T::PMUINTS: WKTMRIF Position           */
#define CLK_PMUINTS_WKTMRIF_Msk         (0x1ul << CLK_PMUINTS_WKTMRIF_Pos)          /*!< CLK_T::PMUINTS: WKTMRIF Mask               */

#define CLK_PMUINTS_WKIOA0IF_Pos        (8)                                         /*!< CLK_T::PMUINTS: WKIOA0IF Position          */
#define CLK_PMUINTS_WKIOA0IF_Msk        (0x1ul << CLK_PMUINTS_WKIOA0IF_Pos)         /*!< CLK_T::PMUINTS: WKIOA0IF Mask              */

#define CLK_PMUINTS_WKIOB0IF_Pos        (9)                                         /*!< CLK_T::PMUINTS: WKIOB0IF Position          */
#define CLK_PMUINTS_WKIOB0IF_Msk        (0x1ul << CLK_PMUINTS_WKIOB0IF_Pos)         /*!< CLK_T::PMUINTS: WKIOB0IF Mask              */

#define CLK_PMUINTS_WKIOC0IF_Pos        (10)                                        /*!< CLK_T::PMUINTS: WKIOC0IF Position          */
#define CLK_PMUINTS_WKIOC0IF_Msk        (0x1ul << CLK_PMUINTS_WKIOC0IF_Pos)         /*!< CLK_T::PMUINTS: WKIOC0IF Mask              */

#define CLK_PMUINTS_WKIOD0IF_Pos        (11)                                        /*!< CLK_T::PMUINTS: WKIOD0IF Position          */
#define CLK_PMUINTS_WKIOD0IF_Msk        (0x1ul << CLK_PMUINTS_WKIOD0IF_Pos)         /*!< CLK_T::PMUINTS: WKIOD0IF Mask              */

#define CLK_PMUINTS_WKIOA1IF_Pos        (12)                                        /*!< CLK_T::PMUINTS: WKIOA1IF Position          */
#define CLK_PMUINTS_WKIOA1IF_Msk        (0x1ul << CLK_PMUINTS_WKIOA1IF_Pos)         /*!< CLK_T::PMUINTS: WKIOA1IF Mask              */

#define CLK_PMUINTS_WKIOB1IF_Pos        (13)                                        /*!< CLK_T::PMUINTS: WKIOB1IF Position          */
#define CLK_PMUINTS_WKIOB1IF_Msk        (0x1ul << CLK_PMUINTS_WKIOB1IF_Pos)         /*!< CLK_T::PMUINTS: WKIOB1IF Mask              */

#define CLK_PMUINTS_WKIOC1IF_Pos        (14)                                        /*!< CLK_T::PMUINTS: WKIOC1IF Position          */
#define CLK_PMUINTS_WKIOC1IF_Msk        (0x1ul << CLK_PMUINTS_WKIOC1IF_Pos)         /*!< CLK_T::PMUINTS: WKIOC1IF Mask              */

#define CLK_PMUINTS_WKIOD1IF_Pos        (15)                                        /*!< CLK_T::PMUINTS: WKIOD1IF Position          */
#define CLK_PMUINTS_WKIOD1IF_Msk        (0x1ul << CLK_PMUINTS_WKIOD1IF_Pos)         /*!< CLK_T::PMUINTS: WKIOD1IF Mask              */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __CLK_REG_H__ */
