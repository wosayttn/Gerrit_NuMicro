/**************************************************************************//**
 * @file     sys_reg.h
 * @version  V1.00
 * @brief    SYS register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SYS_REG_H__
#define __SYS_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup SYS System Manger Controller (SYS)
    Memory Mapped Structure for SYS Controller
    @{
*/

typedef struct
{
    /**
     * @var SYS_T::PDID
     * Offset: 0x00  Part Device Identification Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |Part Device Identification Number (Read Only)
     * |        |          |This register reflects device part number code
     * |        |          |Software can read this register to identify which device is used.
     * @var SYS_T::RSTSTS
     * Offset: 0x04  System Reset Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PORF      |POR Reset Flag
     * |        |          |The POR reset flag is set by the "Reset Signal" from the Power-on Reset (POR) Controller or bit CHIPRST (SYS_IPRST0[0]) to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIPRST.
     * |        |          |1 = Power-on Reset (POR) or CHIPRST had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[1]     |PINRF     |NRESET Pin Reset Flag
     * |        |          |The nRESET pin reset flag is set by the "Reset Signal" from the nRESET Pin to indicate the previous reset source.
     * |        |          |0 = No reset from nRESET pin.
     * |        |          |1 = Pin nRESET had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[2]     |WDTRF     |WDT Reset Flag
     * |        |          |The WDT reset flag is set by the "Reset Signal" from the Watchdog Timer or Window Watchdog Timer to indicate the previous reset source.
     * |        |          |0 = No reset from watchdog timer or window watchdog timer.
     * |        |          |1 = The watchdog timer or window watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note1: Write 1 to clear this bit to 0.
     * |        |          |Note2: Watchdog Timer register RSTF(WDT_CTL[2]) bit is set if the system has been reset by WDT time-out reset
     * |        |          |Window Watchdog Timer register WWDTRF(WWDT_STATUS[1]) bit is set if the system has been reset by WWDT time-out reset.
     * |[3]     |LVRF      |LVR Reset Flag
     * |        |          |The LVR reset flag is set by the "Reset Signal" from the Low Voltage Reset Controller to indicate the previous reset source.
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = LVR controller had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |BODRF     |BOD Reset Flag
     * |        |          |The BOD reset flag is set by the "Reset Signal" from the Brown-Out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD reset had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[5]     |SYSRF     |System Reset Flag
     * |        |          |The system reset flag is set by the "Reset Signal" from the Cortex-M23 core to indicate the previous reset source.
     * |        |          |0 = No reset from Cortex-M23.
     * |        |          |1 = The Cortex-M23 had issued the reset signal to reset the system by writing 1 to the bit SYSRESETREQ(AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M23 core.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[6]     |HRESETRF  |HRESET Reset Flag
     * |        |          |The HRESET reset flag is set by the "Reset Signal" from the HRESET.
     * |        |          |0 = No reset from HRESET.
     * |        |          |1 = Reset from HRESET.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[7]     |CPURF     |CPU Reset Flag
     * |        |          |The CPU reset flag is set by hardware if software writes CPURST (SYS_IPRST0[1]) 1 to reset Cortex-M23 core and Flash Memory Controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = The Cortex-M23 core and FMC are reset by software setting CPURST to 1.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[8]     |CPULKRF   |CPU Lockup Reset Flag
     * |        |          |0 = No reset from CPU lockup happened.
     * |        |          |1 = The Cortex-M23 lockup happened and chip is reset.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: When CPU lockup happened under ICE is connected, this flag will set to 1 but chip will not reset.
     * @var SYS_T::IPRST0
     * Offset: 0x08  Peripheral  Reset Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIPRST   |Chip One-shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the whole chip, including Processor core and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIPRST is same as the POR reset, all the chip controllers is reset and the chip setting from Flash are also reload.
     * |        |          |About the difference between CHIPRST and SYSRESETREQ(AIRCR[2]), please refer to section 7.2.2
     * |        |          |0 = Chip normal operation.
     * |        |          |1 = Chip one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |CPURST    |Processor Core One-shot Reset (Write Protect)
     * |        |          |Setting this bit will only reset the processor core and Flash Memory Controller(FMC), and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |0 = Processor core normal operation.
     * |        |          |1 = Processor core one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |PDMA0RST  |PDMA0 Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the PDMA0.
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = PDMA0 controller normal operation.
     * |        |          |1 = PDMA0 controller reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |CRCRST    |CRC Calculation Controller Reset (Write Protect)
     * |        |          |Set this bit to 1 will generate a reset signal to the CRC calculation controller
     * |        |          |User needs to set this bit to 0 to release from the reset state.
     * |        |          |0 = CRC calculation controller normal operation.
     * |        |          |1 = CRC calculation controller reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[12]    |CRPTRST   |CRYPTO Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the CRYPTO controller
     * |        |          |User needs to set this bit to 0 to release from the reset state.
     * |        |          |0 = CRYPTO controller normal operation.
     * |        |          |1 = CRYPTO controller reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[24]    |GPIORST   |GPIO Controller Reset
     * |        |          |0 = GPIO controller normal operation.
     * |        |          |1 = GPIO controller reset.
     * @var SYS_T::IPRST1
     * Offset: 0x0C  Peripheral Reset Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |TMR0RST   |Timer0 Controller Reset
     * |        |          |0 = Timer0 controller normal operation.
     * |        |          |1 = Timer0 controller reset.
     * |[3]     |TMR1RST   |Timer1 Controller Reset
     * |        |          |0 = Timer1 controller normal operation.
     * |        |          |1 = Timer1 controller reset.
     * |[4]     |TMR2RST   |Timer2 Controller Reset
     * |        |          |0 = Timer2 controller normal operation.
     * |        |          |1 = Timer2 controller reset.
     * |[5]     |TMR3RST   |Timer3 Controller Reset
     * |        |          |0 = Timer3 controller normal operation.
     * |        |          |1 = Timer3 controller reset.
     * |[7]     |ACMP01RST |ACMP01 Controller Reset
     * |        |          |0 = ACMP01 controller normal operation.
     * |        |          |1 = ACMP01 controller reset.
     * |[8]     |I2C0RST   |I2C0 Controller Reset
     * |        |          |0 = I2C0 controller normal operation.
     * |        |          |1 = I2C0 controller reset.
     * |[9]     |I2C1RST   |I2C1 Controller Reset
     * |        |          |0 = I2C1 controller normal operation.
     * |        |          |1 = I2C1 controller reset.
     * |[10]    |I2C2RST   |I2C2 Controller Reset
     * |        |          |0 = I2C2 controller normal operation.
     * |        |          |1 = I2C2 controller reset.`.
     * |[12]    |SPI0RST   |SPI0 Controller Reset
     * |        |          |0 = SPI0 controller normal operation.
     * |        |          |1 = SPI0 controller reset.
     * |[13]    |SPI1RST   |SPI1 Controller Reset
     * |        |          |0 = SPI1 controller normal operation.
     * |        |          |1 = SPI1 controller reset.
     * |[14]    |SPI2RST   |SPI2 Controller Reset
     * |        |          |0 = SPI2 controller normal operation.
     * |        |          |1 = SPI2 controller reset.
     * |[16]    |UART0RST  |UART0 Controller Reset
     * |        |          |0 = UART0 controller normal operation.
     * |        |          |1 = UART0 controller reset.
     * |[17]    |UART1RST  |UART1 Controller Reset
     * |        |          |0 = UART1 controller normal operation.
     * |        |          |1 = UART1 controller reset.
     * |[18]    |UART2RST  |UART2 Controller Reset
     * |        |          |0 = UART2 controller normal operation.
     * |        |          |1 = UART2 controller reset.
     * |[20]    |USCI0RST  |USCI0 Controller Reset
     * |        |          |0 = USCI0 controller normal operation.
     * |        |          |1 = USCI0 controller reset.
     * |[21]    |WWDTRST   |WWDT Controller Reset
     * |        |          |0 = WWDT controller normal operation.
     * |        |          |1 = WWDT controller reset.
     * |[22]    |PWM0RST   |PWM0 Controller Reset
     * |        |          |0 = PWM0 controller normal operation.
     * |        |          |1 = PWM0 controller reset.
     * |[23]    |BPWM0RST  |BPWM0 Controller Reset
     * |        |          |0 = BPWM0 controller normal operation.
     * |        |          |1 = BPWM0 controller reset.
     * |[24]    |LCDRST    |LCD Controller Reset
     * |        |          |0 = LCD controller normal operation.
     * |        |          |1 = LCD controller reset.
     * |[28]    |ADC0RST   |ADC0 Controller Reset
     * |        |          |0 = ADC0 controller normal operation.
     * |        |          |1 = ADC0 controller reset.
     * @var SYS_T::BODCTL
     * Offset: 0x18  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODEN     |Brown-out Detector Enable Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBODEN (CONFIG0 [19]).
     * |        |          |0 = Brown-out Detector function Disabled.
     * |        |          |1 = Brown-out Detector function Enabled.
     * |        |          |Note 1: BOD can wakeup chip from SPD0 mode after setting this bit 1.
     * |        |          |Note 2: BOD function not support in NPD2.
     * |        |          |Note 3: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2:1]   |BODSIFEN  |Brown-out Detector Edge Selection for Interrupt Enable
     * |        |          |This bits are BOD edge selection for interrupt enable control for detect any voltage draft at AVDD up/down through the voltage of BODVL setting.
     * |        |          |00 = BOD Interrupt enable for AVDD up/down through BODVL voltage or AVDD below BODVL voltage
     * |        |          |(Default)
     * |        |          |01 = BOD Interrupt enable for AVDD down through the BODVL voltage.
     * |        |          |10 = BOD Interrupt enable for AVDD up through BODVL voltage.
     * |        |          |11 = Reserved.
     * |        |          |Note: Referance BOD Datasheet for BOD respond time(TBOD_RE).
     * |[3]     |BODRSTEN  |Brown-out Reset Enable Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBORST(CONFIG0[20]) bit.
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled.
     * |        |          |1 = Brown-out "RESET" function Enabled.
     * |        |          |Note 1: While the Brown-out Detector function is enabled (BODEN high) and BOD reset function is enabled (BODRSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BODOUT high).
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |BODIF     |Brown-out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at AVDD down through or up through the voltage of BODVL setting.
     * |        |          |1 = When Brown-out Detector detects the AVDD is dropped down through the voltage of BODVL setting or the AVDD is raised up through the voltage of BODVL setting, this bit is set to 1 and the brown-out interrupt is requested if brown-out interrupt is enabled.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: BOD must be enabled (BODEN=1), when this bit is written 1 to clear.
     * |[5]     |BODLPM    |Brown-out Detector Low Power Mode (Write Protect)
     * |        |          |0 = BOD operate in normal mode (default).
     * |        |          |1 = BOD Low Power mode Enabled.
     * |        |          |Note 1: The BOD low power mode can reduce the current to about 1/50 but slow the BOD response.
     * |        |          |Note 2: This bit is suppoorted during chip in SPD0 mode.
     * |        |          |Note 3: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |BODOUT    |Brown-out Detector Output Status (Read Only)
     * |        |          |0 = Brown-out Detector output status is 0.
     * |        |          |It means the detected voltage is higher than BODVL setting or BODEN is 0.
     * |        |          |1 = Brown-out Detector output status is 1.
     * |        |          |It means the detected voltage is lower than BODVL setting
     * |        |          |If the BODEN is 0, BOD function disabled , this bit always responds 0000.
     * |[7]     |LVREN     |Low Voltage Reset Enable Bit (Write Protect)
     * |        |          |The LVR function resets the chip when the input power voltage is lower than LVR circuit setting
     * |        |          |LVR function is enabled by default.
     * |        |          |0 = Low Voltage Reset function Disabled.
     * |        |          |1 = Low Voltage Reset function Enabled.
     * |        |          |Note1: After enabling the bit, the LVR function will be active with 100us delay for LVR output stable (default).
     * |        |          |Note2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[10:8]  |BODDGSEL  |Brown-out Detector Output De-glitch Time Select (Write Protect)
     * |        |          |000 = BOD output is sampled by LIRC clock.
     * |        |          |001 = 3~4 system clock (HCLK).
     * |        |          |010 = 7~8 system clock (HCLK).
     * |        |          |011 = 15~16 system clock (HCLK).
     * |        |          |100 = 31~32 system clock (HCLK).
     * |        |          |101 = 63~64 system clock (HCLK).
     * |        |          |110 = 127~128 system clock (HCLK).
     * |        |          |111 = 255~256 system clock (HCLK).
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[14:12] |LVRDGSEL  |LVR Output De-glitch Time Select (Write Protect)
     * |        |          |000 = Without de-glitch function.
     * |        |          |001 = 4 system clock (HCLK).
     * |        |          |010 = 8 system clock (HCLK).
     * |        |          |011 = 16 system clock (HCLK).
     * |        |          |100 = 32 system clock (HCLK).
     * |        |          |101 = 64 system clock (HCLK).
     * |        |          |110 = 128 system clock (HCLK).
     * |        |          |111 = 256 system clock (HCLK).
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[15]    |LVRRDY    |LVR Enable Ready Flag (Read Only)
     * |        |          |0 = LVR disabled and not ready.
     * |        |          |1 = LVR enabled and ready.
     * |[18:16] |BODVL     |Brown-out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBOV (CONFIG0[23:21]).
     * |        |          |000 = Reserved.
     * |        |          |001 = Brown-Out Detector threshold voltage is 1.8V.
     * |        |          |010 = Brown-Out Detector threshold voltage is 2.0V.
     * |        |          |011 = Brown-Out Detector threshold voltage is 2.4V.
     * |        |          |100 = Brown-Out Detector threshold voltage is 2.7V.
     * |        |          |101 = Brown-Out Detector threshold voltage is 3.0V.
     * |        |          |110 = Brown-Out Detector threshold voltage is 3.7V.
     * |        |          |111 = Brown-Out Detector threshold voltage is 4.4V.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::IVSCTL
     * Offset: 0x1C  Internal Voltage Source Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VTEMPEN   |Temperature Sensor Enable Bit
     * |        |          |This bit is used to enable/disable temperature sensor function.
     * |        |          |0 = Temperature sensor function Disabled (default).
     * |        |          |1 = Temperature sensor function Enabled.
     * |[2]     |AVDDDIV4EN|AVDD divide 4 Enable Bit
     * |        |          |This bit is used to enable/disable AVDD divide 4 function.
     * |        |          |0 = AVDD divide 4 function Disabled (default).
     * |        |          |1 = AVDD divide 4 function Enabled.
     * |        |          |Note: After this bit is set to 1, the value of AVDD divide 4 output voltage can be obtained from ADC conversion result
     * @var SYS_T::VREFCTL
     * Offset: 0x28  VREF Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VREFEN    |Internal Voltage Reference VREF Enable Bit (Write Protect)
     * |        |          |0 = Internal voltage reference function is Disabled.
     * |        |          |1 = Internal voltage reference function is Enabled.
     * |        |          |Note: Only supported while package includes VREF pin with external capacitor.
     * |[3:1]   |VREFSEL   |Internal Voltage Reference VREF Scale Select (Write Protect)
     * |        |          |000 = Internal voltage reference(INT_VREF) is set to 1.536V.
     * |        |          |001 = Internal voltage reference(INT_VREF) is set to 2.048V.
     * |        |          |010 = Internal voltage reference(INT_VREF) is set to 2.56V.
     * |        |          |011 = Internal voltage reference(INT_VREF) is set to 3.072V.
     * |        |          |100 = Internal voltage reference(INT_VREF) is set to 4.096V.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[7:6]   |PRELOADSEL|Internal Voltage Reference Pre-load Timing Selection (Write Protect)
     * |        |          |00 = pre-load time is 15ms, for VREF output 4.7uF Capacitor.
     * |        |          |10 = pre-load time is 3500us, for VREF output 1uF Capacitor.
     * |        |          |Others = Reserved.
     * |[10]    |PRLDSTS   |Internal Voltage Reference Pre-load Status (Read Only)
     * |        |          |This bit indicates Internal Voltage Reference pre-load status.
     * |        |          |When VREFEN is set to 1, this bit will be set, until selected Pre-load Timing (PRELOAD_SEL) is completed, this bit will be clear automatically.
     * |        |          |0 = Internal Voltage Reference is not under pre-load status.
     * |        |          |1 = Internal Voltage Reference is under pre-load status.
     * |        |          |Note 1: User must wait until PRLDSTS is 0 before chip enter power-down mode, to avoid additional preload power consumption.
     * |        |          |Note 2: User must check PRLDSTS to wait for INT_VREF stable before using ADC and so on.
     * |[24]    |VBGFEN    |Chip Internal Voltage Band-gap Force Enable Bit (Write Only)
     * |        |          |0 = Chip internal voltage band-gap controlled by ADC/ACMP if source selected.
     * |        |          |1 = Chip internal voltage band-gap force enable.
     * @var SYS_T::GPA_MFPL
     * Offset: 0x30  GPIOA Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PA0MFP    |PA.0 Multi-function Pin Selection
     * |        |          |04 = SPI0_MOSI.
     * |        |          |07 = UART0_RXD.
     * |        |          |08 = UART1_nRTS.
     * |        |          |09 = I2C2_SDA.
     * |        |          |12 = BPWM0_CH0.
     * |        |          |13 = PWM0_CH5.
     * |[7:4]   |PA1MFP    |PA.1 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = SPI0_MISO.
     * |        |          |05 = PBUF.
     * |        |          |07 = UART0_TXD.
     * |        |          |08 = UART1_nCTS.
     * |        |          |09 = I2C2_SCL.
     * |        |          |12 = BPWM0_CH1.
     * |        |          |13 = PWM0_CH4.
     * |[11:8]  |PA2MFP    |PA.2 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = SPI0_CLK.
     * |        |          |05 = PBUF.
     * |        |          |07 = UART1_RXD.
     * |        |          |09 = I2C1_SDA.
     * |        |          |10 = I2C0_SMBSUS.
     * |        |          |12 = BPWM0_CH2.
     * |        |          |13 = PWM0_CH3.
     * |[15:12] |PA3MFP    |PA.3 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = SPI0_SS.
     * |        |          |05 = PBUF.
     * |        |          |07 = UART1_TXD.
     * |        |          |09 = I2C1_SCL.
     * |        |          |10 = I2C0_SMBAL.
     * |        |          |12 = BPWM0_CH3.
     * |        |          |13 = PWM0_CH2.
     * |[19:16] |PA4MFP    |PA.4 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |05 = PBUF.
     * |        |          |07 = UART0_nRTS.
     * |        |          |09 = I2C0_SDA.
     * |        |          |11 = UART0_RXD.
     * |        |          |12 = BPWM0_CH4.
     * |        |          |13 = PWM0_CH1.
     * |[23:20] |PA5MFP    |PA.5 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |05 = PBUF.
     * |        |          |07 = UART0_nCTS.
     * |        |          |09 = I2C0_SCL.
     * |        |          |11 = UART0_TXD.
     * |        |          |12 = BPWM0_CH5.
     * |        |          |13 = PWM0_CH0.
     * |[27:24] |PA6MFP    |PA.6 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = SPI1_SS.
     * |        |          |07 = UART0_RXD.
     * |        |          |08 = LCD_PIN20.
     * |        |          |09 = I2C1_SDA.
     * |        |          |10 = PBUF.
     * |        |          |13 = ACMP1_WLAT.
     * |        |          |14 = TM3.
     * |        |          |15 = INT0.
     * |[31:28] |PA7MFP    |PA.7 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = SPI1_CLK.
     * |        |          |07 = UART0_TXD.
     * |        |          |08 = LCD_PIN19.
     * |        |          |09 = I2C1_SCL.
     * |        |          |10 = PBUF.
     * |        |          |13 = ACMP0_WLAT.
     * |        |          |14 = TM2.
     * |        |          |15 = INT1.
     * @var SYS_T::GPA_MFPH
     * Offset: 0x34  GPIOA High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PA8MFP    |PA.8 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = SPI2_MOSI.
     * |        |          |06 = USCI0_CTL1.
     * |        |          |07 = UART1_RXD.
     * |        |          |08 = LCD_PIN8.
     * |        |          |09 = BPWM0_CH3.
     * |        |          |10 = PBUF.
     * |        |          |13 = TM3_EXT.
     * |        |          |15 = INT4.
     * |[7:4]   |PA9MFP    |PA.9 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = SPI2_MISO.
     * |        |          |06 = USCI0_DAT1.
     * |        |          |07 = UART1_TXD.
     * |        |          |08 = LCD_POWER.
     * |        |          |09 = BPWM0_CH2.
     * |        |          |10 = PBUF.
     * |        |          |13 = TM2_EXT.
     * |[11:8]  |PA10MFP   |PA.10 Multi-function Pin Selection
     * |        |          |01 = Analog7.
     * |        |          |02 = PINV.
     * |        |          |04 = SPI2_CLK.
     * |        |          |06 = USCI0_DAT0.
     * |        |          |07 = I2C2_SDA.
     * |        |          |08 = LCD_POWER.
     * |        |          |09 = BPWM0_CH1.
     * |        |          |10 = PBUF.
     * |        |          |13 = TM1_EXT.
     * |[15:12] |PA11MFP   |PA.11 Multi-function Pin Selection
     * |        |          |01 = Analog6.
     * |        |          |02 = PINV.
     * |        |          |04 = SPI2_SS.
     * |        |          |06 = USCI0_CLK.
     * |        |          |07 = I2C2_SCL.
     * |        |          |08 = LCD_POWER.
     * |        |          |09 = BPWM0_CH0.
     * |        |          |10 = PBUF.
     * |        |          |13 = TM0_EXT.
     * |[19:16] |PA12MFP   |PA.12 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = I2C1_SCL.
     * |        |          |05 = SPI2_SS.
     * |        |          |10 = PBUF.
     * |[23:20] |PA13MFP   |PA.13 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = I2C1_SDA.
     * |        |          |05 = SPI2_CLK.
     * |        |          |10 = PBUF.
     * |[27:24] |PA14MFP   |PA.14 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |03 = UART0_TXD.
     * |        |          |04 = I2C1_SMBAL.
     * |        |          |05 = SPI2_MISO.
     * |        |          |06 = I2C2_SCL.
     * |        |          |09 = I2C0_SMBAL.
     * |        |          |10 = PBUF.
     * |[31:28] |PA15MFP   |PA.15 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |03 = UART0_RXD.
     * |        |          |04 = I2C1_SMBSUS.
     * |        |          |05 = SPI2_MOSI.
     * |        |          |06 = I2C2_SDA.
     * |        |          |09 = I2C0_SMBSUS.
     * |        |          |10 = PBUF.
     * @var SYS_T::GPB_MFPL
     * Offset: 0x38  GPIOB Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PB0MFP    |PB.0 Multi-function Pin Selection
     * |        |          |01 = Analog5.
     * |        |          |06 = USCI0_CTL0.
     * |        |          |07 = UART2_RXD.
     * |        |          |08 = LCD_POWER.
     * |        |          |09 = I2C1_SDA.
     * |        |          |11 = PWM0_CH5.
     * |        |          |13 = PWM0_BRAKE1.
     * |[7:4]   |PB1MFP    |PB.1 Multi-function Pin Selection
     * |        |          |01 = Analog4.
     * |        |          |02 = PINV.
     * |        |          |07 = UART2_TXD.
     * |        |          |08 = LCD_POWER.
     * |        |          |09 = I2C1_SCL.
     * |        |          |10 = PBUF.
     * |        |          |11 = PWM0_CH4.
     * |        |          |13 = PWM0_BRAKE0.
     * |[11:8]  |PB2MFP    |PB.2 Multi-function Pin Selection
     * |        |          |01 = Analog3.
     * |        |          |02 = PINV.
     * |        |          |05 = SPI1_SS.
     * |        |          |06 = UART1_RXD.
     * |        |          |08 = LCD_PIN3.
     * |        |          |10 = PBUF.
     * |        |          |11 = PWM0_CH3.
     * |        |          |12 = I2C1_SDA.
     * |        |          |14 = TM3.
     * |        |          |15 = INT3.
     * |[15:12] |PB3MFP    |PB.3 Multi-function Pin Selection
     * |        |          |01 = Analog2.
     * |        |          |02 = PINV.
     * |        |          |05 = SPI1_CLK.
     * |        |          |06 = UART1_TXD.
     * |        |          |08 = LCD_PIN2.
     * |        |          |10 = PBUF.
     * |        |          |11 = PWM0_CH2.
     * |        |          |12 = I2C1_SCL.
     * |        |          |14 = TM2.
     * |        |          |15 = INT2.
     * |[19:16] |PB4MFP    |PB.4 Multi-function Pin Selection
     * |        |          |01 = Analog1.
     * |        |          |02 = PINV.
     * |        |          |05 = SPI1_MOSI.
     * |        |          |06 = I2C0_SDA.
     * |        |          |08 = LCD_PIN1.
     * |        |          |10 = PBUF.
     * |        |          |11 = PWM0_CH1.
     * |        |          |12 = UART2_RXD.
     * |        |          |14 = TM1.
     * |        |          |15 = INT1.
     * |[23:20] |PB5MFP    |PB.5 Multi-function Pin Selection
     * |        |          |01 = Analog0.
     * |        |          |02 = PINV.
     * |        |          |05 = SPI1_MISO.
     * |        |          |06 = I2C0_SCL.
     * |        |          |08 = LCD_PIN0.
     * |        |          |10 = PBUF.
     * |        |          |11 = PWM0_CH0.
     * |        |          |12 = UART2_TXD.
     * |        |          |14 = TM0.
     * |        |          |15 = INT0.
     * |[27:24] |PB6MFP    |PB.6 Multi-function Pin Selection
     * |        |          |01 = Analog17.
     * |        |          |02 = PINV.
     * |        |          |06 = UART1_RXD.
     * |        |          |08 = LCD_PIN49.
     * |        |          |09 = SPI0_CLK.
     * |        |          |10 = PBUF.
     * |        |          |13 = INT4.
     * |        |          |15 = ACMP1_O.
     * |[31:28] |PB7MFP    |PB.7 Multi-function Pin Selection
     * |        |          |01 = Analog16.
     * |        |          |02 = PINV.
     * |        |          |06 = UART1_TXD.
     * |        |          |08 = LCD_PIN48.
     * |        |          |09 = SPI0_SS.
     * |        |          |10 = PBUF.
     * |        |          |13 = INT5.
     * |        |          |15 = ACMP0_O.
     * @var SYS_T::GPB_MFPH
     * Offset: 0x3C  GPIOB High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PB8MFP    |PB.8 Multi-function Pin Selection
     * |        |          |01 = Analog15.
     * |        |          |02 = PINV.
     * |        |          |05 = UART0_RXD.
     * |        |          |06 = UART1_nRTS.
     * |        |          |07 = I2C1_SMBSUS.
     * |        |          |08 = LCD_PIN47.
     * |        |          |09 = I2C0_SDA.
     * |        |          |10 = PBUF.
     * |        |          |13 = INT6.
     * |[7:4]   |PB9MFP    |PB.9 Multi-function Pin Selection
     * |        |          |01 = Analog14.
     * |        |          |02 = PINV.
     * |        |          |05 = UART0_TXD.
     * |        |          |06 = UART1_nCTS.
     * |        |          |07 = I2C1_SMBAL.
     * |        |          |08 = LCD_PIN46.
     * |        |          |09 = I2C0_SCL.
     * |        |          |10 = PBUF.
     * |        |          |13 = INT7.
     * |[11:8]  |PB10MFP   |PB.10 Multi-function Pin Selection
     * |        |          |01 = Analog13.
     * |        |          |02 = PINV.
     * |        |          |05 = UART0_nRTS.
     * |        |          |07 = I2C1_SDA.
     * |        |          |08 = LCD_PIN45.
     * |        |          |10 = PBUF.
     * |[15:12] |PB11MFP   |PB.11 Multi-function Pin Selection
     * |        |          |01 = Analog12.
     * |        |          |02 = PINV.
     * |        |          |05 = UART0_nCTS.
     * |        |          |07 = I2C1_SCL.
     * |        |          |08 = LCD_PIN44.
     * |        |          |10 = PBUF.
     * |[19:16] |PB12MFP   |PB.12 Multi-function Pin Selection
     * |        |          |01 = Analog11.
     * |        |          |02 = PINV.
     * |        |          |04 = SPI0_MOSI.
     * |        |          |05 = USCI0_CLK.
     * |        |          |06 = UART0_RXD.
     * |        |          |07 = I2C2_SDA.
     * |        |          |08 = LCD_PIN43.
     * |        |          |10 = PBUF.
     * |        |          |13 = TM3_EXT.
     * |[23:20] |PB13MFP   |PB.13 Multi-function Pin Selection
     * |        |          |01 = Analog10.
     * |        |          |02 = PINV.
     * |        |          |04 = SPI0_MISO.
     * |        |          |05 = USCI0_DAT0.
     * |        |          |06 = UART0_TXD.
     * |        |          |07 = I2C2_SCL.
     * |        |          |08 = LCD_PIN42.
     * |        |          |09 = CLKO.
     * |        |          |10 = PBUF.
     * |        |          |13 = TM2_EXT.
     * |[27:24] |PB14MFP   |PB.14 Multi-function Pin Selection
     * |        |          |01 = Analog9.
     * |        |          |02 = PINV.
     * |        |          |04 = SPI0_CLK.
     * |        |          |05 = USCI0_DAT1.
     * |        |          |06 = UART0_nRTS.
     * |        |          |07 = I2C2_SMBSUS.
     * |        |          |08 = LCD_PIN41.
     * |        |          |10 = PBUF.
     * |        |          |13 = TM1_EXT.
     * |        |          |14 = CLKO.
     * |[31:28] |PB15MFP   |PB.15 Multi-function Pin Selection
     * |        |          |01 = Analog8.
     * |        |          |02 = PINV.
     * |        |          |04 = SPI0_SS.
     * |        |          |05 = USCI0_CTL1.
     * |        |          |06 = UART0_nCTS.
     * |        |          |07 = I2C2_SMBAL.
     * |        |          |08 = LCD_PIN40.
     * |        |          |10 = PBUF.
     * |        |          |13 = TM0_EXT.
     * @var SYS_T::GPC_MFPL
     * Offset: 0x40  GPIOC Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PC0MFP    |PC.0 Multi-function Pin Selection
     * |        |          |06 = UART2_RXD.
     * |        |          |07 = SPI1_SS.
     * |        |          |08 = LCD_PIN29.
     * |        |          |09 = I2C0_SDA.
     * |        |          |14 = ACMP1_O.
     * |[7:4]   |PC1MFP    |PC.1 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |06 = UART2_TXD.
     * |        |          |07 = SPI1_CLK.
     * |        |          |08 = LCD_PIN28.
     * |        |          |09 = I2C0_SCL.
     * |        |          |10 = PBUF.
     * |        |          |14 = ACMP0_O.
     * |        |          |15 = ADC0_ST.
     * |[11:8]  |PC2MFP    |PC.2 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |06 = UART2_nCTS.
     * |        |          |07 = SPI1_MOSI.
     * |        |          |08 = LCD_PIN27.
     * |        |          |09 = I2C0_SMBSUS.
     * |        |          |10 = PBUF.
     * |[15:12] |PC3MFP    |PC.3 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |06 = UART2_nRTS.
     * |        |          |07 = SPI1_MISO.
     * |        |          |08 = LCD_PIN26.
     * |        |          |09 = I2C0_SMBAL.
     * |        |          |10 = PBUF.
     * |[19:16] |PC4MFP    |PC.4 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |06 = UART2_RXD.
     * |        |          |08 = LCD_PIN25.
     * |        |          |09 = I2C1_SDA.
     * |        |          |10 = PBUF.
     * |        |          |13 = ACMP1_WLAT.
     * |[23:20] |PC5MFP    |PC.5 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |06 = UART2_TXD.
     * |        |          |08 = LCD_PIN24.
     * |        |          |09 = I2C1_SCL.
     * |        |          |10 = PBUF.
     * |        |          |13 = ACMP0_WLAT.
     * |[27:24] |PC6MFP    |PC.6 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = SPI1_MOSI.
     * |        |          |07 = UART0_nRTS.
     * |        |          |08 = LCD_PIN18.
     * |        |          |09 = I2C1_SMBSUS.
     * |        |          |10 = PBUF.
     * |        |          |14 = TM1.
     * |        |          |15 = INT2.
     * |[31:28] |PC7MFP    |PC.7 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = SPI1_MISO.
     * |        |          |07 = UART0_nCTS.
     * |        |          |08 = LCD_PIN17.
     * |        |          |09 = I2C1_SMBAL.
     * |        |          |10 = PBUF.
     * |        |          |14 = TM0.
     * |        |          |15 = INT3.
     * @var SYS_T::GPC_MFPH
     * Offset: 0x44  GPIOC High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PC8MFP    |PC.8 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |04 = I2C0_SDA.
     * |        |          |08 = LCD_PIN16.
     * |        |          |09 = UART1_RXD.
     * |        |          |10 = PBUF.
     * |[7:4]   |PC9MFP    |PC.9 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |08 = LCD_PIN7.
     * |        |          |10 = PBUF.
     * |[11:8]  |PC10MFP   |PC.10 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |08 = LCD_PIN6.
     * |        |          |10 = PBUF.
     * |[15:12] |PC11MFP   |PC.11 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |03 = UART0_RXD.
     * |        |          |04 = I2C0_SDA.
     * |        |          |08 = LCD_PIN5.
     * |        |          |10 = PBUF.
     * |        |          |14 = ACMP1_O.
     * |[19:16] |PC12MFP   |PC.12 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |03 = UART0_TXD.
     * |        |          |04 = I2C0_SCL.
     * |        |          |08 = LCD_PIN4.
     * |        |          |10 = PBUF.
     * |        |          |14 = ACMP0_O.
     * |[27:24] |PC14MFP   |PC.14 Multi-function Pin Selection
     * |        |          |02 = PINV.
     * |        |          |05 = USCI0_CTL0.
     * |        |          |08 = LCD_PIN39.
     * |        |          |10 = PBUF.
     * |        |          |13 = TM1.
     * @var SYS_T::GPD_MFPL
     * Offset: 0x48  GPIOD Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PD0MFP    |PD.0 Multi-function Pin Selection
     * |        |          |03 = USCI0_CLK.
     * |        |          |04 = SPI0_MOSI.
     * |        |          |06 = I2C2_SDA.
     * |        |          |08 = LCD_PIN33.
     * |        |          |14 = TM2.
     * |[7:4]   |PD1MFP    |PD.1 Multi-function Pin Selection
     * |        |          |03 = USCI0_DAT0.
     * |        |          |04 = SPI0_MISO.
     * |        |          |06 = I2C2_SCL.
     * |        |          |08 = LCD_PIN32.
     * |[11:8]  |PD2MFP    |PD.2 Multi-function Pin Selection
     * |        |          |03 = USCI0_DAT1.
     * |        |          |04 = SPI0_CLK.
     * |        |          |08 = LCD_PIN31.
     * |        |          |09 = UART0_RXD.
     * |[15:12] |PD3MFP    |PD.3 Multi-function Pin Selection
     * |        |          |03 = USCI0_CTL1.
     * |        |          |04 = SPI0_SS.
     * |        |          |08 = LCD_PIN30.
     * |        |          |09 = UART0_TXD.
     * |[19:16] |PD4MFP    |PD.4 Multi-function Pin Selection
     * |        |          |03 = USCI0_CTL0.
     * |        |          |04 = I2C1_SDA.
     * |        |          |05 = SPI1_SS.
     * |[23:20] |PD5MFP    |PD.5 Multi-function Pin Selection
     * |        |          |04 = I2C1_SCL.
     * |        |          |05 = SPI1_CLK.
     * |[27:24] |PD6MFP    |PD.6 Multi-function Pin Selection
     * |        |          |03 = UART1_RXD.
     * |        |          |04 = I2C0_SDA.
     * |        |          |05 = SPI1_MOSI.
     * |[31:28] |PD7MFP    |PD.7 Multi-function Pin Selection
     * |        |          |03 = UART1_TXD.
     * |        |          |04 = I2C0_SCL.
     * |        |          |05 = SPI1_MISO.
     * @var SYS_T::GPD_MFPH
     * Offset: 0x4C  GPIOD High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PD8MFP    |PD.8 Multi-function Pin Selection
     * |        |          |03 = I2C2_SDA.
     * |        |          |04 = UART2_nRTS.
     * |        |          |08 = LCD_PIN23.
     * |[7:4]   |PD9MFP    |PD.9 Multi-function Pin Selection
     * |        |          |03 = I2C2_SCL.
     * |        |          |04 = UART2_nCTS.
     * |        |          |08 = LCD_PIN22.
     * |[11:8]  |PD10MFP   |PD.10 Multi-function Pin Selection
     * |        |          |03 = UART1_RXD.
     * |        |          |08 = LCD_PIN10.
     * |        |          |15 = INT7.
     * |[15:12] |PD11MFP   |PD.11 Multi-function Pin Selection
     * |        |          |03 = UART1_TXD.
     * |        |          |08 = LCD_PIN9.
     * |        |          |15 = INT6.
     * |[19:16] |PD12MFP   |PD.12 Multi-function Pin Selection
     * |        |          |07 = UART2_RXD.
     * |        |          |09 = BPWM0_CH5.
     * |        |          |13 = CLKO.
     * |        |          |14 = ADC0_ST.
     * |        |          |15 = INT5.
     * |[23:20] |PD13MFP   |PD.13 Multi-function Pin Selection
     * |        |          |08 = LCD_PIN34.
     * |        |          |11 = BPWM0_CH0.
     * |        |          |14 = CLKO.
     * |[31:28] |PD15MFP   |PD.15 Multi-function Pin Selection
     * @var SYS_T::GPE_MFPL
     * Offset: 0x50  GPIOE Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PE0MFP    |PE.0 Multi-function Pin Selection
     * |        |          |06 = SPI1_MOSI.
     * |        |          |07 = I2C1_SDA.
     * |[7:4]   |PE1MFP    |PE.1 Multi-function Pin Selection
     * |        |          |06 = SPI1_MISO.
     * |        |          |07 = I2C1_SCL.
     * |[11:8]  |PE2MFP    |PE.2 Multi-function Pin Selection
     * |        |          |07 = USCI0_CLK.
     * |        |          |13 = BPWM0_CH0.
     * |[15:12] |PE3MFP    |PE.3 Multi-function Pin Selection
     * |        |          |07 = USCI0_DAT0.
     * |        |          |13 = BPWM0_CH1.
     * |[19:16] |PE4MFP    |PE.4 Multi-function Pin Selection
     * |        |          |07 = USCI0_DAT1.
     * |        |          |13 = BPWM0_CH2.
     * |        |          |15 = SPI1_MOSI.
     * |[23:20] |PE5MFP    |PE.5 Multi-function Pin Selection
     * |        |          |03 = UART0_RXD.
     * |        |          |04 = I2C1_SMBSUS.
     * |        |          |05 = SPI2_MOSI.
     * |        |          |06 = I2C2_SDA.
     * |        |          |07 = USCI0_CTL1.
     * |        |          |09 = I2C0_SMBSUS.
     * |        |          |13 = BPWM0_CH3.
     * |        |          |15 = SPI1_MISO.
     * |[27:24] |PE6MFP    |PE.6 Multi-function Pin Selection
     * |        |          |03 = UART0_TXD.
     * |        |          |04 = I2C1_SMBAL.
     * |        |          |05 = SPI2_MISO.
     * |        |          |06 = I2C2_SCL.
     * |        |          |07 = USCI0_CTL0.
     * |        |          |08 = LCD_PIN36.
     * |        |          |09 = I2C0_SMBAL.
     * |        |          |13 = BPWM0_CH4.
     * |[31:28] |PE7MFP    |PE.7 Multi-function Pin Selection
     * |        |          |04 = I2C1_SDA.
     * |        |          |05 = SPI2_CLK.
     * |        |          |08 = LCD_PIN35.
     * |        |          |13 = BPWM0_CH5.
     * @var SYS_T::GPE_MFPH
     * Offset: 0x54  GPIOE High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PE8MFP    |PE.8 Multi-function Pin Selection
     * |        |          |05 = SPI2_CLK.
     * |        |          |07 = UART2_TXD.
     * |        |          |08 = LCD_PIN11.
     * |        |          |11 = PWM0_BRAKE0.
     * |[7:4]   |PE9MFP    |PE.9 Multi-function Pin Selection
     * |        |          |05 = SPI2_MISO.
     * |        |          |07 = UART2_RXD.
     * |        |          |08 = LCD_PIN12.
     * |        |          |11 = PWM0_BRAKE1.
     * |[11:8]  |PE10MFP   |PE.10 Multi-function Pin Selection
     * |        |          |05 = SPI2_MOSI.
     * |        |          |08 = LCD_PIN13.
     * |[15:12] |PE11MFP   |PE.11 Multi-function Pin Selection
     * |        |          |05 = SPI2_SS.
     * |        |          |08 = LCD_PIN14.
     * |        |          |09 = UART1_nCTS.
     * |[19:16] |PE12MFP   |PE.12 Multi-function Pin Selection
     * |        |          |09 = UART1_nRTS.
     * |[23:20] |PE13MFP   |PE.13 Multi-function Pin Selection
     * |        |          |04 = I2C0_SCL.
     * |        |          |08 = LCD_PIN15.
     * |        |          |09 = UART1_TXD.
     * |[27:24] |PE14MFP   |PE.14 Multi-function Pin Selection
     * |        |          |03 = UART2_TXD.
     * |        |          |08 = LCD_PIN21.
     * |        |          |09 = I2C1_SCL.
     * |[31:28] |PE15MFP   |PE.15 Multi-function Pin Selection
     * |        |          |03 = UART2_RXD.
     * |        |          |09 = I2C1_SDA.
     * @var SYS_T::GPF_MFPL
     * Offset: 0x58  GPIOF Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PF0MFP    |PF.0 Multi-function Pin Selection
     * |        |          |02 = UART1_TXD.
     * |        |          |03 = I2C1_SCL.
     * |        |          |04 = UART0_TXD.
     * |        |          |07 = USCI0_CTL1.
     * |        |          |08 = UART2_TXD.
     * |        |          |09 = I2C0_SCL.
     * |        |          |13 = ACMP0_O.
     * |        |          |14 = ICE_DAT.
     * |[7:4]   |PF1MFP    |PF.1 Multi-function Pin Selection
     * |        |          |02 = UART1_RXD.
     * |        |          |03 = I2C1_SDA.
     * |        |          |04 = UART0_RXD.
     * |        |          |07 = USCI0_DAT1.
     * |        |          |08 = UART2_RXD.
     * |        |          |09 = I2C0_SDA.
     * |        |          |13 = ACMP1_O.
     * |        |          |14 = ICE_CLK.
     * |[11:8]  |PF2MFP    |PF.2 Multi-function Pin Selection
     * |        |          |03 = UART0_RXD.
     * |        |          |04 = I2C0_SDA.
     * |        |          |08 = I2C2_SMBSUS.
     * |        |          |15 = TM1_EXT.
     * |[15:12] |PF3MFP    |PF.3 Multi-function Pin Selection
     * |        |          |03 = UART0_TXD.
     * |        |          |04 = I2C0_SCL.
     * |        |          |08 = I2C2_SMBAL.
     * |        |          |15 = TM0_EXT.
     * |[19:16] |PF4MFP    |PF.4 Multi-function Pin Selection
     * |        |          |02 = UART2_TXD.
     * |        |          |04 = UART2_nRTS.
     * |        |          |05 = UART0_nRTS.
     * |        |          |08 = BPWM0_CH5.
     * |        |          |10 = X32_OUT.
     * |[23:20] |PF5MFP    |PF.5 Multi-function Pin Selection
     * |        |          |02 = UART2_RXD.
     * |        |          |04 = UART2_nCTS.
     * |        |          |05 = UART0_nCTS.
     * |        |          |08 = BPWM0_CH4.
     * |        |          |10 = X32_IN.
     * |[27:24] |PF6MFP    |PF.6 Multi-function Pin Selection
     * |        |          |05 = SPI0_MOSI.
     * |[31:28] |PF7MFP    |PF.7 Multi-function Pin Selection
     * |        |          |05 = SPI0_MISO.
     * @var SYS_T::GPF_MFPH
     * Offset: 0x5C  GPIOF High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PF8MFP    |PF.8 Multi-function Pin Selection
     * |        |          |05 = SPI0_CLK.
     * |[7:4]   |PF9MFP    |PF.9 Multi-function Pin Selection
     * |        |          |05 = SPI0_SS.
     * |[27:24] |PF14MFP   |PF.14 Multi-function Pin Selection
     * @var SYS_T::GPG_MFPL
     * Offset: 0x60  GPIOG Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:8]  |PG2MFP    |PG.2 Multi-function Pin Selection
     * |        |          |03 = SPI2_SS.
     * |        |          |04 = I2C0_SMBAL.
     * |        |          |05 = I2C1_SCL.
     * |        |          |13 = TM0.
     * |[15:12] |PG3MFP    |PG.3 Multi-function Pin Selection
     * |        |          |03 = SPI2_CLK.
     * |        |          |04 = I2C0_SMBSUS.
     * |        |          |05 = I2C1_SDA.
     * |        |          |13 = TM1.
     * |[19:16] |PG4MFP    |PG.4 Multi-function Pin Selection
     * |        |          |03 = SPI2_MISO.
     * |        |          |13 = TM2.
     * @var SYS_T::GPH_MFPH
     * Offset: 0x6C  GPIOH High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PH8MFP    |PH.8 Multi-function Pin Selection
     * |        |          |06 = SPI1_CLK.
     * |        |          |07 = I2C1_SMBAL.
     * |        |          |08 = LCD_PIN37.
     * |        |          |09 = I2C2_SCL.
     * |        |          |10 = UART1_TXD.
     * |[7:4]   |PH9MFP    |PH.9 Multi-function Pin Selection
     * |        |          |06 = SPI1_SS.
     * |        |          |07 = I2C1_SMBSUS.
     * |        |          |08 = LCD_PIN38.
     * |        |          |09 = I2C2_SDA.
     * |        |          |10 = UART1_RXD.
     * @var SYS_T::GPA_MFOS
     * Offset: 0x80  GPIOA Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MFOS      |GPIOA-H Pin[n] Multiple Function Pin Output Mode Select
     * |        |          |This bit is used to select multiple function output mode type for Px.0~15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: MFOS[n] is used to select multiple function output mode type for Px.n pin.
     * |        |          |Note 2: For more information about Px.n, please refer to the "PIN CONFIGURATION" chapter.
     * @var SYS_T::GPB_MFOS
     * Offset: 0x84  GPIOB Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MFOS      |GPIOA-H Pin[n] Multiple Function Pin Output Mode Select
     * |        |          |This bit is used to select multiple function output mode type for Px.0~15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: MFOS[n] is used to select multiple function output mode type for Px.n pin.
     * |        |          |Note 2: For more information about Px.n, please refer to the "PIN CONFIGURATION" chapter.
     * @var SYS_T::GPC_MFOS
     * Offset: 0x88  GPIOC Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MFOS      |GPIOA-H Pin[n] Multiple Function Pin Output Mode Select
     * |        |          |This bit is used to select multiple function output mode type for Px.0~15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: MFOS[n] is used to select multiple function output mode type for Px.n pin.
     * |        |          |Note 2: For more information about Px.n, please refer to the "PIN CONFIGURATION" chapter.
     * @var SYS_T::GPD_MFOS
     * Offset: 0x8C  GPIOD Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MFOS      |GPIOA-H Pin[n] Multiple Function Pin Output Mode Select
     * |        |          |This bit is used to select multiple function output mode type for Px.0~15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: MFOS[n] is used to select multiple function output mode type for Px.n pin.
     * |        |          |Note 2: For more information about Px.n, please refer to the "PIN CONFIGURATION" chapter.
     * @var SYS_T::GPE_MFOS
     * Offset: 0x90  GPIOE Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MFOS      |GPIOA-H Pin[n] Multiple Function Pin Output Mode Select
     * |        |          |This bit is used to select multiple function output mode type for Px.0~15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: MFOS[n] is used to select multiple function output mode type for Px.n pin.
     * |        |          |Note 2: For more information about Px.n, please refer to the "PIN CONFIGURATION" chapter.
     * @var SYS_T::GPF_MFOS
     * Offset: 0x94  GPIOF Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MFOS      |GPIOA-H Pin[n] Multiple Function Pin Output Mode Select
     * |        |          |This bit is used to select multiple function output mode type for Px.0~15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: MFOS[n] is used to select multiple function output mode type for Px.n pin.
     * |        |          |Note 2: For more information about Px.n, please refer to the "PIN CONFIGURATION" chapter.
     * @var SYS_T::GPG_MFOS
     * Offset: 0x98  GPIOG Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MFOS      |GPIOA-H Pin[n] Multiple Function Pin Output Mode Select
     * |        |          |This bit is used to select multiple function output mode type for Px.0~15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: MFOS[n] is used to select multiple function output mode type for Px.n pin.
     * |        |          |Note 2: For more information about Px.n, please refer to the "PIN CONFIGURATION" chapter.
     * @var SYS_T::GPH_MFOS
     * Offset: 0x9C  GPIOH Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MFOS      |GPIOA-H Pin[n] Multiple Function Pin Output Mode Select
     * |        |          |This bit is used to select multiple function output mode type for Px.0~15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: MFOS[n] is used to select multiple function output mode type for Px.n pin.
     * |        |          |Note 2: For more information about Px.n, please refer to the "PIN CONFIGURATION" chapter.
     * @var SYS_T::MIRCTCTL
     * Offset: 0xB0  MIRC Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |AUTRIMEN  |Auto Trim Enable Bit
     * |        |          |0 = MIRC auto trim function Disable.
     * |        |          |1 = MIRC auto trim function Enable.
     * |[3:2]   |ACCURSEL  |Trim Accuracy Selection
     * |        |          |This field defines the target frequency accuracy of MIRC auto trim.
     * |        |          |00 = Accuracy is +-0.25% deviation.
     * |        |          |01 = Accuracy is +-0.50% deviation.
     * |        |          |10 = Accuracy is +-0.75% deviation.
     * |        |          |11 = Accuracy is +-1% deviation.
     * |        |          |Note: The accuracy is not include frequency drift of clock sources.
     * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection
     * |        |          |This field defines that trim value calculation is based on how many reference clocks.
     * |        |          |00 = Trim value calculation is based on average difference in 32 reference clocks.
     * |        |          |01 = Trim value calculation is based on average difference in 64 reference clocks.
     * |        |          |10 = Trim value calculation is based on average difference in 96 reference clocks.
     * |        |          |11 = Trim value calculation is based on average difference in 128 reference clocks.
     * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count
     * |        |          |This field defines that how many times the auto trim circuit will try to update the MIRC trim value before the frequency of MIRC locked
     * |        |          |If auto trim can't be locked within the RETRYCNT, TFAILIF (MIRCTISTS[1]) will be set.
     * |        |          |00 = Trim retry count limitation is 64 loops.
     * |        |          |01 = Trim retry count limitation is 128 loops.
     * |        |          |10 = Trim retry count limitation is 256 loops.
     * |        |          |11 = Trim retry count limitation is 512 loops.
     * |[8]     |CESTOPEN  |Clock Error Stop Enable Bit
     * |        |          |0 = When CLKERRIF (MIRCTISTS[2]) is set, the trim operation is keep going.
     * |        |          |1 = When CLKERRIF (MIRCTISTS[2]) is set, the trim operation is stopped and AUTRIMEN (MIRCTCTL[0]) will also be cleared to 0.
     * |[9]     |BOUNDEN   |Boundary Enable Bit
     * |        |          |0 = Boundary function Disabled.
     * |        |          |1 = Boundary function Enabled.
     * |[10]    |REFCKSEL  |Reference Clock Selection
     * |        |          |0 = MIRC trim reference clock is from LXT.
     * |        |          |1 = MIRC trim reference clock is from LIRC.
     * |[12]    |TFSTOPEN  |Trim Fail Stop Enable Bit
     * |        |          |0 = When TFAILIF (MIRCTISTS[1]) is set, the trim operation is keep going.
     * |        |          |1 = When TFAILIF (MIRCTISTS[1]) is set, the trim operation is stopped and AUTRIMEN (MIRCTCTL[0]) will also be cleared to 0.
     * |[13]    |AUTOEN    |MIRC Auto Operation Function Enable Bit
     * |        |          |0 = MIRC auto trim in NPD0/1 Disable.
     * |        |          |1 = MIRC auto trim in NPD0/1 Enable.
     * |[20:16] |BOUNDARY  |Boundary Selection
     * |        |          |This field defines the update value range
     * |        |          |If difference between previous and current trim value is larger than the BOUNDARY range, the current trim value will not be adopted and MIRC will keep the previous trim value.
     * |        |          |Fill the boundary range from 0x1 to 0x31, 0x0 is reserved.
     * |        |          |Note: This field is effective only when the BOUNDEN(SYS_MIRCTCTL[9]) is enabled.
     * @var SYS_T::MIRCTIEN
     * Offset: 0xB4  MIRC Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TFAILIEN  |Trim Failure Interrupt Enable Bit
     * |        |          |This field is TFAILIF (SYS_MIRCTISTS[1]) interrupt enable control bit.
     * |        |          |If this bit set to 1, and TFAILIF is alerted during auto trim operation, an interrupt will be generated.
     * |        |          |0 = Disable TFAILIF flag to generate an interrupt to CPU.
     * |        |          |1 = Enable TFAILIF flag to generate an interrupt to CPU.
     * |[2]     |CLKEIEN   |Clock Error Interrupt Enable Bit
     * |        |          |This field is CLKERRIF (SYS_MIRCTISTS[2]) interrupt enable control bit.
     * |        |          |If this bit set to 1, and CLKERRIF is alerted during auto trim operation, an interrupt will be generated.
     * |        |          |0 = Disable CLKERRIF flag to trigger an interrupt to CPU.
     * |        |          |1 = Enable CLKERRIF flag to trigger an interrupt to CPU.
     * |[4]     |CLKWKEN   |Clock Error Wakeup Enable Bit
     * |        |          |This field is CLKERRIF (SYS_MIRCTISTS[2]) or TFAILIF (SYS_MIRCTISTS[1]) wakeup enable control bit.
     * |        |          |If this bit set to 1, and CLKERRIF is alerted during auto operation, chip will be waked up from NPD0/1 mode.
     * |        |          |0 = Disable CLKERRIF wakeup function.
     * |        |          |1 = Enable CLKERRIF wakeup function.
     * @var SYS_T::MIRCTISTS
     * Offset: 0xB8  MIRC Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQLOCK  |MIRC Frequency Lock Status
     * |        |          |This bit indicates the MIRC frequency is locked or not
     * |        |          |This flag will keep update during the auto trim operation.
     * |        |          |0 = The MIRC frequency doesn't lock.
     * |        |          |1 = The MIRC frequency locked.
     * |        |          |Note: Software can write1 to clear this bit.
     * |[1]     |TFAILIF   |Trim Fail Flag Status
     * |        |          |This bit indicates that auto trim can't be locked within RETRYCNT (SYS_MIRCTCTL[7:6]), or the reference clock source is selected to LIRC but not high-accuracy mode.
     * |        |          |0 = Trim fail condition did not occur.
     * |        |          |1 = Trim fail condition occured.
     * |        |          |Note: Software can write1 to clear this bit.
     * |[2]     |CLKERRIF  |Clock Error Flag Status
     * |        |          |This bit indicates that the clock source is stopped.
     * |        |          |0 = Clock frequency is not stopped.
     * |        |          |1 = Clock frequency is stopped.
     * |        |          |Note: Software can write1 to clear this bit.
     * |[3]     |OVBDIF    |Over Boundary Status
     * |        |          |This bit indicates that the difference between current and previous trim value is larger than BOUNDARY range.
     * |        |          |0 = Over boundary condition did not occur.
     * |        |          |1 = Over boundary condition occurred.
     * |        |          |Note 1: Software can write 1 to clear this flag.
     * |        |          |Note 2: This field is effective only when the BOUNDEN (SYS_MIRCTCTL[9]) is enabled.
     * @var SYS_T::SRAM_BISTCTL
     * Offset: 0xD0  System SRAM BIST Test Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SRBIST0   |SRAM Bank0 BIST Enable Bit (Write Protect)
     * |        |          |This bit enables BIST test for SRAM bank0.
     * |        |          |0 = system SRAM bank0 BIST Disabled.
     * |        |          |1 = system SRAM bank0 BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |CRBIST    |CACHE BIST Enable Bit (Write Protect)
     * |        |          |This bit enables BIST test for CACHE RAM.
     * |        |          |0 = system CACHE BIST Disabled.
     * |        |          |1 = system CACHE BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::SRAM_BISTSTS
     * Offset: 0xD4  System SRAM BIST Test Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SRBISTEF0 |1st System SRAM BIST Fail Flag
     * |        |          |0 = 1st system SRAM BIST test pass.
     * |        |          |1 = 1st system SRAM BIST test fail.
     * |[2]     |CRBISTEF  |CACHE SRAM BIST Fail Flag
     * |        |          |0 = System CACHE RAM BIST test pass.
     * |        |          |1 = System CACHE RAM BIST test fail.
     * |[16]    |SRBEND0   |1st SRAM BIST Test Finish
     * |        |          |0 = 1st system SRAM BIST active.
     * |        |          |1 =1st system SRAM BIST finish.
     * |[18]    |CRBEND    |CACHE SRAM BIST Test Finish
     * |        |          |0 = System CACHE RAM BIST is active.
     * |        |          |1 = System CACHE RAM BIST test finish.
     * @var SYS_T::HIRCTCTL
     * Offset: 0xF0  HIRC Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |AUTRIMEN  |Auto Trim Enable Bit
     * |        |          |0 = HIRC auto trim function Disable.
     * |        |          |1 = HIRC auto trim function Enable.
     * |[3:2]   |ACCURSEL  |Trim Accuracy Selection
     * |        |          |This field defines the target frequency accuracy of HIRC auto trim.
     * |        |          |00 = Accuracy is +-0.25% deviation.
     * |        |          |01 = Accuracy is +-0.50% deviation.
     * |        |          |10 = Accuracy is +-0.75% deviation.
     * |        |          |11 = Accuracy is +-1% deviation.
     * |        |          |Note: The accuracy is not include frequency drift of clock sources.
     * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection
     * |        |          |This field defines that trim value calculation is based on how many reference clocks.
     * |        |          |00 = Trim value calculation is based on average difference in 32 reference clocks.
     * |        |          |01 = Trim value calculation is based on average difference in 64 reference clocks.
     * |        |          |10 = Trim value calculation is based on average difference in 128 reference clocks.
     * |        |          |11 = Trim value calculation is based on average difference in 256 reference clocks.
     * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count
     * |        |          |This field defines that how many times the auto trim circuit will try to update the HIRC trim value before the frequency of HIRC locked
     * |        |          |If auto trim can't be locked within the RETRYCNT, TFAILIF (SYS_HIRCTISTS[1]) will be set.
     * |        |          |00 = Trim retry count limitation is 64 loops.
     * |        |          |01 = Trim retry count limitation is 128 loops.
     * |        |          |10 = Trim retry count limitation is 256 loops.
     * |        |          |11 = Trim retry count limitation is 512 loops.
     * |[8]     |CESTOPEN  |Clock Error Stop Enable Bit
     * |        |          |0 = When CLKERRIF (SYS_HIRCTISTS[2]) is set, the trim operation is keep going.
     * |        |          |1 = When CLKERRIF (SYS_HIRCTISTS[2]) is set, the trim operation is stopped and AUTRIMEN (IRCTRIMCTL[0]) will also be cleared to 0.
     * |[9]     |BOUNDEN   |Boundary Enable Bit
     * |        |          |0 = Boundary function Disabled.
     * |        |          |1 = Boundary function Enabled.
     * |[10]    |REFCKSEL  |Reference Clock Selection
     * |        |          |0 = HIRC trim reference clock is from LXT.
     * |        |          |1 = HIRC trim reference clock is from LIRC.
     * |[12]    |TFSTOPEN  |Trim Fail Stop Enable Bit
     * |        |          |0 = When TFAILIF (SYS_HIRCTISTS[1]) is set, the trim operation is keep going.
     * |        |          |1 = When TFAILIF (SYS_HIRCTISTS[1]) is set, the trim operation is stopped and AUTRIMEN (IRCTRIMCTL[0]) will also be cleared to 0.
     * |[20:16] |BOUNDARY  |Boundary Selection
     * |        |          |This field defines the update value range
     * |        |          |If difference between previous and current trim value is larger than the BOUNDARY range, the current trim value will not be adopted and HIRC will keep the previous trim value.
     * |        |          |Fill the boundary range from 0x1 to 0x31, 0x0 is reserved.
     * |        |          |Note: This field is effective only when the BOUNDEN(SYS_HIRCTCTL[9]) is enabled.
     * @var SYS_T::HIRCTIEN
     * Offset: 0xF4  HIRC Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TFAILIEN  |Trim Failure Interrupt Enable Bit
     * |        |          |This field is TFAILIF (SYS_HIRCTISTS[1]) interrupt enable control bit.
     * |        |          |If this bit set to 1, and TFAILIF is alerted during auto trim operation, an interrupt will be generated.
     * |        |          |0 = Disable TFAILIF flag to generate an interrupt to CPU.
     * |        |          |1 = Enable TFAILIF flag to generate an interrupt to CPU.
     * |[2]     |CLKEIEN   |Clock Error Interrupt Enable Bit
     * |        |          |This field is CLKERRIF (SYS_HIRCTISTS[2]) interrupt enable control bit.
     * |        |          |If this bit set to 1, and CLKERRIF is alerted during auto trim operation, an interrupt will be generated.
     * |        |          |0 = Disable CLKERRIF flag to trigger an interrupt to CPU.
     * |        |          |1 = Enable CLKERRIF flag to trigger an interrupt to CPU.
     * @var SYS_T::HIRCTISTS
     * Offset: 0xF8  HIRC Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQLOCK  |HIRC Frequency Lock Status
     * |        |          |This bit indicates the HIRC frequency is locked or not
     * |        |          |This flag will keep update during the auto trim operation.
     * |        |          |0 = The HIRC frequency doesn't lock.
     * |        |          |1 = The HIRC frequency locked.
     * |        |          |Note: Software can write1 to clear this bit.
     * |[1]     |TFAILIF   |Trim Fail Flag Status
     * |        |          |This bit indicates that auto trim can't be locked within RETRYCNT (SYS_HIRCTCTL[7:6]), or the reference clock source is selected to LIRC but not high-accuracy mode.
     * |        |          |0 = Trim fail condition did not occur.
     * |        |          |1 = Trim fail condition occured.
     * |        |          |Note: Software can write1 to clear this bit.
     * |[2]     |CLKERRIF  |Clock Error Flag Status
     * |        |          |This bit indicates that the clock source is stopped.
     * |        |          |0 = Clock frequency is not stopped.
     * |        |          |1 = Clock frequency is stopped.
     * |        |          |Note: Software can write1 to clear this bit.
     * |[3]     |OVBDIF    |Over Boundary Status
     * |        |          |This bit indicates that the difference between current and previous trim value is larger than BOUNDARY range.
     * |        |          |0 = Over boundary condition did not occur.
     * |        |          |1 = Over boundary condition occurred.
     * |        |          |Note 1: Software can write 1 to clear this flag.
     * |        |          |Note 2: This field is effective only when the BOUNDEN (SYS_HIRCTCTL[9]) is enabled.
     * @var SYS_T::REGLCTL
     * Offset: 0x100  Register Lock Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |REGLCTL   |Register Lock Control Code (Write Only)
     * |        |          |Some registers have write-protection function
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value "59h", "16h", "88h" to this field
     * |        |          |After this sequence is completed, the REGLCTL bit will be set to 1 and write-protection registers can be normal write.
     * |        |          |REGLCTL[0]
     * |        |          |Register Lock Control Disable Index (Read Only)
     * |        |          |0 = Write-protection Enabled for writing protected registers
     * |        |          |Any write to the protected register is ignored.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * @var SYS_T::PORDISAN
     * Offset: 0x1EC  Analog POR Disable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |POROFFAN  |Power-on Reset Enable Bit (Write Protect)
     * |        |          |After powered on, User can turn off internal analog POR circuit to save power by writing 0x5AA5 to this field.
     * |        |          |The analog POR circuit will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::CSERVER
     * Offset: 0x1F4  Chip Series Version Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |VERSION   |Chip Series Version
     * |        |          |These bits indicate the series version of chip.
     * |        |          |00 = M2U5xG/M2U5xE/ M2U5xD
     * |        |          |01 = M2U51xC
     * |        |          |Others = Reserved.
     * @var SYS_T::PLCTL
     * Offset: 0x1F8  Power Level Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |MVRS      |Main Voltage Regulator Type Select (Write Protect)
     * |        |          |This bit field sets main voltage regulator type.
     * |        |          |0 = Set main voltage regulator to LDO.
     * |        |          |1 = Set main voltage regulator to DCDC.
     * |        |          |After setting main voltage regulator type to DCDC, system will set main voltage regulator type change busy flag MVRCBUSY(SYS_PLSTS[1]), detect inductor connection and update inductor connection status LCONS (SYS_PLSTS[3])
     * |        |          |if inductor exist LCONS will be cleared and main voltage regulator type can switch to DCDC (CURMVR (SYS_PLSTS[12])=1).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit not be reset when wake-up from SPD mode
     * |        |          |Note 3: Set NRLDCSEL(CLK_PMUCTL[16])=0 before change MVRS between LDO and DCDC.
     * @var SYS_T::PLSTS
     * Offset: 0x1FC  Power Level Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |MVRCBUSY  |Main Voltage Regulator Type Change Busy Bit (Read Only)
     * |        |          |This bit is set by hardware when main voltage regulator type is changing
     * |        |          |After main voltage regulator type change is completed, this bit will be cleared automatically by hardware.
     * |        |          |0 = Main voltage regulator type change is completed.
     * |        |          |1 = Main voltage regulator type change is ongoing.
     * |[2]     |MVRCERR   |Main Voltage Regulator Type Change Error Bit (Write Protect)
     * |        |          |This bit is set to 1 when main voltage regulator type change from LDO to DCDC error.
     * |        |          |0 = No main voltage regulator type change error.
     * |        |          |1 = Main voltage regulator type change to DCDC error occurred.
     * |        |          |The following conditions will cause change errors.
     * |        |          |- System changed to DC-DC mode but LDO change voltage process not finish.
     * |        |          |- Detect inductor failed.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write 1 to clear this bit to 0.
     * |[3]     |LCONS     |Inductor for DCDC Connect Status (Read Only)
     * |        |          |This bit is valid whencurrent main voltage regulator type is DCDC (CURMVR (SYS_PLSTS[12])=1)
     * |        |          |If current main voltage regulator type is LDO (CURMVR (SYS_PLSTS[12])=0) this bit is set to 1.
     * |        |          |0 = Inductor connect between Vsw and LDO_CAP pin.
     * |        |          |1 = No Inductor connect between Vsw and LDO_CAP pin.
     * |        |          |Note: This bit is 1 when main voltage regulator is LDO.
     * |[12]    |CURMVR    |Current Main Voltage Regulator Type (Read Only)
     * |        |          |This bit field reflects current main voltage regulator type.
     * |        |          |0 = Current main voltage regulator in active and Idle mode is LDO.
     * |        |          |1 = Current main voltage regulator in active and Idle mode is DCDC.
     * @var SYS_T::INIVTOR
     * Offset: 0x310  Initial VTOR Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:10] |INIVTOR   |Initial VTOR Control Register
     * |        |          |This is the register to set the address of vector table after CPU is reset or chip wakes up from SPD0 mode.
     * |        |          |The value will be loaded to Vector Table Offset Register, which is at the address 0xE000ED08, when CPU is reset or chip wakes up from SPD0 mode.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     */

    __I  uint32_t PDID;             /*!< [0x0000] Part Device Identification Number Register                */
    __IO uint32_t RSTSTS;           /*!< [0x0004] System Reset Status Register                              */
    __IO uint32_t IPRST0;           /*!< [0x0008] Peripheral Reset Control Register 0                       */
    __IO uint32_t IPRST1;           /*!< [0x000c] Peripheral Reset Control Register 1                       */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t BODCTL;           /*!< [0x0018] Brown-out Detector Control Register                       */
    __IO uint32_t IVSCTL;           /*!< [0x001c] Internal Voltage Source Control Register                  */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t VREFCTL;          /*!< [0x0028] VREF Control Register                                     */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t GPA_MFPL;         /*!< [0x0030] GPIOA Low Byte Multiple Function Control Register         */
    __IO uint32_t GPA_MFPH;         /*!< [0x0034] GPIOA High Byte Multiple Function Control Register        */
    __IO uint32_t GPB_MFPL;         /*!< [0x0038] GPIOB Low Byte Multiple Function Control Register         */
    __IO uint32_t GPB_MFPH;         /*!< [0x003c] GPIOB High Byte Multiple Function Control Register        */
    __IO uint32_t GPC_MFPL;         /*!< [0x0040] GPIOC Low Byte Multiple Function Control Register         */
    __IO uint32_t GPC_MFPH;         /*!< [0x0044] GPIOC High Byte Multiple Function Control Register        */
    __IO uint32_t GPD_MFPL;         /*!< [0x0048] GPIOD Low Byte Multiple Function Control Register         */
    __IO uint32_t GPD_MFPH;         /*!< [0x004c] GPIOD High Byte Multiple Function Control Register        */
    __IO uint32_t GPE_MFPL;         /*!< [0x0050] GPIOE Low Byte Multiple Function Control Register         */
    __IO uint32_t GPE_MFPH;         /*!< [0x0054] GPIOE High Byte Multiple Function Control Register        */
    __IO uint32_t GPF_MFPL;         /*!< [0x0058] GPIOF Low Byte Multiple Function Control Register         */
    __IO uint32_t GPF_MFPH;         /*!< [0x005c] GPIOF High Byte Multiple Function Control Register        */
    __IO uint32_t GPG_MFPL;         /*!< [0x0060] GPIOG Low Byte Multiple Function Control Register         */
    __I  uint32_t RESERVE3[2];
    __IO uint32_t GPH_MFPH;         /*!< [0x006c] GPIOH High Byte Multiple Function Control Register        */
    __I  uint32_t RESERVE4[4];
    __IO uint32_t GPA_MFOS;         /*!< [0x0080] GPIOA Multiple Function Output Select Register            */
    __IO uint32_t GPB_MFOS;         /*!< [0x0084] GPIOB Multiple Function Output Select Register            */
    __IO uint32_t GPC_MFOS;         /*!< [0x0088] GPIOC Multiple Function Output Select Register            */
    __IO uint32_t GPD_MFOS;         /*!< [0x008c] GPIOD Multiple Function Output Select Register            */
    __IO uint32_t GPE_MFOS;         /*!< [0x0090] GPIOE Multiple Function Output Select Register            */
    __IO uint32_t GPF_MFOS;         /*!< [0x0094] GPIOF Multiple Function Output Select Register            */
    __IO uint32_t GPG_MFOS;         /*!< [0x0098] GPIOG Multiple Function Output Select Register            */
    __IO uint32_t GPH_MFOS;         /*!< [0x009c] GPIOH Multiple Function Output Select Register            */
    __I  uint32_t RESERVE5[4];
    __IO uint32_t MIRCTCTL;         /*!< [0x00b0] MIRC Trim Control Register                                */
    __IO uint32_t MIRCTIEN;         /*!< [0x00b4] MIRC Trim Interrupt Enable Register                       */
    __IO uint32_t MIRCTISTS;        /*!< [0x00b8] MIRC Trim Interrupt Status Register                       */
    __I  uint32_t RESERVE6[5];
    __IO uint32_t SRAM_BISTCTL;     /*!< [0x00d0] System SRAM BIST Test Control Register                    */
    __I  uint32_t SRAM_BISTSTS;     /*!< [0x00d4] System SRAM BIST Test Status Register                     */
    __I  uint32_t RESERVE7[6];
    __IO uint32_t HIRCTCTL;         /*!< [0x00f0] HIRC Trim Control Register                                */
    __IO uint32_t HIRCTIEN;         /*!< [0x00f4] HIRC Trim Interrupt Enable Register                       */
    __IO uint32_t HIRCTISTS;        /*!< [0x00f8] HIRC Trim Interrupt Status Register                       */
    __I  uint32_t RESERVE8[1];
    __O  uint32_t REGLCTL;          /*!< [0x0100] Register Lock Control Register                            */
    __I  uint32_t RESERVE9[58];
    __IO uint32_t PORDISAN;         /*!< [0x01ec] Analog POR Disable Control Register                       */
    __I  uint32_t RESERVE10[1];
    __I  uint32_t CSERVER;          /*!< [0x01f4] Chip Series Version Register                              */
    __IO uint32_t PLCTL;            /*!< [0x01f8] Power Level Control Register                              */
    __IO uint32_t PLSTS;            /*!< [0x01fc] Power Level Status Register                               */
    __I  uint32_t RESERVE11[68];
    __IO uint32_t INIVTOR;          /*!< [0x0310] Initial VTOR Control Register                             */
} SYS_T;


typedef struct
{
    __IO uint32_t NMIEN;            /*!< [0x0000] NMI Source Interrupt Enable Register                      */
    __I  uint32_t NMISTS;           /*!< [0x0004] NMI Source Interrupt Status Register                      */
    __IO uint32_t NMIMSEL;          /*!< [0x0008] NMI Multiple Source Select Control Register               */
} NMI_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

#define SYS_PDID_PDID_Pos               (0)                                         /*!< SYS_T::PDID: PDID Position                 */
#define SYS_PDID_PDID_Msk               (0xfffffffful << SYS_PDID_PDID_Pos)         /*!< SYS_T::PDID: PDID Mask                     */

#define SYS_RSTSTS_PORF_Pos             (0)                                         /*!< SYS_T::RSTSTS: PORF Position               */
#define SYS_RSTSTS_PORF_Msk             (0x1ul << SYS_RSTSTS_PORF_Pos)              /*!< SYS_T::RSTSTS: PORF Mask                   */

#define SYS_RSTSTS_PINRF_Pos            (1)                                         /*!< SYS_T::RSTSTS: PINRF Position              */
#define SYS_RSTSTS_PINRF_Msk            (0x1ul << SYS_RSTSTS_PINRF_Pos)             /*!< SYS_T::RSTSTS: PINRF Mask                  */

#define SYS_RSTSTS_WDTRF_Pos            (2)                                         /*!< SYS_T::RSTSTS: WDTRF Position              */
#define SYS_RSTSTS_WDTRF_Msk            (0x1ul << SYS_RSTSTS_WDTRF_Pos)             /*!< SYS_T::RSTSTS: WDTRF Mask                  */

#define SYS_RSTSTS_LVRF_Pos             (3)                                         /*!< SYS_T::RSTSTS: LVRF Position               */
#define SYS_RSTSTS_LVRF_Msk             (0x1ul << SYS_RSTSTS_LVRF_Pos)              /*!< SYS_T::RSTSTS: LVRF Mask                   */

#define SYS_RSTSTS_BODRF_Pos            (4)                                         /*!< SYS_T::RSTSTS: BODRF Position              */
#define SYS_RSTSTS_BODRF_Msk            (0x1ul << SYS_RSTSTS_BODRF_Pos)             /*!< SYS_T::RSTSTS: BODRF Mask                  */

#define SYS_RSTSTS_SYSRF_Pos            (5)                                         /*!< SYS_T::RSTSTS: SYSRF Position              */
#define SYS_RSTSTS_SYSRF_Msk            (0x1ul << SYS_RSTSTS_SYSRF_Pos)             /*!< SYS_T::RSTSTS: SYSRF Mask                  */

#define SYS_RSTSTS_HRESETRF_Pos         (6)                                         /*!< SYS_T::RSTSTS: HRESETRF Position           */
#define SYS_RSTSTS_HRESETRF_Msk         (0x1ul << SYS_RSTSTS_HRESETRF_Pos)          /*!< SYS_T::RSTSTS: HRESETRF Mask               */

#define SYS_RSTSTS_CPURF_Pos            (7)                                         /*!< SYS_T::RSTSTS: CPURF Position              */
#define SYS_RSTSTS_CPURF_Msk            (0x1ul << SYS_RSTSTS_CPURF_Pos)             /*!< SYS_T::RSTSTS: CPURF Mask                  */

#define SYS_RSTSTS_CPULKRF_Pos          (8)                                         /*!< SYS_T::RSTSTS: CPULKRF Position            */
#define SYS_RSTSTS_CPULKRF_Msk          (0x1ul << SYS_RSTSTS_CPULKRF_Pos)           /*!< SYS_T::RSTSTS: CPULKRF Mask                */

#define SYS_IPRST0_CHIPRST_Pos          (0)                                         /*!< SYS_T::IPRST0: CHIPRST Position            */
#define SYS_IPRST0_CHIPRST_Msk          (0x1ul << SYS_IPRST0_CHIPRST_Pos)           /*!< SYS_T::IPRST0: CHIPRST Mask                */

#define SYS_IPRST0_CPURST_Pos           (1)                                         /*!< SYS_T::IPRST0: CPURST Position             */
#define SYS_IPRST0_CPURST_Msk           (0x1ul << SYS_IPRST0_CPURST_Pos)            /*!< SYS_T::IPRST0: CPURST Mask                 */

#define SYS_IPRST0_PDMA0RST_Pos         (2)                                         /*!< SYS_T::IPRST0: PDMA0RST Position           */
#define SYS_IPRST0_PDMA0RST_Msk         (0x1ul << SYS_IPRST0_PDMA0RST_Pos)          /*!< SYS_T::IPRST0: PDMA0RST Mask               */

#define SYS_IPRST0_CRCRST_Pos           (7)                                         /*!< SYS_T::IPRST0: CRCRST Position             */
#define SYS_IPRST0_CRCRST_Msk           (0x1ul << SYS_IPRST0_CRCRST_Pos)            /*!< SYS_T::IPRST0: CRCRST Mask                 */

#define SYS_IPRST0_CRPTRST_Pos          (12)                                        /*!< SYS_T::IPRST0: CRPTRST Position            */
#define SYS_IPRST0_CRPTRST_Msk          (0x1ul << SYS_IPRST0_CRPTRST_Pos)           /*!< SYS_T::IPRST0: CRPTRST Mask                */

#define SYS_IPRST0_GPIORST_Pos          (24)                                        /*!< SYS_T::IPRST0: GPIORST Position            */
#define SYS_IPRST0_GPIORST_Msk          (0x1ul << SYS_IPRST0_GPIORST_Pos)           /*!< SYS_T::IPRST0: GPIORST Mask                */

#define SYS_IPRST1_TMR0RST_Pos          (2)                                         /*!< SYS_T::IPRST1: TMR0RST Position            */
#define SYS_IPRST1_TMR0RST_Msk          (0x1ul << SYS_IPRST1_TMR0RST_Pos)           /*!< SYS_T::IPRST1: TMR0RST Mask                */

#define SYS_IPRST1_TMR1RST_Pos          (3)                                         /*!< SYS_T::IPRST1: TMR1RST Position            */
#define SYS_IPRST1_TMR1RST_Msk          (0x1ul << SYS_IPRST1_TMR1RST_Pos)           /*!< SYS_T::IPRST1: TMR1RST Mask                */

#define SYS_IPRST1_TMR2RST_Pos          (4)                                         /*!< SYS_T::IPRST1: TMR2RST Position            */
#define SYS_IPRST1_TMR2RST_Msk          (0x1ul << SYS_IPRST1_TMR2RST_Pos)           /*!< SYS_T::IPRST1: TMR2RST Mask                */

#define SYS_IPRST1_TMR3RST_Pos          (5)                                         /*!< SYS_T::IPRST1: TMR3RST Position            */
#define SYS_IPRST1_TMR3RST_Msk          (0x1ul << SYS_IPRST1_TMR3RST_Pos)           /*!< SYS_T::IPRST1: TMR3RST Mask                */

#define SYS_IPRST1_ACMP01RST_Pos        (7)                                         /*!< SYS_T::IPRST1: ACMP01RST Position          */
#define SYS_IPRST1_ACMP01RST_Msk        (0x1ul << SYS_IPRST1_ACMP01RST_Pos)         /*!< SYS_T::IPRST1: ACMP01RST Mask              */

#define SYS_IPRST1_I2C0RST_Pos          (8)                                         /*!< SYS_T::IPRST1: I2C0RST Position            */
#define SYS_IPRST1_I2C0RST_Msk          (0x1ul << SYS_IPRST1_I2C0RST_Pos)           /*!< SYS_T::IPRST1: I2C0RST Mask                */

#define SYS_IPRST1_I2C1RST_Pos          (9)                                         /*!< SYS_T::IPRST1: I2C1RST Position            */
#define SYS_IPRST1_I2C1RST_Msk          (0x1ul << SYS_IPRST1_I2C1RST_Pos)           /*!< SYS_T::IPRST1: I2C1RST Mask                */

#define SYS_IPRST1_I2C2RST_Pos          (10)                                        /*!< SYS_T::IPRST1: I2C2RST Position            */
#define SYS_IPRST1_I2C2RST_Msk          (0x1ul << SYS_IPRST1_I2C2RST_Pos)           /*!< SYS_T::IPRST1: I2C2RST Mask                */

#define SYS_IPRST1_SPI0RST_Pos          (12)                                        /*!< SYS_T::IPRST1: SPI0RST Position            */
#define SYS_IPRST1_SPI0RST_Msk          (0x1ul << SYS_IPRST1_SPI0RST_Pos)           /*!< SYS_T::IPRST1: SPI0RST Mask                */

#define SYS_IPRST1_SPI1RST_Pos          (13)                                        /*!< SYS_T::IPRST1: SPI1RST Position            */
#define SYS_IPRST1_SPI1RST_Msk          (0x1ul << SYS_IPRST1_SPI1RST_Pos)           /*!< SYS_T::IPRST1: SPI1RST Mask                */

#define SYS_IPRST1_SPI2RST_Pos          (14)                                        /*!< SYS_T::IPRST1: SPI2RST Position            */
#define SYS_IPRST1_SPI2RST_Msk          (0x1ul << SYS_IPRST1_SPI2RST_Pos)           /*!< SYS_T::IPRST1: SPI2RST Mask                */

#define SYS_IPRST1_UART0RST_Pos         (16)                                        /*!< SYS_T::IPRST1: UART0RST Position           */
#define SYS_IPRST1_UART0RST_Msk         (0x1ul << SYS_IPRST1_UART0RST_Pos)          /*!< SYS_T::IPRST1: UART0RST Mask               */

#define SYS_IPRST1_UART1RST_Pos         (17)                                        /*!< SYS_T::IPRST1: UART1RST Position           */
#define SYS_IPRST1_UART1RST_Msk         (0x1ul << SYS_IPRST1_UART1RST_Pos)          /*!< SYS_T::IPRST1: UART1RST Mask               */

#define SYS_IPRST1_UART2RST_Pos         (18)                                        /*!< SYS_T::IPRST1: UART2RST Position           */
#define SYS_IPRST1_UART2RST_Msk         (0x1ul << SYS_IPRST1_UART2RST_Pos)          /*!< SYS_T::IPRST1: UART2RST Mask               */

#define SYS_IPRST1_USCI0RST_Pos         (20)                                        /*!< SYS_T::IPRST1: USCI0RST Position           */
#define SYS_IPRST1_USCI0RST_Msk         (0x1ul << SYS_IPRST1_USCI0RST_Pos)          /*!< SYS_T::IPRST1: USCI0RST Mask               */

#define SYS_IPRST1_WWDTRST_Pos          (21)                                        /*!< SYS_T::IPRST1: WWDTRST Position            */
#define SYS_IPRST1_WWDTRST_Msk          (0x1ul << SYS_IPRST1_WWDTRST_Pos)           /*!< SYS_T::IPRST1: WWDTRST Mask                */

#define SYS_IPRST1_PWM0RST_Pos          (22)                                        /*!< SYS_T::IPRST1: PWM0RST Position            */
#define SYS_IPRST1_PWM0RST_Msk          (0x1ul << SYS_IPRST1_PWM0RST_Pos)           /*!< SYS_T::IPRST1: PWM0RST Mask                */

#define SYS_IPRST1_BPWM0RST_Pos         (23)                                        /*!< SYS_T::IPRST1: BPWM0RST Position           */
#define SYS_IPRST1_BPWM0RST_Msk         (0x1ul << SYS_IPRST1_BPWM0RST_Pos)          /*!< SYS_T::IPRST1: BPWM0RST Mask               */

#define SYS_IPRST1_LCDRST_Pos           (24)                                        /*!< SYS_T::IPRST1: LCDRST Position             */
#define SYS_IPRST1_LCDRST_Msk           (0x1ul << SYS_IPRST1_LCDRST_Pos)            /*!< SYS_T::IPRST1: LCDRST Mask                 */

#define SYS_IPRST1_ADC0RST_Pos          (28)                                        /*!< SYS_T::IPRST1: ADC0RST Position            */
#define SYS_IPRST1_ADC0RST_Msk          (0x1ul << SYS_IPRST1_ADC0RST_Pos)           /*!< SYS_T::IPRST1: ADC0RST Mask                */

#define SYS_BODCTL_BODEN_Pos            (0)                                         /*!< SYS_T::BODCTL: BODEN Position              */
#define SYS_BODCTL_BODEN_Msk            (0x1ul << SYS_BODCTL_BODEN_Pos)             /*!< SYS_T::BODCTL: BODEN Mask                  */

#define SYS_BODCTL_BODSIFEN_Pos         (1)                                         /*!< SYS_T::BODCTL: BODSIFEN Position           */
#define SYS_BODCTL_BODSIFEN_Msk         (0x3ul << SYS_BODCTL_BODSIFEN_Pos)          /*!< SYS_T::BODCTL: BODSIFEN Mask               */

#define SYS_BODCTL_BODRSTEN_Pos         (3)                                         /*!< SYS_T::BODCTL: BODRSTEN Position           */
#define SYS_BODCTL_BODRSTEN_Msk         (0x1ul << SYS_BODCTL_BODRSTEN_Pos)          /*!< SYS_T::BODCTL: BODRSTEN Mask               */

#define SYS_BODCTL_BODIF_Pos            (4)                                         /*!< SYS_T::BODCTL: BODIF Position              */
#define SYS_BODCTL_BODIF_Msk            (0x1ul << SYS_BODCTL_BODIF_Pos)             /*!< SYS_T::BODCTL: BODIF Mask                  */

#define SYS_BODCTL_BODLPM_Pos           (5)                                         /*!< SYS_T::BODCTL: BODLPM Position             */
#define SYS_BODCTL_BODLPM_Msk           (0x1ul << SYS_BODCTL_BODLPM_Pos)            /*!< SYS_T::BODCTL: BODLPM Mask                 */

#define SYS_BODCTL_BODOUT_Pos           (6)                                         /*!< SYS_T::BODCTL: BODOUT Position             */
#define SYS_BODCTL_BODOUT_Msk           (0x1ul << SYS_BODCTL_BODOUT_Pos)            /*!< SYS_T::BODCTL: BODOUT Mask                 */

#define SYS_BODCTL_LVREN_Pos            (7)                                         /*!< SYS_T::BODCTL: LVREN Position              */
#define SYS_BODCTL_LVREN_Msk            (0x1ul << SYS_BODCTL_LVREN_Pos)             /*!< SYS_T::BODCTL: LVREN Mask                  */

#define SYS_BODCTL_BODDGSEL_Pos         (8)                                         /*!< SYS_T::BODCTL: BODDGSEL Position           */
#define SYS_BODCTL_BODDGSEL_Msk         (0x7ul << SYS_BODCTL_BODDGSEL_Pos)          /*!< SYS_T::BODCTL: BODDGSEL Mask               */

#define SYS_BODCTL_LVRDGSEL_Pos         (12)                                        /*!< SYS_T::BODCTL: LVRDGSEL Position           */
#define SYS_BODCTL_LVRDGSEL_Msk         (0x7ul << SYS_BODCTL_LVRDGSEL_Pos)          /*!< SYS_T::BODCTL: LVRDGSEL Mask               */

#define SYS_BODCTL_LVRRDY_Pos           (15)                                        /*!< SYS_T::BODCTL: LVRRDY Position             */
#define SYS_BODCTL_LVRRDY_Msk           (0x1ul << SYS_BODCTL_LVRRDY_Pos)            /*!< SYS_T::BODCTL: LVRRDY Mask                 */

#define SYS_BODCTL_BODVL_Pos            (16)                                        /*!< SYS_T::BODCTL: BODVL Position              */
#define SYS_BODCTL_BODVL_Msk            (0x7ul << SYS_BODCTL_BODVL_Pos)             /*!< SYS_T::BODCTL: BODVL Mask                  */

#define SYS_IVSCTL_VTEMPEN_Pos          (0)                                         /*!< SYS_T::IVSCTL: VTEMPEN Position            */
#define SYS_IVSCTL_VTEMPEN_Msk          (0x1ul << SYS_IVSCTL_VTEMPEN_Pos)           /*!< SYS_T::IVSCTL: VTEMPEN Mask                */

#define SYS_IVSCTL_AVDDDIV4EN_Pos       (2)                                         /*!< SYS_T::IVSCTL: AVDDDIV4EN Position         */
#define SYS_IVSCTL_AVDDDIV4EN_Msk       (0x1ul << SYS_IVSCTL_AVDDDIV4EN_Pos)        /*!< SYS_T::IVSCTL: AVDDDIV4EN Mask             */

#define SYS_VREFCTL_VREFEN_Pos          (0)                                         /*!< SYS_T::VREFCTL: VREFEN Position            */
#define SYS_VREFCTL_VREFEN_Msk          (0x1ul << SYS_VREFCTL_VREFEN_Pos)           /*!< SYS_T::VREFCTL: VREFEN Mask                */

#define SYS_VREFCTL_VREFSEL_Pos         (1)                                         /*!< SYS_T::VREFCTL: VREFSEL Position           */
#define SYS_VREFCTL_VREFSEL_Msk         (0x7ul << SYS_VREFCTL_VREFSEL_Pos)          /*!< SYS_T::VREFCTL: VREFSEL Mask               */

#define SYS_VREFCTL_PRELOADSEL_Pos      (6)                                         /*!< SYS_T::VREFCTL: PRELOADSEL Position        */
#define SYS_VREFCTL_PRELOADSEL_Msk      (0x3ul << SYS_VREFCTL_PRELOADSEL_Pos)       /*!< SYS_T::VREFCTL: PRELOADSEL Mask            */

#define SYS_VREFCTL_PRLDSTS_Pos         (10)                                        /*!< SYS_T::VREFCTL: PRLDSTS Position           */
#define SYS_VREFCTL_PRLDSTS_Msk         (0x1ul << SYS_VREFCTL_PRLDSTS_Pos)          /*!< SYS_T::VREFCTL: PRLDSTS Mask               */

#define SYS_VREFCTL_VBGFEN_Pos          (24)                                        /*!< SYS_T::VREFCTL: VBGFEN Position            */
#define SYS_VREFCTL_VBGFEN_Msk          (0x1ul << SYS_VREFCTL_VBGFEN_Pos)           /*!< SYS_T::VREFCTL: VBGFEN Mask                */

#define SYS_GPA_MFPL_PA0MFP_Pos         (0)                                         /*!< SYS_T::GPA_MFPL: PA0MFP Position           */
#define SYS_GPA_MFPL_PA0MFP_Msk         (0xful << SYS_GPA_MFPL_PA0MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA0MFP Mask               */

#define SYS_GPA_MFPL_PA1MFP_Pos         (4)                                         /*!< SYS_T::GPA_MFPL: PA1MFP Position           */
#define SYS_GPA_MFPL_PA1MFP_Msk         (0xful << SYS_GPA_MFPL_PA1MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA1MFP Mask               */

#define SYS_GPA_MFPL_PA2MFP_Pos         (8)                                         /*!< SYS_T::GPA_MFPL: PA2MFP Position           */
#define SYS_GPA_MFPL_PA2MFP_Msk         (0xful << SYS_GPA_MFPL_PA2MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA2MFP Mask               */

#define SYS_GPA_MFPL_PA3MFP_Pos         (12)                                        /*!< SYS_T::GPA_MFPL: PA3MFP Position           */
#define SYS_GPA_MFPL_PA3MFP_Msk         (0xful << SYS_GPA_MFPL_PA3MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA3MFP Mask               */

#define SYS_GPA_MFPL_PA4MFP_Pos         (16)                                        /*!< SYS_T::GPA_MFPL: PA4MFP Position           */
#define SYS_GPA_MFPL_PA4MFP_Msk         (0xful << SYS_GPA_MFPL_PA4MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA4MFP Mask               */

#define SYS_GPA_MFPL_PA5MFP_Pos         (20)                                        /*!< SYS_T::GPA_MFPL: PA5MFP Position           */
#define SYS_GPA_MFPL_PA5MFP_Msk         (0xful << SYS_GPA_MFPL_PA5MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA5MFP Mask               */

#define SYS_GPA_MFPL_PA6MFP_Pos         (24)                                        /*!< SYS_T::GPA_MFPL: PA6MFP Position           */
#define SYS_GPA_MFPL_PA6MFP_Msk         (0xful << SYS_GPA_MFPL_PA6MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA6MFP Mask               */

#define SYS_GPA_MFPL_PA7MFP_Pos         (28)                                        /*!< SYS_T::GPA_MFPL: PA7MFP Position           */
#define SYS_GPA_MFPL_PA7MFP_Msk         (0xful << SYS_GPA_MFPL_PA7MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA7MFP Mask               */

#define SYS_GPA_MFPH_PA8MFP_Pos         (0)                                         /*!< SYS_T::GPA_MFPH: PA8MFP Position           */
#define SYS_GPA_MFPH_PA8MFP_Msk         (0xful << SYS_GPA_MFPH_PA8MFP_Pos)          /*!< SYS_T::GPA_MFPH: PA8MFP Mask               */

#define SYS_GPA_MFPH_PA9MFP_Pos         (4)                                         /*!< SYS_T::GPA_MFPH: PA9MFP Position           */
#define SYS_GPA_MFPH_PA9MFP_Msk         (0xful << SYS_GPA_MFPH_PA9MFP_Pos)          /*!< SYS_T::GPA_MFPH: PA9MFP Mask               */

#define SYS_GPA_MFPH_PA10MFP_Pos        (8)                                         /*!< SYS_T::GPA_MFPH: PA10MFP Position          */
#define SYS_GPA_MFPH_PA10MFP_Msk        (0xful << SYS_GPA_MFPH_PA10MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA10MFP Mask              */

#define SYS_GPA_MFPH_PA11MFP_Pos        (12)                                        /*!< SYS_T::GPA_MFPH: PA11MFP Position          */
#define SYS_GPA_MFPH_PA11MFP_Msk        (0xful << SYS_GPA_MFPH_PA11MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA11MFP Mask              */

#define SYS_GPA_MFPH_PA12MFP_Pos        (16)                                        /*!< SYS_T::GPA_MFPH: PA12MFP Position          */
#define SYS_GPA_MFPH_PA12MFP_Msk        (0xful << SYS_GPA_MFPH_PA12MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA12MFP Mask              */

#define SYS_GPA_MFPH_PA13MFP_Pos        (20)                                        /*!< SYS_T::GPA_MFPH: PA13MFP Position          */
#define SYS_GPA_MFPH_PA13MFP_Msk        (0xful << SYS_GPA_MFPH_PA13MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA13MFP Mask              */

#define SYS_GPA_MFPH_PA14MFP_Pos        (24)                                        /*!< SYS_T::GPA_MFPH: PA14MFP Position          */
#define SYS_GPA_MFPH_PA14MFP_Msk        (0xful << SYS_GPA_MFPH_PA14MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA14MFP Mask              */

#define SYS_GPA_MFPH_PA15MFP_Pos        (28)                                        /*!< SYS_T::GPA_MFPH: PA15MFP Position          */
#define SYS_GPA_MFPH_PA15MFP_Msk        (0xful << SYS_GPA_MFPH_PA15MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA15MFP Mask              */

#define SYS_GPB_MFPL_PB0MFP_Pos         (0)                                         /*!< SYS_T::GPB_MFPL: PB0MFP Position           */
#define SYS_GPB_MFPL_PB0MFP_Msk         (0xful << SYS_GPB_MFPL_PB0MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB0MFP Mask               */

#define SYS_GPB_MFPL_PB1MFP_Pos         (4)                                         /*!< SYS_T::GPB_MFPL: PB1MFP Position           */
#define SYS_GPB_MFPL_PB1MFP_Msk         (0xful << SYS_GPB_MFPL_PB1MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB1MFP Mask               */

#define SYS_GPB_MFPL_PB2MFP_Pos         (8)                                         /*!< SYS_T::GPB_MFPL: PB2MFP Position           */
#define SYS_GPB_MFPL_PB2MFP_Msk         (0xful << SYS_GPB_MFPL_PB2MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB2MFP Mask               */

#define SYS_GPB_MFPL_PB3MFP_Pos         (12)                                        /*!< SYS_T::GPB_MFPL: PB3MFP Position           */
#define SYS_GPB_MFPL_PB3MFP_Msk         (0xful << SYS_GPB_MFPL_PB3MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB3MFP Mask               */

#define SYS_GPB_MFPL_PB4MFP_Pos         (16)                                        /*!< SYS_T::GPB_MFPL: PB4MFP Position           */
#define SYS_GPB_MFPL_PB4MFP_Msk         (0xful << SYS_GPB_MFPL_PB4MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB4MFP Mask               */

#define SYS_GPB_MFPL_PB5MFP_Pos         (20)                                        /*!< SYS_T::GPB_MFPL: PB5MFP Position           */
#define SYS_GPB_MFPL_PB5MFP_Msk         (0xful << SYS_GPB_MFPL_PB5MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB5MFP Mask               */

#define SYS_GPB_MFPL_PB6MFP_Pos         (24)                                        /*!< SYS_T::GPB_MFPL: PB6MFP Position           */
#define SYS_GPB_MFPL_PB6MFP_Msk         (0xful << SYS_GPB_MFPL_PB6MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB6MFP Mask               */

#define SYS_GPB_MFPL_PB7MFP_Pos         (28)                                        /*!< SYS_T::GPB_MFPL: PB7MFP Position           */
#define SYS_GPB_MFPL_PB7MFP_Msk         (0xful << SYS_GPB_MFPL_PB7MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB7MFP Mask               */

#define SYS_GPB_MFPH_PB8MFP_Pos         (0)                                         /*!< SYS_T::GPB_MFPH: PB8MFP Position           */
#define SYS_GPB_MFPH_PB8MFP_Msk         (0xful << SYS_GPB_MFPH_PB8MFP_Pos)          /*!< SYS_T::GPB_MFPH: PB8MFP Mask               */

#define SYS_GPB_MFPH_PB9MFP_Pos         (4)                                         /*!< SYS_T::GPB_MFPH: PB9MFP Position           */
#define SYS_GPB_MFPH_PB9MFP_Msk         (0xful << SYS_GPB_MFPH_PB9MFP_Pos)          /*!< SYS_T::GPB_MFPH: PB9MFP Mask               */

#define SYS_GPB_MFPH_PB10MFP_Pos        (8)                                         /*!< SYS_T::GPB_MFPH: PB10MFP Position          */
#define SYS_GPB_MFPH_PB10MFP_Msk        (0xful << SYS_GPB_MFPH_PB10MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB10MFP Mask              */

#define SYS_GPB_MFPH_PB11MFP_Pos        (12)                                        /*!< SYS_T::GPB_MFPH: PB11MFP Position          */
#define SYS_GPB_MFPH_PB11MFP_Msk        (0xful << SYS_GPB_MFPH_PB11MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB11MFP Mask              */

#define SYS_GPB_MFPH_PB12MFP_Pos        (16)                                        /*!< SYS_T::GPB_MFPH: PB12MFP Position          */
#define SYS_GPB_MFPH_PB12MFP_Msk        (0xful << SYS_GPB_MFPH_PB12MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB12MFP Mask              */

#define SYS_GPB_MFPH_PB13MFP_Pos        (20)                                        /*!< SYS_T::GPB_MFPH: PB13MFP Position          */
#define SYS_GPB_MFPH_PB13MFP_Msk        (0xful << SYS_GPB_MFPH_PB13MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB13MFP Mask              */

#define SYS_GPB_MFPH_PB14MFP_Pos        (24)                                        /*!< SYS_T::GPB_MFPH: PB14MFP Position          */
#define SYS_GPB_MFPH_PB14MFP_Msk        (0xful << SYS_GPB_MFPH_PB14MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB14MFP Mask              */

#define SYS_GPB_MFPH_PB15MFP_Pos        (28)                                        /*!< SYS_T::GPB_MFPH: PB15MFP Position          */
#define SYS_GPB_MFPH_PB15MFP_Msk        (0xful << SYS_GPB_MFPH_PB15MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB15MFP Mask              */

#define SYS_GPC_MFPL_PC0MFP_Pos         (0)                                         /*!< SYS_T::GPC_MFPL: PC0MFP Position           */
#define SYS_GPC_MFPL_PC0MFP_Msk         (0xful << SYS_GPC_MFPL_PC0MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC0MFP Mask               */

#define SYS_GPC_MFPL_PC1MFP_Pos         (4)                                         /*!< SYS_T::GPC_MFPL: PC1MFP Position           */
#define SYS_GPC_MFPL_PC1MFP_Msk         (0xful << SYS_GPC_MFPL_PC1MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC1MFP Mask               */

#define SYS_GPC_MFPL_PC2MFP_Pos         (8)                                         /*!< SYS_T::GPC_MFPL: PC2MFP Position           */
#define SYS_GPC_MFPL_PC2MFP_Msk         (0xful << SYS_GPC_MFPL_PC2MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC2MFP Mask               */

#define SYS_GPC_MFPL_PC3MFP_Pos         (12)                                        /*!< SYS_T::GPC_MFPL: PC3MFP Position           */
#define SYS_GPC_MFPL_PC3MFP_Msk         (0xful << SYS_GPC_MFPL_PC3MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC3MFP Mask               */

#define SYS_GPC_MFPL_PC4MFP_Pos         (16)                                        /*!< SYS_T::GPC_MFPL: PC4MFP Position           */
#define SYS_GPC_MFPL_PC4MFP_Msk         (0xful << SYS_GPC_MFPL_PC4MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC4MFP Mask               */

#define SYS_GPC_MFPL_PC5MFP_Pos         (20)                                        /*!< SYS_T::GPC_MFPL: PC5MFP Position           */
#define SYS_GPC_MFPL_PC5MFP_Msk         (0xful << SYS_GPC_MFPL_PC5MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC5MFP Mask               */

#define SYS_GPC_MFPL_PC6MFP_Pos         (24)                                        /*!< SYS_T::GPC_MFPL: PC6MFP Position           */
#define SYS_GPC_MFPL_PC6MFP_Msk         (0xful << SYS_GPC_MFPL_PC6MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC6MFP Mask               */

#define SYS_GPC_MFPL_PC7MFP_Pos         (28)                                        /*!< SYS_T::GPC_MFPL: PC7MFP Position           */
#define SYS_GPC_MFPL_PC7MFP_Msk         (0xful << SYS_GPC_MFPL_PC7MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC7MFP Mask               */

#define SYS_GPC_MFPH_PC8MFP_Pos         (0)                                         /*!< SYS_T::GPC_MFPH: PC8MFP Position           */
#define SYS_GPC_MFPH_PC8MFP_Msk         (0xful << SYS_GPC_MFPH_PC8MFP_Pos)          /*!< SYS_T::GPC_MFPH: PC8MFP Mask               */

#define SYS_GPC_MFPH_PC9MFP_Pos         (4)                                         /*!< SYS_T::GPC_MFPH: PC9MFP Position           */
#define SYS_GPC_MFPH_PC9MFP_Msk         (0xful << SYS_GPC_MFPH_PC9MFP_Pos)          /*!< SYS_T::GPC_MFPH: PC9MFP Mask               */

#define SYS_GPC_MFPH_PC10MFP_Pos        (8)                                         /*!< SYS_T::GPC_MFPH: PC10MFP Position          */
#define SYS_GPC_MFPH_PC10MFP_Msk        (0xful << SYS_GPC_MFPH_PC10MFP_Pos)         /*!< SYS_T::GPC_MFPH: PC10MFP Mask              */

#define SYS_GPC_MFPH_PC11MFP_Pos        (12)                                        /*!< SYS_T::GPC_MFPH: PC11MFP Position          */
#define SYS_GPC_MFPH_PC11MFP_Msk        (0xful << SYS_GPC_MFPH_PC11MFP_Pos)         /*!< SYS_T::GPC_MFPH: PC11MFP Mask              */

#define SYS_GPC_MFPH_PC12MFP_Pos        (16)                                        /*!< SYS_T::GPC_MFPH: PC12MFP Position          */
#define SYS_GPC_MFPH_PC12MFP_Msk        (0xful << SYS_GPC_MFPH_PC12MFP_Pos)         /*!< SYS_T::GPC_MFPH: PC12MFP Mask              */

#define SYS_GPC_MFPH_PC14MFP_Pos        (24)                                        /*!< SYS_T::GPC_MFPH: PC14MFP Position          */
#define SYS_GPC_MFPH_PC14MFP_Msk        (0xful << SYS_GPC_MFPH_PC14MFP_Pos)         /*!< SYS_T::GPC_MFPH: PC14MFP Mask              */

#define SYS_GPD_MFPL_PD0MFP_Pos         (0)                                         /*!< SYS_T::GPD_MFPL: PD0MFP Position           */
#define SYS_GPD_MFPL_PD0MFP_Msk         (0xful << SYS_GPD_MFPL_PD0MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD0MFP Mask               */

#define SYS_GPD_MFPL_PD1MFP_Pos         (4)                                         /*!< SYS_T::GPD_MFPL: PD1MFP Position           */
#define SYS_GPD_MFPL_PD1MFP_Msk         (0xful << SYS_GPD_MFPL_PD1MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD1MFP Mask               */

#define SYS_GPD_MFPL_PD2MFP_Pos         (8)                                         /*!< SYS_T::GPD_MFPL: PD2MFP Position           */
#define SYS_GPD_MFPL_PD2MFP_Msk         (0xful << SYS_GPD_MFPL_PD2MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD2MFP Mask               */

#define SYS_GPD_MFPL_PD3MFP_Pos         (12)                                        /*!< SYS_T::GPD_MFPL: PD3MFP Position           */
#define SYS_GPD_MFPL_PD3MFP_Msk         (0xful << SYS_GPD_MFPL_PD3MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD3MFP Mask               */

#define SYS_GPD_MFPL_PD4MFP_Pos         (16)                                        /*!< SYS_T::GPD_MFPL: PD4MFP Position           */
#define SYS_GPD_MFPL_PD4MFP_Msk         (0xful << SYS_GPD_MFPL_PD4MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD4MFP Mask               */

#define SYS_GPD_MFPL_PD5MFP_Pos         (20)                                        /*!< SYS_T::GPD_MFPL: PD5MFP Position           */
#define SYS_GPD_MFPL_PD5MFP_Msk         (0xful << SYS_GPD_MFPL_PD5MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD5MFP Mask               */

#define SYS_GPD_MFPL_PD6MFP_Pos         (24)                                        /*!< SYS_T::GPD_MFPL: PD6MFP Position           */
#define SYS_GPD_MFPL_PD6MFP_Msk         (0xful << SYS_GPD_MFPL_PD6MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD6MFP Mask               */

#define SYS_GPD_MFPL_PD7MFP_Pos         (28)                                        /*!< SYS_T::GPD_MFPL: PD7MFP Position           */
#define SYS_GPD_MFPL_PD7MFP_Msk         (0xful << SYS_GPD_MFPL_PD7MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD7MFP Mask               */

#define SYS_GPD_MFPH_PD8MFP_Pos         (0)                                         /*!< SYS_T::GPD_MFPH: PD8MFP Position           */
#define SYS_GPD_MFPH_PD8MFP_Msk         (0xful << SYS_GPD_MFPH_PD8MFP_Pos)          /*!< SYS_T::GPD_MFPH: PD8MFP Mask               */

#define SYS_GPD_MFPH_PD9MFP_Pos         (4)                                         /*!< SYS_T::GPD_MFPH: PD9MFP Position           */
#define SYS_GPD_MFPH_PD9MFP_Msk         (0xful << SYS_GPD_MFPH_PD9MFP_Pos)          /*!< SYS_T::GPD_MFPH: PD9MFP Mask               */

#define SYS_GPD_MFPH_PD10MFP_Pos        (8)                                         /*!< SYS_T::GPD_MFPH: PD10MFP Position          */
#define SYS_GPD_MFPH_PD10MFP_Msk        (0xful << SYS_GPD_MFPH_PD10MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD10MFP Mask              */

#define SYS_GPD_MFPH_PD11MFP_Pos        (12)                                        /*!< SYS_T::GPD_MFPH: PD11MFP Position          */
#define SYS_GPD_MFPH_PD11MFP_Msk        (0xful << SYS_GPD_MFPH_PD11MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD11MFP Mask              */

#define SYS_GPD_MFPH_PD12MFP_Pos        (16)                                        /*!< SYS_T::GPD_MFPH: PD12MFP Position          */
#define SYS_GPD_MFPH_PD12MFP_Msk        (0xful << SYS_GPD_MFPH_PD12MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD12MFP Mask              */

#define SYS_GPD_MFPH_PD13MFP_Pos        (20)                                        /*!< SYS_T::GPD_MFPH: PD13MFP Position          */
#define SYS_GPD_MFPH_PD13MFP_Msk        (0xful << SYS_GPD_MFPH_PD13MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD13MFP Mask              */

#define SYS_GPD_MFPH_PD15MFP_Pos        (28)                                        /*!< SYS_T::GPD_MFPH: PD15MFP Position          */
#define SYS_GPD_MFPH_PD15MFP_Msk        (0xful << SYS_GPD_MFPH_PD15MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD15MFP Mask              */

#define SYS_GPE_MFPL_PE0MFP_Pos         (0)                                         /*!< SYS_T::GPE_MFPL: PE0MFP Position           */
#define SYS_GPE_MFPL_PE0MFP_Msk         (0xful << SYS_GPE_MFPL_PE0MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE0MFP Mask               */

#define SYS_GPE_MFPL_PE1MFP_Pos         (4)                                         /*!< SYS_T::GPE_MFPL: PE1MFP Position           */
#define SYS_GPE_MFPL_PE1MFP_Msk         (0xful << SYS_GPE_MFPL_PE1MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE1MFP Mask               */

#define SYS_GPE_MFPL_PE2MFP_Pos         (8)                                         /*!< SYS_T::GPE_MFPL: PE2MFP Position           */
#define SYS_GPE_MFPL_PE2MFP_Msk         (0xful << SYS_GPE_MFPL_PE2MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE2MFP Mask               */

#define SYS_GPE_MFPL_PE3MFP_Pos         (12)                                        /*!< SYS_T::GPE_MFPL: PE3MFP Position           */
#define SYS_GPE_MFPL_PE3MFP_Msk         (0xful << SYS_GPE_MFPL_PE3MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE3MFP Mask               */

#define SYS_GPE_MFPL_PE4MFP_Pos         (16)                                        /*!< SYS_T::GPE_MFPL: PE4MFP Position           */
#define SYS_GPE_MFPL_PE4MFP_Msk         (0xful << SYS_GPE_MFPL_PE4MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE4MFP Mask               */

#define SYS_GPE_MFPL_PE5MFP_Pos         (20)                                        /*!< SYS_T::GPE_MFPL: PE5MFP Position           */
#define SYS_GPE_MFPL_PE5MFP_Msk         (0xful << SYS_GPE_MFPL_PE5MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE5MFP Mask               */

#define SYS_GPE_MFPL_PE6MFP_Pos         (24)                                        /*!< SYS_T::GPE_MFPL: PE6MFP Position           */
#define SYS_GPE_MFPL_PE6MFP_Msk         (0xful << SYS_GPE_MFPL_PE6MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE6MFP Mask               */

#define SYS_GPE_MFPL_PE7MFP_Pos         (28)                                        /*!< SYS_T::GPE_MFPL: PE7MFP Position           */
#define SYS_GPE_MFPL_PE7MFP_Msk         (0xful << SYS_GPE_MFPL_PE7MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE7MFP Mask               */

#define SYS_GPE_MFPH_PE8MFP_Pos         (0)                                         /*!< SYS_T::GPE_MFPH: PE8MFP Position           */
#define SYS_GPE_MFPH_PE8MFP_Msk         (0xful << SYS_GPE_MFPH_PE8MFP_Pos)          /*!< SYS_T::GPE_MFPH: PE8MFP Mask               */

#define SYS_GPE_MFPH_PE9MFP_Pos         (4)                                         /*!< SYS_T::GPE_MFPH: PE9MFP Position           */
#define SYS_GPE_MFPH_PE9MFP_Msk         (0xful << SYS_GPE_MFPH_PE9MFP_Pos)          /*!< SYS_T::GPE_MFPH: PE9MFP Mask               */

#define SYS_GPE_MFPH_PE10MFP_Pos        (8)                                         /*!< SYS_T::GPE_MFPH: PE10MFP Position          */
#define SYS_GPE_MFPH_PE10MFP_Msk        (0xful << SYS_GPE_MFPH_PE10MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE10MFP Mask              */

#define SYS_GPE_MFPH_PE11MFP_Pos        (12)                                        /*!< SYS_T::GPE_MFPH: PE11MFP Position          */
#define SYS_GPE_MFPH_PE11MFP_Msk        (0xful << SYS_GPE_MFPH_PE11MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE11MFP Mask              */

#define SYS_GPE_MFPH_PE12MFP_Pos        (16)                                        /*!< SYS_T::GPE_MFPH: PE12MFP Position          */
#define SYS_GPE_MFPH_PE12MFP_Msk        (0xful << SYS_GPE_MFPH_PE12MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE12MFP Mask              */

#define SYS_GPE_MFPH_PE13MFP_Pos        (20)                                        /*!< SYS_T::GPE_MFPH: PE13MFP Position          */
#define SYS_GPE_MFPH_PE13MFP_Msk        (0xful << SYS_GPE_MFPH_PE13MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE13MFP Mask              */

#define SYS_GPE_MFPH_PE14MFP_Pos        (24)                                        /*!< SYS_T::GPE_MFPH: PE14MFP Position          */
#define SYS_GPE_MFPH_PE14MFP_Msk        (0xful << SYS_GPE_MFPH_PE14MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE14MFP Mask              */

#define SYS_GPE_MFPH_PE15MFP_Pos        (28)                                        /*!< SYS_T::GPE_MFPH: PE15MFP Position          */
#define SYS_GPE_MFPH_PE15MFP_Msk        (0xful << SYS_GPE_MFPH_PE15MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE15MFP Mask              */

#define SYS_GPF_MFPL_PF0MFP_Pos         (0)                                         /*!< SYS_T::GPF_MFPL: PF0MFP Position           */
#define SYS_GPF_MFPL_PF0MFP_Msk         (0xful << SYS_GPF_MFPL_PF0MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF0MFP Mask               */

#define SYS_GPF_MFPL_PF1MFP_Pos         (4)                                         /*!< SYS_T::GPF_MFPL: PF1MFP Position           */
#define SYS_GPF_MFPL_PF1MFP_Msk         (0xful << SYS_GPF_MFPL_PF1MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF1MFP Mask               */

#define SYS_GPF_MFPL_PF2MFP_Pos         (8)                                         /*!< SYS_T::GPF_MFPL: PF2MFP Position           */
#define SYS_GPF_MFPL_PF2MFP_Msk         (0xful << SYS_GPF_MFPL_PF2MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF2MFP Mask               */

#define SYS_GPF_MFPL_PF3MFP_Pos         (12)                                        /*!< SYS_T::GPF_MFPL: PF3MFP Position           */
#define SYS_GPF_MFPL_PF3MFP_Msk         (0xful << SYS_GPF_MFPL_PF3MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF3MFP Mask               */

#define SYS_GPF_MFPL_PF4MFP_Pos         (16)                                        /*!< SYS_T::GPF_MFPL: PF4MFP Position           */
#define SYS_GPF_MFPL_PF4MFP_Msk         (0xful << SYS_GPF_MFPL_PF4MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF4MFP Mask               */

#define SYS_GPF_MFPL_PF5MFP_Pos         (20)                                        /*!< SYS_T::GPF_MFPL: PF5MFP Position           */
#define SYS_GPF_MFPL_PF5MFP_Msk         (0xful << SYS_GPF_MFPL_PF5MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF5MFP Mask               */

#define SYS_GPF_MFPL_PF6MFP_Pos         (24)                                        /*!< SYS_T::GPF_MFPL: PF6MFP Position           */
#define SYS_GPF_MFPL_PF6MFP_Msk         (0xful << SYS_GPF_MFPL_PF6MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF6MFP Mask               */

#define SYS_GPF_MFPL_PF7MFP_Pos         (28)                                        /*!< SYS_T::GPF_MFPL: PF7MFP Position           */
#define SYS_GPF_MFPL_PF7MFP_Msk         (0xful << SYS_GPF_MFPL_PF7MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF7MFP Mask               */

#define SYS_GPF_MFPH_PF8MFP_Pos         (0)                                         /*!< SYS_T::GPF_MFPH: PF8MFP Position           */
#define SYS_GPF_MFPH_PF8MFP_Msk         (0xful << SYS_GPF_MFPH_PF8MFP_Pos)          /*!< SYS_T::GPF_MFPH: PF8MFP Mask               */

#define SYS_GPF_MFPH_PF9MFP_Pos         (4)                                         /*!< SYS_T::GPF_MFPH: PF9MFP Position           */
#define SYS_GPF_MFPH_PF9MFP_Msk         (0xful << SYS_GPF_MFPH_PF9MFP_Pos)          /*!< SYS_T::GPF_MFPH: PF9MFP Mask               */

#define SYS_GPF_MFPH_PF14MFP_Pos        (24)                                        /*!< SYS_T::GPF_MFPH: PF14MFP Position          */
#define SYS_GPF_MFPH_PF14MFP_Msk        (0xful << SYS_GPF_MFPH_PF14MFP_Pos)         /*!< SYS_T::GPF_MFPH: PF14MFP Mask              */

#define SYS_GPG_MFPL_PG2MFP_Pos         (8)                                         /*!< SYS_T::GPG_MFPL: PG2MFP Position           */
#define SYS_GPG_MFPL_PG2MFP_Msk         (0xful << SYS_GPG_MFPL_PG2MFP_Pos)          /*!< SYS_T::GPG_MFPL: PG2MFP Mask               */

#define SYS_GPG_MFPL_PG3MFP_Pos         (12)                                        /*!< SYS_T::GPG_MFPL: PG3MFP Position           */
#define SYS_GPG_MFPL_PG3MFP_Msk         (0xful << SYS_GPG_MFPL_PG3MFP_Pos)          /*!< SYS_T::GPG_MFPL: PG3MFP Mask               */

#define SYS_GPG_MFPL_PG4MFP_Pos         (16)                                        /*!< SYS_T::GPG_MFPL: PG4MFP Position           */
#define SYS_GPG_MFPL_PG4MFP_Msk         (0xful << SYS_GPG_MFPL_PG4MFP_Pos)          /*!< SYS_T::GPG_MFPL: PG4MFP Mask               */

#define SYS_GPH_MFPH_PH8MFP_Pos         (0)                                         /*!< SYS_T::GPH_MFPH: PH8MFP Position           */
#define SYS_GPH_MFPH_PH8MFP_Msk         (0xful << SYS_GPH_MFPH_PH8MFP_Pos)          /*!< SYS_T::GPH_MFPH: PH8MFP Mask               */

#define SYS_GPH_MFPH_PH9MFP_Pos         (4)                                         /*!< SYS_T::GPH_MFPH: PH9MFP Position           */
#define SYS_GPH_MFPH_PH9MFP_Msk         (0xful << SYS_GPH_MFPH_PH9MFP_Pos)          /*!< SYS_T::GPH_MFPH: PH9MFP Mask               */

#define SYS_GPA_MFOS_MFOS_Pos           (0)                                         /*!< SYS_T::GPA_MFOS: MFOS Position             */
#define SYS_GPA_MFOS_MFOS_Msk           (0xfffful << SYS_GPA_MFOS_MFOS_Pos)         /*!< SYS_T::GPA_MFOS: MFOS Mask                 */

#define SYS_GPB_MFOS_MFOS_Pos           (0)                                         /*!< SYS_T::GPB_MFOS: MFOS Position             */
#define SYS_GPB_MFOS_MFOS_Msk           (0xfffful << SYS_GPB_MFOS_MFOS_Pos)         /*!< SYS_T::GPB_MFOS: MFOS Mask                 */

#define SYS_GPC_MFOS_MFOS_Pos           (0)                                         /*!< SYS_T::GPC_MFOS: MFOS Position             */
#define SYS_GPC_MFOS_MFOS_Msk           (0xfffful << SYS_GPC_MFOS_MFOS_Pos)         /*!< SYS_T::GPC_MFOS: MFOS Mask                 */

#define SYS_GPD_MFOS_MFOS_Pos           (0)                                         /*!< SYS_T::GPD_MFOS: MFOS Position             */
#define SYS_GPD_MFOS_MFOS_Msk           (0xfffful << SYS_GPD_MFOS_MFOS_Pos)         /*!< SYS_T::GPD_MFOS: MFOS Mask                 */

#define SYS_GPE_MFOS_MFOS_Pos           (0)                                         /*!< SYS_T::GPE_MFOS: MFOS Position             */
#define SYS_GPE_MFOS_MFOS_Msk           (0xfffful << SYS_GPE_MFOS_MFOS_Pos)         /*!< SYS_T::GPE_MFOS: MFOS Mask                 */

#define SYS_GPF_MFOS_MFOS_Pos           (0)                                         /*!< SYS_T::GPF_MFOS: MFOS Position             */
#define SYS_GPF_MFOS_MFOS_Msk           (0xfffful << SYS_GPF_MFOS_MFOS_Pos)         /*!< SYS_T::GPF_MFOS: MFOS Mask                 */

#define SYS_GPG_MFOS_MFOS_Pos           (0)                                         /*!< SYS_T::GPG_MFOS: MFOS Position             */
#define SYS_GPG_MFOS_MFOS_Msk           (0xfffful << SYS_GPG_MFOS_MFOS_Pos)         /*!< SYS_T::GPG_MFOS: MFOS Mask                 */

#define SYS_GPH_MFOS_MFOS_Pos           (0)                                         /*!< SYS_T::GPH_MFOS: MFOS Position             */
#define SYS_GPH_MFOS_MFOS_Msk           (0xfffful << SYS_GPH_MFOS_MFOS_Pos)         /*!< SYS_T::GPH_MFOS: MFOS Mask                 */

#define SYS_MIRCTCTL_AUTRIMEN_Pos       (0)                                         /*!< SYS_T::MIRCTCTL: AUTRIMEN Position         */
#define SYS_MIRCTCTL_AUTRIMEN_Msk       (0x1ul << SYS_MIRCTCTL_AUTRIMEN_Pos)        /*!< SYS_T::MIRCTCTL: AUTRIMEN Mask             */

#define SYS_MIRCTCTL_ACCURSEL_Pos       (2)                                         /*!< SYS_T::MIRCTCTL: ACCURSEL Position         */
#define SYS_MIRCTCTL_ACCURSEL_Msk       (0x3ul << SYS_MIRCTCTL_ACCURSEL_Pos)        /*!< SYS_T::MIRCTCTL: ACCURSEL Mask             */

#define SYS_MIRCTCTL_LOOPSEL_Pos        (4)                                         /*!< SYS_T::MIRCTCTL: LOOPSEL Position          */
#define SYS_MIRCTCTL_LOOPSEL_Msk        (0x3ul << SYS_MIRCTCTL_LOOPSEL_Pos)         /*!< SYS_T::MIRCTCTL: LOOPSEL Mask              */

#define SYS_MIRCTCTL_RETRYCNT_Pos       (6)                                         /*!< SYS_T::MIRCTCTL: RETRYCNT Position         */
#define SYS_MIRCTCTL_RETRYCNT_Msk       (0x3ul << SYS_MIRCTCTL_RETRYCNT_Pos)        /*!< SYS_T::MIRCTCTL: RETRYCNT Mask             */

#define SYS_MIRCTCTL_CESTOPEN_Pos       (8)                                         /*!< SYS_T::MIRCTCTL: CESTOPEN Position         */
#define SYS_MIRCTCTL_CESTOPEN_Msk       (0x1ul << SYS_MIRCTCTL_CESTOPEN_Pos)        /*!< SYS_T::MIRCTCTL: CESTOPEN Mask             */

#define SYS_MIRCTCTL_BOUNDEN_Pos        (9)                                         /*!< SYS_T::MIRCTCTL: BOUNDEN Position          */
#define SYS_MIRCTCTL_BOUNDEN_Msk        (0x1ul << SYS_MIRCTCTL_BOUNDEN_Pos)         /*!< SYS_T::MIRCTCTL: BOUNDEN Mask              */

#define SYS_MIRCTCTL_REFCKSEL_Pos       (10)                                        /*!< SYS_T::MIRCTCTL: REFCKSEL Position         */
#define SYS_MIRCTCTL_REFCKSEL_Msk       (0x1ul << SYS_MIRCTCTL_REFCKSEL_Pos)        /*!< SYS_T::MIRCTCTL: REFCKSEL Mask             */

#define SYS_MIRCTCTL_TFSTOPEN_Pos       (12)                                        /*!< SYS_T::MIRCTCTL: TFSTOPEN Position         */
#define SYS_MIRCTCTL_TFSTOPEN_Msk       (0x1ul << SYS_MIRCTCTL_TFSTOPEN_Pos)        /*!< SYS_T::MIRCTCTL: TFSTOPEN Mask             */

#define SYS_MIRCTCTL_AUTOEN_Pos         (13)                                        /*!< SYS_T::MIRCTCTL: AUTOEN Position           */
#define SYS_MIRCTCTL_AUTOEN_Msk         (0x1ul << SYS_MIRCTCTL_AUTOEN_Pos)          /*!< SYS_T::MIRCTCTL: AUTOEN Mask               */

#define SYS_MIRCTCTL_BOUNDARY_Pos       (16)                                        /*!< SYS_T::MIRCTCTL: BOUNDARY Position         */
#define SYS_MIRCTCTL_BOUNDARY_Msk       (0x1ful << SYS_MIRCTCTL_BOUNDARY_Pos)       /*!< SYS_T::MIRCTCTL: BOUNDARY Mask             */

#define SYS_MIRCTIEN_TFAILIEN_Pos       (1)                                         /*!< SYS_T::MIRCTIEN: TFAILIEN Position         */
#define SYS_MIRCTIEN_TFAILIEN_Msk       (0x1ul << SYS_MIRCTIEN_TFAILIEN_Pos)        /*!< SYS_T::MIRCTIEN: TFAILIEN Mask             */

#define SYS_MIRCTIEN_CLKEIEN_Pos        (2)                                         /*!< SYS_T::MIRCTIEN: CLKEIEN Position          */
#define SYS_MIRCTIEN_CLKEIEN_Msk        (0x1ul << SYS_MIRCTIEN_CLKEIEN_Pos)         /*!< SYS_T::MIRCTIEN: CLKEIEN Mask              */

#define SYS_MIRCTIEN_CLKWKEN_Pos        (4)                                         /*!< SYS_T::MIRCTIEN: CLKWKEN Position          */
#define SYS_MIRCTIEN_CLKWKEN_Msk        (0x1ul << SYS_MIRCTIEN_CLKWKEN_Pos)         /*!< SYS_T::MIRCTIEN: CLKWKEN Mask              */

#define SYS_MIRCTISTS_FREQLOCK_Pos      (0)                                         /*!< SYS_T::MIRCTISTS: FREQLOCK Position        */
#define SYS_MIRCTISTS_FREQLOCK_Msk      (0x1ul << SYS_MIRCTISTS_FREQLOCK_Pos)       /*!< SYS_T::MIRCTISTS: FREQLOCK Mask            */

#define SYS_MIRCTISTS_TFAILIF_Pos       (1)                                         /*!< SYS_T::MIRCTISTS: TFAILIF Position         */
#define SYS_MIRCTISTS_TFAILIF_Msk       (0x1ul << SYS_MIRCTISTS_TFAILIF_Pos)        /*!< SYS_T::MIRCTISTS: TFAILIF Mask             */

#define SYS_MIRCTISTS_CLKERRIF_Pos      (2)                                         /*!< SYS_T::MIRCTISTS: CLKERRIF Position        */
#define SYS_MIRCTISTS_CLKERRIF_Msk      (0x1ul << SYS_MIRCTISTS_CLKERRIF_Pos)       /*!< SYS_T::MIRCTISTS: CLKERRIF Mask            */

#define SYS_MIRCTISTS_OVBDIF_Pos        (3)                                         /*!< SYS_T::MIRCTISTS: OVBDIF Position          */
#define SYS_MIRCTISTS_OVBDIF_Msk        (0x1ul << SYS_MIRCTISTS_OVBDIF_Pos)         /*!< SYS_T::MIRCTISTS: OVBDIF Mask              */

#define SYS_SRAM_BISTCTL_SRBIST0_Pos    (0)                                         /*!< SYS_T::SRAM_BISTCTL: SRBIST0 Position      */
#define SYS_SRAM_BISTCTL_SRBIST0_Msk    (0x1ul << SYS_SRAM_BISTCTL_SRBIST0_Pos)     /*!< SYS_T::SRAM_BISTCTL: SRBIST0 Mask          */

#define SYS_SRAM_BISTCTL_CRBIST_Pos     (2)                                         /*!< SYS_T::SRAM_BISTCTL: CRBIST Position       */
#define SYS_SRAM_BISTCTL_CRBIST_Msk     (0x1ul << SYS_SRAM_BISTCTL_CRBIST_Pos)      /*!< SYS_T::SRAM_BISTCTL: CRBIST Mask           */

#define SYS_SRAM_BISTSTS_SRBISTEF0_Pos  (0)                                         /*!< SYS_T::SRAM_BISTSTS: SRBISTEF0 Position    */
#define SYS_SRAM_BISTSTS_SRBISTEF0_Msk  (0x1ul << SYS_SRAM_BISTSTS_SRBISTEF0_Pos)   /*!< SYS_T::SRAM_BISTSTS: SRBISTEF0 Mask        */

#define SYS_SRAM_BISTSTS_CRBISTEF_Pos   (2)                                         /*!< SYS_T::SRAM_BISTSTS: CRBISTEF Position     */
#define SYS_SRAM_BISTSTS_CRBISTEF_Msk   (0x1ul << SYS_SRAM_BISTSTS_CRBISTEF_Pos)    /*!< SYS_T::SRAM_BISTSTS: CRBISTEF Mask         */

#define SYS_SRAM_BISTSTS_SRBEND0_Pos    (16)                                        /*!< SYS_T::SRAM_BISTSTS: SRBEND0 Position      */
#define SYS_SRAM_BISTSTS_SRBEND0_Msk    (0x1ul << SYS_SRAM_BISTSTS_SRBEND0_Pos)     /*!< SYS_T::SRAM_BISTSTS: SRBEND0 Mask          */

#define SYS_SRAM_BISTSTS_CRBEND_Pos     (18)                                        /*!< SYS_T::SRAM_BISTSTS: CRBEND Position       */
#define SYS_SRAM_BISTSTS_CRBEND_Msk     (0x1ul << SYS_SRAM_BISTSTS_CRBEND_Pos)      /*!< SYS_T::SRAM_BISTSTS: CRBEND Mask           */

#define SYS_HIRCTCTL_AUTRIMEN_Pos       (0)                                         /*!< SYS_T::HIRCTCTL: AUTRIMEN Position         */
#define SYS_HIRCTCTL_AUTRIMEN_Msk       (0x1ul << SYS_HIRCTCTL_AUTRIMEN_Pos)        /*!< SYS_T::HIRCTCTL: AUTRIMEN Mask             */

#define SYS_HIRCTCTL_ACCURSEL_Pos       (2)                                         /*!< SYS_T::HIRCTCTL: ACCURSEL Position         */
#define SYS_HIRCTCTL_ACCURSEL_Msk       (0x3ul << SYS_HIRCTCTL_ACCURSEL_Pos)        /*!< SYS_T::HIRCTCTL: ACCURSEL Mask             */

#define SYS_HIRCTCTL_LOOPSEL_Pos        (4)                                         /*!< SYS_T::HIRCTCTL: LOOPSEL Position          */
#define SYS_HIRCTCTL_LOOPSEL_Msk        (0x3ul << SYS_HIRCTCTL_LOOPSEL_Pos)         /*!< SYS_T::HIRCTCTL: LOOPSEL Mask              */

#define SYS_HIRCTCTL_RETRYCNT_Pos       (6)                                         /*!< SYS_T::HIRCTCTL: RETRYCNT Position         */
#define SYS_HIRCTCTL_RETRYCNT_Msk       (0x3ul << SYS_HIRCTCTL_RETRYCNT_Pos)        /*!< SYS_T::HIRCTCTL: RETRYCNT Mask             */

#define SYS_HIRCTCTL_CESTOPEN_Pos       (8)                                         /*!< SYS_T::HIRCTCTL: CESTOPEN Position         */
#define SYS_HIRCTCTL_CESTOPEN_Msk       (0x1ul << SYS_HIRCTCTL_CESTOPEN_Pos)        /*!< SYS_T::HIRCTCTL: CESTOPEN Mask             */

#define SYS_HIRCTCTL_BOUNDEN_Pos        (9)                                         /*!< SYS_T::HIRCTCTL: BOUNDEN Position          */
#define SYS_HIRCTCTL_BOUNDEN_Msk        (0x1ul << SYS_HIRCTCTL_BOUNDEN_Pos)         /*!< SYS_T::HIRCTCTL: BOUNDEN Mask              */

#define SYS_HIRCTCTL_REFCKSEL_Pos       (10)                                        /*!< SYS_T::HIRCTCTL: REFCKSEL Position         */
#define SYS_HIRCTCTL_REFCKSEL_Msk       (0x1ul << SYS_HIRCTCTL_REFCKSEL_Pos)        /*!< SYS_T::HIRCTCTL: REFCKSEL Mask             */

#define SYS_HIRCTCTL_TFSTOPEN_Pos       (12)                                        /*!< SYS_T::HIRCTCTL: TFSTOPEN Position         */
#define SYS_HIRCTCTL_TFSTOPEN_Msk       (0x1ul << SYS_HIRCTCTL_TFSTOPEN_Pos)        /*!< SYS_T::HIRCTCTL: TFSTOPEN Mask             */

#define SYS_HIRCTCTL_BOUNDARY_Pos       (16)                                        /*!< SYS_T::HIRCTCTL: BOUNDARY Position         */
#define SYS_HIRCTCTL_BOUNDARY_Msk       (0x1ful << SYS_HIRCTCTL_BOUNDARY_Pos)       /*!< SYS_T::HIRCTCTL: BOUNDARY Mask             */

#define SYS_HIRCTIEN_TFAILIEN_Pos       (1)                                         /*!< SYS_T::HIRCTIEN: TFAILIEN Position         */
#define SYS_HIRCTIEN_TFAILIEN_Msk       (0x1ul << SYS_HIRCTIEN_TFAILIEN_Pos)        /*!< SYS_T::HIRCTIEN: TFAILIEN Mask             */

#define SYS_HIRCTIEN_CLKEIEN_Pos        (2)                                         /*!< SYS_T::HIRCTIEN: CLKEIEN Position          */
#define SYS_HIRCTIEN_CLKEIEN_Msk        (0x1ul << SYS_HIRCTIEN_CLKEIEN_Pos)         /*!< SYS_T::HIRCTIEN: CLKEIEN Mask              */

#define SYS_HIRCTISTS_FREQLOCK_Pos      (0)                                         /*!< SYS_T::HIRCTISTS: FREQLOCK Position        */
#define SYS_HIRCTISTS_FREQLOCK_Msk      (0x1ul << SYS_HIRCTISTS_FREQLOCK_Pos)       /*!< SYS_T::HIRCTISTS: FREQLOCK Mask            */

#define SYS_HIRCTISTS_TFAILIF_Pos       (1)                                         /*!< SYS_T::HIRCTISTS: TFAILIF Position         */
#define SYS_HIRCTISTS_TFAILIF_Msk       (0x1ul << SYS_HIRCTISTS_TFAILIF_Pos)        /*!< SYS_T::HIRCTISTS: TFAILIF Mask             */

#define SYS_HIRCTISTS_CLKERRIF_Pos      (2)                                         /*!< SYS_T::HIRCTISTS: CLKERRIF Position        */
#define SYS_HIRCTISTS_CLKERRIF_Msk      (0x1ul << SYS_HIRCTISTS_CLKERRIF_Pos)       /*!< SYS_T::HIRCTISTS: CLKERRIF Mask            */

#define SYS_HIRCTISTS_OVBDIF_Pos        (3)                                         /*!< SYS_T::HIRCTISTS: OVBDIF Position          */
#define SYS_HIRCTISTS_OVBDIF_Msk        (0x1ul << SYS_HIRCTISTS_OVBDIF_Pos)         /*!< SYS_T::HIRCTISTS: OVBDIF Mask              */

#define SYS_REGLCTL_REGLCTL_Pos         (0)                                         /*!< SYS_T::REGLCTL: REGLCTL Position           */
#define SYS_REGLCTL_REGLCTL_Msk         (0xfful << SYS_REGLCTL_REGLCTL_Pos)         /*!< SYS_T::REGLCTL: REGLCTL Mask               */

#define SYS_PORDISAN_POROFFAN_Pos       (0)                                         /*!< SYS_T::PORDISAN: POROFFAN Position         */
#define SYS_PORDISAN_POROFFAN_Msk       (0xfffful << SYS_PORDISAN_POROFFAN_Pos)     /*!< SYS_T::PORDISAN: POROFFAN Mask             */

#define SYS_CSERVER_VERSION_Pos         (0)                                         /*!< SYS_T::CSERVER: VERSION Position           */
#define SYS_CSERVER_VERSION_Msk         (0xfful << SYS_CSERVER_VERSION_Pos)         /*!< SYS_T::CSERVER: VERSION Mask               */

#define SYS_PLCTL_MVRS_Pos              (4)                                         /*!< SYS_T::PLCTL: MVRS Position                */
#define SYS_PLCTL_MVRS_Msk              (0x1ul << SYS_PLCTL_MVRS_Pos)               /*!< SYS_T::PLCTL: MVRS Mask                    */

#define SYS_PLSTS_MVRCBUSY_Pos          (1)                                         /*!< SYS_T::PLSTS: MVRCBUSY Position            */
#define SYS_PLSTS_MVRCBUSY_Msk          (0x1ul << SYS_PLSTS_MVRCBUSY_Pos)           /*!< SYS_T::PLSTS: MVRCBUSY Mask                */

#define SYS_PLSTS_MVRCERR_Pos           (2)                                         /*!< SYS_T::PLSTS: MVRCERR Position             */
#define SYS_PLSTS_MVRCERR_Msk           (0x1ul << SYS_PLSTS_MVRCERR_Pos)            /*!< SYS_T::PLSTS: MVRCERR Mask                 */

#define SYS_PLSTS_LCONS_Pos             (3)                                         /*!< SYS_T::PLSTS: LCONS Position               */
#define SYS_PLSTS_LCONS_Msk             (0x1ul << SYS_PLSTS_LCONS_Pos)              /*!< SYS_T::PLSTS: LCONS Mask                   */

#define SYS_PLSTS_CURMVR_Pos            (12)                                        /*!< SYS_T::PLSTS: CURMVR Position              */
#define SYS_PLSTS_CURMVR_Msk            (0x1ul << SYS_PLSTS_CURMVR_Pos)             /*!< SYS_T::PLSTS: CURMVR Mask                  */

#define SYS_INIVTOR_INIVTOR_Pos         (10)                                        /*!< SYS_T::INIVTOR: INIVTOR Position           */
#define SYS_INIVTOR_INIVTOR_Msk         (0x3ffffful << SYS_INIVTOR_INIVTOR_Pos)     /*!< SYS_T::INIVTOR: INIVTOR Mask               */

#define NMI_NMIEN_BODOUT_Pos            (0)                                         /*!< NMI_T::NMIEN: BODOUT Position              */
#define NMI_NMIEN_BODOUT_Msk            (0x1ul << NMI_NMIEN_BODOUT_Pos)             /*!< NMI_T::NMIEN: BODOUT Mask                  */

#define NMI_NMIEN_IRC_INT_Pos           (1)                                         /*!< NMI_T::NMIEN: IRC_INT Position             */
#define NMI_NMIEN_IRC_INT_Msk           (0x1ul << NMI_NMIEN_IRC_INT_Pos)            /*!< NMI_T::NMIEN: IRC_INT Mask                 */

#define NMI_NMIEN_PWRWU_INT_Pos         (2)                                         /*!< NMI_T::NMIEN: PWRWU_INT Position           */
#define NMI_NMIEN_PWRWU_INT_Msk         (0x1ul << NMI_NMIEN_PWRWU_INT_Pos)          /*!< NMI_T::NMIEN: PWRWU_INT Mask               */

#define NMI_NMIEN_CLKFAIL_Pos           (4)                                         /*!< NMI_T::NMIEN: CLKFAIL Position             */
#define NMI_NMIEN_CLKFAIL_Msk           (0x1ul << NMI_NMIEN_CLKFAIL_Pos)            /*!< NMI_T::NMIEN: CLKFAIL Mask                 */

#define NMI_NMIEN_RTC_INT_Pos           (6)                                         /*!< NMI_T::NMIEN: RTC_INT Position             */
#define NMI_NMIEN_RTC_INT_Msk           (0x1ul << NMI_NMIEN_RTC_INT_Pos)            /*!< NMI_T::NMIEN: RTC_INT Mask                 */

#define NMI_NMIEN_EINT0_Pos             (8)                                         /*!< NMI_T::NMIEN: EINT0 Position               */
#define NMI_NMIEN_EINT0_Msk             (0x1ul << NMI_NMIEN_EINT0_Pos)              /*!< NMI_T::NMIEN: EINT0 Mask                   */

#define NMI_NMIEN_EINT1_Pos             (9)                                         /*!< NMI_T::NMIEN: EINT1 Position               */
#define NMI_NMIEN_EINT1_Msk             (0x1ul << NMI_NMIEN_EINT1_Pos)              /*!< NMI_T::NMIEN: EINT1 Mask                   */

#define NMI_NMIEN_EINT2_Pos             (10)                                        /*!< NMI_T::NMIEN: EINT2 Position               */
#define NMI_NMIEN_EINT2_Msk             (0x1ul << NMI_NMIEN_EINT2_Pos)              /*!< NMI_T::NMIEN: EINT2 Mask                   */

#define NMI_NMIEN_EINT3_Pos             (11)                                        /*!< NMI_T::NMIEN: EINT3 Position               */
#define NMI_NMIEN_EINT3_Msk             (0x1ul << NMI_NMIEN_EINT3_Pos)              /*!< NMI_T::NMIEN: EINT3 Mask                   */

#define NMI_NMIEN_EINT4_Pos             (12)                                        /*!< NMI_T::NMIEN: EINT4 Position               */
#define NMI_NMIEN_EINT4_Msk             (0x1ul << NMI_NMIEN_EINT4_Pos)              /*!< NMI_T::NMIEN: EINT4 Mask                   */

#define NMI_NMIEN_EINT5_Pos             (13)                                        /*!< NMI_T::NMIEN: EINT5 Position               */
#define NMI_NMIEN_EINT5_Msk             (0x1ul << NMI_NMIEN_EINT5_Pos)              /*!< NMI_T::NMIEN: EINT5 Mask                   */

#define NMI_NMIEN_UART0_INT_Pos         (14)                                        /*!< NMI_T::NMIEN: UART0_INT Position           */
#define NMI_NMIEN_UART0_INT_Msk         (0x1ul << NMI_NMIEN_UART0_INT_Pos)          /*!< NMI_T::NMIEN: UART0_INT Mask               */

#define NMI_NMIEN_UART1_INT_Pos         (15)                                        /*!< NMI_T::NMIEN: UART1_INT Position           */
#define NMI_NMIEN_UART1_INT_Msk         (0x1ul << NMI_NMIEN_UART1_INT_Pos)          /*!< NMI_T::NMIEN: UART1_INT Mask               */

#define NMI_NMIEN_UART2_INT_Pos         (16)                                        /*!< NMI_T::NMIEN: UART2_INT Position           */
#define NMI_NMIEN_UART2_INT_Msk         (0x1ul << NMI_NMIEN_UART2_INT_Pos)          /*!< NMI_T::NMIEN: UART2_INT Mask               */

#define NMI_NMIEN_EINT6_Pos             (20)                                        /*!< NMI_T::NMIEN: EINT6 Position               */
#define NMI_NMIEN_EINT6_Msk             (0x1ul << NMI_NMIEN_EINT6_Pos)              /*!< NMI_T::NMIEN: EINT6 Mask                   */

#define NMI_NMIEN_EINT7_Pos             (21)                                        /*!< NMI_T::NMIEN: EINT7 Position               */
#define NMI_NMIEN_EINT7_Msk             (0x1ul << NMI_NMIEN_EINT7_Pos)              /*!< NMI_T::NMIEN: EINT7 Mask                   */

#define NMI_NMIEN_INTMUX_Pos            (22)                                        /*!< NMI_T::NMIEN: INTMUX Position              */
#define NMI_NMIEN_INTMUX_Msk            (0x1ul << NMI_NMIEN_INTMUX_Pos)             /*!< NMI_T::NMIEN: INTMUX Mask                  */

#define NMI_NMISTS_BODOUT_Pos           (0)                                         /*!< NMI_T::NMISTS: BODOUT Position             */
#define NMI_NMISTS_BODOUT_Msk           (0x1ul << NMI_NMISTS_BODOUT_Pos)            /*!< NMI_T::NMISTS: BODOUT Mask                 */

#define NMI_NMISTS_IRC_INT_Pos          (1)                                         /*!< NMI_T::NMISTS: IRC_INT Position            */
#define NMI_NMISTS_IRC_INT_Msk          (0x1ul << NMI_NMISTS_IRC_INT_Pos)           /*!< NMI_T::NMISTS: IRC_INT Mask                */

#define NMI_NMISTS_PWRWU_INT_Pos        (2)                                         /*!< NMI_T::NMISTS: PWRWU_INT Position          */
#define NMI_NMISTS_PWRWU_INT_Msk        (0x1ul << NMI_NMISTS_PWRWU_INT_Pos)         /*!< NMI_T::NMISTS: PWRWU_INT Mask              */

#define NMI_NMISTS_CLKFAIL_Pos          (4)                                         /*!< NMI_T::NMISTS: CLKFAIL Position            */
#define NMI_NMISTS_CLKFAIL_Msk          (0x1ul << NMI_NMISTS_CLKFAIL_Pos)           /*!< NMI_T::NMISTS: CLKFAIL Mask                */

#define NMI_NMISTS_RTC_INT_Pos          (6)                                         /*!< NMI_T::NMISTS: RTC_INT Position            */
#define NMI_NMISTS_RTC_INT_Msk          (0x1ul << NMI_NMISTS_RTC_INT_Pos)           /*!< NMI_T::NMISTS: RTC_INT Mask                */

#define NMI_NMISTS_EINT0_Pos            (8)                                         /*!< NMI_T::NMISTS: EINT0 Position              */
#define NMI_NMISTS_EINT0_Msk            (0x1ul << NMI_NMISTS_EINT0_Pos)             /*!< NMI_T::NMISTS: EINT0 Mask                  */

#define NMI_NMISTS_EINT1_Pos            (9)                                         /*!< NMI_T::NMISTS: EINT1 Position              */
#define NMI_NMISTS_EINT1_Msk            (0x1ul << NMI_NMISTS_EINT1_Pos)             /*!< NMI_T::NMISTS: EINT1 Mask                  */

#define NMI_NMISTS_EINT2_Pos            (10)                                        /*!< NMI_T::NMISTS: EINT2 Position              */
#define NMI_NMISTS_EINT2_Msk            (0x1ul << NMI_NMISTS_EINT2_Pos)             /*!< NMI_T::NMISTS: EINT2 Mask                  */

#define NMI_NMISTS_EINT3_Pos            (11)                                        /*!< NMI_T::NMISTS: EINT3 Position              */
#define NMI_NMISTS_EINT3_Msk            (0x1ul << NMI_NMISTS_EINT3_Pos)             /*!< NMI_T::NMISTS: EINT3 Mask                  */

#define NMI_NMISTS_EINT4_Pos            (12)                                        /*!< NMI_T::NMISTS: EINT4 Position              */
#define NMI_NMISTS_EINT4_Msk            (0x1ul << NMI_NMISTS_EINT4_Pos)             /*!< NMI_T::NMISTS: EINT4 Mask                  */

#define NMI_NMISTS_EINT5_Pos            (13)                                        /*!< NMI_T::NMISTS: EINT5 Position              */
#define NMI_NMISTS_EINT5_Msk            (0x1ul << NMI_NMISTS_EINT5_Pos)             /*!< NMI_T::NMISTS: EINT5 Mask                  */

#define NMI_NMISTS_UART0_INT_Pos        (14)                                        /*!< NMI_T::NMISTS: UART0_INT Position          */
#define NMI_NMISTS_UART0_INT_Msk        (0x1ul << NMI_NMISTS_UART0_INT_Pos)         /*!< NMI_T::NMISTS: UART0_INT Mask              */

#define NMI_NMISTS_UART1_INT_Pos        (15)                                        /*!< NMI_T::NMISTS: UART1_INT Position          */
#define NMI_NMISTS_UART1_INT_Msk        (0x1ul << NMI_NMISTS_UART1_INT_Pos)         /*!< NMI_T::NMISTS: UART1_INT Mask              */

#define NMI_NMISTS_UART2_INT_Pos        (16)                                        /*!< NMI_T::NMISTS: UART2_INT Position          */
#define NMI_NMISTS_UART2_INT_Msk        (0x1ul << NMI_NMISTS_UART2_INT_Pos)         /*!< NMI_T::NMISTS: UART2_INT Mask              */

#define NMI_NMISTS_EINT6_Pos            (20)                                        /*!< NMI_T::NMISTS: EINT6 Position              */
#define NMI_NMISTS_EINT6_Msk            (0x1ul << NMI_NMISTS_EINT6_Pos)             /*!< NMI_T::NMISTS: EINT6 Mask                  */

#define NMI_NMISTS_EINT7_Pos            (21)                                        /*!< NMI_T::NMISTS: EINT7 Position              */
#define NMI_NMISTS_EINT7_Msk            (0x1ul << NMI_NMISTS_EINT7_Pos)             /*!< NMI_T::NMISTS: EINT7 Mask                  */

#define NMI_NMISTS_INTMUX_Pos           (22)                                        /*!< NMI_T::NMISTS: INTMUX Position             */
#define NMI_NMISTS_INTMUX_Msk           (0x1ul << NMI_NMISTS_INTMUX_Pos)            /*!< NMI_T::NMISTS: INTMUX Mask                 */

#define NMI_NMIMSEL_NMIMSEL_Pos         (0)                                         /*!< NMI_T::NMIMSEL: NMIMSEL Position           */
#define NMI_NMIMSEL_NMIMSEL_Msk         (0x3Ful << NMI_NMIMSEL_NMIMSEL_Pos)         /*!< NMI_T::NMIMSEL: NMIMSEL Mask               */

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __SYS_REG_H__ */
