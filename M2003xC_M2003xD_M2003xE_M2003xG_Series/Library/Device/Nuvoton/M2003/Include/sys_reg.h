/**************************************************************************//**
 * @file     sys_reg.h
 * @version  V1.00
 * @brief    SYS register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SYS_REG_H__
#define __SYS_REG_H__

/** @addtogroup REGISTER Control Register
  @{
*/

/*---------------------- System Manger Controller -------------------------*/
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
     * |        |          |This register reflects device part number code.
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
     * |[1]     |PINRF     |nRESET Pin Reset Flag
     * |        |          |The nRESET pin reset flag is set by the "Reset Signal" from the nRESET Pin to indicate the previous reset source.
     * |        |          |0 = No reset from nRESET pin.
     * |        |          |1 = Pin nRESET had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[2]     |WDTRF     |WDT Reset Flag
     * |        |          |The WDT reset flag is set by the "Reset Signal" from the Watchdog Timer or Window Watchdog Timer to indicate the previous reset source.
     * |        |          |0 = No reset from watchdog timer or window watchdog timer.
     * |        |          |1 = The watchdog timer or window watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: Watchdog Timer register RSTF(WDT_CTL[2]) bit is set if the system has been reset by WDT time-out reset.
     * |        |          |Window Watchdog Timer register WWDTRF(WWDT_STATUS[1]) bit is set if the system has been reset by WWDT time-out reset.
     * |[3]     |LVRF      |LVR Reset Flag
     * |        |          |The LVR reset flag is set by the "Reset Signal" from the Low Voltage Reset Controller to indicate the previous reset source, and LVR reset flag is not set in SPD mode when LVR reset occur.
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = The LVR controller had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |BODRF     |BOD Reset Flag
     * |        |          |The BOD reset flag is set by the "Reset Signal" from the Brown-out Detector to indicate the previous reset source, and BOD reset flag is not set in SPD mode when BOD reset occur.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[5]     |SYSRF     |System Reset Flag
     * |        |          |The system reset flag is set by the "Reset Signal" from the Cortex-M23 core to indicate the previous reset source.
     * |        |          |0 = No reset from Cortex-M23.
     * |        |          |1 = The Cortex-M23 had issued the reset signal to reset the system by writing 1 to the bit SYSRESETREQ(AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M23 core.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[7]     |CPURF     |CPU Reset Flag
     * |        |          |The CPU reset flag is set by hardware if software writes CPURST (SYS_IPRST0[1]) 1 to reset Cortex-M23 core and Flash Memory Controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = The Cortex-M23 core and FMC are reset by software setting CPURST to 1.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[8]     |CPULKRF   |CPU Lockup Reset Flag
     * |        |          |The CPU Lockup reset flag is set by hardware if Cortex-M23 lockup happened.
     * |        |          |0 = No reset from CPU lockup happened.
     * |        |          |1 = The Cortex-M23 lockup happened and chip is reset.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: When CPU lockup happened under ICE is connected, this flag will set to 1 but chip will not reset.
     * @var SYS_T::IPRST0
     * Offset: 0x08  Peripheral Reset Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIPRST   |Chip One-shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the whole chip, including Processor core and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIPRST is same as the POR reset, all the chip controllers is reset and the chip setting from Flash are also reload.
     * |        |          |About the difference between CHIPRST and SYSRESETREQ(AIRCR[2]), please refer to section 6.2.2
     * |        |          |0 = Chip normal operation.
     * |        |          |1 = Chip one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |CPURST    |Processor Core One-shot Reset (Write Protect)
     * |        |          |Setting this bit will only reset the processor core and Flash Memory Controller(FMC), and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |0 = Processor core normal operation.
     * |        |          |1 = Processor core one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |PDMARST   |PDMA Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the PDMA.
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = PDMA controller normal operation.
     * |        |          |1 = PDMA controller reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |CRCRST    |CRC Calculation Controller Reset (Write Protect)
     * |        |          |Set this bit to 1 will generate a reset signal to the CRC calculation controller.
     * |        |          |User needs to set this bit to 0 to release from the reset state.
     * |        |          |0 = CRC calculation controller normal operation.
     * |        |          |1 = CRC calculation controller reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::IPRST1
     * Offset: 0x0C  Peripheral Reset Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIORST   |GPIO Controller Reset
     * |        |          |0 = GPIO controller normal operation.
     * |        |          |1 = GPIO controller reset.
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
     * |[8]     |I2C0RST   |I2C0 Controller Reset
     * |        |          |0 = I2C0 controller normal operation.
     * |        |          |1 = I2C0 controller reset.
     * |[12]    |QSPI0RST  |Qual SPI0 Controller Reset
     * |        |          |0 = Qual SPI0 controller normal operation.
     * |        |          |1 = Qual SPI0 controller reset.
     * |[16]    |UART0RST  |UART0 Controller Reset
     * |        |          |0 = UART0 controller normal operation.
     * |        |          |1 = UART0 controller reset.
     * |[17]    |UART1RST  |UART1 Controller Reset
     * |        |          |0 = UART1 controller normal operation.
     * |        |          |1 = UART1 controller reset.
     * |[18]    |UART2RST  |UART2 Controller Reset
     * |        |          |0 = UART2 controller normal operation.
     * |        |          |1 = UART2 controller reset.
     * |[24]    |CAN0RST   |CAN0 Controller Reset
     * |        |          |0 = CAN0 controller normal operation.
     * |        |          |1 = CAN0 controller reset.
     * |[28]    |ADCRST    |ADC Controller Reset
     * |        |          |0 = ADC controller normal operation.
     * |        |          |1 = ADC controller reset.
     * @var SYS_T::IPRST2
     * Offset: 0x10  Peripheral Reset Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |USCI0RST  |USCI0 Controller Reset
     * |        |          |0 = USCI0 controller normal operation.
     * |        |          |1 = USCI0 controller reset.
     * |[9]     |USCI1RST  |USCI1 Controller Reset
     * |        |          |0 = USCI1 controller normal operation.
     * |        |          |1 = USCI1 controller reset.
     * |[10]    |USCI2RST  |USCI1 Controller Reset
     * |        |          |0 = USCI2 controller normal operation.
     * |        |          |1 = USCI2 controller reset.
     * |[11]    |USCI3RST  |USCI3 Controller Reset
     * |        |          |0 = USCI3 controller normal operation.
     * |        |          |1 = USCI3 controller reset.
     * |[16]    |PWM0RST   |PWM0 Controller Reset
     * |        |          |0 = PWM0 controller normal operation.
     * |        |          |1 = PWM0 controller reset.
     * |[17]    |PWM1RST   |PWM1 Controller Reset
     * |        |          |0 = PWM1 controller normal operation.
     * |        |          |1 = PWM1 controller reset.
     * |[26]    |ECAP0RST  |ECAP0 Controller Reset
     * |        |          |0 = ECAP0 controller normal operation.
     * |        |          |1 = ECAP0 controller reset.
     * @var SYS_T::BODCTL
     * Offset: 0x18  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODEN     |Brown-out Detector Enable Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBODEN (CONFIG0 [23]).
     * |        |          |0 = Brown-out Detector function Disabled.
     * |        |          |1 = Brown-out Detector function Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |BODRSTEN  |Brown-out Reset Enable Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBORST(CONFIG0[20]) bit.
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled.
     * |        |          |1 = Brown-out "RESET" function Enabled.
     * |        |          |Note 1: When the Brown-out Detector function is enabled (BODEN high) and BOD reset function is enabled (BODRSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BODOUT high).
     * |        |          |When the BOD function is enabled (BODEN high) and BOD interrupt function is enabled (BODRSTEN low), BOD will assert an interrupt if AVDD is lower than BODVL, BOD interrupt will keep till the BODIF set to 0.
     * |        |          |BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (setting BODEN low).
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |BODIF     |Brown-out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BODVL setting.
     * |        |          |1 = When Brown-out Detector detects the VDD is dropped down through the voltage of BODVL setting or the VDD is raised up through the voltage of BODVL setting, this bit is set to 1 and the brown-out interrupt is requested if brown-out interrupt is enabled.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[5]     |BODLPM    |Brown-out Detector Low Power Mode (Write Protect)
     * |        |          |0 = BOD operate in normal mode (default).
     * |        |          |1 = BOD Low Power mode Enabled.
     * |        |          |Note 1: The BOD consumes about 100uA in normal mode, the low power mode can reduce the current to about 1/10 but slow the BOD response.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |BODOUT    |Brown-out Detector Output Status (Read Only)
     * |        |          |0 = Brown-out Detector output status is 0.
     * |        |          |It means the detected voltage is higher than BODVL setting or BODEN is 0.
     * |        |          |1 = Brown-out Detector output status is 1.
     * |        |          |It means the detected voltage is lower than BODVL setting.
     * |        |          |If the BODEN is 0, BOD function disabled, this bit always responds 0.
     * |[7]     |LVREN     |Low Voltage Reset Enable Bit (Write Protect)
     * |        |          |The LVR function resets the chip when the input power voltage is lower than LVR circuit setting.
     * |        |          |LVR function is enabled by default.
     * |        |          |0 = Low Voltage Reset function Disabled.
     * |        |          |1 = Low Voltage Reset function Enabled.
     * |        |          |Note 1: After enabling the bit, the LVR function will be active with 200us delay for LVR output stable (default).
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[10:8]  |BODDGSEL  |Brown-out Detector Output De-glitch Time Select (Write Protect)
     * |        |          |000 = BOD output is sampled by LIRC clock.
     * |        |          |001 = 4 system clock (HCLK).
     * |        |          |010 = 8 system clock (HCLK).
     * |        |          |011 = 16 system clock (HCLK).
     * |        |          |100 = 32 system clock (HCLK).
     * |        |          |101 = 64 system clock (HCLK).
     * |        |          |110 = 128 system clock (HCLK).
     * |        |          |111 = 256 system clock (HCLK).
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[14:12] |LVRDGSEL  |LVR Output De-glitch Time Select (Write Protect)
     * |        |          |000 = Without de-glitch function.
     * |        |          |001 = 4 system clock (HCLK).
     * |        |          |010 = 8 system clock (HCLK).
     * |        |          |011 = 16 system clock (HCLK).
     * |        |          |100 = 32 system clock (HCLK).
     * |        |          |101 = 64 system clock (HCLK).
     * |        |          |110 = 128 system clock (HCLK).
     * |        |          |111 = 256 system clock (HCLK).
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[18:16] |BODVL     |Brown-out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBOV (CONFIG0 [23:21]).
     * |        |          |000 = Brown-out Detector threshold voltage is 2.2V.
     * |        |          |001 = Brown-out Detector threshold voltage is 2.2V.
     * |        |          |010 = Brown-out Detector threshold voltage is 2.7V.
     * |        |          |011 = Brown-out Detector threshold voltage is 2.7V.
     * |        |          |100 = Brown-out Detector threshold voltage is 3.7V.
     * |        |          |101 = Brown-out Detector threshold voltage is 3.7V.
     * |        |          |110 = Brown-out Detector threshold voltage is 4.4V.
     * |        |          |111 = Brown-out Detector threshold voltage is 4.4V.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[31]    |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If SYS_BODCTL is written, this bit is asserted automatically by hardware, and is de-asserted when write procedure is finished.
     * |        |          |0 = SYS_BODCTL register is ready for write operation, and indicates LVR and BOD already stable, system cannot detect LVR and BOD event when this bit is not de-asserted.
     * |        |          |1 = SYS_BODCTL register is busy on the last write operation. Other write operations are ignored.
     * @var SYS_T::IVSCTL
     * Offset: 0x1C  Internal Voltage Source Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VTEMPEN   |Temperature Sensor Enable Bit
     * |        |          |This bit is used to enable/disable temperature sensor function.
     * |        |          |0 = Temperature sensor function Disabled (default).
     * |        |          |1 = Temperature sensor function Enabled.
     * |        |          |Note: After this bit is set to 1, the value of temperature sensor output can be obtained from ADC conversion result
     * |        |          |Please refer to ADC chapter for details.
     * @var SYS_T::PORCTL0
     * Offset: 0x24  Power-on Reset Controller Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PORMASK   |Power-on Reset Mask Enable Bit (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.
     * |        |          |User can mask  internal POR signal to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::GPA_MFPL
     * Offset: 0x30  GPIOA Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PA0MFP    |PA.0 Multi-function Pin Selection
     * |[7:4]   |PA1MFP    |PA.1 Multi-function Pin Selection
     * |[11:8]  |PA2MFP    |PA.2 Multi-function Pin Selection
     * |[15:12] |PA3MFP    |PA.3 Multi-function Pin Selection
     * |[19:16] |PA4MFP    |PA.4 Multi-function Pin Selection
     * |[23:20] |PA5MFP    |PA.5 Multi-function Pin Selection
     * |[27:24] |PA6MFP    |PA.6 Multi-function Pin Selection
     * |[31:28] |PA7MFP    |PA.7 Multi-function Pin Selection
     * @var SYS_T::GPA_MFPH
     * Offset: 0x34  GPIOA High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PA8MFP    |PA.8 Multi-function Pin Selection
     * |[7:4]   |PA9MFP    |PA.9 Multi-function Pin Selection
     * |[11:8]  |PA10MFP   |PA.10 Multi-function Pin Selection
     * |[15:12] |PA11MFP   |PA.11 Multi-function Pin Selection
     * |[19:16] |PA12MFP   |PA.12 Multi-function Pin Selection
     * |[23:20] |PA13MFP   |PA.13 Multi-function Pin Selection
     * |[27:24] |PA14MFP   |PA.14 Multi-function Pin Selection
     * |[31:28] |PA15MFP   |PA.15 Multi-function Pin Selection
     * @var SYS_T::GPB_MFPL
     * Offset: 0x38  GPIOB Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PB0MFP    |PB.0 Multi-function Pin Selection
     * |[7:4]   |PB1MFP    |PB.1 Multi-function Pin Selection
     * |[11:8]  |PB2MFP    |PB.2 Multi-function Pin Selection
     * |[15:12] |PB3MFP    |PB.3 Multi-function Pin Selection
     * |[19:16] |PB4MFP    |PB.4 Multi-function Pin Selection
     * |[23:20] |PB5MFP    |PB.5 Multi-function Pin Selection
     * |[27:24] |PB6MFP    |PB.6 Multi-function Pin Selection
     * |[31:28] |PB7MFP    |PB.7 Multi-function Pin Selection
     * @var SYS_T::GPB_MFPH
     * Offset: 0x3C  GPIOB High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PB8MFP    |PB.8 Multi-function Pin Selection
     * |[7:4]   |PB9MFP    |PB.9 Multi-function Pin Selection
     * |[11:8]  |PB10MFP   |PB.10 Multi-function Pin Selection
     * |[15:12] |PB11MFP   |PB.11 Multi-function Pin Selection
     * |[19:16] |PB12MFP   |PB.12 Multi-function Pin Selection
     * |[23:20] |PB13MFP   |PB.13 Multi-function Pin Selection
     * |[27:24] |PB14MFP   |PB.14 Multi-function Pin Selection
     * |[31:28] |PB15MFP   |PB.15 Multi-function Pin Selection
     * @var SYS_T::GPC_MFPL
     * Offset: 0x40  GPIOC Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PC0MFP    |PC.0 Multi-function Pin Selection
     * |[7:4]   |PC1MFP    |PC.1 Multi-function Pin Selection
     * |[11:8]  |PC2MFP    |PC.2 Multi-function Pin Selection
     * |[15:12] |PC3MFP    |PC.3 Multi-function Pin Selection
     * |[19:16] |PC4MFP    |PC.4 Multi-function Pin Selection
     * |[23:20] |PC5MFP    |PC.5 Multi-function Pin Selection
     * |[27:24] |PC6MFP    |PC.6 Multi-function Pin Selection
     * |[31:28] |PC7MFP    |PC.7 Multi-function Pin Selection
     * @var SYS_T::GPC_MFPH
     * Offset: 0x44  GPIOC High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[27:24] |PC14MFP   |PC.14 Multi-function Pin Selection
     * @var SYS_T::GPD_MFPL
     * Offset: 0x48  GPIOD Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PD0MFP    |PD.0 Multi-function Pin Selection
     * |[7:4]   |PD1MFP    |PD.1 Multi-function Pin Selection
     * |[11:8]  |PD2MFP    |PD.2 Multi-function Pin Selection
     * |[15:12] |PD3MFP    |PD.3 Multi-function Pin Selection
     * @var SYS_T::GPD_MFPH
     * Offset: 0x4C  GPIOD High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:28] |PD15MFP   |PD.15 Multi-function Pin Selection
     * @var SYS_T::GPE_MFPH
     * Offset: 0x54  GPIOE High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:28] |PE15MFP   |PE.15 Multi-function Pin Selection
     * @var SYS_T::GPF_MFPL
     * Offset: 0x58  GPIOF Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PF0MFP    |PF.0 Multi-function Pin Selection
     * |[7:4]   |PF1MFP    |PF.1 Multi-function Pin Selection
     * |[11:8]  |PF2MFP    |PF.2 Multi-function Pin Selection
     * |[15:12] |PF3MFP    |PF.3 Multi-function Pin Selection
     * |[19:16] |PF4MFP    |PF.4 Multi-function Pin Selection
     * |[23:20] |PF5MFP    |PF.5 Multi-function Pin Selection
     * |[27:24] |PF6MFP    |PF.6 Multi-function Pin Selection
     * @var SYS_T::GPF_MFPH
     * Offset: 0x5C  GPIOF High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[27:24] |PF14MFP   |PF.14 Multi-function Pin Selection
     * |[31:28] |PF15MFP   |PF.15 Multi-function Pin Selection
     * @var SYS_T::GPG_MFPH
     * Offset: 0x64  GPIOG High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:4]   |PG9MFP    |PG.9 Multi-function Pin Selection
     * |[11:8]  |PG10MFP   |PG.10 Multi-function Pin Selection
     * |[15:12] |PG11MFP   |PG.11 Multi-function Pin Selection
     * |[19:16] |PG12MFP   |PG.12 Multi-function Pin Selection
     * @var SYS_T::GPA_MFOS
     * Offset: 0x80  GPIOA Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PA0MFOS   |PA.0 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.0 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[1]     |PA1MFOS   |PA.1 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.1 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[2]     |PA2MFOS   |PA.2 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.2 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[3]     |PA3MFOS   |PA.3 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.3 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[4]     |PA4MFOS   |PA.4 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.4 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[5]     |PA5MFOS   |PA.5 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.5 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[6]     |PA6MFOS   |PA.6 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.6 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[7]     |PA7MFOS   |PA.7 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.7 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[8]     |PA8MFOS   |PA.8 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.8 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[9]     |PA9MFOS   |PA.9 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.9 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[10]    |PA10MFOS  |PA.10 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.10 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[11]    |PA11MFOS  |PA.11 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.11 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[12]    |PA12MFOS  |PA.12 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.12 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[13]    |PA13MFOS  |PA.13 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.13 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[14]    |PA14MFOS  |PA.14 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.14 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[15]    |PA15MFOS  |PA.15 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPB_MFOS
     * Offset: 0x84  GPIOB Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PB0MFOS   |PB.0 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.0 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[1]     |PB1MFOS   |PB.1 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.1 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[2]     |PB2MFOS   |PB.2 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.2 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[3]     |PB3MFOS   |PB.3 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.3 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[4]     |PB4MFOS   |PB.4 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.4 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[5]     |PB5MFOS   |PB.5 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.5 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[6]     |PB6MFOS   |PB.6 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.6 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[7]     |PB7MFOS   |PB.7 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.7 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[8]     |PB8MFOS   |PB.8 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.8 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[9]     |PB9MFOS   |PB.9 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.9 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[10]    |PB10MFOS  |PB.10 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.10 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[11]    |PB11MFOS  |PB.11 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.11 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[12]    |PB12MFOS  |PB.12 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.12 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[13]    |PB13MFOS  |PB.13 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.13 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[14]    |PB14MFOS  |PB.14 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.14 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[15]    |PB15MFOS  |PB.15 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPC_MFOS
     * Offset: 0x88  GPIOC Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PC0MFOS   |PC.0 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.0 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[1]     |PC1MFOS   |PC.1 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.1 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[2]     |PC2MFOS   |PC.2 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.2 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[3]     |PC3MFOS   |PC.3 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.3 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[4]     |PC4MFOS   |PC.4 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.4 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[5]     |PC5MFOS   |PC.5 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.5 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[6]     |PC6MFOS   |PC.6 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.6 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[7]     |PC7MFOS   |PC.7 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.7 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[14]    |PC14MFOS  |PC.14 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.14 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPD_MFOS
     * Offset: 0x8C  GPIOD Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PD0MFOS   |PD.0 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.0 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[1]     |PD1MFOS   |PD.1 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.1 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[2]     |PD2MFOS   |PD.2 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.2 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[3]     |PD3MFOS   |PD.3 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.3 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[15]    |PD15MFOS  |PD.15 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPF_MFOS
     * Offset: 0x94  GPIOF Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PF0MFOS   |PF.0 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.0 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[1]     |PF1MFOS   |PF.1 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.1 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[2]     |PF2MFOS   |PF.2 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.2 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[3]     |PF3MFOS   |PF.3 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.3 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[4]     |PF4MFOS   |PF.4 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.4 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[5]     |PF5MFOS   |PF.5 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.5 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[15]    |PF15MFOS  |PF.15 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.15 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPG_MFOS
     * Offset: 0x98  GPIOG Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9]     |PG9MFOS   |PG.9 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.9 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[10]    |PG10MFOS  |PG.10 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.10 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[11]    |PG11MFOS  |PG.11 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.11 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[12]    |PG12MFOS  |PG.12 Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.12 pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::REGLCTL
     * Offset: 0x100  Register Lock Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |REGLCTL   |Register Lock Control Code
     * |        |          |Some registers have write-protection function
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value "59h", "16h", "88h" to this field.
     * |        |          |After this sequence is completed, the REGLCTL bit will be set to 1 and write-protection registers can be normal write.
     * |        |          |REGLCTL[0]
     * |        |          |Register Lock Control Disable Index
     * |        |          |0 = Write-protection Enabled for writing protected registers.
     * |        |          |Any write to the protected register is ignored.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * @var SYS_T::ADOFS
     * Offset: 0x148  ADC Offset Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |ADOFS     |ADC Offset Trim (Write Protect)
     * |        |          |000 = Offset no change
     * |        |          |001 = Offset change to +1 LSB
     * |        |          |010 = Offset change to +2 LSB
     * |        |          |011 = Offset change to +3 LSB
     * |        |          |100 = Offset change to -4 LSB
     * |        |          |101 = Offset change to -3 LSB
     * |        |          |110 = Offset change to -2 LSB
     * |        |          |111 = Offset change to -1 LSB
     * @var SYS_T::CPUCFG
     * Offset: 0x1D8  CPU General Configuration Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTRTEN   |CPU Interrupt Realtime Enable Bit
     * |        |          |When this bit is 0, the latency of CPU entering interrupt service routine (ISR) will be various but shorter.
     * |        |          |When this bit is 1, the latency of CPU entering ISR will be kept constant.
     * |        |          |0 = CPU Interrupt Realtime Disabled.
     * |        |          |1 = CPU Interrupt Realtime Enabled.
     * @var SYS_T::PORCTL1
     * Offset: 0x1EC  Power-on Reset Controller Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |POROFF    |Power-on Reset Enable Bit (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.
     * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR function will be active again when  this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     */

    __I  uint32_t PDID;                  /*!< [0x0000] Part Device Identification Number Register                       */
    __IO uint32_t RSTSTS;                /*!< [0x0004] System Reset Status Register                                     */
    __IO uint32_t IPRST0;                /*!< [0x0008] Peripheral  Reset Control Register 0                             */
    __IO uint32_t IPRST1;                /*!< [0x000c] Peripheral Reset Control Register 1                              */
    __IO uint32_t IPRST2;                /*!< [0x0010] Peripheral Reset Control Register 2                              */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t BODCTL;                /*!< [0x0018] Brown-out Detector Control Register                              */
    __IO uint32_t IVSCTL;                /*!< [0x001c] Internal Voltage Source Control Register                         */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t PORCTL0;               /*!< [0x0024] Power-on Reset Controller Register 0                             */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t GPA_MFPL;              /*!< [0x0030] GPIOA Low Byte Multiple Function Control Register                */
    __IO uint32_t GPA_MFPH;              /*!< [0x0034] GPIOA High Byte Multiple Function Control Register               */
    __IO uint32_t GPB_MFPL;              /*!< [0x0038] GPIOB Low Byte Multiple Function Control Register                */
    __IO uint32_t GPB_MFPH;              /*!< [0x003c] GPIOB High Byte Multiple Function Control Register               */
    __IO uint32_t GPC_MFPL;              /*!< [0x0040] GPIOC Low Byte Multiple Function Control Register                */
    __IO uint32_t GPC_MFPH;              /*!< [0x0044] GPIOC High Byte Multiple Function Control Register               */
    __IO uint32_t GPD_MFPL;              /*!< [0x0048] GPIOD Low Byte Multiple Function Control Register                */
    __IO uint32_t GPD_MFPH;              /*!< [0x004c] GPIOD High Byte Multiple Function Control Register               */
    __I  uint32_t RESERVE3[1];
    __IO uint32_t GPE_MFPH;              /*!< [0x0054] GPIOE High Byte Multiple Function Control Register               */
    __IO uint32_t GPF_MFPL;              /*!< [0x0058] GPIOF Low Byte Multiple Function Control Register                */
    __IO uint32_t GPF_MFPH;              /*!< [0x005c] GPIOF High Byte Multiple Function Control Register               */
    __I  uint32_t RESERVE4[1];
    __IO uint32_t GPG_MFPH;              /*!< [0x0064] GPIOG High Byte Multiple Function Control Register               */
    __I  uint32_t RESERVE5[6];
    __IO uint32_t GPA_MFOS;              /*!< [0x0080] GPIOA Multiple Function Output Select Register                   */
    __IO uint32_t GPB_MFOS;              /*!< [0x0084] GPIOB Multiple Function Output Select Register                   */
    __IO uint32_t GPC_MFOS;              /*!< [0x0088] GPIOC Multiple Function Output Select Register                   */
    __IO uint32_t GPD_MFOS;              /*!< [0x008c] GPIOD Multiple Function Output Select Register                   */
    __I  uint32_t RESERVE6[1];
    __IO uint32_t GPF_MFOS;              /*!< [0x0094] GPIOF Multiple Function Output Select Register                   */
    __IO uint32_t GPG_MFOS;              /*!< [0x0098] GPIOG Multiple Function Output Select Register                   */
    __I  uint32_t RESERVE7[25];
    __IO uint32_t REGLCTL;               /*!< [0x0100] Register Lock Control Register                                   */
    __I  uint32_t RESERVE8[17];
    __IO uint32_t ADOFS;                 /*!< [0x0148] ADC Offset Trim Control Register                                 */
    __I  uint32_t RESERVE9[35];
    __IO uint32_t CPUCFG;                /*!< [0x01d8] CPU General Configuration Register                               */
    __I  uint32_t RESERVE10[4];
    __IO uint32_t PORCTL1;               /*!< [0x01ec] Power-on Reset Controller Register 1                             */

} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
  @{
*/

#define SYS_PDID_PDID_Pos                (0)                                               /*!< SYS_T::PDID: PDID Position             */
#define SYS_PDID_PDID_Msk                (0xfffffffful << SYS_PDID_PDID_Pos)               /*!< SYS_T::PDID: PDID Mask                 */

#define SYS_RSTSTS_PORF_Pos              (0)                                               /*!< SYS_T::RSTSTS: PORF Position           */
#define SYS_RSTSTS_PORF_Msk              (0x1ul << SYS_RSTSTS_PORF_Pos)                    /*!< SYS_T::RSTSTS: PORF Mask               */

#define SYS_RSTSTS_PINRF_Pos             (1)                                               /*!< SYS_T::RSTSTS: PINRF Position          */
#define SYS_RSTSTS_PINRF_Msk             (0x1ul << SYS_RSTSTS_PINRF_Pos)                   /*!< SYS_T::RSTSTS: PINRF Mask              */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS_T::RSTSTS: WDTRF Position          */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS_T::RSTSTS: WDTRF Mask              */

#define SYS_RSTSTS_LVRF_Pos              (3)                                               /*!< SYS_T::RSTSTS: LVRF Position           */
#define SYS_RSTSTS_LVRF_Msk              (0x1ul << SYS_RSTSTS_LVRF_Pos)                    /*!< SYS_T::RSTSTS: LVRF Mask               */

#define SYS_RSTSTS_BODRF_Pos             (4)                                               /*!< SYS_T::RSTSTS: BODRF Position          */
#define SYS_RSTSTS_BODRF_Msk             (0x1ul << SYS_RSTSTS_BODRF_Pos)                   /*!< SYS_T::RSTSTS: BODRF Mask              */

#define SYS_RSTSTS_SYSRF_Pos             (5)                                               /*!< SYS_T::RSTSTS: SYSRF Position          */
#define SYS_RSTSTS_SYSRF_Msk             (0x1ul << SYS_RSTSTS_SYSRF_Pos)                   /*!< SYS_T::RSTSTS: SYSRF Mask              */

#define SYS_RSTSTS_CPURF_Pos             (7)                                               /*!< SYS_T::RSTSTS: CPURF Position          */
#define SYS_RSTSTS_CPURF_Msk             (0x1ul << SYS_RSTSTS_CPURF_Pos)                   /*!< SYS_T::RSTSTS: CPURF Mask              */

#define SYS_RSTSTS_CPULKRF_Pos           (8)                                               /*!< SYS_T::RSTSTS: CPULKRF Position        */
#define SYS_RSTSTS_CPULKRF_Msk           (0x1ul << SYS_RSTSTS_CPULKRF_Pos)                 /*!< SYS_T::RSTSTS: CPULKRF Mask            */

#define SYS_IPRST0_CHIPRST_Pos           (0)                                               /*!< SYS_T::IPRST0: CHIPRST Position        */
#define SYS_IPRST0_CHIPRST_Msk           (0x1ul << SYS_IPRST0_CHIPRST_Pos)                 /*!< SYS_T::IPRST0: CHIPRST Mask            */

#define SYS_IPRST0_CPURST_Pos            (1)                                               /*!< SYS_T::IPRST0: CPURST Position         */
#define SYS_IPRST0_CPURST_Msk            (0x1ul << SYS_IPRST0_CPURST_Pos)                  /*!< SYS_T::IPRST0: CPURST Mask             */

#define SYS_IPRST0_PDMARST_Pos           (2)                                               /*!< SYS_T::IPRST0: PDMARST Position        */
#define SYS_IPRST0_PDMARST_Msk           (0x1ul << SYS_IPRST0_PDMARST_Pos)                 /*!< SYS_T::IPRST0: PDMARST Mask            */

#define SYS_IPRST0_CRCRST_Pos            (7)                                               /*!< SYS_T::IPRST0: CRCRST Position         */
#define SYS_IPRST0_CRCRST_Msk            (0x1ul << SYS_IPRST0_CRCRST_Pos)                  /*!< SYS_T::IPRST0: CRCRST Mask             */

#define SYS_IPRST1_GPIORST_Pos           (1)                                               /*!< SYS_T::IPRST1: GPIORST Position        */
#define SYS_IPRST1_GPIORST_Msk           (0x1ul << SYS_IPRST1_GPIORST_Pos)                 /*!< SYS_T::IPRST1: GPIORST Mask            */

#define SYS_IPRST1_TMR0RST_Pos           (2)                                               /*!< SYS_T::IPRST1: TMR0RST Position        */
#define SYS_IPRST1_TMR0RST_Msk           (0x1ul << SYS_IPRST1_TMR0RST_Pos)                 /*!< SYS_T::IPRST1: TMR0RST Mask            */

#define SYS_IPRST1_TMR1RST_Pos           (3)                                               /*!< SYS_T::IPRST1: TMR1RST Position        */
#define SYS_IPRST1_TMR1RST_Msk           (0x1ul << SYS_IPRST1_TMR1RST_Pos)                 /*!< SYS_T::IPRST1: TMR1RST Mask            */

#define SYS_IPRST1_TMR2RST_Pos           (4)                                               /*!< SYS_T::IPRST1: TMR2RST Position        */
#define SYS_IPRST1_TMR2RST_Msk           (0x1ul << SYS_IPRST1_TMR2RST_Pos)                 /*!< SYS_T::IPRST1: TMR2RST Mask            */

#define SYS_IPRST1_TMR3RST_Pos           (5)                                               /*!< SYS_T::IPRST1: TMR3RST Position        */
#define SYS_IPRST1_TMR3RST_Msk           (0x1ul << SYS_IPRST1_TMR3RST_Pos)                 /*!< SYS_T::IPRST1: TMR3RST Mask            */

#define SYS_IPRST1_I2C0RST_Pos           (8)                                               /*!< SYS_T::IPRST1: I2C0RST Position        */
#define SYS_IPRST1_I2C0RST_Msk           (0x1ul << SYS_IPRST1_I2C0RST_Pos)                 /*!< SYS_T::IPRST1: I2C0RST Mask            */

#define SYS_IPRST1_QSPI0RST_Pos          (12)                                              /*!< SYS_T::IPRST1: QSPI0RST Position       */
#define SYS_IPRST1_QSPI0RST_Msk          (0x1ul << SYS_IPRST1_QSPI0RST_Pos)                /*!< SYS_T::IPRST1: QSPI0RST Mask           */

#define SYS_IPRST1_UART0RST_Pos          (16)                                              /*!< SYS_T::IPRST1: UART0RST Position       */
#define SYS_IPRST1_UART0RST_Msk          (0x1ul << SYS_IPRST1_UART0RST_Pos)                /*!< SYS_T::IPRST1: UART0RST Mask           */

#define SYS_IPRST1_UART1RST_Pos          (17)                                              /*!< SYS_T::IPRST1: UART1RST Position       */
#define SYS_IPRST1_UART1RST_Msk          (0x1ul << SYS_IPRST1_UART1RST_Pos)                /*!< SYS_T::IPRST1: UART1RST Mask           */

#define SYS_IPRST1_UART2RST_Pos          (18)                                              /*!< SYS_T::IPRST1: UART2RST Position       */
#define SYS_IPRST1_UART2RST_Msk          (0x1ul << SYS_IPRST1_UART2RST_Pos)                /*!< SYS_T::IPRST1: UART2RST Mask           */

#define SYS_IPRST1_CAN0RST_Pos           (24)                                              /*!< SYS_T::IPRST1: CAN0RST Position        */
#define SYS_IPRST1_CAN0RST_Msk           (0x1ul << SYS_IPRST1_CAN0RST_Pos)                 /*!< SYS_T::IPRST1: CAN0RST Mask            */

#define SYS_IPRST1_ADCRST_Pos            (28)                                              /*!< SYS_T::IPRST1: ADCRST Position         */
#define SYS_IPRST1_ADCRST_Msk            (0x1ul << SYS_IPRST1_ADCRST_Pos)                  /*!< SYS_T::IPRST1: ADCRST Mask             */

#define SYS_IPRST2_USCI0RST_Pos          (8)                                               /*!< SYS_T::IPRST2: USCI0RST Position       */
#define SYS_IPRST2_USCI0RST_Msk          (0x1ul << SYS_IPRST2_USCI0RST_Pos)                /*!< SYS_T::IPRST2: USCI0RST Mask           */

#define SYS_IPRST2_USCI1RST_Pos          (9)                                               /*!< SYS_T::IPRST2: USCI1RST Position       */
#define SYS_IPRST2_USCI1RST_Msk          (0x1ul << SYS_IPRST2_USCI1RST_Pos)                /*!< SYS_T::IPRST2: USCI1RST Mask           */

#define SYS_IPRST2_USCI2RST_Pos          (10)                                              /*!< SYS_T::IPRST2: USCI2RST Position       */
#define SYS_IPRST2_USCI2RST_Msk          (0x1ul << SYS_IPRST2_USCI2RST_Pos)                /*!< SYS_T::IPRST2: USCI2RST Mask           */

#define SYS_IPRST2_USCI3RST_Pos          (11)                                              /*!< SYS_T::IPRST2: USCI3RST Position       */
#define SYS_IPRST2_USCI3RST_Msk          (0x1ul << SYS_IPRST2_USCI3RST_Pos)                /*!< SYS_T::IPRST2: USCI3RST Mask           */

#define SYS_IPRST2_PWM0RST_Pos           (16)                                              /*!< SYS_T::IPRST2: PWM0RST Position        */
#define SYS_IPRST2_PWM0RST_Msk           (0x1ul << SYS_IPRST2_PWM0RST_Pos)                 /*!< SYS_T::IPRST2: PWM0RST Mask            */

#define SYS_IPRST2_PWM1RST_Pos           (17)                                              /*!< SYS_T::IPRST2: PWM1RST Position        */
#define SYS_IPRST2_PWM1RST_Msk           (0x1ul << SYS_IPRST2_PWM1RST_Pos)                 /*!< SYS_T::IPRST2: PWM1RST Mask            */

#define SYS_IPRST2_ECAP0RST_Pos          (26)                                              /*!< SYS_T::IPRST2: ECAP0RST Position       */
#define SYS_IPRST2_ECAP0RST_Msk          (0x1ul << SYS_IPRST2_ECAP0RST_Pos)                /*!< SYS_T::IPRST2: ECAP0RST Mask           */

#define SYS_BODCTL_BODEN_Pos             (0)                                               /*!< SYS_T::BODCTL: BODEN Position          */
#define SYS_BODCTL_BODEN_Msk             (0x1ul << SYS_BODCTL_BODEN_Pos)                   /*!< SYS_T::BODCTL: BODEN Mask              */

#define SYS_BODCTL_BODRSTEN_Pos          (3)                                               /*!< SYS_T::BODCTL: BODRSTEN Position       */
#define SYS_BODCTL_BODRSTEN_Msk          (0x1ul << SYS_BODCTL_BODRSTEN_Pos)                /*!< SYS_T::BODCTL: BODRSTEN Mask           */

#define SYS_BODCTL_BODIF_Pos             (4)                                               /*!< SYS_T::BODCTL: BODIF Position          */
#define SYS_BODCTL_BODIF_Msk             (0x1ul << SYS_BODCTL_BODIF_Pos)                   /*!< SYS_T::BODCTL: BODIF Mask              */

#define SYS_BODCTL_BODLPM_Pos            (5)                                               /*!< SYS_T::BODCTL: BODLPM Position         */
#define SYS_BODCTL_BODLPM_Msk            (0x1ul << SYS_BODCTL_BODLPM_Pos)                  /*!< SYS_T::BODCTL: BODLPM Mask             */

#define SYS_BODCTL_BODOUT_Pos            (6)                                               /*!< SYS_T::BODCTL: BODOUT Position         */
#define SYS_BODCTL_BODOUT_Msk            (0x1ul << SYS_BODCTL_BODOUT_Pos)                  /*!< SYS_T::BODCTL: BODOUT Mask             */

#define SYS_BODCTL_LVREN_Pos             (7)                                               /*!< SYS_T::BODCTL: LVREN Position          */
#define SYS_BODCTL_LVREN_Msk             (0x1ul << SYS_BODCTL_LVREN_Pos)                   /*!< SYS_T::BODCTL: LVREN Mask              */

#define SYS_BODCTL_BODDGSEL_Pos          (8)                                               /*!< SYS_T::BODCTL: BODDGSEL Position       */
#define SYS_BODCTL_BODDGSEL_Msk          (0x7ul << SYS_BODCTL_BODDGSEL_Pos)                /*!< SYS_T::BODCTL: BODDGSEL Mask           */

#define SYS_BODCTL_LVRDGSEL_Pos          (12)                                              /*!< SYS_T::BODCTL: LVRDGSEL Position       */
#define SYS_BODCTL_LVRDGSEL_Msk          (0x7ul << SYS_BODCTL_LVRDGSEL_Pos)                /*!< SYS_T::BODCTL: LVRDGSEL Mask           */

#define SYS_BODCTL_BODVL_Pos             (16)                                              /*!< SYS_T::BODCTL: BODVL Position          */
#define SYS_BODCTL_BODVL_Msk             (0x7ul << SYS_BODCTL_BODVL_Pos)                   /*!< SYS_T::BODCTL: BODVL Mask              */

#define SYS_BODCTL_WRBUSY_Pos            (31)                                              /*!< SYS_T::BODCTL: WRBUSY Position         */
#define SYS_BODCTL_WRBUSY_Msk            (0x1ul << SYS_BODCTL_WRBUSY_Pos)                  /*!< SYS_T::BODCTL: WRBUSY Mask             */

#define SYS_IVSCTL_VTEMPEN_Pos           (0)                                               /*!< SYS_T::IVSCTL: VTEMPEN Position        */
#define SYS_IVSCTL_VTEMPEN_Msk           (0x1ul << SYS_IVSCTL_VTEMPEN_Pos)                 /*!< SYS_T::IVSCTL: VTEMPEN Mask            */

#define SYS_PORCTL0_PORMASK_Pos          (0)                                               /*!< SYS_T::PORCTL0: PORMASK Position       */
#define SYS_PORCTL0_PORMASK_Msk          (0xfffful << SYS_PORCTL0_PORMASK_Pos)             /*!< SYS_T::PORCTL0: PORMASK Mask           */

#define SYS_GPA_MFPL_PA0MFP_Pos          (0)                                               /*!< SYS_T::GPA_MFPL: PA0MFP Position       */
#define SYS_GPA_MFPL_PA0MFP_Msk          (0xful << SYS_GPA_MFPL_PA0MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA0MFP Mask           */

#define SYS_GPA_MFPL_PA1MFP_Pos          (4)                                               /*!< SYS_T::GPA_MFPL: PA1MFP Position       */
#define SYS_GPA_MFPL_PA1MFP_Msk          (0xful << SYS_GPA_MFPL_PA1MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA1MFP Mask           */

#define SYS_GPA_MFPL_PA2MFP_Pos          (8)                                               /*!< SYS_T::GPA_MFPL: PA2MFP Position       */
#define SYS_GPA_MFPL_PA2MFP_Msk          (0xful << SYS_GPA_MFPL_PA2MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA2MFP Mask           */

#define SYS_GPA_MFPL_PA3MFP_Pos          (12)                                              /*!< SYS_T::GPA_MFPL: PA3MFP Position       */
#define SYS_GPA_MFPL_PA3MFP_Msk          (0xful << SYS_GPA_MFPL_PA3MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA3MFP Mask           */

#define SYS_GPA_MFPL_PA4MFP_Pos          (16)                                              /*!< SYS_T::GPA_MFPL: PA4MFP Position       */
#define SYS_GPA_MFPL_PA4MFP_Msk          (0xful << SYS_GPA_MFPL_PA4MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA4MFP Mask           */

#define SYS_GPA_MFPL_PA5MFP_Pos          (20)                                              /*!< SYS_T::GPA_MFPL: PA5MFP Position       */
#define SYS_GPA_MFPL_PA5MFP_Msk          (0xful << SYS_GPA_MFPL_PA5MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA5MFP Mask           */

#define SYS_GPA_MFPL_PA6MFP_Pos          (24)                                              /*!< SYS_T::GPA_MFPL: PA6MFP Position       */
#define SYS_GPA_MFPL_PA6MFP_Msk          (0xful << SYS_GPA_MFPL_PA6MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA6MFP Mask           */

#define SYS_GPA_MFPL_PA7MFP_Pos          (28)                                              /*!< SYS_T::GPA_MFPL: PA7MFP Position       */
#define SYS_GPA_MFPL_PA7MFP_Msk          (0xful << SYS_GPA_MFPL_PA7MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA7MFP Mask           */

#define SYS_GPA_MFPH_PA8MFP_Pos          (0)                                               /*!< SYS_T::GPA_MFPH: PA8MFP Position       */
#define SYS_GPA_MFPH_PA8MFP_Msk          (0xful << SYS_GPA_MFPH_PA8MFP_Pos)                /*!< SYS_T::GPA_MFPH: PA8MFP Mask           */

#define SYS_GPA_MFPH_PA9MFP_Pos          (4)                                               /*!< SYS_T::GPA_MFPH: PA9MFP Position       */
#define SYS_GPA_MFPH_PA9MFP_Msk          (0xful << SYS_GPA_MFPH_PA9MFP_Pos)                /*!< SYS_T::GPA_MFPH: PA9MFP Mask           */

#define SYS_GPA_MFPH_PA10MFP_Pos         (8)                                               /*!< SYS_T::GPA_MFPH: PA10MFP Position      */
#define SYS_GPA_MFPH_PA10MFP_Msk         (0xful << SYS_GPA_MFPH_PA10MFP_Pos)               /*!< SYS_T::GPA_MFPH: PA10MFP Mask          */

#define SYS_GPA_MFPH_PA11MFP_Pos         (12)                                              /*!< SYS_T::GPA_MFPH: PA11MFP Position      */
#define SYS_GPA_MFPH_PA11MFP_Msk         (0xful << SYS_GPA_MFPH_PA11MFP_Pos)               /*!< SYS_T::GPA_MFPH: PA11MFP Mask          */

#define SYS_GPA_MFPH_PA12MFP_Pos         (16)                                              /*!< SYS_T::GPA_MFPH: PA12MFP Position      */
#define SYS_GPA_MFPH_PA12MFP_Msk         (0xful << SYS_GPA_MFPH_PA12MFP_Pos)               /*!< SYS_T::GPA_MFPH: PA12MFP Mask          */

#define SYS_GPA_MFPH_PA13MFP_Pos         (20)                                              /*!< SYS_T::GPA_MFPH: PA13MFP Position      */
#define SYS_GPA_MFPH_PA13MFP_Msk         (0xful << SYS_GPA_MFPH_PA13MFP_Pos)               /*!< SYS_T::GPA_MFPH: PA13MFP Mask          */

#define SYS_GPA_MFPH_PA14MFP_Pos         (24)                                              /*!< SYS_T::GPA_MFPH: PA14MFP Position      */
#define SYS_GPA_MFPH_PA14MFP_Msk         (0xful << SYS_GPA_MFPH_PA14MFP_Pos)               /*!< SYS_T::GPA_MFPH: PA14MFP Mask          */

#define SYS_GPA_MFPH_PA15MFP_Pos         (28)                                              /*!< SYS_T::GPA_MFPH: PA15MFP Position      */
#define SYS_GPA_MFPH_PA15MFP_Msk         (0xful << SYS_GPA_MFPH_PA15MFP_Pos)               /*!< SYS_T::GPA_MFPH: PA15MFP Mask          */

#define SYS_GPB_MFPL_PB0MFP_Pos          (0)                                               /*!< SYS_T::GPB_MFPL: PB0MFP Position       */
#define SYS_GPB_MFPL_PB0MFP_Msk          (0xful << SYS_GPB_MFPL_PB0MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB0MFP Mask           */

#define SYS_GPB_MFPL_PB1MFP_Pos          (4)                                               /*!< SYS_T::GPB_MFPL: PB1MFP Position       */
#define SYS_GPB_MFPL_PB1MFP_Msk          (0xful << SYS_GPB_MFPL_PB1MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB1MFP Mask           */

#define SYS_GPB_MFPL_PB2MFP_Pos          (8)                                               /*!< SYS_T::GPB_MFPL: PB2MFP Position       */
#define SYS_GPB_MFPL_PB2MFP_Msk          (0xful << SYS_GPB_MFPL_PB2MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB2MFP Mask           */

#define SYS_GPB_MFPL_PB3MFP_Pos          (12)                                              /*!< SYS_T::GPB_MFPL: PB3MFP Position       */
#define SYS_GPB_MFPL_PB3MFP_Msk          (0xful << SYS_GPB_MFPL_PB3MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB3MFP Mask           */

#define SYS_GPB_MFPL_PB4MFP_Pos          (16)                                              /*!< SYS_T::GPB_MFPL: PB4MFP Position       */
#define SYS_GPB_MFPL_PB4MFP_Msk          (0xful << SYS_GPB_MFPL_PB4MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB4MFP Mask           */

#define SYS_GPB_MFPL_PB5MFP_Pos          (20)                                              /*!< SYS_T::GPB_MFPL: PB5MFP Position       */
#define SYS_GPB_MFPL_PB5MFP_Msk          (0xful << SYS_GPB_MFPL_PB5MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB5MFP Mask           */

#define SYS_GPB_MFPL_PB6MFP_Pos          (24)                                              /*!< SYS_T::GPB_MFPL: PB6MFP Position       */
#define SYS_GPB_MFPL_PB6MFP_Msk          (0xful << SYS_GPB_MFPL_PB6MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB6MFP Mask           */

#define SYS_GPB_MFPL_PB7MFP_Pos          (28)                                              /*!< SYS_T::GPB_MFPL: PB7MFP Position       */
#define SYS_GPB_MFPL_PB7MFP_Msk          (0xful << SYS_GPB_MFPL_PB7MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB7MFP Mask           */

#define SYS_GPB_MFPH_PB8MFP_Pos          (0)                                               /*!< SYS_T::GPB_MFPH: PB8MFP Position       */
#define SYS_GPB_MFPH_PB8MFP_Msk          (0xful << SYS_GPB_MFPH_PB8MFP_Pos)                /*!< SYS_T::GPB_MFPH: PB8MFP Mask           */

#define SYS_GPB_MFPH_PB9MFP_Pos          (4)                                               /*!< SYS_T::GPB_MFPH: PB9MFP Position       */
#define SYS_GPB_MFPH_PB9MFP_Msk          (0xful << SYS_GPB_MFPH_PB9MFP_Pos)                /*!< SYS_T::GPB_MFPH: PB9MFP Mask           */

#define SYS_GPB_MFPH_PB10MFP_Pos         (8)                                               /*!< SYS_T::GPB_MFPH: PB10MFP Position      */
#define SYS_GPB_MFPH_PB10MFP_Msk         (0xful << SYS_GPB_MFPH_PB10MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB10MFP Mask          */

#define SYS_GPB_MFPH_PB11MFP_Pos         (12)                                              /*!< SYS_T::GPB_MFPH: PB11MFP Position      */
#define SYS_GPB_MFPH_PB11MFP_Msk         (0xful << SYS_GPB_MFPH_PB11MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB11MFP Mask          */

#define SYS_GPB_MFPH_PB12MFP_Pos         (16)                                              /*!< SYS_T::GPB_MFPH: PB12MFP Position      */
#define SYS_GPB_MFPH_PB12MFP_Msk         (0xful << SYS_GPB_MFPH_PB12MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB12MFP Mask          */

#define SYS_GPB_MFPH_PB13MFP_Pos         (20)                                              /*!< SYS_T::GPB_MFPH: PB13MFP Position      */
#define SYS_GPB_MFPH_PB13MFP_Msk         (0xful << SYS_GPB_MFPH_PB13MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB13MFP Mask          */

#define SYS_GPB_MFPH_PB14MFP_Pos         (24)                                              /*!< SYS_T::GPB_MFPH: PB14MFP Position      */
#define SYS_GPB_MFPH_PB14MFP_Msk         (0xful << SYS_GPB_MFPH_PB14MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB14MFP Mask          */

#define SYS_GPB_MFPH_PB15MFP_Pos         (28)                                              /*!< SYS_T::GPB_MFPH: PB15MFP Position      */
#define SYS_GPB_MFPH_PB15MFP_Msk         (0xful << SYS_GPB_MFPH_PB15MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB15MFP Mask          */

#define SYS_GPC_MFPL_PC0MFP_Pos          (0)                                               /*!< SYS_T::GPC_MFPL: PC0MFP Position       */
#define SYS_GPC_MFPL_PC0MFP_Msk          (0xful << SYS_GPC_MFPL_PC0MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC0MFP Mask           */

#define SYS_GPC_MFPL_PC1MFP_Pos          (4)                                               /*!< SYS_T::GPC_MFPL: PC1MFP Position       */
#define SYS_GPC_MFPL_PC1MFP_Msk          (0xful << SYS_GPC_MFPL_PC1MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC1MFP Mask           */

#define SYS_GPC_MFPL_PC2MFP_Pos          (8)                                               /*!< SYS_T::GPC_MFPL: PC2MFP Position       */
#define SYS_GPC_MFPL_PC2MFP_Msk          (0xful << SYS_GPC_MFPL_PC2MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC2MFP Mask           */

#define SYS_GPC_MFPL_PC3MFP_Pos          (12)                                              /*!< SYS_T::GPC_MFPL: PC3MFP Position       */
#define SYS_GPC_MFPL_PC3MFP_Msk          (0xful << SYS_GPC_MFPL_PC3MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC3MFP Mask           */

#define SYS_GPC_MFPL_PC4MFP_Pos          (16)                                              /*!< SYS_T::GPC_MFPL: PC4MFP Position       */
#define SYS_GPC_MFPL_PC4MFP_Msk          (0xful << SYS_GPC_MFPL_PC4MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC4MFP Mask           */

#define SYS_GPC_MFPL_PC5MFP_Pos          (20)                                              /*!< SYS_T::GPC_MFPL: PC5MFP Position       */
#define SYS_GPC_MFPL_PC5MFP_Msk          (0xful << SYS_GPC_MFPL_PC5MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC5MFP Mask           */

#define SYS_GPC_MFPL_PC6MFP_Pos          (24)                                              /*!< SYS_T::GPC_MFPL: PC6MFP Position       */
#define SYS_GPC_MFPL_PC6MFP_Msk          (0xful << SYS_GPC_MFPL_PC6MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC6MFP Mask           */

#define SYS_GPC_MFPL_PC7MFP_Pos          (28)                                              /*!< SYS_T::GPC_MFPL: PC7MFP Position       */
#define SYS_GPC_MFPL_PC7MFP_Msk          (0xful << SYS_GPC_MFPL_PC7MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC7MFP Mask           */

#define SYS_GPC_MFPH_PC14MFP_Pos         (24)                                              /*!< SYS_T::GPC_MFPH: PC14MFP Position      */
#define SYS_GPC_MFPH_PC14MFP_Msk         (0xful << SYS_GPC_MFPH_PC14MFP_Pos)               /*!< SYS_T::GPC_MFPH: PC14MFP Mask          */

#define SYS_GPD_MFPL_PD0MFP_Pos          (0)                                               /*!< SYS_T::GPD_MFPL: PD0MFP Position       */
#define SYS_GPD_MFPL_PD0MFP_Msk          (0xful << SYS_GPD_MFPL_PD0MFP_Pos)                /*!< SYS_T::GPD_MFPL: PD0MFP Mask           */

#define SYS_GPD_MFPL_PD1MFP_Pos          (4)                                               /*!< SYS_T::GPD_MFPL: PD1MFP Position       */
#define SYS_GPD_MFPL_PD1MFP_Msk          (0xful << SYS_GPD_MFPL_PD1MFP_Pos)                /*!< SYS_T::GPD_MFPL: PD1MFP Mask           */

#define SYS_GPD_MFPL_PD2MFP_Pos          (8)                                               /*!< SYS_T::GPD_MFPL: PD2MFP Position       */
#define SYS_GPD_MFPL_PD2MFP_Msk          (0xful << SYS_GPD_MFPL_PD2MFP_Pos)                /*!< SYS_T::GPD_MFPL: PD2MFP Mask           */

#define SYS_GPD_MFPL_PD3MFP_Pos          (12)                                              /*!< SYS_T::GPD_MFPL: PD3MFP Position       */
#define SYS_GPD_MFPL_PD3MFP_Msk          (0xful << SYS_GPD_MFPL_PD3MFP_Pos)                /*!< SYS_T::GPD_MFPL: PD3MFP Mask           */

#define SYS_GPD_MFPH_PD15MFP_Pos         (28)                                              /*!< SYS_T::GPD_MFPH: PD15MFP Position      */
#define SYS_GPD_MFPH_PD15MFP_Msk         (0xful << SYS_GPD_MFPH_PD15MFP_Pos)               /*!< SYS_T::GPD_MFPH: PD15MFP Mask          */

#define SYS_GPE_MFPH_PE15MFP_Pos         (28)                                              /*!< SYS_T::GPE_MFPH: PE15MFP Position      */
#define SYS_GPE_MFPH_PE15MFP_Msk         (0xful << SYS_GPE_MFPH_PE15MFP_Pos)               /*!< SYS_T::GPE_MFPH: PE15MFP Mask          */

#define SYS_GPF_MFPL_PF0MFP_Pos          (0)                                               /*!< SYS_T::GPF_MFPL: PF0MFP Position       */
#define SYS_GPF_MFPL_PF0MFP_Msk          (0xful << SYS_GPF_MFPL_PF0MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF0MFP Mask           */

#define SYS_GPF_MFPL_PF1MFP_Pos          (4)                                               /*!< SYS_T::GPF_MFPL: PF1MFP Position       */
#define SYS_GPF_MFPL_PF1MFP_Msk          (0xful << SYS_GPF_MFPL_PF1MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF1MFP Mask           */

#define SYS_GPF_MFPL_PF2MFP_Pos          (8)                                               /*!< SYS_T::GPF_MFPL: PF2MFP Position       */
#define SYS_GPF_MFPL_PF2MFP_Msk          (0xful << SYS_GPF_MFPL_PF2MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF2MFP Mask           */

#define SYS_GPF_MFPL_PF3MFP_Pos          (12)                                              /*!< SYS_T::GPF_MFPL: PF3MFP Position       */
#define SYS_GPF_MFPL_PF3MFP_Msk          (0xful << SYS_GPF_MFPL_PF3MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF3MFP Mask           */

#define SYS_GPF_MFPL_PF4MFP_Pos          (16)                                              /*!< SYS_T::GPF_MFPL: PF4MFP Position       */
#define SYS_GPF_MFPL_PF4MFP_Msk          (0xful << SYS_GPF_MFPL_PF4MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF4MFP Mask           */

#define SYS_GPF_MFPL_PF5MFP_Pos          (20)                                              /*!< SYS_T::GPF_MFPL: PF5MFP Position       */
#define SYS_GPF_MFPL_PF5MFP_Msk          (0xful << SYS_GPF_MFPL_PF5MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF5MFP Mask           */

#define SYS_GPF_MFPL_PF6MFP_Pos          (24)                                              /*!< SYS_T::GPF_MFPL: PF6MFP Position       */
#define SYS_GPF_MFPL_PF6MFP_Msk          (0xful << SYS_GPF_MFPL_PF6MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF6MFP Mask           */

#define SYS_GPF_MFPH_PF14MFP_Pos         (24)                                              /*!< SYS_T::GPF_MFPH: PF14MFP Position      */
#define SYS_GPF_MFPH_PF14MFP_Msk         (0xful << SYS_GPF_MFPH_PF14MFP_Pos)               /*!< SYS_T::GPF_MFPH: PF14MFP Mask          */

#define SYS_GPF_MFPH_PF15MFP_Pos         (28)                                              /*!< SYS_T::GPF_MFPH: PF15MFP Position      */
#define SYS_GPF_MFPH_PF15MFP_Msk         (0xful << SYS_GPF_MFPH_PF15MFP_Pos)               /*!< SYS_T::GPF_MFPH: PF15MFP Mask          */

#define SYS_GPG_MFPH_PG9MFP_Pos          (4)                                               /*!< SYS_T::GPG_MFPH: PG9MFP Position       */
#define SYS_GPG_MFPH_PG9MFP_Msk          (0xful << SYS_GPG_MFPH_PG9MFP_Pos)                /*!< SYS_T::GPG_MFPH: PG9MFP Mask           */

#define SYS_GPG_MFPH_PG10MFP_Pos         (8)                                               /*!< SYS_T::GPG_MFPH: PG10MFP Position      */
#define SYS_GPG_MFPH_PG10MFP_Msk         (0xful << SYS_GPG_MFPH_PG10MFP_Pos)               /*!< SYS_T::GPG_MFPH: PG10MFP Mask          */

#define SYS_GPG_MFPH_PG11MFP_Pos         (12)                                              /*!< SYS_T::GPG_MFPH: PG11MFP Position      */
#define SYS_GPG_MFPH_PG11MFP_Msk         (0xful << SYS_GPG_MFPH_PG11MFP_Pos)               /*!< SYS_T::GPG_MFPH: PG11MFP Mask          */

#define SYS_GPG_MFPH_PG12MFP_Pos         (16)                                              /*!< SYS_T::GPG_MFPH: PG12MFP Position      */
#define SYS_GPG_MFPH_PG12MFP_Msk         (0xful << SYS_GPG_MFPH_PG12MFP_Pos)               /*!< SYS_T::GPG_MFPH: PG12MFP Mask          */

#define SYS_GPA_MFOS_PA0MFOS_Pos         (0)                                               /*!< SYS_T::GPA_MFOS: PA0MFOS Position      */
#define SYS_GPA_MFOS_PA0MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA0MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA0MFOS Mask          */

#define SYS_GPA_MFOS_PA1MFOS_Pos         (1)                                               /*!< SYS_T::GPA_MFOS: PA1MFOS Position      */
#define SYS_GPA_MFOS_PA1MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA1MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA1MFOS Mask          */

#define SYS_GPA_MFOS_PA2MFOS_Pos         (2)                                               /*!< SYS_T::GPA_MFOS: PA2MFOS Position      */
#define SYS_GPA_MFOS_PA2MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA2MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA2MFOS Mask          */

#define SYS_GPA_MFOS_PA3MFOS_Pos         (3)                                               /*!< SYS_T::GPA_MFOS: PA3MFOS Position      */
#define SYS_GPA_MFOS_PA3MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA3MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA3MFOS Mask          */

#define SYS_GPA_MFOS_PA4MFOS_Pos         (4)                                               /*!< SYS_T::GPA_MFOS: PA4MFOS Position      */
#define SYS_GPA_MFOS_PA4MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA4MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA4MFOS Mask          */

#define SYS_GPA_MFOS_PA5MFOS_Pos         (5)                                               /*!< SYS_T::GPA_MFOS: PA5MFOS Position      */
#define SYS_GPA_MFOS_PA5MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA5MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA5MFOS Mask          */

#define SYS_GPA_MFOS_PA6MFOS_Pos         (6)                                               /*!< SYS_T::GPA_MFOS: PA6MFOS Position      */
#define SYS_GPA_MFOS_PA6MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA6MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA6MFOS Mask          */

#define SYS_GPA_MFOS_PA7MFOS_Pos         (7)                                               /*!< SYS_T::GPA_MFOS: PA7MFOS Position      */
#define SYS_GPA_MFOS_PA7MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA7MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA7MFOS Mask          */

#define SYS_GPA_MFOS_PA8MFOS_Pos         (8)                                               /*!< SYS_T::GPA_MFOS: PA8MFOS Position      */
#define SYS_GPA_MFOS_PA8MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA8MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA8MFOS Mask          */

#define SYS_GPA_MFOS_PA9MFOS_Pos         (9)                                               /*!< SYS_T::GPA_MFOS: PA9MFOS Position      */
#define SYS_GPA_MFOS_PA9MFOS_Msk         (0x1ul << SYS_GPA_MFOS_PA9MFOS_Pos)               /*!< SYS_T::GPA_MFOS: PA9MFOS Mask          */

#define SYS_GPA_MFOS_PA10MFOS_Pos        (10)                                              /*!< SYS_T::GPA_MFOS: PA10MFOS Position     */
#define SYS_GPA_MFOS_PA10MFOS_Msk        (0x1ul << SYS_GPA_MFOS_PA10MFOS_Pos)              /*!< SYS_T::GPA_MFOS: PA10MFOS Mask         */

#define SYS_GPA_MFOS_PA11MFOS_Pos        (11)                                              /*!< SYS_T::GPA_MFOS: PA11MFOS Position     */
#define SYS_GPA_MFOS_PA11MFOS_Msk        (0x1ul << SYS_GPA_MFOS_PA11MFOS_Pos)              /*!< SYS_T::GPA_MFOS: PA11MFOS Mask         */

#define SYS_GPA_MFOS_PA12MFOS_Pos        (12)                                              /*!< SYS_T::GPA_MFOS: PA12MFOS Position     */
#define SYS_GPA_MFOS_PA12MFOS_Msk        (0x1ul << SYS_GPA_MFOS_PA12MFOS_Pos)              /*!< SYS_T::GPA_MFOS: PA12MFOS Mask         */

#define SYS_GPA_MFOS_PA13MFOS_Pos        (13)                                              /*!< SYS_T::GPA_MFOS: PA13MFOS Position     */
#define SYS_GPA_MFOS_PA13MFOS_Msk        (0x1ul << SYS_GPA_MFOS_PA13MFOS_Pos)              /*!< SYS_T::GPA_MFOS: PA13MFOS Mask         */

#define SYS_GPA_MFOS_PA14MFOS_Pos        (14)                                              /*!< SYS_T::GPA_MFOS: PA14MFOS Position     */
#define SYS_GPA_MFOS_PA14MFOS_Msk        (0x1ul << SYS_GPA_MFOS_PA14MFOS_Pos)              /*!< SYS_T::GPA_MFOS: PA14MFOS Mask         */

#define SYS_GPA_MFOS_PA15MFOS_Pos        (15)                                              /*!< SYS_T::GPA_MFOS: PA15MFOS Position     */
#define SYS_GPA_MFOS_PA15MFOS_Msk        (0x1ul << SYS_GPA_MFOS_PA15MFOS_Pos)              /*!< SYS_T::GPA_MFOS: PA15MFOS Mask         */

#define SYS_GPB_MFOS_PB0MFOS_Pos         (0)                                               /*!< SYS_T::GPB_MFOS: PB0MFOS Position      */
#define SYS_GPB_MFOS_PB0MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB0MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB0MFOS Mask          */

#define SYS_GPB_MFOS_PB1MFOS_Pos         (1)                                               /*!< SYS_T::GPB_MFOS: PB1MFOS Position      */
#define SYS_GPB_MFOS_PB1MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB1MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB1MFOS Mask          */

#define SYS_GPB_MFOS_PB2MFOS_Pos         (2)                                               /*!< SYS_T::GPB_MFOS: PB2MFOS Position      */
#define SYS_GPB_MFOS_PB2MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB2MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB2MFOS Mask          */

#define SYS_GPB_MFOS_PB3MFOS_Pos         (3)                                               /*!< SYS_T::GPB_MFOS: PB3MFOS Position      */
#define SYS_GPB_MFOS_PB3MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB3MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB3MFOS Mask          */

#define SYS_GPB_MFOS_PB4MFOS_Pos         (4)                                               /*!< SYS_T::GPB_MFOS: PB4MFOS Position      */
#define SYS_GPB_MFOS_PB4MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB4MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB4MFOS Mask          */

#define SYS_GPB_MFOS_PB5MFOS_Pos         (5)                                               /*!< SYS_T::GPB_MFOS: PB5MFOS Position      */
#define SYS_GPB_MFOS_PB5MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB5MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB5MFOS Mask          */

#define SYS_GPB_MFOS_PB6MFOS_Pos         (6)                                               /*!< SYS_T::GPB_MFOS: PB6MFOS Position      */
#define SYS_GPB_MFOS_PB6MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB6MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB6MFOS Mask          */

#define SYS_GPB_MFOS_PB7MFOS_Pos         (7)                                               /*!< SYS_T::GPB_MFOS: PB7MFOS Position      */
#define SYS_GPB_MFOS_PB7MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB7MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB7MFOS Mask          */

#define SYS_GPB_MFOS_PB8MFOS_Pos         (8)                                               /*!< SYS_T::GPB_MFOS: PB8MFOS Position      */
#define SYS_GPB_MFOS_PB8MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB8MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB8MFOS Mask          */

#define SYS_GPB_MFOS_PB9MFOS_Pos         (9)                                               /*!< SYS_T::GPB_MFOS: PB9MFOS Position      */
#define SYS_GPB_MFOS_PB9MFOS_Msk         (0x1ul << SYS_GPB_MFOS_PB9MFOS_Pos)               /*!< SYS_T::GPB_MFOS: PB9MFOS Mask          */

#define SYS_GPB_MFOS_PB10MFOS_Pos        (10)                                              /*!< SYS_T::GPB_MFOS: PB10MFOS Position     */
#define SYS_GPB_MFOS_PB10MFOS_Msk        (0x1ul << SYS_GPB_MFOS_PB10MFOS_Pos)              /*!< SYS_T::GPB_MFOS: PB10MFOS Mask         */

#define SYS_GPB_MFOS_PB11MFOS_Pos        (11)                                              /*!< SYS_T::GPB_MFOS: PB11MFOS Position     */
#define SYS_GPB_MFOS_PB11MFOS_Msk        (0x1ul << SYS_GPB_MFOS_PB11MFOS_Pos)              /*!< SYS_T::GPB_MFOS: PB11MFOS Mask         */

#define SYS_GPB_MFOS_PB12MFOS_Pos        (12)                                              /*!< SYS_T::GPB_MFOS: PB12MFOS Position     */
#define SYS_GPB_MFOS_PB12MFOS_Msk        (0x1ul << SYS_GPB_MFOS_PB12MFOS_Pos)              /*!< SYS_T::GPB_MFOS: PB12MFOS Mask         */

#define SYS_GPB_MFOS_PB13MFOS_Pos        (13)                                              /*!< SYS_T::GPB_MFOS: PB13MFOS Position     */
#define SYS_GPB_MFOS_PB13MFOS_Msk        (0x1ul << SYS_GPB_MFOS_PB13MFOS_Pos)              /*!< SYS_T::GPB_MFOS: PB13MFOS Mask         */

#define SYS_GPB_MFOS_PB14MFOS_Pos        (14)                                              /*!< SYS_T::GPB_MFOS: PB14MFOS Position     */
#define SYS_GPB_MFOS_PB14MFOS_Msk        (0x1ul << SYS_GPB_MFOS_PB14MFOS_Pos)              /*!< SYS_T::GPB_MFOS: PB14MFOS Mask         */

#define SYS_GPB_MFOS_PB15MFOS_Pos        (15)                                              /*!< SYS_T::GPB_MFOS: PB15MFOS Position     */
#define SYS_GPB_MFOS_PB15MFOS_Msk        (0x1ul << SYS_GPB_MFOS_PB15MFOS_Pos)              /*!< SYS_T::GPB_MFOS: PB15MFOS Mask         */

#define SYS_GPC_MFOS_PC0MFOS_Pos         (0)                                               /*!< SYS_T::GPC_MFOS: PC0MFOS Position      */
#define SYS_GPC_MFOS_PC0MFOS_Msk         (0x1ul << SYS_GPC_MFOS_PC0MFOS_Pos)               /*!< SYS_T::GPC_MFOS: PC0MFOS Mask          */

#define SYS_GPC_MFOS_PC1MFOS_Pos         (1)                                               /*!< SYS_T::GPC_MFOS: PC1MFOS Position      */
#define SYS_GPC_MFOS_PC1MFOS_Msk         (0x1ul << SYS_GPC_MFOS_PC1MFOS_Pos)               /*!< SYS_T::GPC_MFOS: PC1MFOS Mask          */

#define SYS_GPC_MFOS_PC2MFOS_Pos         (2)                                               /*!< SYS_T::GPC_MFOS: PC2MFOS Position      */
#define SYS_GPC_MFOS_PC2MFOS_Msk         (0x1ul << SYS_GPC_MFOS_PC2MFOS_Pos)               /*!< SYS_T::GPC_MFOS: PC2MFOS Mask          */

#define SYS_GPC_MFOS_PC3MFOS_Pos         (3)                                               /*!< SYS_T::GPC_MFOS: PC3MFOS Position      */
#define SYS_GPC_MFOS_PC3MFOS_Msk         (0x1ul << SYS_GPC_MFOS_PC3MFOS_Pos)               /*!< SYS_T::GPC_MFOS: PC3MFOS Mask          */

#define SYS_GPC_MFOS_PC4MFOS_Pos         (4)                                               /*!< SYS_T::GPC_MFOS: PC4MFOS Position      */
#define SYS_GPC_MFOS_PC4MFOS_Msk         (0x1ul << SYS_GPC_MFOS_PC4MFOS_Pos)               /*!< SYS_T::GPC_MFOS: PC4MFOS Mask          */

#define SYS_GPC_MFOS_PC5MFOS_Pos         (5)                                               /*!< SYS_T::GPC_MFOS: PC5MFOS Position      */
#define SYS_GPC_MFOS_PC5MFOS_Msk         (0x1ul << SYS_GPC_MFOS_PC5MFOS_Pos)               /*!< SYS_T::GPC_MFOS: PC5MFOS Mask          */

#define SYS_GPC_MFOS_PC6MFOS_Pos         (6)                                               /*!< SYS_T::GPC_MFOS: PC6MFOS Position      */
#define SYS_GPC_MFOS_PC6MFOS_Msk         (0x1ul << SYS_GPC_MFOS_PC6MFOS_Pos)               /*!< SYS_T::GPC_MFOS: PC6MFOS Mask          */

#define SYS_GPC_MFOS_PC7MFOS_Pos         (7)                                               /*!< SYS_T::GPC_MFOS: PC7MFOS Position      */
#define SYS_GPC_MFOS_PC7MFOS_Msk         (0x1ul << SYS_GPC_MFOS_PC7MFOS_Pos)               /*!< SYS_T::GPC_MFOS: PC7MFOS Mask          */

#define SYS_GPC_MFOS_PC14MFOS_Pos        (14)                                              /*!< SYS_T::GPC_MFOS: PC14MFOS Position     */
#define SYS_GPC_MFOS_PC14MFOS_Msk        (0x1ul << SYS_GPC_MFOS_PC14MFOS_Pos)              /*!< SYS_T::GPC_MFOS: PC14MFOS Mask         */

#define SYS_GPD_MFOS_PD0MFOS_Pos         (0)                                               /*!< SYS_T::GPD_MFOS: PD0MFOS Position      */
#define SYS_GPD_MFOS_PD0MFOS_Msk         (0x1ul << SYS_GPD_MFOS_PD0MFOS_Pos)               /*!< SYS_T::GPD_MFOS: PD0MFOS Mask          */

#define SYS_GPD_MFOS_PD1MFOS_Pos         (1)                                               /*!< SYS_T::GPD_MFOS: PD1MFOS Position      */
#define SYS_GPD_MFOS_PD1MFOS_Msk         (0x1ul << SYS_GPD_MFOS_PD1MFOS_Pos)               /*!< SYS_T::GPD_MFOS: PD1MFOS Mask          */

#define SYS_GPD_MFOS_PD2MFOS_Pos         (2)                                               /*!< SYS_T::GPD_MFOS: PD2MFOS Position      */
#define SYS_GPD_MFOS_PD2MFOS_Msk         (0x1ul << SYS_GPD_MFOS_PD2MFOS_Pos)               /*!< SYS_T::GPD_MFOS: PD2MFOS Mask          */

#define SYS_GPD_MFOS_PD3MFOS_Pos         (3)                                               /*!< SYS_T::GPD_MFOS: PD3MFOS Position      */
#define SYS_GPD_MFOS_PD3MFOS_Msk         (0x1ul << SYS_GPD_MFOS_PD3MFOS_Pos)               /*!< SYS_T::GPD_MFOS: PD3MFOS Mask          */

#define SYS_GPD_MFOS_PD15MFOS_Pos        (15)                                              /*!< SYS_T::GPD_MFOS: PD15MFOS Position     */
#define SYS_GPD_MFOS_PD15MFOS_Msk        (0x1ul << SYS_GPD_MFOS_PD15MFOS_Pos)              /*!< SYS_T::GPD_MFOS: PD15MFOS Mask         */

#define SYS_GPF_MFOS_PF0MFOS_Pos         (0)                                               /*!< SYS_T::GPF_MFOS: PF0MFOS Position      */
#define SYS_GPF_MFOS_PF0MFOS_Msk         (0x1ul << SYS_GPF_MFOS_PF0MFOS_Pos)               /*!< SYS_T::GPF_MFOS: PF0MFOS Mask          */

#define SYS_GPF_MFOS_PF1MFOS_Pos         (1)                                               /*!< SYS_T::GPF_MFOS: PF1MFOS Position      */
#define SYS_GPF_MFOS_PF1MFOS_Msk         (0x1ul << SYS_GPF_MFOS_PF1MFOS_Pos)               /*!< SYS_T::GPF_MFOS: PF1MFOS Mask          */

#define SYS_GPF_MFOS_PF2MFOS_Pos         (2)                                               /*!< SYS_T::GPF_MFOS: PF2MFOS Position      */
#define SYS_GPF_MFOS_PF2MFOS_Msk         (0x1ul << SYS_GPF_MFOS_PF2MFOS_Pos)               /*!< SYS_T::GPF_MFOS: PF2MFOS Mask          */

#define SYS_GPF_MFOS_PF3MFOS_Pos         (3)                                               /*!< SYS_T::GPF_MFOS: PF3MFOS Position      */
#define SYS_GPF_MFOS_PF3MFOS_Msk         (0x1ul << SYS_GPF_MFOS_PF3MFOS_Pos)               /*!< SYS_T::GPF_MFOS: PF3MFOS Mask          */

#define SYS_GPF_MFOS_PF4MFOS_Pos         (4)                                               /*!< SYS_T::GPF_MFOS: PF4MFOS Position      */
#define SYS_GPF_MFOS_PF4MFOS_Msk         (0x1ul << SYS_GPF_MFOS_PF4MFOS_Pos)               /*!< SYS_T::GPF_MFOS: PF4MFOS Mask          */

#define SYS_GPF_MFOS_PF5MFOS_Pos         (5)                                               /*!< SYS_T::GPF_MFOS: PF5MFOS Position      */
#define SYS_GPF_MFOS_PF5MFOS_Msk         (0x1ul << SYS_GPF_MFOS_PF5MFOS_Pos)               /*!< SYS_T::GPF_MFOS: PF5MFOS Mask          */

#define SYS_GPF_MFOS_PF15MFOS_Pos        (15)                                              /*!< SYS_T::GPF_MFOS: PF15MFOS Position     */
#define SYS_GPF_MFOS_PF15MFOS_Msk        (0x1ul << SYS_GPF_MFOS_PF15MFOS_Pos)              /*!< SYS_T::GPF_MFOS: PF15MFOS Mask         */

#define SYS_GPG_MFOS_PG9MFOS_Pos         (9)                                               /*!< SYS_T::GPG_MFOS: PG9MFOS Position      */
#define SYS_GPG_MFOS_PG9MFOS_Msk         (0x1ul << SYS_GPG_MFOS_PG9MFOS_Pos)               /*!< SYS_T::GPG_MFOS: PG9MFOS Mask          */

#define SYS_GPG_MFOS_PG10MFOS_Pos        (10)                                              /*!< SYS_T::GPG_MFOS: PG10MFOS Position     */
#define SYS_GPG_MFOS_PG10MFOS_Msk        (0x1ul << SYS_GPG_MFOS_PG10MFOS_Pos)              /*!< SYS_T::GPG_MFOS: PG10MFOS Mask         */

#define SYS_GPG_MFOS_PG11MFOS_Pos        (11)                                              /*!< SYS_T::GPG_MFOS: PG11MFOS Position     */
#define SYS_GPG_MFOS_PG11MFOS_Msk        (0x1ul << SYS_GPG_MFOS_PG11MFOS_Pos)              /*!< SYS_T::GPG_MFOS: PG11MFOS Mask         */

#define SYS_GPG_MFOS_PG12MFOS_Pos        (12)                                              /*!< SYS_T::GPG_MFOS: PG12MFOS Position     */
#define SYS_GPG_MFOS_PG12MFOS_Msk        (0x1ul << SYS_GPG_MFOS_PG12MFOS_Pos)              /*!< SYS_T::GPG_MFOS: PG12MFOS Mask         */

#define SYS_REGLCTL_REGLCTL_Pos          (0)                                               /*!< SYS_T::REGLCTL: REGLCTL Position       */
#define SYS_REGLCTL_REGLCTL_Msk          (0xfful << SYS_REGLCTL_REGLCTL_Pos)               /*!< SYS_T::REGLCTL: REGLCTL Mask           */

#define SYS_ADOFS_ADOFS_Pos              (0)                                               /*!< SYS_T::ADOFS: ADOFS Position           */
#define SYS_ADOFS_ADOFS_Msk              (0x7ul << SYS_ADOFS_ADOFS_Pos)                    /*!< SYS_T::ADOFS: ADOFS Mask               */

#define SYS_CPUCFG_INTRTEN_Pos           (0)                                               /*!< SYS_T::CPUCFG: INTRTEN Position        */
#define SYS_CPUCFG_INTRTEN_Msk           (0x1ul << SYS_CPUCFG_INTRTEN_Pos)                 /*!< SYS_T::CPUCFG: INTRTEN Mask            */

#define SYS_PORCTL1_POROFF_Pos           (0)                                               /*!< SYS_T::PORCTL1: POROFF Position        */
#define SYS_PORCTL1_POROFF_Msk           (0xfffful << SYS_PORCTL1_POROFF_Pos)              /*!< SYS_T::PORCTL1: POROFF Mask            */

/**@}*/ /* SYS_CONST */

typedef struct
{
    /**
     * @var NMI_T::NMIEN
     * Offset: 0x00  NMI Source Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |BOD NMI Source Enable (Write Protect)
     * |        |          |0 = BOD NMI source Disabled.
     * |        |          |1 = BOD NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |PWRWUINT  |Power-down Mode Wake-up NMI Source Enable (Write Protect)
     * |        |          |0 = Power-down mode wake-up NMI source Disabled.
     * |        |          |1 = Power-down mode wake-up NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |CLKFAIL   |Clock Fail Detected NMI Source Enable (Write Protect)
     * |        |          |0 = Clock fail detected interrupt NMI source Disabled.
     * |        |          |1 = Clock fail detected interrupt NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[8]     |EINT0     |External Interrupt From PA.6, PB.5 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PA.6, PB.5 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PA.6, PB.5 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[9]     |EINT1     |External Interrupt From PA.7, PB.4 or PD.15 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PA.7, PB.4 or PD.15 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PA.7, PB.4 or PD.15 NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[10]    |EINT2     |External Interrupt From PB.3 or PC.6 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.3 or PC.6 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.3 or PC.6 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[11]    |EINT3     |External Interrupt From PB.2 or PC.7 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.2 or PC.7pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.2 or PC.7 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[12]    |EINT4     |External Interrupt From PA.8, PB.6 or PF.15 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PA.8, PB.6 or PF.15 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PA.8, PB.6 or PF.15 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[13]    |EINT5     |External Interrupt From PB.7 or PF.14 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.7 or PF.14 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.7 or PF.14 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[14]    |UART0INT  |UART0 NMI Source Enable (Write Protect)
     * |        |          |0 = UART0 NMI source Disabled.
     * |        |          |1 = UART0 NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[15]    |UART1INT  |UART1 NMI Source Enable (Write Protect)
     * |        |          |0 = UART1 NMI source Disabled.
     * |        |          |1 = UART1 NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var NMI_T::NMISTS
     * Offset: 0x04  NMI Source Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |BOD Interrupt Flag (Read Only)
     * |        |          |0 = BOD interrupt is deasserted.
     * |        |          |1 = BOD interrupt is asserted.
     * |[2]     |PWRWUINT  |Power-down Mode Wake-up Interrupt Flag (Read Only)
     * |        |          |0 = Power-down mode wake-up interrupt is de-asserted.
     * |        |          |1 = Power-down mode wake-up interrupt is asserted.
     * |[4]     |CLKFAIL   |Clock Fail Detected Interrupt Flag (Read Only)
     * |        |          |0 = Clock fail detected interrupt is deasserted.
     * |        |          |1 = Clock fail detected interrupt is asserted.
     * |[8]     |EINT0     |External Interrupt From PA.6, PB.5 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PA.6, PB.5 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PA.6 PB.5 interrupt is asserted.
     * |[9]     |EINT1     |External Interrupt From PA.7, PB.4 or PD.15 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PA.7, PB.4 or PD.15 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PA.7, PB.4 or PD.15 interrupt is asserted.
     * |[10]    |EINT2     |External Interrupt From PB.3 or PC.6 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PB.3 or PC.6 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PB.3 or PC.6 interrupt is asserted.
     * |[11]    |EINT3     |External Interrupt From PB.2 or PC.7 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PB.2 or PC.7 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PB.2 or PC.7 interrupt is asserted.
     * |[12]    |EINT4     |External Interrupt From PA.8, PB.6 or PF.15 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PA.8, PB.6 or PF.15 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PA.8, PB.6 or PF.15 interrupt is asserted.
     * |[13]    |EINT5     |External Interrupt From PB.7 or PF.14 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PB.7 or PF.14 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PB.7 or PF.14 interrupt is asserted.
     * |[14]    |UART0INT  |UART0 Interrupt Flag (Read Only)
     * |        |          |0 = UART1 interrupt is deasserted.
     * |        |          |1 = UART1 interrupt is asserted.
     * |[15]    |UART1INT  |UART1 Interrupt Flag (Read Only)
     * |        |          |0 = UART1 interrupt is deasserted.
     * |        |          |1 = UART1 interrupt is asserted.
     */
    __IO uint32_t NMIEN;                /*!< [0x0000] NMI Source Interrupt Enable Register                              */
    __I  uint32_t NMISTS;               /*!< [0x0004] NMI Source Interrupt Status Register                              */
} NMI_T;

/**
    @addtogroup NMI_CONST NMI Bit Field Definition
    Constant Definitions for NMI Controller
  @{
*/

#define NMI_NMIEN_BODOUT_Pos            (0)                                             /*!< NMI_T::NMIEN: BODOUT Position          */
#define NMI_NMIEN_BODOUT_Msk            (0x1ul << NMI_NMIEN_BODOUT_Pos)                 /*!< NMI_T::NMIEN: BODOUT Mask              */

#define NMI_NMIEN_PWRWUINT_Pos          (2)                                             /*!< NMI_T::NMIEN: PWRWUINT Position        */
#define NMI_NMIEN_PWRWUINT_Msk          (0x1ul << NMI_NMIEN_PWRWUINT_Pos)               /*!< NMI_T::NMIEN: PWRWUINT Mask            */

#define NMI_NMIEN_CLKFAIL_Pos           (4)                                             /*!< NMI_T::NMIEN: CLKFAIL Position         */
#define NMI_NMIEN_CLKFAIL_Msk           (0x1ul << NMI_NMIEN_CLKFAIL_Pos)                /*!< NMI_T::NMIEN: CLKFAIL Mask             */

#define NMI_NMIEN_EINT0_Pos             (8)                                             /*!< NMI_T::NMIEN: EINT0 Position           */
#define NMI_NMIEN_EINT0_Msk             (0x1ul << NMI_NMIEN_EINT0_Pos)                  /*!< NMI_T::NMIEN: EINT0 Mask               */

#define NMI_NMIEN_EINT1_Pos             (9)                                             /*!< NMI_T::NMIEN: EINT1 Position           */
#define NMI_NMIEN_EINT1_Msk             (0x1ul << NMI_NMIEN_EINT1_Pos)                  /*!< NMI_T::NMIEN: EINT1 Mask               */

#define NMI_NMIEN_EINT2_Pos             (10)                                            /*!< NMI_T::NMIEN: EINT2 Position           */
#define NMI_NMIEN_EINT2_Msk             (0x1ul << NMI_NMIEN_EINT2_Pos)                  /*!< NMI_T::NMIEN: EINT2 Mask               */

#define NMI_NMIEN_EINT3_Pos             (11)                                            /*!< NMI_T::NMIEN: EINT3 Position           */
#define NMI_NMIEN_EINT3_Msk             (0x1ul << NMI_NMIEN_EINT3_Pos)                  /*!< NMI_T::NMIEN: EINT3 Mask               */

#define NMI_NMIEN_EINT4_Pos             (12)                                            /*!< NMI_T::NMIEN: EINT4 Position           */
#define NMI_NMIEN_EINT4_Msk             (0x1ul << NMI_NMIEN_EINT4_Pos)                  /*!< NMI_T::NMIEN: EINT4 Mask               */

#define NMI_NMIEN_EINT5_Pos             (13)                                            /*!< NMI_T::NMIEN: EINT5 Position           */
#define NMI_NMIEN_EINT5_Msk             (0x1ul << NMI_NMIEN_EINT5_Pos)                  /*!< NMI_T::NMIEN: EINT5 Mask               */

#define NMI_NMIEN_UART0INT_Pos          (14)                                            /*!< NMI_T::NMIEN: UART0INT Position        */
#define NMI_NMIEN_UART0INT_Msk          (0x1ul << NMI_NMIEN_UART0INT_Pos)               /*!< NMI_T::NMIEN: UART0INT Mask            */

#define NMI_NMIEN_UART1INT_Pos          (15)                                            /*!< NMI_T::NMIEN: UART1INT Position        */
#define NMI_NMIEN_UART1INT_Msk          (0x1ul << NMI_NMIEN_UART1INT_Pos)               /*!< NMI_T::NMIEN: UART1INT Mask            */

#define NMI_NMISTS_BODOUT_Pos           (0)                                             /*!< NMI_T::NMISTS: BODOUT Position         */
#define NMI_NMISTS_BODOUT_Msk           (0x1ul << NMI_NMISTS_BODOUT_Pos)                /*!< NMI_T::NMISTS: BODOUT Mask             */

#define NMI_NMISTS_PWRWUINT_Pos         (2)                                             /*!< NMI_T::NMISTS: PWRWUINT Position       */
#define NMI_NMISTS_PWRWUINT_Msk         (0x1ul << NMI_NMISTS_PWRWUINT_Pos)              /*!< NMI_T::NMISTS: PWRWUINT Mask           */

#define NMI_NMISTS_CLKFAIL_Pos          (4)                                             /*!< NMI_T::NMISTS: CLKFAIL Position        */
#define NMI_NMISTS_CLKFAIL_Msk          (0x1ul << NMI_NMISTS_CLKFAIL_Pos)               /*!< NMI_T::NMISTS: CLKFAIL Mask            */

#define NMI_NMISTS_EINT0_Pos            (8)                                             /*!< NMI_T::NMISTS: EINT0 Position          */
#define NMI_NMISTS_EINT0_Msk            (0x1ul << NMI_NMISTS_EINT0_Pos)                 /*!< NMI_T::NMISTS: EINT0 Mask              */

#define NMI_NMISTS_EINT1_Pos            (9)                                             /*!< NMI_T::NMISTS: EINT1 Position          */
#define NMI_NMISTS_EINT1_Msk            (0x1ul << NMI_NMISTS_EINT1_Pos)                 /*!< NMI_T::NMISTS: EINT1 Mask              */

#define NMI_NMISTS_EINT2_Pos            (10)                                            /*!< NMI_T::NMISTS: EINT2 Position          */
#define NMI_NMISTS_EINT2_Msk            (0x1ul << NMI_NMISTS_EINT2_Pos)                 /*!< NMI_T::NMISTS: EINT2 Mask              */

#define NMI_NMISTS_EINT3_Pos            (11)                                            /*!< NMI_T::NMISTS: EINT3 Position          */
#define NMI_NMISTS_EINT3_Msk            (0x1ul << NMI_NMISTS_EINT3_Pos)                 /*!< NMI_T::NMISTS: EINT3 Mask              */

#define NMI_NMISTS_EINT4_Pos            (12)                                            /*!< NMI_T::NMISTS: EINT4 Position          */
#define NMI_NMISTS_EINT4_Msk            (0x1ul << NMI_NMISTS_EINT4_Pos)                 /*!< NMI_T::NMISTS: EINT4 Mask              */

#define NMI_NMISTS_EINT5_Pos            (13)                                            /*!< NMI_T::NMISTS: EINT5 Position          */
#define NMI_NMISTS_EINT5_Msk            (0x1ul << NMI_NMISTS_EINT5_Pos)                 /*!< NMI_T::NMISTS: EINT5 Mask              */

#define NMI_NMISTS_UART0INT_Pos         (14)                                            /*!< NMI_T::NMISTS: UART0INT Position       */
#define NMI_NMISTS_UART0INT_Msk         (0x1ul << NMI_NMISTS_UART0INT_Pos)              /*!< NMI_T::NMISTS: UART0INT Mask           */

#define NMI_NMISTS_UART1INT_Pos         (15)                                            /*!< NMI_T::NMISTS: UART1INT Position       */
#define NMI_NMISTS_UART1INT_Msk         (0x1ul << NMI_NMISTS_UART1INT_Pos)              /*!< NMI_T::NMISTS: UART1INT Mask           */

/**@}*/ /* NMI_CONST */
/**@}*/ /* end of SYS register group */
/**@}*/ /* end of REGISTER group */

#endif /* __SYS_REG_H__ */
