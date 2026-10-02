/**************************************************************************//**
 * @file     sys_reg.h
 * @version  V1.00
 * @brief    SYS register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
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
     * |        |          |This register reflects device part number code
     * |        |          |Software can read this register to identify which device is used
     * @var SYS_T::RSTCTL
     * Offset: 0x04  System Reset Control Register
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
     * |[7]     |CPURST    |Processor Core One-shot Reset (Write Protect)
     * |        |          |Setting this bit will only reset the processor core and Flash Memory Controller(FMC), and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |0 = Processor core normal operation.
     * |        |          |1 = Processor core one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::RSTSTS
     * Offset: 0x08  System Reset Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PORF      |POR Reset Flag
     * |        |          |The POR reset flag is set by the "Reset Signal" from the Power-on Reset (POR) Controller or bit CHIPRST (SYS_RSTCTL[0]) to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIPRST.
     * |        |          |1 = Power-on Reset (POR) or CHIPRST had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |[1]     |PINRF     |nRESET Pin Reset Flag
     * |        |          |The nRESET pin reset flag is set by the "Reset Signal" from the nRESET Pin to indicate the previous reset source.
     * |        |          |0 = No reset from nRESET pin.
     * |        |          |1 = Pin nRESET had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |[2]     |WDTRF     |WDT Reset Flag
     * |        |          |The WDT reset flag is set by the "Reset Signal" from the Watchdog Timer or Window Watchdog Timer to indicate the previous reset source.
     * |        |          |0 = No reset from watchdog timer or window watchdog timer.
     * |        |          |1 = The watchdog timer or window watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: Watchdog Timer register RSTF(WDT_CTL[2]) bit is set if the system has been reset by WDT time-out reset
     * |        |          |Window Watchdog Timer register WWDTRF(WWDT_STATUS[1]) bit is set if the system has been reset by WWDT time-out reset.
     * |        |          |Note 3: Extra Watchdog Timer register RSTF(EWDT_CTL[2]) bit is set if the system has been reset by EWDT time-out reset
     * |        |          |Extra Window Watchdog Timer register WWDTRF(EWWDT_STATUS[1]) bit is set if the system has been reset by EWWDT time-out reset.
     * |[3]     |LVRRF     |LVR Reset Flag
     * |        |          |The LVR reset flag is set by the "Reset Signal" from the Low Voltage Reset Controller to indicate the previous reset source.
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = The LVR controller had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |[4]     |BODRF     |BOD Reset Flag
     * |        |          |The BOD reset flag is set by the "Reset Signal" from the Brown-out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |[5]     |SYSRF     |System Reset Flag
     * |        |          |The system reset flag is set by the "Reset Signal" from the Cortex-M23 core to indicate the previous reset source.
     * |        |          |0 = No reset from the Cortex-M23.
     * |        |          |1 = The Cortex-M23 had issued the reset signal to reset the system by writing 1 to the bit SYSRESETREQ(AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M23 core.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |[7]     |CPURF     |CPU Reset Flag
     * |        |          |The CPU reset flag is set by hardware if software writes CPURST (SYS_RSTCTL[7]) 1 to reset the Cortex-M23 core and Flash Memory Controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = The Cortex-M23 core and FMC are reset by software setting CPURST to 1.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |[8]     |CPULKRF   |CPU Lockup Reset Flag
     * |        |          |The CPU Lockup reset flag is set by hardware if Cortex-M23 lockup happened.
     * |        |          |0 = No reset from CPU lockup happened.
     * |        |          |1 = The Cortex-M23 lockup happened and chip is reset.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: When CPU lockup happened under ICE is connected, this flag will set to 1 but chip will not reset.
     * @var SYS_T::VTORSET
     * Offset: 0x0C  VTOR Setting Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:8]  |VTORSET   |VTOR Setting After SPD Wakeup (Write Protect)
     * |        |          |The value will be loaded to Vector Table Offset Register, when CPU is reseted.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::BODCTL
     * Offset: 0x20  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODEN     |Brown-out Detector Enable Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBODEN (CONFIG0 [19]).
     * |        |          |0 = Brown-out Detector function Disabled.
     * |        |          |1 = Brown-out Detector function Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: The default value is set by inverse UCFG0[19].
     * |[1]     |BODLPM    |Brown-out Detector Low Power Mode (Write Protect)
     * |        |          |0 = BOD operated in normal mode (default).
     * |        |          |1 = BOD Low Power mode Enabled.
     * |        |          |Note 1: The BOD consumes about 10uA in normal mode, the low power mode can reduce the current to about 1/10 but slow the BOD response.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: Write operation is ignored when WRBUSY is high.
     * |[2]     |BODRSTEN  |Brown-out Reset Enable Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBORST(CONFIG0[20]) bit.
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled.
     * |        |          |1 = Brown-out "RESET" function Enabled.
     * |        |          |Note 1: When the Brown-out Detector function is enabled (BODEN high) and BOD reset function is enabled (BODRSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BODOUT high).
     * |        |          |When the BOD function is enabled (BODEN high) and BOD interrupt function is enabled (BODRSTEN low), BOD will assert an interrupt if AVDD is lower than BODVL, BOD interrupt will keep till the BODIF set to 0
     * |        |          |BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (setting BODEN low).
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 4: The default value is set by inverse UCFG0[20]
     * |[6:4]   |BODDGSEL  |Brown-out Detector Output De-glitch Time Select (Write Protect)
     * |        |          |000 = BOD output is sampled by LIRC clock.
     * |        |          |001 = 4 system clock (HCLK).
     * |        |          |010 = 8 system clock (HCLK).
     * |        |          |011 = 16 system clock (HCLK).
     * |        |          |100 = 32 system clock (HCLK).
     * |        |          |101 = 64 system clock (HCLK).
     * |        |          |110 = 128 system clock (HCLK).
     * |        |          |111 = 256 system clock (HCLK).
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: Because SCLK is invalid in Power-down mode, the de-glitch time must change to LIRC before system enters Power-down mode.
     * |        |          |Note 4: If BODLPM(SYS_BODCTL[1]) is 1, the Brown-out Detector Output de-glitch circuit always uses LIRC clock even if BODDGSEL is not 000.
     * |[9:8]   |BODVL     |Brown-out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBOV (CONFIG0 [22:21]).
     * |        |          |00 = Brown-out Detector threshold voltage is 2.4V.
     * |        |          |01 = Brown-out Detector threshold voltage is 2.7V.
     * |        |          |10 = Brown-out Detector threshold voltage is 3.7V.
     * |        |          |11 = Brown-out Detector threshold voltage is 4.4V.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: The default value is set by UCFG0[22:21].
     * |[13:12] |BODWKEN   |Brown-out Detector Wake-Up Selection (Write Protect)
     * |        |          |00 = Brown-out Detector wake-up function Disable.
     * |        |          |01 = Brown-out Detector power rise wake-up function Enable.
     * |        |          |10 = Brown-out Detector power drop wake-up function Enable.
     * |        |          |11 = Brown-out Detector power rise and drop wake-up function Enable.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |[16]    |LVREN     |Low Voltage Reset Enable Bit (Write Protect)
     * |        |          |The LVR function resets the chip when the input power voltage is lower than LVR circuit setting
     * |        |          |LVR function is enabled by default.
     * |        |          |0 = Low Voltage Reset function Disabled.
     * |        |          |1 = Low Voltage Reset function Enabled.
     * |        |          |Note 1: After enabling the bit, the LVR function will be active with 200us delay for LVR output stable (default).
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: Write operation is ignored when WRBUSY is high.
     * |[22:20] |LVRDGSEL  |LVR Output De-glitch Time Select (Write Protect)
     * |        |          |000 = Without de-glitch function.
     * |        |          |001 = 4 system clock (HCLK).
     * |        |          |010 = 8 system clock (HCLK).
     * |        |          |011 = 16 system clock (HCLK).
     * |        |          |100 = 32 system clock (HCLK).
     * |        |          |101 = 64 system clock (HCLK).
     * |        |          |110 = 128 system clock (HCLK).
     * |        |          |111 = 256 system clock (HCLK).
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Because SCLK is invalid in Power-down mode, the de-glitch time must change to without de-glitch before system enters Power-down mode.
     * |[31]    |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If SYS_BODCTL is written, this bit is asserted automatically by hardware, and is de-asserted when write procedure is finished.
     * |        |          |0 = SYS_BODCTL register is ready for write operation.
     * |        |          |1 = SYS_BODCTL register is busy on the last write operation. Other write operations are ignored.
     * @var SYS_T::BODSTS
     * Offset: 0x24  Brown-out Detector Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |Brown-out Detector Output Status (Read Only)
     * |        |          |0 = Brown-out Detector output status is 0.
     * |        |          |It means the detected voltage is higher than BODVL(SYS_BODCTL[9:8]) setting or BODEN is 0.
     * |        |          |1 = Brown-out Detector output status is 1.
     * |        |          |It means the detected voltage is lower than BODVL(SYS_BODCTL[9:8]) setting
     * |        |          |If the BODEN is 0, BOD function disabled, this bit always responds 0.
     * |[1]     |BODIF     |Brown-out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BODVL(SYS_BODCTL[9:8]) setting.
     * |        |          |1 = When Brown-out Detector detects the VDD is dropped down through the voltage of BODVL(SYS_BODCTL[9:8]) setting or the VDD is raised up through the voltage of BODVL(SYS_BODCTL[9:8]) setting, this bit is set to 1 and the brown-out interrupt is requested if brown-out interrupt is enabled.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * @var SYS_T::PORMKCTL
     * Offset: 0x28  Power-on Reset Mask Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PORLMASK  |Low Voltage Domian Power-on Reset Mask Enable Bit (Write Protect)
     * |        |          |When powered on, the POR15 circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR15 active again
     * |        |          |User can mask internal POR15 signal to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR15 function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[31:16] |PORHMASK  |High Voltage Voltage Domian Power-on Reset Mask Enable Bit (Write Protect)
     * |        |          |When powered on, the POR50 circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR50 active again
     * |        |          |User can mask internal POR50 signal to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR50 function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::PORCTL
     * Offset: 0x2C  Power-on Reset Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PORLOFF   |Low Voltage Domain Power-on Reset Enable Bit (Write Protect)
     * |        |          |0x5AA5 = POR15 function Disabled.
     * |        |          |Other = POR15 function Enabled.
     * |        |          |The POR15 function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[31:16] |PORHOFF   |High Voltage Domain Power-on Reset Enable Bit (Write Protect)
     * |        |          |0x5AA5 = POR50 function Disabled.
     * |        |          |Other = POR50 function Enabled.
     * |        |          |The POR50 function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
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
     * |[3:0]   |PC8MFP    |PC.8 Multi-function Pin Selection
     * |[7:4]   |PC9MFP    |PC.9 Multi-function Pin Selection
     * |[11:8]  |PC10MFP   |PC.10 Multi-function Pin Selection
     * |[15:12] |PC11MFP   |PC.11 Multi-function Pin Selection
     * |[19:16] |PC12MFP   |PC.12 Multi-function Pin Selection
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
     * |[19:16] |PD4MFP    |PD.4 Multi-function Pin Selection
     * |[23:20] |PD5MFP    |PD.5 Multi-function Pin Selection
     * |[27:24] |PD6MFP    |PD.6 Multi-function Pin Selection
     * |[31:28] |PD7MFP    |PD.7 Multi-function Pin Selection
     * @var SYS_T::GPD_MFPH
     * Offset: 0x4C  GPIOD High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PD8MFP    |PD.8 Multi-function Pin Selection
     * |[7:4]   |PD9MFP    |PD.9 Multi-function Pin Selection
     * |[11:8]  |PD10MFP   |PD.10 Multi-function Pin Selection
     * |[15:12] |PD11MFP   |PD.11 Multi-function Pin Selection
     * |[19:16] |PD12MFP   |PD.12 Multi-function Pin Selection
     * |[23:20] |PD13MFP   |PD.13 Multi-function Pin Selection
     * |[31:28] |PD15MFP   |PD.15 Multi-function Pin Selection
     * @var SYS_T::GPE_MFPL
     * Offset: 0x50  GPIOE Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PE0MFP    |PE.0 Multi-function Pin Selection
     * |[7:4]   |PE1MFP    |PE.1 Multi-function Pin Selection
     * |[11:8]  |PE2MFP    |PE.2 Multi-function Pin Selection
     * |[15:12] |PE3MFP    |PE.3 Multi-function Pin Selection
     * |[19:16] |PE4MFP    |PE.4 Multi-function Pin Selection
     * |[23:20] |PE5MFP    |PE.5 Multi-function Pin Selection
     * |[27:24] |PE6MFP    |PE.6 Multi-function Pin Selection
     * |[31:28] |PE7MFP    |PE.7 Multi-function Pin Selection
     * @var SYS_T::GPE_MFPH
     * Offset: 0x54  GPIOE High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PE8MFP    |PE.8 Multi-function Pin Selection
     * |[7:4]   |PE9MFP    |PE.9 Multi-function Pin Selection
     * |[11:8]  |PE10MFP   |PE.10 Multi-function Pin Selection
     * |[15:12] |PE11MFP   |PE.11 Multi-function Pin Selection
     * |[19:16] |PE12MFP   |PE.12 Multi-function Pin Selection
     * |[23:20] |PE13MFP   |PE.13 Multi-function Pin Selection
     * |[27:24] |PE14MFP   |PE.14 Multi-function Pin Selection
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
     * |[31:28] |PF7MFP    |PF.7 Multi-function Pin Selection
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
     * |[11:8]  |PG10MFP   |PG.10 Multi-function Pin Selection
     * |[15:12] |PG11MFP   |PG.11 Multi-function Pin Selection
     * |[19:16] |PG12MFP   |PG.12 Multi-function Pin Selection
     * @var SYS_T::GPA_MFOS
     * Offset: 0x80  GPIOA Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[1]     |MFOS1     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[2]     |MFOS2     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[3]     |MFOS3     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[4]     |MFOS4     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[5]     |MFOS5     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[6]     |MFOS6     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[7]     |MFOS7     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[8]     |MFOS8     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[9]     |MFOS9     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[10]    |MFOS10    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[11]    |MFOS11    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[12]    |MFOS12    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[13]    |MFOS13    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[14]    |MFOS14    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[15]    |MFOS15    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPB_MFOS
     * Offset: 0x84  GPIOB Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[1]     |MFOS1     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[2]     |MFOS2     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[3]     |MFOS3     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[4]     |MFOS4     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[5]     |MFOS5     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[6]     |MFOS6     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[7]     |MFOS7     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[8]     |MFOS8     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[9]     |MFOS9     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[10]    |MFOS10    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[11]    |MFOS11    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[12]    |MFOS12    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[13]    |MFOS13    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[14]    |MFOS14    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[15]    |MFOS15    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPC_MFOS
     * Offset: 0x88  GPIOC Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[1]     |MFOS1     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[2]     |MFOS2     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[3]     |MFOS3     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[4]     |MFOS4     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[5]     |MFOS5     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[6]     |MFOS6     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[7]     |MFOS7     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[8]     |MFOS8     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[9]     |MFOS9     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[10]    |MFOS10    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[11]    |MFOS11    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[12]    |MFOS12    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[13]    |MFOS13    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[14]    |MFOS14    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPD_MFOS
     * Offset: 0x8C  GPIOD Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOSx     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPE_MFOS
     * Offset: 0x90  GPIOE Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[1]     |MFOS1     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[2]     |MFOS2     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[3]     |MFOS3     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[4]     |MFOS4     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[5]     |MFOS5     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[6]     |MFOS6     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[7]     |MFOS7     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[8]     |MFOS8     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[9]     |MFOS9     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[10]    |MFOS10    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[11]    |MFOS11    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[12]    |MFOS12    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[13]    |MFOS13    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[14]    |MFOS14    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[15]    |MFOS15    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPF_MFOS
     * Offset: 0x94  GPIOF Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOSx     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::GPG_MFOS
     * Offset: 0x98  GPIOG Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[10]    |MFOS10    |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[11]    |MFOS11    |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |[12]    |MFOS12    |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * @var SYS_T::REGLCTL
     * Offset: 0x100  Register Lock Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |REGLCTL   |Register Lock Control Code
     * |        |          |Some registers have write-protection function
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value "59h", "16h", "88h" to this field
     * |        |          |After this sequence is completed, the REGLCTL bit will be set to 1 and write-protection registers can be normal write.
     * |        |          |REGLCTL[0]
     * |        |          |Register Lock Control Disable Index
     * |        |          |0 = Write-protection Enabled for writing protected registers
     * |        |          |Any write to the protected register is ignored.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * @var SYS_T::CPUCFG
     * Offset: 0x104  CPU General Configuration Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTRTEN   |CPU Interrupt Realtime Enable Bit
     * |        |          |When this bit is 0, the latency of CPU entering interrupt service routine (ISR) will be various but shorter.
     * |        |          |When this bit is 1, the latency of CPU entering ISR will be kept constant.
     * |        |          |0 = CPU Interrupt Realtime Disabled.
     * |        |          |1 = CPU Interrupt Realtime Enabled.
     * @var SYS_T::TCTLHIRC
     * Offset: 0x110  HIRC Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |FREQSEL   |Trim Frequency Selection
     * |        |          |This field indicates the target frequency of 12 MHz internal high speed RC oscillator (HIRC) auto trim.
     * |        |          |During auto trim operation, if clock error detected with CESTOPEN is set to 1 or trim retry limitation count reached, this field will be cleared to 00 automatically.
     * |        |          |00 = Disable HIRC auto trim function.
     * |        |          |01 = Enable HIRC auto trim function and trim HIRC to 12 MHz.
     * |        |          |10 = Reserved..
     * |        |          |11 = Reserved.
     * |[3:2]   |ACCURSEL  |Trim Accuracy Selection
     * |        |          |This field indicates the target frequency accuracy of 12 MHz internal high speed RC oscillator (IRC) auto trim.
     * |        |          |00 = Accuracy is +-0.25% deviation within all temperature ranges.
     * |        |          |01 = Accuracy is +-0.50% deviation within all temperature ranges.
     * |        |          |10 = Accuracy is +-0.75% deviation within all temperature ranges.
     * |        |          |11 = Accuracy is +-1% deviation within all temperature ranges.
     * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection
     * |        |          |This field defines that trim value calculation is based on how many reference clocks.
     * |        |          |00 = Trim value calculation is based on average difference in 4 clocks of reference clock.
     * |        |          |01 = Trim value calculation is based on average difference in 8 clocks of reference clock.
     * |        |          |10 = Trim value calculation is based on average difference in 16 clocks of reference clock.
     * |        |          |11 = Trim value calculation is based on average difference in 32 clocks of reference clock.
     * |        |          |Note: For example, if LOOPSEL is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 clocks of reference clock.
     * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count
     * |        |          |This field defines that how many times the auto trim circuit will try to update the HIRC trim value before the frequency of HIRC locked.
     * |        |          |Once the HIRC locked, the internal trim value update counter will be reset.
     * |        |          |If the trim value update counter reached this limitation value and frequency of HIRC is still not locked, the auto trim operation will be disabled and FREQSEL will be cleared to 00.
     * |        |          |00 = Trim retry count limitation is 64 loops.
     * |        |          |01 = Trim retry count limitation is 128 loops.
     * |        |          |10 = Trim retry count limitation is 256 loops.
     * |        |          |11 = Trim retry count limitation is 512 loops.
     * |[8]     |CESTOPEN  |Clock Error Stop Enable Bit
     * |        |          |0 = The trim operation keeps going if clock is inaccurate.
     * |        |          |1 = The trim operation is stopped if clock is inaccurate.
     * |[9]     |BOUNDEN   |Boundary Enable Bit
     * |        |          |0 = Boundary function Disabled.
     * |        |          |1 = Boundary function Enabled.
     * |[20:16] |BOUNDARY  |Boundary Selection
     * |        |          |Fill the boundary range from 0x1 to 0x1F, 0x0 is reserved.
     * |        |          |Note: This field is effective only when the BOUNDEN(SYS_TCTL[9]) is enabled.
     * @var SYS_T::TIENHIRC
     * Offset: 0x114  HIRC Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TFAILIEN  |Trim Failure Interrupt Enable Bit
     * |        |          |This bit controls if an interrupt will be triggered while HIRC trim value update limitation count reached and HIRC frequency still not locked on target frequency set by FREQSEL(SYS_TCTL[1:0]).
     * |        |          |If this bit is high and TFAILIF(SYS_TSTS[1]) is set during auto trim operation, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
     * |        |          |0 = Disable TFAILIF(SYS_TISTS[1]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable TFAILIF(SYS_TISTS[1]) status to trigger an interrupt to CPU.
     * |[2]     |CLKEIEN   |Clock Error Interrupt Enable Bit
     * |        |          |This bit controls if CPU would get an interrupt while clock is inaccuracy during auto trim operation.
     * |        |          |If this bit is set to1, and CLKERRIF(SYS_TISTS[2]) is set during auto trim operation, an interrupt will be triggered to notify the clock frequency is inaccuracy.
     * |        |          |0 = Disable CLKERRIF(SYS_TISTS[2]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable CLKERRIF(SYS_TISTS[2]) status to trigger an interrupt to CPU.
     * @var SYS_T::TISTSHIRC
     * Offset: 0x118  HIRC Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQLOCK  |HIRC Frequency Lock Status
     * |        |          |This bit indicates the HIRC frequency is locked.
     * |        |          |This is a status bit and doesn't trigger any interrupt.
     * |        |          |This bit will be set automatically, if the frequency is lock and the RC_TRIM is enabled.
     * |        |          |0 = The internal high-speed oscillator frequency is not locked at 12 MHz yet.
     * |        |          |1 = The internal high-speed oscillator frequency locked at 12 MHz.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[1]     |TFAILIF   |Trim Failure Interrupt Status
     * |        |          |This bit indicates that HIRC trim value update limitation count reached and the HIRC clock frequency is still not locked
     * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL(SYS_TCTL[1:0]) will be cleared to 00 by hardware automatically.
     * |        |          |If this bit is set and TFAILIEN(SYS_TIEN[1]) is high, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
     * |        |          |0 = Trim value update limitation count did not reach.
     * |        |          |1 = Trim value update limitation count reached and HIRC frequency still not locked.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[2]     |CLKERRIF  |Clock Error Interrupt Status
     * |        |          |When the frequency of 32.768 kHz external low speed crystal oscillator (LXT) or 12 MHz internal high speed RC oscillator (HIRC) is shift larger to unreasonable value, this bit will be set and to be an indicate that clock frequency is inaccuracy
     * |        |          |Once this bit is set to 1, the auto trim operation stopped and FREQSEL(SYS_TCTL[1:0]) will be cleared to 00 by hardware automatically if CESTOPEN(SYS_TCTL[8]) is set to 1.
     * |        |          |If this bit is set and CLKEIEN(SYS_IRCTIEN[2]) is high, an interrupt will be triggered to notify the clock frequency is inaccuracy.
     * |        |          |0 = Clock frequency is accurate.
     * |        |          |1 = Clock frequency is inaccurate.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[3]     |OVBDIF    |Over Boundary Status
     * |        |          |When the over boundary function is set, if there occurs the over boundary condition, this flag will be set.
     * |        |          |This is a status bit and doesn't trigger any interrupt.
     * |        |          |0 = Over boundary condition did not occur.
     * |        |          |1 = Over boundary condition occurred.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * @var SYS_T::ADCRST
     * Offset: 0x120  ADC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADC0RST   |ADCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the ADCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = ADCx controller normal operation.
     * |        |          |1 = ADCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::BPWMRST
     * Offset: 0x124  BPWM Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BPWM0RST  |BPWMx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the BPWMx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = BPWMx controller normal operation.
     * |        |          |1 = BPWMx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |BPWM1RST  |BPWMx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the BPWMx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = BPWMx controller normal operation.
     * |        |          |1 = BPWMx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::CRCRST
     * Offset: 0x128  CRC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRC0RST   |CRCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the CRCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = CRCx controller normal operation.
     * |        |          |1 = CRCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::FMCRST
     * Offset: 0x12C  FMC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FMC0RST   |FMCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the FMCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = FMCx controller normal operation.
     * |        |          |1 = FMCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[8]     |DFMC0RST  |Data FMCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the DFMCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = DFMCx controller normal operation.
     * |        |          |1 = DFMCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::GPIORST
     * Offset: 0x130  GPIO Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPIO0RST  |GPIOx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the GPIOx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = GPIOx controller normal operation.
     * |        |          |1 = GPIOx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::I2CRST
     * Offset: 0x134  I2C Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2C0RST   |I2Cx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I2Cx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I2Cx controller normal operation.
     * |        |          |1 = I2Cx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |I2C1RST   |I2Cx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I2Cx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I2Cx controller normal operation.
     * |        |          |1 = I2Cx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |I2C2RST   |I2Cx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I2Cx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I2Cx controller normal operation.
     * |        |          |1 = I2Cx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::PDMARST
     * Offset: 0x138  PDMA Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDMA0RST  |PDMAx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the PDMAx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = PDMAx controller normal operation.
     * |        |          |1 = PDMAx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::TMRRST
     * Offset: 0x140  Timer Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TMR0RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |TMR1RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |TMR2RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |TMR3RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |TMR4RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |TMR5RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |TMR6RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |TMR7RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[8]     |TMR8RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::UARTRST
     * Offset: 0x144  UART Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |UART0RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |UART1RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |UART2RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |UART3RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |UART4RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::USCIRST
     * Offset: 0x148  USCI Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USCI0RST  |USCIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the USCIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = USCIx controller normal operation.
     * |        |          |1 = USCIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |USCI1RST  |USCIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the USCIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = USCIx controller normal operation.
     * |        |          |1 = USCIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |USCI2RST  |USCIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the USCIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = USCIx controller normal operation.
     * |        |          |1 = USCIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |USCI3RST  |USCIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the USCIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = USCIx controller normal operation.
     * |        |          |1 = USCIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |USCI4RST  |USCIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the USCIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = USCIx controller normal operation.
     * |        |          |1 = USCIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::WWDTRST
     * Offset: 0x14C  WWDT Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT0RST  |WWDTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the WWDTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = WWDTx controller normal operation.
     * |        |          |1 = WWDTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::NMIEN
     * Offset: 0x160  NMI Source Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |BOD NMI Source Enable (Write Protect)
     * |        |          |0 = BOD NMI source Disabled.
     * |        |          |1 = BOD NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |IRCINT    |IRC TRIM NMI Source Enable (Write Protect)
     * |        |          |0 = IRC TRIM NMI source Disabled.
     * |        |          |1 = IRC TRIM NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |PWRWUINT  |Power-down Mode Wake-up NMI Source Enable (Write Protect)
     * |        |          |0 = Power-down mode wake-up NMI source Disabled.
     * |        |          |1 = Power-down mode wake-up NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |INTMUX    |NMI Multiplexer Source Enable (Write Protect)
     * |        |          |0 = NMI multiplexer source Disabled.
     * |        |          |1 = NMI multiplexer source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |CLKFAIL   |Clock Fail Detected NMI Source Enable (Write Protect)
     * |        |          |0 = Clock fail detected interrupt NMI source Disabled.
     * |        |          |1 = Clock fail detected interrupt NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |RTCINT    |RTC NMI Source Enable (Write Protect)
     * |        |          |0 = RTC NMI source Disabled.
     * |        |          |1 = RTC NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[8]     |EINT0     |External Interrupt From PA.6, or PB.5 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PA.6, or PB.5 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PA.6, or PB.5 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[9]     |EINT1     |External Interrupt From PA.7 or PB.4 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PA.7 or PB.4 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PA.7 or PB.4 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[10]    |EINT2     |External Interrupt From PB.3 or PC.6 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.3 or PC.6 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.3 or PC.6 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[11]    |EINT3     |External Interrupt From PB.2 or PC.7 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.2 or PC.7pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.2 or PC.7 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[12]    |EINT4     |External Interrupt From PA.8, PB.6 or PF.15 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PA.8, PB.6 or PF.15 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PA.8, PB.6 or PF.15 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[13]    |EINT5     |External Interrupt From PB.7 or PF.14 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.7 or PF.14 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.7 or PF.14 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[16]    |UART0INT  |UART0 NMI Source Enable (Write Protect)
     * |        |          |0 = UART0 NMI source Disabled.
     * |        |          |1 = UART0 NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[17]    |UART1INT  |UART1 NMI Source Enable (Write Protect)
     * |        |          |0 = UART1 NMI source Disabled.
     * |        |          |1 = UART1 NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register..
     * |[18]    |UART2INT  |UART2 NMI Source Enable (Write Protect)
     * |        |          |0 = UART2 NMI source Disabled.
     * |        |          |1 = UART2 NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[19]    |UART3INT  |UART3 NMI Source Enable (Write Protect)
     * |        |          |0 = UART3 NMI source Disabled.
     * |        |          |1 = UART3 NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[20]    |UART4INT  |UART4 NMI Source Enable (Write Protect)
     * |        |          |0 = UART4 NMI source Disabled.
     * |        |          |1 = UART4 NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[24]    |SR0MBECC  |SRAM 0 ECC Ckeck Multi Bit Error NMI Source Enable (Write Protect)
     * |        |          |0 = System SRAM0 ECC check error multi bit error NMI source Disabled.
     * |        |          |1 = System SRAM0 ECC check error multi bit error NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[25]    |SR1MBECC  |SRAM 1 ECC Ckeck Multi Bit Error NMI Source Enable (Write Protect)
     * |        |          |0 = System SRAM1 ECC check error multi bit error NMI source Disabled.
     * |        |          |1 = System SRAM1 ECC check error multi bit error NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[26]    |FSHMBECC  |FLASH ECC Ckeck Multi Bit Error NMI Source Enable (Write Protect)
     * |        |          |0 = FLASH ECC check error multi bit error NMI source Disabled.
     * |        |          |1 = FLASH ECC check error multi bit error NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::NMISTS
     * Offset: 0x164  NMI Source Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |BOD Interrupt Flag (Read Only)
     * |        |          |0 = BOD interrupt is deasserted.
     * |        |          |1 = BOD interrupt is asserted.
     * |[1]     |IRCINT    |IRC TRIM Interrupt Flag (Read Only)
     * |        |          |0 = HIRC TRIM interrupt is deasserted.
     * |        |          |1 = HIRC TRIM interrupt is asserted.
     * |[2]     |PWRWUINT  |Power-down Mode Wake-up Interrupt Flag (Read Only)
     * |        |          |0 = Power-down mode wake-up interrupt is deasserted.
     * |        |          |1 = Power-down mode wake-up interrupt is asserted.
     * |[3]     |INTMUX    |NMI Multiple source Interrupt Flag (Read Only)
     * |        |          |0 = NMI Multiple source Interrupt is deasserted.
     * |        |          |1 = NMI Multiple source Interrupt is asserted.
     * |[5]     |CLKFAIL   |Clock Fail Detected Interrupt Flag (Read Only)
     * |        |          |0 = Clock fail detected interrupt is deasserted.
     * |        |          |1 = Clock fail detected interrupt is asserted.
     * |[6]     |RTCINT    |RTC Interrupt Flag (Read Only)
     * |        |          |0 = RTC interrupt is deasserted.
     * |        |          |1 = RTC interrupt is asserted.
     * |[8]     |EINT0     |External Interrupt From PA.6, or PB.5 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PA.6, or PB.5 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PA.6, or PB.5 interrupt is asserted.
     * |[9]     |EINT1     |External Interrupt From PA.7, or PB.4 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PA.7, or PB.4 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PA.7, or PB.4 interrupt is asserted.
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
     * |[16]    |UART0INT  |UART0 Interrupt Flag (Read Only)
     * |        |          |0 = UART1 interrupt is deasserted.
     * |        |          |1 = UART1 interrupt is asserted.
     * |[17]    |UART1INT  |UART1 Interrupt Flag (Read Only)
     * |        |          |0 = UART1 interrupt is deasserted.
     * |        |          |1 = UART1 interrupt is asserted.
     * |[18]    |UART2INT  |UART2 Interrupt Flag (Read Only)
     * |        |          |0 = UART2 interrupt is deasserted.
     * |        |          |1 = UART2 interrupt is asserted.
     * |[19]    |UART3INT  |UART3 Interrupt Flag (Read Only)
     * |        |          |0 = UART3 interrupt is deasserted.
     * |        |          |1 = UART3 interrupt is asserted.
     * |[20]    |UART4INT  |UART4 Interrupt Flag (Read Only)
     * |        |          |0 = UART4 interrupt is deasserted.
     * |        |          |1 = UART4 interrupt is asserted.
     * |[24]    |SR0MBECC  |SRAM0 ECC Ckeck Multi Bit Error Interrupt Flag (Read Only)
     * |        |          |0 = SRAM0 ECC check error multi bit error interrupt is deasserted.
     * |        |          |1 = SRAM0 ECC check error multi bit error interrupt is asserted.
     * |[25]    |SR1MBECC  |SRAM1 ECC Ckeck Multi Bit Error Interrupt Flag (Read Only)
     * |        |          |0 = SRAM1 ECC check error multi bit error interrupt is deasserted.
     * |        |          |1 = SRAM1 ECC check error multi bit error interrupt is asserted.
     * |[26]    |FSHMBECC  |FLASH ECC Ckeck Multi Bit Error Interrupt Flag (Read Only)
     * |        |          |0 = FLASH ECC check error multi bit error interrupt is deasserted.
     * |        |          |1 = FLASH ECC check error multi bit error interrupt is asserted.
     * @var SYS_T::NMIMSEL
     * Offset: 0x168  NMI Multiplexer Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |NMIMSEL   |NMI Multiplexer Source Selection
     * |        |          |The interrupt numbers in Table 1.11-10 correspond to the selection signals of this multiplexer.
     * @var SYS_T::AHBCTL
     * Offset: 0x16C  AHB Bus Matrix Priority Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTACTEN  |Highest AHB Bus Priority of CortexM23 Core Enable Bit (Write Protect)
     * |        |          |Enable Cortex-M23 core with highest AHB bus priority in AHB bus matrix.
     * |        |          |0 = Round robin mode.
     * |        |          |1 = Cortex-M23 CPU with highest bus priority when interrupt occur.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::SRAMBCTL
     * Offset: 0x170  System SRAM BIST Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SR0BIST   |System SRAM 0 BIST Enable Bit (Write Protect)
     * |        |          |This bit enables System SRAM 0 BIST function.
     * |        |          |0 = System SRAM 0 BIST Disabled.
     * |        |          |1 = System SRAM 0 BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |SR1BIST   |System SRAM 1 BIST Enable Bit (Write Protect)
     * |        |          |This bit enables System SRAM 1 BIST function.
     * |        |          |0 = System SRAM 1 BIST Disabled.
     * |        |          |1 = System SRAM 1 BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::SRAMBFF
     * Offset: 0x174  System SRAM BIST Finish Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SR0BFF    |System SRAM 0 BIST Finish Flag (Read Only)
     * |        |          |0 = System SRAM 0 BIST is active.
     * |        |          |1 = System SRAM 0 BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when SR0BIST(SYS_SRAMBCTL[0]) set to 0.
     * |[1]     |SR1BFF    |System SRAM 1 BIST Finish Flag (Read Only)
     * |        |          |0 = System SRAM 1 BIST is active.
     * |        |          |1 = System SRAM 1 BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when SR1BIST(SYS_SRAMBCTL[1]) set to 0.
     * @var SYS_T::SRAMBRF
     * Offset: 0x178  System SRAM BIST Result Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SR0BRF    |System SRAM 0 BIST Result Flag (Read Only)
     * |        |          |0 = System SRAM 0 BIST test pass.
     * |        |          |1 = System SRAM 0 BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when SR0BIST(SYS_SRAMBCTL[0]) set to 0.
     * |[1]     |SR1BRF    |System SRAM 1 BIST Result Flag (Read Only)
     * |        |          |0 = System SRAM 1 BIST test pass.
     * |        |          |1 = System SRAM 1 BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when SR1BIST(SYS_SRAMBCTL[1]) set to 0.
     * @var SYS_T::SRAM0ICTL
     * Offset: 0x180  System SRAM0 Error Interrupt Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ECCEN     |SRAM0 ECC Enable Bit (Write Protect)
     * |        |          |0 = SRAM0 ECC disable.
     * |        |          |1 = SRAM0 ECC enable.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: The default value is set by ROMMAP0[20].
     * |[1]     |SBERRIEN  |SRAM0 ECC Single Bit Error Interrupt Enable Bit
     * |        |          |0 = SRAM0 error correcting code single bit error interrupt Disabled.
     * |        |          |1 = SRAM0 error correcting code single bit error interrupt Enabled.
     * |[2]     |MBERRIEN  |SRAM0 ECC Multi Bit Error Interrupt Enable Bit
     * |        |          |0 = SRAM0 error correcting code multi bit error interrupt Disabled.
     * |        |          |1 = SRAM0 error correcting code mulit bit error interrupt Enabled.
     * @var SYS_T::SRAM0STS
     * Offset: 0x184  System SRAM0 Error Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |SBERRIF   |SRAM0 ECC Check Single Bit Error Flag
     * |        |          |This bit indicates the System SRAM0 ECC single bit error occurred.
     * |        |          |0 = No System SRAM0 ECC single bit error.
     * |        |          |1 = System SRAM0 ECC single bit error occured.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM0
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[2]     |MBERRIF   |SRAM0 ECC Check Multi Bit Error Flag
     * |        |          |This bit indicates the System SRAM0 ECC single bit error occurred.
     * |        |          |0 = No System SRAM0 ECC multi bit error.
     * |        |          |1 = System SRAM0 ECC multi bit error occured.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM0
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[8]     |B0ECCEIF  |SRAM0 Byte 0 ECC Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM0 byte 0 ECC error occurred.
     * |        |          |0 = No System SRAM0 byte 0 ECC error.
     * |        |          |1 = System SRAM0 byte 0 ECC error occured.
     * |        |          |Note 1: Write 1 to SBERRIF or MBERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM0
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[9]     |B1ECCEIF  |SRAM0 Byte 1 ECC Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM0 byte 1 ECC error occurred.
     * |        |          |0 = No System SRAM0 byte 1 ECC error.
     * |        |          |1 = System SRAM0 byte 1 ECC error occured.
     * |        |          |Note 1: Write 1 to SBERRIF or MBERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM0
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[10]    |B2ECCEIF  |SRAM0 Byte 2 ECC Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM0 byte 2 ECC error occurred.
     * |        |          |0 = No System SRAM0 byte 2 ECC error.
     * |        |          |1 = System SRAM0 byte 2 ECC error occured.
     * |        |          |Note 1: Write 1 to SBERRIF or MBERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM0
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[11]    |B3ECCEIF  |SRAM0 Byte 3 ECC Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM0 byte 3 ECC error occurred.
     * |        |          |0 = No System SRAM0 byte 3 ECC error.
     * |        |          |1 = System SRAM0 byte 3 ECC error occured.
     * |        |          |Note 1: Write 1 to SBERRIF or MBERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM0
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * @var SYS_T::SRAM0EADR
     * Offset: 0x188  System SRAM0 Error Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:2]  |ECCEADDR  |System SRAM0 ECC Error Address (Read Only)
     * |        |          |This register shows system SRAM0 ECC error word offset.
     * @var SYS_T::SRAM1ICTL
     * Offset: 0x190  System SRAM1 Error Interrupt Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ECCEN     |SRAM1 ECC Enable Bit (Write Protect)
     * |        |          |0 = SRAM1 ECC disable.
     * |        |          |1 = SRAM1 ECC enable.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: The default value is set by ROMMAP0[20].
     * |[1]     |SBERRIEN  |SRAM1 ECC Single Bit Error Interrupt Enable Bit
     * |        |          |0 = SRAM1 error correcting code single bit error interrupt Disabled.
     * |        |          |1 = SRAM1 error correcting code single bit error interrupt Enabled.
     * |[2]     |MBERRIEN  |SRAM1 ECC Multi Bit Error Interrupt Enable Bit
     * |        |          |0 = SRAM1 error correcting code multi bit error interrupt Disabled.
     * |        |          |1 = SRAM1 error correcting code mulit bit error interrupt Enabled.
     * @var SYS_T::SRAM1STS
     * Offset: 0x194  System SRAM1 Error Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |SBERRIF   |SRAM1 ECC Check Single Bit Error Flag
     * |        |          |This bit indicates the System SRAM1 ECC single bit error occurred.
     * |        |          |0 = No System SRAM1 ECC single bit error.
     * |        |          |1 = System SRAM1 ECC single bit error occured.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM1
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[2]     |MBERRIF   |SRAM1 ECC Check Multi Bit Error Flag
     * |        |          |This bit indicates the System SRAM1 ECC single bit error occurred.
     * |        |          |0 = No System SRAM1 ECC multi bit error.
     * |        |          |1 = System SRAM1 ECC multi bit error occured.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM1
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[8]     |B0ECCEIF  |SRAM0 Byte 1 ECC Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM1 byte 0 ECC error occurred.
     * |        |          |0 = No System SRAM1 byte 0 ECC error.
     * |        |          |1 = System SRAM1 byte 0 ECC error occured.
     * |        |          |Note 1: Write 1 to SBERRIF or MBERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM1
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[9]     |B1ECCEIF  |SRAM1 Byte 1 ECC Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM1 byte 1 ECC error occurred.
     * |        |          |0 = No System SRAM1 byte 1 ECC error.
     * |        |          |1 = System SRAM1 byte 1 ECC error occured.
     * |        |          |Note 1: Write 1 to SBERRIF or MBERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM1
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[10]    |B2ECCEIF  |SRAM1 Byte 2 ECC Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM1 byte 2 ECC error occurred.
     * |        |          |0 = No System SRAM1 byte 2 ECC error.
     * |        |          |1 = System SRAM1 byte 2 ECC error occured.
     * |        |          |Note 1: Write 1 to SBERRIF or MBERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM1
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |[11]    |B3ECCEIF  |SRAM1 Byte 3 ECC Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM1 byte 3 ECC error occurred.
     * |        |          |0 = No System SRAM1 byte 3 ECC error.
     * |        |          |1 = System SRAM1 byte 3 ECC error occured.
     * |        |          |Note 1: Write 1 to SBERRIF or MBERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM1
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * @var SYS_T::SRAM1EADR
     * Offset: 0x198  System SRAM1 Error Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:2]  |ECCEADDR  |System SRAM1 ECC Error Address (Read Only)
     * |        |          |This register shows system SRAM0 ECC error word offset.
     * @var SYS_T::INTEN
     * Offset: 0x200  System Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDWKIEN   |Power-down Mode Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Power-down mode wake-up interrupt Disabled.
     * |        |          |1 = Power-down mode wake-up interrupt Enabled.
     * |        |          |Note 1: The Power-down mode wake-up interrupt will occur when both PDWKIF(SYS_INTSTS[0] and PDWKIEN(SYS_INTEN[0]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::INTSTS
     * Offset: 0x204  System Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDWKIF    |Power-down Mode Wake-up Interrupt Status (Read Only)
     * |        |          |Set by "Power-down wake-up event", it indicates that resume from Power-down mode.
     * |        |          |The flag is set if any wake-up source occurred. Refer Power Modes and Wake-up Sources chapter.
     * |        |          |Note 1: The Power-down mode wake-up interrupt will occur when both PDWKIF(SYS_INTSTS[0] and PDWKIEN(SYS_INTEN[0]) are 1.
     * |[31]    |CLRIF     |Clear Interrupt Flag
     * |        |          |Clearing the interrupt flag is not timely
     * |        |          |Flags in SYS_INTSTS will not be immediately cleared in the next cycle when CLRIF written to 1.
     * |        |          |0 = No clear.
     * |        |          |1 = Clear all interrupt flags.
     * |        |          |Note 1: This bit is auto cleared by hardware.
     * @var SYS_T::PWRCTL
     * Offset: 0x300  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[12]    |PRSTDBEN  |Pin Reset De-Bounce Enable Bit (Write Protect)
     * |        |          |When this bit is set to 1, pin reset clock base de-bounce circuit is enable in Power-down mode.
     * |        |          |0 = Pin reset de-bounce circuit Disable, and de-bounce time about 300ns.
     * |        |          |1 = Pin reset de-bounce circuit Enable, and de-bounce time about 32us.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |[31]    |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If PMC_PWRCTL is written, this bit is asserted automatically by hardware, and is de-asserted when write procedure is finished.
     * |        |          |0 = PMC_PWRCTL register is ready for write operation.
     * |        |          |1 = PMC_PWRCTL register is busy on the last write operation. Other write operations are ignored.
     */
    __I  uint32_t PDID;             /*!< [0x000] Part Device Identification Number Register             */
    __IO uint32_t RSTCTL;           /*!< [0x004] System Reset Control Register                          */
    __IO uint32_t RSTSTS;           /*!< [0x008] System Reset Status Register                           */
    __IO uint32_t VTORSET;          /*!< [0x00C] VTOR Setting Register                                  */
    __I  uint32_t RESERVE0[4];
    __IO uint32_t BODCTL;           /*!< [0x020] Brown-out Detector Control Register                    */
    __IO uint32_t BODSTS;           /*!< [0x024] Brown-out Detector Status Register                     */
    __IO uint32_t PORMKCTL;         /*!< [0x028] Power-on Reset Mask Controller Register                */
    __IO uint32_t PORCTL;           /*!< [0x02C] Power-on Reset Controller Register                     */
    __IO uint32_t GPA_MFPL;         /*!< [0x030] GPIOA Low Byte Multiple Function Control Register      */
    __IO uint32_t GPA_MFPH;         /*!< [0x034] GPIOA High Byte Multiple Function Control Register     */
    __IO uint32_t GPB_MFPL;         /*!< [0x038] GPIOB Low Byte Multiple Function Control Register      */
    __IO uint32_t GPB_MFPH;         /*!< [0x03C] GPIOB High Byte Multiple Function Control Register     */
    __IO uint32_t GPC_MFPL;         /*!< [0x040] GPIOC Low Byte Multiple Function Control Register      */
    __IO uint32_t GPC_MFPH;         /*!< [0x044] GPIOC High Byte Multiple Function Control Register     */
    __IO uint32_t GPD_MFPL;         /*!< [0x048] GPIOD Low Byte Multiple Function Control Register      */
    __IO uint32_t GPD_MFPH;         /*!< [0x04C] GPIOD High Byte Multiple Function Control Register     */
    __IO uint32_t GPE_MFPL;         /*!< [0x050] GPIOE Low Byte Multiple Function Control Register      */
    __IO uint32_t GPE_MFPH;         /*!< [0x054] GPIOE High Byte Multiple Function Control Register     */
    __IO uint32_t GPF_MFPL;         /*!< [0x058] GPIOF Low Byte Multiple Function Control Register      */
    __IO uint32_t GPF_MFPH;         /*!< [0x05C] GPIOF High Byte Multiple Function Control Register     */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t GPG_MFPH;         /*!< [0x064] GPIOG High Byte Multiple Function Control Register     */
    __I  uint32_t RESERVE2[6];
    __IO uint32_t GPA_MFOS;         /*!< [0x080] GPIOA Multiple Function Output Select Register         */
    __IO uint32_t GPB_MFOS;         /*!< [0x084] GPIOB Multiple Function Output Select Register         */
    __IO uint32_t GPC_MFOS;         /*!< [0x088] GPIOC Multiple Function Output Select Register         */
    __IO uint32_t GPD_MFOS;         /*!< [0x08C] GPIOD Multiple Function Output Select Register         */
    __IO uint32_t GPE_MFOS;         /*!< [0x090] GPIOE Multiple Function Output Select Register         */
    __IO uint32_t GPF_MFOS;         /*!< [0x094] GPIOF Multiple Function Output Select Register         */
    __IO uint32_t GPG_MFOS;         /*!< [0x098] GPIOG Multiple Function Output Select Register         */
    __I  uint32_t RESERVE3[25];
    __IO uint32_t REGLCTL;          /*!< [0x100] Register Lock Control Register                         */
    __IO uint32_t CPUCFG;           /*!< [0x104] CPU General Configuration Register                     */
    __I  uint32_t RESERVE4[2];
    __IO uint32_t TCTLHIRC;         /*!< [0x110] HIRC Trim Control Register                             */
    __IO uint32_t TIENHIRC;         /*!< [0x114] HIRC Trim Interrupt Enable Register                    */
    __IO uint32_t TISTSHIRC;        /*!< [0x118] HIRC Trim Interrupt Status Register                    */
    __I  uint32_t RESERVE5[1];
    __IO uint32_t ADCRST;           /*!< [0x120] ADC Reset Control Register                             */
    __IO uint32_t BPWMRST;          /*!< [0x124] BPWM Reset Control Register                            */
    __IO uint32_t CRCRST;           /*!< [0x128] CRC Reset Control Register                             */
    __IO uint32_t FMCRST;           /*!< [0x12C] FMC Reset Control Register                             */
    __IO uint32_t GPIORST;          /*!< [0x130] GPIO Reset Control Register                            */
    __IO uint32_t I2CRST;           /*!< [0x134] I2C Reset Control Register                             */
    __IO uint32_t PDMARST;          /*!< [0x138] PDMA Reset Control Register                            */
    __I  uint32_t RESERVE6[1];
    __IO uint32_t TMRRST;           /*!< [0x140] Timer Reset Control Register                           */
    __IO uint32_t UARTRST;          /*!< [0x144] UART Reset Control Register                            */
    __IO uint32_t USCIRST;          /*!< [0x148] USCI Reset Control Register                            */
    __IO uint32_t WWDTRST;          /*!< [0x14C] WWDT Reset Control Register                            */
    __I  uint32_t RESERVE7[4];
    __IO uint32_t NMIEN;            /*!< [0x160] NMI Source Interrupt Enable Register                   */
    __I  uint32_t NMISTS;           /*!< [0x164] NMI Source Interrupt Status Register                   */
    __IO uint32_t NMIMSEL;          /*!< [0x168] NMI Multiplexer Source Select Control Register         */
    __IO uint32_t AHBCTL;           /*!< [0x16C] AHB Bus Matrix Priority Control Register               */
    __IO uint32_t SRAMBCTL;         /*!< [0x170] System SRAM BIST Control Register                      */
    __I  uint32_t SRAMBFF;          /*!< [0x174] System SRAM BIST Finish Flag Register                  */
    __I  uint32_t SRAMBRF;          /*!< [0x178] System SRAM BIST Result Flag Register                  */
    __I  uint32_t RESERVE8[1];
    __IO uint32_t SRAM0ICTL;        /*!< [0x180] System SRAM0 Error Interrupt Enable Control Register   */
    __IO uint32_t SRAM0STS;         /*!< [0x184] System SRAM0 Error Status Register                     */
    __I  uint32_t SRAM0EADR;        /*!< [0x188] System SRAM0 Error Address Register                    */
    __I  uint32_t RESERVE9[1];
    __IO uint32_t SRAM1ICTL;        /*!< [0x190] System SRAM1 Error Interrupt Enable Control Register   */
    __IO uint32_t SRAM1STS;         /*!< [0x194] System SRAM1 Error Status Register                     */
    __I  uint32_t SRAM1EADR;        /*!< [0x198] System SRAM1 Error Address Register                    */
    __I  uint32_t RESERVE10[25];
    __IO uint32_t INTEN;            /*!< [0x200] System Interrupt Enable Register                       */
    __IO uint32_t INTSTS;           /*!< [0x204] System Interrupt Status Register                       */
    __I  uint32_t RESERVE11[62];
    __IO uint32_t PWRCTL;           /*!< [0x300] System Power-down Control Register                     */
} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
  @{
*/

#define SYS_PDID_PDID_Pos               (0)                                         /*!< SYS_T::PDID: PDID Position                 */
#define SYS_PDID_PDID_Msk               (0xFFFFFFFFUL << SYS_PDID_PDID_Pos)         /*!< SYS_T::PDID: PDID Mask                     */

#define SYS_RSTCTL_CHIPRST_Pos          (0)                                         /*!< SYS_T::RSTCTL: CHIPRST Position            */
#define SYS_RSTCTL_CHIPRST_Msk          (0x1UL << SYS_RSTCTL_CHIPRST_Pos)           /*!< SYS_T::RSTCTL: CHIPRST Mask                */

#define SYS_RSTCTL_CPURST_Pos           (7)                                         /*!< SYS_T::RSTCTL: CPURST Position             */
#define SYS_RSTCTL_CPURST_Msk           (0x1UL << SYS_RSTCTL_CPURST_Pos)            /*!< SYS_T::RSTCTL: CPURST Mask                 */

#define SYS_RSTSTS_PORF_Pos             (0)                                         /*!< SYS_T::RSTSTS: PORF Position               */
#define SYS_RSTSTS_PORF_Msk             (0x1UL << SYS_RSTSTS_PORF_Pos)              /*!< SYS_T::RSTSTS: PORF Mask                   */

#define SYS_RSTSTS_PINRF_Pos            (1)                                         /*!< SYS_T::RSTSTS: PINRF Position              */
#define SYS_RSTSTS_PINRF_Msk            (0x1UL << SYS_RSTSTS_PINRF_Pos)             /*!< SYS_T::RSTSTS: PINRF Mask                  */

#define SYS_RSTSTS_WDTRF_Pos            (2)                                         /*!< SYS_T::RSTSTS: WDTRF Position              */
#define SYS_RSTSTS_WDTRF_Msk            (0x1UL << SYS_RSTSTS_WDTRF_Pos)             /*!< SYS_T::RSTSTS: WDTRF Mask                  */

#define SYS_RSTSTS_LVRRF_Pos            (3)                                         /*!< SYS_T::RSTSTS: LVRRF Position              */
#define SYS_RSTSTS_LVRRF_Msk            (0x1UL << SYS_RSTSTS_LVRRF_Pos)             /*!< SYS_T::RSTSTS: LVRRF Mask                  */

#define SYS_RSTSTS_BODRF_Pos            (4)                                         /*!< SYS_T::RSTSTS: BODRF Position              */
#define SYS_RSTSTS_BODRF_Msk            (0x1UL << SYS_RSTSTS_BODRF_Pos)             /*!< SYS_T::RSTSTS: BODRF Mask                  */

#define SYS_RSTSTS_SYSRF_Pos            (5)                                         /*!< SYS_T::RSTSTS: SYSRF Position              */
#define SYS_RSTSTS_SYSRF_Msk            (0x1UL << SYS_RSTSTS_SYSRF_Pos)             /*!< SYS_T::RSTSTS: SYSRF Mask                  */

#define SYS_RSTSTS_CPURF_Pos            (7)                                         /*!< SYS_T::RSTSTS: CPURF Position              */
#define SYS_RSTSTS_CPURF_Msk            (0x1UL << SYS_RSTSTS_CPURF_Pos)             /*!< SYS_T::RSTSTS: CPURF Mask                  */

#define SYS_RSTSTS_CPULKRF_Pos          (8)                                         /*!< SYS_T::RSTSTS: CPULKRF Position            */
#define SYS_RSTSTS_CPULKRF_Msk          (0x1UL << SYS_RSTSTS_CPULKRF_Pos)           /*!< SYS_T::RSTSTS: CPULKRF Mask                */

#define SYS_VTORSET_VTORSET_Pos         (8)                                         /*!< SYS_T::VTORSET: VTORSET Position           */
#define SYS_VTORSET_VTORSET_Msk         (0xFFFFFFUL << SYS_VTORSET_VTORSET_Pos)     /*!< SYS_T::VTORSET: VTORSET Mask               */

#define SYS_BODCTL_BODEN_Pos            (0)                                         /*!< SYS_T::BODCTL: BODEN Position              */
#define SYS_BODCTL_BODEN_Msk            (0x1UL << SYS_BODCTL_BODEN_Pos)             /*!< SYS_T::BODCTL: BODEN Mask                  */

#define SYS_BODCTL_BODLPM_Pos           (1)                                         /*!< SYS_T::BODCTL: BODLPM Position             */
#define SYS_BODCTL_BODLPM_Msk           (0x1UL << SYS_BODCTL_BODLPM_Pos)            /*!< SYS_T::BODCTL: BODLPM Mask                 */

#define SYS_BODCTL_BODRSTEN_Pos         (2)                                         /*!< SYS_T::BODCTL: BODRSTEN Position           */
#define SYS_BODCTL_BODRSTEN_Msk         (0x1UL << SYS_BODCTL_BODRSTEN_Pos)          /*!< SYS_T::BODCTL: BODRSTEN Mask               */

#define SYS_BODCTL_BODDGSEL_Pos         (4)                                         /*!< SYS_T::BODCTL: BODDGSEL Position           */
#define SYS_BODCTL_BODDGSEL_Msk         (0x7UL << SYS_BODCTL_BODDGSEL_Pos)          /*!< SYS_T::BODCTL: BODDGSEL Mask               */

#define SYS_BODCTL_BODVL_Pos            (8)                                         /*!< SYS_T::BODCTL: BODVL Position              */
#define SYS_BODCTL_BODVL_Msk            (0x3UL << SYS_BODCTL_BODVL_Pos)             /*!< SYS_T::BODCTL: BODVL Mask                  */

#define SYS_BODCTL_BODWKEN_Pos          (12)                                        /*!< SYS_T::BODCTL: BODWKEN Position            */
#define SYS_BODCTL_BODWKEN_Msk          (0x3UL << SYS_BODCTL_BODWKEN_Pos)           /*!< SYS_T::BODCTL: BODWKEN Mask                */

#define SYS_BODCTL_LVREN_Pos            (16)                                        /*!< SYS_T::BODCTL: LVREN Position              */
#define SYS_BODCTL_LVREN_Msk            (0x1UL << SYS_BODCTL_LVREN_Pos)             /*!< SYS_T::BODCTL: LVREN Mask                  */

#define SYS_BODCTL_LVRDGSEL_Pos         (20)                                        /*!< SYS_T::BODCTL: LVRDGSEL Position           */
#define SYS_BODCTL_LVRDGSEL_Msk         (0x7UL << SYS_BODCTL_LVRDGSEL_Pos)          /*!< SYS_T::BODCTL: LVRDGSEL Mask               */

#define SYS_BODCTL_WRBUSY_Pos           (31)                                        /*!< SYS_T::BODCTL: WRBUSY Position             */
#define SYS_BODCTL_WRBUSY_Msk           (0x1UL << SYS_BODCTL_WRBUSY_Pos)            /*!< SYS_T::BODCTL: WRBUSY Mask                 */

#define SYS_BODSTS_BODOUT_Pos           (0)                                         /*!< SYS_T::BODSTS: BODOUT Position             */
#define SYS_BODSTS_BODOUT_Msk           (0x1UL << SYS_BODSTS_BODOUT_Pos)            /*!< SYS_T::BODSTS: BODOUT Mask                 */

#define SYS_BODSTS_BODIF_Pos            (1)                                         /*!< SYS_T::BODSTS: BODIF Position              */
#define SYS_BODSTS_BODIF_Msk            (0x1UL << SYS_BODSTS_BODIF_Pos)             /*!< SYS_T::BODSTS: BODIF Mask                  */

#define SYS_PORMKCTL_PORLMASK_Pos       (0)                                         /*!< SYS_T::PORMKCTL: PORLMASK Position         */
#define SYS_PORMKCTL_PORLMASK_Msk       (0xFFFFUL << SYS_PORMKCTL_PORLMASK_Pos)     /*!< SYS_T::PORMKCTL: PORLMASK Mask             */

#define SYS_PORMKCTL_PORHMASK_Pos       (16)                                        /*!< SYS_T::PORMKCTL: PORHMASK Position         */
#define SYS_PORMKCTL_PORHMASK_Msk       (0xFFFFUL << SYS_PORMKCTL_PORHMASK_Pos)     /*!< SYS_T::PORMKCTL: PORHMASK Mask             */

#define SYS_PORCTL_PORLOFF_Pos          (0)                                         /*!< SYS_T::PORCTL: PORLOFF Position            */
#define SYS_PORCTL_PORLOFF_Msk          (0xFFFFUL << SYS_PORCTL_PORLOFF_Pos)        /*!< SYS_T::PORCTL: PORLOFF Mask                */

#define SYS_PORCTL_PORHOFF_Pos          (16)                                        /*!< SYS_T::PORCTL: PORHOFF Position            */
#define SYS_PORCTL_PORHOFF_Msk          (0xFFFFUL << SYS_PORCTL_PORHOFF_Pos)        /*!< SYS_T::PORCTL: PORHOFF Mask                */

#define SYS_GPA_MFPL_PA0MFP_Pos         (0)                                         /*!< SYS_T::GPA_MFPL: PA0MFP Position           */
#define SYS_GPA_MFPL_PA0MFP_Msk         (0xFUL << SYS_GPA_MFPL_PA0MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA0MFP Mask               */

#define SYS_GPA_MFPL_PA1MFP_Pos         (4)                                         /*!< SYS_T::GPA_MFPL: PA1MFP Position           */
#define SYS_GPA_MFPL_PA1MFP_Msk         (0xFUL << SYS_GPA_MFPL_PA1MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA1MFP Mask               */

#define SYS_GPA_MFPL_PA2MFP_Pos         (8)                                         /*!< SYS_T::GPA_MFPL: PA2MFP Position           */
#define SYS_GPA_MFPL_PA2MFP_Msk         (0xFUL << SYS_GPA_MFPL_PA2MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA2MFP Mask               */

#define SYS_GPA_MFPL_PA3MFP_Pos         (12)                                        /*!< SYS_T::GPA_MFPL: PA3MFP Position           */
#define SYS_GPA_MFPL_PA3MFP_Msk         (0xFUL << SYS_GPA_MFPL_PA3MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA3MFP Mask               */

#define SYS_GPA_MFPL_PA4MFP_Pos         (16)                                        /*!< SYS_T::GPA_MFPL: PA4MFP Position           */
#define SYS_GPA_MFPL_PA4MFP_Msk         (0xFUL << SYS_GPA_MFPL_PA4MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA4MFP Mask               */

#define SYS_GPA_MFPL_PA5MFP_Pos         (20)                                        /*!< SYS_T::GPA_MFPL: PA5MFP Position           */
#define SYS_GPA_MFPL_PA5MFP_Msk         (0xFUL << SYS_GPA_MFPL_PA5MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA5MFP Mask               */

#define SYS_GPA_MFPL_PA6MFP_Pos         (24)                                        /*!< SYS_T::GPA_MFPL: PA6MFP Position           */
#define SYS_GPA_MFPL_PA6MFP_Msk         (0xFUL << SYS_GPA_MFPL_PA6MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA6MFP Mask               */

#define SYS_GPA_MFPL_PA7MFP_Pos         (28)                                        /*!< SYS_T::GPA_MFPL: PA7MFP Position           */
#define SYS_GPA_MFPL_PA7MFP_Msk         (0xFUL << SYS_GPA_MFPL_PA7MFP_Pos)          /*!< SYS_T::GPA_MFPL: PA7MFP Mask               */

#define SYS_GPA_MFPH_PA8MFP_Pos         (0)                                         /*!< SYS_T::GPA_MFPH: PA8MFP Position           */
#define SYS_GPA_MFPH_PA8MFP_Msk         (0xFUL << SYS_GPA_MFPH_PA8MFP_Pos)          /*!< SYS_T::GPA_MFPH: PA8MFP Mask               */

#define SYS_GPA_MFPH_PA9MFP_Pos         (4)                                         /*!< SYS_T::GPA_MFPH: PA9MFP Position           */
#define SYS_GPA_MFPH_PA9MFP_Msk         (0xFUL << SYS_GPA_MFPH_PA9MFP_Pos)          /*!< SYS_T::GPA_MFPH: PA9MFP Mask               */

#define SYS_GPA_MFPH_PA10MFP_Pos        (8)                                         /*!< SYS_T::GPA_MFPH: PA10MFP Position          */
#define SYS_GPA_MFPH_PA10MFP_Msk        (0xFUL << SYS_GPA_MFPH_PA10MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA10MFP Mask              */

#define SYS_GPA_MFPH_PA11MFP_Pos        (12)                                        /*!< SYS_T::GPA_MFPH: PA11MFP Position          */
#define SYS_GPA_MFPH_PA11MFP_Msk        (0xFUL << SYS_GPA_MFPH_PA11MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA11MFP Mask              */

#define SYS_GPA_MFPH_PA12MFP_Pos        (16)                                        /*!< SYS_T::GPA_MFPH: PA12MFP Position          */
#define SYS_GPA_MFPH_PA12MFP_Msk        (0xFUL << SYS_GPA_MFPH_PA12MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA12MFP Mask              */

#define SYS_GPA_MFPH_PA13MFP_Pos        (20)                                        /*!< SYS_T::GPA_MFPH: PA13MFP Position          */
#define SYS_GPA_MFPH_PA13MFP_Msk        (0xFUL << SYS_GPA_MFPH_PA13MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA13MFP Mask              */

#define SYS_GPA_MFPH_PA14MFP_Pos        (24)                                        /*!< SYS_T::GPA_MFPH: PA14MFP Position          */
#define SYS_GPA_MFPH_PA14MFP_Msk        (0xFUL << SYS_GPA_MFPH_PA14MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA14MFP Mask              */

#define SYS_GPA_MFPH_PA15MFP_Pos        (28)                                        /*!< SYS_T::GPA_MFPH: PA15MFP Position          */
#define SYS_GPA_MFPH_PA15MFP_Msk        (0xFUL << SYS_GPA_MFPH_PA15MFP_Pos)         /*!< SYS_T::GPA_MFPH: PA15MFP Mask              */

#define SYS_GPB_MFPL_PB0MFP_Pos         (0)                                         /*!< SYS_T::GPB_MFPL: PB0MFP Position           */
#define SYS_GPB_MFPL_PB0MFP_Msk         (0xFUL << SYS_GPB_MFPL_PB0MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB0MFP Mask               */

#define SYS_GPB_MFPL_PB1MFP_Pos         (4)                                         /*!< SYS_T::GPB_MFPL: PB1MFP Position           */
#define SYS_GPB_MFPL_PB1MFP_Msk         (0xFUL << SYS_GPB_MFPL_PB1MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB1MFP Mask               */

#define SYS_GPB_MFPL_PB2MFP_Pos         (8)                                         /*!< SYS_T::GPB_MFPL: PB2MFP Position           */
#define SYS_GPB_MFPL_PB2MFP_Msk         (0xFUL << SYS_GPB_MFPL_PB2MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB2MFP Mask               */

#define SYS_GPB_MFPL_PB3MFP_Pos         (12)                                        /*!< SYS_T::GPB_MFPL: PB3MFP Position           */
#define SYS_GPB_MFPL_PB3MFP_Msk         (0xFUL << SYS_GPB_MFPL_PB3MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB3MFP Mask               */

#define SYS_GPB_MFPL_PB4MFP_Pos         (16)                                        /*!< SYS_T::GPB_MFPL: PB4MFP Position           */
#define SYS_GPB_MFPL_PB4MFP_Msk         (0xFUL << SYS_GPB_MFPL_PB4MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB4MFP Mask               */

#define SYS_GPB_MFPL_PB5MFP_Pos         (20)                                        /*!< SYS_T::GPB_MFPL: PB5MFP Position           */
#define SYS_GPB_MFPL_PB5MFP_Msk         (0xFUL << SYS_GPB_MFPL_PB5MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB5MFP Mask               */

#define SYS_GPB_MFPL_PB6MFP_Pos         (24)                                        /*!< SYS_T::GPB_MFPL: PB6MFP Position           */
#define SYS_GPB_MFPL_PB6MFP_Msk         (0xFUL << SYS_GPB_MFPL_PB6MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB6MFP Mask               */

#define SYS_GPB_MFPL_PB7MFP_Pos         (28)                                        /*!< SYS_T::GPB_MFPL: PB7MFP Position           */
#define SYS_GPB_MFPL_PB7MFP_Msk         (0xFUL << SYS_GPB_MFPL_PB7MFP_Pos)          /*!< SYS_T::GPB_MFPL: PB7MFP Mask               */

#define SYS_GPB_MFPH_PB8MFP_Pos         (0)                                         /*!< SYS_T::GPB_MFPH: PB8MFP Position           */
#define SYS_GPB_MFPH_PB8MFP_Msk         (0xFUL << SYS_GPB_MFPH_PB8MFP_Pos)          /*!< SYS_T::GPB_MFPH: PB8MFP Mask               */

#define SYS_GPB_MFPH_PB9MFP_Pos         (4)                                         /*!< SYS_T::GPB_MFPH: PB9MFP Position           */
#define SYS_GPB_MFPH_PB9MFP_Msk         (0xFUL << SYS_GPB_MFPH_PB9MFP_Pos)          /*!< SYS_T::GPB_MFPH: PB9MFP Mask               */

#define SYS_GPB_MFPH_PB10MFP_Pos        (8)                                         /*!< SYS_T::GPB_MFPH: PB10MFP Position          */
#define SYS_GPB_MFPH_PB10MFP_Msk        (0xFUL << SYS_GPB_MFPH_PB10MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB10MFP Mask              */

#define SYS_GPB_MFPH_PB11MFP_Pos        (12)                                        /*!< SYS_T::GPB_MFPH: PB11MFP Position          */
#define SYS_GPB_MFPH_PB11MFP_Msk        (0xFUL << SYS_GPB_MFPH_PB11MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB11MFP Mask              */

#define SYS_GPB_MFPH_PB12MFP_Pos        (16)                                        /*!< SYS_T::GPB_MFPH: PB12MFP Position          */
#define SYS_GPB_MFPH_PB12MFP_Msk        (0xFUL << SYS_GPB_MFPH_PB12MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB12MFP Mask              */

#define SYS_GPB_MFPH_PB13MFP_Pos        (20)                                        /*!< SYS_T::GPB_MFPH: PB13MFP Position          */
#define SYS_GPB_MFPH_PB13MFP_Msk        (0xFUL << SYS_GPB_MFPH_PB13MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB13MFP Mask              */

#define SYS_GPB_MFPH_PB14MFP_Pos        (24)                                        /*!< SYS_T::GPB_MFPH: PB14MFP Position          */
#define SYS_GPB_MFPH_PB14MFP_Msk        (0xFUL << SYS_GPB_MFPH_PB14MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB14MFP Mask              */

#define SYS_GPB_MFPH_PB15MFP_Pos        (28)                                        /*!< SYS_T::GPB_MFPH: PB15MFP Position          */
#define SYS_GPB_MFPH_PB15MFP_Msk        (0xFUL << SYS_GPB_MFPH_PB15MFP_Pos)         /*!< SYS_T::GPB_MFPH: PB15MFP Mask              */

#define SYS_GPC_MFPL_PC0MFP_Pos         (0)                                         /*!< SYS_T::GPC_MFPL: PC0MFP Position           */
#define SYS_GPC_MFPL_PC0MFP_Msk         (0xFUL << SYS_GPC_MFPL_PC0MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC0MFP Mask               */

#define SYS_GPC_MFPL_PC1MFP_Pos         (4)                                         /*!< SYS_T::GPC_MFPL: PC1MFP Position           */
#define SYS_GPC_MFPL_PC1MFP_Msk         (0xFUL << SYS_GPC_MFPL_PC1MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC1MFP Mask               */

#define SYS_GPC_MFPL_PC2MFP_Pos         (8)                                         /*!< SYS_T::GPC_MFPL: PC2MFP Position           */
#define SYS_GPC_MFPL_PC2MFP_Msk         (0xFUL << SYS_GPC_MFPL_PC2MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC2MFP Mask               */

#define SYS_GPC_MFPL_PC3MFP_Pos         (12)                                        /*!< SYS_T::GPC_MFPL: PC3MFP Position           */
#define SYS_GPC_MFPL_PC3MFP_Msk         (0xFUL << SYS_GPC_MFPL_PC3MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC3MFP Mask               */

#define SYS_GPC_MFPL_PC4MFP_Pos         (16)                                        /*!< SYS_T::GPC_MFPL: PC4MFP Position           */
#define SYS_GPC_MFPL_PC4MFP_Msk         (0xFUL << SYS_GPC_MFPL_PC4MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC4MFP Mask               */

#define SYS_GPC_MFPL_PC5MFP_Pos         (20)                                        /*!< SYS_T::GPC_MFPL: PC5MFP Position           */
#define SYS_GPC_MFPL_PC5MFP_Msk         (0xFUL << SYS_GPC_MFPL_PC5MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC5MFP Mask               */

#define SYS_GPC_MFPL_PC6MFP_Pos         (24)                                        /*!< SYS_T::GPC_MFPL: PC6MFP Position           */
#define SYS_GPC_MFPL_PC6MFP_Msk         (0xFUL << SYS_GPC_MFPL_PC6MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC6MFP Mask               */

#define SYS_GPC_MFPL_PC7MFP_Pos         (28)                                        /*!< SYS_T::GPC_MFPL: PC7MFP Position           */
#define SYS_GPC_MFPL_PC7MFP_Msk         (0xFUL << SYS_GPC_MFPL_PC7MFP_Pos)          /*!< SYS_T::GPC_MFPL: PC7MFP Mask               */

#define SYS_GPC_MFPH_PC8MFP_Pos         (0)                                         /*!< SYS_T::GPC_MFPH: PC8MFP Position           */
#define SYS_GPC_MFPH_PC8MFP_Msk         (0xFUL << SYS_GPC_MFPH_PC8MFP_Pos)          /*!< SYS_T::GPC_MFPH: PC8MFP Mask               */

#define SYS_GPC_MFPH_PC9MFP_Pos         (4)                                         /*!< SYS_T::GPC_MFPH: PC9MFP Position           */
#define SYS_GPC_MFPH_PC9MFP_Msk         (0xFUL << SYS_GPC_MFPH_PC9MFP_Pos)          /*!< SYS_T::GPC_MFPH: PC9MFP Mask               */

#define SYS_GPC_MFPH_PC10MFP_Pos        (8)                                         /*!< SYS_T::GPC_MFPH: PC10MFP Position          */
#define SYS_GPC_MFPH_PC10MFP_Msk        (0xFUL << SYS_GPC_MFPH_PC10MFP_Pos)         /*!< SYS_T::GPC_MFPH: PC10MFP Mask              */

#define SYS_GPC_MFPH_PC11MFP_Pos        (12)                                        /*!< SYS_T::GPC_MFPH: PC11MFP Position          */
#define SYS_GPC_MFPH_PC11MFP_Msk        (0xFUL << SYS_GPC_MFPH_PC11MFP_Pos)         /*!< SYS_T::GPC_MFPH: PC11MFP Mask              */

#define SYS_GPC_MFPH_PC12MFP_Pos        (16)                                        /*!< SYS_T::GPC_MFPH: PC12MFP Position          */
#define SYS_GPC_MFPH_PC12MFP_Msk        (0xFUL << SYS_GPC_MFPH_PC12MFP_Pos)         /*!< SYS_T::GPC_MFPH: PC12MFP Mask              */

#define SYS_GPC_MFPH_PC14MFP_Pos        (24)                                        /*!< SYS_T::GPC_MFPH: PC14MFP Position          */
#define SYS_GPC_MFPH_PC14MFP_Msk        (0xFUL << SYS_GPC_MFPH_PC14MFP_Pos)         /*!< SYS_T::GPC_MFPH: PC14MFP Mask              */

#define SYS_GPD_MFPL_PD0MFP_Pos         (0)                                         /*!< SYS_T::GPD_MFPL: PD0MFP Position           */
#define SYS_GPD_MFPL_PD0MFP_Msk         (0xFUL << SYS_GPD_MFPL_PD0MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD0MFP Mask               */

#define SYS_GPD_MFPL_PD1MFP_Pos         (4)                                         /*!< SYS_T::GPD_MFPL: PD1MFP Position           */
#define SYS_GPD_MFPL_PD1MFP_Msk         (0xFUL << SYS_GPD_MFPL_PD1MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD1MFP Mask               */

#define SYS_GPD_MFPL_PD2MFP_Pos         (8)                                         /*!< SYS_T::GPD_MFPL: PD2MFP Position           */
#define SYS_GPD_MFPL_PD2MFP_Msk         (0xFUL << SYS_GPD_MFPL_PD2MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD2MFP Mask               */

#define SYS_GPD_MFPL_PD3MFP_Pos         (12)                                        /*!< SYS_T::GPD_MFPL: PD3MFP Position           */
#define SYS_GPD_MFPL_PD3MFP_Msk         (0xFUL << SYS_GPD_MFPL_PD3MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD3MFP Mask               */

#define SYS_GPD_MFPL_PD4MFP_Pos         (16)                                        /*!< SYS_T::GPD_MFPL: PD4MFP Position           */
#define SYS_GPD_MFPL_PD4MFP_Msk         (0xFUL << SYS_GPD_MFPL_PD4MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD4MFP Mask               */

#define SYS_GPD_MFPL_PD5MFP_Pos         (20)                                        /*!< SYS_T::GPD_MFPL: PD5MFP Position           */
#define SYS_GPD_MFPL_PD5MFP_Msk         (0xFUL << SYS_GPD_MFPL_PD5MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD5MFP Mask               */

#define SYS_GPD_MFPL_PD6MFP_Pos         (24)                                        /*!< SYS_T::GPD_MFPL: PD6MFP Position           */
#define SYS_GPD_MFPL_PD6MFP_Msk         (0xFUL << SYS_GPD_MFPL_PD6MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD6MFP Mask               */

#define SYS_GPD_MFPL_PD7MFP_Pos         (28)                                        /*!< SYS_T::GPD_MFPL: PD7MFP Position           */
#define SYS_GPD_MFPL_PD7MFP_Msk         (0xFUL << SYS_GPD_MFPL_PD7MFP_Pos)          /*!< SYS_T::GPD_MFPL: PD7MFP Mask               */

#define SYS_GPD_MFPH_PD8MFP_Pos         (0)                                         /*!< SYS_T::GPD_MFPH: PD8MFP Position           */
#define SYS_GPD_MFPH_PD8MFP_Msk         (0xFUL << SYS_GPD_MFPH_PD8MFP_Pos)          /*!< SYS_T::GPD_MFPH: PD8MFP Mask               */

#define SYS_GPD_MFPH_PD9MFP_Pos         (4)                                         /*!< SYS_T::GPD_MFPH: PD9MFP Position           */
#define SYS_GPD_MFPH_PD9MFP_Msk         (0xFUL << SYS_GPD_MFPH_PD9MFP_Pos)          /*!< SYS_T::GPD_MFPH: PD9MFP Mask               */

#define SYS_GPD_MFPH_PD10MFP_Pos        (8)                                         /*!< SYS_T::GPD_MFPH: PD10MFP Position          */
#define SYS_GPD_MFPH_PD10MFP_Msk        (0xFUL << SYS_GPD_MFPH_PD10MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD10MFP Mask              */

#define SYS_GPD_MFPH_PD11MFP_Pos        (12)                                        /*!< SYS_T::GPD_MFPH: PD11MFP Position          */
#define SYS_GPD_MFPH_PD11MFP_Msk        (0xFUL << SYS_GPD_MFPH_PD11MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD11MFP Mask              */

#define SYS_GPD_MFPH_PD12MFP_Pos        (16)                                        /*!< SYS_T::GPD_MFPH: PD12MFP Position          */
#define SYS_GPD_MFPH_PD12MFP_Msk        (0xFUL << SYS_GPD_MFPH_PD12MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD12MFP Mask              */

#define SYS_GPD_MFPH_PD13MFP_Pos        (20)                                        /*!< SYS_T::GPD_MFPH: PD13MFP Position          */
#define SYS_GPD_MFPH_PD13MFP_Msk        (0xFUL << SYS_GPD_MFPH_PD13MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD13MFP Mask              */

#define SYS_GPD_MFPH_PD15MFP_Pos        (28)                                        /*!< SYS_T::GPD_MFPH: PD15MFP Position          */
#define SYS_GPD_MFPH_PD15MFP_Msk        (0xFUL << SYS_GPD_MFPH_PD15MFP_Pos)         /*!< SYS_T::GPD_MFPH: PD15MFP Mask              */

#define SYS_GPE_MFPL_PE0MFP_Pos         (0)                                         /*!< SYS_T::GPE_MFPL: PE0MFP Position           */
#define SYS_GPE_MFPL_PE0MFP_Msk         (0xFUL << SYS_GPE_MFPL_PE0MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE0MFP Mask               */

#define SYS_GPE_MFPL_PE1MFP_Pos         (4)                                         /*!< SYS_T::GPE_MFPL: PE1MFP Position           */
#define SYS_GPE_MFPL_PE1MFP_Msk         (0xFUL << SYS_GPE_MFPL_PE1MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE1MFP Mask               */

#define SYS_GPE_MFPL_PE2MFP_Pos         (8)                                         /*!< SYS_T::GPE_MFPL: PE2MFP Position           */
#define SYS_GPE_MFPL_PE2MFP_Msk         (0xFUL << SYS_GPE_MFPL_PE2MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE2MFP Mask               */

#define SYS_GPE_MFPL_PE3MFP_Pos         (12)                                        /*!< SYS_T::GPE_MFPL: PE3MFP Position           */
#define SYS_GPE_MFPL_PE3MFP_Msk         (0xFUL << SYS_GPE_MFPL_PE3MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE3MFP Mask               */

#define SYS_GPE_MFPL_PE4MFP_Pos         (16)                                        /*!< SYS_T::GPE_MFPL: PE4MFP Position           */
#define SYS_GPE_MFPL_PE4MFP_Msk         (0xFUL << SYS_GPE_MFPL_PE4MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE4MFP Mask               */

#define SYS_GPE_MFPL_PE5MFP_Pos         (20)                                        /*!< SYS_T::GPE_MFPL: PE5MFP Position           */
#define SYS_GPE_MFPL_PE5MFP_Msk         (0xFUL << SYS_GPE_MFPL_PE5MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE5MFP Mask               */

#define SYS_GPE_MFPL_PE6MFP_Pos         (24)                                        /*!< SYS_T::GPE_MFPL: PE6MFP Position           */
#define SYS_GPE_MFPL_PE6MFP_Msk         (0xFUL << SYS_GPE_MFPL_PE6MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE6MFP Mask               */

#define SYS_GPE_MFPL_PE7MFP_Pos         (28)                                        /*!< SYS_T::GPE_MFPL: PE7MFP Position           */
#define SYS_GPE_MFPL_PE7MFP_Msk         (0xFUL << SYS_GPE_MFPL_PE7MFP_Pos)          /*!< SYS_T::GPE_MFPL: PE7MFP Mask               */

#define SYS_GPE_MFPH_PE8MFP_Pos         (0)                                         /*!< SYS_T::GPE_MFPH: PE8MFP Position           */
#define SYS_GPE_MFPH_PE8MFP_Msk         (0xFUL << SYS_GPE_MFPH_PE8MFP_Pos)          /*!< SYS_T::GPE_MFPH: PE8MFP Mask               */

#define SYS_GPE_MFPH_PE9MFP_Pos         (4)                                         /*!< SYS_T::GPE_MFPH: PE9MFP Position           */
#define SYS_GPE_MFPH_PE9MFP_Msk         (0xFUL << SYS_GPE_MFPH_PE9MFP_Pos)          /*!< SYS_T::GPE_MFPH: PE9MFP Mask               */

#define SYS_GPE_MFPH_PE10MFP_Pos        (8)                                         /*!< SYS_T::GPE_MFPH: PE10MFP Position          */
#define SYS_GPE_MFPH_PE10MFP_Msk        (0xFUL << SYS_GPE_MFPH_PE10MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE10MFP Mask              */

#define SYS_GPE_MFPH_PE11MFP_Pos        (12)                                        /*!< SYS_T::GPE_MFPH: PE11MFP Position          */
#define SYS_GPE_MFPH_PE11MFP_Msk        (0xFUL << SYS_GPE_MFPH_PE11MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE11MFP Mask              */

#define SYS_GPE_MFPH_PE12MFP_Pos        (16)                                        /*!< SYS_T::GPE_MFPH: PE12MFP Position          */
#define SYS_GPE_MFPH_PE12MFP_Msk        (0xFUL << SYS_GPE_MFPH_PE12MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE12MFP Mask              */

#define SYS_GPE_MFPH_PE13MFP_Pos        (20)                                        /*!< SYS_T::GPE_MFPH: PE13MFP Position          */
#define SYS_GPE_MFPH_PE13MFP_Msk        (0xFUL << SYS_GPE_MFPH_PE13MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE13MFP Mask              */

#define SYS_GPE_MFPH_PE14MFP_Pos        (24)                                        /*!< SYS_T::GPE_MFPH: PE14MFP Position          */
#define SYS_GPE_MFPH_PE14MFP_Msk        (0xFUL << SYS_GPE_MFPH_PE14MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE14MFP Mask              */

#define SYS_GPE_MFPH_PE15MFP_Pos        (28)                                        /*!< SYS_T::GPE_MFPH: PE15MFP Position          */
#define SYS_GPE_MFPH_PE15MFP_Msk        (0xFUL << SYS_GPE_MFPH_PE15MFP_Pos)         /*!< SYS_T::GPE_MFPH: PE15MFP Mask              */

#define SYS_GPF_MFPL_PF0MFP_Pos         (0)                                         /*!< SYS_T::GPF_MFPL: PF0MFP Position           */
#define SYS_GPF_MFPL_PF0MFP_Msk         (0xFUL << SYS_GPF_MFPL_PF0MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF0MFP Mask               */

#define SYS_GPF_MFPL_PF1MFP_Pos         (4)                                         /*!< SYS_T::GPF_MFPL: PF1MFP Position           */
#define SYS_GPF_MFPL_PF1MFP_Msk         (0xFUL << SYS_GPF_MFPL_PF1MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF1MFP Mask               */

#define SYS_GPF_MFPL_PF2MFP_Pos         (8)                                         /*!< SYS_T::GPF_MFPL: PF2MFP Position           */
#define SYS_GPF_MFPL_PF2MFP_Msk         (0xFUL << SYS_GPF_MFPL_PF2MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF2MFP Mask               */

#define SYS_GPF_MFPL_PF3MFP_Pos         (12)                                        /*!< SYS_T::GPF_MFPL: PF3MFP Position           */
#define SYS_GPF_MFPL_PF3MFP_Msk         (0xFUL << SYS_GPF_MFPL_PF3MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF3MFP Mask               */

#define SYS_GPF_MFPL_PF4MFP_Pos         (16)                                        /*!< SYS_T::GPF_MFPL: PF4MFP Position           */
#define SYS_GPF_MFPL_PF4MFP_Msk         (0xFUL << SYS_GPF_MFPL_PF4MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF4MFP Mask               */

#define SYS_GPF_MFPL_PF5MFP_Pos         (20)                                        /*!< SYS_T::GPF_MFPL: PF5MFP Position           */
#define SYS_GPF_MFPL_PF5MFP_Msk         (0xFUL << SYS_GPF_MFPL_PF5MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF5MFP Mask               */

#define SYS_GPF_MFPL_PF6MFP_Pos         (24)                                        /*!< SYS_T::GPF_MFPL: PF6MFP Position           */
#define SYS_GPF_MFPL_PF6MFP_Msk         (0xFUL << SYS_GPF_MFPL_PF6MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF6MFP Mask               */

#define SYS_GPF_MFPL_PF7MFP_Pos         (28)                                        /*!< SYS_T::GPF_MFPL: PF7MFP Position           */
#define SYS_GPF_MFPL_PF7MFP_Msk         (0xFUL << SYS_GPF_MFPL_PF7MFP_Pos)          /*!< SYS_T::GPF_MFPL: PF7MFP Mask               */

#define SYS_GPF_MFPH_PF14MFP_Pos        (24)                                        /*!< SYS_T::GPF_MFPH: PF14MFP Position          */
#define SYS_GPF_MFPH_PF14MFP_Msk        (0xFUL << SYS_GPF_MFPH_PF14MFP_Pos)         /*!< SYS_T::GPF_MFPH: PF14MFP Mask              */

#define SYS_GPF_MFPH_PF15MFP_Pos        (28)                                        /*!< SYS_T::GPF_MFPH: PF15MFP Position          */
#define SYS_GPF_MFPH_PF15MFP_Msk        (0xFUL << SYS_GPF_MFPH_PF15MFP_Pos)         /*!< SYS_T::GPF_MFPH: PF15MFP Mask              */

#define SYS_GPG_MFPH_PG10MFP_Pos        (8)                                         /*!< SYS_T::GPG_MFPH: PG10MFP Position          */
#define SYS_GPG_MFPH_PG10MFP_Msk        (0xFUL << SYS_GPG_MFPH_PG10MFP_Pos)         /*!< SYS_T::GPG_MFPH: PG10MFP Mask              */

#define SYS_GPG_MFPH_PG11MFP_Pos        (12)                                        /*!< SYS_T::GPG_MFPH: PG11MFP Position          */
#define SYS_GPG_MFPH_PG11MFP_Msk        (0xFUL << SYS_GPG_MFPH_PG11MFP_Pos)         /*!< SYS_T::GPG_MFPH: PG11MFP Mask              */

#define SYS_GPG_MFPH_PG12MFP_Pos        (16)                                        /*!< SYS_T::GPG_MFPH: PG12MFP Position          */
#define SYS_GPG_MFPH_PG12MFP_Msk        (0xFUL << SYS_GPG_MFPH_PG12MFP_Pos)         /*!< SYS_T::GPG_MFPH: PG12MFP Mask              */

#define SYS_GPA_MFOS_MFOS0_Pos          (0)                                         /*!< SYS_T::GPA_MFOS: MFOS0 Position            */
#define SYS_GPA_MFOS_MFOS0_Msk          (0x1UL << SYS_GPA_MFOS_MFOS0_Pos)           /*!< SYS_T::GPA_MFOS: MFOS0 Mask                */

#define SYS_GPA_MFOS_MFOS1_Pos          (1)                                         /*!< SYS_T::GPA_MFOS: MFOS1 Position            */
#define SYS_GPA_MFOS_MFOS1_Msk          (0x1UL << SYS_GPA_MFOS_MFOS1_Pos)           /*!< SYS_T::GPA_MFOS: MFOS1 Mask                */

#define SYS_GPA_MFOS_MFOS2_Pos          (2)                                         /*!< SYS_T::GPA_MFOS: MFOS2 Position            */
#define SYS_GPA_MFOS_MFOS2_Msk          (0x1UL << SYS_GPA_MFOS_MFOS2_Pos)           /*!< SYS_T::GPA_MFOS: MFOS2 Mask                */

#define SYS_GPA_MFOS_MFOS3_Pos          (3)                                         /*!< SYS_T::GPA_MFOS: MFOS3 Position            */
#define SYS_GPA_MFOS_MFOS3_Msk          (0x1UL << SYS_GPA_MFOS_MFOS3_Pos)           /*!< SYS_T::GPA_MFOS: MFOS3 Mask                */

#define SYS_GPA_MFOS_MFOS4_Pos          (4)                                         /*!< SYS_T::GPA_MFOS: MFOS4 Position            */
#define SYS_GPA_MFOS_MFOS4_Msk          (0x1UL << SYS_GPA_MFOS_MFOS4_Pos)           /*!< SYS_T::GPA_MFOS: MFOS4 Mask                */

#define SYS_GPA_MFOS_MFOS5_Pos          (5)                                         /*!< SYS_T::GPA_MFOS: MFOS5 Position            */
#define SYS_GPA_MFOS_MFOS5_Msk          (0x1UL << SYS_GPA_MFOS_MFOS5_Pos)           /*!< SYS_T::GPA_MFOS: MFOS5 Mask                */

#define SYS_GPA_MFOS_MFOS6_Pos          (6)                                         /*!< SYS_T::GPA_MFOS: MFOS6 Position            */
#define SYS_GPA_MFOS_MFOS6_Msk          (0x1UL << SYS_GPA_MFOS_MFOS6_Pos)           /*!< SYS_T::GPA_MFOS: MFOS6 Mask                */

#define SYS_GPA_MFOS_MFOS7_Pos          (7)                                         /*!< SYS_T::GPA_MFOS: MFOS7 Position            */
#define SYS_GPA_MFOS_MFOS7_Msk          (0x1UL << SYS_GPA_MFOS_MFOS7_Pos)           /*!< SYS_T::GPA_MFOS: MFOS7 Mask                */

#define SYS_GPA_MFOS_MFOS8_Pos          (8)                                         /*!< SYS_T::GPA_MFOS: MFOS8 Position            */
#define SYS_GPA_MFOS_MFOS8_Msk          (0x1UL << SYS_GPA_MFOS_MFOS8_Pos)           /*!< SYS_T::GPA_MFOS: MFOS8 Mask                */

#define SYS_GPA_MFOS_MFOS9_Pos          (9)                                         /*!< SYS_T::GPA_MFOS: MFOS9 Position            */
#define SYS_GPA_MFOS_MFOS9_Msk          (0x1UL << SYS_GPA_MFOS_MFOS9_Pos)           /*!< SYS_T::GPA_MFOS: MFOS9 Mask                */

#define SYS_GPA_MFOS_MFOS10_Pos         (10)                                        /*!< SYS_T::GPA_MFOS: MFOS10 Position           */
#define SYS_GPA_MFOS_MFOS10_Msk         (0x1UL << SYS_GPA_MFOS_MFOS10_Pos)          /*!< SYS_T::GPA_MFOS: MFOS10 Mask               */

#define SYS_GPA_MFOS_MFOS11_Pos         (11)                                        /*!< SYS_T::GPA_MFOS: MFOS11 Position           */
#define SYS_GPA_MFOS_MFOS11_Msk         (0x1UL << SYS_GPA_MFOS_MFOS11_Pos)          /*!< SYS_T::GPA_MFOS: MFOS11 Mask               */

#define SYS_GPA_MFOS_MFOS12_Pos         (12)                                        /*!< SYS_T::GPA_MFOS: MFOS12 Position           */
#define SYS_GPA_MFOS_MFOS12_Msk         (0x1UL << SYS_GPA_MFOS_MFOS12_Pos)          /*!< SYS_T::GPA_MFOS: MFOS12 Mask               */

#define SYS_GPA_MFOS_MFOS13_Pos         (13)                                        /*!< SYS_T::GPA_MFOS: MFOS13 Position           */
#define SYS_GPA_MFOS_MFOS13_Msk         (0x1UL << SYS_GPA_MFOS_MFOS13_Pos)          /*!< SYS_T::GPA_MFOS: MFOS13 Mask               */

#define SYS_GPA_MFOS_MFOS14_Pos         (14)                                        /*!< SYS_T::GPA_MFOS: MFOS14 Position           */
#define SYS_GPA_MFOS_MFOS14_Msk         (0x1UL << SYS_GPA_MFOS_MFOS14_Pos)          /*!< SYS_T::GPA_MFOS: MFOS14 Mask               */

#define SYS_GPA_MFOS_MFOS15_Pos         (15)                                        /*!< SYS_T::GPA_MFOS: MFOS15 Position           */
#define SYS_GPA_MFOS_MFOS15_Msk         (0x1UL << SYS_GPA_MFOS_MFOS15_Pos)          /*!< SYS_T::GPA_MFOS: MFOS15 Mask               */

#define SYS_GPB_MFOS_MFOS0_Pos          (0)                                         /*!< SYS_T::GPB_MFOS: MFOS0 Position            */
#define SYS_GPB_MFOS_MFOS0_Msk          (0x1UL << SYS_GPB_MFOS_MFOS0_Pos)           /*!< SYS_T::GPB_MFOS: MFOS0 Mask                */

#define SYS_GPB_MFOS_MFOS1_Pos          (1)                                         /*!< SYS_T::GPB_MFOS: MFOS1 Position            */
#define SYS_GPB_MFOS_MFOS1_Msk          (0x1UL << SYS_GPB_MFOS_MFOS1_Pos)           /*!< SYS_T::GPB_MFOS: MFOS1 Mask                */

#define SYS_GPB_MFOS_MFOS2_Pos          (2)                                         /*!< SYS_T::GPB_MFOS: MFOS2 Position            */
#define SYS_GPB_MFOS_MFOS2_Msk          (0x1UL << SYS_GPB_MFOS_MFOS2_Pos)           /*!< SYS_T::GPB_MFOS: MFOS2 Mask                */

#define SYS_GPB_MFOS_MFOS3_Pos          (3)                                         /*!< SYS_T::GPB_MFOS: MFOS3 Position            */
#define SYS_GPB_MFOS_MFOS3_Msk          (0x1UL << SYS_GPB_MFOS_MFOS3_Pos)           /*!< SYS_T::GPB_MFOS: MFOS3 Mask                */

#define SYS_GPB_MFOS_MFOS4_Pos          (4)                                         /*!< SYS_T::GPB_MFOS: MFOS4 Position            */
#define SYS_GPB_MFOS_MFOS4_Msk          (0x1UL << SYS_GPB_MFOS_MFOS4_Pos)           /*!< SYS_T::GPB_MFOS: MFOS4 Mask                */

#define SYS_GPB_MFOS_MFOS5_Pos          (5)                                         /*!< SYS_T::GPB_MFOS: MFOS5 Position            */
#define SYS_GPB_MFOS_MFOS5_Msk          (0x1UL << SYS_GPB_MFOS_MFOS5_Pos)           /*!< SYS_T::GPB_MFOS: MFOS5 Mask                */

#define SYS_GPB_MFOS_MFOS6_Pos          (6)                                         /*!< SYS_T::GPB_MFOS: MFOS6 Position            */
#define SYS_GPB_MFOS_MFOS6_Msk          (0x1UL << SYS_GPB_MFOS_MFOS6_Pos)           /*!< SYS_T::GPB_MFOS: MFOS6 Mask                */

#define SYS_GPB_MFOS_MFOS7_Pos          (7)                                         /*!< SYS_T::GPB_MFOS: MFOS7 Position            */
#define SYS_GPB_MFOS_MFOS7_Msk          (0x1UL << SYS_GPB_MFOS_MFOS7_Pos)           /*!< SYS_T::GPB_MFOS: MFOS7 Mask                */

#define SYS_GPB_MFOS_MFOS8_Pos          (8)                                         /*!< SYS_T::GPB_MFOS: MFOS8 Position            */
#define SYS_GPB_MFOS_MFOS8_Msk          (0x1UL << SYS_GPB_MFOS_MFOS8_Pos)           /*!< SYS_T::GPB_MFOS: MFOS8 Mask                */

#define SYS_GPB_MFOS_MFOS9_Pos          (9)                                         /*!< SYS_T::GPB_MFOS: MFOS9 Position            */
#define SYS_GPB_MFOS_MFOS9_Msk          (0x1UL << SYS_GPB_MFOS_MFOS9_Pos)           /*!< SYS_T::GPB_MFOS: MFOS9 Mask                */

#define SYS_GPB_MFOS_MFOS10_Pos         (10)                                        /*!< SYS_T::GPB_MFOS: MFOS10 Position           */
#define SYS_GPB_MFOS_MFOS10_Msk         (0x1UL << SYS_GPB_MFOS_MFOS10_Pos)          /*!< SYS_T::GPB_MFOS: MFOS10 Mask               */

#define SYS_GPB_MFOS_MFOS11_Pos         (11)                                        /*!< SYS_T::GPB_MFOS: MFOS11 Position           */
#define SYS_GPB_MFOS_MFOS11_Msk         (0x1UL << SYS_GPB_MFOS_MFOS11_Pos)          /*!< SYS_T::GPB_MFOS: MFOS11 Mask               */

#define SYS_GPB_MFOS_MFOS12_Pos         (12)                                        /*!< SYS_T::GPB_MFOS: MFOS12 Position           */
#define SYS_GPB_MFOS_MFOS12_Msk         (0x1UL << SYS_GPB_MFOS_MFOS12_Pos)          /*!< SYS_T::GPB_MFOS: MFOS12 Mask               */

#define SYS_GPB_MFOS_MFOS13_Pos         (13)                                        /*!< SYS_T::GPB_MFOS: MFOS13 Position           */
#define SYS_GPB_MFOS_MFOS13_Msk         (0x1UL << SYS_GPB_MFOS_MFOS13_Pos)          /*!< SYS_T::GPB_MFOS: MFOS13 Mask               */

#define SYS_GPB_MFOS_MFOS14_Pos         (14)                                        /*!< SYS_T::GPB_MFOS: MFOS14 Position           */
#define SYS_GPB_MFOS_MFOS14_Msk         (0x1UL << SYS_GPB_MFOS_MFOS14_Pos)          /*!< SYS_T::GPB_MFOS: MFOS14 Mask               */

#define SYS_GPB_MFOS_MFOS15_Pos         (15)                                        /*!< SYS_T::GPB_MFOS: MFOS15 Position           */
#define SYS_GPB_MFOS_MFOS15_Msk         (0x1UL << SYS_GPB_MFOS_MFOS15_Pos)          /*!< SYS_T::GPB_MFOS: MFOS15 Mask               */

#define SYS_GPC_MFOS_MFOS0_Pos          (0)                                         /*!< SYS_T::GPC_MFOS: MFOS0 Position            */
#define SYS_GPC_MFOS_MFOS0_Msk          (0x1UL << SYS_GPC_MFOS_MFOS0_Pos)           /*!< SYS_T::GPC_MFOS: MFOS0 Mask                */

#define SYS_GPC_MFOS_MFOS1_Pos          (1)                                         /*!< SYS_T::GPC_MFOS: MFOS1 Position            */
#define SYS_GPC_MFOS_MFOS1_Msk          (0x1UL << SYS_GPC_MFOS_MFOS1_Pos)           /*!< SYS_T::GPC_MFOS: MFOS1 Mask                */

#define SYS_GPC_MFOS_MFOS2_Pos          (2)                                         /*!< SYS_T::GPC_MFOS: MFOS2 Position            */
#define SYS_GPC_MFOS_MFOS2_Msk          (0x1UL << SYS_GPC_MFOS_MFOS2_Pos)           /*!< SYS_T::GPC_MFOS: MFOS2 Mask                */

#define SYS_GPC_MFOS_MFOS3_Pos          (3)                                         /*!< SYS_T::GPC_MFOS: MFOS3 Position            */
#define SYS_GPC_MFOS_MFOS3_Msk          (0x1UL << SYS_GPC_MFOS_MFOS3_Pos)           /*!< SYS_T::GPC_MFOS: MFOS3 Mask                */

#define SYS_GPC_MFOS_MFOS4_Pos          (4)                                         /*!< SYS_T::GPC_MFOS: MFOS4 Position            */
#define SYS_GPC_MFOS_MFOS4_Msk          (0x1UL << SYS_GPC_MFOS_MFOS4_Pos)           /*!< SYS_T::GPC_MFOS: MFOS4 Mask                */

#define SYS_GPC_MFOS_MFOS5_Pos          (5)                                         /*!< SYS_T::GPC_MFOS: MFOS5 Position            */
#define SYS_GPC_MFOS_MFOS5_Msk          (0x1UL << SYS_GPC_MFOS_MFOS5_Pos)           /*!< SYS_T::GPC_MFOS: MFOS5 Mask                */

#define SYS_GPC_MFOS_MFOS6_Pos          (6)                                         /*!< SYS_T::GPC_MFOS: MFOS6 Position            */
#define SYS_GPC_MFOS_MFOS6_Msk          (0x1UL << SYS_GPC_MFOS_MFOS6_Pos)           /*!< SYS_T::GPC_MFOS: MFOS6 Mask                */

#define SYS_GPC_MFOS_MFOS7_Pos          (7)                                         /*!< SYS_T::GPC_MFOS: MFOS7 Position            */
#define SYS_GPC_MFOS_MFOS7_Msk          (0x1UL << SYS_GPC_MFOS_MFOS7_Pos)           /*!< SYS_T::GPC_MFOS: MFOS7 Mask                */

#define SYS_GPC_MFOS_MFOS8_Pos          (8)                                         /*!< SYS_T::GPC_MFOS: MFOS8 Position            */
#define SYS_GPC_MFOS_MFOS8_Msk          (0x1UL << SYS_GPC_MFOS_MFOS8_Pos)           /*!< SYS_T::GPC_MFOS: MFOS8 Mask                */

#define SYS_GPC_MFOS_MFOS9_Pos          (9)                                         /*!< SYS_T::GPC_MFOS: MFOS9 Position            */
#define SYS_GPC_MFOS_MFOS9_Msk          (0x1UL << SYS_GPC_MFOS_MFOS9_Pos)           /*!< SYS_T::GPC_MFOS: MFOS9 Mask                */

#define SYS_GPC_MFOS_MFOS10_Pos         (10)                                        /*!< SYS_T::GPC_MFOS: MFOS10 Position           */
#define SYS_GPC_MFOS_MFOS10_Msk         (0x1UL << SYS_GPC_MFOS_MFOS10_Pos)          /*!< SYS_T::GPC_MFOS: MFOS10 Mask               */

#define SYS_GPC_MFOS_MFOS11_Pos         (11)                                        /*!< SYS_T::GPC_MFOS: MFOS11 Position           */
#define SYS_GPC_MFOS_MFOS11_Msk         (0x1UL << SYS_GPC_MFOS_MFOS11_Pos)          /*!< SYS_T::GPC_MFOS: MFOS11 Mask               */

#define SYS_GPC_MFOS_MFOS12_Pos         (12)                                        /*!< SYS_T::GPC_MFOS: MFOS12 Position           */
#define SYS_GPC_MFOS_MFOS12_Msk         (0x1UL << SYS_GPC_MFOS_MFOS12_Pos)          /*!< SYS_T::GPC_MFOS: MFOS12 Mask               */

#define SYS_GPC_MFOS_MFOS14_Pos         (14)                                        /*!< SYS_T::GPC_MFOS: MFOS14 Position           */
#define SYS_GPC_MFOS_MFOS14_Msk         (0x1UL << SYS_GPC_MFOS_MFOS14_Pos)          /*!< SYS_T::GPC_MFOS: MFOS14 Mask               */

#define SYS_GPD_MFOS_MFOS0_Pos          (0)                                         /*!< SYS_T::GPD_MFOS: MFOS0 Position            */
#define SYS_GPD_MFOS_MFOS0_Msk          (0x1UL << SYS_GPD_MFOS_MFOS0_Pos)           /*!< SYS_T::GPD_MFOS: MFOS0 Mask                */

#define SYS_GPD_MFOS_MFOS1_Pos          (1)                                         /*!< SYS_T::GPD_MFOS: MFOS1 Position            */
#define SYS_GPD_MFOS_MFOS1_Msk          (0x1UL << SYS_GPD_MFOS_MFOS1_Pos)           /*!< SYS_T::GPD_MFOS: MFOS1 Mask                */

#define SYS_GPD_MFOS_MFOS2_Pos          (2)                                         /*!< SYS_T::GPD_MFOS: MFOS2 Position            */
#define SYS_GPD_MFOS_MFOS2_Msk          (0x1UL << SYS_GPD_MFOS_MFOS2_Pos)           /*!< SYS_T::GPD_MFOS: MFOS2 Mask                */

#define SYS_GPD_MFOS_MFOS3_Pos          (3)                                         /*!< SYS_T::GPD_MFOS: MFOS3 Position            */
#define SYS_GPD_MFOS_MFOS3_Msk          (0x1UL << SYS_GPD_MFOS_MFOS3_Pos)           /*!< SYS_T::GPD_MFOS: MFOS3 Mask                */

#define SYS_GPD_MFOS_MFOS4_Pos          (4)                                         /*!< SYS_T::GPD_MFOS: MFOS4 Position            */
#define SYS_GPD_MFOS_MFOS4_Msk          (0x1UL << SYS_GPD_MFOS_MFOS4_Pos)           /*!< SYS_T::GPD_MFOS: MFOS4 Mask                */

#define SYS_GPD_MFOS_MFOS5_Pos          (5)                                         /*!< SYS_T::GPD_MFOS: MFOS5 Position            */
#define SYS_GPD_MFOS_MFOS5_Msk          (0x1UL << SYS_GPD_MFOS_MFOS5_Pos)           /*!< SYS_T::GPD_MFOS: MFOS5 Mask                */

#define SYS_GPD_MFOS_MFOS6_Pos          (6)                                         /*!< SYS_T::GPD_MFOS: MFOS6 Position            */
#define SYS_GPD_MFOS_MFOS6_Msk          (0x1UL << SYS_GPD_MFOS_MFOS6_Pos)           /*!< SYS_T::GPD_MFOS: MFOS6 Mask                */

#define SYS_GPD_MFOS_MFOS7_Pos          (7)                                         /*!< SYS_T::GPD_MFOS: MFOS7 Position            */
#define SYS_GPD_MFOS_MFOS7_Msk          (0x1UL << SYS_GPD_MFOS_MFOS7_Pos)           /*!< SYS_T::GPD_MFOS: MFOS7 Mask                */

#define SYS_GPD_MFOS_MFOS8_Pos          (8)                                         /*!< SYS_T::GPD_MFOS: MFOS8 Position            */
#define SYS_GPD_MFOS_MFOS8_Msk          (0x1UL << SYS_GPD_MFOS_MFOS8_Pos)           /*!< SYS_T::GPD_MFOS: MFOS8 Mask                */

#define SYS_GPD_MFOS_MFOS9_Pos          (9)                                         /*!< SYS_T::GPD_MFOS: MFOS9 Position            */
#define SYS_GPD_MFOS_MFOS9_Msk          (0x1UL << SYS_GPD_MFOS_MFOS9_Pos)           /*!< SYS_T::GPD_MFOS: MFOS9 Mask                */

#define SYS_GPD_MFOS_MFOS10_Pos         (10)                                        /*!< SYS_T::GPD_MFOS: MFOS10 Position           */
#define SYS_GPD_MFOS_MFOS10_Msk         (0x1UL << SYS_GPD_MFOS_MFOS10_Pos)          /*!< SYS_T::GPD_MFOS: MFOS10 Mask               */

#define SYS_GPD_MFOS_MFOS11_Pos         (11)                                        /*!< SYS_T::GPD_MFOS: MFOS11 Position           */
#define SYS_GPD_MFOS_MFOS11_Msk         (0x1UL << SYS_GPD_MFOS_MFOS11_Pos)          /*!< SYS_T::GPD_MFOS: MFOS11 Mask               */

#define SYS_GPD_MFOS_MFOS12_Pos         (12)                                        /*!< SYS_T::GPD_MFOS: MFOS12 Position           */
#define SYS_GPD_MFOS_MFOS12_Msk         (0x1UL << SYS_GPD_MFOS_MFOS12_Pos)          /*!< SYS_T::GPD_MFOS: MFOS12 Mask               */

#define SYS_GPD_MFOS_MFOS13_Pos         (13)                                        /*!< SYS_T::GPD_MFOS: MFOS13 Position           */
#define SYS_GPD_MFOS_MFOS13_Msk         (0x1UL << SYS_GPD_MFOS_MFOS13_Pos)          /*!< SYS_T::GPD_MFOS: MFOS13 Mask               */

#define SYS_GPD_MFOS_MFOS15_Pos         (15)                                        /*!< SYS_T::GPD_MFOS: MFOS15 Position           */
#define SYS_GPD_MFOS_MFOS15_Msk         (0x1UL << SYS_GPD_MFOS_MFOS15_Pos)          /*!< SYS_T::GPD_MFOS: MFOS15 Mask               */

#define SYS_GPE_MFOS_MFOS0_Pos          (0)                                         /*!< SYS_T::GPE_MFOS: MFOS0 Position            */
#define SYS_GPE_MFOS_MFOS0_Msk          (0x1UL << SYS_GPE_MFOS_MFOS0_Pos)           /*!< SYS_T::GPE_MFOS: MFOS0 Mask                */

#define SYS_GPE_MFOS_MFOS1_Pos          (1)                                         /*!< SYS_T::GPE_MFOS: MFOS1 Position            */
#define SYS_GPE_MFOS_MFOS1_Msk          (0x1UL << SYS_GPE_MFOS_MFOS1_Pos)           /*!< SYS_T::GPE_MFOS: MFOS1 Mask                */

#define SYS_GPE_MFOS_MFOS2_Pos          (2)                                         /*!< SYS_T::GPE_MFOS: MFOS2 Position            */
#define SYS_GPE_MFOS_MFOS2_Msk          (0x1UL << SYS_GPE_MFOS_MFOS2_Pos)           /*!< SYS_T::GPE_MFOS: MFOS2 Mask                */

#define SYS_GPE_MFOS_MFOS3_Pos          (3)                                         /*!< SYS_T::GPE_MFOS: MFOS3 Position            */
#define SYS_GPE_MFOS_MFOS3_Msk          (0x1UL << SYS_GPE_MFOS_MFOS3_Pos)           /*!< SYS_T::GPE_MFOS: MFOS3 Mask                */

#define SYS_GPE_MFOS_MFOS4_Pos          (4)                                         /*!< SYS_T::GPE_MFOS: MFOS4 Position            */
#define SYS_GPE_MFOS_MFOS4_Msk          (0x1UL << SYS_GPE_MFOS_MFOS4_Pos)           /*!< SYS_T::GPE_MFOS: MFOS4 Mask                */

#define SYS_GPE_MFOS_MFOS5_Pos          (5)                                         /*!< SYS_T::GPE_MFOS: MFOS5 Position            */
#define SYS_GPE_MFOS_MFOS5_Msk          (0x1UL << SYS_GPE_MFOS_MFOS5_Pos)           /*!< SYS_T::GPE_MFOS: MFOS5 Mask                */

#define SYS_GPE_MFOS_MFOS6_Pos          (6)                                         /*!< SYS_T::GPE_MFOS: MFOS6 Position            */
#define SYS_GPE_MFOS_MFOS6_Msk          (0x1UL << SYS_GPE_MFOS_MFOS6_Pos)           /*!< SYS_T::GPE_MFOS: MFOS6 Mask                */

#define SYS_GPE_MFOS_MFOS7_Pos          (7)                                         /*!< SYS_T::GPE_MFOS: MFOS7 Position            */
#define SYS_GPE_MFOS_MFOS7_Msk          (0x1UL << SYS_GPE_MFOS_MFOS7_Pos)           /*!< SYS_T::GPE_MFOS: MFOS7 Mask                */

#define SYS_GPE_MFOS_MFOS8_Pos          (8)                                         /*!< SYS_T::GPE_MFOS: MFOS8 Position            */
#define SYS_GPE_MFOS_MFOS8_Msk          (0x1UL << SYS_GPE_MFOS_MFOS8_Pos)           /*!< SYS_T::GPE_MFOS: MFOS8 Mask                */

#define SYS_GPE_MFOS_MFOS9_Pos          (9)                                         /*!< SYS_T::GPE_MFOS: MFOS9 Position            */
#define SYS_GPE_MFOS_MFOS9_Msk          (0x1UL << SYS_GPE_MFOS_MFOS9_Pos)           /*!< SYS_T::GPE_MFOS: MFOS9 Mask                */

#define SYS_GPE_MFOS_MFOS10_Pos         (10)                                        /*!< SYS_T::GPE_MFOS: MFOS10 Position           */
#define SYS_GPE_MFOS_MFOS10_Msk         (0x1UL << SYS_GPE_MFOS_MFOS10_Pos)          /*!< SYS_T::GPE_MFOS: MFOS10 Mask               */

#define SYS_GPE_MFOS_MFOS11_Pos         (11)                                        /*!< SYS_T::GPE_MFOS: MFOS11 Position           */
#define SYS_GPE_MFOS_MFOS11_Msk         (0x1UL << SYS_GPE_MFOS_MFOS11_Pos)          /*!< SYS_T::GPE_MFOS: MFOS11 Mask               */

#define SYS_GPE_MFOS_MFOS12_Pos         (12)                                        /*!< SYS_T::GPE_MFOS: MFOS12 Position           */
#define SYS_GPE_MFOS_MFOS12_Msk         (0x1UL << SYS_GPE_MFOS_MFOS12_Pos)          /*!< SYS_T::GPE_MFOS: MFOS12 Mask               */

#define SYS_GPE_MFOS_MFOS13_Pos         (13)                                        /*!< SYS_T::GPE_MFOS: MFOS13 Position           */
#define SYS_GPE_MFOS_MFOS13_Msk         (0x1UL << SYS_GPE_MFOS_MFOS13_Pos)          /*!< SYS_T::GPE_MFOS: MFOS13 Mask               */

#define SYS_GPE_MFOS_MFOS14_Pos         (14)                                        /*!< SYS_T::GPE_MFOS: MFOS14 Position           */
#define SYS_GPE_MFOS_MFOS14_Msk         (0x1UL << SYS_GPE_MFOS_MFOS14_Pos)          /*!< SYS_T::GPE_MFOS: MFOS14 Mask               */

#define SYS_GPE_MFOS_MFOS15_Pos         (15)                                        /*!< SYS_T::GPE_MFOS: MFOS15 Position           */
#define SYS_GPE_MFOS_MFOS15_Msk         (0x1UL << SYS_GPE_MFOS_MFOS15_Pos)          /*!< SYS_T::GPE_MFOS: MFOS15 Mask               */

#define SYS_GPF_MFOS_MFOS0_Pos          (0)                                         /*!< SYS_T::GPF_MFOS: MFOS0 Position            */
#define SYS_GPF_MFOS_MFOS0_Msk          (0x1UL << SYS_GPF_MFOS_MFOS0_Pos)           /*!< SYS_T::GPF_MFOS: MFOS0 Mask                */

#define SYS_GPF_MFOS_MFOS1_Pos          (1)                                         /*!< SYS_T::GPF_MFOS: MFOS1 Position            */
#define SYS_GPF_MFOS_MFOS1_Msk          (0x1UL << SYS_GPF_MFOS_MFOS1_Pos)           /*!< SYS_T::GPF_MFOS: MFOS1 Mask                */

#define SYS_GPF_MFOS_MFOS2_Pos          (2)                                         /*!< SYS_T::GPF_MFOS: MFOS2 Position            */
#define SYS_GPF_MFOS_MFOS2_Msk          (0x1UL << SYS_GPF_MFOS_MFOS2_Pos)           /*!< SYS_T::GPF_MFOS: MFOS2 Mask                */

#define SYS_GPF_MFOS_MFOS3_Pos          (3)                                         /*!< SYS_T::GPF_MFOS: MFOS3 Position            */
#define SYS_GPF_MFOS_MFOS3_Msk          (0x1UL << SYS_GPF_MFOS_MFOS3_Pos)           /*!< SYS_T::GPF_MFOS: MFOS3 Mask                */

#define SYS_GPF_MFOS_MFOS4_Pos          (4)                                         /*!< SYS_T::GPF_MFOS: MFOS4 Position            */
#define SYS_GPF_MFOS_MFOS4_Msk          (0x1UL << SYS_GPF_MFOS_MFOS4_Pos)           /*!< SYS_T::GPF_MFOS: MFOS4 Mask                */

#define SYS_GPF_MFOS_MFOS5_Pos          (5)                                         /*!< SYS_T::GPF_MFOS: MFOS5 Position            */
#define SYS_GPF_MFOS_MFOS5_Msk          (0x1UL << SYS_GPF_MFOS_MFOS5_Pos)           /*!< SYS_T::GPF_MFOS: MFOS5 Mask                */

#define SYS_GPF_MFOS_MFOS6_Pos          (6)                                         /*!< SYS_T::GPF_MFOS: MFOS6 Position            */
#define SYS_GPF_MFOS_MFOS6_Msk          (0x1UL << SYS_GPF_MFOS_MFOS6_Pos)           /*!< SYS_T::GPF_MFOS: MFOS6 Mask                */

#define SYS_GPF_MFOS_MFOS7_Pos          (7)                                         /*!< SYS_T::GPF_MFOS: MFOS7 Position            */
#define SYS_GPF_MFOS_MFOS7_Msk          (0x1UL << SYS_GPF_MFOS_MFOS7_Pos)           /*!< SYS_T::GPF_MFOS: MFOS7 Mask                */

#define SYS_GPF_MFOS_MFOS14_Pos         (14)                                        /*!< SYS_T::GPF_MFOS: MFOS14 Position           */
#define SYS_GPF_MFOS_MFOS14_Msk         (0x1UL << SYS_GPF_MFOS_MFOS14_Pos)          /*!< SYS_T::GPF_MFOS: MFOS14 Mask               */

#define SYS_GPF_MFOS_MFOS15_Pos         (15)                                        /*!< SYS_T::GPF_MFOS: MFOS15 Position           */
#define SYS_GPF_MFOS_MFOS15_Msk         (0x1UL << SYS_GPF_MFOS_MFOS15_Pos)          /*!< SYS_T::GPF_MFOS: MFOS15 Mask               */

#define SYS_GPG_MFOS_MFOS10_Pos         (10)                                        /*!< SYS_T::GPG_MFOS: MFOS10 Position           */
#define SYS_GPG_MFOS_MFOS10_Msk         (0x1UL << SYS_GPG_MFOS_MFOS10_Pos)          /*!< SYS_T::GPG_MFOS: MFOS10 Mask               */

#define SYS_GPG_MFOS_MFOS11_Pos         (11)                                        /*!< SYS_T::GPG_MFOS: MFOS11 Position           */
#define SYS_GPG_MFOS_MFOS11_Msk         (0x1UL << SYS_GPG_MFOS_MFOS11_Pos)          /*!< SYS_T::GPG_MFOS: MFOS11 Mask               */

#define SYS_GPG_MFOS_MFOS12_Pos         (12)                                        /*!< SYS_T::GPG_MFOS: MFOS12 Position           */
#define SYS_GPG_MFOS_MFOS12_Msk         (0x1UL << SYS_GPG_MFOS_MFOS12_Pos)          /*!< SYS_T::GPG_MFOS: MFOS12 Mask               */

#define SYS_REGLCTL_REGLCTL_Pos         (0)                                         /*!< SYS_T::REGLCTL: REGLCTL Position           */
#define SYS_REGLCTL_REGLCTL_Msk         (0xFFUL << SYS_REGLCTL_REGLCTL_Pos)         /*!< SYS_T::REGLCTL: REGLCTL Mask               */

#define SYS_CPUCFG_INTRTEN_Pos          (0)                                         /*!< SYS_T::CPUCFG: INTRTEN Position            */
#define SYS_CPUCFG_INTRTEN_Msk          (0x1UL << SYS_CPUCFG_INTRTEN_Pos)           /*!< SYS_T::CPUCFG: INTRTEN Mask                */

#define SYS_TCTLHIRC_FREQSEL_Pos        (0)                                         /*!< SYS_T::TCTLHIRC: FREQSEL Position          */
#define SYS_TCTLHIRC_FREQSEL_Msk        (0x3UL << SYS_TCTLHIRC_FREQSEL_Pos)         /*!< SYS_T::TCTLHIRC: FREQSEL Mask              */

#define SYS_TCTLHIRC_ACCURSEL_Pos       (2)                                         /*!< SYS_T::TCTLHIRC: ACCURSEL Position         */
#define SYS_TCTLHIRC_ACCURSEL_Msk       (0x3UL << SYS_TCTLHIRC_ACCURSEL_Pos)        /*!< SYS_T::TCTLHIRC: ACCURSEL Mask             */

#define SYS_TCTLHIRC_LOOPSEL_Pos        (4)                                         /*!< SYS_T::TCTLHIRC: LOOPSEL Position          */
#define SYS_TCTLHIRC_LOOPSEL_Msk        (0x3UL << SYS_TCTLHIRC_LOOPSEL_Pos)         /*!< SYS_T::TCTLHIRC: LOOPSEL Mask              */

#define SYS_TCTLHIRC_RETRYCNT_Pos       (6)                                         /*!< SYS_T::TCTLHIRC: RETRYCNT Position         */
#define SYS_TCTLHIRC_RETRYCNT_Msk       (0x3UL << SYS_TCTLHIRC_RETRYCNT_Pos)        /*!< SYS_T::TCTLHIRC: RETRYCNT Mask             */

#define SYS_TCTLHIRC_CESTOPEN_Pos       (8)                                         /*!< SYS_T::TCTLHIRC: CESTOPEN Position         */
#define SYS_TCTLHIRC_CESTOPEN_Msk       (0x1UL << SYS_TCTLHIRC_CESTOPEN_Pos)        /*!< SYS_T::TCTLHIRC: CESTOPEN Mask             */

#define SYS_TCTLHIRC_BOUNDEN_Pos        (9)                                         /*!< SYS_T::TCTLHIRC: BOUNDEN Position          */
#define SYS_TCTLHIRC_BOUNDEN_Msk        (0x1UL << SYS_TCTLHIRC_BOUNDEN_Pos)         /*!< SYS_T::TCTLHIRC: BOUNDEN Mask              */

#define SYS_TCTLHIRC_BOUNDARY_Pos       (16)                                        /*!< SYS_T::TCTLHIRC: BOUNDARY Position         */
#define SYS_TCTLHIRC_BOUNDARY_Msk       (0x1FUL << SYS_TCTLHIRC_BOUNDARY_Pos)       /*!< SYS_T::TCTLHIRC: BOUNDARY Mask             */

#define SYS_TIENHIRC_TFAILIEN_Pos       (1)                                         /*!< SYS_T::TIENHIRC: TFAILIEN Position         */
#define SYS_TIENHIRC_TFAILIEN_Msk       (0x1UL << SYS_TIENHIRC_TFAILIEN_Pos)        /*!< SYS_T::TIENHIRC: TFAILIEN Mask             */

#define SYS_TIENHIRC_CLKEIEN_Pos        (2)                                         /*!< SYS_T::TIENHIRC: CLKEIEN Position          */
#define SYS_TIENHIRC_CLKEIEN_Msk        (0x1UL << SYS_TIENHIRC_CLKEIEN_Pos)         /*!< SYS_T::TIENHIRC: CLKEIEN Mask              */

#define SYS_TISTSHIRC_FREQLOCK_Pos      (0)                                         /*!< SYS_T::TISTSHIRC: FREQLOCK Position        */
#define SYS_TISTSHIRC_FREQLOCK_Msk      (0x1UL << SYS_TISTSHIRC_FREQLOCK_Pos)       /*!< SYS_T::TISTSHIRC: FREQLOCK Mask            */

#define SYS_TISTSHIRC_TFAILIF_Pos       (1)                                         /*!< SYS_T::TISTSHIRC: TFAILIF Position         */
#define SYS_TISTSHIRC_TFAILIF_Msk       (0x1UL << SYS_TISTSHIRC_TFAILIF_Pos)        /*!< SYS_T::TISTSHIRC: TFAILIF Mask             */

#define SYS_TISTSHIRC_CLKERRIF_Pos      (2)                                         /*!< SYS_T::TISTSHIRC: CLKERRIF Position        */
#define SYS_TISTSHIRC_CLKERRIF_Msk      (0x1UL << SYS_TISTSHIRC_CLKERRIF_Pos)       /*!< SYS_T::TISTSHIRC: CLKERRIF Mask            */

#define SYS_TISTSHIRC_OVBDIF_Pos        (3)                                         /*!< SYS_T::TISTSHIRC: OVBDIF Position          */
#define SYS_TISTSHIRC_OVBDIF_Msk        (0x1UL << SYS_TISTSHIRC_OVBDIF_Pos)         /*!< SYS_T::TISTSHIRC: OVBDIF Mask              */

#define SYS_ADCRST_ADC0RST_Pos          (0)                                         /*!< SYS_T::ADCRST: ADC0RST Position            */
#define SYS_ADCRST_ADC0RST_Msk          (0x1UL << SYS_ADCRST_ADC0RST_Pos)           /*!< SYS_T::ADCRST: ADC0RST Mask                */

#define SYS_BPWMRST_BPWM0RST_Pos        (0)                                         /*!< SYS_T::BPWMRST: BPWM0RST Position          */
#define SYS_BPWMRST_BPWM0RST_Msk        (0x1UL << SYS_BPWMRST_BPWM0RST_Pos)         /*!< SYS_T::BPWMRST: BPWM0RST Mask              */

#define SYS_BPWMRST_BPWM1RST_Pos        (1)                                         /*!< SYS_T::BPWMRST: BPWM1RST Position          */
#define SYS_BPWMRST_BPWM1RST_Msk        (0x1UL << SYS_BPWMRST_BPWM1RST_Pos)         /*!< SYS_T::BPWMRST: BPWM1RST Mask              */

#define SYS_CRCRST_CRC0RST_Pos          (0)                                         /*!< SYS_T::CRCRST: CRC0RST Position            */
#define SYS_CRCRST_CRC0RST_Msk          (0x1UL << SYS_CRCRST_CRC0RST_Pos)           /*!< SYS_T::CRCRST: CRC0RST Mask                */

#define SYS_FMCRST_FMC0RST_Pos          (0)                                         /*!< SYS_T::FMCRST: FMC0RST Position            */
#define SYS_FMCRST_FMC0RST_Msk          (0x1UL << SYS_FMCRST_FMC0RST_Pos)           /*!< SYS_T::FMCRST: FMC0RST Mask                */

#define SYS_FMCRST_DFMC0RST_Pos         (8)                                         /*!< SYS_T::FMCRST: DFMC0RST Position           */
#define SYS_FMCRST_DFMC0RST_Msk         (0x1UL << SYS_FMCRST_DFMC0RST_Pos)          /*!< SYS_T::FMCRST: DFMC0RST Mask               */

#define SYS_GPIORST_GPIO0RST_Pos        (0)                                         /*!< SYS_T::GPIORST: GPIO0RST Position          */
#define SYS_GPIORST_GPIO0RST_Msk        (0x1UL << SYS_GPIORST_GPIO0RST_Pos)         /*!< SYS_T::GPIORST: GPIO0RST Mask              */

#define SYS_I2CRST_I2C0RST_Pos          (0)                                         /*!< SYS_T::I2CRST: I2C0RST Position            */
#define SYS_I2CRST_I2C0RST_Msk          (0x1UL << SYS_I2CRST_I2C0RST_Pos)           /*!< SYS_T::I2CRST: I2C0RST Mask                */

#define SYS_I2CRST_I2C1RST_Pos          (1)                                         /*!< SYS_T::I2CRST: I2C1RST Position            */
#define SYS_I2CRST_I2C1RST_Msk          (0x1UL << SYS_I2CRST_I2C1RST_Pos)           /*!< SYS_T::I2CRST: I2C1RST Mask                */

#define SYS_I2CRST_I2C2RST_Pos          (2)                                         /*!< SYS_T::I2CRST: I2C2RST Position            */
#define SYS_I2CRST_I2C2RST_Msk          (0x1UL << SYS_I2CRST_I2C2RST_Pos)           /*!< SYS_T::I2CRST: I2C2RST Mask                */

#define SYS_PDMARST_PDMA0RST_Pos        (0)                                         /*!< SYS_T::PDMARST: PDMA0RST Position          */
#define SYS_PDMARST_PDMA0RST_Msk        (0x1UL << SYS_PDMARST_PDMA0RST_Pos)         /*!< SYS_T::PDMARST: PDMA0RST Mask              */

#define SYS_TMRRST_TMR0RST_Pos          (0)                                         /*!< SYS_T::TMRRST: TMR0RST Position            */
#define SYS_TMRRST_TMR0RST_Msk          (0x1UL << SYS_TMRRST_TMR0RST_Pos)           /*!< SYS_T::TMRRST: TMR0RST Mask                */

#define SYS_TMRRST_TMR1RST_Pos          (1)                                         /*!< SYS_T::TMRRST: TMR1RST Position            */
#define SYS_TMRRST_TMR1RST_Msk          (0x1UL << SYS_TMRRST_TMR1RST_Pos)           /*!< SYS_T::TMRRST: TMR1RST Mask                */

#define SYS_TMRRST_TMR2RST_Pos          (2)                                         /*!< SYS_T::TMRRST: TMR2RST Position            */
#define SYS_TMRRST_TMR2RST_Msk          (0x1UL << SYS_TMRRST_TMR2RST_Pos)           /*!< SYS_T::TMRRST: TMR2RST Mask                */

#define SYS_TMRRST_TMR3RST_Pos          (3)                                         /*!< SYS_T::TMRRST: TMR3RST Position            */
#define SYS_TMRRST_TMR3RST_Msk          (0x1UL << SYS_TMRRST_TMR3RST_Pos)           /*!< SYS_T::TMRRST: TMR3RST Mask                */

#define SYS_TMRRST_TMR4RST_Pos          (4)                                         /*!< SYS_T::TMRRST: TMR4RST Position            */
#define SYS_TMRRST_TMR4RST_Msk          (0x1UL << SYS_TMRRST_TMR4RST_Pos)           /*!< SYS_T::TMRRST: TMR4RST Mask                */

#define SYS_TMRRST_TMR5RST_Pos          (5)                                         /*!< SYS_T::TMRRST: TMR5RST Position            */
#define SYS_TMRRST_TMR5RST_Msk          (0x1UL << SYS_TMRRST_TMR5RST_Pos)           /*!< SYS_T::TMRRST: TMR5RST Mask                */

#define SYS_TMRRST_TMR6RST_Pos          (6)                                         /*!< SYS_T::TMRRST: TMR6RST Position            */
#define SYS_TMRRST_TMR6RST_Msk          (0x1UL << SYS_TMRRST_TMR6RST_Pos)           /*!< SYS_T::TMRRST: TMR6RST Mask                */

#define SYS_TMRRST_TMR7RST_Pos          (7)                                         /*!< SYS_T::TMRRST: TMR7RST Position            */
#define SYS_TMRRST_TMR7RST_Msk          (0x1UL << SYS_TMRRST_TMR7RST_Pos)           /*!< SYS_T::TMRRST: TMR7RST Mask                */

#define SYS_TMRRST_TMR8RST_Pos          (8)                                         /*!< SYS_T::TMRRST: TMR8RST Position            */
#define SYS_TMRRST_TMR8RST_Msk          (0x1UL << SYS_TMRRST_TMR8RST_Pos)           /*!< SYS_T::TMRRST: TMR8RST Mask                */

#define SYS_UARTRST_UART0RST_Pos        (0)                                         /*!< SYS_T::UARTRST: UART0RST Position          */
#define SYS_UARTRST_UART0RST_Msk        (0x1UL << SYS_UARTRST_UART0RST_Pos)         /*!< SYS_T::UARTRST: UART0RST Mask              */

#define SYS_UARTRST_UART1RST_Pos        (1)                                         /*!< SYS_T::UARTRST: UART1RST Position          */
#define SYS_UARTRST_UART1RST_Msk        (0x1UL << SYS_UARTRST_UART1RST_Pos)         /*!< SYS_T::UARTRST: UART1RST Mask              */

#define SYS_UARTRST_UART2RST_Pos        (2)                                         /*!< SYS_T::UARTRST: UART2RST Position          */
#define SYS_UARTRST_UART2RST_Msk        (0x1UL << SYS_UARTRST_UART2RST_Pos)         /*!< SYS_T::UARTRST: UART2RST Mask              */

#define SYS_UARTRST_UART3RST_Pos        (3)                                         /*!< SYS_T::UARTRST: UART3RST Position          */
#define SYS_UARTRST_UART3RST_Msk        (0x1UL << SYS_UARTRST_UART3RST_Pos)         /*!< SYS_T::UARTRST: UART3RST Mask              */

#define SYS_UARTRST_UART4RST_Pos        (4)                                         /*!< SYS_T::UARTRST: UART4RST Position          */
#define SYS_UARTRST_UART4RST_Msk        (0x1UL << SYS_UARTRST_UART4RST_Pos)         /*!< SYS_T::UARTRST: UART4RST Mask              */

#define SYS_USCIRST_USCI0RST_Pos        (0)                                         /*!< SYS_T::USCIRST: USCI0RST Position          */
#define SYS_USCIRST_USCI0RST_Msk        (0x1UL << SYS_USCIRST_USCI0RST_Pos)         /*!< SYS_T::USCIRST: USCI0RST Mask              */

#define SYS_USCIRST_USCI1RST_Pos        (1)                                         /*!< SYS_T::USCIRST: USCI1RST Position          */
#define SYS_USCIRST_USCI1RST_Msk        (0x1UL << SYS_USCIRST_USCI1RST_Pos)         /*!< SYS_T::USCIRST: USCI1RST Mask              */

#define SYS_USCIRST_USCI2RST_Pos        (2)                                         /*!< SYS_T::USCIRST: USCI2RST Position          */
#define SYS_USCIRST_USCI2RST_Msk        (0x1UL << SYS_USCIRST_USCI2RST_Pos)         /*!< SYS_T::USCIRST: USCI2RST Mask              */

#define SYS_USCIRST_USCI3RST_Pos        (3)                                         /*!< SYS_T::USCIRST: USCI3RST Position          */
#define SYS_USCIRST_USCI3RST_Msk        (0x1UL << SYS_USCIRST_USCI3RST_Pos)         /*!< SYS_T::USCIRST: USCI3RST Mask              */

#define SYS_USCIRST_USCI4RST_Pos        (4)                                         /*!< SYS_T::USCIRST: USCI4RST Position          */
#define SYS_USCIRST_USCI4RST_Msk        (0x1UL << SYS_USCIRST_USCI4RST_Pos)         /*!< SYS_T::USCIRST: USCI4RST Mask              */

#define SYS_WWDTRST_WWDT0RST_Pos        (0)                                         /*!< SYS_T::WWDTRST: WWDT0RST Position          */
#define SYS_WWDTRST_WWDT0RST_Msk        (0x1UL << SYS_WWDTRST_WWDT0RST_Pos)         /*!< SYS_T::WWDTRST: WWDT0RST Mask              */

#define SYS_NMIEN_BODOUT_Pos            (0)                                         /*!< SYS_T::NMIEN: BODOUT Position              */
#define SYS_NMIEN_BODOUT_Msk            (0x1UL << SYS_NMIEN_BODOUT_Pos)             /*!< SYS_T::NMIEN: BODOUT Mask                  */

#define SYS_NMIEN_IRCINT_Pos            (1)                                         /*!< SYS_T::NMIEN: IRCINT Position              */
#define SYS_NMIEN_IRCINT_Msk            (0x1UL << SYS_NMIEN_IRCINT_Pos)             /*!< SYS_T::NMIEN: IRCINT Mask                  */

#define SYS_NMIEN_PWRWUINT_Pos          (2)                                         /*!< SYS_T::NMIEN: PWRWUINT Position            */
#define SYS_NMIEN_PWRWUINT_Msk          (0x1UL << SYS_NMIEN_PWRWUINT_Pos)           /*!< SYS_T::NMIEN: PWRWUINT Mask                */

#define SYS_NMIEN_INTMUX_Pos            (3)                                         /*!< SYS_T::NMIEN: INTMUX Position              */
#define SYS_NMIEN_INTMUX_Msk            (0x1UL << SYS_NMIEN_INTMUX_Pos)             /*!< SYS_T::NMIEN: INTMUX Mask                  */

#define SYS_NMIEN_CLKFAIL_Pos           (5)                                         /*!< SYS_T::NMIEN: CLKFAIL Position             */
#define SYS_NMIEN_CLKFAIL_Msk           (0x1UL << SYS_NMIEN_CLKFAIL_Pos)            /*!< SYS_T::NMIEN: CLKFAIL Mask                 */

#define SYS_NMIEN_RTCINT_Pos            (6)                                         /*!< SYS_T::NMIEN: RTCINT Position              */
#define SYS_NMIEN_RTCINT_Msk            (0x1UL << SYS_NMIEN_RTCINT_Pos)             /*!< SYS_T::NMIEN: RTCINT Mask                  */

#define SYS_NMIEN_EINT0_Pos             (8)                                         /*!< SYS_T::NMIEN: EINT0 Position               */
#define SYS_NMIEN_EINT0_Msk             (0x1UL << SYS_NMIEN_EINT0_Pos)              /*!< SYS_T::NMIEN: EINT0 Mask                   */

#define SYS_NMIEN_EINT1_Pos             (9)                                         /*!< SYS_T::NMIEN: EINT1 Position               */
#define SYS_NMIEN_EINT1_Msk             (0x1UL << SYS_NMIEN_EINT1_Pos)              /*!< SYS_T::NMIEN: EINT1 Mask                   */

#define SYS_NMIEN_EINT2_Pos             (10)                                        /*!< SYS_T::NMIEN: EINT2 Position               */
#define SYS_NMIEN_EINT2_Msk             (0x1UL << SYS_NMIEN_EINT2_Pos)              /*!< SYS_T::NMIEN: EINT2 Mask                   */

#define SYS_NMIEN_EINT3_Pos             (11)                                        /*!< SYS_T::NMIEN: EINT3 Position               */
#define SYS_NMIEN_EINT3_Msk             (0x1UL << SYS_NMIEN_EINT3_Pos)              /*!< SYS_T::NMIEN: EINT3 Mask                   */

#define SYS_NMIEN_EINT4_Pos             (12)                                        /*!< SYS_T::NMIEN: EINT4 Position               */
#define SYS_NMIEN_EINT4_Msk             (0x1UL << SYS_NMIEN_EINT4_Pos)              /*!< SYS_T::NMIEN: EINT4 Mask                   */

#define SYS_NMIEN_EINT5_Pos             (13)                                        /*!< SYS_T::NMIEN: EINT5 Position               */
#define SYS_NMIEN_EINT5_Msk             (0x1UL << SYS_NMIEN_EINT5_Pos)              /*!< SYS_T::NMIEN: EINT5 Mask                   */

#define SYS_NMIEN_UART0INT_Pos          (16)                                        /*!< SYS_T::NMIEN: UART0INT Position            */
#define SYS_NMIEN_UART0INT_Msk          (0x1UL << SYS_NMIEN_UART0INT_Pos)           /*!< SYS_T::NMIEN: UART0INT Mask                */

#define SYS_NMIEN_UART1INT_Pos          (17)                                        /*!< SYS_T::NMIEN: UART1INT Position            */
#define SYS_NMIEN_UART1INT_Msk          (0x1UL << SYS_NMIEN_UART1INT_Pos)           /*!< SYS_T::NMIEN: UART1INT Mask                */

#define SYS_NMIEN_UART2INT_Pos          (18)                                        /*!< SYS_T::NMIEN: UART2INT Position            */
#define SYS_NMIEN_UART2INT_Msk          (0x1UL << SYS_NMIEN_UART2INT_Pos)           /*!< SYS_T::NMIEN: UART2INT Mask                */

#define SYS_NMIEN_UART3INT_Pos          (19)                                        /*!< SYS_T::NMIEN: UART3INT Position            */
#define SYS_NMIEN_UART3INT_Msk          (0x1UL << SYS_NMIEN_UART3INT_Pos)           /*!< SYS_T::NMIEN: UART3INT Mask                */

#define SYS_NMIEN_UART4INT_Pos          (20)                                        /*!< SYS_T::NMIEN: UART4INT Position            */
#define SYS_NMIEN_UART4INT_Msk          (0x1UL << SYS_NMIEN_UART4INT_Pos)           /*!< SYS_T::NMIEN: UART4INT Mask                */

#define SYS_NMIEN_SR0MBECC_Pos          (24)                                        /*!< SYS_T::NMIEN: SR0MBECC Position            */
#define SYS_NMIEN_SR0MBECC_Msk          (0x1UL << SYS_NMIEN_SR0MBECC_Pos)           /*!< SYS_T::NMIEN: SR0MBECC Mask                */

#define SYS_NMIEN_SR1MBECC_Pos          (25)                                        /*!< SYS_T::NMIEN: SR1MBECC Position            */
#define SYS_NMIEN_SR1MBECC_Msk          (0x1UL << SYS_NMIEN_SR1MBECC_Pos)           /*!< SYS_T::NMIEN: SR1MBECC Mask                */

#define SYS_NMIEN_FSHMBECC_Pos          (26)                                        /*!< SYS_T::NMIEN: FSHMBECC Position            */
#define SYS_NMIEN_FSHMBECC_Msk          (0x1UL << SYS_NMIEN_FSHMBECC_Pos)           /*!< SYS_T::NMIEN: FSHMBECC Mask                */

#define SYS_NMISTS_BODOUT_Pos           (0)                                         /*!< SYS_T::NMISTS: BODOUT Position             */
#define SYS_NMISTS_BODOUT_Msk           (0x1UL << SYS_NMISTS_BODOUT_Pos)            /*!< SYS_T::NMISTS: BODOUT Mask                 */

#define SYS_NMISTS_IRCINT_Pos           (1)                                         /*!< SYS_T::NMISTS: IRCINT Position             */
#define SYS_NMISTS_IRCINT_Msk           (0x1UL << SYS_NMISTS_IRCINT_Pos)            /*!< SYS_T::NMISTS: IRCINT Mask                 */

#define SYS_NMISTS_PWRWUINT_Pos         (2)                                         /*!< SYS_T::NMISTS: PWRWUINT Position           */
#define SYS_NMISTS_PWRWUINT_Msk         (0x1UL << SYS_NMISTS_PWRWUINT_Pos)          /*!< SYS_T::NMISTS: PWRWUINT Mask               */

#define SYS_NMISTS_INTMUX_Pos           (3)                                         /*!< SYS_T::NMISTS: INTMUX Position             */
#define SYS_NMISTS_INTMUX_Msk           (0x1UL << SYS_NMISTS_INTMUX_Pos)            /*!< SYS_T::NMISTS: INTMUX Mask                 */

#define SYS_NMISTS_CLKFAIL_Pos          (5)                                         /*!< SYS_T::NMISTS: CLKFAIL Position            */
#define SYS_NMISTS_CLKFAIL_Msk          (0x1UL << SYS_NMISTS_CLKFAIL_Pos)           /*!< SYS_T::NMISTS: CLKFAIL Mask                */

#define SYS_NMISTS_RTCINT_Pos           (6)                                         /*!< SYS_T::NMISTS: RTCINT Position             */
#define SYS_NMISTS_RTCINT_Msk           (0x1UL << SYS_NMISTS_RTCINT_Pos)            /*!< SYS_T::NMISTS: RTCINT Mask                 */

#define SYS_NMISTS_EINT0_Pos            (8)                                         /*!< SYS_T::NMISTS: EINT0 Position              */
#define SYS_NMISTS_EINT0_Msk            (0x1UL << SYS_NMISTS_EINT0_Pos)             /*!< SYS_T::NMISTS: EINT0 Mask                  */

#define SYS_NMISTS_EINT1_Pos            (9)                                         /*!< SYS_T::NMISTS: EINT1 Position              */
#define SYS_NMISTS_EINT1_Msk            (0x1UL << SYS_NMISTS_EINT1_Pos)             /*!< SYS_T::NMISTS: EINT1 Mask                  */

#define SYS_NMISTS_EINT2_Pos            (10)                                        /*!< SYS_T::NMISTS: EINT2 Position              */
#define SYS_NMISTS_EINT2_Msk            (0x1UL << SYS_NMISTS_EINT2_Pos)             /*!< SYS_T::NMISTS: EINT2 Mask                  */

#define SYS_NMISTS_EINT3_Pos            (11)                                        /*!< SYS_T::NMISTS: EINT3 Position              */
#define SYS_NMISTS_EINT3_Msk            (0x1UL << SYS_NMISTS_EINT3_Pos)             /*!< SYS_T::NMISTS: EINT3 Mask                  */

#define SYS_NMISTS_EINT4_Pos            (12)                                        /*!< SYS_T::NMISTS: EINT4 Position              */
#define SYS_NMISTS_EINT4_Msk            (0x1UL << SYS_NMISTS_EINT4_Pos)             /*!< SYS_T::NMISTS: EINT4 Mask                  */

#define SYS_NMISTS_EINT5_Pos            (13)                                        /*!< SYS_T::NMISTS: EINT5 Position              */
#define SYS_NMISTS_EINT5_Msk            (0x1UL << SYS_NMISTS_EINT5_Pos)             /*!< SYS_T::NMISTS: EINT5 Mask                  */

#define SYS_NMISTS_UART0INT_Pos         (16)                                        /*!< SYS_T::NMISTS: UART0INT Position           */
#define SYS_NMISTS_UART0INT_Msk         (0x1UL << SYS_NMISTS_UART0INT_Pos)          /*!< SYS_T::NMISTS: UART0INT Mask               */

#define SYS_NMISTS_UART1INT_Pos         (17)                                        /*!< SYS_T::NMISTS: UART1INT Position           */
#define SYS_NMISTS_UART1INT_Msk         (0x1UL << SYS_NMISTS_UART1INT_Pos)          /*!< SYS_T::NMISTS: UART1INT Mask               */

#define SYS_NMISTS_UART2INT_Pos         (18)                                        /*!< SYS_T::NMISTS: UART2INT Position           */
#define SYS_NMISTS_UART2INT_Msk         (0x1UL << SYS_NMISTS_UART2INT_Pos)          /*!< SYS_T::NMISTS: UART2INT Mask               */

#define SYS_NMISTS_UART3INT_Pos         (19)                                        /*!< SYS_T::NMISTS: UART3INT Position           */
#define SYS_NMISTS_UART3INT_Msk         (0x1UL << SYS_NMISTS_UART3INT_Pos)          /*!< SYS_T::NMISTS: UART3INT Mask               */

#define SYS_NMISTS_UART4INT_Pos         (20)                                        /*!< SYS_T::NMISTS: UART4INT Position           */
#define SYS_NMISTS_UART4INT_Msk         (0x1UL << SYS_NMISTS_UART4INT_Pos)          /*!< SYS_T::NMISTS: UART4INT Mask               */

#define SYS_NMISTS_SR0MBECC_Pos         (24)                                        /*!< SYS_T::NMISTS: SR0MBECC Position           */
#define SYS_NMISTS_SR0MBECC_Msk         (0x1UL << SYS_NMISTS_SR0MBECC_Pos)          /*!< SYS_T::NMISTS: SR0MBECC Mask               */

#define SYS_NMISTS_SR1MBECC_Pos         (25)                                        /*!< SYS_T::NMISTS: SR1MBECC Position           */
#define SYS_NMISTS_SR1MBECC_Msk         (0x1UL << SYS_NMISTS_SR1MBECC_Pos)          /*!< SYS_T::NMISTS: SR1MBECC Mask               */

#define SYS_NMISTS_FSHMBECC_Pos         (26)                                        /*!< SYS_T::NMISTS: FSHMBECC Position           */
#define SYS_NMISTS_FSHMBECC_Msk         (0x1UL << SYS_NMISTS_FSHMBECC_Pos)          /*!< SYS_T::NMISTS: FSHMBECC Mask               */

#define SYS_NMIMSEL_NMIMSEL_Pos         (0)                                         /*!< SYS_T::NMIMSEL: NMIMSEL Position           */
#define SYS_NMIMSEL_NMIMSEL_Msk         (0xFFUL << SYS_NMIMSEL_NMIMSEL_Pos)         /*!< SYS_T::NMIMSEL: NMIMSEL Mask               */

#define SYS_AHBCTL_INTACTEN_Pos         (0)                                         /*!< SYS_T::AHBCTL: INTACTEN Position           */
#define SYS_AHBCTL_INTACTEN_Msk         (0x1UL << SYS_AHBCTL_INTACTEN_Pos)          /*!< SYS_T::AHBCTL: INTACTEN Mask               */

#define SYS_SRAMBCTL_SR0BIST_Pos        (0)                                         /*!< SYS_T::SRAMBCTL: SR0BIST Position          */
#define SYS_SRAMBCTL_SR0BIST_Msk        (0x1UL << SYS_SRAMBCTL_SR0BIST_Pos)         /*!< SYS_T::SRAMBCTL: SR0BIST Mask              */

#define SYS_SRAMBCTL_SR1BIST_Pos        (1)                                         /*!< SYS_T::SRAMBCTL: SR1BIST Position          */
#define SYS_SRAMBCTL_SR1BIST_Msk        (0x1UL << SYS_SRAMBCTL_SR1BIST_Pos)         /*!< SYS_T::SRAMBCTL: SR1BIST Mask              */

#define SYS_SRAMBFF_SR0BFF_Pos          (0)                                         /*!< SYS_T::SRAMBFF: SR0BFF Position            */
#define SYS_SRAMBFF_SR0BFF_Msk          (0x1UL << SYS_SRAMBFF_SR0BFF_Pos)           /*!< SYS_T::SRAMBFF: SR0BFF Mask                */

#define SYS_SRAMBFF_SR1BFF_Pos          (1)                                         /*!< SYS_T::SRAMBFF: SR1BFF Position            */
#define SYS_SRAMBFF_SR1BFF_Msk          (0x1UL << SYS_SRAMBFF_SR1BFF_Pos)           /*!< SYS_T::SRAMBFF: SR1BFF Mask                */

#define SYS_SRAMBRF_SR0BRF_Pos          (0)                                         /*!< SYS_T::SRAMBRF: SR0BRF Position            */
#define SYS_SRAMBRF_SR0BRF_Msk          (0x1UL << SYS_SRAMBRF_SR0BRF_Pos)           /*!< SYS_T::SRAMBRF: SR0BRF Mask                */

#define SYS_SRAMBRF_SR1BRF_Pos          (1)                                         /*!< SYS_T::SRAMBRF: SR1BRF Position            */
#define SYS_SRAMBRF_SR1BRF_Msk          (0x1UL << SYS_SRAMBRF_SR1BRF_Pos)           /*!< SYS_T::SRAMBRF: SR1BRF Mask                */

#define SYS_SRAM0ICTL_ECCEN_Pos         (0)                                         /*!< SYS_T::SRAM0ICTL: ECCEN Position           */
#define SYS_SRAM0ICTL_ECCEN_Msk         (0x1UL << SYS_SRAM0ICTL_ECCEN_Pos)          /*!< SYS_T::SRAM0ICTL: ECCEN Mask               */

#define SYS_SRAM0ICTL_SBERRIEN_Pos      (1)                                         /*!< SYS_T::SRAM0ICTL: SBERRIEN Position        */
#define SYS_SRAM0ICTL_SBERRIEN_Msk      (0x1UL << SYS_SRAM0ICTL_SBERRIEN_Pos)       /*!< SYS_T::SRAM0ICTL: SBERRIEN Mask            */

#define SYS_SRAM0ICTL_MBERRIEN_Pos      (2)                                         /*!< SYS_T::SRAM0ICTL: MBERRIEN Position        */
#define SYS_SRAM0ICTL_MBERRIEN_Msk      (0x1UL << SYS_SRAM0ICTL_MBERRIEN_Pos)       /*!< SYS_T::SRAM0ICTL: MBERRIEN Mask            */

#define SYS_SRAM0STS_SBERRIF_Pos        (1)                                         /*!< SYS_T::SRAM0STS: SBERRIF Position          */
#define SYS_SRAM0STS_SBERRIF_Msk        (0x1UL << SYS_SRAM0STS_SBERRIF_Pos)         /*!< SYS_T::SRAM0STS: SBERRIF Mask              */

#define SYS_SRAM0STS_MBERRIF_Pos        (2)                                         /*!< SYS_T::SRAM0STS: MBERRIF Position          */
#define SYS_SRAM0STS_MBERRIF_Msk        (0x1UL << SYS_SRAM0STS_MBERRIF_Pos)         /*!< SYS_T::SRAM0STS: MBERRIF Mask              */

#define SYS_SRAM0STS_B0ECCEIF_Pos       (8)                                         /*!< SYS_T::SRAM0STS: B0ECCEIF Position         */
#define SYS_SRAM0STS_B0ECCEIF_Msk       (0x1UL << SYS_SRAM0STS_B0ECCEIF_Pos)        /*!< SYS_T::SRAM0STS: B0ECCEIF Mask             */

#define SYS_SRAM0STS_B1ECCEIF_Pos       (9)                                         /*!< SYS_T::SRAM0STS: B1ECCEIF Position         */
#define SYS_SRAM0STS_B1ECCEIF_Msk       (0x1UL << SYS_SRAM0STS_B1ECCEIF_Pos)        /*!< SYS_T::SRAM0STS: B1ECCEIF Mask             */

#define SYS_SRAM0STS_B2ECCEIF_Pos       (10)                                        /*!< SYS_T::SRAM0STS: B2ECCEIF Position         */
#define SYS_SRAM0STS_B2ECCEIF_Msk       (0x1UL << SYS_SRAM0STS_B2ECCEIF_Pos)        /*!< SYS_T::SRAM0STS: B2ECCEIF Mask             */

#define SYS_SRAM0STS_B3ECCEIF_Pos       (11)                                        /*!< SYS_T::SRAM0STS: B3ECCEIF Position         */
#define SYS_SRAM0STS_B3ECCEIF_Msk       (0x1UL << SYS_SRAM0STS_B3ECCEIF_Pos)        /*!< SYS_T::SRAM0STS: B3ECCEIF Mask             */

#define SYS_SRAM0EADR_ECCEADDR_Pos      (2)                                             /*!< SYS_T::SRAM0EADR: ECCEADDR Position        */
#define SYS_SRAM0EADR_ECCEADDR_Msk      (0x3FFFFFFFUL << SYS_SRAM0EADR_ECCEADDR_Pos)    /*!< SYS_T::SRAM0EADR: ECCEADDR Mask            */

#define SYS_SRAM1ICTL_ECCEN_Pos         (0)                                         /*!< SYS_T::SRAM1ICTL: ECCEN Position           */
#define SYS_SRAM1ICTL_ECCEN_Msk         (0x1UL << SYS_SRAM1ICTL_ECCEN_Pos)          /*!< SYS_T::SRAM1ICTL: ECCEN Mask               */

#define SYS_SRAM1ICTL_SBERRIEN_Pos      (1)                                         /*!< SYS_T::SRAM1ICTL: SBERRIEN Position        */
#define SYS_SRAM1ICTL_SBERRIEN_Msk      (0x1UL << SYS_SRAM1ICTL_SBERRIEN_Pos)       /*!< SYS_T::SRAM1ICTL: SBERRIEN Mask            */

#define SYS_SRAM1ICTL_MBERRIEN_Pos      (2)                                         /*!< SYS_T::SRAM1ICTL: MBERRIEN Position        */
#define SYS_SRAM1ICTL_MBERRIEN_Msk      (0x1UL << SYS_SRAM1ICTL_MBERRIEN_Pos)       /*!< SYS_T::SRAM1ICTL: MBERRIEN Mask            */

#define SYS_SRAM1STS_SBERRIF_Pos        (1)                                         /*!< SYS_T::SRAM1STS: SBERRIF Position          */
#define SYS_SRAM1STS_SBERRIF_Msk        (0x1UL << SYS_SRAM1STS_SBERRIF_Pos)         /*!< SYS_T::SRAM1STS: SBERRIF Mask              */

#define SYS_SRAM1STS_MBERRIF_Pos        (2)                                         /*!< SYS_T::SRAM1STS: MBERRIF Position          */
#define SYS_SRAM1STS_MBERRIF_Msk        (0x1UL << SYS_SRAM1STS_MBERRIF_Pos)         /*!< SYS_T::SRAM1STS: MBERRIF Mask              */

#define SYS_SRAM1STS_B0ECCEIF_Pos       (8)                                         /*!< SYS_T::SRAM1STS: B0ECCEIF Position         */
#define SYS_SRAM1STS_B0ECCEIF_Msk       (0x1UL << SYS_SRAM1STS_B0ECCEIF_Pos)        /*!< SYS_T::SRAM1STS: B0ECCEIF Mask             */

#define SYS_SRAM1STS_B1ECCEIF_Pos       (9)                                         /*!< SYS_T::SRAM1STS: B1ECCEIF Position         */
#define SYS_SRAM1STS_B1ECCEIF_Msk       (0x1UL << SYS_SRAM1STS_B1ECCEIF_Pos)        /*!< SYS_T::SRAM1STS: B1ECCEIF Mask             */

#define SYS_SRAM1STS_B2ECCEIF_Pos       (10)                                        /*!< SYS_T::SRAM1STS: B2ECCEIF Position         */
#define SYS_SRAM1STS_B2ECCEIF_Msk       (0x1UL << SYS_SRAM1STS_B2ECCEIF_Pos)        /*!< SYS_T::SRAM1STS: B2ECCEIF Mask             */

#define SYS_SRAM1STS_B3ECCEIF_Pos       (11)                                        /*!< SYS_T::SRAM1STS: B3ECCEIF Position         */
#define SYS_SRAM1STS_B3ECCEIF_Msk       (0x1UL << SYS_SRAM1STS_B3ECCEIF_Pos)        /*!< SYS_T::SRAM1STS: B3ECCEIF Mask             */

#define SYS_SRAM1EADR_ECCEADDR_Pos      (2)                                             /*!< SYS_T::SRAM1EADR: ECCEADDR Position        */
#define SYS_SRAM1EADR_ECCEADDR_Msk      (0x3FFFFFFFUL << SYS_SRAM1EADR_ECCEADDR_Pos)    /*!< SYS_T::SRAM1EADR: ECCEADDR Mask            */

#define SYS_INTEN_PDWKIEN_Pos           (0)                                         /*!< SYS_T::INTEN: PDWKIEN Position             */
#define SYS_INTEN_PDWKIEN_Msk           (0x1UL << SYS_INTEN_PDWKIEN_Pos)            /*!< SYS_T::INTEN: PDWKIEN Mask                 */

#define SYS_INTSTS_PDWKIF_Pos           (0)                                         /*!< SYS_T::INTSTS: PDWKIF Position             */
#define SYS_INTSTS_PDWKIF_Msk           (0x1UL << SYS_INTSTS_PDWKIF_Pos)            /*!< SYS_T::INTSTS: PDWKIF Mask                 */

#define SYS_INTSTS_CLRIF_Pos            (31)                                        /*!< SYS_T::INTSTS: CLRIF Position              */
#define SYS_INTSTS_CLRIF_Msk            (0x1UL << SYS_INTSTS_CLRIF_Pos)             /*!< SYS_T::INTSTS: CLRIF Mask                  */

#define SYS_PWRCTL_PRSTDBEN_Pos         (12)                                        /*!< SYS_T::PWRCTL: PRSTDBEN Position           */
#define SYS_PWRCTL_PRSTDBEN_Msk         (0x1UL << SYS_PWRCTL_PRSTDBEN_Pos)          /*!< SYS_T::PWRCTL: PRSTDBEN Mask               */

#define SYS_PWRCTL_WRBUSY_Pos           (31)                                        /*!< SYS_T::PWRCTL: WRBUSY Position             */
#define SYS_PWRCTL_WRBUSY_Msk           (0x1UL << SYS_PWRCTL_WRBUSY_Pos)            /*!< SYS_T::PWRCTL: WRBUSY Mask                 */

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */
/**@}*/ /* end of REGISTER group */

#endif /* __SYS_REG_H__ */
