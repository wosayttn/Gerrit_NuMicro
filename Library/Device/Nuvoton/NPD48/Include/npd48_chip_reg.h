/*---------------------- ?????????????????????????????????????????? -------------------------*/
/**
    @addtogroup CHIP ??????????????????????????????????????????(CHIP)
    Memory Mapped Structure for CHIP Controller
@{ */

/**
 * @var CHIP_T::PEIF
 * Offset: 0x16  Protect Event Interrupt Flag
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |OVPIF     |Over   Voltage Protect Interrupt Flag
 * |        |          |0 = No over   Voltage protect interrupt happened.
 * |        |          |1 = Over Voltage protect   interrupt happened.
 * |        |          |Note: User can write 1 to clear this bit.
 * |        |          |Note: This protect is level detect, the bit will not be   clear if the protect event still present.
 * |[1]     |UVPIF     |Under   Voltage Protect Interrupt Flag
 * |        |          |0 = No under   voltage protect interrupt happened.
 * |        |          |1 = Under voltage protect   interrupt happened.
 * |        |          |Note: User can write 1 to clear this bit.
 * |        |          |Note: This protect is level detect, the bit   will not be clear if the protect event still present.
 * |[2]     |OCPIF     |Overcurrent   Protect Interrupt Flag
 * |        |          |0 = No   overcurrent protect interrupt happened.
 * |        |          |1 = Overcurrent protect   interrupt happened.
 * |        |          |Note: User can write 1 to clear this bit.
 * |        |          |Note: This protect is level detect, the bit   will not be clear if the protect event still present.
 * |[3]     |NTCPIF    |NTC   Protect Interrupt Flag
 * |        |          |0 = No NTC   protect interrupt happened.
 * |        |          |1 = NTC protect interrupt   happened.
 * |        |          |Note: User can write 1 to clear this bit.
 * |        |          |Note: This protect is level detect, the bit   will not be clear if the protect event still present.
 * |[4]     |CCOVPIF   |CC Pin   Over Voltage Protect Interrupt Flag
 * |        |          |0 = No CC   pins over Voltage protect interrupt happened.
 * |        |          |1 = CC pins over Voltage   protect interrupt happened.
 * |        |          |Note: User can write 1 to clear this bit.
 * |        |          |Note: This protect is level detect, the bit will not be   clear if the protect event still present.
 * |[5]     |VCOCPIF   |VCONN   Overcurrent Protect Interrupt Flag
 * |        |          |0 = No VCONN overcurrent   protect interrupt happened.
 * |        |          |1 = VCONN overcurrent protect   interrupt happened.
 * |        |          |Note: User can write 1 to clear this bit.
 * |        |          |Note: This protect is level detect, the bit will not be   clear if the protect event still present.
 * @var CHIP_T::TRIMDB0
 * Offset: 0xD0  Trim Data Byte 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TRIMDB0   |Trim Data Byte 0
 * |        |          |TBD
 * @var CHIP_T::TRIMDB1
 * Offset: 0xD1  Trim Data Byte 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TRIMDB1   |Trim Data Byte 1
 * |        |          |TBD
 * @var CHIP_T::TRIMDB2
 * Offset: 0xD2  Trim Data Byte 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TRIMDB2   |Trim Data Byte 2
 * |        |          |TBD
 * @var CHIP_T::TRIMDB3
 * Offset: 0xD3  Trim Data Byte 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TRIMDB3   |Trim Data Byte 3
 * |        |          |TBD
 * @var CHIP_T::TRIMDB4
 * Offset: 0xD4  Trim Data Byte 4
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TRIMDB4   |Trim Data Byte 4
 * |        |          |TBD
 * @var CHIP_T::TRIMDB5
 * Offset: 0xD5  Trim Data Byte 5
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TRIMDB5   |Trim Data Byte 5
 * |        |          |TBD
 * @var CHIP_T::TRIMDB6
 * Offset: 0xD6  Trim Data Byte 6
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TRIMDB6   |Trim Data Byte 6
 * |        |          |TBD
 * @var CHIP_T::TRIMDB7
 * Offset: 0xD7  Trim Data Byte 7
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TRIMDB7   |Trim Data Byte 7
 * |        |          |TBD
 * @var CHIP_T::ATESTMD
 * Offset: 0xF4  Alpha Test Mode Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PA10OMD   |PA.10 Output Mode
 * |        |          |0 = Keep original PA.10 control.
 * |        |          |1 = Force PA.10 as push-pull mode (IODIRA10 control is ignore).
 * |[1]     |PA11OMD   |PA.11 Output Mode
 * |        |          |0 = Keep original PA.11 control.
 * |        |          |1 = Force PA.11 as push-pull mode (IODIRA11 control is ignore).
 * |[2]     |VREFTM    |VREF Test Mode
 * |        |          |0 = VREF test mode disabled.
 * |        |          |1 = VREF test mode enabled
 * |        |          |(User can measure VREF voltage through PFC pin) u2013 need check with CR50 if PFC can keep floating?
 * |        |          |[Design] control VRFCAPLESS pin VRF_TEST_EN
 * |[3]     |VINGA2EN  |VIN Gate 2 Enable Bit
 * |        |          |0 = VIN gate driver 2 Disabled.
 * |        |          |1 = VIN gate driver 2 Enabled.
 * |        |          |Note: Before set VINGA2EN = 1, user should enable GDCPEN (PWRCR[7]) = 1 and wait 1ms stable time.
 * |        |          |[Design] control GATEDRIVER pin Gate3_EN
 * |[4]     |PGATMEN   |PGA Offset Test Mode Enable Bit
 * |        |          |0 = PGA offset test mode Disabled.
 * |        |          |1 = PGA offset test mode Enabled.
 * |        |          |[Design] control MEGPGACATH pin TEST
 * |[5]     |LDO18TM   |LDO18 Test Mode Enable Bit
 * |        |          |0 = LDO18 test mode Disabled.
 * |        |          |1 = LDO18 test mode Enabled.
 * |        |          |Note: When LDO18 test mode enable, the 1.8V power can be measureed through ??
 * |        |          |[Design] control MEGPM pin TEST18
 * @var CHIP_T::TMCODE0
 * Offset: 0xF5  Test Mode Entry Code 0 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TMCODE0   |Test Mode Entry Code
 * |        |          |When user want to enter test   mode, should write TMCODE0 to 0x38, then write TMCODE1 for following   definition.
 * |        |          |TMCODE1 = 0x80, enter debug4   mode.
 * |        |          |TMCODE1 = 0x40, enter ADC   test mode.
 * |        |          |TMCODE1 = 0x20, enter PHY   mode.
 * |        |          |TMCODE1 = 0x10, enter   digital test mode.
 * |        |          |TMCODE1 = 0x08, enter debug3   mode.
 * |        |          |TMCODE1 = 0x04, enter debug2   mode.
 * |        |          |TMCODE1 = 0x02, enter debug1   mode.
 * |        |          |TMCODE1 = 0x01, enter DFT   mode.
 * @var CHIP_T::TMCODE1
 * Offset: 0xF6  Test Mode Entry Code 1 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TMECODE1  |Test Mode Entry Code
 * |        |          |When user want to enter test   mode, should write TMCODE0 to 0x38, then write TMCODE1 for following   definition.
 * |        |          |TMCODE1 = 0x80, enter debug4   mode.
 * |        |          |TMCODE1 = 0x40, enter ADC   test mode.
 * |        |          |TMCODE1 = 0x20, enter PHY   mode.
 * |        |          |TMCODE1 = 0x10, enter   digital test mode.
 * |        |          |TMCODE1 = 0x08, enter debug3   mode.
 * |        |          |TMCODE1 = 0x04, enter debug2   mode.
 * |        |          |TMCODE1 = 0x02, enter debug1   mode.
 * |        |          |TMCODE1 = 0x01, enter dft   mode.
 * @var CHIP_T::OSC12MTC
 * Offset: 0xF7  OSC12M Temperature Cofficient Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |NTC       |OSC12M Negative   Temperature Coefficient Trim
 * |        |          |Default = 1111 (0%).
 * |        |          |[Design]   control OSC12M pin OSC12M_NTRIM
 * |[7:4]   |PTC       |OSC12M Positive   Temperature Coefficient Trim
 * |        |          |Default = 1111 (0%).
 * |        |          |[Design]   control OSC12M pin OSC12M_PTRIM
 * @var CHIP_T::VCOCPLVL
 * Offset: 0xF8  VCONN OCP Trigger Level Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:4]   |OCPLVLL   |VCONN OCP Low Level   Selection
 * |        |          |VCONN switch overcurrent   state eliminated level setting bits.
 * |[7:6]   |OCPLVLH   |VCONN OCP High Level   Selection
 * |        |          |VCONN switch overcurrent   trigger level setting bits.
 * @var CHIP_T::PWRCR
 * Offset: 0x100  Power Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VCONN1EN  |VCONN to CC1 Power Enable
 * |        |          |0 = Disable VCONN to CC1   power.
 * |        |          |1 = Enable VCONN to CC1   power.
 * |[1]     |VCONN2EN  |VCONN to CC2 Power Enable
 * |        |          |0 = Disable VCONN to CC2 power.
 * |        |          |1 = Enable VCONN to CC2 power.
 * |[2]     |LDO33EN   |LDO 3.3v Power Enable
 * |        |          |0 = Disable LDO 3.3v   power.
 * |        |          |1 = Enable LDO 3.3v   power.
 * |        |          |[Design]   control MEGBC12LDO pin BC12_LDO33_EN
 * |[3]     |VRF256EN  |VREF 2.56v Power Enable
 * |        |          |0 = Disable VREF 2.56v   power.
 * |        |          |1 = Enable VREF 2.56v   power.
 * |        |          |[Design]   control VRFCAPLESS pin ENVRF
 * |[6]     |TMPEN     |Temperature Sensor Enable
 * |        |          |0 = Temperature Sensor is Disabled.
 * |        |          |1 = Temperature Sensor is Enabled.
 * |        |          |[Design]   control TEM5V pin Vtemp_EN
 * |[7]     |GDCPEN    |Gate Driver Charge Pump   Enable Bit
 * |        |          |0 = Gate driver charge pump   Disabled.
 * |        |          |1 = Gate driver charge pump   Enabled.
 * |        |          |[Design]   control GATEDRIVER pin EN0
 * @var CHIP_T::RSTCR
 * Offset: 0x101  Reset Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SWRST     |Software Reset Bit
 * |        |          |When write 1 to SWRST, chip will be reset   and keep ?? period to reset whole chip
 * |        |          |Then this bit will be auto clear by   hardware
 * |        |          |When doing software reset,   the eFUSE will be reload again? (TBD)
 * |[1]     |ADCRST    |ADC Reset Bit
 * |        |          |0 = ADC macro no reset
 * |        |          |1 = ADC macro is reset
 * |        |          |[Design]   control ADC12B600K pin RESETB_NM
 * |[2]     |ADCCRST   |ADC Calibration Reset Bit
 * |        |          |0 = ADC macro calibration result no reset
 * |        |          |1 = ADC macro calibration result is   reset
 * |        |          |[Design]   control ADC12B600K pin RESETB
 * @var CHIP_T::RSTSTS
 * Offset: 0x102  Reset Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PORF      |POR Reset   Flag
 * |        |          |The POR reset   flag is set by the u201CReset Signalu201D from the Power-on Reset (POR) Controller to   indicate the previous reset source.
 * |        |          |0 = No reset   from POR.
 * |        |          |1 = Power-on   Reset (POR) had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to   clear this bit to 0.
 * |[1]     |PINRF     |NRESET Pin   Reset Flag
 * |        |          |The nRESET   pin reset flag is set by the u201CReset Signalu201D from the nRESET Pin to indicate the   previous reset source.
 * |        |          |0 = No reset   from nRESET pin.
 * |        |          |1 = Pin nRESET   had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[2]     |SWRF      |Software   Reset Flag
 * |        |          |The software   reset flag is set by hardware if user writes SWRST (RSTCR [0]) 1 to reset   Cortex-M23 core and Flash Memory Controller (FMC).
 * |        |          |0 = No reset   from software.
 * |        |          |1 = chip   reset by software setting SWRST to 1.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * @var CHIP_T::PEIE
 * Offset: 0x103  Protect Event Interrupt Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |OVPIE     |Over   Voltage Protect Interrupt Flag
 * |        |          |0 = Over   Voltage protect interrupt Disabled.
 * |        |          |1 = Over Voltage protect   interrupt Enabled.
 * |[1]     |UVPIE     |Under   Voltage Protect Interrupt Flag
 * |        |          |0 = Under   voltage protect interrupt Disabled.
 * |        |          |1 = Under voltage protect   interrupt Enabled.
 * |[2]     |OCPIE     |Overcurrent   Protect Interrupt Flag
 * |        |          |0 = Overcurrent   protect interrupt Disabled.
 * |        |          |1 = Overcurrent protect   interrupt Enabled.
 * |[3]     |NTCPIE    |NTC   Protect Interrupt Flag
 * |        |          |0 = NTC   protect interrupt Disabled.
 * |        |          |1 = NTC protect interrupt Enabled.
 * |[4]     |CCOVPIE   |CC Pin   Over Voltage Protect Interrupt Flag
 * |        |          |0 = CC pins   over Voltage protect interrupt Disabled.
 * |        |          |1 = CC pins over Voltage   protect interrupt Enabled.
 * |[5]     |VCOCPIE   |VCONN   Overcurrent Protect Interrupt Flag
 * |        |          |0 = VCONN overcurrent   protect interrupt Disabled .
 * |        |          |1 = VCONN overcurrent protect   interrupt Enabled.
 * @var CHIP_T::PORCTL
 * Offset: 0x104  Power on Reset Controll Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |POROFF    |Power-on   Reset Disable Bit
 * |        |          |When powered   on, the POR circuit generates a reset signal to reset the whole chip   function, but noise on the power may cause the POR active again
 * |        |          |User can   disable internal POR circuit to avoid unpredictable noise to cause RTC reset   by writing 0x5A to this field.
 * |        |          |The POR   function will be active again when this field is set to another value or chip   is reset by other reset source, including:
 * |        |          |nRESET, and   the software-chip reset function.
 * @var CHIP_T::PFCCTL
 * Offset: 0x105  PFC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |PFCCLS    |PFC Current Level Selection
 * |        |          |00 = PFC current level 0. (source current 1 mA)
 * |        |          |01 = PFC current level 1. (source current 0.7 mA)
 * |        |          |10 = PFC current level 2. (source current 0.5 mA)
 * |        |          |11 = PFC current level 3. (source current 0.25 mA)
 * |        |          |[Design] Connect to MEGPMu2019s V2I_1mA_CLEVEL.
 * |[2]     |PFCEN     |PFC Current Enable
 * |        |          |0 = PFC current Disabled.
 * |        |          |1 = PFC current Enabled.
 * |        |          |[Design]   Connect to MEGPMu2019s V2I_1mA_EN .
 * @var CHIP_T::DISCCTL
 * Offset: 0x106  Discharge Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VBDISC1   |VBUS Discharge Enable 1   Bit
 * |        |          |0 = VBUS 1st discharge MOS Disabled. (1st discharge   ability is 15mA)
 * |        |          |1 = VBUS 1st discharge MOS Enabled. (1st discharge   ability is 15mA)
 * |        |          |[Design] logic   combinational control MEGRDIVSWu2019s VBUSDISC0
 * |[1]     |VBDISC2   |VBUS Force Discharge   Level
 * |        |          |0 = VBUS 2nd discharge MOS Disabled. (2nd discharge   ability is 15mA)
 * |        |          |1 = VBUS 2nd discharge MOS Enabled. (2nd discharge   ability is 15mA)
 * |        |          |[Design]   logic combinational control MEGRDIVSWu2019s VBUSDISC1
 * |[2]     |VBDISC3   |VBUS Bleed Discharge   Enable Bit
 * |        |          |0 = VBUS 3rd discharge MOS Disabled. (3rd discharge   ability is 20mA)
 * |        |          |1 = VBUS 3rd discharge MOS Enabled. (3rd discharge   ability is 20mA)
 * |        |          |[Design]   logic combinational control MEGRDIVSWu2019s VBUSDISC2
 * |[3]     |VBDISC4   |VBUS Bleed Discharge   Level
 * |        |          |0 = VBUS 4th discharge MOS Disabled. (4th discharge   ability is 60mA)
 * |        |          |1 = VBUS 4th discharge MOS Enabled. (4th discharge   ability is 60mA)
 * |        |          |[Design]   logic combinational control MEGRDIVSWu2019s VBUSDISC3
 * |[4]     |VBGAEN    |VBUS Gate Enable Bit
 * |        |          |0 = VBUS gate driver Disabled.
 * |        |          |1 = VBUS gate driver Enabled.
 * |        |          |Note: Before set VBGAEN = 1, user   should enable GDCPEN (PWRCR[7]) = 1 and wait 1ms stable time.
 * |        |          |[Design]   control GATEDRIVER pin Gate2_EN
 * |[5]     |VINGAEN   |VIN Gate Enable Bit
 * |        |          |0 = VIN gate driver   Disabled.
 * |        |          |1 = VIN gate driver Enabled.
 * |        |          |Note: Before set VINGAEN = 1,   user should enable GDCPEN (PWRCR[7]) = 1 and wait 1ms stable time.
 * |        |          |[Design]   control GATEDRIVER pin Gate1_EN
 * |[6]     |FOFFVBUS  |Force Off VBUS Enable Bit
 * |        |          |When force off VBUS enable,   if the pin FORCE_OFF_VBUS go high, the VBUS_GATE will go low
 * |        |          |When force off   VBUS disable, VBUS_GATE is contorlled by VBGAEN ( DISCCTL[4])
 * |        |          |0 = Force off VBUS Disabled.
 * |        |          |1 = Force off VBUS Enabled.
 * |[7]     |CCOVPEN   |CC Pin OVP Enable Bit
 * |        |          |0 = CC Pin OVP Disabled.
 * |        |          |1 = CC Pin OVP Enabled.
 * |        |          |[Design]   control LDOUHV40MA pin ????
 * @var CHIP_T::VINDCTL
 * Offset: 0x107  VIN Discharge Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VINBDEN   |VIN Bleed Discharge   Enable Bit
 * |        |          |0 = VIN bleed discharge MOS Disabled.
 * |        |          |1 = VIN bleed discharge MOS   Enabled.
 * |        |          |[Design] logic   combinational control MEGRDIVSWu2019s BDISC0 & BDISC1
 * |[1]     |VINBDLVL  |VIN Bleed Discharge Level
 * |        |          |0 = VIN bleed discharge low   ability. (MOS current is 60 mA)
 * |        |          |1 = VIN bleed discharge high   ability. (MOS current is 80 mA)
 * |        |          |[Design] logic   combinational control MEGRDIVSWu2019s BDISC0 & BDISC1
 */
#define NPD48_PEIF         0x0016           /*!< [0x0016] Protect Event Interrupt Flag                                     */
#define NPD48_PEGCTL       0x00c8           /*!< [0x00c8] Protect Event Gate Control Register                              */
#define NPD48_TRIMDB0      0x00d0           /*!< [0x00d0] Trim Data Byte 0                                                 */
#define NPD48_TRIMDB1      0x00d1           /*!< [0x00d1] Trim Data Byte 1                                                 */
#define NPD48_TRIMDB2      0x00d2           /*!< [0x00d2] Trim Data Byte 2                                                 */
#define NPD48_TRIMDB3      0x00d3           /*!< [0x00d3] Trim Data Byte 3                                                 */
#define NPD48_TRIMDB4      0x00d4           /*!< [0x00d4] Trim Data Byte 4                                                 */
#define NPD48_TRIMDB5      0x00d5           /*!< [0x00d5] Trim Data Byte 5                                                 */
#define NPD48_TRIMDB6      0x00d6           /*!< [0x00d6] Trim Data Byte 6                                                 */
#define NPD48_TRIMDB7      0x00d7           /*!< [0x00d7] Trim Data Byte 7                                                 */
#define NPD48_GDDBGCR      0x00f0           /*!< [0x00f0] Gate Driver Debug Mode Contorl Register                          */
#define NPD48_GDCR         0x00f1           /*!< [0x00f1] Gate Driver Contorl Register                                     */
#define NPD48_TRIMTMD      0x00f2           /*!< [0x00f2] Trim Test Mode Register                                          */
#define NPD48_IOPUDCR      0x00f3           /*!< [0x00f3] IO Pull-up and Pull-down Control Register                        */
#define NPD48_ATESTMD      0x00f4           /*!< [0x00f4] Alpha Test Mode Register                                         */
#define NPD48_WRPROT       0x00f5           /*!< [0x00f5] Write Protect Control Register                                   */
#define NPD48_TMCODE0      0x00f5           /*!< [0x00f5] Test Mode Entry Code 0 Register                                  */
#define NPD48_TMCODE1      0x00f6           /*!< [0x00f6] Test Mode Entry Code 1 Register                                  */
#define NPD48_OSC12MTC     0x00f7           /*!< [0x00f7] OSC12M Temperature Cofficient Control Register                   */
#define NPD48_VCOCPLVL     0x00f8           /*!< [0x00f8] VCONN OCP Trigger Level Control Register                         */
#define NPD48_CLKTMD       0x00fd           /*!< [0x00fd] Clock Test Mode Register                                         */
#define NPD48_PORCTL       0x00fe           /*!< [0x00fe] Power on Reset Controll Register                                 */
#define NPD48_PWRCR        0x0100           /*!< [0x0100] Power Control Register                                           */
#define NPD48_RSTCR        0x0101           /*!< [0x0101] Reset Control Register                                           */
#define NPD48_RSTSTS       0x0102           /*!< [0x0102] Reset Status Register                                            */
#define NPD48_PEIE         0x0103           /*!< [0x0103] Protect Event Interrupt Enable                                   */
#define NPD48_PECTL        0x0104           /*!< [0x0104] Protect Event Control Register                                   */
#define NPD48_PFCCTL       0x0105           /*!< [0x0105] PFC Control Register                                             */
#define NPD48_DISCCTL      0x0106           /*!< [0x0106] Discharge Control Register                                       */
#define NPD48_CLKCTL       0x0107           /*!< [0x0107] Clock Control Register                                           */

/**
    @addtogroup CHIP_CONST CHIP Bit Field Definition
    Constant Definitions for CHIP Controller
@{ */

#define NPD48_PEIF_OVPIF_Pos              (0)                                               /*!< CHIP_T::PEIF: OVPIF Position           */
#define NPD48_PEIF_OVPIF_Msk              (0x1ul << NPD48_PEIF_OVPIF_Pos)                   /*!< CHIP_T::PEIF: OVPIF Mask               */

#define NPD48_PEIF_UVPIF_Pos              (1)                                               /*!< CHIP_T::PEIF: UVPIF Position           */
#define NPD48_PEIF_UVPIF_Msk              (0x1ul << NPD48_PEIF_UVPIF_Pos)                   /*!< CHIP_T::PEIF: UVPIF Mask               */

#define NPD48_PEIF_OCPIF_Pos              (2)                                               /*!< CHIP_T::PEIF: OCPIF Position           */
#define NPD48_PEIF_OCPIF_Msk              (0x1ul << NPD48_PEIF_OCPIF_Pos)                   /*!< CHIP_T::PEIF: OCPIF Mask               */

#define NPD48_PEIF_NTCPIF_Pos             (3)                                               /*!< CHIP_T::PEIF: NTCPIF Position          */
#define NPD48_PEIF_NTCPIF_Msk             (0x1ul << NPD48_PEIF_NTCPIF_Pos)                  /*!< CHIP_T::PEIF: NTCPIF Mask              */

#define NPD48_PEIF_CCOVPIF_Pos            (4)                                               /*!< CHIP_T::PEIF: CCOVPIF Position         */
#define NPD48_PEIF_CCOVPIF_Msk            (0x1ul << NPD48_PEIF_CCOVPIF_Pos)                 /*!< CHIP_T::PEIF: CCOVPIF Mask             */

#define NPD48_PEIF_VCOCPIF_Pos            (5)                                               /*!< CHIP_T::PEIF: VCOCPIF Position         */
#define NPD48_PEIF_VCOCPIF_Msk            (0x1ul << NPD48_PEIF_VCOCPIF_Pos)                 /*!< CHIP_T::PEIF: VCOCPIF Mask             */

#define NPD48_PEIF_COVPIF_Pos             (6)                                               /*!< CHIP_T::PEIF: COVPIF Position          */
#define NPD48_PEIF_COVPIF_Msk             (0x1ul << NPD48_PEIF_COVPIF_Pos)                  /*!< CHIP_T::PEIF: COVPIF Mask              */

#define NPD48_PEIF_ITPIF_Pos              (7)                                               /*!< CHIP_T::PEIF: ITPIF Position           */
#define NPD48_PEIF_ITPIF_Msk              (0x1ul << NPD48_PEIF_ITPIF_Pos)                   /*!< CHIP_T::PEIF: ITPIF Mask               */

#define NPD48_TRIMDB0_TRIMDB0_Pos         (0)                                               /*!< CHIP_T::TRIMDB0: TRIMDB0 Position      */
#define NPD48_TRIMDB0_TRIMDB0_Msk         (0xfful << NPD48_TRIMDB0_TRIMDB0_Pos)             /*!< CHIP_T::TRIMDB0: TRIMDB0 Mask          */

#define NPD48_TRIMDB1_TRIMDB1_Pos         (0)                                               /*!< CHIP_T::TRIMDB1: TRIMDB1 Position      */
#define NPD48_TRIMDB1_TRIMDB1_Msk         (0xfful << NPD48_TRIMDB1_TRIMDB1_Pos)             /*!< CHIP_T::TRIMDB1: TRIMDB1 Mask          */

#define NPD48_TRIMDB2_TRIMDB2_Pos         (0)                                               /*!< CHIP_T::TRIMDB2: TRIMDB2 Position      */
#define NPD48_TRIMDB2_TRIMDB2_Msk         (0xfful << NPD48_TRIMDB2_TRIMDB2_Pos)             /*!< CHIP_T::TRIMDB2: TRIMDB2 Mask          */

#define NPD48_TRIMDB3_TRIMDB3_Pos         (0)                                               /*!< CHIP_T::TRIMDB3: TRIMDB3 Position      */
#define NPD48_TRIMDB3_TRIMDB3_Msk         (0xfful << NPD48_TRIMDB3_TRIMDB3_Pos)             /*!< CHIP_T::TRIMDB3: TRIMDB3 Mask          */

#define NPD48_TRIMDB4_TRIMDB4_Pos         (0)                                               /*!< CHIP_T::TRIMDB4: TRIMDB4 Position      */
#define NPD48_TRIMDB4_TRIMDB4_Msk         (0xfful << NPD48_TRIMDB4_TRIMDB4_Pos)             /*!< CHIP_T::TRIMDB4: TRIMDB4 Mask          */

#define NPD48_TRIMDB5_TRIMDB5_Pos         (0)                                               /*!< CHIP_T::TRIMDB5: TRIMDB5 Position      */
#define NPD48_TRIMDB5_TRIMDB5_Msk         (0xfful << NPD48_TRIMDB5_TRIMDB5_Pos)             /*!< CHIP_T::TRIMDB5: TRIMDB5 Mask          */

#define NPD48_TRIMDB6_TRIMDB6_Pos         (0)                                               /*!< CHIP_T::TRIMDB6: TRIMDB6 Position      */
#define NPD48_TRIMDB6_TRIMDB6_Msk         (0xfful << NPD48_TRIMDB6_TRIMDB6_Pos)             /*!< CHIP_T::TRIMDB6: TRIMDB6 Mask          */

#define NPD48_TRIMDB7_TRIMDB7_Pos         (0)                                               /*!< CHIP_T::TRIMDB7: TRIMDB7 Position      */
#define NPD48_TRIMDB7_TRIMDB7_Msk         (0xfful << NPD48_TRIMDB7_TRIMDB7_Pos)             /*!< CHIP_T::TRIMDB7: TRIMDB7 Mask          */

#define NPD48_ATESTMD_PA10OMD_Pos         (0)                                               /*!< CHIP_T::ATESTMD: PA10OMD Position      */
#define NPD48_ATESTMD_PA10OMD_Msk         (0x1ul << NPD48_ATESTMD_PA10OMD_Pos)              /*!< CHIP_T::ATESTMD: PA10OMD Mask          */

#define NPD48_ATESTMD_PA11OMD_Pos         (1)                                               /*!< CHIP_T::ATESTMD: PA11OMD Position      */
#define NPD48_ATESTMD_PA11OMD_Msk         (0x1ul << NPD48_ATESTMD_PA11OMD_Pos)              /*!< CHIP_T::ATESTMD: PA11OMD Mask          */

#define NPD48_ATESTMD_VREFTM_Pos          (2)                                               /*!< CHIP_T::ATESTMD: VREFTM Position       */
#define NPD48_ATESTMD_VREFTM_Msk          (0x1ul << NPD48_ATESTMD_VREFTM_Pos)               /*!< CHIP_T::ATESTMD: VREFTM Mask           */

#define NPD48_ATESTMD_VINGA2EN_Pos        (3)                                               /*!< CHIP_T::ATESTMD: VINGA2EN Position     */
#define NPD48_ATESTMD_VINGA2EN_Msk        (0x1ul << NPD48_ATESTMD_VINGA2EN_Pos)             /*!< CHIP_T::ATESTMD: VINGA2EN Mask         */

#define NPD48_ATESTMD_PGATMEN_Pos         (4)                                               /*!< CHIP_T::ATESTMD: PGATMEN Position      */
#define NPD48_ATESTMD_PGATMEN_Msk         (0x1ul << NPD48_ATESTMD_PGATMEN_Pos)              /*!< CHIP_T::ATESTMD: PGATMEN Mask          */

#define NPD48_ATESTMD_LDO18TM_Pos         (5)                                               /*!< CHIP_T::ATESTMD: LDO18TM Position      */
#define NPD48_ATESTMD_LDO18TM_Msk         (0x1ul << NPD48_ATESTMD_LDO18TM_Pos)              /*!< CHIP_T::ATESTMD: LDO18TM Mask          */

#define NPD48_WRPROT_WRPROT_Pos           (0)                                               /*!< CHIP_T::WRPROT: WRPROT Position        */
#define NPD48_WRPROT_WRPROT_Msk           (0xfful << NPD48_WRPROT_WRPROT_Pos)               /*!< CHIP_T::WRPROT: WRPROT Mask            */

#define NPD48_TMCODE0_TMCODE0_Pos         (0)                                               /*!< CHIP_T::TMCODE0: TMCODE0 Position      */
#define NPD48_TMCODE0_TMCODE0_Msk         (0xfful << NPD48_TMCODE0_TMCODE0_Pos)             /*!< CHIP_T::TMCODE0: TMCODE0 Mask          */

#define NPD48_TMCODE1_TMECODE1_Pos        (0)                                               /*!< CHIP_T::TMCODE1: TMECODE1 Position     */
#define NPD48_TMCODE1_TMECODE1_Msk        (0xfful << NPD48_TMCODE1_TMECODE1_Pos)            /*!< CHIP_T::TMCODE1: TMECODE1 Mask         */

#define NPD48_OSC12MTC_NTC_Pos            (0)                                               /*!< CHIP_T::OSC12MTC: NTC Position         */
#define NPD48_OSC12MTC_NTC_Msk            (0xful << NPD48_OSC12MTC_NTC_Pos)                 /*!< CHIP_T::OSC12MTC: NTC Mask             */

#define NPD48_OSC12MTC_PTC_Pos            (4)                                               /*!< CHIP_T::OSC12MTC: PTC Position         */
#define NPD48_OSC12MTC_PTC_Msk            (0xful << NPD48_OSC12MTC_PTC_Pos)                 /*!< CHIP_T::OSC12MTC: PTC Mask             */

#define NPD48_VCOCPLVL_OCPLVLL_Pos        (4)                                               /*!< CHIP_T::VCOCPLVL: OCPLVLL Position     */
#define NPD48_VCOCPLVL_OCPLVLL_Msk        (0x3ul << NPD48_VCOCPLVL_OCPLVLL_Pos)             /*!< CHIP_T::VCOCPLVL: OCPLVLL Mask         */

#define NPD48_VCOCPLVL_OCPLVLH_Pos        (6)                                               /*!< CHIP_T::VCOCPLVL: OCPLVLH Position     */
#define NPD48_VCOCPLVL_OCPLVLH_Msk        (0x3ul << NPD48_VCOCPLVL_OCPLVLH_Pos)             /*!< CHIP_T::VCOCPLVL: OCPLVLH Mask         */

#define NPD48_PORCTL_POROFF_Pos           (0)                                               /*!< CHIP_T::PORCTL: POROFF Position        */
#define NPD48_PORCTL_POROFF_Msk           (0xfful << NPD48_PORCTL_POROFF_Pos)               /*!< CHIP_T::PORCTL: POROFF Mask            */

#define NPD48_PWRCR_LDO33EN_Pos           (2)                                               /*!< CHIP_T::PWRCR: LDO33EN Position        */
#define NPD48_PWRCR_LDO33EN_Msk           (0x1ul << NPD48_PWRCR_LDO33EN_Pos)                /*!< CHIP_T::PWRCR: LDO33EN Mask            */

#define NPD48_PWRCR_VRF256EN_Pos          (3)                                               /*!< CHIP_T::PWRCR: VRF256EN Position       */
#define NPD48_PWRCR_VRF256EN_Msk          (0x1ul << NPD48_PWRCR_VRF256EN_Pos)               /*!< CHIP_T::PWRCR: VRF256EN Mask           */

#define NPD48_PWRCR_NTCEN_Pos             (4)                                               /*!< CHIP_T::PWRCR: NTCEN Position          */
#define NPD48_PWRCR_NTCEN_Msk             (0x1ul << NPD48_PWRCR_NTCEN_Pos)                  /*!< CHIP_T::PWRCR: NTCEN Mask              */

#define NPD48_PWRCR_TMPEN_Pos             (6)                                               /*!< CHIP_T::PWRCR: TMPEN Position          */
#define NPD48_PWRCR_TMPEN_Msk             (0x1ul << NPD48_PWRCR_TMPEN_Pos)                  /*!< CHIP_T::PWRCR: TMPEN Mask              */

#define NPD48_PWRCR_GDCPEN_Pos            (7)                                               /*!< CHIP_T::PWRCR: GDCPEN Position         */
#define NPD48_PWRCR_GDCPEN_Msk            (0x1ul << NPD48_PWRCR_GDCPEN_Pos)                 /*!< CHIP_T::PWRCR: GDCPEN Mask             */

#define NPD48_RSTCR_CHIPRST_Pos           (0)                                               /*!< CHIP_T::RSTCR: CHIPRST Position        */
#define NPD48_RSTCR_CHIPRST_Msk           (0x1ul << NPD48_RSTCR_CHIPRST_Pos)                /*!< CHIP_T::RSTCR: CHIPRST Mask            */

#define NPD48_RSTCR_ADCRST_Pos            (1)                                               /*!< CHIP_T::RSTCR: ADCRST Position         */
#define NPD48_RSTCR_ADCRST_Msk            (0x1ul << NPD48_RSTCR_ADCRST_Pos)                 /*!< CHIP_T::RSTCR: ADCRST Mask             */

#define NPD48_RSTCR_ADCCRST_Pos           (2)                                               /*!< CHIP_T::RSTCR: ADCCRST Position        */
#define NPD48_RSTCR_ADCCRST_Msk           (0x1ul << NPD48_RSTCR_ADCCRST_Pos)                /*!< CHIP_T::RSTCR: ADCCRST Mask            */

#define NPD48_RSTSTS_PORF_Pos             (0)                                               /*!< CHIP_T::RSTSTS: PORF Position          */
#define NPD48_RSTSTS_PORF_Msk             (0x1ul << NPD48_RSTSTS_PORF_Pos)                  /*!< CHIP_T::RSTSTS: PORF Mask              */

#define NPD48_RSTSTS_PINRF_Pos            (1)                                               /*!< CHIP_T::RSTSTS: PINRF Position         */
#define NPD48_RSTSTS_PINRF_Msk            (0x1ul << NPD48_RSTSTS_PINRF_Pos)                 /*!< CHIP_T::RSTSTS: PINRF Mask             */

#define NPD48_RSTSTS_CHIPRF_Pos           (2)                                               /*!< CHIP_T::RSTSTS: CHIPRF Position        */
#define NPD48_RSTSTS_CHIPRF_Msk           (0x1ul << NPD48_RSTSTS_CHIPRF_Pos)                /*!< CHIP_T::RSTSTS: CHIPRF Mask            */

#define NPD48_PEIE_OVPIE_Pos              (0)                                               /*!< CHIP_T::PEIE: OVPIE Position           */
#define NPD48_PEIE_OVPIE_Msk              (0x1ul << NPD48_PEIE_OVPIE_Pos)                   /*!< CHIP_T::PEIE: OVPIE Mask               */

#define NPD48_PEIE_UVPIE_Pos              (1)                                               /*!< CHIP_T::PEIE: UVPIE Position           */
#define NPD48_PEIE_UVPIE_Msk              (0x1ul << NPD48_PEIE_UVPIE_Pos)                   /*!< CHIP_T::PEIE: UVPIE Mask               */

#define NPD48_PEIE_OCPIE_Pos              (2)                                               /*!< CHIP_T::PEIE: OCPIE Position           */
#define NPD48_PEIE_OCPIE_Msk              (0x1ul << NPD48_PEIE_OCPIE_Pos)                   /*!< CHIP_T::PEIE: OCPIE Mask               */

#define NPD48_PEIE_NTCPIE_Pos             (3)                                               /*!< CHIP_T::PEIE: NTCPIE Position          */
#define NPD48_PEIE_NTCPIE_Msk             (0x1ul << NPD48_PEIE_NTCPIE_Pos)                  /*!< CHIP_T::PEIE: NTCPIE Mask              */

#define NPD48_PEIE_CCOVPIE_Pos            (4)                                               /*!< CHIP_T::PEIE: CCOVPIE Position         */
#define NPD48_PEIE_CCOVPIE_Msk            (0x1ul << NPD48_PEIE_CCOVPIE_Pos)                 /*!< CHIP_T::PEIE: CCOVPIE Mask             */

#define NPD48_PEIE_VCOCPIE_Pos            (5)                                               /*!< CHIP_T::PEIE: VCOCPIE Position         */
#define NPD48_PEIE_VCOCPIE_Msk            (0x1ul << NPD48_PEIE_VCOCPIE_Pos)                 /*!< CHIP_T::PEIE: VCOCPIE Mask             */

#define NPD48_PEIE_COVPIE_Pos             (6)                                               /*!< CHIP_T::PEIE: COVPIE Position          */
#define NPD48_PEIE_COVPIE_Msk             (0x1ul << NPD48_PEIE_COVPIE_Pos)                  /*!< CHIP_T::PEIE: COVPIE Mask              */

#define NPD48_PEIE_ITPIE_Pos              (7)                                               /*!< CHIP_T::PEIE: ITPIE Position           */
#define NPD48_PEIE_ITPIE_Msk              (0x1ul << NPD48_PEIE_ITPIE_Pos)                   /*!< CHIP_T::PEIE: ITPIE Mask               */

#define NPD48_PECTL_OVPEN_Pos             (0)                                               /*!< CHIP_T::PECTL: OVPEN Position          */
#define NPD48_PECTL_OVPEN_Msk             (0x1ul << NPD48_PECTL_OVPEN_Pos)                  /*!< CHIP_T::PECTL: OVPEN Mask              */

#define NPD48_PECTL_UVPEN_Pos             (1)                                               /*!< CHIP_T::PECTL: UVPEN Position          */
#define NPD48_PECTL_UVPEN_Msk             (0x1ul << NPD48_PECTL_UVPEN_Pos)                  /*!< CHIP_T::PECTL: UVPEN Mask              */

#define NPD48_PECTL_OCPEN_Pos             (2)                                               /*!< CHIP_T::PECTL: OCPEN Position          */
#define NPD48_PECTL_OCPEN_Msk             (0x1ul << NPD48_PECTL_OCPEN_Pos)                  /*!< CHIP_T::PECTL: OCPEN Mask              */

#define NPD48_PECTL_NTCPEN_Pos            (3)                                               /*!< CHIP_T::PECTL: NTCPEN Position         */
#define NPD48_PECTL_NTCPEN_Msk            (0x1ul << NPD48_PECTL_NTCPEN_Pos)                 /*!< CHIP_T::PECTL: NTCPEN Mask             */

#define NPD48_PECTL_CCOVPEN_Pos           (4)                                               /*!< CHIP_T::PECTL: CCOVPEN Position        */
#define NPD48_PECTL_CCOVPEN_Msk           (0x1ul << NPD48_PECTL_CCOVPEN_Pos)                /*!< CHIP_T::PECTL: CCOVPEN Mask            */

#define NPD48_PECTL_VCOCPEN_Pos           (5)                                               /*!< CHIP_T::PECTL: VCOCPEN Position        */
#define NPD48_PECTL_VCOCPEN_Msk           (0x1ul << NPD48_PECTL_VCOCPEN_Pos)                /*!< CHIP_T::PECTL: VCOCPEN Mask            */

#define NPD48_PECTL_COVPEN_Pos            (6)                                               /*!< CHIP_T::PECTL: COVPEN Position         */
#define NPD48_PECTL_COVPEN_Msk            (0x1ul << NPD48_PECTL_COVPEN_Pos)                 /*!< CHIP_T::PECTL: COVPEN Mask             */

#define NPD48_PECTL_ITPEN_Pos             (7)                                               /*!< CHIP_T::PECTL: ITPEN Position          */
#define NPD48_PECTL_ITPEN_Msk             (0x1ul << NPD48_PECTL_ITPEN_Pos)                  /*!< CHIP_T::PECTL: ITPEN Mask              */

#define NPD48_CLKTMD_CKOEN_Pos            (0)                                               /*!< CHIP_T::CLKTMD: CKOEN Position         */
#define NPD48_CLKTMD_CKOEN_Msk            (0x1ul << NPD48_CLKTMD_CKOEN_Pos)                 /*!< CHIP_T::CLKTMD: CKOEN Mask             */

#define NPD48_CLKTMD_OSC12MGEN_Pos        (1)                                               /*!< CHIP_T::CLKTMD: OSC12MGEN Position     */
#define NPD48_CLKTMD_OSC12MGEN_Msk        (0x1ul << NPD48_CLKTMD_OSC12MGEN_Pos)             /*!< CHIP_T::CLKTMD: OSC12MGEN Mask         */

#define NPD48_CLKTMD_CKOSEL_Pos           (7)                                               /*!< CHIP_T::CLKTMD: CKOSEL Position        */
#define NPD48_CLKTMD_CKOSEL_Msk           (0x1ul << NPD48_CLKTMD_CKOSEL_Pos)                /*!< CHIP_T::CLKTMD: CKOSEL Mask            */

#define NPD48_PORCTL_POROFF_Pos           (0)                                               /*!< CHIP_T::PORCTL: POROFF Position        */
#define NPD48_PORCTL_POROFF_Msk           (0xfful << NPD48_PORCTL_POROFF_Pos)               /*!< CHIP_T::PORCTL: POROFF Mask            */

#define NPD48_PFCCTL_PFCCLS_Pos           (0)                                               /*!< CHIP_T::PFCCTL: PFCCLS Position        */
#define NPD48_PFCCTL_PFCCLS_Msk           (0x3ul << NPD48_PFCCTL_PFCCLS_Pos)                /*!< CHIP_T::PFCCTL: PFCCLS Mask            */

#define NPD48_PFCCTL_PFCEN_Pos            (2)                                               /*!< CHIP_T::PFCCTL: PFCEN Position         */
#define NPD48_PFCCTL_PFCEN_Msk            (0x1ul << NPD48_PFCCTL_PFCEN_Pos)                 /*!< CHIP_T::PFCCTL: PFCEN Mask             */

#define NPD48_DISCCTL_VBDISC1_Pos         (0)                                               /*!< CHIP_T::DISCCTL: VBDISC1 Position      */
#define NPD48_DISCCTL_VBDISC1_Msk         (0x1ul << NPD48_DISCCTL_VBDISC1_Pos)              /*!< CHIP_T::DISCCTL: VBDISC1 Mask          */

#define NPD48_DISCCTL_VBDISC2_Pos         (1)                                               /*!< CHIP_T::DISCCTL: VBDISC2 Position      */
#define NPD48_DISCCTL_VBDISC2_Msk         (0x1ul << NPD48_DISCCTL_VBDISC2_Pos)              /*!< CHIP_T::DISCCTL: VBDISC2 Mask          */

#define NPD48_DISCCTL_VBDISC3_Pos         (2)                                               /*!< CHIP_T::DISCCTL: VBDISC3 Position      */
#define NPD48_DISCCTL_VBDISC3_Msk         (0x1ul << NPD48_DISCCTL_VBDISC3_Pos)              /*!< CHIP_T::DISCCTL: VBDISC3 Mask          */

#define NPD48_DISCCTL_VBDISC4_Pos         (3)                                               /*!< CHIP_T::DISCCTL: VBDISC4 Position      */
#define NPD48_DISCCTL_VBDISC4_Msk         (0x1ul << NPD48_DISCCTL_VBDISC4_Pos)              /*!< CHIP_T::DISCCTL: VBDISC4 Mask          */

#define NPD48_DISCCTL_VBGAEN_Pos          (4)                                               /*!< CHIP_T::DISCCTL: VBGAEN Position       */
#define NPD48_DISCCTL_VBGAEN_Msk          (0x1ul << NPD48_DISCCTL_VBGAEN_Pos)               /*!< CHIP_T::DISCCTL: VBGAEN Mask           */

#define NPD48_DISCCTL_VINGAEN_Pos         (5)                                               /*!< CHIP_T::DISCCTL: VINGAEN Position      */
#define NPD48_DISCCTL_VINGAEN_Msk         (0x1ul << NPD48_DISCCTL_VINGAEN_Pos)              /*!< CHIP_T::DISCCTL: VINGAEN Mask          */

#define NPD48_DISCCTL_FOFFVBUS_Pos        (6)                                               /*!< CHIP_T::DISCCTL: FOFFVBUS Position     */
#define NPD48_DISCCTL_FOFFVBUS_Msk        (0x1ul << NPD48_DISCCTL_FOFFVBUS_Pos)             /*!< CHIP_T::DISCCTL: FOFFVBUS Mask         */

#define NPD48_DISCCTL_CCOVPEN_Pos         (7)                                               /*!< CHIP_T::DISCCTL: CCOVPEN Position      */
#define NPD48_DISCCTL_CCOVPEN_Msk         (0x1ul << NPD48_DISCCTL_CCOVPEN_Pos)              /*!< CHIP_T::DISCCTL: CCOVPEN Mask          */

#define NPD48_CLKCTL_ADCKEN_Pos           (0)                                               /*!< CHIP_T::CLKCTL: ADCKEN Position        */
#define NPD48_CLKCTL_ADCKEN_Msk           (0x1ul << NPD48_CLKCTL_ADCKEN_Pos)                /*!< CHIP_T::CLKCTL: ADCKEN Mask            */

#define NPD48_CLKCTL_CLKSTOP_Pos          (2)                                               /*!< CHIP_T::CLKCTL: CLKSTOP Position       */
#define NPD48_CLKCTL_CLKSTOP_Msk          (0x1ul << NPD48_CLKCTL_CLKSTOP_Pos)               /*!< CHIP_T::CLKCTL: CLKSTOP Mask           */

#define NPD48_CLKCTL_CLKDIV_Pos           (3)                                               /*!< CHIP_T::CLKCTL: CLKDIV Position        */
#define NPD48_CLKCTL_CLKDIV_Msk           (0x1ul << NPD48_CLKCTL_CLKDIV_Pos)                /*!< CHIP_T::CLKCTL: CLKDIV Mask            */

#define NPD48_CLKCTL_ADCCKSEL_Pos         (5)                                               /*!< CHIP_T::CLKCTL: ADCCKSEL Position      */
#define NPD48_CLKCTL_ADCCKSEL_Msk         (0x1ul << NPD48_CLKCTL_ADCCKSEL_Pos)              /*!< CHIP_T::CLKCTL: ADCCKSEL Mask          */

#define NPD48_CLKCTL_GDCKSEL_Pos          (6)                                               /*!< CHIP_T::CLKCTL: GDCKSEL Position       */
#define NPD48_CLKCTL_GDCKSEL_Msk          (0x3ul << NPD48_CLKCTL_GDCKSEL_Pos)               /*!< CHIP_T::CLKCTL: GDCKSEL Mask           */

/**@}*/ /* CHIP_CONST */
/**@}*/ /* end of CHIP register group */
