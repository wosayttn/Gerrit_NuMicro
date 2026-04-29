/*---------------------- Analog to Digital Converter -------------------------*/
/**
    @addtogroup ADC Analog to Digital Converter(ADC)
    Memory Mapped Structure for ADC Controller
@{ */

/**
 * @var ADC_T::ADCR
 * Offset: 0x108  ADC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADCEN     |ADC Active Enable Bit
 * |        |          |0 = ADC is Disabled.
 * |        |          |1 = ADC start continuous   conversion.
 * |[1]     |SCANMD    |ADC Scan Mode
 * |        |          |0 = ADC scan with round robin priority.
 * |        |          |1 = ADC scan with channel   0/1 highest priority.
 * |        |          |Note: When SCANMD = 1, the channel 0 and   channel 1 are default enable channel with higher priority during ADC channel   scan sequence
 * |        |          |User cannot disable channel 0 and channel 1 by setting ADCCHEN   register
 * @var ADC_T::ADCALCTL
 * Offset: 0x10A  ADC Calibration Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CALST     |ADC Calibration Start Bit
 * |        |          |Write 1 to this bit will make ADC enter   calibration mode and start calibration
 * |        |          |After calibration finish, user need   write 0 to leave calibration mode and enter conversion mode.
 * |        |          |Note 1: This bit is write only, read this will   always get 0.
 * |        |          |Note 2: User should do ADC calibratioh after   chip power-on and any reset from hardware or software.
 * |[1]     |CALF      |ADC Calibration Finish Flag
 * |        |          |0 = ADC calibration not finish .
 * |        |          |1 = ADC calibration finish .
 * |        |          |Write 1 can clear this flag
 * @var ADC_T::ADEXTEND
 * Offset: 0x10B  ADC Extend Sampling Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EXTSMP    |ADC Extend Sampling Period
 * |        |          |The ADC sampling time period setting
 * |        |          |0 = Reserved.
 * |        |          |1 = The ADC conversion will extend 1   ADC_CLK to start conversion after last channel conversion finish.
 * |        |          |u2026
 * |        |          |255 = The ADC conversion will extend   255 ADC_CLK to start conversion after last channel conversion finish.
 * @var ADC_T::ADCCHEN
 * Offset: 0x10C  ADC Channel Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CH0EN     |ADC Channel 0 Enable
 * |        |          |Channel 0 source is VIN / 20 voltage reflect OVP detect source
 * |        |          |0 = Channel 0 Disabled.
 * |        |          |1 = Channel 0 Enabled.
 * |[1]     |CH1EN     |ADC Channel 1 Enable
 * |        |          |Channel 1 source is PGA0 output voltage   reflect OCP detect source
 * |        |          |0 = Channel 1 Disabled.
 * |        |          |1 = Channel 1 Enabled.
 * |[2]     |CH2EN     |ADC Channel 2 Enable
 * |        |          |Channel 2 source is NTC pin reflect   NTCP detect source
 * |        |          |0 = Channel 2 Disabled.
 * |        |          |1 = Channel 2 Enabled.
 * |[3]     |CH3EN     |ADC Channel 3 Enable
 * |        |          |Channel 3 source is Built-in   temperature sensor reflect ITP detect source
 * |        |          |0 = Channel 3 Disabled.
 * |        |          |1 = Channel 3 Enabled.
 * |[4]     |CH4EN     |ADC Channel 4 Enable
 * |        |          |Channel 4 source is external   ADC4 pin
 * |        |          |0 = Channel 4 Disabled.
 * |        |          |1 = Channel 4 Enabled.
 * |[5]     |CH5EN     |ADC Channel 5 Enable
 * |        |          |Channel 5 source is external   ADC5 pin
 * |        |          |0 = Channel 5 Disabled.
 * |        |          |1 = Channel 5 Enabled.
 * |[6]     |CH6EN     |ADC Channel 6 Enable
 * |        |          |Channel 6 source is VIN / 26.67 voltage reflect COVP detect   source
 * |        |          |0 = Channel 6 Disabled.
 * |        |          |1 = Channel 6 Enabled.
 * |[7]     |CH7EN     |ADC Channel 7 Enable
 * |        |          |Channel 7 source is VBUS / 20 voltage
 * |        |          |0 = Channel 7 Disabled.
 * |        |          |1 = Channel 7 Enabled.
 * @var ADC_T::ADCCH0L
 * Offset: 0x110  ADC CH0 Convert Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ADCCH0L   |ADC Channel 0 Data Buffer Low Byte[7:0]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH0H
 * Offset: 0x111  ADC CH0 Convert Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |ADCCH0H   |ADC channel 0 data buffer high byte[11:8]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH1L
 * Offset: 0x112  ADC CH1 Convert Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ADCCH1L   |ADC Channel 1 Data Buffer Low Byte[7:0]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH1H
 * Offset: 0x113  ADC CH1 Convert Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |ADCCH1H   |ADC channel 1 data buffer high byte[11:8]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH2L
 * Offset: 0x114  ADC CH2 Convert Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ADCCH2L   |ADC Channel 2 Data Buffer Low Byte[7:0]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH2H
 * Offset: 0x115  ADC CH2 Convert Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |ADCCH2H   |ADC channel 2 data buffer high byte[11:8]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH3L
 * Offset: 0x116  ADC CH3 Convert Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ADCCH3L   |ADC Channel 3 Data Buffer Low Byte[7:0]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH3H
 * Offset: 0x117  ADC CH3 Convert Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |ADCCH3H   |ADC channel 3 data buffer high byte[11:8]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH4L
 * Offset: 0x118  ADC CH4 Convert Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ADCCH4L   |ADC Channel 4 Data Buffer Low Byte[7:0]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH4H
 * Offset: 0x119  ADC CH4 Convert Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |ADCCH4H   |ADC channel 4 data buffer high byte[11:8]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH5L
 * Offset: 0x11A  ADC CH5 Convert Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ADCCH5L   |ADC Channel 5 Data Buffer Low Byte[7:0]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH5H
 * Offset: 0x11B  ADC CH5 Convert Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |ADCCH5H   |ADC channel 5 data buffer high byte[11:8]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH6L
 * Offset: 0x11C  ADC CH6 Convert Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ADCCH6L   |ADC Channel 6 Data Buffer Low Byte[7:0]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH6H
 * Offset: 0x11D  ADC CH6 Convert Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |ADCCH6H   |ADC channel 6 data buffer high byte[11:8]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH7L
 * Offset: 0x11E  ADC CH7 Convert Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ADCCH7L   |ADC Channel 7 Data Buffer Low Byte[7:0]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::ADCCH7H
 * Offset: 0x11F  ADC CH7 Convert Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |ADCCH7H   |ADC channel 7 data buffer high byte[11:8]
 * |        |          |The ADC conversion result is record in related ADC data buffer.
 * @var ADC_T::OVPCMPL
 * Offset: 0x130  OVP Digital Compare Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |OVPCMPL   |Over Voltage Protect Compare Value Low Byte
 * |        |          |OVPCMPL is over voltage protect compare data buffer low byte[7:0].
 * @var ADC_T::OVPCMPH
 * Offset: 0x131  OVP Digital Compare High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |OVPCMPH   |Over Voltage Protect Compare Value High Byte
 * |        |          |OVPCMPH is over voltage protect compare data buffer high byte[11:8].
 * @var ADC_T::UVPCMPL
 * Offset: 0x132  UVP Digital Compare Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |UVPCMPL   |Under Voltage Protect Compare Value Low Byte
 * |        |          |UVPCMPL is under voltage protect compare data buffer low byte[7:0].
 * @var ADC_T::UVPCMPH
 * Offset: 0x133  UVP Digital Compare High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |UVPCMPH   |Under Voltage Protect Compare Value High Byte
 * |        |          |UVPCMPH is under voltage protect compare data buffer high byte[11:8].
 * @var ADC_T::OCPCMPL
 * Offset: 0x134  OCP Digital Compare Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |OCPCMPL   |Overcurrent Protect Compare Value Low Byte
 * |        |          |OCPCMPL is overcurrent protect compare data buffer low byte[7:0].
 * @var ADC_T::OCPCMPH
 * Offset: 0x135  OCP Digital Compare High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |OCPCMPH   |Overcurrent Protect Compare Value High Byte
 * |        |          |OCPCMPH is overcurrent protect compare data buffer high byte[11:8].
 * @var ADC_T::NTCPCMPL
 * Offset: 0x136  NTCP Digital Compare Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |NTCPCMPL  |NTC Protect Compare Value Low Byte
 * |        |          |NTCPCMPL is NTC protect compare data buffer low byte[7:0].
 * @var ADC_T::NTCPCMPH
 * Offset: 0x137  NTCP Digital Compare High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |NTCPCMPH  |NTC Protect Compare Value High Byte
 * |        |          |NTCPCMPH is NTC protect compare data buffer high byte[11:8].
 * @var ADC_T::COVPCMPL
 * Offset: 0x138  Chip OVP Digital Compare Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |COVPCMPL  |Chip Over Voltage Protect Compare Value Low Byte
 * |        |          |COVPCMPL is over voltage protect compare data buffer low byte[7:0].
 * @var ADC_T::COVPCMPH
 * Offset: 0x139  Chip OVP Digital Compare High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |COVPCMPH  |Chip Over Voltage Protect Compare Value High Byte
 * |        |          |COVPCMPH is over voltage protect compare data buffer high byte[11:8].
 * @var ADC_T::ITPCMPL
 * Offset: 0x13A  Internal Temperature Protect Digital Compare Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ITPCMPL   |Internal Temperature Protect Compare Value Low Byte
 * |        |          |ITPCMPL is internal temperature protect compare data buffer low byte[7:0].
 * @var ADC_T::ITPCMPH
 * Offset: 0x13B  Internal Temperature Protect Digital Compare High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |ITPCMPH   |Internal Temperature Protect Compare Value High Byte
 * |        |          |ITPCMPL is internal temperature protect compare data buffer high byte[11:8].
 */
 
#define NPD48_ADCR         0x0108           /*!< [0x0108] ADC Control Register                                             */
#define NPD48_ADCALCTL     0x010a           /*!< [0x010a] ADC Calibration Control Register                                 */
#define NPD48_ADEXTEND     0x010b           /*!< [0x010b] ADC Extend Sampling Register                                     */
#define NPD48_ADCHEN       0x010c           /*!< [0x010c] ADC Channel Enable Register                                      */

#define NPD48_ADCALDCR     0x010d           /*!< [0x010d] ADC Calibration Debug Control Register                           */
#define NPD48_ADCALDDR     0x010e           /*!< [0x010e] ADC Calibration Debug Data Register                              */
#define NPD48_ADCADJR      0x010f           /*!< [0x010f] ADC Adjust Register                                              */

#define NPD48_ADCCH0L      0x0110           /*!< [0x0110] ADC CH0 Convert Buffer Low Byte                                  */
#define NPD48_ADCCH0H      0x0111           /*!< [0x0111] ADC CH0 Convert Buffer High Byte                                 */
#define NPD48_ADCCH1L      0x0112           /*!< [0x0112] ADC CH1 Convert Buffer Low Byte                                  */
#define NPD48_ADCCH1H      0x0113           /*!< [0x0113] ADC CH1 Convert Buffer High Byte                                 */
#define NPD48_ADCCH2L      0x0114           /*!< [0x0114] ADC CH2 Convert Buffer Low Byte                                  */
#define NPD48_ADCCH2H      0x0115           /*!< [0x0115] ADC CH2 Convert Buffer High Byte                                 */
#define NPD48_ADCCH3L      0x0116           /*!< [0x0116] ADC CH3 Convert Buffer Low Byte                                  */
#define NPD48_ADCCH3H      0x0117           /*!< [0x0117] ADC CH3 Convert Buffer High Byte                                 */
#define NPD48_ADCCH4L      0x0118           /*!< [0x0118] ADC CH4 Convert Buffer Low Byte                                  */
#define NPD48_ADCCH4H      0x0119           /*!< [0x0119] ADC CH4 Convert Buffer High Byte                                 */
#define NPD48_ADCCH5L      0x011a           /*!< [0x011a] ADC CH5 Convert Buffer Low Byte                                  */
#define NPD48_ADCCH5H      0x011b           /*!< [0x011b] ADC CH5 Convert Buffer High Byte                                 */
#define NPD48_ADCCH6L      0x011c           /*!< [0x011c] ADC CH6 Convert Buffer Low Byte                                  */
#define NPD48_ADCCH6H      0x011d           /*!< [0x011d] ADC CH6 Convert Buffer High Byte                                 */
#define NPD48_ADCCH7L      0x011e           /*!< [0x011e] ADC CH7 Convert Buffer Low Byte                                  */
#define NPD48_ADCCH7H      0x011f           /*!< [0x011f] ADC CH7 Convert Buffer High Byte                                 */
#define NPD48_UFCSCR       0x0121           /*!< [0x0121] UFCS Control Register                                            */
#define NPD48_UFCSSR       0x0122           /*!< [0x0122] UFCS Status Register                                             */
#define NPD48_BCTMCR       0x0125           /*!< [0x0125] Battery Charger Test Mode Control Register                       */
#define NPD48_BCAMCR       0x0126           /*!< [0x0126] Battery Charger Alpha Mode Control Register                      */
#define NPD48_OVPCMPL      0x0130           /*!< [0x0130] OVP Digital Compare Low Byte                                     */
#define NPD48_OVPCMPH      0x0131           /*!< [0x0131] OVP Digital Compare High Byte                                    */
#define NPD48_UVPCMPL      0x0132           /*!< [0x0132] UVP Digital Compare Low Byte                                     */
#define NPD48_UVPCMPH      0x0133           /*!< [0x0133] UVP Digital Compare High Byte                                    */
#define NPD48_OCPCMPL      0x0134           /*!< [0x0134] OCP Digital Compare Low Byte                                     */
#define NPD48_OCPCMPH      0x0135           /*!< [0x0135] OCP Digital Compare High Byte                                    */
#define NPD48_NTCPCMPL     0x0136           /*!< [0x0136] NTCP Digital Compare Low Byte                                    */
#define NPD48_NTCPCMPH     0x0137           /*!< [0x0137] NTCP Digital Compare High Byte                                   */
#define NPD48_COVPCMPL     0x0138           /*!< [0x0138] Chip OVP Digital Compare Low Byte                                */
#define NPD48_COVPCMPH     0x0139           /*!< [0x0139] Chip OVP Digital Compare High Byte                               */
#define NPD48_ITPCMPL      0x013A           /*!< [0x013A] Internal Temperature Protect Digital Compare Low Byte            */
#define NPD48_ITPCMPH      0x013B           /*!< [0x013B] Internal Temperature Protect Digital Compare High Byte           */
#define NPD48_ADFCR        0x013F           /*!< [0x013F] ADC FPGA Control Register                                        */

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */

#define NPD48_ADCR_ADCEN_Pos               (0)                                               /*!< ADC_T::ADCR: ADCEN Position            */
#define NPD48_ADCR_ADCEN_Msk               (0x1ul << NPD48_ADCR_ADCEN_Pos)                   /*!< ADC_T::ADCR: ADCEN Mask                */

#define NPD48_ADCR_SCANMD_Pos              (1)                                               /*!< ADC_T::ADCR: SCANMD Position           */
#define NPD48_ADCR_SCANMD_Msk              (0x1ul << NPD48_ADCR_SCANMD_Pos)                  /*!< ADC_T::ADCR: SCANMD Mask               */

#define NPD48_ADCALCTL_CALST_Pos           (0)                                               /*!< ADC_T::ADCALCTL: CALST Position        */
#define NPD48_ADCALCTL_CALST_Msk           (0x1ul << NPD48_ADCALCTL_CALST_Pos)               /*!< ADC_T::ADCALCTL: CALST Mask            */

#define NPD48_ADCALCTL_CALF_Pos            (1)                                               /*!< ADC_T::ADCALCTL: CALF Position         */
#define NPD48_ADCALCTL_CALF_Msk            (0x1ul << NPD48_ADCALCTL_CALF_Pos)                /*!< ADC_T::ADCALCTL: CALF Mask             */

#define NPD48_ADCALCTL_MODEVM_Pos          (2)                                               /*!< ADC_T::ADCALCTL: MODEVM Position       */
#define NPD48_ADCALCTL_MODEVM_Msk          (0x1ul << NPD48_ADCALCTL_MODEVM_Pos)              /*!< ADC_T::ADCALCTL: MODEVM Mask           */

#define NPD48_ADCALCTL_SEL16T_Pos          (3)                                               /*!< ADC_T::ADCALCTL: SEL16T Position       */
#define NPD48_ADCALCTL_SEL16T_Msk          (0x1ul << NPD48_ADCALCTL_SEL16T_Pos)              /*!< ADC_T::ADCALCTL: SEL16T Mask           */

#define NPD48_ADCALCTL_CALSEL_Pos          (4)                                               /*!< ADC_T::ADCALCTL: CALSEL Position       */
#define NPD48_ADCALCTL_CALSEL_Msk          (0x7ul << NPD48_ADCALCTL_CALSEL_Pos)              /*!< ADC_T::ADCALCTL: CALSEL Mask           */

#define NPD48_ADCALCTL_CALCCO_Pos          (7)                                               /*!< ADC_T::ADCALCTL: CALCCO Position       */
#define NPD48_ADCALCTL_CALCCO_Msk          (0x1ul << NPD48_ADCALCTL_CALCCO_Pos)              /*!< ADC_T::ADCALCTL: CALCCO Mask           */

#define NPD48_ADEXTEND_EXTSMP_Pos          (0)                                               /*!< ADC_T::ADEXTEND: EXTSMP Position       */
#define NPD48_ADEXTEND_EXTSMP_Msk          (0xfful << NPD48_ADEXTEND_EXTSMP_Pos)             /*!< ADC_T::ADEXTEND: EXTSMP Mask           */

#define NPD48_ADCHEN_CH0EN_Pos             (0)                                               /*!< ADC_T::ADCHEN: CH0EN Position          */
#define NPD48_ADCHEN_CH0EN_Msk             (0x1ul << NPD48_ADCHEN_CH0EN_Pos)                 /*!< ADC_T::ADCHEN: CH0EN Mask              */

#define NPD48_ADCHEN_CH1EN_Pos             (1)                                               /*!< ADC_T::ADCHEN: CH1EN Position          */
#define NPD48_ADCHEN_CH1EN_Msk             (0x1ul << NPD48_ADCHEN_CH1EN_Pos)                 /*!< ADC_T::ADCHEN: CH1EN Mask              */

#define NPD48_ADCHEN_CH2EN_Pos             (2)                                               /*!< ADC_T::ADCHEN: CH2EN Position          */
#define NPD48_ADCHEN_CH2EN_Msk             (0x1ul << NPD48_ADCHEN_CH2EN_Pos)                 /*!< ADC_T::ADCHEN: CH2EN Mask              */

#define NPD48_ADCHEN_CH3EN_Pos             (3)                                               /*!< ADC_T::ADCHEN: CH3EN Position          */
#define NPD48_ADCHEN_CH3EN_Msk             (0x1ul << NPD48_ADCHEN_CH3EN_Pos)                 /*!< ADC_T::ADCHEN: CH3EN Mask              */

#define NPD48_ADCHEN_CH4EN_Pos             (4)                                               /*!< ADC_T::ADCHEN: CH4EN Position          */
#define NPD48_ADCHEN_CH4EN_Msk             (0x1ul << NPD48_ADCHEN_CH4EN_Pos)                 /*!< ADC_T::ADCHEN: CH4EN Mask              */

#define NPD48_ADCHEN_CH5EN_Pos             (5)                                               /*!< ADC_T::ADCHEN: CH5EN Position          */
#define NPD48_ADCHEN_CH5EN_Msk             (0x1ul << NPD48_ADCHEN_CH5EN_Pos)                 /*!< ADC_T::ADCHEN: CH5EN Mask              */

#define NPD48_ADCHEN_CH6EN_Pos             (6)                                               /*!< ADC_T::ADCHEN: CH6EN Position          */
#define NPD48_ADCHEN_CH6EN_Msk             (0x1ul << NPD48_ADCHEN_CH6EN_Pos)                 /*!< ADC_T::ADCHEN: CH6EN Mask              */

#define NPD48_ADCHEN_CH7EN_Pos             (7)                                               /*!< ADC_T::ADCHEN: CH7EN Position          */
#define NPD48_ADCHEN_CH7EN_Msk             (0x1ul << NPD48_ADCHEN_CH7EN_Pos)                 /*!< ADC_T::ADCHEN: CH7EN Mask              */

#define NPD48_ADCALDCR_CALADDR_Pos         (0)                                               /*!< ADC_T::ADCALDCR: CALADDR Position      */
#define NPD48_ADCALDCR_CALADDR_Msk         (0x1ful << NPD48_ADCALDCR_CALADDR_Pos)            /*!< ADC_T::ADCALDCR: CALADDR Mask          */

#define NPD48_ADCALDCR_CALWR_Pos           (5)                                               /*!< ADC_T::ADCALDCR: CALWR Position        */
#define NPD48_ADCALDCR_CALWR_Msk           (0x1ul << NPD48_ADCALDCR_CALWR_Pos)               /*!< ADC_T::ADCALDCR: CALWR Mask            */

#define NPD48_ADCALDCR_CALRD_Pos           (6)                                               /*!< ADC_T::ADCALDCR: CALRD Position        */
#define NPD48_ADCALDCR_CALRD_Msk           (0x1ul << NPD48_ADCALDCR_CALRD_Pos)               /*!< ADC_T::ADCALDCR: CALRD Mask            */

#define NPD48_ADCALDCR_OUTSEL_Pos          (7)                                               /*!< ADC_T::ADCALDCR: OUTSEL Position       */
#define NPD48_ADCALDCR_OUTSEL_Msk          (0x1ul << NPD48_ADCALDCR_OUTSEL_Pos)              /*!< ADC_T::ADCALDCR: OUTSEL Mask           */

#define NPD48_ADCALDDR_CALDATA_Pos         (0)                                               /*!< ADC_T::ADCALDDR: CALDATA Position      */
#define NPD48_ADCALDDR_CALDATA_Msk         (0xfful << NPD48_ADCALDDR_CALDATA_Pos)            /*!< ADC_T::ADCALDDR: CALDATA Mask          */

#define NPD48_ADCADJR_ASDELAYC_Pos         (0)                                               /*!< ADC_T::ADCADJR: ASDELAYC Position      */
#define NPD48_ADCADJR_ASDELAYC_Msk         (0x3ul << NPD48_ADCADJR_ASDELAYC_Pos)             /*!< ADC_T::ADCADJR: ASDELAYC Mask          */

#define NPD48_ADCADJR_ASDELAYF_Pos         (2)                                               /*!< ADC_T::ADCADJR: ASDELAYF Position      */
#define NPD48_ADCADJR_ASDELAYF_Msk         (0x3ul << NPD48_ADCADJR_ASDELAYF_Pos)             /*!< ADC_T::ADCADJR: ASDELAYF Mask          */

#define NPD48_ADCADJR_DECADD_Pos           (7)                                               /*!< ADC_T::ADCADJR: DECADD Position        */
#define NPD48_ADCADJR_DECADD_Msk           (0x1ul << NPD48_ADCADJR_DECADD_Pos)               /*!< ADC_T::ADCADJR: DECADD Mask            */

#define NPD48_ADCCH0L_ADCCH0L_Pos          (0)                                               /*!< ADC_T::ADCCH0L: ADCCH0L Position       */
#define NPD48_ADCCH0L_ADCCH0L_Msk          (0xfful << NPD48_ADCCH0L_ADCCH0L_Pos)             /*!< ADC_T::ADCCH0L: ADCCH0L Mask           */

#define NPD48_ADCCH0H_ADCCH0H_Pos          (0)                                               /*!< ADC_T::ADCCH0H: ADCCH0H Position       */
#define NPD48_ADCCH0H_ADCCH0H_Msk          (0xful << NPD48_ADCCH0H_ADCCH0H_Pos)              /*!< ADC_T::ADCCH0H: ADCCH0H Mask           */

#define NPD48_ADCCH1L_ADCCH1L_Pos          (0)                                               /*!< ADC_T::ADCCH1L: ADCCH1L Position       */
#define NPD48_ADCCH1L_ADCCH1L_Msk          (0xfful << NPD48_ADCCH1L_ADCCH1L_Pos)             /*!< ADC_T::ADCCH1L: ADCCH1L Mask           */

#define NPD48_ADCCH1H_ADCCH1H_Pos          (0)                                               /*!< ADC_T::ADCCH1H: ADCCH1H Position       */
#define NPD48_ADCCH1H_ADCCH1H_Msk          (0xful << NPD48_ADCCH1H_ADCCH1H_Pos)              /*!< ADC_T::ADCCH1H: ADCCH1H Mask           */

#define NPD48_ADCCH2L_ADCCH2L_Pos          (0)                                               /*!< ADC_T::ADCCH2L: ADCCH2L Position       */
#define NPD48_ADCCH2L_ADCCH2L_Msk          (0xfful << NPD48_ADCCH2L_ADCCH2L_Pos)             /*!< ADC_T::ADCCH2L: ADCCH2L Mask           */

#define NPD48_ADCCH2H_ADCCH2H_Pos          (0)                                               /*!< ADC_T::ADCCH2H: ADCCH2H Position       */
#define NPD48_ADCCH2H_ADCCH2H_Msk          (0xful << NPD48_ADCCH2H_ADCCH2H_Pos)              /*!< ADC_T::ADCCH2H: ADCCH2H Mask           */

#define NPD48_ADCCH3L_ADCCH3L_Pos          (0)                                               /*!< ADC_T::ADCCH3L: ADCCH3L Position       */
#define NPD48_ADCCH3L_ADCCH3L_Msk          (0xfful << NPD48_ADCCH3L_ADCCH3L_Pos)             /*!< ADC_T::ADCCH3L: ADCCH3L Mask           */

#define NPD48_ADCCH3H_ADCCH3H_Pos          (0)                                               /*!< ADC_T::ADCCH3H: ADCCH3H Position       */
#define NPD48_ADCCH3H_ADCCH3H_Msk          (0xful << NPD48_ADCCH3H_ADCCH3H_Pos)              /*!< ADC_T::ADCCH3H: ADCCH3H Mask           */

#define NPD48_ADCCH4L_ADCCH4L_Pos          (0)                                               /*!< ADC_T::ADCCH4L: ADCCH4L Position       */
#define NPD48_ADCCH4L_ADCCH4L_Msk          (0xfful << NPD48_ADCCH4L_ADCCH4L_Pos)             /*!< ADC_T::ADCCH4L: ADCCH4L Mask           */

#define NPD48_ADCCH4H_ADCCH4H_Pos          (0)                                               /*!< ADC_T::ADCCH4H: ADCCH4H Position       */
#define NPD48_ADCCH4H_ADCCH4H_Msk          (0xful << NPD48_ADCCH4H_ADCCH4H_Pos)              /*!< ADC_T::ADCCH4H: ADCCH4H Mask           */

#define NPD48_ADCCH5L_ADCCH5L_Pos          (0)                                               /*!< ADC_T::ADCCH5L: ADCCH5L Position       */
#define NPD48_ADCCH5L_ADCCH5L_Msk          (0xfful << NPD48_ADCCH5L_ADCCH5L_Pos)             /*!< ADC_T::ADCCH5L: ADCCH5L Mask           */

#define NPD48_ADCCH5H_ADCCH5H_Pos          (0)                                               /*!< ADC_T::ADCCH5H: ADCCH5H Position       */
#define NPD48_ADCCH5H_ADCCH5H_Msk          (0xful << NPD48_ADCCH5H_ADCCH5H_Pos)              /*!< ADC_T::ADCCH5H: ADCCH5H Mask           */

#define NPD48_ADCCH6L_ADCCH6L_Pos          (0)                                               /*!< ADC_T::ADCCH6L: ADCCH6L Position       */
#define NPD48_ADCCH6L_ADCCH6L_Msk          (0xfful << NPD48_ADCCH6L_ADCCH6L_Pos)             /*!< ADC_T::ADCCH6L: ADCCH6L Mask           */

#define NPD48_ADCCH6H_ADCCH6H_Pos          (0)                                               /*!< ADC_T::ADCCH6H: ADCCH6H Position       */
#define NPD48_ADCCH6H_ADCCH6H_Msk          (0xful << NPD48_ADCCH6H_ADCCH6H_Pos)              /*!< ADC_T::ADCCH6H: ADCCH6H Mask           */

#define NPD48_ADCCH7L_ADCCH7L_Pos          (0)                                               /*!< ADC_T::ADCCH7L: ADCCH7L Position       */
#define NPD48_ADCCH7L_ADCCH7L_Msk          (0xfful << NPD48_ADCCH7L_ADCCH7L_Pos)             /*!< ADC_T::ADCCH7L: ADCCH7L Mask           */

#define NPD48_ADCCH7H_ADCCH7H_Pos          (0)                                               /*!< ADC_T::ADCCH7H: ADCCH7H Position       */
#define NPD48_ADCCH7H_ADCCH7H_Msk          (0xful << NPD48_ADCCH7H_ADCCH7H_Pos)              /*!< ADC_T::ADCCH7H: ADCCH7H Mask           */

#define NPD48_OVPCMPL_OVPCMPL_Pos          (0)                                               /*!< ADC_T::OVPCMPL: OVPCMPL Position       */
#define NPD48_OVPCMPL_OVPCMPL_Msk          (0xfful << NPD48_OVPCMPL_OVPCMPL_Pos)             /*!< ADC_T::OVPCMPL: OVPCMPL Mask           */

#define NPD48_OVPCMPH_OVPCMPH_Pos          (0)                                               /*!< ADC_T::OVPCMPH: OVPCMPH Position       */
#define NPD48_OVPCMPH_OVPCMPH_Msk          (0xful << NPD48_OVPCMPH_OVPCMPH_Pos)              /*!< ADC_T::OVPCMPH: OVPCMPH Mask           */

#define NPD48_UVPCMPL_UVPCMPL_Pos          (0)                                               /*!< ADC_T::UVPCMPL: UVPCMPL Position       */
#define NPD48_UVPCMPL_UVPCMPL_Msk          (0xfful << NPD48_UVPCMPL_UVPCMPL_Pos)             /*!< ADC_T::UVPCMPL: UVPCMPL Mask           */

#define NPD48_UVPCMPH_UVPCMPH_Pos          (0)                                               /*!< ADC_T::UVPCMPH: UVPCMPH Position       */
#define NPD48_UVPCMPH_UVPCMPH_Msk          (0xful << NPD48_UVPCMPH_UVPCMPH_Pos)              /*!< ADC_T::UVPCMPH: UVPCMPH Mask           */

#define NPD48_OCPCMPL_OCPCMPL_Pos          (0)                                               /*!< ADC_T::OCPCMPL: OCPCMPL Position       */
#define NPD48_OCPCMPL_OCPCMPL_Msk          (0xfful << NPD48_OCPCMPL_OCPCMPL_Pos)             /*!< ADC_T::OCPCMPL: OCPCMPL Mask           */

#define NPD48_OCPCMPH_OCPCMPH_Pos          (0)                                               /*!< ADC_T::OCPCMPH: OCPCMPH Position       */
#define NPD48_OCPCMPH_OCPCMPH_Msk          (0xful << NPD48_OCPCMPH_OCPCMPH_Pos)              /*!< ADC_T::OCPCMPH: OCPCMPH Mask           */

#define NPD48_NTCPCMPL_NTCPCMPL_Pos        (0)                                               /*!< ADC_T::NTCPCMPL: NTCPCMPL Position     */
#define NPD48_NTCPCMPL_NTCPCMPL_Msk        (0xfful << NPD48_NTCPCMPL_NTCPCMPL_Pos)           /*!< ADC_T::NTCPCMPL: NTCPCMPL Mask         */

#define NPD48_NTCPCMPH_NTCPCMPH_Pos        (0)                                               /*!< ADC_T::NTCPCMPH: NTCPCMPH Position     */
#define NPD48_NTCPCMPH_NTCPCMPH_Msk        (0xful << NPD48_NTCPCMPH_NTCPCMPH_Pos)            /*!< ADC_T::NTCPCMPH: NTCPCMPH Mask         */

#define NPD48_COVPCMPL_COVPCMPL_Pos        (0)                                               /*!< ADC_T::COVPCMPL: COVPCMPL Position     */
#define NPD48_COVPCMPL_COVPCMPL_Msk        (0xfful << NPD48_COVPCMPL_COVPCMPL_Pos)           /*!< ADC_T::COVPCMPL: COVPCMPL Mask         */

#define NPD48_COVPCMPH_COVPCMPH_Pos        (0)                                               /*!< ADC_T::COVPCMPH: COVPCMPH Position     */
#define NPD48_COVPCMPH_COVPCMPH_Msk        (0xful << NPD48_COVPCMPH_COVPCMPH_Pos)            /*!< ADC_T::COVPCMPH: COVPCMPH Mask         */

#define NPD48_ITPCMPL_ITPCMPL_Pos          (0)                                               /*!< ADC_T::ITPCMPL: ITPCMPL Position       */
#define NPD48_ITPCMPL_ITPCMPL_Msk          (0xfful << NPD48_ITPCMPL_ITPCMPL_Pos)             /*!< ADC_T::ITPCMPL: ITPCMPL Mask           */

#define NPD48_ITPCMPH_ITPCMPH_Pos          (0)                                               /*!< ADC_T::ITPCMPH: ITPCMPH Position       */
#define NPD48_ITPCMPH_ITPCMPH_Msk          (0xful << NPD48_ITPCMPH_ITPCMPH_Pos)              /*!< ADC_T::ITPCMPH: ITPCMPH Mask           */

#define NPD48_ADFCR_ADFCR_Pos              (0)                                               /*!< ADC_T::ADFCR: ADFCR Position           */
#define NPD48_ADFCR_ADFCR_Msk              (0xfful << NPD48_ADFCR_ADFCR_Pos)                 /*!< ADC_T::ADFCR: ADFCR Mask               */

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */
