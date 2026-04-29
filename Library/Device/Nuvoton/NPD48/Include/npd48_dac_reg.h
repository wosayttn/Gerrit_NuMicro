/*---------------------- Digital to Analog Converter -------------------------*/
/**
    @addtogroup DAC Digital to Analog Converter(DAC)
    Memory Mapped Structure for DAC Controller
@{ */

/**
 * @var DAC_T::DACOPCR
 * Offset: 0x128  DAC and OPA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DAC0EN    |DAC0 Enable Bit
 * |        |          |0 = DAC0 Disabled.
 * |        |          |1 = DAC0 Enabled.
 * |        |          |Note: When DAC0EN = 0, the DAC0   analog macro output will become tri-state.
 * |        |          |[Design]   control DAC12MONO pin BUF_EN
 * |[4]     |OPA0EN    |OPA0 Enable Bit
 * |        |          |0 = OPA0 Disable.
 * |        |          |1 = OPA0 Enable.
 * |        |          |Note: OPA0 is default enable   macro
 * |        |          |[Design]   control MEGPGACATH pin EN0
 * |[5]     |OPA1EN    |OPA0 Enable Bit
 * |        |          |0 = OPA1 Disable.
 * |        |          |1 = OPA1 Enable.
 * |        |          |[Design]   control MEGPGACATH pin EN1
 * |[6]     |PGA0EN    |PGA0 Enable Bit
 * |        |          |0 = PGA0 Disable.
 * |        |          |1 = PGA0 Enable.
 * |        |          |[Design]   control MEGPGACATH pin EN2
 * |[7]     |ISSWAP    |IS Swap Enable Bit
 * |        |          |0 = IS+/IS- swap Disable.
 * |        |          |1 = IS+/IS- swap Enable.
 * |        |          |[Design]   control MEGPGACATH pin SWAP
 * @var DAC_T::DAC0DBL
 * Offset: 0x129  DAC 0 Data Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DAC0DBL   |DAC0 Data Buffer Low Byte
 * |        |          |DAC0 data buffer low byte [7:0]
 * |        |          |Note: for DAC0 operating, only write DAC0DBH will trigger DAC0 update voltage with the content of DAC0DBH & DAC0DBL
 * |        |          |Write DAC0DBL will no effect for DAC0.
 * |        |          |[Design] control DAC12MONO pin DIN[7:0]
 * @var DAC_T::DAC0DBH
 * Offset: 0x12A  DAC 0 Data Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |DAC0DBH   |DAC0 Data Buffer High Byte
 * |        |          |DAC0 data buffer high byte [11:8]
 * |        |          |Note: for DAC0 operating, only write DAC0DBH will trigger DAC0 update voltage with the content of DAC0DBH & DAC0DBL
 * |        |          |Write DAC0DBL will no effect for DAC0.
 * |        |          |[Design] control DAC12MONO pin DIN[11:8]
 * @var DAC_T::DAC1DBL
 * Offset: 0x12B  DAC 1 Data Buffer Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DAC1DBL   |DAC1 Data Buffer Low Byte
 * |        |          |DAC1 data buffer low byte [7:0]
 * |        |          |Note: for DAC1 operating, only write DAC1DBH will trigger DAC1 update voltage with the content of DAC1DBH & DAC1DBL
 * |        |          |Write DAC1DBL will no effect for DAC1.
 * |        |          |[Design] control DAC10MONO pin DIN[11:8]
 * |        |          |1 step 0.001V, PPS 50mAu5C0Du61C90.002V (TBC)
 * @var DAC_T::DAC1DBH
 * Offset: 0x12C  DAC 1 Data Buffer High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |DAC1DBH   |DAC1 Data Buffer High Byte
 * |        |          |DAC1 data buffer high byte [9:8]
 * |        |          |Note: for DAC1 operating, only write DAC1DBH will trigger DAC1 update voltage with the content of DAC1DBH & DAC1DBL
 * |        |          |Write DAC1DBL will no effect for DAC1.
 * |        |          |[Design] control DAC10MONO pin DIN[9:8]
 * |        |          |1 step 0.001V, PPS 50mAu5C0Du61C90.002V (TBC)
 */
#define NPD48_DACOPCR      0x0128           /*!< [0x0128] DAC and OPA Control Register                                     */
#define NPD48_DAC0DBL      0x0129           /*!< [0x0129] DAC 0 Data Buffer Low Byte                                       */
#define NPD48_DAC0DBH      0x012a           /*!< [0x012a] DAC 0 Data Buffer High Byte                                      */
#define NPD48_DAC1DBL      0x012b           /*!< [0x012b] DAC 1 Data Buffer Low Byte                                       */
#define NPD48_DAC1DBH      0x012c           /*!< [0x012c] DAC 1 Data Buffer High Byte                                      */

/**
    @addtogroup DAC_CONST DAC Bit Field Definition
    Constant Definitions for DAC Controller
@{ */

#define NPD48_DACOPCR_DAC0EN_Pos           (0)                                               /*!< DAC_T::DACOPCR: DAC0EN Position        */
#define NPD48_DACOPCR_DAC0EN_Msk           (0x1ul << NPD48_DACOPCR_DAC0EN_Pos)               /*!< DAC_T::DACOPCR: DAC0EN Mask            */

#define NPD48_DACOPCR_OPA0EN_Pos           (4)                                               /*!< DAC_T::DACOPCR: OPA0EN Position        */
#define NPD48_DACOPCR_OPA0EN_Msk           (0x1ul << NPD48_DACOPCR_OPA0EN_Pos)               /*!< DAC_T::DACOPCR: OPA0EN Mask            */

#define NPD48_DACOPCR_OPA1EN_Pos           (5)                                               /*!< DAC_T::DACOPCR: OPA1EN Position        */
#define NPD48_DACOPCR_OPA1EN_Msk           (0x1ul << NPD48_DACOPCR_OPA1EN_Pos)               /*!< DAC_T::DACOPCR: OPA1EN Mask            */

#define NPD48_DACOPCR_PGA0EN_Pos           (6)                                               /*!< DAC_T::DACOPCR: PGA0EN Position        */
#define NPD48_DACOPCR_PGA0EN_Msk           (0x1ul << NPD48_DACOPCR_PGA0EN_Pos)               /*!< DAC_T::DACOPCR: PGA0EN Mask            */

#define NPD48_DACOPCR_ISSWAP_Pos           (7)                                               /*!< DAC_T::DACOPCR: ISSWAP Position        */
#define NPD48_DACOPCR_ISSWAP_Msk           (0x1ul << NPD48_DACOPCR_ISSWAP_Pos)               /*!< DAC_T::DACOPCR: ISSWAP Mask            */

#define NPD48_DAC0DBL_DAC0DBL_Pos          (0)                                               /*!< DAC_T::DAC0DBL: DAC0DBL Position       */
#define NPD48_DAC0DBL_DAC0DBL_Msk          (0xfful << NPD48_DAC0DBL_DAC0DBL_Pos)             /*!< DAC_T::DAC0DBL: DAC0DBL Mask           */

#define NPD48_DAC0DBH_DAC0DBH_Pos          (0)                                               /*!< DAC_T::DAC0DBH: DAC0DBH Position       */
#define NPD48_DAC0DBH_DAC0DBH_Msk          (0xful << NPD48_DAC0DBH_DAC0DBH_Pos)              /*!< DAC_T::DAC0DBH: DAC0DBH Mask           */

#define NPD48_DAC1DBL_DAC1DBL_Pos          (0)                                               /*!< DAC_T::DAC1DBL: DAC1DBL Position       */
#define NPD48_DAC1DBL_DAC1DBL_Msk          (0xfful << NPD48_DAC1DBL_DAC1DBL_Pos)             /*!< DAC_T::DAC1DBL: DAC1DBL Mask           */

#define NPD48_DAC1DBH_DAC1DBH_Pos          (0)                                               /*!< DAC_T::DAC1DBH: DAC1DBH Position       */
#define NPD48_DAC1DBH_DAC1DBH_Msk          (0x3ul << NPD48_DAC1DBH_DAC1DBH_Pos)              /*!< DAC_T::DAC1DBH: DAC1DBH Mask           */

/**@}*/ /* DAC_CONST */
/**@}*/ /* end of DAC register group */
