/*---------------------- ?????????????????????????????????????????? -------------------------*/
/**
    @addtogroup EFUSE ??????????????????????????????????????????(EFUSE)
    Memory Mapped Structure for EFUSE Controller
@{ */

/**
 * @var EFUSE_T::EFDB0
 * Offset: 0xD8  eFUSE Data Byte 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EFDB0     |eFUSE Data Byte 0
 * |        |          |eFUSE[7:0]
 * @var EFUSE_T::EFDB1
 * Offset: 0xD9  eFUSE Data Byte 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EFDB1     |eFUSE Data Byte 1
 * |        |          |eFUSE[15:8]
 * @var EFUSE_T::EFDB2
 * Offset: 0xDA  eFUSE Data Byte 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EFDB2     |eFUSE Data Byte 2
 * |        |          |eFUSE[23:16]
 * @var EFUSE_T::EFDB3
 * Offset: 0xDB  eFUSE Data Byte 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EFDB3     |eFUSE Data Byte 3
 * |        |          |eFUSE[31:24]
 * @var EFUSE_T::EFDB4
 * Offset: 0xDC  eFUSE Data Byte 4
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EFDB4     |eFUSE Data Byte 4
 * |        |          |eFUSE[39:32]
 * @var EFUSE_T::EFDB5
 * Offset: 0xDD  eFUSE Data Byte 5
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EFDB5     |eFUSE Data Byte 5
 * |        |          |eFUSE[47:40]
 * @var EFUSE_T::EFDB6
 * Offset: 0xDE  eFUSE Data Byte 6
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EFDB6     |eFUSE Data Byte 6
 * |        |          |eFUSE[55:48]
 * @var EFUSE_T::EFDB7
 * Offset: 0xDF  eFUSE Data Byte 7
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EFDB7     |eFUSE Data Byte 7
 * |        |          |eFUSE[63:56]
 * @var EFUSE_T::EFCR
 * Offset: 0xE0  eFUSE Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |EFRW      |eFUSE RW Contorl
 * |        |          |0 = eFUSE control at read   state.
 * |        |          |1 = eFUSE control at program   state.
 * |[7]     |EFACT     |eFUSE Action Bit
 * |        |          |When set this bit to 1, the   eFUSE will start program action if RW is 1
 * |        |          |After program finish, this will   will be clear automatically by hardware
 * |        |          |When set this bit to 1, the   eFUSE will start program action if RW is 1
 * |        |          |After program finish, this will   will be clear automatically by hardware
 * @var EFUSE_T::EFKEY
 * Offset: 0xE1  eFUSE Key Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |EFKEY     |eFUSE Key
 * |        |          |Before user do eFUSE read or   program action, the EFKSY should be filled 0x5a in advance and keep during   whole eFUSE read or program period
 * |        |          |After finish eFUSE read or program, user   should set EFKEY to 0x00 to block other eFUSE behavior.
 * @var EFUSE_T::TCPCSFT
 * Offset: 0xE2  TCPC State Filter Time Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |STFILT    |TCPC State Filter Time
 * |        |          |Time = 31.2 us x TCPC filter   time value = 31.2us.
 * @var EFUSE_T::I2CCTL
 * Offset: 0xE4  I2C Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CKPDEN    |Clock   Power-down Enable Bit
 * |        |          |0 = I2C clock   not stop.
 * |        |          |1 = I2C clock   IDLE and Stop. 
 * |[1]     |I2CWKSEL  |I2C Wake-up   Select
 * |        |          |0 = I2C wake   after address match is Detected.
 * |        |          |1 = I2C wake   up in 1st bit of data phase after address   match.
 * |[2]     |PECEN     |PEC Enable
 * |        |          |0 = Packet Error Checking   (PEC) is Disabled.
 * |        |          |1 = Packet Error   Checking (PEC) is Enabled
 * |[3]     |PECSEL    |PEC Select
 * |        |          |0 = CRC is Enabled all the   time.
 * |        |          |1 = CRC is Disabled in start   and identify period.
 */
#define NPD48_EFDB0        0x00d8           /*!< [0x00d8] eFUSE Data Byte 0                                                */
#define NPD48_EFDB1        0x00d9           /*!< [0x00d9] eFUSE Data Byte 1                                                */
#define NPD48_EFDB2        0x00da           /*!< [0x00da] eFUSE Data Byte 2                                                */
#define NPD48_EFDB3        0x00db           /*!< [0x00db] eFUSE Data Byte 3                                                */
#define NPD48_EFDB4        0x00dc           /*!< [0x00dc] eFUSE Data Byte 4                                                */
#define NPD48_EFDB5        0x00dd           /*!< [0x00dd] eFUSE Data Byte 5                                                */
#define NPD48_EFDB6        0x00de           /*!< [0x00de] eFUSE Data Byte 6                                                */
#define NPD48_EFDB7        0x00df           /*!< [0x00df] eFUSE Data Byte 7                                                */
#define NPD48_EFCR         0x00e0           /*!< [0x00e0] eFUSE Control Register                                           */
#define NPD48_EFKEY        0x00e1           /*!< [0x00e1] eFUSE Key Register                                               */
#define NPD48_TCPCSFT      0x00e2           /*!< [0x00e2] TCPC State Filter Time Register                                  */
#define NPD48_I2CCTL       0x00e4           /*!< [0x00e4] I2C Control Register                                             */

/**
    @addtogroup EFUSE_CONST EFUSE Bit Field Definition
    Constant Definitions for EFUSE Controller
@{ */

#define NPD48_EFDB0_EFDB0_Pos            (0)                                               /*!< EFUSE_T::EFDB0: EFDB0 Position         */
#define NPD48_EFDB0_EFDB0_Msk            (0xfful << NPD48_EFDB0_EFDB0_Pos)                 /*!< EFUSE_T::EFDB0: EFDB0 Mask             */

#define NPD48_EFDB1_EFDB1_Pos            (0)                                               /*!< EFUSE_T::EFDB1: EFDB1 Position         */
#define NPD48_EFDB1_EFDB1_Msk            (0xfful << NPD48_EFDB1_EFDB1_Pos)                 /*!< EFUSE_T::EFDB1: EFDB1 Mask             */

#define NPD48_EFDB2_EFDB2_Pos            (0)                                               /*!< EFUSE_T::EFDB2: EFDB2 Position         */
#define NPD48_EFDB2_EFDB2_Msk            (0xfful << NPD48_EFDB2_EFDB2_Pos)                 /*!< EFUSE_T::EFDB2: EFDB2 Mask             */

#define NPD48_EFDB3_EFDB3_Pos            (0)                                               /*!< EFUSE_T::EFDB3: EFDB3 Position         */
#define NPD48_EFDB3_EFDB3_Msk            (0xfful << NPD48_EFDB3_EFDB3_Pos)                 /*!< EFUSE_T::EFDB3: EFDB3 Mask             */

#define NPD48_EFDB4_EFDB4_Pos            (0)                                               /*!< EFUSE_T::EFDB4: EFDB4 Position         */
#define NPD48_EFDB4_EFDB4_Msk            (0xfful << NPD48_EFDB4_EFDB4_Pos)                 /*!< EFUSE_T::EFDB4: EFDB4 Mask             */

#define NPD48_EFDB5_EFDB5_Pos            (0)                                               /*!< EFUSE_T::EFDB5: EFDB5 Position         */
#define NPD48_EFDB5_EFDB5_Msk            (0xfful << NPD48_EFDB5_EFDB5_Pos)                 /*!< EFUSE_T::EFDB5: EFDB5 Mask             */

#define NPD48_EFDB6_EFDB6_Pos            (0)                                               /*!< EFUSE_T::EFDB6: EFDB6 Position         */
#define NPD48_EFDB6_EFDB6_Msk            (0xfful << NPD48_EFDB6_EFDB6_Pos)                 /*!< EFUSE_T::EFDB6: EFDB6 Mask             */

#define NPD48_EFDB7_EFDB7_Pos            (0)                                               /*!< EFUSE_T::EFDB7: EFDB7 Position         */
#define NPD48_EFDB7_EFDB7_Msk            (0xfful << NPD48_EFDB7_EFDB7_Pos)                 /*!< EFUSE_T::EFDB7: EFDB7 Mask             */

#define NPD48_EFCR_EFRW_Pos              (0)                                               /*!< EFUSE_T::EFCR: EFRW Position           */
#define NPD48_EFCR_EFRW_Msk              (0x1ul << NPD48_EFCR_EFRW_Pos)                    /*!< EFUSE_T::EFCR: EFRW Mask               */

#define NPD48_EFCR_EFACT_Pos             (7)                                               /*!< EFUSE_T::EFCR: EFACT Position          */
#define NPD48_EFCR_EFACT_Msk             (0x1ul << NPD48_EFCR_EFACT_Pos)                   /*!< EFUSE_T::EFCR: EFACT Mask              */

#define NPD48_EFKEY_EFKEY_Pos            (0)                                               /*!< EFUSE_T::EFKEY: EFKEY Position         */
#define NPD48_EFKEY_EFKEY_Msk            (0xfful << NPD48_EFKEY_EFKEY_Pos)                 /*!< EFUSE_T::EFKEY: EFKEY Mask             */

#define NPD48_TCPCSFT_STFILT_Pos         (0)                                               /*!< EFUSE_T::TCPCSFT: STFILT Position      */
#define NPD48_TCPCSFT_STFILT_Msk         (0xfful << NPD48_TCPCSFT_STFILT_Pos)              /*!< EFUSE_T::TCPCSFT: STFILT Mask          */

#define NPD48_I2CCTL_CKPDEN_Pos          (0)                                               /*!< EFUSE_T::I2CCTL: CKPDEN Position       */
#define NPD48_I2CCTL_CKPDEN_Msk          (0x1ul << NPD48_I2CCTL_CKPDEN_Pos)                /*!< EFUSE_T::I2CCTL: CKPDEN Mask           */

#define NPD48_I2CCTL_I2CWKSEL_Pos        (1)                                               /*!< EFUSE_T::I2CCTL: I2CWKSEL Position     */
#define NPD48_I2CCTL_I2CWKSEL_Msk        (0x1ul << NPD48_I2CCTL_I2CWKSEL_Pos)              /*!< EFUSE_T::I2CCTL: I2CWKSEL Mask         */

#define NPD48_I2CCTL_PECEN_Pos           (2)                                               /*!< EFUSE_T::I2CCTL: PECEN Position        */
#define NPD48_I2CCTL_PECEN_Msk           (0x1ul << NPD48_I2CCTL_PECEN_Pos)                 /*!< EFUSE_T::I2CCTL: PECEN Mask            */

#define NPD48_I2CCTL_PECSEL_Pos          (3)                                               /*!< EFUSE_T::I2CCTL: PECSEL Position       */
#define NPD48_I2CCTL_PECSEL_Msk          (0x1ul << NPD48_I2CCTL_PECSEL_Pos)                /*!< EFUSE_T::I2CCTL: PECSEL Mask           */

/**@}*/ /* EFUSE_CONST */
/**@}*/ /* end of EFUSE register group */
