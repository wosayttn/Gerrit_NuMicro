/*---------------------- Inter-IC Bus Controller -------------------------*/
/**
    @addtogroup I2C Inter-IC Bus Controller(I2C)
    Memory Mapped Structure for I2C Controller
@{ */
 
/**
 * @var I2C_T::I2COCR
 * Offset: 0xFB  I2C Option Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |SDETSEL   |START and   STOP Event Detect Phase Selection
 * |        |          |00 = Detect   phase 0. Use delay 6 & delay 9 do XOR.
 * |        |          |01 = Detect   phase 0. Use delay 9 & delay 12 do XOR.
 * |        |          |10 = Detect   phase 0. Use delay 12 & delay 15 do XOR.
 * |        |          |11 = Detect   phase 0. Use delay 15 & delay 18 do XOR.
 * |        |          |Not release   to customer
 */

#define NPD48_I2COCR       0x00fb           /*!< [0x00fb] I2C Option Control Register                                      */
                                                                                         
/**
    @addtogroup I2C_CONST I2C Bit Field Definition
    Constant Definitions for I2C Controller
@{ */

#define NPD48_I2COCR_SDETSEL_Pos           (0)                                               /*!< I2C_T::I2COCR: SDETSEL Position        */
#define NPD48_I2COCR_SDETSEL_Msk           (0x3ul << NPD48_I2COCR_SDETSEL_Pos)               /*!< I2C_T::I2COCR: SDETSEL Mask            */

/**@}*/ /* I2C_CONST */
/**@}*/ /* end of I2C register group */
