/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GPIO General Purpose Input/Output Controller(GPIO)
    Memory Mapped Structure for GPIO Controller
@{ */

/**
 * @var GPIO_T::IODIR0
 * Offset: 0xB0  GPIO Direction Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IODIRA0   |GPIOA0 Direction Control Bit
 * |        |          |0 = GPIOA0 is   open drain mode.
 * |        |          |1 = GPIOA0 is input mode.
 * |        |          |Default is CC1 function
 * |[1]     |IODIRA1   |GPIOA1 Direction Control Bit
 * |        |          |0 = GPIOA1 is   open drain mode.
 * |        |          |1 = GPIOA1 is input mode.
 * |        |          |Default is CC2 function
 * |[3]     |IODIRA3   |GPIOA3 Direction Control Bit
 * |        |          |0 = GPIOA3 is   open drain mode.
 * |        |          |1 = GPIOA3 is input mode.
 * |        |          |Default is VBUS_Gate   function
 * |[4]     |IODIRA4   |GPIOA4 Direction Control Bit
 * |        |          |0 = GPIOA4 is   open drain mode.
 * |        |          |1 = GPIOA4 is input mode.
 * |        |          |Default is Vin_Gate   function
 * |[6]     |IODIRA6   |GPIOA6 Direction Control Bit
 * |        |          |0 = GPIOA6 is   open drain mode.
 * |        |          |1 = GPIOA6 is input mode.
 * |        |          |Default is IS+ function
 * |[7]     |IODIRA7   |GPIOA7 Direction Control Bit
 * |        |          |0 = GPIOA7 is   open drain mode.
 * |        |          |1 = GPIOA7 is input mode.
 * |        |          |Default is IS- function
 * @var GPIO_T::IODIR1
 * Offset: 0xB1  GPIO Direction Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IODIRA8   |GPIOA8 Direction Control Bit
 * |        |          |0 = GPIOA8 is   open drain mode.
 * |        |          |1 = GPIOA8 is input mode.
 * |        |          |Default is VBUS   function
 * |[1]     |IODIRA9   |GPIOA9 Direction Control Bit
 * |        |          |0 = GPIOA9 is   open drain mode.
 * |        |          |1 = GPIOA9 is input mode.
 * |        |          |Default is CATH function
 * |[2]     |IODIRA10  |GPIOA10 Direction Control Bit
 * |        |          |0 = GPIOA10   is open drain mode.
 * |        |          |1 = GPIOA10 is input mode.
 * |        |          |Default is UART_RX function
 * |[3]     |IODIRA11  |GPIOA11 Direction Control Bit
 * |        |          |0 = GPIOA11   is open drain mode.
 * |        |          |1 = GPIOA11 is input mode.
 * |        |          |Default is UART_TX function
 * |[4]     |IODIRA12  |GPIOA12 Direction Control Bit
 * |        |          |0 = GPIOA12   is open drain mode.
 * |        |          |1 = GPIOA12 is input mode.
 * |        |          |Default is VFB function
 * |[5]     |IODIRA13  |GPIOA13 Direction Control Bit
 * |        |          |0 = GPIOA13   is open drain mode.
 * |        |          |1 = GPIOA13 is input mode.
 * |        |          |Default is IFB function
 * |[6]     |IODIRA14  |GPIOA14 Direction Control Bit
 * |        |          |0 = GPIOA14   is open drain mode.
 * |        |          |1 = GPIOA14 is input mode.
 * |        |          |Default is NTC function
 * |[7]     |IODIRA15  |GPIOA15 Direction Control Bit
 * |        |          |0 = GPIOA15   is open drain mode.
 * |        |          |1 = GPIOA15 is input mode.
 * |        |          |Default is ADC4 function
 * @var GPIO_T::IODIR2
 * Offset: 0xB2  GPIO Direction Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IODIRA16  |GPIOA16 Direction Control Bit
 * |        |          |0 = GPIOA16   is open drain mode.
 * |        |          |1 = GPIOA16 is input mode.
 * |        |          |Default is USB_D+ or ADC5 function
 * |[1]     |IODIRA17  |GPIOA17 Direction Control Bit
 * |        |          |0 = GPIOA17   is open drain mode.
 * |        |          |1 = GPIOA17 is input mode.
 * |        |          |Default is USB_D- or ADC6   function
 * |[2]     |IODIRA18  |GPIOA18 Direction Control Bit
 * |        |          |0 = GPIOA18   is open drain mode.
 * |        |          |1 = GPIOA18 is input mode.
 * |        |          |Default is PFC_I/O function
 * @var GPIO_T::IOOUTD0
 * Offset: 0xB4  GPIO Output Data Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IOODA0    |GPIOA0 Output Data Bit
 * |        |          |0 = GPIOA0   output low.
 * |        |          |1 = GPIOA0 output high. (open drain)
 * |[1]     |IOODA1    |GPIOA1 Output Data Bit
 * |        |          |0 = GPIOA1   output low.
 * |        |          |1 = GPIOA1 output high. (open drain)
 * |[3]     |IOODA3    |GPIOA3 Output Data Bit
 * |        |          |0 = GPIOA3   output low.
 * |        |          |1 = GPIOA3 output high. (open drain)
 * |[4]     |IOODA4    |GPIOA4 Output Data Bit
 * |        |          |0 = GPIOA4   output low.
 * |        |          |1 = GPIOA4 output high. (open drain)
 * |[6]     |IOODA6    |GPIOA6 Output Data Bit
 * |        |          |0 = GPIOA6   output low.
 * |        |          |1 = GPIOA6 output high. (open drain)
 * |[7]     |IOODA7    |GPIOA7 Output Data Bit
 * |        |          |0 = GPIOA7   output low.
 * |        |          |1 = GPIOA7 output high. (open drain)
 * @var GPIO_T::IOOUTD1
 * Offset: 0xB5  GPIO Output Data Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IOODA8    |GPIOA8 Output Data Bit
 * |        |          |0 = GPIOA8   output low.
 * |        |          |1 = GPIOA8 output high. (open drain)
 * |[1]     |IOODA9    |GPIOA9 Output Data Bit
 * |        |          |0 = GPIOA9   output low.
 * |        |          |1 = GPIOA9 output high. (open drain)
 * |[2]     |IOODA10   |GPIOA10 Output Data Bit
 * |        |          |0 = GPIOA10   output low.
 * |        |          |1 = GPIOA10 output high. (open drain)
 * |[3]     |IOODA11   |GPIOA11 Output Data Bit
 * |        |          |0 = GPIOA11   output low.
 * |        |          |1 = GPIOA11 output high. (open drain)
 * |[4]     |IOODA12   |GPIOA12 Output Data Bit
 * |        |          |0 = GPIOA12   output low.
 * |        |          |1 = GPIOA12 output high. (open drain)
 * |[5]     |IOODA13   |GPIOA13 Output Data Bit
 * |        |          |0 = GPIOA13   output low.
 * |        |          |1 = GPIOA13 output high. (open drain)
 * |[6]     |IOODA14   |GPIOA14 Output Data Bit
 * |        |          |0 = GPIOA14   output low.
 * |        |          |1 = GPIOA14 output high. (open drain)
 * |[7]     |IOODA15   |GPIOA15 Output Data Bit
 * |        |          |0 = GPIOA15   output low.
 * |        |          |1 = GPIOA15 output high. (open drain)
 * @var GPIO_T::IOOUTD2
 * Offset: 0xB6  GPIO Output Data Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IOODA16   |GPIOA16 Output Data Bit
 * |        |          |0 = GPIOA16   output low.
 * |        |          |1 = GPIOA16 output high. (open drain)
 * |[1]     |IOODA17   |GPIOA17 Output Data Bit
 * |        |          |0 = GPIOA17   output low.
 * |        |          |1 = GPIOA17 output high. (open drain)
 * |[2]     |IOODA18   |GPIOA18 Output Data Bit
 * |        |          |0 = GPIOA18   output low.
 * |        |          |1 = GPIOA18 output high. (open drain)
 * @var GPIO_T::IOIND0
 * Offset: 0xB8  GPIO Input Data Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IOODA0    |GPIOA0 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[1]     |IOODA1    |GPIOA1 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[3]     |IOODA3    |GPIOA3 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[4]     |IOODA4    |GPIOA4 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[6]     |IOODA6    |GPIOA6 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[7]     |IOODA7    |GPIOA7 Input Data Bit
 * |        |          |This bit   reflects the actual status of the respective I/O pin
 * |        |          |If the bit is 1, it   indicates the corresponding pin status is high; else the pin status is low.
 * @var GPIO_T::IOIND1
 * Offset: 0xB9  GPIO Input Data Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IOODA8    |GPIOA8 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[1]     |IOODA9    |GPIOA9 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[2]     |IOODA10   |GPIOA10 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[3]     |IOODA11   |GPIOA11 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[4]     |IOODA12   |GPIOA12 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin status   is high; else the pin status is low.
 * |[5]     |IOODA13   |GPIOA13 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[6]     |IOODA14   |GPIOA14 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[7]     |IOODA15   |GPIOA15 Input Data Bit
 * |        |          |This bit   reflects the actual status of the respective I/O pin
 * |        |          |If the bit is 1, it   indicates the corresponding pin status is high; else the pin status is low.
 * @var GPIO_T::IOIND2
 * Offset: 0xBA  GPIO Input Data Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IOODA16   |GPIOA16 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[1]     |IOODA17   |GPIOA17 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * |[2]     |IOODA18   |GPIOA18 Output Data Bit
 * |        |          |This bit reflects the actual status of the   respective I/O pin
 * |        |          |If the bit is 1, it indicates the corresponding pin   status is high; else the pin status is low.
 * @var GPIO_T::IOMFP0
 * Offset: 0xBC  GPIO Multi-function Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IOMFPA0   |GPIOA0 Multi-function Bit
 * |        |          |0 = GPIOA0 is   GPIO mode.
 * |        |          |1 = GPIOA0 is analog mode. (CC1 function)
 * |[1]     |IOMFPA1   |GPIOA1 Multi-function Bit
 * |        |          |0 = GPIOA1 is   GPIO mode.
 * |        |          |1 = GPIOA1 is analog mode. (CC2 function)
 * |[3]     |IOMFPA3   |GPIOA3 Multi-function Bit
 * |        |          |0 = GPIOA3 is   GPIO mode.
 * |        |          |1 = GPIOA3 is analog mode. (VBUS_Gate   function)
 * |[4]     |IOMFPA4   |GPIOA4 Multi-function Bit
 * |        |          |0 = GPIOA4 is   GPIO mode.
 * |        |          |1 = GPIOA4 is analog mode. (Vin function)
 * |[6]     |IOMFPA6   |GPIOA6 Multi-function Bit
 * |        |          |0 = GPIOA6 is   GPIO mode.
 * |        |          |1 = GPIOA6 is analog mode. (IS+ function)
 * |[7]     |IOMFP7    |GPIOA7 Multi-function Bit
 * |        |          |0 = GPIOA7 is   GPIO mode.
 * |        |          |1 = GPIOA7 is analog mode. (IS- function)
 * @var GPIO_T::IOMFP1
 * Offset: 0xBD  GPIO Multi-function Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IOMFPA8   |GPIOA8 Multi-function Bit
 * |        |          |0 = GPIOA8 is   GPIO mode.
 * |        |          |1 = GPIOA8 is analog mode. (VBUS   function)
 * |[1]     |IOMFPA9   |GPIOA9 Multi-function Bit
 * |        |          |0 = GPIOA9 is   GPIO mode.
 * |        |          |1 = GPIOA9 is analog mode. (CATH function)
 * |[2]     |IOMFPA10  |GPIOA10 Multi-function Bit
 * |        |          |0 = GPIOA10   is GPIO mode.
 * |        |          |1 = GPIOA10 is analog mode. (UART_RX   function)
 * |[3]     |IOMFPA11  |GPIOA11 Multi-function Bit
 * |        |          |0 = GPIOA11   is GPIO mode.
 * |        |          |1 = GPIOA11 is analog mode. (UART_TX   function)
 * |[4]     |IOMFPA12  |GPIOA12 Multi-function Bit
 * |        |          |0 = GPIOA12   is GPIO mode.
 * |        |          |1 = GPIOA12 is analog mode. (VFB function)
 * |[5]     |IOMFPA13  |GPIOA13 Multi-function Bit
 * |        |          |0 = GPIOA13   is GPIO mode.
 * |        |          |1 = GPIOA13 is analog mode. (IFB function)
 * |[6]     |IOMFPA14  |GPIOA14 Multi-function Bit
 * |        |          |0 = GPIOA14   is GPIO mode.
 * |        |          |1 = GPIOA14 is analog mode. (NTC function)
 * |[7]     |IOMFPA15  |GPIOA15 Multi-function Bit
 * |        |          |0 = GPIOA15   is GPIO mode.
 * |        |          |1 = GPIOA15 is analog mode. (ADC4   function)
 * @var GPIO_T::IOMFP2
 * Offset: 0xBE  GPIO Multi-function Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IOMFPA16  |GPIOA16 Multi-function Bit
 * |        |          |0 = GPIOA16   is GPIO mode.
 * |        |          |1 = GPIOA16 is analog mode. (UAB_D+ or   ADC5 function)
 * |[1]     |IOMFPA17  |GPIOA17 Multi-function Bit
 * |        |          |0 = GPIOA17   is GPIO mode.
 * |        |          |1 = GPIOA17 is analog mode. (UAB_D- or   ADC6 function)
 * |[2]     |IOMFPA18  |GPIOA18 Multi-function Bit
 * |        |          |0 = GPIOA18   is GPIO mode.
 * |        |          |1 = GPIOA18 is analog mode. (PFC_I/O   function)
 * |[6]     |CC1DISC   |CC1 pin FRS Discharge Function Enable Bit
 * |        |          |0 = CC1   pin FRS Discharge Function Disable.
 * |        |          |1 = CC1 pin FRS Discharge Function Enable.
 * |        |          |If CC1DISC enable, CC1 I/O will do discharge 85us   when TCPD trigger a FRS action.
 * |[7]     |CC2DISC   |CC2 pin FRS Discharge Function Enable Bit
 * |        |          |0 = CC2   pin FRS Discharge Function Disable.
 * |        |          |1 = CC2 pin FRS Discharge Function Enable.
 * |        |          |If CC2DISC enable, CC2 I/O will do discharge 85us   when TCPD trigger a FRS action.
 */
#define NPD48_IODIR0       0x00b0           /*!< [0x00b0] GPIO Direction Control Register 0                                */
#define NPD48_IODIR1       0x00b1           /*!< [0x00b1] GPIO Direction Control Register 1                                */
#define NPD48_IODIR2       0x00b2           /*!< [0x00b2] GPIO Direction Control Register 2                                */
#define NPD48_IOOUTD0      0x00b4           /*!< [0x00b4] GPIO Output Data Register 0                                      */
#define NPD48_IOOUTD1      0x00b5           /*!< [0x00b5] GPIO Output Data Register 1                                      */
#define NPD48_IOOUTD2      0x00b6           /*!< [0x00b6] GPIO Output Data Register 2                                      */
#define NPD48_IOIND0       0x00b8           /*!< [0x00b8] GPIO Input Data Register 0                                       */
#define NPD48_IOIND1       0x00b9           /*!< [0x00b9] GPIO Input Data Register 1                                       */
#define NPD48_IOIND2       0x00ba           /*!< [0x00ba] GPIO Input Data Register 2                                       */
#define NPD48_IOMFP0       0x00bc           /*!< [0x00bc] GPIO Multi-function Register 0                                   */
#define NPD48_IOMFP1       0x00bd           /*!< [0x00bd] GPIO Multi-function Register 1                                   */
#define NPD48_IOMFP2       0x00be           /*!< [0x00be] GPIO Multi-function Register 2                                   */

/**
    @addtogroup GPIO_CONST GPIO Bit Field Definition
    Constant Definitions for GPIO Controller
@{ */

#define NPD48_IODIR0_IODIRA0_Pos          (0)                                               /*!< GPIO_T::IODIR0: IODIRA0 Position       */
#define NPD48_IODIR0_IODIRA0_Msk          (0x1ul << NPD48_IODIR0_IODIRA0_Pos)               /*!< GPIO_T::IODIR0: IODIRA0 Mask           */

#define NPD48_IODIR0_IODIRA1_Pos          (1)                                               /*!< GPIO_T::IODIR0: IODIRA1 Position       */
#define NPD48_IODIR0_IODIRA1_Msk          (0x1ul << NPD48_IODIR0_IODIRA1_Pos)               /*!< GPIO_T::IODIR0: IODIRA1 Mask           */

#define NPD48_IODIR0_IODIRA3_Pos          (3)                                               /*!< GPIO_T::IODIR0: IODIRA3 Position       */
#define NPD48_IODIR0_IODIRA3_Msk          (0x1ul << NPD48_IODIR0_IODIRA3_Pos)               /*!< GPIO_T::IODIR0: IODIRA3 Mask           */

#define NPD48_IODIR0_IODIRA4_Pos          (4)                                               /*!< GPIO_T::IODIR0: IODIRA4 Position       */
#define NPD48_IODIR0_IODIRA4_Msk          (0x1ul << NPD48_IODIR0_IODIRA4_Pos)               /*!< GPIO_T::IODIR0: IODIRA4 Mask           */

#define NPD48_IODIR0_IODIRA6_Pos          (6)                                               /*!< GPIO_T::IODIR0: IODIRA6 Position       */
#define NPD48_IODIR0_IODIRA6_Msk          (0x1ul << NPD48_IODIR0_IODIRA6_Pos)               /*!< GPIO_T::IODIR0: IODIRA6 Mask           */

#define NPD48_IODIR0_IODIRA7_Pos          (7)                                               /*!< GPIO_T::IODIR0: IODIRA7 Position       */
#define NPD48_IODIR0_IODIRA7_Msk          (0x1ul << NPD48_IODIR0_IODIRA7_Pos)               /*!< GPIO_T::IODIR0: IODIRA7 Mask           */

#define NPD48_IODIR1_IODIRA8_Pos          (0)                                               /*!< GPIO_T::IODIR1: IODIRA8 Position       */
#define NPD48_IODIR1_IODIRA8_Msk          (0x1ul << NPD48_IODIR1_IODIRA8_Pos)               /*!< GPIO_T::IODIR1: IODIRA8 Mask           */

#define NPD48_IODIR1_IODIRA9_Pos          (1)                                               /*!< GPIO_T::IODIR1: IODIRA9 Position       */
#define NPD48_IODIR1_IODIRA9_Msk          (0x1ul << NPD48_IODIR1_IODIRA9_Pos)               /*!< GPIO_T::IODIR1: IODIRA9 Mask           */

#define NPD48_IODIR1_IODIRA10_Pos         (2)                                               /*!< GPIO_T::IODIR1: IODIRA10 Position      */
#define NPD48_IODIR1_IODIRA10_Msk         (0x1ul << NPD48_IODIR1_IODIRA10_Pos)              /*!< GPIO_T::IODIR1: IODIRA10 Mask          */

#define NPD48_IODIR1_IODIRA11_Pos         (3)                                               /*!< GPIO_T::IODIR1: IODIRA11 Position      */
#define NPD48_IODIR1_IODIRA11_Msk         (0x1ul << NPD48_IODIR1_IODIRA11_Pos)              /*!< GPIO_T::IODIR1: IODIRA11 Mask          */

#define NPD48_IODIR1_IODIRA12_Pos         (4)                                               /*!< GPIO_T::IODIR1: IODIRA12 Position      */
#define NPD48_IODIR1_IODIRA12_Msk         (0x1ul << NPD48_IODIR1_IODIRA12_Pos)              /*!< GPIO_T::IODIR1: IODIRA12 Mask          */

#define NPD48_IODIR1_IODIRA13_Pos         (5)                                               /*!< GPIO_T::IODIR1: IODIRA13 Position      */
#define NPD48_IODIR1_IODIRA13_Msk         (0x1ul << NPD48_IODIR1_IODIRA13_Pos)              /*!< GPIO_T::IODIR1: IODIRA13 Mask          */

#define NPD48_IODIR1_IODIRA14_Pos         (6)                                               /*!< GPIO_T::IODIR1: IODIRA14 Position      */
#define NPD48_IODIR1_IODIRA14_Msk         (0x1ul << NPD48_IODIR1_IODIRA14_Pos)              /*!< GPIO_T::IODIR1: IODIRA14 Mask          */

#define NPD48_IODIR1_IODIRA15_Pos         (7)                                               /*!< GPIO_T::IODIR1: IODIRA15 Position      */
#define NPD48_IODIR1_IODIRA15_Msk         (0x1ul << NPD48_IODIR1_IODIRA15_Pos)              /*!< GPIO_T::IODIR1: IODIRA15 Mask          */

#define NPD48_IODIR2_IODIRA16_Pos         (0)                                               /*!< GPIO_T::IODIR2: IODIRA16 Position      */
#define NPD48_IODIR2_IODIRA16_Msk         (0x1ul << NPD48_IODIR2_IODIRA16_Pos)              /*!< GPIO_T::IODIR2: IODIRA16 Mask          */

#define NPD48_IODIR2_IODIRA17_Pos         (1)                                               /*!< GPIO_T::IODIR2: IODIRA17 Position      */
#define NPD48_IODIR2_IODIRA17_Msk         (0x1ul << NPD48_IODIR2_IODIRA17_Pos)              /*!< GPIO_T::IODIR2: IODIRA17 Mask          */

#define NPD48_IODIR2_IODIRA18_Pos         (2)                                               /*!< GPIO_T::IODIR2: IODIRA18 Position      */
#define NPD48_IODIR2_IODIRA18_Msk         (0x1ul << NPD48_IODIR2_IODIRA18_Pos)              /*!< GPIO_T::IODIR2: IODIRA18 Mask          */

#define NPD48_IOOUTD0_IOODA0_Pos          (0)                                               /*!< GPIO_T::IOOUTD0: IOODA0 Position       */
#define NPD48_IOOUTD0_IOODA0_Msk          (0x1ul << NPD48_IOOUTD0_IOODA0_Pos)               /*!< GPIO_T::IOOUTD0: IOODA0 Mask           */

#define NPD48_IOOUTD0_IOODA1_Pos          (1)                                               /*!< GPIO_T::IOOUTD0: IOODA1 Position       */
#define NPD48_IOOUTD0_IOODA1_Msk          (0x1ul << NPD48_IOOUTD0_IOODA1_Pos)               /*!< GPIO_T::IOOUTD0: IOODA1 Mask           */

#define NPD48_IOOUTD0_IOODA3_Pos          (3)                                               /*!< GPIO_T::IOOUTD0: IOODA3 Position       */
#define NPD48_IOOUTD0_IOODA3_Msk          (0x1ul << NPD48_IOOUTD0_IOODA3_Pos)               /*!< GPIO_T::IOOUTD0: IOODA3 Mask           */

#define NPD48_IOOUTD0_IOODA4_Pos          (4)                                               /*!< GPIO_T::IOOUTD0: IOODA4 Position       */
#define NPD48_IOOUTD0_IOODA4_Msk          (0x1ul << NPD48_IOOUTD0_IOODA4_Pos)               /*!< GPIO_T::IOOUTD0: IOODA4 Mask           */

#define NPD48_IOOUTD0_IOODA6_Pos          (6)                                               /*!< GPIO_T::IOOUTD0: IOODA6 Position       */
#define NPD48_IOOUTD0_IOODA6_Msk          (0x1ul << NPD48_IOOUTD0_IOODA6_Pos)               /*!< GPIO_T::IOOUTD0: IOODA6 Mask           */

#define NPD48_IOOUTD0_IOODA7_Pos          (7)                                               /*!< GPIO_T::IOOUTD0: IOODA7 Position       */
#define NPD48_IOOUTD0_IOODA7_Msk          (0x1ul << NPD48_IOOUTD0_IOODA7_Pos)               /*!< GPIO_T::IOOUTD0: IOODA7 Mask           */

#define NPD48_IOOUTD1_IOODA8_Pos          (0)                                               /*!< GPIO_T::IOOUTD1: IOODA8 Position       */
#define NPD48_IOOUTD1_IOODA8_Msk          (0x1ul << NPD48_IOOUTD1_IOODA8_Pos)               /*!< GPIO_T::IOOUTD1: IOODA8 Mask           */

#define NPD48_IOOUTD1_IOODA9_Pos          (1)                                               /*!< GPIO_T::IOOUTD1: IOODA9 Position       */
#define NPD48_IOOUTD1_IOODA9_Msk          (0x1ul << NPD48_IOOUTD1_IOODA9_Pos)               /*!< GPIO_T::IOOUTD1: IOODA9 Mask           */

#define NPD48_IOOUTD1_IOODA10_Pos         (2)                                               /*!< GPIO_T::IOOUTD1: IOODA10 Position      */
#define NPD48_IOOUTD1_IOODA10_Msk         (0x1ul << NPD48_IOOUTD1_IOODA10_Pos)              /*!< GPIO_T::IOOUTD1: IOODA10 Mask          */

#define NPD48_IOOUTD1_IOODA11_Pos         (3)                                               /*!< GPIO_T::IOOUTD1: IOODA11 Position      */
#define NPD48_IOOUTD1_IOODA11_Msk         (0x1ul << NPD48_IOOUTD1_IOODA11_Pos)              /*!< GPIO_T::IOOUTD1: IOODA11 Mask          */

#define NPD48_IOOUTD1_IOODA12_Pos         (4)                                               /*!< GPIO_T::IOOUTD1: IOODA12 Position      */
#define NPD48_IOOUTD1_IOODA12_Msk         (0x1ul << NPD48_IOOUTD1_IOODA12_Pos)              /*!< GPIO_T::IOOUTD1: IOODA12 Mask          */

#define NPD48_IOOUTD1_IOODA13_Pos         (5)                                               /*!< GPIO_T::IOOUTD1: IOODA13 Position      */
#define NPD48_IOOUTD1_IOODA13_Msk         (0x1ul << NPD48_IOOUTD1_IOODA13_Pos)              /*!< GPIO_T::IOOUTD1: IOODA13 Mask          */

#define NPD48_IOOUTD1_IOODA14_Pos         (6)                                               /*!< GPIO_T::IOOUTD1: IOODA14 Position      */
#define NPD48_IOOUTD1_IOODA14_Msk         (0x1ul << NPD48_IOOUTD1_IOODA14_Pos)              /*!< GPIO_T::IOOUTD1: IOODA14 Mask          */

#define NPD48_IOOUTD1_IOODA15_Pos         (7)                                               /*!< GPIO_T::IOOUTD1: IOODA15 Position      */
#define NPD48_IOOUTD1_IOODA15_Msk         (0x1ul << NPD48_IOOUTD1_IOODA15_Pos)              /*!< GPIO_T::IOOUTD1: IOODA15 Mask          */

#define NPD48_IOOUTD2_IOODA16_Pos         (0)                                               /*!< GPIO_T::IOOUTD2: IOODA16 Position      */
#define NPD48_IOOUTD2_IOODA16_Msk         (0x1ul << NPD48_IOOUTD2_IOODA16_Pos)              /*!< GPIO_T::IOOUTD2: IOODA16 Mask          */

#define NPD48_IOOUTD2_IOODA17_Pos         (1)                                               /*!< GPIO_T::IOOUTD2: IOODA17 Position      */
#define NPD48_IOOUTD2_IOODA17_Msk         (0x1ul << NPD48_IOOUTD2_IOODA17_Pos)              /*!< GPIO_T::IOOUTD2: IOODA17 Mask          */

#define NPD48_IOOUTD2_IOODA18_Pos         (2)                                               /*!< GPIO_T::IOOUTD2: IOODA18 Position      */
#define NPD48_IOOUTD2_IOODA18_Msk         (0x1ul << NPD48_IOOUTD2_IOODA18_Pos)              /*!< GPIO_T::IOOUTD2: IOODA18 Mask          */

#define NPD48_IOIND0_IOODA0_Pos           (0)                                               /*!< GPIO_T::IOIND0: IOODA0 Position        */
#define NPD48_IOIND0_IOODA0_Msk           (0x1ul << NPD48_IOIND0_IOODA0_Pos)                /*!< GPIO_T::IOIND0: IOODA0 Mask            */

#define NPD48_IOIND0_IOODA1_Pos           (1)                                               /*!< GPIO_T::IOIND0: IOODA1 Position        */
#define NPD48_IOIND0_IOODA1_Msk           (0x1ul << NPD48_IOIND0_IOODA1_Pos)                /*!< GPIO_T::IOIND0: IOODA1 Mask            */

#define NPD48_IOIND0_IOODA3_Pos           (3)                                               /*!< GPIO_T::IOIND0: IOODA3 Position        */
#define NPD48_IOIND0_IOODA3_Msk           (0x1ul << NPD48_IOIND0_IOODA3_Pos)                /*!< GPIO_T::IOIND0: IOODA3 Mask            */

#define NPD48_IOIND0_IOODA4_Pos           (4)                                               /*!< GPIO_T::IOIND0: IOODA4 Position        */
#define NPD48_IOIND0_IOODA4_Msk           (0x1ul << NPD48_IOIND0_IOODA4_Pos)                /*!< GPIO_T::IOIND0: IOODA4 Mask            */

#define NPD48_IOIND0_IOODA6_Pos           (6)                                               /*!< GPIO_T::IOIND0: IOODA6 Position        */
#define NPD48_IOIND0_IOODA6_Msk           (0x1ul << NPD48_IOIND0_IOODA6_Pos)                /*!< GPIO_T::IOIND0: IOODA6 Mask            */

#define NPD48_IOIND0_IOODA7_Pos           (7)                                               /*!< GPIO_T::IOIND0: IOODA7 Position        */
#define NPD48_IOIND0_IOODA7_Msk           (0x1ul << NPD48_IOIND0_IOODA7_Pos)                /*!< GPIO_T::IOIND0: IOODA7 Mask            */

#define NPD48_IOIND1_IOODA8_Pos           (0)                                               /*!< GPIO_T::IOIND1: IOODA8 Position        */
#define NPD48_IOIND1_IOODA8_Msk           (0x1ul << NPD48_IOIND1_IOODA8_Pos)                /*!< GPIO_T::IOIND1: IOODA8 Mask            */

#define NPD48_IOIND1_IOODA9_Pos           (1)                                               /*!< GPIO_T::IOIND1: IOODA9 Position        */
#define NPD48_IOIND1_IOODA9_Msk           (0x1ul << NPD48_IOIND1_IOODA9_Pos)                /*!< GPIO_T::IOIND1: IOODA9 Mask            */

#define NPD48_IOIND1_IOODA10_Pos          (2)                                               /*!< GPIO_T::IOIND1: IOODA10 Position       */
#define NPD48_IOIND1_IOODA10_Msk          (0x1ul << NPD48_IOIND1_IOODA10_Pos)               /*!< GPIO_T::IOIND1: IOODA10 Mask           */

#define NPD48_IOIND1_IOODA11_Pos          (3)                                               /*!< GPIO_T::IOIND1: IOODA11 Position       */
#define NPD48_IOIND1_IOODA11_Msk          (0x1ul << NPD48_IOIND1_IOODA11_Pos)               /*!< GPIO_T::IOIND1: IOODA11 Mask           */

#define NPD48_IOIND1_IOODA12_Pos          (4)                                               /*!< GPIO_T::IOIND1: IOODA12 Position       */
#define NPD48_IOIND1_IOODA12_Msk          (0x1ul << NPD48_IOIND1_IOODA12_Pos)               /*!< GPIO_T::IOIND1: IOODA12 Mask           */

#define NPD48_IOIND1_IOODA13_Pos          (5)                                               /*!< GPIO_T::IOIND1: IOODA13 Position       */
#define NPD48_IOIND1_IOODA13_Msk          (0x1ul << NPD48_IOIND1_IOODA13_Pos)               /*!< GPIO_T::IOIND1: IOODA13 Mask           */

#define NPD48_IOIND1_IOODA14_Pos          (6)                                               /*!< GPIO_T::IOIND1: IOODA14 Position       */
#define NPD48_IOIND1_IOODA14_Msk          (0x1ul << NPD48_IOIND1_IOODA14_Pos)               /*!< GPIO_T::IOIND1: IOODA14 Mask           */

#define NPD48_IOIND1_IOODA15_Pos          (7)                                               /*!< GPIO_T::IOIND1: IOODA15 Position       */
#define NPD48_IOIND1_IOODA15_Msk          (0x1ul << NPD48_IOIND1_IOODA15_Pos)               /*!< GPIO_T::IOIND1: IOODA15 Mask           */

#define NPD48_IOIND2_IOODA16_Pos          (0)                                               /*!< GPIO_T::IOIND2: IOODA16 Position       */
#define NPD48_IOIND2_IOODA16_Msk          (0x1ul << NPD48_IOIND2_IOODA16_Pos)               /*!< GPIO_T::IOIND2: IOODA16 Mask           */

#define NPD48_IOIND2_IOODA17_Pos          (1)                                               /*!< GPIO_T::IOIND2: IOODA17 Position       */
#define NPD48_IOIND2_IOODA17_Msk          (0x1ul << NPD48_IOIND2_IOODA17_Pos)               /*!< GPIO_T::IOIND2: IOODA17 Mask           */

#define NPD48_IOIND2_IOODA18_Pos          (2)                                               /*!< GPIO_T::IOIND2: IOODA18 Position       */
#define NPD48_IOIND2_IOODA18_Msk          (0x1ul << NPD48_IOIND2_IOODA18_Pos)               /*!< GPIO_T::IOIND2: IOODA18 Mask           */

#define NPD48_IOMFP0_IOMFPA0_Pos          (0)                                               /*!< GPIO_T::IOMFP0: IOMFPA0 Position       */
#define NPD48_IOMFP0_IOMFPA0_Msk          (0x1ul << NPD48_IOMFP0_IOMFPA0_Pos)               /*!< GPIO_T::IOMFP0: IOMFPA0 Mask           */

#define NPD48_IOMFP0_IOMFPA1_Pos          (1)                                               /*!< GPIO_T::IOMFP0: IOMFPA1 Position       */
#define NPD48_IOMFP0_IOMFPA1_Msk          (0x1ul << NPD48_IOMFP0_IOMFPA1_Pos)               /*!< GPIO_T::IOMFP0: IOMFPA1 Mask           */

#define NPD48_IOMFP0_IOMFPA3_Pos          (3)                                               /*!< GPIO_T::IOMFP0: IOMFPA3 Position       */
#define NPD48_IOMFP0_IOMFPA3_Msk          (0x1ul << NPD48_IOMFP0_IOMFPA3_Pos)               /*!< GPIO_T::IOMFP0: IOMFPA3 Mask           */

#define NPD48_IOMFP0_IOMFPA4_Pos          (4)                                               /*!< GPIO_T::IOMFP0: IOMFPA4 Position       */
#define NPD48_IOMFP0_IOMFPA4_Msk          (0x1ul << NPD48_IOMFP0_IOMFPA4_Pos)               /*!< GPIO_T::IOMFP0: IOMFPA4 Mask           */

#define NPD48_IOMFP0_IOMFPA6_Pos          (6)                                               /*!< GPIO_T::IOMFP0: IOMFPA6 Position       */
#define NPD48_IOMFP0_IOMFPA6_Msk          (0x1ul << NPD48_IOMFP0_IOMFPA6_Pos)               /*!< GPIO_T::IOMFP0: IOMFPA6 Mask           */

#define NPD48_IOMFP0_IOMFP7_Pos           (7)                                               /*!< GPIO_T::IOMFP0: IOMFP7 Position        */
#define NPD48_IOMFP0_IOMFP7_Msk           (0x1ul << NPD48_IOMFP0_IOMFP7_Pos)                /*!< GPIO_T::IOMFP0: IOMFP7 Mask            */

#define NPD48_IOMFP1_IOMFPA8_Pos          (0)                                               /*!< GPIO_T::IOMFP1: IOMFPA8 Position       */
#define NPD48_IOMFP1_IOMFPA8_Msk          (0x1ul << NPD48_IOMFP1_IOMFPA8_Pos)               /*!< GPIO_T::IOMFP1: IOMFPA8 Mask           */

#define NPD48_IOMFP1_IOMFPA9_Pos          (1)                                               /*!< GPIO_T::IOMFP1: IOMFPA9 Position       */
#define NPD48_IOMFP1_IOMFPA9_Msk          (0x1ul << NPD48_IOMFP1_IOMFPA9_Pos)               /*!< GPIO_T::IOMFP1: IOMFPA9 Mask           */

#define NPD48_IOMFP1_IOMFPA10_Pos         (2)                                               /*!< GPIO_T::IOMFP1: IOMFPA10 Position      */
#define NPD48_IOMFP1_IOMFPA10_Msk         (0x1ul << NPD48_IOMFP1_IOMFPA10_Pos)              /*!< GPIO_T::IOMFP1: IOMFPA10 Mask          */

#define NPD48_IOMFP1_IOMFPA11_Pos         (3)                                               /*!< GPIO_T::IOMFP1: IOMFPA11 Position      */
#define NPD48_IOMFP1_IOMFPA11_Msk         (0x1ul << NPD48_IOMFP1_IOMFPA11_Pos)              /*!< GPIO_T::IOMFP1: IOMFPA11 Mask          */

#define NPD48_IOMFP1_IOMFPA12_Pos         (4)                                               /*!< GPIO_T::IOMFP1: IOMFPA12 Position      */
#define NPD48_IOMFP1_IOMFPA12_Msk         (0x1ul << NPD48_IOMFP1_IOMFPA12_Pos)              /*!< GPIO_T::IOMFP1: IOMFPA12 Mask          */

#define NPD48_IOMFP1_IOMFPA13_Pos         (5)                                               /*!< GPIO_T::IOMFP1: IOMFPA13 Position      */
#define NPD48_IOMFP1_IOMFPA13_Msk         (0x1ul << NPD48_IOMFP1_IOMFPA13_Pos)              /*!< GPIO_T::IOMFP1: IOMFPA13 Mask          */

#define NPD48_IOMFP1_IOMFPA14_Pos         (6)                                               /*!< GPIO_T::IOMFP1: IOMFPA14 Position      */
#define NPD48_IOMFP1_IOMFPA14_Msk         (0x1ul << NPD48_IOMFP1_IOMFPA14_Pos)              /*!< GPIO_T::IOMFP1: IOMFPA14 Mask          */

#define NPD48_IOMFP1_IOMFPA15_Pos         (7)                                               /*!< GPIO_T::IOMFP1: IOMFPA15 Position      */
#define NPD48_IOMFP1_IOMFPA15_Msk         (0x1ul << NPD48_IOMFP1_IOMFPA15_Pos)              /*!< GPIO_T::IOMFP1: IOMFPA15 Mask          */

#define NPD48_IOMFP2_IOMFPA16_Pos         (0)                                               /*!< GPIO_T::IOMFP2: IOMFPA16 Position      */
#define NPD48_IOMFP2_IOMFPA16_Msk         (0x1ul << NPD48_IOMFP2_IOMFPA16_Pos)              /*!< GPIO_T::IOMFP2: IOMFPA16 Mask          */

#define NPD48_IOMFP2_IOMFPA17_Pos         (1)                                               /*!< GPIO_T::IOMFP2: IOMFPA17 Position      */
#define NPD48_IOMFP2_IOMFPA17_Msk         (0x1ul << NPD48_IOMFP2_IOMFPA17_Pos)              /*!< GPIO_T::IOMFP2: IOMFPA17 Mask          */

#define NPD48_IOMFP2_IOMFPA18_Pos         (2)                                               /*!< GPIO_T::IOMFP2: IOMFPA18 Position      */
#define NPD48_IOMFP2_IOMFPA18_Msk         (0x1ul << NPD48_IOMFP2_IOMFPA18_Pos)              /*!< GPIO_T::IOMFP2: IOMFPA18 Mask          */

#define NPD48_IOMFP2_CC1DISC_Pos          (6)                                               /*!< GPIO_T::IOMFP2: CC1DISC Position       */
#define NPD48_IOMFP2_CC1DISC_Msk          (0x1ul << NPD48_IOMFP2_CC1DISC_Pos)               /*!< GPIO_T::IOMFP2: CC1DISC Mask           */

#define NPD48_IOMFP2_CC2DISC_Pos          (7)                                               /*!< GPIO_T::IOMFP2: CC2DISC Position       */
#define NPD48_IOMFP2_CC2DISC_Msk          (0x1ul << NPD48_IOMFP2_CC2DISC_Pos)               /*!< GPIO_T::IOMFP2: CC2DISC Mask           */

/**@}*/ /* GPIO_CONST */
/**@}*/ /* end of GPIO register group */
