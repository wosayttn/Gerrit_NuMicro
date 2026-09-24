/**************************************************************************//**
 * @file     sys.h
 * @version  V1.00
 * @brief    M2U51 Series SYS Driver Header File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup SYS_Driver SYS Driver
    @{
*/

/** @addtogroup SYS_EXPORTED_CONSTANTS SYS Exported Constants
    @{
*/

/*--------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                     */
/*--------------------------------------------------------------------------*/
#define CHIP_RST        ((0UL<<24) | SYS_IPRST0_CHIPRST_Pos)        /*!< Reset CHIP \hideinitializer    */
#define CPU_RST         ((0UL<<24) | SYS_IPRST0_CPURST_Pos)         /*!< Reset CPU \hideinitializer     */
#define PDMA0_RST       ((0UL<<24) | SYS_IPRST0_PDMA0RST_Pos)       /*!< Reset PDMA0 \hideinitializer   */
#define CRC_RST         ((0UL<<24) | SYS_IPRST0_CRCRST_Pos)         /*!< Reset CRC \hideinitializer     */
#define CRPT_RST        ((0UL<<24) | SYS_IPRST0_CRPTRST_Pos)        /*!< Reset CRPT \hideinitializer    */
#define GPIO_RST        ((0UL<<24) | SYS_IPRST0_GPIORST_Pos)        /*!< Reset GPIO \hideinitializer    */

#define TMR0_RST        ((4UL<<24) | SYS_IPRST1_TMR0RST_Pos)        /*!< Reset TMR0 \hideinitializer    */
#define TMR1_RST        ((4UL<<24) | SYS_IPRST1_TMR1RST_Pos)        /*!< Reset TMR1 \hideinitializer    */
#define TMR2_RST        ((4UL<<24) | SYS_IPRST1_TMR2RST_Pos)        /*!< Reset TMR2 \hideinitializer    */
#define TMR3_RST        ((4UL<<24) | SYS_IPRST1_TMR3RST_Pos)        /*!< Reset TMR3 \hideinitializer    */
#define ACMP01_RST      ((4UL<<24) | SYS_IPRST1_ACMP01RST_Pos)      /*!< Reset ACMP01 \hideinitializer  */
#define I2C0_RST        ((4UL<<24) | SYS_IPRST1_I2C0RST_Pos)        /*!< Reset I2C0 \hideinitializer    */
#define I2C1_RST        ((4UL<<24) | SYS_IPRST1_I2C1RST_Pos)        /*!< Reset I2C1 \hideinitializer    */
#define I2C2_RST        ((4UL<<24) | SYS_IPRST1_I2C2RST_Pos)        /*!< Reset I2C2 \hideinitializer    */
#define SPI0_RST        ((4UL<<24) | SYS_IPRST1_SPI0RST_Pos)        /*!< Reset SPI0 \hideinitializer    */
#define SPI1_RST        ((4UL<<24) | SYS_IPRST1_SPI1RST_Pos)        /*!< Reset SPI1 \hideinitializer    */
#define SPI2_RST        ((4UL<<24) | SYS_IPRST1_SPI2RST_Pos)        /*!< Reset SPI2 \hideinitializer    */
#define UART0_RST       ((4UL<<24) | SYS_IPRST1_UART0RST_Pos)       /*!< Reset UART0 \hideinitializer   */
#define UART1_RST       ((4UL<<24) | SYS_IPRST1_UART1RST_Pos)       /*!< Reset UART1 \hideinitializer   */
#define UART2_RST       ((4UL<<24) | SYS_IPRST1_UART2RST_Pos)       /*!< Reset UART2 \hideinitializer   */
#define USCI0_RST       ((4UL<<24) | SYS_IPRST1_USCI0RST_Pos)       /*!< Reset USCI0 \hideinitializer   */
#define WWDT_RST        ((4UL<<24) | SYS_IPRST1_WWDTRST_Pos)        /*!< Reset WWDT \hideinitializer    */
#define PWM0_RST        ((4UL<<24) | SYS_IPRST1_PWM0RST_Pos)        /*!< Reset PWM0 \hideinitializer    */
#define BPWM0_RST       ((4UL<<24) | SYS_IPRST1_BPWM0RST_Pos)       /*!< Reset BPWM0 \hideinitializer   */
#define LCD_RST         ((4UL<<24) | SYS_IPRST1_LCDRST_Pos)         /*!< Reset LCD \hideinitializer     */
#define ADC0_RST        ((4UL<<24) | SYS_IPRST1_ADC0RST_Pos)        /*!< Reset ADC0 \hideinitializer    */

/*--------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.    */
/*--------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_RST_EN           (1UL << SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Reset Enable     \hideinitializer */
#define SYS_BODCTL_BOD_INTERRUPT_EN     (0UL << SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Interrupt Enable \hideinitializer */

#define SYS_BODCTL_BOD_BOTHEDGE         (0UL << SYS_BODCTL_BODSIFEN_Pos)    /*!< Brown-out Detector Edge Selection for Interrupt is BODOUT Both Edge.       \hideinitializer */
#define SYS_BODCTL_BOD_DOWN_THROUGH     (1UL << SYS_BODCTL_BODSIFEN_Pos)    /*!< Brown-out Detector Edge Selection for Interrupt is AVdd   Down Through.    \hideinitializer */
#define SYS_BODCTL_BOD_RISING           (SYS_BODCTL_BOD_DOWN_THROUGH)       /*!< Brown-out Detector Edge Selection for Interrupt is BODOUT Rising Edge.     \hideinitializer */
#define SYS_BODCTL_BOD_UP_THROUGH       (2UL << SYS_BODCTL_BODSIFEN_Pos)    /*!< Brown-out Detector Edge Selection for Interrupt is AVdd   Up Through.      \hideinitializer */
#define SYS_BODCTL_BOD_FALLING          (SYS_BODCTL_BOD_UP_THROUGH)         /*!< Brown-out Detector Edge Selection for Interrupt is BODOUT Falling Edge.    \hideinitializer */

#define SYS_BODCTL_BODVL_4_4V           (7UL << SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 4.4V \hideinitializer */
#define SYS_BODCTL_BODVL_3_7V           (6UL << SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.7V \hideinitializer */
#define SYS_BODCTL_BODVL_3_0V           (5UL << SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.0V \hideinitializer */
#define SYS_BODCTL_BODVL_2_7V           (4UL << SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.7V \hideinitializer */
#define SYS_BODCTL_BODVL_2_4V           (3UL << SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.4V \hideinitializer */
#define SYS_BODCTL_BODVL_2_0V           (2UL << SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.0V \hideinitializer */
#define SYS_BODCTL_BODVL_1_8V           (1UL << SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 1.8V \hideinitializer */

#define SYS_BODCTL_LVRDGSEL_0HCLK       (0UL<<SYS_BODCTL_LVRDGSEL_Pos)      /*!< LVR Output De-glitch Time Without de-glitch function.  \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_4HCLK       (1UL<<SYS_BODCTL_LVRDGSEL_Pos)      /*!< LVR Output De-glitch Time is selected 4HCLK            \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_8HCLK       (2UL<<SYS_BODCTL_LVRDGSEL_Pos)      /*!< LVR Output De-glitch Time is selected 8HCLK            \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_16HCLK      (3UL<<SYS_BODCTL_LVRDGSEL_Pos)      /*!< LVR Output De-glitch Time is selected 16HCLK           \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_32HCLK      (4UL<<SYS_BODCTL_LVRDGSEL_Pos)      /*!< LVR Output De-glitch Time is selected 32HCLK           \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_64HCLK      (5UL<<SYS_BODCTL_LVRDGSEL_Pos)      /*!< LVR Output De-glitch Time is selected 64HCLK           \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_128HCLK     (6UL<<SYS_BODCTL_LVRDGSEL_Pos)      /*!< LVR Output De-glitch Time is selected 128HCLK          \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_256HCLK     (7UL<<SYS_BODCTL_LVRDGSEL_Pos)      /*!< LVR Output De-glitch Time is selected 256HCLK          \hideinitializer */

#define SYS_BODCTL_BODDGSEL_0HCLK       (0UL<<SYS_BODCTL_BODDGSEL_Pos)      /*!< BOD Output De-glitch Time is sampled by RC10K clock.   \hideinitializer */
#define SYS_BODCTL_BODDGSEL_4HCLK       (1UL<<SYS_BODCTL_BODDGSEL_Pos)      /*!< BOD Output De-glitch Time is selected 4HCLK            \hideinitializer */
#define SYS_BODCTL_BODDGSEL_8HCLK       (2UL<<SYS_BODCTL_BODDGSEL_Pos)      /*!< BOD Output De-glitch Time is selected 8HCLK            \hideinitializer */
#define SYS_BODCTL_BODDGSEL_16HCLK      (3UL<<SYS_BODCTL_BODDGSEL_Pos)      /*!< BOD Output De-glitch Time is selected 16HCLK           \hideinitializer */
#define SYS_BODCTL_BODDGSEL_32HCLK      (4UL<<SYS_BODCTL_BODDGSEL_Pos)      /*!< BOD Output De-glitch Time is selected 32HCLK           \hideinitializer */
#define SYS_BODCTL_BODDGSEL_64HCLK      (5UL<<SYS_BODCTL_BODDGSEL_Pos)      /*!< BOD Output De-glitch Time is selected 64HCLK           \hideinitializer */
#define SYS_BODCTL_BODDGSEL_128HCLK     (6UL<<SYS_BODCTL_BODDGSEL_Pos)      /*!< BOD Output De-glitch Time is selected 128HCLK          \hideinitializer */
#define SYS_BODCTL_BODDGSEL_256HCLK     (7UL<<SYS_BODCTL_BODDGSEL_Pos)      /*!< BOD Output De-glitch Time is selected 256HCLK          \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  Internal Voltage Source Control constant definitions.                   */
/*--------------------------------------------------------------------------*/
#define SYS_IVSCTL_AVDDDIV4_DISABLE     (0x0UL << SYS_IVSCTL_AVDDDIV4EN_Pos)    /*!< IVSCTL AVdd divide 4 disable  \hideinitializer */
#define SYS_IVSCTL_AVDDDIV4_ENABLE      (0x1UL << SYS_IVSCTL_AVDDDIV4EN_Pos)    /*!< IVSCTL AVdd divide 4 enable \hideinitializer */

#define SYS_IVSCTL_VTEMP_DISABLE        (0x0UL << SYS_IVSCTL_VTEMPEN_Pos)       /*!< IVSCTL Temperature Sensor disable  \hideinitializer */
#define SYS_IVSCTL_VTEMP_ENABLE         (0x1UL << SYS_IVSCTL_VTEMPEN_Pos)       /*!< IVSCTL Temperature Sensor enable \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  VREFCTL constant definitions. (Write-Protection Register)               */
/*--------------------------------------------------------------------------*/
#define SYS_VREFCTL_VREF_1_5V           (0x0UL << SYS_VREFCTL_VREFSEL_Pos)  /*!< Vref = 1.536V  \hideinitializer */
#define SYS_VREFCTL_VREF_2_0V           (0x1UL << SYS_VREFCTL_VREFSEL_Pos)  /*!< Vref = 2.048V  \hideinitializer */
#define SYS_VREFCTL_VREF_2_5V           (0x2UL << SYS_VREFCTL_VREFSEL_Pos)  /*!< Vref = 2.560V  \hideinitializer */
#define SYS_VREFCTL_VREF_3_0V           (0x3UL << SYS_VREFCTL_VREFSEL_Pos)  /*!< Vref = 3.072V  \hideinitializer */
#define SYS_VREFCTL_VREF_4_0V           (0x4UL << SYS_VREFCTL_VREFSEL_Pos)  /*!< Vref = 4.096V  \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  PLCTL constant definitions. (Write-Protection Register)                 */
/*--------------------------------------------------------------------------*/
#define SYS_PLCTL_MVRS_LDO              (0x0UL<<SYS_PLCTL_MVRS_Pos)         /*!< Set main voltage regulator type to LDO */
#define SYS_PLCTL_MVRS_DCDC             (0x1UL<<SYS_PLCTL_MVRS_Pos)         /*!< Set main voltage regulator type to DCDC */

/*--------------------------------------------------------------------------*/
/*  PLSTS constant definitions. (Write-Protection Register)                 */
/*--------------------------------------------------------------------------*/
#define SYS_PLSTS_CURMVR_LDO            (0x0UL<<SYS_PLSTS_CURMVR_Pos)       /*!< Main voltage regulator type is LDO */
#define SYS_PLSTS_CURMVR_DCDC           (0x1UL<<SYS_PLSTS_CURMVR_Pos)       /*!< Main voltage regulator type is DCDC */

/*--------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                    */
/*--------------------------------------------------------------------------*/
/* How to use below #define?
Example 1: If user want to set PA.0 as SC0_CLK in initial function,
           user can issue following command to achieve it.

           SYS->GPA_MFPL  = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk) ) | SYS_GPA_MFPL_PA0_MFP_SC0_CLK  ;

*/

/* PA.0 MFP */
#define SYS_GPA_MFPL_PA0MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for GPIO              */
#define SYS_GPA_MFPL_PA0MFP_SPI0_MOSI       (0x4UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for SPI0_MOSI         */
#define SYS_GPA_MFPL_PA0MFP_UART0_RXD       (0x7UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for UART0_RXD         */
#define SYS_GPA_MFPL_PA0MFP_UART1_nRTS      (0x8UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for UART1_nRTS        */
#define SYS_GPA_MFPL_PA0MFP_I2C2_SDA        (0x9UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for I2C2_SDA          */
#define SYS_GPA_MFPL_PA0MFP_BPWM0_CH0       (0xCUL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for BPWM0_CH0         */
#define SYS_GPA_MFPL_PA0MFP_PWM0_CH5        (0xDUL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for PWM0_CH5          */

/* PA.1 MFP */
#define SYS_GPA_MFPL_PA1MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for GPIO              */
#define SYS_GPA_MFPL_PA1MFP_PINV            (0x2UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for PINV              */
#define SYS_GPA_MFPL_PA1MFP_SPI0_MISO       (0x4UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for SPI0_MISO         */
#define SYS_GPA_MFPL_PA1MFP_PBUF            (0x5UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for PBUF              */
#define SYS_GPA_MFPL_PA1MFP_UART0_TXD       (0x7UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for UART0_TXD         */
#define SYS_GPA_MFPL_PA1MFP_UART1_nCTS      (0x8UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for UART1_nCTS        */
#define SYS_GPA_MFPL_PA1MFP_I2C2_SCL        (0x9UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for I2C2_SCL          */
#define SYS_GPA_MFPL_PA1MFP_BPWM0_CH1       (0xCUL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for BPWM0_CH1         */
#define SYS_GPA_MFPL_PA1MFP_PWM0_CH4        (0xDUL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for PWM0_CH4          */

/* PA.2 MFP */
#define SYS_GPA_MFPL_PA2MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for GPIO              */
#define SYS_GPA_MFPL_PA2MFP_PINV            (0x2UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for PINV              */
#define SYS_GPA_MFPL_PA2MFP_SPI0_CLK        (0x4UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for SPI0_CLK          */
#define SYS_GPA_MFPL_PA2MFP_PBUF            (0x5UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for PBUF              */
#define SYS_GPA_MFPL_PA2MFP_UART1_RXD       (0x7UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for UART1_RXD         */
#define SYS_GPA_MFPL_PA2MFP_I2C1_SDA        (0x9UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for I2C1_SDA          */
#define SYS_GPA_MFPL_PA2MFP_I2C0_SMBSUS     (0xAUL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for I2C0_SMBSUS       */
#define SYS_GPA_MFPL_PA2MFP_BPWM0_CH2       (0xCUL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for BPWM0_CH2         */
#define SYS_GPA_MFPL_PA2MFP_PWM0_CH3        (0xDUL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for PWM0_CH3          */

/* PA.3 MFP */
#define SYS_GPA_MFPL_PA3MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for GPIO              */
#define SYS_GPA_MFPL_PA3MFP_PINV            (0x2UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for PINV              */
#define SYS_GPA_MFPL_PA3MFP_SPI0_SS         (0x4UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for SPI0_SS           */
#define SYS_GPA_MFPL_PA3MFP_PBUF            (0x5UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for PBUF              */
#define SYS_GPA_MFPL_PA3MFP_UART1_TXD       (0x7UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for UART1_TXD         */
#define SYS_GPA_MFPL_PA3MFP_I2C1_SCL        (0x9UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for I2C1_SCL          */
#define SYS_GPA_MFPL_PA3MFP_I2C0_SMBAL      (0xAUL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for I2C0_SMBAL        */
#define SYS_GPA_MFPL_PA3MFP_BPWM0_CH3       (0xCUL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for BPWM0_CH3         */
#define SYS_GPA_MFPL_PA3MFP_PWM0_CH2        (0xDUL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for PWM0_CH2          */

/* PA.4 MFP */
#define SYS_GPA_MFPL_PA4MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for GPIO              */
#define SYS_GPA_MFPL_PA4MFP_PINV            (0x2UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for PINV              */
#define SYS_GPA_MFPL_PA4MFP_PBUF            (0x5UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for PBUF              */
#define SYS_GPA_MFPL_PA4MFP_UART0_nRTS      (0x7UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for UART0_nRTS        */
#define SYS_GPA_MFPL_PA4MFP_I2C0_SDA        (0x9UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for I2C0_SDA          */
#define SYS_GPA_MFPL_PA4MFP_UART0_RXD       (0xBUL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for UART0_RXD         */
#define SYS_GPA_MFPL_PA4MFP_BPWM0_CH4       (0xCUL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for BPWM0_CH4         */
#define SYS_GPA_MFPL_PA4MFP_PWM0_CH1        (0xDUL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for PWM0_CH1          */

/* PA.5 MFP */
#define SYS_GPA_MFPL_PA5MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for GPIO              */
#define SYS_GPA_MFPL_PA5MFP_PINV            (0x2UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for PINV              */
#define SYS_GPA_MFPL_PA5MFP_PBUF            (0x5UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for PBUF              */
#define SYS_GPA_MFPL_PA5MFP_UART0_nCTS      (0x7UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for UART0_nCTS        */
#define SYS_GPA_MFPL_PA5MFP_I2C0_SCL        (0x9UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for I2C0_SCL          */
#define SYS_GPA_MFPL_PA5MFP_UART0_TXD       (0xBUL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for UART0_TXD         */
#define SYS_GPA_MFPL_PA5MFP_BPWM0_CH5       (0xCUL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for BPWM0_CH5         */
#define SYS_GPA_MFPL_PA5MFP_PWM0_CH0        (0xDUL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for PWM0_CH0          */

/* PA.6 MFP */
#define SYS_GPA_MFPL_PA6MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for GPIO              */
#define SYS_GPA_MFPL_PA6MFP_PINV            (0x2UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for PINV              */
#define SYS_GPA_MFPL_PA6MFP_SPI1_SS         (0x4UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for SPI1_SS           */
#define SYS_GPA_MFPL_PA6MFP_UART0_RXD       (0x7UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for UART0_RXD         */
#define SYS_GPA_MFPL_PA6MFP_LCD_PIN20       (0x8UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for LCD_PIN20         */
#define SYS_GPA_MFPL_PA6MFP_I2C1_SDA        (0x9UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for I2C1_SDA          */
#define SYS_GPA_MFPL_PA6MFP_PBUF            (0xAUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for PBUF              */
#define SYS_GPA_MFPL_PA6MFP_ACMP1_WLAT      (0xDUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for ACMP1_WLAT        */
#define SYS_GPA_MFPL_PA6MFP_TM3             (0xEUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for TM3               */
#define SYS_GPA_MFPL_PA6MFP_INT0            (0xFUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for INT0              */

/* PA.7 MFP */
#define SYS_GPA_MFPL_PA7MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for GPIO              */
#define SYS_GPA_MFPL_PA7MFP_PINV            (0x2UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for PINV              */
#define SYS_GPA_MFPL_PA7MFP_SPI1_CLK        (0x4UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for SPI1_CLK          */
#define SYS_GPA_MFPL_PA7MFP_UART0_TXD       (0x7UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for UART0_TXD         */
#define SYS_GPA_MFPL_PA7MFP_LCD_PIN19       (0x8UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for LCD_PIN19         */
#define SYS_GPA_MFPL_PA7MFP_I2C1_SCL        (0x9UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for I2C1_SCL          */
#define SYS_GPA_MFPL_PA7MFP_PBUF            (0xAUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for PBUF              */
#define SYS_GPA_MFPL_PA7MFP_ACMP0_WLAT      (0xDUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for ACMP0_WLAT        */
#define SYS_GPA_MFPL_PA7MFP_TM2             (0xEUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for TM2               */
#define SYS_GPA_MFPL_PA7MFP_INT1            (0xFUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for INT1              */

/* PA.8 MFP */
#define SYS_GPA_MFPH_PA8MFP_GPIO            (0x0UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for GPIO              */
#define SYS_GPA_MFPH_PA8MFP_PINV            (0x2UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for PINV              */
#define SYS_GPA_MFPH_PA8MFP_SPI2_MOSI       (0x4UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for SPI2_MOSI         */
#define SYS_GPA_MFPH_PA8MFP_USCI0_CTL1      (0x6UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for USCI0_CTL1        */
#define SYS_GPA_MFPH_PA8MFP_UART1_RXD       (0x7UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for UART1_RXD         */
#define SYS_GPA_MFPH_PA8MFP_LCD_PIN8        (0x8UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for LCD_PIN8          */
#define SYS_GPA_MFPH_PA8MFP_BPWM0_CH3       (0x9UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for BPWM0_CH3         */
#define SYS_GPA_MFPH_PA8MFP_PBUF            (0xAUL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for PBUF              */
#define SYS_GPA_MFPH_PA8MFP_TM3_EXT         (0xDUL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for TM3_EXT           */
#define SYS_GPA_MFPH_PA8MFP_INT4            (0xFUL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for INT4              */

/* PA.9 MFP */
#define SYS_GPA_MFPH_PA9MFP_GPIO            (0x0UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for GPIO              */
#define SYS_GPA_MFPH_PA9MFP_PINV            (0x2UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for PINV              */
#define SYS_GPA_MFPH_PA9MFP_SPI2_MISO       (0x4UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for SPI2_MISO         */
#define SYS_GPA_MFPH_PA9MFP_USCI0_DAT1      (0x6UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for USCI0_DAT1        */
#define SYS_GPA_MFPH_PA9MFP_UART1_TXD       (0x7UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for UART1_TXD         */
#define SYS_GPA_MFPH_PA9MFP_LCD_POWER       (0x8UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for LCD_POWER         */
#define SYS_GPA_MFPH_PA9MFP_BPWM0_CH2       (0x9UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for BPWM0_CH2         */
#define SYS_GPA_MFPH_PA9MFP_PBUF            (0xAUL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for PBUF              */
#define SYS_GPA_MFPH_PA9MFP_TM2_EXT         (0xDUL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for TM2_EXT           */

/* PA.10 MFP */
#define SYS_GPA_MFPH_PA10MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for GPIO             */
#define SYS_GPA_MFPH_PA10MFP_ACMP1_P0       (0x1UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for ACMP1_P0         */
#define SYS_GPA_MFPH_PA10MFP_PINV           (0x2UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for PINV             */
#define SYS_GPA_MFPH_PA10MFP_SPI2_CLK       (0x4UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for SPI2_CLK         */
#define SYS_GPA_MFPH_PA10MFP_USCI0_DAT0     (0x6UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for USCI0_DAT0       */
#define SYS_GPA_MFPH_PA10MFP_I2C2_SDA       (0x7UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for I2C2_SDA         */
#define SYS_GPA_MFPH_PA10MFP_LCD_POWER      (0x8UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for LCD_POWER        */
#define SYS_GPA_MFPH_PA10MFP_BPWM0_CH1      (0x9UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for BPWM0_CH1        */
#define SYS_GPA_MFPH_PA10MFP_PBUF           (0xAUL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for PBUF             */
#define SYS_GPA_MFPH_PA10MFP_TM1_EXT        (0xDUL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for TM1_EXT          */

/* PA.11 MFP */
#define SYS_GPA_MFPH_PA11MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for GPIO             */
#define SYS_GPA_MFPH_PA11MFP_ACMP0_P0       (0x1UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for ACMP0_P0         */
#define SYS_GPA_MFPH_PA11MFP_PINV           (0x2UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for PINV             */
#define SYS_GPA_MFPH_PA11MFP_SPI2_SS        (0x4UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for SPI2_SS          */
#define SYS_GPA_MFPH_PA11MFP_USCI0_CLK      (0x6UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for USCI0_CLK        */
#define SYS_GPA_MFPH_PA11MFP_I2C2_SCL       (0x7UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for I2C2_SCL         */
#define SYS_GPA_MFPH_PA11MFP_LCD_POWER      (0x8UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for LCD_POWER        */
#define SYS_GPA_MFPH_PA11MFP_BPWM0_CH0      (0x9UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for BPWM0_CH0        */
#define SYS_GPA_MFPH_PA11MFP_PBUF           (0xAUL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for PBUF             */
#define SYS_GPA_MFPH_PA11MFP_TM0_EXT        (0xDUL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for TM0_EXT          */

/* PA.12 MFP */
#define SYS_GPA_MFPH_PA12MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for GPIO             */
#define SYS_GPA_MFPH_PA12MFP_PINV           (0x2UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for PINV             */
#define SYS_GPA_MFPH_PA12MFP_I2C1_SCL       (0x4UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for I2C1_SCL         */
#define SYS_GPA_MFPH_PA12MFP_SPI2_SS        (0x5UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for SPI2_SS          */
#define SYS_GPA_MFPH_PA12MFP_PBUF           (0xAUL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for PBUF             */

/* PA.13 MFP */
#define SYS_GPA_MFPH_PA13MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for GPIO             */
#define SYS_GPA_MFPH_PA13MFP_PINV           (0x2UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for PINV             */
#define SYS_GPA_MFPH_PA13MFP_I2C1_SDA       (0x4UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for I2C1_SDA         */
#define SYS_GPA_MFPH_PA13MFP_SPI2_CLK       (0x5UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for SPI2_CLK         */
#define SYS_GPA_MFPH_PA13MFP_PBUF           (0xAUL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for PBUF             */

/* PA.14 MFP */
#define SYS_GPA_MFPH_PA14MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for GPIO             */
#define SYS_GPA_MFPH_PA14MFP_PINV           (0x2UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for PINV             */
#define SYS_GPA_MFPH_PA14MFP_UART0_TXD      (0x3UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for UART0_TXD        */
#define SYS_GPA_MFPH_PA14MFP_I2C1_SMBAL     (0x4UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for I2C1_SMBAL       */
#define SYS_GPA_MFPH_PA14MFP_SPI2_MISO      (0x5UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for SPI2_MISO        */
#define SYS_GPA_MFPH_PA14MFP_I2C2_SCL       (0x6UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for I2C2_SCL         */
#define SYS_GPA_MFPH_PA14MFP_I2C0_SMBAL     (0x9UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for I2C0_SMBAL       */
#define SYS_GPA_MFPH_PA14MFP_PBUF           (0xAUL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for PBUF             */

/* PA.15 MFP */
#define SYS_GPA_MFPH_PA15MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for GPIO             */
#define SYS_GPA_MFPH_PA15MFP_PINV           (0x2UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for PINV             */
#define SYS_GPA_MFPH_PA15MFP_UART0_RXD      (0x3UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for UART0_RXD        */
#define SYS_GPA_MFPH_PA15MFP_I2C1_SMBSUS    (0x4UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for I2C1_SMBSUS      */
#define SYS_GPA_MFPH_PA15MFP_SPI2_MOSI      (0x5UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for SPI2_MOSI        */
#define SYS_GPA_MFPH_PA15MFP_I2C2_SDA       (0x6UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for I2C2_SDA         */
#define SYS_GPA_MFPH_PA15MFP_I2C0_SMBSUS    (0x9UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for I2C0_SMBSUS      */
#define SYS_GPA_MFPH_PA15MFP_PBUF           (0xAUL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for PBUF             */

/* PB.0 MFP */
#define SYS_GPB_MFPL_PB0MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for GPIO              */
#define SYS_GPB_MFPL_PB0MFP_ADC0_CH0        (0x1UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for ADC0_CH0          */
#define SYS_GPB_MFPL_PB0MFP_USCI0_CTL0      (0x6UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for USCI0_CTL0        */
#define SYS_GPB_MFPL_PB0MFP_UART2_RXD       (0x7UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for UART2_RXD         */
#define SYS_GPB_MFPL_PB0MFP_LCD_POWER       (0x8UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for LCD_POWER         */
#define SYS_GPB_MFPL_PB0MFP_I2C1_SDA        (0x9UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for I2C1_SDA          */
#define SYS_GPB_MFPL_PB0MFP_PWM0_CH5        (0xBUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for PWM0_CH5          */
#define SYS_GPB_MFPL_PB0MFP_PWM0_BRAKE1     (0xDUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for PWM0_BRAKE1       */

/* PB.1 MFP */
#define SYS_GPB_MFPL_PB1MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for GPIO              */
#define SYS_GPB_MFPL_PB1MFP_ADC0_CH1        (0x1UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for ADC0_CH1          */
#define SYS_GPB_MFPL_PB1MFP_PINV            (0x2UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for PINV              */
#define SYS_GPB_MFPL_PB1MFP_UART2_TXD       (0x7UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for UART2_TXD         */
#define SYS_GPB_MFPL_PB1MFP_LCD_POWER       (0x8UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for LCD_POWER         */
#define SYS_GPB_MFPL_PB1MFP_I2C1_SCL        (0x9UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for I2C1_SCL          */
#define SYS_GPB_MFPL_PB1MFP_PBUF            (0xAUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for PBUF              */
#define SYS_GPB_MFPL_PB1MFP_PWM0_CH4        (0xBUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for PWM0_CH4          */
#define SYS_GPB_MFPL_PB1MFP_PWM0_BRAKE0     (0xDUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for PWM0_BRAKE0       */

/* PB.2 MFP */
#define SYS_GPB_MFPL_PB2MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for GPIO              */
#define SYS_GPB_MFPL_PB2MFP_ADC0_CH2        (0x1UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for ADC0_CH2          */
#define SYS_GPB_MFPL_PB2MFP_ACMP0_P1        (0x1UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for ACMP0_P1          */
#define SYS_GPB_MFPL_PB2MFP_PINV            (0x2UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for PINV              */
#define SYS_GPB_MFPL_PB2MFP_SPI1_SS         (0x5UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for SPI1_SS           */
#define SYS_GPB_MFPL_PB2MFP_UART1_RXD       (0x6UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for UART1_RXD         */
#define SYS_GPB_MFPL_PB2MFP_LCD_PIN3        (0x8UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for LCD_PIN3          */
#define SYS_GPB_MFPL_PB2MFP_PBUF            (0xAUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for PBUF              */
#define SYS_GPB_MFPL_PB2MFP_PWM0_CH3        (0xBUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for PWM0_CH3          */
#define SYS_GPB_MFPL_PB2MFP_I2C1_SDA        (0xCUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for I2C1_SDA          */
#define SYS_GPB_MFPL_PB2MFP_TM3             (0xEUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for TM3               */
#define SYS_GPB_MFPL_PB2MFP_INT3            (0xFUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for INT3              */

/* PB.3 MFP */
#define SYS_GPB_MFPL_PB3MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for GPIO              */
#define SYS_GPB_MFPL_PB3MFP_ADC0_CH3        (0x1UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for ADC0_CH3          */
#define SYS_GPB_MFPL_PB3MFP_ACMP0_N         (0x1UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for ACMP0_N           */
#define SYS_GPB_MFPL_PB3MFP_PINV            (0x2UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for PINV              */
#define SYS_GPB_MFPL_PB3MFP_SPI1_CLK        (0x5UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for SPI1_CLK          */
#define SYS_GPB_MFPL_PB3MFP_UART1_TXD       (0x6UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for UART1_TXD         */
#define SYS_GPB_MFPL_PB3MFP_LCD_PIN2        (0x8UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for LCD_PIN2          */
#define SYS_GPB_MFPL_PB3MFP_PBUF            (0xAUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for PBUF              */
#define SYS_GPB_MFPL_PB3MFP_PWM0_CH2        (0xBUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for PWM0_CH2          */
#define SYS_GPB_MFPL_PB3MFP_I2C1_SCL        (0xCUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for I2C1_SCL          */
#define SYS_GPB_MFPL_PB3MFP_TM2             (0xEUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for TM2               */
#define SYS_GPB_MFPL_PB3MFP_INT2            (0xFUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for INT2              */

/* PB.4 MFP */
#define SYS_GPB_MFPL_PB4MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for GPIO              */
#define SYS_GPB_MFPL_PB4MFP_ADC0_CH4        (0x1UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for ADC0_CH4          */
#define SYS_GPB_MFPL_PB4MFP_ACMP1_P1        (0x1UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for ACMP1_P1          */
#define SYS_GPB_MFPL_PB4MFP_PINV            (0x2UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for PINV              */
#define SYS_GPB_MFPL_PB4MFP_SPI1_MOSI       (0x5UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for SPI1_MOSI         */
#define SYS_GPB_MFPL_PB4MFP_I2C0_SDA        (0x6UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for I2C0_SDA          */
#define SYS_GPB_MFPL_PB4MFP_LCD_PIN1        (0x8UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for LCD_PIN1          */
#define SYS_GPB_MFPL_PB4MFP_PBUF            (0xAUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for PBUF              */
#define SYS_GPB_MFPL_PB4MFP_PWM0_CH1        (0xBUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for PWM0_CH1          */
#define SYS_GPB_MFPL_PB4MFP_UART2_RXD       (0xCUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for UART2_RXD         */
#define SYS_GPB_MFPL_PB4MFP_TM1             (0xEUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for TM1               */
#define SYS_GPB_MFPL_PB4MFP_INT1            (0xFUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for INT1              */

/* PB.5 MFP */
#define SYS_GPB_MFPL_PB5MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for GPIO              */
#define SYS_GPB_MFPL_PB5MFP_ADC0_CH5        (0x1UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for ADC0_CH5          */
#define SYS_GPB_MFPL_PB5MFP_ACMP1_N         (0x1UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for ACMP1_N           */
#define SYS_GPB_MFPL_PB5MFP_PINV            (0x2UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for PINV              */
#define SYS_GPB_MFPL_PB5MFP_SPI1_MISO       (0x5UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for SPI1_MISO         */
#define SYS_GPB_MFPL_PB5MFP_I2C0_SCL        (0x6UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for I2C0_SCL          */
#define SYS_GPB_MFPL_PB5MFP_LCD_PIN0        (0x8UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for LCD_PIN0          */
#define SYS_GPB_MFPL_PB5MFP_PBUF            (0xAUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for PBUF              */
#define SYS_GPB_MFPL_PB5MFP_PWM0_CH0        (0xBUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for PWM0_CH0          */
#define SYS_GPB_MFPL_PB5MFP_UART2_TXD       (0xCUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for UART2_TXD         */
#define SYS_GPB_MFPL_PB5MFP_TM0             (0xEUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for TM0               */
#define SYS_GPB_MFPL_PB5MFP_INT0            (0xFUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for INT0              */

/* PB.6 MFP */
#define SYS_GPB_MFPL_PB6MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for GPIO              */
#define SYS_GPB_MFPL_PB6MFP_ADC0_CH6        (0x1UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for ADC0_CH6          */
#define SYS_GPB_MFPL_PB6MFP_PINV            (0x2UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for PINV              */
#define SYS_GPB_MFPL_PB6MFP_UART1_RXD       (0x6UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for UART1_RXD         */
#define SYS_GPB_MFPL_PB6MFP_LCD_PIN49       (0x8UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for LCD_PIN49         */
#define SYS_GPB_MFPL_PB6MFP_SPI0_CLK        (0x9UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for SPI0_CLK          */
#define SYS_GPB_MFPL_PB6MFP_PBUF            (0xAUL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for PBUF              */
#define SYS_GPB_MFPL_PB6MFP_INT4            (0xDUL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for INT4              */
#define SYS_GPB_MFPL_PB6MFP_ACMP1_O         (0xFUL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for ACMP1_O           */

/* PB.7 MFP */
#define SYS_GPB_MFPL_PB7MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for GPIO              */
#define SYS_GPB_MFPL_PB7MFP_ADC0_CH7        (0x1UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for ADC0_CH7          */
#define SYS_GPB_MFPL_PB7MFP_PINV            (0x2UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for PINV              */
#define SYS_GPB_MFPL_PB7MFP_UART1_TXD       (0x6UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for UART1_TXD         */
#define SYS_GPB_MFPL_PB7MFP_LCD_PIN48       (0x8UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for LCD_PIN48         */
#define SYS_GPB_MFPL_PB7MFP_SPI0_SS         (0x9UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for SPI0_SS           */
#define SYS_GPB_MFPL_PB7MFP_PBUF            (0xAUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for PBUF              */
#define SYS_GPB_MFPL_PB7MFP_INT5            (0xDUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for INT5              */
#define SYS_GPB_MFPL_PB7MFP_ACMP0_O         (0xFUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for ACMP0_O           */

/* PB.8 MFP */
#define SYS_GPB_MFPH_PB8MFP_GPIO            (0x0UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for GPIO              */
#define SYS_GPB_MFPH_PB8MFP_ADC0_CH8        (0x1UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for ADC0_CH8          */
#define SYS_GPB_MFPH_PB8MFP_PINV            (0x2UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for PINV              */
#define SYS_GPB_MFPH_PB8MFP_UART0_RXD       (0x5UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for UART0_RXD         */
#define SYS_GPB_MFPH_PB8MFP_UART1_nRTS      (0x6UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for UART1_nRTS        */
#define SYS_GPB_MFPH_PB8MFP_I2C1_SMBSUS     (0x7UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for I2C1_SMBSUS       */
#define SYS_GPB_MFPH_PB8MFP_LCD_PIN47       (0x8UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for LCD_PIN47         */
#define SYS_GPB_MFPH_PB8MFP_I2C0_SDA        (0x9UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for I2C0_SDA          */
#define SYS_GPB_MFPH_PB8MFP_PBUF            (0xAUL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for PBUF              */
#define SYS_GPB_MFPH_PB8MFP_INT6            (0xDUL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for INT6              */

/* PB.9 MFP */
#define SYS_GPB_MFPH_PB9MFP_GPIO            (0x0UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for GPIO              */
#define SYS_GPB_MFPH_PB9MFP_ADC0_CH9        (0x1UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for ADC0_CH9          */
#define SYS_GPB_MFPH_PB9MFP_PINV            (0x2UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for PINV              */
#define SYS_GPB_MFPH_PB9MFP_UART0_TXD       (0x5UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for UART0_TXD         */
#define SYS_GPB_MFPH_PB9MFP_UART1_nCTS      (0x6UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for UART1_nCTS        */
#define SYS_GPB_MFPH_PB9MFP_I2C1_SMBAL      (0x7UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for I2C1_SMBAL        */
#define SYS_GPB_MFPH_PB9MFP_LCD_PIN46       (0x8UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for LCD_PIN46         */
#define SYS_GPB_MFPH_PB9MFP_I2C0_SCL        (0x9UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for I2C0_SCL          */
#define SYS_GPB_MFPH_PB9MFP_PBUF            (0xAUL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for PBUF              */
#define SYS_GPB_MFPH_PB9MFP_INT7            (0xDUL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for INT7              */

/* PB.10 MFP */
#define SYS_GPB_MFPH_PB10MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for GPIO             */
#define SYS_GPB_MFPH_PB10MFP_ADC0_CH10      (0x1UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for ADC0_CH10        */
#define SYS_GPB_MFPH_PB10MFP_PINV           (0x2UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for PINV             */
#define SYS_GPB_MFPH_PB10MFP_UART0_nRTS     (0x5UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for UART0_nRTS       */
#define SYS_GPB_MFPH_PB10MFP_I2C1_SDA       (0x7UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for I2C1_SDA         */
#define SYS_GPB_MFPH_PB10MFP_LCD_PIN45      (0x8UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for LCD_PIN45        */
#define SYS_GPB_MFPH_PB10MFP_PBUF           (0xAUL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for PBUF             */

/* PB.11 MFP */
#define SYS_GPB_MFPH_PB11MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for GPIO             */
#define SYS_GPB_MFPH_PB11MFP_ADC0_CH11      (0x1UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for ADC0_CH11        */
#define SYS_GPB_MFPH_PB11MFP_PINV           (0x2UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for PINV             */
#define SYS_GPB_MFPH_PB11MFP_UART0_nCTS     (0x5UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for UART0_nCTS       */
#define SYS_GPB_MFPH_PB11MFP_I2C1_SCL       (0x7UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for I2C1_SCL         */
#define SYS_GPB_MFPH_PB11MFP_LCD_PIN44      (0x8UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for LCD_PIN44        */
#define SYS_GPB_MFPH_PB11MFP_PBUF           (0xAUL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for PBUF             */

/* PB.12 MFP */
#define SYS_GPB_MFPH_PB12MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for GPIO             */
#define SYS_GPB_MFPH_PB12MFP_ADC0_CH12      (0x1UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for ADC0_CH12        */
#define SYS_GPB_MFPH_PB12MFP_ACMP0_P2       (0x1UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for ACMP0_P2         */
#define SYS_GPB_MFPH_PB12MFP_ACMP1_P2       (0x1UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for ACMP1_P2         */
#define SYS_GPB_MFPH_PB12MFP_PINV           (0x2UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for PINV             */
#define SYS_GPB_MFPH_PB12MFP_SPI0_MOSI      (0x4UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for SPI0_MOSI        */
#define SYS_GPB_MFPH_PB12MFP_USCI0_CLK      (0x5UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for USCI0_CLK        */
#define SYS_GPB_MFPH_PB12MFP_UART0_RXD      (0x6UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for UART0_RXD        */
#define SYS_GPB_MFPH_PB12MFP_I2C2_SDA       (0x7UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for I2C2_SDA         */
#define SYS_GPB_MFPH_PB12MFP_LCD_PIN43      (0x8UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for LCD_PIN43        */
#define SYS_GPB_MFPH_PB12MFP_PBUF           (0xAUL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for PBUF             */
#define SYS_GPB_MFPH_PB12MFP_TM3_EXT        (0xDUL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for TM3_EXT          */

/* PB.13 MFP */
#define SYS_GPB_MFPH_PB13MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for GPIO             */
#define SYS_GPB_MFPH_PB13MFP_ADC0_CH13      (0x1UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for ADC0_CH13        */
#define SYS_GPB_MFPH_PB13MFP_ACMP0_P3       (0x1UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for ACMP0_P3         */
#define SYS_GPB_MFPH_PB13MFP_ACMP1_P3       (0x1UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for ACMP1_P3         */
#define SYS_GPB_MFPH_PB13MFP_PINV           (0x2UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for PINV             */
#define SYS_GPB_MFPH_PB13MFP_SPI0_MISO      (0x4UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for SPI0_MISO        */
#define SYS_GPB_MFPH_PB13MFP_USCI0_DAT0     (0x5UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for USCI0_DAT0       */
#define SYS_GPB_MFPH_PB13MFP_UART0_TXD      (0x6UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for UART0_TXD        */
#define SYS_GPB_MFPH_PB13MFP_I2C2_SCL       (0x7UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for I2C2_SCL         */
#define SYS_GPB_MFPH_PB13MFP_LCD_PIN42      (0x8UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for LCD_PIN42        */
#define SYS_GPB_MFPH_PB13MFP_CLKO           (0x9UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for CLKO             */
#define SYS_GPB_MFPH_PB13MFP_PBUF           (0xAUL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for PBUF             */
#define SYS_GPB_MFPH_PB13MFP_TM2_EXT        (0xDUL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for TM2_EXT          */

/* PB.14 MFP */
#define SYS_GPB_MFPH_PB14MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for GPIO             */
#define SYS_GPB_MFPH_PB14MFP_ADC0_CH14      (0x1UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for ADC0_CH14        */
#define SYS_GPB_MFPH_PB14MFP_PINV           (0x2UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for PINV             */
#define SYS_GPB_MFPH_PB14MFP_SPI0_CLK       (0x4UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for SPI0_CLK         */
#define SYS_GPB_MFPH_PB14MFP_USCI0_DAT1     (0x5UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for USCI0_DAT1       */
#define SYS_GPB_MFPH_PB14MFP_UART0_nRTS     (0x6UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for UART0_nRTS       */
#define SYS_GPB_MFPH_PB14MFP_I2C2_SMBSUS    (0x7UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for I2C2_SMBSUS      */
#define SYS_GPB_MFPH_PB14MFP_LCD_PIN41      (0x8UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for LCD_PIN41        */
#define SYS_GPB_MFPH_PB14MFP_PBUF           (0xAUL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for PBUF             */
#define SYS_GPB_MFPH_PB14MFP_TM1_EXT        (0xDUL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for TM1_EXT          */
#define SYS_GPB_MFPH_PB14MFP_CLKO           (0xEUL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for CLKO             */

/* PB.15 MFP */
#define SYS_GPB_MFPH_PB15MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for GPIO             */
#define SYS_GPB_MFPH_PB15MFP_ADC0_CH15      (0x1UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for ADC0_CH15        */
#define SYS_GPB_MFPH_PB15MFP_PINV           (0x2UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for PINV             */
#define SYS_GPB_MFPH_PB15MFP_SPI0_SS        (0x4UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for SPI0_SS          */
#define SYS_GPB_MFPH_PB15MFP_USCI0_CTL1     (0x5UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for USCI0_CTL1       */
#define SYS_GPB_MFPH_PB15MFP_UART0_nCTS     (0x6UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for UART0_nCTS       */
#define SYS_GPB_MFPH_PB15MFP_I2C2_SMBAL     (0x7UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for I2C2_SMBAL       */
#define SYS_GPB_MFPH_PB15MFP_LCD_PIN40      (0x8UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for LCD_PIN40        */
#define SYS_GPB_MFPH_PB15MFP_PBUF           (0xAUL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for PBUF             */
#define SYS_GPB_MFPH_PB15MFP_TM0_EXT        (0xDUL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for TM0_EXT          */

/* PC.0 MFP */
#define SYS_GPC_MFPL_PC0MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for GPIO              */
#define SYS_GPC_MFPL_PC0MFP_UART2_RXD       (0x6UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for UART2_RXD         */
#define SYS_GPC_MFPL_PC0MFP_SPI1_SS         (0x7UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for SPI1_SS           */
#define SYS_GPC_MFPL_PC0MFP_LCD_PIN29       (0x8UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for LCD_PIN29         */
#define SYS_GPC_MFPL_PC0MFP_I2C0_SDA        (0x9UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for I2C0_SDA          */
#define SYS_GPC_MFPL_PC0MFP_ACMP1_O         (0xEUL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for ACMP1_O           */

/* PC.1 MFP */
#define SYS_GPC_MFPL_PC1MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for GPIO              */
#define SYS_GPC_MFPL_PC1MFP_PINV            (0x2UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for PINV              */
#define SYS_GPC_MFPL_PC1MFP_UART2_TXD       (0x6UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for UART2_TXD         */
#define SYS_GPC_MFPL_PC1MFP_SPI1_CLK        (0x7UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for SPI1_CLK          */
#define SYS_GPC_MFPL_PC1MFP_LCD_PIN28       (0x8UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for LCD_PIN28         */
#define SYS_GPC_MFPL_PC1MFP_I2C0_SCL        (0x9UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for I2C0_SCL          */
#define SYS_GPC_MFPL_PC1MFP_PBUF            (0xAUL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for PBUF              */
#define SYS_GPC_MFPL_PC1MFP_ACMP0_O         (0xEUL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for ACMP0_O           */
#define SYS_GPC_MFPL_PC1MFP_ADC0_ST         (0xFUL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for ADC0_ST           */

/* PC.2 MFP */
#define SYS_GPC_MFPL_PC2MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for GPIO              */
#define SYS_GPC_MFPL_PC2MFP_PINV            (0x2UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for PINV              */
#define SYS_GPC_MFPL_PC2MFP_UART2_nCTS      (0x6UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for UART2_nCTS        */
#define SYS_GPC_MFPL_PC2MFP_SPI1_MOSI       (0x7UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for SPI1_MOSI         */
#define SYS_GPC_MFPL_PC2MFP_LCD_PIN27       (0x8UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for LCD_PIN27         */
#define SYS_GPC_MFPL_PC2MFP_I2C0_SMBSUS     (0x9UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for I2C0_SMBSUS       */
#define SYS_GPC_MFPL_PC2MFP_PBUF            (0xAUL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for PBUF              */

/* PC.3 MFP */
#define SYS_GPC_MFPL_PC3MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for GPIO              */
#define SYS_GPC_MFPL_PC3MFP_PINV            (0x2UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for PINV              */
#define SYS_GPC_MFPL_PC3MFP_UART2_nRTS      (0x6UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for UART2_nRTS        */
#define SYS_GPC_MFPL_PC3MFP_SPI1_MISO       (0x7UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for SPI1_MISO         */
#define SYS_GPC_MFPL_PC3MFP_LCD_PIN26       (0x8UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for LCD_PIN26         */
#define SYS_GPC_MFPL_PC3MFP_I2C0_SMBAL      (0x9UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for I2C0_SMBAL        */
#define SYS_GPC_MFPL_PC3MFP_PBUF            (0xAUL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for PBUF              */

/* PC.4 MFP */
#define SYS_GPC_MFPL_PC4MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for GPIO              */
#define SYS_GPC_MFPL_PC4MFP_PINV            (0x2UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for PINV              */
#define SYS_GPC_MFPL_PC4MFP_UART2_RXD       (0x6UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for UART2_RXD         */
#define SYS_GPC_MFPL_PC4MFP_LCD_PIN25       (0x8UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for LCD_PIN25         */
#define SYS_GPC_MFPL_PC4MFP_I2C1_SDA        (0x9UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for I2C1_SDA          */
#define SYS_GPC_MFPL_PC4MFP_PBUF            (0xAUL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for PBUF              */
#define SYS_GPC_MFPL_PC4MFP_ACMP1_WLAT      (0xDUL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for ACMP1_WLAT        */

/* PC.5 MFP */
#define SYS_GPC_MFPL_PC5MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for GPIO              */
#define SYS_GPC_MFPL_PC5MFP_PINV            (0x2UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for PINV              */
#define SYS_GPC_MFPL_PC5MFP_UART2_TXD       (0x6UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for UART2_TXD         */
#define SYS_GPC_MFPL_PC5MFP_LCD_PIN24       (0x8UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for LCD_PIN24         */
#define SYS_GPC_MFPL_PC5MFP_I2C1_SCL        (0x9UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for I2C1_SCL          */
#define SYS_GPC_MFPL_PC5MFP_PBUF            (0xAUL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for PBUF              */
#define SYS_GPC_MFPL_PC5MFP_ACMP0_WLAT      (0xDUL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for ACMP0_WLAT        */

/* PC.6 MFP */
#define SYS_GPC_MFPL_PC6MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for GPIO              */
#define SYS_GPC_MFPL_PC6MFP_PINV            (0x2UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for PINV              */
#define SYS_GPC_MFPL_PC6MFP_SPI1_MOSI       (0x4UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for SPI1_MOSI         */
#define SYS_GPC_MFPL_PC6MFP_UART0_nRTS      (0x7UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for UART0_nRTS        */
#define SYS_GPC_MFPL_PC6MFP_LCD_PIN18       (0x8UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for LCD_PIN18         */
#define SYS_GPC_MFPL_PC6MFP_I2C1_SMBSUS     (0x9UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for I2C1_SMBSUS       */
#define SYS_GPC_MFPL_PC6MFP_PBUF            (0xAUL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for PBUF              */
#define SYS_GPC_MFPL_PC6MFP_TM1             (0xEUL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for TM1               */
#define SYS_GPC_MFPL_PC6MFP_INT2            (0xFUL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for INT2              */

/* PC.7 MFP */
#define SYS_GPC_MFPL_PC7MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for GPIO              */
#define SYS_GPC_MFPL_PC7MFP_PINV            (0x2UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for PINV              */
#define SYS_GPC_MFPL_PC7MFP_SPI1_MISO       (0x4UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for SPI1_MISO         */
#define SYS_GPC_MFPL_PC7MFP_UART0_nCTS      (0x7UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for UART0_nCTS        */
#define SYS_GPC_MFPL_PC7MFP_LCD_PIN17       (0x8UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for LCD_PIN17         */
#define SYS_GPC_MFPL_PC7MFP_I2C1_SMBAL      (0x9UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for I2C1_SMBAL        */
#define SYS_GPC_MFPL_PC7MFP_PBUF            (0xAUL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for PBUF              */
#define SYS_GPC_MFPL_PC7MFP_TM0             (0xEUL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for TM0               */
#define SYS_GPC_MFPL_PC7MFP_INT3            (0xFUL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for INT3              */

/* PC.8 MFP */
#define SYS_GPC_MFPH_PC8MFP_GPIO            (0x0UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for GPIO              */
#define SYS_GPC_MFPH_PC8MFP_PINV            (0x2UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for PINV              */
#define SYS_GPC_MFPH_PC8MFP_I2C0_SDA        (0x4UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for I2C0_SDA          */
#define SYS_GPC_MFPH_PC8MFP_LCD_PIN16       (0x8UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for LCD_PIN16         */
#define SYS_GPC_MFPH_PC8MFP_UART1_RXD       (0x9UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for UART1_RXD         */
#define SYS_GPC_MFPH_PC8MFP_PBUF            (0xAUL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for PBUF              */

/* PC.9 MFP */
#define SYS_GPC_MFPH_PC9MFP_GPIO            (0x0UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for GPIO              */
#define SYS_GPC_MFPH_PC9MFP_PINV            (0x2UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for PINV              */
#define SYS_GPC_MFPH_PC9MFP_LCD_PIN7        (0x8UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for LCD_PIN7          */
#define SYS_GPC_MFPH_PC9MFP_PBUF            (0xAUL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for PBUF              */

/* PC.10 MFP */
#define SYS_GPC_MFPH_PC10MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for GPIO             */
#define SYS_GPC_MFPH_PC10MFP_PINV           (0x2UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for PINV             */
#define SYS_GPC_MFPH_PC10MFP_LCD_PIN6       (0x8UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for LCD_PIN6         */
#define SYS_GPC_MFPH_PC10MFP_PBUF           (0xAUL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for PBUF             */

/* PC.11 MFP */
#define SYS_GPC_MFPH_PC11MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for GPIO             */
#define SYS_GPC_MFPH_PC11MFP_PINV           (0x2UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for PINV             */
#define SYS_GPC_MFPH_PC11MFP_UART0_RXD      (0x3UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for UART0_RXD        */
#define SYS_GPC_MFPH_PC11MFP_I2C0_SDA       (0x4UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for I2C0_SDA         */
#define SYS_GPC_MFPH_PC11MFP_LCD_PIN5       (0x8UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for LCD_PIN5         */
#define SYS_GPC_MFPH_PC11MFP_PBUF           (0xAUL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for PBUF             */
#define SYS_GPC_MFPH_PC11MFP_ACMP1_O        (0xEUL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for ACMP1_O          */

/* PC.12 MFP */
#define SYS_GPC_MFPH_PC12MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for GPIO             */
#define SYS_GPC_MFPH_PC12MFP_PINV           (0x2UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for PINV             */
#define SYS_GPC_MFPH_PC12MFP_UART0_TXD      (0x3UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for UART0_TXD        */
#define SYS_GPC_MFPH_PC12MFP_I2C0_SCL       (0x4UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for I2C0_SCL         */
#define SYS_GPC_MFPH_PC12MFP_LCD_PIN4       (0x8UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for LCD_PIN4         */
#define SYS_GPC_MFPH_PC12MFP_PBUF           (0xAUL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for PBUF             */
#define SYS_GPC_MFPH_PC12MFP_ACMP0_O        (0xEUL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for ACMP0_O          */

/* PC.14 MFP */
#define SYS_GPC_MFPH_PC14MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for GPIO             */
#define SYS_GPC_MFPH_PC14MFP_PINV           (0x2UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for PINV             */
#define SYS_GPC_MFPH_PC14MFP_USCI0_CTL0     (0x5UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for USCI0_CTL0       */
#define SYS_GPC_MFPH_PC14MFP_LCD_PIN39      (0x8UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for LCD_PIN39        */
#define SYS_GPC_MFPH_PC14MFP_PBUF           (0xAUL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for PBUF             */
#define SYS_GPC_MFPH_PC14MFP_TM1            (0xDUL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for TM1              */

/* PD.0 MFP */
#define SYS_GPD_MFPL_PD0MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for GPIO              */
#define SYS_GPD_MFPL_PD0MFP_USCI0_CLK       (0x3UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for USCI0_CLK         */
#define SYS_GPD_MFPL_PD0MFP_SPI0_MOSI       (0x4UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for SPI0_MOSI         */
#define SYS_GPD_MFPL_PD0MFP_I2C2_SDA        (0x6UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for I2C2_SDA          */
#define SYS_GPD_MFPL_PD0MFP_LCD_PIN33       (0x8UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for LCD_PIN33         */
#define SYS_GPD_MFPL_PD0MFP_TM2             (0xEUL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for TM2               */

/* PD.1 MFP */
#define SYS_GPD_MFPL_PD1MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for GPIO              */
#define SYS_GPD_MFPL_PD1MFP_USCI0_DAT0      (0x3UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for USCI0_DAT0        */
#define SYS_GPD_MFPL_PD1MFP_SPI0_MISO       (0x4UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for SPI0_MISO         */
#define SYS_GPD_MFPL_PD1MFP_I2C2_SCL        (0x6UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for I2C2_SCL          */
#define SYS_GPD_MFPL_PD1MFP_LCD_PIN32       (0x8UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for LCD_PIN32         */

/* PD.2 MFP */
#define SYS_GPD_MFPL_PD2MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for GPIO              */
#define SYS_GPD_MFPL_PD2MFP_USCI0_DAT1      (0x3UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for USCI0_DAT1        */
#define SYS_GPD_MFPL_PD2MFP_SPI0_CLK        (0x4UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for SPI0_CLK          */
#define SYS_GPD_MFPL_PD2MFP_LCD_PIN31       (0x8UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for LCD_PIN31         */
#define SYS_GPD_MFPL_PD2MFP_UART0_RXD       (0x9UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for UART0_RXD         */

/* PD.3 MFP */
#define SYS_GPD_MFPL_PD3MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for GPIO              */
#define SYS_GPD_MFPL_PD3MFP_USCI0_CTL1      (0x3UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for USCI0_CTL1        */
#define SYS_GPD_MFPL_PD3MFP_SPI0_SS         (0x4UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for SPI0_SS           */
#define SYS_GPD_MFPL_PD3MFP_LCD_PIN30       (0x8UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for LCD_PIN30         */
#define SYS_GPD_MFPL_PD3MFP_UART0_TXD       (0x9UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for UART0_TXD         */

/* PD.4 MFP */
#define SYS_GPD_MFPL_PD4MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for GPIO              */
#define SYS_GPD_MFPL_PD4MFP_USCI0_CTL0      (0x3UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for USCI0_CTL0        */
#define SYS_GPD_MFPL_PD4MFP_I2C1_SDA        (0x4UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for I2C1_SDA          */
#define SYS_GPD_MFPL_PD4MFP_SPI1_SS         (0x5UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for SPI1_SS           */

/* PD.5 MFP */
#define SYS_GPD_MFPL_PD5MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for GPIO              */
#define SYS_GPD_MFPL_PD5MFP_I2C1_SCL        (0x4UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for I2C1_SCL          */
#define SYS_GPD_MFPL_PD5MFP_SPI1_CLK        (0x5UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for SPI1_CLK          */

/* PD.6 MFP */
#define SYS_GPD_MFPL_PD6MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for GPIO              */
#define SYS_GPD_MFPL_PD6MFP_UART1_RXD       (0x3UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for UART1_RXD         */
#define SYS_GPD_MFPL_PD6MFP_I2C0_SDA        (0x4UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for I2C0_SDA          */
#define SYS_GPD_MFPL_PD6MFP_SPI1_MOSI       (0x5UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for SPI1_MOSI         */

/* PD.7 MFP */
#define SYS_GPD_MFPL_PD7MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for GPIO              */
#define SYS_GPD_MFPL_PD7MFP_UART1_TXD       (0x3UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for UART1_TXD         */
#define SYS_GPD_MFPL_PD7MFP_I2C0_SCL        (0x4UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for I2C0_SCL          */
#define SYS_GPD_MFPL_PD7MFP_SPI1_MISO       (0x5UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for SPI1_MISO         */

/* PD.8 MFP */
#define SYS_GPD_MFPH_PD8MFP_GPIO            (0x0UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for GPIO              */
#define SYS_GPD_MFPH_PD8MFP_I2C2_SDA        (0x3UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for I2C2_SDA          */
#define SYS_GPD_MFPH_PD8MFP_UART2_nRTS      (0x4UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for UART2_nRTS        */
#define SYS_GPD_MFPH_PD8MFP_LCD_PIN23       (0x8UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for LCD_PIN23         */

/* PD.9 MFP */
#define SYS_GPD_MFPH_PD9MFP_GPIO            (0x0UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for GPIO              */
#define SYS_GPD_MFPH_PD9MFP_I2C2_SCL        (0x3UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for I2C2_SCL          */
#define SYS_GPD_MFPH_PD9MFP_UART2_nCTS      (0x4UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for UART2_nCTS        */
#define SYS_GPD_MFPH_PD9MFP_LCD_PIN22       (0x8UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for LCD_PIN22         */

/* PD.10 MFP */
#define SYS_GPD_MFPH_PD10MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for GPIO             */
#define SYS_GPD_MFPH_PD10MFP_UART1_RXD      (0x3UL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for UART1_RXD        */
#define SYS_GPD_MFPH_PD10MFP_LCD_PIN10      (0x8UL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for LCD_PIN10        */
#define SYS_GPD_MFPH_PD10MFP_INT7           (0xFUL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for INT7             */

/* PD.11 MFP */
#define SYS_GPD_MFPH_PD11MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for GPIO             */
#define SYS_GPD_MFPH_PD11MFP_UART1_TXD      (0x3UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for UART1_TXD        */
#define SYS_GPD_MFPH_PD11MFP_LCD_PIN9       (0x8UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for LCD_PIN9         */
#define SYS_GPD_MFPH_PD11MFP_INT6           (0xFUL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for INT6             */

/* PD.12 MFP */
#define SYS_GPD_MFPH_PD12MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for GPIO             */
#define SYS_GPD_MFPH_PD12MFP_UART2_RXD      (0x7UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for UART2_RXD        */
#define SYS_GPD_MFPH_PD12MFP_BPWM0_CH5      (0x9UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for BPWM0_CH5        */
#define SYS_GPD_MFPH_PD12MFP_CLKO           (0xDUL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for CLKO             */
#define SYS_GPD_MFPH_PD12MFP_ADC0_ST        (0xEUL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for ADC0_ST          */
#define SYS_GPD_MFPH_PD12MFP_INT5           (0xFUL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for INT5             */

/* PD.13 MFP */
#define SYS_GPD_MFPH_PD13MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for GPIO             */
#define SYS_GPD_MFPH_PD13MFP_LCD_PIN34      (0x8UL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for LCD_PIN34        */
#define SYS_GPD_MFPH_PD13MFP_BPWM0_CH0      (0xBUL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for BPWM0_CH0        */
#define SYS_GPD_MFPH_PD13MFP_CLKO           (0xEUL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for CLKO             */

/* PD.15 MFP */
#define SYS_GPD_MFPH_PD15MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for GPIO             */

/* PE.0 MFP */
#define SYS_GPE_MFPL_PE0MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for GPIO              */
#define SYS_GPE_MFPL_PE0MFP_SPI1_MOSI       (0x6UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for SPI1_MOSI         */
#define SYS_GPE_MFPL_PE0MFP_I2C1_SDA        (0x7UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for I2C1_SDA          */

/* PE.1 MFP */
#define SYS_GPE_MFPL_PE1MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for GPIO              */
#define SYS_GPE_MFPL_PE1MFP_SPI1_MISO       (0x6UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for SPI1_MISO         */
#define SYS_GPE_MFPL_PE1MFP_I2C1_SCL        (0x7UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for I2C1_SCL          */

/* PE.2 MFP */
#define SYS_GPE_MFPL_PE2MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for GPIO              */
#define SYS_GPE_MFPL_PE2MFP_USCI0_CLK       (0x7UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for USCI0_CLK         */
#define SYS_GPE_MFPL_PE2MFP_BPWM0_CH0       (0xDUL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for BPWM0_CH0         */

/* PE.3 MFP */
#define SYS_GPE_MFPL_PE3MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for GPIO              */
#define SYS_GPE_MFPL_PE3MFP_USCI0_DAT0      (0x7UL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for USCI0_DAT0        */
#define SYS_GPE_MFPL_PE3MFP_BPWM0_CH1       (0xDUL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for BPWM0_CH1         */

/* PE.4 MFP */
#define SYS_GPE_MFPL_PE4MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for GPIO              */
#define SYS_GPE_MFPL_PE4MFP_USCI0_DAT1      (0x7UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for USCI0_DAT1        */
#define SYS_GPE_MFPL_PE4MFP_BPWM0_CH2       (0xDUL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for BPWM0_CH2         */
#define SYS_GPE_MFPL_PE4MFP_SPI1_MOSI       (0xFUL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for SPI1_MOSI         */

/* PE.5 MFP */
#define SYS_GPE_MFPL_PE5MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for GPIO              */
#define SYS_GPE_MFPL_PE5MFP_UART0_RXD       (0x3UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for UART0_RXD         */
#define SYS_GPE_MFPL_PE5MFP_I2C1_SMBSUS     (0x4UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for I2C1_SMBSUS       */
#define SYS_GPE_MFPL_PE5MFP_SPI2_MOSI       (0x5UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for SPI2_MOSI         */
#define SYS_GPE_MFPL_PE5MFP_I2C2_SDA        (0x6UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for I2C2_SDA          */
#define SYS_GPE_MFPL_PE5MFP_USCI0_CTL1      (0x7UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for USCI0_CTL1        */
#define SYS_GPE_MFPL_PE5MFP_I2C0_SMBSUS     (0x9UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for I2C0_SMBSUS       */
#define SYS_GPE_MFPL_PE5MFP_BPWM0_CH3       (0xDUL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for BPWM0_CH3         */
#define SYS_GPE_MFPL_PE5MFP_SPI1_MISO       (0xFUL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for SPI1_MISO         */

/* PE.6 MFP */
#define SYS_GPE_MFPL_PE6MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for GPIO              */
#define SYS_GPE_MFPL_PE6MFP_UART0_TXD       (0x3UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for UART0_TXD         */
#define SYS_GPE_MFPL_PE6MFP_I2C1_SMBAL      (0x4UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for I2C1_SMBAL        */
#define SYS_GPE_MFPL_PE6MFP_SPI2_MISO       (0x5UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for SPI2_MISO         */
#define SYS_GPE_MFPL_PE6MFP_I2C2_SCL        (0x6UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for I2C2_SCL          */
#define SYS_GPE_MFPL_PE6MFP_USCI0_CTL0      (0x7UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for USCI0_CTL0        */
#define SYS_GPE_MFPL_PE6MFP_LCD_PIN36       (0x8UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for LCD_PIN36         */
#define SYS_GPE_MFPL_PE6MFP_I2C0_SMBAL      (0x9UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for I2C0_SMBAL        */
#define SYS_GPE_MFPL_PE6MFP_BPWM0_CH4       (0xDUL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for BPWM0_CH4         */

/* PE.7 MFP */
#define SYS_GPE_MFPL_PE7MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for GPIO              */
#define SYS_GPE_MFPL_PE7MFP_I2C1_SDA        (0x4UL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for I2C1_SDA          */
#define SYS_GPE_MFPL_PE7MFP_SPI2_CLK        (0x5UL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for SPI2_CLK          */
#define SYS_GPE_MFPL_PE7MFP_LCD_PIN35       (0x8UL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for LCD_PIN35         */
#define SYS_GPE_MFPL_PE7MFP_BPWM0_CH5       (0xDUL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for BPWM0_CH5         */

/* PE.8 MFP */
#define SYS_GPE_MFPH_PE8MFP_GPIO            (0x0UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for GPIO              */
#define SYS_GPE_MFPH_PE8MFP_SPI2_CLK        (0x5UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for SPI2_CLK          */
#define SYS_GPE_MFPH_PE8MFP_UART2_TXD       (0x7UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for UART2_TXD         */
#define SYS_GPE_MFPH_PE8MFP_LCD_PIN11       (0x8UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for LCD_PIN11         */
#define SYS_GPE_MFPH_PE8MFP_PWM0_BRAKE0     (0xBUL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for PWM0_BRAKE0       */

/* PE.9 MFP */
#define SYS_GPE_MFPH_PE9MFP_GPIO            (0x0UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for GPIO              */
#define SYS_GPE_MFPH_PE9MFP_SPI2_MISO       (0x5UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for SPI2_MISO         */
#define SYS_GPE_MFPH_PE9MFP_UART2_RXD       (0x7UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for UART2_RXD         */
#define SYS_GPE_MFPH_PE9MFP_LCD_PIN12       (0x8UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for LCD_PIN12         */
#define SYS_GPE_MFPH_PE9MFP_PWM0_BRAKE1     (0xBUL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for PWM0_BRAKE1       */

/* PE.10 MFP */
#define SYS_GPE_MFPH_PE10MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for GPIO             */
#define SYS_GPE_MFPH_PE10MFP_SPI2_MOSI      (0x5UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for SPI2_MOSI        */
#define SYS_GPE_MFPH_PE10MFP_LCD_PIN13      (0x8UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for LCD_PIN13        */

/* PE.11 MFP */
#define SYS_GPE_MFPH_PE11MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for GPIO             */
#define SYS_GPE_MFPH_PE11MFP_SPI2_SS        (0x5UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for SPI2_SS          */
#define SYS_GPE_MFPH_PE11MFP_LCD_PIN14      (0x8UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for LCD_PIN14        */
#define SYS_GPE_MFPH_PE11MFP_UART1_nCTS     (0x9UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for UART1_nCTS       */

/* PE.12 MFP */
#define SYS_GPE_MFPH_PE12MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for GPIO             */
#define SYS_GPE_MFPH_PE12MFP_UART1_nRTS     (0x9UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for UART1_nRTS       */

/* PE.13 MFP */
#define SYS_GPE_MFPH_PE13MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for GPIO             */
#define SYS_GPE_MFPH_PE13MFP_I2C0_SCL       (0x4UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for I2C0_SCL         */
#define SYS_GPE_MFPH_PE13MFP_LCD_PIN15      (0x8UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for LCD_PIN15        */
#define SYS_GPE_MFPH_PE13MFP_UART1_TXD      (0x9UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for UART1_TXD        */

/* PE.14 MFP */
#define SYS_GPE_MFPH_PE14MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE14MFP_Pos)   /*!< GPE_MFPH PE14 setting for GPIO             */
#define SYS_GPE_MFPH_PE14MFP_UART2_TXD      (0x3UL<<SYS_GPE_MFPH_PE14MFP_Pos)   /*!< GPE_MFPH PE14 setting for UART2_TXD        */
#define SYS_GPE_MFPH_PE14MFP_LCD_PIN21      (0x8UL<<SYS_GPE_MFPH_PE14MFP_Pos)   /*!< GPE_MFPH PE14 setting for LCD_PIN21        */
#define SYS_GPE_MFPH_PE14MFP_I2C1_SCL       (0x9UL<<SYS_GPE_MFPH_PE14MFP_Pos)   /*!< GPE_MFPH PE14 setting for I2C1_SCL         */

/* PE.15 MFP */
#define SYS_GPE_MFPH_PE15MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for GPIO             */
#define SYS_GPE_MFPH_PE15MFP_UART2_RXD      (0x3UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for UART2_RXD        */
#define SYS_GPE_MFPH_PE15MFP_I2C1_SDA       (0x9UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for I2C1_SDA         */

/* PF.0 MFP */
#define SYS_GPF_MFPL_PF0MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for GPIO              */
#define SYS_GPF_MFPL_PF0MFP_UART1_TXD       (0x2UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for UART1_TXD         */
#define SYS_GPF_MFPL_PF0MFP_I2C1_SCL        (0x3UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for I2C1_SCL          */
#define SYS_GPF_MFPL_PF0MFP_UART0_TXD       (0x4UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for UART0_TXD         */
#define SYS_GPF_MFPL_PF0MFP_USCI0_CTL1      (0x7UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for USCI0_CTL1        */
#define SYS_GPF_MFPL_PF0MFP_UART2_TXD       (0x8UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for UART2_TXD         */
#define SYS_GPF_MFPL_PF0MFP_I2C0_SCL        (0x9UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for I2C0_SCL          */
#define SYS_GPF_MFPL_PF0MFP_ACMP0_O         (0xDUL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for ACMP0_O           */
#define SYS_GPF_MFPL_PF0MFP_ICE_DAT         (0xEUL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for ICE_DAT           */

/* PF.1 MFP */
#define SYS_GPF_MFPL_PF1MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for GPIO              */
#define SYS_GPF_MFPL_PF1MFP_UART1_RXD       (0x2UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for UART1_RXD         */
#define SYS_GPF_MFPL_PF1MFP_I2C1_SDA        (0x3UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for I2C1_SDA          */
#define SYS_GPF_MFPL_PF1MFP_UART0_RXD       (0x4UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for UART0_RXD         */
#define SYS_GPF_MFPL_PF1MFP_USCI0_DAT1      (0x7UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for USCI0_DAT1        */
#define SYS_GPF_MFPL_PF1MFP_UART2_RXD       (0x8UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for UART2_RXD         */
#define SYS_GPF_MFPL_PF1MFP_I2C0_SDA        (0x9UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for I2C0_SDA          */
#define SYS_GPF_MFPL_PF1MFP_ACMP1_O         (0xDUL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for ACMP1_O           */
#define SYS_GPF_MFPL_PF1MFP_ICE_CLK         (0xEUL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for ICE_CLK           */

/* PF.2 MFP */
#define SYS_GPF_MFPL_PF2MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for GPIO              */
#define SYS_GPF_MFPL_PF2MFP_UART0_RXD       (0x3UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for UART0_RXD         */
#define SYS_GPF_MFPL_PF2MFP_I2C0_SDA        (0x4UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for I2C0_SDA          */
#define SYS_GPF_MFPL_PF2MFP_I2C2_SMBSUS     (0x8UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for I2C2_SMBSUS       */
#define SYS_GPF_MFPL_PF2MFP_TM1_EXT         (0xFUL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for TM1_EXT           */

/* PF.3 MFP */
#define SYS_GPF_MFPL_PF3MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for GPIO              */
#define SYS_GPF_MFPL_PF3MFP_UART0_TXD       (0x3UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for UART0_TXD         */
#define SYS_GPF_MFPL_PF3MFP_I2C0_SCL        (0x4UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for I2C0_SCL          */
#define SYS_GPF_MFPL_PF3MFP_I2C2_SMBAL      (0x8UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for I2C2_SMBAL        */
#define SYS_GPF_MFPL_PF3MFP_TM0_EXT         (0xFUL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for TM0_EXT           */

/* PF.4 MFP */
#define SYS_GPF_MFPL_PF4MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for GPIO              */
#define SYS_GPF_MFPL_PF4MFP_UART2_TXD       (0x2UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for UART2_TXD         */
#define SYS_GPF_MFPL_PF4MFP_UART2_nRTS      (0x4UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for UART2_nRTS        */
#define SYS_GPF_MFPL_PF4MFP_UART0_nRTS      (0x5UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for UART0_nRTS        */
#define SYS_GPF_MFPL_PF4MFP_BPWM0_CH5       (0x8UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for BPWM0_CH5         */
#define SYS_GPF_MFPL_PF4MFP_X32_OUT         (0xAUL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for X32_OUT           */

/* PF.5 MFP */
#define SYS_GPF_MFPL_PF5MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for GPIO              */
#define SYS_GPF_MFPL_PF5MFP_UART2_RXD       (0x2UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for UART2_RXD         */
#define SYS_GPF_MFPL_PF5MFP_UART2_nCTS      (0x4UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for UART2_nCTS        */
#define SYS_GPF_MFPL_PF5MFP_UART0_nCTS      (0x5UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for UART0_nCTS        */
#define SYS_GPF_MFPL_PF5MFP_BPWM0_CH4       (0x8UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for BPWM0_CH4         */
#define SYS_GPF_MFPL_PF5MFP_X32_IN          (0xAUL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for X32_IN            */

/* PF.6 MFP */
#define SYS_GPF_MFPL_PF6MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF6MFP_Pos)    /*!< GPF_MFPL PF6 setting for GPIO              */
#define SYS_GPF_MFPL_PF6MFP_SPI0_MOSI       (0x5UL<<SYS_GPF_MFPL_PF6MFP_Pos)    /*!< GPF_MFPL PF6 setting for SPI0_MOSI         */

/* PF.7 MFP */
#define SYS_GPF_MFPL_PF7MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF7MFP_Pos)    /*!< GPF_MFPL PF7 setting for GPIO              */
#define SYS_GPF_MFPL_PF7MFP_SPI0_MISO       (0x5UL<<SYS_GPF_MFPL_PF7MFP_Pos)    /*!< GPF_MFPL PF7 setting for SPI0_MISO         */

/* PF.8 MFP */
#define SYS_GPF_MFPH_PF8MFP_GPIO            (0x0UL<<SYS_GPF_MFPH_PF8MFP_Pos)    /*!< GPF_MFPH PF8 setting for GPIO              */
#define SYS_GPF_MFPH_PF8MFP_SPI0_CLK        (0x5UL<<SYS_GPF_MFPH_PF8MFP_Pos)    /*!< GPF_MFPH PF8 setting for SPI0_CLK          */

/* PF.9 MFP */
#define SYS_GPF_MFPH_PF9MFP_GPIO            (0x0UL<<SYS_GPF_MFPH_PF9MFP_Pos)    /*!< GPF_MFPH PF9 setting for GPIO              */
#define SYS_GPF_MFPH_PF9MFP_SPI0_SS         (0x5UL<<SYS_GPF_MFPH_PF9MFP_Pos)    /*!< GPF_MFPH PF9 setting for SPI0_SS           */

/* PF.14 MFP */
#define SYS_GPF_MFPH_PF14MFP_GPIO           (0x0UL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for GPIO             */

/* PG.2 MFP */
#define SYS_GPG_MFPL_PG2MFP_GPIO            (0x0UL<<SYS_GPG_MFPL_PG2MFP_Pos)    /*!< GPG_MFPL PG2 setting for GPIO              */
#define SYS_GPG_MFPL_PG2MFP_SPI2_SS         (0x3UL<<SYS_GPG_MFPL_PG2MFP_Pos)    /*!< GPG_MFPL PG2 setting for SPI2_SS           */
#define SYS_GPG_MFPL_PG2MFP_I2C0_SMBAL      (0x4UL<<SYS_GPG_MFPL_PG2MFP_Pos)    /*!< GPG_MFPL PG2 setting for I2C0_SMBAL        */
#define SYS_GPG_MFPL_PG2MFP_I2C1_SCL        (0x5UL<<SYS_GPG_MFPL_PG2MFP_Pos)    /*!< GPG_MFPL PG2 setting for I2C1_SCL          */
#define SYS_GPG_MFPL_PG2MFP_TM0             (0xDUL<<SYS_GPG_MFPL_PG2MFP_Pos)    /*!< GPG_MFPL PG2 setting for TM0               */

/* PG.3 MFP */
#define SYS_GPG_MFPL_PG3MFP_GPIO            (0x0UL<<SYS_GPG_MFPL_PG3MFP_Pos)    /*!< GPG_MFPL PG3 setting for GPIO              */
#define SYS_GPG_MFPL_PG3MFP_SPI2_CLK        (0x3UL<<SYS_GPG_MFPL_PG3MFP_Pos)    /*!< GPG_MFPL PG3 setting for SPI2_CLK          */
#define SYS_GPG_MFPL_PG3MFP_I2C0_SMBSUS     (0x4UL<<SYS_GPG_MFPL_PG3MFP_Pos)    /*!< GPG_MFPL PG3 setting for I2C0_SMBSUS       */
#define SYS_GPG_MFPL_PG3MFP_I2C1_SDA        (0x5UL<<SYS_GPG_MFPL_PG3MFP_Pos)    /*!< GPG_MFPL PG3 setting for I2C1_SDA          */
#define SYS_GPG_MFPL_PG3MFP_TM1             (0xDUL<<SYS_GPG_MFPL_PG3MFP_Pos)    /*!< GPG_MFPL PG3 setting for TM1               */

/* PG.4 MFP */
#define SYS_GPG_MFPL_PG4MFP_GPIO            (0x0UL<<SYS_GPG_MFPL_PG4MFP_Pos)    /*!< GPG_MFPL PG4 setting for GPIO              */
#define SYS_GPG_MFPL_PG4MFP_SPI2_MISO       (0x3UL<<SYS_GPG_MFPL_PG4MFP_Pos)    /*!< GPG_MFPL PG4 setting for SPI2_MISO         */
#define SYS_GPG_MFPL_PG4MFP_TM2             (0xDUL<<SYS_GPG_MFPL_PG4MFP_Pos)    /*!< GPG_MFPL PG4 setting for TM2               */

/* PH.8 MFP */
#define SYS_GPH_MFPH_PH8MFP_GPIO            (0x0UL<<SYS_GPH_MFPH_PH8MFP_Pos)    /*!< GPH_MFPH PH8 setting for GPIO              */
#define SYS_GPH_MFPH_PH8MFP_SPI1_CLK        (0x6UL<<SYS_GPH_MFPH_PH8MFP_Pos)    /*!< GPH_MFPH PH8 setting for SPI1_CLK          */
#define SYS_GPH_MFPH_PH8MFP_I2C1_SMBAL      (0x7UL<<SYS_GPH_MFPH_PH8MFP_Pos)    /*!< GPH_MFPH PH8 setting for I2C1_SMBAL        */
#define SYS_GPH_MFPH_PH8MFP_LCD_PIN37       (0x8UL<<SYS_GPH_MFPH_PH8MFP_Pos)    /*!< GPH_MFPH PH8 setting for LCD_PIN37         */
#define SYS_GPH_MFPH_PH8MFP_I2C2_SCL        (0x9UL<<SYS_GPH_MFPH_PH8MFP_Pos)    /*!< GPH_MFPH PH8 setting for I2C2_SCL          */
#define SYS_GPH_MFPH_PH8MFP_UART1_TXD       (0xAUL<<SYS_GPH_MFPH_PH8MFP_Pos)    /*!< GPH_MFPH PH8 setting for UART1_TXD         */

/* PH.9 MFP */
#define SYS_GPH_MFPH_PH9MFP_GPIO            (0x0UL<<SYS_GPH_MFPH_PH9MFP_Pos)    /*!< GPH_MFPH PH9 setting for GPIO              */
#define SYS_GPH_MFPH_PH9MFP_SPI1_SS         (0x6UL<<SYS_GPH_MFPH_PH9MFP_Pos)    /*!< GPH_MFPH PH9 setting for SPI1_SS           */
#define SYS_GPH_MFPH_PH9MFP_I2C1_SMBSUS     (0x7UL<<SYS_GPH_MFPH_PH9MFP_Pos)    /*!< GPH_MFPH PH9 setting for I2C1_SMBSUS       */
#define SYS_GPH_MFPH_PH9MFP_LCD_PIN38       (0x8UL<<SYS_GPH_MFPH_PH9MFP_Pos)    /*!< GPH_MFPH PH9 setting for LCD_PIN38         */
#define SYS_GPH_MFPH_PH9MFP_I2C2_SDA        (0x9UL<<SYS_GPH_MFPH_PH9MFP_Pos)    /*!< GPH_MFPH PH9 setting for I2C2_SDA          */
#define SYS_GPH_MFPH_PH9MFP_UART1_RXD       (0xAUL<<SYS_GPH_MFPH_PH9MFP_Pos)    /*!< GPH_MFPH PH9 setting for UART1_RXD         */

#define SYS_TIMEOUT_ERR             (-1)    /*!< SYS timeout error value \hideinitializer */

/*@}*/ /* end of group SYS_EXPORTED_CONSTANTS */


/** @addtogroup SYS_EXPORTED_MACROS SYS Exported Macros
  @{
*/

/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  * \hideinitializer
  */
#define SYS_CLEAR_BOD_INT_FLAG()        (SYS->BODCTL |= SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_CLEAR_BOD_LPM()             (SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_BOD()               (SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_BOD()                (SYS->BODCTL |= SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  * \hideinitializer
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCTL & SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  * \hideinitializer
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCTL & SYS_BODCTL_BODOUT_Msk)

/**
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_BOD_RST()           (SYS->BODCTL &= ~SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_SET_BOD_LPM()               (SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_4_4V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_3_0V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_2_4V
  *             - \ref SYS_BODCTL_BODVL_2_0V
  *             - \ref SYS_BODCTL_BODVL_1_8V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_SET_BOD_LEVEL(u32Level)     (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level))

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  * \hideinitializer
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSTS & SYS_RSTSTS_BODRF_Msk)

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  * \hideinitializer
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)

/**
  * @brief      Get reset source is from LVR Reset
  * @param      None
  * @retval     0   Previous reset source is not from Low-Voltage-Reset
  * @retval     >=1 Previous reset source is from Low-Voltage-Reset
  * @details    This macro get previous reset source is from Low-Voltage-Reset.
  * \hideinitializer
  */
#define SYS_IS_LVR_RST()                (SYS->RSTSTS & SYS_RSTSTS_LVRF_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  * \hideinitializer
  */
#define SYS_IS_POR_RST()                (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  * \hideinitializer
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  * \hideinitializer
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSTS & SYS_RSTSTS_SYSRF_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  * \hideinitializer
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)

/**
  * @brief      Disable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_LVR()               (SYS->BODCTL &= ~SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_LVR()                (SYS->BODCTL |= SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_POR()               (SYS->PORDISAN = 0x5AA5UL)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_POR()               (SYS->PORDISAN = 0UL)

/**
  * @brief      Clear reset source flag
  * @param[in]  u32RstSrc is reset source. Including :
  *             - \ref SYS_RSTSTS_PORF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_CPULKRF_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  * \hideinitializer
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) ((SYS->RSTSTS) = (u32RstSrc) )

/*@}*/ /* end of group SYS_EXPORTED_MACROS */

extern int32_t g_SYS_i32ErrCode;

/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/
/*--------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*--------------------------------------------------------------------------*/
/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void SYS_UnlockReg(void);
__STATIC_INLINE void SYS_LockReg(void);

/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    do
    {
        SYS->REGLCTL = 0x59UL;
        SYS->REGLCTL = 0x16UL;
        SYS->REGLCTL = 0x88UL;
    }
    while(SYS->REGLCTL == 0UL);
}

/**
  * @brief      Enable register write-protection function
  * @param      None
  * @return     None
  * @details    This function is used to enable register write-protection function.
  *             To lock the protected register to forbid write access.
  */
__STATIC_INLINE void SYS_LockReg(void)
{
    SYS->REGLCTL = 0UL;
}


void     SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void     SYS_ResetChip(void);
void     SYS_ResetCPU(void);
void     SYS_ResetModule(uint32_t u32ModuleIndex);
void     SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void     SYS_DisableBOD(void);
void     SYS_SetPowerLevel(uint32_t u32PowerLevel);
uint32_t SYS_SetPowerRegulator(uint32_t u32PowerRegulator);
void     SYS_SetVRef(uint32_t u32VRefCTL);

/*@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SYS_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __SYS_H__ */

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
