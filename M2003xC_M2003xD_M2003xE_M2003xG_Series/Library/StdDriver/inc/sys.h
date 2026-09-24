/**************************************************************************//**
 * @file     sys.h
 * @version  V0.10
 * @brief    System Manager (SYS) driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
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

/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_RST    ((0x0UL<<24)|(uint32_t)SYS_IPRST0_PDMARST_Pos)      /*!< PDMA reset is one of the SYS_ResetModule parameter */
#define CRC_RST     ((0x0UL<<24)|(uint32_t)SYS_IPRST0_CRCRST_Pos)       /*!< CRC reset is one of the SYS_ResetModule parameter */

#define GPIO_RST    ((0x4UL<<24)|(uint32_t)SYS_IPRST1_GPIORST_Pos)      /*!< GPIO reset is one of the SYS_ResetModule parameter */
#define TMR0_RST    ((0x4UL<<24)|(uint32_t)SYS_IPRST1_TMR0RST_Pos)      /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST    ((0x4UL<<24)|(uint32_t)SYS_IPRST1_TMR1RST_Pos)      /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST    ((0x4UL<<24)|(uint32_t)SYS_IPRST1_TMR2RST_Pos)      /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST    ((0x4UL<<24)|(uint32_t)SYS_IPRST1_TMR3RST_Pos)      /*!< TMR3 reset is one of the SYS_ResetModule parameter */
#define I2C0_RST    ((0x4UL<<24)|(uint32_t)SYS_IPRST1_I2C0RST_Pos)      /*!< I2C0 reset is one of the SYS_ResetModule parameter */
#define QSPI0_RST   ((0x4UL<<24)|(uint32_t)SYS_IPRST1_QSPI0RST_Pos)     /*!< QSPI0 reset is one of the SYS_ResetModule parameter */
#define UART0_RST   ((0x4UL<<24)|(uint32_t)SYS_IPRST1_UART0RST_Pos)     /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST   ((0x4UL<<24)|(uint32_t)SYS_IPRST1_UART1RST_Pos)     /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define UART2_RST   ((0x4UL<<24)|(uint32_t)SYS_IPRST1_UART2RST_Pos)     /*!< UART2 reset is one of the SYS_ResetModule parameter */
#define CAN0_RST    ((0x4UL<<24)|(uint32_t)SYS_IPRST1_CAN0RST_Pos)      /*!< CAN0 reset is one of the SYS_ResetModule parameter */
#define ADC_RST     ((0x4UL<<24)|(uint32_t)SYS_IPRST1_ADCRST_Pos)       /*!< ADC reset is one of the SYS_ResetModule parameter */

#define USCI0_RST   ((0x8UL<<24)|(uint32_t)SYS_IPRST2_USCI0RST_Pos)     /*!< USCI0 reset is one of the SYS_ResetModule parameter */
#define USCI1_RST   ((0x8UL<<24)|(uint32_t)SYS_IPRST2_USCI1RST_Pos)     /*!< USCI1 reset is one of the SYS_ResetModule parameter */
#define USCI2_RST   ((0x8UL<<24)|(uint32_t)SYS_IPRST2_USCI2RST_Pos)     /*!< USCI2 reset is one of the SYS_ResetModule parameter */
#define USCI3_RST   ((0x8UL<<24)|(uint32_t)SYS_IPRST2_USCI3RST_Pos)     /*!< USCI3 reset is one of the SYS_ResetModule parameter */
#define PWM0_RST    ((0x8UL<<24)|(uint32_t)SYS_IPRST2_PWM0RST_Pos)      /*!< PWM0 reset is one of the SYS_ResetModule parameter */
#define PWM1_RST    ((0x8UL<<24)|(uint32_t)SYS_IPRST2_PWM1RST_Pos)      /*!< PWM1 reset is one of the SYS_ResetModule parameter */
#define ECAP0_RST   ((0x8UL<<24)|(uint32_t)SYS_IPRST2_ECAP0RST_Pos)     /*!< ECAP0 reset is one of the SYS_ResetModule parameter */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_RST_EN           (1UL<<SYS_BODCTL_BODRSTEN_Pos)  /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_INTERRUPT_EN     (0UL<<SYS_BODCTL_BODRSTEN_Pos)  /*!< Brown-out Interrupt Enable */

#define SYS_BODCTL_BODVL_4_4V           (6UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 4.4V */
#define SYS_BODCTL_BODVL_3_7V           (4UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCTL_BODVL_2_7V           (2UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCTL_BODVL_2_2V           (0UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */

#define SYS_BODCTL_LVRDGSEL_0HCLK       (0x0UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time Without de-glitch function.  \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_4HCLK       (0x1UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 4HCLK            \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_8HCLK       (0x2UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 8HCLK            \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_16HCLK      (0x3UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 16HCLK           \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_32HCLK      (0x4UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 32HCLK           \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_64HCLK      (0x5UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 64HCLK           \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_128HCLK     (0x6UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 128HCLK          \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_256HCLK     (0x7UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 256HCLK          \hideinitializer */

#define SYS_BODCTL_BODDGSEL_0HCLK       (0x0UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is sampled by RC10K clock.   \hideinitializer */
#define SYS_BODCTL_BODDGSEL_4HCLK       (0x1UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 4HCLK            \hideinitializer */
#define SYS_BODCTL_BODDGSEL_8HCLK       (0x2UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 8HCLK            \hideinitializer */
#define SYS_BODCTL_BODDGSEL_16HCLK      (0x3UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 16HCLK           \hideinitializer */
#define SYS_BODCTL_BODDGSEL_32HCLK      (0x4UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 32HCLK           \hideinitializer */
#define SYS_BODCTL_BODDGSEL_64HCLK      (0x5UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 64HCLK           \hideinitializer */
#define SYS_BODCTL_BODDGSEL_128HCLK     (0x6UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 128HCLK          \hideinitializer */
#define SYS_BODCTL_BODDGSEL_256HCLK     (0x7UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 256HCLK          \hideinitializer */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* How to use below #define?

Example: If user want to set PA.1 as UART0_TXD and PA.0 as UART0_RXD in initial function,
         user can issue following command to achieve it.

    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_UART0_TXD;
*/

/* PA.0 MFP */
#define SYS_GPA_MFPL_PA0MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for GPIO              */
#define SYS_GPA_MFPL_PA0MFP_QSPI0_MOSI0     (0x3UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for QSPI0_MOSI0       */
#define SYS_GPA_MFPL_PA0MFP_UART0_RXD       (0x7UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for UART0_RXD         */
#define SYS_GPA_MFPL_PA0MFP_UART1_nRTS      (0x8UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for UART1_nRTS        */
#define SYS_GPA_MFPL_PA0MFP_PWM0_CH5        (0xdUL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for PWM0_CH5          */

/* PA.1 MFP */
#define SYS_GPA_MFPL_PA1MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for GPIO              */
#define SYS_GPA_MFPL_PA1MFP_QSPI0_MISO0     (0x3UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for QSPI0_MISO0       */
#define SYS_GPA_MFPL_PA1MFP_UART0_TXD       (0x7UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for UART0_TXD         */
#define SYS_GPA_MFPL_PA1MFP_UART1_nCTS      (0x8UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for UART1_nCTS        */
#define SYS_GPA_MFPL_PA1MFP_PWM0_CH4        (0xdUL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for PWM0_CH4          */

/* PA.2 MFP */
#define SYS_GPA_MFPL_PA2MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for GPIO              */
#define SYS_GPA_MFPL_PA2MFP_QSPI0_CLK       (0x3UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for QSPI0_CLK         */
#define SYS_GPA_MFPL_PA2MFP_UART1_RXD       (0x8UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for UART1_RXD         */
#define SYS_GPA_MFPL_PA2MFP_I2C0_SMBSUS     (0x9UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for I2C0_SMBSUS       */
#define SYS_GPA_MFPL_PA2MFP_PWM0_CH3        (0xdUL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for PWM0_CH3          */

/* PA.3 MFP */
#define SYS_GPA_MFPL_PA3MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for GPIO              */
#define SYS_GPA_MFPL_PA3MFP_QSPI0_SS        (0x3UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for QSPI0_SS          */
#define SYS_GPA_MFPL_PA3MFP_UART1_TXD       (0x8UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for UART1_TXD         */
#define SYS_GPA_MFPL_PA3MFP_I2C0_SMBAL      (0x9UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for I2C0_SMBAL        */
#define SYS_GPA_MFPL_PA3MFP_PWM0_CH2        (0xdUL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for PWM0_CH2          */
#define SYS_GPA_MFPL_PA3MFP_CLKO            (0xeUL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for CLKO              */
#define SYS_GPA_MFPL_PA3MFP_PWM1_BRAKE1     (0xfUL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for PWM1_BRAKE1       */

/* PA.4 MFP */
#define SYS_GPA_MFPL_PA4MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for GPIO              */
#define SYS_GPA_MFPL_PA4MFP_QSPI0_MOSI1     (0x3UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for QSPI0_MOSI1       */
#define SYS_GPA_MFPL_PA4MFP_UART0_nRTS      (0x7UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for UART0_nRTS        */
#define SYS_GPA_MFPL_PA4MFP_UART0_RXD       (0x8UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for UART0_RXD         */
#define SYS_GPA_MFPL_PA4MFP_I2C0_SDA        (0x9UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for I2C0_SDA          */
#define SYS_GPA_MFPL_PA4MFP_CAN0_RXD        (0xaUL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for CAN0_RXD          */
#define SYS_GPA_MFPL_PA4MFP_PWM0_CH1        (0xdUL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for PWM0_CH1          */

/* PA.5 MFP */
#define SYS_GPA_MFPL_PA5MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for GPIO              */
#define SYS_GPA_MFPL_PA5MFP_QSPI0_MISO1     (0x3UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for QSPI0_MISO1       */
#define SYS_GPA_MFPL_PA5MFP_CAN0_TXH        (0x6UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for CAN0_TXH          */
#define SYS_GPA_MFPL_PA5MFP_UART0_nCTS      (0x7UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for UART0_nCTS        */
#define SYS_GPA_MFPL_PA5MFP_UART0_TXD       (0x8UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for UART0_TXD         */
#define SYS_GPA_MFPL_PA5MFP_I2C0_SCL        (0x9UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for I2C0_SCL          */
#define SYS_GPA_MFPL_PA5MFP_CAN0_TXD        (0xaUL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for CAN0_TXD          */
#define SYS_GPA_MFPL_PA5MFP_PWM0_CH0        (0xdUL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for PWM0_CH0          */

/* PA.6 MFP */
#define SYS_GPA_MFPL_PA6MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for GPIO              */
#define SYS_GPA_MFPL_PA6MFP_CAN0_TXL        (0x6UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for CAN0_TXL          */
#define SYS_GPA_MFPL_PA6MFP_UART0_RXD       (0x7UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for UART0_RXD         */
#define SYS_GPA_MFPL_PA6MFP_PWM1_CH5        (0xbUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for PWM1_CH5          */
#define SYS_GPA_MFPL_PA6MFP_TM3             (0xeUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for TM3               */
#define SYS_GPA_MFPL_PA6MFP_INT0            (0xfUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for INT0              */

/* PA.7 MFP */
#define SYS_GPA_MFPL_PA7MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for GPIO              */
#define SYS_GPA_MFPL_PA7MFP_UART0_TXD       (0x7UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for UART0_TXD         */
#define SYS_GPA_MFPL_PA7MFP_PWM1_CH4        (0xbUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for PWM1_CH4          */
#define SYS_GPA_MFPL_PA7MFP_TM2             (0xeUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for TM2               */
#define SYS_GPA_MFPL_PA7MFP_INT1            (0xfUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for INT1              */

/* PA.8 MFP */
#define SYS_GPA_MFPH_PA8MFP_GPIO            (0x0UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for GPIO              */
#define SYS_GPA_MFPH_PA8MFP_USCI0_CTL1      (0x6UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for USCI0_CTL1        */
#define SYS_GPA_MFPH_PA8MFP_UART1_RXD       (0x7UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for UART1_RXD         */
#define SYS_GPA_MFPH_PA8MFP_ECAP0_IC2       (0xbUL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for ECAP0_IC2         */
#define SYS_GPA_MFPH_PA8MFP_TM3_EXT         (0xdUL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for TM3_EXT           */
#define SYS_GPA_MFPH_PA8MFP_INT4            (0xfUL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for INT4              */

/* PA.9 MFP */
#define SYS_GPA_MFPH_PA9MFP_GPIO            (0x0UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for GPIO              */
#define SYS_GPA_MFPH_PA9MFP_USCI0_DAT1      (0x6UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for USCI0_DAT1        */
#define SYS_GPA_MFPH_PA9MFP_UART1_TXD       (0x7UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for UART1_TXD         */
#define SYS_GPA_MFPH_PA9MFP_ECAP0_IC1       (0xbUL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for ECAP0_IC1         */
#define SYS_GPA_MFPH_PA9MFP_TM2_EXT         (0xdUL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for TM2_EXT           */

/* PA.10 MFP */
#define SYS_GPA_MFPH_PA10MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for GPIO             */
#define SYS_GPA_MFPH_PA10MFP_USCI0_DAT0     (0x6UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for USCI0_DAT0       */
#define SYS_GPA_MFPH_PA10MFP_ECAP0_IC0      (0xbUL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for ECAP0_IC0        */
#define SYS_GPA_MFPH_PA10MFP_TM1_EXT        (0xdUL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for TM1_EXT          */

/* PA.11 MFP */
#define SYS_GPA_MFPH_PA11MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for GPIO             */
#define SYS_GPA_MFPH_PA11MFP_USCI0_CLK      (0x6UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for USCI0_CLK        */
#define SYS_GPA_MFPH_PA11MFP_PWM0_BRAKE1    (0xbUL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for PWM0_BRAKE1      */
#define SYS_GPA_MFPH_PA11MFP_TM0_EXT        (0xdUL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for TM0_EXT          */

/* PA.12 MFP */
#define SYS_GPA_MFPH_PA12MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for GPIO             */
#define SYS_GPA_MFPH_PA12MFP_CAN0_TXD       (0x6UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for CAN0_TXD         */
#define SYS_GPA_MFPH_PA12MFP_CAN0_TXH       (0x7UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for CAN0_TXH         */
#define SYS_GPA_MFPH_PA12MFP_USCI2_CTL0     (0x9UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for USCI2_CTL0       */

/* PA.13 MFP */
#define SYS_GPA_MFPH_PA13MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for GPIO             */
#define SYS_GPA_MFPH_PA13MFP_CAN0_RXD       (0x6UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for CAN0_RXD         */
#define SYS_GPA_MFPH_PA13MFP_USCI2_CTL1     (0x9UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for USCI2_CTL1       */

/* PA.14 MFP */
#define SYS_GPA_MFPH_PA14MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for GPIO             */
#define SYS_GPA_MFPH_PA14MFP_UART0_TXD      (0x3UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for UART0_TXD        */
#define SYS_GPA_MFPH_PA14MFP_CAN0_TXL       (0x7UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for CAN0_TXL         */
#define SYS_GPA_MFPH_PA14MFP_USCI2_DAT1     (0x9UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for USCI2_DAT1       */

/* PA.15 MFP */
#define SYS_GPA_MFPH_PA15MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for GPIO             */
#define SYS_GPA_MFPH_PA15MFP_UART0_RXD      (0x3UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for UART0_RXD        */
#define SYS_GPA_MFPH_PA15MFP_USCI2_DAT0     (0x9UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for USCI2_DAT0       */

/* PB.0 MFP */
#define SYS_GPB_MFPL_PB0MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for GPIO              */
#define SYS_GPB_MFPL_PB0MFP_ADC0_CH0        (0x1UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for ADC0_CH0          */
#define SYS_GPB_MFPL_PB0MFP_ADC0_ST         (0x2UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for ADC0_ST           */
#define SYS_GPB_MFPL_PB0MFP_PWM1_CH5        (0x4UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for PWM1_CH5          */
#define SYS_GPB_MFPL_PB0MFP_QSPI0_MOSI1     (0x5UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for QSPI0_MOSI1       */
#define SYS_GPB_MFPL_PB0MFP_USCI0_CTL0      (0x6UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for USCI0_CTL0        */
#define SYS_GPB_MFPL_PB0MFP_PWM0_CH3        (0x7UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for PWM0_CH3          */
#define SYS_GPB_MFPL_PB0MFP_PWM0_CH5        (0xbUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for PWM0_CH5          */
#define SYS_GPB_MFPL_PB0MFP_PWM0_BRAKE1     (0xdUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for PWM0_BRAKE1       */
#define SYS_GPB_MFPL_PB0MFP_TM3_EXT         (0xfUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for TM3_EXT           */

/* PB.1 MFP */
#define SYS_GPB_MFPL_PB1MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for GPIO              */
#define SYS_GPB_MFPL_PB1MFP_ADC0_CH1        (0x1UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for ADC0_CH1          */
#define SYS_GPB_MFPL_PB1MFP_PWM1_CH4        (0x4UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for PWM1_CH4          */
#define SYS_GPB_MFPL_PB1MFP_QSPI0_MISO1     (0x5UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for QSPI0_MISO1       */
#define SYS_GPB_MFPL_PB1MFP_USCI3_CTL1      (0x6UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for USCI3_CTL1        */
#define SYS_GPB_MFPL_PB1MFP_PWM0_CH2        (0x7UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for PWM0_CH2          */
#define SYS_GPB_MFPL_PB1MFP_USCI1_CLK       (0x8UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for USCI1_CLK         */
#define SYS_GPB_MFPL_PB1MFP_PWM0_CH4        (0xbUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for PWM0_CH4          */
#define SYS_GPB_MFPL_PB1MFP_ECAP0_IC0       (0xcUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for ECAP0_IC0         */
#define SYS_GPB_MFPL_PB1MFP_PWM0_BRAKE0     (0xdUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for PWM0_BRAKE0       */
#define SYS_GPB_MFPL_PB1MFP_TM0             (0xeUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for TM0               */
#define SYS_GPB_MFPL_PB1MFP_TM0_EXT         (0xfUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for TM0_EXT           */

/* PB.2 MFP */
#define SYS_GPB_MFPL_PB2MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for GPIO              */
#define SYS_GPB_MFPL_PB2MFP_ADC0_CH2        (0x1UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for ADC0_CH2          */
#define SYS_GPB_MFPL_PB2MFP_UART0_TXD       (0x5UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for UART0_TXD         */
#define SYS_GPB_MFPL_PB2MFP_UART1_RXD       (0x6UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for UART1_RXD         */
#define SYS_GPB_MFPL_PB2MFP_USCI1_DAT0      (0x8UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for USCI1_DAT0        */
#define SYS_GPB_MFPL_PB2MFP_PWM0_CH3        (0xbUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for PWM0_CH3          */
#define SYS_GPB_MFPL_PB2MFP_ECAP0_IC1       (0xcUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for ECAP0_IC1         */
#define SYS_GPB_MFPL_PB2MFP_TM3             (0xeUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for TM3               */
#define SYS_GPB_MFPL_PB2MFP_INT3            (0xfUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for INT3              */

/* PB.3 MFP */
#define SYS_GPB_MFPL_PB3MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for GPIO              */
#define SYS_GPB_MFPL_PB3MFP_ADC0_CH3        (0x1UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for ADC0_CH3          */
#define SYS_GPB_MFPL_PB3MFP_UART0_RXD       (0x5UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for UART0_RXD         */
#define SYS_GPB_MFPL_PB3MFP_UART1_TXD       (0x6UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for UART1_TXD         */
#define SYS_GPB_MFPL_PB3MFP_USCI1_DAT1      (0x8UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for USCI1_DAT1        */
#define SYS_GPB_MFPL_PB3MFP_PWM0_CH2        (0xbUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for PWM0_CH2          */
#define SYS_GPB_MFPL_PB3MFP_ECAP0_IC2       (0xcUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for ECAP0_IC2         */
#define SYS_GPB_MFPL_PB3MFP_PWM0_BRAKE0     (0xdUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for PWM0_BRAKE0       */
#define SYS_GPB_MFPL_PB3MFP_TM2             (0xeUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for TM2               */
#define SYS_GPB_MFPL_PB3MFP_INT2            (0xfUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for INT2              */

/* PB.4 MFP */
#define SYS_GPB_MFPL_PB4MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for GPIO              */
#define SYS_GPB_MFPL_PB4MFP_ADC0_CH4        (0x1UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for ADC0_CH4          */
#define SYS_GPB_MFPL_PB4MFP_I2C0_SDA        (0x6UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for I2C0_SDA          */
#define SYS_GPB_MFPL_PB4MFP_USCI1_CTL1      (0x8UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for USCI1_CTL1        */
#define SYS_GPB_MFPL_PB4MFP_PWM0_CH1        (0xbUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for PWM0_CH1          */
#define SYS_GPB_MFPL_PB4MFP_UART2_RXD       (0xdUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for UART2_RXD         */
#define SYS_GPB_MFPL_PB4MFP_TM1             (0xeUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for TM1               */
#define SYS_GPB_MFPL_PB4MFP_INT1            (0xfUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for INT1              */

/* PB.5 MFP */
#define SYS_GPB_MFPL_PB5MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for GPIO              */
#define SYS_GPB_MFPL_PB5MFP_ADC0_CH5        (0x1UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for ADC0_CH5          */
#define SYS_GPB_MFPL_PB5MFP_I2C0_SCL        (0x6UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for I2C0_SCL          */
#define SYS_GPB_MFPL_PB5MFP_USCI1_CTL0      (0x8UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for USCI1_CTL0        */
#define SYS_GPB_MFPL_PB5MFP_PWM0_CH0        (0xbUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for PWM0_CH0          */
#define SYS_GPB_MFPL_PB5MFP_UART2_TXD       (0xdUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for UART2_TXD         */
#define SYS_GPB_MFPL_PB5MFP_TM0             (0xeUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for TM0               */
#define SYS_GPB_MFPL_PB5MFP_INT0            (0xfUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for INT0              */

/* PB.6 MFP */
#define SYS_GPB_MFPL_PB6MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for GPIO              */
#define SYS_GPB_MFPL_PB6MFP_ADC0_CH6        (0x1UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for ADC0_CH6          */
#define SYS_GPB_MFPL_PB6MFP_USCI1_DAT1      (0x4UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for USCI1_DAT1        */
#define SYS_GPB_MFPL_PB6MFP_UART1_RXD       (0x6UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for UART1_RXD         */
#define SYS_GPB_MFPL_PB6MFP_PWM1_BRAKE1     (0x7UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for PWM1_BRAKE1       */
#define SYS_GPB_MFPL_PB6MFP_PWM1_CH5        (0x8UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for PWM1_CH5          */
#define SYS_GPB_MFPL_PB6MFP_PWM0_BRAKE1     (0xbUL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for PWM0_BRAKE1       */
#define SYS_GPB_MFPL_PB6MFP_PWM0_CH5        (0xcUL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for PWM0_CH5          */
#define SYS_GPB_MFPL_PB6MFP_INT4            (0xdUL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for INT4              */

/* PB.7 MFP */
#define SYS_GPB_MFPL_PB7MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for GPIO              */
#define SYS_GPB_MFPL_PB7MFP_ADC0_CH7        (0x1UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for ADC0_CH7          */
#define SYS_GPB_MFPL_PB7MFP_USCI0_CLK       (0x2UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for USCI0_CLK         */
#define SYS_GPB_MFPL_PB7MFP_USCI1_DAT0      (0x4UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for USCI1_DAT0        */
#define SYS_GPB_MFPL_PB7MFP_UART1_TXD       (0x6UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for UART1_TXD         */
#define SYS_GPB_MFPL_PB7MFP_PWM1_BRAKE0     (0x7UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for PWM1_BRAKE0       */
#define SYS_GPB_MFPL_PB7MFP_PWM1_CH4        (0x8UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for PWM1_CH4          */
#define SYS_GPB_MFPL_PB7MFP_TM2_EXT         (0x9UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for TM2_EXT           */
#define SYS_GPB_MFPL_PB7MFP_PWM0_BRAKE0     (0xbUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for PWM0_BRAKE0       */
#define SYS_GPB_MFPL_PB7MFP_PWM0_CH4        (0xcUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for PWM0_CH4          */
#define SYS_GPB_MFPL_PB7MFP_INT5            (0xdUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for INT5              */
#define SYS_GPB_MFPL_PB7MFP_PWM0_CH2        (0xeUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for PWM0_CH2          */
#define SYS_GPB_MFPL_PB7MFP_USCI3_CTL0      (0xfUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for USCI3_CTL0        */

/* PB.8 MFP */
#define SYS_GPB_MFPH_PB8MFP_GPIO            (0x0UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for GPIO              */
#define SYS_GPB_MFPH_PB8MFP_ADC0_CH8        (0x1UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for ADC0_CH8          */
#define SYS_GPB_MFPH_PB8MFP_USCI0_DAT0      (0x2UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for USCI0_DAT0        */
#define SYS_GPB_MFPH_PB8MFP_USCI1_CLK       (0x4UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for USCI1_CLK         */
#define SYS_GPB_MFPH_PB8MFP_UART0_RXD       (0x5UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for UART0_RXD         */
#define SYS_GPB_MFPH_PB8MFP_UART1_nRTS      (0x6UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for UART1_nRTS        */
#define SYS_GPB_MFPH_PB8MFP_I2C0_SDA        (0x8UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for I2C0_SDA          */
#define SYS_GPB_MFPH_PB8MFP_TM3_EXT         (0x9UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for TM3_EXT           */
#define SYS_GPB_MFPH_PB8MFP_PWM0_CH3        (0xcUL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for PWM0_CH3          */
#define SYS_GPB_MFPH_PB8MFP_TM1             (0xeUL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for TM1               */
#define SYS_GPB_MFPH_PB8MFP_USCI3_CLK       (0xfUL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for USCI3_CLK         */

/* PB.9 MFP */
#define SYS_GPB_MFPH_PB9MFP_GPIO            (0x0UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for GPIO              */
#define SYS_GPB_MFPH_PB9MFP_ADC0_CH9        (0x1UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for ADC0_CH9          */
#define SYS_GPB_MFPH_PB9MFP_USCI0_DAT1      (0x2UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for USCI0_DAT1        */
#define SYS_GPB_MFPH_PB9MFP_USCI1_CTL1      (0x4UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for USCI1_CTL1        */
#define SYS_GPB_MFPH_PB9MFP_UART0_TXD       (0x5UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for UART0_TXD         */
#define SYS_GPB_MFPH_PB9MFP_UART1_nCTS      (0x6UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for UART1_nCTS        */
#define SYS_GPB_MFPH_PB9MFP_I2C0_SCL        (0x8UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for I2C0_SCL          */
#define SYS_GPB_MFPH_PB9MFP_TM1_EXT         (0x9UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for TM1_EXT           */
#define SYS_GPB_MFPH_PB9MFP_PWM0_CH4        (0xcUL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for PWM0_CH4          */
#define SYS_GPB_MFPH_PB9MFP_USCI3_DAT1      (0xfUL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for USCI3_DAT1        */

/* PB.10 MFP */
#define SYS_GPB_MFPH_PB10MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for GPIO             */
#define SYS_GPB_MFPH_PB10MFP_ADC0_CH10      (0x1UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for ADC0_CH10        */
#define SYS_GPB_MFPH_PB10MFP_USCI1_CTL0     (0x4UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for USCI1_CTL0       */
#define SYS_GPB_MFPH_PB10MFP_UART0_nRTS     (0x5UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for UART0_nRTS       */
#define SYS_GPB_MFPH_PB10MFP_CAN0_RXD       (0x8UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for CAN0_RXD         */
#define SYS_GPB_MFPH_PB10MFP_USCI3_DAT0     (0xfUL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for USCI3_DAT0       */

/* PB.11 MFP */
#define SYS_GPB_MFPH_PB11MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for GPIO             */
#define SYS_GPB_MFPH_PB11MFP_ADC0_CH11      (0x1UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for ADC0_CH11        */
#define SYS_GPB_MFPH_PB11MFP_USCI0_CTL0     (0x2UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for USCI0_CTL0       */
#define SYS_GPB_MFPH_PB11MFP_UART0_nCTS     (0x5UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for UART0_nCTS       */
#define SYS_GPB_MFPH_PB11MFP_CAN0_TXD       (0x8UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for CAN0_TXD         */
#define SYS_GPB_MFPH_PB11MFP_TM2_EXT        (0x9UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for TM2_EXT          */
#define SYS_GPB_MFPH_PB11MFP_CAN0_TXH       (0xbUL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for CAN0_TXH         */
#define SYS_GPB_MFPH_PB11MFP_PWM0_CH5       (0xcUL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for PWM0_CH5         */
#define SYS_GPB_MFPH_PB11MFP_USCI3_CTL1     (0xfUL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for USCI3_CTL1       */

/* PB.12 MFP */
#define SYS_GPB_MFPH_PB12MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for GPIO             */
#define SYS_GPB_MFPH_PB12MFP_ADC0_CH12      (0x1UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for ADC0_CH12        */
#define SYS_GPB_MFPH_PB12MFP_USCI0_CLK      (0x5UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for USCI0_CLK        */
#define SYS_GPB_MFPH_PB12MFP_UART0_RXD      (0x6UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for UART0_RXD        */
#define SYS_GPB_MFPH_PB12MFP_PWM1_CH3       (0x8UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for PWM1_CH3         */
#define SYS_GPB_MFPH_PB12MFP_CAN0_TXL       (0x9UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for CAN0_TXL         */
#define SYS_GPB_MFPH_PB12MFP_PWM0_CH1       (0xaUL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for PWM0_CH1         */
#define SYS_GPB_MFPH_PB12MFP_QSPI0_CLK      (0xcUL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for QSPI0_CLK        */
#define SYS_GPB_MFPH_PB12MFP_TM3_EXT        (0xdUL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for TM3_EXT          */
#define SYS_GPB_MFPH_PB12MFP_CLKO           (0xeUL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for CLKO             */

/* PB.13 MFP */
#define SYS_GPB_MFPH_PB13MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for GPIO             */
#define SYS_GPB_MFPH_PB13MFP_ADC0_CH13      (0x1UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for ADC0_CH13        */
#define SYS_GPB_MFPH_PB13MFP_USCI0_DAT0     (0x5UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for USCI0_DAT0       */
#define SYS_GPB_MFPH_PB13MFP_UART0_TXD      (0x6UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for UART0_TXD        */
#define SYS_GPB_MFPH_PB13MFP_UART0_nRTS     (0x7UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for UART0_nRTS       */
#define SYS_GPB_MFPH_PB13MFP_PWM1_CH2       (0x8UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for PWM1_CH2         */
#define SYS_GPB_MFPH_PB13MFP_CLKO           (0x9UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for CLKO             */
#define SYS_GPB_MFPH_PB13MFP_PWM0_CH0       (0xaUL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for PWM0_CH0         */
#define SYS_GPB_MFPH_PB13MFP_USCI0_CTL0     (0xcUL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for USCI0_CTL0       */
#define SYS_GPB_MFPH_PB13MFP_TM2_EXT        (0xdUL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for TM2_EXT          */

/* PB.14 MFP */
#define SYS_GPB_MFPH_PB14MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for GPIO             */
#define SYS_GPB_MFPH_PB14MFP_ADC0_ST        (0x2UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for ADC0_ST          */
#define SYS_GPB_MFPH_PB14MFP_I2C0_SCL       (0x3UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for I2C0_SCL         */
#define SYS_GPB_MFPH_PB14MFP_USCI0_DAT1     (0x5UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for USCI0_DAT1       */
#define SYS_GPB_MFPH_PB14MFP_UART0_nRTS     (0x6UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for UART0_nRTS       */
#define SYS_GPB_MFPH_PB14MFP_UART0_RXD      (0x7UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for UART0_RXD        */
#define SYS_GPB_MFPH_PB14MFP_PWM1_CH1       (0x8UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for PWM1_CH1         */
#define SYS_GPB_MFPH_PB14MFP_TM2_EXT        (0xbUL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for TM2_EXT          */
#define SYS_GPB_MFPH_PB14MFP_TM1_EXT        (0xdUL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for TM1_EXT          */
#define SYS_GPB_MFPH_PB14MFP_CLKO           (0xeUL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for CLKO             */
#define SYS_GPB_MFPH_PB14MFP_PWM0_BRAKE1    (0xfUL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for PWM0_BRAKE1      */

/* PB.15 MFP */
#define SYS_GPB_MFPH_PB15MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for GPIO             */
#define SYS_GPB_MFPH_PB15MFP_I2C0_SDA       (0x3UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for I2C0_SDA         */
#define SYS_GPB_MFPH_PB15MFP_USCI0_CTL1     (0x5UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for USCI0_CTL1       */
#define SYS_GPB_MFPH_PB15MFP_UART0_nCTS     (0x6UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for UART0_nCTS       */
#define SYS_GPB_MFPH_PB15MFP_UART0_TXD      (0x7UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for UART0_TXD        */
#define SYS_GPB_MFPH_PB15MFP_PWM1_CH0       (0x8UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for PWM1_CH0         */
#define SYS_GPB_MFPH_PB15MFP_USCI2_CLK      (0x9UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for USCI2_CLK        */
#define SYS_GPB_MFPH_PB15MFP_PWM0_CH1       (0xaUL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for PWM0_CH1         */
#define SYS_GPB_MFPH_PB15MFP_TM1            (0xbUL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for TM1              */
#define SYS_GPB_MFPH_PB15MFP_TM0_EXT        (0xdUL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for TM0_EXT          */
#define SYS_GPB_MFPH_PB15MFP_PWM0_BRAKE1    (0xfUL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for PWM0_BRAKE1      */

/* PC.0 MFP */
#define SYS_GPC_MFPL_PC0MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for GPIO              */
#define SYS_GPC_MFPL_PC0MFP_QSPI0_MOSI0     (0x4UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for QSPI0_MOSI0       */
#define SYS_GPC_MFPL_PC0MFP_USCI1_CTL0      (0x6UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for USCI1_CTL0        */
#define SYS_GPC_MFPL_PC0MFP_UART2_RXD       (0x7UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for UART2_RXD         */
#define SYS_GPC_MFPL_PC0MFP_I2C0_SDA        (0x9UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for I2C0_SDA          */
#define SYS_GPC_MFPL_PC0MFP_PWM1_CH5        (0xcUL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for PWM1_CH5          */

/* PC.1 MFP */
#define SYS_GPC_MFPL_PC1MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for GPIO              */
#define SYS_GPC_MFPL_PC1MFP_QSPI0_MISO0     (0x4UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for QSPI0_MISO0       */
#define SYS_GPC_MFPL_PC1MFP_USCI1_CTL1      (0x6UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for USCI1_CTL1        */
#define SYS_GPC_MFPL_PC1MFP_UART2_TXD       (0x7UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for UART2_TXD         */
#define SYS_GPC_MFPL_PC1MFP_I2C0_SCL        (0x9UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for I2C0_SCL          */
#define SYS_GPC_MFPL_PC1MFP_ADC0_ST         (0xbUL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for ADC0_ST           */
#define SYS_GPC_MFPL_PC1MFP_PWM1_CH4        (0xcUL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for PWM1_CH4          */

/* PC.2 MFP */
#define SYS_GPC_MFPL_PC2MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for GPIO              */
#define SYS_GPC_MFPL_PC2MFP_QSPI0_CLK       (0x4UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for QSPI0_CLK         */
#define SYS_GPC_MFPL_PC2MFP_USCI1_DAT0      (0x6UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for USCI1_DAT0        */
#define SYS_GPC_MFPL_PC2MFP_UART2_nCTS      (0x7UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for UART2_nCTS        */
#define SYS_GPC_MFPL_PC2MFP_I2C0_SMBSUS     (0x9UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for I2C0_SMBSUS       */
#define SYS_GPC_MFPL_PC2MFP_PWM1_CH3        (0xcUL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for PWM1_CH3          */

/* PC.3 MFP */
#define SYS_GPC_MFPL_PC3MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for GPIO              */
#define SYS_GPC_MFPL_PC3MFP_QSPI0_SS        (0x4UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for QSPI0_SS          */
#define SYS_GPC_MFPL_PC3MFP_USCI1_DAT1      (0x6UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for USCI1_DAT1        */
#define SYS_GPC_MFPL_PC3MFP_UART2_nRTS      (0x7UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for UART2_nRTS        */
#define SYS_GPC_MFPL_PC3MFP_I2C0_SMBAL      (0x9UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for I2C0_SMBAL        */
#define SYS_GPC_MFPL_PC3MFP_PWM1_CH2        (0xcUL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for PWM1_CH2          */

/* PC.4 MFP */
#define SYS_GPC_MFPL_PC4MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for GPIO              */
#define SYS_GPC_MFPL_PC4MFP_QSPI0_MOSI1     (0x4UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for QSPI0_MOSI1       */
#define SYS_GPC_MFPL_PC4MFP_USCI1_CLK       (0x6UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for USCI1_CLK         */
#define SYS_GPC_MFPL_PC4MFP_UART2_RXD       (0x7UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for UART2_RXD         */
#define SYS_GPC_MFPL_PC4MFP_CAN0_RXD        (0xaUL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for CAN0_RXD          */
#define SYS_GPC_MFPL_PC4MFP_PWM1_CH1        (0xcUL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for PWM1_CH1          */

/* PC.5 MFP */
#define SYS_GPC_MFPL_PC5MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for GPIO              */
#define SYS_GPC_MFPL_PC5MFP_QSPI0_MISO1     (0x4UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for QSPI0_MISO1       */
#define SYS_GPC_MFPL_PC5MFP_CAN0_TXH        (0x6UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for CAN0_TXH          */
#define SYS_GPC_MFPL_PC5MFP_UART2_TXD       (0x7UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for UART2_TXD         */
#define SYS_GPC_MFPL_PC5MFP_CAN0_TXD        (0xaUL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for CAN0_TXD          */
#define SYS_GPC_MFPL_PC5MFP_PWM1_CH0        (0xcUL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for PWM1_CH0          */

/* PC.6 MFP */
#define SYS_GPC_MFPL_PC6MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for GPIO              */
#define SYS_GPC_MFPL_PC6MFP_UART0_nRTS      (0x7UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for UART0_nRTS        */
#define SYS_GPC_MFPL_PC6MFP_PWM1_CH3        (0xbUL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for PWM1_CH3          */
#define SYS_GPC_MFPL_PC6MFP_TM1             (0xeUL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for TM1               */
#define SYS_GPC_MFPL_PC6MFP_INT2            (0xfUL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for INT2              */

/* PC.7 MFP */
#define SYS_GPC_MFPL_PC7MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for GPIO              */
#define SYS_GPC_MFPL_PC7MFP_UART0_nCTS      (0x7UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for UART0_nCTS        */
#define SYS_GPC_MFPL_PC7MFP_PWM1_CH2        (0xbUL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for PWM1_CH2          */
#define SYS_GPC_MFPL_PC7MFP_TM0             (0xeUL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for TM0               */
#define SYS_GPC_MFPL_PC7MFP_INT3            (0xfUL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for INT3              */

/* PC.14 MFP */
#define SYS_GPC_MFPH_PC14MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for GPIO             */
#define SYS_GPC_MFPH_PC14MFP_USCI0_CTL0     (0x5UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for USCI0_CTL0       */
#define SYS_GPC_MFPH_PC14MFP_QSPI0_CLK      (0x6UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for QSPI0_CLK        */
#define SYS_GPC_MFPH_PC14MFP_UART0_TXD      (0x7UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for UART0_TXD        */
#define SYS_GPC_MFPH_PC14MFP_PWM0_CH5       (0xaUL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for PWM0_CH5         */
#define SYS_GPC_MFPH_PC14MFP_TM1            (0xdUL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for TM1              */
#define SYS_GPC_MFPH_PC14MFP_TM3_EXT        (0xfUL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for TM3_EXT          */

/* PD.0 MFP */
#define SYS_GPD_MFPL_PD0MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for GPIO              */
#define SYS_GPD_MFPL_PD0MFP_USCI0_CLK       (0x3UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for USCI0_CLK         */
#define SYS_GPD_MFPL_PD0MFP_CAN0_TXL        (0x7UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for CAN0_TXL          */
#define SYS_GPD_MFPL_PD0MFP_TM2             (0xeUL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for TM2               */

/* PD.1 MFP */
#define SYS_GPD_MFPL_PD1MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for GPIO              */
#define SYS_GPD_MFPL_PD1MFP_USCI0_DAT0      (0x3UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for USCI0_DAT0        */

/* PD.2 MFP */
#define SYS_GPD_MFPL_PD2MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for GPIO              */
#define SYS_GPD_MFPL_PD2MFP_USCI0_DAT1      (0x3UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for USCI0_DAT1        */
#define SYS_GPD_MFPL_PD2MFP_UART0_RXD       (0x9UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for UART0_RXD         */

/* PD.3 MFP */
#define SYS_GPD_MFPL_PD3MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for GPIO              */
#define SYS_GPD_MFPL_PD3MFP_USCI0_CTL1      (0x3UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for USCI0_CTL1        */
#define SYS_GPD_MFPL_PD3MFP_USCI1_CTL0      (0x7UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for USCI1_CTL0        */
#define SYS_GPD_MFPL_PD3MFP_UART0_TXD       (0x9UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for UART0_TXD         */

/* PD.15 MFP */
#define SYS_GPD_MFPH_PD15MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for GPIO             */
#define SYS_GPD_MFPH_PD15MFP_PWM0_CH5       (0xcUL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for PWM0_CH5         */
#define SYS_GPD_MFPH_PD15MFP_TM3            (0xeUL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for TM3              */
#define SYS_GPD_MFPH_PD15MFP_INT1           (0xfUL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for INT1             */

/* PE.15 MFP */
#define SYS_GPE_MFPH_PE15MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for GPIO             */
#define SYS_GPE_MFPH_PE15MFP_nRESET         (0x2UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for nRESET           */

/* PF.0 MFP */
#define SYS_GPF_MFPL_PF0MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for GPIO              */
#define SYS_GPF_MFPL_PF0MFP_UART1_TXD       (0x2UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for UART1_TXD         */
#define SYS_GPF_MFPL_PF0MFP_UART0_TXD       (0x4UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for UART0_TXD         */
#define SYS_GPF_MFPL_PF0MFP_I2C0_SDA        (0x5UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for I2C0_SDA          */
#define SYS_GPF_MFPL_PF0MFP_USCI0_CTL1      (0x7UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for USCI0_CTL1        */
#define SYS_GPF_MFPL_PF0MFP_UART2_TXD       (0x8UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for UART2_TXD         */
#define SYS_GPF_MFPL_PF0MFP_I2C0_SCL        (0x9UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for I2C0_SCL          */
#define SYS_GPF_MFPL_PF0MFP_PWM1_CH4        (0xbUL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for PWM1_CH4          */
#define SYS_GPF_MFPL_PF0MFP_ICE_DAT         (0xeUL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for ICE_DAT           */

/* PF.1 MFP */
#define SYS_GPF_MFPL_PF1MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for GPIO              */
#define SYS_GPF_MFPL_PF1MFP_UART1_RXD       (0x2UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for UART1_RXD         */
#define SYS_GPF_MFPL_PF1MFP_UART0_RXD       (0x4UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for UART0_RXD         */
#define SYS_GPF_MFPL_PF1MFP_I2C0_SCL        (0x5UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for I2C0_SCL          */
#define SYS_GPF_MFPL_PF1MFP_CAN0_TXL        (0x6UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for CAN0_TXL          */
#define SYS_GPF_MFPL_PF1MFP_USCI0_DAT1      (0x7UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for USCI0_DAT1        */
#define SYS_GPF_MFPL_PF1MFP_UART2_RXD       (0x8UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for UART2_RXD         */
#define SYS_GPF_MFPL_PF1MFP_I2C0_SDA        (0x9UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for I2C0_SDA          */
#define SYS_GPF_MFPL_PF1MFP_PWM1_CH5        (0xbUL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for PWM1_CH5          */
#define SYS_GPF_MFPL_PF1MFP_ICE_CLK         (0xeUL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for ICE_CLK           */
#define SYS_GPF_MFPL_PF1MFP_TM2_EXT         (0xfUL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for TM2_EXT           */

/* PF.2 MFP */
#define SYS_GPF_MFPL_PF2MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for GPIO              */
#define SYS_GPF_MFPL_PF2MFP_UART2_RXD       (0x2UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for UART2_RXD         */
#define SYS_GPF_MFPL_PF2MFP_UART0_RXD       (0x3UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for UART0_RXD         */
#define SYS_GPF_MFPL_PF2MFP_I2C0_SDA        (0x4UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for I2C0_SDA          */
#define SYS_GPF_MFPL_PF2MFP_QSPI0_CLK       (0x5UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for QSPI0_CLK         */
#define SYS_GPF_MFPL_PF2MFP_USCI1_CTL1      (0x8UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for USCI1_CTL1        */
#define SYS_GPF_MFPL_PF2MFP_PWM1_CH1        (0x9UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for PWM1_CH1          */
#define SYS_GPF_MFPL_PF2MFP_XT1_OUT         (0xaUL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for XT1_OUT           */
#define SYS_GPF_MFPL_PF2MFP_TM1_EXT         (0xfUL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for TM1_EXT           */

/* PF.3 MFP */
#define SYS_GPF_MFPL_PF3MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for GPIO              */
#define SYS_GPF_MFPL_PF3MFP_UART2_TXD       (0x2UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for UART2_TXD         */
#define SYS_GPF_MFPL_PF3MFP_UART0_TXD       (0x3UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for UART0_TXD         */
#define SYS_GPF_MFPL_PF3MFP_I2C0_SCL        (0x4UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for I2C0_SCL          */
#define SYS_GPF_MFPL_PF3MFP_PWM1_CH0        (0x9UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for PWM1_CH0          */
#define SYS_GPF_MFPL_PF3MFP_XT1_IN          (0xaUL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for XT1_IN            */
#define SYS_GPF_MFPL_PF3MFP_TM0_EXT         (0xfUL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for TM0_EXT           */

/* PF.4 MFP */
#define SYS_GPF_MFPL_PF4MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for GPIO              */
#define SYS_GPF_MFPL_PF4MFP_UART2_TXD       (0x3UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for UART2_TXD         */
#define SYS_GPF_MFPL_PF4MFP_UART2_nRTS      (0x4UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for UART2_nRTS        */
#define SYS_GPF_MFPL_PF4MFP_UART0_nRTS      (0x5UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for UART0_nRTS        */
#define SYS_GPF_MFPL_PF4MFP_PWM0_CH1        (0x7UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for PWM0_CH1          */

/* PF.5 MFP */
#define SYS_GPF_MFPL_PF5MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for GPIO              */
#define SYS_GPF_MFPL_PF5MFP_UART2_RXD       (0x3UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for UART2_RXD         */
#define SYS_GPF_MFPL_PF5MFP_UART2_nCTS      (0x4UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for UART2_nCTS        */
#define SYS_GPF_MFPL_PF5MFP_UART0_nCTS      (0x5UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for UART0_nCTS        */
#define SYS_GPF_MFPL_PF5MFP_PWM0_CH0        (0x7UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for PWM0_CH0          */
#define SYS_GPF_MFPL_PF5MFP_ADC0_ST         (0xbUL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for ADC0_ST           */

/* PF.6 MFP */
#define SYS_GPF_MFPL_PF6MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF6MFP_Pos)    /*!< GPF_MFPL PF6 setting for GPIO              */
#define SYS_GPF_MFPL_PF6MFP_TM1             (0xeUL<<SYS_GPF_MFPL_PF6MFP_Pos)    /*!< GPF_MFPL PF6 setting for TM1               */

/* PF.14 MFP */
#define SYS_GPF_MFPH_PF14MFP_GPIO           (0x0UL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for GPIO             */
#define SYS_GPF_MFPH_PF14MFP_PWM0_BRAKE0    (0xaUL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for PWM0_BRAKE0      */
#define SYS_GPF_MFPH_PF14MFP_PWM0_CH4       (0xcUL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for PWM0_CH4         */
#define SYS_GPF_MFPH_PF14MFP_CLKO           (0xdUL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for CLKO             */
#define SYS_GPF_MFPH_PF14MFP_TM3            (0xeUL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for TM3              */
#define SYS_GPF_MFPH_PF14MFP_INT5           (0xfUL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for INT5             */

/* PF.15 MFP */
#define SYS_GPF_MFPH_PF15MFP_GPIO           (0x0UL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for GPIO             */
#define SYS_GPF_MFPH_PF15MFP_PWM0_BRAKE0    (0xbUL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for PWM0_BRAKE0      */
#define SYS_GPF_MFPH_PF15MFP_PWM0_CH1       (0xcUL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for PWM0_CH1         */
#define SYS_GPF_MFPH_PF15MFP_TM2            (0xdUL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for TM2              */
#define SYS_GPF_MFPH_PF15MFP_CLKO           (0xeUL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for CLKO             */
#define SYS_GPF_MFPH_PF15MFP_INT4           (0xfUL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for INT4             */

/* PG.9 MFP */
#define SYS_GPG_MFPH_PG9MFP_GPIO            (0x0UL<<SYS_GPG_MFPH_PG9MFP_Pos)    /*!< GPG_MFPH PG9 setting for GPIO              */

/* PG.10 MFP */
#define SYS_GPG_MFPH_PG10MFP_GPIO           (0x0UL<<SYS_GPG_MFPH_PG10MFP_Pos)   /*!< GPG_MFPH PG10 setting for GPIO             */

/* PG.11 MFP */
#define SYS_GPG_MFPH_PG11MFP_GPIO           (0x0UL<<SYS_GPG_MFPH_PG11MFP_Pos)   /*!< GPG_MFPH PG11 setting for GPIO             */

/* PG.12 MFP */
#define SYS_GPG_MFPH_PG12MFP_GPIO           (0x0UL<<SYS_GPG_MFPH_PG12MFP_Pos)   /*!< GPG_MFPH PG12 setting for GPIO             */

#define SYS_TIMEOUT_ERR             (-1)    /*!< SYS timeout error value \hideinitializer */

/**@}*/ /* end of group SYS_EXPORTED_CONSTANTS */

extern int32_t g_SYS_i32ErrCode;

/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This macro clear Brown-out detector interrupt flag.
  */
__STATIC_INLINE int32_t SYS_CLEAR_BOD_INT_FLAG(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    SYS->BODCTL |= SYS_BODCTL_BODIF_Msk;

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

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
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
__STATIC_INLINE int32_t SYS_DISABLE_BOD(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk;

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
__STATIC_INLINE int32_t SYS_ENABLE_BOD(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    SYS->BODCTL |= SYS_BODCTL_BODEN_Msk;

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCTL & SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCTL & SYS_BODCTL_BODOUT_Msk)

/**
  * @brief      Disable Brown-out detector reset function
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This macro disable Brown-out detector reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
__STATIC_INLINE int32_t SYS_DISABLE_BOD_RST(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    SYS->BODCTL &= ~SYS_BODCTL_BODRSTEN_Msk;

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
__STATIC_INLINE int32_t SYS_ENABLE_BOD_RST(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk;

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

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
  *             - \ref SYS_BODCTL_BODVL_2_2V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_4_4V
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  */
__STATIC_INLINE int32_t SYS_SET_BOD_LEVEL(uint32_t u32Level)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level);

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSTS & SYS_RSTSTS_BODRF_Msk)

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)

/**
  * @brief      Get reset source is from LVR Reset
  * @param      None
  * @retval     0   Previous reset source is not from Low-Voltage-Reset
  * @retval     >=1 Previous reset source is from Low-Voltage-Reset
  * @details    This macro get previous reset source is from Low-Voltage-Reset.
  */
#define SYS_IS_LVR_RST()                (SYS->RSTSTS & SYS_RSTSTS_LVRF_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  */
#define SYS_IS_POR_RST()                (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSTS & SYS_RSTSTS_SYSRF_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)

/**
  * @brief      Disable Low-Voltage-Reset function
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This macro disable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
__STATIC_INLINE int32_t SYS_DISABLE_LVR(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    SYS->BODCTL &= ~SYS_BODCTL_LVREN_Msk;

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This macro enable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
__STATIC_INLINE int32_t SYS_ENABLE_LVR(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    SYS->BODCTL |= SYS_BODCTL_LVREN_Msk;

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             PORCTL0 disable digital logic POR.
  *             PORCTL1 disable analog part POR.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_POR() \
            (SYS->PORCTL0 = 0x5AA5); \
            (SYS->PORCTL1 = 0x5AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             PORCTL0 enable digital logic POR.
  *             PORCTL1 enable analog part POR.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_POR() \
            (SYS->PORCTL0 = 0); \
            (SYS->PORCTL1 = 0)

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
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) ((SYS->RSTSTS) = (u32RstSrc) )


/*---------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
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
int32_t  SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
int32_t  SYS_DisableBOD(void);
void     SYS_SetPowerLevel(uint32_t u32PowerLevel);
uint32_t SYS_SetPowerRegulator(uint32_t u32PowerRegulator);
void     SYS_SetSSRAMPowerMode(uint32_t u32SRAMSel, uint32_t u32PowerMode);
void     SYS_SetPSRAMPowerMode(uint32_t u32SRAMSel, uint32_t u32PowerMode);
void     SYS_SetVRef(uint32_t u32VRefCTL);


/**@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group SYS_Driver */

/**@}*/ /* end of group Standard_Driver */


#ifdef __cplusplus
}
#endif

#endif /* __SYS_H__ */
