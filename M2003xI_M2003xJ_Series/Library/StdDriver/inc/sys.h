/**************************************************************************//**
 * @file     sys.h
 * @version  V0.10
 * @brief    M2003J series System Manager (SYS) driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
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
/* SYS Timeout constant definitions.                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_TIMEOUT         SystemCoreClock     /*!< SYS timeout counter (1 second timeout) \hideinitializer */
#define SYS_TIMEOUT_ERR     (-1)                /*!< SYS timeout error value                \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* SYS Define Error Code                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_OK              ( 0L)               /*!< SYS operation OK */
#define SYS_ERR_FAIL        (-1L)               /*!< SYS operation failed */
#define SYS_ERR_TIMEOUT     (-2L)               /*!< SYS operation abort due to timeout error */

/*----------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                     */
/*----------------------------------------------------------------------------------------------------------*/
#define ADC0_RST    ((0x120UL<<20)|(uint32_t)SYS_ADCRST_ADC0RST_Pos)    /*!< ADC0 reset is one of the SYS_ResetModule parameter */
#define BPWM0_RST   ((0x124UL<<20)|(uint32_t)SYS_BPWMRST_BPWM0RST_Pos)  /*!< BPWM0 reset is one of the SYS_ResetModule parameter */
#define BPWM1_RST   ((0x124UL<<20)|(uint32_t)SYS_BPWMRST_BPWM1RST_Pos)  /*!< BPWM1 reset is one of the SYS_ResetModule parameter */
#define CRC0_RST    ((0x128UL<<20)|(uint32_t)SYS_CRCRST_CRC0RST_Pos)    /*!< CRC0 reset is one of the SYS_ResetModule parameter */
#define FMC0_RST    ((0x12CUL<<20)|(uint32_t)SYS_FMCRST_FMC0RST_Pos)    /*!< FMC0 reset is one of the SYS_ResetModule parameter */
#define DFMC0_RST   ((0x12CUL<<20)|(uint32_t)SYS_FMCRST_DFMC0RST_Pos)   /*!< DFMC0 reset is one of the SYS_ResetModule parameter */
#define GPIO0_RST   ((0x130UL<<20)|(uint32_t)SYS_GPIORST_GPIO0RST_Pos)  /*!< GPIO0 reset is one of the SYS_ResetModule parameter */
#define I2C0_RST    ((0x134UL<<20)|(uint32_t)SYS_I2CRST_I2C0RST_Pos)    /*!< I2C0 reset is one of the SYS_ResetModule parameter */
#define I2C1_RST    ((0x134UL<<20)|(uint32_t)SYS_I2CRST_I2C1RST_Pos)    /*!< I2C1 reset is one of the SYS_ResetModule parameter */
#define I2C2_RST    ((0x134UL<<20)|(uint32_t)SYS_I2CRST_I2C2RST_Pos)    /*!< I2C2 reset is one of the SYS_ResetModule parameter */
#define PDMA0_RST   ((0x138UL<<20)|(uint32_t)SYS_PDMARST_PDMA0RST_Pos)  /*!< PDMA0 reset is one of the SYS_ResetModule parameter */
#define RTC0_RST    ((0x13CUL<<20)|(uint32_t)SYS_RTCRST_RTC0RST_Pos)    /*!< RTC0 reset is one of the SYS_ResetModule parameter */
#define TMR0_RST    ((0x140UL<<20)|(uint32_t)SYS_TMRRST_TMR0RST_Pos)    /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST    ((0x140UL<<20)|(uint32_t)SYS_TMRRST_TMR1RST_Pos)    /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST    ((0x140UL<<20)|(uint32_t)SYS_TMRRST_TMR2RST_Pos)    /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST    ((0x140UL<<20)|(uint32_t)SYS_TMRRST_TMR3RST_Pos)    /*!< TMR3 reset is one of the SYS_ResetModule parameter */
#define TMR4_RST    ((0x140UL<<20)|(uint32_t)SYS_TMRRST_TMR4RST_Pos)    /*!< TMR4 reset is one of the SYS_ResetModule parameter */
#define TMR5_RST    ((0x140UL<<20)|(uint32_t)SYS_TMRRST_TMR5RST_Pos)    /*!< TMR5 reset is one of the SYS_ResetModule parameter */
#define TMR6_RST    ((0x140UL<<20)|(uint32_t)SYS_TMRRST_TMR6RST_Pos)    /*!< TMR6 reset is one of the SYS_ResetModule parameter */
#define TMR7_RST    ((0x140UL<<20)|(uint32_t)SYS_TMRRST_TMR7RST_Pos)    /*!< TMR7 reset is one of the SYS_ResetModule parameter */
#define TMR8_RST    ((0x140UL<<20)|(uint32_t)SYS_TMRRST_TMR8RST_Pos)    /*!< TMR8 reset is one of the SYS_ResetModule parameter */
#define UART0_RST   ((0x144UL<<20)|(uint32_t)SYS_UARTRST_UART0RST_Pos)  /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST   ((0x144UL<<20)|(uint32_t)SYS_UARTRST_UART1RST_Pos)  /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define UART2_RST   ((0x144UL<<20)|(uint32_t)SYS_UARTRST_UART2RST_Pos)  /*!< UART2 reset is one of the SYS_ResetModule parameter */
#define UART3_RST   ((0x144UL<<20)|(uint32_t)SYS_UARTRST_UART3RST_Pos)  /*!< UART3 reset is one of the SYS_ResetModule parameter */
#define UART4_RST   ((0x144UL<<20)|(uint32_t)SYS_UARTRST_UART4RST_Pos)  /*!< UART4 reset is one of the SYS_ResetModule parameter */
#define USCI0_RST   ((0x148UL<<20)|(uint32_t)SYS_USCIRST_USCI0RST_Pos)  /*!< USCI0 reset is one of the SYS_ResetModule parameter */
#define USCI1_RST   ((0x148UL<<20)|(uint32_t)SYS_USCIRST_USCI1RST_Pos)  /*!< USCI1 reset is one of the SYS_ResetModule parameter */
#define USCI2_RST   ((0x148UL<<20)|(uint32_t)SYS_USCIRST_USCI2RST_Pos)  /*!< USCI2 reset is one of the SYS_ResetModule parameter */
#define USCI3_RST   ((0x148UL<<20)|(uint32_t)SYS_USCIRST_USCI3RST_Pos)  /*!< USCI3 reset is one of the SYS_ResetModule parameter */
#define USCI4_RST   ((0x148UL<<20)|(uint32_t)SYS_USCIRST_USCI4RST_Pos)  /*!< USCI4 reset is one of the SYS_ResetModule parameter */
#define WWDT0_RST   ((0x14CUL<<20)|(uint32_t)SYS_WWDTRST_WWDT0RST_Pos)  /*!< WWDT0 reset is one of the SYS_ResetModule parameter */

/*----------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                    */
/*----------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_INTERRUPT_EN     (0UL<<SYS_BODCTL_BODRSTEN_Pos)  /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_BOD_RST_EN           (1UL<<SYS_BODCTL_BODRSTEN_Pos)  /*!< Brown-out Reset Enable */

#define SYS_BODCTL_BODVL_2_4V           (0UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.4V */
#define SYS_BODCTL_BODVL_2_7V           (1UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCTL_BODVL_3_7V           (2UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCTL_BODVL_4_4V           (3UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 4.4V */

/*----------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Wake-Up Selection constant definitions.                                              */
/*----------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BODWKEN_DISABLE      (0UL<<SYS_BODCTL_BODWKEN_Pos)    /*!< Brown Out Detector Wake-up Function Disable */
#define SYS_BODCTL_BODWKEN_RISE         (1UL<<SYS_BODCTL_BODWKEN_Pos)    /*!< Brown Out Detector Power Rise Wake-up Function Enable */
#define SYS_BODCTL_BODWKEN_DROP         (2UL<<SYS_BODCTL_BODWKEN_Pos)    /*!< Brown Out Detector Power Drop Wake-up Function Enable */
#define SYS_BODCTL_BODWKEN_BOTH         (3UL<<SYS_BODCTL_BODWKEN_Pos)    /*!< Brown Out Detector Power Rise and Drop Wake-up Function Enable*/

/*----------------------------------------------------------------------------------------------------------*/
/*  Brown-out Detector Output De-glitch Time constant definitions.                                          */
/*----------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BODDGSEL_LIRC        (0x0UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is sampled by LIRC clock.    \hideinitializer */
#define SYS_BODCTL_BODDGSEL_4HCLK       (0x1UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 4HCLK            \hideinitializer */
#define SYS_BODCTL_BODDGSEL_8HCLK       (0x2UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 8HCLK            \hideinitializer */
#define SYS_BODCTL_BODDGSEL_16HCLK      (0x3UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 16HCLK           \hideinitializer */
#define SYS_BODCTL_BODDGSEL_32HCLK      (0x4UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 32HCLK           \hideinitializer */
#define SYS_BODCTL_BODDGSEL_64HCLK      (0x5UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 64HCLK           \hideinitializer */
#define SYS_BODCTL_BODDGSEL_128HCLK     (0x6UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 128HCLK          \hideinitializer */
#define SYS_BODCTL_BODDGSEL_256HCLK     (0x7UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< BOD Output De-glitch Time is selected 256HCLK          \hideinitializer */

/*----------------------------------------------------------------------------------------------------------*/
/*  LVR Output De-glitch Time constant definitions.                                                         */
/*----------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_LVRDGSEL_0HCLK       (0x0UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time Without de-glitch function.  \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_4HCLK       (0x1UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 4HCLK            \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_8HCLK       (0x2UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 8HCLK            \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_16HCLK      (0x3UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 16HCLK           \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_32HCLK      (0x4UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 32HCLK           \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_64HCLK      (0x5UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 64HCLK           \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_128HCLK     (0x6UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 128HCLK          \hideinitializer */
#define SYS_BODCTL_LVRDGSEL_256HCLK     (0x7UL<<SYS_BODCTL_LVRDGSEL_Pos)    /*!< LVR Output De-glitch Time is selected 256HCLK          \hideinitializer */

/*----------------------------------------------------------------------------------------------------------*/
/*  SRAM ECC Control constant definitions.                                                                  */
/*----------------------------------------------------------------------------------------------------------*/
#define SYS_SRAM_ECC_ENABLE             (SYS_SRAM0ICTL_ECCEN_Msk)           /*!< SRAM ECC Enable bit                                \ideinitializer */
#define SYS_SARM_ECC_SB_IEN             (SYS_SRAM0ICTL_SBERRIEN_Msk)        /*!< SRAM ECC Single Bit Error Interrupt Enable bit     \ideinitializer */
#define SYS_SARM_ECC_MB_IEN             (SYS_SRAM0ICTL_MBERRIEN_Msk)        /*!< SRAM ECC Multiple Bit Error Interrupt Enable bit   \ideinitializer */

/*----------------------------------------------------------------------------------------------------------*/
/*  SRAM ECC Status constant definitions.                                                                   */
/*----------------------------------------------------------------------------------------------------------*/
#define SYS_SARM_ECC_SB_IF              (SYS_SRAM0STS_SBERRIF_Msk)          /*!< SRAM ECC Single Bit Error flag     \ideinitializer */
#define SYS_SARM_ECC_MB_IF              (SYS_SRAM0STS_MBERRIF_Msk)          /*!< SRAM ECC Multiple Bit Error flag   \ideinitializer */
#define SYS_SARM_ECC_B0_IF              (SYS_SRAM0STS_B0ECCEIF_Msk)         /*!< SRAM ECC Bye 0 Error flag          \ideinitializer */
#define SYS_SARM_ECC_B1_IF              (SYS_SRAM0STS_B1ECCEIF_Msk)         /*!< SRAM ECC Bye 1 Error flag          \ideinitializer */
#define SYS_SARM_ECC_B2_IF              (SYS_SRAM0STS_B2ECCEIF_Msk)         /*!< SRAM ECC Bye 2 Error flag          \ideinitializer */
#define SYS_SARM_ECC_B3_IF              (SYS_SRAM0STS_B3ECCEIF_Msk)         /*!< SRAM ECC Bye 3 Error flag          \ideinitializer */

/*----------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                    */
/*----------------------------------------------------------------------------------------------------------*/

/* How to use below #define?

Example: If user want to set PA.1 as UART0_TXD and PA.0 as UART0_RXD in initial function,
         user can issue following command to achieve it.

    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_UART0_TXD;
*/

/* PA.0 MFP */
#define SYS_GPA_MFPL_PA0MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for GPIO              */
#define SYS_GPA_MFPL_PA0MFP_I2C2_SDA        (0x4UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for I2C2_SDA          */
#define SYS_GPA_MFPL_PA0MFP_UART0_RXD       (0x7UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for UART0_RXD         */
#define SYS_GPA_MFPL_PA0MFP_UART1_nRTS      (0x8UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for UART1_nRTS        */
#define SYS_GPA_MFPL_PA0MFP_BPWM0_CH5       (0xCUL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for BPWM0_CH5         */

/* PA.1 MFP */
#define SYS_GPA_MFPL_PA1MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for GPIO              */
#define SYS_GPA_MFPL_PA1MFP_I2C2_SCL        (0x4UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for I2C2_SCL          */
#define SYS_GPA_MFPL_PA1MFP_UART0_TXD       (0x7UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for UART0_TXD         */
#define SYS_GPA_MFPL_PA1MFP_UART1_nCTS      (0x8UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for UART1_nCTS        */
#define SYS_GPA_MFPL_PA1MFP_BPWM0_CH4       (0xCUL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for BPWM0_CH4         */

/* PA.2 MFP */
#define SYS_GPA_MFPL_PA2MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for GPIO              */
#define SYS_GPA_MFPL_PA2MFP_I2C1_SDA        (0x4UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for I2C1_SDA          */
#define SYS_GPA_MFPL_PA2MFP_UART4_RXD       (0x7UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for UART4_RXD         */
#define SYS_GPA_MFPL_PA2MFP_UART1_RXD       (0x8UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for UART1_RXD         */
#define SYS_GPA_MFPL_PA2MFP_BPWM0_CH3       (0xCUL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for BPWM0_CH3         */

/* PA.3 MFP */
#define SYS_GPA_MFPL_PA3MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for GPIO              */
#define SYS_GPA_MFPL_PA3MFP_I2C1_SCL        (0x4UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for I2C1_SCL          */
#define SYS_GPA_MFPL_PA3MFP_UART4_TXD       (0x7UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for UART4_TXD         */
#define SYS_GPA_MFPL_PA3MFP_UART1_TXD       (0x8UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for UART1_TXD         */
#define SYS_GPA_MFPL_PA3MFP_BPWM0_CH2       (0xCUL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for BPWM0_CH2         */
#define SYS_GPA_MFPL_PA3MFP_CLKO            (0xEUL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for CLKO              */

/* PA.4 MFP */
#define SYS_GPA_MFPL_PA4MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for GPIO              */
#define SYS_GPA_MFPL_PA4MFP_UART0_nRTS      (0x7UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for UART0_nRTS        */
#define SYS_GPA_MFPL_PA4MFP_UART0_RXD       (0x8UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for UART0_RXD         */
#define SYS_GPA_MFPL_PA4MFP_I2C0_SDA        (0x9UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for I2C0_SDA          */
#define SYS_GPA_MFPL_PA4MFP_BPWM0_CH1       (0xCUL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for BPWM0_CH1         */

/* PA.5 MFP */
#define SYS_GPA_MFPL_PA5MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for GPIO              */
#define SYS_GPA_MFPL_PA5MFP_UART0_nCTS      (0x7UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for UART0_nCTS        */
#define SYS_GPA_MFPL_PA5MFP_UART0_TXD       (0x8UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for UART0_TXD         */
#define SYS_GPA_MFPL_PA5MFP_I2C0_SCL        (0x9UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for I2C0_SCL          */
#define SYS_GPA_MFPL_PA5MFP_BPWM0_CH0       (0xCUL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for BPWM0_CH0         */

/* PA.6 MFP */
#define SYS_GPA_MFPL_PA6MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for GPIO              */
#define SYS_GPA_MFPL_PA6MFP_I2C1_SDA        (0x4UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for I2C1_SDA          */
#define SYS_GPA_MFPL_PA6MFP_UART0_RXD       (0x7UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for UART0_RXD         */
#define SYS_GPA_MFPL_PA6MFP_USCI4_CTL0      (0x8UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for USCI4_CTL0        */
#define SYS_GPA_MFPL_PA6MFP_BPWM1_CH5       (0xBUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for BPWM1_CH5         */
#define SYS_GPA_MFPL_PA6MFP_TM3             (0xEUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for TM3               */
#define SYS_GPA_MFPL_PA6MFP_INT0            (0xFUL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for INT0              */

/* PA.7 MFP */
#define SYS_GPA_MFPL_PA7MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for GPIO              */
#define SYS_GPA_MFPL_PA7MFP_I2C1_SCL        (0x4UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for I2C1_SCL          */
#define SYS_GPA_MFPL_PA7MFP_UART0_TXD       (0x7UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for UART0_TXD         */
#define SYS_GPA_MFPL_PA7MFP_USCI4_CLK       (0x8UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for USCI4_CLK         */
#define SYS_GPA_MFPL_PA7MFP_BPWM1_CH4       (0xBUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for BPWM1_CH4         */
#define SYS_GPA_MFPL_PA7MFP_TM2             (0xEUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for TM2               */
#define SYS_GPA_MFPL_PA7MFP_INT1            (0xFUL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for INT1              */

/* PA.8 MFP */
#define SYS_GPA_MFPH_PA8MFP_GPIO            (0x0UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for GPIO              */
#define SYS_GPA_MFPH_PA8MFP_ADC0_CH19       (0x1UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for ADC0_CH19         */
#define SYS_GPA_MFPH_PA8MFP_USCI0_CTL1      (0x6UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for USCI0_CTL1        */
#define SYS_GPA_MFPH_PA8MFP_UART1_RXD       (0x7UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for UART1_RXD         */
#define SYS_GPA_MFPH_PA8MFP_TM3_EXT         (0xDUL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for TM3_EXT           */
#define SYS_GPA_MFPH_PA8MFP_INT4            (0xFUL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for INT4              */

/* PA.9 MFP */
#define SYS_GPA_MFPH_PA9MFP_GPIO            (0x0UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for GPIO              */
#define SYS_GPA_MFPH_PA9MFP_ADC0_CH21       (0x1UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for ADC0_CH21         */
#define SYS_GPA_MFPH_PA9MFP_USCI0_DAT1      (0x6UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for USCI0_DAT1        */
#define SYS_GPA_MFPH_PA9MFP_UART1_TXD       (0x7UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for UART1_TXD         */
#define SYS_GPA_MFPH_PA9MFP_TM2_EXT         (0xDUL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for TM2_EXT           */

/* PA.10 MFP */
#define SYS_GPA_MFPH_PA10MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for GPIO             */
#define SYS_GPA_MFPH_PA10MFP_ADC0_CH22      (0x1UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for ADC0_CH22        */
#define SYS_GPA_MFPH_PA10MFP_I2C2_SDA       (0x4UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for I2C2_SDA         */
#define SYS_GPA_MFPH_PA10MFP_USCI0_DAT0     (0x6UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for USCI0_DAT0       */
#define SYS_GPA_MFPH_PA10MFP_TM1_EXT        (0xDUL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for TM1_EXT          */

/* PA.11 MFP */
#define SYS_GPA_MFPH_PA11MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for GPIO             */
#define SYS_GPA_MFPH_PA11MFP_ADC0_CH23      (0x1UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for ADC0_CH23        */
#define SYS_GPA_MFPH_PA11MFP_I2C2_SCL       (0x4UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for I2C2_SCL         */
#define SYS_GPA_MFPH_PA11MFP_USCI0_CLK      (0x6UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for USCI0_CLK        */
#define SYS_GPA_MFPH_PA11MFP_TM0_EXT        (0xDUL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for TM0_EXT          */

/* PA.12 MFP */
#define SYS_GPA_MFPH_PA12MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for GPIO             */
#define SYS_GPA_MFPH_PA12MFP_UART4_TXD      (0x3UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for UART4_TXD        */
#define SYS_GPA_MFPH_PA12MFP_I2C1_SCL       (0x4UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for I2C1_SCL         */
#define SYS_GPA_MFPH_PA12MFP_USCI2_CTL0     (0x9UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for USCI2_CTL0       */

/* PA.13 MFP */
#define SYS_GPA_MFPH_PA13MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for GPIO             */
#define SYS_GPA_MFPH_PA13MFP_UART4_RXD      (0x3UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for UART4_RXD        */
#define SYS_GPA_MFPH_PA13MFP_I2C1_SDA       (0x4UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for I2C1_SDA         */
#define SYS_GPA_MFPH_PA13MFP_USCI2_CTL1     (0x9UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for USCI2_CTL1       */

/* PA.14 MFP */
#define SYS_GPA_MFPH_PA14MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for GPIO             */
#define SYS_GPA_MFPH_PA14MFP_UART0_TXD      (0x3UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for UART0_TXD        */
#define SYS_GPA_MFPH_PA14MFP_I2C2_SCL       (0x4UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for I2C2_SCL         */
#define SYS_GPA_MFPH_PA14MFP_USCI2_DAT1     (0x9UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for USCI2_DAT1       */

/* PA.15 MFP */
#define SYS_GPA_MFPH_PA15MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for GPIO             */
#define SYS_GPA_MFPH_PA15MFP_UART0_RXD      (0x3UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for UART0_RXD        */
#define SYS_GPA_MFPH_PA15MFP_I2C2_SDA       (0x4UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for I2C2_SDA         */
#define SYS_GPA_MFPH_PA15MFP_USCI2_DAT0     (0x9UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for USCI2_DAT0       */

/* PB.0 MFP */
#define SYS_GPB_MFPL_PB0MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for GPIO              */
#define SYS_GPB_MFPL_PB0MFP_ADC0_CH0        (0x1UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for ADC0_CH0          */
#define SYS_GPB_MFPL_PB0MFP_ADC0_ST         (0x2UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for ADC0_ST           */
#define SYS_GPB_MFPL_PB0MFP_UART2_RXD       (0x3UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for UART2_RXD         */
#define SYS_GPB_MFPL_PB0MFP_BPWM1_CH5       (0x4UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for BPWM1_CH5         */
#define SYS_GPB_MFPL_PB0MFP_USCI0_CTL0      (0x6UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for USCI0_CTL0        */
#define SYS_GPB_MFPL_PB0MFP_BPWM0_CH5       (0x8UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for BPWM0_CH5         */
//#define SYS_GPB_MFPL_PB0MFP_USCI0_CTL0      (0x9UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for USCI0_CTL0        */
#define SYS_GPB_MFPL_PB0MFP_I2C1_SDA        (0xAUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for I2C1_SDA          */
#define SYS_GPB_MFPL_PB0MFP_BPWM0_CH3       (0xCUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for BPWM0_CH3         */
#define SYS_GPB_MFPL_PB0MFP_TM8             (0xEUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for TM8               */
#define SYS_GPB_MFPL_PB0MFP_TM3_EXT         (0xFUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for TM3_EXT           */

/* PB.1 MFP */
#define SYS_GPB_MFPL_PB1MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for GPIO              */
#define SYS_GPB_MFPL_PB1MFP_ADC0_CH1        (0x1UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for ADC0_CH1          */
#define SYS_GPB_MFPL_PB1MFP_BPWM0_CH2       (0x2UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for BPWM0_CH2         */
#define SYS_GPB_MFPL_PB1MFP_UART2_TXD       (0x3UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for UART2_TXD         */
#define SYS_GPB_MFPL_PB1MFP_BPWM1_CH4       (0x4UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for BPWM1_CH4         */
#define SYS_GPB_MFPL_PB1MFP_USCI3_CTL1      (0x6UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for USCI3_CTL1        */
#define SYS_GPB_MFPL_PB1MFP_USCI1_CLK       (0x8UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for USCI1_CLK         */
#define SYS_GPB_MFPL_PB1MFP_BPWM0_CH4       (0x9UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for BPWM0_CH4         */
#define SYS_GPB_MFPL_PB1MFP_I2C1_SCL        (0xAUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for I2C1_SCL          */
#define SYS_GPB_MFPL_PB1MFP_TM0             (0xEUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for TM0               */
#define SYS_GPB_MFPL_PB1MFP_TM0_EXT         (0xFUL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for TM0_EXT           */

/* PB.2 MFP */
#define SYS_GPB_MFPL_PB2MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for GPIO              */
#define SYS_GPB_MFPL_PB2MFP_ADC0_CH2        (0x1UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for ADC0_CH2          */
#define SYS_GPB_MFPL_PB2MFP_I2C1_SDA        (0x4UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for I2C1_SDA          */
#define SYS_GPB_MFPL_PB2MFP_UART0_TXD       (0x5UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for UART0_TXD         */
#define SYS_GPB_MFPL_PB2MFP_UART1_RXD       (0x6UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for UART1_RXD         */
#define SYS_GPB_MFPL_PB2MFP_USCI1_DAT0      (0x8UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for USCI1_DAT0        */
#define SYS_GPB_MFPL_PB2MFP_BPWM0_CH3       (0xAUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for BPWM0_CH3         */
#define SYS_GPB_MFPL_PB2MFP_TM3             (0xEUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for TM3               */
#define SYS_GPB_MFPL_PB2MFP_INT3            (0xFUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for INT3              */

/* PB.3 MFP */
#define SYS_GPB_MFPL_PB3MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for GPIO              */
#define SYS_GPB_MFPL_PB3MFP_ADC0_CH3        (0x1UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for ADC0_CH3          */
#define SYS_GPB_MFPL_PB3MFP_I2C1_SCL        (0x4UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for I2C1_SCL          */
#define SYS_GPB_MFPL_PB3MFP_UART0_RXD       (0x5UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for UART0_RXD         */
#define SYS_GPB_MFPL_PB3MFP_UART1_TXD       (0x6UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for UART1_TXD         */
#define SYS_GPB_MFPL_PB3MFP_USCI1_DAT1      (0x8UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for USCI1_DAT1        */
#define SYS_GPB_MFPL_PB3MFP_BPWM0_CH2       (0xAUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for BPWM0_CH2         */
#define SYS_GPB_MFPL_PB3MFP_TM2             (0xEUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for TM2               */
#define SYS_GPB_MFPL_PB3MFP_INT2            (0xFUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for INT2              */

/* PB.4 MFP */
#define SYS_GPB_MFPL_PB4MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for GPIO              */
#define SYS_GPB_MFPL_PB4MFP_ADC0_CH4        (0x1UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for ADC0_CH4          */
#define SYS_GPB_MFPL_PB4MFP_I2C0_SDA        (0x6UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for I2C0_SDA          */
#define SYS_GPB_MFPL_PB4MFP_USCI1_CTL1      (0x8UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for USCI1_CTL1        */
#define SYS_GPB_MFPL_PB4MFP_BPWM0_CH1       (0xAUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for BPWM0_CH1         */
#define SYS_GPB_MFPL_PB4MFP_UART2_RXD       (0xDUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for UART2_RXD         */
#define SYS_GPB_MFPL_PB4MFP_TM1             (0xEUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for TM1               */
#define SYS_GPB_MFPL_PB4MFP_INT1            (0xFUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for INT1              */

/* PB.5 MFP */
#define SYS_GPB_MFPL_PB5MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for GPIO              */
#define SYS_GPB_MFPL_PB5MFP_ADC0_CH5        (0x1UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for ADC0_CH5          */
#define SYS_GPB_MFPL_PB5MFP_I2C0_SCL        (0x6UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for I2C0_SCL          */
#define SYS_GPB_MFPL_PB5MFP_USCI1_CTL0      (0x8UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for USCI1_CTL0        */
#define SYS_GPB_MFPL_PB5MFP_BPWM0_CH0       (0xAUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for BPWM0_CH0         */
#define SYS_GPB_MFPL_PB5MFP_UART2_TXD       (0xDUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for UART2_TXD         */
#define SYS_GPB_MFPL_PB5MFP_TM0             (0xEUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for TM0               */
#define SYS_GPB_MFPL_PB5MFP_INT0            (0xFUL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for INT0              */

/* PB.6 MFP */
#define SYS_GPB_MFPL_PB6MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for GPIO              */
#define SYS_GPB_MFPL_PB6MFP_ADC0_CH6        (0x1UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for ADC0_CH6          */
#define SYS_GPB_MFPL_PB6MFP_USCI1_DAT1      (0x4UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for USCI1_DAT1        */
#define SYS_GPB_MFPL_PB6MFP_UART1_RXD       (0x6UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for UART1_RXD         */
#define SYS_GPB_MFPL_PB6MFP_BPWM1_CH5       (0x8UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for BPWM1_CH5         */
#define SYS_GPB_MFPL_PB6MFP_INT4            (0xDUL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for INT4              */

/* PB.7 MFP */
#define SYS_GPB_MFPL_PB7MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for GPIO              */
#define SYS_GPB_MFPL_PB7MFP_ADC0_CH7        (0x1UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for ADC0_CH7          */
#define SYS_GPB_MFPL_PB7MFP_USCI0_CLK       (0x2UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for USCI0_CLK         */
#define SYS_GPB_MFPL_PB7MFP_USCI1_DAT0      (0x4UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for USCI1_DAT0        */
#define SYS_GPB_MFPL_PB7MFP_UART1_TXD       (0x6UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for UART1_TXD         */
#define SYS_GPB_MFPL_PB7MFP_BPWM1_CH4       (0x8UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for BPWM1_CH4         */
#define SYS_GPB_MFPL_PB7MFP_TM2_EXT         (0x9UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for TM2_EXT           */
#define SYS_GPB_MFPL_PB7MFP_BPWM0_CH2       (0xAUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for BPWM0_CH2         */
#define SYS_GPB_MFPL_PB7MFP_INT5            (0xDUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for INT5              */
#define SYS_GPB_MFPL_PB7MFP_USCI3_CTL0      (0xFUL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for USCI3_CTL0        */

/* PB.8 MFP */
#define SYS_GPB_MFPH_PB8MFP_GPIO            (0x0UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for GPIO              */
#define SYS_GPB_MFPH_PB8MFP_ADC0_CH8        (0x1UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for ADC0_CH8          */
#define SYS_GPB_MFPH_PB8MFP_USCI0_DAT0      (0x2UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for USCI0_DAT0        */
#define SYS_GPB_MFPH_PB8MFP_USCI1_CLK       (0x4UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for USCI1_CLK         */
#define SYS_GPB_MFPH_PB8MFP_UART0_RXD       (0x5UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for UART0_RXD         */
#define SYS_GPB_MFPH_PB8MFP_UART1_nRTS      (0x6UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for UART1_nRTS        */
#define SYS_GPB_MFPH_PB8MFP_I2C0_SDA        (0x8UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for I2C0_SDA          */
#define SYS_GPB_MFPH_PB8MFP_TM3_EXT         (0x9UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for TM3_EXT           */
#define SYS_GPB_MFPH_PB8MFP_BPWM0_CH3       (0xAUL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for BPWM0_CH3         */
#define SYS_GPB_MFPH_PB8MFP_TM1             (0xEUL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for TM1               */
#define SYS_GPB_MFPH_PB8MFP_USCI3_CLK       (0xFUL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for USCI3_CLK         */

/* PB.9 MFP */
#define SYS_GPB_MFPH_PB9MFP_GPIO            (0x0UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for GPIO              */
#define SYS_GPB_MFPH_PB9MFP_ADC0_CH9        (0x1UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for ADC0_CH9          */
#define SYS_GPB_MFPH_PB9MFP_USCI0_DAT1      (0x2UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for USCI0_DAT1        */
#define SYS_GPB_MFPH_PB9MFP_USCI1_CTL1      (0x4UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for USCI1_CTL1        */
#define SYS_GPB_MFPH_PB9MFP_UART0_TXD       (0x5UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for UART0_TXD         */
#define SYS_GPB_MFPH_PB9MFP_UART1_nCTS      (0x6UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for UART1_nCTS        */
#define SYS_GPB_MFPH_PB9MFP_I2C0_SCL        (0x8UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for I2C0_SCL          */
#define SYS_GPB_MFPH_PB9MFP_TM1_EXT         (0x9UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for TM1_EXT           */
#define SYS_GPB_MFPH_PB9MFP_BPWM0_CH4       (0xAUL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for BPWM0_CH4         */
#define SYS_GPB_MFPH_PB9MFP_USCI3_DAT1      (0xFUL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for USCI3_DAT1        */

/* PB.10 MFP */
#define SYS_GPB_MFPH_PB10MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for GPIO             */
#define SYS_GPB_MFPH_PB10MFP_ADC0_CH10      (0x1UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for ADC0_CH10        */
#define SYS_GPB_MFPH_PB10MFP_UART4_RXD      (0x3UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for UART4_RXD        */
#define SYS_GPB_MFPH_PB10MFP_USCI1_CTL0     (0x4UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for USCI1_CTL0       */
#define SYS_GPB_MFPH_PB10MFP_UART0_nRTS     (0x5UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for UART0_nRTS       */
#define SYS_GPB_MFPH_PB10MFP_I2C1_SDA       (0x7UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for I2C1_SDA         */
#define SYS_GPB_MFPH_PB10MFP_USCI3_DAT0     (0xFUL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for USCI3_DAT0       */

/* PB.11 MFP */
#define SYS_GPB_MFPH_PB11MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for GPIO             */
#define SYS_GPB_MFPH_PB11MFP_ADC0_CH11      (0x1UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for ADC0_CH11        */
#define SYS_GPB_MFPH_PB11MFP_USCI0_CTL0     (0x2UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for USCI0_CTL0       */
#define SYS_GPB_MFPH_PB11MFP_UART4_TXD      (0x3UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for UART4_TXD        */
#define SYS_GPB_MFPH_PB11MFP_UART0_nCTS     (0x5UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for UART0_nCTS       */
#define SYS_GPB_MFPH_PB11MFP_I2C1_SCL       (0x7UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for I2C1_SCL         */
#define SYS_GPB_MFPH_PB11MFP_TM2_EXT        (0x9UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for TM2_EXT          */
#define SYS_GPB_MFPH_PB11MFP_BPWM0_CH5      (0xAUL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for BPWM0_CH5        */
#define SYS_GPB_MFPH_PB11MFP_USCI3_CTL1     (0xFUL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for USCI3_CTL1       */

/* PB.12 MFP */
#define SYS_GPB_MFPH_PB12MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for GPIO             */
#define SYS_GPB_MFPH_PB12MFP_ADC0_CH12      (0x1UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for ADC0_CH12        */
#define SYS_GPB_MFPH_PB12MFP_BPWM0_CH1      (0x3UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for BPWM0_CH1        */
#define SYS_GPB_MFPH_PB12MFP_UART3_nCTS     (0x4UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for UART3_nCTS       */
#define SYS_GPB_MFPH_PB12MFP_USCI0_CLK      (0x5UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for USCI0_CLK        */
#define SYS_GPB_MFPH_PB12MFP_UART0_RXD      (0x6UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for UART0_RXD        */
#define SYS_GPB_MFPH_PB12MFP_BPWM1_CH3      (0x8UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for BPWM1_CH3        */
#define SYS_GPB_MFPH_PB12MFP_I2C2_SDA       (0xBUL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for I2C2_SDA         */
#define SYS_GPB_MFPH_PB12MFP_TM3_EXT        (0xDUL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for TM3_EXT          */
#define SYS_GPB_MFPH_PB12MFP_CLKO           (0xEUL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for CLKO             */

/* PB.13 MFP */
#define SYS_GPB_MFPH_PB13MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for GPIO             */
#define SYS_GPB_MFPH_PB13MFP_ADC0_CH13      (0x1UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for ADC0_CH13        */
#define SYS_GPB_MFPH_PB13MFP_BPWM0_CH0      (0x3UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for BPWM0_CH0        */
#define SYS_GPB_MFPH_PB13MFP_UART3_nRTS     (0x4UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for UART3_nRTS       */
#define SYS_GPB_MFPH_PB13MFP_USCI0_DAT0     (0x5UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for USCI0_DAT0       */
#define SYS_GPB_MFPH_PB13MFP_UART0_TXD      (0x6UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for UART0_TXD        */
#define SYS_GPB_MFPH_PB13MFP_UART0_nRTS     (0x7UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for UART0_nRTS       */
#define SYS_GPB_MFPH_PB13MFP_BPWM1_CH2      (0x8UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for BPWM1_CH2        */
#define SYS_GPB_MFPH_PB13MFP_CLKO           (0x9UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for CLKO             */
#define SYS_GPB_MFPH_PB13MFP_I2C2_SCL       (0xBUL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for I2C2_SCL         */
#define SYS_GPB_MFPH_PB13MFP_USCI0_CTL0     (0xCUL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for USCI0_CTL0       */
#define SYS_GPB_MFPH_PB13MFP_TM2_EXT        (0xDUL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for TM2_EXT          */

/* PB.14 MFP */
#define SYS_GPB_MFPH_PB14MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for GPIO             */
#define SYS_GPB_MFPH_PB14MFP_ADC0_CH14      (0x1UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for ADC0_CH14        */
#define SYS_GPB_MFPH_PB14MFP_ADC0_ST        (0x2UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for ADC0_ST          */
#define SYS_GPB_MFPH_PB14MFP_I2C0_SCL       (0x3UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for I2C0_SCL         */
#define SYS_GPB_MFPH_PB14MFP_UART3_RXD      (0x4UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for UART3_RXD        */
#define SYS_GPB_MFPH_PB14MFP_USCI0_DAT1     (0x5UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for USCI0_DAT1       */
#define SYS_GPB_MFPH_PB14MFP_UART0_nRTS     (0x6UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for UART0_nRTS       */
#define SYS_GPB_MFPH_PB14MFP_UART0_RXD      (0x7UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for UART0_RXD        */
#define SYS_GPB_MFPH_PB14MFP_BPWM1_CH1      (0x8UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for BPWM1_CH1        */
#define SYS_GPB_MFPH_PB14MFP_TM1_EXT        (0xDUL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for TM1_EXT          */
#define SYS_GPB_MFPH_PB14MFP_CLKO           (0xEUL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for CLKO             */

/* PB.15 MFP */
#define SYS_GPB_MFPH_PB15MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for GPIO             */
#define SYS_GPB_MFPH_PB15MFP_ADC0_CH15      (0x1UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for ADC0_CH15        */
#define SYS_GPB_MFPH_PB15MFP_I2C0_SDA       (0x3UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for I2C0_SDA         */
#define SYS_GPB_MFPH_PB15MFP_UART3_TXD      (0x4UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for UART3_TXD        */
#define SYS_GPB_MFPH_PB15MFP_USCI0_CTL1     (0x5UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for USCI0_CTL1       */
#define SYS_GPB_MFPH_PB15MFP_UART0_nCTS     (0x6UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for UART0_nCTS       */
#define SYS_GPB_MFPH_PB15MFP_UART0_TXD      (0x7UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for UART0_TXD        */
#define SYS_GPB_MFPH_PB15MFP_BPWM1_CH0      (0x8UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for BPWM1_CH0        */
#define SYS_GPB_MFPH_PB15MFP_USCI2_CLK      (0x9UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for USCI2_CLK        */
#define SYS_GPB_MFPH_PB15MFP_TM1            (0xBUL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for TM1              */
#define SYS_GPB_MFPH_PB15MFP_BPWM0_CH1      (0xCUL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for BPWM0_CH1        */
#define SYS_GPB_MFPH_PB15MFP_TM0_EXT        (0xDUL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for TM0_EXT          */

/* PC.0 MFP */
#define SYS_GPC_MFPL_PC0MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for GPIO              */
#define SYS_GPC_MFPL_PC0MFP_USCI1_CTL0      (0x6UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for USCI1_CTL0        */
#define SYS_GPC_MFPL_PC0MFP_UART2_RXD       (0x7UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for UART2_RXD         */
#define SYS_GPC_MFPL_PC0MFP_I2C0_SDA        (0x9UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for I2C0_SDA          */
#define SYS_GPC_MFPL_PC0MFP_BPWM1_CH5       (0xCUL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for BPWM1_CH5         */

/* PC.1 MFP */
#define SYS_GPC_MFPL_PC1MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for GPIO              */
#define SYS_GPC_MFPL_PC1MFP_USCI1_CTL1      (0x6UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for USCI1_CTL1        */
#define SYS_GPC_MFPL_PC1MFP_UART2_TXD       (0x7UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for UART2_TXD         */
#define SYS_GPC_MFPL_PC1MFP_I2C0_SCL        (0x9UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for I2C0_SCL          */
#define SYS_GPC_MFPL_PC1MFP_ADC0_ST         (0xBUL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for ADC0_ST           */
#define SYS_GPC_MFPL_PC1MFP_BPWM1_CH4       (0xCUL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for BPWM1_CH4         */

/* PC.2 MFP */
#define SYS_GPC_MFPL_PC2MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for GPIO              */
#define SYS_GPC_MFPL_PC2MFP_UART3_RXD       (0x2UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for UART3_RXD         */
#define SYS_GPC_MFPL_PC2MFP_USCI1_DAT0      (0x6UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for USCI1_DAT0        */
#define SYS_GPC_MFPL_PC2MFP_UART2_nCTS      (0x7UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for UART2_nCTS        */
#define SYS_GPC_MFPL_PC2MFP_BPWM1_CH3       (0xCUL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for BPWM1_CH3         */

/* PC.3 MFP */
#define SYS_GPC_MFPL_PC3MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for GPIO              */
#define SYS_GPC_MFPL_PC3MFP_UART3_TXD       (0x2UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for UART3_TXD         */
#define SYS_GPC_MFPL_PC3MFP_USCI1_DAT1      (0x6UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for USCI1_DAT1        */
#define SYS_GPC_MFPL_PC3MFP_UART2_nRTS      (0x7UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for UART2_nRTS        */
#define SYS_GPC_MFPL_PC3MFP_BPWM1_CH2       (0xCUL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for BPWM1_CH2         */

/* PC.4 MFP */
#define SYS_GPC_MFPL_PC4MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for GPIO              */
#define SYS_GPC_MFPL_PC4MFP_UART4_RXD       (0x2UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for UART4_RXD         */
#define SYS_GPC_MFPL_PC4MFP_I2C1_SDA        (0x5UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for I2C1_SDA          */
#define SYS_GPC_MFPL_PC4MFP_USCI1_CLK       (0x6UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for USCI1_CLK         */
#define SYS_GPC_MFPL_PC4MFP_UART2_RXD       (0x7UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for UART2_RXD         */
#define SYS_GPC_MFPL_PC4MFP_BPWM1_CH1       (0xCUL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for BPWM1_CH1         */

/* PC.5 MFP */
#define SYS_GPC_MFPL_PC5MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for GPIO              */
#define SYS_GPC_MFPL_PC5MFP_UART4_TXD       (0x2UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for UART4_TXD         */
#define SYS_GPC_MFPL_PC5MFP_I2C1_SCL        (0x5UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for I2C1_SCL          */
#define SYS_GPC_MFPL_PC5MFP_UART2_TXD       (0x7UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for UART2_TXD         */
#define SYS_GPC_MFPL_PC5MFP_BPWM1_CH0       (0xCUL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for BPWM1_CH0         */

/* PC.6 MFP */
#define SYS_GPC_MFPL_PC6MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for GPIO              */
#define SYS_GPC_MFPL_PC6MFP_UART4_RXD       (0x5UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for UART4_RXD         */
#define SYS_GPC_MFPL_PC6MFP_UART0_nRTS      (0x7UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for UART0_nRTS        */
#define SYS_GPC_MFPL_PC6MFP_USCI4_DAT1      (0x8UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for USCI4_DAT1        */
#define SYS_GPC_MFPL_PC6MFP_BPWM1_CH3       (0xBUL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for BPWM1_CH3         */
#define SYS_GPC_MFPL_PC6MFP_TM1             (0xEUL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for TM1               */
#define SYS_GPC_MFPL_PC6MFP_INT2            (0xFUL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for INT2              */

/* PC.7 MFP */
#define SYS_GPC_MFPL_PC7MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for GPIO              */
#define SYS_GPC_MFPL_PC7MFP_UART4_TXD       (0x5UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for UART4_TXD         */
#define SYS_GPC_MFPL_PC7MFP_UART0_nCTS      (0x7UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for UART0_nCTS        */
#define SYS_GPC_MFPL_PC7MFP_USCI4_DAT0      (0x8UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for USCI4_DAT0        */
#define SYS_GPC_MFPL_PC7MFP_BPWM1_CH2       (0xBUL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for BPWM1_CH2         */
#define SYS_GPC_MFPL_PC7MFP_TM0             (0xEUL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for TM0               */
#define SYS_GPC_MFPL_PC7MFP_INT3            (0xFUL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for INT3              */

/* PC.8 MFP */
#define SYS_GPC_MFPH_PC8MFP_GPIO            (0x0UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for GPIO              */
#define SYS_GPC_MFPH_PC8MFP_UART1_RXD       (0x2UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for UART1_RXD         */
#define SYS_GPC_MFPH_PC8MFP_I2C0_SDA        (0x4UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for I2C0_SDA          */
#define SYS_GPC_MFPH_PC8MFP_UART4_nCTS      (0x5UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for UART4_nCTS        */
#define SYS_GPC_MFPH_PC8MFP_USCI4_CTL1      (0x8UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for USCI4_CTL1        */

/* PC.9 MFP */
#define SYS_GPC_MFPH_PC9MFP_GPIO            (0x0UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for GPIO              */
#define SYS_GPC_MFPH_PC9MFP_ADC0_CH20       (0x1UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for ADC0_CH20         */
#define SYS_GPC_MFPH_PC9MFP_UART3_RXD       (0x3UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for UART3_RXD         */
#define SYS_GPC_MFPH_PC9MFP_USCI3_CTL0      (0x6UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for USCI3_CTL0        */
#define SYS_GPC_MFPH_PC9MFP_TM7             (0xEUL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for TM7               */

/* PC.10 MFP */
#define SYS_GPC_MFPH_PC10MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for GPIO             */
#define SYS_GPC_MFPH_PC10MFP_UART3_TXD      (0x3UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for UART3_TXD        */
#define SYS_GPC_MFPH_PC10MFP_USCI3_CLK      (0x6UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for USCI3_CLK        */
#define SYS_GPC_MFPH_PC10MFP_TM6            (0xEUL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for TM6              */

/* PC.11 MFP */
#define SYS_GPC_MFPH_PC11MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for GPIO             */
#define SYS_GPC_MFPH_PC11MFP_UART0_RXD      (0x3UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for UART0_RXD        */
#define SYS_GPC_MFPH_PC11MFP_I2C0_SDA       (0x4UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for I2C0_SDA         */
#define SYS_GPC_MFPH_PC11MFP_UART0_TXD      (0x5UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for UART0_TXD        */
#define SYS_GPC_MFPH_PC11MFP_USCI3_DAT1     (0x6UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for USCI3_DAT1       */
#define SYS_GPC_MFPH_PC11MFP_TM5            (0xEUL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for TM5              */

/* PC.12 MFP */
#define SYS_GPC_MFPH_PC12MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for GPIO             */
#define SYS_GPC_MFPH_PC12MFP_UART0_TXD      (0x3UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for UART0_TXD        */
#define SYS_GPC_MFPH_PC12MFP_I2C0_SCL       (0x4UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for I2C0_SCL         */
#define SYS_GPC_MFPH_PC12MFP_UART0_RXD      (0x5UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for UART0_RXD        */
#define SYS_GPC_MFPH_PC12MFP_USCI3_DAT0     (0x6UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for USCI3_DAT0       */
#define SYS_GPC_MFPH_PC12MFP_TM4            (0xEUL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for TM4              */

/* PC.14 MFP */
#define SYS_GPC_MFPH_PC14MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for GPIO             */
#define SYS_GPC_MFPH_PC14MFP_USCI0_CTL0     (0x5UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for USCI0_CTL0       */
#define SYS_GPC_MFPH_PC14MFP_UART0_TXD      (0x7UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for UART0_TXD        */
#define SYS_GPC_MFPH_PC14MFP_BPWM0_CH5      (0xCUL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for BPWM0_CH5        */
#define SYS_GPC_MFPH_PC14MFP_TM1            (0xDUL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for TM1              */
#define SYS_GPC_MFPH_PC14MFP_TM3_EXT        (0xFUL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for TM3_EXT          */

/* PD.0 MFP */
#define SYS_GPD_MFPL_PD0MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for GPIO              */
#define SYS_GPD_MFPL_PD0MFP_UART3_RXD       (0x2UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for UART3_RXD         */
#define SYS_GPD_MFPL_PD0MFP_USCI0_CLK       (0x3UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for USCI0_CLK         */
#define SYS_GPD_MFPL_PD0MFP_I2C2_SDA        (0x4UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for I2C2_SDA          */
#define SYS_GPD_MFPL_PD0MFP_TM2             (0xEUL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for TM2               */

/* PD.1 MFP */
#define SYS_GPD_MFPL_PD1MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for GPIO              */
#define SYS_GPD_MFPL_PD1MFP_UART3_TXD       (0x2UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for UART3_TXD         */
#define SYS_GPD_MFPL_PD1MFP_USCI0_DAT0      (0x3UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for USCI0_DAT0        */
#define SYS_GPD_MFPL_PD1MFP_I2C2_SCL        (0x4UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for I2C2_SCL          */

/* PD.2 MFP */
#define SYS_GPD_MFPL_PD2MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for GPIO              */
#define SYS_GPD_MFPL_PD2MFP_UART3_nCTS      (0x2UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for UART3_nCTS        */
#define SYS_GPD_MFPL_PD2MFP_USCI0_DAT1      (0x3UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for USCI0_DAT1        */
#define SYS_GPD_MFPL_PD2MFP_UART0_RXD       (0x9UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for UART0_RXD         */

/* PD.3 MFP */
#define SYS_GPD_MFPL_PD3MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for GPIO              */
#define SYS_GPD_MFPL_PD3MFP_UART3_nRTS      (0x2UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for UART3_nRTS        */
#define SYS_GPD_MFPL_PD3MFP_USCI0_CTL1      (0x3UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for USCI0_CTL1        */
#define SYS_GPD_MFPL_PD3MFP_USCI1_CTL0      (0x7UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for USCI1_CTL0        */
#define SYS_GPD_MFPL_PD3MFP_UART0_TXD       (0x9UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for UART0_TXD         */

/* PD.4 MFP */
#define SYS_GPD_MFPL_PD4MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for GPIO              */
#define SYS_GPD_MFPL_PD4MFP_USCI0_CTL0      (0x3UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for USCI0_CTL0        */
#define SYS_GPD_MFPL_PD4MFP_I2C1_SDA        (0x4UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for I2C1_SDA          */
#define SYS_GPD_MFPL_PD4MFP_USCI1_CTL1      (0x7UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for USCI1_CTL1        */

/* PD.5 MFP */
#define SYS_GPD_MFPL_PD5MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for GPIO              */
#define SYS_GPD_MFPL_PD5MFP_I2C1_SCL        (0x4UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for I2C1_SCL          */
#define SYS_GPD_MFPL_PD5MFP_USCI1_DAT0      (0x7UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for USCI1_DAT0        */
#define SYS_GPD_MFPL_PD5MFP_ADC0_ST         (0xBUL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for ADC0_ST           */

/* PD.6 MFP */
#define SYS_GPD_MFPL_PD6MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for GPIO              */
#define SYS_GPD_MFPL_PD6MFP_UART1_RXD       (0x3UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for UART1_RXD         */
#define SYS_GPD_MFPL_PD6MFP_I2C0_SDA        (0x4UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for I2C0_SDA          */
#define SYS_GPD_MFPL_PD6MFP_USCI1_DAT1      (0x7UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for USCI1_DAT1        */

/* PD.7 MFP */
#define SYS_GPD_MFPL_PD7MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for GPIO              */
#define SYS_GPD_MFPL_PD7MFP_UART1_TXD       (0x3UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for UART1_TXD         */
#define SYS_GPD_MFPL_PD7MFP_I2C0_SCL        (0x4UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for I2C0_SCL          */
#define SYS_GPD_MFPL_PD7MFP_USCI1_CLK       (0x7UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for USCI1_CLK         */

/* PD.8 MFP */
#define SYS_GPD_MFPH_PD8MFP_GPIO            (0x0UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for GPIO              */
#define SYS_GPD_MFPH_PD8MFP_I2C2_SDA        (0x3UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for I2C2_SDA          */
#define SYS_GPD_MFPH_PD8MFP_UART2_nRTS      (0x4UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for UART2_nRTS        */

/* PD.9 MFP */
#define SYS_GPD_MFPH_PD9MFP_GPIO            (0x0UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for GPIO              */
#define SYS_GPD_MFPH_PD9MFP_I2C2_SCL        (0x3UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for I2C2_SCL          */
#define SYS_GPD_MFPH_PD9MFP_UART2_nCTS      (0x4UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for UART2_nCTS        */

/* PD.10 MFP */
#define SYS_GPD_MFPH_PD10MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for GPIO             */
#define SYS_GPD_MFPH_PD10MFP_ADC0_CH16      (0x1UL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for ADC0_CH16        */
#define SYS_GPD_MFPH_PD10MFP_UART1_RXD      (0x3UL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for UART1_RXD        */

/* PD.11 MFP */
#define SYS_GPD_MFPH_PD11MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for GPIO             */
#define SYS_GPD_MFPH_PD11MFP_ADC0_CH17      (0x1UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for ADC0_CH17        */
#define SYS_GPD_MFPH_PD11MFP_UART1_TXD      (0x3UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for UART1_TXD        */
#define SYS_GPD_MFPH_PD11MFP_USCI4_DAT0     (0x8UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for USCI4_DAT0       */

/* PD.12 MFP */
#define SYS_GPD_MFPH_PD12MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for GPIO             */
#define SYS_GPD_MFPH_PD12MFP_ADC0_CH18      (0x1UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for ADC0_CH18        */
#define SYS_GPD_MFPH_PD12MFP_UART2_RXD      (0x3UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for UART2_RXD        */
#define SYS_GPD_MFPH_PD12MFP_ADC0_ST        (0xAUL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for ADC0_ST          */

/* PD.13 MFP */
#define SYS_GPD_MFPH_PD13MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for GPIO             */
#define SYS_GPD_MFPH_PD13MFP_CLKO           (0xAUL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for CLKO             */
#define SYS_GPD_MFPH_PD13MFP_ADC0_ST        (0xBUL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for ADC0_ST          */

/* PD.15 MFP */
#define SYS_GPD_MFPH_PD15MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for GPIO             */
#define SYS_GPD_MFPH_PD15MFP_BPWM0_CH5      (0xDUL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for BPWM0_CH5        */
#define SYS_GPD_MFPH_PD15MFP_TM3            (0xEUL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for TM3              */
#define SYS_GPD_MFPH_PD15MFP_INT1           (0xFUL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for INT1             */

/* PE.0 MFP */
#define SYS_GPE_MFPL_PE0MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for GPIO              */
#define SYS_GPE_MFPL_PE0MFP_UART3_RXD       (0x2UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for UART3_RXD         */
#define SYS_GPE_MFPL_PE0MFP_UART4_nRTS      (0x3UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for UART4_nRTS        */
#define SYS_GPE_MFPL_PE0MFP_I2C1_SDA        (0x4UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for I2C1_SDA          */
#define SYS_GPE_MFPL_PE0MFP_TM0_EXT         (0xEUL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for TM0_EXT           */

/* PE.1 MFP */
#define SYS_GPE_MFPL_PE1MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for GPIO              */
#define SYS_GPE_MFPL_PE1MFP_UART3_TXD       (0x2UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for UART3_TXD         */
#define SYS_GPE_MFPL_PE1MFP_UART4_nCTS      (0x3UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for UART4_nCTS        */
#define SYS_GPE_MFPL_PE1MFP_I2C1_SCL        (0x4UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for I2C1_SCL          */
#define SYS_GPE_MFPL_PE1MFP_TM8_EXT         (0xEUL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for TM8_EXT           */

/* PE.2 MFP */
#define SYS_GPE_MFPL_PE2MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for GPIO              */
#define SYS_GPE_MFPL_PE2MFP_USCI0_CLK       (0x7UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for USCI0_CLK         */
#define SYS_GPE_MFPL_PE2MFP_TM7_EXT         (0xEUL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for TM7_EXT           */

/* PE.3 MFP */
#define SYS_GPE_MFPL_PE3MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for GPIO              */
#define SYS_GPE_MFPL_PE3MFP_USCI0_DAT0      (0x7UL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for USCI0_DAT0        */
#define SYS_GPE_MFPL_PE3MFP_TM6_EXT         (0xEUL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for TM6_EXT           */

/* PE.4 MFP */
#define SYS_GPE_MFPL_PE4MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for GPIO              */
#define SYS_GPE_MFPL_PE4MFP_USCI0_DAT1      (0x7UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for USCI0_DAT1        */
#define SYS_GPE_MFPL_PE4MFP_TM5_EXT         (0xEUL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for TM5_EXT           */

/* PE.5 MFP */
#define SYS_GPE_MFPL_PE5MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for GPIO              */
#define SYS_GPE_MFPL_PE5MFP_USCI0_CTL1      (0x7UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for USCI0_CTL1        */
#define SYS_GPE_MFPL_PE5MFP_TM4_EXT         (0xEUL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for TM4_EXT           */

/* PE.6 MFP */
#define SYS_GPE_MFPL_PE6MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for GPIO              */
#define SYS_GPE_MFPL_PE6MFP_USCI0_CTL0      (0x7UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for USCI0_CTL0        */
#define SYS_GPE_MFPL_PE6MFP_TM0_EXT         (0xEUL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for TM0_EXT           */

/* PE.7 MFP */
#define SYS_GPE_MFPL_PE7MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for GPIO              */
#define SYS_GPE_MFPL_PE7MFP_TM8_EXT         (0xEUL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for TM8_EXT           */

/* PE.8 MFP */
#define SYS_GPE_MFPH_PE8MFP_GPIO            (0x0UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for GPIO              */
#define SYS_GPE_MFPH_PE8MFP_UART4_RXD       (0x3UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for UART4_RXD         */
#define SYS_GPE_MFPH_PE8MFP_TM0             (0xEUL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for TM0               */

/* PE.9 MFP */
#define SYS_GPE_MFPH_PE9MFP_GPIO            (0x0UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for GPIO              */
#define SYS_GPE_MFPH_PE9MFP_UART3_TXD       (0x3UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for UART3_TXD         */
#define SYS_GPE_MFPH_PE9MFP_USCI1_CTL0      (0x8UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for USCI1_CTL0        */

/* PE.10 MFP */
#define SYS_GPE_MFPH_PE10MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for GPIO             */
#define SYS_GPE_MFPH_PE10MFP_UART1_nCTS     (0x2UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for UART1_nCTS       */
#define SYS_GPE_MFPH_PE10MFP_UART3_RXD      (0x3UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for UART3_RXD        */
#define SYS_GPE_MFPH_PE10MFP_USCI1_DAT0     (0x8UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for USCI1_DAT0       */

/* PE.11 MFP */
#define SYS_GPE_MFPH_PE11MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for GPIO             */
#define SYS_GPE_MFPH_PE11MFP_UART1_nRTS     (0x2UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for UART1_nRTS       */
#define SYS_GPE_MFPH_PE11MFP_USCI1_DAT1     (0x8UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for USCI1_DAT1       */

/* PE.12 MFP */
#define SYS_GPE_MFPH_PE12MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for GPIO             */
#define SYS_GPE_MFPH_PE12MFP_TM1            (0xEUL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for TM1              */

/* PE.13 MFP */
#define SYS_GPE_MFPH_PE13MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for GPIO             */
#define SYS_GPE_MFPH_PE13MFP_UART1_TXD      (0x2UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for UART1_TXD        */
#define SYS_GPE_MFPH_PE13MFP_I2C0_SCL       (0x4UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for I2C0_SCL         */
#define SYS_GPE_MFPH_PE13MFP_UART4_nRTS     (0x5UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for UART4_nRTS       */
#define SYS_GPE_MFPH_PE13MFP_USCI1_CLK      (0x8UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for USCI1_CLK        */

/* PE.14 MFP */
#define SYS_GPE_MFPH_PE14MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE14MFP_Pos)   /*!< GPE_MFPH PE14 setting for GPIO             */
#define SYS_GPE_MFPH_PE14MFP_UART3_TXD      (0x2UL<<SYS_GPE_MFPH_PE14MFP_Pos)   /*!< GPE_MFPH PE14 setting for UART3_TXD        */
#define SYS_GPE_MFPH_PE14MFP_UART2_TXD      (0x3UL<<SYS_GPE_MFPH_PE14MFP_Pos)   /*!< GPE_MFPH PE14 setting for UART2_TXD        */
#define SYS_GPE_MFPH_PE14MFP_UART4_nCTS     (0x4UL<<SYS_GPE_MFPH_PE14MFP_Pos)   /*!< GPE_MFPH PE14 setting for UART4_nCTS       */
#define SYS_GPE_MFPH_PE14MFP_I2C1_SCL       (0x5UL<<SYS_GPE_MFPH_PE14MFP_Pos)   /*!< GPE_MFPH PE14 setting for I2C1_SCL         */

/* PE.15 MFP */
#define SYS_GPE_MFPH_PE15MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for GPIO             */
#define SYS_GPE_MFPH_PE15MFP_UART3_RXD      (0x2UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for UART3_RXD        */
#define SYS_GPE_MFPH_PE15MFP_UART2_RXD      (0x3UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for UART2_RXD        */
#define SYS_GPE_MFPH_PE15MFP_UART4_nRTS     (0x4UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for UART4_nRTS       */
#define SYS_GPE_MFPH_PE15MFP_I2C1_SDA       (0x5UL<<SYS_GPE_MFPH_PE15MFP_Pos)   /*!< GPE_MFPH PE15 setting for I2C1_SDA         */

/* PF.0 MFP */
#define SYS_GPF_MFPL_PF0MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for GPIO              */
#define SYS_GPF_MFPL_PF0MFP_UART1_TXD       (0x2UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for UART1_TXD         */
#define SYS_GPF_MFPL_PF0MFP_I2C1_SCL        (0x3UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for I2C1_SCL          */
#define SYS_GPF_MFPL_PF0MFP_UART0_TXD       (0x4UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for UART0_TXD         */
#define SYS_GPF_MFPL_PF0MFP_I2C0_SDA        (0x5UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for I2C0_SDA          */
#define SYS_GPF_MFPL_PF0MFP_USCI0_CTL1      (0x7UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for USCI0_CTL1        */
#define SYS_GPF_MFPL_PF0MFP_UART2_TXD       (0x8UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for UART2_TXD         */
#define SYS_GPF_MFPL_PF0MFP_I2C0_SCL        (0x9UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for I2C0_SCL          */
#define SYS_GPF_MFPL_PF0MFP_BPWM1_CH4       (0xBUL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for BPWM1_CH4         */
#define SYS_GPF_MFPL_PF0MFP_ICE_DAT         (0xEUL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for ICE_DAT           */

/* PF.1 MFP */
#define SYS_GPF_MFPL_PF1MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for GPIO              */
#define SYS_GPF_MFPL_PF1MFP_UART1_RXD       (0x2UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for UART1_RXD         */
#define SYS_GPF_MFPL_PF1MFP_I2C1_SDA        (0x3UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for I2C1_SDA          */
#define SYS_GPF_MFPL_PF1MFP_UART0_RXD       (0x4UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for UART0_RXD         */
#define SYS_GPF_MFPL_PF1MFP_I2C0_SCL        (0x5UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for I2C0_SCL          */
#define SYS_GPF_MFPL_PF1MFP_USCI0_DAT1      (0x7UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for USCI0_DAT1        */
#define SYS_GPF_MFPL_PF1MFP_UART2_RXD       (0x8UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for UART2_RXD         */
#define SYS_GPF_MFPL_PF1MFP_I2C0_SDA        (0x9UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for I2C0_SDA          */
#define SYS_GPF_MFPL_PF1MFP_BPWM1_CH5       (0xBUL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for BPWM1_CH5         */
#define SYS_GPF_MFPL_PF1MFP_ICE_CLK         (0xEUL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for ICE_CLK           */
#define SYS_GPF_MFPL_PF1MFP_TM2_EXT         (0xFUL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for TM2_EXT           */

/* PF.2 MFP */
#define SYS_GPF_MFPL_PF2MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for GPIO              */
#define SYS_GPF_MFPL_PF2MFP_UART2_RXD       (0x2UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for UART2_RXD         */
#define SYS_GPF_MFPL_PF2MFP_UART0_RXD       (0x3UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for UART0_RXD         */
#define SYS_GPF_MFPL_PF2MFP_I2C0_SDA        (0x4UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for I2C0_SDA          */
#define SYS_GPF_MFPL_PF2MFP_USCI1_CTL1      (0x8UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for USCI1_CTL1        */
#define SYS_GPF_MFPL_PF2MFP_BPWM1_CH1       (0x9UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for BPWM1_CH1         */
#define SYS_GPF_MFPL_PF2MFP_XT1_OUT         (0xAUL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for XT1_OUT           */
#define SYS_GPF_MFPL_PF2MFP_TM1_EXT         (0xFUL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for TM1_EXT           */

/* PF.3 MFP */
#define SYS_GPF_MFPL_PF3MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for GPIO              */
#define SYS_GPF_MFPL_PF3MFP_UART2_TXD       (0x2UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for UART2_TXD         */
#define SYS_GPF_MFPL_PF3MFP_UART0_TXD       (0x3UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for UART0_TXD         */
#define SYS_GPF_MFPL_PF3MFP_I2C0_SCL        (0x4UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for I2C0_SCL          */
#define SYS_GPF_MFPL_PF3MFP_BPWM1_CH0       (0x9UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for BPWM1_CH0         */
#define SYS_GPF_MFPL_PF3MFP_XT1_IN          (0xAUL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for XT1_IN            */
#define SYS_GPF_MFPL_PF3MFP_TM0_EXT         (0xFUL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for TM0_EXT           */

/* PF.4 MFP */
#define SYS_GPF_MFPL_PF4MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for GPIO              */
#define SYS_GPF_MFPL_PF4MFP_UART2_TXD       (0x3UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for UART2_TXD         */
#define SYS_GPF_MFPL_PF4MFP_UART2_nRTS      (0x4UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for UART2_nRTS        */
#define SYS_GPF_MFPL_PF4MFP_UART0_nRTS      (0x5UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for UART0_nRTS        */
#define SYS_GPF_MFPL_PF4MFP_BPWM0_CH1       (0x6UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for BPWM0_CH1         */
#define SYS_GPF_MFPL_PF4MFP_X32_OUT         (0xAUL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for X32_OUT           */

/* PF.5 MFP */
#define SYS_GPF_MFPL_PF5MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for GPIO              */
#define SYS_GPF_MFPL_PF5MFP_UART2_RXD       (0x3UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for UART2_RXD         */
#define SYS_GPF_MFPL_PF5MFP_UART2_nCTS      (0x4UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for UART2_nCTS        */
#define SYS_GPF_MFPL_PF5MFP_UART0_nCTS      (0x5UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for UART0_nCTS        */
#define SYS_GPF_MFPL_PF5MFP_BPWM0_CH0       (0x6UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for BPWM0_CH0         */
#define SYS_GPF_MFPL_PF5MFP_USCI4_CTL1      (0x8UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for USCI4_CTL1        */
#define SYS_GPF_MFPL_PF5MFP_X32_IN          (0xAUL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for X32_IN            */
#define SYS_GPF_MFPL_PF5MFP_ADC0_ST         (0xBUL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for ADC0_ST           */

/* PF.6 MFP */
#define SYS_GPF_MFPL_PF6MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF6MFP_Pos)    /*!< GPF_MFPL PF6 setting for GPIO              */
#define SYS_GPF_MFPL_PF6MFP_UART3_TXD       (0x3UL<<SYS_GPF_MFPL_PF6MFP_Pos)    /*!< GPF_MFPL PF6 setting for UART3_TXD         */
#define SYS_GPF_MFPL_PF6MFP_TM1             (0xEUL<<SYS_GPF_MFPL_PF6MFP_Pos)    /*!< GPF_MFPL PF6 setting for TM1               */

/* PF.7 MFP */
#define SYS_GPF_MFPL_PF7MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF7MFP_Pos)    /*!< GPF_MFPL PF7 setting for GPIO              */
#define SYS_GPF_MFPL_PF7MFP_UART4_TXD       (0x3UL<<SYS_GPF_MFPL_PF7MFP_Pos)    /*!< GPF_MFPL PF7 setting for UART4_TXD         */

/* PF.14 MFP */
#define SYS_GPF_MFPH_PF14MFP_GPIO           (0x0UL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for GPIO             */
#define SYS_GPF_MFPH_PF14MFP_UART3_RXD      (0x3UL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for UART3_RXD        */
#define SYS_GPF_MFPH_PF14MFP_BPWM0_CH4      (0xBUL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for BPWM0_CH4        */
#define SYS_GPF_MFPH_PF14MFP_CLKO           (0xDUL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for CLKO             */
#define SYS_GPF_MFPH_PF14MFP_TM3            (0xEUL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for TM3              */
#define SYS_GPF_MFPH_PF14MFP_INT5           (0xFUL<<SYS_GPF_MFPH_PF14MFP_Pos)   /*!< GPF_MFPH PF14 setting for INT5             */

/* PF.15 MFP */
#define SYS_GPF_MFPH_PF15MFP_GPIO           (0x0UL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for GPIO             */
#define SYS_GPF_MFPH_PF15MFP_BPWM0_CH1      (0xCUL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for BPWM0_CH1        */
#define SYS_GPF_MFPH_PF15MFP_TM2            (0xDUL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for TM2              */
#define SYS_GPF_MFPH_PF15MFP_CLKO           (0xEUL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for CLKO             */
#define SYS_GPF_MFPH_PF15MFP_INT4           (0xFUL<<SYS_GPF_MFPH_PF15MFP_Pos)   /*!< GPF_MFPH PF15 setting for INT4             */


#define SET_ADC0_CH0_PB0()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_ADC0_CH0)       /*!< Set PB0 function to ADC0_CH0       */
#define SET_ADC0_CH10_PB10()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB10MFP_Msk)) | SYS_GPB_MFPH_PB10MFP_ADC0_CH10)    /*!< Set PB10 function to ADC0_CH10     */
#define SET_ADC0_CH11_PB11()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_ADC0_CH11)    /*!< Set PB11 function to ADC0_CH11     */
#define SET_ADC0_CH12_PB12()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_ADC0_CH12)    /*!< Set PB12 function to ADC0_CH12     */
#define SET_ADC0_CH13_PB13()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_ADC0_CH13)    /*!< Set PB13 function to ADC0_CH13     */
#define SET_ADC0_CH14_PB14()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_ADC0_CH14)    /*!< Set PB14 function to ADC0_CH14     */
#define SET_ADC0_CH15_PB15()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_ADC0_CH15)    /*!< Set PB15 function to ADC0_CH15     */
#define SET_ADC0_CH16_PD10()    SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD10MFP_Msk)) | SYS_GPD_MFPH_PD10MFP_ADC0_CH16)    /*!< Set PD10 function to ADC0_CH16     */
#define SET_ADC0_CH17_PD11()    SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD11MFP_Msk)) | SYS_GPD_MFPH_PD11MFP_ADC0_CH17)    /*!< Set PD11 function to ADC0_CH17     */
#define SET_ADC0_CH18_PD12()    SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD12MFP_Msk)) | SYS_GPD_MFPH_PD12MFP_ADC0_CH18)    /*!< Set PD12 function to ADC0_CH18     */
#define SET_ADC0_CH19_PA8()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA8MFP_Msk)) | SYS_GPA_MFPH_PA8MFP_ADC0_CH19)      /*!< Set PA8 function to ADC0_CH19      */
#define SET_ADC0_CH1_PB1()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_ADC0_CH1)       /*!< Set PB1 function to ADC0_CH1       */
#define SET_ADC0_CH20_PC9()     SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk)) | SYS_GPC_MFPH_PC9MFP_ADC0_CH20)      /*!< Set PC9 function to ADC0_CH20      */
#define SET_ADC0_CH21_PA9()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA9MFP_Msk)) | SYS_GPA_MFPH_PA9MFP_ADC0_CH21)      /*!< Set PA9 function to ADC0_CH21      */
#define SET_ADC0_CH22_PA10()    SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA10MFP_Msk)) | SYS_GPA_MFPH_PA10MFP_ADC0_CH22)    /*!< Set PA10 function to ADC0_CH22     */
#define SET_ADC0_CH23_PA11()    SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA11MFP_Msk)) | SYS_GPA_MFPH_PA11MFP_ADC0_CH23)    /*!< Set PA11 function to ADC0_CH23     */
#define SET_ADC0_CH2_PB2()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SYS_GPB_MFPL_PB2MFP_ADC0_CH2)       /*!< Set PB2 function to ADC0_CH2       */
#define SET_ADC0_CH3_PB3()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SYS_GPB_MFPL_PB3MFP_ADC0_CH3)       /*!< Set PB3 function to ADC0_CH3       */
#define SET_ADC0_CH4_PB4()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_ADC0_CH4)       /*!< Set PB4 function to ADC0_CH4       */
#define SET_ADC0_CH5_PB5()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_ADC0_CH5)       /*!< Set PB5 function to ADC0_CH5       */
#define SET_ADC0_CH6_PB6()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk)) | SYS_GPB_MFPL_PB6MFP_ADC0_CH6)       /*!< Set PB6 function to ADC0_CH6       */
#define SET_ADC0_CH7_PB7()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_ADC0_CH7)       /*!< Set PB7 function to ADC0_CH7       */
#define SET_ADC0_CH8_PB8()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_ADC0_CH8)       /*!< Set PB8 function to ADC0_CH8       */
#define SET_ADC0_CH9_PB9()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | SYS_GPB_MFPH_PB9MFP_ADC0_CH9)       /*!< Set PB9 function to ADC0_CH9       */
#define SET_ADC0_ST_PB0()       SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_ADC0_ST)        /*!< Set PB0 function to ADC0_ST        */
#define SET_ADC0_ST_PB14()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_ADC0_ST)      /*!< Set PB14 function to ADC0_ST       */
#define SET_ADC0_ST_PC1()       SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk)) | SYS_GPC_MFPL_PC1MFP_ADC0_ST)        /*!< Set PC1 function to ADC0_ST        */
#define SET_ADC0_ST_PD12()      SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD12MFP_Msk)) | SYS_GPD_MFPH_PD12MFP_ADC0_ST)      /*!< Set PD12 function to ADC0_ST       */
#define SET_ADC0_ST_PD13()      SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD13MFP_Msk)) | SYS_GPD_MFPH_PD13MFP_ADC0_ST)      /*!< Set PD13 function to ADC0_ST       */
#define SET_ADC0_ST_PD5()       SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD5MFP_Msk)) | SYS_GPD_MFPL_PD5MFP_ADC0_ST)        /*!< Set PD5 function to ADC0_ST        */
#define SET_ADC0_ST_PF5()       SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_ADC0_ST)        /*!< Set PF5 function to ADC0_ST        */
#define SET_BPWM0_CH0_PA5()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA5MFP_Msk)) | SYS_GPA_MFPL_PA5MFP_BPWM0_CH0)      /*!< Set PA5 function to BPWM0_CH0      */
#define SET_BPWM0_CH0_PB13()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_BPWM0_CH0)    /*!< Set PB13 function to BPWM0_CH0     */
#define SET_BPWM0_CH0_PB5()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_BPWM0_CH0)      /*!< Set PB5 function to BPWM0_CH0      */
#define SET_BPWM0_CH0_PF5()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_BPWM0_CH0)      /*!< Set PF5 function to BPWM0_CH0      */
#define SET_BPWM0_CH1_PA4()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA4MFP_Msk)) | SYS_GPA_MFPL_PA4MFP_BPWM0_CH1)      /*!< Set PA4 function to BPWM0_CH1      */
#define SET_BPWM0_CH1_PB12()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_BPWM0_CH1)    /*!< Set PB12 function to BPWM0_CH1     */
#define SET_BPWM0_CH1_PB15()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_BPWM0_CH1)    /*!< Set PB15 function to BPWM0_CH1     */
#define SET_BPWM0_CH1_PB4()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_BPWM0_CH1)      /*!< Set PB4 function to BPWM0_CH1      */
#define SET_BPWM0_CH1_PF15()    SYS->GPF_MFPH = ((SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF15MFP_Msk)) | SYS_GPF_MFPH_PF15MFP_BPWM0_CH1)    /*!< Set PF15 function to BPWM0_CH1     */
#define SET_BPWM0_CH1_PF4()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) | SYS_GPF_MFPL_PF4MFP_BPWM0_CH1)      /*!< Set PF4 function to BPWM0_CH1      */
#define SET_BPWM0_CH2_PA3()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_BPWM0_CH2)      /*!< Set PA3 function to BPWM0_CH2      */
#define SET_BPWM0_CH2_PB1()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_BPWM0_CH2)      /*!< Set PB1 function to BPWM0_CH2      */
#define SET_BPWM0_CH2_PB3()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SYS_GPB_MFPL_PB3MFP_BPWM0_CH2)      /*!< Set PB3 function to BPWM0_CH2      */
#define SET_BPWM0_CH2_PB7()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_BPWM0_CH2)      /*!< Set PB7 function to BPWM0_CH2      */
#define SET_BPWM0_CH3_PA2()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_BPWM0_CH3)      /*!< Set PA2 function to BPWM0_CH3      */
#define SET_BPWM0_CH3_PB0()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_BPWM0_CH3)      /*!< Set PB0 function to BPWM0_CH3      */
#define SET_BPWM0_CH3_PB2()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SYS_GPB_MFPL_PB2MFP_BPWM0_CH3)      /*!< Set PB2 function to BPWM0_CH3      */
#define SET_BPWM0_CH3_PB8()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_BPWM0_CH3)      /*!< Set PB8 function to BPWM0_CH3      */
#define SET_BPWM0_CH4_PA1()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_BPWM0_CH4)      /*!< Set PA1 function to BPWM0_CH4      */
#define SET_BPWM0_CH4_PB1()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_BPWM0_CH4)      /*!< Set PB1 function to BPWM0_CH4      */
#define SET_BPWM0_CH4_PB9()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | SYS_GPB_MFPH_PB9MFP_BPWM0_CH4)      /*!< Set PB9 function to BPWM0_CH4      */
#define SET_BPWM0_CH4_PF14()    SYS->GPF_MFPH = ((SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF14MFP_Msk)) | SYS_GPF_MFPH_PF14MFP_BPWM0_CH4)    /*!< Set PF14 function to BPWM0_CH4     */
#define SET_BPWM0_CH5_PA0()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH5)      /*!< Set PA0 function to BPWM0_CH5      */
#define SET_BPWM0_CH5_PB0()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_BPWM0_CH5)      /*!< Set PB0 function to BPWM0_CH5      */
#define SET_BPWM0_CH5_PB11()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_BPWM0_CH5)    /*!< Set PB11 function to BPWM0_CH5     */
#define SET_BPWM0_CH5_PC14()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC14MFP_Msk)) | SYS_GPC_MFPH_PC14MFP_BPWM0_CH5)    /*!< Set PC14 function to BPWM0_CH5     */
#define SET_BPWM0_CH5_PD15()    SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD15MFP_Msk)) | SYS_GPD_MFPH_PD15MFP_BPWM0_CH5)    /*!< Set PD15 function to BPWM0_CH5     */
#define SET_BPWM1_CH0_PB15()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_BPWM1_CH0)    /*!< Set PB15 function to BPWM1_CH0     */
#define SET_BPWM1_CH0_PC5()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_BPWM1_CH0)      /*!< Set PC5 function to BPWM1_CH0      */
#define SET_BPWM1_CH0_PF3()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_BPWM1_CH0)      /*!< Set PF3 function to BPWM1_CH0      */
#define SET_BPWM1_CH1_PB14()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_BPWM1_CH1)    /*!< Set PB14 function to BPWM1_CH1     */
#define SET_BPWM1_CH1_PC4()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_BPWM1_CH1)      /*!< Set PC4 function to BPWM1_CH1      */
#define SET_BPWM1_CH1_PF2()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_BPWM1_CH1)      /*!< Set PF2 function to BPWM1_CH1      */
#define SET_BPWM1_CH2_PB13()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_BPWM1_CH2)    /*!< Set PB13 function to BPWM1_CH2     */
#define SET_BPWM1_CH2_PC3()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_BPWM1_CH2)      /*!< Set PC3 function to BPWM1_CH2      */
#define SET_BPWM1_CH2_PC7()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC7MFP_Msk)) | SYS_GPC_MFPL_PC7MFP_BPWM1_CH2)      /*!< Set PC7 function to BPWM1_CH2      */
#define SET_BPWM1_CH3_PB12()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_BPWM1_CH3)    /*!< Set PB12 function to BPWM1_CH3     */
#define SET_BPWM1_CH3_PC2()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk)) | SYS_GPC_MFPL_PC2MFP_BPWM1_CH3)      /*!< Set PC2 function to BPWM1_CH3      */
#define SET_BPWM1_CH3_PC6()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC6MFP_Msk)) | SYS_GPC_MFPL_PC6MFP_BPWM1_CH3)      /*!< Set PC6 function to BPWM1_CH3      */
#define SET_BPWM1_CH4_PA7()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA7MFP_Msk)) | SYS_GPA_MFPL_PA7MFP_BPWM1_CH4)      /*!< Set PA7 function to BPWM1_CH4      */
#define SET_BPWM1_CH4_PB1()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_BPWM1_CH4)      /*!< Set PB1 function to BPWM1_CH4      */
#define SET_BPWM1_CH4_PB7()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_BPWM1_CH4)      /*!< Set PB7 function to BPWM1_CH4      */
#define SET_BPWM1_CH4_PC1()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk)) | SYS_GPC_MFPL_PC1MFP_BPWM1_CH4)      /*!< Set PC1 function to BPWM1_CH4      */
#define SET_BPWM1_CH4_PF0()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_BPWM1_CH4)      /*!< Set PF0 function to BPWM1_CH4      */
#define SET_BPWM1_CH5_PA6()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA6MFP_Msk)) | SYS_GPA_MFPL_PA6MFP_BPWM1_CH5)      /*!< Set PA6 function to BPWM1_CH5      */
#define SET_BPWM1_CH5_PB0()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_BPWM1_CH5)      /*!< Set PB0 function to BPWM1_CH5      */
#define SET_BPWM1_CH5_PB6()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk)) | SYS_GPB_MFPL_PB6MFP_BPWM1_CH5)      /*!< Set PB6 function to BPWM1_CH5      */
#define SET_BPWM1_CH5_PC0()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk)) | SYS_GPC_MFPL_PC0MFP_BPWM1_CH5)      /*!< Set PC0 function to BPWM1_CH5      */
#define SET_BPWM1_CH5_PF1()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_BPWM1_CH5)      /*!< Set PF1 function to BPWM1_CH5      */
#define SET_CLKO_PA3()          SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_CLKO)           /*!< Set PA3 function to CLKO           */
#define SET_CLKO_PB12()         SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_CLKO)         /*!< Set PB12 function to CLKO          */
#define SET_CLKO_PB13()         SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_CLKO)         /*!< Set PB13 function to CLKO          */
#define SET_CLKO_PB14()         SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_CLKO)         /*!< Set PB14 function to CLKO          */
#define SET_CLKO_PD13()         SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD13MFP_Msk)) | SYS_GPD_MFPH_PD13MFP_CLKO)         /*!< Set PD13 function to CLKO          */
#define SET_CLKO_PF14()         SYS->GPF_MFPH = ((SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF14MFP_Msk)) | SYS_GPF_MFPH_PF14MFP_CLKO)         /*!< Set PF14 function to CLKO          */
#define SET_CLKO_PF15()         SYS->GPF_MFPH = ((SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF15MFP_Msk)) | SYS_GPF_MFPH_PF15MFP_CLKO)         /*!< Set PF15 function to CLKO          */
#define SET_I2C0_SCL_PA5()      SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA5MFP_Msk)) | SYS_GPA_MFPL_PA5MFP_I2C0_SCL)       /*!< Set PA5 function to I2C0_SCL       */
#define SET_I2C0_SCL_PB14()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_I2C0_SCL)     /*!< Set PB14 function to I2C0_SCL      */
#define SET_I2C0_SCL_PB5()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_I2C0_SCL)       /*!< Set PB5 function to I2C0_SCL       */
#define SET_I2C0_SCL_PB9()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | SYS_GPB_MFPH_PB9MFP_I2C0_SCL)       /*!< Set PB9 function to I2C0_SCL       */
#define SET_I2C0_SCL_PC1()      SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk)) | SYS_GPC_MFPL_PC1MFP_I2C0_SCL)       /*!< Set PC1 function to I2C0_SCL       */
#define SET_I2C0_SCL_PC12()     SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk)) | SYS_GPC_MFPH_PC12MFP_I2C0_SCL)     /*!< Set PC12 function to I2C0_SCL      */
#define SET_I2C0_SCL_PD7()      SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD7MFP_Msk)) | SYS_GPD_MFPL_PD7MFP_I2C0_SCL)       /*!< Set PD7 function to I2C0_SCL       */
#define SET_I2C0_SCL_PE13()     SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE13MFP_Msk)) | SYS_GPE_MFPH_PE13MFP_I2C0_SCL)     /*!< Set PE13 function to I2C0_SCL      */
#define SET_I2C0_SCL_PF0()      SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_I2C0_SCL)       /*!< Set PF0 function to I2C0_SCL       */
#define SET_I2C0_SCL_PF1()      SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_I2C0_SCL)       /*!< Set PF1 function to I2C0_SCL       */
#define SET_I2C0_SCL_PF3()      SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_I2C0_SCL)       /*!< Set PF3 function to I2C0_SCL       */
#define SET_I2C0_SDA_PA4()      SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA4MFP_Msk)) | SYS_GPA_MFPL_PA4MFP_I2C0_SDA)       /*!< Set PA4 function to I2C0_SDA       */
#define SET_I2C0_SDA_PB15()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_I2C0_SDA)     /*!< Set PB15 function to I2C0_SDA      */
#define SET_I2C0_SDA_PB4()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_I2C0_SDA)       /*!< Set PB4 function to I2C0_SDA       */
#define SET_I2C0_SDA_PB8()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_I2C0_SDA)       /*!< Set PB8 function to I2C0_SDA       */
#define SET_I2C0_SDA_PC0()      SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk)) | SYS_GPC_MFPL_PC0MFP_I2C0_SDA)       /*!< Set PC0 function to I2C0_SDA       */
#define SET_I2C0_SDA_PC11()     SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk)) | SYS_GPC_MFPH_PC11MFP_I2C0_SDA)     /*!< Set PC11 function to I2C0_SDA      */
#define SET_I2C0_SDA_PC8()      SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk)) | SYS_GPC_MFPH_PC8MFP_I2C0_SDA)       /*!< Set PC8 function to I2C0_SDA       */
#define SET_I2C0_SDA_PD6()      SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD6MFP_Msk)) | SYS_GPD_MFPL_PD6MFP_I2C0_SDA)       /*!< Set PD6 function to I2C0_SDA       */
#define SET_I2C0_SDA_PF0()      SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_I2C0_SDA)       /*!< Set PF0 function to I2C0_SDA       */
#define SET_I2C0_SDA_PF1()      SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_I2C0_SDA)       /*!< Set PF1 function to I2C0_SDA       */
#define SET_I2C0_SDA_PF2()      SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_I2C0_SDA)       /*!< Set PF2 function to I2C0_SDA       */
#define SET_I2C1_SCL_PA12()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA12MFP_Msk)) | SYS_GPA_MFPH_PA12MFP_I2C1_SCL)     /*!< Set PA12 function to I2C1_SCL      */
#define SET_I2C1_SCL_PA3()      SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_I2C1_SCL)       /*!< Set PA3 function to I2C1_SCL       */
#define SET_I2C1_SCL_PA7()      SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA7MFP_Msk)) | SYS_GPA_MFPL_PA7MFP_I2C1_SCL)       /*!< Set PA7 function to I2C1_SCL       */
#define SET_I2C1_SCL_PB1()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_I2C1_SCL)       /*!< Set PB1 function to I2C1_SCL       */
#define SET_I2C1_SCL_PB11()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_I2C1_SCL)     /*!< Set PB11 function to I2C1_SCL      */
#define SET_I2C1_SCL_PB3()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SYS_GPB_MFPL_PB3MFP_I2C1_SCL)       /*!< Set PB3 function to I2C1_SCL       */
#define SET_I2C1_SCL_PC5()      SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_I2C1_SCL)       /*!< Set PC5 function to I2C1_SCL       */
#define SET_I2C1_SCL_PD5()      SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD5MFP_Msk)) | SYS_GPD_MFPL_PD5MFP_I2C1_SCL)       /*!< Set PD5 function to I2C1_SCL       */
#define SET_I2C1_SCL_PE1()      SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE1MFP_Msk)) | SYS_GPE_MFPL_PE1MFP_I2C1_SCL)       /*!< Set PE1 function to I2C1_SCL       */
#define SET_I2C1_SCL_PE14()     SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE14MFP_Msk)) | SYS_GPE_MFPH_PE14MFP_I2C1_SCL)     /*!< Set PE14 function to I2C1_SCL      */
#define SET_I2C1_SCL_PF0()      SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_I2C1_SCL)       /*!< Set PF0 function to I2C1_SCL       */
#define SET_I2C1_SDA_PA13()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA13MFP_Msk)) | SYS_GPA_MFPH_PA13MFP_I2C1_SDA)     /*!< Set PA13 function to I2C1_SDA      */
#define SET_I2C1_SDA_PA2()      SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_I2C1_SDA)       /*!< Set PA2 function to I2C1_SDA       */
#define SET_I2C1_SDA_PA6()      SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA6MFP_Msk)) | SYS_GPA_MFPL_PA6MFP_I2C1_SDA)       /*!< Set PA6 function to I2C1_SDA       */
#define SET_I2C1_SDA_PB0()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_I2C1_SDA)       /*!< Set PB0 function to I2C1_SDA       */
#define SET_I2C1_SDA_PB10()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB10MFP_Msk)) | SYS_GPB_MFPH_PB10MFP_I2C1_SDA)     /*!< Set PB10 function to I2C1_SDA      */
#define SET_I2C1_SDA_PB2()      SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SYS_GPB_MFPL_PB2MFP_I2C1_SDA)       /*!< Set PB2 function to I2C1_SDA       */
#define SET_I2C1_SDA_PC4()      SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_I2C1_SDA)       /*!< Set PC4 function to I2C1_SDA       */
#define SET_I2C1_SDA_PD4()      SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD4MFP_Msk)) | SYS_GPD_MFPL_PD4MFP_I2C1_SDA)       /*!< Set PD4 function to I2C1_SDA       */
#define SET_I2C1_SDA_PE0()      SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE0MFP_Msk)) | SYS_GPE_MFPL_PE0MFP_I2C1_SDA)       /*!< Set PE0 function to I2C1_SDA       */
#define SET_I2C1_SDA_PE15()     SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE15MFP_Msk)) | SYS_GPE_MFPH_PE15MFP_I2C1_SDA)     /*!< Set PE15 function to I2C1_SDA      */
#define SET_I2C1_SDA_PF1()      SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_I2C1_SDA)       /*!< Set PF1 function to I2C1_SDA       */
#define SET_I2C2_SCL_PA1()      SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_I2C2_SCL)       /*!< Set PA1 function to I2C2_SCL       */
#define SET_I2C2_SCL_PA11()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA11MFP_Msk)) | SYS_GPA_MFPH_PA11MFP_I2C2_SCL)     /*!< Set PA11 function to I2C2_SCL      */
#define SET_I2C2_SCL_PA14()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA14MFP_Msk)) | SYS_GPA_MFPH_PA14MFP_I2C2_SCL)     /*!< Set PA14 function to I2C2_SCL      */
#define SET_I2C2_SCL_PB13()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_I2C2_SCL)     /*!< Set PB13 function to I2C2_SCL      */
#define SET_I2C2_SCL_PD1()      SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD1MFP_Msk)) | SYS_GPD_MFPL_PD1MFP_I2C2_SCL)       /*!< Set PD1 function to I2C2_SCL       */
#define SET_I2C2_SCL_PD9()      SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD9MFP_Msk)) | SYS_GPD_MFPH_PD9MFP_I2C2_SCL)       /*!< Set PD9 function to I2C2_SCL       */
#define SET_I2C2_SDA_PA0()      SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_I2C2_SDA)       /*!< Set PA0 function to I2C2_SDA       */
#define SET_I2C2_SDA_PA10()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA10MFP_Msk)) | SYS_GPA_MFPH_PA10MFP_I2C2_SDA)     /*!< Set PA10 function to I2C2_SDA      */
#define SET_I2C2_SDA_PA15()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA15MFP_Msk)) | SYS_GPA_MFPH_PA15MFP_I2C2_SDA)     /*!< Set PA15 function to I2C2_SDA      */
#define SET_I2C2_SDA_PB12()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_I2C2_SDA)     /*!< Set PB12 function to I2C2_SDA      */
#define SET_I2C2_SDA_PD0()      SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD0MFP_Msk)) | SYS_GPD_MFPL_PD0MFP_I2C2_SDA)       /*!< Set PD0 function to I2C2_SDA       */
#define SET_I2C2_SDA_PD8()      SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD8MFP_Msk)) | SYS_GPD_MFPH_PD8MFP_I2C2_SDA)       /*!< Set PD8 function to I2C2_SDA       */
#define SET_ICE_CLK_PF1()       SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_ICE_CLK)        /*!< Set PF1 function to ICE_CLK        */
#define SET_ICE_DAT_PF0()       SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_ICE_DAT)        /*!< Set PF0 function to ICE_DAT        */
#define SET_INT0_PA6()          SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA6MFP_Msk)) | SYS_GPA_MFPL_PA6MFP_INT0)           /*!< Set PA6 function to INT0           */
#define SET_INT0_PB5()          SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_INT0)           /*!< Set PB5 function to INT0           */
#define SET_INT1_PA7()          SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA7MFP_Msk)) | SYS_GPA_MFPL_PA7MFP_INT1)           /*!< Set PA7 function to INT1           */
#define SET_INT1_PB4()          SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_INT1)           /*!< Set PB4 function to INT1           */
#define SET_INT1_PD15()         SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD15MFP_Msk)) | SYS_GPD_MFPH_PD15MFP_INT1)         /*!< Set PD15 function to INT1          */
#define SET_INT2_PB3()          SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SYS_GPB_MFPL_PB3MFP_INT2)           /*!< Set PB3 function to INT2           */
#define SET_INT2_PC6()          SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC6MFP_Msk)) | SYS_GPC_MFPL_PC6MFP_INT2)           /*!< Set PC6 function to INT2           */
#define SET_INT3_PB2()          SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SYS_GPB_MFPL_PB2MFP_INT3)           /*!< Set PB2 function to INT3           */
#define SET_INT3_PC7()          SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC7MFP_Msk)) | SYS_GPC_MFPL_PC7MFP_INT3)           /*!< Set PC7 function to INT3           */
#define SET_INT4_PA8()          SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA8MFP_Msk)) | SYS_GPA_MFPH_PA8MFP_INT4)           /*!< Set PA8 function to INT4           */
#define SET_INT4_PB6()          SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk)) | SYS_GPB_MFPL_PB6MFP_INT4)           /*!< Set PB6 function to INT4           */
#define SET_INT4_PF15()         SYS->GPF_MFPH = ((SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF15MFP_Msk)) | SYS_GPF_MFPH_PF15MFP_INT4)         /*!< Set PF15 function to INT4          */
#define SET_INT5_PB7()          SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_INT5)           /*!< Set PB7 function to INT5           */
#define SET_INT5_PF14()         SYS->GPF_MFPH = ((SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF14MFP_Msk)) | SYS_GPF_MFPH_PF14MFP_INT5)         /*!< Set PF14 function to INT5          */
#define SET_TM0_PB1()           SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_TM0)            /*!< Set PB1 function to TM0            */
#define SET_TM0_PB5()           SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_TM0)            /*!< Set PB5 function to TM0            */
#define SET_TM0_PC7()           SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC7MFP_Msk)) | SYS_GPC_MFPL_PC7MFP_TM0)            /*!< Set PC7 function to TM0            */
#define SET_TM0_PE8()           SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE8MFP_Msk)) | SYS_GPE_MFPH_PE8MFP_TM0)            /*!< Set PE8 function to TM0            */
#define SET_TM0_EXT_PA11()      SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA11MFP_Msk)) | SYS_GPA_MFPH_PA11MFP_TM0_EXT)      /*!< Set PA11 function to TM0_EXT       */
#define SET_TM0_EXT_PB1()       SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_TM0_EXT)        /*!< Set PB1 function to TM0_EXT        */
#define SET_TM0_EXT_PB15()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_TM0_EXT)      /*!< Set PB15 function to TM0_EXT       */
#define SET_TM0_EXT_PE0()       SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE0MFP_Msk)) | SYS_GPE_MFPL_PE0MFP_TM0_EXT)        /*!< Set PE0 function to TM0_EXT        */
#define SET_TM0_EXT_PE6()       SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE6MFP_Msk)) | SYS_GPE_MFPL_PE6MFP_TM0_EXT)        /*!< Set PE6 function to TM0_EXT        */
#define SET_TM0_EXT_PF3()       SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_TM0_EXT)        /*!< Set PF3 function to TM0_EXT        */
#define SET_TM1_PB15()          SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_TM1)          /*!< Set PB15 function to TM1           */
#define SET_TM1_PB4()           SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_TM1)            /*!< Set PB4 function to TM1            */
#define SET_TM1_PB8()           SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_TM1)            /*!< Set PB8 function to TM1            */
#define SET_TM1_PC14()          SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC14MFP_Msk)) | SYS_GPC_MFPH_PC14MFP_TM1)          /*!< Set PC14 function to TM1           */
#define SET_TM1_PC6()           SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC6MFP_Msk)) | SYS_GPC_MFPL_PC6MFP_TM1)            /*!< Set PC6 function to TM1            */
#define SET_TM1_PE12()          SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE12MFP_Msk)) | SYS_GPE_MFPH_PE12MFP_TM1)          /*!< Set PE12 function to TM1           */
#define SET_TM1_PF6()           SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF6MFP_Msk)) | SYS_GPF_MFPL_PF6MFP_TM1)            /*!< Set PF6 function to TM1            */
#define SET_TM1_EXT_PA10()      SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA10MFP_Msk)) | SYS_GPA_MFPH_PA10MFP_TM1_EXT)      /*!< Set PA10 function to TM1_EXT       */
#define SET_TM1_EXT_PB14()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_TM1_EXT)      /*!< Set PB14 function to TM1_EXT       */
#define SET_TM1_EXT_PB9()       SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | SYS_GPB_MFPH_PB9MFP_TM1_EXT)        /*!< Set PB9 function to TM1_EXT        */
#define SET_TM1_EXT_PF2()       SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_TM1_EXT)        /*!< Set PF2 function to TM1_EXT        */
#define SET_TM2_PA7()           SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA7MFP_Msk)) | SYS_GPA_MFPL_PA7MFP_TM2)            /*!< Set PA7 function to TM2            */
#define SET_TM2_PB3()           SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SYS_GPB_MFPL_PB3MFP_TM2)            /*!< Set PB3 function to TM2            */
#define SET_TM2_PD0()           SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD0MFP_Msk)) | SYS_GPD_MFPL_PD0MFP_TM2)            /*!< Set PD0 function to TM2            */
#define SET_TM2_PF15()          SYS->GPF_MFPH = ((SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF15MFP_Msk)) | SYS_GPF_MFPH_PF15MFP_TM2)          /*!< Set PF15 function to TM2           */
#define SET_TM2_EXT_PA9()       SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA9MFP_Msk)) | SYS_GPA_MFPH_PA9MFP_TM2_EXT)        /*!< Set PA9 function to TM2_EXT        */
#define SET_TM2_EXT_PB11()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_TM2_EXT)      /*!< Set PB11 function to TM2_EXT       */
#define SET_TM2_EXT_PB13()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_TM2_EXT)      /*!< Set PB13 function to TM2_EXT       */
#define SET_TM2_EXT_PB7()       SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_TM2_EXT)        /*!< Set PB7 function to TM2_EXT        */
#define SET_TM2_EXT_PF1()       SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_TM2_EXT)        /*!< Set PF1 function to TM2_EXT        */
#define SET_TM3_PA6()           SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA6MFP_Msk)) | SYS_GPA_MFPL_PA6MFP_TM3)            /*!< Set PA6 function to TM3            */
#define SET_TM3_PB2()           SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SYS_GPB_MFPL_PB2MFP_TM3)            /*!< Set PB2 function to TM3            */
#define SET_TM3_PD15()          SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD15MFP_Msk)) | SYS_GPD_MFPH_PD15MFP_TM3)          /*!< Set PD15 function to TM3           */
#define SET_TM3_PF14()          SYS->GPF_MFPH = ((SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF14MFP_Msk)) | SYS_GPF_MFPH_PF14MFP_TM3)          /*!< Set PF14 function to TM3           */
#define SET_TM3_EXT_PA8()       SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA8MFP_Msk)) | SYS_GPA_MFPH_PA8MFP_TM3_EXT)        /*!< Set PA8 function to TM3_EXT        */
#define SET_TM3_EXT_PB0()       SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_TM3_EXT)        /*!< Set PB0 function to TM3_EXT        */
#define SET_TM3_EXT_PB12()      SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_TM3_EXT)      /*!< Set PB12 function to TM3_EXT       */
#define SET_TM3_EXT_PB8()       SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_TM3_EXT)        /*!< Set PB8 function to TM3_EXT        */
#define SET_TM3_EXT_PC14()      SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC14MFP_Msk)) | SYS_GPC_MFPH_PC14MFP_TM3_EXT)      /*!< Set PC14 function to TM3_EXT       */
#define SET_TM4_PC12()          SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk)) | SYS_GPC_MFPH_PC12MFP_TM4)          /*!< Set PC12 function to TM4           */
#define SET_TM4_EXT_PE5()       SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE5MFP_Msk)) | SYS_GPE_MFPL_PE5MFP_TM4_EXT)        /*!< Set PE5 function to TM4_EXT        */
#define SET_TM5_PC11()          SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk)) | SYS_GPC_MFPH_PC11MFP_TM5)          /*!< Set PC11 function to TM5           */
#define SET_TM5_EXT_PE4()       SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE4MFP_Msk)) | SYS_GPE_MFPL_PE4MFP_TM5_EXT)        /*!< Set PE4 function to TM5_EXT        */
#define SET_TM6_PC10()          SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk)) | SYS_GPC_MFPH_PC10MFP_TM6)          /*!< Set PC10 function to TM6           */
#define SET_TM6_EXT_PE3()       SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE3MFP_Msk)) | SYS_GPE_MFPL_PE3MFP_TM6_EXT)        /*!< Set PE3 function to TM6_EXT        */
#define SET_TM7_PC9()           SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk)) | SYS_GPC_MFPH_PC9MFP_TM7)            /*!< Set PC9 function to TM7            */
#define SET_TM7_EXT_PE2()       SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE2MFP_Msk)) | SYS_GPE_MFPL_PE2MFP_TM7_EXT)        /*!< Set PE2 function to TM7_EXT        */
#define SET_TM8_PB0()           SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_TM8)            /*!< Set PB0 function to TM8            */
#define SET_TM8_EXT_PE1()       SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE1MFP_Msk)) | SYS_GPE_MFPL_PE1MFP_TM8_EXT)        /*!< Set PE1 function to TM8_EXT        */
#define SET_TM8_EXT_PE7()       SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE7MFP_Msk)) | SYS_GPE_MFPL_PE7MFP_TM8_EXT)        /*!< Set PE7 function to TM8_EXT        */
#define SET_UART0_RXD_PA0()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_UART0_RXD)      /*!< Set PA0 function to UART0_RXD      */
#define SET_UART0_RXD_PA15()    SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA15MFP_Msk)) | SYS_GPA_MFPH_PA15MFP_UART0_RXD)    /*!< Set PA15 function to UART0_RXD     */
#define SET_UART0_RXD_PA4()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA4MFP_Msk)) | SYS_GPA_MFPL_PA4MFP_UART0_RXD)      /*!< Set PA4 function to UART0_RXD      */
#define SET_UART0_RXD_PA6()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA6MFP_Msk)) | SYS_GPA_MFPL_PA6MFP_UART0_RXD)      /*!< Set PA6 function to UART0_RXD      */
#define SET_UART0_RXD_PB12()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_UART0_RXD)    /*!< Set PB12 function to UART0_RXD     */
#define SET_UART0_RXD_PB14()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_UART0_RXD)    /*!< Set PB14 function to UART0_RXD     */
#define SET_UART0_RXD_PB3()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SYS_GPB_MFPL_PB3MFP_UART0_RXD)      /*!< Set PB3 function to UART0_RXD      */
#define SET_UART0_RXD_PB8()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_UART0_RXD)      /*!< Set PB8 function to UART0_RXD      */
#define SET_UART0_RXD_PC11()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk)) | SYS_GPC_MFPH_PC11MFP_UART0_RXD)    /*!< Set PC11 function to UART0_RXD     */
#define SET_UART0_RXD_PC12()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk)) | SYS_GPC_MFPH_PC12MFP_UART0_RXD)    /*!< Set PC12 function to UART0_RXD     */
#define SET_UART0_RXD_PD2()     SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD2MFP_Msk)) | SYS_GPD_MFPL_PD2MFP_UART0_RXD)      /*!< Set PD2 function to UART0_RXD      */
#define SET_UART0_RXD_PF1()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_UART0_RXD)      /*!< Set PF1 function to UART0_RXD      */
#define SET_UART0_RXD_PF2()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_UART0_RXD)      /*!< Set PF2 function to UART0_RXD      */
#define SET_UART0_TXD_PA1()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_UART0_TXD)      /*!< Set PA1 function to UART0_TXD      */
#define SET_UART0_TXD_PA14()    SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA14MFP_Msk)) | SYS_GPA_MFPH_PA14MFP_UART0_TXD)    /*!< Set PA14 function to UART0_TXD     */
#define SET_UART0_TXD_PA5()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA5MFP_Msk)) | SYS_GPA_MFPL_PA5MFP_UART0_TXD)      /*!< Set PA5 function to UART0_TXD      */
#define SET_UART0_TXD_PA7()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA7MFP_Msk)) | SYS_GPA_MFPL_PA7MFP_UART0_TXD)      /*!< Set PA7 function to UART0_TXD      */
#define SET_UART0_TXD_PB13()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_UART0_TXD)    /*!< Set PB13 function to UART0_TXD     */
#define SET_UART0_TXD_PB15()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_UART0_TXD)    /*!< Set PB15 function to UART0_TXD     */
#define SET_UART0_TXD_PB2()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SYS_GPB_MFPL_PB2MFP_UART0_TXD)      /*!< Set PB2 function to UART0_TXD      */
#define SET_UART0_TXD_PB9()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | SYS_GPB_MFPH_PB9MFP_UART0_TXD)      /*!< Set PB9 function to UART0_TXD      */
#define SET_UART0_TXD_PC11()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk)) | SYS_GPC_MFPH_PC11MFP_UART0_TXD)    /*!< Set PC11 function to UART0_TXD     */
#define SET_UART0_TXD_PC12()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk)) | SYS_GPC_MFPH_PC12MFP_UART0_TXD)    /*!< Set PC12 function to UART0_TXD     */
#define SET_UART0_TXD_PC14()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC14MFP_Msk)) | SYS_GPC_MFPH_PC14MFP_UART0_TXD)    /*!< Set PC14 function to UART0_TXD     */
#define SET_UART0_TXD_PD3()     SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD3MFP_Msk)) | SYS_GPD_MFPL_PD3MFP_UART0_TXD)      /*!< Set PD3 function to UART0_TXD      */
#define SET_UART0_TXD_PF0()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_UART0_TXD)      /*!< Set PF0 function to UART0_TXD      */
#define SET_UART0_TXD_PF3()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_UART0_TXD)      /*!< Set PF3 function to UART0_TXD      */
#define SET_UART0_nCTS_PA5()    SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA5MFP_Msk)) | SYS_GPA_MFPL_PA5MFP_UART0_nCTS)     /*!< Set PA5 function to UART0_nCTS     */
#define SET_UART0_nCTS_PB11()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_UART0_nCTS)   /*!< Set PB11 function to UART0_nCTS    */
#define SET_UART0_nCTS_PB15()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_UART0_nCTS)   /*!< Set PB15 function to UART0_nCTS    */
#define SET_UART0_nCTS_PC7()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC7MFP_Msk)) | SYS_GPC_MFPL_PC7MFP_UART0_nCTS)     /*!< Set PC7 function to UART0_nCTS     */
#define SET_UART0_nCTS_PF5()    SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_UART0_nCTS)     /*!< Set PF5 function to UART0_nCTS     */
#define SET_UART0_nRTS_PA4()    SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA4MFP_Msk)) | SYS_GPA_MFPL_PA4MFP_UART0_nRTS)     /*!< Set PA4 function to UART0_nRTS     */
#define SET_UART0_nRTS_PB10()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB10MFP_Msk)) | SYS_GPB_MFPH_PB10MFP_UART0_nRTS)   /*!< Set PB10 function to UART0_nRTS    */
#define SET_UART0_nRTS_PB13()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_UART0_nRTS)   /*!< Set PB13 function to UART0_nRTS    */
#define SET_UART0_nRTS_PB14()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_UART0_nRTS)   /*!< Set PB14 function to UART0_nRTS    */
#define SET_UART0_nRTS_PC6()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC6MFP_Msk)) | SYS_GPC_MFPL_PC6MFP_UART0_nRTS)     /*!< Set PC6 function to UART0_nRTS     */
#define SET_UART0_nRTS_PF4()    SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) | SYS_GPF_MFPL_PF4MFP_UART0_nRTS)     /*!< Set PF4 function to UART0_nRTS     */
#define SET_UART1_RXD_PA2()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_UART1_RXD)      /*!< Set PA2 function to UART1_RXD      */
#define SET_UART1_RXD_PA8()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA8MFP_Msk)) | SYS_GPA_MFPH_PA8MFP_UART1_RXD)      /*!< Set PA8 function to UART1_RXD      */
#define SET_UART1_RXD_PB2()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SYS_GPB_MFPL_PB2MFP_UART1_RXD)      /*!< Set PB2 function to UART1_RXD      */
#define SET_UART1_RXD_PB6()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk)) | SYS_GPB_MFPL_PB6MFP_UART1_RXD)      /*!< Set PB6 function to UART1_RXD      */
#define SET_UART1_RXD_PC8()     SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk)) | SYS_GPC_MFPH_PC8MFP_UART1_RXD)      /*!< Set PC8 function to UART1_RXD      */
#define SET_UART1_RXD_PD10()    SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD10MFP_Msk)) | SYS_GPD_MFPH_PD10MFP_UART1_RXD)    /*!< Set PD10 function to UART1_RXD     */
#define SET_UART1_RXD_PD6()     SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD6MFP_Msk)) | SYS_GPD_MFPL_PD6MFP_UART1_RXD)      /*!< Set PD6 function to UART1_RXD      */
#define SET_UART1_RXD_PF1()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_UART1_RXD)      /*!< Set PF1 function to UART1_RXD      */
#define SET_UART1_TXD_PA3()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_UART1_TXD)      /*!< Set PA3 function to UART1_TXD      */
#define SET_UART1_TXD_PA9()     SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA9MFP_Msk)) | SYS_GPA_MFPH_PA9MFP_UART1_TXD)      /*!< Set PA9 function to UART1_TXD      */
#define SET_UART1_TXD_PB3()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SYS_GPB_MFPL_PB3MFP_UART1_TXD)      /*!< Set PB3 function to UART1_TXD      */
#define SET_UART1_TXD_PB7()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_UART1_TXD)      /*!< Set PB7 function to UART1_TXD      */
#define SET_UART1_TXD_PD11()    SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD11MFP_Msk)) | SYS_GPD_MFPH_PD11MFP_UART1_TXD)    /*!< Set PD11 function to UART1_TXD     */
#define SET_UART1_TXD_PD7()     SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD7MFP_Msk)) | SYS_GPD_MFPL_PD7MFP_UART1_TXD)      /*!< Set PD7 function to UART1_TXD      */
#define SET_UART1_TXD_PE13()    SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE13MFP_Msk)) | SYS_GPE_MFPH_PE13MFP_UART1_TXD)    /*!< Set PE13 function to UART1_TXD     */
#define SET_UART1_TXD_PF0()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_UART1_TXD)      /*!< Set PF0 function to UART1_TXD      */
#define SET_UART1_nCTS_PA1()    SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_UART1_nCTS)     /*!< Set PA1 function to UART1_nCTS     */
#define SET_UART1_nCTS_PB9()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | SYS_GPB_MFPH_PB9MFP_UART1_nCTS)     /*!< Set PB9 function to UART1_nCTS     */
#define SET_UART1_nCTS_PE10()   SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE10MFP_Msk)) | SYS_GPE_MFPH_PE10MFP_UART1_nCTS)   /*!< Set PE10 function to UART1_nCTS    */
#define SET_UART1_nRTS_PA0()    SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_UART1_nRTS)     /*!< Set PA0 function to UART1_nRTS     */
#define SET_UART1_nRTS_PB8()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_UART1_nRTS)     /*!< Set PB8 function to UART1_nRTS     */
#define SET_UART1_nRTS_PE11()   SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE11MFP_Msk)) | SYS_GPE_MFPH_PE11MFP_UART1_nRTS)   /*!< Set PE11 function to UART1_nRTS    */
#define SET_UART2_RXD_PB0()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART2_RXD)      /*!< Set PB0 function to UART2_RXD      */
#define SET_UART2_RXD_PB4()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_UART2_RXD)      /*!< Set PB4 function to UART2_RXD      */
#define SET_UART2_RXD_PC0()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk)) | SYS_GPC_MFPL_PC0MFP_UART2_RXD)      /*!< Set PC0 function to UART2_RXD      */
#define SET_UART2_RXD_PC4()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_UART2_RXD)      /*!< Set PC4 function to UART2_RXD      */
#define SET_UART2_RXD_PD12()    SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD12MFP_Msk)) | SYS_GPD_MFPH_PD12MFP_UART2_RXD)    /*!< Set PD12 function to UART2_RXD     */
#define SET_UART2_RXD_PE15()    SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE15MFP_Msk)) | SYS_GPE_MFPH_PE15MFP_UART2_RXD)    /*!< Set PE15 function to UART2_RXD     */
#define SET_UART2_RXD_PF1()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_UART2_RXD)      /*!< Set PF1 function to UART2_RXD      */
#define SET_UART2_RXD_PF2()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_UART2_RXD)      /*!< Set PF2 function to UART2_RXD      */
#define SET_UART2_RXD_PF5()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_UART2_RXD)      /*!< Set PF5 function to UART2_RXD      */
#define SET_UART2_TXD_PB1()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART2_TXD)      /*!< Set PB1 function to UART2_TXD      */
#define SET_UART2_TXD_PB5()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_UART2_TXD)      /*!< Set PB5 function to UART2_TXD      */
#define SET_UART2_TXD_PC1()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk)) | SYS_GPC_MFPL_PC1MFP_UART2_TXD)      /*!< Set PC1 function to UART2_TXD      */
#define SET_UART2_TXD_PC5()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_UART2_TXD)      /*!< Set PC5 function to UART2_TXD      */
#define SET_UART2_TXD_PE14()    SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE14MFP_Msk)) | SYS_GPE_MFPH_PE14MFP_UART2_TXD)    /*!< Set PE14 function to UART2_TXD     */
#define SET_UART2_TXD_PF0()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_UART2_TXD)      /*!< Set PF0 function to UART2_TXD      */
#define SET_UART2_TXD_PF3()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_UART2_TXD)      /*!< Set PF3 function to UART2_TXD      */
#define SET_UART2_TXD_PF4()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) | SYS_GPF_MFPL_PF4MFP_UART2_TXD)      /*!< Set PF4 function to UART2_TXD      */
#define SET_UART2_nCTS_PC2()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk)) | SYS_GPC_MFPL_PC2MFP_UART2_nCTS)     /*!< Set PC2 function to UART2_nCTS     */
#define SET_UART2_nCTS_PD9()    SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD9MFP_Msk)) | SYS_GPD_MFPH_PD9MFP_UART2_nCTS)     /*!< Set PD9 function to UART2_nCTS     */
#define SET_UART2_nCTS_PF5()    SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_UART2_nCTS)     /*!< Set PF5 function to UART2_nCTS     */
#define SET_UART2_nRTS_PC3()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_UART2_nRTS)     /*!< Set PC3 function to UART2_nRTS     */
#define SET_UART2_nRTS_PD8()    SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD8MFP_Msk)) | SYS_GPD_MFPH_PD8MFP_UART2_nRTS)     /*!< Set PD8 function to UART2_nRTS     */
#define SET_UART2_nRTS_PF4()    SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) | SYS_GPF_MFPL_PF4MFP_UART2_nRTS)     /*!< Set PF4 function to UART2_nRTS     */
#define SET_UART3_RXD_PB14()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_UART3_RXD)    /*!< Set PB14 function to UART3_RXD     */
#define SET_UART3_RXD_PC2()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk)) | SYS_GPC_MFPL_PC2MFP_UART3_RXD)      /*!< Set PC2 function to UART3_RXD      */
#define SET_UART3_RXD_PC9()     SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk)) | SYS_GPC_MFPH_PC9MFP_UART3_RXD)      /*!< Set PC9 function to UART3_RXD      */
#define SET_UART3_RXD_PD0()     SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD0MFP_Msk)) | SYS_GPD_MFPL_PD0MFP_UART3_RXD)      /*!< Set PD0 function to UART3_RXD      */
#define SET_UART3_RXD_PE0()     SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE0MFP_Msk)) | SYS_GPE_MFPL_PE0MFP_UART3_RXD)      /*!< Set PE0 function to UART3_RXD      */
#define SET_UART3_RXD_PE10()    SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE10MFP_Msk)) | SYS_GPE_MFPH_PE10MFP_UART3_RXD)    /*!< Set PE10 function to UART3_RXD     */
#define SET_UART3_RXD_PE15()    SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE15MFP_Msk)) | SYS_GPE_MFPH_PE15MFP_UART3_RXD)    /*!< Set PE15 function to UART3_RXD     */
#define SET_UART3_RXD_PF14()    SYS->GPF_MFPH = ((SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF14MFP_Msk)) | SYS_GPF_MFPH_PF14MFP_UART3_RXD)    /*!< Set PF14 function to UART3_RXD     */
#define SET_UART3_TXD_PB15()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_UART3_TXD)    /*!< Set PB15 function to UART3_TXD     */
#define SET_UART3_TXD_PC10()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk)) | SYS_GPC_MFPH_PC10MFP_UART3_TXD)    /*!< Set PC10 function to UART3_TXD     */
#define SET_UART3_TXD_PC3()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_UART3_TXD)      /*!< Set PC3 function to UART3_TXD      */
#define SET_UART3_TXD_PD1()     SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD1MFP_Msk)) | SYS_GPD_MFPL_PD1MFP_UART3_TXD)      /*!< Set PD1 function to UART3_TXD      */
#define SET_UART3_TXD_PE1()     SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE1MFP_Msk)) | SYS_GPE_MFPL_PE1MFP_UART3_TXD)      /*!< Set PE1 function to UART3_TXD      */
#define SET_UART3_TXD_PE14()    SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE14MFP_Msk)) | SYS_GPE_MFPH_PE14MFP_UART3_TXD)    /*!< Set PE14 function to UART3_TXD     */
#define SET_UART3_TXD_PE9()     SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE9MFP_Msk)) | SYS_GPE_MFPH_PE9MFP_UART3_TXD)      /*!< Set PE9 function to UART3_TXD      */
#define SET_UART3_TXD_PF6()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF6MFP_Msk)) | SYS_GPF_MFPL_PF6MFP_UART3_TXD)      /*!< Set PF6 function to UART3_TXD      */
#define SET_UART3_nCTS_PB12()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_UART3_nCTS)   /*!< Set PB12 function to UART3_nCTS    */
#define SET_UART3_nCTS_PD2()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD2MFP_Msk)) | SYS_GPD_MFPL_PD2MFP_UART3_nCTS)     /*!< Set PD2 function to UART3_nCTS     */
#define SET_UART3_nRTS_PB13()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_UART3_nRTS)   /*!< Set PB13 function to UART3_nRTS    */
#define SET_UART3_nRTS_PD3()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD3MFP_Msk)) | SYS_GPD_MFPL_PD3MFP_UART3_nRTS)     /*!< Set PD3 function to UART3_nRTS     */
#define SET_UART4_RXD_PA13()    SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA13MFP_Msk)) | SYS_GPA_MFPH_PA13MFP_UART4_RXD)    /*!< Set PA13 function to UART4_RXD     */
#define SET_UART4_RXD_PA2()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_UART4_RXD)      /*!< Set PA2 function to UART4_RXD      */
#define SET_UART4_RXD_PB10()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB10MFP_Msk)) | SYS_GPB_MFPH_PB10MFP_UART4_RXD)    /*!< Set PB10 function to UART4_RXD     */
#define SET_UART4_RXD_PC4()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_UART4_RXD)      /*!< Set PC4 function to UART4_RXD      */
#define SET_UART4_RXD_PC6()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC6MFP_Msk)) | SYS_GPC_MFPL_PC6MFP_UART4_RXD)      /*!< Set PC6 function to UART4_RXD      */
#define SET_UART4_RXD_PE8()     SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE8MFP_Msk)) | SYS_GPE_MFPH_PE8MFP_UART4_RXD)      /*!< Set PE8 function to UART4_RXD      */
#define SET_UART4_TXD_PA12()    SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA12MFP_Msk)) | SYS_GPA_MFPH_PA12MFP_UART4_TXD)    /*!< Set PA12 function to UART4_TXD     */
#define SET_UART4_TXD_PA3()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_UART4_TXD)      /*!< Set PA3 function to UART4_TXD      */
#define SET_UART4_TXD_PB11()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_UART4_TXD)    /*!< Set PB11 function to UART4_TXD     */
#define SET_UART4_TXD_PC5()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_UART4_TXD)      /*!< Set PC5 function to UART4_TXD      */
#define SET_UART4_TXD_PC7()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC7MFP_Msk)) | SYS_GPC_MFPL_PC7MFP_UART4_TXD)      /*!< Set PC7 function to UART4_TXD      */
#define SET_UART4_TXD_PF7()     SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF7MFP_Msk)) | SYS_GPF_MFPL_PF7MFP_UART4_TXD)      /*!< Set PF7 function to UART4_TXD      */
#define SET_UART4_nCTS_PC8()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk)) | SYS_GPC_MFPH_PC8MFP_UART4_nCTS)     /*!< Set PC8 function to UART4_nCTS     */
#define SET_UART4_nCTS_PE1()    SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE1MFP_Msk)) | SYS_GPE_MFPL_PE1MFP_UART4_nCTS)     /*!< Set PE1 function to UART4_nCTS     */
#define SET_UART4_nCTS_PE14()   SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE14MFP_Msk)) | SYS_GPE_MFPH_PE14MFP_UART4_nCTS)   /*!< Set PE14 function to UART4_nCTS    */
#define SET_UART4_nRTS_PE0()    SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE0MFP_Msk)) | SYS_GPE_MFPL_PE0MFP_UART4_nRTS)     /*!< Set PE0 function to UART4_nRTS     */
#define SET_UART4_nRTS_PE13()   SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE13MFP_Msk)) | SYS_GPE_MFPH_PE13MFP_UART4_nRTS)   /*!< Set PE13 function to UART4_nRTS    */
#define SET_UART4_nRTS_PE15()   SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE15MFP_Msk)) | SYS_GPE_MFPH_PE15MFP_UART4_nRTS)   /*!< Set PE15 function to UART4_nRTS    */
#define SET_USCI0_CLK_PA11()    SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA11MFP_Msk)) | SYS_GPA_MFPH_PA11MFP_USCI0_CLK)    /*!< Set PA11 function to USCI0_CLK     */
#define SET_USCI0_CLK_PB12()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_USCI0_CLK)    /*!< Set PB12 function to USCI0_CLK     */
#define SET_USCI0_CLK_PB7()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_USCI0_CLK)      /*!< Set PB7 function to USCI0_CLK      */
#define SET_USCI0_CLK_PD0()     SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD0MFP_Msk)) | SYS_GPD_MFPL_PD0MFP_USCI0_CLK)      /*!< Set PD0 function to USCI0_CLK      */
#define SET_USCI0_CLK_PE2()     SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE2MFP_Msk)) | SYS_GPE_MFPL_PE2MFP_USCI0_CLK)      /*!< Set PE2 function to USCI0_CLK      */
#define SET_USCI0_CTL0_PB0()    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_USCI0_CTL0)     /*!< Set PB0 function to USCI0_CTL0     */
#define SET_USCI0_CTL0_PB11()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_USCI0_CTL0)   /*!< Set PB11 function to USCI0_CTL0    */
#define SET_USCI0_CTL0_PB13()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_USCI0_CTL0)   /*!< Set PB13 function to USCI0_CTL0    */
#define SET_USCI0_CTL0_PC14()   SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC14MFP_Msk)) | SYS_GPC_MFPH_PC14MFP_USCI0_CTL0)   /*!< Set PC14 function to USCI0_CTL0    */
#define SET_USCI0_CTL0_PD4()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD4MFP_Msk)) | SYS_GPD_MFPL_PD4MFP_USCI0_CTL0)     /*!< Set PD4 function to USCI0_CTL0     */
#define SET_USCI0_CTL0_PE6()    SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE6MFP_Msk)) | SYS_GPE_MFPL_PE6MFP_USCI0_CTL0)     /*!< Set PE6 function to USCI0_CTL0     */
#define SET_USCI0_CTL1_PA8()    SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA8MFP_Msk)) | SYS_GPA_MFPH_PA8MFP_USCI0_CTL1)     /*!< Set PA8 function to USCI0_CTL1     */
#define SET_USCI0_CTL1_PB15()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_USCI0_CTL1)   /*!< Set PB15 function to USCI0_CTL1    */
#define SET_USCI0_CTL1_PD3()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD3MFP_Msk)) | SYS_GPD_MFPL_PD3MFP_USCI0_CTL1)     /*!< Set PD3 function to USCI0_CTL1     */
#define SET_USCI0_CTL1_PE5()    SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE5MFP_Msk)) | SYS_GPE_MFPL_PE5MFP_USCI0_CTL1)     /*!< Set PE5 function to USCI0_CTL1     */
#define SET_USCI0_CTL1_PF0()    SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_USCI0_CTL1)     /*!< Set PF0 function to USCI0_CTL1     */
#define SET_USCI0_DAT0_PA10()   SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA10MFP_Msk)) | SYS_GPA_MFPH_PA10MFP_USCI0_DAT0)   /*!< Set PA10 function to USCI0_DAT0    */
#define SET_USCI0_DAT0_PB13()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_USCI0_DAT0)   /*!< Set PB13 function to USCI0_DAT0    */
#define SET_USCI0_DAT0_PB8()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_USCI0_DAT0)     /*!< Set PB8 function to USCI0_DAT0     */
#define SET_USCI0_DAT0_PD1()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD1MFP_Msk)) | SYS_GPD_MFPL_PD1MFP_USCI0_DAT0)     /*!< Set PD1 function to USCI0_DAT0     */
#define SET_USCI0_DAT0_PE3()    SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE3MFP_Msk)) | SYS_GPE_MFPL_PE3MFP_USCI0_DAT0)     /*!< Set PE3 function to USCI0_DAT0     */
#define SET_USCI0_DAT1_PA9()    SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA9MFP_Msk)) | SYS_GPA_MFPH_PA9MFP_USCI0_DAT1)     /*!< Set PA9 function to USCI0_DAT1     */
#define SET_USCI0_DAT1_PB14()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_USCI0_DAT1)   /*!< Set PB14 function to USCI0_DAT1    */
#define SET_USCI0_DAT1_PB9()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | SYS_GPB_MFPH_PB9MFP_USCI0_DAT1)     /*!< Set PB9 function to USCI0_DAT1     */
#define SET_USCI0_DAT1_PD2()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD2MFP_Msk)) | SYS_GPD_MFPL_PD2MFP_USCI0_DAT1)     /*!< Set PD2 function to USCI0_DAT1     */
#define SET_USCI0_DAT1_PE4()    SYS->GPE_MFPL = ((SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE4MFP_Msk)) | SYS_GPE_MFPL_PE4MFP_USCI0_DAT1)     /*!< Set PE4 function to USCI0_DAT1     */
#define SET_USCI0_DAT1_PF1()    SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_USCI0_DAT1)     /*!< Set PF1 function to USCI0_DAT1     */
#define SET_USCI1_CLK_PB1()     SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_USCI1_CLK)      /*!< Set PB1 function to USCI1_CLK      */
#define SET_USCI1_CLK_PB8()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_USCI1_CLK)      /*!< Set PB8 function to USCI1_CLK      */
#define SET_USCI1_CLK_PC4()     SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_USCI1_CLK)      /*!< Set PC4 function to USCI1_CLK      */
#define SET_USCI1_CLK_PD7()     SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD7MFP_Msk)) | SYS_GPD_MFPL_PD7MFP_USCI1_CLK)      /*!< Set PD7 function to USCI1_CLK      */
#define SET_USCI1_CLK_PE13()    SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE13MFP_Msk)) | SYS_GPE_MFPH_PE13MFP_USCI1_CLK)    /*!< Set PE13 function to USCI1_CLK     */
#define SET_USCI1_CTL0_PB10()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB10MFP_Msk)) | SYS_GPB_MFPH_PB10MFP_USCI1_CTL0)   /*!< Set PB10 function to USCI1_CTL0    */
#define SET_USCI1_CTL0_PB5()    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_USCI1_CTL0)     /*!< Set PB5 function to USCI1_CTL0     */
#define SET_USCI1_CTL0_PC0()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk)) | SYS_GPC_MFPL_PC0MFP_USCI1_CTL0)     /*!< Set PC0 function to USCI1_CTL0     */
#define SET_USCI1_CTL0_PD3()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD3MFP_Msk)) | SYS_GPD_MFPL_PD3MFP_USCI1_CTL0)     /*!< Set PD3 function to USCI1_CTL0     */
#define SET_USCI1_CTL0_PE9()    SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE9MFP_Msk)) | SYS_GPE_MFPH_PE9MFP_USCI1_CTL0)     /*!< Set PE9 function to USCI1_CTL0     */
#define SET_USCI1_CTL1_PB4()    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_USCI1_CTL1)     /*!< Set PB4 function to USCI1_CTL1     */
#define SET_USCI1_CTL1_PB9()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | SYS_GPB_MFPH_PB9MFP_USCI1_CTL1)     /*!< Set PB9 function to USCI1_CTL1     */
#define SET_USCI1_CTL1_PC1()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk)) | SYS_GPC_MFPL_PC1MFP_USCI1_CTL1)     /*!< Set PC1 function to USCI1_CTL1     */
#define SET_USCI1_CTL1_PD4()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD4MFP_Msk)) | SYS_GPD_MFPL_PD4MFP_USCI1_CTL1)     /*!< Set PD4 function to USCI1_CTL1     */
#define SET_USCI1_CTL1_PF2()    SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_USCI1_CTL1)     /*!< Set PF2 function to USCI1_CTL1     */
#define SET_USCI1_DAT0_PB2()    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SYS_GPB_MFPL_PB2MFP_USCI1_DAT0)     /*!< Set PB2 function to USCI1_DAT0     */
#define SET_USCI1_DAT0_PB7()    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_USCI1_DAT0)     /*!< Set PB7 function to USCI1_DAT0     */
#define SET_USCI1_DAT0_PC2()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk)) | SYS_GPC_MFPL_PC2MFP_USCI1_DAT0)     /*!< Set PC2 function to USCI1_DAT0     */
#define SET_USCI1_DAT0_PD5()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD5MFP_Msk)) | SYS_GPD_MFPL_PD5MFP_USCI1_DAT0)     /*!< Set PD5 function to USCI1_DAT0     */
#define SET_USCI1_DAT0_PE10()   SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE10MFP_Msk)) | SYS_GPE_MFPH_PE10MFP_USCI1_DAT0)   /*!< Set PE10 function to USCI1_DAT0    */
#define SET_USCI1_DAT1_PB3()    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SYS_GPB_MFPL_PB3MFP_USCI1_DAT1)     /*!< Set PB3 function to USCI1_DAT1     */
#define SET_USCI1_DAT1_PB6()    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk)) | SYS_GPB_MFPL_PB6MFP_USCI1_DAT1)     /*!< Set PB6 function to USCI1_DAT1     */
#define SET_USCI1_DAT1_PC3()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_USCI1_DAT1)     /*!< Set PC3 function to USCI1_DAT1     */
#define SET_USCI1_DAT1_PD6()    SYS->GPD_MFPL = ((SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD6MFP_Msk)) | SYS_GPD_MFPL_PD6MFP_USCI1_DAT1)     /*!< Set PD6 function to USCI1_DAT1     */
#define SET_USCI1_DAT1_PE11()   SYS->GPE_MFPH = ((SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE11MFP_Msk)) | SYS_GPE_MFPH_PE11MFP_USCI1_DAT1)   /*!< Set PE11 function to USCI1_DAT1    */
#define SET_USCI2_CLK_PB15()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_USCI2_CLK)    /*!< Set PB15 function to USCI2_CLK     */
#define SET_USCI2_CTL0_PA12()   SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA12MFP_Msk)) | SYS_GPA_MFPH_PA12MFP_USCI2_CTL0)   /*!< Set PA12 function to USCI2_CTL0    */
#define SET_USCI2_CTL1_PA13()   SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA13MFP_Msk)) | SYS_GPA_MFPH_PA13MFP_USCI2_CTL1)   /*!< Set PA13 function to USCI2_CTL1    */
#define SET_USCI2_DAT0_PA15()   SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA15MFP_Msk)) | SYS_GPA_MFPH_PA15MFP_USCI2_DAT0)   /*!< Set PA15 function to USCI2_DAT0    */
#define SET_USCI2_DAT1_PA14()   SYS->GPA_MFPH = ((SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA14MFP_Msk)) | SYS_GPA_MFPH_PA14MFP_USCI2_DAT1)   /*!< Set PA14 function to USCI2_DAT1    */
#define SET_USCI3_CLK_PB8()     SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | SYS_GPB_MFPH_PB8MFP_USCI3_CLK)      /*!< Set PB8 function to USCI3_CLK      */
#define SET_USCI3_CLK_PC10()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk)) | SYS_GPC_MFPH_PC10MFP_USCI3_CLK)    /*!< Set PC10 function to USCI3_CLK     */
#define SET_USCI3_CTL0_PB7()    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_USCI3_CTL0)     /*!< Set PB7 function to USCI3_CTL0     */
#define SET_USCI3_CTL0_PC9()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk)) | SYS_GPC_MFPH_PC9MFP_USCI3_CTL0)     /*!< Set PC9 function to USCI3_CTL0     */
#define SET_USCI3_CTL1_PB1()    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_USCI3_CTL1)     /*!< Set PB1 function to USCI3_CTL1     */
#define SET_USCI3_CTL1_PB11()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_USCI3_CTL1)   /*!< Set PB11 function to USCI3_CTL1    */
#define SET_USCI3_DAT0_PB10()   SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB10MFP_Msk)) | SYS_GPB_MFPH_PB10MFP_USCI3_DAT0)   /*!< Set PB10 function to USCI3_DAT0    */
#define SET_USCI3_DAT0_PC12()   SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk)) | SYS_GPC_MFPH_PC12MFP_USCI3_DAT0)   /*!< Set PC12 function to USCI3_DAT0    */
#define SET_USCI3_DAT1_PB9()    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | SYS_GPB_MFPH_PB9MFP_USCI3_DAT1)     /*!< Set PB9 function to USCI3_DAT1     */
#define SET_USCI3_DAT1_PC11()   SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk)) | SYS_GPC_MFPH_PC11MFP_USCI3_DAT1)   /*!< Set PC11 function to USCI3_DAT1    */
#define SET_USCI4_CLK_PA7()     SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA7MFP_Msk)) | SYS_GPA_MFPL_PA7MFP_USCI4_CLK)      /*!< Set PA7 function to USCI4_CLK      */
#define SET_USCI4_CTL0_PA6()    SYS->GPA_MFPL = ((SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA6MFP_Msk)) | SYS_GPA_MFPL_PA6MFP_USCI4_CTL0)     /*!< Set PA6 function to USCI4_CTL0     */
#define SET_USCI4_CTL1_PC8()    SYS->GPC_MFPH = ((SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk)) | SYS_GPC_MFPH_PC8MFP_USCI4_CTL1)     /*!< Set PC8 function to USCI4_CTL1     */
#define SET_USCI4_CTL1_PF5()    SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_USCI4_CTL1)     /*!< Set PF5 function to USCI4_CTL1     */
#define SET_USCI4_DAT0_PC7()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC7MFP_Msk)) | SYS_GPC_MFPL_PC7MFP_USCI4_DAT0)     /*!< Set PC7 function to USCI4_DAT0     */
#define SET_USCI4_DAT0_PD11()   SYS->GPD_MFPH = ((SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD11MFP_Msk)) | SYS_GPD_MFPH_PD11MFP_USCI4_DAT0)   /*!< Set PD11 function to USCI4_DAT0    */
#define SET_USCI4_DAT1_PC6()    SYS->GPC_MFPL = ((SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC6MFP_Msk)) | SYS_GPC_MFPL_PC6MFP_USCI4_DAT1)     /*!< Set PC6 function to USCI4_DAT1     */
#define SET_X32_IN_PF5()        SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_X32_IN)         /*!< Set PF5 function to X32_IN         */
#define SET_X32_OUT_PF4()       SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) | SYS_GPF_MFPL_PF4MFP_X32_OUT)        /*!< Set PF4 function to X32_OUT        */
#define SET_XT1_IN_PF3()        SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN)         /*!< Set PF3 function to XT1_IN         */
#define SET_XT1_OUT_PF2()       SYS->GPF_MFPL = ((SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT)        /*!< Set PF2 function to XT1_OUT        */


/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  */
#define SYS_CLEAR_BOD_INT_FLAG()    (SYS->BODSTS |= SYS_BODSTS_BODIF_Msk)

/**
  * @brief      Set Brown-out detector function to low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_SET_BOD_LPM() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk; \
    }while(0)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_CLEAR_BOD_LPM() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk; \
    }while(0)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD() \
   do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk; \
    }while(0)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD() \
   do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL |= SYS_BODCTL_BODEN_Msk; \
    }while(0)

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
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD_RST() \
   do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL &= ~SYS_BODCTL_BODRSTEN_Msk; \
    }while(0)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD_RST() \
   do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk; \
    }while(0)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_2_4V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_4_4V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LEVEL(u32Level) \
   do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level); \
    }while(0)

/**
  * @brief      Set Brown-out detector wake-up selection
  * @param[in]  u32Wakeup is Brown-out detector wake-up selection. Including :
  *             - \ref SYS_BODCTL_BODWKEN_DISABLE
  *             - \ref SYS_BODCTL_BODWKEN_RISE
  *             - \ref SYS_BODCTL_BODWKEN_DROP
  *             - \ref SYS_BODCTL_BODWKEN_BOTH
  * @return     None
  * @details    This macro set Brown-out detector wake-up selection.
  *             The write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_SET_BOD_WAKEUP(u32Wakeup) \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODWKEN_Msk) | (u32Wakeup); \
    }while(0)

/**
* @brief      Set Brown-out detector output de-glitch time selection
* @param[in]  u32Wakeup is Brown-out detector wake-up selection. Including :
*             - \ref SYS_BODCTL_BODDGSEL_LIRC
*             - \ref SYS_BODCTL_BODDGSEL_4_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_8_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_16_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_32_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_64_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_128_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_256_SCLK
* @return     None
* @details    This macro set Brown-out detector output de-glitch time selection.
*             The write-protection function should be disabled before using this macro.
* \hideinitializer
*/
#define SYS_SET_BODDGSEL(u32Select) \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODDGSEL_Msk) | (u32Select); \
    }while(0)

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
  * @details    This macr` get previous reset source is from window watch dog reset.
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)

/**
  * @brief      Get reset source is from CPU lockup reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU lockup reset
  * @retval     >=1 Previous reset source is from CPU lockup reset
  * @details    This macro get previous reset source is from CPU lockup reset.
  * \hideinitializer
  */
#define SYS_IS_CPULK_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPULKRF_Msk)

/**
  * @brief      Disable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_LVR() \
   do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL &= ~SYS_BODCTL_LVREN_Msk; \
    }while(0)

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_LVR() \
   do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL |= SYS_BODCTL_LVREN_Msk; \
    }while(0)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_POR()               (SYS->PORCTL = 0x5AA55AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_POR()                (SYS->PORCTL = 0)

/**
  * @brief      Wait SYS_BODCTL Write Busy Flag
  * @param      None
  * @return     None
  * @details    This macro waits SYS_BODCTL write busy flag is cleared and skips when time-out.
  */
#define SYS_WAIT_BODCTL_WRBUSY() \
    do{ \
        uint32_t u32TimeOutCnt = SYS_TIMEOUT; \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk) \
        { \
            if(--u32TimeOutCnt == 0) break; \
        } \
    }while(0)

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

/**
  * @brief      Enable SYS Wake-up Interrupt
  * @details    This macro enables SYS wake-up Interrupt
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_WKINT()      (SYS->INTEN |= SYS_INTEN_PDWKIEN_Msk)

/**
  * @brief      Disable SYS Wake-up Interrupt
  * @details    This macro disables SYS wake-up Interrupt
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_WKINT()     (SYS->INTEN &= ~SYS_INTEN_PDWKIEN_Msk)


/*----------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                  */
/*----------------------------------------------------------------------------------------------------------*/
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
    uint32_t u32TimeOutCount = SYS_TIMEOUT;

    do
    {
        SYS->REGLCTL = 0x59UL;
        SYS->REGLCTL = 0x16UL;
        SYS->REGLCTL = 0x88UL;

        if (--u32TimeOutCount == 0) break;
    } while(SYS->REGLCTL == 0UL);
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
void     SYS_SRAM_ECC_Enable(uint32_t u32Bank);
void     SYS_SRAM_ECC_Disable(uint32_t u32Bank);
void     SYS_SRAM_ECC_EnableInt(uint32_t u32Bank, uint32_t u32Mask);
void     SYS_SRAM_ECC_DisableInt(uint32_t u32Bank, uint32_t u32Mask);
void     SYS_SRAM_ECC_ClearIntFlag(uint32_t u32Bank, uint32_t u32Mask);

/**@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */
/**@}*/ /* end of group SYS_Driver */
/**@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __SYS_H__ */
