/**************************************************************************//**
 * @file     system_M2003.h
 * @version  V0.10
 * @brief    System Setting Header File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

#ifndef __SYSTEM_M2003_H__
#define __SYSTEM_M2003_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Macro Definition                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef DEBUG_PORT
#define DEBUG_PORT      UART0       /*!< Select Debug Port which is used for retarget.c to output debug message to UART */
#endif

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __LIRC      (10000UL)       /*!< Internal 10KHz RC Oscillator Frequency */

#if (CHIP_TYPE == CHIP_TYPE_M2003G)
  #if (M2003G_HIRC == M2003G_HIRC_32MHZ)
    #define __HIRC      (32000000UL)    /*!< Internal 32M RC Oscillator Frequency */
  #elif (M2003G_HIRC == M2003G_HIRC_24MHZ)
    #define __HIRC      (24000000UL)    /*!< Internal 24M RC Oscillator Frequency */
  #else
    #define __HIRC      (40000000UL)    /*!< Internal 40M RC Oscillator Frequency */
  #endif
    #define __HXT       (32000000UL)    /*!< External Crystal Clock Frequency     */
#elif (CHIP_TYPE == CHIP_TYPE_M2003E)
    #define __HIRC      (40000000UL)    /*!< Internal 40M RC Oscillator Frequency */
    #define __HXT       (32000000UL)    /*!< External Crystal Clock Frequency     */
#else   /* for CHIP_TYPE_M2003C */
    #define __HIRC      (24000000UL)    /*!< Internal 24M RC Oscillator Frequency */
    #define __HXT       (       0UL)    /*!< External Crystal Clock Frequency     */
#endif


extern uint32_t SystemCoreClock;    /*!< System Clock Frequency (Core Clock)  */
extern uint32_t CyclesPerUs;        /*!< Cycles per micro second              */
extern uint32_t PllClock;           /*!< PLL Output Clock Frequency           */

/**
  \brief Exception / Interrupt Handler Function Prototype
*/
typedef void(*VECTOR_TABLE_Type)(void);


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the micro controller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit(void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate(void);

/**
 * Set UART0 default multi function pin
 *
 * @param  none
 * @return none
 *
 * @brief  The initialization of uart0 default multiple-function pin.
 */
extern void Uart0DefaultMPF(void);

/**
 * Check if debug message finished
 *
 * @param    None
 *
 * @retval   1: Message is finished
 * @retval   0: Message is transmitting.
 *
 * @details  Check if message finished (FIFO empty of debug port)
 */
extern int IsDebugFifoEmpty(void);


#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_M2003_H__ */
