/**************************************************************************//**
 * @file     system_M2U51.c
 * @version  V0.10
 * @brief    System Setting Source File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

//#include <arm_cmse.h>
#include <stdio.h>
#include <stdint.h>
#include "NuMicro.h"

extern void *__Vectors;                   /* see startup file */

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock  = __SYSTEM_CLOCK;     /*!< System Clock Frequency (Core Clock) */
uint32_t CyclesPerUs      = (__SYSTEM_CLOCK / 1000000UL);  /*!< Cycles per micro second             */
const uint32_t gau32ClkSrcTbl[8] = {__MIRC, __HIRC, __LIRC, __LXT, 0UL, 0UL, 0UL, 0UL};

/**
 * @brief    Update the Variable SystemCoreClock
 *
 * @param    None
 *
 * @return   None
 *
 * @details  This function is used to update the variable SystemCoreClock
 *           and must be called whenever the core clock is changed.
 */
void SystemCoreClockUpdate(void)
{

}


/**
 * @brief    System Initialization
 *
 * @param    None
 *
 * @return   None
 *
 * @details  The necessary initialization of system. Global variables are forbidden here.
 */
void SystemInit(void)
{

}


#if USE_ASSERT

/**
 * @brief      Assert Error Message
 *
 * @param[in]  file  the source file name
 * @param[in]  line  line number
 *
 * @return     None
 *
 * @details    The function prints the source file name and line number where
 *             the ASSERT_PARAM() error occurs, and then stops in an infinite loop.
 */
void AssertError(uint8_t *file, uint32_t line)
{
    printf("[%s] line %u : wrong parameters.\r\n", file, line);

    /* Infinite loop */
    while (1) ;
}
#endif

/**
 * @brief    Set UART0 Default MPF
 *
 * @param    None
 *
 * @return   None
 *
 * @details  The initialization of uart0 default multi function pin.
 */
#if 1
#if defined( __ICCARM__ )
__WEAK
#else
__attribute__((weak))
#endif
void Uart0DefaultMPF(void)
{
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_UART0_RXD;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB13MFP_Msk) | SYS_GPB_MFPH_PB13MFP_UART0_TXD;
}
#endif
