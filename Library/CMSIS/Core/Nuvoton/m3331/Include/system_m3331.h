/*************************************************************************//**
 * @file     system_m3331.h
 * @brief    CMSIS-Core(M) Device Peripheral Access Layer Header File for
 *           Device m3331
 * @version  V1.0.1
 * @date     November 5, 2025
 *****************************************************************************/
/*
 * Copyright (c) 2009-2022 Arm Limited. All rights reserved.
 * Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef SYSTEM_M3331_H
#define SYSTEM_M3331_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

#ifndef __HSI
#define __HSI       (160000000UL)   /*!< PLL default output is 160MHz */
#endif

#ifndef __HXT
#define __HXT       (12000000UL)    /*!< External Crystal Clock Frequency */
#endif

#ifndef __LXT
#define __LXT       (32768UL)       /*!< External Crystal Clock Frequency 32.768KHz */
#endif

#ifndef __HIRC
# define __HIRC     (48000000UL)    /*!< Internal 48M RC Oscillator Frequency */
#endif

#define __LIRC      (38400UL)       /*!< Internal 38.4K RC Oscillator Frequency */

#ifndef DEBUG_PORT
#define DEBUG_PORT  UART0   /*!< Select Debug Port which is used for retarget.c to output debug message to UART */
#endif

#define NS_OFFSET   (0x10000000)

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3L)
# ifdef __NONSECURE_CODE
#  error "Cannot define __NONSECURE_CODE when build with CMSE compiler option"
# else
#  define __SECURE_CODE   1
# endif
#endif


#ifdef __SECURE_CODE
# if defined (__ICCARM__)
#  define __NONSECURE_ENTRY       __cmse_nonsecure_entry
#  define __NONSECURE_ENTRY_WEAK  __cmse_nonsecure_entry //__weak
#  define __NONSECURE_CALL        __cmse_nonsecure_call
# else
#  define __NONSECURE_ENTRY       __attribute__((cmse_nonsecure_entry))
#  define __NONSECURE_ENTRY_WEAK  __attribute__((cmse_nonsecure_entry,weak))
#  define __NONSECURE_CALL        __attribute__((cmse_nonsecure_call))
# endif
#else
# define __NONSECURE_ENTRY
# define __NONSECURE_ENTRY_WEAK
# define __NONSECURE_CALL
#endif


extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */
extern uint32_t CyclesPerUs;         /*!< Cycles per micro second              */
extern uint32_t PllClock;            /*!< PLL Output Clock Frequency           */

/**
  \brief Exception / Interrupt Handler Function Prototype
*/
typedef void(*VECTOR_TABLE_Type)(void);

/**
 * Return true if the code is running in secure state
 *
 * @param  none
 * @return True for secure state, false for non-secure state
 *
 * @brief  Check if the code is running in secure state
 */
#define IS_SECURE() ((__PC() & NS_OFFSET) == 0)


/**
 * Return current program counter
 *
 * @param  none
 * @return Currrent program counter value
 *
 * @brief  Get current program counter
 */
extern uint32_t __PC();

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



#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_M3331_H */
