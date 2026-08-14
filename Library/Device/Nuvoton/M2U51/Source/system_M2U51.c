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
    uint32_t u32Freq, u32ClkSrc;
    uint32_t u32HclkDiv;

    u32ClkSrc = CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk;

    if (u32ClkSrc == CLK_CLKSEL0_HCLKSEL_MIRC)
    {
        u32Freq = CLK_GetMIRCFreq();
    }
    else
    {
        /* Use the clock sources directly */
        u32Freq = gau32ClkSrcTbl[u32ClkSrc];
    }
    u32HclkDiv = (CLK->HCLKDIV & CLK_HCLKDIV_HCLKDIV_Msk) + 1UL;

    /* Update System Core Clock */
    SystemCoreClock = u32Freq / u32HclkDiv;

    CyclesPerUs = (SystemCoreClock + 500000UL) / 1000000UL;

    if (CyclesPerUs == 0UL)
    {
        CyclesPerUs = 1UL;  /* avoid the SYSTICK cannot count to value */
    }
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
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Disable LDO adaptive bias feature */
    outp32(0x40000168, inp32(0x40000168) | BIT28);

    /* Set Band-gap LP mode wakeup mode to "Analog refresh and digital timeout" */
    outp32(0x40000180, inp32(0x40000180) | BIT30);
    
    /* Set the Band-gap to LP mode to save power consumption. */
    CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_NRBGLPEL_Msk)) | CLK_PMUCTL_NRBGLPEL_LP;

    /* All GPIO digital input path default value are disabled for M2U51. */
    /* Enable them by software when system startup. */
    /* Enable all GPIO clocks */
    CLK->AHBCLK0 |= 0xFF000000UL;
    /* Enable digital input path */
    PA->DINOFF = 0UL;
    PB->DINOFF = 0UL;
    PC->DINOFF = 0UL;
    PD->DINOFF = 0UL;
    PE->DINOFF = 0UL;
    PF->DINOFF = 0UL;
    PG->DINOFF = 0UL;
    PH->DINOFF = 0UL;
    /* Disable all GPIO clocks, returning them to their default value */
    CLK->AHBCLK0 &= ~0xFF000000UL;

    /* Lock protected registers */
    SYS_LockReg();
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
