/**************************************************************************//**
 * @file     system_M2003.c
 * @version  V0.10
 * @brief    System Setting Source File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include "NuMicro.h"

extern void *__Vectors;                   /* see startup file */
extern const uint32_t gau32ClkSrcTbl[8];

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock  = __HIRC;              /*!< System Clock Frequency (Core Clock) */
uint32_t CyclesPerUs      = (__HIRC / 1000000UL);  /*!< Cycles per micro second             */
uint32_t PllClock         = __HIRC;              /*!< PLL Output Clock Frequency          */
const uint32_t gau32ClkSrcTbl[8] = {__HXT, 0UL, 0UL, __LIRC, 0UL, __HIRC, 0UL, 0UL};


/**
 * @brief    Update the Variable SystemCoreClock
 *
 * @note     No parameters.
 *
 *
 * @details  This function is used to update the variable SystemCoreClock
 *           and must be called whenever the core clock is changed.
 */
void SystemCoreClockUpdate(void)
{
    uint32_t u32Freq;
    uint32_t u32ClkSrc;
    uint32_t u32HclkDiv;

    u32ClkSrc = CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk;

    /* Use the clock sources directly */
    u32Freq = gau32ClkSrcTbl[u32ClkSrc];

    u32HclkDiv = (CLK->CLKDIV0 & CLK_CLKDIV0_HCLKDIV_Msk) + 1UL;

    /* Update System Core Clock */
    SystemCoreClock = u32Freq / u32HclkDiv;

    CyclesPerUs = (SystemCoreClock + 500000UL) / 1000000UL;
}

/**
 * @brief    Read UID on FMC.
 *
 * @note     No parameters.
 *
 *
 * @details  The necessary initialization of system. Global variables are forbidden here.
 */
static uint32_t Read_UID(uint32_t offset)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;      /* Set ISP Command Code */
    FMC->ISPADDR = offset;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;     /* Trigger to start ISP procedure */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}  /* Waiting for ISP Done */
    return FMC->ISPDAT;
}

/**
 * @brief    Load HIRC trim value for different HIRC frequency.
 *
 * @note     No parameters.
 *
 *
 * @details  The necessary initialization of system. Global variables are forbidden here.
 */
static void TrimHIRC(void)
{
    volatile uint32_t *pHIRCTrimBase = (volatile uint32_t *)(SYS_BASE + 0x110UL);

    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;

#if (__HIRC == FREQ_40MHZ)
    *pHIRCTrimBase = Read_UID(0x64);
#elif (__HIRC == FREQ_32MHZ)
    *pHIRCTrimBase = Read_UID(0x6C);
#elif (__HIRC == FREQ_24MHZ)
    *pHIRCTrimBase = Read_UID(0x68);
#endif
}

/**
 * @brief    System Initialization
 *
 * @note     No parameters.
 *
 *
 * @details  The necessary initialization of system. Global variables are forbidden here.
 */
void SystemInit(void)
{
	SYS_UnlockReg();
    CLK->IOPDCTL = CLK_IOPDCTL_IOHR_Msk;
    TrimHIRC();
	SYS_LockReg();
}


#if USE_ASSERT

/**
 * @brief      Assert Error Message
 *
 * @param[in]  file  the source file name
 * @param[in]  line  line number
 *
 *
 * @details    The function prints the source file name and line number where
 *             the ASSERT_PARAM() error occurs, and then stops in an infinite loop.
 */
void AssertError(uint8_t *file, uint32_t line)
{
    printf("[%s] line %u : wrong parameters.\r\n", file, line);

    /* Infinite loop */
    for (;;)
    {
        /* Wait for reset after an assertion failure. */
    }
}
#endif

/**
 * @brief    Set UART0 Default MPF
 *
 * @note     No parameters.
 *
 * @return   None
 *
 * @details  The initialization of uart0 default multi function pin.
 */
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
