/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use DMIC_VAD to wake up system from Power-down mode periodically.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/*
 * This sample uses internal RC as APLL0 clock source and UART to print messages.
 * Users may need to do extra system configuration according to their system design.
 *
 * Debug UART
 *   system_M55M1.c has three weak functions as below to configure debug UART port.
 *     SetDebugUartMFP, SetDebugUartCLK and InitDebugUart
 *   Users can re-implement these functions according to system design.
 */
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"


// Sampling rate in Hertz for voice activity detection.
#define VAD_DETECT_SAMPLERATE      (8000)

// Number of stability checks before entering power-down mode.
#define VAD_STABLECNT      (100)

// Stores power-down wake-up source flags.
static volatile uint32_t g_u32PDWK;

// Enable the VAD0 hardware to start voice activity detection.
void VAD_Start(void)
{
    DMIC_VAD_ENABLE(VAD0);
}

// Disable the VAD0 hardware and its interrupt to stop detection.
void VAD_Stop(void)
{
    DMIC_VAD_DISABLE(VAD0);
    NVIC_DisableIRQ(DMIC0VAD_IRQn);
}


// Step-by-step configuration of voice activity detection parameters.
void VAD_Init(void)
{
    // Structure for BIQ filter coefficients
    DMIC_VAD_BIQ_T sBIQCoeff;

    // Set downsample ratio to 64 to reduce data rate for VAD processing
    DMIC_VAD_SET_DOWNSAMPLE(VAD0, DMIC_VAD_DOWNSAMPLE_64);

    // Set detection sample rate (Hz) for VAD algorithm
    printf("VAD SampleRate is %d\n", DMIC_VAD_SetSampleRate(VAD0, VAD_DETECT_SAMPLERATE));

    // Assign BIQ filter coefficients for band-in-band IIR filter: A1, A2, B0, B1, B2
    sBIQCoeff.u16BIQCoeffA1 = 0xC174;
    sBIQCoeff.u16BIQCoeffA2 = 0x1F23;
    sBIQCoeff.u16BIQCoeffB0 = 0x016B;
    sBIQCoeff.u16BIQCoeffB1 = 0xFE0C;
    sBIQCoeff.u16BIQCoeffB2 = 0x008E;

    // Apply BIQ coefficients to VAD hardware
    DMIC_VAD_SetBIQCoeff(VAD0, &sBIQCoeff);
    // Enable the BIQ filter for improved noise suppression
    DMIC_VAD_ENABLE_BIQ(VAD0);
    // Configure short-term energy attack time to 2 ms (fast reaction to speech energy changes)
    DMIC_VAD_SET_STAT(VAD0, DMIC_VAD_STAT_2MS);
    // Configure long-term energy attack time to 64 ms (tracking background noise energy)
    DMIC_VAD_SET_LTAT(VAD0, DMIC_VAD_LTAT_64MS);

    // Configure power thresholds for speech detection
    // Short-term power threshold: 0 dB (max variation required not to trigger activity)
    DMIC_VAD_SET_STTHRE(VAD0, DMIC_VAD_POWERTHRE_0DB);
    // Long-term power threshold: -90 dB
    DMIC_VAD_SET_LTTHRE(VAD0, DMIC_VAD_POWERTHRE_M90DB);
    // Deviation threshold: 0 dB (max variation required not to trigger activity)
    DMIC_VAD_SET_DEVTHRE(VAD0, DMIC_VAD_POWERTHRE_0DB);

    // Disable VAD interrupt until system is fully initialized and ready
    NVIC_DisableIRQ(DMIC0VAD_IRQn);
}

// Perform stability checks by ensuring deviation and short-term power stay below thresholds.
// Parameters:
//   u32StableCount - number of stable readings required before proceeding.
void VAD_WaitStable(uint32_t u32StableCount)
{
    // Repeat stability check u32StableCount times
    while (u32StableCount--)
    {
        // Wait until deviation and short-term power drop below the -60dB threshold
        while ((DMIC_VAD_GET_DEV(VAD0) > DMIC_VAD_POWERTHRE_M60DB) || (DMIC_VAD_GET_STP(VAD0) > DMIC_VAD_POWERTHRE_M60DB))
        {
            // busy-wait: drop into low-level loop until stable
        }

        // Clear any pending active flags to reset detection before next check
        while (DMIC_VAD_IS_ACTIVE(VAD0))
        {
            DMIC_VAD_CLR_ACTIVE(VAD0);
        }
    }
}

// Handle VAD interrupt, clear active flag and wait for hardware to clear.
NVT_ITCM void DMIC0VAD_IRQHandler()
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    DMIC_VAD_CLR_ACTIVE(VAD0);

    __DSB();
    __ISB();

    while (DMIC_VAD_IS_ACTIVE(VAD0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for VAD0 IntFlag time-out!\n");
            break;
        }
    }
}

// Handle power-down wake-up interrupt and clear wake-up flags.
NVT_ITCM void PMC_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    g_u32PDWK = PMC_GetPMCWKSrc();

    /* check power down wakeup flag */
    if (g_u32PDWK & PMC_INTSTS_PDWKIF_Msk)
    {
        PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;

        __DSB();
        __ISB();

        while (PMC_GetPMCWKSrc() & PMC_INTSTS_PDWKIF_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for PMC IntFlag time-out!\n");
                break;
            }
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    // Unlock protected registers to allow configuration changes
    SYS_UnlockReg();
    // Enable power-down wake-up interrupt in PMC
    PMC_ENABLE_WKINT();
    // Enable PMC IRQ in NVIC to handle wake-up events
    NVIC_EnableIRQ(PMC_IRQn);
    // Switch system clock source to internal HIRC for low-power operation
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_HIRC, CLK_APLLCTL_APLLSRC_HIRC, 0);
    // Wait until all debug UART transmissions have completed
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    // Keep MIRC and HIRC oscillators active during power-down
    PMC_DISABLE_AOCKPD();
    // Configure and set power-down mode to PL1
    PMC_SetPowerDownMode(PMC_NPD0, PMC_PLCTL_PLSEL_PL1);

    // Lower VAD short-term power threshold to -60 dB to avoid false triggers
    DMIC_VAD_SET_STTHRE(VAD0, DMIC_VAD_POWERTHRE_M60DB);
    // Lower VAD deviation threshold to -60 dB for stability
    DMIC_VAD_SET_DEVTHRE(VAD0, DMIC_VAD_POWERTHRE_M60DB);
    // Wait for VAD signal to stabilize before entering power-down
    VAD_WaitStable(VAD_STABLECNT);
    // Enable VAD interrupt to wake system on voice detection
    NVIC_EnableIRQ(DMIC0VAD_IRQn);
    // Enter power-down mode; CPU sleeps until a wake-up interrupt
    PMC_PowerDown();
    // Upon wake-up, restore system clock to APLL0 at 220 MHz
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
    // Lock protected registers to secure configuration
    SYS_LockReg();
}

// Initialize system clocks, debug UART, DMIC module, and I/O multifunction pins.
static void SYS_Init(void)
{
    // Unlock protected registers to allow configuration changes
    SYS_UnlockReg();

    // Configure system clock to 220 MHz via PLL0 sourced from HIRC
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    // Update SystemCoreClock variable to match the current core frequency
    SystemCoreClockUpdate();

    // Enable clock for debug UART module to allow printf outputs
    SetDebugUartCLK();

    // Configure DMIC module clock source to HIRC for VAD operation
    CLK_SetModuleClock(DMIC0_MODULE, CLK_DMICSEL_DMIC0SEL_HIRC, 0UL);
    // Configure VAD module clock source to HIRC
    CLK_SetModuleClock(VAD0SEL_MODULE, CLK_DMICSEL_VAD0SEL_HIRC, 0UL);
    // Enable DMIC_VAD module clock
    CLK_EnableModuleClock(VAD0SEL_MODULE);
    // Reset DMIC0 module to ensure a clean initial state
    SYS_ResetModule(SYS_DMIC0RST);

    // Configure multi-function pins for debug UART
    SetDebugUartMFP();

    // Enable GPIOB module clock for DMIC pin multiplexing
    CLK_EnableModuleClock(GPIOB_MODULE);
    // Set DMIC0 data pin to PB.5
    SET_DMIC0_DAT_PB5();
    // Set DMIC0 clock pin to PB.6
    SET_DMIC0_CLKLP_PB6();
    // Set system clock output pin to PC.13 for external clock monitoring
    SET_CLKO_PC13();
    // Enable clock output on the configured pin with no division
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_SYSCLK, 0, 1);

    // Lock protected registers to prevent unintended modifications
    SYS_LockReg();
}

// Entry point; initialize system, start VAD, enter power-down, and report wake-up status.
int main(void)
{
    // Initialize system clocks and multi-function I/O pins
    SYS_Init();

    // Initialize debug UART (115200-8N1) for console output
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    // Initialize semihosting for debug prints when applicable
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("DMIC_VAD power down/wake up sample code\n");

    // Configure VAD hardware parameters and start detection
    VAD_Init();
    VAD_Start();
    // Clear previous wake-up source flags
    g_u32PDWK = 0;

    printf(" Press any key to Enter Power-down !\n");
    getchar();

    // Ensure background noise is stable before entering power-down
    VAD_WaitStable(VAD_STABLECNT);
    printf("Enter Power-down !\n");
    // Enter low-power mode; CPU will wake on VAD or PMC interrupt
    PowerDownFunction();

    // Poll for power-down wake-up event
    while (!g_u32PDWK) {}

    printf("Wake Up PASS\n");

    // Clear any residual VAD active flags after wake-up
    while (DMIC_VAD_IS_ACTIVE(VAD0))
    {
        DMIC_VAD_CLR_ACTIVE(VAD0);
    }

    // Stop VAD hardware and disable its interrupt
    VAD_Stop();

    // Loop indefinitely after wake-up handling
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
