/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to config semihost function to output and input character
 *          within IDE console window.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/*
 * This is a template semihost project for M3351 series MCU.
 * Users can create their own application based on this project.
 *
 * This template uses internal RC (HIRC) as APLL0 clock source and UART to print messages.
 * Users may need to do extra system configuration according to their system design.
 *
 * Debug UART
 *   system_M3351.c has three weak functions as below to configure DEBUG_PORT debug port.
 *     SetDebugUartMFP, SetDebugUartCLK and InitDebugUart
 *   Users can re-implement these functions according to system design.
 */
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /*
    This sample code is used to show how to print message/getchar on IDE debug environment.

    To enable semihost in KEIL MDK or IAR Workbench
        User must define "DEBUG_ENABLE_SEMIHOST" constant when building sample code.
        If defined DEBUG_ENABLE_SEMIHOST = 1 or 2 and ICE connected, the message will output to ICE.
        If defined DEBUG_ENABLE_SEMIHOST = 1 and ICE off line, the message will re-direct to UART debug port.
        If defined DEBUG_ENABLE_SEMIHOST = 2 and ICE off line, no any debug message output.

        In KEIL MDK, user need to open "View->Serial Window->UART #1" windows in debug mode.
        In IAR Workbench, user need to open "View->Terminal I/O" in debug mode.

        NOTE1: Hardfault_Handler is used for semihost. User cannot overwrite it when using semihost.
           If it is necessary to process hardfault, user can append code to ProcessHardfault of retarget.c
        NOTE2: Semihost only works with Nuvoton NuLink ICE Dongle in debug mode.
        NOTE3: The message will output to debug port if Nuvoton NuLink ICE Dongle is not connected.


        Semihost On/Off | NuLink Connected | Output Path
        ==============================================================
          1         |         1        |  ICE
          1         |         0        |  UART Debug Port / NULL when DEBUG_ENABLE_SEMIHOST=2
          0         |         1        |  UART Debug Port
          0         |         0        |  UART Debug Port
        --------------------------------------------------------------
    */

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\nStart SEMIHOST test: \n");

    while (1)
    {
        int8_t i8item;
        i8item = getchar();
        printf("%c\n", i8item);
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
