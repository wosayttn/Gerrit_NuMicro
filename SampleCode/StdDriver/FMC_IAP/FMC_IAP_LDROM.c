/******************************************************************************
* @file    main.c
* @version V1.00
* @brief   IAP boot from LDROM sample code
*
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
/* Empty function to reduce code size for GCC */
void ProcessHardFault(uint32_t *pu32StackFrame) {}
#endif

typedef void (*PFN_FUNC_PTR)(void);

/* Simplified UART functions to reduce code size */
/**
 * @brief       Routine to send a char
 * @param[in]   ch Character to send to debug port.
 * @returns     Send value from UART debug port
 * @details     Send a target char to UART debug port .
 */
static void SendChar_ToUART(char cChar)
{
    while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    DEBUG_PORT->DAT = cChar;

    if (cChar == '\n')
    {
        while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

        DEBUG_PORT->DAT = '\r';
    }
}

/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while (1)
    {
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            return (DEBUG_PORT->DAT);
        }
    }
}

static void PutString(char *pcStr)
{
    while (*pcStr != '\0')
    {
        SendChar_ToUART(*pcStr++);
    }
}

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

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(ISP_MODULE);

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

int main()
{
    uint32_t     u32TimeOutCnt;     /* time-out counter */

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for print message */
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
    DEBUG_PORT->LINE = (UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1);
    DEBUG_PORT->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200));
#else
    InitDebugUart();
#endif

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    PutString("\n\n");
    PutString("+-------------------------------+\n");
    PutString("|      FMC IAP Sample Code      |\n");
    PutString("|         [LDROM code]          |\n");
    PutString("+-------------------------------+\n");

    SYS_UnlockReg();    /* Unlock protected registers */
    FMC_Open();         /* Enable FMC ISP function */

    PutString("\n\nPress any key to branch to APROM ...\n");
    GetChar();                         /* block on waiting for any one character input from UART */

    PutString("\n\nChange VECMAP and branch to APROM ...\n");
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!(DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk))        /* wait until UART TX FIFO is empty */
        if (--u32TimeOutCnt == 0) break;

    /*  NOTE!
     *     Before change VECMAP, user MUST disable all interrupts.
     */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);        /* Vector remap APROM page 0 to address 0. */

    if (g_FMC_i32ErrCode != 0)
    {
        PutString("FMC_SetVectorPageAddr(FMC_APROM_BASE) failed!\n");
        goto lexit;
    }

    SYS_LockReg();                                /* Lock protected registers */

    /* Software reset to boot to APROM */
    NVIC_SystemReset();

lexit:

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
