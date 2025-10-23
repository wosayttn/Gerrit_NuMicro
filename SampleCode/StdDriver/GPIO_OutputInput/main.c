/******************************************************************************
 * @file    main.c
 * @version V1.00
 * @brief   Show how to set GPIO pin mode and use pin data input/output control.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

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
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPB_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
    return 0;
}

int32_t main(void)
{
    int32_t i32Err, i32TimeOutCnt;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------------------+\n");
    printf("|    PB.3 (Output) and PB.2 (Input) Sample Code   |\n");
    printf("+-------------------------------------------------+\n\n");
    /*--------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin   */
    /*--------------------------------------------------------------------------*/
    printf("  >> Please connect PB.3 and PB.2 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();
    /* Configure PB.3 as Output mode and PB.2 as Input mode then close it */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    i32Err = 0;
    printf("GPIO PB.3 (output mode) connect to PB.2 (input mode) ......");
    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    /* Set PB.3 output pin value is low */
    PB3 = 0;
    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for PB.2 input pin status is low for a while */
    while (PB2 != 0)
    {
        if (i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Set PB.3 output pin value is high */
    PB3 = 1;
    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for PB.2 input pin status is high for a while */
    while (PB2 != 1)
    {
        if (i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Print test result */
    if (i32Err)
    {
        printf("  [FAIL].\n");
    }
    else
    {
        printf("  [OK].\n");
    }

    /* Configure PB.3 and PB.2 to default Quasi-bidirectional mode */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, BIT2, GPIO_MODE_QUASI);

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
