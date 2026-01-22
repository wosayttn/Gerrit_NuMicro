/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to transfer data through UART by Xmodem.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "xmodem.h"

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

int32_t main(void)
{
    int32_t i32TranBytes;
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    InitDebugUart();

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|         Xmodem Sample Code             |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    printf("Waiting for Xmodem data transfer ...\n");

    /* Waiting for debug message print out */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk) == 0)
        if (--u32TimeOutCnt == 0) break;

    i32TranBytes = XmodemRecv(FMC_APROM_BANK1_BASE);

    if (i32TranBytes < 0)
    {
        printf("Xmodem transfer fail ! [0x%X]\n", i32TranBytes);
    }
    else
    {
        printf("\nXomdem transfer done.\n");
        printf("Total transfer size is %d\n", i32TranBytes);

        printf("Press any key to read back the transfer file.\n");
        getchar();
        printf("\nWaiting for receiver ...\n");


        i32TranBytes = XmodemSend((uint8_t *)FMC_APROM_BANK1_BASE, i32TranBytes);

        if (i32TranBytes < 0)
        {
            printf("Xmodem transfer fail ! Error code: %d\n", i32TranBytes);
        }
        else
        {
            printf("Transfer done.\nTotal trasnfer size is %d\n", i32TranBytes);
        }
    }

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
