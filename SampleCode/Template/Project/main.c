/******************************************************************************
* @file     main.c
* @version  V1.00
* @brief    Template for NuMicro M3351
*
* SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

int32_t SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(144000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    return 0;
}

int32_t main(void)
{
    SYS_UnlockReg();
    SYS_Init();

#ifdef __PLDM_EMU__
    /* Configure UART Baudrate */
    DEBUG_PORT->LINE = (UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1);
    // The setting is for Palladium
    DEBUG_PORT->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(153600, 38400));
#else
    /* Init UART to 115200-8n1 for print message */
    UART_Open(DEBUG_PORT, 115200);
#endif

    printf("Test project\n");
    printf("System clock: %d Hz\n", SystemCoreClock);
    
#if defined (__TUBE_PRINTF__)
    /* Stop simulation */
    //outp32(0x9FF04400, 4);
#endif
    int8_t ch;
    while(1)
    {
        ch = getchar();
        printf("You pressed: %c\n", ch);
    };
}
