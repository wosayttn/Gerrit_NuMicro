/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to update chip flash data through RS485 interface
 *           between chip RS485 and ISP Tool.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip RS485 and assign update file
 *           of Flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "targetdev.h"
#include "isp_user.h"
#include "uart_transfer.h"

#define nRTSPin         (PB8)
#define RECEIVE_MODE    (0)
#define TRANSMIT_MODE   (1)

int32_t SYS_Init(void)
{
    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable and configure module clock */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART1 RXD and TXD */
    SET_UART1_RXD_PB2();
    SET_UART1_TXD_PB3();

    PB->MODE = (PB->MODE & (~GPIO_MODE_MODE8_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE8_Pos);
    nRTSPin = RECEIVE_MODE;
    SET_GPIO_PB8();

    return 0;
}

int32_t main(void)
{
    int32_t i32TimeoutInMS = 300;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable ISP */
    FMC_Open();

    /* Init System, peripheral clock and multi-function I/O */
    if (SYS_Init() < 0)
        goto _APROM;

    /* Init UART */
    UART_Init();
    /* Enable APROM and Data Flash update */
    FMC_ENABLE_AP_UPDATE();
    FMC_ENABLE_DF_UPDATE();

    /* Get APROM and Data Flash size */
    g_u32ApromSize = GetApromSize();

    /* Wait for CMD_CONNECT command until Systick time-out */
    while (i32TimeoutInMS > 0)
    {
        /* Wait for CMD_CONNECT command */
        if ((g_u8bufhead >= 4) || (g_u8bUartDataReady == TRUE))
        {
            uint32_t u32lcmd;
            u32lcmd = inpw((uint32_t)g_au8uart_rcvbuf);

            if (u32lcmd == CMD_CONNECT)
            {
                goto _ISP;
            }
            else
            {
                g_u8bUartDataReady = FALSE;
                g_u8bufhead = 0;
            }
        }

        CLK_SysTickDelay(1000);
        i32TimeoutInMS--;
    }

    /* Timeout and go to APROM */
    if (i32TimeoutInMS == 0)
        goto _APROM;

_ISP:

    /* Parse command from master and send response back */
    while (1)
    {
        if (g_u8bUartDataReady == TRUE)
        {
            g_u8bUartDataReady = FALSE;         /* Reset UART data ready flag */
            ParseCmd(g_au8uart_rcvbuf, 64);     /* Parse command from master */
            NVIC_DisableIRQ(UART1_IRQn);        /* Disable NVIC */
            nRTSPin = TRANSMIT_MODE;            /* Control RTS in transmit mode */
            PutString();                        /* Send response to master */

            /* Wait for data transmission is finished */
            while ((UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);

            nRTSPin = RECEIVE_MODE;             /* Control RTS in receive mode */
            NVIC_EnableIRQ(UART1_IRQn);         /* Enable NVIC */
        }
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
