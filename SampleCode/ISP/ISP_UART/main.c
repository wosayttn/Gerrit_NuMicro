/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to update chip flash data through UART interface
 *           between chip UART and PC UART.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip UART and assign update file
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

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

int32_t main(void)
{
    uint32_t u32TimeoutInMS = 300;

    /* Init System, peripheral clock and multi-function I/O */
    if (SYS_Init() < 0)
        goto _APROM;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init UART */
    UART_Init();

    /* Enable ISP */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* Get APROM and Data Flash size */
    g_u32ApromSize = GetApromSize();

    /* Wait for CMD_CONNECT command until Systick time-out */
    while (u32TimeoutInMS > 0)
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
        u32TimeoutInMS--;
    }

    /* Timeout then go to APROM */
    if (u32TimeoutInMS == 0)
        goto _APROM;

_ISP:

    /* Parse command from master and send response back */
    while (1)
    {
        if (g_u8bUartDataReady == TRUE)
        {
            g_u8bUartDataReady = FALSE;         /* Reset UART data ready flag */
            ParseCmd(g_au8uart_rcvbuf, 64);     /* Parse command from master */
            PutString();                        /* Send response to master */
        }
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
