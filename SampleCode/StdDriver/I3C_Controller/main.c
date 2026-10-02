/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrates how to initialize the I3C Controller and perform operations to the I3C Target.
 *           This sample code needs to use two boards.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "i3c_DeviceFunc.h"

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
    /* Set multi-function pins for I3C pin */
    SET_I3C0_SDA_PA0();
    SET_I3C0_SCL_PA1();
    /* I3C pins enable schmitt trigger */
    CLK_EnableModuleClock(GPA_MODULE);
    GPIO_ENABLE_SCHMITT_TRIGGER(PA, (BIT0 | BIT1));
    /* Lock protected registers */
    SYS_LockReg();
    return 0;
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("System core clock: %d\n", SystemCoreClock);
    printf("+----------------------------------+\n");
    printf("|    I3C Controller Sample Code    |\n");
    printf("+----------------------------------+\n\n");
    printf("# I3C Controller Settings:\n");
    printf("    - SDA on PA.0\n");
    printf("    - SCL on PA.1\n");
    printf("    - PDMA %s\n", g_I3CDev.is_DMA ? "enabled" : "disabled");
    printf("# Supports:\n");
    printf("    - I2C FM+ Write/Read to I2C Target\n");
    printf("    - I3C Dynamic Address Assignment through ENTDAA CCC for I3C Targets\n");
    printf("        - Reset all Targets to I2C mode RSTDAA CCC\n");
    printf("    - SDR, HDR-DDR and HDR-BT Write/Read to I3C Target\n");
    printf("    - Common Command Codes\n");
    printf("    - Response In-Band Interrupt Request from Target\n");
    printf("    - Response Controller Request Request from Target\n");
    printf("# User can hit [Enter] to display the function menu.\n");
    printf("\n");
    printf("[ User needs to press [e] to send ENTDAA CCC for I3C Target Dynamic Address Assignment ]\n\n");
    /* Initialize the device as I3C Controller Role */
    I3C_ControllerRole(&g_I3CDev, 1);

    while (1)
    {
        /* Device has switched to I3C Target Role */
        I3C_TargetRole(&g_I3CDev, 0);
        /* Device has switched to I3C Controller Role */
        I3C_ControllerRole(&g_I3CDev, 0);
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
