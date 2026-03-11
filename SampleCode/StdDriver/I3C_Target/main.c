/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrates how to initialize the I3C Target and response command from the I3C Controller.
 *           This sample code needs to use two boards.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "i3c_DeviceFunc.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t   g_RxBuf[I3C_DEVICE_RX_BUF_CNT], g_TxBuf[I3C_DEVICE_RX_BUF_CNT];

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 200MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_200MHZ);
    /* Use SystemCoreClockUpdate() to calculate and update SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable peripheral clock */
    CLK_EnableModuleClock(I3C0_MODULE);
    /* Set multi-function pins for I3C0 SDA and SCL */
    SET_I3C0_SCL_PB1();
    SET_I3C0_SDA_PB0();
    SYS_ResetModule(SYS_I3C0RST);
    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set SCL slew rate to GPIO_SLEWCTL_FAST0, SDA slew rate to GPIO_SLEWCTL_HIGH */
    GPIO_SetSlewCtl(PB, BIT1, GPIO_SLEWCTL_FAST0);
    GPIO_SetSlewCtl(PB, BIT0, GPIO_SLEWCTL_FAST0);
    /* I3C pins enable schmitt trigger */
    GPIO_ENABLE_SCHMITT_TRIGGER(PB, BIT0 | BIT1);
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t     ch;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("System core clock: %d\n", SystemCoreClock);
    printf("+------------------------------+\n");
    printf("|    I3C Target Sample Code    |\n");
    printf("+------------------------------+\n\n");
    printf("# I3C Target Settings:\n");
    printf("    - SDA on PB.0\n");
    printf("    - SCL on PB.1\n");
    printf("    - PDMA %s\n", g_I3CDev.is_DMA ? "enabled" : "disabled");
    printf("    - Static Address 0x%02x\n", g_I3CDev.main_target_sa);
    printf("# Supports:\n");
    printf("    - Response Dynamic Address Assignment (ENDTAA)\n");
    printf("    - Response Controller SDR, HDR-DDR and HDR-BT Write/Read operations\n");
    printf("    - Send In-Band Interrupt Request\n");
    printf("    - Send Controller Request\n");
    printf("        - Current Target will switch to as Controller Role\n");
    printf("# User can hit [Enter] to display the function menu.\n");
    printf("\n");

    printf("[0] I3C_SUPPORT_ENTDAA\n");
    printf("[1] I3C_SUPPORT_ADAPTIVE_HJ\n");
    printf("[2] I3C_SUPPORT_IMMEDIATE_HJ\n");
    printf("Select: ");

    while (1)
    {
        ch = getchar();
        printf("%c", ch);

        if (ch == '0')
        {
            g_DA_ASSIGNED_MODE = I3C_SUPPORT_ENTDAA;
            printf("\n[ Wait Controller to send ENTDAA CCC ]\n\n");
            break;
        }
        else if (ch == '1')
        {
            g_DA_ASSIGNED_MODE = I3C_SUPPORT_ADAPTIVE_HJ;
            printf("\n[ Initiate a Hot-Join request when a 7'h7E header on the bus ]\n\n");
            break;
        }
        else if (ch == '2')
        {
            g_DA_ASSIGNED_MODE = I3C_SUPPORT_IMMEDIATE_HJ;
            printf("\n[ Initiate a Hot-Join request immediately after I3C Target enabled ]\n\n");
            break;
        }
    }

    /* Initialize the device as I3C Target Role */
    I3C_TargetRole(&g_I3CDev, 1);

    while (1)
    {
        /* Device has switched to I3C Controller Role */
        I3C_ControllerRole(&g_I3CDev, 0);
        /* Device has switched to I3C Target Role */
        I3C_TargetRole(&g_I3CDev, 0);
    }
}

/*** (C) COPYRIGHT 2026 Nuvoton Technology Corp. ***/
