/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrates how to initialize the I3C Controller and perform operations to the I3C Target.
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
void SYS_Init(void);
void UART_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART_Init();
    
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------+\n");
    printf("|    I3C Controller Sample Code    |\n");
    printf("+----------------------------------+\n\n");
    
    
    /* Set multi-function pins for I3C pin */
    GPIO_ENABLE_SCHMITT_TRIGGER(PA, (BIT0 | BIT1));
    SET_I3C0_SDA_PA0();
    SET_I3C0_SCL_PA1();    
    
    /* Use TYPE-A Resistance Connection */
    PA->PUSEL = ((GPIO_PUSEL_PULL_UP << (0 << 1)) | (GPIO_PUSEL_PULL_UP << (1 << 1)));
    SET_I3C0_PUPEN_PA2();
    
    printf("# I3C Controller Settings:\n");
    printf("    - SDA on PA.0\n");
    printf("    - SCL on PA.1\n");
    printf("    - PDMA %s\n", g_I3CDev.is_DMA? "enabled":"disabled");
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
    
    while(1)
    {
        /* Device has switched to I3C Target Role */
        I3C_TargetRole(&g_I3CDev, 0);
        
        /* Device has switched to I3C Controller Role */
        I3C_ControllerRole(&g_I3CDev, 0);
    }

    while(1) {}
}

void SYS_Init(void)
{
     /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 160MHz */
    CLK_SetCoreClock(160000000);
    
    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    
   /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}
