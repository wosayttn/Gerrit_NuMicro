/****************************************************************************
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to use OPA as a non-inverting amplifier.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable OPA module clock */
    CLK_EnableModuleClock(OPA_MODULE);

    /* Enable GPIO module clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select timer 0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PB.0 for OPA0_P0 and PB.2 for OPA0_OUT */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB2MFP_Msk)) |
                    (SYS_GPB_MFP0_PB2MFP_OPA0_O);
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB0MFP_Msk)) |
                    (SYS_GPB_MFP0_PB0MFP_OPA0_P0);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{
    uint8_t u8Option;
    uint32_t opa_gain;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* This sample code uses semihost as the debug port */
    printf("+---------------------------------------------------------------------------+\n");
    printf("|            M2L31 OPA Driver Sample Code                                   |\n");
    printf("+---------------------------------------------------------------------------+\n");
    printf("  This sample code demonstrates how to use OPA0 as a non-inverting amplifier.\n");
    printf("  The assigned pins for OPA0 are listed below.\n\n");
    printf("      OPA0_P0: PB.0  \n");
    printf("      OPA0_N0: unselected \n");
    printf("      OPA0_O : PB.2  \n");
    printf("+----------------------------------------------------------+\n");
    printf("  Hit any key to start!\n");
    getchar();

    /* Enable OPA */
    OPA_POWER_ON(OPA, 0);

    /* Set OPA0 non-inverting input from OPA0_P0 */
    OPA->MODE0 = (OPA->MODE0&~OPA_MODE_POSCHEN_Msk) | OPA_MODE_POSCHEN_P0;

    /* Set OPA0 inverting input from internal Vf_int */
    OPA->MODE0 = (OPA->MODE0&~OPA_MODE_NEGCHEN_Msk) | OPA_MODE_NEGCHEN_VF_INT;

    /* Set OPA0 resistor end switch to AVSS */
    OPA->MODE0 = (OPA->MODE0&~OPA_MODE_SWSEL_Msk) | OPA_MODE_SWSEL_AVSS;

    /* Enable OPA0 output */
    OPA_ENABLE_OUTPUT(OPA, 0);

    while(1)
    {
        printf("+----------------------------------------------------------+\n");
        printf("| OPA gain selecton (if voltage input is from OPA0_P0)     |\n");
        printf("+----------------------------------------------------------+\n");
        printf("| [0] open loop                                            |\n");
        printf("| [1] x1                                                   |\n");
        printf("| [2] x2                                                   |\n");
        printf("| [3] x4                                                   |\n");
        printf("| [4] x8                                                   |\n");
        printf("| [5] x16                                                  |\n");
        printf("| [6] x32                                                  |\n");
        printf("+----------------------------------------------------------+\n");
        u8Option = getchar();
        switch(u8Option)
        {
        default:
            printf("\n");
            printf("Incorrect selection, please try again.\n");
            printf("+--------------------------------------------------------+\n");
            continue;
        case '0':
            opa_gain = OPA_OPEN_LOOP;
            break;
        case '1':
            opa_gain = OPA_GAIN_1;
            break;
        case '2':
            opa_gain = OPA_GAIN_2;
            break;
        case '3':
            opa_gain = OPA_GAIN_4;
            break;
        case '4':
            opa_gain = OPA_GAIN_8;
            break;
        case '5':
            opa_gain = OPA_GAIN_16;
            break;
        case '6':
            opa_gain = OPA_GAIN_32;
            break;
        }

        OPA_SET_GAIN(OPA, 0, opa_gain);
        printf("\n");
        printf("Item [%d] is selected.\n", u8Option-0x30);
        printf("Please use an oscillator to probe OPA0 output (PB.2).     \n");
        printf("+--------------------------------------------------------+\n");
    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
