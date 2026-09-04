/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show the usage of EQEI Unit Timer function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"




/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[2] = {0};


void EQEI0_IRQHandler(void)
{
    if(EQEI_GET_INT_FLAG(EQEI0, EQEI_STATUS_UTIEF_Msk))     /* EQEI Unit Timer Event flag */
    {
        EQEI_CLR_INT_FLAG(EQEI0, EQEI_STATUS_UTIEF_Msk);
        //printf("Unit TImer0 INT!\n\n");
        g_au32TMRINTCount[0]++;
    }

}

void EQEI1_IRQHandler(void)
{
    if(EQEI_GET_INT_FLAG(EQEI1, EQEI_STATUS_UTIEF_Msk))     /* EQEI Unit Timer Event flag */
    {
        EQEI_CLR_INT_FLAG(EQEI1, EQEI_STATUS_UTIEF_Msk);
        //printf("Unit TImer1 INT!\n\n");
        g_au32TMRINTCount[1]++;
    }

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable EQEI0 module clock */
    CLK_EnableModuleClock(EQEI0_MODULE);
    CLK_EnableModuleClock(EQEI1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

}

void UART0_Init()
{

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{
    uint32_t u32InitCount, au32Counts[2];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-----------------------------------------+\n");
    printf("|     M2L31 EQEI Unit Timer Sample Code   |\n");
    printf("+-----------------------------------------+\n\n");
    printf("# Unit Timer0 Settings:\n");
    printf("    - Clock source is PCLK0       \n");
    printf("    - Compare Value frequency is 1 Hz\n");
    printf("    - Interrupt enable          \n");
    printf("# Unit Timer1 Settings:\n");
    printf("    - Clock source is PCLK1      \n");
    printf("    - Compare Value frequency is 2 Hz\n");
    printf("    - Interrupt enable          \n");

    /* Set Unit Timer compare value */
    EQEI0->UTCMP = 72000000;
    EQEI1->UTCMP = 72000000/2;

    /* Enable EQEI interrupt */
    EQEI0->CTL2 |= (EQEI_CTL2_UTIEIEN_Msk | EQEI_STATUS_PHEF_Msk);
    EQEI1->CTL2 |= (EQEI_CTL2_UTIEIEN_Msk | EQEI_STATUS_PHEF_Msk);
    NVIC_EnableIRQ(EQEI0_IRQn);
    NVIC_EnableIRQ(EQEI1_IRQn);


    /* Clear Unit Timer0 ~ Timer1 interrupt counts to 0 */
    g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = 0;
    u32InitCount = g_au32TMRINTCount[0];

    /* Start EQEI Unit TImer */
    EQEI0->CTL2 |= EQEI_CTL2_UTEN_Msk;
    EQEI1->CTL2 |= EQEI_CTL2_UTEN_Msk;

    /* Check EQEI Unit Timer0 ~ Timer1 interrupt counts */
    printf("# EQEI Unit Timer interrupt counts :\n");
    while(u32InitCount < 20)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
            au32Counts[0] = g_au32TMRINTCount[0];
            au32Counts[1] = g_au32TMRINTCount[1];
            printf("    Unit Timer0:%3d    Unit Timer1:%3d\n", au32Counts[0], au32Counts[1]);
            u32InitCount = g_au32TMRINTCount[0];

            if((au32Counts[1] > (au32Counts[0] * 2 + 1)) || (au32Counts[1] < (au32Counts[0] * 2 - 1)))
            {
                printf("*** FAIL ***\n");
                while(1);
            }
        }
    }

    printf("*** PASS ***\n");

    while(1);

}
