/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO EINT trigger Timer PWM function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"



void EINT1_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);



/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1 default IRQ, declared in startup_m3331.s.
 */
void EINT1_IRQHandler(void)
{
    /* To check if PA.7 external interrupt occurred */
    if(INT1->EDSTS & GPIO_INT_EDSTS_EDIF_Msk)
    {
        INT1->EDSTS = GPIO_INT_EDSTS_EDIF_Msk;
        printf("PA.7 EINT1 occurred.\n");
    }
}

/**
 * @brief       TIMER0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The TIMER0 default IRQ, declared in startup_m3331.s.
 */
void TMR0_IRQHandler(void)
{
    /* Check Timer0 PWM compare up count interrupt flag */
    if(TIMER0->PWMINTSTS0 & TIMER_PWMINTSTS0_CMPUIF_Msk)
    {
        /* Disable Timer0 PWM counting */
        TIMER0->PWMCTL &= ~TIMER_PWMCTL_CNTEN_Msk;

        /* Clear Timer0 PWM compare up count interrupt flag */
        TIMER0->PWMINTSTS0 = TIMER_PWMINTSTS0_CMPUIF_Msk;
        printf("Timer0 PWM compare up count interrupt occurred.\n");
    }
}


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_180MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source as PCLK0 */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pin for EINT1(PA.7) */
    SET_INT1_PA7();
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    GPIO EINT Trigger TPWM Sample Code    |\n");
    printf("+------------------------------------------+\n\n");
    printf("Use EINT1(PA.7) falling edge to start Timer0 PWM counting.\n");

    /* Change Timer0 to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set Timer0 PWM comparator register */
    TPWM_SET_CMPDAT(TIMER0, 0xFFFF);

    /* Set Timer0 PWM period register */
    TPWM_SET_PERIOD(TIMER0, 0xFFFF);

    /* Enable Timer0 PWM compare up count interrupt */
    TIMER0->PWMINTEN0 |= TIMER_PWMINTEN0_CMPUIEN_Msk;
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer0 PWM counting by external trigger EINT1 */
    TIMER0->PWMEXTETCTL = (TPWM_EXT_TGR_PIN_INT1 << TIMER_PWMEXTETCTL_EXTTRGS_Pos) |
                          (TPWM_EXT_TGR_COUNTER_START << TIMER_PWMEXTETCTL_CNTACTS_Pos) |
                          TIMER_PWMEXTETCTL_EXTETEN_Msk;

    /* Configure PA.7 as EINT1 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PA, BIT7, GPIO_PUSEL_PULL_UP);
    INT1->EDETCTL = (INT1->EDETCTL & (~GPIO_INT_EDETCTL_EDETCTL_Msk)) | GPIO_INT_EDETCTL_FALLING;
    INT1->EDINTEN |= GPIO_INT_EDINTEN_EDIEN_Msk;
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT7);

    while(1);

}
