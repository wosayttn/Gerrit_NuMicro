/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO external interrupt function and de-bounce function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t gu32EINT0_count = 0;
uint32_t gu32EINT1_count = 0;
uint32_t gu32EINT2_count = 0;

void EINT0_IRQHandler(void)
{
    /* To check if EINT0 external interrupt occurred */
    if(GPIO_GET_EINT_FLAG(0))
    {
        GPIO_CLR_EINT_FLAG(0);
        printf("EINT0 occurred #%d.\n", ++gu32EINT0_count);
    }
}

void EINT1_IRQHandler(void)
{
    /* To check if EINT1 external interrupt occurred */
    if(GPIO_GET_EINT_FLAG(1))
    {
        GPIO_CLR_EINT_FLAG(1);
        printf("EINT1 occurred #%d.\n", ++gu32EINT1_count);
    }
}

void EINT2_IRQHandler(void)
{
    /* To check if EINT2 external interrupt occurred */
    if(GPIO_GET_EINT_FLAG(2))
    {
        GPIO_CLR_EINT_FLAG(2);
        printf("EINT2 occurred #%d.\n", ++gu32EINT2_count);
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);

    /*----------------------------------------------*/
    /* Init I/O Multi-function                      */
    /*----------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PA multi-function pin for EINT0(PA.6) and EINT1(PA.7) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk)) |
                    (SYS_GPA_MFPL_PA6MFP_INT0 | SYS_GPA_MFPL_PA7MFP_INT1);

    /* Set PB multi-function pin for EINT2(PB.3) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB3MFP_INT2);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------------------+\n");
    printf("| GPIO EINT0/EINT1/EINT2 Interrupt and De-bounce Sample Code |\n");
    printf("+------------------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO External Interrupt Function Test                                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("EINT0 (PA.6), EINT1 (PA.7) and EINT2 (PB.3) are used to test external interrupt\n");
    printf("    PA.6 is falling edge trigger.\n");
    printf("    PA.7 is rising edge trigger.\n");
    printf("    PB.3 is both falling edge and rising edge trigger.\n");

    /* Configure PA.6 as EINT0 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PA, BIT6, GPIO_MODE_INPUT);
    GPIO_EnableEINT(0, GPIO_INT_EDETCTL_FALLING);
    /* Enable external interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_EINT_DEBOUNCE_TIME(0, GPIO_INT_NFSEL_HCLK_DIV_2, 3);
    GPIO_ENABLE_EINT_DEBOUNCE(0);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Configure PA.7 as EINT1 pin and enable interrupt by rising edge trigger */
    GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
    GPIO_EnableEINT(1, GPIO_INT_EDETCTL_RISING);
    /* Enable external interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_EINT_DEBOUNCE_TIME(1, GPIO_INT_NFSEL_HCLK_DIV_64, 5);
    GPIO_ENABLE_EINT_DEBOUNCE(1);
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Configure PB.3 as EINT2 pin and enable interrupt by both falling and rising edge trigger */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);
    GPIO_EnableEINT(2, GPIO_INT_EDETCTL_BOTH_EDGE);
    /* Enable external interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_EINT_DEBOUNCE_TIME(2, GPIO_INT_NFSEL_HCLK_DIV_128, 7);
    GPIO_ENABLE_EINT_DEBOUNCE(2);
    NVIC_EnableIRQ(EINT2_IRQn);

    /* Waiting for interrupts */
    while(1);
}
