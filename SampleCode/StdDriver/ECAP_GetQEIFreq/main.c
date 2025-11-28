/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use ECAP interface to get EQEI-A frequency.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*--------------------------------------------------------------------------*/
/* Global variables                                                         */
/*--------------------------------------------------------------------------*/
volatile uint32_t gu32Status;
volatile uint32_t gu32IC0Hold;

/*--------------------------------------------------------------------------*/
/*  Timer0 IRQ Handler                                                      */
/*--------------------------------------------------------------------------*/
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        /* PC.2 gpio toggle */
        GPIO_TOGGLE(PC2);
    }
}

/*--------------------------------------------------------------------------*/
/*  ECAP0 IRQ Handler                                                       */
/*--------------------------------------------------------------------------*/
void ECAP0_IRQHandler(void)
{
    /* Get input Capture status */
    gu32Status = ECAP_GET_INT_STATUS(ECAP0);

    /* Check input capture channel 0 flag */
    if((gu32Status & ECAP_STATUS_CAPTF0_Msk) == ECAP_STATUS_CAPTF0_Msk)
    {
        /* Clear input capture channel 0 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF0_Msk);

        /* Get input capture counter hold value */
        gu32IC0Hold = ECAP0->HLD0;
    }

    /* Check input capture channel 1 flag */
    if((gu32Status & ECAP_STATUS_CAPTF1_Msk) == ECAP_STATUS_CAPTF1_Msk)
    {
        /* Clear input capture channel 1 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF1_Msk);
    }

    /* Check input capture channel 2 flag */
    if((gu32Status & ECAP_STATUS_CAPTF2_Msk) == ECAP_STATUS_CAPTF2_Msk)
    {
        /* Clear input capture channel 2 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF2_Msk);
    }

    /* Check input capture compare-match flag */
    if((gu32Status & ECAP_STATUS_CAPCMPF_Msk) == ECAP_STATUS_CAPCMPF_Msk)
    {
        /* Clear input capture compare-match flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPCMPF_Msk);
    }

    /* Check input capture overflow flag */
    if((gu32Status & ECAP_STATUS_CAPOVF_Msk) == ECAP_STATUS_CAPOVF_Msk)
    {
        /* Clear input capture overflow flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPOVF_Msk);
    }
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/1 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable ECAP0 module clock */
    CLK_EnableModuleClock(ECAP0_MODULE);

    /* Select TIMER0 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable EQEI0 module clock */
    CLK_EnableModuleClock(EQEI0_MODULE);

    /* Enable GPIO Port C module clock */
    CLK_EnableModuleClock(GPC_MODULE);

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for EQEI0_A, EQEI0_B, EQEI0_INDEX */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA3MFP_Msk)) |
                    (SYS_GPA_MFP0_PA3MFP_EQEI0_B);
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk)) |
                    (SYS_GPA_MFP1_PA4MFP_EQEI0_A | SYS_GPA_MFP1_PA5MFP_EQEI0_INDEX);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void ECAP0_Init(void)
{
    /* Enable ECAP0 */
    ECAP_Open(ECAP0, ECAP_DISABLE_COMPARE);

    /* Select Reload function */
    ECAP_SET_CNT_CLEAR_EVENT(ECAP0, (ECAP_CTL1_CAP0RLDEN_Msk | ECAP_CTL1_CAP1RLDEN_Msk));

    /* Enable ECAP0 source from EQEI */
    ECAP_SEL_INPUT_SRC(ECAP0, ECAP_IC0, ECAP_CAP_INPUT_SRC_FROM_CH);

    /* Input Channel 0 interrupt enabled */
    ECAP_EnableINT(ECAP0, ECAP_CTL0_CAPIEN0_Msk);
}

void EQEI0_Init(void)
{
    EQEI_Open(EQEI0, EQEI_CTL_X4_FREE_COUNTING_MODE, 0);
}

void Timer0_Init(void)
{
    /* Open Timer0 in periodic mode, enable interrupt and 10000 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 10000);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
}

/*--------------------------------------------------------------------------*/
/*  Main Function                                                           */
/*--------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32Hz = 0, u32Hz_DET = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n");
    printf("+------------------------------------+\n");
    printf("|     ECAP with EQEI Sample Code     |\n");
    printf("+------------------------------------+\n");
    printf("\n");
    printf("  !! GPIO PC.2 toggle periodically   !!\n");
    printf("  !! Connect PC.2 --> PA.4 (EQEI0-A) !!\n\n");
    printf("     Press any key to start test\n\n");

    getchar();

    /* Initial ECAP0 function */
    ECAP0_Init();

    /* Initial EQEI0 function */
    EQEI0_Init();

    /* Initial Timer0 function */
    Timer0_Init();

    /* Configure PC.2 as output mode */
    GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* Start EQEI0 */
    EQEI_Start(EQEI0);

    /* Delay 200ms */
    CLK_SysTickDelay(200000);

    /* Init & clear ECAP interrupt status flags */
    gu32Status = ECAP_GET_INT_STATUS(ECAP0);
    ECAP0->STATUS = gu32Status;

    /* ECAP_CNT starts up-counting */
    ECAP_CNT_START(ECAP0);

    while(1)
    {
        if(gu32Status != 0)
        {
            /* Input Capture status is changed, and get a new hold value of input capture counter */
            gu32Status = 0;

            /* Calculate the IC0 input frequency */
            u32Hz_DET = (SystemCoreClock / 2) / (gu32IC0Hold + 1);

            if(u32Hz != u32Hz_DET)
            {
                /* If IC0 input frequency is changed, Update frquency */
                u32Hz = u32Hz_DET;
            }
            else
            {
                printf("\nECAP0_IC0 input frequency is %d (Hz), gu32IC0Hold=0x%08x\n", u32Hz, gu32IC0Hold);
                TIMER_Stop(TIMER0);
                break;
            }
        }
    }

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ECAP0_IRQn);
    NVIC_DisableIRQ(TMR0_IRQn);

    /* Disable ECAP function */
    ECAP_Close(ECAP0);

    /* Disable Timer0 IP clock */
    CLK_DisableModuleClock(TMR0_MODULE);

    /* Disable ECAP IP clock */
    CLK_DisableModuleClock(ECAP0_MODULE);

    printf("\nExit ECAP sample code\n");

    while(1);
}
