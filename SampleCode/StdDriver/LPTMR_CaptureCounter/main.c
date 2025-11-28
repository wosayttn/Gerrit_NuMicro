/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to use the LPTMR capture function to capture LPTMR counter value.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


volatile uint32_t g_au32LPTMR1INTCount = 0;

void LPTMR1_IRQHandler(void)
{
    if(LPTMR_GetCaptureIntFlag(LPTMR1) == 1)
    {
        /* Clear LPTMR1 capture trigger interrupt flag */
        LPTMR_ClearCaptureIntFlag(LPTMR1);

        g_au32LPTMR1INTCount++;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable LPTMR and TIMER peripheral clock */
    CLK_SetModuleClock(LPTMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);
    CLK_SetModuleClock(LPTMR1_MODULE, LPSCC_CLKSEL0_LPTMR1SEL_HIRC, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HIRC, 0);
    CLK_EnableModuleClock(LPTMR0_MODULE);
    CLK_EnableModuleClock(LPTMR1_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB multi-function pin for LPTMR1 external capture pin */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(SYS_GPB_MFP3_PB14MFP_Msk)) |
                    (SYS_GPB_MFP3_PB14MFP_LPTM1_EXT);

    /* Set multi-function pins for LPTMR0 toggle-output pin and LPTMR1 event counter pin */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB5MFP_Msk | SYS_GPB_MFP1_PB4MFP_Msk)) |
                    (SYS_GPB_MFP1_PB5MFP_LPTM0 | SYS_GPB_MFP1_PB4MFP_LPTM1);

    /* Set multi-function pins for Timer3 toggle-output pin */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB2MFP_Msk)) |
                    (SYS_GPB_MFP0_PB2MFP_TM3);

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
    uint32_t u32InitCount;
    uint32_t au32CAPValue[10], u32CAPDiff;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();


    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    LPTMR1 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    printf("# LPTMR0 Settings:\n");
    printf("    - Clock source is HIRC\n");
    printf("    - Time-out frequency is 1000 Hz\n");
    printf("    - Toggle-output mode and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HIRC\n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Toggle-output mode and frequency is 1 Hz\n");
    printf("# LPTMR1 Settings:\n");
    printf("    - Clock source is HIRC              \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Interrupt enable                  \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - Event counter mode enable         \n");
    printf("    - External capture mode enable      \n");
    printf("    - Capture trigger interrupt enable  \n");
    printf("# Connect LPTM0(PB.5) toggle-output pin to LPTM1(PB.4) event counter pin.\n");
    printf("# Connect TM3(PB.2)   toggle-output pin to LPTM1_EXT(PB.14) external capture pin.\n\n");

    /* Enable LPTMR1 NVIC */
    NVIC_EnableIRQ(LPTMR1_IRQn);

    /* Open LPTMR0 in toggle-output mode and toggle-output frequency is 500 Hz*/
    LPTMR_Open(LPTMR0, LPTMR_TOGGLE_MODE, 1000);

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 Hz */
    TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, 2);

    /* Enable LPTMR1 event counter input and external capture function */
    LPTMR_Open(LPTMR1, LPTMR_CONTINUOUS_MODE, 1);
    LPTMR_SET_PRESCALE_VALUE(LPTMR1, 0);
    LPTMR_SET_CMP_VALUE(LPTMR1, LPTMR_CMP_MAX_VALUE);
    LPTMR_EnableEventCounter(LPTMR1, LPTMR_COUNTER_EVENT_FALLING);
    LPTMR_EnableCapture(LPTMR1, LPTMR_CAPTURE_FREE_COUNTING_MODE, LPTMR_CAPTURE_EVENT_FALLING);
    LPTMR_EnableInt(LPTMR1);
    LPTMR_EnableCaptureInt(LPTMR1);

    /* case 1. */
    printf("# Period between two FALLING EDGE captured event should be 500 counts.\n");

    /* Clear LPTMR1 interrupt counts to 0 */
    u32InitCount = g_au32LPTMR1INTCount = 0;

    /* Start LPTMR0, Timer3 and LPTMR1 counting */
    LPTMR_Start(LPTMR0);
    TIMER_Start(TIMER3);
    LPTMR_Start(LPTMR1);

    /* Check LPTMR1 capture trigger interrupt counts */
    while(g_au32LPTMR1INTCount <= 10)
    {
        if(g_au32LPTMR1INTCount != u32InitCount)
        {
            au32CAPValue[u32InitCount] = LPTMR_GetCaptureData(LPTMR1);
            if(u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32LPTMR1INTCount, au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32LPTMR1INTCount, au32CAPValue[u32InitCount], u32CAPDiff);
                if(u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    while(1);
                }
            }
            u32InitCount = g_au32LPTMR1INTCount;
        }
    }
    printf("*** PASS ***\n\n");

    /* case 2. */
    LPTMR_DisableCapture(LPTMR1);
    LPTMR_Stop(LPTMR1);
    while(LPTMR_IS_ACTIVE(LPTMR1));
    LPTMR_ClearIntFlag(LPTMR1);
    LPTMR_ClearCaptureIntFlag(LPTMR1);

    printf("# Period between two RISING EDGE captured event should be 500 counts.\n");

    /* Clear LPTMR1 interrupt counts to 0 */
    u32InitCount = g_au32LPTMR1INTCount = 0;

    /* Enable LPTMR1 event counter input and external capture function */
    LPTMR_Open(LPTMR1, LPTMR_CONTINUOUS_MODE, 1);
    LPTMR_SET_PRESCALE_VALUE(LPTMR1, 0);
    LPTMR_SET_CMP_VALUE(LPTMR1, LPTMR_CMP_MAX_VALUE);
    LPTMR_EnableEventCounter(LPTMR1, LPTMR_COUNTER_EVENT_FALLING);
    LPTMR_EnableCapture(LPTMR1, LPTMR_CAPTURE_FREE_COUNTING_MODE, LPTMR_CAPTURE_EVENT_RISING);
    LPTMR_EnableInt(LPTMR1);
    LPTMR_EnableCaptureInt(LPTMR1);
    LPTMR_Start(LPTMR1);

    /* Check LPTMR1 capture trigger interrupt counts */
    while ((g_au32LPTMR1INTCount <= 10) && (u32InitCount < 10))
    {
        if(g_au32LPTMR1INTCount != u32InitCount)
        {
            au32CAPValue[u32InitCount] = LPTMR_GetCaptureData(LPTMR1);
            if(u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32LPTMR1INTCount, au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32LPTMR1INTCount, au32CAPValue[u32InitCount], u32CAPDiff);
                if(u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    while(1);
                }
            }
            u32InitCount = g_au32LPTMR1INTCount;
        }
    }

    /* Stop LPTMR0, LPTMR1 and Timer3 counting */
    LPTMR_Stop(LPTMR0);
    LPTMR_Stop(LPTMR1);
    TIMER_Stop(TIMER3);

    printf("*** PASS ***\n");

    while(1);
}
