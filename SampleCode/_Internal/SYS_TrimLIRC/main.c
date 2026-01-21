/**************************************************************************//**
 * @file         main.c
 * @version      V3.00
 * @brief        Demonstrate how to use timer to trim LIRC
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TIMER       TIMER0
#define TRIM_LOOP   1024
#define LIRC_38K    38400


#define GET_LIRC_TRIM_VALUE (inpw(0x40000138)&0x7F)
#define SET_LIRC_TRIM_VALUE(value) (outpw(0x40000138, (inpw(0x40000138)&(~0x7F))|(value&0x7F)))
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*-----------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                   */
    /*-----------------------------------------------------------------------------------------------------*/

    /* Enable HIRC, HXT and LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for HIRC, HXT and LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Select HCLK clock source as HXT and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER module clock source as PCLK and TIMER module clock divider as 1 */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    /*-------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                               */
    /*-------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* Set PB multi-function pins for CLKO(PB.14) */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) | SYS_GPB_MFPH_PB14MFP_CLKO;

}

void UART0_Init(void)
{
    /*--------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                              */
    /*--------------------------------------------------------------------------------------------------------*/

    UART_Open(UART0, 115200); /* Configure UART0 and set UART0 Baud rate */
}

void TrimLIRC()
{
    uint32_t u32CapVal0 = 0, u32CapVal1 = 0, u32CapVal_Interval = 0;
    uint32_t u32TrimLoopIndex = 0, u32RCValue = 0;
    uint32_t u32Freq_PCLK0_DIV_LIRC = ((CLK_GetPCLK0Freq() * 10) / __LIRC + 5) / 10;

    /* Set timer continuous counting mode */
    TIMER->CTL |= TIMER_CTL_OPMODE_Msk;

    /* Set timer prescale value */
    TIMER_SET_PRESCALE_VALUE(TIMER, 0x0);

    /*Set timer compared value*/
    TIMER_SET_CMP_VALUE(TIMER, 0xFFFFFF);

    /* Enable timer capture function */
    TIMER_EnableCapture(TIMER, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_FALLING);

    /* Select timer capture source from internal signal*/
    TIMER_CaptureSelect(TIMER, TIMER_CAPTURE_FROM_INTERNAL);

    /* Select timer capture source as LIRC */
    TIMER->EXTCTL = (TIMER->EXTCTL & ~(TIMER_EXTCTL_INTERCAPSEL_Msk)) | TIMER_INTER_CAPTURE_FROM_LIRC;

    /* Enable CLKO and output frequency = LIRC  */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_LIRC, 0, 1);

    /* Start timer counting */
    TIMER_Start(TIMER);

    /* Start timer capture function */
    TIMER_StartCapture(TIMER);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Load LIRC ADJ value */
    u32RCValue = GET_LIRC_TRIM_VALUE;

    /* Set a incurract trim value for test */
    SET_LIRC_TRIM_VALUE(u32RCValue - 0x30);

    /* Load LIRC ADJ value */
    u32RCValue = GET_LIRC_TRIM_VALUE;

    while (1)
    {
        for (u32TrimLoopIndex = 0, u32CapVal_Interval = 0; u32TrimLoopIndex < TRIM_LOOP; u32TrimLoopIndex++)
        {
            /* Clear timer capture interrupt flag */
            TIMER_ClearCaptureIntFlag(TIMER);

            /* Get timer capture interrupt flag */
            while (TIMER_GetCaptureIntFlag(TIMER) == 0);

            /* Get capture value */
            u32CapVal0 = TIMER_GetCaptureData(TIMER);

            /* Clear timer capture interrupt flag */
            TIMER_ClearCaptureIntFlag(TIMER);

            /* Get timer capture interrupt flag */
            while (TIMER_GetCaptureIntFlag(TIMER) == 0);

            /* Get capture value */
            u32CapVal1 = TIMER_GetCaptureData(TIMER);

            /* Summary capture value */
            u32CapVal_Interval += u32CapVal1 - u32CapVal0;
        }

        /* Update LIRC ADJ value */
        if ((((u32CapVal_Interval * 10) / TRIM_LOOP + 5) / 10) > u32Freq_PCLK0_DIV_LIRC)
        {
            u32RCValue--;
        }
        else if ((((u32CapVal_Interval * 10) / TRIM_LOOP + 5) / 10) < u32Freq_PCLK0_DIV_LIRC)
        {
            u32RCValue++;
        }
        else
        {
            printf("[LIRCADJ]0x%x, LIRC Trim PASS!\n", GET_LIRC_TRIM_VALUE);
            break;
        }

        if (u32RCValue > 0xFF)
        {
            SET_LIRC_TRIM_VALUE(0x70);
            printf("[LIRCADJ]0x%x, LIRC Trim Fail!\n", GET_LIRC_TRIM_VALUE);
            break;
        }
        else
        {
            printf("[u32RCValue]0x%x!\n", u32RCValue);
            /* Update LIRC ADJ value */
            SET_LIRC_TRIM_VALUE(u32RCValue);
        }
    }

    /* Lock protected registers */
    SYS_LockReg();

    /* Stop timer capture function */
    TIMER_StopCapture(TIMER);

    /* Stop timer counting */
    TIMER_Stop(TIMER);

}

int32_t main(void)
{
    SYS_UnlockReg(); /* Unlock protected registers */

    /* Init System, IP clock and multi-function I/O
    In the end of SYS_Init() will issue SYS_LockReg()
    to lock protected register. If user want to write
    protected register, please issue SYS_UnlockReg()
    to unlock protected register if necessary */
    SYS_Init();

#if 0
    PE->PUSEL = GPIO_PUSEL_PULL_DOWN;
    GPIO_SetMode(PE, BIT0, GPIO_MODE_INPUT);
#endif

    SYS_LockReg(); /* Lock protected registers */

    UART0_Init(); /* Init UART0 for printf */

    /* Trim LIRC to 38.4KHz */
    TrimLIRC();

    while (1);
}


/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
