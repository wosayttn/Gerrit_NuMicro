/**************************************************************************//**
 * @file         main.c
 * @version      V1.00
 * @brief        Demonstrate how to use timer to trim LIRC
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TIMER       TIMER0
#define TRIM_LOOP   128

#define LIRC_WRITE_FREQTRIM(u32Value) (outpw(0x40000F20, u32Value))
#define LIRC_READ_FREQTRIM() (inpw(0x40000F20))

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    SYS_UnlockReg(); /* Unlock protected registers */

    /*-----------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                   */
    /*-----------------------------------------------------------------------------------------------------*/

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable Internal Medium Speed clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal  Medium Speed clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 220MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER module clock source as PCLK and TIMER module clock divider as 1 */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Lock protected registers */
    SYS_LockReg();
}

void TrimLIRC()
{
    uint32_t u32CapVal0, u32CapVal1, u32CapVal_Interval;
    uint32_t u32TrimLoopIndex, u32RCValue;
    uint32_t u32Freq_PCLK0_DIV_LIRC = ((CLK_GetPCLK1Freq() * 10) / __LIRC + 5) / 10;

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

    /* Select timer capture source as LIRC and capture source divide 16 */
    TIMER->EXTCTL = (TIMER->EXTCTL & ~(TIMER_EXTCTL_ICAPSEL_Msk)) | TIMER_INTER_CAPTURE_FROM_LIRC;
    TIMER->EXTCTL = (TIMER->EXTCTL & ~(TIMER_EXTCTL_CAPDIVSCL_Msk)) | TIMER_CAPTURE_SRCDIV_16;
    TIMER_CaptureSelect(TIMER0, TIMER_CAPTURE_FROM_HIRC);

    /* Enable CLKO and output frequency = LIRC  */
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_LIRC, 0, CLK_CLKOCTL_DIV1EN_DIV_1);

    /* Start timer counting */
    TIMER_Start(TIMER);

    /* Start timer capture function */
    TIMER_StartCapture(TIMER);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Load LIRC ADJ value */
    u32RCValue = LIRC_READ_FREQTRIM();

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
        if ((((u32CapVal_Interval * 10) / TRIM_LOOP + 5) / 10) / 16 < u32Freq_PCLK0_DIV_LIRC)
        {
            u32RCValue++;
        }
        else if ((((u32CapVal_Interval * 10) / TRIM_LOOP + 5) / 10) / 16 > u32Freq_PCLK0_DIV_LIRC)
        {
            u32RCValue--;
        }
        else
        {
            printf("[LIRC_FREQTRIM]0x%x, LIRC Trim PASS!\n", LIRC_READ_FREQTRIM());
            break;
        }

        if (u32RCValue > 0x1FF)
        {
            LIRC_WRITE_FREQTRIM(0x000300F3);
            printf("[LIRC_FREQTRIM]0x%x, LIRC Trim Fail!\n", LIRC_READ_FREQTRIM());
            break;
        }
        else
        {
            /* Update LIRC Trim value */
            LIRC_WRITE_FREQTRIM(0x00030000 + u32RCValue);
            printf("Update LIRC Trim Vlaue: 0x%x\n", LIRC_READ_FREQTRIM());
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

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Trim LIRC to 32kHz */
    TrimLIRC();

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
