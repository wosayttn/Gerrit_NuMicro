/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show the usage of LXT clock frequency monitor function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define TIMER1_DELAY_COUNTER            (10)
#define RTC_CLK_FAIL_BOUNDARY           (5)
#define RTC_CLK_STOP_BOUNDARY           (10)
#define RTC_CLK_DETECTOR_MAX_BOUNDARY   (255)

void SYS_Init(void);
void RTC_Init(void);
void TIMER0_Init(void);
void GetActiveLXTandLIRC32Freq(uint32_t *u32LXTFreq, uint32_t *LIRC32Freq);

/**
 * @brief       IRQ Handler for RTC Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_IRQHandler is default IRQ of RTC.
 */
NVT_ITCM void RTCTAMPER_IRQHandler(void)
{
    uint32_t intflag;

    if (RTC_GET_CLKFAIL_INT_FLAG(RTC))
    {
        printf("LXT clock frequency monitor fail interrupt is happened!\n");
        printf("LXT frequency is abnormal! RTC clock is switched to LIRC32.\n\n");

        /* Disable LXT clock frequency monitor fail interrupt */
        RTC_DisableInt(RTC_INTEN_CLKFIEN_Msk);

        /* Write 1 to clear LXT Clock frequency monitor fail interrupt */
        RTC_CLEAR_CLKFAIL_INT_FLAG(RTC);
    }

    if (RTC_GET_CLKSTOP_INT_FLAG(RTC))
    {
        printf("LXT clock frequency monitor stop interrupt is happened!\n");
        printf("LXT frequency is abnormal! RTC clock is switched to LIRC32.\n\n");

        /* Disable LXT clock frequency monitor stop interrupt */
        RTC_DisableInt(RTC_INTEN_CLKSTIEN_Msk);;

        /* Write 1 to clear LXT Clock frequency monitor stop interrupt */
        RTC_CLEAR_CLKSTOP_INT_FLAG(RTC);
    }

    /* make sure that interrupt flag has been cleared. */
    intflag = RTC->INTSTS;
    NVT_UNUSED(intflag);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Set X32_OUT(PF.4) and X32_IN(PF.5)*/
    SET_X32_IN_PF5();
    SET_X32_OUT_PF4();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Enable External LXT clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LXTEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Waiting for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(RTC0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PC multi-function pins for CLKO(PC.13) */
    SET_CLKO_PC13();
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init RTC clock and clear interrupt flag status                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_Init(void)
{
    /* Back to RTC initial setting */
    RTC_CLEAR_CLKFAIL_INT_FLAG(RTC);
    RTC_CLEAR_CLKSTOP_INT_FLAG(RTC);
    RTC_DisableClockFrequencyDetector();
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Get LXT and LIRC Frequency                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void GetActiveLXTandLIRC32Freq(uint32_t *u32LXTFreq, uint32_t *LIRC32Freq)
{
    uint32_t u32GetCNT;
    /* select TIMER0/1 clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_TMRSEL_TMR1SEL_LXT, 0);
    /* enable TIMER0/1 module */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    /* Timer0 config*/
    TIMER_SET_CMP_VALUE(TIMER0, __HIRC);
    TIMER_SET_OPMODE(TIMER0, TIMER_ONESHOT_MODE);
    TIMER_Stop(TIMER0);
    /* Timer1 config*/
    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFF);
    TIMER_SET_OPMODE(TIMER1, TIMER_PERIODIC_MODE);
    TIMER_Start(TIMER1);

    /* LXT source from external LXT */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);
    CLK_SysTickDelay(10000);
    /* clear TIMER0 interrupt flag*/
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    /* reset TIMER1 counter*/
    TIMER_ResetCounter(TIMER1);

    while (1)
    {
        if (TIMER_GetCounter(TIMER1) == TIMER1_DELAY_COUNTER)
        {
            TIMER_Start(TIMER0);
            break;
        }
    }

    while (TIMER_GetIntFlag(TIMER0) == 0);

    u32GetCNT = TIMER_GetCounter(TIMER1);
    *u32LXTFreq = u32GetCNT - TIMER1_DELAY_COUNTER;

    /* LXT source from LIRC32 */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LIRC32K);
    CLK_SysTickDelay(10000);
    /* clear TIMER0 interrupt flag*/
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    /* reset TIMER1 counter*/
    TIMER_ResetCounter(TIMER1);

    while (1)
    {
        if (TIMER_GetCounter(TIMER1) == TIMER1_DELAY_COUNTER)
        {
            TIMER_Start(TIMER0);
            break;
        }
    }

    while (TIMER_GetIntFlag(TIMER0) == 0);

    u32GetCNT =  TIMER_GetCounter(TIMER1);
    *LIRC32Freq = u32GetCNT - TIMER1_DELAY_COUNTER;

    /* LXT source from external LXT */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32LXTFreq, u32LIRC32Freq, u32STDCount, u32FailSTDCount, u32StopSTDCount;
    int32_t i32Option;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init RTC */
    RTC_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %uHz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|             RTC Frequency Detection Sample Code             |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("| LXT clock frequency monitor fail/stop interrupt will happen |\n");
    printf("| if LXT clock frequency isn't oscillating in normal range.   |\n");
    printf("| User can switch RTC clock source from LXT to LIRC32.        |\n");
    printf("+-------------------------------------------------------------+\n");

    GetActiveLXTandLIRC32Freq(&u32LXTFreq, &u32LIRC32Freq);

    /* calculation the LXT clock source detector boundary*/
    /** if usr wnet detector LXT clock soucre fail is 31000Hz (define LXT_Fail_HZ),
     *  and than if LXT clock source normal is 32000Hz(define LXT_Normak_HZ),
     *  the fail boundary value is (LXT_Fail_HZ / LXT_Normak_HZ) * 255(The boundary max value) = 247,
     *  the stop boundary value need less the fail boundary value.
     */
    if (u32LXTFreq > u32LIRC32Freq)
    {
        u32STDCount = ((1000000000 / u32LXTFreq) * RTC_CLK_DETECTOR_MAX_BOUNDARY) / (1000000000 / u32LIRC32Freq);
        printf("LXT is faster than LIRC32. [%uHz > %uHz]\n", u32LXTFreq, u32LIRC32Freq);
    }
    else
    {
        u32STDCount = ((1000000000 / u32LIRC32Freq) * RTC_CLK_DETECTOR_MAX_BOUNDARY) / (1000000000 / u32LXTFreq);
        printf("LIRC32 is faster than LXT. [%uHz > %uHz]\n", u32LIRC32Freq, u32LXTFreq);
    }

    /* Enable clock output, select CLKO clock source as LXT and set clock output frequency is LXT.
       RTC clock source will be switched to LIRC32 if LXT frequency isn't oscillating in normal range.
       You can check if RTC clock source is switched to LIRC32 by clock output pin output frequency.
    */

    /* Output selected clock to CKO, CKO Clock = LXT / 1 */
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_LXT, 0, 1);

    /* Lock protected registers */
    SYS_LockReg();

    /* Set the LXT clock frequency monitor fail/stop boundary value.
       The fail/stop boundary value should be less than u32STDCount.
    */
    u32FailSTDCount = u32STDCount - RTC_CLK_FAIL_BOUNDARY;
    u32StopSTDCount = u32STDCount - RTC_CLK_STOP_BOUNDARY;

    RTC_EnableClockFrequencyDetector(u32FailSTDCount, u32StopSTDCount);

    /* Clock frequency monitor fail/stop interrupt enabled */
    RTC_EnableInt((RTC_INTEN_CLKFIEN_Msk | RTC_INTEN_CLKSTIEN_Msk));

    /* Enable RTC tamper detection interrupt */
    NVIC_EnableIRQ(RTCTAMPER_IRQn);

    printf("\nSelect:\n");
    printf("    [0] Smaller frequency deviation monitoring\n");
    printf("    [1] Larger frequency deviation monitoring\n");

    i32Option = getchar();
    printf("\n");
    printf("Select item [%c]\n", i32Option);

    if (i32Option == '0')
    {
        printf("Modify LXT fail boundary to detect FAIL flag.\n\n");

        /* Modify the LXT clock frequency monitor fail boundary value to detect FAIL flag.
           The fail boundary value should be more than u32STDCount.
        */
        u32FailSTDCount = u32STDCount + RTC_CLK_FAIL_BOUNDARY;

        /* Check Fail boundary more than RTC_CLK_DETECTOR_MAX_BOUNDARY */
        if (u32FailSTDCount > RTC_CLK_DETECTOR_MAX_BOUNDARY)
        {
            u32FailSTDCount = RTC_CLK_DETECTOR_MAX_BOUNDARY;
        }

        RTC_SET_CLKFAIL_BOUNDARY(RTC, u32FailSTDCount);
    }
    else if (i32Option == '1')
    {
        printf("Stop LXT to detect FAIL and STOP flag.\n\n");
    }

    /* Wait for clock frequency monitor fail/stop detector interrupt happened */
    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
