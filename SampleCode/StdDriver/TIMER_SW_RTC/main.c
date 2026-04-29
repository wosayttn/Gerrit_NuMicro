/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This sample code performs how to use software to simulate RTC.
 *           In power down mode, using the Timer to wake-up the MCU and add RTC value per second.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "SW_RTC.h"

/*----------------------------------------------*/
/* Define global variables and constants        */
/*----------------------------------------------*/
/* WakeUp frequency (times/sec) */
#define WakeUpFreq  1
volatile uint32_t CntForUpdateRTC = 0;
extern DATETIME rtc;

void SYS_Init(void)
{
	/*------------------------------------------*/
	/* Init System Clock                        */
	/*------------------------------------------*/
	/* Unlock protected registers */
	SYS_UnlockReg();

	/* Enable Internal RC 24 MHz clock */
	CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

	/* Wait for HIRC clock ready */
	CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

	/* Enable Internal RC 10 KHz clock */
	CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

	/* Wait for LIRC clock ready */
	CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

	/* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
	SystemCoreClockUpdate();

	/* Select UART clock source from HXT */
	CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

	/* Enable UART clock */
	CLK_EnableModuleClock(UART0_MODULE);

	/* Enable TIMER peripheral clock */
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_LIRC, 0);
	CLK_EnableModuleClock(TMR0_MODULE);

	/*----------------------------------*/
	/* Init I/O Multi-function          */
	/*----------------------------------*/
	/* Set GPB multi-function pins for UART0 RXD and TXD */
	SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_UART0_RXD;
	SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB13MFP_Msk) | SYS_GPB_MFPH_PB13MFP_UART0_TXD;

	/* Lock protected registers */
	SYS_LockReg();
}

void UART0_Init(void)
{
	/* Reset UART0 */
	SYS_ResetModule(UART0_RST);

	/* Configure UART0 and set UART0 baud rate */
	UART_Open(UART0, 115200);
}

void TMR0_IRQHandler(void)
{
	if (TIMER_GetIntFlag(TIMER0) == TRUE)
    {
		/* Clear wake up flag */
		TIMER_ClearWakeupFlag(TIMER0);
		/* Clear interrupt flag */
		TIMER_ClearIntFlag(TIMER0);

		CntForUpdateRTC++;
		/* If 1 second passed, updating the RTC value. */
		if (CntForUpdateRTC == WakeUpFreq)
        {
			CntForUpdateRTC = 0;
			RTC_Process();
		}
	}
}

void TIMER_FunctionTest()
{
	/* Initial Timer0 to periodic mode with WakeUpFreq Hz.
	 User can modify the marco "WakeUpFreq" to change wake-up frequency */
	TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, WakeUpFreq);

	/* Enable timer wake up system */
	TIMER_EnableWakeup(TIMER0);

	/* Enable Timer0 interrupt */
	TIMER_EnableInt(TIMER0);
	NVIC_EnableIRQ(TMR0_IRQn);

	/* Start Timer0 counting */
	TIMER_Start(TIMER0);

	printf("+----------------------------------------------------------------------+\n");
	printf("|   This sample code performs how to use software to simulate RTC.     |\n");
	printf("|   In power down mode, using the Timer to wake-up the MCU             |\n");
	printf("|      and add RTC value per second.                                   |\n");
	printf("|   (1) Modify the macro WakeUpFreq to adjust the wake-up frequency.   |\n");
	printf("|   (2) This sample code is usung LIRC for counting in power-down mode.|\n");
	printf("+----------------------------------------------------------------------+\n\n");

	/* Wait uart transfer message */
	while (!UART_IS_TX_EMPTY(UART0));

	while (1)
    {
		/* Unlock protected registers */
		SYS_UnlockReg();
		/* Make MCU power-down */
		CLK_PowerDown();
		/* lock protected registers */
		SYS_LockReg();

		/* Display the RTC clock in 24H format */
#if DisplayIn24H
        printf("%d/%d/%d   ", rtc.date.year, rtc.date.month, rtc.date.day);
        printf("%d:%d:%d   Day of week : %d \n",rtc.time.hour, rtc.time.minutes, rtc.time.seconds, rtc.date.dayofweek);
#endif

		/* Display the RTC clock in 12H format */
#if DisplayIn12H
		printf("%d/%d/%d   ", rtc.date.year, rtc.date.month, rtc.date.day);
		if (rtc.time.hour > 12)
			printf("%d:%d:%d PM   Day of week : %d \n", rtc.time.hour - 12,
					rtc.time.minutes, rtc.time.seconds, rtc.date.dayofweek);
		else
			printf("%d:%d:%d AM   Day of week : %d \n", rtc.time.hour,
					rtc.time.minutes, rtc.time.seconds, rtc.date.dayofweek);
#endif

		/* Wait uart transfer message */
		while (!UART_IS_TX_EMPTY(UART0));
	}
}

int main()
{
	/* Init System, IP clock and multi-function I/O */
	SYS_Init();

	/* Init UART to print message */
	UART0_Init();

    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);

	/* TIMER function test */
	TIMER_FunctionTest();

	printf("Exit TIMER sample code\n");

	/* Got no where to go, just loop forever */
	while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
