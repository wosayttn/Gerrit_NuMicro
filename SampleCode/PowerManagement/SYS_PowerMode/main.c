/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to set different core voltage.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

static volatile uint8_t s_u8IsINTEvent;

/*----------------------------------------------------------------------*/
/*  WDT IRQ Handler                                                     */
/*----------------------------------------------------------------------*/
void WDT_IRQHandler(void)
{
    if(WDT_GET_TIMEOUT_INT_FLAG())
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }

    if(WDT_GET_TIMEOUT_WAKEUP_FLAG())
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }

    s_u8IsINTEvent = 1;
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                        */
/*----------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/*----------------------------------------------------------------------*/
/*  Simple calculation test function                                    */
/*----------------------------------------------------------------------*/
#define PI_NUM  256
static int32_t s_ai32f[PI_NUM + 1];
static uint32_t s_au32piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

static int32_t s_ai32piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for(; b - c;)
        s_ai32f[b++] = a / 5;

    i = 0;
    for(; (void)(d = 0), g = c * 2; c -= 14, s_ai32piResult[i++] = e + d / a, e = d % a)
    {
        if(i == 19)
            break;

        for(b = c; (void)(d += s_ai32f[b] * a), (void)(s_ai32f[b] = d % --g), (void)(d /= g--), --b; d *= b);
    }
    i32Err = 0;
    for(i = 0; i < 19; i++)
    {
        if(s_au32piTbl[i] != (uint32_t)s_ai32piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

void CheckSystemWork(void)
{
    if(pi())
    {
        printf("[FAIL]\n");
    }
    else
    {
        printf("[OK]\n");
    }
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release I/O hold status */
    CLK->IOPDCTL = 1;

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Select WDT module clock source */
    CLK_SetModuleClock(WDT_MODULE, LPSCC_CLKSEL0_WDTSEL_LIRC, 0);

    /* Enable WDT module clock */
    CLK_EnableModuleClock(WDT_MODULE);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/*  Main Function                                                       */
/*----------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCnt;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|         Power Mode Sample Code        |\n");
    printf("+---------------------------------------+\n");

    /* Unlock protected registers before setting power level. */
    SYS_UnlockReg();

    printf("Set power level to PL1 ");
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL1);

    /* Set core clock as 64MHz from PLL */
    CLK_SetCoreClock(64000000);

    /* Check system work */
    CheckSystemWork();

    /* Enter to Power-down Mode and wake-up by WDT in LDO mode */
    printf("Enter to Power-down Mode and wake-up ");

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW14, (uint32_t)NULL, FALSE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Check if WDT time-out interrupt and wake-up occurred or not */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(s_u8IsINTEvent == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for WDT interrupt time-out!\n");
            break;
        }
    }

    /* Check system work */
    CheckSystemWork();

    printf("\nSample code end.");

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
