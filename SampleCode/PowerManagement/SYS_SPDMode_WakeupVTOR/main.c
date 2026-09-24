/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to continue executing code after wake-up from SPD Power-down
 *           mode by VTOR function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void);
int32_t RTC_Init(void);
void UART0_Init(void);
extern uint32_t __Enter_SPD(void);

static volatile uint32_t s_u32RTCTickINT=0;

void PowerDownFunction(void)
{
    /* Select SPD0 Power-down mode */
    CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_SPD0);

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled */
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    /* Enter to Power-down mode */
    __Enter_SPD();

    /* Initialization after wake-up from SPD */
    if(CLK->PMUSTS & CLK_PMUSTS_RTCWK_Msk)
    {
        SCB->VTOR = 0;                              /* Restore VTOR setting */
        SYS->INIVTOR = 0;                           /* Restore INIVTOR setting */
        SYS_Init();                                 /* Init System, peripheral clock and multi-function I/O */
        SYS_UnlockReg();                            /* Unlock protected registers */
        UART0_Init();                               /* Init UART0 for printf */
        GPIO_SetMode(PA, BIT10, GPIO_MODE_OUTPUT);  /* Set PA.10 as output mode */
        CLK->IOPDCTL = 1;                           /* Release I/O hold status */
        RTC_Init();                        
    }
}


/**
 * @brief       IRQ Handler for RTC Interrupt
 * @param       None
 * @return      None
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in mystartup_M2L31_xxx.s.
 */
void RTC_IRQHandler(void)
{
    uint32_t u32TimeOutCnt;

    /* To check if RTC tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG() == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((RTC_GET_TICK_INT_FLAG() == 1))
            if(--u32TimeOutCnt == 0) break;
    }

    s_u32RTCTickINT = 1;
    GPIO_TOGGLE(PA10);  /* LED toggle in RTC interrupt */
}

int32_t RTC_Init(void)
{
    S_RTC_TIME_DATA_T sWriteRTC;

    /* Enable LXT clock if it is not enabled before */
    if( (CLK->PWRCTL & CLK_PWRCTL_LXTEN_Msk) == 0 )
    {
        CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);
    }

    /* Enable RTC clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* RTC clock source select LXT */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Initial RTC if it is not initialed before */
    if(RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        /* Open RTC */
        sWriteRTC.u32Year       = 2017;
        sWriteRTC.u32Month      = 3;
        sWriteRTC.u32Day        = 16;
        sWriteRTC.u32DayOfWeek  = RTC_THURSDAY;
        sWriteRTC.u32Hour       = 0;
        sWriteRTC.u32Minute     = 0;
        sWriteRTC.u32Second     = 0;
        sWriteRTC.u32TimeScale  = RTC_CLOCK_24;
        if( RTC_Open(&sWriteRTC) < 0 )
        {
            printf("Initialize RTC module and start counting failed!\n");
            return -1;
        }
        printf("# Set RTC current date/time: 2017/03/16 00:00:00.\n\n");

        /* Enable RTC tick interrupt and wake-up function will be enabled also */
        RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
        RTC_SetTickPeriod(RTC_TICK_1_SEC);
    }

    /* Enable RTC wake-up from SPD and DPD */
    CLK_ENABLE_RTCWK();

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    return 0;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release I/O hold status */
    CLK->IOPDCTL = 1;

    /* Set PF multi-function pins for X32_OUT (PF.4) and X32_IN (PF.5) */
    SYS->GPF_MFP1 = (SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) |
                    (SYS_GPF_MFP1_PF4MFP_X32_OUT);
    SYS->GPF_MFP1 = (SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) |
                    (SYS_GPF_MFP1_PF5MFP_X32_IN);

    /* Enable HIRC and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk);

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

    /* Enable GPIO Port A module clock */
    CLK_EnableModuleClock(GPA_MODULE);

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

int main( void )
{
    S_RTC_TIME_DATA_T sReadRTC;
    uint32_t u32TimeOutCnt;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* LED toggle in RTC interrupt */
    GPIO_SetMode(PA, BIT10, GPIO_MODE_OUTPUT);

    /* Unlock protected registers before setting Power-down mode */
    SYS_UnlockReg();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|   SPD Power-down Mode Wake-up and Return Sample Code   |\n");
    printf("|   with VTOR Function                                   |\n");
    printf("+--------------------------------------------------------+\n");

    /* Init RTC */
    if( RTC_Init() < 0 ) goto lexit;

    while(1)
    {
        /* Read current RTC date/time */
        RTC_GetDateAndTime(&sReadRTC);
        printf("# Get RTC current date/time: %d/%02d/%02d %02d:%02d:%02d.\n",
               sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

        s_u32RTCTickINT = 0;

        /* Enter to SPD Power-down mode */
        printf("Enter to SPD0 ... ");
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        UART_WAIT_TX_EMPTY(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;
        PowerDownFunction();

        /* Wait RTC interrupt */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(s_u32RTCTickINT==0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for RTC interrupt time-out!");
                break;
            }
        }

        /* Check wake-up from SPD by RTC flag */
        if(CLK->PMUSTS&CLK_PMUSTS_RTCWK_Msk)
        {
            printf("RTC ");

            /* Clear wake-up flag */
            CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        }

        printf("wake-up!\n\n");
    }

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
