/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to enter to different Power-down mode and wake-up by RTC.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
extern void initialise_monitor_handles(void);
#endif

#define PDMD_FLAG_ADDR      0x20000FFC

static uint32_t s_u32PowerDownMode;
static volatile uint32_t s_u32RTCTickINT;

/**
 * @brief       IRQ Handler for RTC Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in startup_M2L31.s.
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG() == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG();
    }

    s_u32RTCTickINT++;
}

/*----------------------------------------------------------------------*/
/*  Function for RTC wake-up source setting                             */
/*----------------------------------------------------------------------*/
int32_t RTC_Init(void)
{
    S_RTC_TIME_DATA_T sWriteRTC;

    /* Init RTC in the start of sample code */
    if(RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        printf("\n\nCPU @ %dHz\n", SystemCoreClock);
        printf("+------------------------------------------+\n");
        printf("|    Power-down and Wake-up Sample Code    |\n");
        printf("+------------------------------------------+\n\n");

        /* Open RTC */
        sWriteRTC.u32Year       = 2017;
        sWriteRTC.u32Month      = 3;
        sWriteRTC.u32Day        = 16;
        sWriteRTC.u32DayOfWeek  = RTC_MONDAY;
        sWriteRTC.u32Hour       = 0;
        sWriteRTC.u32Minute     = 0;
        sWriteRTC.u32Second     = 0;
        sWriteRTC.u32TimeScale  = RTC_CLOCK_24;
        if( RTC_Open(&sWriteRTC) < 0 )
        {
            printf("Initialize RTC module and start counting failed\n");
            return -1;
        }
        printf("# Set RTC current date/time: 2017/03/16 00:00:00.\n");

        /* It is the start of sample code */
        M32(PDMD_FLAG_ADDR) = 0;
    }

    /* Clear RTC tick interrupt flag */
    RTC_CLEAR_TICK_INT_FLAG();

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Enable RTC tick interrupt and wake-up function will be enabled also */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable RTC wake-up from SPD and DPD */
    CLK_ENABLE_RTCWK();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                        */
/*----------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Select Power-down mode */
    CLK_SetPowerDownMode(s_u32PowerDownMode);

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/*----------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                             */
/*----------------------------------------------------------------------*/
int32_t CheckPowerSource(void)
{
    S_RTC_TIME_DATA_T sReadRTC;

    /* Get Power Manager Status */
    uint32_t u32Status = CLK_GetPMUWKSrc();

    /* Clear wake-up status flag */
    CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;

    /* Check Power Manager Status is wake-up by RTC */
    if(u32Status & CLK_PMUSTS_RTCWK_Msk)
    {
        s_u32PowerDownMode = M32(PDMD_FLAG_ADDR);
        switch(s_u32PowerDownMode)
        {
        case CLK_PMUCTL_PDMSEL_NPD0:

            /* It is the start of sample code by pressing reset button */
            printf("\n\nCPU @ %dHz\n", SystemCoreClock);
            printf("+------------------------------------------+\n");
            printf("|    Power-down and Wake-up Sample Code    |\n");
            printf("+------------------------------------------+\n");
            break;

        case CLK_PMUCTL_PDMSEL_SPD0:

            /* Wake-up from Standby Power-down Mode */
            printf("Wake-up!\n");

            /* Read current RTC date/time */
            RTC_GetDateAndTime(&sReadRTC);
            printf("# Get RTC current date/time: %d/%02d/%02d %02d:%02d:%02d.\n",
                   sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

            /* Next Power-down Mode is Deep Power-down Mode */
            M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_DPD0;
            break;

        default:

            /* Wake-up from Deep Power-down Mode */
            printf("Wake-up!\n");

            /* Read current RTC date/time */
            RTC_GetDateAndTime(&sReadRTC);
            printf("# Get RTC current date/time: %d/%02d/%02d %02d:%02d:%02d.\n",
                   sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

            /* End of sample code and clear Power-down Mode flag */
            printf("\nSample code end. Press Reset Button and continue.\n");
            M32(PDMD_FLAG_ADDR) = 0;
            return 1;
        }
    }

    return 0;
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release I/O hold status */
    CLK->IOPDCTL = 1;

    /* Set PF multi-function pins for XT1_OUT (PF.2) and XT1_IN (PF.3) */
    SYS->GPF_MFP0 = (SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) |
                    (SYS_GPF_MFP0_PF2MFP_XT1_OUT);
    SYS->GPF_MFP0 = (SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) |
                    (SYS_GPF_MFP0_PF3MFP_XT1_IN);

    /* Set PF multi-function pins for X32_OUT (PF.4) and X32_IN (PF.5) */
    SYS->GPF_MFP1 = (SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) |
                    (SYS_GPF_MFP1_PF4MFP_X32_OUT);
    SYS->GPF_MFP1 = (SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) |
                    (SYS_GPF_MFP1_PF5MFP_X32_IN);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable RTC module clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB multi-function pins for CLKO (PB.14) */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~SYS_GPB_MFP3_PB14MFP_Msk) |
                    (SYS_GPB_MFP3_PB14MFP_CLKO);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/*  Main Function                                                       */
/*----------------------------------------------------------------------*/
int32_t main(void)
{
    S_RTC_TIME_DATA_T sReadRTC;
    uint32_t u32TimeOutCnt;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    /* Enable Clock Output function, output clock is stopped in Power-down mode */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 3, 0);

    /* Get power manager wake up source */
    if( CheckPowerSource() != 0)
        goto lexit;

    /* RTC wake-up source setting */
    if( RTC_Init() < 0 )
        goto lexit;

    /*
        This sample code will enter to different Power-down mode and wake-up by RTC:
        1. Normal Power-down mode (NPD0 ~ NPD5).
        2. Standby Power-down mode (SPD0 ~ SPD2).
        3. Deep Power-down mode (DPD0 ~ DPD1).
    */
    while(1)
    {
        /* Select Power-down mode */
        s_u32PowerDownMode = M32(PDMD_FLAG_ADDR);
        switch(s_u32PowerDownMode)
        {
        case CLK_PMUCTL_PDMSEL_NPD0:
            printf("\nSystem enters to NPD0 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_NPD1:
            printf("\nSystem enters to NPD1 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_NPD2:
            printf("\nSystem enters to NPD2 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_NPD3:
            printf("\nSystem enters to NPD3 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_NPD4:
            printf("\nSystem enters to NPD4 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_NPD5:
            printf("\nSystem enters to NPD5 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_SPD0:
            printf("\nSystem enters to SPD0 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_SPD1:
            printf("\nSystem enters to SPD1 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_SPD2:
            printf("\nSystem enters to SPD2 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_DPD0:
            printf("\nSystem enters to DPD0 power-down mode ... ");
            break;
        case CLK_PMUCTL_PDMSEL_DPD1:
            printf("\nSystem enters to DPD1 power-down mode ... ");
            break;
        default:
            printf("\nInit sample code. Press Reset Button and continue.\n");
            M32(PDMD_FLAG_ADDR) = 0;
            goto lexit;
        }

        /* Unlock protected registers before setting Power-down mode */
        SYS_UnlockReg();

        /* Enter to Power-down mode */
        PowerDownFunction();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(s_u32RTCTickINT == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for RTC interrupt time-out!");
                break;
            }
        }
        printf("Wake-up!\n");

        /* Read current RTC date/time after wake-up */
        RTC_GetDateAndTime(&sReadRTC);
        printf("# Get RTC current date/time: %d/%02d/%02d %02d:%02d:%02d.\n",
               sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

        /* Select next Power-down mode */
        switch(s_u32PowerDownMode)
        {
        case CLK_PMUCTL_PDMSEL_NPD0:
            M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_NPD1;
            break;
        case CLK_PMUCTL_PDMSEL_NPD1:
            M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_NPD2;
            break;
        case CLK_PMUCTL_PDMSEL_NPD2:
            M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_NPD3;
            break;
        case CLK_PMUCTL_PDMSEL_NPD3:
            M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_NPD4;
            break;
        case CLK_PMUCTL_PDMSEL_NPD4:
            M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_NPD5;
            break;
        case CLK_PMUCTL_PDMSEL_NPD5:
            M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_SPD0;
            break;
        default:
            printf("\nPress Reset Button and continue.\n");
            M32(PDMD_FLAG_ADDR) = 0;
            goto lexit;
        }
    }

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
