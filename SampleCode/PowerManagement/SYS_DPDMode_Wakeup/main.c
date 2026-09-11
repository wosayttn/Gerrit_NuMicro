/**************************************************************************//**
 * @file     main.c
 * @version  V1.01
 * @brief    Show how to wake up system from DPD Power-down mode by Wake-up pin 0 (PC.0)
 *           or Wake-up Timer or RTC Tick or RTC Alarm or RTC Tamper 0.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
extern void initialise_monitor_handles(void);
#endif

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                        */
/*----------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)
    if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  Wake-up pin 0                                                       */
/*----------------------------------------------------------------------*/
void WakeUpPinFunction(uint32_t u32PDMode, uint32_t u32EdgeType)
{
    printf("Enter to DPD0 Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Configure GPIO as input mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);

    /* Set Wake-up pin 0 trigger type at Deep Power down mode */
    CLK_EnableDPDWKPin(CLK_DPDWKPIN_0, u32EdgeType);

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  Wake-up Timer                                                       */
/*----------------------------------------------------------------------*/
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval)
{
    printf("Enter to DPD0 Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Set Wake-up Timer Time-out Interval */
    CLK_SET_WKTMR_INTERVAL(u32Interval);

    /* Enable Wake-up Timer */
    CLK_ENABLE_WKTMR();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  RTC Tick                                                            */
/*----------------------------------------------------------------------*/
void WakeUpRTCTickFunction(uint32_t u32PDMode)
{
    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* RTC clock source select LXT */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Open RTC and start counting */
    if( RTC_Open(NULL) < 0 )
    {
        printf("Initialize RTC module and start counting failed\n");
        return;
    }

    /* Clear tick status */
    RTC_CLEAR_TICK_INT_FLAG();

    /* Enable RTC Tick interrupt */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Set RTC tick period as 1 second */
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    /* Enable RTC wake-up */
    CLK_ENABLE_RTCWK();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    printf("Enter to DPD Power-down mode......\n");
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  RTC Alarm                                                           */
/*----------------------------------------------------------------------*/
void  WakeUpRTCAlarmFunction(uint32_t u32PDMode)
{
    S_RTC_TIME_DATA_T sWriteRTC;

    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* RTC clock source select LXT */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Open RTC and start counting */
    sWriteRTC.u32Year       = 2016;
    sWriteRTC.u32Month      = 5;
    sWriteRTC.u32Day        = 11;
    sWriteRTC.u32DayOfWeek  = 3;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 4;
    sWriteRTC.u32Second     = 10;
    sWriteRTC.u32TimeScale  = 1;
    if( RTC_Open(&sWriteRTC) < 0 )
    {
        printf("Initialize RTC module and start counting failed\n");
        return;
    }

    /* Set RTC alarm date/time */
    sWriteRTC.u32Year       = 2016;
    sWriteRTC.u32Month      = 5;
    sWriteRTC.u32Day        = 11;
    sWriteRTC.u32DayOfWeek  = 3;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 4;
    sWriteRTC.u32Second     = 15;
    RTC_SetAlarmDateAndTime(&sWriteRTC);

    printf("# Set RTC current date/time: 2016/05/11 15:04:10.\n");
    printf("# Set RTC alarm date/time:   2016/05/11 15:04:%d.\n", sWriteRTC.u32Second);
    printf("Enter to DPD Power-down mode......\n");

    /* clear alarm status */
    RTC_CLEAR_ALARM_INT_FLAG();

    /* Enable RTC alarm interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable RTC wake-up */
    CLK_ENABLE_RTCWK();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}


/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  RTC Tamper                                                          */
/*----------------------------------------------------------------------*/
void  WakeUpRTCTamperFunction(uint32_t u32PDMode)
{
    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* RTC clock source select LXT */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Open RTC and start counting */
    if( RTC_Open(NULL) < 0 )
    {
        printf("Initialize RTC module and start counting failed\n");
        return;
    }

    /* Set RTC Tamper 0 as low level detect */
    RTC_TamperEnable(RTC_TAMPER0_SELECT, RTC_TAMPER_LOW_LEVEL_DETECT, RTC_TAMPER_DEBOUNCE_DISABLE);

    /* Clear Tamper 0 status */
    RTC_CLEAR_TAMPER_INT_FLAG(RTC_INTSTS_TAMP0IF_Msk);

    /* Disable Spare Register */
    RTC->SPRCTL = RTC_SPRCTL_SPRCSTS_Msk;

    /* Enable RTC Tamper 0 */
    RTC_EnableInt(RTC_INTEN_TAMP0IEN_Msk);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable RTC wake-up */
    CLK_ENABLE_RTCWK();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    printf("Enter to DPD Power-down mode......\n");
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                             */
/*----------------------------------------------------------------------*/
void CheckPowerSource(void)
{
    uint32_t u32RegRstsrc;
    u32RegRstsrc = CLK_GetPMUWKSrc();

    printf("Power manager Power Manager Status 0x%08X\n", u32RegRstsrc);

    if((u32RegRstsrc & CLK_PMUSTS_RTCWK_Msk) != 0)
        printf("Wake-up source is RTC.\n");
    if((u32RegRstsrc & CLK_PMUSTS_TMRWK_Msk) != 0)
        printf("Wake-up source is Wake-up Timer.\n");
    if((u32RegRstsrc & CLK_PMUSTS_WKPIN0_Msk) != 0)
        printf("Wake-up source is Wake-up Pin 0.\n");

    /* Clear all wake-up flag */
    CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;
}

/*----------------------------------------------------------------------*/
/*  Function for GPIO Setting                                           */
/*----------------------------------------------------------------------*/
void GpioPinSetting(void)
{
    /* Set function pin to GPIO mode */
    SYS->GPA_MFP0 = 0;
    SYS->GPA_MFP1 = 0;
    SYS->GPA_MFP2 = 0;
    SYS->GPA_MFP3 = 0;
    SYS->GPB_MFP0 = 0;
    SYS->GPB_MFP1 = 0;
    SYS->GPB_MFP2 = 0;
    SYS->GPB_MFP3 = 0;
    SYS->GPC_MFP0 = 0;
    SYS->GPC_MFP1 = 0;
    SYS->GPC_MFP2 = 0;
    SYS->GPC_MFP3 = 0;
    SYS->GPD_MFP0 = 0;
    SYS->GPD_MFP1 = 0;
    SYS->GPD_MFP2 = 0;
    SYS->GPD_MFP3 = 0;
    SYS->GPE_MFP0 = 0;
    SYS->GPE_MFP1 = 0;
    SYS->GPE_MFP2 = 0;
    SYS->GPE_MFP3 = 0;
    SYS->GPF_MFP0 = (SYS_GPF_MFP0_PF0MFP_ICE_DAT | SYS_GPF_MFP0_PF1MFP_ICE_CLK);
    SYS->GPF_MFP1 = 0;
    SYS->GPF_MFP2 = 0;
    SYS->GPG_MFP0 = 0;
    SYS->GPG_MFP1 = 0;
    SYS->GPG_MFP2 = 0;
    SYS->GPG_MFP3 = 0;
    SYS->GPH_MFP1 = 0;
    SYS->GPH_MFP2 = 0;

    /* Set all GPIOs are output high */
    GPIO_SET_OUT_DATA(PA, 0xFFFF);
    GPIO_SET_OUT_DATA(PB, 0xFFFF);
    GPIO_SET_OUT_DATA(PC, 0xFFFF);
    GPIO_SET_OUT_DATA(PD, 0xFFFF);
    GPIO_SET_OUT_DATA(PE, 0xFFFF);
    GPIO_SET_OUT_DATA(PF, 0xFFFF);
    GPIO_SET_OUT_DATA(PG, 0xFFFF);
    GPIO_SET_OUT_DATA(PH, 0xFFFF);

    /* Set all GPIOs are output mode */
    GPIO_SetMode(PA, 0xFFFF, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, 0xFFFF, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, 0xFFFF, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, 0xFFFF, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PE, 0xFFFF, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PF, 0xFFFF, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PG, 0xFFFF, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, 0xFFFF, GPIO_MODE_OUTPUT);

    /* Set PF.4~PF.11 as Quasi mode output high by RTC control */
    CLK_EnableModuleClock(RTC_MODULE);
    RTC->GPIOCTL1 = RTC_GPIOCTL1_DOUT7_Msk | RTC_GPIOCTL1_OPMODE7_Msk |
                    RTC_GPIOCTL1_DOUT6_Msk | RTC_GPIOCTL1_OPMODE6_Msk |
                    RTC_GPIOCTL1_DOUT5_Msk | RTC_GPIOCTL1_OPMODE5_Msk |
                    RTC_GPIOCTL1_DOUT4_Msk | RTC_GPIOCTL1_OPMODE4_Msk;
    RTC->GPIOCTL0 = RTC_GPIOCTL0_DOUT3_Msk | RTC_GPIOCTL0_OPMODE3_Msk |
                    RTC_GPIOCTL0_DOUT2_Msk | RTC_GPIOCTL0_OPMODE2_Msk |
                    RTC_GPIOCTL0_DOUT1_Msk | RTC_GPIOCTL0_OPMODE1_Msk |
                    RTC_GPIOCTL0_DOUT0_Msk | RTC_GPIOCTL0_OPMODE0_Msk;
    CLK_DisableModuleClock(RTC_MODULE);
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

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

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

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB multi-function pins for CLKO (PB.14) */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~SYS_GPB_MFP3_PB14MFP_Msk) |
                    (SYS_GPB_MFP3_PB14MFP_CLKO);

    /* Set PF multi-function pins for TAMPER0 (PF.6) */
    SYS->GPF_MFP1 = (SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) |
                    (SYS_GPF_MFP1_PF6MFP_TAMPER0);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/*  Main Function                                                       */
/*----------------------------------------------------------------------*/
int main(void)
{
    int32_t i32Item;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set IO State and all IPs clock disable for power consumption */
    GpioPinSetting();

    CLK->APBCLK0 = 0x00000000;
    CLK->APBCLK1 = 0x00000000;
    CLK->APBCLK2 = 0x00000000;

    /* ---------- Turn off RTC  -------- */
    CLK_EnableModuleClock(RTC_MODULE);
    RTC->INTEN = 0;
    CLK_DisableModuleClock(RTC_MODULE);

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    /* Unlock protected registers before setting Power-down mode */
    SYS_UnlockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 3, 0);

    /* Get power manager wake up source */
    CheckPowerSource();

    printf("+----------------------------------------------------------------+\n");
    printf("|    DPD Power-down Mode and Wake-up Sample Code.                |\n");
    printf("|    Please Select Wake up source.                               |\n");
    printf("+----------------------------------------------------------------+\n");
    printf("|[1] DPD Wake-up Pin 0 (PC.0) trigger type is rising edge.       |\n");
    printf("|[2] DPD Wake-up TIMER time-out interval is 8192 LIRC clocks.    |\n");
    printf("|[3] DPD Wake-up by RTC Tick (1 second).                         |\n");
    printf("|[4] DPD Wake-up by RTC Alarm.                                   |\n");
    printf("|[5] DPD Wake-up by RTC Tamper0 (PF.6).                          |\n");
    printf("|    Tamper pin detect voltage level is low.                     |\n");
    printf("+----------------------------------------------------------------+\n");
    i32Item = getchar();

    switch(i32Item)
    {
    case '1':
        WakeUpPinFunction(CLK_PMUCTL_PDMSEL_DPD0, CLK_DPDWKPIN_RISING);
        break;
    case '2':
        WakeUpTimerFunction(CLK_PMUCTL_PDMSEL_DPD0, CLK_PMUWKCTL_WKTMRIS_8192);
        break;
    case '3':
        WakeUpRTCTickFunction(CLK_PMUCTL_PDMSEL_DPD0);
        break;
    case '4':
        WakeUpRTCAlarmFunction(CLK_PMUCTL_PDMSEL_DPD0);
        break;
    case '5':
        WakeUpRTCTamperFunction(CLK_PMUCTL_PDMSEL_DPD0);
        break;
    default:
        break;
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
