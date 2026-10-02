/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up system from SPD Power-down mode by Wake-up pin (PC.0)
 *           or Wake-up Timer or Wake-up ACMP or RTC Tick or RTC Alarm or RTC Tamper 0
 *           or BOD or LVR.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

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
/*  GPIO Wake-up pin                                                    */
/*----------------------------------------------------------------------*/
void WakeUpPinFunction(uint32_t u32PDMode)
{
    printf("Enter to SPD0 Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Configure GPIO as input mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);

    /* GPIO SPD Power-down Wake-up Pin Select */
    CLK_EnableSPDWKPin(2, 0, CLK_SPDWKPIN_RISING, CLK_SPDWKPIN_DEBOUNCEDIS);

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  Wake-up Timer                                                       */
/*----------------------------------------------------------------------*/
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval)
{
    printf("Enter to SPD0 Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Set Wake-up Timer Time-out Interval */
    CLK_SET_WKTMR_INTERVAL(u32Interval);

    /* Enable Wake-up Timer */
    CLK_ENABLE_WKTMR();

    /* Enter to Power-down mode */
    PowerDownFunction();
}


/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  Wake-up ACMP0                                                       */
/*----------------------------------------------------------------------*/
void WakeUpACMP0Function(uint32_t u32PDMode)
{
    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Set PA11 multi-function pin for ACMP0 positive input pin */
    SYS->GPA_MFP2 = (SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) |
                    (SYS_GPA_MFP2_PA11MFP_ACMP0_P0);
    /* Disable digital input path of analog pin ACMP0_P0 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PA, BIT11);

    /* Set PB7 multi-function pin for ACMP0 output pin */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) |
                    (SYS_GPB_MFP1_PB7MFP_ACMP0_O);

    printf("\nUsing ACMP0_P0 (PA11) as ACMP0 positive input.\n");
    printf("Using internal band-gap voltage as the negative input.\n\n");
    printf("Enter to SPD0 Power-down mode......\n");

    /* Configure ACMP0. Enable ACMP0 and select band-gap voltage as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 0, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 0);
    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);

    /* Enable wake-up function */
    ACMP_ENABLE_WAKEUP(ACMP01, 0);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable Wake-up ACMP */
    CLK_ENABLE_SPDACMP();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  RTC Tick                                                            */
/*----------------------------------------------------------------------*/
void  WakeUpRTCTickFunction(uint32_t u32PDMode)
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

    /* clear tick status */
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
    printf("Enter to SPD0 Power-down mode......\n");
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  RTC Alarm                                                           */
/*----------------------------------------------------------------------*/
void WakeUpRTCAlarmFunction(uint32_t u32PDMode)
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
    printf("Enter to SPD0 Power-down mode......\n");

    /* Clear alarm status */
    RTC_CLEAR_ALARM_INT_FLAG();

    /* Enable RTC alarm interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable RTC wake-up */
    CLK_ENABLE_RTCWK();

    /* Enter to Power-down mode */
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
    RTC->SPRCTL = (1 << 5);

    /* Enable RTC Tamper 0 */
    RTC_EnableInt(RTC_INTEN_TAMP0IEN_Msk);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable RTC wake-up */
    CLK_ENABLE_RTCWK();

    /* Enter to Power-down mode */
    printf("Enter to SPD0 Power-down mode......\n");
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  LVR                                                                 */
/*----------------------------------------------------------------------*/
void WakeUpLVRFunction(uint32_t u32PDMode)
{
    printf("Enter to SPD0 Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enter to Power-down mode */
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by  */
/*  BOD                                                                 */
/*----------------------------------------------------------------------*/
void WakeUpBODFunction(uint32_t u32PDMode)
{
    printf("Enter to SPD0 Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable Brown-out detector function */
    SYS_ENABLE_BOD();

    /* Set Brown-out detector voltage level as 3.0V */
    SYS_SET_BOD_LEVEL(SYS_BODCTL_BODVL_3_0V);

    /* Enable Brown-out detector reset function */
    SYS_ENABLE_BOD_RST();

    /* Enter to Power-down mode */
    PowerDownFunction();
}

/*----------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                             */
/*----------------------------------------------------------------------*/
void CheckPowerSource(void)
{
    uint32_t u32RegRstsrc;
    u32RegRstsrc = CLK_GetPMUWKSrc();

    printf("Power manager Power Manager Status 0x%x\n", u32RegRstsrc);

    if((u32RegRstsrc & CLK_PMUSTS_ACMPWK0_Msk) != 0)
        printf("Wake-up source is ACMP.\n");
    if((u32RegRstsrc & CLK_PMUSTS_RTCWK_Msk) != 0)
        printf("Wake-up source is RTC.\n");
    if((u32RegRstsrc & CLK_PMUSTS_TMRWK_Msk) != 0)
        printf("Wake-up source is Wake-up Timer.\n");
    if((u32RegRstsrc & CLK_PMUSTS_GPCWK0_Msk) != 0)
        printf("Wake-up source is GPIO PortC.\n");
    if((u32RegRstsrc & CLK_PMUSTS_LVRWK_Msk) != 0)
        printf("Wake-up source is LVR.\n");
    if((u32RegRstsrc & CLK_PMUSTS_BODWK_Msk) != 0)
        printf("Wake-up source is BOD.\n");

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

    /* Enable HIRC and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LXTSTB_Msk);

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
int32_t main(void)
{
    int32_t i32Item;

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

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    /* Unlock protected registers before setting Power-down mode */
    SYS_UnlockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 3, 0);

    /* Get power manager wake up source */
    CheckPowerSource();

    printf("+-----------------------------------------------------------------+\n");
    printf("|    SPD Power-down Mode and Wake-up Sample Code                  |\n");
    printf("|    Please Select Power Down Mode and Wake up source.            |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("|[1] SPD GPIO Wake-up pin (PC.0) and using rising edge wake up.   |\n");
    printf("|[2] SPD Wake-up TIMER time-out interval is 1024 LIRC clocks.     |\n");
    printf("|[3] SPD Wake-up by ACMP0. (band-gap voltage)                     |\n");
    printf("|[4] SPD Wake-up by RTC Tick.                                     |\n");
    printf("|[5] SPD Wake-up by RTC Alarm.                                    |\n");
    printf("|[6] SPD Wake-up by RTC Tamper0 (PF.6), Low level.                |\n");
    printf("|[7] SPD Wake-up by BOD.                                          |\n");
    printf("|[8] SPD Wake-up by LVR.                                          |\n");
    printf("+-----------------------------------------------------------------+\n");
    i32Item = getchar();

    switch(i32Item)
    {
    case '1':
        WakeUpPinFunction(CLK_PMUCTL_PDMSEL_SPD0);
        break;
    case '2':
        WakeUpTimerFunction(CLK_PMUCTL_PDMSEL_SPD0, CLK_PMUWKCTL_WKTMRIS_8192);
        break;
    case '3':
        WakeUpACMP0Function(CLK_PMUCTL_PDMSEL_SPD0);
        break;
    case '4':
        WakeUpRTCTickFunction(CLK_PMUCTL_PDMSEL_SPD0);
        break;
    case '5':
        WakeUpRTCAlarmFunction(CLK_PMUCTL_PDMSEL_SPD0);
        break;
    case '6':
        WakeUpRTCTamperFunction(CLK_PMUCTL_PDMSEL_SPD0);
        break;
    case '7':
        WakeUpBODFunction(CLK_PMUCTL_PDMSEL_SPD0);
        break;
    case '8':
        WakeUpLVRFunction(CLK_PMUCTL_PDMSEL_SPD0);
        break;
    default:
        break;
    }

    /* Wait for Power-down mode wake-up reset happen */
    while(1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
