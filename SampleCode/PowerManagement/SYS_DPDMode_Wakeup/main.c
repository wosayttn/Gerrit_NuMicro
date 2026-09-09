/**************************************************************************//**
 * @file     main.c
 * @version  V1.01
 * @brief    Show how to wake up system from DPD Power-down mode by Wake-up pin 0 (PC.0)
 *           or Wake-up Timer or RTC Tick or RTC Alarm or RTC Tamper 0.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
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
    SYS->GPA_MFPL = 0;
    SYS->GPA_MFPH = 0;
    SYS->GPB_MFPL = 0;
    SYS->GPB_MFPH = 0;
    SYS->GPC_MFPL = 0;
    SYS->GPC_MFPH = 0;
    SYS->GPD_MFPL = 0;
    SYS->GPD_MFPH = 0;
    SYS->GPE_MFPL = 0;
    SYS->GPE_MFPH = 0;
    SYS->GPF_MFPL = (SYS_GPF_MFPL_PF0MFP_ICE_DAT | SYS_GPF_MFPL_PF1MFP_ICE_CLK);
    SYS->GPF_MFPH = 0;
    SYS->GPG_MFPL = 0;
    SYS->GPH_MFPH = 0;

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
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release I/O hold status */
    CLK->IOPDCTL = 1;

    /* Set PF multi-function pins for X32_OUT (PF.4) and X32_IN (PF.5) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) |
                    (SYS_GPF_MFPL_PF4MFP_X32_OUT);
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) |
                    (SYS_GPF_MFPL_PF5MFP_X32_IN);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    CLK_EnableModuleClock(GPC_MODULE);

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB multi-function pins for CLKO (PB.14) */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) |
                    (SYS_GPB_MFPH_PB14MFP_CLKO);

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

    CLK->AHBCLK0 = 0x00000000;
    CLK->APBCLK0 = 0x00000000;

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
    printf("+----------------------------------------------------------------+\n");
    i32Item = getchar();

    switch(i32Item)
    {
    case '1':
        WakeUpPinFunction(CLK_PMUCTL_PDMSEL_DPD0, CLK_DPDWKPIN_RISING);
        break;
    default:
        break;
    }

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
