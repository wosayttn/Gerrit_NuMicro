/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up system from SPD Power-down mode by different wakeup sources.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

void PowerDownFunction(void);
void WakeUpPinFunction(uint32_t u32PDMode);
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval);
void WakeUpLVRFunction(uint32_t u32PDMode);
void WakeUpBODFunction(uint32_t u32PDMode);
void CheckPowerSource(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    u32TimeOutCnt = SystemCoreClock;

    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up pin                         */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpPinFunction(uint32_t u32PDMode)
{
    printf("Enter to SPD Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPC_MODULE);

    /* Configure GPIO as pull-up mode */
    GPIO_SetPullCtl(PC, BIT0, GPIO_PUSEL_PULL_UP);

    /* Configure GPIO as input mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);

    /* Set Wake-up pin trigger type at deep power down mode */
    CLK_EnableSPDWKPin(2, 0, CLK_SPDWKPIN0_FALLING, CLK_SPDWKPIN0_DEBOUNCEDIS);

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up Timer                         */
/*-----------------------------------------------------------------------------------------------------------*/
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval)
{
    printf("Enter to SPD Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Set wake-up Time-out Interval */
    CLK_SET_WKTMR_INTERVAL(u32Interval);

    /* Enable wake-up timer */
    CLK_ENABLE_WKTMR();

    /* Enter to Power-down mode and wait for wake-up reset */
    PowerDownFunction();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by LVR                                 */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpLVRFunction(uint32_t u32PDMode)
{
    uint32_t u32TimeOutCnt = SystemCoreClock;
    printf("Enter to SPD Power-down mode......\n");

    /* Enable LVR function */
    SYS_ENABLE_LVR();

    /* Wiat LVR function ready */
    while ((SYS->BODCTL & SYS_BODCTL_LVRRDY_Msk) != SYS_BODCTL_LVRRDY_Msk)
    {
        if (--u32TimeOutCnt == 0)
        {
            break;
        }
    }

    printf("Real Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by BOD                                 */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpBODFunction(uint32_t u32PDMode)
{
    printf("Enter to SPD Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable Brown-out detector function */
    SYS_ENABLE_BOD();

    /* Set Brown-out detector voltage level as 2.7V */
    SYS_SET_BOD_LEVEL(SYS_BODCTL_BODVL_2_7V);

    /* Enable Brown-out detector reset function */
    SYS_ENABLE_BOD_RST();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                                                                  */
/*-----------------------------------------------------------------------------------------------------------*/
void CheckPowerSource(void)
{
    uint32_t u32RegRstsrc1;

    u32RegRstsrc1 = CLK_GetPMUWKSrc();

    printf("u32RegRstsrc1 = 0x%08X\n", u32RegRstsrc1);

    if ((u32RegRstsrc1 & CLK_PMUSTS_GPCWK0_Msk) != 0)
        printf("\nWake-up source is Wake-up Pin.\n");

    if ((u32RegRstsrc1 & CLK_PMUSTS_TMRWK_Msk) != 0)
        printf("\nWake-up source is Wake-up Timer.\n");

    if ((u32RegRstsrc1 & CLK_PMUSTS_BODWK_Msk) != 0)
        printf("\nWake-up source is BOD.\n");

    if ((u32RegRstsrc1 & CLK_PMUSTS_LVRWK_Msk) != 0)
        printf("\nWake-up source is LVR.\n");

    /* Clear all wake-up flag */
    CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;

    /* Clear all wake-up flag */
    SYS->RSTSTS = SYS->RSTSTS;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release GPIO hold status */
    CLK_RELEASE_GPIO();

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal low speed RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for Internal low speed RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t u8Item;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Get power manager wake up source */
    CheckPowerSource();

    printf("\nCPU @ %d Hz\n", SystemCoreClock);

    printf("+-----------------------------------------------------------------+\n");
    printf("|    SPD Power-down Mode and Wake-up Sample Code                  |\n");
    printf("|    Please Select Power Down Mode and Wake up source.            |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("|[1] SPD TG pin(PC.0) and using falling edge wake up.             |\n");
    printf("|[2] SPD Wake-up TIMER time-out interval is 4096 LIRC clocks.     |\n");
    printf("|[3] SPD Wake-up by BOD.                                          |\n");
    printf("|[4] SPD Wake-up by LVR.                                          |\n");
    printf("+-----------------------------------------------------------------+\n");
    u8Item = (uint8_t)getchar();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Clock Output function, output clock is stopped in Power-down mode */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 3, FALSE);

    /* Disable wake-up timer */
    CLK_DISABLE_WKTMR();

    switch (u8Item)
    {
        case '1':
            WakeUpPinFunction(CLK_PMUCTL_PDMSEL_SPD0);
            break;

        case '2':
            WakeUpTimerFunction(CLK_PMUCTL_PDMSEL_SPD0, CLK_PMUCTL_WKTMRIS_4096);
            break;

        case '3':
            WakeUpBODFunction(CLK_PMUCTL_PDMSEL_SPD1);
            break;

        case '4':
            WakeUpLVRFunction(CLK_PMUCTL_PDMSEL_SPD1);
            break;

        default:
            break;
    }

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}
