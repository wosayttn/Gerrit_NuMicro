/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up MCU from Power-down mode by ACMP wake-up function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define NPD0_MODE   0    // Power-down mode 0
#define NPD1_MODE   1    // Power-down mode 1
#define NPD2_MODE   2    // Power-down mode 2

/*---------------------------------------------------------------------------------------------------------*/
/*                                       Global variables                                                  */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                 Define functions prototype                                              */
/*---------------------------------------------------------------------------------------------------------*/
void ACMP01_IRQHandler(void);
void EnterToPowerDown(uint32_t u32PDMode);
void SYS_Init(void);
int32_t main(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*                                         ACMP01  Handle                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void ACMP01_IRQHandler(void)
{
    printf("\nACMP1 interrupt!\n");
    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);
    /* Clear wake-up interrupt flag */
    ACMP_CLR_WAKEUP_INT_FLAG(ACMP01, 1);
}

/**
  * @brief      Enter To Power Down
  * @param[in]   u32PDMode    The specified Power down module.
  *                               - \ref CLK_PMUCTL_PDMSEL_PD      : Power-down mode
  *                               - \ref CLK_PMUCTL_PDMSEL_FWPD    : Fast wake up
  *
  * @return     None
  *
  * @details    This API is used to get the current RTC date and time value.
  */

void EnterToPowerDown(uint32_t u32PDMode)
{
    SYS_UnlockReg();
    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;

    if (u32PDMode == NPD0_MODE)
        CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_NPD0);   //Power down
    else if (u32PDMode == NPD1_MODE)
        CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_NPD1);   //Power down
    else if (u32PDMode == NPD2_MODE)
        CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_NPD2);   //Power down

    /* Set system Power-down enabled and Power-down entry condition */
    CLK_PowerDown();
    SYS_LockReg();
}



void SYS_Init(void)
{

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PB4 multi-function pin for ACMP1 positive input pin and PC0 multi-function pin for ACMP1 output pin*/
    SET_ACMP1_P1_PB4();
    SET_ACMP1_O_PC0();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));

}

/*
 * When the voltage of the positive input is greater than the voltage of the negative input,
 * the analog comparator outputs logical one; otherwise, it outputs logical zero.
 * This chip will be waked up from power down mode when detecting a transition of analog comparator's output.
 */

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nThis sample code demonstrates ACMP1 function. Using ACMP1_P1 (PB4) as ACMP1\n");
    printf("positive input and using internal CRV as the negative input.\n");
    printf("The compare result reflects on ACMP1_O (PC0).\n");

    /* Configure ACMP1. Enable ACMP1 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_CRV, ACMP_CTL_HYSTERESIS_DISABLE);

    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV1_SRC(ACMP01, ACMP_VREF_CRV1SSEL_VDDA);
    /* Select CRV1 level: VDDA * 127 / 255 */
    ACMP_CRV1_SEL(ACMP01, 127);

    /* Select P1 as ACMP positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);

    CLK_SysTickDelay(10); /* For ACMP setup time */

    /* Clear ACMP1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    /* Enable wake-up function */
    ACMP_ENABLE_WAKEUP(ACMP01, 1);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 1);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while (1)
    {
        printf("\nSystem enter power-down mode ... \n");

        /* To check if all the debug messages are finished */
        UART_WAIT_TX_EMPTY(DEBUG_PORT);

        EnterToPowerDown(NPD0_MODE);

        printf("Wake up by ACMP1!\n");
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
