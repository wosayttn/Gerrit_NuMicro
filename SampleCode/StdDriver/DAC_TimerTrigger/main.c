/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to trigger DAC by timer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


static const uint16_t g_au16Sine[] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                                      4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                                      3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
                                      639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
                                      238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876
                                     };

static const uint32_t g_u32ArraySize = sizeof(g_au16Sine) / sizeof(uint16_t);
static uint32_t g_u32Index = 0;

NVT_ITCM void DAC01_IRQHandler(void);
void SYS_Init(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

NVT_ITCM void DAC01_IRQHandler(void)
{
    if (DAC_GET_INT_FLAG(DAC0, 0))
    {

        if (g_u32Index == g_u32ArraySize)
            g_u32Index = 0;
        else
        {
            uint32_t u32TimeOutCnt = 1000;
            /* Clear the DAC conversion complete finish flag */
            DAC_CLR_INT_FLAG(DAC0, 0);
            DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index++]);

            /* Sync-up STATUS register of DAC. */
            while (DAC_GET_INT_FLAG(DAC0, 0) != 0)
            {
                if ((--u32TimeOutCnt) == 0)
                    break;
            }
        }
    }

    return;
}


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);
    /* Set PCLK1 divide 4 */
    CLK_SET_PCLK1DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable DAC01 module clock */
    CLK_EnableModuleClock(DAC01_MODULE);

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select Timer 0 module clock source as HXT */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);

    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set multi-function pin for DAC voltage output */
    SET_DAC0_OUT_PB12();

    /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 12));

}


int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          DAC Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("\nStart to trigger DAC by Timer 0!\n");

    /* Set the timer 0 trigger,enable DAC even trigger mode and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_TIMER0_TRIGGER);

    /* The DAC conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index]);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the DAC interrupt.  */
    DAC_ENABLE_INT(DAC0, 0);
    NVIC_EnableIRQ(DAC01_IRQn);

    /* Configure Timer 0 frequency to 1000Hz and start it. */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_DAC);
    TIMER_Start(TIMER0);

    while (1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
