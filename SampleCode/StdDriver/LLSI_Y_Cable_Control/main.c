/******************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is a LLSI demo for Y-cable control.
 *           It needs to be used with AP6112Y LED strip.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "global_variable.h"
#include "hdiv.h"
#include "led_control.h"
#include "led_gen2_control.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable PDMA0 peripheral clock */
    CLK_EnableModuleClock(PDMA0_MODULE);
    /* Enable GPB/GPC peripheral clock for CMD/ACK */
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

void SysTick_Initial(void)
{
    /* Set SysTick clock source to HCLK */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;

    /* Set load value to make SysTick period is 1 ms */
    SysTick->LOAD = __HSI / 1000;

    /* Clear SysTick counter */
    SysTick->VAL = 0;

    /* Enable SysTick interrupt */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    /* Enable SysTick NVIC */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Change interrupt priority to normal */
    NVIC_SetPriority(SysTick_IRQn, INT_PRIORITY_NORMAL);

    /* Start SysTick interrupt */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
    uint8_t i, j;

    /* Clear interrupt flag */
    SysTick->VAL = 0;

    /* Add LED mode counter */
    TimeCounterFrameUpdate++;
    StripA_LEDSetting.TimeCounter++;
    StripB_LEDSetting.TimeCounter++;
    FAN1_LEDSetting.TimeCounter++;
    FAN2_LEDSetting.TimeCounter++;
    FAN3_LEDSetting.TimeCounter++;
    FAN4_LEDSetting.TimeCounter++;
    FAN5_LEDSetting.TimeCounter++;
    FAN6_LEDSetting.TimeCounter++;
    FAN7_LEDSetting.TimeCounter++;
    FAN8_LEDSetting.TimeCounter++;

    /* LED Gen2 */
    for (i = 0; i < LED_GEN2_MAX_SUPPORT_PORT; i++)
    {
        for (j = 0; j < LED_Gen2_Port_Setting[i].Strip_Count; j++)
            LED_Gen2_Port_Setting[i].LED_Gen2_Setting[j + 1].LEDSetting.TimeCounter++;
    }

    /* Update LED frame data */
    if (TimeCounterFrameUpdate >= TIMER_FRAME_UPDATE)
    {
        LED_Frame_Update_flag = 1;
        TimeCounterFrameUpdate -= TIMER_FRAME_UPDATE;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN Function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t i, j, k;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock: %d\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    LLSI Y-cable Control Sample Code    |\n");
    printf("+----------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with AP6112Y LED strip.\n\n");

    /* Set LED effects */
    LED_Effect_Setting(0x0A, 255, 0, 0, 0x0F);

    /* Set Serial LED configuration */
    for (i = 0; i < TOTAL_LED_AREA; i++)
    {
        PDMA_Initial(i);
        /* Clear Color Data */
        Clear_LED_Data(LED_Mapping[i]);
        LLSI_Initial(i);
    }

    for (i = 0; i < TOTAL_LED_AREA; i++)
        while (!LED_Mapping[i]->fPDMA_Done);

    /* Detect LED Gen2 */
    Init_Gen2_LED_Capture();

    for (i = 0; i < LED_GEN2_MAX_SUPPORT_PORT; i++)
    {
        /* Initial LED Gen2 Setting */
        LED_Gen2_Initial_Setting(i);
        LED_Gen2_LLSI_PDMA_Init(i);
        /* Detect LED Gen2 */
        LED_Gen2_Set_Ctrl_Setting(i);
        LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Status = 0x01;
        LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Flag = 1;

        /* Start detect */
        while (LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Control_Flag == 1)
        {
            LED_Gen2_Control_Port();
        }

        /* LED Gen2 Test */
        LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Use_Gen2 = TRUE;

        /* Enable/Disable LED Gen2 */
        LED_Gen2_Enable_Control(i, LED_Gen2_Port_Setting[Gen2_Ctrl.Current_Port].Use_Gen2);
    }

    /* Initial SysTick, enable interrupt and 1000 interrupt tick per second to add counter */
    SysTick_Initial();

    /* Start SysTick */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    /* Use SysTick counter as seed of random function */
    srand(SysTick->VAL);

    while (1)
    {
        /* Serial LED Control */
        if (LED_Frame_Update_flag)
        {
            /* Check no LED Gen2 control process */
            for (j = 0; j < TOTAL_LED_AREA; j++)
            {
                if (LED_Gen2_Port_Setting[j].Control_Flag == 1)
                    goto EXIT;
            }

            for (j = 0; j < TOTAL_LED_AREA; j++)
            {
                /* Use Gen1 LED */
                if (LED_Gen2_Port_Setting[j].Use_Gen2 == 0)
                {
                    if (LED_Mapping[j]->AP_Sync == 0)
                    {
                        for (i = 0; i < LED_Mapping[j]->Array_Size; i++)
                        {
                            *(LED_Mapping[j]->LED_Data + i) = 0x0;
                        }

                        /* LED data */
                        LED_Mapping[j]->Mode_FUNC(LED_Mapping[j]);
                    }

                    /* Set Serial Data */
                    Set_LED_Data(LED_Mapping[j]);
                }
                else
                {
                    LED_Gen2_Control(j);
                }
            }

            for (k = 0; k < TOTAL_LED_AREA; k++)
                LLSI_SET_PDMA_MODE(LLSI_Port_Mapping[k]);

EXIT:
            /* Clear flag */
            LED_Frame_Update_flag = 0;
        }

        /* Control LED Gen2 on target port */
        for (j = 0; j < TOTAL_LED_AREA; j++)
        {
            if ((LED_Mapping[j]->fPDMA_Done == 1) && (LED_Gen2_Port_Setting[j].Control_Flag == 1))
            {
                /* Check whole data transmission done */
                for (k = 0; k < TOTAL_LED_AREA; k++)
                {
                    if (LED_Mapping[k]->fPDMA_Done == 0)
                        goto WAIT;
                }

                /* Set active target port */
                if (LED_Gen2_Port_Setting[j].Control_Status == 0x01)
                {
                    /* Initial LED Gen2 Setting */
                    LED_Gen2_Initial_Setting(j);
                    LED_Gen2_LLSI_PDMA_Init(j);
                    LED_Gen2_Set_Ctrl_Setting(j);
                    /* Restore setting */
                    LED_Gen2_Port_Setting[j].Control_Status = 0x01;
                    LED_Gen2_Port_Setting[j].Control_Flag = 1;
                }
                else if (LED_Gen2_Port_Setting[j].Control_Status == 0x20)
                {
                    /* Initial LED Gen2 Setting */
                    LED_Gen2_Set_Ctrl_Setting(j);
                }

                LED_Gen2_Control_Port();
            }
        }

        /* Polling if PDMA aborts */
        Polling_PDMA_Abort();

WAIT:
        __NOP();
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
