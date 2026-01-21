/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement MBI5153 LED by PSIO.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "MBI5153_driver_LED.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_BLANK,
} LED_PATTERN_T;

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
FRAME_BUF_t sLED_Pattern = {0};

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO_MODULE);

    /* Select PSIO module clock source as HXT and PSIO module clock divider as 1 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_HIRC, CLK_CLKDIV1_PSIO(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.2) and TXD(PD.3) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD2MFP_UART0_RXD | SYS_GPD_MFPL_PD3MFP_UART0_TXD);

    /* PSIO multi-function pin CH0(PB.15) and CH1(PB.14) */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB15MFP_Msk) | SYS_GPB_MFPH_PB15MFP_PSIO0_CH0;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) | SYS_GPB_MFPH_PB14MFP_PSIO0_CH1;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB13MFP_Msk) | SYS_GPB_MFPH_PB13MFP_PSIO0_CH2;

    /* Set timer toggle out pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk) | SYS_GPB_MFPL_PB5MFP_TM0;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main()
{
    LED_PATTERN_T color = COLOR_GREEN;
    int i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 8000000);

    // Fastest possible timer working freq is (u32Clk / 2). While cmpr = 2, pre-scale = 0.
    TIMER_PORT->CTL = TIMER_TOGGLE_MODE;
    TIMER_PORT->CMP = 2;

    printf("******************************************************\n");
    printf("|               LED PANEL SAMPLE CODE                |\n");
    printf("******************************************************\n");

    MBI5153_Open();

    while (1)
    {
        switch (color)
        {
            case COLOR_RED:
#if (PATTERN_ON_FLASH==1)
                sLED_Pattern.pu32Pattern = (uint32_t *)RED_COLOR;
#else

                for (i = 0; i < LED_PANEL_WIDTH * LED_PANEL_HEIGHT; i++)
                {
                    sLED_Pattern.red[i]     = 0xFFFF;
                    sLED_Pattern.green[i]   = 0x0;
                    sLED_Pattern.blue[i]    = 0x0;
                }

#endif
                color = COLOR_GREEN;
                break;

            case COLOR_GREEN:
#if (PATTERN_ON_FLASH==1)
                sLED_Pattern.pu32Pattern = (uint32_t *)GREEN_COLOR;
#else

                for (i = 0; i < LED_PANEL_WIDTH * LED_PANEL_HEIGHT; i++)
                {
                    sLED_Pattern.red[i]     = 0;
                    sLED_Pattern.green[i]   = 0xFFFF;
                    sLED_Pattern.blue[i]    = 0x0;
                }

#endif
                color = COLOR_BLUE;
                break;

            case COLOR_BLUE:
#if (PATTERN_ON_FLASH==1)
                sLED_Pattern.pu32Pattern = (uint32_t *)BLUE_COLOR;
#else

                for (i = 0; i < LED_PANEL_WIDTH * LED_PANEL_HEIGHT; i++)
                {
                    sLED_Pattern.red[i]     = 0x0;
                    sLED_Pattern.green[i]   = 0x0;
                    sLED_Pattern.blue[i]    = 0xFFFF;
                }

#endif
                color = COLOR_BLANK;
                break;

            case COLOR_BLANK:
#if (PATTERN_ON_FLASH==1)
                sLED_Pattern.pu32Pattern = (uint32_t *)BLANK_COLOR;
#else

                for (i = 0; i < LED_PANEL_WIDTH * LED_PANEL_HEIGHT; i++)
                {
                    sLED_Pattern.red[i]     = 0x0;
                    sLED_Pattern.green[i]   = 0x0;
                    sLED_Pattern.blue[i]    = 0x0;
                }

#endif
                color = COLOR_RED;
                break;
        }

#if (PATTERN_ON_FLASH==1)
        MBI5153_LoadPattern(sLED_Pattern);
        MBI5153_SetGrayScale();

        while (MBI5153_CheckStatus() == TRANSFER_BUSY);

#else

        for (i = 0; i < MBI5152_SCAN_LINE_NUM; i++)
        {
            MBI5153_LoadPattern(&sLED_Pattern, i);
            MBI5153_SetGrayScale();

            while (MBI5153_CheckStatus() == TRANSFER_BUSY);
        }

#endif
        MBI5153_verticalSync();
        CLK_SysTickDelay(1000000);
    }

    MBI5153_Close();
}


/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
