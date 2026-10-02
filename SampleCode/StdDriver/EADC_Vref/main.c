/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to select EADC reference voltage source and trigger delay time.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/1 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable EADC peripheral clock */
    /* Note: The EADC_CLK speed should meet datasheet spec (<36MHz) and rules in following table.   */
    /* +--------------+------------------+                                                          */
    /* | PCLK divider | EADC_CLK divider |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 1            | 1, 2, 3, 4, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 2, 4, 8, 16  | 2, 4, 6, 8, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(72));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

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

void EADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData;

    printf("\n");
    printf("+---------------------------------------------------+\n");
    printf("|            EADC Reference Voltage test            |\n");
    printf("+---------------------------------------------------+\n");

    while(1)
    {
        printf("Select EADC Reference Voltage source for Band-gap:\n");
        printf("  [1] AVdd (default setting)\n");
        printf("  [2] External Vref pin\n");
        printf("  [3] Internal Vref 3.072V\n");
        printf("  Other keys: exit test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            printf("Set EADC reference voltage to AVdd.\n");
            EADC_SetVRef(EADC, EADC_VREF_REFSEL_AVDD);
        }
        else if(u8Option == '2')
        {
            printf("Set EADC reference voltage to External Vref pin.\n");
            SYS_UnlockReg();
            SYS_SetVRef(SYS_VREFCTL_VREF_PIN);
            SYS_LockReg();
            EADC_SetVRef(EADC, EADC_VREF_REFSEL_VREFP);
        }
        else if(u8Option == '3')
        {
            printf("Set EADC reference voltage to Internal Vref 3.072V.\n");
            SYS_UnlockReg();
            SYS_SetVRef(SYS_VREFCTL_VREF_3_0V);
            SYS_LockReg();
            EADC_SetVRef(EADC, EADC_VREF_REFSEL_VREFP);
        }
        else
        {
            printf("Wrong EADC reference voltage setting %c !!\n", u8Option);
            return;
        }

        /* Set input mode as single-end and enable the A/D converter */
        EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

        /* Configure the sample module 24 for Band-gap voltage and software trigger source.*/
        EADC_ConfigSampleModule(EADC, 24, EADC_SOFTWARE_TRIGGER, (uint32_t)NULL);

        /* Configure the trigger delay time of module 24 to (255 * 16 * EADC clock period).*/
        EADC_SetTriggerDelayTime(EADC, 24, 255, EADC_SCTL_TRGDLYDIV_DIVIDER_16);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

        /* Enable ADINT0 interrupt. */
        EADC_ENABLE_INT(EADC, BIT0);
        /* Enable the sample module 24 interrupt and bind to ADINT0. */
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT24);
        NVIC_EnableIRQ(EADC0_INT0_IRQn);

        /* Reset the ADC interrupt indicator and trigger sample module 24 to start A/D conversion */
        g_u32AdcIntFlag = 0;
        EADC_START_CONV(EADC, BIT24);

        /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
        while(g_u32AdcIntFlag == 0);

        /* Disable the ADINT0 interrupt */
        EADC_DISABLE_INT(EADC, BIT0);

        /* Get the conversion result of the sample module 24 */
        i32ConversionData = EADC_GET_CONV_DATA(EADC, 24);
        printf("Conversion result of Band-gap: 0x%X (%d)\n", i32ConversionData, i32ConversionData);
        printf("EADC reference voltage = %d mV if Band-gap = 1200 mV\n\n", 4095 * 1200 / i32ConversionData);
    }
}


void EADC0_INT0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);    /* Clear the A/D ADINT0 interrupt flag */
}


int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while(1);
}
