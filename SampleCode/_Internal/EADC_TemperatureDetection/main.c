/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to measure temperature by EADC.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define VREF_VOLTAGE (3.3)

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32AdcIntFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
double GetTemperature(void);
uint32_t GetTemperatureCodeFromADC(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* EADC IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void EADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);
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

    /* Enable EADC peripheral clock */
    CLK_SetModuleClock(EADC0_MODULE, CLK_EADCSEL_EADC0SEL_PCLK0, CLK_EADCDIV_EADC0DIV(15));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Enable temperature sensor */
    SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Get temperature                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
double GetTemperature(void)
{
    /* The equation of converting to real temperature is as below
    *      Vtemp = Tc * (temperature - Ta) + dmVoffset
    *      Vtemp = EADC_result / 4095 * ADC_Vref
    *      so, temperature = Ta + (Vtemp - Vtemp_os) / Tc
    *                      = Ta + ((dTemperatureData / 4095 * ADC_Vref) - dmVoffset) / dTc
    *      where Vtemp_os, Tc, and Ta can be got from the data sheet document.
    *            ADC_Vref is the ADC Vref that according to the configuration of SYS and EADC.
    */
    uint32_t u32Ta = 25;
    double dmVoffset = 1030;
    double dTc = -2.7;
    double dTemperatureData = 0;
    double dmVT = 0;
    double dT;

    /* Get ADC code of temperature sensor */
    dTemperatureData = GetTemperatureCodeFromADC();
    /* ADC code to voltage conversion formula: */
    dmVT = (dTemperatureData  / 4095) * 3300;

    /* Get temperature with temperature sensor */
    dT = u32Ta + ((dmVT - dmVoffset) / (dTc));

    return (dT);
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetTemperatureCodeFromADC(void)
{

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 25 external sampling time to 0xFF */
    EADC_SetExtendSampleTime(EADC0, 25, 0xFF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 25 interrupt.  */
    EADC_ENABLE_INT(EADC0, BIT0);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT25);//Enable sample module 25 interrupt.
    NVIC_EnableIRQ(EADC00_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 25 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC0, BIT25);

    /* Wait EADC conversion done */
    while (g_u32AdcIntFlag == 0);

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC0, BIT0);

    /* Return the conversion result of the sample module 25 */

    return EADC_GET_CONV_DATA(EADC0, 25);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    double dTemperature = 0.0;

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


    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|   EADC Tamperature Detection Sample Code   |\n");
    printf("+--------------------------------------------+\n\n");

    /* Measure temperature */
    dTemperature = GetTemperature();

    if (dTemperature > 50)
        printf("Abnormal temperature: %2.2f C\n", dTemperature);
    else
        printf("Chip temperature: %2.2f C\n", dTemperature);

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Reset EADC module */
    SYS_ResetModule(SYS_EADC0RST);
    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);

    while (1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
