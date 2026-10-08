/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to measure AVDD voltage by EADC.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define VBG_VOLTAGE (1200) /* 1.20V = 1200 mV (Typical band-gap voltage) */
#define ADC_SAMPLE_COUNT 128 /* The last line of GetAVDDCodeByADC() needs to be revised when ADC_SAMPLE_COUNT is changed. */
/* For example, if ADC_SAMPLE_COUNT is changed to 64, then the code need revised to "return (u32Sum >> 7);" */

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t s_u8ADF;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
uint32_t GetAVDDVoltage(void);
uint32_t GetAVDDCodeByADC(void);
NVT_ITCM void EADC0_IRQHandler(void);

/*---------------------------------------------------------------------------------------------------------*/
/* EADC IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void EADC00_IRQHandler(void)
{
    uint32_t u32Flag;

    /* Get ADC conversion finish interrupt flag */
    u32Flag = EADC_GET_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

    /* Check ADC conversion finish */
    if (u32Flag & EADC_STATUS2_ADIF0_Msk)
        s_u8ADF = 1;

    /* Clear conversion finish flag */
    EADC_CLR_INT_FLAG(EADC0, u32Flag);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

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

}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: GetAVDDVoltage                                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   AVDD voltage(mV).                                                                                     */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Use Band-gap voltage to calculate AVDD voltage                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetAVDDVoltage(void)
{
    uint32_t  u32ConversionResult;
    uint64_t  u64MvAVDD;

    /* Calculate Vref by using conversion result of VBG */
    u32ConversionResult = GetAVDDCodeByADC();

    /* u32ConversionResult = VBG * 4096 / Vref, Vref = AVDD */
    /* => AVDD = VBG * 4096 / u32ConversionResult */
    u64MvAVDD = (VBG_VOLTAGE << 12) / (uint64_t)u32ConversionResult;

    printf("Conversion result: 0x%X\n", u32ConversionResult);

    return (uint32_t)u64MvAVDD;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: GetAVDDCodeByADC                                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   ADC code of AVDD voltage.                                                                             */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Get EADC conversion result of Band-gap voltage.                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetAVDDCodeByADC(void)
{
    uint32_t u32Count, u32Sum, u32Data;

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 24 external sampling time to 0xFF */
    EADC_SetExtendSampleTime(EADC0, 24, 0xFF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 24 interrupt. */
    EADC_ENABLE_INT(EADC0, BIT0);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT24);//Enable sample module 24 interrupt.
    NVIC_EnableIRQ(EADC00_IRQn);

    s_u8ADF = 0;
    u32Sum = 0;

    /* sample times are according to ADC_SAMPLE_COUNT definition */
    for (u32Count = 0; u32Count < ADC_SAMPLE_COUNT; u32Count++)
    {
        /* Delay for band-gap voltage stability */
        CLK_SysTickDelay(100);

        /* Start A/D conversion */
        EADC_START_CONV(EADC0, BIT24);

        u32Data = 0;

        /* Wait conversion done */
        while (s_u8ADF == 0);

        s_u8ADF = 0;
        /* Get the conversion result */
        u32Data = EADC_GET_CONV_DATA(EADC0, 24);
        /* Sum each conversion data */
        u32Sum += u32Data;
    }

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC0, BIT0);

    /* Disable ADC */
    EADC_Close(EADC0);

    /* Return the average of ADC_SAMPLE_COUNT samples */
    return (u32Sum >> 7);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Main Function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32AVDDVoltage = 0;

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
    printf("+----------------------------------------+\n");
    printf("|   EADC Voltage Detection Sample Code   |\n");
    printf("+----------------------------------------+\n\n");

    printf("In this sample code, software will get voltage value from AVDD.\n");
    printf("Notice that the Vref of EADC is from AVDD.\n\n");

    /* Measure AVDD */
    u32AVDDVoltage = GetAVDDVoltage();

    if (u32AVDDVoltage < 1620 || u32AVDDVoltage > 3600)
        printf("Abnormal AVDD Voltage: %dmV\n", u32AVDDVoltage);
    else
        printf("AVDD Voltage: %dmV\n", u32AVDDVoltage);

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
