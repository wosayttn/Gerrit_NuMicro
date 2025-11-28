/**************************************************************************//**
 * @file     ec_adc.c
 * @version  V0.10
 * @brief    The driver use timer to trigger EADC periodicly to generate VBUS and VCONN Voltage.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "utcpdlib.h"

volatile uint32_t g_u32AdcIntFlag0;
volatile uint32_t g_u32AdcIntFlag1;
volatile uint32_t g_u32AdcIntFlag2;

void EADC0_INT0_IRQHandler(void)
{
    g_u32AdcIntFlag0 = 1;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}
void EADC0_INT1_IRQHandler(void)
{
    g_u32AdcIntFlag1 = 1;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

/**
	* Using Timer 1 trigger ADC to converse VBUS, VCONN and OCP every 10ms
  * C
  *
  **/
uint32_t gu32RefVolmV;
volatile uint32_t gu32ADCV;
uint32_t gu32CurrmA;
void EADC0_INT2_IRQHandler(void)
{
    g_u32AdcIntFlag2 = 1;
    gu32ADCV = EADC_GET_CONV_DATA(EADC, 2);
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

void EADC_SetReferenceVoltage(uint32_t u32RefmV)
{
    gu32RefVolmV = u32RefmV;
}

#if 0 
/**
	* It is Used to Measuure VBUS Current if MAD026 UTCPD_V1  
  *   Vdiff : Vref = ADC : 4096
  *   Vdiff (mV) = 20*u32Current*20m Ohm
  **/ 
uint32_t EADC_GetConsumptionCurr(int port)
{
    uint32_t u32OpaOutmV, u32Current;
    uint32_t u32OpaInmV;
    //printf("\n\nADC = %d\n", gu32ADCV);
    u32OpaOutmV = gu32RefVolmV * gu32ADCV / 4096;
    //printf("OPA O = %d mv\n", u32OpaOutmV);
#if 0
    u32OpaInmV = u32OpaOutmV / 20;
    printf("OPA I = %d mv\n", u32OpaInmV);
    u32Current = u32OpaInmV / 20 * 1000;
#else
    u32Current = u32OpaOutmV * 1000 / 20 / 20;
#endif
    //printf("u32Curr = %d\n", u32Current);
    return u32Current;
}
#endif

void EADC_ConfigPins(void)
{
    /* Set PB.15 - PB.3 to input mode For EADC pin to measure VBUS and VCONN */
    GPIO_SetMode(PB, BIT15, GPIO_MODE_INPUT);    //For VBUS Voltage

    /* Configure the PB.15 ADC analog input pins for VBUS. */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(SYS_GPB_MFP3_PB15MFP_Msk)) |
                    (SYS_GPB_MFP3_PB15MFP_EADC0_CH15);

    GPIO_DISABLE_DIGITAL_PATH(PB, BIT15);
}
void EADC_Init(void)
{
    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Configure the sample module 0 for analog input channel 15 (PB15) and timer0 trigger source.*/
    EADC_ConfigSampleModule(EADC, 0, EADC_TIMER1_TRIGGER, 15);
    EADC_SetExtendSampleTime(EADC, 0, 0x80);

    /* Enable the sample module 0 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT0);                    //Enable sample module 0 ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0);   //Enable sample module 0 interrupt.
    NVIC_EnableIRQ(EADC0_INT0_IRQn);

    /* PA12 acts as VBUS Detection Input pin */
    GPIO_DISABLE_DIGITAL_PATH(PA, BIT12);

    /* Use EADC channel 15 and channel 3 to measure VBUS and VCONN */
    /* However, the system didn't use VCONN */
    UTCPD->MUXSEL = (UTCPD->MUXSEL & ~(UTCPD_MUXSEL_ADCSELVB_Msk | UTCPD_MUXSEL_ADCSELVC_Msk)) | ((15 << UTCPD_MUXSEL_ADCSELVB_Pos) | (3 << UTCPD_MUXSEL_ADCSELVC_Pos));
}