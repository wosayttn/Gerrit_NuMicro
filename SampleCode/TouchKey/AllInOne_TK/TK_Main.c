/**************************************************************************//**
 * @file     TK_Main.c
 * @version  V1.00
 * @brief    Demonstrate how to calibration Touch Key through NuSenAdj Tool.
 *           After calibration stage, how free-run the Touch Key bases on the calibration data.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "tklib.h"
#include "TK_Demo.h"



volatile uint8_t u8EventKeyScan = 0;
volatile int8_t i8SliderPercentage = 0;
volatile int8_t i8WheelPercentage = 0;

#ifdef MASS_FINETUNE
void TK_MassProduction(int8_t* pai8Signal);
#endif

void TickCallback_KeyScan(void)
{
    u8EventKeyScan = 1;
}

#if defined(_OPT_DBG_BASELINE)

extern S_KEYINFO S_KeyInfo[];
void TickCallback_BaseLine(void)
{
    static uint8_t u8EventDbgShow=0;

    _TK_ShowKeyInfo(0, S_KeyInfo);

}
#endif //defined(_OPT_DBG_BASELINE)



int8_t SliderPercentage(int8_t* pu8SliderBuf, uint8_t u8Count)
{
    int8_t i; //i8M, i8N,
    float i16M=0.0, i16N=0.0;
    for(i=0; i<u8Count; i=i+1)
    {
        if(pu8SliderBuf[i] > 1)
        {
            i16M += (i+1)*pu8SliderBuf[i];
            i16N += pu8SliderBuf[i];
        }
    }
    return (int8_t)((((i16M*10)/i16N)-10)*10)/((u8Count-1));
}

/**
  *  Report touching or un-touching state depends on debounce parameter you set on calibration statge
  *  For example,
  *      TK_ScanKey() may report someone key pressed but its signal is less than threshold of the key.
  *      The root cause is the key still under debouce stage.
  */
void TK_RawDataView(void)
{

    int8_t i8Ret = 0;
    int8_t ai8Signal[TKLIB_TOL_NUM_KEY]; //u8MaxScKeyNum];
    int8_t ai8TmpSignal[TKLIB_TOL_NUM_KEY]; //u8MaxScKeyNum];
    if (u8EventKeyScan == 1)
    {
        u8EventKeyScan = 0;
        /**
          * TK_ScanKey() scan all enable key, slider and wheel channels.
          * i8Ret : Key/slider/wheel channel with max amplitude. -1: means no any key's amplitude over the key's threshold.
          * ai8Signal[]: The buffer size is equal to the M258 TK channels. It reports the signal amplitude on this round
          */
        i8Ret = TK_ScanKey(&ai8Signal[0]);
        if (i8Ret != -1)
        {
            uint32_t u32ChnMsk, i;

            u32ChnMsk = TK_GetEnabledChannelMask(TK_KEY);
            u32ChnMsk |= TK_GetEnabledChannelMask(TK_SLIDER);
            u32ChnMsk |= TK_GetEnabledChannelMask(TK_WHEEL);

            for (i = 0 ; i < u8MaxScKeyNum ; i++)
            {
                if (u32ChnMsk & (1ul << i))
                {
                    if( ai8Signal[i] >=  TK_GetChannelThreshold(i) )
                    {
                        //Turn on.
                        if(TK_DebounceChannel(i) == E_SIGNAL_OVER_DEBOUNCED)
                            TK_lightLED(TRUE, i);
                    }
                    else
                    {
                        //Turn off.
                        if(TK_DebounceChannel(i) == E_NOISE_OVER_DEBOUNCED)
                            TK_lightLED(FALSE, i);
                    }
                }
            }
        }
        else
        {
            TK_LEDAllOff();
        }

#ifdef MASS_FINETUNE
        TK_MassProduction(ai8Signal);
#endif

#if defined(OPT_SLIDER)

        {
            /** To save buffer size, re-used the ai8Signal[] buffer
              * Remember that the buffer will be destroied
              */
            uint16_t u16ChnMsk; /* ML56 is only 15 TK channels */
            uint8_t u8Count = 0, i;
            static uint8_t updatecount = 0;

            updatecount = updatecount+1;
            if(updatecount < 5)
                return;
            updatecount = 0;

            u16ChnMsk = TK_GetEnabledChannelMask(TK_SLIDER);

            if(TK_CheckSliderWheelPressed(TK_SLIDER) == 1)
            {
                for (i = 0; i < /*TKLIB_TOL_NUM_KEY*/ u8MaxScKeyNum ; i++)
                {
                    if (u16ChnMsk & (1ul << i))
                    {
                        ai8TmpSignal[u8Count] = ai8Signal[i];
                        u8Count = u8Count+1;
                    }
                }
                i8SliderPercentage = TK_SliderPercentage(ai8TmpSignal, u8Count);
            }
        }
#endif
#if defined(OPT_WHEEL)

        {
            /** To save buffer size, re-used the ai8Signal[] buffer
              * Remember that the buffer will be destroied
              */
            uint32_t u32ChnMsk; /* ML56 is only 15 TK channels */
            uint8_t u8Count = 0, i;

            u32ChnMsk = TK_GetEnabledChannelMask(TK_WHEEL);

            if(TK_CheckSliderWheelPressed(TK_WHEEL)  == 1)
            {
                for (i = 0; i < /*TKLIB_TOL_NUM_KEY*/ u8MaxScKeyNum ; i++)
                {
                    if (u32ChnMsk & (1ul << i))
                    {
                        ai8TmpSignal[u8Count] = ai8Signal[i];
                        u8Count = u8Count+1;
                    }
                }
                i8WheelPercentage = TK_WheelPercentage(ai8TmpSignal, u8Count);
                DBG_PRINTF("Wheel %d\n", i8WheelPercentage);
            }
        }
#endif



    }

}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK->AHBCLK0 |= 0xFF000000;     /* Enable GPIOA ~ GPIOH clock*/

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART3_MODULE);

    /* Enable TIMER0 peripheral clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable TIMER1 peripheral clock */
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Enable TIMER2 peripheral clock */
    CLK_EnableModuleClock(TMR2_MODULE);

    /* Enable TK peripheral clock */
#if 1
    CLK_EnableModuleClock(TK_MODULE);
#else
    LPSCC->CLKEN0 |= LPSCC_CLKEN0_TKCKEN_Msk;
    LPSCC->IPRST0 |= LPSCC_IPRST0_TKRST_Msk;
    LPSCC->IPRST0 &= ~LPSCC_IPRST0_TKRST_Msk;
    LPSCC->CLKSEL0 &= ~LPSCC_CLKSEL0_TKSEL_Msk;     /* Clock from HIRC */
#endif

    /* Enable PA peripheral clock */
    CLK_EnableModuleClock(GPA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
#if defined(COOL_CHAIN_TIM)
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF2MFP_Msk) | SYS_GPF_MFPL_PF2MFP_UART0_RXD;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF3MFP_Msk) | SYS_GPF_MFPL_PF3MFP_UART0_TXD;
#else
#if 1
    /* Set PB multi-function pins for UART0 RXD and TXD */
    //Uart0DefaultMPF();
    SYS->GPC_MFP3 = (SYS->GPC_MFP3 & ~SYS_GPC_MFP3_PC12MFP_Msk) | SYS_GPC_MFP3_PC12MFP_UART0_TXD;
    SYS->GPC_MFP2 = (SYS->GPC_MFP2 & ~SYS_GPC_MFP2_PC11MFP_Msk) | SYS_GPC_MFP2_PC11MFP_UART0_RXD;
#else
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(SYS_GPB_MFP3_PB12MFP_Msk | SYS_GPB_MFP3_PB13MFP_Msk)) |
                    (SYS_GPB_MFP3_PB12MFP_UART0_RXD | SYS_GPB_MFP3_PB13MFP_UART0_TXD);
#endif
#endif /* COOL_CHAIN_TIM */

    /* Set GPB multi-function pins for UART1 RXD and TXD */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB6MFP_Msk) | SYS_GPB_MFPL_PB6MFP_UART1_RXD;
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB7MFP_Msk) | SYS_GPB_MFPL_PB7MFP_UART1_TXD;

#if 0
    /* Set GPE multi-function pins for UART3 RXD and TXD */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~SYS_GPE_MFPL_PE0MFP_Msk) | SYS_GPE_MFPL_PE0MFP_UART3_RXD;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~SYS_GPE_MFPL_PE1MFP_Msk) | SYS_GPE_MFPL_PE1MFP_UART3_TXD;
#else
    /* Set GPE multi-function pins for UART3 RXD and TXD */
    SYS->GPE_MFP0 = (SYS->GPE_MFP0 & ~(SYS_GPE_MFP0_PE0MFP_Msk | SYS_GPE_MFP0_PE1MFP_Msk)) |
                    (SYS_GPE_MFP0_PE0MFP_UART3_RXD | SYS_GPE_MFP0_PE1MFP_UART3_TXD);
#endif


}

int32_t main(void)
{
    uint32_t u32ChanelMsk;
    int8_t i8Ret = 0;

    SYS_Init();
    RMC_Open();
    RMC_ENABLE_AP_UPDATE();

    //RMC_Erase(0x1C000);

#ifdef  DEMO_CALIBRATION
    UART0_Init();
#endif

#ifdef USE_DEBUG_PORT
    UART3_Init();
    printf("UART Init\n");
#endif

    CLK->AHBCLK0 |= 0xFF000000;     /* Enable GPIOA ~ GPIOH */

#ifndef BOARD_TIM
    InitLEDIO();
#endif

    i8Ret = TK_LoadPara(&u32ChanelMsk);

#ifdef DEMO_CALIBRATION
    /* Initialize FMC to Load TK setting and calibration data from flash */
    RMC_Open();
    RMC_ENABLE_AP_UPDATE();

    if(i8Ret == -1)
    {
        /** i8Ret = -1 means that no any calibration data stored in flash
          * If no any data stored in flash. Get TK setting and calibration data from UART port
          * Program will be blocked in the function until received START_CALIBRATION command. The return vlue will be 1
          */
        i8Ret = TK_GetPacket(&u32ChanelMsk);
    }

    /* Init TK Controller */
    TK_Init();
		
    /* CAPACITOR_BANK_SEL to MODE0 8 bits */
    //TK_EXTEND_CAPACITOR_BANK_SEL(TK_CAPACITOR_BANK_SEL_MODE0); 
		
    /* CAPACITOR_BANK_SEL to MODE1 8 bits --> 9 bit (Analog) */
    TK_EXTEND_CAPACITOR_BANK_SEL(TK_CAPACITOR_BANK_SEL_MODE1); 
		
    /* Initialize Multiple Function Pins for TK */
    SetTkMultiFun(u32ChanelMsk);

    /* Init systick 20ms/tick */
    Init_SysTick();

    /* Install Tick Event Handler To Drive Key Scan */
    TickSetTickEvent(1, (void *)TickCallback_KeyScan);

#if defined(_OPT_USE_SEVEN_SEGMENT)
    TickSetTickEvent(1, TickCallback_Disp7Segment);
#endif //defined(_OPT_USE_SEVEN_SEGMENT)
//              for(;;){ TK_RawDataView();}
    do
    {
        if(i8Ret == 1)
        {
            /** Receive Start calibration command
              * The function will be blocked until calibration done
              */
            TK_Calibration_Untouch();
            /* Inform UART module calibration done */
            UART_SetCalibrationDone();
        }

        i8Ret = TK_GetPacket(&u32ChanelMsk);

        /** May change configurations through UART port
          * Init TK Controller again
          */
        TK_Init();

        /* CAPACITOR_BANK_SEL to MODE0 8 bits */
        //TK_EXTEND_CAPACITOR_BANK_SEL(TK_CAPACITOR_BANK_SEL_MODE0); 
		
        /* CAPACITOR_BANK_SEL to MODE1 8 bits --> 9 bit (Analog) */
        TK_EXTEND_CAPACITOR_BANK_SEL(TK_CAPACITOR_BANK_SEL_MODE1); 

        /* Initialize Multiple Function Pins for TK again */
        SetTkMultiFun(u32ChanelMsk);

//                              _TK_ShowCONFIG();
    }
    while(1);
#endif /* DEMO_CALIBRATION */

#ifdef DEMO_FREERUN

    if(i8Ret < 0)
    {
        /* DBG_PRINTF("Please run target TK_Application first to calibrate touchkey\n"); */
        while(1);
    }

    /* Init TK Controller */
    TK_Init();

    /* CAPACITOR_BANK_SEL to MODE0 8 bits */
    //TK_EXTEND_CAPACITOR_BANK_SEL(TK_CAPACITOR_BANK_SEL_MODE0); 
		
    /* CAPACITOR_BANK_SEL to MODE1 8 bits --> 9 bit (Analog) */
    TK_EXTEND_CAPACITOR_BANK_SEL(TK_CAPACITOR_BANK_SEL_MODE1); 

    /* Initialize Multiple Function Pins for TK */
    SetTkMultiFun(u32ChanelMsk);

    /* Init systick 20ms/tick */
    Init_SysTick();

    /* Install Tick Event Handler To Drive Key Scan */
    TickSetTickEvent(1, (void *)TickCallback_KeyScan);

    do
    {

#if defined(WAKEUP_BY_CHANNEL) || defined(WAKEUP_BY_ALL)
        _TK_WakeUpTest();
#else
        TK_RawDataView();
#endif

    }
    while (1);
#endif  /* DEMO_FREERUN */
}


