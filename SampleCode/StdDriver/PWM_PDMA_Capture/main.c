/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Capture the PWM1 Channel 0 waveform by PWM1 Channel 2, and use PDMA to transfer captured data.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define PWM_CAP_CH    2UL
#define PWM_OUT_CH    0UL
#define PDMA_CH       0UL

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
__attribute__((aligned(4))) static uint16_t g_au16Count[4];
static volatile uint32_t g_u32IsTestOver = 0;

void PDMA0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA0) & (PDMA_ABTSTS_ABTIF0_Msk << PDMA_CH))
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA0, (PDMA_ABTSTS_ABTIF0_Msk << PDMA_CH));
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if (PDMA_GET_TD_STS(PDMA0) & (PDMA_TDSTS_TDIF0_Msk << PDMA_CH))
            g_u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA0, (PDMA_TDSTS_TDIF0_Msk << PDMA_CH));
    }
    else
        printf("unknown interrupt %x !!\n", u32Status);

    while (PDMA_GET_INT_STATUS(PDMA0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA IntFlag time-out!\n");
            break;
        }
    }
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* g_au16Count[4] : Keep the internal counter value when input signal rising / falling  */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
int32_t CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    (void)PWM;
    (void)u32Ch;

    g_u32IsTestOver = 0;
    /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA interrupt time-out!\n");
            return (-1);
        }
    }

    u16RisingTime = g_au16Count[1];

    u16FallingTime = g_au16Count[2];

    u16HighPeriod = g_au16Count[1] - g_au16Count[2];

    u16LowPeriod = (uint16_t)(0x10000 - g_au16Count[3]);

    u16TotalPeriod = (uint16_t)(0x10000 - g_au16Count[2]);

    printf("\nPWM generate: \nHigh Period=17279 ~ 17281, Low Period=40319 ~ 40321, Total Period=57599 ~ 57601\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);

    if ((u16HighPeriod < 17279) || (u16HighPeriod > 17281) || (u16LowPeriod < 40319) || (u16LowPeriod > 40321) || (u16TotalPeriod < 57599) || (u16TotalPeriod > 57601))
    {
        printf("Capture Test Fail!!\n");
        return (-1);
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
    }
}

static void SYS_Init(void)
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

    /* Enable UART module clock */
    SetDebugUartCLK();
    /* Select PWM1 module clock source */
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

    /* Enable PWM1 module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);
    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPC_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for PWM */
    SET_PWM1_CH0_PC5();
    SET_PWM1_CH2_PC3();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                    PWM PDMA Capture Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM1 channel 2 to capture the signal from PWM1 channel 0.\n");
    printf("  And the captured data is transferred by PDMA channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    PWM1 channel 2(PC.3) <--> PWM1 channel 0(PC.5)\n\n");
    printf("Use PWM1 Channel 2(PC.3) to capture the PWM1 Channel 0(PC.5) Waveform\n");

    while (1)
    {
        printf("\n\nPress any key to start PWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM1 Channel 0 as PWM output function.                                     */
        /*--------------------------------------------------------------------------------------*/

        /* Assume PWM output frequency is 250Hz and duty ratio is 30%, user can calculate PWM settings by follows.(up counter type)
           duty ratio = (CMR)/(CNR+1)
           cycle time = CNR+1
           High level = CMR
           PWM clock source frequency = PLL/2 = 72000000
           (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
                   = 72000000/5/250 = 57600
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 57600
           duty ratio = 30% ==> (CMR)/(CNR+1) = 30%
           CMR = 17280
           Prescale value is 4 : prescaler= 5
        */

        /* Set PWM1 channel 0 output configuration */
        PWM_ConfigOutputChannel(PWM1, PWM_OUT_CH, 250, 30);

        /* Enable PWM Output path for PWM1 channel 0 */
        PWM_EnableOutput(PWM1, PWM_CH_0_MASK);

        /* Enable Timer for PWM1 channel 0 */
        PWM_Start(PWM1, PWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Configure PDMA peripheral mode form PWM to memory                                   */
        /*--------------------------------------------------------------------------------------*/
        /* Open Channel 0 */
        PDMA_Open(PDMA0, (BIT0 << PDMA_CH));

        /* Transfer width is half word(16 bit) and transfer count is 4 */
        PDMA_SetTransferCnt(PDMA0, PDMA_CH, PDMA_WIDTH_16, 4);

        /* Set source address as PWM capture channel PDMA register(no increment) and destination address as g_au16Count array(increment) */
        PDMA_SetTransferAddr(PDMA0, PDMA_CH, (uint32_t)&PWM1->PDMACAP[(PWM_CAP_CH >> 1UL)], PDMA_SAR_FIX, (uint32_t)&g_au16Count[0], PDMA_DAR_INC);

        /* Select PDMA request source as PWM RX(PWM1 channel 2 should be PWM1 pair 2) */
        PDMA_SetTransferMode(PDMA0, PDMA_CH, PDMA_PWM1_P2_RX, FALSE, 0);

        /* Set PDMA as single request type for PWM */
        PDMA_SetBurstType(PDMA0, PDMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_4);

        PDMA_EnableInt(PDMA0, PDMA_CH, PDMA_INT_TRANS_DONE);
        NVIC_EnableIRQ(PDMA0_IRQn);

        /* Enable PDMA for PWM1 channel 2 capture function, and set capture order as falling first, */
        /* And select capture mode as both rising and falling to do PDMA transfer. */
        PWM_EnablePDMA(PWM1, PWM_CAP_CH, FALSE, PWM_CAPTURE_PDMA_RISING_FALLING_LATCH);

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM1 channel 2 for capture function                                         */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PLL/2 = 72000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 72000000/5/250 = 57600
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

           Capture unit time = 1000000000/Capture clock source frequency*prescaler
           69 ns = 1000000000/72000000*5
        */

        /* Set PWM1 channel 2 capture configuration */
        PWM_ConfigCaptureChannel(PWM1, PWM_CAP_CH, 69, 0);
        PWM_SET_PRESCALER(PWM1, PWM_CAP_CH, PWM_GET_PRESCALER(PWM1, PWM_OUT_CH));

        /* Enable Timer for PWM1 channel 2 */
        PWM_Start(PWM1, PWM_CH_2_MASK);

        /* Enable Capture Function for PWM1 channel 2 */
        PWM_EnableCapture(PWM1, PWM_CH_2_MASK);

        /* Enable falling capture reload */
        PWM1->CAPCTL |= PWM_CAPCTL_FCRLDEN2_Msk;

        /* Wait until PWM1 channel 2 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while ((PWM1->CNT[PWM_CAP_CH]) == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM1 channel 2 Timer start time-out!\n");
                goto lexit;
            }
        }

        /* Capture the Input Waveform Data */
        if (CalPeriodTime(PWM1, PWM_CAP_CH) < 0)
            goto lexit;

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop PWM1 channel 0 (Recommended procedure method 1)                                                      */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set PWM1 channel 0 loaded value as 0 */
        PWM_Stop(PWM1, PWM_CH_0_MASK);

        /* Wait until PWM1 channel 0 Timer Stop */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while ((PWM1->CNT[PWM_OUT_CH] & PWM_CNT_CNT_Msk) != 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM1 channel 0 Timer stop time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for PWM1 channel 0 */
        PWM_ForceStop(PWM1, PWM_CH_0_MASK);

        /* Disable PWM Output path for PWM1 channel 0 */
        PWM_DisableOutput(PWM1, PWM_CH_0_MASK);

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop PWM1 channel 2 (Recommended procedure method 1)                                                      */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set loaded value as 0 for PWM1 channel 2 */
        PWM_Stop(PWM1, PWM_CH_2_MASK);

        /* Wait until PWM1 channel 2 current counter reach to 0 */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while ((PWM1->CNT[PWM_CAP_CH] & PWM_CNT_CNT_Msk) != 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM1 channel 2 current counter reach to 0 time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for PWM1 channel 2 */
        PWM_ForceStop(PWM1, PWM_CH_2_MASK);

        /* Disable Capture Function and Capture Input path for  PWM1 channel 2 */
        PWM_DisableCapture(PWM1, PWM_CH_2_MASK);

        /* Clear Capture Interrupt flag for PWM1 channel 2 */
        PWM_ClearCaptureIntFlag(PWM1, PWM_CAP_CH, PWM_CAPTURE_INT_FALLING_LATCH | PWM_CAPTURE_INT_RISING_LATCH);

        /* Disable PDMA NVIC */
        NVIC_DisableIRQ(PDMA0_IRQn);

        /* Close PDMA */
        PDMA_Close(PDMA0);
    }

lexit:

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
