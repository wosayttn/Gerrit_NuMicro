/******************************************************************************
 * @file    main.c
 * @version V1.00
 * @brief    Show the usage of GPIO EINT trigger PWM function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

void EINT1_IRQHandler(void)
{
    /* To check if external interrupt occurred */
    if (GPIO_GET_EINT_FLAG(1))
    {
        GPIO_CLR_EINT_FLAG(1);
        printf("EINT1 occurred.\n");
    }
}

void PWM0P1_IRQHandler(void)
{
    /* Check PWM0 channel 2 compare up count interrupt flag */
    if (PWM0->INTSTS0 & PWM_INTSTS0_CMPUIF2_Msk)
    {
        /* Disable PWM0 channel 2 counting */
        PWM0->CNTEN &= ~PWM_CNTEN_CNTEN2_Msk;
        /* Clear PWM0 channel 2 compare up count interrupt flag */
        PWM0->INTSTS0 = PWM_INTSTS0_CMPUIF2_Msk;
        printf("PWM0 channel 2 compare up count interrupt occurred.\n");
    }
}


int32_t SYS_Init(void)
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
    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);
    /* Select PWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);
    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPB_MODULE);
    /* Enable GPIO Port C clock */
    CLK_EnableModuleClock(GPC_MODULE);
    /* Enable Internal low speed RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    /* Waiting for Internal low speed RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);
    /* Set multi-function pin for EINT1(PA.7) */
    SET_INT1_PA7();
    /* Lock protected registers */
    SYS_LockReg();
    return 0;
}

int32_t main(void)
{
    uint32_t u32ChannelNum;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    GPIO EINT Trigger PWM Sample Code     |\n");
    printf("+------------------------------------------+\n\n");
    printf("Use EINT1(PA.7) falling edge to start PWM0 channel 2 counting.\n");
    u32ChannelNum = 2;
    /* Set PWM0 channel 2 comparator register */
    PWM_SET_CMR(PWM0, u32ChannelNum, 0xFFFF);
    /* Set PWM0 channel 2 period register */
    PWM_SET_CNR(PWM0, u32ChannelNum, 0xFFFF);
    /* Enable PWM0 channel 2 compare up count interrupt */
    PWM0->INTEN0 |= PWM_INTEN0_CMPUIEN2_Msk;
    NVIC_EnableIRQ(PWM0P1_IRQn);
    /* Start PWM0 channel 2 counting by external trigger EINT1 */
    PWM0->EXTETCTL[(u32ChannelNum >> 1) << 1] = (0x1 << PWM_EXTETCTL2_EXTTRGS_Pos) | (0x1 << PWM_EXTETCTL0_CNTACTS_Pos) | PWM_EXTETCTL2_EXTETEN_Msk;
    PWM_EnableExtEventTrigger(PWM0, u32ChannelNum, PWM_EXT_TGR_PIN_INT1, PWM_EXT_TGR_COUNTER_START);
    /* Configure PA.7 as EINT1 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PA, BIT7, GPIO_PUSEL_PULL_UP);
    GPIO_EnableEINT(1, GPIO_INT_EDETCTL_FALLING);
    NVIC_EnableIRQ(EINT1_IRQn);
    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT7);

    while (1) {};
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
