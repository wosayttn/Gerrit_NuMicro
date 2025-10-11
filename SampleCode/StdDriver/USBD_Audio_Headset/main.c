/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to implement a USB audio class device.
 *           Codec is used in this sample code to play the audio data from Host.
 *           It also supports to record data from codec to Host.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"

#define CRYSTAL_LESS        0
#define TRIM_INIT           (SYS_BASE+0x10C)

void SetDebugUartCLK(void)
{
#if !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING)

#if (CRYSTAL_LESS)

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(DEBUG_PORT_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

#else

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(DEBUG_PORT_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

#endif

    /* Enable UART clock */
    CLK_EnableModuleClock(DEBUG_PORT_MODULE);

    /* Reset UART module */
    SYS_ResetModule(DEBUG_PORT_RST);

#endif /* !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING) */
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

#if (!CRYSTAL_LESS)

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Select USB clock source as PLL and USB clock divider as 3 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(3));

#else

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Select USB clock source as HIRC and USB clock divider as 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC, CLK_CLKDIV0_USB(1));

#endif

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable all GPIO module clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk ;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();

//    /* Set multi-function pins for I2S0 */
//    SET_SPII2S0_BCLK_PI6();
//    SET_I2S0_MCLK_PI7();
//    SET_I2S0_DI_PI8();
//    SET_I2S0_DO_PI9();
//    SET_I2S0_LRCK_PI10();

//    /* Enable I2S0 clock pin (PI6) schmitt trigger */
//    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;

    /* Set I2C0 multi-function pins */
    SET_I2C0_SDA_PC11();
    SET_I2C0_SCL_PC12();

    /* Enable I2C3 clock pin (PG0) schmitt trigger */
    PG->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;

}

/* Init I2C interface */
void I2C_Init(void)
{
    /* Open I2C_PORT and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);

    /* Get I2C Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C_PORT));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    InitDebugUart();

#ifdef __HID__
    GPIO_Init();
#endif

    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB Audio CODEC Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C to access codec */
    I2C_Init();

    /* Open SPI0 as slave mode */
    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, AUDIO_RATE, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK */
    SPII2S_EnableMCLK(SPI0, 12000000);

    /* Lock protected registers */
    SYS_LockReg();

    /* Set JK-EN low to enable phone jack on NuMaker board. */
    SET_GPIO_PD1();
    GPIO_SetMode(PD, BIT1, GPIO_MODE_OUTPUT);
    PD1 = 0;


#if NAU8822
    NAU8822_Setup();
#else
    NAU88L25_Reset();
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(10000);
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(20000);
    NAU88L25_Setup();
#endif

    /* Configure PDMA */
    PDMA_Init();

#ifndef __FEEDBACK__
    /* Configure TIMER0 for adjusting codec's PLL */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 500);
    TIMER_EnableInt(TIMER0);
    NVIC_SetPriority(TIMER0_IRQn, 3);
    NVIC_EnableIRQ(TIMER0_IRQn);
#endif

    USBD_Open(&gsInfo, UAC_ClassRequest, UAC_SetInterface);

    /* Endpoint configuration */
    UAC_Init();
    NVIC_SetPriority(USBD_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while (1)
    {
#ifdef __HID__
        HID_UpdateHidData();
#endif

#if CRYSTAL_LESS

        /* Start USB trim if it is not enabled. */
        if ((SYS->HIRCTCTL & SYS_HIRCTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /* Re-enable crystal-less */
                SYS->HIRCTCTL = 0x01;
                SYS->HIRCTCTL |= SYS_HIRCTCTL_REFCKSEL_Msk | SYS_HIRCTCTL_BOUNDEN_Msk | (8 << SYS_HIRCTCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if (SYS->HIRCTISTS & (SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->HIRCTCTL = 0;

            /* Clear error flags */
            SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
        }

#endif
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

