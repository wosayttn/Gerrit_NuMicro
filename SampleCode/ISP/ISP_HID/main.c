/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to update chip flash data through USB HID interface
 *           between chip USB device and PC.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip USB device and assign update file
 *           of Flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "targetdev.h"
#include "isp_user.h"
#include "hid_transfer.h"

int32_t SYS_Init(void)
{
    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Select USB clock source as HIRC48M and USB clock divider as 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC, CLK_CLKDIV0_USB(1));

    /* Enable module clock */
    CLK_EnableModuleClock(ISP_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk;
    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set DETECT_PIN to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk);
    SET_GPIO_PB0();

    return 0;
}

int32_t main(void)
{
    uint32_t u32TrimInit;

    /* Unlock write-protected registers */
    SYS_UnlockReg();
    /* Enable ISP */
    FMC_Open();

    /* Init System, peripheral clock and multi-function I/O */
    /* Check if DETECT_PIN is low to enter ISP flow */
    if ((SYS_Init() < 0) || (DETECT_PIN != 0))
        goto _APROM;

    /* Get APROM and Data Flash size */
    FMC_ENABLE_AP_UPDATE();
    FMC_ENABLE_DF_UPDATE();

    /* Get APROM and Data Flash size */
    g_u32ApromSize = GetApromSize();

    while (1)
    {
        /* Open USB controller */
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);
        /*Init Endpoint configuration for HID */
        HID_Init();
        /* Start USB device */
        USBD_Start();

        /* Backup default trim */
        u32TrimInit = M32(TRIM_INIT);
        /* Clear SOF */
        USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

        /* Enable USBD interrupt */
        NVIC_EnableIRQ(USBD_IRQn);

        while (1)
        {
            /* Start USB trim if it is not enabled. */
            if ((SYS->HIRCTCTL & SYS_HIRCTCTL_FREQSEL_Msk) != 1)
            {
                /* Start USB trim only when SOF */
                if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
                {
                    /* Clear SOF */
                    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                    /*
                        USB clock trim function:
                        HIRC Trimming with boundary function enhances robustility
                        and keeps HIRC in right frequency while receiving unstable USB signal
                    */
                    SYS->HIRCTCTL = (0x1 << SYS_HIRCTCTL_FREQSEL_Pos) | SYS_HIRCTCTL_REFCKSEL_Msk |
                                    SYS_HIRCTCTL_BOUNDEN_Msk | (8 << SYS_HIRCTCTL_BOUNDARY_Pos);
                }
            }

            /* Disable USB Trim when any error found */
            if (SYS->HIRCTISTS & (SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk))
            {
                /* Init TRIM */
                M32(TRIM_INIT) = u32TrimInit;

                /* Disable crystal-less */
                SYS->HIRCTCTL = 0;

                /* Clear trim error flags */
                SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk;

                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
            }

            if (g_u8UsbDataReady == TRUE)
            {
                ParseCmd((uint8_t *)g_u8UsbRcvBuff, 64);
                EP2_Handler();
                g_u8UsbDataReady = FALSE;
            }
        }
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
