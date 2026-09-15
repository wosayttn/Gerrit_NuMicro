/******************************************************************************//**
 * @file     main.c
 * @version  V1.01
 * @brief    Use embedded data flash as storage to implement a USB Mass-Storage device.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "M2L31_user.h"
#include "massstorage.h"

#define TRIM_INIT           (SYS_BASE+0x10C)

void USBD_Start(void);
int32_t RMC_SetVectorAddr(uint32_t u32PageAddr);

/* Add implementations to fix linker warnings from the newlib-nano C library in VSCode-GCC14.3.1 */
void _close(void) {}
void _lseek(void) {}
void _read_r(void) {}
void _write_r(void) {}

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable HIRC48 clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48MEN_Msk);

    /* Waiting for HIRC48 clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC48M, CLK_CLKDIV0_USB(1));

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Enable GPB module clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SYS->GPA_MFP3 &= ~(SYS_GPA_MFP3_PA12MFP_Msk | SYS_GPA_MFP3_PA13MFP_Msk | SYS_GPA_MFP3_PA14MFP_Msk | SYS_GPA_MFP3_PA15MFP_Msk);
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA12MFP_USB_VBUS | SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TrimInit;

    /* The code should boot from LDROM: check the boot setting */
    SYS_UnlockReg();

    SYS_Init();

    RMC->ISPCTL = RMC_ISPCTL_ISPEN_Msk | RMC_ISPCTL_APUEN_Msk;

    USBD_Open(&gsInfo);

    /* Endpoint configuration */
    MSC_Init();

    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

    /* Backup default trim value */
    u32TrimInit = M32(TRIM_INIT);

    /* Clear SOF */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

    /* Check if GPB.15 is low */
    while (PB15 == 0)
    {
        /* Start USB trim function if it is not enabled. */
        if ((SYS->HIRCTCTL & SYS_HIRCTCTL_FREQSEL_Msk) != 0x1)
        {
            /* Start USB trim only when USB signal arrived */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /*
                    USB clock trim function:
                    HIRC Trimming with boundary function enhances robustility
                    and keeps HIRC in right frequency while receiving unstable USB signal
                */
                SYS->HIRCTCTL = (0x1 << SYS_HIRCTCTL_REFCKSEL_Pos)
                                | (0x1 << SYS_HIRCTCTL_FREQSEL_Pos)
                                | (0x0 << SYS_HIRCTCTL_LOOPSEL_Pos)
                                | (0x1 << SYS_HIRCTCTL_BOUNDEN_Pos)
                                | (10  << SYS_HIRCTCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when any error found */
        if (SYS->HIRCTISTS & (SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable USB clock trim function */
            SYS->HIRCTCTL = 0;

            /* Clear trim error flags */
            SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
        }

        MSC_ProcessCmd();
    }

    /* Boot from APROM */
    RMC_SetVectorAddr(RMC_APROM_BASE);
    RMC_SET_APROM_BOOT();
    NVIC_SystemReset();

    while (1);
}
