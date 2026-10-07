/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Use embedded data flash as storage to implement a USB Mass-Storage device.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "massstorage.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)

void SYS_Init(void);
void PowerDown(void);

#ifdef VBUS_DIVIDER

#include "utcpd.c"
#include "i2c_controller.c"

int port = 0;
int alert;
uint32_t u32VBUSDetEn, u32VBUSPresent = 0, u32VBUSPrevious = 0, u32VCONNPresent, u32SnkVBUS;

void GPIO_Init(void)
{
    /* Enable PA13~14 (D+ / D-) interrupt for wakeup */
    GPIO_CLR_INT_FLAG(PA, BIT13 | BIT14);
    GPIO_EnableInt(PA, 13, GPIO_INT_BOTH_EDGE);
    GPIO_EnableInt(PA, 14, GPIO_INT_BOTH_EDGE);
    GPIO_ENABLE_DEBOUNCE(PA, BIT13 | BIT14);   /* Enable key debounce */
}
#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

#if (!CRYSTAL_LESS)
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, FREQ_144MHZ);

    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(3));

    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(3));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
#else
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
#endif

#ifdef VBUS_DIVIDER
    /* Enable UTCPD module clock */
    CLK_EnableModuleClock(UTCPD0_MODULE);

    /* Enable GPA module clock */
    CLK_EnableModuleClock(GPA_MODULE);
#endif
    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

#ifdef VBUS_DIVIDER
    SYS->GPA_MFP3 &= ~(SYS_GPA_MFP3_PA13MFP_Msk | SYS_GPA_MFP3_PA14MFP_Msk | SYS_GPA_MFP3_PA15MFP_Msk);

    /* USBD multi-function pins for D+, D-, and ID pins */
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID);
#endif

    /* Set multi-function pins */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}

#ifdef VBUS_DIVIDER
void UTCPD_IRQHandler(void)
{
    UTCPD_GetAlertStatus(0, &alert);

    if(alert & UTCPD_ALERT_PWRSCHIS)
    {
        UTCPD_GetPwrSts(port, &u32VBUSDetEn, &u32VBUSPresent, &u32VCONNPresent, &u32SnkVBUS);

        if(u32VBUSPresent != u32VBUSPrevious)
        {
            if(u32VBUSPresent)
            {
                /* USB Plug In */
                USBD_ENABLE_USB();
                printf("Plug\n");
            }
            else
            {
                /* USB Un-plug */
                USBD_DISABLE_USB();
                printf("Un-Plug\n");
            }
            u32VBUSPrevious = u32VBUSPresent;
        }
    }
    UTCPD_ClearAlertStatus(0, alert);
}
#endif

void PowerDown(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    UART_WAIT_TX_EMPTY(DEBUG_PORT);

#ifdef VBUS_DIVIDER
    /* Change USBD multi-function pins (D+, D-) to GPIO */
    SYS->GPA_MFP3 &= ~(SYS_GPA_MFP3_PA12MFP_Msk | SYS_GPA_MFP3_PA13MFP_Msk | SYS_GPA_MFP3_PA14MFP_Msk);

    GPIO_CLR_INT_FLAG(PA, BIT13 | BIT14);
#endif

    CLK_PowerDown();

#ifdef VBUS_DIVIDER
    GPIO_CLR_INT_FLAG(PA, BIT13 | BIT14);

    /* Change PA13 & PA14 to USBD multi-function pins (D+, D-) */
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P);
#else
    /* Change PA13 & PA14 to USBD multi-function pins (D+, D-) */
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA12MFP_USB_VBUS | SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID);
#endif

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART0_Init();

    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB MassStorage Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    printf("NuMicro USB MassStorage Start!\n");

#ifdef VBUS_DIVIDER

    GPIO_DISABLE_DIGITAL_PATH(PA, BIT12);

    UTCPD->MUXSEL = (UTCPD->MUXSEL & ~(UTCPD_MUXSEL_ADCSELVB_Msk | UTCPD_MUXSEL_ADCSELVC_Msk)) | ((2 << UTCPD_MUXSEL_ADCSELVB_Pos) | (3 << UTCPD_MUXSEL_ADCSELVC_Pos));

    UTCPD->VBVOL = (UTCPD->VBVOL & ~UTCPD_VBVOL_VBSCALE_Msk) | (2 << UTCPD_VBVOL_VBSCALE_Pos);

    UTCPD_Open(port);

    UTCPD_SetRoleCtrl(port, (uint32_t)NULL, UTCPD_ROLECTL_RPVALUE_1P5A, UTCPD_ROLECTL_CC2_RD, UTCPD_ROLECTL_CC1_RD);

    UTCPD_DisablePowerCtrl (port, UTCPD_PWRCTL_VBMONI_DIS);

    UTCPD_IsssueCmd(port, UTCPD_CMD_ENABLE_VBUS_DETECT);

    UTCPD_EnableAlertMask(0, UTCPD_ALERTM_PWRSCHIE);

    UTCPD_EnablePowerStatusMask(0, UTCPD_PWRSM_VBPSIE);

    NVIC_EnableIRQ(UTCPD_IRQn);

    GPIO_Init();
#else
    SYS->GPA_MFP3 &= ~(SYS_GPA_MFP3_PA12MFP_Msk | SYS_GPA_MFP3_PA13MFP_Msk | SYS_GPA_MFP3_PA14MFP_Msk | SYS_GPA_MFP3_PA15MFP_Msk);

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA12MFP_USB_VBUS | SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID);
#endif

    USBD_Open(&gsInfo, MSC_ClassRequest, NULL);

    USBD_SetConfigCallback(MSC_SetConfig);

    /* Endpoint configuration */
    MSC_Init();

    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);

    /* Waiting for USB bus stable */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

    /* Enable USB crystal-less */
    SYS->HIRCTCTL |= (SYS_HIRCTCTL_REFCKSEL_Msk | 0x1);
#endif

    while(1)
    {
#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled. */
        if((SYS->HIRCTCTL & SYS_HIRCTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->HIRCTCTL = 0x01;
                SYS->HIRCTCTL |= SYS_HIRCTCTL_REFCKSEL_Msk | SYS_HIRCTCTL_BOUNDEN_Msk | (8 << SYS_HIRCTCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if(SYS->HIRCTISTS & (SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->HIRCTCTL = 0;

            /* Clear error flags */
            SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif

        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();

        MSC_ProcessCmd();
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/