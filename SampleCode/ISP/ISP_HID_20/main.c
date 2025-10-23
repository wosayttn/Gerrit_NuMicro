/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to update chip flash data through HUSB HID interface
 *           between chip HUSB device and PC.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip USB device and assign update file
 *           of Flash.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "targetdev.h"
#include "isp_user.h"
#include "hid_transfer.h"

// Empty function to reduce code size
uint32_t ProcessHardFault(uint32_t *pu32StackFrame)
{
    NVT_UNUSED(pu32StackFrame);
    return 0;
}

uint32_t CLK_GetPLLClockFreq(void)
{
    return PLL_CLOCK;
}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1; /* 500ms time-out */

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz and HXT clock */
    CLK->SRCCTL |= (CLK_SRCCTL_HIRCEN_Msk | CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for Internal RC clock ready */
    while ((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk)) != (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk))
    {
        if (--u32TimeOutCnt == 0)
        {
            return -1;
        }
    }

    /* Select SCLK to HIRC before APLL setting*/
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);
    /* Enable APLL0 220MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);
    /* Set clock with limitations */
    CLK_SET_HCLK2DIV(2);
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);
    /* Switch SCLK clock source to APLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select HSUSBD */
    SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;

    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSOTGPHYEN_Msk;
    CLK_SysTickDelay(20);   // delay > 10 us
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Enable module clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(ISP0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set DETECT_PIN to input mode */
    PI->MODE &= ~(GPIO_MODE_MODE11_Msk);
    SET_GPIO_PI11();

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();
    /* Enable ISP */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* Init System, peripheral clock and multi-function I/O */
    /* Check if DETECT_PIN is low to enter ISP flow */
    if ((SYS_Init() < 0) || (DETECT_PIN != 0))
        goto _APROM;

    /* Get APROM and Data Flash size */
    g_u32ApromSize = GetApromSize();

    /* Open HSUSBD controller */
    HSUSBD_Open(NULL, NULL, NULL);

    /* Endpoint configuration */
    HID_Init();

    /* M5531 has 8 KB LDROM, changed to use IRQ mode */
    /* Enable HSUSBD interrupt */
    NVIC_EnableIRQ(HSUSBD_IRQn);

    /* Start transaction */
    HSUSBD_Start();

    while (1)
    {
        if (g_u8UsbDataReady == TRUE)
        {
            ParseCmd((uint8_t *)g_u8UsbRcvBuff, 64);
            EPA_Handler();
            g_u8UsbDataReady = FALSE;
        }
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
