/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    This sample uses connect/disconnect callback to become aware of
 *           device connect and disconnect events. It also shows device information
 *           represented in UDEV_T.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "usb.h"
#include "hub.h"
#include "usbh_lib.h"

static volatile uint32_t s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void connect_func(struct udev_t *udev, int i8Param);
void disconnect_func(struct udev_t *udev, int i8Param);
void SYS_Init(void);
void UART0_Init(void);

void SysTick_Handler(void)
{
    s_u32TickCnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    s_u32TickCnt = 0;
    if(SysTick_Config(SystemCoreClock / (uint32_t)ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while(1);
    }
}

uint32_t get_ticks(void)
{
    return s_u32TickCnt;
}

/*
 *  USB device connect callback function.
 *  User invokes usbh_pooling_hubs() to let USB core able to scan and handle events of
 *  USBH port and USB hub device ports. Once a new device connected, it will be detected
 *  and enumerated in the call to usbh_pooling_hubs(). This callback will be invoked from
 *  USB core once a newly connected device was successfully enumerated.
 */
void connect_func(struct udev_t *udev, int i8Param)
{
    struct hub_dev_t *parent;
    int i8Cnt;

    (void)i8Param;

    parent = udev->parent;

    printf("Device [0x%x,0x%x] was connected.\n",
           udev->descriptor.idVendor, udev->descriptor.idProduct);
    printf("    Speed:    %s-speed\n", (udev->speed == SPEED_HIGH) ? "high" : ((udev->speed == SPEED_FULL) ? "full" : "low"));
    printf("    Location: ");

    if(parent == NULL)
    {
        if(udev->port_num == 1)
            printf("USB 2.0 port\n");
        else
            printf("USB 1.1 port\n");
    }
    else
    {
        if(parent->pos_id[0] == '1')
            printf("USB 2.0 port");
        else
            printf("USB 1.1 port");

        for(i8Cnt = 1; parent->pos_id[i8Cnt] != 0; i8Cnt++)
        {
            printf(" => Hub port %c", parent->pos_id[i8Cnt]);
        }

        printf(" => Hub port %d\n", udev->port_num);

        printf("\n");
    }
    printf("\n");
}

/*
 *  USB device disconnect callback function.
 *  User invokes usbh_pooling_hubs() to let USB core able to scan and handle events of
 *  USBH port and USB hub device ports. Once a device was disconnected, it will be detected
 *  and removed in the call to usbh_pooling_hubs(). This callback will be invoked from
 *  USB core prior to remove that device.
 */
void disconnect_func(struct udev_t *udev, int i8Param)
{
    (void)i8Param;

    printf("Device [0x%x,0x%x] was disconnected.\n",
           udev->descriptor.idVendor, udev->descriptor.idProduct);
}

/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HIRC;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    TIMER0->CTL = 0;        /* disable timer */
    TIMER0->INTSTS = (TIMER_INTSTS_TIF_Msk | TIMER_INTSTS_TWKF_Msk);   /* write 1 to clear for safety */
    TIMER0->CMP = (uint32_t)usec;
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;

    while(!TIMER0->INTSTS);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC, FREQ_144MHZ);

    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(3));

    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable USBH module clock */
    CLK_EnableModuleClock(USBH_MODULE);

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins */
    Uart0DefaultMPF();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{
    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();

    enable_sys_tick(100);

    printf("\n\n");
    printf("+--------------------------------------------+\n");
    printf("|                                            |\n");
    printf("| USB Host Connect/Disconnect callback demo  |\n");
    printf("|                                            |\n");
    printf("+--------------------------------------------+\n");

    /* Set OTG as USB Host role */
    SYS->USBPHY = SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk | (0x1 << SYS_USBPHY_USBROLE_Pos);

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15 */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~SYS_GPB_MFP3_PB15MFP_Msk) | SYS_GPB_MFP3_PB15MFP_USB_VBUS_EN;

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PC.14 */
    SYS->GPC_MFP3 = (SYS->GPC_MFP3 & ~SYS_GPC_MFP3_PC14MFP_Msk) | SYS_GPC_MFP3_PC14MFP_USB_VBUS_ST;

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SYS->GPA_MFP3 &= ~(SYS_GPA_MFP3_PA12MFP_Msk | SYS_GPA_MFP3_PA13MFP_Msk |
                       SYS_GPA_MFP3_PA14MFP_Msk | SYS_GPA_MFP3_PA15MFP_Msk);
    SYS->GPA_MFP3 |= SYS_GPA_MFP3_PA12MFP_USB_VBUS | SYS_GPA_MFP3_PA13MFP_USB_D_N |
                     SYS_GPA_MFP3_PA14MFP_USB_D_P | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID;
    usbh_core_init();

    usbh_install_conn_callback(connect_func, disconnect_func);

    while(1)
    {
        if(usbh_pooling_hubs())              /* USB Host port detect polling and management */
        {
            // usbh_memory_used();           /* print out USB memory allocating information */
        }
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/