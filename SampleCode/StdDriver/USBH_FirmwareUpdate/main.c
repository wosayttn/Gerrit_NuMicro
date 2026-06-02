/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Automatically search and read new firmware from USB drive, if found,
 *           update APROM flash with it. This sample requires booting from LDROM with IAP.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usbh_lib.h"
#include "ff.h"
#include "diskio.h"

#define USE_USB_PLL_CLOCK         0
#define USB_UPDATER_BASE           0x000000

/*
        This sample program assumed flash resource allocated like the following figure:

                                     Address            Description
                                     ---------------    -------------------------
     +---------------------------+   0xF101FFF
     |  LDROM                    |                      USB firmware update code boots from LDROM
     |  (updater boot part)      |                      and branches to USB_UPDATER_BASE in APROM.
     +---------------------------+   0xF100000
     |                           |
     ~                           ~
     |                           |
     +---------------------------+   0x60000
     |                           |
     |                           |
     |                           |
     |  APROM                    |                      USB firmware updater USB function part.
     |(updater USB function part)|
     +---------------------------+   0x20000
     |                           |
     |  APROM                    |                      User application
     |                           |
     |                           |
     |                           |
     |                           |
     +---------------------------+   USB_UPDATER_BASE (0x000000)
*/


extern void usbh_firmware_update(void);     /* USB firmware update main function          */

volatile uint32_t  g_tick_cnt;              /* SYSTICK timer counter                      */

void SysTick_Handler(void)
{
    g_tick_cnt++;                           /* timer tick counting 100 per second         */
}

void enable_sys_tick(int ticks_per_second)
{
    g_tick_cnt = 0;

    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");

        while (1);
    }
}

/*
 *  This function is necessary for USB Host library.
 */
uint32_t get_ticks()                        /* Get timer tick                             */
{
    return g_tick_cnt;
}

/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from HIRC_48M. Prescale 48
     */
    /* TIMER0 clock from HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    CLK_EnableModuleClock(TMR0_MODULE);

    TIMER_SET_PRESCALE_VALUE(TIMER0, (48 - 1));
    /* stop timer0 */
    TIMER_Stop(TIMER0);
    /* write 1 to clear for safety */
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    /* set timer cmp value */
    TIMER_SET_CMP_VALUE(TIMER0, usec);
    /* Timer0 config to oneshot mode */
    TIMER_SET_OPMODE(TIMER0, TIMER_ONESHOT_MODE);
    /* start timer0*/
    TIMER_Start(TIMER0);

    while (TIMER_GetIntFlag(TIMER0) == 0);
}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */
/*---------------------------------------------------------*/
DWORD get_fattime(void)
{
    DWORD tmr;
    tmr = 0x00000;
    return tmr;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 144MHZ */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Enable GPIOA module clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

#if (USE_USB_PLL_CLOCK)
    /* USB Host desired input clock is 48 MHz. Set as PLL divided by 3 (144/3 = 48) */
    CLK_SetModuleClock(USBH_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(3));
#else
    /* USB Host desired input clock is 48 MHz. Set as HIRC48M divided by 1 (48/1 = 48) */
    CLK_SetModuleClock(USBH_MODULE, CLK_CLKSEL0_USBSEL_HIRC, CLK_CLKDIV0_USB(1));
#endif

    /* Enable USBH module clock */
    CLK_EnableModuleClock(USBH_MODULE);

    /* Set OTG as USB Host role */
    SYS->USBPHY = SYS_USBPHY_USBEN_Msk | (0x1 << SYS_USBPHY_USBROLE_Pos);
    delay_us(20);

    /* Set Debug Uart CLK*/
    SetDebugUartCLK();
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SET_USB_VBUS_EN_PB15();

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.14   */
    SET_USB_VBUS_ST_PB14();


    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{
    SYS_Init();                        /* Init System, IP clock and multi-function I/O    */

    InitDebugUart();                   /* Init DebugUART for printf                       */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+---------------------------------------------+\n");
    printf("|                                             |\n");
    printf("|  USB Host Firmware Update sample program    |\n");
    printf("|                                             |\n");
    printf("+---------------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    FMC_Open();                             /* Enable FMC ISP functions                   */

    usbh_core_init();
    usbh_umas_init();
    usbh_pooling_hubs();

    delay_us(1000000);                       /* delay 100ms for some slow response pen drive. */

    usbh_pooling_hubs();
    usbh_pooling_hubs();

    usbh_firmware_update();

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /* disable SYSTICK (prevent interrupt)   */

    /* Switch HCLK clock source to HIRC. Restore HCLK to default setting. */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    FMC_SetVectorPageAddr(USB_UPDATER_BASE);               /* Set vector remap to APROM address 0x0      */

    delay_us(1000000);                       /* delay 100ms for some slow response pen drive. */

    FMC_SetVectorPageAddr(0);                   /* Change boot source as APROM                */

    SYS_ResetCPU();                         /* Let CPU reset. Will boot from APROM.       */

    /* This reset can bring up user application   */
    /* in APROM address 0x100000.                 */
    /* Please make sure user application has been */
    /* programmed to APROM 0x100000 at this time. */
    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/