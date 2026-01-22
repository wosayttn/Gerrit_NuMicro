/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to initialize a SecureISP server mode via
 *           USB host connected to SecureISPDemo.
 *           This sample code needs to work with SecureISPDemo sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "usbh_lib.h"
#include "usbh_hid.h"

HID_DEV_T   *g_hid_list[CONFIG_HID_MAX_DEV];
uint8_t     s_au8BuffPool[1024] __ALIGNED(32);

extern int32_t Process_USBHCommand(HID_DEV_T *hdev);

volatile uint32_t  g_tick_cnt;
void SysTick_Handler(void)
{
    g_tick_cnt++;
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

uint32_t get_ticks()
{
    return g_tick_cnt;
}

void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from HIRC.
     */
    /* TIMER0 clock from HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    CLK_EnableModuleClock(TMR0_MODULE);
    /* Stop TIMER0 */
    TIMER_Stop(TIMER0);
    /* Clear TIMER0 interrupt flags for safety */
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    TIMER_SET_CMP_VALUE(TIMER0, usec);
    TIMER_SET_PRESCALE_VALUE(TIMER0, (__HIRC / FREQ_1MHZ) - 1);
    TIMER_SET_OPMODE(TIMER0, TIMER_ONESHOT_MODE);
    /* Start TIMER0 */
    TIMER_Start(TIMER0);

    while (TIMER_GetIntFlag(TIMER0) == 0);
}

void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;

    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);

        for (i = 0; (i < 16) && (nBytes > 0); i++)
        {
            printf("%02x ", pucBuff[nIdx + i]);
            nBytes--;
        }

        nIdx += 16;
        printf("\n");
    }

    printf("\n");
}

int  is_a_new_hid_device(HID_DEV_T *hdev)
{
    int    i;

    for (i = 0; i < CONFIG_HID_MAX_DEV; i++)
    {
        if ((g_hid_list[i] != NULL) && (g_hid_list[i] == hdev) &&
                (g_hid_list[i]->uid == hdev->uid))
            return 0;
    }

    return 1;
}

void update_hid_device_list(HID_DEV_T *hdev)
{
    int  i = 0;
    memset(g_hid_list, 0, sizeof(g_hid_list));

    while ((i < CONFIG_HID_MAX_DEV) && (hdev != NULL))
    {
        g_hid_list[i++] = hdev;
        hdev = hdev->next;
    }
}

int  init_hid_device(HID_DEV_T *hdev)
{
    uint8_t   *pu8DataBuff;
    int32_t   i32Ret = 0;

    pu8DataBuff = (uint8_t *)((uint32_t)s_au8BuffPool);

    printf("\n\n==================================\n");
    printf("  Init HID device : 0x%x\n", (int)hdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", hdev->idVendor, hdev->idProduct);

    i32Ret = usbh_hid_get_report_descriptor(hdev, pu8DataBuff, 1024);

    if (i32Ret > 0)
    {
        printf("\nDump report descriptor =>\n");
        dump_buff_hex(pu8DataBuff, i32Ret);
    }

    return 0;
}

uint32_t CLK_GetUSBFreq(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Get USB Peripheral Clock                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USB Peripheral clock = HIRC / (USBDIV + 1) */
    return __HIRC / (CLK_GetModuleClockDivider(USBH_MODULE) + 1);
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

    /* USB Host desired input clock is 48 MHz. Set as HIRC divided by 1 (48/1 = 48) */
    CLK_SetModuleClock(USBH_MODULE, CLK_CLKSEL0_USBSEL_HIRC, CLK_CLKDIV0_USB(1));

    /* Enable module clock */
    CLK_EnableModuleClock(USBH_MODULE);
    /* Enable CRYPTO and TRNG clock */
    CLK_EnableModuleClock(CRPT_MODULE);
    CLK_EnableModuleClock(TRNG_MODULE);
    RNG_Open();

    /* Set OTG as USB Host role */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | (SYS_USBPHY_USBROLE_STD_USBH | SYS_USBPHY_USBEN_Msk);
    delay_us(20);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SET_USB_VBUS_EN_PB15();
    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.14   */
    SET_USB_VBUS_ST_PB14();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

int32_t main(void)
{
    HID_DEV_T    *hdev, *hdev_list;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Disable CACHE to ensure execution */
    CACHE_Disable();
    printf("\n\n");
    printf("System clock:   %d Hz.\n", SystemCoreClock);
    printf("USB Host clock: %d Hz.\n", CLK_GetUSBFreq());
    printf("+-------------------------------------------+\n");
    printf("|      USB Host Secure ISP Sample Code      |\n");
    printf("+-------------------------------------------+\n");
    printf("Wait until any HID devices connected...\n\n");

    usbh_core_init();
    usbh_hid_init();
    usbh_memory_used();

    memset(g_hid_list, 0, sizeof(g_hid_list));

    while (1)
    {
        if (usbh_pooling_hubs())             /* USB Host port detect polling and management */
        {
            usbh_memory_used();              /* print out USB memory allocating information */

            printf("\n Has hub events.\n");
            hdev_list = usbh_hid_get_device_list();
            hdev = hdev_list;

            while (hdev != NULL)
            {
                if (is_a_new_hid_device(hdev))
                {
                    init_hid_device(hdev);
                    Process_USBHCommand(hdev);
                }

                hdev = hdev->next;
            }

            update_hid_device_list(hdev_list);
            usbh_memory_used();
        }
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
