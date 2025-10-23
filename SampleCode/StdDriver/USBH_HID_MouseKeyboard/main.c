/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use USB Host core driver and HID driver. This sample demonstrates how
 *           to support mouse and keyboard input.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usbh_lib.h"
#include "usbh_hid.h"

#define USE_USB_PLL_CLOCK         1

static HID_DEV_T *s_hid_list[CONFIG_HID_MAX_DEV];

extern void keycode_process(KEYBOARD_EVENT_T *kbd);

static volatile uint32_t s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void dump_buff_hex(uint8_t *pu8Buff, int i8Bytes);
int is_a_new_hid_device(HID_DEV_T *hdev);
void update_hid_device_list(HID_DEV_T *hdev);
void int_read_callback(HID_DEV_T *hdev, uint16_t u16EpAddr, int i8Status, uint8_t *pu8RData, uint32_t u32DataLen);
int init_hid_device(HID_DEV_T *hdev);
void mouse_callback(struct usbhid_dev *hdev, MOUSE_EVENT_T *mouse);
void keyboard_callback(struct usbhid_dev *hdev, KEYBOARD_EVENT_T *kbd);
void SYS_Init(void);

/**
 * @brief    Check any char input from UART
 *
 * @param    None
 *
 * @retval   0: No any char input
 * @retval   1: Have some char input
 *
 * @details  Check UART RSR RX EMPTY or not to determine if any char input from UART
 */ 
int kbhit(void)
{
    return !((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == UART_FIFOSTS_RXEMPTY_Msk);
}

void SysTick_Handler(void)
{
    s_u32TickCnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    s_u32TickCnt = 0;

    if (SysTick_Config(SystemCoreClock / (uint32_t)ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");

        while (1);
    }
}

uint32_t get_ticks(void)
{
    return s_u32TickCnt;
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

void dump_buff_hex(uint8_t *pu8Buff, int i8Bytes)
{
    int nIdx, i;

    nIdx = 0;

    while (i8Bytes > 0)
    {
        printf("0x%04X  ", nIdx);

        for (i = 0; (i < 16) && (i8Bytes > 0); i++)
        {
            printf("%02x ", pu8Buff[nIdx + i]);
            i8Bytes--;
        }

        nIdx += 16;
        printf("\n");
    }

    printf("\n");
}

int is_a_new_hid_device(HID_DEV_T *hdev)
{
    int i;

    for (i = 0; i < CONFIG_HID_MAX_DEV; i++)
    {
        if ((s_hid_list[i] != NULL) && (s_hid_list[i] == hdev) &&
                (s_hid_list[i]->uid == hdev->uid))
            return 0;
    }

    return 1;
}

void update_hid_device_list(HID_DEV_T *hdev)
{
    int i = 0;

    memset(s_hid_list, 0, sizeof(s_hid_list));

    while ((i < CONFIG_HID_MAX_DEV) && (hdev != NULL))
    {
        s_hid_list[i++] = hdev;
        hdev = hdev->next;
    }
}

void int_read_callback(HID_DEV_T *hdev, uint16_t u16EpAddr, int i8Status, uint8_t *pu8RData, uint32_t u32DataLen)
{
    /* This callback is in interrupt context! */
    /*
     *  USB host HID driver notify user the transfer status via <i8Status> parameter. If the
     *  If <i8Status> is 0, the USB transfer is fine. If <i8Status> is not zero, this interrupt in
     *  transfer failed and HID driver will stop this pipe. It can be caused by USB transfer error
     *  or device disconnected.
     */
    if (i8Status < 0)
    {
        printf("Interrupt in transfer failed! status: %d\n", i8Status);
        return;
    }

    printf("Device [0x%x,0x%x] ep 0x%x, %d bytes received =>\n",
           hdev->idVendor, hdev->idProduct, u16EpAddr, u32DataLen);
    dump_buff_hex(pu8RData, (int)u32DataLen);
}

int init_hid_device(HID_DEV_T *hdev)
{
    int ret;

    printf("\n\n==================================\n");
    printf("  Init HID device : 0x%x\n", (int)hdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", hdev->idVendor, hdev->idProduct);

    printf("  bSubClassCode: 0x%x, bProtocolCode: 0x%x\n\n", hdev->bSubClassCode, hdev->bProtocolCode);

    printf("\nUSBH_HidStartIntReadPipe...\n");
    ret = usbh_hid_start_int_read(hdev, 0, int_read_callback);

    if (ret != HID_RET_OK)
        printf("usbh_hid_start_int_read failed! %d\n", ret);
    else
        printf("Interrupt in transfer started...\n");

    return 0;
}

/*
 *  Mouse callback function
 */
void mouse_callback(struct usbhid_dev *hdev, MOUSE_EVENT_T *mouse)
{
    (void)hdev;

    /* This callback is in interrupt context! */
    printf("X: %d, Y: %d, W: %d, button: 0x%x\n", mouse->X, mouse->Y, mouse->wheel, mouse->button_map);
}

/*
 *  Keyboard callback function
 */
void keyboard_callback(struct usbhid_dev *hdev, KEYBOARD_EVENT_T *kbd)
{
    (void)hdev;

    /* This callback is in interrupt context! */
    //int   i;
    //printf("[0x%x] ", kbd->modifier);
    //for (i = 0; i < kbd->key_cnt; i++)
    //  printf("%x ", kbd->keycode[i]);
    //printf("(0x%x)\n", kbd->lock_state);
    keycode_process(kbd);
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
    CLK_SetModuleClock(USBH0_MODULE, CLK_CLKSEL0_USBSEL_HIRC, CLK_USBDIV_USBDIV(1));
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
    HID_DEV_T *hdev, *hdev_list;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    InitDebugUart();                   /* Init DebugUART for printf */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+--------------------------------------------+\n");
    printf("|                                            |\n");
    printf("|       USB Host HID class sample demo       |\n");
    printf("|                                            |\n");
    printf("+--------------------------------------------+\n");

    usbh_core_init();
    usbh_hid_init();
    usbh_memory_used();

    usbh_hid_regitser_mouse_callback(mouse_callback);
    usbh_hid_regitser_keyboard_callback(keyboard_callback);

    memset(s_hid_list, 0, sizeof(s_hid_list));

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
                }

                hdev = hdev->next;
            }

            update_hid_device_list(hdev_list);
            usbh_memory_used();
        }

#ifndef DEBUG_ENABLE_SEMIHOST

        if (!kbhit())
        {
            getchar();
            usbh_memory_used();
        }

#endif
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/