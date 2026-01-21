/*----------------------------------------------------------------------------
 * Name:    main.c
 *----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os2.h"                   // ARM.API::CMSIS Driver Validation:Framework
#include "NuMicro.h"
#include "rl_usb.h"
/* Private functions ---------------------------------------------------------*/

#define USE_USB_APLL1_CLOCK         1

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Wait for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

#if (USE_USB_APLL1_CLOCK)
    /* Enable APLL1 96MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, 96000000, CLK_APLL1_SELECT);
#endif

    /* Enable GPIOA module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);
    /* Select HSOTG PHY Reference clock frequency which is from HXT*/
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);

#if (USE_USB_APLL1_CLOCK)
    /* USB Host desired input clock is 48 MHz. Set as APLL1 divided by 2 (96/2 = 48) */
    CLK_SetModuleClock(USBH0_MODULE, CLK_USBSEL_USBSEL_APLL1_DIV2, CLK_USBDIV_USBDIV(1));
#else
    /* USB Host desired input clock is 48 MHz. Set as HIRC48M divided by 1 (48/1 = 48) */
    CLK_SetModuleClock(USBH0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));
#endif
    /* Enable USBH module clock */
    CLK_EnableModuleClock(USBH0_MODULE);
    CLK_EnableModuleClock(USBD0_MODULE);
    CLK_EnableModuleClock(OTG0_MODULE);
    /* Enable HSUSBH module clock */
    CLK_EnableModuleClock(HSUSBH0_MODULE);

    /* Set OTG as USB Host role */
    SYS->USBPHY = (0x1ul << (SYS_USBPHY_HSOTGPHYEN_Pos)) | (0x1ul << (SYS_USBPHY_HSUSBROLE_Pos)) | (0x1ul << (SYS_USBPHY_OTGPHYEN_Pos)) | (0x1 << SYS_USBPHY_USBROLE_Pos);
    osDelay(20);
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SET_USB_VBUS_EN_PB15();

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.14   */
    SET_USB_VBUS_ST_PB14();

    /* HSUSB_VBUS_EN (USB 2.0 VBUS power enable pin) multi-function pin - PJ.13   */
    SET_HSUSB_VBUS_EN_PJ13();

    /* HSUSB_VBUS_ST (USB 2.0 over-current detect pin) multi-function pin - PJ.12 */
    SET_HSUSB_VBUS_ST_PJ12();

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();

    /* Lock protected registers */
    SYS_LockReg();
}

//USB KeyBoard Thread
/*-----------------------------------------------------------------------------
 * Application main thread
 *----------------------------------------------------------------------------*/
__NO_RETURN void app_main_thread(void *argument)
{
    usbStatus usb_status;                 // USB status
    usbStatus hid_status[3];                 // HID status
    int       ch;                         // Character
    uint8_t   con[3] = {0U, 0U, 0U};                 // Connection status of keyboard
    uint8_t   ctrl = 0U;

    (void)argument;

    printf("USB Host HID Keyboard example\n");

    usb_status = USBH_Initialize(0U);     // Initialize USB Host 0

    if (usb_status != usbOK)
    {
        printf("USBH0 - Fail\n");

        for (;;) {}                         // Handle USB Host 0 init failure
    }

    usb_status = USBH_Initialize(1U);     // Initialize USB Host 1

    if (usb_status != usbOK)
    {
        printf("USBH1 - Fail\n");

        for (;;) {}                         // Handle USB Host 1 init failure
    }

    usb_status = USBH_Initialize(2U);     // Initialize USB Host 2

    if (usb_status != usbOK)
    {
        //for (;;) {}                         // Handle USB Host 2 init failure
        printf("USBH2 - Fail\n");
    }

    for (;;)
    {
        hid_status[ctrl] = USBH_HID_GetDeviceStatus(ctrl);   // Get HID device status

        if (hid_status[ctrl] == usbOK)
        {
            if (con[ctrl] == 0U)                    // If keyboard was not connected previously
            {
                con[ctrl] = 1U;                       // Keyboard got connected
                printf("Keyboard connected!\n");
            }
        }
        else
        {
            if (con[ctrl] == 1U)                    // If keyboard was connected previously
            {
                con[ctrl] = 0U;                       // Keyboard got disconnected
                printf("\nKeyboard disconnected!\n");
            }
        }

        if (con[ctrl] != 0U)                      // If keyboard is active
        {
            ch = USBH_HID_GetKeyboardKey(ctrl);  // Get pressed key

            if (ch != -1)                     // If valid key value
            {
                if ((ch & 0x10000) != 0)        // Handle non-ASCII translated keys (Keypad 0 .. 9)
                {
                    // Bit  16:    non-ASCII bit (0 = ASCII, 1 = not ASCII)
                    // Bits 15..8: modifiers (SHIFT, ALT, CTRL, GUI)
                    // Bits  7..0: ASCII or HID key Usage ID if not ASCII
                    // HID Usage ID values can be found in following link:
                    // http://www.usb.org/developers/hidpage/Hut1_12v2.pdf
                    ch &= 0xFF;                   // Remove non-ASCII bit and modifiers

                    if ((ch >= 0x59) && (ch <= 0x61)) // Keypad 1 .. 9 key convert to
                    {
                        ch = (ch - 0x59) + '1';     // ASCII  1 .. 9
                    }
                    else if (ch == 0x62)          // Keypad 0 key convert to
                    {
                        ch = '0';                   // ASCII  0
                    }
                    else                          // If not Keypad 0 .. 9
                    {
                        ch = -1;                    // invalidate the key
                    }
                }

                if ((ch > 0) && (ch < 128))     // Output ASCII 0 .. 127 range
                {
                    putchar(ch);
                    fflush(stdout);
                }
            }
        }

        ctrl += 1;

        if (ctrl == 3) ctrl = 0;

        osDelay(10U);
    }
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();

    osKernelInitialize();                // Initialize CMSIS-RTOS2

    osThreadNew(app_main_thread, NULL, NULL);   // Create validation main thread

    osKernelStart();                     // Start thread execution

    for (;;) {}
}
