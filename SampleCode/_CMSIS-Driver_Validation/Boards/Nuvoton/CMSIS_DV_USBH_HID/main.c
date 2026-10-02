/*----------------------------------------------------------------------------
 * Name:    main.c
 *----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os2.h"                   // ARM.API::CMSIS Driver Validation:Framework
#include "NuMicro.h"
#include "rl_usb.h"
#include <stdio.h>
/* Private functions ---------------------------------------------------------*/

#define USE_USB_APLL1_CLOCK         1

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

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

#if (USE_USB_APLL1_CLOCK)
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
    osDelay(20);

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
