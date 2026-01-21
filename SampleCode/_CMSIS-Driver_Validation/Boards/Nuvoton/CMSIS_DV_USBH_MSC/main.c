/*----------------------------------------------------------------------------
 * Name:    main.c
 *----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os2.h"                   // ARM.API::CMSIS Driver Validation:Framework
#include "NuMicro.h"
#include "USBH_MSC.h"

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
//USB MSC Thread
/*-----------------------------------------------------------------------------
 * Application main thread
 *----------------------------------------------------------------------------*/
char rbuf[100];
__NO_RETURN void app_main_thread(void *argument)
{
    usbStatus usb_status;                 // USB status
    int32_t   msc_status;                 // MSC status
    FILE    *f;
    uint8_t   con = 0U;                   // Connection status of MSC(s)

    (void)argument;
    printf("USB Host Mass Storage Test...\n");

    usb_status = USBH_Initialize(0U);     // Initialize USB Host 0

    if (usb_status != usbOK)
    {
        for (;;) {}                         // Handle USB Host 0 init failure
    }

    printf("USBH_Initialize - Done\n");

    for (;;)
    {
        msc_status = USBH_MSC_DriveGetMediaStatus("U0:");   // Get MSC device status

        if (msc_status == USBH_MSC_OK)
        {
            if (con == 0U)                    // If stick was not connected previously
            {
                con = 1U;                       // Stick got connected
                printf("DriveMount-Start\n");
                msc_status = USBH_MSC_DriveMount("U0:");
                printf("msc_status=%d\n", msc_status);

                if (msc_status != USBH_MSC_OK)
                {
                    continue;                     // Handle U0: mount failure
                }

                printf("write test ...\n");

                f = fopen("Test5.txt", "w");     // Open/create file for writing

                fprintf(f, "USB Host Mass Storage-1!\n");
                fprintf(f, "USB Host Mass Storage-2!\n");
                fprintf(f, "USB Host Mass Storage-3!\n");

                fclose(f);
                printf("write test done and close file.\n");
                printf("\n");

                printf("read test ...\n");

                f = fopen("Test5.txt", "r");     // Open/create file for writing

                fread(rbuf, 100, 1, f);


                printf("%s", rbuf);
                printf("\n");

                fclose(f);

                printf("read test done and close file.\n");

                msc_status = USBH_MSC_DriveUnmount("U0:");
                printf("msc_status=%d\n", msc_status);

                if (msc_status != USBH_MSC_OK)
                {
                    continue;                     // Handle U0: dismount failure
                }
            }
        }
        else
        {
            if (con == 1U)                    // If stick was connected previously
            {
                con = 0U;                       // Stick got disconnected
            }
        }

        osDelay(100U);
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
