/*----------------------------------------------------------------------------
 * Name:    main.c
 *----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os2.h"                   // ARM.API::CMSIS Driver Validation:Framework
#include "NuMicro.h"
#include "USBH_MSC.h"
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
