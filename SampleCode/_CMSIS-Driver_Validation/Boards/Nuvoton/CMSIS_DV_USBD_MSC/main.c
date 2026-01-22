/*----------------------------------------------------------------------------
 * Name:    main.c
 *----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "NuMicro.h"
#include "RTE_Device/RTE_Device.h"
#define USE_USB_APLL1_CLOCK        1
extern int app_main(void);
/* Private functions ---------------------------------------------------------*/
__WEAK void SetUsbdCLK(void)
{
#if (RTE_USBD0)
#if (USE_USB_APLL1_CLOCK)
    /* Select USB clock source as PLL/2 and USB clock divider as 2 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(1));
#else
    /* Select USBD module clock source as HIRC and USBD module clock divider as 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC, CLK_CLKDIV0_USB(1));
#endif
    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk ;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);
#endif
};

__WEAK void SetUsbdMFP(void)
{
#if (RTE_USBD0)
    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    //    SET_USB_VBUS_PA12();
    //    SET_USB_D_MINUS_PA13();
    //    SET_USB_D_PLUS_PA14();
    //    SET_USB_OTG_ID_PA15();
#endif
#if (RTE_USBD1)
    /* fixed-function pins */
#endif
};

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

#if (USE_USB_APLL1_CLOCK)
    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);
#else
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(144000000);
#endif

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable Usbd driver module clock */
    SetUsbdCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set USBD driver module MFP */
    SetUsbdMFP();

    /* Lock protected registers */
    SYS_LockReg();
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

    return (app_main());
}
