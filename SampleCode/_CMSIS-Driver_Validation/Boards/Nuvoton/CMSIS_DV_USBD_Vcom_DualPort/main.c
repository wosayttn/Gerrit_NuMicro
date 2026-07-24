/*----------------------------------------------------------------------------
 * Name:    main.c
 *----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "NuMicro.h"
#include "RTE_Device/RTE_Device.h"
#include "../CMSIS_DV_USART/USART_PinConfig.h"          // PinConfig for USART driver
#define USE_USB_APLL1_CLOCK        1
extern int app_main(void);
/* Private functions ---------------------------------------------------------*/
__WEAK void SetUsartCLK(void)
{
#if (RTE_USART0)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL0_UART0SEL_HIRC, CLK_UARTDIV0_UART0DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
#endif
#if (RTE_USART1)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART1_MODULE, CLK_UARTSEL0_UART1SEL_HIRC, CLK_UARTDIV0_UART1DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
#endif
#if (RTE_USART2)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART2_MODULE, CLK_UARTSEL0_UART2SEL_HIRC, CLK_UARTDIV0_UART2DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART2_MODULE);
#endif
#if (RTE_USART3)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART3_MODULE, CLK_UARTSEL0_UART3SEL_HIRC, CLK_UARTDIV0_UART3DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART3_MODULE);
#endif
#if (RTE_USART4)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART4_MODULE, CLK_UARTSEL0_UART4SEL_HIRC, CLK_UARTDIV0_UART4DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART4_MODULE);
#endif
#if (RTE_USART5)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART5_MODULE, CLK_UARTSEL0_UART5SEL_HIRC, CLK_UARTDIV0_UART5DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART5_MODULE);
#endif
#if (RTE_USART6)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART6_MODULE, CLK_UARTSEL0_UART6SEL_HIRC, CLK_UARTDIV0_UART6DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART6_MODULE);
#endif
#if (RTE_USART7)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART7_MODULE, CLK_UARTSEL0_UART7SEL_HIRC, CLK_UARTDIV0_UART7DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART7_MODULE);
#endif
#if (RTE_USART8)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART8_MODULE, CLK_UARTSEL1_UART8SEL_HIRC, CLK_UARTDIV1_UART8DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART8_MODULE);
#endif
#if (RTE_USART9)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(UART9_MODULE, CLK_UARTSEL1_UART9SEL_HIRC, CLK_UARTDIV1_UART9DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(UART9_MODULE);
#endif
#if (RTE_USART10)
    /* Enable SC0 module clock and clock source from HIRC divide 1 */
    CLK_SetModuleClock(SC0_MODULE, CLK_SCSEL_SC0SEL_HIRC, CLK_SCDIV_SC0DIV(1));

    /* Enable module clock */
    CLK_EnableModuleClock(SC0_MODULE);
#endif
#if (RTE_USART11)
    /* Enable SC1 module clock and clock source from HIRC divide 1 */
    CLK_SetModuleClock(SC1_MODULE, CLK_SCSEL_SC1SEL_HIRC, CLK_SCDIV_SC1DIV(1));

    /* Enable module clock */
    CLK_EnableModuleClock(SC1_MODULE);
#endif
#if (RTE_USART12)
    /* Enable SC2 module clock and clock source from HIRC divide 1 */
    CLK_SetModuleClock(SC2_MODULE, CLK_SCSEL_SC2SEL_HIRC, CLK_SCDIV_SC2DIV(1));

    /* Enable module clock */
    CLK_EnableModuleClock(SC2_MODULE);
#endif
#if (RTE_USART13)
    /* Set clock source from HIRC divide 1 */
    CLK_SetModuleClock(LPUART0_MODULE, CLK_LPUARTSEL_LPUART0SEL_HIRC, CLK_LPUARTDIV_LPUART0DIV(1));
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(LPUART0_MODULE);
#endif
#if (RTE_USART14)
    /* Enable module peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);
#endif
};

__WEAK void SetUsartMFP(void)
{
#if (RTE_USART0)
    RTE_SET_USART0_TX_PIN();
    RTE_SET_USART0_RX_PIN();
    RTE_SET_USART0_CTS_PIN();
    RTE_SET_USART0_RTS_PIN();
#endif
#if (RTE_USART1)
    RTE_SET_USART1_TX_PIN();
    RTE_SET_USART1_RX_PIN();
    RTE_SET_USART1_CTS_PIN();
    RTE_SET_USART1_RTS_PIN();
#endif
#if (RTE_USART2)
    RTE_SET_USART2_TX_PIN();
    RTE_SET_USART2_RX_PIN();
    RTE_SET_USART2_CTS_PIN();
    RTE_SET_USART2_RTS_PIN();
#endif
#if (RTE_USART3)
    RTE_SET_USART3_TX_PIN();
    RTE_SET_USART3_RX_PIN();
    RTE_SET_USART3_CTS_PIN();
    RTE_SET_USART3_RTS_PIN();
#endif
#if (RTE_USART4)
    RTE_SET_USART4_TX_PIN();
    RTE_SET_USART4_RX_PIN();
    RTE_SET_USART4_CTS_PIN();
    RTE_SET_USART4_RTS_PIN();
#endif
#if (RTE_USART5)
    RTE_SET_USART5_TX_PIN();
    RTE_SET_USART5_RX_PIN();
    RTE_SET_USART5_CTS_PIN();
    RTE_SET_USART5_RTS_PIN();
#endif
#if (RTE_USART6)
    RTE_SET_USART6_TX_PIN();
    RTE_SET_USART6_RX_PIN();
    RTE_SET_USART6_CTS_PIN();
    RTE_SET_USART6_RTS_PIN();
#endif
#if (RTE_USART7)
    RTE_SET_USART7_TX_PIN();
    RTE_SET_USART7_RX_PIN();
    RTE_SET_USART7_CTS_PIN();
    RTE_SET_USART7_RTS_PIN();
#endif
#if (RTE_USART8)
    RTE_SET_USART8_TX_PIN();
    RTE_SET_USART8_RX_PIN();
    RTE_SET_USART8_CTS_PIN();
    RTE_SET_USART8_RTS_PIN();
#endif
#if (RTE_USART9)
    RTE_SET_USART9_TX_PIN();
    RTE_SET_USART9_RX_PIN();
    RTE_SET_USART9_CTS_PIN();
    RTE_SET_USART9_RTS_PIN();
#endif
#if (RTE_USART10)
    SET_SC0_CLK_PA0();
    SET_SC0_DAT_PA1();
#endif
#if (RTE_USART11)
    SET_SC1_CLK_PH1();
    SET_SC1_DAT_PH0();
#endif
#if (RTE_USART12)
    SET_SC2_CLK_PA15();
    SET_SC2_DAT_PA14();
#endif
#if (RTE_USART13)
    RTE_SET_USART13_TX_PIN();
    RTE_SET_USART13_RX_PIN();
    RTE_SET_USART13_CTS_PIN();
    RTE_SET_USART13_RTS_PIN();
#endif
#if (RTE_USART14)
    RTE_SET_USART14_TX_PIN();
    RTE_SET_USART14_RX_PIN();
    RTE_SET_USART14_CTS_PIN();
    RTE_SET_USART14_RTS_PIN();
#endif
};

__WEAK void SetUsbdCLK(void)
{
#if (RTE_USBD0)
#if (USE_USB_APLL1_CLOCK)
    /* Select USB clock source as PLL/2 and USB clock divider as 2 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_APLL1_DIV2, CLK_USBDIV_USBDIV(2));
#else
    /* Select USB clock source as HIRC48M and USB clock divider as 1 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));
#endif
    /* Enable OTG0 module clock */
    CLK_EnableModuleClock(OTG0_MODULE);

    /* Select USB role to USBD and Enable PHY */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk ;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD0_MODULE);
#endif
#if (RTE_USBD1)
    /* Enable External RC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);
    /* Select HSOTG PHY Reference clock frequency which is from HXT */
#if (__HXT == 19200000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_19_2M);
#elif (__HXT == 20000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_20_0M);
#elif (__HXT == 24000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);
#elif (__HXT == 16000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_16_0M);
#elif (__HXT == 26000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_26_0M);
#elif (__HXT == 32000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_32_0M);
#else
#warning This HXT cannot make HSUSB work properly.
#endif

    /* Set HSUSB role to HSUSBD */
    SET_HSUSBDROLE();

    /* Enable HSUSB PHY */
    SYS_Enable_HSUSB_PHY();

    /* Enable HSUSBD peripheral clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);
#endif
};

__WEAK void SetUsbdMFP(void)
{
#if (RTE_USBD0)
    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();
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
    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

#if (USE_USB_APLL1_CLOCK)
    /* Enable APLL0 192MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_192MHZ, CLK_APLL1_SELECT);
#else
    /* Enable HIRC48M clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);
    /* Waiting for HIRC48M clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);
#endif

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable debug UART module clock */
    SetDebugUartCLK();

    /* Enable USART driver module clock */
    SetUsartCLK();

    /* Enable Usbd driver module clock */
    SetUsbdCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set debug UART module MFP */
    SetDebugUartMFP();
    /* Set USART driver module MFP */
    SetUsartMFP();

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
