/*----------------------------------------------------------------------------
 * Name:    main.c
 *----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "cmsis_dv.h"                   // ARM.API::CMSIS Driver Validation:Framework
#include "NuMicro.h"
#include "RTE_Device/RTE_Device.h"
#include "USART_PinConfig.h"            // PinConfig for USART driver
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

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable USART driver module clock */
    SetUsartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set USART driver module MFP */
    SetUsartMFP();

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

    osKernelInitialize();                // Initialize CMSIS-RTOS2

    osThreadNew(cmsis_dv, NULL, NULL);   // Create validation main thread

    osKernelStart();                     // Start thread execution

    for (;;) {}
}
