#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Driver_SPI.h"
#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS2:Keil RTX5
#include "NuMicro.h"
#include "DV_Framework.h"
#include "cmsis_dv.h"                   // ARM.API::CMSIS Driver Validation:Framework
#include "DV_SPI_Config.h"

//------------------------------------------------------------------------------
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

#if (DRV_SPI == 0)
    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select SPI0 module clock source as PCLK1 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_PCLK0, MODULE_NoMsk);
#elif (DRV_SPI == 4)
    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

    /* Select SPI0 module clock source as PCLK1 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_QSPISEL_QSPI0SEL_PCLK0, MODULE_NoMsk);
#elif (DRV_SPI == 6)
    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(LPSPI0_MODULE);

    /* Select SPI0 module clock source as PCLK1 */
    CLK_SetModuleClock(LPSPI0_MODULE, CLK_LPSPISEL_LPSPI0SEL_PCLK4, MODULE_NoMsk);
#elif (DRV_SPI == 7)
    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(USCI0_MODULE);
#elif (DRV_SPI == 8)
    /* Enable SPIM0 module clock */
    CLK_EnableModuleClock(SPIM0_MODULE);
#endif

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

#if (DRV_SPI == 0)
    /* Setup SPI0 multi-function pins */
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();
    SET_SPI0_CLK_PA2();
    SET_SPI0_SS_PA3();
#elif (DRV_SPI == 4)
    SET_QSPI0_MOSI0_PA0();
    SET_QSPI0_MISO0_PA1();
    SET_QSPI0_CLK_PA2();
    SET_QSPI0_SS_PA3();
#elif (DRV_SPI == 6)
    SET_LPSPI0_MOSI_PA0();
    SET_LPSPI0_MISO_PA1();
    SET_LPSPI0_CLK_PA2();
    SET_LPSPI0_SS_PA3();
#elif (DRV_SPI == 7)
    /* Set USCI0_SPI multi-function pins */
    SET_USCI0_CTL0_PB0();
    SET_USCI0_CLK_PA11();
    SET_USCI0_DAT0_PA10();
    SET_USCI0_DAT1_PA9();
#elif (DRV_SPI == 8)
    uint32_t u32SlewRate = GPIO_SLEWCTL_HIGH;

    /* Init SPIM multi-function pins */
    SET_SPIM0_CLK_PH13();
    SET_SPIM0_MISO_PJ4();
    SET_SPIM0_MOSI_PJ3();
    SET_SPIM0_D2_PJ5();
    SET_SPIM0_D3_PJ6();
    SET_SPIM0_SS_PJ7();

    PH->SMTEN |= (GPIO_SMTEN_SMTEN13_Msk);

    PJ->SMTEN |= (GPIO_SMTEN_SMTEN3_Msk |
                  GPIO_SMTEN_SMTEN4_Msk |
                  GPIO_SMTEN_SMTEN5_Msk |
                  GPIO_SMTEN_SMTEN6_Msk |
                  GPIO_SMTEN_SMTEN7_Msk);

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    GPIO_SetSlewCtl(PH, BIT13, u32SlewRate);

    GPIO_SetSlewCtl(PJ, BIT3, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT4, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT5, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT6, u32SlewRate);

#endif

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* USCI_SPI clock pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN11_Msk;

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

    printf("\nCMSIS Driver SPI Test\n");

    osKernelInitialize();                       // Initialize CMSIS-RTOS2

    osThreadNew((osThreadFunc_t)cmsis_dv, NULL, NULL);      // Create validation main thread

    osKernelStart();                            // Start thread execution

    for (;;) {}
}
