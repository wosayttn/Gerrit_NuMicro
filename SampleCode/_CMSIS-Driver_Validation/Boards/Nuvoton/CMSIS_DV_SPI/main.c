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

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LIRC clock */
    //CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for LXT clock ready */
    //CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable LXT clock */
    //CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    //CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

#if (DRV_SPI == 0)
    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select SPI0 module clock source as PCLK */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
#elif (DRV_SPI == 1)
    /* Enable SPI1 module clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Select SPI1 module clock source as PCLK */
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);
#elif (DRV_SPI == 2)
    /* Enable QSPI0 module clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

    /* Select SPI0 module clock source as PCLK1 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk);
#elif (DRV_SPI == 3)
    /* Enable USPI0 module clock */
    CLK_EnableModuleClock(USCI0_MODULE);
#elif (DRV_SPI == 4)
    /* Enable USPI1 module clock */
    CLK_EnableModuleClock(USCI1_MODULE);
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
#elif (DRV_SPI == 1)
    /* Setup SPI0 multi-function pins */
    SET_SPI1_MOSI_PC2();
    SET_SPI1_MISO_PC3();
    SET_SPI1_CLK_PC1();
    SET_SPI1_SS_PC0();
#elif (DRV_SPI == 2)
    SET_QSPI0_MOSI0_PA0();
    SET_QSPI0_MISO0_PA1();
    SET_QSPI0_CLK_PA2();
    SET_QSPI0_SS_PA3();
#elif (DRV_SPI == 3)
    /* Set USCI0_SPI multi-function pins */
    SET_USCI0_CTL0_PB0();
    SET_USCI0_CLK_PA11();
    SET_USCI0_DAT0_PA10();
    SET_USCI0_DAT1_PA9();
#elif (DRV_SPI == 4)
    /* Set USCI0_SPI multi-function pins */
    SET_USCI1_CTL0_PB10();
    SET_USCI1_CLK_PB1();
    SET_USCI1_DAT0_PB2();
    SET_USCI1_DAT1_PB3();
#endif

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Enable SPI1 clock pin (PC1) schmitt trigger */
    PC->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;

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
