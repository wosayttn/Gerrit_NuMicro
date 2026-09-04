/*----------------------------------------------------------------------------
 * Name:    main.c
 *----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "cmsis_dv.h"                   // ARM.API::CMSIS Driver Validation:Framework
#include "NuMicro.h"
/* Private functions ---------------------------------------------------------*/

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL 144MHz clock from HIRC and switch HCLK clock source to PLL */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Select CAN FD0 clock source is APLL0/2 */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_PLL, CLK_CLKDIV1_CANFD0(1));

    /* Select CAN FD0 clock source is APLL0/2 */
    CLK_SetModuleClock(CANFD1_MODULE, CLK_CLKSEL0_CANFD1SEL_PLL, CLK_CLKDIV1_CANFD1(1));

    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD1_MODULE);

    /* Set PJ multi-function pins for CANFD RXD and TXD */
    SET_CANFD0_RXD_PC4();
    SET_CANFD0_TXD_PC5();
    SET_CANFD1_RXD_PB6();
    SET_CANFD1_TXD_PB7();

    /* Lock protected registers */
    //SYS_LockReg();
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
