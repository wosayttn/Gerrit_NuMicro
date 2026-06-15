#include <stdio.h>
#include "NuMicro.h"

extern int i2c_pair(void);

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
    /* Use SystemCoreClockUpdate() to calculate and update SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL0_UART0SEL_HIRC, CLK_UARTDIV0_UART0DIV(1));
    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Reset UART module */
    SYS_ResetModule(SYS_UART0RST);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
    /* Lock protected registers */
    SYS_LockReg();
}

__STATIC_INLINE void I2C_Init(void)
{
    printf("\nI2C Pin settings (SCL, SDA):\n");
    printf(" - I2C0   (PB5,  PB4)\n");
    printf(" - I2C1   (PB11, PB10)\n");
    printf(" - I2C2   (PA1,  PA0)\n");
    printf(" - I2C3   (PC3,  PC2)\n");
    printf(" - LPI2C0 (PC12, PC11)\n");
    printf(" - UI2C0  (PE2,  PE3)\n");
    printf(" - I3C    (PB1,  PB0)\n");
    SET_I2C0_SCL_PB5();
    SET_I2C0_SDA_PB4();
    SET_I2C1_SCL_PB11();
    SET_I2C1_SDA_PB10();
    SET_I2C2_SCL_PA1();
    SET_I2C2_SDA_PA0();
    SET_I2C3_SCL_PC3();
    SET_I2C3_SDA_PC2();
    SET_LPI2C0_SCL_PC12();
    SET_LPI2C0_SDA_PC11();
    SET_USCI0_CLK_PE2();
    SET_USCI0_DAT0_PE3();
    SET_I3C0_SCL_PB1();
    SET_I3C0_SDA_PB0();
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    GPIO_ENABLE_SCHMITT_TRIGGER(PA, (BIT0 | BIT1));
    GPIO_ENABLE_SCHMITT_TRIGGER(PB, (BIT0 | BIT1 | BIT4 | BIT5 | BIT10 | BIT11));
    GPIO_ENABLE_SCHMITT_TRIGGER(PC, (BIT2 | BIT3 | BIT11 | BIT12));
    GPIO_ENABLE_SCHMITT_TRIGGER(PE, (BIT2 | BIT3));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
    //
    I2C_Init();
    i2c_pair();
    printf("\nTest Done.\n");

    while (1);
}
