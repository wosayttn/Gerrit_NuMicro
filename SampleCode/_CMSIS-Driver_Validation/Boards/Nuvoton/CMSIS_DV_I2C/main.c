#include <stdio.h>
#include "NuMicro.h"

extern int i2c_pair(void);

void SYS_Init(void)
{
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);
    /* Set core clock to 144MHz */
    CLK_SetCoreClock(144000000);
    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
#ifdef __PLDM_EMU__
    /* Configure UART Baudrate */
    DEBUG_PORT->LINE = (UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1);
    // The setting is for Palladium
    DEBUG_PORT->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(153600, 38400));
#else
    /* Init UART to 115200-8n1 for print message */
    UART_Open(DEBUG_PORT, 115200);
#endif
}

void I2C_Init(void)
{
    CLK_EnableModuleClock(PDMA0_MODULE);
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);
    /* internal GPIO Pull-up control are only available for real chip */
    GPIO_SetPullCtl(PA, (BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7 | BIT10 | BIT11), GPIO_PUSEL_PULL_UP);
    GPIO_SetPullCtl(PB, (BIT0 | BIT1 | BIT4 | BIT5 | BIT8 | BIT9 | BIT14 | BIT15), GPIO_PUSEL_PULL_UP);
    GPIO_SetPullCtl(PC, (BIT2 | BIT3), GPIO_PUSEL_PULL_UP);
    GPIO_SetPullCtl(PG, (BIT0 | BIT1 | BIT2 | BIT3), GPIO_PUSEL_PULL_UP);
    //
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);
    CLK_EnableModuleClock(I2C2_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);
    CLK_EnableModuleClock(USCI1_MODULE);
    CLK_EnableModuleClock(I3C0_MODULE);

#if (I2C_VALIDATION_SET == 2) // I3C0
    // I3C0
    SET_I3C0_SCL_PA5();
    SET_I3C0_SDA_PA4();
#else
    // I2C0
    SET_I2C0_SCL_PA5();
    SET_I2C0_SDA_PA4();
#endif
    //
    SET_I2C1_SCL_PC5();
    SET_I2C1_SDA_PC4();
    //
    SET_I2C2_SCL_PA1();
    SET_I2C2_SDA_PA0();
    //
    SET_USCI0_CLK_PA11();
    SET_USCI0_DAT0_PA10();
    SET_USCI1_CLK_PD7();
    SET_USCI1_DAT0_PD5();
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
    UART_Init();
    //
    I2C_Init();
    i2c_pair();
    printf("\nTest Done.\n");

    while (1);
}
