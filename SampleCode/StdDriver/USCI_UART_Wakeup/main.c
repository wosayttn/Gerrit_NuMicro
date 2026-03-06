/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to wake up system from Power-down mode by USCI interrupt in UART mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/* UUART can support NPD0 ~ NDP2 power-down mode */
#define TEST_POWER_DOWN_MODE    CLK_PMUCTL_PDMSEL_NPD2

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void USCI_UART_DataWakeUp(void);
void USCI_UART_CTSWakeUp(void);
void USCI_UART_PowerDown_TestItem(void);
void USCI_UART_PowerDownWakeUpTest(void);
void PowerDownFunction(void);
void SYS_Init(void);
void UART0_Init(void);
void USCI0_Init(void);
void USCI0_IRQHandler(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    /* Set Power-down mode */
    CLK_SetPowerDownMode(TEST_POWER_DOWN_MODE);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART and USCI module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set multi-function pins for USCI0_DAT0(PA.10), USCI0_DAT1(PA.9), USCI0_CTL0(PB.0) and USCI0_CTL1(PA.8) */
    SYS->GPA_MFP2 &= ~(SYS_GPA_MFP2_PA8MFP_Msk | SYS_GPA_MFP2_PA9MFP_Msk | SYS_GPA_MFP2_PA10MFP_Msk);
    SYS->GPA_MFP2 |= (SYS_GPA_MFP2_PA8MFP_USCI0_CTL1 | SYS_GPA_MFP2_PA9MFP_USCI0_DAT1 |SYS_GPA_MFP2_PA10MFP_USCI0_DAT0);
    SYS->GPB_MFP0 &= ~(SYS_GPB_MFP0_PB0MFP_Msk);
    SYS->GPB_MFP0 |= (SYS_GPB_MFP0_PB0MFP_USCI0_CTL0);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void USCI0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS_ResetModule(USCI0_RST);

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 9600);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init USCI0 for test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("USCI UART Sample Program\n\n");

    /* USCI UART Power-down and Wake-up sample function */
    USCI_UART_PowerDownWakeUpTest();

    printf("\nUSCI UART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI0 interrupt event                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    uint32_t u32IntSts = UUART_GET_PROT_STATUS(UUART0);
    uint32_t u32WkSts = UUART_GET_WAKEUP_FLAG(UUART0);

    if(u32WkSts & UUART_WKSTS_WKF_Msk)  /* USCI UART wake-up flag */
    {
        UUART_CLR_WAKEUP_FLAG(UUART0);
        printf("USCI UART wake-up.\n");
    }
    else if(u32IntSts & UUART_PROTSTS_RXENDIF_Msk)  /* USCI UART receive end interrupt flag */
    {
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);

        while(UUART_GET_RX_EMPTY(UUART0) == 0)
        {
            printf("Data: 0x%X\n", UUART_READ(UUART0));
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_CTSWakeUp(void)
{
    /* Enable UART nCTS wake-up function */
    UUART_EnableWakeup(UUART0, UUART_PROTCTL_CTSWKEN_Msk);

    printf("System enter to Power-down mode.\n");
    printf("Toggle USCI-UART0 nCTS to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_DataWakeUp(void)
{
    uint16_t u16regCLKDIV, u16regPDSCNT;
    uint32_t u32WakeupCount, u32PCLK;
    double fWakeupTime = 71;  /* unit: us */

    /* Get PCLK */
    u32PCLK = CLK_GetPCLK0Freq();

    /* Get Divider */
    u16regCLKDIV = (UUART0->BRGEN & UUART_BRGEN_CLKDIV_Msk) >> UUART_BRGEN_CLKDIV_Pos;
    u16regPDSCNT = (UUART0->BRGEN & UUART_BRGEN_PDSCNT_Msk) >> UUART_BRGEN_PDSCNT_Pos;

    /* Calculate wake-up counter */
    u32WakeupCount = (uint32_t)(fWakeupTime * (u32PCLK / 1000000) / ((u16regCLKDIV + 1) * (u16regPDSCNT + 1)));
    if(u32WakeupCount > 15)
    {
        printf("Fail to calculate wake-up counter. USCI-UART would not get correct data after wake-up.\n");
    }

    /* Enable UART data wake-up function */
    UUART_EnableWakeup(UUART0, UUART_PROTCTL_DATWKEN_Msk);

    /* Set UART data wake-up counter.
       It indicates how many clock cycle does UART can get the first bit (start bit)
       when the device is wake-up from Power-down mode.
       Clock is (USCI clock source)/((CLKDIV+1)(PDSCNT+1)) */
    UUART0->PROTCTL = (UUART0->PROTCTL & (~UUART_PROTCTL_WAKECNT_Msk)) | (u32WakeupCount << UUART_PROTCTL_WAKECNT_Pos);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to USCI-UART0 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  USCI-UART Power-down and wake-up test                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                           - [Others] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    printf("Due to PLL clock stable too slow.\n");
    printf("Before demo USCI UART wake-up, this demo code will switch HCLK from PLL to HIRC.\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nCPU @ %dHz\n", SystemCoreClock);

    /* Set UUART0 Line config */
    UUART_SetLine_Config(UUART0, 9600, UUART_WORD_LEN_8, UUART_PARITY_NONE, UUART_STOP_BIT_1);

    /* Enable UART receive end interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_EnableIRQ(USCI0_IRQn);

    USCI_UART_PowerDown_TestItem();
    u32Item = (uint32_t)getchar();
    printf("%c\n\n", u32Item);
    switch(u32Item)
    {
    case '1':
        USCI_UART_CTSWakeUp();
        break;
    case '2':
        USCI_UART_DataWakeUp();
        break;
    default:
        return;
    }

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Wait until USCI UART data transmission is finished */
    while(UUART0->PROTSTS & UUART_PROTSTS_RXBUSY_Msk);

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    printf("Enter any key to end test.\n");
    getchar();

    /* Disable UART wake-up function */
    UUART_DisableWakeup(UUART0);

    /* Disable UART receive end interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_DisableIRQ(USCI0_IRQn);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Lock protected registers */
    SYS_LockReg();

    /* Set UUART0 Line config */
    UUART_SetLine_Config(UUART0, 9600, UUART_WORD_LEN_8, UUART_PARITY_NONE, UUART_STOP_BIT_1);

}
