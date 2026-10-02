/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 20/08/06 5:45p $
 * @brief    Show how to wake up system from Power-down mode by LPUART interrupt.
 * @note
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define RS485_ADDRESS 0xC0

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_DataWakeUp(void);
void LPUART_CTSWakeUp(void);
void LPUART_RxThresholdWakeUp(void);
void LPUART_RS485WakeUp(void);
void LPUART_PowerDown_TestItem(void);
void LPUART_PowerDownWakeUpTest(void);

/* LPUART can support NPD0 ~ NDP4 power-down mode */
#define TEST_POWER_DOWN_MODE    CLK_PMUCTL_PDMSEL_NPD4

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Set Power-down mode */
    CLK_SetPowerDownMode(TEST_POWER_DOWN_MODE);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

void SYS_Init(void)
{

    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    GPIO_SetMode(PF, (BIT4 | BIT5), GPIO_MODE_INPUT);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC and LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable Low Power UART0 peripheral clock */
    CLK_EnableModuleClock(LPUART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select Low Power UART0 clock source is HIRC and Low Power UART module clock divider as 1*/
    CLK_SetModuleClock(LPUART0_MODULE, LPSCC_CLKSEL0_LPUART0SEL_HIRC, LPSCC_CLKDIV0_LPUART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for Low Power UART0 */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk )) |    \
                    (SYS_GPA_MFP0_PA0MFP_LPUART0_RXD | SYS_GPA_MFP0_PA1MFP_LPUART0_TXD );
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk)) |    \
                    (SYS_GPA_MFP1_PA4MFP_LPUART0_nRTS | SYS_GPA_MFP1_PA5MFP_LPUART0_nCTS);


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

void LPUART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LPUART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset Low Power UART0 */
    SYS_ResetModule(LPUART0_RST);

    /* Configure Low Power UART0 and set Low Power UART0 Baudrate */
    LPUART_Open(LPUART0, 9600);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init LPUART0 for test */
    LPUART0_Init();

    /* clear all wake-up flag */
    CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nLPUART Sample Program\n");

    /* LPUART Power-down and Wake-up sample function */
    LPUART_PowerDownWakeUpTest();

    printf("LPUART Sample Program End.\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle LPUART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART0_IRQHandler(void)
{
    uint32_t u32Data;

    if(LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_WKINT_Msk))     /* LPUART wake-up interrupt flag */
    {
        LPUART_ClearIntFlag(LPUART0, LPUART_INTSTS_WKINT_Msk);
        printf("LPUART wake-up.\n");
        UART_WAIT_TX_EMPTY(UART0);
    }
    else if(LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_RDAINT_Msk | LPUART_INTSTS_RXTOINT_Msk))     /* LPUART receive data available flag */
    {
        while(LPUART_GET_RX_EMPTY(LPUART0) == 0)
        {
            u32Data = LPUART_READ(LPUART0);
            if(u32Data & LPUART_DAT_PARITY_Msk)
                printf("Address: 0x%X\n", (u32Data & 0xFF));
            else
                printf("Data: 0x%X\n", u32Data);
        }
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_CTSWakeUp(void)
{
    /* Enable LPUART nCTS wake-up function */
    LPUART0->WKCTL |= LPUART_WKCTL_WKCTSEN_Msk;

    printf("System enter to Power-down mode NPD%d.\n", (int)(CLK->PMUCTL & CLK_PMUCTL_PDMSEL_Msk));
    printf("Toggle LPUART0 nCTS to wake-up system.\n\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_DataWakeUp(void)
{
    /* Enable LPUART data wake-up function */
    LPUART0->WKCTL |= LPUART_WKCTL_WKDATEN_Msk;

    /* Set LPUART data wake-up start bit compensation value.
       It indicates how many clock cycle selected by LPUART_CLK does the LPUART controller can get the first bit (start bit)
       when the device is wake-up from power-down mode.
       If LPUART_CLK is selected as HIRC(12MHz) and the HIRC stable time is about 52.03us,
       the data wake-up start bit compensation value can be set as 0x270. */
    LPUART0->DWKCOMP = 0x270;

    printf("System enter to Power-down mode NPD%d.\n", (int)(CLK->PMUCTL & CLK_PMUCTL_PDMSEL_Msk));
    printf("Send data with baud rate 9600bps to LPUART0 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Rx threshold and time-out Function                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_RxThresholdWakeUp(void)
{
    /* Wait data transmission is finished and select LPUART clock source as LXT */
    while((LPUART0->FIFOSTS & LPUART_FIFOSTS_TXEMPTYF_Msk) == 0);
    while((LPUART0->FIFOSTS & LPUART_FIFOSTS_RXIDLE_Msk) == 0);
    CLK_SetModuleClock(LPUART0_MODULE, LPSCC_CLKSEL0_LPUART0SEL_LXT, LPSCC_CLKDIV0_LPUART0(1));

    /* Keep LPUART clock for LXT work in power down mode */
    LPSCC->CLKKEEP0 |= LPSCC_CLKKEEP0_LPUART0KEEP_Msk;

    /* Set LPUART baud rate and baud rate compensation */
    LPUART_Open(LPUART0, 9600);
    LPUART0->BRCOMP = 0xA5;

    /* Enable LPUART Rx Threshold and Rx time-out wake-up function */
    LPUART0->WKCTL |= LPUART_WKCTL_WKRFRTEN_Msk | LPUART_WKCTL_WKTOUTEN_Msk;

    /* Set Rx FIFO interrupt trigger level */
    LPUART0->FIFO = (LPUART0->FIFO & (~LPUART_FIFO_RFITL_Msk)) | LPUART_FIFO_RFITL_4BYTES;

    /* Enable LPUART Rx time-out function */
    LPUART_SetTimeoutCnt(LPUART0, 40);

    printf("System enter to Power-down mode NPD%d.\n", (int)(CLK->PMUCTL & CLK_PMUCTL_PDMSEL_Msk));
    printf("Send data with baud rate 9600bps to LPUART0 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART RS485 address match (AAD mode) function                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_RS485WakeUp(void)
{
    /* Wait data transmission is finished and select LPUART clock source as LXT */
    CLK_SetModuleClock(LPUART0_MODULE, LPSCC_CLKSEL0_LPUART0SEL_LXT, LPSCC_CLKDIV0_LPUART0(1));

    /* Keep LPUART clock for LXT work in power down mode */
    LPSCC->CLKKEEP0 |= LPSCC_CLKKEEP0_LPUART0KEEP_Msk;

    /* Set LPUART baud rate and baud rate compensation */
    LPUART_Open(LPUART0, 9600);
    LPUART0->BRCOMP = 0xA5;

    /* RS485 address match (AAD mode) setting */
    LPUART_SelectRS485Mode(LPUART0, LPUART_ALTCTL_RS485AAD_Msk, RS485_ADDRESS);

    /* Enable parity source selection function */
    LPUART0->LINE |= (LPUART_LINE_PSS_Msk | LPUART_LINE_PBE_Msk);

    /* Enable LPUART RS485 address match, Rx Threshold and Rx time-out wake-up function */
    LPUART0->WKCTL |= LPUART_WKCTL_WKRFRTEN_Msk | LPUART_WKCTL_WKRS485EN_Msk | LPUART_WKCTL_WKTOUTEN_Msk;

    /* Enable LPUART Rx time-out function */
    LPUART_SetTimeoutCnt(LPUART0, 40);

    printf("System enter to Power-down mode NPD%d.\n", (int)(CLK->PMUCTL & CLK_PMUCTL_PDMSEL_Msk));
    printf("Send RS485 address byte 0x%X with baud rate 9600bps to LPUART0 to wake-up system.\n\n", RS485_ADDRESS);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  LPUART Power-down and wake-up test                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("| [3] Rx threshold and time-out wake-up test                |\n");
    printf("| [4] RS485 wake-up test                                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                           - [Others] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~4): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Lock protected registers */
    SYS_LockReg();

    /* Enable LPUART wake-up and receive data available interrupt */
    NVIC_EnableIRQ(LPUART0_IRQn);
    LPUART_EnableInt(LPUART0, LPUART_INTEN_WKIEN_Msk | LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RXTOIEN_Msk);

    LPUART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);
    switch(u32Item)
    {
    case '1':
        LPUART_CTSWakeUp();
        break;
    case '2':
        LPUART_DataWakeUp();
        break;
    case '3':
        LPUART_RxThresholdWakeUp();
        break;
    case '4':
        LPUART_RS485WakeUp();
        break;
    default:
        return;
    }

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    printf("Enter any key to end test.\n\n");
    getchar();

    /* Disable LPUART wake-up function */
    LPUART0->WKCTL = 0;

    /* Disable LPUART Interrupt */
    LPUART_DisableInt(LPUART0, LPUART_INTEN_WKIEN_Msk | LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RXTOIEN_Msk);

}
