/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive data using auto flow control.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK   FREQ_72MHZ

#define RXBUFSIZE 256

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t g_i32pointer = 0;
uint8_t g_u8RecData[RXBUFSIZE] = {0};

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest(void);
void AutoFlow_FunctionTxTest(void);
void AutoFlow_FunctionRxTest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select Low Power UART0 clock source is HIRC and Low Power UART module clock divider as 1*/
    CLK_SetModuleClock(LPUART0_MODULE, LPSCC_CLKSEL0_LPUART0SEL_HIRC, LPSCC_CLKDIV0_LPUART0(1));

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable Low Power UART0 peripheral clock */
    CLK_EnableModuleClock(LPUART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

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

    /* Lock protected registers */
    SYS_LockReg();

}
/*---------------------------------------------------------------------------------------------------------*/
/* Init UART0                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init LPUART0                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART0_Init(void)
{
    /* Reset Low Power UART0 */
    SYS_ResetModule(LPUART0_RST);

    /* Configure Low Power UART0 and set Low Power UART0 Baudrate */
    LPUART_Open(LPUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init LPUART0 */
    LPUART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------+\n");
    printf("|  Auto-Flow function test |\n");
    printf("+--------------------------+\n");

    /* LPUART auto flow sample function */
    AutoFlow_FunctionTest();

    printf("\nLPUART Sample Program End\n");

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest(void)
{
    uint8_t u8Item;

    printf("\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  ______                                                _____  |\n");
    printf("| |      |                                              |     | |\n");
    printf("| |Master|--LPUART0_TXD(PA.1)  <==> LPUART0_RXD(PA.0) --|Slave| |\n");
    printf("| |      |--LPUART0_nCTS(PA.5) <==> LPUART0_nRTS(PA.4)--|     | |\n");
    printf("| |______|                                              |_____| |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n");

    printf("\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test                                  |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  Description :                                                |\n");
    printf("|    The sample code needs two boards. One is Master and        |\n");
    printf("|    the other is slave. Master will send 1k bytes data         |\n");
    printf("|    to slave. Slave will check if received data is correct     |\n");
    printf("|  Please select Master or Slave test                           |\n");
    printf("|  [0] Master    [1] Slave                                      |\n");
    printf("+---------------------------------------------------------------+\n");
    u8Item = getchar();

    if(u8Item == '0')
        AutoFlow_FunctionTxTest();
    else
        AutoFlow_FunctionRxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Master)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTxTest(void)
{
    uint32_t u32i;

    /* Enable RTS and CTS autoflow control */
    LPUART_EnableFlowCtrl(LPUART0);

    /* Send 1k bytes data */
    for(u32i = 0; u32i < RXBUFSIZE; u32i++)
    {
        /* Send 1 byte data */
        LPUART_WRITE(LPUART0, u32i & 0xFF);

        /* Wait if Tx FIFO is full */
        while(LPUART_IS_TX_FULL(LPUART0));
    }

    printf("\n Transmit Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Slave)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionRxTest(void)
{
    uint32_t u32i;

    /* Enable RTS and CTS autoflow control */
    LPUART_EnableFlowCtrl(LPUART0);

    /* Set RTS Trigger Level as 8 bytes */
    LPUART0->FIFO = (LPUART0->FIFO & (~LPUART_FIFO_RTSTRGLV_Msk)) | LPUART_FIFO_RTSTRGLV_8BYTES;

    /* Set RX Trigger Level as 8 bytes */
    LPUART0->FIFO = (LPUART0->FIFO & (~LPUART_FIFO_RFITL_Msk)) | LPUART_FIFO_RFITL_8BYTES;

    /* Set Timeout time 0x3E bit-time and time-out counter enable */
    LPUART_SetTimeoutCnt(LPUART0, 0x3E);

    /* Enable RDA and RTO Interrupt */
    NVIC_EnableIRQ(LPUART0_IRQn);
    LPUART_EnableInt(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RLSIEN_Msk | LPUART_INTEN_RXTOIEN_Msk));

    printf("\n Starting to receive data...\n");

    /* Wait for receive 1k bytes data */
    while(g_i32pointer < RXBUFSIZE);

    /* Compare Data */
    for(u32i = 0; u32i < RXBUFSIZE; u32i++)
    {
        if(g_u8RecData[u32i] != (u32i & 0xFF))
        {
            printf("Compare Data Failed\n");
            while(1);
        }
    }
    printf("\n Receive OK & Check OK\n");

    /* Disable RDA and RTO Interrupt */
    LPUART_DisableInt(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RLSIEN_Msk | LPUART_INTEN_RXTOIEN_Msk));

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle LPUART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART0_IRQHandler(void)
{
    uint8_t u8InChar = 0xFF;

    /* Rx Ready or Time-out INT */
    if(LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_RDAINT_Msk | LPUART_INTSTS_RXTOINT_Msk))
    {
        /* Read data until RX FIFO is empty */
        while(LPUART_GET_RX_EMPTY(LPUART0) == 0)
        {
            u8InChar = LPUART_READ(LPUART0);
            g_u8RecData[g_i32pointer++] = u8InChar;
        }
    }

    if(LPUART0->FIFOSTS & (LPUART_FIFOSTS_BIF_Msk | LPUART_FIFOSTS_FEF_Msk | LPUART_FIFOSTS_PEF_Msk | LPUART_FIFOSTS_RXOVIF_Msk))
    {
        LPUART_ClearIntFlag(LPUART0, (LPUART_INTSTS_RLSINT_Msk| LPUART_INTSTS_BUFERRINT_Msk));
    }
}



/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
