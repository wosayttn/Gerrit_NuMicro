/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive data using Interrupt control.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ


#define BUFSIZE 1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8TxData[BUFSIZE] = {0};
volatile uint8_t g_au8RecData[BUFSIZE]  = {0};
volatile uint32_t g_u32RecLen  =  0;
volatile uint32_t g_u32TxLen  =  0;
volatile int32_t  g_i32RecOK  = FALSE;
volatile int32_t  g_i32TxStart  = FALSE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void UART_TEST_HANDLE(void);
void UART1_IRQHandler(void);
int32_t main(void);
void UART_FunctionTest(void);
void ClearRxFIFO(UART_T *uart);
uint8_t CheckPattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length);
void BuildSrcPattern(uint32_t u32Addr, uint8_t u8Type, uint32_t u32Length);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

#if (CLK_SOURCE == CLK_HIRC )
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));


#else

    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Select IP clock source */
    /* Select UART0 clock source is HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));


#endif


    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for UART1 TXD and RXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_UART1_RXD | SYS_GPA_MFPL_PA3MFP_UART1_TXD);

    /* Set PB multi-function pins for UART1 TXD and RXD */
    //    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    //    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);


    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                                 Init UART0                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{

    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                                 Init UART1                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{

    UART_Open(UART1, 115200);
}

/**
 *    @brief        Clear the UART Rx FIFO buffer function
 *
 *    @param[in]    uart    The pointer of the specified UART module.
 *
 *    @return       None
 *
 *    @details      The function is used to clear UART Rx FIFO buffer data.
 */
void ClearRxFIFO(UART_T *uart)
{
    uart->FIFO |= UART_FIFO_RXRST_Msk;

    while (uart->FIFO & UART_FIFO_RXRST_Msk) {};
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

    /* Init UART1 */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-----------------------------------+\n");
    printf("|  UART transmission function test  |\n");
    printf("+-----------------------------------+\n");

    UART_FunctionTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE(void)
{
    volatile uint32_t u32IntSts = UART1->INTSTS;;


    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while (!UART_GET_RX_EMPTY(UART1))
        {
            /* Get the character from UART Buffer */
            g_au8RecData[g_u32RecLen] = UART_READ(UART1);

            if (g_u32RecLen == (BUFSIZE - 1))
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    if (u32IntSts &  UART_INTSTS_THREINT_Msk)
    {

        if (g_i32TxStart == TRUE)
        {
            while (UART_IS_TX_FULL(UART1)) {}; /* Wait Tx is not full to transmit data */

            if (g_u32TxLen == BUFSIZE)
            {
                g_i32TxStart = FALSE;
                g_u32TxLen = 0;
                UART_DISABLE_INT(UART1, (UART_INTEN_THREIEN_Msk));
            }
            else
            {

                UART_Write(UART1, (uint8_t *)&g_au8TxData[g_u32TxLen], 1);
                g_u32TxLen++;
            }
        }
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest(void)
{
    uint8_t u8Item;

    printf("+-----------------------------------------------------------+\n");
    printf("|                      Pin Configure                        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ________                                      _______    |\n");
    printf("| |        |                                    |       |   |\n");
    printf("| | Master |---TXD1    <============>    RXD1---| Slave |   |\n");
    printf("| |________|                                    |_______|   |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n\n");



    printf("+-----------------------------------------------------------+\n");
    printf("|           Transmission  Function Test                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave.Slave will check if received data is correct  |\n");
    printf("|    after getting 1k bytes data.                           |\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n\n");
    u8Item = getchar();
    g_u32RecLen  =  0;
    g_u32TxLen  =  0;
    g_i32RecOK  = FALSE;
    g_i32TxStart  = FALSE;

    BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE);

    if (u8Item == '0')
    {
        /* Enable THRE Interrupt  */
        UART_ENABLE_INT(UART1, UART_INTEN_THREIEN_Msk);
        /* Using the Tx Interrupt to Send data */
        g_i32TxStart = TRUE;
        /* Enable UART IRQ Interrupt  */
        NVIC_EnableIRQ(UART1_IRQn);

        while (g_i32TxStart != FALSE) {};

        NVIC_DisableIRQ(UART1_IRQn);

        printf("\n Transmit Done\n");
    }
    else
    {
        /* Clear Rx FIFO register */
        ClearRxFIFO(UART1);
        /* Enable RDA\RLS\RTO Interrupt  */
        UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
        /* Set RX Trigger Level = 8 */
        UART1->FIFO = (UART1->FIFO & ~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_8BYTES;

        /* Set Timeout time 0x3E bit-time */
        UART_SetTimeoutCnt(UART1, 0x3E);

        NVIC_EnableIRQ(UART1_IRQn);

        printf("Starting to receive %d bytes data...\n", BUFSIZE);

        while (g_i32RecOK != TRUE) {}

        if (CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE))
        {
            printf("\n Receive OK & Check OK\n");
        }
        else
        {
            printf("\n Receive Fail or Check Error\n");
        }

        NVIC_DisableIRQ(UART1_IRQn);
        UART_DISABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    }



}

/*---------------------------------------------------------------------------------------------------------*/
/*                              Bulid Source Pattern function                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint8_t u8Type, uint32_t u32Length)
{
    uint32_t u32Index = 0, u32Pattern = 0;
    uint8_t *pu8Addr;
    pu8Addr = (uint8_t *)u32Addr;

    if (u8Type == 0)      u32Pattern = 0x1f;
    else if (u8Type == 1) u32Pattern = 0x3f;
    else if (u8Type == 2) u32Pattern = 0x7f;
    else if (u8Type == 3) u32Pattern = 0xff;
    else  u32Pattern = 0xff;

    for (u32Index = 0; u32Index < u32Length ; u32Index++)
        pu8Addr[u32Index] = (u32Index & u32Pattern);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                    Verify that the received data is correct                                             */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CheckPattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length)
{
    uint32_t u32Index = 0;
    uint8_t u8Result = 1;
    uint8_t *pu8Addr0;
    uint8_t *pu8Addr1;
    pu8Addr0 = (uint8_t *)u32Addr0;
    pu8Addr1 = (uint8_t *)u32Addr1;

    for (u32Index = 0; u32Index < u32Length ; u32Index++)
    {
        if (pu8Addr0[u32Index] != pu8Addr1[u32Index])
        {
            printf("Data Error Index=%d,tx =%d,rx=%d\n", u32Index, pu8Addr0[u32Index], pu8Addr1[u32Index]) ;
            u8Result = 0;
        }
    }

    return u8Result;
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
