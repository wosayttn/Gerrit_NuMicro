/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Two Single-Wire data transfer test.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_96MHZ
#define BUFSIZE   128

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[BUFSIZE] = {0};
//uint8_t g_u8TxData [BUFSIZE] = {0};
//volatile uint32_t g_u32RecLen  =  0;
//volatile int32_t  g_i32RecOK  = FALSE;
volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART1_TEST_HANDLE(void);
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);
void SingleWire_FunctionTxTest(void);
void SingleWire_FunctionRxTest(void);


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 24MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL2_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PB multi-function pins for UART1 RXD */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB2MFP_Msk) | SYS_GPB_MFPL_PB2MFP_UART1_RXD;

    /* Lock protected registers */
    SYS_LockReg();

}
/*---------------------------------------------------------------------------------------------------------*/
/*                                     Init UART0                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                     Init UART1                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init()
{

    /* Configure UART1 and set UART1 baud rate */
    UART_Open(UART1, 115200);

    /*Set PB2 IO status is Pull-up for SingleWire */
    GPIO_SetPullCtl(PB, BIT2, GPIO_PUSEL_PULL_UP);

}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* Debug port control the Single wire 1(UART1) send data to Single wire 1(UART1)                           */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for SingleWire Test */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                           SAMPLE CODE                                                   */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;

    if (UART_GET_INT_FLAG(UART0,UART_INTSTS_RDAINT_Msk))
    {
        printf("\nInput:");

        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0))
        {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);

            printf("%c ", u8InChar);

            if(u8InChar == '0')
            {
                g_bWait = FALSE;
            }

            /* Check if buffer full */
            if(g_u32comRbytes < BUFSIZE)
            {
                /* Enqueue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (BUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
            }
        }
        printf("\nTransmission Test: %c", u8InChar);
    }

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_THREINT_Msk))
    {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp)
        {
            u8InChar = g_u8RecData[g_u32comRhead];
            while(UART_IS_TX_FULL(UART1));  /* Wait Tx is not full to transmit data */
            UART_WRITE(UART1, u8InChar);
            g_u32comRhead = (g_u32comRhead == (BUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART1_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART1 Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE()
{

    if (UART_GET_INT_FLAG(UART1,UART_INTSTS_SWBEIF_Msk))
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART1, UART_INTSTS_SWBEINT_Msk);

    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest()
{
    char cmmd ;

    printf("+-------------------------------------------------------------+\n");
    printf("|                     Pin Configure                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  ______                                      ______         |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |Master|--RXD1(PB.2)   <====>   RXD1(PB.2)--|Slave |        |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |______|                                    |______|        |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");

    /*
        Semi-host is set to debug port for message.
        The SingleWire sample code needs two module board to execute.
        Set the master board is SingleWire TX Mode and the other is SingleWire Rx mode.
        Inputting char on terminal will be sent to the UART0 of master.
        After the master receiving, the inputting char will send to UART0 again.
        At the same time, it also sends to UART0 TX pin by SingleWire mode.
        Slave will print received char after UART0 send out.
    */


    printf("\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     SingleWire Function Test                                |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and      |\n");
    printf("|    the other is slave.  Master will send data based on      |\n");
    printf("|    terminal input and Slave will printf received data on    |\n");
    printf("|    terminal screen.                                         |\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n\n");
    cmmd = getchar();

    if (cmmd == '0')
        SingleWire_FunctionTxTest();
    else
        SingleWire_FunctionRxTest();

    /* Disable UART1 RDA/Time-out interrupt */
    UART_DisableInt(UART1, UART_INTEN_SWBEIEN_Msk);

    printf("\nUART Sample Demo End.\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*                             SingleWire Function Transmit Test                                           */
/*---------------------------------------------------------------------------------------------------------*/
void SingleWire_FunctionTxTest(void)
{
    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     SingleWire Function Tx Mode Test                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Input char by UART0 terminal.                         |\n");
    printf("| 2). UART0 will send a char to UART1 according to step 1.  |\n");
    printf("| 3). Return step 1. (Press '0' to exit)                    |\n");
    printf("+-----------------------------------------------------------+\n\n");

    printf("\nSingleWire Sample Code Start. \n");

    /* Select Single Rx mode */
    UART_SelectSingleWireMode(UART1);

    /* Enable UART RDA and THRE interrupt */
    NVIC_EnableIRQ(UART0_IRQn);
    UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk));
    while(g_bWait);

    /* Disable UART RDA and THRE interrupt */
    UART_DisableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk));
    g_bWait = TRUE;

    /* terminate slave */
    UART_WRITE(UART1, '0');
    printf("\nUART TX Sample Demo End.\n");

}
/*---------------------------------------------------------------------------------------------------------*/
/*                             SingleWire Function Receive Test                                            */
/*---------------------------------------------------------------------------------------------------------*/
void SingleWire_FunctionRxTest(void)
{
    uint8_t u8InChar = 0xFF;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     SingleWire Function Rx Mode Test                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Polling RDA_Flag to check data input though UART      |\n");
    printf("| 2). If received data is '0', the program will exit.       |\n");
    printf("|     Otherwise, print received data on terminal            |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* Select Single Rx mode */
    UART_SelectSingleWireMode(UART1);

    /* Enable UART1 RDA/Time-out/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, UART_INTEN_SWBEIEN_Msk);

    /* Reset Rx FIFO */
    UART1->FIFO |= UART_FIFO_RXRST_Msk;
    while(UART1->FIFO & UART_FIFO_RXRST_Msk);

    printf("Waiting...\n");

    /* Use polling method to wait master data */
    do
    {
        if (UART_IS_RX_READY(UART1))
        {
            u8InChar = UART_READ(UART1);
            printf("Receive : %c \n", u8InChar);
        }
    }
    while (u8InChar != '0');

    UART_DisableInt(UART1, UART_INTEN_SWBEIEN_Msk);
    printf("\nUART RX Sample Demo End.\n");

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/


