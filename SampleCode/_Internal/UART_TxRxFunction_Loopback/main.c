/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Transmit and receive data from UART port in Loopback mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

#define BUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8RecData[BUFSIZE] = {0};
uint8_t g_au8TxData[BUFSIZE] = {0};
volatile uint32_t g_u32RecOffset =  0;
volatile uint32_t g_u32RecLen  =  0;
volatile int32_t  g_i32RecOK  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void UART1_IRQHandler(void);
void UART1_TEST_HANDLE(void);
void UART_FunctionTest(void);
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
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

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

    //    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    //    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART0                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART1                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{
    /* Configure UART1 and set UART1 baud rate */
    UART_Open(UART1, 115200);
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
    /* Init UART1 for Data Transmission Test*/
    UART1_Init();


    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while (1);

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
void UART1_TEST_HANDLE(void)
{
    uint32_t u32IntSts = UART1->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while (!UART_GET_RX_EMPTY(UART1))
        {
            /* Get the character from UART Buffer */
            g_au8RecData[g_u32RecLen] = UART_READ(UART1);

            if (g_u32RecLen == (BUFSIZE - 1 - g_u32RecOffset))
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
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest(void)
{
    char chCmd ;
    printf("+-----------------------------------------------------------+\n");
    printf("|            UART Transmission Feature Test                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo the UART1 Transmission       |\n");
    printf("|    Loopback control.                                      |\n");
    printf("|    The user must connect the UART1 TX pin(PA3) to         |\n");
    printf("|    UART1_Rx Pin(PA2).                                     |\n");
    printf("|    Please enter any to start                              |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART0 and PC.
        UART0 is set to debug port. UART1 is enable RDA and RLS interrupt.
        When UART1 Tx Send data to Rx, RDA interrupt occurs and receive data
          to verify the correctness.UART0 will print the data comparison results on screen.
    */

    /* Enable UART1 RDA/Time-out interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    do
    {
        printf("+-------------------------------------------------------------------+\n");
        printf("|                  UART LoopBack Test Item                          |\n");
        printf("+-------------------------------------------------------------------+\n");
        printf("|  (1)Loopback UART1(PA3)send data to UART1(PA2). RFITL: 1 byte     |\n");
        printf("|  (2)Loopback UART1(PA3)send data to UART1(PA2). RFITL: 4 bytes    |\n");
        printf("|  (3)Loopback UART1(PA3)send data to UART1(PA2). RFITL: 8 bytes    |\n");
        printf("|  (4)Loopback UART1(PA3)send data to UART1(PA2). RFITL:14 bytes    |\n");
        printf("|  (E)Exit                                                          |\n");
        printf("+-------------------------------------------------------------------+\n");

        chCmd = getchar();

        switch (chCmd)
        {
            case '1':
            {
                printf("(1)UART1(Tx) --> UART1(Rx) Test:");
                g_u32RecOffset =  0;
                g_i32RecOK = FALSE;
                UART1->FIFO = (UART1->FIFO & ~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_1BYTE;
                BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE);
                UART_Write(UART1, g_au8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {}

                CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");
                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_au8TxData, 0, BUFSIZE);
                memset((uint8_t *)g_au8RecData, 0, BUFSIZE);
            }
            break;

            case '2':
            {
                printf("(2)UART1(Tx) --> UART1(Rx) Test :");
                g_u32RecOffset =  0;
                g_i32RecOK = FALSE;
                UART1->FIFO = (UART1->FIFO & ~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_4BYTES;
                BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE);
                UART_Write(UART1, g_au8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {}

                CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");
                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_au8TxData, 0, BUFSIZE);
                memset((uint8_t *)g_au8RecData, 0, BUFSIZE);
            }
            break;

            case '3':
            {
                printf("(3)UART1(Tx) --> UART1(Rx) Test :");
                g_u32RecOffset =  0;
                g_i32RecOK = FALSE;
                UART1->FIFO = (UART1->FIFO & ~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_8BYTES;
                BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE);
                UART_Write(UART1, g_au8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {}

                CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");
                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_au8TxData, 0, BUFSIZE);
                memset((uint8_t *)g_au8RecData, 0, BUFSIZE);
            }
            break;

            case '4':
            {
                printf("(4)UART1(Tx) --> UART1(Rx) Test :");
                g_u32RecOffset =  2;
                g_i32RecOK = FALSE;
                UART1->FIFO = (UART1->FIFO & ~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_14BYTES;
                BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE - g_u32RecOffset);
                UART_Write(UART1, g_au8TxData, BUFSIZE - g_u32RecOffset);

                while (g_i32RecOK != TRUE) {}

                CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");
                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_au8TxData, 0, BUFSIZE);
                memset((uint8_t *)g_au8RecData, 0, BUFSIZE);
            }
            break;

            default:
                break;
        }

    } while ((chCmd != 'E') && (chCmd != 'e'));


    /* Disable UART1 RDA/Time-out interrupt */
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    printf("\nUART Sample Demo End.\n");

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
    uint8_t  u8Result = 1;
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


