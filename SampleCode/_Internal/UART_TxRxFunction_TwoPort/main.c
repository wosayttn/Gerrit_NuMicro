/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Transmit and receive data from  UART port in Loopback mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

#define PLL_CLOCK   FREQ_48MHZ
#define BUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8RecData[BUFSIZE] = {0};
uint8_t g_au8TxData[BUFSIZE] = {0};
volatile uint32_t g_u32RecLen  =  0;
volatile int32_t  g_i32RecOK  = FALSE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART1_TEST_HANDLE(void);
void UART2_TEST_HANDLE(void);
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

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select IP clock source */
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    /* Select UART2 clock source is HIRC */
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));

    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
    /* Enable UART2 peripheral clock */
    CLK_EnableModuleClock(UART2_MODULE);
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting*/
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for Debug UART RXD and TXD */
    UartDebugMFP();

    /* Set PB multi-function pins for UART1 TXD and RXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

    /* Set PB multi-function pins for UART2 TXD and RXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_UART2_RXD | SYS_GPB_MFPL_PB1MFP_UART2_TXD);

    /* Lock protected registers */
    SYS_LockReg();

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
/* Init UART2                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_Init(void)
{
    /* Configure UART2 and set UART2 baud rate */
    UART_Open(UART2, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

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

    /* Init Debug UART for printf */
    UartDebugInit();
    /* Init UART1 and UART2 for Data Transmission Test*/
    UART1_Init();
    UART2_Init();

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
        while (UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            g_au8RecData[g_u32RecLen] = UART_READ(UART1);

            if (g_u32RecLen == BUFSIZE - 1)
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

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 2 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_IRQHandler(void)
{
    UART2_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART2 Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_TEST_HANDLE(void)
{
    uint32_t u32IntSts = UART2->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART2))
        {
            /* Get the character from UART Buffer */
            g_au8RecData[g_u32RecLen] = UART_READ(UART2);

            if (g_u32RecLen == BUFSIZE - 1)
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
    printf("|    The sample code will demo the UART1 and UART2          |\n");
    printf("|    Transmission Loopback control.                         |\n");
    printf("|    The user must connect the UART1 TX pin(PB3) to         |\n");
    printf("|    UART2_Rx Pin(PB0) and UART2 TX pin(PB1) to             |\n");
    printf("|    UART1_Rx Pin(PB2).                                     |\n");
    printf("|    UART1(PB3)send data to UART 2(PB0).                    |\n");
    printf("|    UART2(PB1)send data to UART 1(PB2).                    |\n");
    printf("|    Please enter any to start    (Press 'E' to exit)       |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART4 and PC.
        UART0 is set to debug port. UART1 and UART2 is enable RDA and RLS interrupt.
        When UART1 Tx Send data to UART2 Rx, RDA interrupt occurs and receive data
          to verify the correctness.UART4 will print the data comparison results on screen.
    */

    /* Enable UART RDA/Time-out interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    /* Enable UART RDA/Time-out interrupt */
    NVIC_EnableIRQ(UART2_IRQn);
    UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    do
    {
        printf("+------------------------------------------------------------+\n");
        printf("|            UART Transmission Test Item                     |\n");
        printf("+------------------------------------------------------------+\n");
        printf("|    (1)UART1(PB3)send data to UART2(PB0).                   |\n");
        printf("|    (2)UART2(PB1)send data to UART1(PB2).                   |\n");
        printf("|    (E)Exit                                                 |\n");
        printf("+------------------------------------------------------------+\n");

        chCmd = getchar();

        switch (chCmd)
        {
            case '1':
            {
                printf("UART1(Tx) --> UART2(Rx) Test :");
                g_i32RecOK  = FALSE;
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
                printf("UART2(Tx) --> UART1(Rx) Test :");
                g_i32RecOK  = FALSE;
                BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE);

                UART_Write(UART2, g_au8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {};

                CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE) ? printf(" Pass\n") :   printf(" Fail\n");

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
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    /* Disable UART2 RDA/Time-out interrupt */
    UART_DisableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
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
    uint8_t u8Result = 1;
    uint8_t *pu8Addr0;
    uint8_t *pu8Addr1;
    pu8Addr0 = (uint8_t *)u32Addr0;
    pu8Addr1 = (uint8_t *)u32Addr1;

    for (u32Index = 0; u32Index < u32Length ; u32Index++)
    {
        if (pu8Addr0[u32Index] != pu8Addr1[u32Index])
        {
            printf("Data Error Idex=%d,tx =%d,rx=%d\n", u32Index, pu8Addr0[u32Index], pu8Addr1[u32Index]) ;
            u8Result = 0;
        }
    }

    return u8Result;
}


