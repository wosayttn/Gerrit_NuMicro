/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 20/08/06 5:45p $
 * @brief
 *           Transmit and receive data from PC terminal through RS232 interface.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define RXBUFSIZE   256

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void LPUART_TEST_HANDLE(void);
void LPUART_FunctionTest(void);


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

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

    /* Enable LPUART module clock */
    CLK_EnableModuleClock(LPUART0_MODULE);

    /* Select Low Power UART0 clock source is HIRC and Low Power UART module clock divider as 1*/
    CLK_SetModuleClock(LPUART0_MODULE, LPSCC_CLKSEL0_LPUART0SEL_HIRC, LPSCC_CLKDIV0_LPUART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PA multi-function pins for Low Power UART0 TXD and RXD */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk)) |    \
                    (SYS_GPA_MFP0_PA0MFP_LPUART0_RXD | SYS_GPA_MFP0_PA1MFP_LPUART0_TXD);
}

void LPUART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LPUART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset LPUART0 */
    SYS_ResetModule(LPUART0_RST);

    /* Configure LPUART0 and set LPUART0 baud rate */
    LPUART_Open(LPUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART Test Sample                                                                                        */
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

    /* Init LPUART0 for printf and test */
    LPUART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nLPUART Sample Program\n");

    /* LPUART sample function */
    LPUART_FunctionTest();

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle LPUART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART0_IRQHandler(void)
{
    LPUART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;

    if (LPUART_GET_INT_FLAG(LPUART0,LPUART_INTSTS_RDAINT_Msk))
    {
        printf("\nInput:");

        /* Get all the input characters */
        while(LPUART_IS_RX_READY(LPUART0))
        {
            /* Get the character from LPUART Buffer */
            u8InChar = LPUART_READ(LPUART0);

            printf("%c ", u8InChar);

            if(u8InChar == '0')
            {
                g_bWait = FALSE;
            }

            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
            }
        }
        printf("\nTransmission Test:");
    }

    if(LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_THREINT_Msk))
    {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp)
        {
            u8InChar = g_u8RecData[g_u32comRhead];
            while(LPUART_IS_TX_FULL(LPUART0));  /* Wait Tx is not full to transmit data */
            LPUART_WRITE(LPUART0, u8InChar);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
    }

    if(LPUART0->FIFOSTS & (LPUART_FIFOSTS_BIF_Msk | LPUART_FIFOSTS_FEF_Msk | LPUART_FIFOSTS_PEF_Msk | LPUART_FIFOSTS_RXOVIF_Msk))
    {
        LPUART_ClearIntFlag(LPUART0, (LPUART_INTSTS_RLSINT_Msk| LPUART_INTSTS_BUFERRINT_Msk));
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_FunctionTest()
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  LPUART Function Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect LPUART0 and PC.
        LPUART0 is set to debug port. LPUART0 is enable RDA and RLS interrupt.
        When inputting char to terminal screen, RDA interrupt will happen and
        LPUART0 will print the received char on screen.
    */

    /* Enable LPUART RDA and THRE interrupt */
    NVIC_EnableIRQ(LPUART0_IRQn);
    LPUART_EnableInt(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_THREIEN_Msk));
    while(g_bWait);

    /* Disable LPUART RDA and THRE interrupt */
    LPUART_DisableInt(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_THREIEN_Msk));
    g_bWait = TRUE;
    printf("\nLPUART Sample Demo End.\n");

}
