/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Transmit and receive data from Any MUC through RS232 interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_48MHZ
#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/*                                  Global variables                                                       */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8RecData[RXBUFSIZE]  = {0};
volatile uint32_t g_u32ComRtail  = 0;
volatile uint32_t g_u32ShowMsg  = 0;


/*---------------------------------------------------------------------------------------------------------*/
/*                               Define functions prototype                                                */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void UART1_Init(void);
void UART1_IRQHandler(void);
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/*                                  Init System Clock                                                      */
/*---------------------------------------------------------------------------------------------------------*/

void SYS_Init(void)
{

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

    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

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

    /* Set PB multi-function pins for UART1 RXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD);

    /*Set PB2 IO status is Pull-up*/
    GPIO_SetPullCtl(PB, BIT2, GPIO_PUSEL_PULL_UP);

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                     Init Single Wire(UART1)                                             */
/*---------------------------------------------------------------------------------------------------------*/

void UART1_Init(void)
{

    /* Configure Single Wire(UART1) and set Single Wire(UART1) baud rate */
    UART_Open(UART1, 115200);
    UART_SelectSingleWireMode(UART1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                   Main Function                                                         */
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
    /* Init UART1 for Single Wire Test*/
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
/*                         ISR to handle UART Channel 1 interrupt event                                    */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                  UART Callback function                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE(void)
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART1->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART1);
            /* Enqueue the character */
            g_au8RecData[g_u32ComRtail] = u8InChar;
            g_u32ComRtail = (g_u32ComRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32ComRtail + 1);
            printf("\nReceive Data = %c", g_au8RecData[g_u32ShowMsg]);
            g_u32ShowMsg = (g_u32ShowMsg == (RXBUFSIZE - 1)) ? 0 : (g_u32ShowMsg + 1);
        }
    }

    if (u32IntSts & UART_INTSTS_SWBEINT_Msk)
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART1, UART_INTSTS_SWBEINT_Msk);
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest(void)
{
    char chCmd ;

    printf("+-----------------------------------------------------------+\n");
    printf("|            UART Single Wire Function Test                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    The user must connect the UART1 RX pin(PB2) to the     |\n");
    printf("|    terminal or MCU Rx pin that supports single wire mode  |\n");
    printf("|    Please enter any to start    (Press '0' to exit)       |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART4 and PC.
        UART4 is set to debug port. UART1 is enable RDA and RLS interrupt.
        The user can use UART4 to control the transmission or reception of UART1(Single Wire mode)
        When inputting char to terminal screen, RDA interrupt will happen and
        UART1 will print the received char on screen.
    */

    /* Enable UART1 RDA/Time-out/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));

    do
    {
        chCmd = getchar();
        printf("\nTransmission Data: %c", chCmd);
        UART_Write(UART1, (uint8_t *)&chCmd, 1);
    } while (chCmd != '0');

    /* Disable UART1 RDA/Time-out interrupt */
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));

    printf("\nUART Sample Demo End.\n");

}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


