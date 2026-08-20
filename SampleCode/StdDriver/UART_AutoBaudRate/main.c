/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to use auto baud rate detection function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define UART_TEST_PORT   UART1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void AutoBaudRate_Test(void);
void AutoBaudRate_TxTest(void);
void AutoBaudRate_RxTest(void);
uint32_t GetUartBaudrate(UART_T *uart);

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

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HIRC and UART module clock divider as 1*/
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL4_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

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
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for UART1 TXD and RXD */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA2MFP_Msk | SYS_GPA_MFP0_PA3MFP_Msk)) |    \
                    (SYS_GPA_MFP0_PA2MFP_UART1_RXD | SYS_GPA_MFP0_PA3MFP_UART1_TXD);

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
/* Init UART1                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
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
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/


    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART auto baud rate sample function */
    AutoBaudRate_Test();

    printf("\nUART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_Test(void)
{
    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PA.3) <====> UART1_RXD(PA.2)--|Slave| |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send input pattern    |\n");
    printf("|    0x1 with different baud rate. It can check if Slave    |\n");
    printf("|    calculates correct baud rate.                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    u32Item = getchar();

    if(u32Item == '0')
        AutoBaudRate_TxTest();
    else
        AutoBaudRate_RxTest();

}
/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_TxTest(void)
{
    uint32_t u32Item;

    do
    {

        printf("\n");
        printf("+-----------------------------------------------------------+\n");
        printf("|     Auto Baud Rate Function Test (Master)                 |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| [1] baud rate 38400 bps                                   |\n");
        printf("| [2] baud rate 57600 bps                                   |\n");
        printf("| [3] baud rate 115200 bps                                  |\n");
        printf("|                                                           |\n");
        printf("| Select baud rate and master will send 0x1 to slave ...    |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| Quit                                              - [ESC] |\n");
        printf("+-----------------------------------------------------------+\n\n");
        u32Item = getchar();
        printf("%c\n", u32Item);

        /* Set different baud rate */
        switch(u32Item)
        {
        case '1':
            UART_TEST_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 38400);
            break;
        case '2':
            UART_TEST_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 57600);
            break;
        default:
            UART_TEST_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
            break;
        }

        /* Send input pattern 0x1 for auto baud rate detection bit length is 1-bit */
        UART_WRITE(UART_TEST_PORT, 0x1);

    }
    while(u32Item != 27);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Get UART Baud Rate Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetUartBaudrate(UART_T *uart)
{
    uint8_t u8UartClkSrcSel=0, u8UartClkDivNum=0;
    uint32_t u32ClkTbl[] = {__HXT, 0ul, __LXT, __HIRC};
    uint32_t u32Baud_Div;

    /* Get UART clock source selection and UART clock divider number */
    switch((uint32_t)uart)
    {
    case UART0_BASE:
        u8UartClkSrcSel = (CLK->CLKSEL4 & CLK_CLKSEL4_UART0SEL_Msk) >> CLK_CLKSEL4_UART0SEL_Pos;
        u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;
        break;
    case UART1_BASE:
        u8UartClkSrcSel = (CLK->CLKSEL4 & CLK_CLKSEL4_UART1SEL_Msk) >> CLK_CLKSEL4_UART1SEL_Pos;
        u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;
        break;
    case UART2_BASE:
        u8UartClkSrcSel = (CLK->CLKSEL4 & CLK_CLKSEL4_UART2SEL_Msk) >> CLK_CLKSEL4_UART2SEL_Pos;
        u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART2DIV_Msk) >> CLK_CLKDIV4_UART2DIV_Pos;
        break;
    case UART3_BASE:
        u8UartClkSrcSel = (CLK->CLKSEL4 & CLK_CLKSEL4_UART3SEL_Msk) >> CLK_CLKSEL4_UART3SEL_Pos;
        u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART3DIV_Msk) >> CLK_CLKDIV4_UART3DIV_Pos;
        break;
    case UART4_BASE:
        u8UartClkSrcSel = (CLK->CLKSEL4 & CLK_CLKSEL4_UART4SEL_Msk) >> CLK_CLKSEL4_UART4SEL_Pos;
        u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART4DIV_Msk) >> CLK_CLKDIV4_UART4DIV_Pos;
        break;
    case UART5_BASE:
        u8UartClkSrcSel = (CLK->CLKSEL4 & CLK_CLKSEL4_UART5SEL_Msk) >> CLK_CLKSEL4_UART5SEL_Pos;
        u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART5DIV_Msk) >> CLK_CLKDIV4_UART5DIV_Pos;
        break;
    case UART6_BASE:
        u8UartClkSrcSel = (CLK->CLKSEL4 & CLK_CLKSEL4_UART6SEL_Msk) >> CLK_CLKSEL4_UART6SEL_Pos;
        u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART6DIV_Msk) >> CLK_CLKDIV4_UART6DIV_Pos;
        break;
    case UART7_BASE:
        u8UartClkSrcSel = (CLK->CLKSEL4 & CLK_CLKSEL4_UART7SEL_Msk) >> CLK_CLKSEL4_UART7SEL_Pos;
        u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART7DIV_Msk) >> CLK_CLKDIV4_UART7DIV_Pos;
        break;
    default:
        return 0;
    }

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if (u8UartClkSrcSel == 1ul)
    {
        u32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLClockFreq();
    }

    /* Get UART baud rate divider */
    u32Baud_Div = (uart->BAUD & UART_BAUD_BRD_Msk) >> UART_BAUD_BRD_Pos;

    /* Calculate UART baud rate if baud rate is set in MODE 0 */
    if ((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE0)
        return ((u32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1ul) / (u32Baud_Div + 2ul)) >> 4;

    /* Calculate UART baud rate if baud rate is set in MODE 2 */
    else if ((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE2)
        return ((u32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1ul) / (u32Baud_Div + 2ul));

    /* Calculate UART baud rate if baud rate is set in MODE 1 */
    else if ((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_BAUDM1_Msk)
        return ((u32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1ul) / (u32Baud_Div + 2ul)) / (((uart->BAUD & UART_BAUD_EDIVM1_Msk) >> UART_BAUD_EDIVM1_Pos) + 1ul);

    /* Unsupported baud rate setting */
    else
        return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Rx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_RxTest(void)
{
    /* Enable auto baud rate detect function */
    UART_TEST_PORT->ALTCTL |= UART_ALTCTL_ABRDEN_Msk;

    printf("\nreceiving input pattern... ");

    /* Wait until auto baud rate detect finished or time-out */
    while((UART_TEST_PORT->ALTCTL & UART_ALTCTL_ABRIF_Msk) == 0);

    if(UART_TEST_PORT->FIFOSTS & UART_FIFOSTS_ABRDIF_Msk)
    {
        /* Clear auto baud rate detect finished flag */
        UART_TEST_PORT->FIFOSTS = UART_FIFOSTS_ABRDIF_Msk;
        printf("Baud rate is %dbps.\n", GetUartBaudrate(UART_TEST_PORT));
    }
    else if(UART_TEST_PORT->FIFOSTS & UART_FIFOSTS_ABRDTOIF_Msk)
    {
        /* Clear auto baud rate detect time-out flag */
        UART_TEST_PORT->FIFOSTS = UART_FIFOSTS_ABRDTOIF_Msk;
        printf("Time-out!\n");
    }

}




/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
