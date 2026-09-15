/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate PDMA0 channel 1 get/clear timeout flag with UART1.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMA_TEST_LENGTH 100
#define PDMA_TIME 0x5555

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH];
#else
__attribute__((aligned(4))) static uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH];
__attribute__((aligned(4))) static uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH];
#endif

volatile uint32_t u32IsTxTestOver = 0;
volatile uint32_t u32IsRxTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void);
void UART_PDMATest(void);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Select UART0 clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select UART1 clock source from HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL4_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set GPA multi-function pins for UART1 TXD, RXD */
    SYS->GPA_MFP0 &= ~(SYS_GPA_MFP0_PA2MFP_Msk | SYS_GPA_MFP0_PA3MFP_Msk);
    SYS->GPA_MFP0 |= (SYS_GPA_MFP0_PA2MFP_UART1_RXD | SYS_GPA_MFP0_PA3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    UART_Open(UART0, 115200);
}

void UART1_Init(void)
{
    UART_Open(UART1, 115200);
}

void PDMA_Init(void)
{
    /* Open PDMA0 Channel */
    PDMA_Open(PDMA0, 1 << 0); // Channel 0 for UART1 TX
    PDMA_Open(PDMA0, 1 << 1); // Channel 1 for UART1 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA0, 0, PDMA_UART1_TX, 0, 0);
    PDMA_SetTransferMode(PDMA0, 1, PDMA_UART1_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0, 1, PDMA_WIDTH_8, PDMA_TEST_LENGTH + 1);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA0, 0, ((uint32_t)(&g_u8Tx_Buffer[0])), PDMA_SAR_INC, UART1_BASE, PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA0, 1, UART1_BASE, PDMA_SAR_FIX, ((uint32_t)(&g_u8Rx_Buffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, 1, PDMA_REQ_SINGLE, 0);
    //Set timeout
    PDMA_SetTimeOut(PDMA0, 1, 1, PDMA_TIME);

    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, 1, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, 1, PDMA_INT_TIMEOUT);
    NVIC_EnableIRQ(PDMA0_IRQn);
    u32IsRxTestOver = 0;
    u32IsTxTestOver = 0;
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 */
    UART1_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    UART_PDMATest();

    while(1);
}

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M2L31.s.
 */
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & 0x1)   /* abort */
    {
        printf("target abort interrupt !!\n");
        if (PDMA_GET_ABORT_STS(PDMA0) & 0x1)
        {
            u32IsTxTestOver = 2;
        }
        if (PDMA_GET_ABORT_STS(PDMA0) & 0x2)
        {
            u32IsRxTestOver = 2;
        }
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_GET_ABORT_STS(PDMA0));
    }
    else if (u32Status & 0x2)     /* done */
    {
        if ( (PDMA_GET_TD_STS(PDMA0) & (1 << 0)))
        {
            u32IsTxTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA0, (1 << 0));
        }

        if ( (PDMA_GET_TD_STS(PDMA0) & (1 << 1)))
        {
            u32IsRxTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA0, (1 << 1));
        }
    }
    else if (u32Status & 0x100)     /* channel 0 timeout */
    {
        u32IsTxTestOver = 3;
        PDMA_CLR_TMOUT_FLAG(PDMA0, 0);
    }
    else if (u32Status & 0x200)     /* channel 1 timeout */
    {
        u32IsRxTestOver = 3;
        PDMA_SetTimeOut(PDMA0, 1, 0, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA0, 1);
        PDMA_SetTimeOut(PDMA0, 1, 0, PDMA_TIME);
    }
    else
        printf("unknown interrupt !!\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART PDMA Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PDMATest(void)
{
    uint32_t i;

    printf("+-----------------------------------------------------------+\n");
    printf("|  PDMA timeout Test                                        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo PDMA timeout function.       |\n");
    printf("|    Please connect UART1_TX and UART1_RX pin.              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");

    getchar();

    /*
        Using UART1 external loop back.
        This code will send data from UART1_TX and receive data from UART1_RX.
        UART1_TX :  Total transfer length =  (PDMA_TEST_LENGTH  ) * 8 bits
        UART1_RX :  Total transfer length =  (PDMA_TEST_LENGTH+1) * 8 bits
    */

    for (i=0; i<PDMA_TEST_LENGTH; i++)
    {
        g_u8Tx_Buffer[i] = i;
        g_u8Rx_Buffer[i] = 0xff;
    }

    while(1)
    {
        PDMA_Init();

        UART_PDMA_ENABLE(UART1, UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);

        while(u32IsTxTestOver == 0);

        if (u32IsTxTestOver == 1)
            printf("UART1 TX transfer done...\n");
        else if (u32IsTxTestOver == 2)
            printf("UART1 TX transfer abort...\n");
        else if (u32IsTxTestOver == 3)
            printf("UART1 TX timeout...\n");

        while(u32IsRxTestOver == 0);

        if (u32IsRxTestOver == 1)
            printf("UART1 RX transfer done...\n");
        else if (u32IsRxTestOver == 2)
            printf("UART1 RX transfer abort...\n");
        else if (u32IsRxTestOver == 3)
        {
            printf("UART1 RX timeout...\n");
        }

        UART_PDMA_DISABLE(UART1, UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);

        printf("PDMA timeout test Pass, Please press any key to next time. \n\n");
        getchar();
    }

}

