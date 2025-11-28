/**************************************************************************//**
 * @file     main.c
 * @version  V1.0
 * $Revision: 2 $
 * $Date: 20/08/11 2:40p $
 * @brief
 *           Demonstrate SPI data transfer with PDMA.
 *           SPI0 will be configured as Master mode and connect MISO_0 pin and
 *           MOSI_0 pin together. Both TX PDMA function and RX PDMA function
 *           will be enabled.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LPSPI_MASTER_TX_DMA_CH 0
#define LPSPI_MASTER_RX_DMA_CH 1
#define LPSPI_OPENED_CH   ((1 << LPSPI_MASTER_TX_DMA_CH) | (1 << LPSPI_MASTER_RX_DMA_CH))

#define DATA_COUNT      32
#define TEST_CYCLE      10000
#define TEST_PATTERN    0x55000000
#define LPSPI_CLK_FREQ  2000000

/* Function prototype declaration */
void SYS_Init(void);
void LPSPI_Init(void);
void SpiLoopTest_WithPDMA(void);

/* Global variable declaration */
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
uint32_t g_au32MasterToSlaveTestPattern[DATA_COUNT] __attribute__ ((section(".lpSram")));
uint32_t g_au32MasterRxBuffer[DATA_COUNT] __attribute__ ((section(".lpSram")));
#else
uint32_t g_au32MasterToSlaveTestPattern[DATA_COUNT] __attribute__ ((section(".ARM.__at_0x28000000")));
uint32_t g_au32MasterRxBuffer[DATA_COUNT] __attribute__ ((section(".ARM.__at_0x28000100")));
#endif

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    LPSPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|                  LPSPI + LPPDMA Sample Code                  |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure LPSPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for loopback test:\n");
    printf("    LPSPI0_MISO(PA.1) <--> LPSPI0_MOSI(PA.0)\n\n");
    printf("Please press any key to start transmission ...\n");
    getchar();
    printf("\n");

    SpiLoopTest_WithPDMA();

    printf("\n\nExit SPI driver sample code.\n");

    /* Close SPI0 */
    LPSPI_Close(LPSPI0);
    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select PCLK1 as the clock source of SPI0 */
    CLK_SetModuleClock(LPSPI0_MODULE, LPSCC_CLKSEL0_LPSPI0SEL_HIRC, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(LPSPI0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Setup SPI0 multi-function pins */
    /* PA.3 is LPSPI0_SS,   PA.2 is LPSPI0_CLK,
       PA.1 is LPSPI0_MISO, PA.0 is LPSPI0_MOSI*/
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA3MFP_Msk |
                                       SYS_GPA_MFP0_PA2MFP_Msk |
                                       SYS_GPA_MFP0_PA1MFP_Msk |
                                       SYS_GPA_MFP0_PA0MFP_Msk)) |
                    (SYS_GPA_MFP0_PA3MFP_LPSPI0_SS |
                     SYS_GPA_MFP0_PA2MFP_LPSPI0_CLK |
                     SYS_GPA_MFP0_PA1MFP_LPSPI0_MISO |
                     SYS_GPA_MFP0_PA0MFP_LPSPI0_MOSI);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void LPSPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2 MHz */
    LPSPI_Open(LPSPI0, LPSPI_MASTER, LPSPI_MODE_0, 32, LPSPI_CLK_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    LPSPI_EnableAutoSS(LPSPI0, LPSPI_SS, LPSPI_SS_ACTIVE_LOW);
}

void SpiLoopTest_WithPDMA(void)
{
    uint32_t u32DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;
    uint32_t u32TimeOutCount;

    printf("\nLPSPI0 Loopback test with LPPDMA \n");

    /* Source data initiation */
    for(u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
    {
        g_au32MasterToSlaveTestPattern[u32DataCount] = TEST_PATTERN | (u32DataCount + 1);
    }

    /* Reset PDMA module */
    SYS_ResetModule(LPPDMA0_RST);

    /* Enable PDMA channels */
    LPPDMA_Open(LPPDMA0, LPSPI_OPENED_CH);

    /*=======================================================================
      SPI master PDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = DATA_COUNT
        Source = g_au32MasterToSlaveTestPattern
        Source Address = Incresing
        Destination = SPI0->TX
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, (uint32_t)g_au32MasterToSlaveTestPattern, LPPDMA_SAR_INC, (uint32_t)&LPSPI0->TX, LPPDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, LPPDMA_LPSPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    LPPDMA_SetBurstType(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, LPPDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    LPPDMA0->LPDSCT[LPSPI_MASTER_TX_DMA_CH].CTL |= LPPDMA_DSCT_CTL_TBINTDIS_Msk;

    /*=======================================================================
      SPI master PDMA RX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = DATA_COUNT
        Source = SPI0->RX
        Source Address = Fixed
        Destination = g_au32MasterRxBuffer
        Destination Address = Increasing
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, (uint32_t)&LPSPI0->RX, LPPDMA_SAR_FIX, (uint32_t)g_au32MasterRxBuffer, LPPDMA_DAR_INC);
    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_LPSPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    LPPDMA_SetBurstType(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    LPPDMA0->LPDSCT[LPSPI_MASTER_RX_DMA_CH].CTL |= LPPDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Enable SPI master DMA function */
    LPSPI_TRIGGER_TX_PDMA(LPSPI0);
    LPSPI_TRIGGER_RX_PDMA(LPSPI0);

    i32Err = 0;
    for(u32TestCycle = 0; u32TestCycle < TEST_CYCLE; u32TestCycle++)
    {
        if((u32TestCycle & 0x1FF) == 0)
            putchar('.');

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        while(1)
        {
            /* Get interrupt status */
            u32RegValue = LPPDMA_GET_INT_STATUS(LPPDMA0);
            /* Check the PDMA transfer done interrupt flag */
            if(u32RegValue & LPPDMA_INTSTS_TDIF_Msk)
            {
                /* Check the PDMA transfer done flags */
                if((LPPDMA_GET_TD_STS(LPPDMA0) & LPSPI_OPENED_CH) == LPSPI_OPENED_CH)
                {
                    /* Clear the PDMA transfer done flags */
                    LPPDMA_CLR_TD_FLAG(LPPDMA0, LPSPI_OPENED_CH);
                    /* Disable SPI master's PDMA transfer function */
                    LPSPI_DISABLE_TX_PDMA(LPSPI0);
                    LPSPI_DISABLE_RX_PDMA(LPSPI0);
                    /* Check the transfer data */
                    for(u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
                    {
                        if(g_au32MasterToSlaveTestPattern[u32DataCount] != g_au32MasterRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                    }

                    if(u32TestCycle >= TEST_CYCLE)
                        break;

                    /* Source data initiation */
                    for(u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
                    {
                        g_au32MasterToSlaveTestPattern[u32DataCount]++;
                    }
                    /* Re-trigger */
                    /* Master PDMA TX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    LPPDMA_SetTransferCnt(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
                    /* Set request source; set basic mode. */
                    LPPDMA_SetTransferMode(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, LPPDMA_LPSPI0_TX, FALSE, 0);

                    /* Master PDMA RX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    LPPDMA_SetTransferCnt(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
                    /* Set request source; set basic mode. */
                    LPPDMA_SetTransferMode(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_LPSPI0_RX, FALSE, 0);

                    /* Enable master's DMA transfer function */
                    LPSPI_TRIGGER_TX_PDMA(LPSPI0);
                    LPSPI_TRIGGER_RX_PDMA(LPSPI0);
                    break;
                }
            }
            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & LPPDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = LPPDMA_GET_ABORT_STS(LPPDMA0);
                /* Clear the target abort flag */
                LPPDMA_CLR_ABORT_FLAG(LPPDMA0, u32Abort);
                i32Err = 1;
                break;
            }

            if(u32TimeOutCount == 0)
            {
                printf("\nSomething is wrong, please check if pin connection is correct. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        if(i32Err)
            break;
    }

    /* Disable all PDMA channels */
    LPPDMA_Close(LPPDMA0);

    if(i32Err)
    {
        printf(" [FAIL]\n");
    }
    else
    {
        printf(" [PASS]\n");
    }

    return;
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

