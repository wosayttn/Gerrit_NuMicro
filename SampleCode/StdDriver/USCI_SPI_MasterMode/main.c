/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *          Configure USCI_SPI0 as Master mode and demonstrate how to communicate with an off-chip SPI Slave device.
 *          This sample code needs to work with USCI_SPI_SlaveMode sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define TEST_COUNT  16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void USCI_SPI_Init(void);


int main()
{
    uint32_t u32DataCount;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init USCI_SPI0 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------+\n");
    printf("|             USCI_SPI Master Mode Sample Code           |\n");
    printf("+--------------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI0 as a master.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI0:\n");
    printf("    USCI_SPI0_SS (PB.0)\n    USCI_SPI0_CLK (PD.0)\n");
    printf("    USCI_SPI0_MISO (PD.2)\n    USCI_SPI0_MOSI (PD.1)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The USCI_SPI master configuration is ready.\n");

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = 0x5500 + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.");
    getchar();
    printf("\n");

    /* Enable TX end interrupt */
    USPI_EnableInt(USPI0, USPI_TXEND_INT_MASK);
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(USCI0_IRQn);

    /* Write to TX Buffer */
    USPI_WRITE_TX(USPI0, g_au32SourceData[g_u32TxDataCount++]);

    /* Wait for transfer done */
    while(g_u32RxDataCount < TEST_COUNT);

    /* Print the received data */
    printf("Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }
    /* Disable TX end interrupt */
    USPI_DisableInt(USPI0, USPI_TXEND_INT_MASK);
    NVIC_DisableIRQ(USCI0_IRQn);
    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Close USCI_SPI0 */
    USPI_Close(USPI0);
    while(1);
}

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

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPD_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set USCI0_SPI multi-function pins */
    SYS->GPD_MFP0 = SYS->GPD_MFP0 & ~(SYS_GPD_MFP0_PD0MFP_Msk|SYS_GPD_MFP0_PD1MFP_Msk|SYS_GPD_MFP0_PD2MFP_Msk);
    SYS->GPD_MFP0 = SYS->GPD_MFP0 | (SYS_GPD_MFP0_PD0MFP_USCI0_CLK | SYS_GPD_MFP0_PD1MFP_USCI0_DAT0 | SYS_GPD_MFP0_PD2MFP_USCI0_DAT1);
    SYS->GPB_MFP0 = SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB0MFP_Msk);
    SYS->GPB_MFP0 = SYS->GPB_MFP0 | (SYS_GPB_MFP0_PB0MFP_USCI0_CTL0);

    /* USCI_SPI clock pin enable schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void USCI_SPI_Init(void)
{
    /* Configure USCI_SPI0 as a master, USCI_SPI0 clock rate 2 MHz,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI_Open(USPI0, USPI_MASTER, USPI_MODE_0, 16, 2000000);
    /* Enable the automatic hardware slave selection function and configure USCI_SPI_SS pin as low-active. */
    USPI_EnableAutoSS(USPI0, 0, USPI_SS_ACTIVE_LOW);
}

void USCI0_IRQHandler(void)
{
    uint32_t u32RxData;

    /* Clear TX end interrupt flag */
    USPI_CLR_PROT_INT_FLAG(USPI0, USPI_PROTSTS_TXENDIF_Msk);

    /* Waiting for RX is not empty */
    while(USPI_GET_RX_EMPTY_FLAG(USPI0) == 1);

    /* Check RX EMPTY flag */
    while(USPI_GET_RX_EMPTY_FLAG(USPI0) == 0)
    {
        /* Read RX Buffer */
        u32RxData = USPI_READ_RX(USPI0);
        g_au32DestinationData[g_u32RxDataCount++] = u32RxData;
    }
    /* Check TX data count */
    if(g_u32TxDataCount < TEST_COUNT)
    {
        /* Write to TX Buffer */
        USPI_WRITE_TX(USPI0, g_au32SourceData[g_u32TxDataCount++]);
    }
}


/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
