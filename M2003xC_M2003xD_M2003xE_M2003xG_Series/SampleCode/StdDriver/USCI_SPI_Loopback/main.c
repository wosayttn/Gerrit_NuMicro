/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Implement USCI_SPI0 Master loop back transfer.
 *           This sample code needs to connect USCI_SPI0_MISO pin and USCI_SPI0_MOSI pin together.
 *           It will compare the received data with transmitted data.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define TEST_COUNT  64


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];


/* Function prototype declaration */
void SYS_Init(void);
void USCI_SPI_Init(void);


int main()
{
    uint32_t u32DataCount, u32TestCount, u32Err;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init USCI_SPI0 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|                   USCI_SPI Driver Sample Code                    |\n");
    printf("+------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates USCI_SPI0 self loop back data transfer.\n");
    printf(" USCI_SPI0 configuration:\n");
    printf("     Master mode; data width 16 bits.\n");
    printf(" I/O connection:\n");
    printf("     PD.1 USCI_SPI0_MOSI <--> PD.2 USCI_SPI0_MISO \n");

    printf("\nUSCI_SPI0 Loopback test ");

    /* set the source data and clear the destination buffer */
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        g_au32SourceData[u32DataCount] = u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }

    u32Err = 0;
    for(u32TestCount = 0; u32TestCount < 0x1000; u32TestCount++)
    {
        /* set the source data and clear the destination buffer */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            g_au32SourceData[u32DataCount]++;
            g_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount = 0;

        if((u32TestCount & 0x1FF) == 0)
        {
            putchar('.');
        }

        while(1)
        {
            /* Write to TX register */
            USPI_WRITE_TX(USPI0, g_au32SourceData[u32DataCount]);
            /* Check USPI0 busy status */
            while(USPI_IS_BUSY(USPI0));
            /* Read received data */
            g_au32DestinationData[u32DataCount] = USPI_READ_RX(USPI0);
            u32DataCount++;
            if(u32DataCount == TEST_COUNT)
                break;
        }

        /*  Check the received data */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            if(g_au32DestinationData[u32DataCount] != g_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if(u32Err)
            break;
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    /* Close USCI_SPI0 */
    USPI_Close(USPI0);

    while(1);
}

void SYS_Init(void)
{
    /* Enable internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    
    /* Switch HCLK clock source to internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPD_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set the UART debug port */
    Uart0DefaultMPF();

    /* Set USCI0_SPI multi-function pins */
    SYS->GPD_MFPL = SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD0MFP_Msk|SYS_GPD_MFPL_PD1MFP_Msk|SYS_GPD_MFPL_PD2MFP_Msk);
    SYS->GPD_MFPL = SYS->GPD_MFPL | (SYS_GPD_MFPL_PD0MFP_USCI0_CLK | SYS_GPD_MFPL_PD1MFP_USCI0_DAT0 | SYS_GPD_MFPL_PD2MFP_USCI0_DAT1);
    SYS->GPB_MFPL = SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk);
    SYS->GPB_MFPL = SYS->GPB_MFPL | (SYS_GPB_MFPL_PB0MFP_USCI0_CTL0);

    /* USCI_SPI clock pin enable schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void USCI_SPI_Init(void)
{
    /* Configure USCI_SPI0 as a master, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set USCI_SPI0 clock rate = 2MHz */
    USPI_Open(USPI0, USPI_MASTER, USPI_MODE_0, 16, 2000000);
}


/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
