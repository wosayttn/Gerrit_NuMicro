/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to do LPSPI loopback test in Auto-operation mode
 *           when chip enters power-down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*--------------------------------------------------------------------------*/
/* Define global variables and constants                                    */
/*--------------------------------------------------------------------------*/
volatile uint32_t g_u32WakeupCount = 0;
volatile uint32_t g_u32LPPdmaIntFlag;
volatile uint32_t g_u32Ifr = 0;

#define DATA_COUNT                  32
#define RX_PHASE_TCNT               1
#define LPTMR0_FREQ                 1
#define LPSPI_MASTER_TX_DMA_CH      0
#define LPSPI_MASTER_RX_DMA_CH      1
#define LPSPI_OPENED_CH             ((1 << LPSPI_MASTER_TX_DMA_CH) | (1 << LPSPI_MASTER_RX_DMA_CH))

#define TEST_PATTERN                0x55000000
#define LPSPI_CLK_FREQ              2000000

#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
uint32_t g_au32MasterToSlaveTestPattern[DATA_COUNT] __attribute__ ((section(".lpSram")));
uint32_t g_au32MasterRxBuffer[DATA_COUNT] __attribute__ ((section(".lpSram")));
#else
uint32_t g_au32MasterToSlaveTestPattern[DATA_COUNT] __attribute__ ((section(".ARM.__at_0x28000000")));
uint32_t g_au32MasterRxBuffer[DATA_COUNT] __attribute__ ((section(".ARM.__at_0x28000100")));
#endif


void LPSPI0_IRQHandler(void)
{
    /* for Auto Operation mode test */
    g_u32Ifr = LPSPI0->AUTOSTS;
    LPSPI0->AUTOSTS = LPSPI0->AUTOSTS;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LIRC */
//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set HCLK1 to HIRC/1 */
    CLK_SetModuleClock(HCLK1_MODULE, CLK_CLKSEL0_HCLK1SEL_HIRC, LPSCC_CLKDIV0_HCLK1(1));

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Set PCLK2 to HCLK1/1 */
    LPSCC->CLKDIV0 = (LPSCC->CLKDIV0 & ~(LPSCC_CLKDIV0_APB2DIV_Msk)) |
                     (LPSCC_CLKDIV0_APB2DIV_DIV1);

    /* LPSPI clock source is from HIRC (24MHz) and set HCLK1 divider to 1, given HCLK1 being also 24 MHz */
    CLK_SetModuleClock(LPSPI0_MODULE, LPSCC_CLKSEL0_LPSPI0SEL_HIRC, LPSCC_CLKDIV0_HCLK1(1));

    /* Enable LPSPI module clock */
    CLK_EnableModuleClock(LPSPI0_MODULE);

    /* Select LPTMR0 module clock source from HIRC */
    CLK_SetModuleClock(LPTMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);

    /* Enable LPTMR0 module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Enable LPPDMA clock source */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Enable LPSRAM clock source */
    /* LPPDMA only can access LPSRAM and cannot access normal SRAM. */
    CLK_EnableModuleClock(LPSRAM_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Setup SPI0 multi-function pins */
    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA3MFP_Msk |
                                       SYS_GPA_MFP0_PA2MFP_Msk |
                                       SYS_GPA_MFP0_PA1MFP_Msk |
                                       SYS_GPA_MFP0_PA0MFP_Msk)) |
                    (SYS_GPA_MFP0_PA3MFP_LPSPI0_SS |
                     SYS_GPA_MFP0_PA2MFP_LPSPI0_CLK |
                     SYS_GPA_MFP0_PA1MFP_LPSPI0_MISO |
                     SYS_GPA_MFP0_PA0MFP_LPSPI0_MOSI);

    /* Clock output HCLK to PB14 */
    SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(SYS_GPB_MFP3_PB14MFP_Msk)) |
                    (SYS_GPB_MFP3_PB14MFP_CLKO);
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 0, 1);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void LPTMR0_Init(void)
{
    /* Open LPTMR */
    LPTMR_Open(LPTMR0, LPTMR_PERIODIC_MODE, LPTMR0_FREQ);

    /* Set LPTMR trigger source by time out event */
    LPTMR_SetTriggerSource(LPTMR0, LPTMR_TRGSRC_TIMEOUT_EVENT);

    /* Enable LPTMR clock in power-down mode */
    LPTMR0->CTL |= LPTMR_CTL_PDCLKEN_Msk;

    /* Begin LPTMR trigger function */
    LPTMR0->TRGCTL = LPTMR_TRGCTL_TRGEN_Msk;
}

void LPPDMA_Init(void)
{
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
    PDMA->DSCT[LPSPI_MASTER_TX_DMA_CH].CTL |= LPPDMA_DSCT_CTL_TBINTDIS_Msk;

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
    LPPDMA_SetTransferCnt(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT+RX_PHASE_TCNT);
    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, (uint32_t)&LPSPI0->RX, LPPDMA_SAR_FIX, (uint32_t)g_au32MasterRxBuffer, LPPDMA_DAR_INC);
    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_LPSPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    LPPDMA_SetBurstType(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[LPSPI_MASTER_RX_DMA_CH].CTL |= LPPDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Clear transfer done flag */
    LPPDMA_CLR_TD_FLAG(LPPDMA0, LPPDMA_TDSTS_TDIF0_Msk << LPSPI_MASTER_TX_DMA_CH);
    LPPDMA_CLR_TD_FLAG(LPPDMA0, LPPDMA_TDSTS_TDIF0_Msk << LPSPI_MASTER_RX_DMA_CH);
}

void LPSPI_Init(void)
{
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge.
       Set IP clock divider. SPI clock rate = 2 MHz */
    LPSPI_Open(LPSPI0, LPSPI_MASTER, LPSPI_MODE_0, 32, LPSPI_CLK_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    LPSPI_EnableAutoSS(LPSPI0, LPSPI_SS, LPSPI_SS_ACTIVE_LOW);

    /* Select LPSPI Auto Trigger source from LPTMR0 */
    LPSPI_SET_AUTO_TRIG_SOURCE(LPSPI0, LPSPI_AUTOCTL_TRIGSEL_LPTMR0);

    /* Enable Auto Trigger mode */
    LPSPI_ENABLE_AUTO_TRIG(LPSPI0);

    /* Enable Full RX (data recived in TX phase) */
    LPSPI_ENABLE_AUTO_FULLRX(LPSPI0);

    /* Enable TCNT in RX phase */
    LPSPI_SET_AUTO_RX_TCNT(LPSPI0, RX_PHASE_TCNT);

    /* Enable Auto CNT match wake up */
    LPSPI_ENABLE_AUTO_CNT_WAKEUP(LPSPI0);

    /* Clear CNT match wake up flag */
    LPSPI_CLR_AUTO_CNTWK_FLAG(LPSPI0);

    /* Clear CNT match interrupt flag */
    LPSPI_CLR_AUTO_CNTWK_FLAG(LPSPI0);

    /* Enable Auto CNT match interrupt */
    LPSPI_ENABLE_AUTO_CNT_INT(LPSPI0);

    /* Enable Auto mode */
    LPSPI_ENABLE_AUTO(LPSPI0);

    /* Enable LPSPI0 interrupt */
    NVIC_EnableIRQ(LPSPI0_IRQn);
}

void AutoOperation_FunctionTest()
{
    uint32_t i, u32DataCount, u32TotalRxCount, *ptr;
    uint32_t u32PdmaDoneFlag;

    g_u32WakeupCount = 0;

    /* Init LPSPI runs in Auto Operation Mode */
    LPSPI_Init();

    /* Init LPTMR */
    LPTMR0_Init();
    LPTMR_Start(LPTMR0);

    while(1)
    {
        /* Source data initiation */
        for(u32DataCount=0; u32DataCount<DATA_COUNT; u32DataCount++)
        {
            g_au32MasterToSlaveTestPattern[u32DataCount] = ((TEST_PATTERN | (u32DataCount + 1)) + g_u32WakeupCount) ;
            g_au32MasterRxBuffer[u32DataCount] = 0x00;
        }

        LPPDMA_Init();

        printf("\nPower down and wait LPPDMA to wake up CPU ...\n\n");
        UART_WAIT_TX_EMPTY(UART0);

        /* Clear all wake-up status flags */
        CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        g_u32LPPdmaIntFlag = 0;

        SYS_UnlockReg();
        CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_NPD2);
        CLK_PowerDown();
        SYS_LockReg();

        printf("Woken up %d times !!\n", ++g_u32WakeupCount);

        /* g_u32Ifr is set in LPSPI0 interrupt service routine */
        while (g_u32Ifr == 0);

        if ((g_u32Ifr&(LPSPI_AUTOSTS_CNTIF_Msk | LPSPI_AUTOSTS_CNTWKF_Msk))
                != (LPSPI_AUTOSTS_CNTIF_Msk | LPSPI_AUTOSTS_CNTWKF_Msk))
        {
            printf("Some errors happened, LPSPI->AUTOSTS=0x%x \n", g_u32Ifr);
            while(1);
        }

        /* Check transfer result */
        u32PdmaDoneFlag = LPPDMA_GET_TD_STS(LPPDMA0);

        /* check the master's TX DMA interrupt flag */
        if((u32PdmaDoneFlag & (1<<LPSPI_MASTER_TX_DMA_CH))==0) while(1);
        LPPDMA_CLR_TD_FLAG(LPPDMA0, (1<<LPSPI_MASTER_TX_DMA_CH));

        /* check the master's RX DMA interrupt flag */
        if((u32PdmaDoneFlag & (1<<LPSPI_MASTER_RX_DMA_CH))==0) while(1);
        LPPDMA_CLR_TD_FLAG(LPPDMA0, (1<<LPSPI_MASTER_RX_DMA_CH));

        printf("*** Since LPSPI TX doesn't send data in RX phase, RX pin cannot get last one sample in this example code. ***\n\n");

        /* u32TotalRxCount is the total received data number in both TX phase (by FULLRX enabled) and RX phase */
        u32TotalRxCount = DATA_COUNT + RX_PHASE_TCNT;
        ptr = g_au32MasterRxBuffer;
        for (i=0; i<u32TotalRxCount; i++)
        {
            printf("    [%2d]: %08X.\n", i, *ptr);
            ptr++;
        }
    }   /* end of while(1) */
}

int32_t main(void)
{

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------------------------------+\n");
    printf("|         M2L31 LPSPI Auto Operation Mode Sample Code                |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates LPSPI0 self loop back data transfer in Auto Operation Mode,\n");
    printf("  and its procedure is listed below. \n");
    printf(" 1. Initialize LPTMR0, LPPDMA and LPSPI0. \n");
    printf("    LPSPI0 is configured to Master mode with data width is 32 bits.\n");
    printf("    For loop back test, its I/O connection: LPSPI0_MOSI(PA.0) <--> LPSPI0_MISO(PA.1) \n");
    printf(" 2. Let system enter Powerr Mode. \n");
    printf(" 3. LPTMR0 will trigger LPSPI0 to do (32+1)-sample data transfer per one second. After finishing data transfer, \n");
    printf("    system will be woken up and the received data can be checked. \n");
    printf(" 4. The above step-2 and step-3 will be executed repeatedly. \n\n");
    printf("Please hit any key to start test.\n\n");
    getchar();

    AutoOperation_FunctionTest();

    printf("Exit LPSPI Auto-operation sample code\n");

    while(1);

}
