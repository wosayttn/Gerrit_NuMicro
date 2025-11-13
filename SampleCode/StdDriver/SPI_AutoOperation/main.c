/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to do SPI loopback test in Auto-operation mode
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
volatile uint32_t g_u32PdmaIntFlag;
volatile uint32_t g_u32Ifr = 0;

#define DATA_COUNT                  32
#define RX_PHASE_TCNT               1
#define TIMER0_FREQ                 1
#define SPI_MASTER_TX_DMA_CH        0
#define SPI_MASTER_RX_DMA_CH        1
#define SPI_OPENED_CH               ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))
#define TEST_PATTERN                0x55000000
#define SPI_CLK_FREQ                2000000

uint32_t g_au32MasterToSlaveTestPattern[DATA_COUNT];
uint32_t g_au32MasterRxBuffer[DATA_COUNT];


void SPI0_IRQHandler(void)
{
    /* for Auto Operation mode test */
    g_u32Ifr = SPI0->AUTOSTS;
    SPI0->AUTOSTS = SPI0->AUTOSTS;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Setup SPI0 multi-function pins */
    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA3MFP_Msk |
                                       SYS_GPA_MFPL_PA2MFP_Msk |
                                       SYS_GPA_MFPL_PA1MFP_Msk |
                                       SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA3MFP_SPI0_SS |
                     SYS_GPA_MFPL_PA2MFP_SPI0_CLK |
                     SYS_GPA_MFPL_PA1MFP_SPI0_MISO |
                     SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);

    /* Clock output HCLK to PB14 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) |
                    (SYS_GPB_MFPH_PB14MFP_CLKO);
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

void TIMER0_Init(void)
{
    /* Open TIMER */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, TIMER0_FREQ);

    /* Set TIMER trigger source by time out event */
    TIMER_SetTriggerSource(TIMER0, TIMER_TRGSRC_TIMEOUT_EVENT);

    /* Enable TIMER clock in power-down mode */
    TIMER0->CTL |= TIMER_CTL_PDCLKEN_Msk;

    /* Enable TIMER time-out trigger function in Auto Operation Mode */
    TIMER0->ATRGCTL = TIMER_ATRGCTL_ATRGEN_Msk;
}

void PDMA_Init(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Enable PDMA channels */
    PDMA_Open(PDMA0, SPI_OPENED_CH);

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
    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_MASTER_TX_DMA_CH, (uint32_t)g_au32MasterToSlaveTestPattern, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_SPI0_TX , FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

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
    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_32, DATA_COUNT+RX_PHASE_TCNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_MASTER_RX_DMA_CH, (uint32_t)&SPI0->RX, PDMA_SAR_FIX, (uint32_t)g_au32MasterRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Clear transfer done flag */
    PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk << SPI_MASTER_TX_DMA_CH);
    PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk << SPI_MASTER_RX_DMA_CH);
}

void SPI_Init(void)
{
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge.
       Set IP clock divider. SPI clock rate = 2 MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, SPI_CLK_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);

    /* Select SPI Auto Trigger source from TIMER0 */
    SPI_SET_AUTO_TRIG_SOURCE(SPI0, SPI_AUTOCTL_TRIGSEL_TMR0);

    /* Enable Auto Trigger mode */
    SPI_ENABLE_AUTO_TRIG(SPI0);

    /* Enable Full RX (data recived in TX phase) */
    SPI_ENABLE_AUTO_FULLRX(SPI0);

    /* Enable TCNT in RX phase */
    SPI_SET_AUTO_RX_TCNT(SPI0, RX_PHASE_TCNT);

    /* Enable Auto CNT match wake up */
    SPI_ENABLE_AUTO_CNT_WAKEUP(SPI0);

    /* Clear CNT match wake up flag */
    SPI_CLR_AUTO_CNTWK_FLAG(SPI0);

    /* Clear CNT match interrupt flag */
    SPI_CLR_AUTO_CNTWK_FLAG(SPI0);

    /* Enable Auto CNT match interrupt */
    SPI_ENABLE_AUTO_CNT_INT(SPI0);

    /* Enable Auto mode */
    SPI_ENABLE_AUTO(SPI0);

    /* Enable SPI0 interrupt */
    NVIC_EnableIRQ(SPI0_IRQn);
}

void AutoOperation_FunctionTest()
{
    uint32_t i, u32DataCount, u32TotalRxCount, *ptr;
    uint32_t u32PdmaDoneFlag;

    g_u32WakeupCount = 0;

    /* Init SPI runs in Auto Operation Mode */
    SPI_Init();

    /* Init TIMER */
    TIMER0_Init();
    TIMER_Start(TIMER0);

    while(1)
    {
        /* Source data initiation */
        for(u32DataCount=0; u32DataCount<DATA_COUNT; u32DataCount++)
        {
            g_au32MasterToSlaveTestPattern[u32DataCount] = ((TEST_PATTERN | (u32DataCount + 1)) + g_u32WakeupCount) ;
            g_au32MasterRxBuffer[u32DataCount] = 0x00;
        }

        PDMA_Init();

        printf("\nPower down and wait PDMA to wake up CPU ...\n\n");
        UART_WAIT_TX_EMPTY(UART0);

        /* Clear all wake-up status flags */
        CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        g_u32PdmaIntFlag = 0;

        SYS_UnlockReg();
        CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_NPD2);
//        CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_NPD1);
        CLK_PowerDown();
        SYS_LockReg();

        printf("Woken up %d times !!\n", ++g_u32WakeupCount);

        /* g_u32Ifr is set in SPI0 interrupt service routine */
        while (g_u32Ifr == 0);

        if ((g_u32Ifr&(SPI_AUTOSTS_CNTIF_Msk | SPI_AUTOSTS_CNTWKF_Msk))
                != (SPI_AUTOSTS_CNTIF_Msk | SPI_AUTOSTS_CNTWKF_Msk))
        {
            printf("Some errors happened, SPI->AUTOSTS=0x%x \n", g_u32Ifr);
            while(1);
        }

        /* Check transfer result */
        u32PdmaDoneFlag = PDMA_GET_TD_STS(PDMA0);

        /* check the master's TX DMA interrupt flag */
        if((u32PdmaDoneFlag & (1<<SPI_MASTER_TX_DMA_CH))==0) while(1);
        PDMA_CLR_TD_FLAG(PDMA0, (1<<SPI_MASTER_TX_DMA_CH));

        /* check the master's RX DMA interrupt flag */
        if((u32PdmaDoneFlag & (1<<SPI_MASTER_RX_DMA_CH))==0) while(1);
        PDMA_CLR_TD_FLAG(PDMA0, (1<<SPI_MASTER_RX_DMA_CH));

        printf("*** Since SPI TX doesn't send data in RX phase, RX pin cannot get last one sample in this example code. ***\n\n");

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
    printf("|         M2U51 SPI Auto Operation Mode Sample Code                |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates SPI0 self loop back data transfer in Auto Operation Mode,\n");
    printf("  and its procedure is listed below. \n");
    printf(" 1. Initialize TIMER0, PDMA and SPI0. \n");
    printf("    SPI0 is configured to Master mode with data width is 32 bits.\n");
    printf("    For loop back test, its I/O connection: SPI0_MOSI(PA.0) <--> SPI0_MISO(PA.1) \n");
    printf(" 2. Let system enter Powerr Mode. \n");
    printf(" 3. TIMER0 will trigger SPI0 to do (32+1)-sample data transfer per one second. After finishing data transfer, \n");
    printf("    system will be woken up and the received data can be checked. \n");
    printf(" 4. The above step-2 and step-3 will be executed repeatedly. \n\n");
    printf("Please hit any key to start test.\n\n");
    getchar();

    AutoOperation_FunctionTest();

    printf("Exit SPI Auto-operation sample code\n");

    while(1);

}
