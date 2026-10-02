/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate LPI2C Auto-operation mode when chip enters power-down mode.
 *           This sample code needs to work with LPI2C_Slave.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define LPI2C0_LPPDMA_TX_CH    0
#define LPI2C0_LPPDMA_RX_CH    1

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* LPI2C can support NPD0 ~ NDP4 power-down mode */
#define TEST_POWER_DOWN_MODE    CLK_PMUCTL_PDMSEL_NPD4

#define SG_TX_TAB_NUM    8       /* Scater gather table nubmer */
#define SG_RX_TAB_NUM    4
#define SG_TX_LENGTH     4
#define SG_RX_LENGTH     2
#define SG_BASE_ADDR     0x28000000

#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
uint8_t SrcArray[SG_TX_LENGTH * SG_TX_TAB_NUM] __attribute__((section(".lpSram")));
uint8_t DestArray[SG_RX_LENGTH * SG_RX_TAB_NUM] __attribute__((section(".lpSram")));
DMA_DESC_T DMA_DESC_SC[SG_TX_TAB_NUM] __attribute__((section(".lpSram"))) = {0};
#else
uint8_t SrcArray[SG_TX_LENGTH * SG_TX_TAB_NUM] __attribute__((section(".ARM.__at_0x28000000")));
uint8_t DestArray[SG_RX_LENGTH * SG_RX_TAB_NUM] __attribute__((section(".ARM.__at_0x28000100")));
DMA_DESC_T DMA_DESC_SC[SG_TX_TAB_NUM] __attribute__((section(".ARM.__at_0x28000800"))) = {0};
#endif

volatile uint32_t LPPDMA_DONE = 0;
uint8_t g_u8DeviceAddr = 0x15;

typedef void (*LPI2C_FUNC)(uint32_t u32Status);

volatile static LPI2C_FUNC s_LPI2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  Function to Build LPPDMA Scatter-gather table                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSCTab(uint32_t u32TabNum, uint32_t u32TxSize, uint32_t pu8StarAddr)
{
    uint32_t i;

    for(i=0; i<u32TabNum; i++)
    {
        DMA_DESC_SC[i].ctl = LPPDMA_OP_SCATTER |
                             LPPDMA_REQ_SINGLE |
                             LPPDMA_DAR_FIX |                /* source address -> incremented */
                             LPPDMA_SAR_INC |                /* destination address -> fixed(LPUART) */
                             LPPDMA_WIDTH_8 |                /* transfer width -> 8-bit */
                             LPPDMA_TBINTDIS_DISABLE |      /* Table Interrupt Disable*/
                             ((u32TxSize - 1) << LPPDMA_DSCT_CTL_TXCNT_Pos);
        DMA_DESC_SC[i].src = (uint32_t)(pu8StarAddr+i*u32TxSize);
        DMA_DESC_SC[i].dest = (uint32_t)&(LPI2C0->DAT);
        DMA_DESC_SC[i].offset = (uint32_t)&DMA_DESC_SC[0] + 0x10*(i + 1) - SG_BASE_ADDR;
    }

    DMA_DESC_SC[u32TabNum-1].ctl = (DMA_DESC_SC[u32TabNum-1].ctl & ~(PDMA_DSCT_CTL_TBINTDIS_Msk | LPPDMA_DSCT_CTL_OPMODE_Msk))  \
                                   | (LPPDMA_TBINTDIS_ENABLE | LPPDMA_OP_BASIC );
    DMA_DESC_SC[u32TabNum-1].offset = 0;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPTMR Function                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void LPTMR_Trigger_Init(void)
{

    /* Open LPTRM0 to periodic mode and timeout 50 times per second */
    LPTMR_Open(LPTMR0, TIMER_PERIODIC_MODE, 50);

    /* Enable LPTMR Power-down engine clock */
    LPTMR0->CTL |= LPTMR_CTL_PDCLKEN_Msk;

    /* Set LPTMR to trigger LPI2C when LPTMR0 timeout */
    LPTMR_SetTriggerSource(LPTMR0, TIMER_TRGSRC_TIMEOUT_EVENT);

    /* Enable LPTMR0 to trigger Low Power IP */
    LPTMR_SetTriggerTarget(LPTMR0, LPTMR_TRGEN);
}

void LPPDMA_TX_Init(uint8_t u8TestCh, uint32_t u32TabNum, uint32_t u32TestLen)
{
    LPPDMA_Open(LPPDMA0, 1<<u8TestCh);

    /* Setup Scatter-gather table for TX transfer */
    BuildSCTab(u32TabNum, u32TestLen, (uint32_t)&SrcArray);

    LPPDMA_SetTransferMode(LPPDMA0, u8TestCh, LPPDMA_LPI2C0_TX, 1, (uint32_t)&DMA_DESC_SC[0]);
}

void LPPDMA_RX_Init(uint8_t u8TestCh, uint32_t u32TestLen)
{
    LPPDMA_Open(LPPDMA0, 1<<u8TestCh);
    /* Select basic mode */
    LPPDMA_SetTransferMode(LPPDMA0, u8TestCh, LPPDMA_LPI2C0_RX, 0, 0);
    /* Set data width and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA0, u8TestCh, LPPDMA_WIDTH_8, u32TestLen);
    /* Set LPPDMA Transfer Address */
    LPPDMA_SetTransferAddr(LPPDMA0, u8TestCh, (uint32_t)(&(LPI2C0->DAT)), LPPDMA_SAR_FIX, ((uint32_t) (&DestArray[0])), LPPDMA_DAR_INC);
    /* Select Single Request */
    LPPDMA_SetBurstType(LPPDMA0, u8TestCh, PDMA_REQ_SINGLE, 0);
}

void LPI2C_Trigger_Init(uint32_t u32Mode, uint32_t u32Src, uint32_t u32RxCnt, uint32_t u32TxCnt)
{
    /* Set Auto-operation Mode */
    LPI2C0->AUTOCTL = (LPI2C0->AUTOCTL & ~LPI2C_AUTOCTL_AUTOMODE_Msk) | (u32Mode);

    /* Trigger source select */
    LPI2C0->AUTOCTL = (LPI2C0->AUTOCTL & ~LPI2C_AUTOCTL_TGSRCSEL_Msk) | (u32Src);

    LPI2C0->AUTOCNT = (LPI2C0->AUTOCNT & ~(LPI2C_AUTOCNT_RXCNT_Msk | LPI2C_AUTOCNT_TXCNT_Msk));

    if(u32RxCnt != 0)
        LPI2C0->AUTOCNT |= ((u32RxCnt-1) <<  LPI2C_AUTOCNT_RXCNT_Pos);

    if(u32TxCnt != 0)
        LPI2C0->AUTOCNT |= ((u32TxCnt-1) <<  LPI2C_AUTOCNT_TXCNT_Pos);

    LPI2C0->AUTOCTL |= LPI2C_AUTOCTL_TRGEN_Msk;
}

void LPPDMA0_IRQHandler(void)
{
    uint32_t u32Status = LPPDMA0->TDSTS;

    //Master TX
    if(u32Status & (0x1 << LPI2C0_LPPDMA_TX_CH))
    {
        printf(" LPI2C0 Tx done\n");
        LPPDMA0->TDSTS = 0x1 << LPI2C0_LPPDMA_TX_CH;
        LPPDMA_DONE = 1;
    }

    //Master RX
    if (u32Status & (0x1 << LPI2C0_LPPDMA_RX_CH))
    {
        printf(" LPI2C0 Rx done\n");
        LPPDMA0->TDSTS = 0x1 << LPI2C0_LPPDMA_RX_CH;
        LPPDMA_DONE = 1;
    }
}


void LPI2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = LPI2C_GET_STATUS(LPI2C0);

    if (LPI2C_GET_TIMEOUT_FLAG(LPI2C0))
    {
        /* Clear LPI2C0 Timeout Flag */
        LPI2C_ClearTimeoutFlag(LPI2C0);
    }
    else
    {
        if (s_LPI2C0HandlerFn != NULL)
            s_LPI2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C0 LPPDMA Master Tx Callback Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_LPPDMA_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur START interrupt
        */
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted */
    {

    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur address ACK interrupt
        */
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_STOP(LPI2C0);
        LPI2C_START(LPI2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur data ACK interrupt
        */
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C0 LPPDMA Master Rx Callback Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_LPPDMA_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                          /* START has been transmitted */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur START interrupt
        */
    }
    else if(u32Status == 0x18)                     /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur address ACK interrupt
        */
    }
    else if(u32Status == 0x20)                     /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_STOP(LPI2C0);
        LPI2C_START(LPI2C0);
    }
    else if(u32Status == 0x28)                     /* DATA has been transmitted and ACK has been received */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur data ACK interrupt
        */
    }
    else if(u32Status == 0x10)                    /* Repeat START has been transmitted */
    {

    }
    else if(u32Status == 0x40)                    /* SLA+R has been transmitted and ACK has been received */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur address ACK interrupt
        */
    }
    else if(u32Status == 0x50)                    /* DATA has been received and ACK has been returned */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur receive data ACK interrupt
        */
    }
    else if(u32Status == 0x58)                    /* DATA has been received and NACK has been returned */
    {
        /*
           Note:
           During Auto operation, LPI2C controller will not occur receive data NACK interrupt
        */
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    /* Set Power-down mode */
    CLK_SetPowerDownMode(TEST_POWER_DOWN_MODE);

    /* Enter to Power-down mode */
    CLK_PowerDown();
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

    /* Set HCLK1 to HIRC */
    CLK_SetModuleClock(HCLK1_MODULE, CLK_CLKSEL0_HCLK1SEL_HIRC, LPSCC_CLKDIV0_HCLK1(1));

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable LPI2C0 clock */
    CLK_EnableModuleClock(LPI2C0_MODULE);

    /* enable LPSRAM clock */
    CLK_EnableModuleClock(LPSRAM_MODULE);

    /* Enable LPPDMA Clock */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Enable LPTMR 0 module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set LPI2C0 multi-function pins */
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk)) |
                    (SYS_GPA_MFP1_PA4MFP_LPI2C0_SDA | SYS_GPA_MFP1_PA5MFP_LPI2C0_SCL);

    /* LPI2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void LPI2C_Init(void)
{
    /* Open LPI2C0 module and set bus clock */
    LPI2C_Open(LPI2C0, 100000);

    /* Get LPI2C0 Bus Clock */
    printf("Master LPI2C0 clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));

    /* Enable LPI2C0 interrupt */
    LPI2C_EnableInt(LPI2C0);
    NVIC_EnableIRQ(LPI2C0_IRQn);
}

void AutoOperation_FunctionTest(void)
{
    uint32_t i;
    uint8_t * pu8Tmp;

    LPI2C_Init();

    pu8Tmp = SrcArray;
    for (i=0; i<SG_TX_TAB_NUM; i++)
    {
        *pu8Tmp++ = ((g_u8DeviceAddr << 1) | 0x00);    /* 1 byte SLV + W */
        *pu8Tmp++ = (uint8_t)((i & 0xFF00) >> 8);      /* 2 bytes Data address */
        *pu8Tmp++ = (uint8_t)(i & 0x00FF);
        *pu8Tmp++ = i + 3;                             /* 1 byte1 Data */
    }

    s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_LPPDMA_MasterTx;

    LPPDMA_DONE = 0;

    LPTMR_Trigger_Init();

    /* LPPDMA CH0 Scatter gather mode to write data to slave */
    LPPDMA_TX_Init(LPI2C0_LPPDMA_TX_CH, SG_TX_TAB_NUM, SG_TX_LENGTH);

    /* Auto operation TXPDMA transfer mode */
    LPI2C_Trigger_Init(LPI2C_AUTO_TXPDMA, LPI2C_TRGSRC_LPTMR0, 0, SG_TX_LENGTH);

    /* Enable Channel Transfer done interrupt */
    LPPDMA_EnableInt(LPPDMA0, LPI2C0_LPPDMA_TX_CH, LPPDMA_INT_TRANS_DONE );
    NVIC_EnableIRQ(LPPDMA0_IRQn);

    /* Start LPTMR */
    LPTMR_Start(LPTMR0);

    PowerDownFunction();

    while (!LPPDMA_DONE);

    /* Disable Channel Transfer done interrupt */
    LPPDMA_DisableInt(LPPDMA0, LPI2C0_LPPDMA_TX_CH, LPPDMA_INT_TRANS_DONE );

    LPTMR_Stop(LPTMR0);

    /* Test Master RX with Auto operation mode */
    printf("Press any key to start master Auto operation RX mode.\n");
    getchar();

    memset(SrcArray, 0, sizeof(SrcArray));
    memset(DMA_DESC_SC, 0, sizeof(DMA_DESC_SC));

    pu8Tmp = SrcArray;
    for (i=0; i<SG_TX_TAB_NUM; i = i + 2)
    {
        *pu8Tmp++ = ((g_u8DeviceAddr << 1) | 0x00);    /* 1 byte SLV + W */
        *pu8Tmp++ = (uint8_t)((i & 0xFF00) >> 8);      /* 2 bytes Data address */
        *pu8Tmp++ = (uint8_t)(i & 0x00FF);
        *pu8Tmp++ = ((g_u8DeviceAddr << 1) | 0x01);    /* 1 byte SLV + R */
    }

    /* LPI2C0 function to Master receive data */
    s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_LPPDMA_MasterRx;

    LPPDMA_DONE = 0;

    LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_AA);

    /* LPPDMA CH0 Scatter gather mode to write data to slave */
    LPPDMA_TX_Init(LPI2C0_LPPDMA_TX_CH, SG_RX_TAB_NUM, SG_TX_LENGTH);

    /* LPPDMA CH1 Scatter gather mode to receive data from slave */
    LPPDMA_RX_Init(LPI2C0_LPPDMA_RX_CH, SG_RX_LENGTH * SG_RX_TAB_NUM);

    /* Auto operation random read mode with repeat start */
    LPI2C_Trigger_Init(LPI2C_RANDOM_REPEAT_STA, LPI2C_TRGSRC_LPTMR0, SG_RX_LENGTH, SG_TX_LENGTH);

    /* Enable Channel Transfer done interrupt */
    LPPDMA_EnableInt(LPPDMA0, LPI2C0_LPPDMA_RX_CH, LPPDMA_INT_TRANS_DONE );

    /* Start LPTMR */
    LPTMR_Start(LPTMR0);

    PowerDownFunction();

    while (!LPPDMA_DONE);

    /* Disable Channel Transfer done interrupt */
    LPPDMA_DisableInt(LPPDMA0, LPI2C0_LPPDMA_RX_CH, LPPDMA_INT_TRANS_DONE );

    LPTMR_Stop(LPTMR0);

    for (i = 0; i < SG_TX_TAB_NUM; i++)
    {
        if (DestArray[i] != i + 3)
        {
            printf("Receive Data Compare Error !!\n");

            while (1);
        }
    }

    printf("LPI2C AutoOperation test Pass.\n");
}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*--------------------------------------------------------------------------------------------------------------*/
    /* Autmatic Operation Mode Test                                                                                 */
    /* 1. LPI2C uses LPPDMA Channel-0 to trasnfer test pattern at SrcArray                                          */
    /*        and uses LPPDMA Channel-1 to Received RX data to DestArray                                            */
    /* 3. System enter power-down mode and enable LPTMR0 to trigger Low Power I2C TX transfer at power-down mode    */
    /* 4. When RX transfer done interrupt and wake-up system, compare the data between SrcArray and DestArray       */
    /*--------------------------------------------------------------------------------------------------------------*/

    printf("+-------------------------------------------------------+\n");
    printf("|      M2L31 LPI2C Auto Operation Mode Sample Code      |\n");
    printf("|                                                       |\n");
    printf("|  LPI2C Master (LPI2C0) <---> LPI2C Slave(LPI2C0)      |\n");
    printf("+-------------------------------------------------------+\n");
    printf("Press any key to start test\n");
    getchar();

    AutoOperation_FunctionTest();

    while(1);
}
