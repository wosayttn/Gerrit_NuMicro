/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate I2C Auto-operation mode when chip enters power-down mode.
 *           This sample code needs to work with I2C_Slave.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define I2C0_PDMA_TX_CH    0
#define I2C0_PDMA_RX_CH    1

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
/* I2C can support NPD0 ~ NDP1 power-down mode */
#define TEST_POWER_DOWN_MODE    CLK_PMUCTL_PDMSEL_NPD1

#define SG_TX_TAB_NUM    8       /* Scater gather table nubmer */
#define SG_RX_TAB_NUM    4
#define SG_TX_LENGTH     4
#define SG_RX_LENGTH     2
#define SG_BASE_ADDR     0x20000000

uint8_t SrcArray[SG_TX_LENGTH * SG_TX_TAB_NUM];
uint8_t DestArray[SG_RX_LENGTH * SG_RX_TAB_NUM];
DMA_DESC_T DMA_DESC_SC[SG_TX_TAB_NUM];

volatile uint32_t PDMA_DONE = 0;
uint8_t g_u8DeviceAddr = 0x15;

typedef void (*I2C_FUNC)(uint32_t u32Status);

volatile static I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  Function to Build PDMA Scatter-gather table                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSCTab(uint32_t u32TabNum, uint32_t u32TxSize, uint32_t pu8StarAddr)
{
    uint32_t i;

    for(i=0; i<u32TabNum; i++)
    {
        DMA_DESC_SC[i].ctl = PDMA_OP_SCATTER |
                             PDMA_REQ_SINGLE |
                             PDMA_DAR_FIX |                /* destination address -> fixed(I2C) */
                             PDMA_SAR_INC |                /* source address -> incremented */
                             PDMA_WIDTH_8 |                /* transfer width -> 8-bit */
                             PDMA_TBINTDIS_DISABLE |      /* Table Interrupt Disable*/
                             ((u32TxSize - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
        DMA_DESC_SC[i].src = (uint32_t)(pu8StarAddr+i*u32TxSize);
        DMA_DESC_SC[i].dest = (uint32_t)&(I2C0->DAT);
        DMA_DESC_SC[i].offset = (uint32_t)&DMA_DESC_SC[0] + 0x10*(i + 1) - SG_BASE_ADDR;
    }

    DMA_DESC_SC[u32TabNum-1].ctl = (DMA_DESC_SC[u32TabNum-1].ctl & ~(PDMA_DSCT_CTL_TBINTDIS_Msk | PDMA_DSCT_CTL_OPMODE_Msk))  \
                                   | (PDMA_TBINTDIS_ENABLE | PDMA_OP_BASIC );
    DMA_DESC_SC[u32TabNum-1].offset = 0;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  TIMER Function                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void TIMER_Trigger_Init(void)
{

    /* Open TIMER0 to periodic mode and timeout 50 times per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 50);

    /* Enable TIMER Power-down engine clock */
    TIMER0->CTL |= TIMER_CTL_PDCLKEN_Msk;

    /* Set TIMER to trigger I2C when TIMER0 timeout */
    TIMER_SetATriggerSource(TIMER0, TIMER_ATRGSRC_TIMEOUT_EVENT);

    /* Enable TIMER0 to trigger IP */
    TIMER_SetATriggerTarget(TIMER0, TIMER_ATRG_TO_IPS);
}

void PDMA_TX_Init(uint8_t u8TestCh, uint32_t u32TabNum, uint32_t u32TestLen)
{
    PDMA_Open(PDMA0, 1<<u8TestCh);

    /* Setup Scatter-gather table for TX transfer */
    BuildSCTab(u32TabNum, u32TestLen, (uint32_t)&SrcArray);

    PDMA_SetTransferMode(PDMA0, u8TestCh, PDMA_I2C0_TX, 1, (uint32_t)&DMA_DESC_SC[0]);
}

void PDMA_RX_Init(uint8_t u8TestCh, uint32_t u32TestLen)
{
    PDMA_Open(PDMA0, 1<<u8TestCh);
    /* Select basic mode */
    PDMA_SetTransferMode(PDMA0, u8TestCh, PDMA_I2C0_RX, 0, 0);
    /* Set data width and transfer count */
    PDMA_SetTransferCnt(PDMA0, u8TestCh, PDMA_WIDTH_8, u32TestLen);
    /* Set PDMA Transfer Address */
    PDMA_SetTransferAddr(PDMA0, u8TestCh, (uint32_t)(&(I2C0->DAT)), PDMA_SAR_FIX, ((uint32_t) (&DestArray[0])), PDMA_DAR_INC);
    /* Select Single Request */
    PDMA_SetBurstType(PDMA0, u8TestCh, PDMA_REQ_SINGLE, 0);
}

void I2C_Trigger_Init(uint32_t u32Mode, uint32_t u32Src, uint32_t u32RxCnt, uint32_t u32TxCnt)
{
    /* Set Auto-operation Mode */
    I2C0->AUTOCTL = (I2C0->AUTOCTL & ~I2C_AUTOCTL_AUTOMODE_Msk) | (u32Mode);

    /* Trigger source select */
    I2C0->AUTOCTL = (I2C0->AUTOCTL & ~I2C_AUTOCTL_TGSRCSEL_Msk) | (u32Src);

    I2C0->AUTOCNT = (I2C0->AUTOCNT & ~(I2C_AUTOCNT_RXCNT_Msk | I2C_AUTOCNT_TXCNT_Msk));

    if(u32RxCnt != 0)
        I2C0->AUTOCNT |= ((u32RxCnt-1) <<  I2C_AUTOCNT_RXCNT_Pos);

    if(u32TxCnt != 0)
        I2C0->AUTOCNT |= ((u32TxCnt-1) <<  I2C_AUTOCNT_TXCNT_Pos);

    I2C0->AUTOCTL |= I2C_AUTOCTL_TRGEN_Msk;
}

void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA0->TDSTS;

    //Master TX
    if(u32Status & (0x1 << I2C0_PDMA_TX_CH))
    {
        printf(" I2C0 Tx done\n");
        PDMA0->TDSTS = 0x1 << I2C0_PDMA_TX_CH;
        PDMA_DONE = 1;
    }

    //Master RX
    if (u32Status & (0x1 << I2C0_PDMA_RX_CH))
    {
        printf(" I2C0 Rx done\n");
        PDMA0->TDSTS = 0x1 << I2C0_PDMA_RX_CH;
        PDMA_DONE = 1;
    }
}


void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 PDMA Master Tx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted */
    {

    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur address ACK interrupt
        */
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur data ACK interrupt
        */
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 PDMA Master Rx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                          /* START has been transmitted */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if(u32Status == 0x18)                     /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur address ACK interrupt
        */
    }
    else if(u32Status == 0x20)                     /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                     /* DATA has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur data ACK interrupt
        */
    }
    else if(u32Status == 0x10)                    /* Repeat START has been transmitted */
    {

    }
    else if(u32Status == 0x40)                    /* SLA+R has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur address ACK interrupt
        */
    }
    else if(u32Status == 0x50)                    /* DATA has been received and ACK has been returned */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur receive data ACK interrupt
        */
    }
    else if(u32Status == 0x58)                    /* DATA has been received and NACK has been returned */
    {
        /*
           Note:
           During Auto operation, I2C controller will not occur receive data NACK interrupt
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

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable PDMA Clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable TIMER 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set I2C0 multi-function pins */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk)) |
                    (SYS_GPA_MFPL_PA4MFP_I2C0_SDA | SYS_GPA_MFPL_PA5MFP_I2C0_SCL);

    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("Master I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void AutoOperation_FunctionTest(void)
{
    uint32_t i;
    uint8_t * pu8Tmp;

    I2C_Init();

    pu8Tmp = SrcArray;
    for (i=0; i<SG_TX_TAB_NUM; i++)
    {
        *pu8Tmp++ = ((g_u8DeviceAddr << 1) | 0x00);    /* 1 byte SLV + W */
        *pu8Tmp++ = (uint8_t)((i & 0xFF00) >> 8);      /* 2 bytes Data address */
        *pu8Tmp++ = (uint8_t)(i & 0x00FF);
        *pu8Tmp++ = i + 3;                             /* 1 byte1 Data */
    }

    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterTx;

    PDMA_DONE = 0;

    TIMER_Trigger_Init();

    /* PDMA CH0 Scatter gather mode to write data to slave */
    PDMA_TX_Init(I2C0_PDMA_TX_CH, SG_TX_TAB_NUM, SG_TX_LENGTH);

    /* Auto operation TXPDMA transfer mode */
    I2C_Trigger_Init(I2C_AUTO_TXPDMA, I2C_AUTOCTL_TRIGSEL_TMR0, 0, SG_TX_LENGTH);

    /* Enable Channel Transfer done interrupt */
    PDMA_EnableInt(PDMA0, I2C0_PDMA_TX_CH, PDMA_INT_TRANS_DONE );
    NVIC_EnableIRQ(PDMA0_IRQn);

    /* Start TIMER */
    TIMER_Start(TIMER0);

    PowerDownFunction();

    while (!PDMA_DONE);

    /* Disable Channel Transfer done interrupt */
    PDMA_DisableInt(PDMA0, I2C0_PDMA_TX_CH, PDMA_INT_TRANS_DONE );

    TIMER_Stop(TIMER0);

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

    /* I2C0 function to Master receive data */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterRx;

    PDMA_DONE = 0;

    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_AA);

    /* PDMA CH0 Scatter gather mode to write data to slave */
    PDMA_TX_Init(I2C0_PDMA_TX_CH, SG_RX_TAB_NUM, SG_TX_LENGTH);

    /* PDMA CH1 Scatter gather mode to receive data from slave */
    PDMA_RX_Init(I2C0_PDMA_RX_CH, SG_RX_LENGTH * SG_RX_TAB_NUM);

    /* Auto operation random read mode with repeat start */
    I2C_Trigger_Init(I2C_RANDOM_REPEAT_STA, I2C_AUTOCTL_TRIGSEL_TMR0, SG_RX_LENGTH, SG_TX_LENGTH);

    /* Enable Channel Transfer done interrupt */
    PDMA_EnableInt(PDMA0, I2C0_PDMA_RX_CH, PDMA_INT_TRANS_DONE );

    /* Start TIMER */
    TIMER_Start(TIMER0);

    PowerDownFunction();

    while (!PDMA_DONE);

    /* Disable Channel Transfer done interrupt */
    PDMA_DisableInt(PDMA0, I2C0_PDMA_RX_CH, PDMA_INT_TRANS_DONE );

    TIMER_Stop(TIMER0);

    for (i = 0; i < SG_TX_TAB_NUM; i++)
    {
        if (DestArray[i] != i + 3)
        {
            printf("Receive Data Compare Error !!\n");

            while (1);
        }
    }

    printf("I2C AutoOperation test Pass.\n");
}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*--------------------------------------------------------------------------------------------------------------*/
    /* Autmatic Operation Mode Test                                                                                 */
    /* 1. I2C uses PDMA Channel-0 to trasnfer test pattern at SrcArray                                              */
    /*        and uses PDMA Channel-1 to Received RX data to DestArray                                              */
    /* 3. System enter power-down mode and enable TIMER0 to trigger I2C TX transfer at power-down mode              */
    /* 4. When RX transfer done interrupt and wake-up system, compare the data between SrcArray and DestArray       */
    /*--------------------------------------------------------------------------------------------------------------*/

    printf("+-------------------------------------------------------+\n");
    printf("|      M2U51 I2C Auto Operation Mode Sample Code        |\n");
    printf("|                                                       |\n");
    printf("|  I2C Master (I2C0) <---> I2C Slave(I2C0)              |\n");
    printf("+-------------------------------------------------------+\n");
    printf("Press any key to start test\n");
    getchar();

    AutoOperation_FunctionTest();

    while(1);
}
