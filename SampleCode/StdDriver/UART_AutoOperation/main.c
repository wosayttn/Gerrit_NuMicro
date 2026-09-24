/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of UART Automatic Operation Mode
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
extern void initialise_monitor_handles(void);
#endif

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
/* UART can support NPD0 ~ NDP1 power-down mode */
#define TEST_POWER_DOWN_MODE    CLK_PMUCTL_PDMSEL_NPD1

#define MAX_SG_TAB_NUM          8       /* Scater gather table nubmer */
#define SG_TX_LENGTH            32      /* Each Scater gather transfer length */
#define SG_BASE_ADDR            0x20000000

uint8_t SrcArray[MAX_SG_TAB_NUM*SG_TX_LENGTH];
uint8_t DestArray[MAX_SG_TAB_NUM*SG_TX_LENGTH];
DMA_DESC_T DMA_DESC_SC[MAX_SG_TAB_NUM];

/*---------------------------------------------------------------------------------------------------------*/
/* Data Compare function                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t DataCompare(uint8_t InBuffer[],uint8_t OutBuffer[],int32_t len)
{
    int32_t i=0;

    for(i=0; i<len; i++)
    {
        if(InBuffer[i]!=OutBuffer[i])
        {
            printf("In[%d] = %d , Out[%d] = %d\n",i,InBuffer[i],i,OutBuffer[i]);
            return FALSE;
        }
    }
    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Clear Buffer function                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void ClearBuf(uint32_t u32Addr, uint32_t u32Length,uint8_t u8Pattern)
{
    uint8_t* pu8Ptr;
    uint32_t i;

    pu8Ptr = (uint8_t *)u32Addr;

    for (i=0; i<u32Length; i++)
    {
        *pu8Ptr++ = u8Pattern;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Build Src Pattern function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length)
{
    uint32_t i=0,j,loop;
    uint8_t* pAddr;

    pAddr = (uint8_t *)u32Addr;

    do
    {
        if (u32Length > 256)
            loop = 256;
        else
            loop = u32Length;

        u32Length = u32Length - loop;

        for(j=0; j<loop; j++)
            *pAddr++ = (uint8_t)(j+i);

        i++;
    }
    while ((loop !=0) || (u32Length !=0));

}

/*---------------------------------------------------------------------------------------------------------*/
/*  TIMER Function                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void TIMER_trigger_init(void)
{

    /* Open TIMER0 to periodic mode and timeout 100 times per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100);

    /* Enable TIMER Power-down engine clock */
    TIMER0->CTL |= TIMER_CTL_PDCLKEN_Msk;

    /* Set TIMER to trigger UART when TIMER0 timeout */
    TIMER_SetATriggerSource(TIMER0, TIMER_ATRGSRC_TIMEOUT_EVENT);

    /* Enable TIMER0 to trigger IP */
    TIMER_SetATriggerTarget(TIMER0, TIMER_ATRG_TO_IPS);

}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void)
{
    volatile uint32_t u32regISR, temp;

    u32regISR = PDMA0->INTSTS;

    if ((u32regISR & PDMA_INTSTS_TDIF_Msk) == PDMA_INTSTS_TDIF_Msk) /* transfer done */
    {

        temp =PDMA0->TDSTS;

        //printf("PDMA transfer done\n");
        PDMA0->TDSTS = temp;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function to Build PDMA Scatter-gather table                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSCTab(uint32_t u32tabNum, uint32_t u32TxfSize, uint32_t pu8StarAddr)
{
    uint32_t i;

    for(i=0; i<u32tabNum; i++)
    {
        DMA_DESC_SC[i].ctl = PDMA_OP_SCATTER |
                             PDMA_REQ_SINGLE |
                             PDMA_DAR_FIX |                /* source address -> incremented */
                             PDMA_SAR_INC |                /* destination address -> fixed(UART) */
                             PDMA_WIDTH_8 |                /* transfer width -> 8-bit */
                             PDMA_TBINTDIS_DISABLE |      /* Table Interrupt Disable*/
                             ((u32TxfSize - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
        DMA_DESC_SC[i].src = (uint32_t)(pu8StarAddr+i*u32TxfSize);
        DMA_DESC_SC[i].dest = (uint32_t)&(UART1->DAT);
        DMA_DESC_SC[i].offset = (uint32_t)&DMA_DESC_SC[0] + 0x10*(i + 1) - SG_BASE_ADDR;
    }

    DMA_DESC_SC[u32tabNum-1].ctl = (DMA_DESC_SC[u32tabNum-1].ctl & ~(PDMA_DSCT_CTL_TBINTDIS_Msk | PDMA_DSCT_CTL_OPMODE_Msk))  \
                                   | (PDMA_TBINTDIS_ENABLE | PDMA_OP_BASIC );
    DMA_DESC_SC[u32tabNum-1].offset = 0;

}

void PDMA_TX_init(uint8_t u8TestCh, uint32_t u8TestLen)
{
    /* Set SRAM R/W base of Scatter-Gather mode */
    PDMA0->SCATBA = (uint32_t)SG_BASE_ADDR;

    PDMA_Open(PDMA0, 1<<u8TestCh);

    /* Setup Scatter-gather table for TX transfer */
    BuildSCTab(MAX_SG_TAB_NUM, SG_TX_LENGTH, (uint32_t)&SrcArray);

    PDMA_SetTransferMode(PDMA0, 0, PDMA_UART1_TX, 1, (uint32_t)&DMA_DESC_SC[0]);

}

void PDMA_RX_init(uint8_t u8TestCh, uint32_t u8TestLen)
{
    DSCT_T *ch_dsct;

    PDMA_Open(PDMA0, 1<<u8TestCh);

    ch_dsct = (DSCT_T  *)&PDMA0->DSCT[u8TestCh];
    ch_dsct->CTL =  PDMA_OP_BASIC |
                    PDMA_REQ_SINGLE |
                    PDMA_DAR_INC |                /* source address -> fixed(UART) */
                    PDMA_SAR_FIX |                /* destination address -> incremented */
                    PDMA_WIDTH_8 |                /* transfer width -> 8-bit */
                    PDMA_TBINTDIS_DISABLE |       /* Table Interrupt Disable*/
                    ((u8TestLen-1)<<PDMA_DSCT_CTL_TXCNT_Pos);

    ch_dsct->SA = (uint32_t)&(UART1->DAT);        /* PDMA Transfer Source Address */
    ch_dsct->DA = (uint32_t)DestArray;              /* PDMA Transfer Destination Address */
    PDMA_SetTransferMode(PDMA0, 1, PDMA_UART1_RX, 0, (uint32_t)&DestArray[0]);

    /* Enable Channel Transfer done interrupt */
    PDMA_EnableInt(PDMA0,u8TestCh, PDMA_INT_TRANS_DONE );
    NVIC_EnableIRQ(PDMA0_IRQn);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/

void UART_trigger_init(UART_T* uart)
{
    // set Auto Operation mode trigger source from TIMER0
    uart->AUTOCTL &= ~UART_AUTOCTL_TRIGSEL_Msk;
    uart->AUTOCTL |=  UART_AUTOCTL_TRIGSEL_TMR0;

    /* Set Automatic Operation Enable */
    uart->AUTOCTL |= UART_AUTOCTL_AOEN_Msk;

    /* Enable UART PDMA RX/TX */
    uart->INTEN |= (UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);
    uart->AUTOCTL |= UART_AUTOCTL_TRIGEN_Msk;

}


int32_t UART_AutoOP(uint32_t u32PDMode)
{
    TIMER_trigger_init();

    // PDMA CH-0 Scatter gather Mode to send UART TX data
    PDMA_TX_init(0, SG_TX_LENGTH);

    // PDMA CH-1 basic Mode RX to receive UART TX data
    PDMA_RX_init(1, SG_TX_LENGTH*MAX_SG_TAB_NUM);

    // UART TX to send data and RX to receive data
    UART_trigger_init(UART1);

    /* Start TIMER */
    TIMER_Start(TIMER0);

    /* Set Power-down mode */
    SYS_UnlockReg();
    CLK_SetPowerDownMode(u32PDMode);

    /* clear all wakeup flag */
    CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;

    /* Clear PDMA interrupt status */
    PDMA0->INTSTS = PDMA_INTSTS_WKF_Msk;

    printf("     System enter to Power-down mode NPD%d.\n", (int)(CLK->PMUCTL & CLK_PMUCTL_PDMSEL_Msk));

    UART_WAIT_TX_EMPTY(UART0);
    CLK_PowerDown();
    printf("     Wakeup\n");

    PDMA_DisableInt(PDMA0,1, PDMA_INT_TRANS_DONE );
    TIMER_Stop(TIMER0);

    if( DataCompare( SrcArray,DestArray, SG_TX_LENGTH*MAX_SG_TAB_NUM) )
        printf("     Data Compare OK.\n");
    else
        printf("     Data Compare Fail.\n");
    return 0;

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

    /* Set TMR0 clock source from HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Select UART0 clock source is HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Select UART1 clock source is HIRC and UART module clock divider as 1*/
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART1(1));

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* PDMA Clock Enable */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable TMR 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for UART1 TXD and RXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk)) |    \
                    (SYS_GPA_MFPL_PA2MFP_UART1_RXD | SYS_GPA_MFPL_PA3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init()
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
    UART_Open(UART1, 9600);
}

int main(void)
{

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for test */
    UART1_Init();

    /* Prepare Source Test Pattern for UART Tx */
    BuildSrcPattern((uint32_t)SrcArray, MAX_SG_TAB_NUM*SG_TX_LENGTH);

    /* Clear Destination buffer for UART Rx */
    ClearBuf((uint32_t)DestArray, MAX_SG_TAB_NUM*SG_TX_LENGTH, 0x55);

    /*--------------------------------------------------------------------------------------------------------------*/
    /* Autmatic Operation Mode Test                                                                                 */
    /* 1. UART uses PDMA Channel-0 to trasnfer test pattern at SrcArray                                             */
    /*        and uses PDMA Channel-1 to Received RX data to DestArray                                              */
    /* 2. UART1 TX pin (PA.3) is connected to RX pin (PA.2)                                                         */
    /* 3. System enter power-down mode and enable TMR0 to trigger UART TX transfer at power-down mode               */
    /* 4. When RX transfer done interrupt and wake-up system, compare the data between SrcArray and DestArray       */
    /*--------------------------------------------------------------------------------------------------------------*/

    printf("+-------------------------------------------------------------------------------+\n");
    printf("|     M2U51 TMR0 Trigger UART transfer under Power-Down mode Sample Code        |\n");
    printf("+-------------------------------------------------------------------------------+\n\n");
    printf("  >> Please connect PA.3 and PA.2 << \n");
    printf("     Press any key to start test\n\n");
    getchar();

    UART_AutoOP(TEST_POWER_DOWN_MODE);

    printf("\n     End of Test\n");

    while(1);

}



/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
