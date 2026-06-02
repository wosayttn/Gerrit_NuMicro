/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of LPUART Automatic Operation Mode
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
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
/* LPUART can support NPD0 ~ NDP4 power-down mode */
#define TEST_POWER_DOWN_MODE    CLK_PMUCTL_PDMSEL_NPD4

#define MAX_SG_TAB_NUM          8       /* Scater gather table nubmer */
#define SG_TX_LENGTH            32      /* Each Scater gather transfer length */
#define SG_BASE_ADDR            0x28000000

#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
uint8_t SrcArray[MAX_SG_TAB_NUM*SG_TX_LENGTH] __attribute__((section(".lpSram")));
uint8_t DestArray[MAX_SG_TAB_NUM*SG_TX_LENGTH] __attribute__((section(".lpSram")));
DMA_DESC_T DMA_DESC_SC[MAX_SG_TAB_NUM] __attribute__((section(".lpSram"))) = {0};
#else
uint8_t SrcArray[MAX_SG_TAB_NUM*SG_TX_LENGTH] __attribute__((section(".ARM.__at_0x28000000")));
uint8_t DestArray[MAX_SG_TAB_NUM*SG_TX_LENGTH] __attribute__((section(".ARM.__at_0x28000100")));
DMA_DESC_T DMA_DESC_SC[MAX_SG_TAB_NUM] __attribute__((section(".ARM.__at_0x28000800"))) = {0};
#endif

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
/* Bulid Src Pattern function                                                                              */
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
/*  LPTMR Function                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LPTMR_trigger_init(void)
{

    /* Open LPTRM0 to periodic mode and timeout 100 times per second */
    LPTMR_Open(LPTMR0, TIMER_PERIODIC_MODE, 100);

    /* Enable LPTMR Power-down engine clock */
    LPTMR0->CTL |= LPTMR_CTL_PDCLKEN_Msk;

    /* Set LPTTMR to trigger LPUART when LPTMR0 timeout */
    LPTMR_SetTriggerSource(LPTMR0, TIMER_TRGSRC_TIMEOUT_EVENT);

    /* Enable LPTMR0 to trigger Low Power IP */
    LPTMR0->TRGCTL |= LPTMR_TRGCTL_TRGEN_Msk;

}

/*---------------------------------------------------------------------------------------------------------*/
/* LPPDMA Interrupt handler                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LPPDMA0_IRQHandler(void)
{
    volatile uint32_t u32regISR, temp;

    u32regISR = LPPDMA0->INTSTS;
#if 0
    printf("LPDMA ISR INTSTS= 0x%x\n", u32regISR);
    printf("LPDMA CurSCAT= 0x%x\n", LPPDMA0->CURSCAT[0]);
    printf("SC addr = 0x%x\n", (uint32_t)&(DMA_DESC_SC[MAX_SG_TAB_NUM-1]));
#endif

    if ((u32regISR & LPPDMA_INTSTS_TDIF_Msk) == LPPDMA_INTSTS_TDIF_Msk) /* transfer done */
    {

        temp =LPPDMA0->TDSTS;

        //printf("LPPDMA transfer done\n");
        LPPDMA0->TDSTS = temp;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function to Build LPPDMA Scatter-gather table                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSCTab(uint32_t u32tabNum, uint32_t u32TxfSize, uint32_t pu8StarAddr)
{
    uint32_t i;

    for(i=0; i<u32tabNum; i++)
    {
        DMA_DESC_SC[i].ctl = LPPDMA_OP_SCATTER |
                             LPPDMA_REQ_SINGLE |
                             LPPDMA_DAR_FIX |                /* source address -> incremented */
                             LPPDMA_SAR_INC |                /* destination address -> fixed(LPUART) */
                             LPPDMA_WIDTH_8 |                /* transfer width -> 8-bit */
                             LPPDMA_TBINTDIS_DISABLE |      /* Table Interrupt Disable*/
                             ((u32TxfSize - 1) << LPPDMA_DSCT_CTL_TXCNT_Pos);
        DMA_DESC_SC[i].src = (uint32_t)(pu8StarAddr+i*u32TxfSize);
        DMA_DESC_SC[i].dest = (uint32_t)&(LPUART0->DAT);
        DMA_DESC_SC[i].offset = (uint32_t)&DMA_DESC_SC[0] + 0x10*(i + 1) - SG_BASE_ADDR;
    }

    DMA_DESC_SC[u32tabNum-1].ctl = (DMA_DESC_SC[u32tabNum-1].ctl & ~(PDMA_DSCT_CTL_TBINTDIS_Msk | LPPDMA_DSCT_CTL_OPMODE_Msk))  \
                                   | (LPPDMA_TBINTDIS_ENABLE | LPPDMA_OP_BASIC );
    DMA_DESC_SC[u32tabNum-1].offset = 0;

}

void LPPDMA_TX_init(uint8_t u8TestCh, uint32_t u8TestLen)
{
    /* Set SRAM R/W base of Scatter-Gather mode */
    LPPDMA0->SCATBA = (uint32_t)SG_BASE_ADDR;

    LPPDMA_Open(LPPDMA0, 1<<u8TestCh);

    /* Setup Scatter-gather table for TX transfer */
    BuildSCTab(MAX_SG_TAB_NUM, SG_TX_LENGTH, (uint32_t)&SrcArray);

    LPPDMA_SetTransferMode(LPPDMA0, 0, LPPDMA_LPUART0_TX, 1, (uint32_t)&DMA_DESC_SC[0]);

}

void LPPDMA_RX_init(uint8_t u8TestCh, uint32_t u8TestLen)
{
    DSCT_T *ch_dsct;

    LPPDMA_Open(LPPDMA0, 1<<u8TestCh);

    ch_dsct = (DSCT_T  *)&LPPDMA0->LPDSCT[u8TestCh];
    ch_dsct->CTL =  LPPDMA_OP_BASIC |
                    LPPDMA_REQ_SINGLE |
                    LPPDMA_DAR_INC |                /* source address -> fixed(LPUART) */
                    LPPDMA_SAR_FIX |                /* destination address -> incremented */
                    LPPDMA_WIDTH_8 |                /* transfer width -> 8-bit */
                    LPPDMA_TBINTDIS_DISABLE |       /* Table Interrupt Disable*/
                    ((u8TestLen-1)<<LPPDMA_DSCT_CTL_TXCNT_Pos);

    ch_dsct->SA = (uint32_t)&(LPUART0->DAT);        /* LPPDMA Transfer Source Address */
    ch_dsct->DA = (uint32_t)DestArray;              /* LPPDMA Transfer Destination Address */
    LPPDMA_SetTransferMode(LPPDMA0, 1, LPPDMA_LPUART0_RX, 0, (uint32_t)&DestArray[0]);

    /* Enable Channel Transfer done interrupt */
    LPPDMA_EnableInt(LPPDMA0,u8TestCh, LPPDMA_INT_TRANS_DONE );
    NVIC_EnableIRQ(LPPDMA0_IRQn);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Function                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

void LPUART_trigger_init(LPUART_T* lpuart)
{
    // set Auto Operation mode trigger source from LPTMR0
    lpuart->AUTOCTL &= ~LPUART_AUTOCTL_TRIGSEL_Msk;
    lpuart->AUTOCTL |=  LPUART_AUTOCTL_TRIGSEL_LPTMR0;

    /* Set Automatic Operation Enable */
    lpuart->AUTOCTL |= LPUART_AUTOCTL_AOEN_Msk;

    /* Enable LPUART PDMA RX/TX */
    lpuart->INTEN |= (LPUART_INTEN_TXPDMAEN_Msk | LPUART_INTEN_RXPDMAEN_Msk);
    lpuart->AUTOCTL |= LPUART_AUTOCTL_TRIGEN_Msk;

}


int32_t LPUART_AutoOP(uint32_t u32PDMode)
{
    LPTMR_trigger_init();

    // LPPDMA CH-0 Scatter gather Mode to send LUART TX data
    LPPDMA_TX_init(0, SG_TX_LENGTH);

    // LPPDMA CH-1 basic Mode RX to receive LUART TX data
    LPPDMA_RX_init(1, SG_TX_LENGTH*MAX_SG_TAB_NUM);

    // LUART TX to send data and RX to receive data
    LPUART_trigger_init(LPUART0);

    /* Start LPTMR */
    LPTMR_Start(LPTMR0);

    /* Keep LPUART clock at power-down mode NPD3/NDP4 */
    if ((u32PDMode == CLK_PMUCTL_PDMSEL_NPD3) || (u32PDMode == CLK_PMUCTL_PDMSEL_NPD4))
        LPSCC->CLKKEEP0 |= LPSCC_CLKKEEP0_LPUART0KEEP_Msk;

    /* Set Power-down mode */
    SYS_UnlockReg();
    CLK_SetPowerDownMode(u32PDMode);

    /* clear all wakeup flag */
    CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;

    /* Clear LPPDMA interrupt status */
    LPPDMA0->INTSTS = LPPDMA_INTSTS_WKF_Msk | LPPDMA_INTSTS_ALIGNF_Msk | LPPDMA_INTSTS_TDIF_Msk | LPPDMA_INTSTS_ABTIF_Msk;

    printf("     System enter to Power-down mode NPD%d.\n", (int)(CLK->PMUCTL & CLK_PMUCTL_PDMSEL_Msk));

    UART_WAIT_TX_EMPTY(UART0);
    CLK_PowerDown();
    printf("     Wakeup\n");

    LPPDMA_DisableInt(LPPDMA0,1, LPPDMA_INT_TRANS_DONE );
    LPTMR_Stop(LPTMR0);

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

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);


    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK0SEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC48M ;

    /* Set LPTMR0 clock source from MIRC */
    CLK_SetModuleClock(LPTMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);

    /* Select UART0 clock source is HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select Low Power UART0 clock source is HIRC and Low Power UART module clock divider as 1*/
    CLK_SetModuleClock(LPUART0_MODULE, LPSCC_CLKSEL0_LPUART0SEL_MIRC, LPSCC_CLKDIV0_LPUART0(1));

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable LPUART0 peripheral clock */
    CLK_EnableModuleClock(LPUART0_MODULE);

    /* enable LPSRAM clock */
    CLK_EnableModuleClock(LPSRAM_MODULE);

    /* LPPDMA Clock Enable */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Enable LPTMR 0 module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for Low Power UART0 TXD and RXD */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk)) |    \
                    (SYS_GPA_MFP0_PA0MFP_LPUART0_RXD | SYS_GPA_MFP0_PA1MFP_LPUART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init()
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init LPUART0                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART0_Init(void)
{
    /* Reset Low Power UART0 */
    SYS_ResetModule(LPUART0_RST);

    /* Configure Low Power UART0 and set Low Power UART0 Baudrate */
    LPUART_Open(LPUART0, 9600);
}

int main(void)
{

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART_Init();

    /* Init LPUART0 for test */
    LPUART0_Init();

    /* Prepare Source Test Pattern for LPUART Tx */
    BuildSrcPattern((uint32_t)SrcArray, MAX_SG_TAB_NUM*SG_TX_LENGTH);

    /* Clear Destination buffer for LPUART Rx */
    ClearBuf((uint32_t)DestArray, MAX_SG_TAB_NUM*SG_TX_LENGTH, 0x55);

    /*--------------------------------------------------------------------------------------------------------------*/
    /* Autmatic Operation Mode Test                                                                                 */
    /* 1. LPUART uses LPPDMA Channel-0 to trasnfer test pattern at SrcArray                                         */
    /*        and uses LPPDMA Channel-1 to Received RX data to DestArray                                            */
    /* 2. LPUART TX pin (PA.1) is connected to RX pin (PA.0)                                                        */
    /* 3. System enter power-down mode and enable LPTMR0 to trigger Low Power UART TX transfer at power-down mode   */
    /* 4. When RX transfer done interrupt and wake-up system, compare the data between SrcArray and DestArray       */
    /*--------------------------------------------------------------------------------------------------------------*/

    printf("+-------------------------------------------------------------------------------+\n");
    printf("|     M2L31 LPTMR0 Trigger LPUART transfer under Power-Down mode Sample Code    |\n");
    printf("+-------------------------------------------------------------------------------+\n\n");
    printf("  >> Please connect PA.0 and PA.1 << \n");
    printf("     Press any key to start test\n\n");
    getchar();

    LPUART_AutoOP(TEST_POWER_DOWN_MODE);

    printf("\n     End of Test\n");

    while(1);

}



/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
