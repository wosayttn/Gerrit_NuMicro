/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 20/08/06 5:44p $
 * @brief
 *           Transmit and receive LPUART data with LPPDMA.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define LPUART_RX_DMA_CH 0
#define LPUART_TX_DMA_CH 1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t LPUART_TEST_LENGTH = 64;
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
uint8_t SrcArray[64] __attribute__((section(".lpSram")));
uint8_t DestArray[64] __attribute__((section(".lpSram")));
#else
uint8_t SrcArray[64] __attribute__((section(".ARM.__at_0x28000000")));
uint8_t DestArray[64] __attribute__((section(".ARM.__at_0x28000100")));
#endif
volatile int32_t IntCnt;
volatile int32_t IsTestOver;
volatile uint32_t g_u32TwoChannelLpPdmaTest = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Clear buffer function                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern)
{
    uint8_t* pu8Ptr;
    uint32_t i;

    pu8Ptr = (uint8_t *)u32Addr;

    for(i = 0; i < u32Length; i++)
    {
        *pu8Ptr++ = u8Pattern;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Build Src Pattern function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length)
{
    uint32_t i = 0, j, loop;
    uint8_t* pAddr;

    pAddr = (uint8_t *)u32Addr;

    do
    {
        if(u32Length > 256)     /* Pattern from 0 ~ 255 */
            loop = 256;
        else
            loop = u32Length;

        u32Length = u32Length - loop;

        for(j = 0; j < loop; j++)
            *pAddr++ = (uint8_t)(j + i);

        i++;
    }
    while((loop != 0) || (u32Length != 0));
}

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART Tx LPPDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LPPDMA_LPUART_TxTest(void)
{
    /* LPUART Tx LPPDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA0, LPUART_TX_DMA_CH, LPPDMA_WIDTH_8, LPUART_TEST_LENGTH);

    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA0, LPUART_TX_DMA_CH, (uint32_t)SrcArray, LPPDMA_SAR_INC, (uint32_t)&LPUART0->DAT, LPPDMA_DAR_FIX);

    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA0, LPUART_TX_DMA_CH, LPPDMA_LPUART0_TX, FALSE, 0);

    /* Single request type */
    LPPDMA_SetBurstType(LPPDMA0, LPUART_TX_DMA_CH, LPPDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    LPPDMA_DisableInt(LPPDMA0,LPUART_TX_DMA_CH, LPPDMA_INT_TEMPTY );
}

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART Rx LPPDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LPPDMA_LPUART_RxTest(void)
{
    /* LPUART Rx LPPDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA0, LPUART_RX_DMA_CH, LPPDMA_WIDTH_8, LPUART_TEST_LENGTH);

    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA0, LPUART_RX_DMA_CH, (uint32_t)&LPUART0->DAT, LPPDMA_SAR_FIX, (uint32_t)DestArray, LPPDMA_DAR_INC);

    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA0, LPUART_RX_DMA_CH, LPPDMA_LPUART0_RX, FALSE, 0);

    /* Single request type */
    LPPDMA_SetBurstType(LPPDMA0, LPUART_RX_DMA_CH, LPPDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    LPPDMA_DisableInt(LPPDMA0,LPUART_RX_DMA_CH, LPPDMA_INT_TEMPTY );
}

/*---------------------------------------------------------------------------------------------------------*/
/* LPPDMA Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LPPDMA_Callback_0(void)
{
    printf("\tTransfer Done %d!\r", ++IntCnt);

    /* Use LPPDMA to do LPUART loopback test 10 times */
    if(IntCnt < 10)
    {
        /* LPUART Tx and Rx LPPDMA configuration */
        LPPDMA_LPUART_TxTest();
        LPPDMA_LPUART_RxTest();

        /* Enable LPUART Tx and Rx LPPDMA function */
        LPUART_ENABLE_INT(LPUART0, (LPUART_INTEN_RXPDMAEN_Msk | LPUART_INTEN_TXPDMAEN_Msk));
    }
    else
    {
        /* Test is over */
        IsTestOver = TRUE;
    }
}

void LPPDMA_Callback_1(void)
{
    int32_t i ;

    printf("\tTransfer Done %d!\t", ++IntCnt);

    /* Show LPUART Rx data */
    for(i = 0; i < LPUART_TEST_LENGTH; i++)
        printf(" 0x%x(%c),", inpb(((uint32_t)DestArray + i)), inpb(((uint32_t)DestArray + i)));
    printf("\n");

    /* Use LPPDMA to do LPUART Rx test 10 times */
    if(IntCnt < 10)
    {
        /* LPUART Rx LPPDMA configuration */
        LPPDMA_LPUART_RxTest();

        /* Enable LPUART Rx LPPDMA function */
        LPUART_ENABLE_INT(LPUART0, LPUART_INTEN_RXPDMAEN_Msk);
    }
    else
    {
        /* Test is over */
        IsTestOver = TRUE;
    }
}

void LPPDMA0_IRQHandler(void)
{
    /* Get LPPDMA interrupt status */
    uint32_t status = LPPDMA_GET_INT_STATUS(LPPDMA0);

    if(status & LPPDMA_INTSTS_ABTIF_Msk)   /* Target Abort */
    {
        if (LPPDMA_GET_ABORT_STS(LPPDMA0) & LPPDMA_ABTSTS_ABTIF2_Msk)
            IsTestOver = 2;

        LPPDMA_CLR_ABORT_FLAG(LPPDMA0, LPPDMA_GET_ABORT_STS(LPPDMA0));
    }
    else if(status & LPPDMA_INTSTS_TDIF_Msk)     /* Transfer Done */
    {
        /* LPUART Tx LPPDMA transfer done interrupt flag */
        if(LPPDMA_GET_TD_STS(LPPDMA0) & (1 << LPUART_TX_DMA_CH))
        {
            /* Clear LPPDMA transfer done interrupt flag */
            LPPDMA_CLR_TD_FLAG(LPPDMA0, (1 << LPUART_TX_DMA_CH));

            /* Disable LPUART Tx LPPDMA function */
            LPUART_DISABLE_INT(LPUART0, LPUART_INTEN_TXPDMAEN_Msk);
        }

        /* LPUART Rx LPPDMA transfer done interrupt flag */
        if(LPPDMA_GET_TD_STS(LPPDMA0) & (1 << LPUART_RX_DMA_CH))
        {
            /* Clear LPPDMA transfer done interrupt flag */
            LPPDMA_CLR_TD_FLAG(LPPDMA0, (1 << LPUART_RX_DMA_CH));

            /* Disable LPUART Rx LPPDMA function */
            LPUART_DISABLE_INT(LPUART0, LPUART_INTEN_RXPDMAEN_Msk);

            /* Handle LPPDMA transfer done interrupt event */
            if(g_u32TwoChannelLpPdmaTest == 1)
            {
                LPPDMA_Callback_0();
            }
            else if(g_u32TwoChannelLpPdmaTest == 0)
            {
                LPPDMA_Callback_1();
            }
        }
    }
    else
    {
        printf("unknown interrupt, status=0x%x !!\n", status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    /* Get UART0 Rx data and send the data to LPUART0 Tx */
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAIF_Msk))
        LPUART_WRITE(LPUART0,UART_READ(UART0));
}

/*---------------------------------------------------------------------------------------------------------*/
/* LPPDMA Sample Code:                                                                                       */
/*         i32option : ['1'] LPUART0 TX/RX LPPDMA Loopback                                                     */
/*                     [Others] LPUART0 RX LPPDMA test                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void LPPDMA_LPUART(int32_t i32option)
{
    /* Source data initiation */
    BuildSrcPattern((uint32_t)SrcArray, LPUART_TEST_LENGTH);
    ClearBuf((uint32_t)DestArray, LPUART_TEST_LENGTH, 0xFF);

    /* Reset LPPDMA module */
    SYS_ResetModule(LPPDMA0_RST);

    if(i32option == '1')
    {
        printf("  [Using TWO LPPDMA channel].\n");
        printf("  This sample code will use LPPDMA to do LPUART0 loopback test 10 times.\n");
        printf("  Please connect LPUART0_RXD(PA.0) <--> LPUART0_TXD(PA.1) before testing.\n");
        printf("  After connecting PA.0 <--> PA.1, press any key to start transfer.\n");
        g_u32TwoChannelLpPdmaTest = 1;
        getchar();
    }
    else
    {
        LPUART_TEST_LENGTH = 2;      /* Test Length */
        printf("  [Using ONE LPPDMA channel].\n");
        printf("  This sample code will use LPPDMA to do LPUART0 Rx test 10 times.\n");
        printf("  Please connect LPUART0_RXD(PA.0) <--> LPUART0_TXD(PA.1) before testing.\n");
        printf("  After connecting PA.0 <--> PA.1, press any key to start transfer.\n");
        g_u32TwoChannelLpPdmaTest = 0;
        getchar();
        printf("  Please input %d bytes to trigger LPPDMA one time.(Ex: Press 'a''b')\n", LPUART_TEST_LENGTH);
    }

    if(g_u32TwoChannelLpPdmaTest == 1)
    {
        /* Enable LPPDMA channel */
        LPPDMA_Open(LPPDMA0, (1 << LPUART_RX_DMA_CH) | (1 << LPUART_TX_DMA_CH));

        /* LPUART Tx and Rx LPPDMA configuration */
        LPPDMA_LPUART_TxTest();
        LPPDMA_LPUART_RxTest();

        /* Enable LPPDMA Transfer Done Interrupt */
        LPPDMA_EnableInt(LPPDMA0, LPUART_RX_DMA_CH, LPPDMA_INT_TRANS_DONE);
        LPPDMA_EnableInt(LPPDMA0, LPUART_TX_DMA_CH, LPPDMA_INT_TRANS_DONE);
    }
    else
    {
        /* Enable LPPDMA channel */
        LPPDMA_Open(LPPDMA0, (1 << LPUART_RX_DMA_CH));

        /* LPUART Rx LPPDMA configuration */
        LPPDMA_LPUART_RxTest();

        /* Enable LPPDMA Transfer Done Interrupt */
        LPPDMA_EnableInt(LPPDMA0, LPUART_RX_DMA_CH, LPPDMA_INT_TRANS_DONE);
    }

    /* Enable LPPDMA Transfer Done Interrupt */
    IntCnt = 0;
    IsTestOver = FALSE;
    NVIC_EnableIRQ(LPPDMA0_IRQn);

    /* Enable UART0 RDA interrupt */
    if(g_u32TwoChannelLpPdmaTest == 0)
    {
        NVIC_EnableIRQ(UART0_IRQn);
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    }

    /* Enable LPUART Tx and Rx LPPDMA function */
    if(g_u32TwoChannelLpPdmaTest == 1)
        LPUART_ENABLE_INT(LPUART0,LPUART_INTEN_TXPDMAEN_Msk );
    else
        LPUART_DISABLE_INT(LPUART0, LPUART_INTEN_TXPDMAEN_Msk);

    LPUART_ENABLE_INT(LPUART0,LPUART_INTEN_RXPDMAEN_Msk );

    /* Wait for LPPDMA operation finish */
    while(IsTestOver == FALSE);

    /* Check LPPDMA status */
    if(IsTestOver == 2)
        printf("target abort...\n");

    /* Disable LPUART Tx and Rx LPPDMA function */
    LPUART_DISABLE_INT(LPUART0, (LPUART_INTEN_TXPDMAEN_Msk | LPUART_INTEN_RXPDMAEN_Msk));

    /* Disable LPPDMA channel */
    LPPDMA_Close(LPPDMA0);

    /* Disable LPPDMA Interrupt */
    LPPDMA_DisableInt(LPPDMA0, LPUART_RX_DMA_CH, LPPDMA_INT_TRANS_DONE);
    LPPDMA_DisableInt(LPPDMA0, LPUART_TX_DMA_CH, LPPDMA_INT_TRANS_DONE);
    NVIC_DisableIRQ(LPPDMA0_IRQn);

    /* Disable UART0 RDA interrupt */
    UART_DisableInt(UART0, UART_INTEN_RDAIEN_Msk);
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select Low Power UART0 clock source is HIRC and Low Power UART module clock divider as 1*/
    CLK_SetModuleClock(LPUART0_MODULE, LPSCC_CLKSEL0_LPUART0SEL_HIRC, LPSCC_CLKDIV0_LPUART0(1));

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable Low Power UART0 peripheral clock */
    CLK_EnableModuleClock(LPUART0_MODULE);
    /* Enable LPPDMA module clock */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for Low Power UART0 TXD and RXD */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk)) |    \
                    (SYS_GPA_MFP0_PA0MFP_LPUART0_RXD | SYS_GPA_MFP0_PA1MFP_LPUART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void LPUART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LPUART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset Low Power UART0 */
    SYS_ResetModule(LPUART0_RST);

    /* Configure Low Power UART0 and set Low Power UART0 Baudrate */
    LPUART_Open(LPUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    uint8_t unItem;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init LPUART0 */
    LPUART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nLPUART LPPDMA Sample Program");

    /* LPUART LPPDMA sample function */
    do
    {
        printf("\n\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|                      LPUART LPPDMA Driver Sample Code                      |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("| [1] Using TWO LPPDMA channel to test. < TX1(CH1)-->RX1(CH0) >            |\n");
        printf("| [2] Using ONE LPPDMA channel to test. < TX1-->RX1(CH0) >                 |\n");
        printf("+------------------------------------------------------------------------+\n");
        unItem = getchar();

        IsTestOver = FALSE;
        if((unItem == '1') || (unItem == '2'))
        {
            LPPDMA_LPUART(unItem);
            printf("\n\n  LPUART LPPDMA sample code is complete.\n");
        }

    }
    while(unItem != 27);

    while(1);

}
