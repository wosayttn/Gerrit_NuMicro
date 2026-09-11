/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive data in LPUART RS485 mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK   FREQ_72MHZ

#define RXBUFSIZE 1024
#define IS_USE_RS485NMM   1      //1:Select NMM_Mode , 0:Select AAD_Mode
#define MATCH_ADDRSS1     0xC0
#define MATCH_ADDRSS2     0xA2
#define UNMATCH_ADDRSS1   0xB1
#define UNMATCH_ADDRSS2   0xD3

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[12] = {0};

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE(void);
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);
void RS485_9bitModeSlave(void);
void RS485_FunctionTest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

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

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable Low Power UART0 peripheral clock */
    CLK_EnableModuleClock(LPUART0_MODULE);

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select Low Power UART0 clock source is HIRC and Low Power UART module clock divider as 1*/
    CLK_SetModuleClock(LPUART0_MODULE, LPSCC_CLKSEL0_LPUART0SEL_HIRC, LPSCC_CLKDIV0_LPUART0(1));

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
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk)) | (SYS_GPA_MFP1_PA4MFP_LPUART0_nRTS);

    /* Lock protected registers */
    SYS_LockReg();

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}

void LPUART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LPUART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    LPUART_Open(LPUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();

    /* Init LPUART0 */
    LPUART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+------------------------+\n");
    printf("| RS485 function test    |\n");
    printf("+------------------------+\n");

    RS485_FunctionTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle LPUART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART0_IRQHandler(void)
{
    RS485_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* RS485 Callback function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE()
{
    volatile uint32_t addr = 0;
    volatile uint32_t regRX = 0xFF;
    volatile uint32_t u32IntSts = LPUART0->INTSTS;

    if (LPUART_GET_INT_FLAG(LPUART0,LPUART_INTSTS_RLSINT_Msk ) && LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_RDAINT_Msk))       /* RLS INT & RDA INT */ //For RS485 Detect Address
    {
        if (LPUART_RS485_GET_ADDR_FLAG(LPUART0))   /* ADD_IF, RS485 mode */
        {
            addr = LPUART_READ(LPUART0);
            LPUART_RS485_CLEAR_ADDR_FLAG(LPUART0);        /* clear ADD_IF flag */
            printf("\nAddr=0x%x,Get:", addr);

#if (IS_USE_RS485NMM ==1) //RS485_NMM

            /* if address match, enable RX to receive data, otherwise to disable RX. */
            /* In NMM mode,user can decide multi-address filter. In AAD mode,only one address can set */
            if ((addr == MATCH_ADDRSS1) || (addr == MATCH_ADDRSS2))
            {
                LPUART0->FIFO &= ~ LPUART_FIFO_RXOFF_Msk;  /* Enable RS485 RX */
            }
            else
            {

                LPUART0->FIFO |= LPUART_FIFO_RXOFF_Msk;      /* Disable RS485 RX */
                LPUART0->FIFO |= LPUART_FIFO_RXRST_Msk;      /* Clear data from RX FIFO */
                printf("\n");
            }

#endif
        }
    }
    else if (LPUART_GET_INT_FLAG(LPUART0,LPUART_INTSTS_RDAINT_Msk ) || LPUART_GET_INT_FLAG(LPUART0,LPUART_INTSTS_RXTOINT_Msk ))     /* Rx Ready or Time-out INT*/
    {
        /* Handle received data */
        while (!LPUART_GET_RX_EMPTY(LPUART0))
            printf("%2d,", LPUART_READ(LPUART0));

    }
    else if (LPUART_GET_INT_FLAG(LPUART0,LPUART_INTSTS_BUFERRINT_Msk ))     /* Buffer Error INT */
    {
        printf("\nBuffer Error...\n");
        LPUART_ClearIntFlag(LPUART0, LPUART_INTSTS_BUFERRINT_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Control  (Address Byte: Parity Bit =1 , Data Byte:Parity Bit =0)                        */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data)
{
    LPUART_SetLine_Config(LPUART0, 0, LPUART_WORD_LEN_8, LPUART_PARITY_MARK, LPUART_STOP_BIT_1);
    LPUART_WRITE(LPUART0, u8data);
}

void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    LPUART_SetLine_Config(LPUART0, 0, LPUART_WORD_LEN_8, LPUART_PARITY_SPACE, LPUART_STOP_BIT_1);
    LPUART_Write(LPUART0, pu8TxBuf, u32WriteBytes);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeMaster()
{
    int32_t i32;
    uint8_t g_u8SendDataGroup1[10] = {0};
    uint8_t g_u8SendDataGroup2[10] = {0};
    uint8_t g_u8SendDataGroup3[10] = {0};
    uint8_t g_u8SendDataGroup4[10] = {0};

    printf("\n\n");
    printf("+------------------------------------------------------------+\n");
    printf("|               RS485 9-bit Master Test                      |\n");
    printf("+------------------------------------------------------------+\n");
    printf("| The function will send different address with 10 data bytes|\n");
    printf("| to test RS485 9-bit mode. Please connect TX/RX to another  |\n");
    printf("| board and wait its ready to receive.                       |\n");
    printf("| Press any key to start...                                  |\n");
    printf("+------------------------------------------------------------+\n\n");
    getchar();

    /* Set RS485-Master as AUD mode*/
    LPUART_SelectRS485Mode(LPUART0, LPUART_ALTCTL_RS485AUD_Msk, 0);

    LPUART0->MODEM &= ~LPUART_MODEM_RTSACTLV_Msk;

    /* Prepare Data to transmit*/
    for (i32 = 0; i32 < 10; i32++)
    {
        g_u8SendDataGroup1[i32] = i32;
        g_u8SendDataGroup2[i32] = i32 + 10;
        g_u8SendDataGroup3[i32] = i32 + 20;
        g_u8SendDataGroup4[i32] = i32 + 30;
    }

    /* Send For different Address and data for test */
    printf("Send Address %x and data 0~9\n", MATCH_ADDRSS1);
    RS485_SendAddressByte(MATCH_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup1, 10);
    // getchar();
    printf("Send Address %x and data 10~19\n", UNMATCH_ADDRSS1);
    RS485_SendAddressByte(UNMATCH_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup2, 10);
    // getchar();
    printf("Send Address %x and data 20~29\n", MATCH_ADDRSS2);
    RS485_SendAddressByte(MATCH_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup3, 10);
    //  getchar();
    printf("Send Address %x and data 30~39\n", UNMATCH_ADDRSS2);
    RS485_SendAddressByte(UNMATCH_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup4, 10);
    printf("Transfer Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Receive Test  (IS_USE_RS485NMM: 0:AAD  1:NMM)                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeSlave()
{
    /* Set Data Format*/ /* Only need parity enable whenever parity ODD/EVEN */
    LPUART_SetLine_Config(LPUART0, 0, LPUART_WORD_LEN_8, LPUART_PARITY_EVEN, LPUART_STOP_BIT_1);

    /* Reset RX FIFO Before Test */
    LPUART0->FIFO |= LPUART_FIFO_RXRST_Msk;
    LPUART0->FIFO &= ~LPUART_FIFO_RXRST_Msk;

    /* Set RX Trigger Level = 1 */
    LPUART0->FIFO &= ~LPUART_FIFO_RFITL_Msk;
    LPUART0->FIFO |= LPUART_FIFO_RFITL_1BYTE;

    /* Set RTS pin active level as High level active */
    LPUART0->MODEM &= ~UART_MODEM_RTSACTLV_Msk;
    LPUART0->MODEM |= LPUART_RTS_IS_HIGH_LEV_ACTIVE;

#if(IS_USE_RS485NMM == 1)
    printf("+-----------------------------------------------------------+\n");
    printf("|    Normal Multidrop Operation Mode                        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("| Only Address %2x and %2x,data can receive                  |\n", MATCH_ADDRSS1, MATCH_ADDRSS2);
    printf("+-----------------------------------------------------------+\n");

    /* Set RX_DIS enable before set RS485-NMM mode */
    LPUART0->FIFO |= LPUART_FIFO_RXOFF_Msk;

    /* Set RS485-NMM Mode */

    LPUART_SelectRS485Mode(LPUART0, LPUART_ALTCTL_RS485NMM_Msk | LPUART_ALTCTL_ADDRDEN_Msk | LPUART_ALTCTL_RS485AUD_Msk, 0);

#else
    printf("Auto Address Match Operation Mode\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("|    Auto Address Match Operation Mode                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|Only Address %2x,data can receive                          |\n", MATCH_ADDRSS1);
    printf("+-----------------------------------------------------------+\n");

    /* Set RS485-AAD Mode and address match is 0xC0 */
    LPUART_SelectRS485Mode(LPUART0, LPUART_ALTCTL_RS485AAD_Msk | LPUART_ALTCTL_ADDRDEN_Msk | LPUART_ALTCTL_RS485AUD_Msk, MATCH_ADDRSS1);
#endif

    /* Enable RDA\RLS\Time-out Interrupt  */
    LPUART_ENABLE_INT(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RLSIEN_Msk | LPUART_INTEN_RXTOIEN_Msk));
    NVIC_EnableIRQ(LPUART0_IRQn);

    printf("Ready to receive data...(Press any key to stop test)\n");
    getchar();

    /* Flush FIFO */
    while (LPUART_GET_RX_EMPTY(LPUART0) == 0)
    {
        LPUART_READ(LPUART0);
    }

    LPUART_DISABLE_INT(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RLSIEN_Msk | LPUART_INTEN_RXTOIEN_Msk));

    /* Set LPUART Function */
    LPUART_Open(LPUART0, 115200);
    printf("\n\nEnd test\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_FunctionTest()
{
    uint32_t u32Item;
    printf("\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|            IO Setting                                       |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  ______                                 _______             |\n");
    printf("| |      |                               |       |            |\n");
    printf("| |Master|---TXD(PA.1) <===> RXD(PA.0)---| Slave |            |\n");
    printf("| |      |---RTS(PA.4) <===> RTS(PA.4)---|       |            |\n");
    printf("| |______|                               |_______|            |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|       RS485 Function Test                                   |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n\n");
    u32Item = getchar();

    /*
        The sample code is used to test RS485 9-bit mode and needs
        two Module test board to complete the test.
        Master:
            1.Set AUD mode and HW will control RTS pin. RTSACTLV is set to '0'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'. (Set UART_LINE = 0x2B)
            4.Data bytes : the parity bit should be '0'. (Set UART_LINE = 0x3B)
            5.RTS pin is low in idle state. When master is sending,
              RTS pin will be pull high.

        Slave:
            1.Set AAD and AUD mode firstly. RTSACTLV is set to '0'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".  (Default)
            4.AAD: The slave will ignore any data until ADDRESS match address match value.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check if RS485 address byte detect flag is set and read RX FIFO data to clear ADDRESS stored in RX FIFO.

              NMM: The slave will ignore data byte until RXOFF is disabled.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check the ADDRESS is match or not by user in UART_IRQHandler.
              If the ADDRESS is match, clear RXOFF bit to receive data byte.
              If the ADDRESS is not match, set RXOFF bit to avoid data byte stored in FIFO.
    */

    if (u32Item == '0')
        RS485_9bitModeMaster();
    else
        RS485_9bitModeSlave();
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

