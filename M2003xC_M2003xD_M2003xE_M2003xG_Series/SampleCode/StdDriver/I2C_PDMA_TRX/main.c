/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate I2C PDMA mode and need two boards to connect I2C0(Master) and I2C0(Slave).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define I2C0_PDMA_TX_CH_MASTER    0
#define I2C0_PDMA_RX_CH_SLAVE     1
#define I2C0_PDMA_RX_CH_MASTER    2
#define I2C0_PDMA_TX_CH_SLAVE     3
#define PDMA_TEST_LENGTH    5

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_u8MasterTx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8MasterRx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8SlaveTx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8SlaveRx_Buffer[PDMA_TEST_LENGTH];
volatile uint32_t PDMA_DONE = 0;

volatile uint8_t g_u8DeviceAddr = 0x16;
volatile uint8_t g_u8MasterDataLen = 0;
volatile uint8_t g_u8SlaveDataLen = 0;
volatile uint16_t g_u8SlaveBufferAddr = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

volatile static I2C_FUNC s_I2C0HandlerFn = NULL;


void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA0->TDSTS;

    //Master TX
    if(u32Status & (0x1 << I2C0_PDMA_TX_CH_MASTER))
    {
        printf(" I2C0 Tx done\n");
        PDMA0->TDSTS = 0x1 << I2C0_PDMA_TX_CH_MASTER;
        PDMA_DONE = 1;
    }

    //Master RX
    if (u32Status & (0x1 << I2C0_PDMA_RX_CH_MASTER))
    {
        printf(" I2C0 Rx done\n");
        PDMA0->TDSTS = 0x1 << I2C0_PDMA_RX_CH_MASTER;
        PDMA_DONE = 1;
    }

    //Slave RX
    if (u32Status & (0x1 << I2C0_PDMA_RX_CH_SLAVE))
    {
        printf(" I2C0 Rx done\n");
        PDMA0->TDSTS = 0x1 << I2C0_PDMA_RX_CH_SLAVE;
        PDMA_DONE = 1;
    }

    //Slave TX
    if (u32Status & (0x1 << I2C0_PDMA_TX_CH_SLAVE))
    {
        printf(" I2C0 Tx done\n");
        PDMA0->TDSTS = 0x1 << I2C0_PDMA_TX_CH_SLAVE;
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
/*  I2C0 PDMA Slave Rx Callback Function                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveRx(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Address ACK interrupt
        */
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                  Data has been received; ACK has been returned*/
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Data ACK interrupt
        */
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        while(1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 PDMA Slave Tx Callback Function                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveTx(uint32_t u32Status)
{
    uint8_t u8data;

    if (u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) I2C_GET_DATA(I2C0);
        g_u8SlaveRx_Buffer[g_u8SlaveDataLen++] = u8data;
        g_u8SlaveBufferAddr = (g_u8SlaveRx_Buffer[0] << 8) + g_u8SlaveRx_Buffer[1];

        if(g_u8SlaveDataLen == 2)
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

        /* Enable I2C0 Slave TX */
        I2C0->CTL1 = I2C_CTL1_TXPDMAEN_Msk;
    }
    else if (u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0xB8)                  /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);

        while (1);
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
    if(u32Status == 0x08)                          /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x00);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                     /* SLA+W has been transmitted and ACK has been received */
    {
        g_u8MasterDataLen = 1;
        I2C_SET_DATA(I2C0, g_u8MasterTx_Buffer[g_u8MasterDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                     /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                     /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MasterDataLen <= 2)
        {
            I2C_SET_DATA(I2C0, g_u8MasterTx_Buffer[g_u8MasterDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                    /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                    /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

        /* Enable I2C0 Master RX */
        I2C0->CTL1 = I2C_CTL1_RXPDMAEN_Msk;
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
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO | I2C_CTL_SI);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Select UART0 clock source is HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable PDMA Clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* Set I2C0 multi-function pins */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB15MFP_Msk)) |
                    (SYS_GPB_MFPH_PB15MFP_I2C0_SDA | SYS_GPB_MFPH_PB14MFP_I2C0_SCL);

    /* I2C pin enable schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN14_Msk | GPIO_SMTEN_SMTEN15_Msk;
}

void I2C_Master_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("Master I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C_Slave_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("Slave I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x16, 0);   /* Slave Address : 0x16 */
    I2C_SetSlaveAddr(I2C0, 1, 0x36, 0);   /* Slave Address : 0x36 */
    I2C_SetSlaveAddr(I2C0, 2, 0x56, 0);   /* Slave Address : 0x56 */
    I2C_SetSlaveAddr(I2C0, 3, 0x76, 0);   /* Slave Address : 0x76 */

    /* Set I2C0 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x02);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x02);

    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void PDMA_Master_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(PDMA0, 1 << I2C0_PDMA_TX_CH_MASTER); // Channel 0 for I2C0 TX
    PDMA_Open(PDMA0, 1 << I2C0_PDMA_RX_CH_MASTER); // Channel 2 for I2C0 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA0, I2C0_PDMA_TX_CH_MASTER, PDMA_I2C0_TX, 0, 0);
    PDMA_SetTransferMode(PDMA0, I2C0_PDMA_RX_CH_MASTER, PDMA_I2C0_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA0, I2C0_PDMA_TX_CH_MASTER, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0, I2C0_PDMA_RX_CH_MASTER, PDMA_WIDTH_8, PDMA_TEST_LENGTH - 3); // except Slave Address and two bytes Data Address
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA0,I2C0_PDMA_TX_CH_MASTER, ((uint32_t) (&g_u8MasterTx_Buffer[0])), PDMA_SAR_INC, (uint32_t)(&(I2C0->DAT)), PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA0,I2C0_PDMA_RX_CH_MASTER, (uint32_t)(&(I2C0->DAT)), PDMA_SAR_FIX, ((uint32_t) (&g_u8MasterRx_Buffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA0, I2C0_PDMA_TX_CH_MASTER, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, I2C0_PDMA_RX_CH_MASTER, PDMA_REQ_SINGLE, 0);

    PDMA_EnableInt(PDMA0, I2C0_PDMA_TX_CH_MASTER, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, I2C0_PDMA_RX_CH_MASTER, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
}

void PDMA_Slave_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(PDMA0, 1 << I2C0_PDMA_RX_CH_SLAVE); // Channel 1 for I2C0 RX
    PDMA_Open(PDMA0, 1 << I2C0_PDMA_TX_CH_SLAVE); // Channel 3 for I2C0 TX
    // Select basic mode
    PDMA_SetTransferMode(PDMA0, I2C0_PDMA_RX_CH_SLAVE, PDMA_I2C0_RX, 0, 0);
    PDMA_SetTransferMode(PDMA0, I2C0_PDMA_TX_CH_SLAVE, PDMA_I2C0_TX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA0, I2C0_PDMA_RX_CH_SLAVE, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0, I2C0_PDMA_TX_CH_SLAVE, PDMA_WIDTH_8, PDMA_TEST_LENGTH - 3); // except Slave Address and two bytes Data Address
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA0,I2C0_PDMA_RX_CH_SLAVE, (uint32_t)(&(I2C0->DAT)), PDMA_SAR_FIX, ((uint32_t) (&g_u8SlaveRx_Buffer[0])), PDMA_DAR_INC);
    PDMA_SetTransferAddr(PDMA0,I2C0_PDMA_TX_CH_SLAVE, ((uint32_t) (&g_u8SlaveTx_Buffer[0])), PDMA_SAR_INC, (uint32_t)(&(I2C0->DAT)), PDMA_DAR_FIX);
    //Select Single Request
    PDMA_SetBurstType(PDMA0, I2C0_PDMA_RX_CH_SLAVE, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, I2C0_PDMA_TX_CH_SLAVE, PDMA_REQ_SINGLE, 0);

    PDMA_EnableInt(PDMA0, I2C0_PDMA_RX_CH_SLAVE, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, I2C0_PDMA_TX_CH_SLAVE, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
}

void I2C_PDMA_Master(void)
{
    uint32_t i;

    for (i=0; i<PDMA_TEST_LENGTH; i++)
    {
        g_u8MasterTx_Buffer[i] = i;
        g_u8SlaveRx_Buffer[i] = 0xff;
    }

    g_u8MasterTx_Buffer[0] = ((g_u8DeviceAddr << 1) | 0x00);   //1 byte SLV + W
    g_u8MasterTx_Buffer[1] = 0x00;                             //2 bytes Data address
    g_u8MasterTx_Buffer[2] = 0x00;

    PDMA_Master_Init();

    /* Enable I2C TX */
    I2C0->CTL1 = I2C_CTL1_TXPDMAEN_Msk;

    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterTx;

    PDMA_DONE = 0;

    /* Send START condition, start the PDMA data transmit */
    I2C_START(I2C0);

    while (!PDMA_DONE);

    printf("Wait slave receive data done and then press any key to start master PDMA RX mode.\n\n");
    getchar();

    /* Disable I2C0 PDMA TX mode */
    I2C0->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk;

    /* Test Master RX with PDMA function */

    /* I2C0 function to Master receive data */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterRx;

    PDMA_DONE = 0;

    /* Send START condition */
    I2C_START(I2C0);

    while (!PDMA_DONE);

    /* Disable I2C0 PDMA RX mode */
    I2C0->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;

    for (i = 0; i < PDMA_TEST_LENGTH - 3; i++)
    {
        if (g_u8MasterRx_Buffer[i] != g_u8MasterTx_Buffer[i+3])
        {
            printf(" Slave Receive Data Compare Error !!\n");

            while (1);
        }
    }

    printf("I2C PDMA test Pass.\n");
}

void I2C_PDMA_Slave(void)
{
    uint32_t i;

    for (i=0; i<PDMA_TEST_LENGTH; i++)
    {
        g_u8MasterTx_Buffer[i] = i;
        g_u8SlaveRx_Buffer[i] = 0xff;
    }

    g_u8MasterTx_Buffer[0] = ((g_u8DeviceAddr << 1) | 0x00);   //1 byte SLV + W
    g_u8MasterTx_Buffer[1] = 0x00;                             //2 bytes Data address
    g_u8MasterTx_Buffer[2] = 0x00;

    PDMA_DONE = 0;

    PDMA_Slave_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI | I2C_CTL_AA);
    /* Enable I2C0 PDMA RX mode */
    I2C0->CTL1 = I2C_CTL1_RXPDMAEN_Msk;

    /* I2C0 function to Slave receive data */
    s_I2C0HandlerFn = I2C_PDMA_SlaveRx;

    while (!PDMA_DONE);
    printf("I2C slave receive data done.\n\n");

    /* Disable I2C0 PDMA RX mode */
    I2C0->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;

    for (i=0; i<PDMA_TEST_LENGTH; i++)
    {
        if(g_u8SlaveRx_Buffer[i] != g_u8MasterTx_Buffer[i])
        {
            printf(" Receive Data Compare Error !!\n");
            while(1);
        }
        else
        {
            if(i > 2)
                g_u8SlaveTx_Buffer[i-3] = g_u8MasterTx_Buffer[i];
        }
    }

    /* Test Slave TX with PDMA function */
    printf("I2C slave wait to transmit data.\n");

    /* I2C0 function to Slave transmit data */
    s_I2C0HandlerFn = I2C_PDMA_SlaveTx;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int32_t main (void)
{
    uint32_t u32Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|       I2C Driver Sample Code for PDMA             |\n");
    printf("|                                                       |\n");
    printf("|  I2C Master (I2C0) <---> I2C Slave(I2C0)      |\n");
    printf("+-------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                   |\n");
    printf("|  [0] Master    [1] Slave                              |\n");
    printf("+-------------------------------------------------------+\n\n");
    u32Item = (uint32_t)getchar();

    if(u32Item == '0')
    {
        /* Init I2C0 Master */
        I2C_Master_Init();
        I2C_PDMA_Master();
    }
    else
    {
        /* Init I2C0 Slave */
        I2C_Slave_Init();
        I2C_PDMA_Slave();
    }

    while(1);
}
